// Copyright (c) 2026, ArtInChip Technology Co., Ltd
//
// SPDX-License-Identifier: Apache-2.0
//
// The upgrading protocol engine: frames, CBW/CSW transfers and the commands
// the bootloader understands.

use std::time::{Duration, Instant};

use crate::fwc;
use crate::proto::{self, Cbw, Csw, RespHeader};
use crate::transport::{Link, Transport};

/// sizeof(struct hwinfo) in basic_cmd.c.
pub const HWINFO_DATA_LEN: usize = 108;

/// Time we wait for the ACK of a single frame.
const ACK_TIMEOUT: Duration = Duration::from_secs(2);
/// Time the device gets to start sending a frame of its response.
const FRAME_TIMEOUT: Duration = Duration::from_secs(3);
/// Time the device gets to send the CSW once our data is in.
const CSW_TIMEOUT: Duration = Duration::from_secs(5);
/// Like CSW_TIMEOUT, but for the flash commands: a sector erase can take a
/// while before the device answers.
const FLASH_CSW_TIMEOUT: Duration = Duration::from_secs(30);
/// Time the payload of EXEC gets to return.
const EXEC_TIMEOUT: Duration = Duration::from_secs(5);
/// Time a bootloader console command gets to answer. "reset" never answers,
/// the board is already rebooting.
const SHELL_TIMEOUT: Duration = Duration::from_secs(2);
/// Payload carried by one CBW transfer while writing RAM or flash.
const DATA_CYCLE: usize = 32 * 1024;
const MAX_FRAME_RETRY: u32 = 5;
/// Window in which the device answers a connection probe.
const PROBE_WINDOW: Duration = Duration::from_millis(100);
/// Media type of the on board SPI NOR flash.
const MEDIA_TYPE: &str = "spi-nor";
const MEDIA_DEV_ID: u32 = 0;
/// The component name that makes the bootloader probe the flash.
const IMAGE_INFO_COMPONENT: &str = "image.info";
/// Attribute that selects the plain MTD write path in nor_fwc_data_write().
const MTD_ATTR: &str = "mtd,required";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExecOutcome {
    /// The payload returned, the boolean tells whether its status was OK.
    Returned { status_ok: bool },
    /// No response at all, most likely the payload is running now.
    NoResponse,
}

pub struct HwInfo {
    pub init_mode: u8,
    pub curr_mode: u8,
    pub boot_stage: u8,
    pub chip_id: [u32; 4],
}

pub struct Device<L: Link = Transport> {
    t: L,
    /// Block number of the next frame we send.
    blk: u8,
    /// Tag of the last CBW we sent.
    tag: u32,
    /// Block number of the last frame we received from the device.
    dev_blk: Option<u8>,
    verbose: bool,
    /// Partition table reported by the bootloader.
    partitions: Option<Vec<fwc::FlashPartition>>,
    /// Whether the flash has been probed in this session.
    flash_ready: bool,
}

impl<L: Link> Device<L> {
    /// Probe the serial port until a bootloader in download mode answers.
    pub fn connect(t: L, timeout: Duration, verbose: bool) -> Result<Self, String> {
        let mut dev = Device {
            t,
            blk: 1,
            tag: 0,
            dev_blk: None,
            verbose,
            partitions: None,
            flash_ready: false,
        };

        dev.t.flush();
        let deadline = Instant::now() + timeout;
        loop {
            dev.t.write_all(&[proto::SIG_C])?;
            let mut b = [0u8; 1];
            if dev.t.read(&mut b, Instant::now() + PROBE_WINDOW)? == 1 {
                match b[0] {
                    proto::ACK => break,
                    proto::CAN => dev.trace("device entered download mode"),
                    other => dev.trace(&format!("ignoring 0x{other:02X} while probing")),
                }
            }
            if Instant::now() >= deadline {
                return Err(
                    "no answer from the bootloader; make sure the board is in download mode \
                     (hold the download key while powering on) and that port and baud rate match"
                        .to_string(),
                );
            }
        }

        // Ask the device to receive data (DC1) - the ACK confirms that both
        // sides are in sync before the first frame is sent.
        dev.t.flush();
        for _ in 0..3 {
            dev.t.write_all(&[proto::DC1_SEND])?;
            if dev.wait_ack(Instant::now() + Duration::from_secs(1))? == Some(true) {
                dev.t.flush();
                return Ok(dev);
            }
        }

        Err("the device did not acknowledge the DC1 handshake".to_string())
    }

    /// Ask the device for its hardware information.
    pub fn hwinfo(&mut self) -> Result<HwInfo, String> {
        self.send_cmd_header(proto::cmd::GET_HWINFO, 0)?;
        let (_, data) = self.read_response(
            proto::cmd::GET_HWINFO,
            proto::RESP_HEADER_LEN + HWINFO_DATA_LEN,
            FRAME_TIMEOUT,
        )?;

        if &data[..6] != b"HWINFO" {
            return Err("the device answered with a malformed HWINFO block".to_string());
        }

        let chip_id = [44, 48, 52, 56]
            .map(|o| u32::from_le_bytes([data[o], data[o + 1], data[o + 2], data[o + 3]]));

        Ok(HwInfo {
            init_mode: data[38],
            curr_mode: data[39],
            boot_stage: data[40],
            chip_id,
        })
    }

    /// Download `data` to the RAM address `addr`.
    ///
    /// This uses the bootloader WRITE command: `addr` and the total length go
    /// in front of the first data block, the following transfers only carry
    /// data. `progress` is called with the number of bytes transferred so far.
    pub fn write_ram(
        &mut self,
        addr: u32,
        data: &[u8],
        progress: &mut dyn FnMut(usize),
    ) -> Result<(), String> {
        if data.is_empty() {
            return Ok(());
        }
        let total = u32::try_from(data.len()).map_err(|_| {
            format!(
                "{} bytes do not fit into a single WRITE command",
                data.len()
            )
        })?;

        self.send_cmd_header(proto::cmd::WRITE, 8 + total)?;

        let mut off = 0usize;
        while off < data.len() {
            let n = std::cmp::min(DATA_CYCLE, data.len() - off);
            let mut payload = Vec::with_capacity(n + 8);
            if off == 0 {
                payload.extend_from_slice(&addr.to_le_bytes());
                payload.extend_from_slice(&total.to_le_bytes());
            }
            payload.extend_from_slice(&data[off..off + n]);

            let csw = self
                .transfer_out(&payload, CSW_TIMEOUT)?
                .ok_or_else(|| "the device stopped answering while receiving data".to_string())?;
            csw.check("the device rejected the data it received")?;
            off += n;
            progress(off);
        }

        // Pick up the response of the WRITE command: it releases the command
        // state on the device and reports whether the data was accepted.
        let (resp, _) =
            self.read_response(proto::cmd::WRITE, proto::RESP_HEADER_LEN, FRAME_TIMEOUT)?;
        if resp.status != proto::RESP_OK {
            return Err("the device rejected the data it received".to_string());
        }

        Ok(())
    }

    /// Switch the device and this link to another baud rate.
    ///
    /// This uses the bootloader SET_UART_ARGS command: the device answers the
    /// command at the current speed and applies the new one once it is back in
    /// its wait state, so the link is reconfigured right after the response.
    pub fn set_uart_baudrate(&mut self, baud: u32) -> Result<(), String> {
        self.send_cmd_header(proto::cmd::SET_UART_ARGS, 4)?;

        let cbw = Cbw {
            tag: self.next_tag(),
            transfer_len: 4,
            flags: proto::CBW_FLAG_OUT,
            command: proto::TRANS_CMD_WRITE,
        };
        self.send_frame(&cbw.to_bytes())?;
        self.send_frame(&baud.to_le_bytes())?;

        let csw = self
            .read_csw(Instant::now() + CSW_TIMEOUT)?
            .ok_or_else(|| "the device did not acknowledge the baud rate argument".to_string())?;
        csw.check("the device rejected the baud rate argument")?;

        let (resp, _) = self.read_response(
            proto::cmd::SET_UART_ARGS,
            proto::RESP_HEADER_LEN,
            FRAME_TIMEOUT,
        )?;
        if resp.status != proto::RESP_OK {
            return Err("the device rejected the baud rate change".to_string());
        }

        // The device switches at the end of this command, follow it.
        self.set_link_baudrate(baud)?;

        Ok(())
    }

    /// Reconfigure this end of the link only, without talking to the device.
    pub fn set_link_baudrate(&mut self, baud: u32) -> Result<(), String> {
        self.t.flush();
        self.t.set_baudrate(baud)
    }

    /// Call the code at `addr` with the bootloader EXEC command.
    ///
    /// The address is expected to hold a function returning an `int`. The call
    /// happens inside the bootloader protocol loop, so only small, self
    /// contained payloads can be started this way. A payload that never
    /// returns is normal and reported as [`ExecOutcome::NoResponse`].
    pub fn exec(&mut self, addr: u32) -> Result<ExecOutcome, String> {
        self.send_cmd_header(proto::cmd::EXEC, 4)?;

        let cbw = Cbw {
            tag: self.next_tag(),
            transfer_len: 4,
            flags: proto::CBW_FLAG_OUT,
            command: proto::TRANS_CMD_WRITE,
        };
        self.send_frame(&cbw.to_bytes())?;
        self.send_frame(&addr.to_le_bytes())?;

        if self.read_csw(Instant::now() + EXEC_TIMEOUT)?.is_none() {
            return Ok(ExecOutcome::NoResponse);
        }

        let (resp, _) =
            self.read_response(proto::cmd::EXEC, proto::RESP_HEADER_LEN, FRAME_TIMEOUT)?;
        Ok(ExecOutcome::Returned {
            status_ok: resp.status == proto::RESP_OK,
        })
    }

    /// Run a command in the bootloader console (the RUN_SHELL_STR command).
    ///
    /// Returns `false` when the device stopped answering. That is the expected
    /// outcome for "reset": the bootloader reboots the board in the middle of
    /// the request, so the answer never arrives.
    pub fn run_shell(&mut self, command: &str) -> Result<bool, String> {
        let mut args = Vec::with_capacity(4 + command.len());
        args.extend_from_slice(&(command.len() as u32).to_le_bytes());
        args.extend_from_slice(command.as_bytes());

        self.send_cmd_header(proto::cmd::RUN_SHELL_STR, args.len() as u32)?;
        if self.transfer_out(&args, SHELL_TIMEOUT)?.is_none() {
            return Ok(false);
        }

        // The answer may be cut off by the reboot as well.
        match self.read_response(
            proto::cmd::RUN_SHELL_STR,
            proto::RESP_HEADER_LEN,
            SHELL_TIMEOUT,
        ) {
            Ok((resp, _)) if resp.status == proto::RESP_OK => Ok(true),
            Ok((resp, _)) => Err(format!(
                "the bootloader command '{command}' failed (status {})",
                resp.status
            )),
            Err(_) => Ok(false),
        }
    }

    //
    // Flash programming (the "firmware component" flow of fwc_cmd.c)
    //

    /// The flash partitions the bootloader offers.
    pub fn partitions(&mut self) -> Result<Vec<fwc::FlashPartition>, String> {
        if self.partitions.is_none() {
            let table = self.fetch_partition_table()?;
            self.partitions = Some(table);
        }
        Ok(self.partitions.clone().unwrap_or_default())
    }

    /// Program `data` into the flash at `offset`, which has to be the start of
    /// a partition. Returns the partition that was written.
    ///
    /// The bootloader always writes a component from the beginning of its
    /// partition (nor_fwc_mtd_write() counts from 0), so an offset inside a
    /// partition cannot be programmed - see [`Device::partitions`].
    pub fn write_flash(
        &mut self,
        offset: u64,
        data: &[u8],
        progress: &mut dyn FnMut(usize),
    ) -> Result<fwc::FlashPartition, String> {
        if data.is_empty() {
            return Err("refusing to program an empty file".to_string());
        }
        let partition = self.partition_at(offset)?;
        if data.len() as u64 > partition.size {
            return Err(format!(
                "{} bytes do not fit into partition '{}' ({} bytes)",
                data.len(),
                partition.name,
                partition.size
            ));
        }
        let size = u32::try_from(data.len())
            .map_err(|_| format!("{} bytes do not fit into one component", data.len()))?;
        let crc = fwc::crc32(data);

        self.flash_prepare()?;

        let meta = fwc::Meta {
            name: partition.name.clone(),
            partition: partition.name.clone(),
            offset: 0,
            size,
            crc,
            ram: 0,
            attr: MTD_ATTR.to_string(),
        };
        self.send_fwc_meta(&meta)?;

        // The bootloader only writes to the flash once it knows the media: it
        // reports block size 1 while it still treats components as RAM ones.
        let block_size = self.fwc_value(proto::cmd::GET_BLOCK_SIZE, "block size")?;
        if block_size <= 1 {
            return Err(
                "the bootloader did not switch to the flash and would drop the data; \
                 its media selection did not accept the image.info component"
                    .to_string(),
            );
        }
        self.trace(&format!("component block size: {block_size}"));

        self.send_fwc_data(data, progress)?;

        let device_crc = self.fwc_value(proto::cmd::GET_FWC_CRC, "flash CRC")?;
        if device_crc != crc {
            return Err(format!(
                "flash CRC mismatch: the device computed 0x{device_crc:08X}, the file is 0x{crc:08X}"
            ));
        }

        let burn = self.fwc_value(proto::cmd::GET_FWC_BURN_RESULT, "burn result")?;
        if burn != 0 {
            return Err(format!(
                "the bootloader reported a burn failure for partition '{}'",
                partition.name
            ));
        }

        Ok(partition)
    }

    /// Read `len` bytes back from the partition that starts at `offset`.
    ///
    /// The bootloader answers in two transfers: the response header first, then
    /// the flash data in a transfer of its own - its data phase fills the
    /// buffer from offset 0, so both cannot share one transfer.
    pub fn read_flash(&mut self, offset: u64, len: usize) -> Result<Vec<u8>, String> {
        if len == 0 {
            return Ok(Vec::new());
        }
        let partition = self.partition_at(offset)?;
        let args = fwc::media_partition(MEDIA_TYPE, MEDIA_DEV_ID, &partition.name, len as u64);

        self.send_cmd_header(proto::cmd::READ_FWC_DATA, args.len() as u32)?;
        self.transfer_out(&args, FLASH_CSW_TIMEOUT)?
            .ok_or_else(|| "the device did not accept the read request".to_string())?
            .check("the device rejected the read request")?;

        let resp = self.read_resp_header(proto::cmd::READ_FWC_DATA)?;
        if resp.status != proto::RESP_OK {
            return Err(format!(
                "the device could not read partition '{}'",
                partition.name
            ));
        }
        if resp.data_length as usize != len {
            return Err(format!(
                "the device announced {} bytes instead of the {len} that were requested",
                resp.data_length
            ));
        }

        self.begin_read(len)?;
        self.collect_read(len, FLASH_CSW_TIMEOUT)
    }

    /// The partition that starts exactly at `offset`.
    fn partition_at(&mut self, offset: u64) -> Result<fwc::FlashPartition, String> {
        let partitions = self.partitions()?;
        match partitions.iter().find(|p| p.start == offset) {
            Some(partition) => Ok(partition.clone()),
            None => {
                let known = partitions
                    .iter()
                    .map(|p| format!("{}@0x{:X}", p.name, p.start))
                    .collect::<Vec<_>>()
                    .join(", ");
                Err(format!(
                    "no flash partition starts at 0x{offset:X}; the bootloader programs a component \
                     from the start of its partition, available: {known}"
                ))
            }
        }
    }

    /// Ask the device for its partition table (GET_PARTITION_TABLE).
    fn fetch_partition_table(&mut self) -> Result<Vec<fwc::FlashPartition>, String> {
        let args = fwc::storage_media(MEDIA_TYPE, MEDIA_DEV_ID);
        self.send_cmd_header(proto::cmd::GET_PARTITION_TABLE, args.len() as u32)?;
        self.transfer_out(&args, CSW_TIMEOUT)?
            .ok_or_else(|| "the device did not answer the partition table request".to_string())?
            .check("the device rejected the partition table request")?;

        // The header comes in its own transfer, the table follows in the next.
        let resp = self.read_resp_header(proto::cmd::GET_PARTITION_TABLE)?;
        if resp.status != proto::RESP_OK {
            return Err("the device could not report its partition table".to_string());
        }
        self.begin_read(fwc::PARTITION_TABLE_LEN)?;
        let data = self.collect_read(fwc::PARTITION_TABLE_LEN, FRAME_TIMEOUT)?;

        let text = data.split(|&b| b == 0).next().unwrap_or(&data);
        let text = std::str::from_utf8(text)
            .map_err(|_| "the partition table is not valid UTF-8".to_string())?;
        let partitions = fwc::parse_partition_table(text)?;
        self.trace(&format!(
            "partitions: {}",
            partitions
                .iter()
                .map(|p| format!("{}@0x{:X}+0x{:X}", p.name, p.start, p.size))
                .collect::<Vec<_>>()
                .join(" ")
        ));
        Ok(partitions)
    }

    /// Let the bootloader probe the flash and register its partitions.
    ///
    /// Without this the device would treat every component as a RAM one and
    /// silently drop the data: the media type is only known after the
    /// "image.info" component has been seen (ram_fwc.c).
    fn flash_prepare(&mut self) -> Result<(), String> {
        if self.flash_ready {
            return Ok(());
        }

        let info = fwc::image_info(MEDIA_TYPE, MEDIA_DEV_ID);
        let meta = fwc::Meta {
            name: IMAGE_INFO_COMPONENT.to_string(),
            partition: String::new(),
            offset: 0,
            size: info.len() as u32,
            crc: fwc::crc32(&info),
            ram: 0,
            attr: "required".to_string(),
        };
        self.send_fwc_meta(&meta)?;
        self.send_fwc_data(&info, &mut |_| {})?;
        self.flash_ready = true;
        Ok(())
    }

    /// Send one SET_FWC_META command and pick up its response.
    fn send_fwc_meta(&mut self, meta: &fwc::Meta) -> Result<(), String> {
        self.send_cmd_header(proto::cmd::SET_FWC_META, fwc::META_LEN as u32)?;
        self.transfer_out(&meta.to_bytes(), CSW_TIMEOUT)?
            .ok_or_else(|| "the device did not accept the component description".to_string())?
            .check("the device rejected the component description")?;

        let (resp, _) =
            self.read_response(proto::cmd::SET_FWC_META, proto::RESP_HEADER_LEN, FRAME_TIMEOUT)?;
        if resp.status != proto::RESP_OK {
            return Err(format!("the device rejected component '{}'", meta.name));
        }
        Ok(())
    }

    /// Send the data of the current component.
    fn send_fwc_data(
        &mut self,
        data: &[u8],
        progress: &mut dyn FnMut(usize),
    ) -> Result<(), String> {
        self.send_cmd_header(proto::cmd::SEND_FWC_DATA, data.len() as u32)?;

        let mut off = 0usize;
        while off < data.len() {
            let n = std::cmp::min(DATA_CYCLE, data.len() - off);
            self.transfer_out(&data[off..off + n], FLASH_CSW_TIMEOUT)?
                .ok_or_else(|| {
                    "the device stopped answering while programming the flash".to_string()
                })?
                .check("the device failed to program the received data")?;
            off += n;
            progress(off);
        }

        let (resp, _) = self.read_response(
            proto::cmd::SEND_FWC_DATA,
            proto::RESP_HEADER_LEN,
            FLASH_CSW_TIMEOUT,
        )?;
        if resp.status == proto::RESP_FAIL {
            return Err(
                "the bootloader did not receive all bytes of the component".to_string(),
            );
        }
        if resp.status != proto::RESP_OK {
            return Err(format!(
                "the bootloader did not finish the programming (status {})",
                resp.status
            ));
        }
        Ok(())
    }

    /// Send a READ CBW, starting an incoming transfer of `len` bytes.
    fn begin_read(&mut self, len: usize) -> Result<(), String> {
        let cbw = Cbw {
            tag: self.next_tag(),
            transfer_len: len as u32,
            flags: proto::CBW_FLAG_IN,
            command: proto::TRANS_CMD_READ,
        };
        self.send_frame(&cbw.to_bytes())
    }

    /// Read the 4 byte value of one of the GET_FWC_* commands.
    fn fwc_value(&mut self, command: u8, what: &str) -> Result<u32, String> {
        self.send_cmd_header(command, 0)?;

        // The value follows in its own transfer, overwriting the header.
        let resp = self.read_resp_header(command)?;
        if resp.status != proto::RESP_OK {
            return Err(format!("the device could not report the {what}"));
        }
        if resp.data_length != 4 {
            return Err(format!(
                "unexpected {what} response: the header announces {} bytes",
                resp.data_length
            ));
        }

        self.begin_read(4)?;
        let value = self.collect_read(4, FRAME_TIMEOUT)?;
        Ok(u32::from_le_bytes(value[0..4].try_into().unwrap()))
    }

    //
    // Transport helpers
    //

    fn next_tag(&mut self) -> u32 {
        self.tag = self.tag.wrapping_add(1);
        self.tag
    }

    /// Send one command packet: CBW, payload, then the CSW from the device.
    fn transfer_out(
        &mut self,
        payload: &[u8],
        csw_timeout: Duration,
    ) -> Result<Option<CswOutcome>, String> {
        let cbw = Cbw {
            tag: self.next_tag(),
            transfer_len: payload.len() as u32,
            flags: proto::CBW_FLAG_OUT,
            command: proto::TRANS_CMD_WRITE,
        };
        self.send_frame(&cbw.to_bytes())?;
        self.send_data(payload)?;
        self.read_csw(Instant::now() + csw_timeout)
    }

    /// Send a command header: the bootloader consumes it as its own packet,
    /// arguments and data follow in the next packets.
    fn send_cmd_header(&mut self, command: u8, data_length: u32) -> Result<(), String> {
        let header = proto::cmd_header(command, data_length);
        let csw = self
            .transfer_out(&header, CSW_TIMEOUT)?
            .ok_or_else(|| "the device did not acknowledge the command header".to_string())?;
        csw.check("the device rejected the command header")
    }

    /// Read `len` bytes of an incoming transfer and its CSW.
    fn collect_read(&mut self, len: usize, csw_timeout: Duration) -> Result<Vec<u8>, String> {
        let mut buf = Vec::with_capacity(len);
        self.read_data(len, FRAME_TIMEOUT, &mut buf)?;

        let csw = self
            .read_csw(Instant::now() + csw_timeout)?
            .ok_or_else(|| "the device did not send a status for the response".to_string())?;
        csw.check("the device reported a failure for the response")?;
        Ok(buf)
    }

    /// Read the 16 byte response header of `command` as its own transfer.
    ///
    /// Several commands of the bootloader answer with the header in one
    /// transfer and their payload in the next one, overwriting the buffer the
    /// header was built in.
    fn read_resp_header(&mut self, command: u8) -> Result<RespHeader, String> {
        self.begin_read(proto::RESP_HEADER_LEN)?;
        let buf = self.collect_read(proto::RESP_HEADER_LEN, FRAME_TIMEOUT)?;

        let resp = RespHeader::parse(&buf)?;
        if resp.command != command {
            return Err(format!(
                "the device answered command 0x{:02X} instead of 0x{command:02X}",
                resp.command
            ));
        }
        Ok(resp)
    }

    /// Read the response of `command`, `total_len` bytes including the header.
    fn read_response(
        &mut self,
        command: u8,
        total_len: usize,
        timeout: Duration,
    ) -> Result<(RespHeader, Vec<u8>), String> {
        self.begin_read(total_len)?;
        let buf = self.collect_read(total_len, timeout)?;

        let resp = RespHeader::parse(&buf)?;
        if resp.command != command {
            return Err(format!(
                "the device answered command 0x{:02X} instead of 0x{command:02X}",
                resp.command
            ));
        }
        let payload = buf[proto::RESP_HEADER_LEN..].to_vec();
        if resp.data_length as usize != payload.len() {
            return Err(format!(
                "response length mismatch: the header announces {} bytes, {} were sent",
                resp.data_length,
                payload.len()
            ));
        }
        Ok((resp, payload))
    }

    /// Send one frame, retry it until the device acknowledges it.
    fn send_frame(&mut self, data: &[u8]) -> Result<(), String> {
        let frame = proto::pack_frame(self.blk, data)?;
        for attempt in 1..=MAX_FRAME_RETRY {
            self.t.flush();
            self.t.write_all(&frame)?;
            match self.wait_ack(Instant::now() + ACK_TIMEOUT)? {
                Some(true) => {
                    self.blk = self.blk.wrapping_add(1);
                    return Ok(());
                }
                Some(false) => {
                    self.trace(&format!("frame not acknowledged (NAK), retry {attempt}"))
                }
                None => self.trace(&format!("frame not acknowledged, retry {attempt}")),
            }
        }
        Err(format!(
            "the device did not acknowledge a {} byte frame",
            data.len()
        ))
    }

    /// Send `data` as a sequence of protocol frames.
    fn send_data(&mut self, data: &[u8]) -> Result<(), String> {
        let mut off = 0usize;
        while off < data.len() {
            let rest = data.len() - off;
            let n = if rest >= proto::LONG_DATA_LEN {
                proto::LONG_DATA_LEN
            } else {
                std::cmp::min(rest, proto::SHORT_DATA_LEN)
            };
            self.send_frame(&data[off..off + n])?;
            off += n;
        }
        Ok(())
    }

    /// Wait for the ACK/NAK of a frame. `None` means that nothing arrived.
    fn wait_ack(&mut self, deadline: Instant) -> Result<Option<bool>, String> {
        loop {
            match self.t.read_byte(deadline)? {
                None => return Ok(None),
                Some(proto::ACK) => return Ok(Some(true)),
                Some(proto::NAK) => return Ok(Some(false)),
                Some(other) => self.trace(&format!("ignoring 0x{other:02X} while waiting for ACK")),
            }
        }
    }

    /// Read a frame from the device, acknowledge it and return its payload.
    fn read_frame(&mut self, deadline: Instant) -> Result<(u8, Vec<u8>), String> {
        loop {
            let start = match self.read_frame_start(deadline)? {
                Some(start) => start,
                None => return Err("the device stopped sending data".to_string()),
            };
            if let Some(frame) = self.read_frame_body(start, deadline)? {
                return Ok(frame);
            }
        }
    }

    /// Read `len` bytes of payload from the device.
    fn read_data(
        &mut self,
        len: usize,
        timeout: Duration,
        out: &mut Vec<u8>,
    ) -> Result<(), String> {
        while out.len() < len {
            let deadline = Instant::now() + timeout;
            let (blk, data) = self.read_frame(deadline)?;
            if Some(blk) == self.dev_blk {
                self.trace(&format!("dropping repeated frame {blk}"));
                continue;
            }
            self.dev_blk = Some(blk);
            out.extend_from_slice(&data);
        }
        if out.len() != len {
            return Err(format!(
                "the device sent {} bytes where {len} were expected",
                out.len()
            ));
        }
        Ok(())
    }

    /// Read the command status wrapper. `None` means that the device sent
    /// nothing before the deadline.
    fn read_csw(&mut self, deadline: Instant) -> Result<Option<CswOutcome>, String> {
        loop {
            let start = match self.read_frame_start(deadline)? {
                Some(start) => start,
                None => return Ok(None),
            };
            let (_, data) = match self.read_frame_body(start, deadline)? {
                Some(frame) => frame,
                None => continue,
            };
            if data.len() != Csw::LEN {
                return Err(format!(
                    "expected a {} byte CSW, got {} bytes",
                    Csw::LEN,
                    data.len()
                ));
            }
            let csw = Csw::parse(&data)?;
            if csw.tag != self.tag {
                self.trace(&format!(
                    "CSW tag 0x{:08X} does not match the last CBW tag 0x{:08X}",
                    csw.tag, self.tag
                ));
            }
            self.trace(&format!(
                "CSW tag 0x{:08X}, residue {}, status {}",
                csw.tag,
                csw.residue,
                csw.status_str()
            ));
            return Ok(Some(CswOutcome(csw)));
        }
    }

    /// Wait for the first byte of a frame, skipping everything that is not a
    /// frame delimiter (the device sends a CAN when it enters download mode).
    fn read_frame_start(&mut self, deadline: Instant) -> Result<Option<u8>, String> {
        loop {
            match self.t.read_byte(deadline)? {
                None => return Ok(None),
                Some(b @ (proto::SOH | proto::STX)) => return Ok(Some(b)),
                Some(other) => self.trace(&format!("skipping 0x{other:02X} outside of a frame")),
            }
        }
    }

    /// Read the rest of a frame. `None` means that the frame was broken and
    /// has been answered with a NAK.
    fn read_frame_body(
        &mut self,
        start: u8,
        deadline: Instant,
    ) -> Result<Option<(u8, Vec<u8>)>, String> {
        let mut hdr = [0u8; 2];
        self.t.read_exact(&mut hdr, deadline)?;
        let blk = hdr[0];
        if hdr[1] != !blk {
            self.trace(&format!(
                "broken block header 0x{:02X}/0x{:02X}, resynchronizing",
                hdr[0], hdr[1]
            ));
            self.t.write_all(&[proto::NAK])?;
            return Ok(None);
        }

        let dlen = if start == proto::SOH {
            let mut len = [0u8; 1];
            self.t.read_exact(&mut len, deadline)?;
            let len = len[0] as usize;
            if len == 0 || len > proto::SHORT_DATA_LEN {
                self.trace(&format!("bad short frame length {len}"));
                self.t.write_all(&[proto::NAK])?;
                return Ok(None);
            }
            len
        } else {
            proto::LONG_DATA_LEN
        };

        let mut buf = vec![0u8; dlen + 2];
        self.t.read_exact(&mut buf, deadline)?;
        let crc = u16::from_be_bytes([buf[dlen], buf[dlen + 1]]);
        if proto::crc16(&buf[..dlen]) != crc {
            self.trace("CRC error in a frame from the device");
            self.t.write_all(&[proto::NAK])?;
            return Ok(None);
        }
        self.t.write_all(&[proto::ACK])?;

        buf.truncate(dlen);
        Ok(Some((blk, buf)))
    }

    fn trace(&self, msg: &str) {
        if self.verbose {
            eprintln!("        . {msg}");
        }
    }
}

/// The command status wrapper of a transfer.
pub struct CswOutcome(Csw);

impl CswOutcome {
    /// Make sure the device accepted the transfer.
    fn check(self, msg: &str) -> Result<(), String> {
        if self.0.status != proto::CSW_STATUS_PASSED {
            return Err(format!(
                "{msg} (device status: '{}')",
                self.0.status_str()
            ));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::{Sim, SimLink};
    use std::cell::RefCell;
    use std::rc::Rc;

    /// Connect the host implementation to the simulated bootloader.
    fn connected() -> (Device<SimLink>, Rc<RefCell<Sim>>) {
        let sim = Rc::new(RefCell::new(Sim::new()));
        let dev = Device::connect(
            SimLink::new(sim.clone()),
            Duration::from_millis(200),
            false,
        )
        .expect("connect");
        (dev, sim)
    }

    #[test]
    fn connects_and_reads_the_device_info() {
        let (mut dev, sim) = connected();
        let info = dev.hwinfo().expect("hwinfo");
        assert_eq!(info.boot_stage, 1);
        assert_eq!(info.init_mode, 0x01);
        assert_eq!(info.chip_id[0], 0x1234_5678);
        assert!(sim.borrow().fault_free(), "{:?}", sim.borrow().faults);
    }

    #[test]
    fn writes_ram_in_several_cycles() {
        let (mut dev, sim) = connected();
        // More than one data cycle (32 KiB each) and not a multiple of it.
        let data: Vec<u8> = (0..80_000u32).map(|i| (i % 251) as u8).collect();
        let addr = 0x2000_1000u32;

        dev.write_ram(addr, &data, &mut |_| {}).expect("write");

        let sim = sim.borrow();
        assert!(sim.fault_free(), "{:?}", sim.faults);
        assert_eq!(sim.mem.len(), data.len());
        for (i, byte) in data.iter().enumerate() {
            let at = addr + i as u32;
            assert_eq!(sim.mem.get(&at), Some(byte), "byte at 0x{at:08X}");
        }
    }

    #[test]
    fn writes_small_files_and_reports_progress() {
        let (mut dev, sim) = connected();
        let data = [0xA5u8; 300];
        let mut last = 0usize;

        dev.write_ram(0x3000_0000, &data, &mut |done| last = done)
            .expect("write");

        assert_eq!(last, data.len());
        let sim = sim.borrow();
        assert!(sim.fault_free(), "{:?}", sim.faults);
        assert_eq!(sim.mem.get(&0x3000_0000), Some(&0xA5));
        assert_eq!(sim.mem.get(&0x3000_012B), Some(&0xA5));
    }

    #[test]
    fn retries_a_frame_that_was_rejected() {
        let sim = Rc::new(RefCell::new(Sim::new()));
        sim.borrow_mut().nak_first_frame = true;
        let mut dev = Device::connect(
            SimLink::new(sim.clone()),
            Duration::from_millis(200),
            false,
        )
        .expect("connect");

        dev.write_ram(0x3000, &[0x5A; 100], &mut |_| {}).expect("write");

        let sim = sim.borrow();
        assert!(sim.fault_free(), "{:?}", sim.faults);
        assert_eq!(sim.mem.get(&0x3000), Some(&0x5A));
        assert_eq!(sim.mem.get(&(0x3000u32 + 99)), Some(&0x5A));
    }

    #[test]
    fn switches_the_uart_baudrate() {
        let (mut dev, sim) = connected();

        dev.set_uart_baudrate(3_000_000).expect("set baudrate");
        assert_eq!(sim.borrow().baud_updates, vec![3_000_000]);

        // The session stays usable after the speed change.
        dev.write_ram(0x3000_0000, &[0x11; 64], &mut |_| {})
            .expect("write after the switch");
        assert_eq!(sim.borrow().mem.get(&0x3000_0000), Some(&0x11));
        assert!(sim.borrow().fault_free(), "{:?}", sim.borrow().faults);
    }

    #[test]
    fn exec_calls_the_payload() {
        let (mut dev, sim) = connected();

        let outcome = dev.exec(0x2000_0000).expect("exec");
        assert_eq!(outcome, ExecOutcome::Returned { status_ok: true });
        assert_eq!(sim.borrow().exec_calls, vec![0x2000_0000]);
        assert!(sim.borrow().fault_free(), "{:?}", sim.borrow().faults);
    }

    #[test]
    fn exec_reports_a_failing_payload() {
        let (mut dev, sim) = connected();
        sim.borrow_mut().exec_ret = 7;

        let outcome = dev.exec(0x2000_0000).expect("exec");
        assert_eq!(outcome, ExecOutcome::Returned { status_ok: false });
        assert!(sim.borrow().fault_free(), "{:?}", sim.borrow().faults);
    }

    #[test]
    fn a_write_after_an_exec_keeps_the_session_alive() {
        let (mut dev, sim) = connected();

        dev.exec(0x2000_0000).expect("exec");
        dev.write_ram(0x4000, &[1, 2, 3, 4], &mut |_| {}).expect("write");

        let sim = sim.borrow();
        assert!(sim.fault_free(), "{:?}", sim.faults);
        assert_eq!(sim.mem.get(&0x4000), Some(&1));
        assert_eq!(sim.mem.get(&0x4003), Some(&4));
    }

    #[test]
    fn reset_reboots_the_board_without_an_answer() {
        let (mut dev, sim) = connected();

        let answered = dev.run_shell("reset").expect("run reset");

        assert!(!answered, "the bootloader cannot answer a reset");
        let sim = sim.borrow();
        assert_eq!(sim.shell_commands, vec!["reset".to_string()]);
        assert!(sim.rebooted);
        assert!(sim.fault_free(), "{:?}", sim.faults);
    }

    #[test]
    fn runs_other_console_commands() {
        let (mut dev, sim) = connected();

        let answered = dev.run_shell("md 0x30040000 4").expect("run command");

        assert!(answered);
        let sim = sim.borrow();
        assert_eq!(sim.shell_commands, vec!["md 0x30040000 4".to_string()]);
        assert!(!sim.rebooted);
        assert!(sim.fault_free(), "{:?}", sim.faults);
    }

    #[test]
    fn lists_the_flash_partitions() {
        let (mut dev, sim) = connected();
        let partitions = dev.partitions().expect("partition table");

        assert_eq!(partitions.len(), 7);
        assert_eq!(partitions[0].name, "spl");
        assert_eq!(partitions[0].start, 0);
        assert_eq!(partitions[0].size, 0x80000);
        assert_eq!(partitions[4].name, "os");
        assert_eq!(partitions[4].start, 0x10_0000);
        assert!(sim.borrow().fault_free(), "{:?}", sim.borrow().faults);
    }

    #[test]
    fn programs_the_flash_and_reads_it_back() {
        let (mut dev, sim) = connected();
        let data: Vec<u8> = (0..40_000u32).map(|i| (i % 251) as u8).collect();
        let offset = 0x10_0000u64; // the os partition

        let partition = dev
            .write_flash(offset, &data, &mut |_| {})
            .expect("program the flash");
        assert_eq!(partition.name, "os");

        let sim = sim.borrow();
        assert!(sim.fault_free(), "{:?}", sim.faults);
        for (i, byte) in data.iter().enumerate() {
            let at = offset as u32 + i as u32;
            assert_eq!(sim.flash.get(&at), Some(byte), "flash byte at 0x{at:X}");
        }
        drop(sim);

        let read_back = dev.read_flash(offset, data.len()).expect("read back");
        assert_eq!(read_back, data);
    }

    #[test]
    fn programs_two_partitions_in_one_session() {
        let (mut dev, sim) = connected();

        dev.write_flash(0, &[0x11; 5000], &mut |_| {}).expect("spl");
        dev.write_flash(0x40_0000, &[0x22; 5000], &mut |_| {}).expect("data");

        let sim = sim.borrow();
        assert!(sim.fault_free(), "{:?}", sim.faults);
        assert_eq!(sim.flash.get(&0), Some(&0x11));
        assert_eq!(sim.flash.get(&4999), Some(&0x11));
        assert_eq!(sim.flash.get(&0x40_0000), Some(&0x22));
        assert_eq!(sim.flash.get(&(0x40_0000u32 + 4999)), Some(&0x22));
    }

    #[test]
    fn rejects_an_offset_inside_a_partition() {
        let (mut dev, _sim) = connected();

        let err = dev.write_flash(0x8000, &[1, 2, 3], &mut |_| {}).unwrap_err();
        assert!(err.contains("no flash partition starts at 0x8000"), "{err}");
    }

    #[test]
    fn rejects_a_file_that_does_not_fit_into_the_partition() {
        let (mut dev, _sim) = connected();

        // The spl partition is 512 KiB.
        let data = vec![0u8; 0x80000 + 1];
        let err = dev.write_flash(0, &data, &mut |_| {}).unwrap_err();
        assert!(err.contains("do not fit into partition 'spl'"), "{err}");
    }

    #[test]
    fn detects_a_bootloader_that_stays_in_ram_mode() {
        // If the media selection silently fails the bootloader would drop the
        // data and still report success, the block size tells us.
        let sim = Rc::new(RefCell::new(Sim::new()));
        sim.borrow_mut().ignore_image_info = true;
        let mut dev = Device::connect(
            SimLink::new(sim.clone()),
            Duration::from_millis(200),
            false,
        )
        .expect("connect");

        let err = dev.write_flash(0, &[0x5A; 64], &mut |_| {}).unwrap_err();
        assert!(err.contains("did not switch to the flash"), "{err}");
        assert!(sim.borrow().flash.is_empty());
    }
}
