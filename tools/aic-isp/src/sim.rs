// Copyright (c) 2026, ArtInChip Technology Co., Ltd
//
// SPDX-License-Identifier: Apache-2.0
//
// A model of the bootloader side of the upgrading protocol, written from
// application/baremetal/bootloader/lib/aicupg/{uart_proto_layer,
// data_trans_layer,basic_cmd}.c. The unit tests run the host implementation
// against it, so framing, packet boundaries and the command flow can be
// verified without hardware.
//
// The model is deliberately close to the C code: same frame rules, same block
// numbering, same packet boundaries (command header, then arguments and data
// in separate transfers) and the same CSW timing.

use std::cell::RefCell;
use std::collections::{BTreeMap, VecDeque};
use std::rc::Rc;
use std::time::Instant;

use crate::fwc;
use crate::proto;
use crate::transport::Link;

/// TRANS_DATA_BUFF_MAX_SIZE, the slice size the device pulls per transfer.
const TRANS_BUF: usize = 64 * 1024;
/// sizeof(struct hwinfo) in basic_cmd.c.
const HWINFO_LEN: usize = 108;
/// MAX_SHELL_CMD_STR_LEN in basic_cmd.c.
const MAX_SHELL_CMD_STR_LEN: usize = 128;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Dir {
    /// The device receives data from the host.
    HostSend,
    /// The device sends data to the host.
    HostRecv,
}

/// enum Stage of data_trans_layer.c. AICUPG_DATA_OUT_BUF is not modelled, the
/// device switches from it to DATA_OUT in the same call.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Stage {
    ReadCbw,
    DataOut,
    DataIn,
    DataInBuf,
    SendCsw,
    WaitCsw,
}

/// enum upg_cmd_state of upg_internal.h
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum CmdState {
    Idle,
    Start,
    Arg,
    DataIn,
    Resp,
    DataOut,
    End,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Priv {
    None,
    Write { addr: u32, len: u32, index: u32 },
    Exec { status: u8 },
}

struct Task {
    dir: Dir,
    buf: Vec<u8>,
    len: usize,
    xfer: usize,
}

struct Asm {
    start: u8,
    buf: Vec<u8>,
    need: usize,
}

#[derive(Clone, Copy)]
struct CbwInfo {
    tag: u32,
    transfer_len: u32,
    flags: u8,
    command: u8,
}

enum Rx {
    Ignore,
    Ack,
    Nak,
    Frame(Vec<u8>),
}

/// The simulated bootloader.
pub struct Sim {
    // uart protocol layer
    to_host: VecDeque<u8>,
    rx: VecDeque<u8>,
    asm: Option<Asm>,
    recv_blk: u8,
    first_connect: bool,
    send_blk: u8,
    tx_in_flight: bool,
    tx_len: usize,
    task: Option<Task>,
    // data transport layer
    stage: Stage,
    remain: u32,
    csw_residue: u32,
    cbw: Option<CbwInfo>,
    trans_buf: Vec<u8>,
    // command layer
    cur_cmd: Option<u8>,
    cmd_state: CmdState,
    priv_: Priv,
    // simulated device behaviour
    pub mem: BTreeMap<u32, u8>,
    pub hw_info: Vec<u8>,
    pub exec_calls: Vec<u32>,
    pub exec_ret: i32,
    /// Baud rates the device was asked to switch to, in SET_UART_ARGS order.
    pub baud_updates: Vec<u32>,
    pending_baud: Option<u32>,
    pub nak_first_frame: bool,
    /// Simulated SPI NOR content, sparse, unwritten bytes read as 0xFF.
    pub flash: BTreeMap<u32, u8>,
    /// The MTD partition string the bootloader reports.
    pub partition_table: String,
    /// Set once the "image.info" component switched the device to SPI NOR.
    flash_ready: bool,
    /// Test hook: pretend the media switch never happened.
    pub ignore_image_info: bool,
    /// Console commands that were run through RUN_SHELL_STR.
    pub shell_commands: Vec<String>,
    /// Set when a "reset" command made the device stop answering.
    pub rebooted: bool,
    halted: bool,
    /// Current firmware component (fwc_info_data).
    fwc: Option<SimMeta>,
    fwc_written: u32,
    fwc_crc: u32,
    fwc_block: u32,
    read_done: u64,
    read_size: u64,
    read_start: u64,
    pub faults: Vec<String>,
}

/// The fields of struct fwc_meta that the model needs.
#[derive(Clone)]
struct SimMeta {
    name: String,
    partition: String,
    size: u32,
    attr: String,
}

impl Sim {
    pub fn new() -> Self {
        let mut sim = Sim {
            to_host: VecDeque::new(),
            rx: VecDeque::new(),
            asm: None,
            recv_blk: 0,
            first_connect: true,
            send_blk: 0,
            tx_in_flight: false,
            tx_len: 0,
            task: None,
            stage: Stage::ReadCbw,
            remain: 0,
            csw_residue: 0,
            cbw: None,
            trans_buf: vec![0u8; TRANS_BUF],
            cur_cmd: None,
            cmd_state: CmdState::Idle,
            priv_: Priv::None,
            mem: BTreeMap::new(),
            hw_info: {
                let mut info = vec![0u8; HWINFO_LEN];
                info[0..6].copy_from_slice(b"HWINFO");
                info[38] = 0x01; // init mode: full disk upgrade
                info[40] = 1; // boot stage: bootloader
                info[44..48].copy_from_slice(&0x1234_5678u32.to_le_bytes());
                info
            },
            exec_calls: Vec::new(),
            exec_ret: 0,
            baud_updates: Vec::new(),
            pending_baud: None,
            nak_first_frame: false,
            flash: BTreeMap::new(),
            partition_table:
                "spi0.0:512k(spl),128k(env),128k(env_r),256k(userid),1536k(os),1536k(os_r),12288k(data)"
                    .to_string(),
            flash_ready: false,
            ignore_image_info: false,
            shell_commands: Vec::new(),
            rebooted: false,
            halted: false,
            fwc: None,
            fwc_written: 0,
            fwc_crc: 0,
            fwc_block: 0,
            read_done: 0,
            read_size: 0,
            read_start: 0,
            faults: Vec::new(),
        };

        // uart_proto_layer_init() sends CAN, aic_uart_upg_init() reads a CBW.
        sim.to_host.push_back(proto::CAN);
        sim.start_recv(proto::Cbw::LEN);
        sim
    }

    pub fn fault_free(&self) -> bool {
        self.faults.is_empty()
    }

    fn fault(&mut self, msg: impl Into<String>) {
        let msg = msg.into();
        eprintln!("sim: fault: {msg}");
        self.faults.push(msg);
    }

    //
    // Main loop of the device
    //

    pub fn pump(&mut self) {
        for _ in 0..1_000_000 {
            if !self.step() {
                return;
            }
        }
        self.fault("the model did not settle");
    }

    fn step(&mut self) -> bool {
        if self.halted {
            // "reset" took the board down, it answers nothing any more.
            return false;
        }

        // A frame we sent is waiting for its ACK.
        if self.tx_in_flight {
            return match self.rx.pop_front() {
                None => false,
                Some(proto::ACK) => {
                    self.tx_in_flight = false;
                    self.send_blk = self.send_blk.wrapping_add(1);
                    if let Some(task) = self.task.as_mut() {
                        task.xfer += self.tx_len;
                    }
                    if matches!(&self.task, Some(t) if t.xfer >= t.len) {
                        self.on_send_done();
                    }
                    true
                }
                Some(proto::NAK) => {
                    // The frame is sent again, the transfer state is kept.
                    self.tx_in_flight = false;
                    true
                }
                Some(_) => true,
            };
        }

        // Send the next frame of the pending task.
        if let Some(task) = self.task.as_mut() {
            if task.dir == Dir::HostRecv && task.xfer < task.len {
                let rest = task.len - task.xfer;
                let slice = if rest > proto::LONG_DATA_LEN {
                    proto::LONG_DATA_LEN
                } else if rest > proto::SHORT_DATA_LEN {
                    proto::SHORT_DATA_LEN
                } else {
                    rest
                };
                let frame = proto::pack_frame(self.send_blk, &task.buf[task.xfer..task.xfer + slice])
                    .expect("pack frame");
                self.to_host.extend(frame);
                self.tx_in_flight = true;
                self.tx_len = slice;
                return true;
            }
        }

        // Consume one byte from the host.
        if let Some(byte) = self.rx.pop_front() {
            match self.feed(byte) {
                Rx::Ignore => {}
                Rx::Ack => self.to_host.push_back(proto::ACK),
                Rx::Nak => self.to_host.push_back(proto::NAK),
                Rx::Frame(data) => {
                    self.to_host.push_back(proto::ACK);
                    let mut overflow = false;
                    let complete = match self.task.as_mut() {
                        Some(task) => {
                            task.buf.extend_from_slice(&data);
                            task.xfer += data.len();
                            overflow = task.xfer > task.len;
                            task.xfer >= task.len
                        }
                        None => false,
                    };
                    if overflow {
                        self.fault("frame overflow in a receive task");
                    }
                    if complete {
                        self.on_recv_done();
                    }
                }
            }
            return true;
        }

        false
    }

    /// uart_proto_layer_host_send_proc(): frame assembly and handshake.
    fn feed(&mut self, b: u8) -> Rx {
        if !matches!(&self.task, Some(t) if t.dir == Dir::HostSend) {
            return Rx::Ignore;
        }

        if self.asm.is_none() {
            return match b {
                proto::SOH | proto::STX => {
                    self.asm = Some(Asm {
                        start: b,
                        buf: vec![b],
                        need: 0,
                    });
                    Rx::Ignore
                }
                proto::SIG_C | proto::DC1_SEND => Rx::Ack,
                _ => Rx::Ignore,
            };
        }

        let asm = self.asm.as_mut().unwrap();
        asm.buf.push(b);
        if asm.need == 0 {
            if asm.start == proto::SOH && asm.buf.len() == 4 {
                asm.need = 6 + asm.buf[3] as usize;
            } else if asm.start == proto::STX && asm.buf.len() == 3 {
                asm.need = 5 + proto::LONG_DATA_LEN;
            }
        }
        if asm.need == 0 || asm.buf.len() < asm.need {
            return Rx::Ignore;
        }

        let asm = self.asm.take().unwrap();
        let blk = asm.buf[1];
        if asm.buf[2] != !blk {
            return Rx::Nak;
        }
        let (start, dlen) = if asm.start == proto::SOH {
            (4usize, asm.buf[3] as usize)
        } else {
            (3usize, proto::LONG_DATA_LEN)
        };
        if dlen == 0 || (asm.start == proto::SOH && dlen > proto::SHORT_DATA_LEN) {
            return Rx::Nak;
        }
        let data = &asm.buf[start..start + dlen];
        let crc = u16::from_be_bytes([asm.buf[start + dlen], asm.buf[start + dlen + 1]]);
        if proto::crc16(data) != crc {
            return Rx::Nak;
        }
        if self.nak_first_frame {
            // Models a frame that the device could not use at all: it answers
            // with a NAK and keeps its block bookkeeping untouched.
            self.nak_first_frame = false;
            return Rx::Nak;
        }

        // read_short_frame_data() / read_long_frame_data()
        if blk != 1 && self.recv_blk.wrapping_add(1) != blk && !self.first_connect {
            return Rx::Nak;
        }
        if blk == self.recv_blk {
            return Rx::Ack; // repeated frame, acknowledged but not copied
        }
        self.recv_blk = blk;
        self.first_connect = false;

        Rx::Frame(data.to_vec())
    }

    /// Look up a partition by name, like mtd_get_device() in the bootloader.
    fn partition_start(&self, name: &str) -> Option<u32> {
        fwc::parse_partition_table(&self.partition_table)
            .ok()?
            .iter()
            .find(|p| p.name == name)
            .map(|p| p.start as u32)
    }

    fn start_recv(&mut self, len: usize) {
        self.task = Some(Task {
            dir: Dir::HostSend,
            buf: Vec::with_capacity(len),
            len,
            xfer: 0,
        });
    }

    fn start_send(&mut self, data: Vec<u8>) {
        self.task = Some(Task {
            dir: Dir::HostRecv,
            len: data.len(),
            buf: data,
            xfer: 0,
        });
    }

    /// uart_upg_out_callback() -> data_trans_layer_out_proc()
    fn on_recv_done(&mut self) {
        let task = self.task.take().expect("receive task");
        match self.stage {
            Stage::ReadCbw => self.on_cbw(&task.buf),
            Stage::DataOut => {
                // The slice of a data transfer lands in trans_pkt_buf.
                let received = task.buf.clone();
                self.process_write(&received);
            }
            _ => self.fault(format!("receive task completed in stage {:?}", self.stage)),
        }
    }

    /// uart_upg_in_callback() -> data_trans_layer_in_proc()
    fn on_send_done(&mut self) {
        self.task.take();
        match self.stage {
            Stage::DataInBuf => self.process_read(),
            Stage::SendCsw => self.send_csw(proto::CSW_STATUS_PASSED),
            Stage::WaitCsw => {
                self.stage = Stage::ReadCbw;
                self.start_recv(proto::Cbw::LEN);
            }
            _ => self.fault(format!("send task completed in stage {:?}", self.stage)),
        }
    }

    //
    // Data transport layer
    //

    fn on_cbw(&mut self, buf: &[u8]) {
        let cbw = match parse_cbw(buf) {
            Some(cbw) => cbw,
            None => return self.fault("malformed CBW"),
        };
        if cbw.flags != 0 && cbw.flags != proto::CBW_FLAG_IN {
            return self.fault(format!("CBW with invalid flags 0x{:02X}", cbw.flags));
        }
        if cbw.transfer_len == 0 {
            return self.fault("CBW with an empty transfer");
        }

        self.cbw = Some(cbw);
        self.csw_residue = cbw.transfer_len;
        self.remain = cbw.transfer_len;

        match cbw.command {
            proto::TRANS_CMD_WRITE => {
                if cbw.flags != proto::CBW_FLAG_OUT {
                    return self.fault("write CBW without the host-to-device flag");
                }
                self.stage = Stage::DataOut; // write10 + processWriteBuf
                let slice = std::cmp::min(self.remain, TRANS_BUF as u32) as usize;
                self.start_recv(slice);
            }
            proto::TRANS_CMD_READ => {
                if cbw.flags != proto::CBW_FLAG_IN {
                    return self.fault("read CBW without the device-to-host flag");
                }
                self.stage = Stage::DataIn;
                self.process_read();
            }
            other => self.fault(format!("unknown transport command 0x{other:02X}")),
        }
    }

    /// AICUPG_processWrite()
    fn process_write(&mut self, received: &[u8]) {
        let n = received.len();
        self.trans_buf[..n].copy_from_slice(received);

        let consumed = self.data_packet_write(n);
        if consumed != n {
            self.fault(format!("data_packet_write consumed {consumed} of {n} bytes"));
        }
        self.remain -= n as u32;
        self.csw_residue = self.csw_residue.saturating_sub(n as u32);

        if self.remain == 0 {
            self.send_csw(proto::CSW_STATUS_PASSED);
        } else {
            let slice = std::cmp::min(self.remain, TRANS_BUF as u32) as usize;
            self.stage = Stage::DataOut;
            self.start_recv(slice);
        }
    }

    /// AICUPG_processRead()
    fn process_read(&mut self) {
        let slice = std::cmp::min(self.remain, TRANS_BUF as u32) as usize;
        let n = self.data_packet_read(slice);
        if n != slice {
            self.fault(format!("data_packet_read produced {n} of {slice} bytes"));
            self.send_csw(proto::CSW_STATUS_FAILED);
            return;
        }

        // AICUPG_processReadBuf()
        self.stage = Stage::DataInBuf;
        if self.remain <= n as u32 {
            self.remain = 0;
            self.stage = Stage::SendCsw;
        } else {
            self.remain -= n as u32;
        }
        self.start_send(self.trans_buf[..n].to_vec());
    }

    fn send_csw(&mut self, status: u8) {
        let cbw = self.cbw.expect("CBW");
        let mut csw = [0u8; proto::Csw::LEN];
        csw[0..4].copy_from_slice(&proto::CSW_SIGNATURE.to_le_bytes());
        csw[4..8].copy_from_slice(&cbw.tag.to_le_bytes());
        csw[8..12].copy_from_slice(&self.csw_residue.to_le_bytes());
        csw[12] = status;

        self.stage = Stage::WaitCsw;
        self.start_send(csw.to_vec());
    }

    //
    // Upgrading command layer
    //

    /// aicupg_data_packet_write()
    fn data_packet_write(&mut self, len: usize) -> usize {
        let packet = self.trans_buf[..len].to_vec();
        let mut clen = 0usize;

        if len >= 16 {
            if let Some(command) = parse_cmd_header(&packet[..16]) {
                self.cur_cmd = Some(command);
                self.cmd_state = CmdState::Start;
                self.priv_ = Priv::None;
                clen = 16;
            }
        }

        if clen == len {
            return clen;
        }

        if let Some(cmd) = self.cur_cmd {
            clen += self.cmd_write_input(cmd, &packet, len - clen);
        }

        if self.cmd_state == CmdState::End {
            self.end_command();
        }
        clen
    }

    /// aicupg_data_packet_read()
    fn data_packet_read(&mut self, len: usize) -> usize {
        let n = match self.cur_cmd {
            Some(cmd) => self.cmd_read_output(cmd, len),
            None => 0,
        };

        if self.cmd_state == CmdState::End {
            self.end_command();
        }
        n
    }

    fn end_command(&mut self) {
        self.priv_ = Priv::None;
        self.cmd_state = CmdState::Idle;

        // The "end" callbacks of fwc_cmd.c.
        match self.cur_cmd {
            Some(proto::cmd::SET_FWC_META) => {
                // set_fwc_meta_cmd_end() -> nor_fwc_start() / ram_fwc_start()
                self.fwc_block = if self.flash_ready { 2048 } else { 1 };
            }
            Some(proto::cmd::SEND_FWC_DATA) => {
                // send_fwc_data_cmd_end() -> ram_fwc_data_end(): the
                // "image.info" component switches the device to the flash.
                if matches!(&self.fwc, Some(meta) if meta.name == "image.info") && !self.ignore_image_info
                {
                    self.flash_ready = true;
                }
            }
            _ => {}
        }
    }

    /// cmd->write_input_data(), including the fact that a command header is
    /// always consumed as its own packet.
    fn cmd_write_input(&mut self, cmd: u8, buf: &[u8], len: usize) -> usize {
        match cmd {
            proto::cmd::WRITE => {
                let mut clen = 0usize;
                if self.cmd_state == CmdState::Start {
                    self.cmd_state = CmdState::Arg;
                }
                if self.cmd_state == CmdState::Arg {
                    if len < 8 {
                        return 0;
                    }
                    let addr = u32::from_le_bytes(buf[0..4].try_into().unwrap());
                    let total = u32::from_le_bytes(buf[4..8].try_into().unwrap());
                    self.priv_ = Priv::Write {
                        addr,
                        len: total,
                        index: 0,
                    };
                    clen += 8;
                    self.cmd_state = CmdState::DataIn;
                }
                if clen == len {
                    return clen;
                }
                if self.cmd_state == CmdState::DataIn {
                    let n = len - clen;
                    if let Priv::Write { addr, len, index } = &mut self.priv_ {
                        for (i, byte) in buf[clen..clen + n].iter().enumerate() {
                            self.mem.insert(addr.wrapping_add(*index + i as u32), *byte);
                        }
                        *index += n as u32;
                        if *index >= *len {
                            self.cmd_state = CmdState::Resp;
                        }
                    }
                    clen += n;
                }
                clen
            }
            proto::cmd::EXEC => {
                let mut clen = 0usize;
                let mut addr = 0u32;
                if self.cmd_state == CmdState::Start {
                    self.cmd_state = CmdState::Arg;
                }
                if self.cmd_state == CmdState::Arg {
                    if len < 4 {
                        return 0;
                    }
                    addr = u32::from_le_bytes(buf[0..4].try_into().unwrap());
                    clen = 4;
                    self.cmd_state = CmdState::Resp;
                }
                if addr == 0 {
                    self.priv_ = Priv::Exec { status: 1 };
                    return clen;
                }
                // The payload runs inside the bootloader, its return value
                // decides the status of the command.
                self.exec_calls.push(addr);
                self.priv_ = Priv::Exec {
                    status: if self.exec_ret != 0 { 1 } else { 0 },
                };
                clen
            }
            proto::cmd::GET_HWINFO => 0,
            proto::cmd::RUN_SHELL_STR => {
                // run_shell_str_cmd_write_input_data(): [len u32][command]
                let mut clen = 0usize;
                if self.cmd_state == CmdState::Start {
                    self.cmd_state = CmdState::Arg;
                }
                let mut cmdlen = 0usize;
                if self.cmd_state == CmdState::Arg {
                    if len < 4 {
                        return 0;
                    }
                    cmdlen = u32::from_le_bytes(buf[0..4].try_into().unwrap()) as usize;
                    clen += 4;
                    self.cmd_state = CmdState::DataIn;
                }
                if clen == len {
                    return clen;
                }
                if self.cmd_state == CmdState::DataIn {
                    if (len - clen) != cmdlen || cmdlen >= MAX_SHELL_CMD_STR_LEN {
                        self.priv_ = Priv::Exec { status: 1 };
                        clen += len - clen;
                        self.cmd_state = CmdState::Resp;
                        return clen;
                    }
                    let command = String::from_utf8_lossy(&buf[clen..clen + cmdlen]).into_owned();
                    clen += cmdlen;
                    self.shell_commands.push(command.clone());
                    self.priv_ = Priv::Exec { status: 0 };
                    self.cmd_state = CmdState::Resp;
                    // console_run_cmd(): "reset" reboots the board right here
                    // and never returns.
                    if command == "reset" || command == "reboot" {
                        self.rebooted = true;
                        self.halted = true;
                    }
                }
                clen
            }
            proto::cmd::SET_FWC_META => {
                // set_fwc_meta_cmd_write_input_data()
                let mut clen = 0usize;
                if self.cmd_state == CmdState::Start {
                    self.cmd_state = CmdState::DataIn;
                }
                if self.cmd_state == CmdState::DataIn {
                    let n = std::cmp::min(len, fwc::META_LEN);
                    let meta = parse_fwc_meta(&buf[..n]);
                    self.fwc = Some(meta);
                    self.fwc_written = 0;
                    self.fwc_crc = 0;
                    self.cmd_state = CmdState::Resp;
                    clen = n;
                }
                clen
            }
            proto::cmd::SEND_FWC_DATA => {
                // send_fwc_data_cmd_write_input_data() -> nor_fwc_data_write()
                let mut clen = 0usize;
                if self.cmd_state == CmdState::Start {
                    self.cmd_state = CmdState::DataIn;
                }
                if self.cmd_state == CmdState::DataIn {
                    let meta = match self.fwc.clone() {
                        Some(meta) => meta,
                        None => {
                            self.fault("SEND_FWC_DATA without a component");
                            return 0;
                        }
                    };

                    // The CRC covers at most meta.size bytes of the component.
                    let calc = std::cmp::min(len as u32, meta.size.saturating_sub(self.fwc_written))
                        as usize;
                    self.fwc_crc = fwc::crc32_fold(self.fwc_crc, &buf[..calc]);

                    if self.flash_ready && meta.attr.contains("mtd") {
                        let start = match self.partition_start(&meta.partition) {
                            Some(start) => start,
                            None => {
                                self.fault(format!("MTD {} part not found", meta.partition));
                                return 0;
                            }
                        };
                        for (i, byte) in buf[..len].iter().enumerate() {
                            self.flash
                                .insert(start + self.fwc_written + i as u32, *byte);
                        }
                    }

                    self.fwc_written += len as u32;
                    clen = len;
                    if self.fwc_written >= meta.size {
                        self.cmd_state = CmdState::Resp;
                    }
                }
                clen
            }
            proto::cmd::GET_PARTITION_TABLE => {
                // get_partition_table_cmd_write_input_data(): struct storage_media
                let mut clen = 0usize;
                if self.cmd_state == CmdState::Start {
                    self.cmd_state = CmdState::Arg;
                }
                if self.cmd_state == CmdState::Arg {
                    if len < fwc::MEDIA_LEN {
                        return 0;
                    }
                    let media = parse_str(&buf[0..fwc::FWC_STR_MAX_LEN]);
                    if !media.starts_with("spi-nor") {
                        self.fault(format!("partition table asked for media '{media}'"));
                    }
                    clen += fwc::MEDIA_LEN;
                    self.cmd_state = CmdState::Resp;
                }
                clen
            }
            proto::cmd::READ_FWC_DATA => {
                // read_fwc_data_cmd_write_input_data(): struct media_partition
                let mut clen = 0usize;
                if self.cmd_state == CmdState::Start {
                    self.cmd_state = CmdState::Arg;
                }
                if self.cmd_state == CmdState::Arg {
                    if len < fwc::MEDIA_PARTITION_LEN {
                        return 0;
                    }
                    let name = parse_str(&buf[72..72 + fwc::MAX_PARTITION_NAME]);
                    let size = u64::from_le_bytes(buf[232..240].try_into().unwrap());
                    let start = match self.partition_start(&name) {
                        Some(start) => start,
                        None => {
                            self.fault(format!("MTD {name} part not found for a read"));
                            return 0;
                        }
                    };
                    self.read_start = start as u64;
                    self.read_size = size;
                    self.read_done = 0;
                    clen += fwc::MEDIA_PARTITION_LEN;
                    self.cmd_state = CmdState::Resp;
                }
                clen
            }
            proto::cmd::GET_BLOCK_SIZE | proto::cmd::GET_FWC_CRC | proto::cmd::GET_FWC_BURN_RESULT => 0,
            proto::cmd::SET_UART_ARGS => {
                let mut clen = 0usize;
                if self.cmd_state == CmdState::Start {
                    self.cmd_state = CmdState::Arg;
                }
                if self.cmd_state == CmdState::Arg {
                    if len < 4 {
                        return 0;
                    }
                    self.pending_baud = Some(u32::from_le_bytes(buf[0..4].try_into().unwrap()));
                    clen += 4;
                    self.cmd_state = CmdState::Resp;
                }
                clen
            }
            other => {
                self.fault(format!("write input for unsupported command 0x{other:02X}"));
                0
            }
        }
    }

    /// cmd->read_output_data()
    fn cmd_read_output(&mut self, cmd: u8, len: usize) -> usize {
        match cmd {
            proto::cmd::WRITE => {
                if self.cmd_state != CmdState::Resp {
                    self.fault(format!(
                        "write response requested in state {:?}",
                        self.cmd_state
                    ));
                    return 0;
                }
                let ok = matches!(&self.priv_, Priv::Write { index, len, .. } if index >= len);
                let status = if ok { proto::RESP_OK } else { 1 };
                let header = resp_header(proto::cmd::WRITE, status, 0);
                self.trans_buf[..16].copy_from_slice(&header);
                self.cmd_state = CmdState::End;
                16
            }
            proto::cmd::EXEC => {
                if self.cmd_state != CmdState::Resp {
                    self.fault(format!(
                        "exec response requested in state {:?}",
                        self.cmd_state
                    ));
                    return 0;
                }
                let status = match self.priv_ {
                    Priv::Exec { status } => status,
                    _ => 1,
                };
                let header = resp_header(proto::cmd::EXEC, status, 0);
                self.trans_buf[..16].copy_from_slice(&header);
                self.cmd_state = CmdState::End;
                16
            }
            proto::cmd::RUN_SHELL_STR => {
                // run_shell_str_cmd_read_output_data()
                if self.cmd_state != CmdState::Resp {
                    self.fault(format!(
                        "shell command response in state {:?}",
                        self.cmd_state
                    ));
                    return 0;
                }
                let status = match self.priv_ {
                    Priv::Exec { status } => status,
                    _ => 1,
                };
                let header = resp_header(proto::cmd::RUN_SHELL_STR, status, 0);
                self.trans_buf[..16].copy_from_slice(&header);
                self.cmd_state = CmdState::End;
                16
            }
            proto::cmd::GET_HWINFO => {
                let mut siz = 0usize;
                if self.cmd_state == CmdState::Start {
                    self.cmd_state = CmdState::Resp;
                }
                if self.cmd_state == CmdState::Resp {
                    let header = resp_header(proto::cmd::GET_HWINFO, proto::RESP_OK, HWINFO_LEN as u32);
                    self.trans_buf[..16].copy_from_slice(&header);
                    siz = 16;
                    self.cmd_state = CmdState::DataOut;
                }
                if siz == len {
                    return siz;
                }
                if self.cmd_state == CmdState::DataOut {
                    let info = self.hw_info.clone();
                    self.trans_buf[16..16 + info.len()].copy_from_slice(&info);
                    siz += info.len();
                    self.cmd_state = CmdState::End;
                }
                siz
            }
            proto::cmd::SET_FWC_META => {
                // set_fwc_meta_cmd_read_output_data()
                if self.cmd_state != CmdState::Resp {
                    self.fault(format!(
                        "component description response in state {:?}",
                        self.cmd_state
                    ));
                    return 0;
                }
                let header = resp_header(proto::cmd::SET_FWC_META, proto::RESP_OK, 0);
                self.trans_buf[..16].copy_from_slice(&header);
                self.cmd_state = CmdState::End;
                16
            }
            proto::cmd::SEND_FWC_DATA => {
                // send_fwc_data_cmd_read_output_data()
                let status = if self.cmd_state == CmdState::Resp {
                    proto::RESP_OK
                } else {
                    proto::RESP_FAIL
                };
                let header = resp_header(proto::cmd::SEND_FWC_DATA, status, 0);
                self.trans_buf[..16].copy_from_slice(&header);
                self.cmd_state = CmdState::End;
                16
            }
            proto::cmd::GET_BLOCK_SIZE | proto::cmd::GET_FWC_CRC | proto::cmd::GET_FWC_BURN_RESULT => {
                let complete = matches!(&self.fwc, Some(meta) if self.fwc_written >= meta.size);
                let value = match cmd {
                    proto::cmd::GET_BLOCK_SIZE => self.fwc_block,
                    proto::cmd::GET_FWC_CRC => self.fwc_crc,
                    _ => u32::from(!complete),
                };
                let mut siz = 0usize;
                if self.cmd_state == CmdState::Start {
                    self.cmd_state = CmdState::Resp;
                }
                if self.cmd_state == CmdState::Resp {
                    let header = resp_header(cmd, proto::RESP_OK, 4);
                    self.trans_buf[..16].copy_from_slice(&header);
                    siz = 16;
                    self.cmd_state = CmdState::DataOut;
                }
                if siz == len {
                    return siz;
                }
                if self.cmd_state == CmdState::DataOut {
                    // The device writes the value to the start of the buffer,
                    // the response header of the previous step is gone.
                    self.trans_buf[0..4].copy_from_slice(&value.to_le_bytes());
                    siz += 4;
                    self.cmd_state = CmdState::End;
                }
                siz
            }
            proto::cmd::GET_PARTITION_TABLE => {
                let mut siz = 0usize;
                if self.cmd_state == CmdState::Start {
                    self.cmd_state = CmdState::Resp;
                }
                if self.cmd_state == CmdState::Resp {
                    let header = resp_header(
                        proto::cmd::GET_PARTITION_TABLE,
                        proto::RESP_OK,
                        fwc::PARTITION_TABLE_LEN as u32,
                    );
                    self.trans_buf[..16].copy_from_slice(&header);
                    siz = 16;
                    self.cmd_state = CmdState::DataOut;
                }
                if siz == len {
                    return siz;
                }
                if self.cmd_state == CmdState::DataOut {
                    let mut table = vec![0u8; fwc::PARTITION_TABLE_LEN];
                    let text = self.partition_table.as_bytes();
                    let n = std::cmp::min(text.len(), table.len());
                    table[..n].copy_from_slice(&text[..n]);
                    // Like the device: the table goes to the start of the
                    // buffer, replacing the response header.
                    self.trans_buf[..table.len()].copy_from_slice(&table);
                    siz += table.len();
                    self.cmd_state = CmdState::End;
                }
                siz
            }
            proto::cmd::READ_FWC_DATA => {
                let mut siz = 0usize;
                if self.cmd_state == CmdState::Start {
                    self.cmd_state = CmdState::Resp;
                }
                if self.cmd_state == CmdState::Resp {
                    let header = resp_header(
                        proto::cmd::READ_FWC_DATA,
                        proto::RESP_OK,
                        self.read_size as u32,
                    );
                    self.trans_buf[..16].copy_from_slice(&header);
                    siz = 16;
                    self.cmd_state = CmdState::DataOut;
                }
                if siz == len {
                    return siz;
                }
                if self.cmd_state == CmdState::DataOut {
                    // nor_fwc_data_read(): the flash data goes to buf[0..len]
                    // and the return value is that length, the header of the
                    // response is replaced.
                    for i in 0..len {
                        let at = self.read_start + self.read_done + i as u64;
                        let byte = *self.flash.get(&(at as u32)).unwrap_or(&0xFF);
                        self.trans_buf[i] = byte;
                    }
                    self.read_done += len as u64;
                    if self.read_done >= self.read_size {
                        self.cmd_state = CmdState::End;
                    }
                    return len;
                }
                siz
            }
            proto::cmd::SET_UART_ARGS => {
                let mut siz = 0usize;
                if self.cmd_state == CmdState::Start {
                    self.cmd_state = CmdState::Resp;
                }
                if self.cmd_state == CmdState::Resp {
                    let header = resp_header(proto::cmd::SET_UART_ARGS, proto::RESP_OK, 0);
                    self.trans_buf[..16].copy_from_slice(&header);
                    // set_uart_args_cmd_end(): the device applies the new speed
                    // once the command is over.
                    if let Some(baud) = self.pending_baud.take() {
                        self.baud_updates.push(baud);
                    }
                    self.cmd_state = CmdState::End;
                    siz = 16;
                }
                siz
            }
            other => {
                self.fault(format!("read output for unsupported command 0x{other:02X}"));
                0
            }
        }
    }
}

/// The device side of the link.
pub struct SimLink {
    sim: Rc<RefCell<Sim>>,
}

impl SimLink {
    pub fn new(sim: Rc<RefCell<Sim>>) -> Self {
        SimLink { sim }
    }
}

impl Link for SimLink {
    fn write_all(&mut self, data: &[u8]) -> Result<(), String> {
        let mut sim = self.sim.borrow_mut();
        sim.rx.extend(data.iter().copied());
        sim.pump();
        Ok(())
    }

    fn read(&mut self, buf: &mut [u8], _deadline: Instant) -> Result<usize, String> {
        let mut sim = self.sim.borrow_mut();
        sim.pump();
        let n = std::cmp::min(buf.len(), sim.to_host.len());
        for slot in buf.iter_mut().take(n) {
            *slot = sim.to_host.pop_front().unwrap();
        }
        Ok(n)
    }

    fn flush(&mut self) {
        let mut sim = self.sim.borrow_mut();
        sim.pump();
        sim.to_host.clear();
    }
}

//
// Message parsing helpers of the device side
//

fn parse_cbw(buf: &[u8]) -> Option<CbwInfo> {
    if buf.len() < proto::Cbw::LEN {
        return None;
    }
    let signature = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
    if signature != proto::CBW_SIGNATURE {
        return None;
    }
    if buf[14] == 0 {
        // bCBWCBLength
        return None;
    }
    Some(CbwInfo {
        tag: u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]),
        transfer_len: u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
        flags: buf[12],
        command: buf[15],
    })
}

/// check_cmd_header()
fn parse_cmd_header(buf: &[u8]) -> Option<u8> {
    if buf.len() < 16 {
        return None;
    }
    let magic = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
    let protocol = buf[4];
    let version = buf[5];
    let command = buf[6];
    let reserved = buf[7];
    let data_length = u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]);
    let checksum = u32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]);

    if magic != proto::CMD_MAGIC || protocol != proto::PROTO_TYPE || version != proto::PROTO_VERSION
    {
        return None;
    }
    let expect = magic
        .wrapping_add(
            ((reserved as u32) << 24)
                | ((command as u32) << 16)
                | ((version as u32) << 8)
                | protocol as u32,
        )
        .wrapping_add(data_length);
    if checksum != expect {
        return None;
    }
    Some(command)
}

/// aicupg_gen_resp()
fn resp_header(command: u8, status: u8, data_length: u32) -> [u8; 16] {
    let checksum = proto::RESP_MAGIC
        .wrapping_add(
            ((status as u32) << 24)
                | ((command as u32) << 16)
                | ((proto::PROTO_VERSION as u32) << 8)
                | proto::PROTO_TYPE as u32,
        )
        .wrapping_add(data_length);

    let mut buf = [0u8; 16];
    buf[0..4].copy_from_slice(&proto::RESP_MAGIC.to_le_bytes());
    buf[4] = proto::PROTO_TYPE;
    buf[5] = proto::PROTO_VERSION;
    buf[6] = command;
    buf[7] = status;
    buf[8..12].copy_from_slice(&data_length.to_le_bytes());
    buf[12..16].copy_from_slice(&checksum.to_le_bytes());
    buf
}

/// The fields of struct fwc_meta the model needs (upg_internal.h).
fn parse_fwc_meta(buf: &[u8]) -> SimMeta {
    SimMeta {
        name: parse_str(&buf[8..8 + fwc::FWC_STR_MAX_LEN]),
        partition: parse_str(&buf[72..72 + fwc::FWC_STR_MAX_LEN]),
        size: u32::from_le_bytes(buf[140..144].try_into().unwrap()),
        attr: parse_str(&buf[152..152 + fwc::FWC_STR_MAX_LEN]),
    }
}

/// A NUL terminated string inside a fixed size field.
fn parse_str(bytes: &[u8]) -> String {
    let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    String::from_utf8_lossy(&bytes[..end]).into_owned()
}
