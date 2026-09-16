// Copyright (c) 2026, ArtInChip Technology Co., Ltd
//
// SPDX-License-Identifier: Apache-2.0
//
// Wire format of the ArtInChip upgrading ("aicupg") protocol implemented by
// the baremetal bootloader in application/baremetal/bootloader/lib/aicupg/.
//
// Three layers are involved:
//
//   1. the UART protocol layer (uart_proto_layer.c) that wraps every transfer
//      into a block numbered frame protected by a CRC16, each frame has to be
//      acknowledged by the receiver,
//   2. the data transport layer (data_trans_layer.c) that moves CBW/CSW pairs
//      over those frames in the same way a USB BOT transfer does,
//   3. the upgrading command layer (basic_cmd.c, fwc_cmd.c) that carries a
//      cmd_header plus payload inside every CBW transfer.
//
// Little endian is used for all multi byte fields.

/// Frame delimiters and single byte handshake characters.
pub const SOH: u8 = 0x01; // frame with 1..=176 data bytes
pub const STX: u8 = 0x02; // frame with exactly 1024 data bytes
pub const ACK: u8 = 0x06;
pub const DC1_SEND: u8 = 0x11; // host: "I want to send data"
pub const NAK: u8 = 0x15;
pub const CAN: u8 = 0x18; // device: sent when it enters upgrading mode
pub const SIG_C: u8 = 0x43; // host probe, the device answers with ACK

/// Data length of a short / long frame (SHT_FRM_DLEN / LNG_FRM_DLEN).
pub const SHORT_DATA_LEN: usize = 176;
pub const LONG_DATA_LEN: usize = 1024;

/// Command header magic, "UPGC".
pub const CMD_MAGIC: u32 = 0x4347_5055;
/// Response header magic, "UPGR".
pub const RESP_MAGIC: u32 = 0x5247_5055;

pub const PROTO_TYPE: u8 = 0x01;
pub const PROTO_VERSION: u8 = 0x01;

/// Command IDs (UPG_PROTO_CMD_xxx in aicupg.h).
pub mod cmd {
    pub const GET_HWINFO: u8 = 0x00;
    pub const WRITE: u8 = 0x02;
    pub const EXEC: u8 = 0x04;
    pub const RUN_SHELL_STR: u8 = 0x05;
    pub const SET_FWC_META: u8 = 0x10;
    pub const GET_BLOCK_SIZE: u8 = 0x11;
    pub const SEND_FWC_DATA: u8 = 0x12;
    pub const GET_FWC_CRC: u8 = 0x13;
    pub const GET_FWC_BURN_RESULT: u8 = 0x14;
    pub const GET_PARTITION_TABLE: u8 = 0x17;
    pub const READ_FWC_DATA: u8 = 0x18;
    pub const SET_UART_ARGS: u8 = 0x19;
}

/// CBW/CSW signatures and flags (data_trans_layer.h).
pub const CBW_SIGNATURE: u32 = 0x4342_5355; // "USBC"
pub const CSW_SIGNATURE: u32 = 0x5342_5355; // "USBS"

pub const CBW_FLAG_OUT: u8 = 0x00; // data host -> device
pub const CBW_FLAG_IN: u8 = 0x80; // data device -> host

pub const TRANS_CMD_WRITE: u8 = 0x01;
pub const TRANS_CMD_READ: u8 = 0x02;

pub const CSW_STATUS_PASSED: u8 = 0x00;
pub const CSW_STATUS_FAILED: u8 = 0x01;
pub const CSW_STATUS_PHASE_ERR: u8 = 0x02;

/// CRC16-CCITT as used by the bootloader (lib/common/crc16.c): polynomial
/// 0x1021, initial value 0, no reflection and no final xor. The check value
/// for "123456789" is 0x31C3.
pub fn crc16(data: &[u8]) -> u16 {
    const TABLE: [u16; 256] = build_crc16_table();
    let mut crc: u16 = 0;
    for &b in data {
        crc = (crc << 8) ^ TABLE[(((crc >> 8) ^ b as u16) & 0xFF) as usize];
    }
    crc
}

const fn build_crc16_table() -> [u16; 256] {
    let mut table = [0u16; 256];
    let mut i = 0usize;
    while i < 256 {
        let mut crc = (i as u16) << 8;
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
            bit += 1;
        }
        table[i] = crc;
        i += 1;
    }
    table
}

/// Build a protocol frame (pack_frame() in uart_proto_layer.c).
///
/// `data` must be either 1..=176 bytes (short frame) or exactly 1024 bytes
/// (long frame), anything else is rejected by the device.
pub fn pack_frame(blk: u8, data: &[u8]) -> Result<Vec<u8>, String> {
    let crc = crc16(data);
    let mut frame = Vec::with_capacity(data.len() + 6);
    frame.push(blk);
    frame.push(!blk); // the device checks blk1 == 255 - blk2

    if data.len() > SHORT_DATA_LEN {
        if data.len() != LONG_DATA_LEN {
            return Err(format!(
                "frame data length {} is not supported (max {}, or exactly {})",
                data.len(),
                SHORT_DATA_LEN,
                LONG_DATA_LEN
            ));
        }
        frame.insert(0, STX);
    } else {
        if data.is_empty() {
            return Err("frame data must not be empty".to_string());
        }
        frame.insert(0, SOH);
        frame.insert(3, data.len() as u8);
    }

    frame.extend_from_slice(data);
    frame.push((crc >> 8) as u8);
    frame.push(crc as u8);
    Ok(frame)
}

/// A command block wrapper, see struct aic_cbw in data_trans_layer.h.
#[derive(Debug, Clone, Copy)]
pub struct Cbw {
    pub tag: u32,
    pub transfer_len: u32,
    pub flags: u8,
    pub command: u8,
}

impl Cbw {
    pub const LEN: usize = 31;

    pub fn to_bytes(self) -> [u8; Self::LEN] {
        let mut buf = [0u8; Self::LEN];
        buf[0..4].copy_from_slice(&CBW_SIGNATURE.to_le_bytes());
        buf[4..8].copy_from_slice(&self.tag.to_le_bytes());
        buf[8..12].copy_from_slice(&self.transfer_len.to_le_bytes());
        buf[12] = self.flags;
        buf[13] = 0; // reserved
        buf[14] = 1; // bCBWCBLength, the device rejects a zero
        buf[15] = self.command;
        buf
    }
}

/// A command status wrapper, see struct aic_csw in data_trans_layer.h.
#[derive(Debug, Clone, Copy)]
pub struct Csw {
    pub tag: u32,
    pub residue: u32,
    pub status: u8,
}

impl Csw {
    pub const LEN: usize = 13;

    pub fn parse(buf: &[u8]) -> Result<Self, String> {
        if buf.len() < Self::LEN {
            return Err(format!("CSW is {} bytes, expected {}", buf.len(), Self::LEN));
        }
        let signature = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        if signature != CSW_SIGNATURE {
            return Err(format!("bad CSW signature 0x{signature:08X}"));
        }
        Ok(Csw {
            tag: u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]),
            residue: u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
            status: buf[12],
        })
    }

    pub fn status_str(&self) -> &'static str {
        match self.status {
            CSW_STATUS_PASSED => "passed",
            CSW_STATUS_FAILED => "failed",
            CSW_STATUS_PHASE_ERR => "phase error",
            _ => "unknown",
        }
    }
}

/// Build a 16 byte command header (struct cmd_header in aicupg.h).
pub fn cmd_header(command: u8, data_length: u32) -> [u8; 16] {
    let mut buf = [0u8; 16];
    let checksum = CMD_MAGIC
        .wrapping_add(((command as u32) << 16) | ((PROTO_VERSION as u32) << 8) | PROTO_TYPE as u32)
        .wrapping_add(data_length);

    buf[0..4].copy_from_slice(&CMD_MAGIC.to_le_bytes());
    buf[4] = PROTO_TYPE;
    buf[5] = PROTO_VERSION;
    buf[6] = command;
    buf[7] = 0; // reserved
    buf[8..12].copy_from_slice(&data_length.to_le_bytes());
    buf[12..16].copy_from_slice(&checksum.to_le_bytes());
    buf
}

/// A parsed response header (struct resp_header in aicupg.h).
#[derive(Debug, Clone, Copy)]
pub struct RespHeader {
    pub command: u8,
    pub status: u8,
    pub data_length: u32,
}

pub const RESP_HEADER_LEN: usize = 16;
/// Response status of the device (UPG_RESP_OK / UPG_RESP_FAIL).
pub const RESP_OK: u8 = 0;
pub const RESP_FAIL: u8 = 1;

impl RespHeader {
    pub fn parse(buf: &[u8]) -> Result<Self, String> {
        if buf.len() < RESP_HEADER_LEN {
            return Err(format!(
                "response header is {} bytes, expected {}",
                buf.len(),
                RESP_HEADER_LEN
            ));
        }
        let magic = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        if magic != RESP_MAGIC {
            return Err(format!("bad response magic 0x{magic:08X}"));
        }
        let protocol = buf[4];
        let version = buf[5];
        let command = buf[6];
        let status = buf[7];
        let data_length = u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]);
        let checksum = u32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]);

        if protocol != PROTO_TYPE || version != PROTO_VERSION {
            return Err(format!(
                "unsupported protocol {protocol}.{version} in response"
            ));
        }

        let expect = RESP_MAGIC
            .wrapping_add(
                ((status as u32) << 24)
                    | ((command as u32) << 16)
                    | ((version as u32) << 8)
                    | protocol as u32,
            )
            .wrapping_add(data_length);
        if checksum != expect {
            return Err(format!(
                "bad response checksum 0x{checksum:08X}, expected 0x{expect:08X}"
            ));
        }

        Ok(RespHeader {
            command,
            status,
            data_length,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc16_check_value() {
        assert_eq!(crc16(b"123456789"), 0x31C3);
        assert_eq!(crc16(b""), 0x0000);
    }

    #[test]
    fn short_frame_layout() {
        let frame = pack_frame(1, &[0xAA, 0xBB]).unwrap();
        assert_eq!(frame[0], SOH);
        assert_eq!(frame[1], 1);
        assert_eq!(frame[2], 0xFE);
        assert_eq!(frame[3], 2);
        assert_eq!(&frame[4..6], &[0xAA, 0xBB]);
        let crc = crc16(&[0xAA, 0xBB]);
        assert_eq!(frame[6], (crc >> 8) as u8);
        assert_eq!(frame[7], crc as u8);
        assert_eq!(frame.len(), 8);
    }

    #[test]
    fn long_frame_layout() {
        let data = vec![0x5A; LONG_DATA_LEN];
        let frame = pack_frame(7, &data).unwrap();
        assert_eq!(frame[0], STX);
        assert_eq!(frame[1], 7);
        assert_eq!(frame[2], 0xF8);
        assert_eq!(frame.len(), LONG_DATA_LEN + 5);
    }

    #[test]
    fn unsupported_frame_length_is_rejected() {
        assert!(pack_frame(1, &[0u8; 177]).is_err());
        assert!(pack_frame(1, &[]).is_err());
    }

    #[test]
    fn cmd_header_checksum_matches_device() {
        let hdr = cmd_header(cmd::WRITE, 0x2000);
        let sum = u32::from_le_bytes([hdr[0], hdr[1], hdr[2], hdr[3]])
            .wrapping_add(u32::from_le_bytes([hdr[4], hdr[5], hdr[6], hdr[7]]))
            .wrapping_add(u32::from_le_bytes([hdr[8], hdr[9], hdr[10], hdr[11]]));
        assert_eq!(sum, u32::from_le_bytes([hdr[12], hdr[13], hdr[14], hdr[15]]));
    }

    #[test]
    fn resp_header_round_trip() {
        // Build what aicupg_gen_resp() would produce and parse it back.
        let mut buf = [0u8; RESP_HEADER_LEN];
        let (command, status, data_length) = (cmd::GET_HWINFO, RESP_OK, 108u32);
        let checksum = RESP_MAGIC
            .wrapping_add(
                ((status as u32) << 24)
                    | ((command as u32) << 16)
                    | ((PROTO_VERSION as u32) << 8)
                    | PROTO_TYPE as u32,
            )
            .wrapping_add(data_length);
        buf[0..4].copy_from_slice(&RESP_MAGIC.to_le_bytes());
        buf[4] = PROTO_TYPE;
        buf[5] = PROTO_VERSION;
        buf[6] = command;
        buf[7] = status;
        buf[8..12].copy_from_slice(&data_length.to_le_bytes());
        buf[12..16].copy_from_slice(&checksum.to_le_bytes());

        let resp = RespHeader::parse(&buf).unwrap();
        assert_eq!(resp.command, cmd::GET_HWINFO);
        assert_eq!(resp.data_length, 108);
        assert_eq!(resp.status, RESP_OK);
    }
}
