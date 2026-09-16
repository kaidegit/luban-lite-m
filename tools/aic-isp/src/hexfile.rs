// Copyright (c) 2026, ArtInChip Technology Co., Ltd
//
// SPDX-License-Identifier: Apache-2.0
//
// Minimal Intel HEX reader.

use std::fs;
use std::path::Path;

/// A run of consecutive data bytes.
#[derive(Debug)]
pub struct Segment {
    pub addr: u32,
    pub data: Vec<u8>,
}

#[derive(Debug)]
pub struct HexImage {
    pub segments: Vec<Segment>,
    /// Entry address from a type 03/05 record, if the file has one.
    pub entry: Option<u32>,
}

impl HexImage {
    pub fn load(path: &Path) -> Result<Self, String> {
        let text = fs::read_to_string(path)
            .map_err(|e| format!("cannot read {}: {e}", path.display()))?;
        Self::parse(&text).map_err(|e| format!("{}: {e}", path.display()))
    }

    pub fn total_len(&self) -> usize {
        self.segments.iter().map(|s| s.data.len()).sum()
    }

    pub fn parse(text: &str) -> Result<Self, String> {
        let mut segments: Vec<Segment> = Vec::new();
        let mut base: u32 = 0;
        let mut entry = None;
        let mut eof = false;

        for (idx, raw) in text.lines().enumerate() {
            let lineno = idx + 1;
            let line = raw.trim();
            if line.is_empty() {
                continue;
            }
            if eof {
                return Err(format!("line {lineno}: record after end-of-file"));
            }

            let body = line
                .strip_prefix(':')
                .ok_or_else(|| format!("line {lineno}: record does not start with ':'"))?;
            if body.len() % 2 != 0 {
                return Err(format!("line {lineno}: odd number of hexadecimal digits"));
            }
            let bytes = (0..body.len() / 2)
                .map(|i| {
                    u8::from_str_radix(&body[2 * i..2 * i + 2], 16)
                        .map_err(|_| format!("line {lineno}: invalid hexadecimal digit"))
                })
                .collect::<Result<Vec<u8>, String>>()?;

            if bytes.len() < 5 {
                return Err(format!("line {lineno}: record is too short"));
            }
            let count = bytes[0] as usize;
            if bytes.len() != count + 5 {
                return Err(format!(
                    "line {lineno}: byte count {count} does not match the record length {}",
                    bytes.len()
                ));
            }
            if bytes.iter().fold(0u8, |acc, &b| acc.wrapping_add(b)) != 0 {
                return Err(format!("line {lineno}: checksum error"));
            }

            let addr = u16::from_be_bytes([bytes[1], bytes[2]]) as u32;
            let payload = &bytes[4..4 + count];
            match bytes[3] {
                0x00 => {
                    let abs = base
                        .checked_add(addr)
                        .ok_or_else(|| format!("line {lineno}: address overflow"))?;
                    push_data(&mut segments, abs, payload);
                }
                0x01 => eof = true,
                0x02 => {
                    let seg = two_bytes(payload, lineno)?;
                    base = seg as u32 * 16;
                }
                0x03 => {} // start segment address, not needed for loading
                0x04 => {
                    let seg = two_bytes(payload, lineno)?;
                    base = (seg as u32) << 16;
                }
                0x05 => {
                    if payload.len() != 4 {
                        return Err(format!("line {lineno}: start address record is malformed"));
                    }
                    entry = Some(u32::from_be_bytes([
                        payload[0], payload[1], payload[2], payload[3],
                    ]));
                }
                other => {
                    return Err(format!(
                        "line {lineno}: unsupported record type 0x{other:02X}"
                    ))
                }
            }
        }

        if segments.is_empty() {
            return Err("no data records found".to_string());
        }
        Ok(HexImage { segments, entry })
    }
}

fn two_bytes(payload: &[u8], lineno: usize) -> Result<u16, String> {
    if payload.len() != 2 {
        return Err(format!("line {lineno}: segment address record is malformed"));
    }
    Ok(u16::from_be_bytes([payload[0], payload[1]]))
}

/// Append a record, merging it into the previous segment when contiguous.
fn push_data(segments: &mut Vec<Segment>, addr: u32, data: &[u8]) {
    if let Some(last) = segments.last_mut() {
        let end = last.addr as u64 + last.data.len() as u64;
        if end == addr as u64 {
            last.data.extend_from_slice(data);
            return;
        }
    }
    segments.push(Segment {
        addr,
        data: data.to_vec(),
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_and_merges_consecutive_records() {
        let image = HexImage::parse(
            ":03000000010203F7\n\
             :03000300040506EB\n\
             :00000001FF\n",
        )
        .unwrap();
        assert_eq!(image.segments.len(), 1);
        assert_eq!(image.segments[0].addr, 0x0000_0000);
        assert_eq!(
            image.segments[0].data,
            vec![1, 2, 3, 4, 5, 6]
        );
        assert_eq!(image.total_len(), 6);
        assert!(image.entry.is_none());
    }

    #[test]
    fn extended_linear_address_is_applied() {
        let image = HexImage::parse(
            ":020000040001F9\n\
             :02001000AABB89\n\
             :00000001FF\n",
        )
        .unwrap();
        assert_eq!(image.segments[0].addr, 0x0001_0010);
        assert_eq!(image.segments[0].data, vec![0xAA, 0xBB]);
    }

    #[test]
    fn gaps_produce_separate_segments() {
        let image = HexImage::parse(
            ":0100000001FE\n\
             :0100080002F5\n\
             :00000001FF\n",
        )
        .unwrap();
        assert_eq!(image.segments.len(), 2);
        assert_eq!(image.segments[1].addr, 8);
    }

    #[test]
    fn bad_checksum_is_reported() {
        let err = HexImage::parse(":0100000001FF\n:00000001FF\n").unwrap_err();
        assert!(err.contains("checksum"), "{err}");
    }

    #[test]
    fn rejects_data_without_eof() {
        let image = HexImage::parse(":0100000001FE\n").unwrap();
        assert_eq!(image.total_len(), 1);
    }

    #[test]
    fn rejects_data_after_eof() {
        let err = HexImage::parse(":00000001FF\n:0100000001FE\n").unwrap_err();
        assert!(err.contains("end-of-file"), "{err}");
    }
}
