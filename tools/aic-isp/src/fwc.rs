// Copyright (c) 2026, ArtInChip Technology Co., Ltd
//
// SPDX-License-Identifier: Apache-2.0
//
// Structures of the "firmware component" (FWC) layer that programs the flash,
// written from fwc_cmd.c, nor_fwc.c and upg_internal.h. The sizes below were
// checked against the SDK toolchain for the 32 bit RISC-V target.

/// FWC meta data, struct fwc_meta in upg_internal.h.
pub const META_LEN: usize = 512;
/// struct artinchip_image_info in ram_fwc.c.
pub const IMAGE_INFO_LEN: usize = 348;
/// struct media_partition in upg_internal.h.
pub const MEDIA_PARTITION_LEN: usize = 312;
/// PARTITION_TABLE_LEN, the fixed length of the GET_PARTITION_TABLE response.
pub const PARTITION_TABLE_LEN: usize = 128;

pub const FWC_STR_MAX_LEN: usize = 64;
pub const MAX_PARTITION_NAME: usize = 144;

/// struct fwc_meta, the argument of SET_FWC_META.
pub struct Meta {
    pub name: String,
    pub partition: String,
    pub offset: u32,
    pub size: u32,
    pub crc: u32,
    pub ram: u32,
    pub attr: String,
}

impl Meta {
    pub fn to_bytes(&self) -> [u8; META_LEN] {
        let mut buf = [0u8; META_LEN];
        buf[0..4].copy_from_slice(b"META");
        put_str(&mut buf[8..8 + FWC_STR_MAX_LEN], &self.name);
        put_str(
            &mut buf[72..72 + FWC_STR_MAX_LEN],
            &self.partition,
        );
        buf[136..140].copy_from_slice(&self.offset.to_le_bytes());
        buf[140..144].copy_from_slice(&self.size.to_le_bytes());
        buf[144..148].copy_from_slice(&self.crc.to_le_bytes());
        buf[148..152].copy_from_slice(&self.ram.to_le_bytes());
        put_str(&mut buf[152..152 + FWC_STR_MAX_LEN], &self.attr);
        buf
    }
}

/// struct artinchip_image_info, the data of the "image.info" component.
///
/// Sending it makes the bootloader probe the flash and register the MTD
/// partitions before any component is programmed.
pub fn image_info(media_type: &str, media_dev_id: u32) -> [u8; IMAGE_INFO_LEN] {
    let mut buf = [0u8; IMAGE_INFO_LEN];
    buf[0..6].copy_from_slice(b"AIC.FW");
    put_str(&mut buf[8..8 + FWC_STR_MAX_LEN], "d12x");
    put_str(&mut buf[72..72 + FWC_STR_MAX_LEN], "aic-isp");
    put_str(&mut buf[136..136 + FWC_STR_MAX_LEN], "1.0.0");
    put_str(&mut buf[200..200 + FWC_STR_MAX_LEN], media_type);
    buf[264..268].copy_from_slice(&media_dev_id.to_le_bytes());
    buf
}

/// struct media_partition, the argument of READ_FWC_DATA.
///
/// `size` is how many bytes are read, always from the start of the partition.
pub fn media_partition(
    media_type: &str,
    media_dev_id: u32,
    name: &str,
    size: u64,
) -> [u8; MEDIA_PARTITION_LEN] {
    let mut buf = [0u8; MEDIA_PARTITION_LEN];
    buf[..MEDIA_LEN].copy_from_slice(&storage_media(media_type, media_dev_id));
    // struct aic_partition starts at offset 72 on a 32 bit target, its own
    // start/size fields are at offset 152/160 inside it.
    put_str(&mut buf[72..72 + MAX_PARTITION_NAME], name);
    buf[224..232].copy_from_slice(&0u64.to_le_bytes()); // part.start
    buf[232..240].copy_from_slice(&size.to_le_bytes()); // part.size
    put_str(&mut buf[248..248 + FWC_STR_MAX_LEN], "mtd,required");
    buf
}

fn put_str(dst: &mut [u8], value: &str) {
    let bytes = value.as_bytes();
    let n = std::cmp::min(bytes.len(), dst.len() - 1);
    dst[..n].copy_from_slice(&bytes[..n]);
}

/// One flash partition as reported by the bootloader.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FlashPartition {
    pub name: String,
    pub start: u64,
    pub size: u64,
}

/// struct storage_media in upg_internal.h, packed, the argument of
/// GET_PARTITION_TABLE.
pub const MEDIA_LEN: usize = 68;

pub fn storage_media(media_type: &str, media_dev_id: u32) -> [u8; MEDIA_LEN] {
    let mut buf = [0u8; MEDIA_LEN];
    put_str(&mut buf[0..FWC_STR_MAX_LEN], media_type);
    buf[64..68].copy_from_slice(&media_dev_id.to_le_bytes());
    buf
}

/// Parse the MTD partition string of the bootloader, for example
/// "spi0.0:512k(spl),128k(env),1536k(os),12288k(data)".
///
/// This follows aic_part_parse() in bsp/common/include/aic_mtd_partition_parse.h:
/// `<size>[k|m|g][@<start>](<name>)`, entries separated by ','. A '-' as size
/// means "the rest of the flash".
pub fn parse_partition_table(text: &str) -> Result<Vec<FlashPartition>, String> {
    let body = match text.split_once(':') {
        Some((_, body)) => body,
        None => return Err(format!("partition table '{text}' has no device prefix")),
    };

    let mut parts = Vec::new();
    let mut rest = body;
    let mut start: u64 = 0;

    while !rest.is_empty() {
        let (size, tail) = if let Some(tail) = rest.strip_prefix('-') {
            (None, tail)
        } else {
            let (size, tail) = take_number(rest)?;
            (Some(size), tail)
        };

        let (explicit_start, tail) = match tail.strip_prefix('@') {
            Some(tail) => {
                let (value, tail) = take_number(tail)?;
                (Some(value), tail)
            }
            None => (None, tail),
        };
        if let Some(explicit_start) = explicit_start {
            start = explicit_start;
        }

        let tail = tail
            .strip_prefix('(')
            .ok_or_else(|| format!("partition name missing after size in '{rest}'"))?;
        let end = tail
            .find(')')
            .ok_or_else(|| format!("unterminated partition name in '{rest}'"))?;
        let name = &tail[..end];

        let size = size.unwrap_or(u64::MAX);
        parts.push(FlashPartition {
            name: name.to_string(),
            start,
            size,
        });
        start = start.wrapping_add(size);

        // Skip to the next entry, ';' ends the list as in the bootloader.
        let tail = &tail[end + 1..];
        match tail.find([',', ';']) {
            Some(idx) if tail.as_bytes()[idx] == b',' => rest = &tail[idx + 1..],
            _ => break,
        }
    }

    if parts.is_empty() {
        return Err(format!("partition table '{text}' has no entries"));
    }

    Ok(parts)
}

/// strtoul(p, &p, 0) plus the k/m/g suffix of the partition table.
fn take_number(text: &str) -> Result<(u64, &str), String> {
    let (radix, offset) = if text.starts_with("0x") || text.starts_with("0X") {
        (16, 2)
    } else {
        (10, 0)
    };

    let mut end = offset;
    for c in text[offset..].chars() {
        if !c.is_digit(radix) {
            break;
        }
        end += c.len_utf8();
    }
    if end == offset {
        return Err(format!("'{text}' does not start with a number"));
    }

    let mut value = u64::from_str_radix(&text[offset..end], radix)
        .map_err(|_| format!("cannot parse the number in '{text}'"))?;
    let rest = &text[end..];
    if let Some(tail) = rest.strip_prefix(['k', 'K']) {
        value *= 1024;
        Ok((value, tail))
    } else if let Some(tail) = rest.strip_prefix(['m', 'M']) {
        value *= 1024 * 1024;
        Ok((value, tail))
    } else if let Some(tail) = rest.strip_prefix(['g', 'G']) {
        value *= 1024 * 1024 * 1024;
        Ok((value, tail))
    } else {
        Ok((value, rest))
    }
}

/// CRC32 as computed by crc32() in bsp/common/crc32/crc32.c, which is the
/// usual IEEE 802.3 variant: reflected polynomial 0xEDB88320, initial value
/// 0xFFFFFFFF, final xor 0xFFFFFFFF. The check value for "123456789" is
/// 0xCBF43926.
pub fn crc32(data: &[u8]) -> u32 {
    crc32_fold(0, data)
}

/// Continue a CRC32, like crc32(crc, buf, len) on the device: the bootloader
/// folds the component data one transfer at a time.
pub fn crc32_fold(state: u32, data: &[u8]) -> u32 {
    let mut crc = !state;
    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc32_check_value() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
        assert_eq!(crc32(b""), 0);
    }

    #[test]
    fn crc32_folds_like_the_device() {
        // The bootloader folds the component data chunk by chunk with
        // crc32(crc, buf, len); the result must equal one call over all bytes.
        let data: Vec<u8> = (0..1000u32).map(|i| (i % 251) as u8).collect();
        let mut folded = 0u32;
        for chunk in data.chunks(64) {
            folded = crc32_fold(folded, chunk);
        }
        assert_eq!(folded, crc32(&data));
    }

    #[test]
    fn parses_the_demo68_nor_table() {
        let parts = parse_partition_table(
            "spi0.0:512k(spl),128k(env),128k(env_r),256k(userid),1536k(os),1536k(os_r),12288k(data)",
        )
        .unwrap();
        assert_eq!(parts.len(), 7);
        assert_eq!(parts[0], FlashPartition { name: "spl".into(), start: 0, size: 0x80000 });
        assert_eq!(parts[1], FlashPartition { name: "env".into(), start: 0x80000, size: 0x20000 });
        assert_eq!(parts[2], FlashPartition { name: "env_r".into(), start: 0xA0000, size: 0x20000 });
        assert_eq!(parts[3], FlashPartition { name: "userid".into(), start: 0xC0000, size: 0x40000 });
        assert_eq!(parts[4], FlashPartition { name: "os".into(), start: 0x100000, size: 0x180000 });
        assert_eq!(parts[5], FlashPartition { name: "os_r".into(), start: 0x280000, size: 0x180000 });
        assert_eq!(parts[6], FlashPartition { name: "data".into(), start: 0x400000, size: 0xC00000 });
    }

    #[test]
    fn parses_explicit_offsets_and_rest() {
        let parts = parse_partition_table("spi0.0:64k@0x1000(boot),-(data)").unwrap();
        assert_eq!(parts[0].start, 0x1000);
        assert_eq!(parts[0].size, 0x10000);
        assert_eq!(parts[1].name, "data");
    }

    #[test]
    fn rejects_a_broken_table() {
        assert!(parse_partition_table("no colon").is_err());
        assert!(parse_partition_table("spi0.0:512k(spl").is_err());
        assert!(parse_partition_table("spi0.0:").is_err());
    }

    #[test]
    fn meta_layout_matches_the_device() {
        let meta = Meta {
            name: "spl".into(),
            partition: "spl".into(),
            offset: 0,
            size: 0x80000,
            crc: 0xDEAD_BEEF,
            ram: 0,
            attr: "mtd,required".into(),
        };
        let bytes = meta.to_bytes();
        assert_eq!(bytes.len(), META_LEN);
        assert_eq!(&bytes[0..4], b"META");
        assert_eq!(&bytes[8..11], b"spl");
        assert_eq!(&bytes[72..75], b"spl");
        assert_eq!(u32::from_le_bytes(bytes[140..144].try_into().unwrap()), 0x80000);
        assert_eq!(
            u32::from_le_bytes(bytes[144..148].try_into().unwrap()),
            0xDEAD_BEEF
        );
        assert_eq!(&bytes[152..164], b"mtd,required");
    }

    #[test]
    fn media_partition_layout_matches_the_device() {
        let buf = media_partition("spi-nor", 0, "os", 0x1234);
        assert_eq!(buf.len(), MEDIA_PARTITION_LEN);
        assert_eq!(&buf[0..7], b"spi-nor");
        assert_eq!(&buf[72..74], b"os");
        assert_eq!(u64::from_le_bytes(buf[232..240].try_into().unwrap()), 0x1234);
        assert_eq!(&buf[248..252], b"mtd,");
    }

    #[test]
    fn image_info_layout() {
        let buf = image_info("spi-nor", 0);
        assert_eq!(buf.len(), IMAGE_INFO_LEN);
        assert_eq!(&buf[0..6], b"AIC.FW");
        assert_eq!(&buf[200..207], b"spi-nor");
    }
}
