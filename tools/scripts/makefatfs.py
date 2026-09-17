#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# SPDX-License-Identifier: Apache-2.0
#
# Dehuang.Wu
# Copyright (C) 2021-2026 ArtInChip Technology Co., Ltd

import os
import re
import sys
import platform
import argparse
import subprocess
import json
from collections import OrderedDict


def mkimage_get_resource_size(srcdir, cluster_siz):
    total_size = 0
    root_path = srcdir
    for root, dirs, files in os.walk(srcdir):
        for fn in files:
            fpath = os.path.join(root, fn)
            size = os.path.getsize(fpath)
            size = cluster_siz * int(round((size + cluster_siz - 1) / cluster_siz))
            total_size += size
        for d in dirs:
            total_size += cluster_siz
    return total_size


def mkimage_get_part_size(outfile):
    imgname = os.path.basename(outfile)
    partlist = os.path.join(os.path.dirname(outfile), 'partition_file_list.h')
    size = 0
    if not os.path.exists(partlist):
        return 0
    with open(partlist) as f:
        lines = f.readlines()
        for ln in lines:
            ln = ln.strip()
            if not ln:
                continue
            parts = ln.split(',')
            if len(parts) < 3:
                continue
            name = parts[1].replace('"', '').replace('*', '')
            if any(imgname == re.sub(pat, "", name) for pat in [".enc", ".sparse.enc", ".sparse"]):
                size = int(parts[2])
                return size
    print('Image {} is not used in any partition'.format(imgname))
    print('please check your project\'s image_cfg.json')
    return size


def parse_image_cfg(cfgfile):
    """ Load image configuration file
    Args:
        cfgfile: Configuration file name
    """
    with open(cfgfile, "r") as f:
        lines = f.readlines()
        jsonstr = ""
        for line in lines:
            sline = line.strip()
            if sline.startswith("//"):
                continue
            slash_start = sline.find("//")
            if slash_start > 0:
                jsonstr += sline[0:slash_start].strip()
            else:
                jsonstr += sline
        # Use OrderedDict is important, we need to iterate FWC in order.
        jsonstr = jsonstr.replace(",}", "}").replace(",]", "]")
        cfg = json.loads(jsonstr, object_pairs_hook=OrderedDict)
    return cfg


def check_is_nftl_part(outfile):
    imgname = os.path.basename(outfile)
    partjson = os.path.join(os.path.dirname(outfile), 'partition.json')
    if not os.path.exists(partjson):
        return False

    cfg = parse_image_cfg(partjson)
    nftl_value = cfg["partitions"].get("nftl")
    if nftl_value is None:
        return False

    imgname = os.path.basename(outfile)
    parts = imgname.split('.')
    nftl_part = parts[0] + ':'
    if nftl_part in nftl_value:
        return True
    return False


def run_cmd(cmdstr):
    # print(cmdstr)
    cmd = cmdstr.split(' ')
    ret = subprocess.run(cmd, stdout=subprocess.PIPE)
    if ret.returncode != 0:
        sys.exit(1)


FAT_ATTR_DIR = 0x10
FAT_ATTR_ARCHIVE = 0x20


def fat83_name(name):
    name = name.upper()
    if '.' in name:
        base, ext = name.rsplit('.', 1)
    else:
        base, ext = name, ''
    for c in base + ext:
        if ord(c) <= 0x20 or ord(c) >= 0x7f or c in '"*+,./:;<=>?[\\]|':
            raise Exception(
                'filename "{}" is not 8.3, mtools is needed for long names'
                ' on macOS (brew install mtools)'.format(name))
    if not 1 <= len(base) <= 8 or len(ext) > 3:
        raise Exception(
            'filename "{}" is not 8.3, mtools is needed for long names'
            ' on macOS (brew install mtools)'.format(name))
    return (base.ljust(8) + ext.ljust(3)).encode('ascii')


def fat_dir_entry(name83, attr, first, size):
    e = bytearray(32)
    e[0:11] = name83
    e[11] = attr
    e[26:28] = first.to_bytes(2, 'little')
    e[28:32] = size.to_bytes(4, 'little')
    return bytes(e)


def write_fat_image(outimg, srcdir, imgsiz, sector_siz, cluster, volab):
    """Minimal FAT12/FAT16 image builder for macOS (no mtools).

    Matches the geometry used by mformat on Linux: 512B sectors,
    `cluster` sectors per cluster, 2 FATs, 512 root entries.
    """
    total_sect = int(imgsiz // sector_siz)
    rsvd = 1
    nfats = 2
    root_ent = 512
    rootdir_sect = (root_ent * 32 + sector_siz - 1) // sector_siz

    # Select FAT12/FAT16 the same way FatFs/mformat do: by cluster count
    fat_sz = 1
    while True:
        data_sect = total_sect - rsvd - nfats * fat_sz - rootdir_sect
        nclus = data_sect // cluster
        fat_bits = 12 if nclus < 4085 else 16
        entry_siz = 1.5 if fat_bits == 12 else 2
        need = int((nclus + 2) * entry_siz + sector_siz - 1) // sector_siz
        if need <= fat_sz:
            break
        fat_sz += 1
        if fat_sz > total_sect // 4:
            raise Exception('FAT geometry error, image too small?')

    img = bytearray(imgsiz)
    fat_off = rsvd * sector_siz
    root_off = fat_off + nfats * fat_sz * sector_siz
    data_off = root_off + rootdir_sect * sector_siz
    free_clu = [2]

    def set_fat(idx, val):
        if fat_bits == 12:
            o = fat_off + idx * 3 // 2
            if idx % 2 == 0:
                img[o] = val & 0xFF
                img[o + 1] = (img[o + 1] & 0xF0) | ((val >> 8) & 0x0F)
            else:
                img[o] = (img[o] & 0x0F) | ((val << 4) & 0xF0)
                img[o + 1] = (val >> 4) & 0xFF
        else:
            o = fat_off + idx * 2
            img[o:o + 2] = val.to_bytes(2, 'little')

    def clu_off(c):
        return data_off + (c - 2) * cluster * sector_siz

    def alloc_chain(n):
        first = free_clu[0]
        if first + n - 2 > nclus:
            raise Exception('FAT image is full')
        eof = 0xFFF if fat_bits == 12 else 0xFFFF
        for i in range(n):
            set_fat(free_clu[0], eof if i == n - 1 else free_clu[0] + 1)
            free_clu[0] += 1
        return first

    def write_chain(first, data):
        clusize = cluster * sector_siz
        n = max(1, (len(data) + clusize - 1) // clusize)
        for i in range(n):
            chunk = data[i * clusize:(i + 1) * clusize]
            img[clu_off(first + i):clu_off(first + i) + len(chunk)] = chunk

    # The nested-dir case is handled by a two-pass recursion below; root-only
    # content is the common case for fs images.
    def build(path):
        items = []
        if os.path.exists(path):
            for nm in sorted(os.listdir(path)):
                items.append(os.path.join(path, nm))
        return items

    def emit_items(items):
        """Emit a flat list of files (sub-dirs one level deep supported)."""
        out = bytearray()
        for fp in items:
            nm = os.path.basename(fp)
            if os.path.isdir(fp):
                sub_items = build(fp)
                sub_ents = bytearray()
                for s in sub_items:
                    if os.path.isdir(s):
                        raise Exception(
                            'nested sub-dirs deeper than 1 level are not'
                            ' supported by the macOS FAT builder')
                    sub_ents += emit_file(s, os.path.basename(s))
                clusize = cluster * sector_siz
                n = max(1, (len(sub_ents) + 64 + clusize - 1) // clusize)
                first = alloc_chain(n)
                dot = fat_dir_entry(b'.          ', FAT_ATTR_DIR, first, 0)
                dotdot = fat_dir_entry(b'..         ', FAT_ATTR_DIR, 0, 0)
                data = dot + dotdot + bytes(sub_ents)
                data = data[:26] + first.to_bytes(2, 'little') + data[28:]
                write_chain(first, data)
                out += fat_dir_entry(fat83_name(nm), FAT_ATTR_DIR, first, 0)
            else:
                out += emit_file(fp, nm)
        return out

    def emit_file(fp, nm):
        with open(fp, 'rb') as f:
            content = f.read()
        clusize = cluster * sector_siz
        n = max(1, (len(content) + clusize - 1) // clusize)
        first = alloc_chain(n)
        write_chain(first, content)
        return fat_dir_entry(fat83_name(nm), FAT_ATTR_ARCHIVE, first,
                             len(content))

    root_ents = emit_items(build(srcdir))
    if len(root_ents) > root_ent * 32:
        raise Exception('too many root entries for FAT image')
    img[root_off:root_off + len(root_ents)] = root_ents

    set_fat(0, 0xFF8 if fat_bits == 12 else 0xFFF8)
    set_fat(1, 0xFFF if fat_bits == 12 else 0xFFFF)
    fat_bytes = fat_sz * sector_siz
    for copy in range(1, nfats):
        copy_off = fat_off + copy * fat_bytes
        img[copy_off:copy_off + fat_bytes] = img[fat_off:fat_off + fat_bytes]

    # boot sector
    bs = bytearray(sector_siz)
    bs[0:3] = b'\xEB\x3C\x90'
    bs[3:11] = b'MSDOS5.0'
    bs[11:13] = sector_siz.to_bytes(2, 'little')
    bs[13] = cluster
    bs[14:16] = rsvd.to_bytes(2, 'little')
    bs[16] = nfats
    bs[17:19] = root_ent.to_bytes(2, 'little')
    if total_sect < 0x10000:
        bs[19:21] = total_sect.to_bytes(2, 'little')
    else:
        bs[32:36] = total_sect.to_bytes(4, 'little')
    bs[21] = 0xF8
    bs[22:24] = fat_sz.to_bytes(2, 'little')
    bs[36] = 0x80
    bs[38] = 0x29
    bs[39:43] = (0x20250916).to_bytes(4, 'little')
    label = 'NO NAME    ' if volab == 'default' else volab.upper().ljust(11)
    bs[43:54] = label.encode('ascii')[:11]
    bs[54:62] = ('FAT{:2d}   '.format(fat_bits)).encode('ascii')
    bs[510:512] = b'\x55\xAA'
    img[0:sector_siz] = bs

    with open(outimg, 'wb') as f:
        f.write(img)


def gen_fatfs(tooldir, srcdir, outimg, volab, imgsiz, sector_siz, cluster):
    sector_cnt = int(imgsiz / sector_siz)
    if platform.system() == 'Linux':
        truncate = 'truncate -s {} {}'.format(imgsiz, outimg)
        run_cmd(truncate)

        mformat = '{}mformat -i {}'.format(tooldir, outimg)
        mformat += ' -M {} -T {} -c {}'.format(sector_siz, sector_cnt, cluster)
        if volab != 'default':
            mformat += ' -v {}'.format(volab)

        run_cmd(mformat)

        if os.path.exists(srcdir) is True:
            mcopy = '{}mcopy -i {} -s {}// ::/'.format(tooldir, outimg, srcdir)
            run_cmd(mcopy)

        # gen sparse format
        img2simg = '{}img2simg {} {}.sparse 1024'.format(tooldir, outimg, outimg)
        run_cmd(img2simg)

    elif platform.system() == 'Windows':
        outimg = outimg.replace(' ', '\\ ')
        outimg = outimg.replace('/', '\\')
        srcdir = srcdir.replace(' ', '\\ ')
        srcdir = srcdir.replace('/', '\\')

        truncate = '{}truncate.exe {} {}'.format(tooldir, outimg, imgsiz)
        run_cmd(truncate)

        mformat = '{}mformat.exe -i {}'.format(tooldir, outimg)
        mformat += ' -M {} -T {} -c {}'.format(sector_siz, sector_cnt, cluster)
        if volab != 'default':
            mformat += ' -v {}'.format(volab)

        run_cmd(mformat)

        if os.path.exists(srcdir) is True:
            mcopy = '{}mcopy.exe -i {} -s {}\\\\ ::/'.format(tooldir, outimg, srcdir)
            run_cmd(mcopy)

        # gen sparse format
        img2simg = '{}img2simg.exe {} {}.sparse 1024'.format(tooldir, outimg, outimg)
        run_cmd(img2simg)

    elif platform.system() == 'Darwin':
        # macOS: mtools/newfs_msdos are not usable on plain files, build
        # the FAT12/16 image with the built-in pure-python writer
        write_fat_image(outimg, srcdir, imgsiz, sector_siz, cluster, volab)


def round_pow2(x):
    cnt = 0
    shift = 64
    last_one = -1
    for i in range(64):
        if (x >> i) & 0x1:
            last_one = i
            cnt += 1
    if last_one < 0:
        return 0

    if cnt > 1:
        last_one += 1
    value = 1 << last_one
    return value


def aic_calc_checksum(start, size):
    offset = 0
    total = 0
    while offset < size:
        val = int.from_bytes(start[offset: offset + 4], byteorder='little', signed=False)
        total = total + val
        offset = offset + 4
    return (~total) & 0xFFFFFFFF


def set_image_info_to_boot_sector(imgfile, total_size, used_size):
    """ Set AIC FAT Image information after BPB structure, start from 0x80
        struct {
            uint32_t magic; "AICF", byte order: 'F''C''I''A'
            uint32_t cksum32;
            uint32_t length; From Magic to data end
            uint32_t version; 1.1
            uint64_t image_size;
            uint64_t used_size;
            uint32_t dirent_sect_id;
            uint32_t dirent_data_len;
        }
    """
    magic = "AICF"
    cksum = 0
    datalen = 4+4+4+4+8+8
    version = int('0x0101', 16)
    image_size = total_size
    dirent_sect_id = 0
    dirent_datalen = 0

    data = magic.encode(encoding="utf-8")
    data = data + cksum.to_bytes(4, byteorder='little', signed=False)
    data = data + datalen.to_bytes(4, byteorder='little', signed=False)
    data = data + version.to_bytes(4, byteorder='little', signed=False)
    data = data + image_size.to_bytes(8, byteorder='little', signed=False)
    data = data + used_size.to_bytes(8, byteorder='little', signed=False)
    data = data + dirent_sect_id.to_bytes(4, byteorder='little', signed=False)
    data = data + dirent_datalen.to_bytes(4, byteorder='little', signed=False)

    # Update cksum
    cksum = aic_calc_checksum(data, len(data))
    imginfo = data[0:4] + cksum.to_bytes(4, byteorder='little', signed=False) + data[8:]
    with open(imgfile, 'rb+') as f:
        f.seek(128)
        f.write(imginfo)
        f.close()


def main(args):
    cluster = int(args.cluster)
    # cluster should be pow of 2
    cluster = round_pow2(cluster)
    sector_siz = int(args.sector)
    calc_mini = True
    strip_siz = True
    inputdir = args.inputdir
    # First priority is pack folder in the same directory with output file
    inputdir_1st = os.path.join(os.path.dirname(args.outfile), inputdir)
    if os.path.exists(inputdir_1st):
        inputdir = inputdir_1st
    if os.path.exists(inputdir) is False:
        print('Info: inputdir {} is not exist.'.format(inputdir))
    if args.auto:
        # No need to strip size in auto mode
        strip_siz = False
    if args.raw:
        # No need to strip size if user select raw image
        strip_siz = False
    if calc_mini:
        cluster_siz = cluster * sector_siz
        data_siz = mkimage_get_resource_size(inputdir, cluster_siz)
        # Size should alignment with cluster size
        data_siz = cluster_siz * int(((data_siz + cluster_siz - 1) / cluster_siz))

    # Total sector: FAT >= 128, FAT32 >= 0x10000
    min_sect_cnt = 128
    part_size = mkimage_get_part_size(args.outfile)

    if args.auto:
        data_clus_cnt = data_siz / cluster_siz
        data_region_sz = data_clus_cnt * cluster_siz
        if data_clus_cnt < 4096:
            # FAT12
            # - BPB_RsvdSecCnt should be 1
            # - BPB_RootEntCnt max 512
            # - FATsz: 2 * FAT
            # - DATA Region: DATA
            fat_siz = (data_region_sz / cluster_siz) * 12 / 8
            rsvd_siz = 1 * sector_siz
            root_ent_cnt = 512 * 32
        elif data_clus_cnt < 65525:
            # FAT16
            # - BPB_RsvdSecCnt should be 1
            # - BPB_RootEntCnt max 512
            # - FATsz: 2 * FAT
            # - DATA Region: DATA
            fat_siz = (data_region_sz / cluster_siz) * 16 / 8
            rsvd_siz = 1 * sector_siz
            root_ent_cnt = 512 * 32
            imgsiz = rsvd_siz + 2 * fat_siz + root_ent_cnt + data_region_sz
        else:
            # FAT32
            # - BPB_RsvdSecCnt fixed 32
            # - BPB_RootEntCnt fixed 0
            # - FATsz: 2 * FAT
            # - DATA Region: DATA
            fat_siz = data_region_sz / cluster_siz * 4
            rsvd_siz = 32 * sector_siz
            root_ent_cnt = 0
            min_sect_cnt = 0x10000
        fat_siz = sector_siz * int((fat_siz + sector_siz - 1) / sector_siz)
        imgsiz = rsvd_siz + 2 * fat_siz + root_ent_cnt + data_region_sz
        # Round to cluster alignment
        imgsiz = cluster_siz * int(((imgsiz + cluster_siz - 1) / cluster_siz))
        if check_is_nftl_part(args.outfile):
            # Space reserved for bad block management in NFTL
            if args.reserved and (int(args.reserved) > 50):
                NFTL_RESERVED = (int(args.reserved) + 1) * 64 * 2048
            else:
                NFTL_RESERVED = 51 * 64 * 2048
            imgsiz = part_size - NFTL_RESERVED
            if imgsiz < 0:
                print(
                    f'Error, partition size: {part_size} '
                    f'is less than NFTL reserved: {NFTL_RESERVED}.'
                )
                sys.exit(1)
            # Round to cluster alignment
            imgsiz = cluster_siz * int(((imgsiz + cluster_siz - 1) / cluster_siz))
    elif args.fullpart:
        if check_is_nftl_part(args.outfile):
            # Space reserved for bad block management in NFTL
            if args.reserved and (int(args.reserved) > 50):
                NFTL_RESERVED = (int(args.reserved) + 1) * 64 * 2048
            else:
                NFTL_RESERVED = 51 * 64 * 2048
            imgsiz = part_size - NFTL_RESERVED
            if imgsiz < 0:
                print(
                    f'Error, partition size: {part_size} '
                    f'is less than NFTL reserved: {NFTL_RESERVED}.'
                )
                sys.exit(1)
        else:
            imgsiz = part_size
        # Round to cluster alignment
        imgsiz = cluster_siz * int(((imgsiz + cluster_siz - 1) / cluster_siz))
    else:
        imgsiz = int(args.imgsize)

    if (imgsiz / sector_siz) < min_sect_cnt:
        imgsiz = min_sect_cnt * sector_siz
    if part_size and imgsiz > part_size:
        print('Error, fatfs image size is larger than partition size: {}.'.format(args.outfile))
        sys.exit(1)

    gen_fatfs(args.tooldir, inputdir, args.outfile, args.volab, imgsiz, sector_siz, cluster)

    clus_cnt = imgsiz / cluster_siz
    if clus_cnt < 65536:
        # FAT16/FAT12, assume it is FAT16, and evaluate the valid data size
        fat_siz = (clus_cnt * 16) / 8
        rsvd_siz = 1 * sector_siz
        root_ent_cnt = 512 * 32
    else:
        # FAT32, evaluate the valid data size
        fat_siz = (clus_cnt * 32) / 8
        rsvd_siz = 32 * sector_siz
        root_ent_cnt = 0
    minimal_siz = rsvd_siz + 2 * fat_siz + root_ent_cnt + 2 * cluster_siz + data_siz
    # Round to cluster alignment
    minimal_siz = cluster_siz * int(((minimal_siz + cluster_siz - 1) / cluster_siz))
    # Set image information for NFTL offline burn
    set_image_info_to_boot_sector(args.outfile, imgsiz, minimal_siz)

    if platform.system() == 'Linux':
        fatupdate = '{}fatupdate {}'.format(args.tooldir, args.outfile)
        run_cmd(fatupdate)
    elif platform.system() == 'Windows':
        fatupdate = '{}fatupdate.exe {}'.format(args.tooldir, args.outfile)
        run_cmd(fatupdate)
    # Darwin: fatupdate is a Linux binary, skip it
    if strip_siz:
        if platform.system() == 'Linux':
            truncate = 'truncate -s {} {}'.format(minimal_siz, args.outfile)
            run_cmd(truncate)
        elif platform.system() == 'Windows':
            truncate = '{}truncate.exe {} {}'.format(args.tooldir, args.outfile, minimal_siz)
            run_cmd(truncate)
        elif platform.system() == 'Darwin':
            with open(args.outfile, 'r+b') as f:
                f.truncate(int(minimal_siz))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-a", "--auto", action='store_true',
                        help="auto size of FAT image")
    parser.add_argument("-f", "--fullpart", action='store_true',
                        help="image size is partition size")
    parser.add_argument("-i", "--inputdir", type=str,
                        help="input directory")
    parser.add_argument("-o", "--outfile", type=str,
                        help="output file")
    parser.add_argument("-g", "--imgsize", type=str,
                        help="image size")
    parser.add_argument("-s", "--sector", type=str,
                        help="sector size")
    parser.add_argument("-c", "--cluster", type=str,
                        help="cluster size")
    parser.add_argument("-t", "--tooldir", type=str,
                        help="tool directory")
    parser.add_argument("-v", "--volab", type=str,
                        help="volume label")
    parser.add_argument("-r", "--raw", action='store_true',
                        help="Don't strip FAT image, keep raw image")
    parser.add_argument("-b", "--reserved", type=str,
                        help="reserved block count for NFTL")
    args = parser.parse_args()
    main(args)
