# aic-isp

面向 Luban-Lite BootLoader（`aicupg uart 0` 下载模式）的串口 ISP 上位机工具，Rust 实现，当前面向 d12x 开发板。

## 功能

- **烧写 SPI NOR flash**：把文件写入 flash 的某个分区，走 BootLoader 的 FWC（固件组件）流程，
  带 CRC32 校验与烧写结果检查
- `--verify`：烧写后把数据读回并逐字节比对
- `--reset`：全部完成后复位开发板，让 BootLoader 直接启动刚烧进去的应用
- `--ram`：把文件下载到内存（RAM）而不是 flash，可配合 `--exec` 直接运行
- `--bin <FILE@ADDR>`：裸二进制 + flash 偏移（必须是某个分区的起始地址）
- `--hex <FILE>`：Intel HEX 文件，按其记录中的地址烧写（连续记录会合并成一段）
- 支持多个文件，按命令行顺序依次处理；`--isp-baud` 可在连接后把设备与串口切到更高波特率
- 不带任何文件时只连接设备，打印芯片信息与分区表

## 编译

```sh
cd tools/aic-isp
cargo build --release
# 产物：target/release/aic-isp（Windows 下为 aic-isp.exe）
```

## 使用

先让开发板进入下载模式：按住下载键（d12x demo68-nor 为 PA1）再上电，BootLoader 会停在
`aicupg uart 0` 等待上位机。

```sh
# 烧写打包好的 BootLoader 镜像到 spl 分区（偏移 0）
aic-isp -p COM7 --bin bootloader.aic@0x0

# 烧写应用镜像（os 分区，偏移 0x100000），并用 3 Mbps 提速
aic-isp -p COM7 --isp-baud 3000000 --bin d12x_os.itb@0x100000

# 烧写后读回校验
aic-isp -p COM7 --verify --bin data.fatfs@0x400000

# 烧写完自动复位，直接跑新固件（复位前记得松开下载键）
aic-isp -p COM7 --verify --reset --bin d12x_os.itb@0x100000

# 只看设备信息与分区表
aic-isp -p COM7

# 加载到内存并执行（不经 flash）
aic-isp -p COM7 --ram --bin app.bin@0x40000000 --exec 0x40000000
```

| 参数 | 说明 |
| --- | --- |
| `-p, --port <PORT>` | 串口设备名，如 `COM7`、`/dev/ttyUSB0`；只有一个串口时可省略 |
| `-b, --baud <BAUD>` | 连接 BootLoader 用的波特率，默认 `115200`，需与它一致 |
| `--isp-baud <BAUD>` | 连上之后通过 `SET_UART_ARGS` 命令让设备切到该波特率，双方按新速率继续传输（如 `3000000`）；切换失败会退回原速率并报错 |
| `--bin <FILE@ADDR>` | 裸二进制与 flash 偏移（分区起始地址），如 `bootloader.aic@0x0` |
| `--hex <FILE>` | Intel HEX 文件，地址取自文件内容（按 flash 偏移处理） |
| `--ram` | 把 `--bin`/`--hex` 载入内存，此时地址是 RAM 地址 |
| `--verify` | 烧写后通过 `READ_FWC_DATA` 读回比对 |
| `--reset` | 全部完成后通过 `RUN_SHELL_STR` 执行控制台命令 `reset`，设备由 WDT 复位并启动应用 |
| `--exec <ADDR>` | 全部完成后调用该地址的代码（RAM 操作） |
| `-v, --verbose` | 在 stderr 上打印协议细节（帧、CSW、分区表等），便于定位问题 |
| `-h, --help` | 帮助 |
| `-V, --version` | 版本 |

数字参数只有带 `0x`/`0X` 前缀时才按十六进制解析，其余按十进制（例如 `-b 921600`）。

## 烧写规则与注意事项

- **只能烧到分区起始地址**：BootLoader 的 FWC 写入总是从分区第 0 字节开始
  （`nor_fwc_mtd_write()` 的偏移从 0 递增），协议里没有"在分区内任意偏移写入"的命令。
  命令行给出的偏移必须等于某个分区的起始地址，否则工具会报错并列出所有可用分区。分区表由设备
  通过 `GET_PARTITION_TABLE` 返回，d12x demo68-nor 为：

  ```
  spl@0x0(512K) env@0x80000(128K) env_r@0xA0000(128K) userid@0xC0000(256K)
  os@0x100000(1536K) os_r@0x280000(1536K) data@0x400000(12M)
  ```

- **给分区喂什么镜像**：按 `target/{chip}/{board}/pack/image_cfg.json`，spl 分区放打包后的
  `bootloader.aic`（PBP + loader），os 分区放 `d12x_os.itb`，data 放 `data.fatfs`，都在
  `output/{chip}_{board}_{kernel}_{app}/images/` 下。裸 `bootloader.bin` 只是 .aic 中的载荷，
  单独写进 spl 分区无法启动。
- 写入长度可以小于分区大小，未写入的部分保持原样（写入过程中设备按 64K 块擦除，数据结束后的
  若干块可能被一并擦除）。
- 校验手段：每帧 CRC16 + ACK、每 32 KiB 一包的命令状态（CSW）、设备对已收数据计算的 CRC32
  （`GET_FWC_CRC`，工具会与本文件的 CRC32 比对）、`GET_FWC_BURN_RESULT`。
  BootLoader 编译时开启了 `AICUPG_SINGLE_TRANS_BURN_CRC32_VERIFY`，每包写入后会读回比对，
  但失败只打印日志（升级模式下控制台输出被关闭，主机看不到），需要确定性验证时请加 `--verify`。
- 烧写前工具会先发送 `image.info` 组件，让 BootLoader 探测 flash 并注册 MTD 分区；随后用
  `GET_BLOCK_SIZE` 确认设备确实切到了 flash 模式（仍处于 RAM 模式时块大小为 1，工具会报错），
  避免"数据被静默丢弃但命令成功"的情况。
- 写入地址必须落在芯片 flash 内；`--ram` 模式的地址必须落在 RAM（d12x 如 SRAM `0x30040000` 起、
  PSRAM）内，写坏地址会导致设备跑飞。
- `--exec` 是在 BootLoader 的协议循环里直接调用该地址，被调用代码需要返回 `int`。如果代码
  不返回（运行起来的应用通常如此），工具会打印 `did not return` 并以成功结束。
- `--reset` 走的是控制台命令 `reset`（`wdt_expire_now()`），设备在命令中途就重启，不会给
  主机应答，工具把"没有应答"当作成功并提示 `the board went down, it is booting now`。
  **复位前要松开下载键（PA1）**，否则重启后 BootLoader 会再次进入下载模式而不是启动应用。
  执行顺序是：烧写/下载 → `--exec` → `--reset`；如果 `--exec` 的代码不返回，就不会走到复位。
- 芯片 ID 通过 `GET_HWINFO` 获取，BootLoader 未使能 SID（efuse）时显示 `not available`。

## 协议对应关系

| 本工具模块 | SDK 源码 | 职责 |
| --- | --- | --- |
| `src/proto.rs` | `lib/aicupg/uart_proto_layer.c`、`data_trans_layer.c`、`include/aicupg.h` | 帧格式、CRC16、CBW/CSW、命令头/响应头 |
| `src/fwc.rs` | `lib/aicupg/fwc_cmd.c`、`nor_fwc.c`、`ram_fwc.c`、`upg_internal.h` | FWC 组件结构、镜像头、分区表解析、CRC32 |
| `src/device.rs` | `lib/aicupg/upg_main.c`、`basic_cmd.c`、`aicupg_for_uart.c` | 连接握手、内存下载、flash 烧写与回读、执行 |
| `src/transport.rs` | 无（上位机侧） | 串口收发、超时控制与波特率切换 |
| `src/hexfile.rs` | 无（上位机侧） | Intel HEX 解析 |

烧写一个文件的流程：`GET_PARTITION_TABLE`（首次，取分区表）→ `image.info` 组件（首次，让设备
探测 flash 并切到 SPI NOR）→ `SET_FWC_META`（组件名/分区/长度/CRC32/`attr="mtd,required"`）→
`GET_BLOCK_SIZE`（确认处于 flash 模式）→ `SEND_FWC_DATA`（每 32 KiB 一个 CBW 传输）→
`GET_FWC_CRC` → `GET_FWC_BURN_RESULT` → 可选 `READ_FWC_DATA` 回读比对。

注意 `GET_BLOCK_SIZE`/`GET_FWC_CRC`/`GET_FWC_BURN_RESULT`/`GET_PARTITION_TABLE`/`READ_FWC_DATA`
的应答分两次传输：先 16 字节响应头，再是新一次传输里的数据；设备的数据阶段从缓冲区开头写入
（`memcpy(buf, &val, 4)`），与响应头无法放在同一个传输里。

底层传输：`SIG_C` 探活 → `DC1_SEND` 切换方向 → 命令头（单独一个 CBW 传输）→ 参数与数据
（每个 32 KiB 一个 CBW 传输，仅第一个带参数）→ 读取响应并结束命令。

## 测试

```sh
cd tools/aic-isp
cargo test
```

`src/sim.rs` 是按 `application/baremetal/bootloader/lib/aicupg/*.c` 实现的设备侧模型（含 flash、
分区表与 FWC 流程），单元测试让上位机实现与这个模型跑完整流程：连接、读设备信息、内存写入、
flash 烧写与回读、CRC 校验、波特率切换等，因此无需硬件即可验证协议时序。结构体偏移用 SDK 自带
的 riscv32 工具链核对过（`fwc_meta` 512 字节、`artinchip_image_info` 348 字节、
`media_partition` 312 字节）；`src/hexfile.rs`、`src/proto.rs`、`src/fwc.rs` 另有针对解析、
帧格式与 CRC 的用例。
