// Copyright (c) 2026, ArtInChip Technology Co., Ltd
//
// SPDX-License-Identifier: Apache-2.0
//
// UART ISP tool for the ArtInChip bootloader: programs files into the SPI NOR
// flash (the "firmware component" flow), or downloads them into RAM and starts
// them with the EXEC command.

mod device;
mod fwc;
mod hexfile;
mod proto;
mod transport;

#[cfg(test)]
mod sim;

use std::fs;
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::time::{Duration, Instant};

use device::{Device, ExecOutcome, HwInfo};
use hexfile::HexImage;
use transport::Transport;

const VERSION: &str = env!("CARGO_PKG_VERSION");
/// The bootloader handshake always runs at this fixed speed; --baud only
/// switches to a faster download rate once the connection is up.
const DEFAULT_BAUD: u32 = 115200;
/// How long we wait for a board that is still powering up.
const PROBE_TIMEOUT: Duration = Duration::from_secs(30);

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();

    match parse_args(&args) {
        Err(e) => {
            eprintln!("aic-isp: {e}");
            eprintln!("Try 'aic-isp --help' for more information.");
            ExitCode::FAILURE
        }
        Ok(Action::Help) => {
            print_help();
            ExitCode::SUCCESS
        }
        Ok(Action::Version) => {
            println!("aic-isp {VERSION}");
            ExitCode::SUCCESS
        }
        Ok(Action::Run(opts)) => match run(*opts) {
            Ok(()) => ExitCode::SUCCESS,
            Err(e) => {
                eprintln!();
                eprintln!("aic-isp: error: {e}");
                ExitCode::FAILURE
            }
        },
    }
}

//
// Command line
//

enum Item {
    Bin { path: PathBuf, addr: u32 },
    Hex { path: PathBuf },
}

struct Opts {
    port: Option<String>,
    /// Download speed: after connecting at DEFAULT_BAUD, the device is asked
    /// to switch here and the transfer continues at that rate.
    baud: u32,
    items: Vec<Item>,
    exec: Option<u32>,
    ram: bool,
    verify: bool,
    reset: bool,
    verbose: bool,
}

enum Action {
    Run(Box<Opts>),
    Help,
    Version,
}

fn parse_args(args: &[String]) -> Result<Action, String> {
    let mut opts = Opts {
        port: None,
        baud: DEFAULT_BAUD,
        items: Vec::new(),
        exec: None,
        ram: false,
        verify: false,
        reset: false,
        verbose: false,
    };
    let mut idx = 0usize;

    while idx < args.len() {
        let (name, inline) = match args[idx].split_once('=') {
            Some((n, v)) => (n.to_string(), Some(v.to_string())),
            None => (args[idx].clone(), None),
        };
        let value = |idx: &mut usize| -> Result<String, String> {
            if let Some(v) = inline.clone() {
                return Ok(v);
            }
            *idx += 1;
            args.get(*idx)
                .cloned()
                .ok_or_else(|| format!("{name} requires a value"))
        };

        match name.as_str() {
            "-h" | "--help" => return Ok(Action::Help),
            "-V" | "--version" => return Ok(Action::Version),
            "-v" | "--verbose" => opts.verbose = true,
            "-p" | "--port" => opts.port = Some(value(&mut idx)?),
            "-b" | "--baud" => opts.baud = parse_u32(&value(&mut idx)?)?,
            "--bin" => {
                let (path, addr) = parse_bin_spec(&value(&mut idx)?)?;
                opts.items.push(Item::Bin { path, addr });
            }
            "--hex" => {
                let spec = value(&mut idx)?;
                if spec.contains('@') {
                    return Err(format!("--hex expects a file name, got '{spec}'"));
                }
                opts.items.push(Item::Hex {
                    path: PathBuf::from(spec),
                });
            }
            "--exec" => opts.exec = Some(parse_u32(&value(&mut idx)?)?),
            "--ram" => opts.ram = true,
            "--verify" => opts.verify = true,
            "--reset" => opts.reset = true,
            other if other.starts_with('-') => {
                return Err(format!("unknown option '{other}'"));
            }
            other => return Err(format!("unexpected argument '{other}'")),
        }

        idx += 1;
    }

    Ok(Action::Run(Box::new(opts)))
}

fn parse_bin_spec(spec: &str) -> Result<(PathBuf, u32), String> {
    let (path, addr) = spec
        .rsplit_once('@')
        .ok_or_else(|| format!("--bin expects FILE@ADDR, got '{spec}'"))?;
    if path.is_empty() {
        return Err(format!("--bin expects FILE@ADDR, got '{spec}'"));
    }
    Ok((PathBuf::from(path), parse_u32(addr)?))
}

fn parse_u32(text: &str) -> Result<u32, String> {
    let text = text.trim();
    let (digits, radix) = match text.strip_prefix("0x").or_else(|| text.strip_prefix("0X")) {
        Some(hex) => (hex, 16),
        None => (text, 10),
    };
    u32::from_str_radix(digits, radix).map_err(|_| format!("'{text}' is not a valid number"))
}

fn print_help() {
    println!(
        "aic-isp {VERSION} - UART ISP tool for the ArtInChip bootloader

Programs files into the SPI NOR flash of a board that is running the
bootloader in download mode (hold the download key while powering on, the
bootloader then waits in \"aicupg uart 0\" for this tool).

A file is programmed into the flash partition that begins at the given
address: the bootloader writes every component from the start of its
partition, so addresses inside a partition cannot be used. Pass --ram to
load files into memory instead.

USAGE:
    aic-isp -p <PORT> --bin <FILE@ADDR> [--bin <FILE@ADDR>] [--hex <FILE>] [OPTIONS]

OPTIONS:
    -p, --port <PORT>    Serial port of the board, e.g. COM7 or /dev/ttyUSB0
    -b, --baud <BAUD>    The tool always connects at {DEFAULT_BAUD} bps; after the
                         handshake it asks the device to switch to BAUD and
                         continues the download at that speed, e.g. 3000000
                         [default: {DEFAULT_BAUD}]
        --bin <FILE@ADDR>
                         Raw binary plus the flash offset (the start of a
                         partition) it is programmed to, e.g.
                         --bin bootloader.bin@0x0. May be given several times.
        --hex <FILE>     Intel HEX file, programmed to the flash offsets it
                         contains. May be given several times.
        --ram            Load --bin/--hex into RAM instead of programming the
                         flash; with it ADDR is a RAM address
        --verify         Read the programmed data back and compare it
        --reset          Reboot the board once everything is done, so the
                         bootloader starts the application it just got
                         (release the download key before the reset)
        --exec <ADDR>    Call the code at ADDR once every file is done
    -v, --verbose        Trace the protocol on stderr
    -h, --help           Print this help
    -V, --version        Print the version

EXAMPLES:
    aic-isp -p COM7 --bin bootloader.bin@0                # into the spl partition
    aic-isp -p COM7 --bin d12x_os.itb@0x100000 --reset    # program and boot it
    aic-isp -p COM7 --verify --bin data.fatfs@0x400000    # and read it back
    aic-isp -p COM7 --ram --bin app.bin@0x40000000 --exec 0x40000000

Without --bin/--hex/--exec the tool only connects and prints the device
information and the partition table."
    );
}

//
// Work items
//

struct Job {
    label: String,
    addr: u32,
    data: Vec<u8>,
}

fn build_jobs(items: &[Item], verbose: bool) -> Result<Vec<Job>, String> {
    let mut jobs = Vec::new();

    for item in items {
        match item {
            Item::Bin { path, addr } => {
                let data = fs::read(path)
                    .map_err(|e| format!("cannot read {}: {e}", path.display()))?;
                if data.is_empty() {
                    return Err(format!("{} is empty", path.display()));
                }
                jobs.push(Job {
                    label: file_name(path),
                    addr: *addr,
                    data,
                });
            }
            Item::Hex { path } => {
                let image = HexImage::load(path)?;
                let name = file_name(path);
                let count = image.segments.len();
                if verbose {
                    eprintln!(
                        "        . {name}: {} bytes in {} segment(s)",
                        image.total_len(),
                        image.segments.len()
                    );
                    if let Some(entry) = image.entry {
                        eprintln!("        . {name}: entry point 0x{entry:08X}");
                    }
                }
                for (i, seg) in image.segments.into_iter().enumerate() {
                    let label = if count == 1 {
                        name.clone()
                    } else {
                        format!("{name} [{}/{}]", i + 1, count)
                    };
                    jobs.push(Job {
                        label,
                        addr: seg.addr,
                        data: seg.data,
                    });
                }
            }
        }
    }

    Ok(jobs)
}

fn file_name(path: &Path) -> String {
    path.file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_else(|| path.to_string_lossy().into_owned())
}

//
// Main flow
//

fn run(opts: Opts) -> Result<(), String> {
    let port_name = match &opts.port {
        Some(p) => p.clone(),
        None => autodetect_port()?,
    };
    let jobs = build_jobs(&opts.items, opts.verbose)?;
    let total_len: usize = jobs.iter().map(|j| j.data.len()).sum();

    println!("aic-isp {VERSION} - UART ISP tool for the ArtInChip bootloader");
    println!();
    println!("port    : {port_name} @ {DEFAULT_BAUD} bps");
    if opts.baud != DEFAULT_BAUD {
        println!("          switch to {} bps for the download", opts.baud);
    }
    println!(
        "mode    : {}",
        if opts.ram {
            "load into RAM"
        } else {
            "program the SPI NOR flash"
        }
    );
    if jobs.is_empty() {
        println!("files   : none, only probing the device");
    } else {
        println!(
            "files   : {}, {total_len} bytes",
            jobs.iter().map(|j| j.label.as_str()).collect::<Vec<_>>().join(", ")
        );
    }

    let started = Instant::now();
    let transport = Transport::open(&port_name, DEFAULT_BAUD)?;
    print!("probing : waiting for the bootloader in download mode ... ");
    io::stdout().flush().ok();
    let mut dev = Device::connect(transport, PROBE_TIMEOUT, opts.verbose)?;
    println!("ok");

    let info = dev.hwinfo()?;
    print_device(&info);

    if opts.baud != DEFAULT_BAUD {
        print!("baud    : switching the device to {} bps ... ", opts.baud);
        io::stdout().flush().ok();
        dev.set_uart_baudrate(opts.baud)?;
        if let Err(e) = dev.hwinfo() {
            // The device may have kept the old speed, go back to it so
            // the failure is reported instead of losing the session.
            let _ = dev.set_link_baudrate(DEFAULT_BAUD);
            println!("failed");
            return Err(format!("the device did not answer at {} bps: {e}", opts.baud));
        }
        println!("ok");
    }

    if opts.ram {
        for (i, job) in jobs.iter().enumerate() {
            download(&mut dev, job, i + 1, jobs.len())?;
        }
    } else {
        // Resolve every offset against the partition table before the first
        // byte is programmed, so a typo cannot leave a half written board.
        let partitions = dev.partitions()?;
        print_partitions(&partitions);
        let plan = plan_flash(&jobs, &partitions)?;

        for (i, (job, partition)) in jobs.iter().zip(plan.iter()).enumerate() {
            program(&mut dev, job, partition, i + 1, jobs.len(), opts.verify)?;
        }
    }

    if let Some(addr) = opts.exec {
        let outcome = dev.exec(addr)?;
        print!("exec    : 0x{addr:08X} ");
        match outcome {
            ExecOutcome::Returned { status_ok: true } => println!("returned, status OK"),
            ExecOutcome::Returned { status_ok: false } => {
                println!("returned, status FAILED");
                return Err(format!("the payload at 0x{addr:08X} returned a failure"));
            }
            ExecOutcome::NoResponse => {
                println!("did not return, the payload is probably running now")
            }
        }
    }

    if opts.reset {
        print!("reset   : rebooting the board ... ");
        io::stdout().flush().ok();
        let answered = dev.run_shell("reset")?;
        println!(
            "{}",
            if answered {
                "accepted"
            } else {
                "the board went down, it is booting now"
            }
        );
    }

    let elapsed = started.elapsed();
    println!();
    println!(
        "completed: {total_len} bytes in {:.1} s ({})",
        elapsed.as_secs_f64(),
        rate(total_len, elapsed)
    );
    Ok(())
}

/// Check the requested offsets against the partition table of the device.
fn plan_flash(
    jobs: &[Job],
    partitions: &[fwc::FlashPartition],
) -> Result<Vec<fwc::FlashPartition>, String> {
    let mut plan = Vec::with_capacity(jobs.len());

    for job in jobs {
        let partition = partitions
            .iter()
            .find(|p| p.start == job.addr as u64)
            .ok_or_else(|| {
                let known = partitions
                    .iter()
                    .map(|p| format!("{}@0x{:X}", p.name, p.start))
                    .collect::<Vec<_>>()
                    .join(", ");
                format!(
                    "no flash partition starts at 0x{:X} for {}; the bootloader programs a component \
                     from the start of its partition, available: {known}",
                    job.addr, job.label
                )
            })?;

        if job.data.len() as u64 > partition.size {
            return Err(format!(
                "{} ({} bytes) does not fit into partition '{}' ({})",
                job.label,
                job.data.len(),
                partition.name,
                human_size(partition.size)
            ));
        }
        plan.push(partition.clone());
    }

    Ok(plan)
}

fn print_partitions(partitions: &[fwc::FlashPartition]) {
    let list = partitions
        .iter()
        .map(|p| format!("{}@0x{:X}({})", p.name, p.start, human_size(p.size)))
        .collect::<Vec<_>>()
        .join(" ");
    println!("flash   : {list}");
}

/// Program one file into its partition.
fn program(
    dev: &mut Device,
    job: &Job,
    partition: &fwc::FlashPartition,
    index: usize,
    count: usize,
    verify: bool,
) -> Result<(), String> {
    println!();
    println!(
        "[{index}/{count}] {} -> partition '{}' at 0x{:X} ({} bytes)",
        job.label,
        partition.name,
        partition.start,
        job.data.len()
    );

    let total = job.data.len();
    let started = Instant::now();
    let mut progress = |done: usize| {
        print!(
            "\r        {:5.1}%  {done}/{total} bytes",
            done as f64 * 100.0 / total as f64
        );
        io::stdout().flush().ok();
    };

    dev.write_flash(partition.start, &job.data, &mut progress)?;

    let elapsed = started.elapsed();
    println!(
        "\r        programmed {total} bytes in {:.1} s ({})          ",
        elapsed.as_secs_f64(),
        rate(total, elapsed)
    );

    if verify {
        print!("        verifying ... ");
        io::stdout().flush().ok();
        let read_back = dev.read_flash(partition.start, total)?;
        if read_back != job.data {
            let at = read_back
                .iter()
                .zip(job.data.iter())
                .position(|(a, b)| a != b)
                .unwrap_or(0);
            return Err(format!(
                "verification failed: partition '{}' differs from {} at offset 0x{at:X}",
                partition.name, job.label
            ));
        }
        println!("ok");
    }

    Ok(())
}

fn human_size(bytes: u64) -> String {
    if bytes >= 1024 * 1024 && bytes.is_multiple_of(1024 * 1024) {
        format!("{}M", bytes / (1024 * 1024))
    } else if bytes >= 1024 && bytes.is_multiple_of(1024) {
        format!("{}K", bytes / 1024)
    } else {
        format!("{bytes}")
    }
}

fn download(dev: &mut Device, job: &Job, index: usize, count: usize) -> Result<(), String> {
    println!();
    println!(
        "[{index}/{count}] {} -> RAM 0x{:08X} ({} bytes)",
        job.label,
        job.addr,
        job.data.len()
    );

    let total = job.data.len();
    let started = Instant::now();
    let mut progress = |done: usize| {
        print!(
            "\r        {:5.1}%  {done}/{total} bytes",
            done as f64 * 100.0 / total as f64
        );
        io::stdout().flush().ok();
    };

    dev.write_ram(job.addr, &job.data, &mut progress)?;

    let elapsed = started.elapsed();
    println!(
        "\r        done: {total} bytes in {:.1} s ({})          ",
        elapsed.as_secs_f64(),
        rate(total, elapsed)
    );
    Ok(())
}

fn print_device(info: &HwInfo) {
    let stage = match info.boot_stage {
        0 => "boot ROM",
        1 => "bootloader",
        _ => "unknown stage",
    };
    let chip_id = if info.chip_id.iter().all(|&word| word == 0) {
        "not available".to_string()
    } else {
        info.chip_id
            .iter()
            .map(|word| format!("{word:08X}"))
            .collect::<Vec<_>>()
            .join(" ")
    };

    println!("device  : ArtInChip {stage}, chip id: {chip_id}");
    println!(
        "          init mode 0x{:02X}, current mode 0x{:02X}",
        info.init_mode, info.curr_mode
    );
}

fn autodetect_port() -> Result<String, String> {
    let ports = serialport::available_ports()
        .map_err(|e| format!("cannot enumerate the serial ports: {e}"))?;

    match ports.len() {
        0 => Err("no serial port found, select one with --port".to_string()),
        1 => {
            let name = ports[0].port_name.clone();
            println!("note    : using the only serial port found: {name}");
            Ok(name)
        }
        _ => {
            let list = ports
                .iter()
                .map(|p| p.port_name.as_str())
                .collect::<Vec<_>>()
                .join(", ");
            Err(format!(
                "several serial ports found ({list}), select one with --port"
            ))
        }
    }
}

fn rate(bytes: usize, elapsed: Duration) -> String {
    let seconds = elapsed.as_secs_f64().max(1e-6);
    let per_second = bytes as f64 / seconds;

    if per_second >= 1024.0 * 1024.0 {
        format!("{:.2} MiB/s", per_second / (1024.0 * 1024.0))
    } else if per_second >= 1024.0 {
        format!("{:.1} KiB/s", per_second / 1024.0)
    } else {
        format!("{per_second:.0} B/s")
    }
}
