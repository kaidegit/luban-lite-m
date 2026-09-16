// Copyright (c) 2026, ArtInChip Technology Co., Ltd
//
// SPDX-License-Identifier: Apache-2.0
//
// Byte oriented link to the device: a real serial port in production, an
// in-memory model of the bootloader in the unit tests.

use std::io;
use std::time::{Duration, Instant};

use serialport::{ClearBuffer, DataBits, FlowControl, Parity, SerialPort, StopBits};

/// Timeout handed to the OS. Reads are driven by explicit deadlines on top of
/// it, writes are small enough to fit into the driver buffer.
const IO_TIMEOUT: Duration = Duration::from_millis(100);

pub trait Link {
    fn write_all(&mut self, data: &[u8]) -> Result<(), String>;

    /// Read at least one byte before `deadline`. `Ok(0)` means nothing arrived.
    fn read(&mut self, buf: &mut [u8], deadline: Instant) -> Result<usize, String>;

    /// Drop everything that is still buffered in the receive path.
    fn flush(&mut self);

    /// Reconfigure the speed of the link. Transports that cannot do it keep
    /// the default implementation, the device side only uses it for the
    /// SET_UART_ARGS command.
    fn set_baudrate(&mut self, _baud: u32) -> Result<(), String> {
        Ok(())
    }

    fn read_byte(&mut self, deadline: Instant) -> Result<Option<u8>, String> {
        let mut b = [0u8; 1];
        match self.read(&mut b, deadline)? {
            0 => Ok(None),
            _ => Ok(Some(b[0])),
        }
    }

    fn read_exact(&mut self, buf: &mut [u8], deadline: Instant) -> Result<(), String> {
        let mut done = 0;
        while done < buf.len() {
            match self.read(&mut buf[done..], deadline)? {
                0 => {
                    return Err(format!(
                        "timed out after {} of {} bytes from the device",
                        done,
                        buf.len()
                    ))
                }
                n => done += n,
            }
        }
        Ok(())
    }
}

pub struct Transport {
    port: Box<dyn SerialPort>,
}

impl Transport {
    pub fn open(name: &str, baud: u32) -> Result<Self, String> {
        let port = serialport::new(name, baud)
            .data_bits(DataBits::Eight)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .flow_control(FlowControl::None)
            .timeout(IO_TIMEOUT)
            .open()
            .map_err(|e| format!("cannot open {name} at {baud} bps: {e}"))?;

        Ok(Transport { port })
    }
}

impl Link for Transport {
    fn write_all(&mut self, data: &[u8]) -> Result<(), String> {
        self.port
            .write_all(data)
            .map_err(|e| format!("serial write failed: {e}"))
    }

    fn read(&mut self, buf: &mut [u8], deadline: Instant) -> Result<usize, String> {
        loop {
            match self.port.read(buf) {
                Ok(n) if n > 0 => return Ok(n),
                Ok(_) => {
                    // Some platforms report a timeout as a zero length read.
                    if Instant::now() >= deadline {
                        return Ok(0);
                    }
                }
                Err(e) if e.kind() == io::ErrorKind::TimedOut => {
                    if Instant::now() >= deadline {
                        return Ok(0);
                    }
                }
                Err(e) if e.kind() == io::ErrorKind::Interrupted => {}
                Err(e) => return Err(format!("serial read failed: {e}")),
            }
        }
    }

    fn flush(&mut self) {
        let _ = self.port.clear(ClearBuffer::Input);
    }

    fn set_baudrate(&mut self, baud: u32) -> Result<(), String> {
        self.port
            .set_baud_rate(baud)
            .map_err(|e| format!("cannot switch the port to {baud} bps: {e}"))?;
        Ok(())
    }
}
