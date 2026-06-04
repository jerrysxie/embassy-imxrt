//! I2C loopback async example using the `embedded_mcu_hal::i2c::target` trait.
//!
//! This is the trait-based counterpart to `i2c-loopback-async.rs`. The
//! master side is identical (it uses the `embedded_hal_async::i2c::I2c`
//! controller trait as before); only the slave side is changed to go
//! through the `embedded_mcu_hal::i2c::target::asynch::I2c` trait instead
//! of the inherent `I2cSlave` methods.
//!
//! The slave buffer is intentionally set to `MASTER_BUFLEN / 2` to
//! exercise the `NeedMore` / `BufferFull` continuation paths — the slave
//! must loop and re-supply buffers when the master transacts more bytes
//! than fit in a single call.

#![no_std]
#![no_main]

use defmt::{info, warn};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_imxrt::i2c::master::{DutyCycle, I2cMaster};
use embassy_imxrt::i2c::slave::{Address, I2cSlave};
use embassy_imxrt::i2c::{self, Async};
use embassy_imxrt::{bind_interrupts, peripherals};
use embassy_imxrt_examples as _;
use embedded_hal_async::i2c::I2c;
use embedded_mcu_hal::i2c::SevenBitAddress;
use embedded_mcu_hal::i2c::target::asynch::I2c as TargetI2c;
use embedded_mcu_hal::i2c::target::{ReadStatus, Request, WriteStatus};
use panic_probe as _;

const ADDR: u8 = 0x20;
const MASTER_BUFLEN: usize = 8;
// slave buffer is smaller than master buffer to exercise the
// NeedMore / BufferFull continuation paths.
const SLAVE_BUFLEN: usize = MASTER_BUFLEN / 2;
const SLAVE_ADDR: Option<Address> = Address::new(ADDR);

bind_interrupts!(struct Irqs {
    FLEXCOMM2 => i2c::InterruptHandler<peripherals::FLEXCOMM2>;
    FLEXCOMM4 => i2c::InterruptHandler<peripherals::FLEXCOMM4>;
});

#[embassy_executor::task]
async fn slave_service(mut slave: I2cSlave<'static, Async>) {
    let mut expect_repeated_start = false;

    loop {
        let mut r_buf = [0xAA; SLAVE_BUFLEN];
        let mut t_buf = [0xAA; SLAVE_BUFLEN];

        for (i, e) in t_buf.iter_mut().enumerate() {
            *e = i as u8;
        }

        let req: Request<SevenBitAddress> = match TargetI2c::<SevenBitAddress>::listen(&mut slave).await {
            Ok(r) => r,
            Err(e) => {
                info!("listen error: {:?}", defmt::Debug2Format(&e));
                expect_repeated_start = false;
                continue;
            }
        };

        let was_expecting_restart = expect_repeated_start;
        expect_repeated_start = false;

        match req {
            Request::Stop(addr) => {
                info!("Stop @ 0x{:02X} (probe)", addr);
                if was_expecting_restart {
                    warn!(
                        "RACE WATCH: prior respond_to_* reported Restarted but listen() \
                         returned Stop(0x{:02X}); expected RepeatedStart",
                        addr
                    );
                }
            }
            Request::RepeatedStart(prev_addr) => {
                info!("RepeatedStart from prev @ 0x{:02X}", prev_addr);
                if !was_expecting_restart {
                    warn!(
                        "RACE WATCH: RepeatedStart(0x{:02X}) surfaced without a prior \
                         Restarted(_) — likely a spurious edge",
                        prev_addr
                    );
                }
            }
            Request::Read(addr) => {
                info!("Read @ 0x{:02X}", addr);
                loop {
                    match TargetI2c::<SevenBitAddress>::respond_to_read(&mut slave, &t_buf).await {
                        Ok(ReadStatus::Complete(n)) => {
                            info!("Read complete with {} bytes", n);
                            break;
                        }
                        Ok(ReadStatus::EarlyStop(n)) => {
                            info!("Read terminated by controller after {} bytes", n);
                            break;
                        }
                        Ok(ReadStatus::NeedMore(n)) => {
                            info!("Read NeedMore: sent {} bytes so far, more requested", n);
                        }
                        Ok(_) => {
                            info!("Read: unknown status variant");
                            break;
                        }
                        Err(e) => {
                            info!("respond_to_read error: {:?}", defmt::Debug2Format(&e));
                            break;
                        }
                    }
                }
            }
            Request::Write(addr) => {
                info!("Write @ 0x{:02X}", addr);
                loop {
                    match TargetI2c::<SevenBitAddress>::respond_to_write(&mut slave, &mut r_buf).await {
                        Ok(WriteStatus::Stopped(n)) => {
                            info!("Write stopped after {} bytes", n);
                            break;
                        }
                        Ok(WriteStatus::Restarted(n)) => {
                            info!("Write restarted after {} bytes", n);
                            if n == 0 {
                                warn!(
                                    "RACE WATCH: WriteStatus::Restarted(0) — zero-byte restart \
                                     should not occur on a healthy bus."
                                );
                            }
                            expect_repeated_start = true;
                            break;
                        }
                        Ok(WriteStatus::BufferFull(n)) => {
                            info!("Write BufferFull after {} bytes — supplying more buffer space", n);
                        }
                        Ok(_) => {
                            info!("Write: unknown status variant");
                            break;
                        }
                        Err(e) => {
                            info!("respond_to_write error: {:?}", defmt::Debug2Format(&e));
                            break;
                        }
                    }
                }
            }
            _ => {
                info!("unhandled request variant");
            }
        }
    }
}

#[embassy_executor::task]
async fn master_service(mut master: I2cMaster<'static, Async>) {
    const ADDR: u8 = 0x20;

    let mut w_buf = [0xAA; MASTER_BUFLEN];
    let mut r_buf = [0xAA; MASTER_BUFLEN];

    // Initialize write buffer with increment numbers
    for (i, e) in w_buf.iter_mut().enumerate() {
        *e = i as u8;
    }

    let mut i: usize = 0;
    loop {
        if i % 300 < 100 {
            let w_end = i % w_buf.len();
            info!("i2cm write {} bytes", w_end);
            master.write(ADDR, &w_buf[0..w_end]).await.unwrap();
        } else if i % 300 < 200 {
            let r_end = i % (r_buf.len() - 1) + 2;
            info!("i2cm read {} bytes", r_end);
            master.read(ADDR, &mut r_buf[0..r_end]).await.unwrap();
        } else {
            let tend = i % w_buf.len() + 1;
            let r_end = i % (r_buf.len() - 1) + 2;
            info!("i2cm write {} bytes, read {} bytes", tend, r_end);
            master
                .write_read(ADDR, &w_buf[0..tend], &mut r_buf[0..r_end])
                .await
                .unwrap();
        }
        i += 1;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_imxrt::init(Default::default());

    info!("i2c loopback target-trait example");

    let slave = I2cSlave::new_async(p.FLEXCOMM2, p.PIO0_18, p.PIO0_17, Irqs, SLAVE_ADDR.unwrap(), p.DMA0_CH4).unwrap();

    let config = i2c::master::Config {
        speed: i2c::master::Speed::Fast,
        duty_cycle: DutyCycle::new(50).unwrap(),
        ..Default::default()
    };
    let master = I2cMaster::new_async(p.FLEXCOMM4, p.PIO0_29, p.PIO0_30, Irqs, config, p.DMA0_CH9).unwrap();

    spawner.spawn(master_service(master).unwrap());
    spawner.spawn(slave_service(slave).unwrap());
}
