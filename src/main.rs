//! Copy the vector table into ram
//! and set the GPIO vector to call another function
#![no_std]
#![no_main]

use cortex_m::asm::dsb;
use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::fixed_point::FixedPoint;
use panic_probe as _;
use rp2040_hal as hal;

use hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use pac::interrupt;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[repr(C, align(256))]
struct vtor {
    sys_handlers: [usize; 16],
    irqs: [usize; 32],
}

static mut RAM_VTOR: vtor = vtor {
    sys_handlers: [0; 16],
    irqs: [0; 32],
};

fn set_irq_handler(irq_num: usize, func: fn()) {
    unsafe {
        let fptr = func as *const fn();
        let fp_usize = (fptr as *const usize) as usize;
        RAM_VTOR.irqs[irq_num] = fp_usize;
    }
    dsb();
}

fn copy_vtor(vtor_addr: usize, vtor_tgt: &mut vtor) {
    let vtor_ptr = vtor_addr as *const usize;
    for i in 0..16usize {
        vtor_tgt.sys_handlers[i] = unsafe { vtor_ptr.wrapping_add(i).read() };
    }
    for i in 0..32usize {
        vtor_tgt.irqs[i] = unsafe { vtor_ptr.wrapping_add(i + 16).read() };
    }
}

fn switch_to_vtor(address: u32) {
    unsafe {
        cortex_m::interrupt::disable();
        dsb();
        (*(cortex_m::peripheral::SCB::PTR)).vtor.write(address);
        dsb();
        cortex_m::interrupt::enable();
    }
}

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let vtor_addr = unsafe { (*(cortex_m::peripheral::SCB::PTR)).vtor.read() } as usize;
    unsafe {
        copy_vtor(vtor_addr, &mut RAM_VTOR);
    }

    // Re-wire the IO_IRQ_BANK0 interrupt (13) to target function p
    set_irq_handler(13, d);
    let ram_vtor_ptr = unsafe { &RAM_VTOR as *const vtor as *const u32 };
    switch_to_vtor(ram_vtor_ptr as u32);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.gpio25.into_push_pull_output();

    let _input = pins.gpio3.into_pull_up_input();
    // Enable interrupt on edges for GPIO3
    let pac = unsafe { pac::Peripherals::steal() };
    // Enable interrupt on rising and falling edge of
    pac.IO_BANK0.proc0_inte0.modify(|_, w| {
        w.gpio3_edge_high().set_bit();
        w.gpio3_edge_low().set_bit();
        w
    });

    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::IO_IRQ_BANK0);
    };

    loop {
        info!("on!");
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        info!("off!");
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }
}

fn d() {
    info!("triggered new handler")
}

#[interrupt]
unsafe fn IO_IRQ_BANK0() {
    info!("Original irq handler")
}
