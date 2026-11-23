//! Minimal Linux FFI bindings for fbdev, evdev, mmap, and signals.
//! Zero external crate dependencies; works on both x86_64-gnu and aarch64-musl.

#![allow(non_camel_case_types, dead_code)]

use std::ffi::c_void;

pub const O_RDONLY: i32 = 0;
pub const O_RDWR: i32 = 2;
pub const O_NONBLOCK: i32 = 0x800;

pub const PROT_READ: i32 = 1;
pub const PROT_WRITE: i32 = 2;
pub const MAP_SHARED: i32 = 1;

pub const SIGINT: i32 = 2;
pub const SIGTERM: i32 = 15;

// Linux fbdev ioctls
pub const FBIOGET_VSCREENINFO: u64 = 0x4600;
pub const FBIOGET_FSCREENINFO: u64 = 0x4602;

// Linux evdev event types
pub const EV_SYN: u16 = 0x00;
pub const EV_KEY: u16 = 0x01;
pub const EV_ABS: u16 = 0x03;

// Common evdev ABS codes
pub const ABS_X: u16 = 0x00;
pub const ABS_Y: u16 = 0x01;
pub const ABS_Z: u16 = 0x02;
pub const ABS_RX: u16 = 0x03;
pub const ABS_RY: u16 = 0x04;
pub const ABS_RZ: u16 = 0x05;
pub const ABS_THROTTLE: u16 = 0x06;
pub const ABS_RUDDER: u16 = 0x07;
pub const ABS_HAT0X: u16 = 0x10;
pub const ABS_HAT0Y: u16 = 0x11;
pub const ABS_MAX: usize = 0x3f;

// Common evdev KEY codes (keyboard + gamepad buttons)
pub const KEY_ESC: u16 = 1;
pub const KEY_W: u16 = 17;
pub const KEY_E: u16 = 18;
pub const KEY_R: u16 = 19;
pub const KEY_T: u16 = 20;
pub const KEY_A: u16 = 30;
pub const KEY_S: u16 = 31;
pub const KEY_D: u16 = 32;
pub const KEY_F: u16 = 33;
pub const KEY_I: u16 = 23;
pub const KEY_J: u16 = 36;
pub const KEY_K: u16 = 37;
pub const KEY_L: u16 = 38;
pub const KEY_SEMICOLON: u16 = 39;
pub const KEY_Z: u16 = 44;
pub const KEY_X: u16 = 45;
pub const KEY_C: u16 = 46;
pub const KEY_B: u16 = 48;
pub const KEY_N: u16 = 49;
pub const KEY_M: u16 = 50;
pub const KEY_COMMA: u16 = 51;
pub const KEY_DOT: u16 = 52;
pub const KEY_SLASH: u16 = 53;
pub const KEY_ENTER: u16 = 28;
pub const KEY_SPACE: u16 = 57;
pub const KEY_UP: u16 = 103;
pub const KEY_LEFT: u16 = 105;
pub const KEY_RIGHT: u16 = 106;
pub const KEY_DOWN: u16 = 108;
pub const KEY_BACKSPACE: u16 = 14;
pub const KEY_TAB: u16 = 15;

// Gamepad / joystick buttons
pub const BTN_SOUTH: u16 = 0x130; // A / Cross
pub const BTN_EAST: u16 = 0x131;  // B / Circle
pub const BTN_NORTH: u16 = 0x133; // X / Triangle
pub const BTN_WEST: u16 = 0x134;  // Y / Square
pub const BTN_TL: u16 = 0x136;    // LB / L1
pub const BTN_TR: u16 = 0x137;    // RB / R1
pub const BTN_SELECT: u16 = 0x13a;
pub const BTN_START: u16 = 0x13b;
pub const BTN_MODE: u16 = 0x13c;
pub const BTN_TRIGGER: u16 = 0x120; // Generic joystick button 0
pub const BTN_THUMB: u16 = 0x121;   // Generic joystick button 1

#[inline]
pub const fn eviocgname(len: usize) -> u64 {
    (2u64 << 30) | ((len as u64) << 16) | ((b'E' as u64) << 8) | 0x06
}

#[inline]
pub const fn eviocgbit(ev: u32, len: usize) -> u64 {
    (2u64 << 30) | ((len as u64) << 16) | ((b'E' as u64) << 8) | (0x20 + ev as u64)
}

#[inline]
pub const fn eviocgabs(abs: u32) -> u64 {
    (2u64 << 30) | (24u64 << 16) | ((b'E' as u64) << 8) | (0x40 + abs as u64)
}

#[repr(C)]
#[derive(Copy, Clone, Default, Debug)]
pub struct FbBitfield {
    pub offset: u32,
    pub length: u32,
    pub msb_right: u32,
}

#[repr(C)]
#[derive(Copy, Clone, Default, Debug)]
pub struct FbVarScreenInfo {
    pub xres: u32,
    pub yres: u32,
    pub xres_virtual: u32,
    pub yres_virtual: u32,
    pub xoffset: u32,
    pub yoffset: u32,
    pub bits_per_pixel: u32,
    pub grayscale: u32,
    pub red: FbBitfield,
    pub green: FbBitfield,
    pub blue: FbBitfield,
    pub transp: FbBitfield,
    pub nonstd: u32,
    pub activate: u32,
    pub height: u32,
    pub width: u32,
    pub accel_flags: u32,
    pub pixclock: u32,
    pub left_margin: u32,
    pub right_margin: u32,
    pub upper_margin: u32,
    pub lower_margin: u32,
    pub hsync_len: u32,
    pub vsync_len: u32,
    pub sync: u32,
    pub vmode: u32,
    pub rotate: u32,
    pub colorspace: u32,
    pub reserved: [u32; 4],
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct FbFixScreenInfo {
    pub id: [u8; 16],
    pub smem_start: usize,
    pub smem_len: u32,
    pub fb_type: u32,
    pub type_aux: u32,
    pub visual: u32,
    pub xpanstep: u16,
    pub ypanstep: u16,
    pub ywrapstep: u16,
    pub line_length: u32,
    pub mmio_start: usize,
    pub mmio_len: u32,
    pub accel: u32,
    pub capabilities: u16,
    pub reserved: [u16; 2],
}

impl Default for FbFixScreenInfo {
    fn default() -> Self {
        unsafe { std::mem::zeroed() }
    }
}

#[repr(C)]
#[derive(Copy, Clone, Default, Debug)]
pub struct InputEvent {
    pub tv_sec: i64,
    pub tv_usec: i64,
    pub type_: u16,
    pub code: u16,
    pub value: i32,
}

#[repr(C)]
#[derive(Copy, Clone, Default, Debug)]
pub struct InputAbsInfo {
    pub value: i32,
    pub minimum: i32,
    pub maximum: i32,
    pub fuzz: i32,
    pub flat: i32,
    pub resolution: i32,
}

extern "C" {
    pub fn open(pathname: *const std::ffi::c_char, flags: i32, ...) -> i32;
    pub fn close(fd: i32) -> i32;
    pub fn read(fd: i32, buf: *mut c_void, count: usize) -> isize;
    pub fn ioctl(fd: i32, request: u64, ...) -> i32;
    pub fn mmap(
        addr: *mut c_void,
        length: usize,
        prot: i32,
        flags: i32,
        fd: i32,
        offset: i64,
    ) -> *mut c_void;
    pub fn munmap(addr: *mut c_void, length: usize) -> i32;
    pub fn signal(signum: i32, handler: extern "C" fn(i32)) -> usize;
}
