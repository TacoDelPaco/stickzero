//! Minimal Linux FFI bindings for fbdev, evdev, mmap, and signals.
//! Zero external crate dependencies; works on both x86_64-gnu and aarch64-musl.

#![allow(non_camel_case_types, dead_code)]

use std::ffi::c_void;

pub const O_RDONLY: i32 = 0;
pub const O_RDWR: i32 = 2;
pub const O_CREAT: i32 = 0o100;
pub const O_EXCL: i32 = 0o200;
pub const O_NONBLOCK: i32 = 0x800;
pub const O_CLOEXEC: i32 = 0o2000000;

pub const PROT_READ: i32 = 1;
pub const PROT_WRITE: i32 = 2;
pub const MAP_SHARED: i32 = 1;

pub const SIGINT: i32 = 2;
pub const SIGTERM: i32 = 15;

// Linux fbdev ioctls
pub const FBIOGET_VSCREENINFO: u64 = 0x4600;
pub const FBIOGET_FSCREENINFO: u64 = 0x4602;
pub const FBIOPAN_DISPLAY: u64 = 0x4606;

// Linux DRM/KMS ioctls (verified against linux/drm/drm.h & drm_mode.h)
pub const DRM_IOCTL_MODE_GETRESOURCES: u64 = 0xc040_64a0;
pub const DRM_IOCTL_MODE_GETCRTC: u64 = 0xc068_64a1;
pub const DRM_IOCTL_MODE_SETCRTC: u64 = 0xc068_64a2;
pub const DRM_IOCTL_MODE_GETENCODER: u64 = 0xc014_64a6;
pub const DRM_IOCTL_MODE_GETCONNECTOR: u64 = 0xc050_64a7;
pub const DRM_IOCTL_MODE_ADDFB: u64 = 0xc01c_64ae;
pub const DRM_IOCTL_MODE_RMFB: u64 = 0xc004_64af;
pub const DRM_IOCTL_MODE_DIRTYFB: u64 = 0xc018_64b1;
pub const DRM_IOCTL_MODE_CREATE_DUMB: u64 = 0xc020_64b2;
pub const DRM_IOCTL_MODE_MAP_DUMB: u64 = 0xc010_64b3;
pub const DRM_IOCTL_MODE_DESTROY_DUMB: u64 = 0xc004_64b4;
pub const DRM_IOCTL_MODE_ADDFB2: u64 = 0xc068_64b8;

pub const DRM_MODE_CONNECTED: u32 = 1;
pub const DRM_MODE_CONNECTOR_HDMIA: u32 = 11;
pub const DRM_MODE_CONNECTOR_HDMIB: u32 = 12;
pub const DRM_MODE_TYPE_PREFERRED: u32 = 1 << 3;
pub const DRM_FORMAT_RGB565: u32 = 0x3631_4752; // 'R','G','1','6'
pub const DRM_FORMAT_XRGB8888: u32 = 0x3432_5258; // 'X','R','2','4'

// Linux socket / ancillary fd passing & memfd constants (x86_64 & aarch64)
pub const SOL_SOCKET: i32 = 1;
pub const SCM_RIGHTS: i32 = 1;
pub const MSG_DONTWAIT: i32 = 0x40;
pub const MSG_NOSIGNAL: i32 = 0x4000;
pub const MFD_CLOEXEC: u32 = 1;

#[cfg(target_arch = "x86_64")]
pub const SYS_MEMFD_CREATE: i64 = 319;
#[cfg(target_arch = "aarch64")]
pub const SYS_MEMFD_CREATE: i64 = 279;

// Linux evdev event types
pub const EV_SYN: u16 = 0x00;
pub const EV_KEY: u16 = 0x01;
pub const EV_ABS: u16 = 0x03;

// Linux evdev SYN codes
pub const SYN_REPORT: u16 = 0;
pub const SYN_DROPPED: u16 = 3;

// Linux input bus types (linux/input.h)
pub const BUS_USB: u16 = 0x03;
pub const BUS_BLUETOOTH: u16 = 0x05;

// Linux clock IDs for EVIOCSCLOCKID
pub const CLOCK_MONOTONIC: i32 = 1;

// Linux poll(2) event masks
pub const POLLIN: i16 = 0x0001;
pub const POLLERR: i16 = 0x0008;
pub const POLLHUP: i16 = 0x0010;
pub const POLLNVAL: i16 = 0x0020;

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
pub const ABS_MISC: u16 = 0x28;
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

// Linux hidraw constants & ioctls (linux/hidraw.h)
pub const HID_MAX_DESCRIPTOR_SIZE: usize = 4096;

#[inline]
pub const fn eviocgid() -> u64 {
    (2u64 << 30) | (8u64 << 16) | ((b'E' as u64) << 8) | 0x02
}

#[inline]
pub const fn eviocgname(len: usize) -> u64 {
    (2u64 << 30) | ((len as u64) << 16) | ((b'E' as u64) << 8) | 0x06
}

#[inline]
pub const fn eviocgkey(len: usize) -> u64 {
    (2u64 << 30) | ((len as u64) << 16) | ((b'E' as u64) << 8) | 0x18
}

#[inline]
pub const fn eviocgbit(ev: u32, len: usize) -> u64 {
    (2u64 << 30) | ((len as u64) << 16) | ((b'E' as u64) << 8) | (0x20 + ev as u64)
}

#[inline]
pub const fn eviocgabs(abs: u32) -> u64 {
    (2u64 << 30) | (24u64 << 16) | ((b'E' as u64) << 8) | (0x40 + abs as u64)
}

#[inline]
pub const fn eviocsclockid() -> u64 {
    (1u64 << 30) | (4u64 << 16) | ((b'E' as u64) << 8) | 0xa0
}

#[inline]
pub const fn hidiocgrdescsize() -> u64 {
    (2u64 << 30) | (4u64 << 16) | ((b'H' as u64) << 8) | 0x01
}

#[inline]
pub const fn hidiocgrdesc() -> u64 {
    (2u64 << 30) | ((4 + HID_MAX_DESCRIPTOR_SIZE as u64) << 16) | ((b'H' as u64) << 8) | 0x02
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

#[repr(C)]
#[derive(Copy, Clone, Default, Debug)]
pub struct InputId {
    pub bustype: u16,
    pub vendor: u16,
    pub product: u16,
    pub version: u16,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct HidrawReportDescriptor {
    pub size: u32,
    pub value: [u8; HID_MAX_DESCRIPTOR_SIZE],
}

impl Default for HidrawReportDescriptor {
    fn default() -> Self {
        Self {
            size: 0,
            value: [0u8; HID_MAX_DESCRIPTOR_SIZE],
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone, Default, Debug)]
pub struct PollFd {
    pub fd: i32,
    pub events: i16,
    pub revents: i16,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct DrmModeModeInfo {
    pub clock: u32,
    pub hdisplay: u16,
    pub hsync_start: u16,
    pub hsync_end: u16,
    pub htotal: u16,
    pub hskew: u16,
    pub vdisplay: u16,
    pub vsync_start: u16,
    pub vsync_end: u16,
    pub vtotal: u16,
    pub vscan: u16,
    pub vrefresh: u32,
    pub flags: u32,
    pub type_: u32,
    pub name: [u8; 32],
}

impl Default for DrmModeModeInfo {
    fn default() -> Self {
        unsafe { std::mem::zeroed() }
    }
}

#[repr(C)]
#[derive(Copy, Clone, Default, Debug)]
pub struct DrmModeCardRes {
    pub fb_id_ptr: u64,
    pub crtc_id_ptr: u64,
    pub connector_id_ptr: u64,
    pub encoder_id_ptr: u64,
    pub count_fbs: u32,
    pub count_crtcs: u32,
    pub count_connectors: u32,
    pub count_encoders: u32,
    pub min_width: u32,
    pub max_width: u32,
    pub min_height: u32,
    pub max_height: u32,
}

#[repr(C)]
#[derive(Copy, Clone, Default, Debug)]
pub struct DrmModeCrtc {
    pub set_connectors_ptr: u64,
    pub count_connectors: u32,
    pub crtc_id: u32,
    pub fb_id: u32,
    pub x: u32,
    pub y: u32,
    pub gamma_size: u32,
    pub mode_valid: u32,
    pub mode: DrmModeModeInfo,
}

#[repr(C)]
#[derive(Copy, Clone, Default, Debug)]
pub struct DrmModeGetEncoder {
    pub encoder_id: u32,
    pub encoder_type: u32,
    pub crtc_id: u32,
    pub possible_crtcs: u32,
    pub possible_clones: u32,
}

#[repr(C)]
#[derive(Copy, Clone, Default, Debug)]
pub struct DrmModeGetConnector {
    pub encoders_ptr: u64,
    pub modes_ptr: u64,
    pub props_ptr: u64,
    pub prop_values_ptr: u64,
    pub count_modes: u32,
    pub count_props: u32,
    pub count_encoders: u32,
    pub encoder_id: u32,
    pub connector_id: u32,
    pub connector_type: u32,
    pub connector_type_id: u32,
    pub connection: u32,
    pub mm_width: u32,
    pub mm_height: u32,
    pub subpixel: u32,
    pub pad: u32,
}

#[repr(C)]
#[derive(Copy, Clone, Default, Debug)]
pub struct DrmModeFbCmd {
    pub fb_id: u32,
    pub width: u32,
    pub height: u32,
    pub pitch: u32,
    pub bpp: u32,
    pub depth: u32,
    pub handle: u32,
}

#[repr(C)]
#[derive(Copy, Clone, Default, Debug)]
pub struct DrmModeFbCmd2 {
    pub fb_id: u32,
    pub width: u32,
    pub height: u32,
    pub pixel_format: u32,
    pub flags: u32,
    pub handles: [u32; 4],
    pub pitches: [u32; 4],
    pub offsets: [u32; 4],
    pub modifier: [u64; 4],
}

#[repr(C)]
#[derive(Copy, Clone, Default, Debug)]
pub struct DrmModeCreateDumb {
    pub height: u32,
    pub width: u32,
    pub bpp: u32,
    pub flags: u32,
    pub handle: u32,
    pub pitch: u32,
    pub size: u64,
}

#[repr(C)]
#[derive(Copy, Clone, Default, Debug)]
pub struct DrmModeMapDumb {
    pub handle: u32,
    pub pad: u32,
    pub offset: u64,
}

#[repr(C)]
#[derive(Copy, Clone, Default, Debug)]
pub struct DrmModeDestroyDumb {
    pub handle: u32,
}

#[repr(C)]
#[derive(Copy, Clone, Default, Debug)]
pub struct DrmModeFbDirtyCmd {
    pub fb_id: u32,
    pub flags: u32,
    pub color: u32,
    pub num_clips: u32,
    pub clips_ptr: u64,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct Iovec {
    pub iov_base: *mut c_void,
    pub iov_len: usize,
}

/// 64-bit little-endian Linux `struct msghdr` layout (ABI-compatible across x86_64-gnu and aarch64-musl).
#[repr(C)]
#[derive(Copy, Clone)]
pub struct Msghdr {
    pub msg_name: *mut c_void,
    pub msg_namelen: u32,
    pub _pad0: u32,
    pub msg_iov: *mut Iovec,
    pub msg_iovlen: usize,
    pub msg_control: *mut c_void,
    pub msg_controllen: usize,
    pub msg_flags: i32,
    pub _pad1: u32,
}

/// 64-bit little-endian Linux `struct cmsghdr` layout (ABI-compatible across x86_64-gnu and aarch64-musl).
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct Cmsghdr {
    pub cmsg_len: usize,
    pub cmsg_level: i32,
    pub cmsg_type: i32,
}

extern "C" {
    pub fn open(pathname: *const std::ffi::c_char, flags: i32, ...) -> i32;
    pub fn close(fd: i32) -> i32;
    pub fn read(fd: i32, buf: *mut c_void, count: usize) -> isize;
    pub fn poll(fds: *mut PollFd, nfds: usize, timeout: i32) -> i32;
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
    pub fn getuid() -> u32;
    pub fn ftruncate(fd: i32, length: i64) -> i32;
    pub fn unlink(pathname: *const std::ffi::c_char) -> i32;
    pub fn sendmsg(sockfd: i32, msg: *const Msghdr, flags: i32) -> isize;
    pub fn recvmsg(sockfd: i32, msg: *mut Msghdr, flags: i32) -> isize;
    pub fn syscall(number: i64, ...) -> i64;
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::mem::size_of;

    #[test]
    fn test_drm_and_socket_abi_sizes() {
        assert_eq!(size_of::<DrmModeModeInfo>(), 68);
        assert_eq!(size_of::<DrmModeCardRes>(), 64);
        assert_eq!(size_of::<DrmModeCrtc>(), 104);
        assert_eq!(size_of::<DrmModeGetEncoder>(), 20);
        assert_eq!(size_of::<DrmModeGetConnector>(), 80);
        assert_eq!(size_of::<DrmModeFbCmd>(), 28);
        assert_eq!(size_of::<DrmModeFbCmd2>(), 104);
        assert_eq!(size_of::<DrmModeCreateDumb>(), 32);
        assert_eq!(size_of::<DrmModeMapDumb>(), 16);
        assert_eq!(size_of::<DrmModeDestroyDumb>(), 4);
        assert_eq!(size_of::<DrmModeFbDirtyCmd>(), 24);
        assert_eq!(size_of::<Iovec>(), 16);
        assert_eq!(size_of::<Msghdr>(), 56);
        assert_eq!(size_of::<Cmsghdr>(), 16);
    }
}
