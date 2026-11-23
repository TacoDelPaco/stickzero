//! CardputerZero Linux fbdev display backend.
//!
//! Automatically finds `fb_st7789v` via `/proc/fb` (often `/dev/fb1` on CardputerZero),
//! respecting `STICKZERO_FBDEV` and `LV_LINUX_FBDEV_DEVICE` overrides.
//! Supports both 16-bit RGB565 (direct fast blit) and 32-bit XRGB8888 (HDMI fallback).

use crate::linux_sys::{
    close, ioctl, mmap, munmap, open, FbFixScreenInfo, FbVarScreenInfo, FBIOGET_FSCREENINFO,
    FBIOGET_VSCREENINFO, MAP_SHARED, O_RDWR, PROT_READ, PROT_WRITE,
};
use std::ffi::CString;
use std::fs;
use std::ptr;

pub const SCREEN_W: usize = 320;
pub const SCREEN_H: usize = 170;

pub struct FbDisplay {
    fd: i32,
    ptr: *mut u8,
    len: usize,
    width: usize,
    height: usize,
    bpp: usize,
    stride: usize,
    pub device_path: String,
}

impl FbDisplay {
    /// Detect the best framebuffer device for CardputerZero.
    pub fn detect_device() -> String {
        if let Ok(dev) = std::env::var("STICKZERO_FBDEV") {
            if !dev.trim().is_empty() {
                return dev;
            }
        }
        if let Ok(dev) = std::env::var("LV_LINUX_FBDEV_DEVICE") {
            if !dev.trim().is_empty() {
                return dev;
            }
        }
        // Scan /proc/fb for CardputerZero's ST7789V LCD (usually fb1)
        if let Ok(proc_fb) = fs::read_to_string("/proc/fb") {
            for line in proc_fb.lines() {
                let lower = line.to_ascii_lowercase();
                if lower.contains("st7789") || lower.contains("fb_st7789v") {
                    if let Some(num) = line.split_whitespace().next() {
                        return format!("/dev/fb{num}");
                    }
                }
            }
        }
        "/dev/fb0".to_string()
    }

    pub fn open_auto() -> Result<Self, String> {
        let path = Self::detect_device();
        Self::open(&path)
    }

    pub fn open(path: &str) -> Result<Self, String> {
        let cpath = CString::new(path).map_err(|_| "invalid fb path".to_string())?;
        let fd = unsafe { open(cpath.as_ptr(), O_RDWR) };
        if fd < 0 {
            return Err(format!("failed to open framebuffer {path}"));
        }

        let mut vinfo = FbVarScreenInfo::default();
        let mut finfo = FbFixScreenInfo::default();
        let r1 = unsafe { ioctl(fd, FBIOGET_VSCREENINFO, &mut vinfo) };
        let r2 = unsafe { ioctl(fd, FBIOGET_FSCREENINFO, &mut finfo) };
        if r1 < 0 || r2 < 0 {
            unsafe { close(fd) };
            return Err(format!("fbdev ioctl failed on {path}"));
        }

        let len = finfo.smem_len as usize;
        if len == 0 {
            unsafe { close(fd) };
            return Err(format!("fbdev smem_len is 0 on {path}"));
        }

        let mapped = unsafe {
            mmap(
                ptr::null_mut(),
                len,
                PROT_READ | PROT_WRITE,
                MAP_SHARED,
                fd,
                0,
            )
        };
        if mapped == (!0usize as *mut std::ffi::c_void) {
            unsafe { close(fd) };
            return Err(format!("mmap failed on {path}"));
        }

        Ok(Self {
            fd,
            ptr: mapped as *mut u8,
            len,
            width: vinfo.xres as usize,
            height: vinfo.yres as usize,
            bpp: vinfo.bits_per_pixel as usize,
            stride: finfo.line_length as usize,
            device_path: path.to_string(),
        })
    }

    /// Blit our 320x170 RGB565 backbuffer directly into the mmap'd framebuffer.
    pub fn present(&mut self, rgb565: &[u16; SCREEN_W * SCREEN_H]) {
        if self.bpp == 16 {
            // Fast path: CardputerZero native 320x170 RGB565
            if self.width == SCREEN_W && self.height == SCREEN_H && self.stride == SCREEN_W * 2 {
                unsafe {
                    ptr::copy_nonoverlapping(
                        rgb565.as_ptr() as *const u8,
                        self.ptr,
                        SCREEN_W * SCREEN_H * 2,
                    );
                }
                return;
            }
            let copy_w = self.width.min(SCREEN_W);
            let copy_h = self.height.min(SCREEN_H);
            let x_off = (self.width.saturating_sub(copy_w)) / 2;
            let y_off = (self.height.saturating_sub(copy_h)) / 2;
            for y in 0..copy_h {
                let dst_offset = (y + y_off) * self.stride + x_off * 2;
                if dst_offset + copy_w * 2 <= self.len {
                    unsafe {
                        ptr::copy_nonoverlapping(
                            rgb565.as_ptr().add(y * SCREEN_W) as *const u8,
                            self.ptr.add(dst_offset),
                            copy_w * 2,
                        );
                    }
                }
            }
        } else if self.bpp == 32 {
            // 32-bit XRGB8888 fallback (e.g. HDMI / desktop fb) with nearest integer scale
            let scale_x = (self.width / SCREEN_W).max(1);
            let scale_y = (self.height / SCREEN_H).max(1);
            let scale = scale_x.min(scale_y);
            let out_w = SCREEN_W * scale;
            let out_h = SCREEN_H * scale;
            let x_off = (self.width.saturating_sub(out_w)) / 2;
            let y_off = (self.height.saturating_sub(out_h)) / 2;

            for sy in 0..SCREEN_H {
                for sx in 0..SCREEN_W {
                    let c = rgb565[sy * SCREEN_W + sx];
                    let r = (((c >> 11) & 0x1F) * 255 / 31) as u32;
                    let g = (((c >> 5) & 0x3F) * 255 / 63) as u32;
                    let b = ((c & 0x1F) * 255 / 31) as u32;
                    let px = 0xFF00_0000 | (r << 16) | (g << 8) | b;
                    for dy in 0..scale {
                        let py = y_off + sy * scale + dy;
                        let row_off = py * self.stride;
                        for dx in 0..scale {
                            let off = row_off + (x_off + sx * scale + dx) * 4;
                            if off + 4 <= self.len {
                                unsafe {
                                    *(self.ptr.add(off) as *mut u32) = px;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    pub fn clear_black(&mut self) {
        unsafe {
            ptr::write_bytes(self.ptr, 0, self.len);
        }
    }
}

impl Drop for FbDisplay {
    fn drop(&mut self) {
        self.clear_black();
        unsafe {
            munmap(self.ptr as *mut std::ffi::c_void, self.len);
            close(self.fd);
        }
    }
}
