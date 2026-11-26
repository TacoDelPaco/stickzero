//! CardputerZero dual-display backend: 1.9" ST7789v3 LCD + Mini-HDMI (Wayland / DRM-KMS / fbdev).
//!
//! Automatically discovers CardputerZero's built-in 320x170 ST7789v3 RGB565 LCD (honoring
//! `APPLAUNCH_LINUX_FBDEV_DEVICE`, `LV_LINUX_FBDEV_DEVICE`, `/proc/fb`, and `/dev/fb_lcd`)
//! alongside any connected Mini-HDMI output via:
//!   1. Zero-dependency Wayland client (`labwc-pi` on stock CardputerZero OS)
//!   2. Zero-dependency DRM/KMS dumb-buffer scanout (headless / Pi OS Lite / `lightdm` off)
//!   3. Linux fbdev (`vc4drmfb` / `BCM2708 FB` / `simpledrm`) with fast 16/24/32-bit scaling
//!
//! Also supports live HDMI hotplug/unplug detection and explicit `STICKZERO_FBDEV` /
//! `STICKZERO_HDMI_BACKEND` overrides.

use crate::config::{DisplayOutput, HdmiLcdBacklight, HdmiResolution};
use crate::drm::DrmDisplay;
use crate::linux_sys::{
    close, ioctl, mmap, munmap, open, FbFixScreenInfo, FbVarScreenInfo, FBIOGET_FSCREENINFO,
    FBIOGET_VSCREENINFO, MAP_SHARED, O_RDWR, PROT_READ, PROT_WRITE,
};
use crate::wayland::WaylandDisplay;
use std::ffi::CString;
use std::fs;
use std::path::{Path, PathBuf};
use std::ptr;
use std::time::{Duration, Instant};

pub const SCREEN_W: usize = 320;
pub const SCREEN_H: usize = 170;

const HOTPLUG_INTERVAL: Duration = Duration::from_millis(1500);

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum PixelLayout {
    Rgb565,
    Rgb24 {
        r_byte: usize,
        g_byte: usize,
        b_byte: usize,
    },
    Rgb32 {
        r_shift: u32,
        g_shift: u32,
        b_shift: u32,
        alpha_mask: u32,
    },
}

impl PixelLayout {
    pub const fn xrgb8888() -> Self {
        Self::Rgb32 {
            r_shift: 16,
            g_shift: 8,
            b_shift: 0,
            alpha_mask: 0xFF00_0000,
        }
    }

    pub fn from_fb_var(vinfo: &FbVarScreenInfo) -> Option<Self> {
        match vinfo.bits_per_pixel {
            16 => Some(Self::Rgb565),
            24 => {
                let (r, g, b) = if vinfo.red.length > 0 || vinfo.blue.length > 0 {
                    (
                        (vinfo.red.offset / 8).min(2) as usize,
                        (vinfo.green.offset / 8).min(2) as usize,
                        (vinfo.blue.offset / 8).min(2) as usize,
                    )
                } else {
                    (2, 1, 0) // Little-endian BGR24 default
                };
                Some(Self::Rgb24 {
                    r_byte: r,
                    g_byte: g,
                    b_byte: b,
                })
            }
            32 => {
                let (r_shift, g_shift, b_shift) = if vinfo.red.length > 0 || vinfo.blue.length > 0 {
                    (
                        vinfo.red.offset.min(24),
                        vinfo.green.offset.min(24),
                        vinfo.blue.offset.min(24),
                    )
                } else {
                    (16, 8, 0) // XRGB8888 default
                };
                let alpha_mask = if vinfo.transp.length > 0 && vinfo.transp.offset <= 24 {
                    0xFFu32 << vinfo.transp.offset
                } else {
                    let used = (0xFFu32 << r_shift) | (0xFFu32 << g_shift) | (0xFFu32 << b_shift);
                    !used
                };
                Some(Self::Rgb32 {
                    r_shift,
                    g_shift,
                    b_shift,
                    alpha_mask,
                })
            }
            _ => None,
        }
    }
}

/// Compute aspect-preserving centered output rectangle `(out_w, out_h, x_off, y_off)` for
/// mapping a `(src_w, src_h)` framebuffer onto an arbitrary `(dst_w, dst_h)` display.
///
/// Prefers crisp exact integer scaling (e.g. 640x360 -> 2x=1280x720 / 3x=1920x1080,
/// 320x170 -> 4x=1280x680 / 6x=1920x1020) whenever an integer multiple fills at least 85%
/// of either axis; otherwise uses full aspect-preserving nearest-neighbor fit (including
/// downscaling e.g. 640x360 -> 302x170 on the built-in 320x170 LCD).
pub fn compute_fit_rect_for(
    src_w: usize,
    src_h: usize,
    dst_w: usize,
    dst_h: usize,
) -> (usize, usize, usize, usize) {
    if src_w == 0 || src_h == 0 || dst_w == 0 || dst_h == 0 {
        return (0, 0, 0, 0);
    }
    if dst_w == src_w && dst_h == src_h {
        return (dst_w, dst_h, 0, 0);
    }

    let (out_w, out_h) = if dst_w >= src_w && dst_h >= src_h {
        let int_scale = (dst_w / src_w).min(dst_h / src_h);
        if int_scale >= 1
            && (src_w * int_scale * 100 >= dst_w * 85 || src_h * int_scale * 100 >= dst_h * 85)
        {
            (src_w * int_scale, src_h * int_scale)
        } else if dst_w * src_h <= dst_h * src_w {
            (dst_w, ((dst_w * src_h) / src_w).max(1).min(dst_h))
        } else {
            (((dst_h * src_w) / src_h).max(1).min(dst_w), dst_h)
        }
    } else if dst_w * src_h <= dst_h * src_w {
        (dst_w, ((dst_w * src_h) / src_w).max(1).min(dst_h))
    } else {
        (((dst_h * src_w) / src_h).max(1).min(dst_w), dst_h)
    };

    let x_off = (dst_w.saturating_sub(out_w)) / 2;
    let y_off = (dst_h.saturating_sub(out_h)) / 2;
    (out_w, out_h, x_off, y_off)
}

/// Convenience wrapper for mapping CardputerZero's native 320x170 framebuffer onto `(dst_w, dst_h)`.
pub fn compute_fit_rect(dst_w: usize, dst_h: usize) -> (usize, usize, usize, usize) {
    compute_fit_rect_for(SCREEN_W, SCREEN_H, dst_w, dst_h)
}

#[inline(always)]
fn expand_rgb565_to_u32(c: u16, r_shift: u32, g_shift: u32, b_shift: u32, alpha_mask: u32) -> u32 {
    let r5 = ((c >> 11) & 0x1F) as u32;
    let g6 = ((c >> 5) & 0x3F) as u32;
    let b5 = (c & 0x1F) as u32;
    let r = (r5 << 3) | (r5 >> 2);
    let g = (g6 << 2) | (g6 >> 4);
    let b = (b5 << 3) | (b5 >> 2);
    alpha_mask | (r << r_shift) | (g << g_shift) | (b << b_shift)
}

/// Fast zero-allocation-per-frame scaler/converter from an arbitrary `(src_w, src_h)` RGB565
/// framebuffer to any 16-bit, 24-bit, or 32-bit linear framebuffer.
///
/// Uses a precomputed 64K RGB565->u32 LUT and an L1-cached staging row buffer so each
/// source scanline is converted once and burst-copied (`memcpy`) across repeated
/// destination rows — critical for maintaining 60 FPS on Raspberry Pi CM0 LPDDR2.
pub struct Blitter {
    src_w: usize,
    src_h: usize,
    dst_w: usize,
    dst_h: usize,
    stride: usize,
    layout: PixelLayout,
    out_w: usize,
    out_h: usize,
    x_off: usize,
    y_off: usize,
    origin_byte_offset: usize,
    int_scale: Option<usize>,
    x_lut: Vec<u16>,
    y_lut: Vec<u16>,
    lut32: Option<Box<[u32; 65536]>>,
    row16: Vec<u16>,
    row24: Vec<u8>,
    row32: Vec<u32>,
}

impl Blitter {
    pub fn new(dst_w: usize, dst_h: usize, stride: usize, layout: PixelLayout) -> Self {
        Self::with_source_and_origin(SCREEN_W, SCREEN_H, dst_w, dst_h, stride, layout, 0)
    }

    pub fn new_for_source(
        src_w: usize,
        src_h: usize,
        dst_w: usize,
        dst_h: usize,
        stride: usize,
        layout: PixelLayout,
    ) -> Self {
        Self::with_source_and_origin(src_w, src_h, dst_w, dst_h, stride, layout, 0)
    }

    pub fn with_origin(
        dst_w: usize,
        dst_h: usize,
        stride: usize,
        layout: PixelLayout,
        origin_byte_offset: usize,
    ) -> Self {
        Self::with_source_and_origin(
            SCREEN_W,
            SCREEN_H,
            dst_w,
            dst_h,
            stride,
            layout,
            origin_byte_offset,
        )
    }

    pub fn with_source_and_origin(
        src_w: usize,
        src_h: usize,
        dst_w: usize,
        dst_h: usize,
        stride: usize,
        layout: PixelLayout,
        origin_byte_offset: usize,
    ) -> Self {
        let lut32 = match layout {
            PixelLayout::Rgb32 {
                r_shift,
                g_shift,
                b_shift,
                alpha_mask,
            } => {
                let mut table = vec![0u32; 65536].into_boxed_slice();
                for (i, slot) in table.iter_mut().enumerate() {
                    *slot = expand_rgb565_to_u32(i as u16, r_shift, g_shift, b_shift, alpha_mask);
                }
                let boxed: Box<[u32; 65536]> = table.try_into().expect("65536-entry LUT");
                Some(boxed)
            }
            _ => None,
        };

        let mut blitter = Self {
            src_w: 0,
            src_h: 0,
            dst_w,
            dst_h,
            stride,
            layout,
            out_w: 0,
            out_h: 0,
            x_off: 0,
            y_off: 0,
            origin_byte_offset,
            int_scale: None,
            x_lut: Vec::new(),
            y_lut: Vec::new(),
            lut32,
            row16: Vec::new(),
            row24: Vec::new(),
            row32: Vec::new(),
        };
        blitter.ensure_source(src_w, src_h);
        blitter
    }

    /// Reconfigure scaling LUTs if the source framebuffer resolution changed.
    /// Returns `true` if geometry changed (callers can clear stale letterbox borders once).
    pub fn ensure_source(&mut self, src_w: usize, src_h: usize) -> bool {
        if self.src_w == src_w && self.src_h == src_h {
            return false;
        }
        self.src_w = src_w;
        self.src_h = src_h;

        let (out_w, out_h, x_off, y_off) = compute_fit_rect_for(src_w, src_h, self.dst_w, self.dst_h);
        self.out_w = out_w;
        self.out_h = out_h;
        self.x_off = x_off;
        self.y_off = y_off;

        self.int_scale = if src_w > 0
            && src_h > 0
            && out_w >= src_w
            && out_h >= src_h
            && out_w % src_w == 0
            && out_h % src_h == 0
            && out_w / src_w == out_h / src_h
        {
            Some(out_w / src_w)
        } else {
            None
        };

        self.x_lut.clear();
        self.x_lut.reserve(out_w);
        for x in 0..out_w {
            let sx = if out_w < src_w && out_w > 1 {
                (x * (src_w - 1)) / (out_w - 1)
            } else {
                (x * src_w) / out_w.max(1)
            };
            self.x_lut.push(sx.min(src_w.saturating_sub(1)) as u16);
        }
        self.y_lut.clear();
        self.y_lut.reserve(out_h);
        for y in 0..out_h {
            let sy = if out_h < src_h && out_h > 1 {
                (y * (src_h - 1)) / (out_h - 1)
            } else {
                (y * src_h) / out_h.max(1)
            };
            self.y_lut.push(sy.min(src_h.saturating_sub(1)) as u16);
        }

        match self.layout {
            PixelLayout::Rgb565 => self.row16.resize(out_w, 0),
            PixelLayout::Rgb24 { .. } => self.row24.resize(out_w * 3, 0),
            PixelLayout::Rgb32 { .. } => self.row32.resize(out_w, 0),
        }
        true
    }

    pub fn blit_rgb565(
        &mut self,
        rgb565: &[u16],
        src_w: usize,
        src_h: usize,
        dst_ptr: *mut u8,
        dst_len: usize,
    ) {
        if src_w == 0 || src_h == 0 || rgb565.len() < src_w * src_h || dst_ptr.is_null() {
            return;
        }
        self.ensure_source(src_w, src_h);
        if self.out_w == 0 || self.out_h == 0 {
            return;
        }
        match self.layout {
            PixelLayout::Rgb565 => self.blit_16(rgb565, dst_ptr, dst_len),
            PixelLayout::Rgb24 {
                r_byte,
                g_byte,
                b_byte,
            } => self.blit_24(rgb565, dst_ptr, dst_len, r_byte, g_byte, b_byte),
            PixelLayout::Rgb32 { .. } => self.blit_32(rgb565, dst_ptr, dst_len),
        }
    }

    fn blit_16(&mut self, rgb565: &[u16], dst_ptr: *mut u8, dst_len: usize) {
        // Ultra-fast 1:1 path for CardputerZero 320x170 ST7789v3 LCD and 1:1 Wayland RGB565.
        if self.src_w == self.dst_w
            && self.src_h == self.dst_h
            && self.stride == self.dst_w * 2
            && self.x_off == 0
            && self.y_off == 0
        {
            let bytes = self.src_w * self.src_h * 2;
            if self.origin_byte_offset + bytes <= dst_len {
                unsafe {
                    ptr::copy_nonoverlapping(
                        rgb565.as_ptr() as *const u8,
                        dst_ptr.add(self.origin_byte_offset),
                        bytes,
                    );
                }
            }
            return;
        }

        let src_w = self.src_w;
        let row_bytes = self.out_w * 2;
        let x_bytes = self.x_off * 2;
        let mut last_sy = usize::MAX;

        for dy in 0..self.out_h {
            let sy = self.y_lut[dy] as usize;
            if sy != last_sy {
                let src_row = &rgb565[sy * src_w..(sy + 1) * src_w];
                match self.int_scale {
                    Some(1) => self.row16.copy_from_slice(src_row),
                    Some(scale) => {
                        let mut dx = 0;
                        for &px in src_row {
                            self.row16[dx..dx + scale].fill(px);
                            dx += scale;
                        }
                    }
                    None => {
                        for (dx, &sx) in self.x_lut.iter().enumerate() {
                            self.row16[dx] = src_row[sx as usize];
                        }
                    }
                }
                last_sy = sy;
            }
            let off = self.origin_byte_offset + (self.y_off + dy) * self.stride + x_bytes;
            if off + row_bytes <= dst_len {
                unsafe {
                    ptr::copy_nonoverlapping(
                        self.row16.as_ptr() as *const u8,
                        dst_ptr.add(off),
                        row_bytes,
                    );
                }
            }
        }
    }

    fn blit_32(&mut self, rgb565: &[u16], dst_ptr: *mut u8, dst_len: usize) {
        let Some(ref lut) = self.lut32 else {
            return;
        };
        let src_w = self.src_w;
        let row_bytes = self.out_w * 4;
        let x_bytes = self.x_off * 4;
        let mut last_sy = usize::MAX;

        for dy in 0..self.out_h {
            let sy = self.y_lut[dy] as usize;
            if sy != last_sy {
                let src_row = &rgb565[sy * src_w..(sy + 1) * src_w];
                match self.int_scale {
                    Some(1) => {
                        for (dst, &c) in self.row32.iter_mut().zip(src_row.iter()) {
                            *dst = lut[c as usize];
                        }
                    }
                    Some(scale) => {
                        let mut dx = 0;
                        for &c in src_row {
                            let px = lut[c as usize];
                            self.row32[dx..dx + scale].fill(px);
                            dx += scale;
                        }
                    }
                    None => {
                        for (dx, &sx) in self.x_lut.iter().enumerate() {
                            self.row32[dx] = lut[src_row[sx as usize] as usize];
                        }
                    }
                }
                last_sy = sy;
            }
            let off = self.origin_byte_offset + (self.y_off + dy) * self.stride + x_bytes;
            if off + row_bytes <= dst_len {
                unsafe {
                    ptr::copy_nonoverlapping(
                        self.row32.as_ptr() as *const u8,
                        dst_ptr.add(off),
                        row_bytes,
                    );
                }
            }
        }
    }

    fn blit_24(
        &mut self,
        rgb565: &[u16],
        dst_ptr: *mut u8,
        dst_len: usize,
        r_byte: usize,
        g_byte: usize,
        b_byte: usize,
    ) {
        let src_w = self.src_w;
        let row_bytes = self.out_w * 3;
        let x_bytes = self.x_off * 3;
        let mut last_sy = usize::MAX;

        for dy in 0..self.out_h {
            let sy = self.y_lut[dy] as usize;
            if sy != last_sy {
                let src_row = &rgb565[sy * src_w..(sy + 1) * src_w];
                for (dx, &sx) in self.x_lut.iter().enumerate() {
                    let c = src_row[sx as usize];
                    let r5 = ((c >> 11) & 0x1F) as u8;
                    let g6 = ((c >> 5) & 0x3F) as u8;
                    let b5 = (c & 0x1F) as u8;
                    let base = dx * 3;
                    self.row24[base + r_byte] = (r5 << 3) | (r5 >> 2);
                    self.row24[base + g_byte] = (g6 << 2) | (g6 >> 4);
                    self.row24[base + b_byte] = (b5 << 3) | (b5 >> 2);
                }
                last_sy = sy;
            }
            let off = self.origin_byte_offset + (self.y_off + dy) * self.stride + x_bytes;
            if off + row_bytes <= dst_len {
                unsafe {
                    ptr::copy_nonoverlapping(self.row24.as_ptr(), dst_ptr.add(off), row_bytes);
                }
            }
        }
    }
}

pub struct FbDisplay {
    fd: i32,
    ptr: *mut u8,
    len: usize,
    pub width: usize,
    pub height: usize,
    #[allow(dead_code)]
    pub bpp: usize,
    #[allow(dead_code)]
    pub device_path: String,
    blitter: Blitter,
}

impl FbDisplay {
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
        let width = vinfo.xres as usize;
        let height = vinfo.yres as usize;
        let bpp = vinfo.bits_per_pixel as usize;
        let stride = finfo.line_length as usize;
        if len == 0 || width == 0 || height == 0 || stride == 0 {
            unsafe { close(fd) };
            return Err(format!("fbdev invalid geometry on {path}"));
        }

        let Some(layout) = PixelLayout::from_fb_var(&vinfo) else {
            unsafe { close(fd) };
            return Err(format!("unsupported fbdev bpp {bpp} on {path}"));
        };

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
        if mapped == (!0usize as *mut std::ffi::c_void) || mapped.is_null() {
            unsafe { close(fd) };
            return Err(format!("mmap failed on {path}"));
        }

        let bytes_per_px = (bpp / 8).max(1);
        let origin = (vinfo.yoffset as usize) * stride + (vinfo.xoffset as usize) * bytes_per_px;
        let blitter = Blitter::with_origin(width, height, stride, layout, origin);

        let mut disp = Self {
            fd,
            ptr: mapped as *mut u8,
            len,
            width,
            height,
            bpp,
            device_path: path.to_string(),
            blitter,
        };
        // Clear letterbox borders on larger HDMI framebuffers.
        if width > SCREEN_W || height > SCREEN_H {
            disp.clear_black();
        }
        Ok(disp)
    }

    pub fn present(&mut self, rgb565: &[u16], src_w: usize, src_h: usize) {
        if self.blitter.ensure_source(src_w, src_h) {
            self.clear_black();
        }
        self.blitter
            .blit_rgb565(rgb565, src_w, src_h, self.ptr, self.len);
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

/// Zero-dependency sysfs backlight controller for CardputerZero's 1.9" ST7789v3 PWM LCD
/// (`/sys/class/backlight/backlight/{brightness,max_brightness}`).
///
/// Automatically saves the user's pre-HDMI brightness level, dims or turns off the backlight
/// while HDMI output is active, and unconditionally restores the original brightness on HDMI
/// unplug, mode switch, or `Drop` (app exit / SIGINT / SIGTERM).
pub struct LcdBacklight {
    brightness_path: PathBuf,
    max_brightness: u32,
    saved_brightness: Option<u32>,
    applied_mode: Option<HdmiLcdBacklight>,
}

impl LcdBacklight {
    pub fn detect() -> Option<Self> {
        if let Ok(dir) = std::env::var("STICKZERO_BACKLIGHT_DIR") {
            let dir = dir.trim();
            if !dir.is_empty() {
                return Self::open_dir(Path::new(dir));
            }
        }
        // CardputerZero stock driver (`pwm_bl_m5stack.ko`) exposes `/sys/class/backlight/backlight`.
        let preferred = Path::new("/sys/class/backlight/backlight");
        if let Some(bl) = Self::open_dir(preferred) {
            return Some(bl);
        }
        let entries = fs::read_dir("/sys/class/backlight").ok()?;
        let mut dirs: Vec<PathBuf> = entries
            .flatten()
            .map(|e| e.path())
            .filter(|p| p.is_dir())
            .collect();
        dirs.sort();
        for dir in dirs {
            if let Some(bl) = Self::open_dir(&dir) {
                return Some(bl);
            }
        }
        None
    }

    pub fn open_dir(dir: &Path) -> Option<Self> {
        let brightness_path = dir.join("brightness");
        let max_path = dir.join("max_brightness");
        if !brightness_path.exists() || !max_path.exists() {
            return None;
        }
        let max_brightness = read_u32_file(&max_path)?.max(1);
        Some(Self {
            brightness_path,
            max_brightness,
            saved_brightness: None,
            applied_mode: None,
        })
    }

    /// Synchronize the LCD backlight with the current HDMI active state and user policy.
    /// Does zero sysfs I/O on steady-state frames when state has not changed.
    pub fn sync(&mut self, hdmi_active: bool, policy: HdmiLcdBacklight) {
        if !hdmi_active || policy == HdmiLcdBacklight::KeepOn {
            self.restore();
            return;
        }
        if self.applied_mode == Some(policy) {
            return;
        }
        if self.saved_brightness.is_none() {
            if let Some(cur) = read_u32_file(&self.brightness_path) {
                self.saved_brightness = Some(cur.min(self.max_brightness));
            }
        }
        let target = match policy {
            HdmiLcdBacklight::Off => 0,
            HdmiLcdBacklight::Dim => {
                let dim = ((self.max_brightness.saturating_mul(12)) / 100).max(1);
                match self.saved_brightness {
                    Some(orig) => dim.min(orig),
                    None => dim,
                }
            }
            HdmiLcdBacklight::KeepOn => return,
        };
        if write_u32_file(&self.brightness_path, target) {
            self.applied_mode = Some(policy);
        }
    }

    pub fn restore(&mut self) {
        if let Some(orig) = self.saved_brightness.take() {
            let _ = write_u32_file(&self.brightness_path, orig.min(self.max_brightness));
        }
        self.applied_mode = None;
    }
}

impl Drop for LcdBacklight {
    fn drop(&mut self) {
        self.restore();
    }
}

fn read_u32_file(path: &Path) -> Option<u32> {
    fs::read_to_string(path).ok()?.trim().parse::<u32>().ok()
}

fn write_u32_file(path: &Path, val: u32) -> bool {
    fs::write(path, val.to_string()).is_ok()
}

pub enum HdmiOutput {
    Wayland(WaylandDisplay),
    Drm(DrmDisplay),
    Fb(FbDisplay),
}

impl HdmiOutput {
    pub fn present(&mut self, rgb565: &[u16], src_w: usize, src_h: usize) -> bool {
        match self {
            Self::Wayland(w) => w.present(rgb565, src_w, src_h),
            Self::Drm(d) => d.present(rgb565, src_w, src_h),
            Self::Fb(fb) => {
                fb.present(rgb565, src_w, src_h);
                true
            }
        }
    }

    pub fn is_alive(&self) -> bool {
        match self {
            Self::Wayland(w) => w.alive,
            Self::Drm(d) => d.alive,
            Self::Fb(_) => true,
        }
    }

    pub fn dimensions(&self) -> (usize, usize) {
        match self {
            Self::Wayland(w) => (w.width, w.height),
            Self::Drm(d) => (d.width, d.height),
            Self::Fb(fb) => (fb.width, fb.height),
        }
    }

    pub fn backend_tag(&self) -> &'static str {
        match self {
            Self::Wayland(_) => "WL",
            Self::Drm(_) => "DRM",
            Self::Fb(_) => "FB",
        }
    }
}

pub struct DisplayManager {
    lcd: Option<FbDisplay>,
    lcd_canonical: Option<PathBuf>,
    hdmi: Option<HdmiOutput>,
    backlight: Option<LcdBacklight>,
    explicit_fbdev: bool,
    lcd_showing_hdmi_notice: bool,
    last_lcd_backlight_mode: HdmiLcdBacklight,
    last_hotplug_check: Instant,
    last_hdmi_connected: bool,
}

impl DisplayManager {
    pub fn open(
        output_mode: DisplayOutput,
        lcd_backlight_mode: HdmiLcdBacklight,
    ) -> Result<Self, String> {
        // Explicit STICKZERO_FBDEV override retains exact legacy single-display behavior.
        if let Ok(dev) = std::env::var("STICKZERO_FBDEV") {
            let dev = dev.trim();
            if !dev.is_empty() {
                let fb = FbDisplay::open(dev)?;
                let canon = canonicalize_path(dev);
                return Ok(Self {
                    lcd: Some(fb),
                    lcd_canonical: Some(canon),
                    hdmi: None,
                    backlight: None,
                    explicit_fbdev: true,
                    lcd_showing_hdmi_notice: false,
                    last_lcd_backlight_mode: lcd_backlight_mode,
                    last_hotplug_check: Instant::now(),
                    last_hdmi_connected: false,
                });
            }
        }

        let lcd_path = detect_lcd_fb();
        let lcd = lcd_path.as_deref().and_then(|p| FbDisplay::open(p).ok());
        let lcd_canonical = lcd_path.as_deref().map(canonicalize_path);

        let connected = is_hdmi_connected(lcd_canonical.as_deref());
        let hdmi = if output_mode != DisplayOutput::LcdOnly && connected {
            open_hdmi_backend(lcd_canonical.as_deref()).ok()
        } else {
            None
        };

        if lcd.is_none() && hdmi.is_none() {
            // Last-ditch fallback to /dev/fb0 (e.g. non-CardputerZero Linux machine without DRM sysfs).
            if let Ok(fb0) = FbDisplay::open("/dev/fb0") {
                return Ok(Self {
                    lcd: Some(fb0),
                    lcd_canonical: Some(canonicalize_path("/dev/fb0")),
                    hdmi: None,
                    backlight: None,
                    explicit_fbdev: false,
                    lcd_showing_hdmi_notice: false,
                    last_lcd_backlight_mode: lcd_backlight_mode,
                    last_hotplug_check: Instant::now(),
                    last_hdmi_connected: false,
                });
            }
            return Err("no LCD framebuffer or HDMI display found".to_string());
        }

        let mut backlight = if lcd.is_some() {
            LcdBacklight::detect()
        } else {
            None
        };
        if let Some(ref mut bl) = backlight {
            bl.sync(hdmi.is_some(), lcd_backlight_mode);
        }

        Ok(Self {
            lcd,
            lcd_canonical,
            hdmi,
            backlight,
            explicit_fbdev: false,
            lcd_showing_hdmi_notice: false,
            last_lcd_backlight_mode: lcd_backlight_mode,
            last_hotplug_check: Instant::now(),
            last_hdmi_connected: connected,
        })
    }

    #[allow(dead_code)]
    pub fn has_lcd(&self) -> bool {
        self.lcd.is_some()
    }

    #[allow(dead_code)]
    pub fn hdmi_active(&self) -> bool {
        self.hdmi.is_some()
    }

    /// Active software render resolution: uses `hdmi_res` whenever HDMI is active,
    /// otherwise CardputerZero's native `320x170`.
    pub fn desired_render_size(&self, hdmi_res: HdmiResolution) -> (usize, usize) {
        if self.hdmi.is_some() {
            hdmi_res.dimensions()
        } else {
            (SCREEN_W, SCREEN_H)
        }
    }

    /// Short status string for Settings / LCD notice (e.g. `"1280x720 DRM"`, `"1920x1080 WL"`, `"Off"`).
    pub fn hdmi_status_text(&self) -> String {
        if let Some(ref h) = self.hdmi {
            let (w, height) = h.dimensions();
            format!("{w}x{height} {}", h.backend_tag())
        } else if self.last_hdmi_connected {
            "Connected (Off)".to_string()
        } else {
            "No HDMI".to_string()
        }
    }

    /// Compact tag for the Main Menu header / Settings row (e.g. `"720p DRM"`, `"1080p WL"`).
    pub fn hdmi_compact_tag(&self) -> Option<String> {
        let h = self.hdmi.as_ref()?;
        let (_, height) = h.dimensions();
        Some(format!("{height}p {}", h.backend_tag()))
    }

    fn sync_backlight(&mut self, lcd_backlight_mode: HdmiLcdBacklight) {
        if let Some(ref mut bl) = self.backlight {
            bl.sync(self.hdmi.is_some(), lcd_backlight_mode);
        }
    }

    /// Immediately apply new display/backlight settings or check for HDMI hotplug/unplug.
    /// Returns `true` if HDMI active state changed.
    pub fn sync_hotplug(
        &mut self,
        output_mode: DisplayOutput,
        lcd_backlight_mode: HdmiLcdBacklight,
        force: bool,
    ) -> bool {
        if self.explicit_fbdev {
            return false;
        }
        if lcd_backlight_mode != self.last_lcd_backlight_mode {
            self.last_lcd_backlight_mode = lcd_backlight_mode;
            self.lcd_showing_hdmi_notice = false;
            self.sync_backlight(lcd_backlight_mode);
        }
        if !force && self.last_hotplug_check.elapsed() < HOTPLUG_INTERVAL {
            return false;
        }
        self.last_hotplug_check = Instant::now();

        let was_active = self.hdmi.is_some();

        if output_mode == DisplayOutput::LcdOnly {
            if self.lcd.is_some() && self.hdmi.is_some() {
                self.hdmi = None;
                self.lcd_showing_hdmi_notice = false;
            }
            self.last_hdmi_connected = is_hdmi_connected(self.lcd_canonical.as_deref());
            self.sync_backlight(lcd_backlight_mode);
            return was_active != self.hdmi.is_some();
        }

        let connected = is_hdmi_connected(self.lcd_canonical.as_deref());
        self.last_hdmi_connected = connected;

        if !connected {
            if self.hdmi.is_some() && self.lcd.is_some() {
                self.hdmi = None;
                self.lcd_showing_hdmi_notice = false;
            }
            self.sync_backlight(lcd_backlight_mode);
            return was_active != self.hdmi.is_some();
        }

        // Drop dead HDMI output (e.g. compositor closed window or DRM lost master) so we can re-probe.
        if let Some(ref h) = self.hdmi {
            if !h.is_alive() {
                self.hdmi = None;
                self.lcd_showing_hdmi_notice = false;
            }
        }

        if self.hdmi.is_none() {
            if let Ok(h) = open_hdmi_backend(self.lcd_canonical.as_deref()) {
                self.hdmi = Some(h);
                self.lcd_showing_hdmi_notice = false;
            }
        }

        if output_mode != DisplayOutput::HdmiOnly && lcd_backlight_mode != HdmiLcdBacklight::Off {
            self.lcd_showing_hdmi_notice = false;
        }

        self.sync_backlight(lcd_backlight_mode);
        was_active != self.hdmi.is_some()
    }

    /// Returns true when the built-in LCD needs a one-shot static HDMI status screen drawn.
    pub fn needs_hdmi_notice(
        &self,
        output_mode: DisplayOutput,
        lcd_backlight_mode: HdmiLcdBacklight,
    ) -> bool {
        self.lcd.is_some()
            && self.hdmi.is_some()
            && output_mode == DisplayOutput::HdmiOnly
            && lcd_backlight_mode != HdmiLcdBacklight::Off
            && !self.lcd_showing_hdmi_notice
    }

    /// Present the rendered RGB565 frame (`src_w` x `src_h`) to active displays.
    ///
    /// - When `lcd_backlight_mode == Off` and HDMI is active, clears the built-in LCD black once
    ///   and skips 60 Hz SPI blits for maximum battery savings (even in `Auto` mode).
    /// - In `DisplayOutput::HdmiOnly` mode (with backlight Dim/KeepOn), renders the static
    ///   `320x170` `hdmi_notice_fb` to the built-in LCD once and skips 60 Hz SPI blits.
    pub fn present(
        &mut self,
        rgb565: &[u16],
        src_w: usize,
        src_h: usize,
        output_mode: DisplayOutput,
        lcd_backlight_mode: HdmiLcdBacklight,
        hdmi_notice_fb: Option<&[u16]>,
    ) {
        if let Some(ref mut h) = self.hdmi {
            if !h.present(rgb565, src_w, src_h) {
                self.hdmi = None;
                self.lcd_showing_hdmi_notice = false;
                self.sync_backlight(lcd_backlight_mode);
            }
        }

        if let Some(ref mut lcd) = self.lcd {
            let hdmi_on = self.hdmi.is_some();
            if hdmi_on && lcd_backlight_mode == HdmiLcdBacklight::Off {
                if !self.lcd_showing_hdmi_notice {
                    lcd.clear_black();
                    self.lcd_showing_hdmi_notice = true;
                }
            } else if hdmi_on && output_mode == DisplayOutput::HdmiOnly {
                if !self.lcd_showing_hdmi_notice {
                    if let Some(notice) = hdmi_notice_fb {
                        lcd.present(notice, SCREEN_W, SCREEN_H);
                    } else {
                        lcd.clear_black();
                    }
                    self.lcd_showing_hdmi_notice = true;
                }
            } else {
                self.lcd_showing_hdmi_notice = false;
                lcd.present(rgb565, src_w, src_h);
            }
        }
    }
}

fn canonicalize_path(p: &str) -> PathBuf {
    fs::canonicalize(p).unwrap_or_else(|_| PathBuf::from(p))
}

fn open_hdmi_backend(lcd_canonical: Option<&Path>) -> Result<HdmiOutput, String> {
    if let Ok(forced) = std::env::var("STICKZERO_HDMI_BACKEND") {
        match forced.trim().to_ascii_lowercase().as_str() {
            "wayland" | "wl" => return WaylandDisplay::open_hdmi().map(HdmiOutput::Wayland),
            "drm" | "kms" => return DrmDisplay::open_hdmi().map(HdmiOutput::Drm),
            "fb" | "fbdev" => {
                let path = detect_hdmi_fb(lcd_canonical)
                    .ok_or_else(|| "no HDMI framebuffer found".to_string())?;
                return FbDisplay::open(&path).map(HdmiOutput::Fb);
            }
            _ => {}
        }
    }

    // 1. Wayland (`labwc-pi` on stock CardputerZero OS owns /dev/dri/card0 when running)
    match WaylandDisplay::open_hdmi() {
        Ok(wl) => return Ok(HdmiOutput::Wayland(wl)),
        Err(ref e) if e.contains("NOOP") => {
            // Live Wayland compositor is running on /dev/dri/card0 and still bringing up HDMI-A-1;
            // retry on next hotplug tick instead of writing invisibly into vc4drmfb.
            return Err(e.clone());
        }
        Err(_) => {}
    }

    // 2. Direct DRM/KMS (headless / Pi OS Lite / `lightdm` disabled / hotplugged after boot)
    if let Ok(drm) = DrmDisplay::open_hdmi() {
        return Ok(HdmiOutput::Drm(drm));
    }

    // 3. HDMI fbdev (`vc4drmfb`, `BCM2708 FB`, `simpledrm`)
    if let Some(fb_path) = detect_hdmi_fb(lcd_canonical) {
        if let Ok(fb) = FbDisplay::open(&fb_path) {
            return Ok(HdmiOutput::Fb(fb));
        }
    }

    Err("no usable HDMI backend available".to_string())
}

/// Detect CardputerZero's built-in 320x170 ST7789v3 LCD framebuffer path.
pub fn detect_lcd_fb() -> Option<String> {
    for var in ["APPLAUNCH_LINUX_FBDEV_DEVICE", "LV_LINUX_FBDEV_DEVICE"] {
        if let Ok(dev) = std::env::var(var) {
            let dev = dev.trim();
            if !dev.is_empty() && Path::new(dev).exists() {
                return Some(dev.to_string());
            }
        }
    }
    if let Ok(proc_fb) = fs::read_to_string("/proc/fb") {
        if let Some(dev) = parse_lcd_fb_from_proc(&proc_fb) {
            return Some(dev);
        }
    }
    if Path::new("/dev/fb_lcd").exists() {
        return Some("/dev/fb_lcd".to_string());
    }
    // Check sysfs virtual_size for an exact 320x170 framebuffer.
    for idx in 0..4 {
        let vsize_path = format!("/sys/class/graphics/fb{idx}/virtual_size");
        if let Ok(vsize) = fs::read_to_string(&vsize_path) {
            if vsize.trim() == "320,170" {
                return Some(format!("/dev/fb{idx}"));
            }
        }
    }
    None
}

pub fn parse_lcd_fb_from_proc(proc_fb: &str) -> Option<String> {
    for line in proc_fb.lines() {
        let lower = line.to_ascii_lowercase();
        if lower.contains("st7789")
            || lower.contains("fb_st7789v")
            || lower.contains("mipi-dbi")
            || lower.contains("mipi_dbi")
            || lower.contains("fbtft")
        {
            if let Some(num) = line.split_whitespace().next() {
                return Some(format!("/dev/fb{num}"));
            }
        }
    }
    None
}

pub fn parse_hdmi_fb_from_proc(proc_fb: &str, lcd_path: Option<&Path>) -> Option<String> {
    let mut fallback: Option<String> = None;
    for line in proc_fb.lines() {
        let Some(num) = line.split_whitespace().next() else {
            continue;
        };
        let dev = format!("/dev/fb{num}");
        if let Some(lcd) = lcd_path {
            if canonicalize_path(&dev) == lcd || Path::new(&dev) == lcd {
                continue;
            }
        }
        let lower = line.to_ascii_lowercase();
        if lower.contains("st7789") || lower.contains("mipi-dbi") || lower.contains("fbtft") {
            continue;
        }
        if lower.contains("vc4drmfb")
            || lower.contains("bcm2708")
            || lower.contains("drmfb")
            || lower.contains("simpledrm")
        {
            return Some(dev);
        }
        if fallback.is_none() {
            fallback = Some(dev);
        }
    }
    fallback
}

fn detect_hdmi_fb(lcd_canonical: Option<&Path>) -> Option<String> {
    let proc_fb = fs::read_to_string("/proc/fb").ok()?;
    parse_hdmi_fb_from_proc(&proc_fb, lcd_canonical)
}

/// Check physical HDMI connection state via `/sys/class/drm/card*-HDMI-*/status`.
///
/// Falls back to checking for a distinct non-LCD HDMI framebuffer only if no DRM HDMI
/// connector sysfs entries exist (e.g., legacy non-KMS firmware fbdev).
pub fn is_hdmi_connected(lcd_canonical: Option<&Path>) -> bool {
    let mut saw_drm_hdmi_connector = false;
    if let Ok(entries) = fs::read_dir("/sys/class/drm") {
        for entry in entries.flatten() {
            let name = entry.file_name();
            let name_str = name.to_string_lossy();
            if name_str.contains("-HDMI-") {
                saw_drm_hdmi_connector = true;
                let status_path = entry.path().join("status");
                if let Ok(status) = fs::read_to_string(status_path) {
                    if status.trim().eq_ignore_ascii_case("connected") {
                        return true;
                    }
                }
            }
        }
    }
    if saw_drm_hdmi_connector {
        return false;
    }
    detect_hdmi_fb(lcd_canonical).is_some()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_compute_fit_rect_standard_hdmi_resolutions() {
        // 1080p: exact 6x integer scale = 1920x1020 centered with 30px top/bottom letterbox
        assert_eq!(compute_fit_rect(1920, 1080), (1920, 1020, 0, 30));
        // 720p: exact 4x integer scale = 1280x680 centered with 20px top/bottom letterbox
        assert_eq!(compute_fit_rect(1280, 720), (1280, 680, 0, 20));
        // Native LCD: 1:1 exact
        assert_eq!(compute_fit_rect(320, 170), (320, 170, 0, 0));
        // 640x480: exact 2x integer scale = 640x340 centered
        assert_eq!(compute_fit_rect(640, 480), (640, 340, 0, 70));
        // 800x480: 2x is 640x340 (80% < 85%), so uses full aspect fit 800x425
        assert_eq!(compute_fit_rect(800, 480), (800, 425, 0, 27));
        // True 16:9 640x360 -> 720p (2x) and 1080p (3x): zero black bars!
        assert_eq!(compute_fit_rect_for(640, 360, 1280, 720), (1280, 720, 0, 0));
        assert_eq!(compute_fit_rect_for(640, 360, 1920, 1080), (1920, 1080, 0, 0));
        // True 16:9 960x540 -> 1080p (2x) and 720p (aspect fit): zero black bars!
        assert_eq!(compute_fit_rect_for(960, 540, 1920, 1080), (1920, 1080, 0, 0));
        assert_eq!(compute_fit_rect_for(960, 540, 1280, 720), (1280, 720, 0, 0));
        // Downscaling 640x360 -> 320x170 LCD mirror: centered 302x170
        assert_eq!(compute_fit_rect_for(640, 360, 320, 170), (302, 170, 9, 0));
    }

    #[test]
    fn test_proc_fb_lcd_and_hdmi_detection() {
        let proc_fb = "0 vc4drmfb\n1 fb_st7789v\n";
        let lcd = parse_lcd_fb_from_proc(proc_fb).unwrap();
        assert_eq!(lcd, "/dev/fb1");
        let hdmi = parse_hdmi_fb_from_proc(proc_fb, Some(Path::new(&lcd))).unwrap();
        assert_eq!(hdmi, "/dev/fb0");

        let proc_fb_swapped = "0 st7789v_m5stack\n1 vc4drmfb\n";
        let lcd2 = parse_lcd_fb_from_proc(proc_fb_swapped).unwrap();
        assert_eq!(lcd2, "/dev/fb0");
        let hdmi2 = parse_hdmi_fb_from_proc(proc_fb_swapped, Some(Path::new(&lcd2))).unwrap();
        assert_eq!(hdmi2, "/dev/fb1");

        let proc_fb_lcd_only = "0 fb_st7789v\n";
        let lcd3 = parse_lcd_fb_from_proc(proc_fb_lcd_only).unwrap();
        assert!(parse_hdmi_fb_from_proc(proc_fb_lcd_only, Some(Path::new(&lcd3))).is_none());
    }

    #[test]
    fn test_blitter_16bit_scaled_720p_centers_and_scales_cleanly() {
        let mut src = [0u16; SCREEN_W * SCREEN_H];
        // Pure red top-left pixel, pure blue bottom-right pixel
        src[0] = 0xF800;
        src[SCREEN_W * SCREEN_H - 1] = 0x001F;

        let w = 1280usize;
        let h = 720usize;
        let stride = w * 2;
        let mut dst = vec![0u8; stride * h];
        let mut blitter = Blitter::new(w, h, stride, PixelLayout::Rgb565);
        blitter.blit_rgb565(&src, SCREEN_W, SCREEN_H, dst.as_mut_ptr(), dst.len());

        // Letterbox row 0..20 stays black
        let top_px = u16::from_ne_bytes([dst[0], dst[1]]);
        assert_eq!(top_px, 0);

        // First active scaled block at y=20..24, x=0..4 must be 0xF800
        for dy in 20..24 {
            for dx in 0..4 {
                let off = dy * stride + dx * 2;
                let px = u16::from_ne_bytes([dst[off], dst[off + 1]]);
                assert_eq!(px, 0xF800);
            }
        }
        // Last active scaled pixel at y=699, x=1279 must be 0x001F
        let off = 699 * stride + 1279 * 2;
        let last_px = u16::from_ne_bytes([dst[off], dst[off + 1]]);
        assert_eq!(last_px, 0x001F);
        // Bottom letterbox row 700 stays black
        let bot_off = 700 * stride;
        assert_eq!(u16::from_ne_bytes([dst[bot_off], dst[bot_off + 1]]), 0);
    }

    #[test]
    fn test_blitter_dynamic_640x360_upscale_720p_and_downscale_lcd() {
        let (sw, sh) = (640usize, 360usize);
        let mut src = vec![0u16; sw * sh];
        src[0] = 0xF800; // Red top-left
        src[sw * sh - 1] = 0x07E0; // Green bottom-right

        // Upscale 640x360 -> 1280x720 (exact 2x, full frame, no letterbox)
        let (w, h) = (1280usize, 720usize);
        let stride = w * 2;
        let mut dst = vec![0u8; stride * h];
        let mut blitter = Blitter::new(w, h, stride, PixelLayout::Rgb565);
        blitter.blit_rgb565(&src, sw, sh, dst.as_mut_ptr(), dst.len());
        for dy in 0..2 {
            for dx in 0..2 {
                let off = dy * stride + dx * 2;
                assert_eq!(u16::from_ne_bytes([dst[off], dst[off + 1]]), 0xF800);
            }
        }
        let last_off = (h - 1) * stride + (w - 1) * 2;
        assert_eq!(u16::from_ne_bytes([dst[last_off], dst[last_off + 1]]), 0x07E0);

        // Downscale 640x360 -> 320x170 built-in LCD mirror (302x170 centered at x=9)
        let lcd_stride = SCREEN_W * 2;
        let mut lcd_dst = vec![0u8; lcd_stride * SCREEN_H];
        let mut lcd_blitter = Blitter::new(SCREEN_W, SCREEN_H, lcd_stride, PixelLayout::Rgb565);
        lcd_blitter.blit_rgb565(&src, sw, sh, lcd_dst.as_mut_ptr(), lcd_dst.len());
        let first_active = 9 * 2;
        assert_eq!(
            u16::from_ne_bytes([lcd_dst[first_active], lcd_dst[first_active + 1]]),
            0xF800
        );
        let last_active = (SCREEN_H - 1) * lcd_stride + (9 + 301) * 2;
        assert_eq!(
            u16::from_ne_bytes([lcd_dst[last_active], lcd_dst[last_active + 1]]),
            0x07E0
        );
    }

    #[test]
    fn test_blitter_32bit_xrgb8888_and_xbgr8888_channel_order() {
        let mut src = [0u16; SCREEN_W * SCREEN_H];
        src[0] = 0xF800; // Red (r=31, g=0, b=0) -> 0xFF, 0x00, 0x00
        src[1] = 0x07E0; // Green (r=0, g=63, b=0) -> 0x00, 0xFF, 0x00
        src[2] = 0x001F; // Blue (r=0, g=0, b=31) -> 0x00, 0x00, 0xFF

        let stride = SCREEN_W * 4;
        let mut dst = vec![0u8; stride * SCREEN_H];
        let mut blitter = Blitter::new(SCREEN_W, SCREEN_H, stride, PixelLayout::xrgb8888());
        blitter.blit_rgb565(&src, SCREEN_W, SCREEN_H, dst.as_mut_ptr(), dst.len());

        let p0 = u32::from_ne_bytes(dst[0..4].try_into().unwrap());
        let p1 = u32::from_ne_bytes(dst[4..8].try_into().unwrap());
        let p2 = u32::from_ne_bytes(dst[8..12].try_into().unwrap());
        assert_eq!(p0, 0xFFFF_0000);
        assert_eq!(p1, 0xFF00_FF00);
        assert_eq!(p2, 0xFF00_00FF);

        // XBGR8888 (red at bit 0, blue at bit 16)
        let bgr_layout = PixelLayout::Rgb32 {
            r_shift: 0,
            g_shift: 8,
            b_shift: 16,
            alpha_mask: 0xFF00_0000,
        };
        let mut blitter_bgr = Blitter::new(SCREEN_W, SCREEN_H, stride, bgr_layout);
        blitter_bgr.blit_rgb565(&src, SCREEN_W, SCREEN_H, dst.as_mut_ptr(), dst.len());
        let b0 = u32::from_ne_bytes(dst[0..4].try_into().unwrap());
        let b2 = u32::from_ne_bytes(dst[8..12].try_into().unwrap());
        assert_eq!(b0, 0xFF00_00FF);
        assert_eq!(b2, 0xFFFF_0000);
    }

    #[test]
    fn test_blitter_24bit_and_aspect_fit_800x480() {
        let mut src = [0u16; SCREEN_W * SCREEN_H];
        src[0] = 0xF800; // Red
        src[SCREEN_W * SCREEN_H - 1] = 0x07E0; // Green

        let w = 800usize;
        let h = 480usize;
        let stride = w * 3;
        let mut dst = vec![0u8; stride * h];
        let mut blitter = Blitter::new(
            w,
            h,
            stride,
            PixelLayout::Rgb24 {
                r_byte: 2,
                g_byte: 1,
                b_byte: 0,
            },
        );
        blitter.blit_rgb565(&src, SCREEN_W, SCREEN_H, dst.as_mut_ptr(), dst.len());

        // Fit rect for 800x480 is (800, 425, 0, 27)
        assert_eq!(&dst[0..3], &[0, 0, 0]);
        let top_active = 27 * stride;
        assert_eq!(&dst[top_active..top_active + 3], &[0, 0, 255]); // BGR24 red
        let last_active = (27 + 424) * stride + 799 * 3;
        assert_eq!(&dst[last_active..last_active + 3], &[0, 255, 0]); // BGR24 green
    }

    #[test]
    fn test_lcd_backlight_dim_off_and_restore_on_drop() {
        let dir = std::env::temp_dir().join(format!(
            "stickzero-bl-test-{}",
            std::process::id()
        ));
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(&dir).unwrap();
        fs::write(dir.join("max_brightness"), "255\n").unwrap();
        fs::write(dir.join("brightness"), "180\n").unwrap();

        {
            let mut bl = LcdBacklight::open_dir(&dir).unwrap();
            // HDMI active + Dim -> ~12% of 255 = 30
            bl.sync(true, HdmiLcdBacklight::Dim);
            assert_eq!(read_u32_file(&dir.join("brightness")), Some(30));

            // Switch to Off -> 0
            bl.sync(true, HdmiLcdBacklight::Off);
            assert_eq!(read_u32_file(&dir.join("brightness")), Some(0));

            // Unplug HDMI -> restores exact saved 180
            bl.sync(false, HdmiLcdBacklight::Off);
            assert_eq!(read_u32_file(&dir.join("brightness")), Some(180));

            // User already dimmer than 12% (e.g. 10) -> Dim must not brighten it
            fs::write(dir.join("brightness"), "10\n").unwrap();
            bl.sync(true, HdmiLcdBacklight::Dim);
            assert_eq!(read_u32_file(&dir.join("brightness")), Some(10));
            // Drop while dimmed must restore 10 cleanly
        }
        assert_eq!(read_u32_file(&dir.join("brightness")), Some(10));

        // Verify Drop restores from dimmed high brightness too
        fs::write(dir.join("brightness"), "200\n").unwrap();
        {
            let mut bl = LcdBacklight::open_dir(&dir).unwrap();
            bl.sync(true, HdmiLcdBacklight::Off);
            assert_eq!(read_u32_file(&dir.join("brightness")), Some(0));
        }
        assert_eq!(read_u32_file(&dir.join("brightness")), Some(200));

        let _ = fs::remove_dir_all(&dir);
    }
}
