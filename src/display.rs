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

use crate::config::DisplayOutput;
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
/// mapping our 320x170 framebuffer onto an arbitrary `(dst_w, dst_h)` display.
///
/// Prefers crisp exact integer scaling (e.g. 4x = 1280x680 on 720p, 6x = 1920x1020 on 1080p)
/// whenever an integer multiple fills at least 85% of either axis; otherwise uses full
/// aspect-preserving nearest-neighbor fit.
pub fn compute_fit_rect(dst_w: usize, dst_h: usize) -> (usize, usize, usize, usize) {
    if dst_w == 0 || dst_h == 0 {
        return (0, 0, 0, 0);
    }
    if dst_w <= SCREEN_W && dst_h <= SCREEN_H {
        let out_w = dst_w.min(SCREEN_W);
        let out_h = dst_h.min(SCREEN_H);
        return (
            out_w,
            out_h,
            (dst_w.saturating_sub(out_w)) / 2,
            (dst_h.saturating_sub(out_h)) / 2,
        );
    }

    let int_scale = (dst_w / SCREEN_W).min(dst_h / SCREEN_H);
    let (out_w, out_h) = if int_scale >= 1
        && (SCREEN_W * int_scale * 100 >= dst_w * 85 || SCREEN_H * int_scale * 100 >= dst_h * 85)
    {
        (SCREEN_W * int_scale, SCREEN_H * int_scale)
    } else if dst_w * SCREEN_H <= dst_h * SCREEN_W {
        (dst_w, ((dst_w * SCREEN_H) / SCREEN_W).max(1).min(dst_h))
    } else {
        (((dst_h * SCREEN_W) / SCREEN_H).max(1).min(dst_w), dst_h)
    };

    let x_off = (dst_w.saturating_sub(out_w)) / 2;
    let y_off = (dst_h.saturating_sub(out_h)) / 2;
    (out_w, out_h, x_off, y_off)
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

/// Fast zero-allocation-per-frame scaler/converter from 320x170 RGB565 to any
/// 16-bit, 24-bit, or 32-bit linear framebuffer.
///
/// Uses a precomputed 64K RGB565->u32 LUT and an L1-cached staging row buffer so each of
/// the 170 source scanlines is converted once and burst-copied (`memcpy`) across repeated
/// destination rows — critical for maintaining 60 FPS on Raspberry Pi CM0 LPDDR2.
pub struct Blitter {
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
        Self::with_origin(dst_w, dst_h, stride, layout, 0)
    }

    pub fn with_origin(
        dst_w: usize,
        dst_h: usize,
        stride: usize,
        layout: PixelLayout,
        origin_byte_offset: usize,
    ) -> Self {
        let (out_w, out_h, x_off, y_off) = compute_fit_rect(dst_w, dst_h);
        let int_scale = if out_w > 0
            && out_h > 0
            && out_w % SCREEN_W == 0
            && out_h % SCREEN_H == 0
            && out_w / SCREEN_W == out_h / SCREEN_H
        {
            Some(out_w / SCREEN_W)
        } else {
            None
        };

        let x_lut: Vec<u16> = (0..out_w)
            .map(|x| ((x * SCREEN_W) / out_w.max(1)).min(SCREEN_W - 1) as u16)
            .collect();
        let y_lut: Vec<u16> = (0..out_h)
            .map(|y| ((y * SCREEN_H) / out_h.max(1)).min(SCREEN_H - 1) as u16)
            .collect();

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

        let row16 = if matches!(layout, PixelLayout::Rgb565) {
            vec![0u16; out_w]
        } else {
            Vec::new()
        };
        let row24 = if matches!(layout, PixelLayout::Rgb24 { .. }) {
            vec![0u8; out_w * 3]
        } else {
            Vec::new()
        };
        let row32 = if matches!(layout, PixelLayout::Rgb32 { .. }) {
            vec![0u32; out_w]
        } else {
            Vec::new()
        };

        Self {
            dst_w,
            dst_h,
            stride,
            layout,
            out_w,
            out_h,
            x_off,
            y_off,
            origin_byte_offset,
            int_scale,
            x_lut,
            y_lut,
            lut32,
            row16,
            row24,
            row32,
        }
    }

    pub fn blit_rgb565(&mut self, rgb565: &[u16; SCREEN_W * SCREEN_H], dst_ptr: *mut u8, dst_len: usize) {
        if self.out_w == 0 || self.out_h == 0 || dst_ptr.is_null() {
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

    fn blit_16(&mut self, rgb565: &[u16; SCREEN_W * SCREEN_H], dst_ptr: *mut u8, dst_len: usize) {
        // Ultra-fast 1:1 path for CardputerZero 320x170 ST7789v3 LCD and 320x170 Wayland RGB565.
        if self.dst_w == SCREEN_W
            && self.dst_h == SCREEN_H
            && self.stride == SCREEN_W * 2
            && self.x_off == 0
            && self.y_off == 0
        {
            let bytes = SCREEN_W * SCREEN_H * 2;
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

        let row_bytes = self.out_w * 2;
        let x_bytes = self.x_off * 2;
        let mut last_sy = usize::MAX;

        for dy in 0..self.out_h {
            let sy = self.y_lut[dy] as usize;
            if sy != last_sy {
                let src_row = &rgb565[sy * SCREEN_W..(sy + 1) * SCREEN_W];
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

    fn blit_32(&mut self, rgb565: &[u16; SCREEN_W * SCREEN_H], dst_ptr: *mut u8, dst_len: usize) {
        let Some(ref lut) = self.lut32 else {
            return;
        };
        let row_bytes = self.out_w * 4;
        let x_bytes = self.x_off * 4;
        let mut last_sy = usize::MAX;

        for dy in 0..self.out_h {
            let sy = self.y_lut[dy] as usize;
            if sy != last_sy {
                let src_row = &rgb565[sy * SCREEN_W..(sy + 1) * SCREEN_W];
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
        rgb565: &[u16; SCREEN_W * SCREEN_H],
        dst_ptr: *mut u8,
        dst_len: usize,
        r_byte: usize,
        g_byte: usize,
        b_byte: usize,
    ) {
        let row_bytes = self.out_w * 3;
        let x_bytes = self.x_off * 3;
        let mut last_sy = usize::MAX;

        for dy in 0..self.out_h {
            let sy = self.y_lut[dy] as usize;
            if sy != last_sy {
                let src_row = &rgb565[sy * SCREEN_W..(sy + 1) * SCREEN_W];
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

    pub fn present(&mut self, rgb565: &[u16; SCREEN_W * SCREEN_H]) {
        self.blitter.blit_rgb565(rgb565, self.ptr, self.len);
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

pub enum HdmiOutput {
    Wayland(WaylandDisplay),
    Drm(DrmDisplay),
    Fb(FbDisplay),
}

impl HdmiOutput {
    pub fn present(&mut self, rgb565: &[u16; SCREEN_W * SCREEN_H]) -> bool {
        match self {
            Self::Wayland(w) => w.present(rgb565),
            Self::Drm(d) => d.present(rgb565),
            Self::Fb(fb) => {
                fb.present(rgb565);
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
    explicit_fbdev: bool,
    lcd_showing_hdmi_notice: bool,
    last_hotplug_check: Instant,
    last_hdmi_connected: bool,
}

impl DisplayManager {
    pub fn open(output_mode: DisplayOutput) -> Result<Self, String> {
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
                    explicit_fbdev: true,
                    lcd_showing_hdmi_notice: false,
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
                    explicit_fbdev: false,
                    lcd_showing_hdmi_notice: false,
                    last_hotplug_check: Instant::now(),
                    last_hdmi_connected: false,
                });
            }
            return Err("no LCD framebuffer or HDMI display found".to_string());
        }

        Ok(Self {
            lcd,
            lcd_canonical,
            hdmi,
            explicit_fbdev: false,
            lcd_showing_hdmi_notice: false,
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

    /// Immediately apply a new `DisplayOutput` setting or check for HDMI hotplug/unplug.
    /// Returns `true` if HDMI active state changed.
    pub fn sync_hotplug(&mut self, output_mode: DisplayOutput, force: bool) -> bool {
        if self.explicit_fbdev {
            return false;
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
            return was_active != self.hdmi.is_some();
        }

        let connected = is_hdmi_connected(self.lcd_canonical.as_deref());
        self.last_hdmi_connected = connected;

        if !connected {
            if self.hdmi.is_some() && self.lcd.is_some() {
                self.hdmi = None;
                self.lcd_showing_hdmi_notice = false;
            }
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

        if output_mode != DisplayOutput::HdmiOnly {
            self.lcd_showing_hdmi_notice = false;
        }

        was_active != self.hdmi.is_some()
    }

    /// Returns true when the built-in LCD needs a one-shot static HDMI status screen drawn.
    pub fn needs_hdmi_notice(&self, output_mode: DisplayOutput) -> bool {
        self.lcd.is_some()
            && output_mode == DisplayOutput::HdmiOnly
            && self.hdmi.is_some()
            && !self.lcd_showing_hdmi_notice
    }

    /// Present the rendered 320x170 RGB565 frame to active displays.
    ///
    /// In `DisplayOutput::HdmiOnly` mode (when HDMI is active and an LCD is also present),
    /// renders the static `hdmi_notice_fb` to the built-in LCD once and skips 60 Hz SPI blits.
    pub fn present(
        &mut self,
        rgb565: &[u16; SCREEN_W * SCREEN_H],
        output_mode: DisplayOutput,
        hdmi_notice_fb: Option<&[u16; SCREEN_W * SCREEN_H]>,
    ) {
        if let Some(ref mut h) = self.hdmi {
            if !h.present(rgb565) {
                self.hdmi = None;
                self.lcd_showing_hdmi_notice = false;
            }
        }

        if let Some(ref mut lcd) = self.lcd {
            if output_mode == DisplayOutput::HdmiOnly && self.hdmi.is_some() {
                if !self.lcd_showing_hdmi_notice {
                    if let Some(notice) = hdmi_notice_fb {
                        lcd.present(notice);
                    } else {
                        lcd.clear_black();
                    }
                    self.lcd_showing_hdmi_notice = true;
                }
            } else {
                self.lcd_showing_hdmi_notice = false;
                lcd.present(rgb565);
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
        blitter.blit_rgb565(&src, dst.as_mut_ptr(), dst.len());

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
    fn test_blitter_32bit_xrgb8888_and_xbgr8888_channel_order() {
        let mut src = [0u16; SCREEN_W * SCREEN_H];
        src[0] = 0xF800; // Red (r=31, g=0, b=0) -> 0xFF, 0x00, 0x00
        src[1] = 0x07E0; // Green (r=0, g=63, b=0) -> 0x00, 0xFF, 0x00
        src[2] = 0x001F; // Blue (r=0, g=0, b=31) -> 0x00, 0x00, 0xFF

        let stride = SCREEN_W * 4;
        let mut dst = vec![0u8; stride * SCREEN_H];
        let mut blitter = Blitter::new(SCREEN_W, SCREEN_H, stride, PixelLayout::xrgb8888());
        blitter.blit_rgb565(&src, dst.as_mut_ptr(), dst.len());

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
        blitter_bgr.blit_rgb565(&src, dst.as_mut_ptr(), dst.len());
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
        blitter.blit_rgb565(&src, dst.as_mut_ptr(), dst.len());

        // Fit rect for 800x480 is (800, 425, 0, 27)
        assert_eq!(&dst[0..3], &[0, 0, 0]);
        let top_active = 27 * stride;
        assert_eq!(&dst[top_active..top_active + 3], &[0, 0, 255]); // BGR24 red
        let last_active = (27 + 424) * stride + 799 * 3;
        assert_eq!(&dst[last_active..last_active + 3], &[0, 255, 0]); // BGR24 green
    }
}
