//! Zero-dependency pure-Rust Wayland client for CardputerZero HDMI output.
//!
//! On stock CardputerZero OS (Debian 13 / Raspberry Pi OS with `labwc-pi`), the Wayland
//! compositor owns `/dev/dri/card0` (HDMI) while `APPLaunch` drives the 1.9" LCD over fbdev.
//! This lightweight client speaks the Wayland wire protocol directly over the Unix domain
//! socket (`wayland-0`) with zero external crates:
//! - Uses `xdg_wm_base` + `xdg_toplevel::set_fullscreen` targeting the HDMI `wl_output`
//! - Uses `wp_viewporter` when available so the VideoCore IV GPU scales the 320x170 surface
//!   to full 1080p/720p HDMI at ~0.1 ms CPU cost and <450 KB RAM (with CPU-scaled fallback)
//! - Non-blocking double-buffered `wl_shm` presentation so 30 Hz TVs never stall the 60 Hz sim

use crate::display::{compute_fit_rect, compute_fit_rect_for, Blitter, PixelLayout, SCREEN_H, SCREEN_W};
use crate::linux_sys::{
    close, ftruncate, getuid, mmap, munmap, open, poll, recvmsg, sendmsg, syscall, unlink,
    Cmsghdr, Iovec, Msghdr, PollFd, MAP_SHARED, MFD_CLOEXEC, MSG_DONTWAIT, MSG_NOSIGNAL,
    O_CLOEXEC, O_CREAT, O_EXCL, O_RDWR, POLLIN, PROT_READ, PROT_WRITE, SCM_RIGHTS, SOL_SOCKET,
    SYS_MEMFD_CREATE,
};
use std::ffi::{c_void, CString};
use std::io::ErrorKind;
use std::os::unix::io::AsRawFd;
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::ptr;
use std::sync::atomic::{AtomicU32, Ordering};
use std::time::{Duration, Instant};

const WL_DISPLAY_ID: u32 = 1;
const WL_SHM_FORMAT_XRGB8888: u32 = 1;
const WL_SHM_FORMAT_RGB565: u32 = 0x3631_4752; // 'R','G','1','6'

static SHM_COUNTER: AtomicU32 = AtomicU32::new(1);

#[repr(C, align(8))]
struct CmsgFd {
    hdr: Cmsghdr,
    fd: i32,
    _pad: i32,
}

#[derive(Clone, Debug)]
struct OutputInfo {
    global_name: u32,
    obj_id: u32,
    name: Option<String>,
    width: i32,
    height: i32,
}

pub struct WaylandDisplay {
    stream: UnixStream,
    next_id: u32,
    wl_shm_id: u32,
    wl_surface_id: u32,
    xdg_wm_base_id: u32,
    xdg_surface_id: u32,
    xdg_toplevel_id: u32,
    wp_viewport_id: Option<u32>,
    hdmi_output_global: Option<u32>,
    use_viewporter: bool,
    use_rgb565: bool,

    shm_fd: i32,
    shm_ptr: *mut u8,
    shm_len: usize,
    pool_id: u32,
    buf_ids: [u32; 2],
    buf_busy: [bool; 2],
    src_w: usize,
    src_h: usize,
    buf_w: usize,
    buf_h: usize,
    buf_stride: usize,
    buf_size: usize,
    blitter: Blitter,

    pub width: usize,
    pub height: usize,
    pending_cfg_w: i32,
    pending_cfg_h: i32,
    configured: bool,
    pub alive: bool,
    rx_buf: Vec<u8>,
}

impl WaylandDisplay {
    /// Discover a running Wayland compositor socket even when launched from `APPLaunch.service`
    /// (which may not inherit `WAYLAND_DISPLAY` or `XDG_RUNTIME_DIR`).
    pub fn find_socket() -> Option<PathBuf> {
        let mut candidates: Vec<PathBuf> = Vec::new();
        let xdg = std::env::var("XDG_RUNTIME_DIR").ok().filter(|s| !s.is_empty());

        if let Ok(wd) = std::env::var("WAYLAND_DISPLAY") {
            let wd = wd.trim();
            if !wd.is_empty() {
                let p = PathBuf::from(wd);
                if p.is_absolute() {
                    candidates.push(p);
                } else if let Some(ref dir) = xdg {
                    candidates.push(Path::new(dir).join(wd));
                }
            }
        }
        if let Some(ref dir) = xdg {
            candidates.push(Path::new(dir).join("wayland-0"));
            candidates.push(Path::new(dir).join("wayland-1"));
        }
        let uid = unsafe { getuid() };
        candidates.push(PathBuf::from(format!("/run/user/{uid}/wayland-0")));
        candidates.push(PathBuf::from(format!("/run/user/{uid}/wayland-1")));
        if uid != 1000 {
            candidates.push(PathBuf::from("/run/user/1000/wayland-0"));
        }

        for path in candidates {
            if path.exists() {
                return Some(path);
            }
        }
        None
    }

    pub fn open_hdmi() -> Result<Self, String> {
        let sock = Self::find_socket().ok_or_else(|| "no Wayland socket found".to_string())?;
        Self::connect_path(&sock, true)
    }

    pub fn connect_path(sock_path: &Path, require_non_noop_output: bool) -> Result<Self, String> {
        let stream = UnixStream::connect(sock_path)
            .map_err(|e| format!("failed to connect Wayland socket {}: {e}", sock_path.display()))?;
        Self::init_stream(stream, require_non_noop_output)
    }

    fn init_stream(stream: UnixStream, require_non_noop_output: bool) -> Result<Self, String> {
        let fd = stream.as_raw_fd();
        let mut next_id = 2u32;
        let registry_id = next_id;
        next_id += 1;
        let sync1_id = next_id;
        next_id += 1;

        let mut out = Vec::with_capacity(256);
        // wl_display::get_registry(registry_id) [opcode 1]
        push_msg(&mut out, WL_DISPLAY_ID, 1, &[registry_id]);
        // wl_display::sync(sync1_id) [opcode 0]
        push_msg(&mut out, WL_DISPLAY_ID, 0, &[sync1_id]);
        send_bytes(fd, &out, None)?;

        let mut rx_buf = Vec::with_capacity(4096);
        let mut compositor_global: Option<(u32, u32)> = None;
        let mut shm_global: Option<(u32, u32)> = None;
        let mut xdg_global: Option<(u32, u32)> = None;
        let mut viewporter_global: Option<(u32, u32)> = None;
        let mut output_globals: Vec<(u32, u32)> = Vec::new();
        let mut sync1_done = false;

        let deadline = Instant::now() + Duration::from_millis(800);
        while !sync1_done {
            if Instant::now() >= deadline {
                return Err("Wayland registry sync timed out".to_string());
            }
            if !recv_more(fd, &mut rx_buf, 100)? {
                continue;
            }
            while let Some((obj, opcode, payload)) = next_event(&mut rx_buf) {
                if obj == WL_DISPLAY_ID && opcode == 0 {
                    return Err("Wayland protocol error during registry discovery".to_string());
                } else if obj == registry_id && opcode == 0 {
                    if let Some((name, iface, ver)) = parse_registry_global(&payload) {
                        match iface.as_str() {
                            "wl_compositor" => compositor_global = Some((name, ver)),
                            "wl_shm" => shm_global = Some((name, ver)),
                            "xdg_wm_base" => xdg_global = Some((name, ver)),
                            "wp_viewporter" => viewporter_global = Some((name, ver)),
                            "wl_output" => output_globals.push((name, ver)),
                            _ => {}
                        }
                    }
                } else if obj == sync1_id && opcode == 0 {
                    sync1_done = true;
                }
            }
        }

        let (comp_name, comp_ver) =
            compositor_global.ok_or_else(|| "Wayland missing wl_compositor".to_string())?;
        let (shm_name, _shm_ver) =
            shm_global.ok_or_else(|| "Wayland missing wl_shm".to_string())?;
        let (xdg_name, xdg_ver) =
            xdg_global.ok_or_else(|| "Wayland missing xdg_wm_base".to_string())?;

        let wl_compositor_id = next_id;
        next_id += 1;
        let wl_shm_id = next_id;
        next_id += 1;
        let xdg_wm_base_id = next_id;
        next_id += 1;

        out.clear();
        push_bind(&mut out, registry_id, comp_name, "wl_compositor", comp_ver.min(4), wl_compositor_id);
        push_bind(&mut out, registry_id, shm_name, "wl_shm", 1, wl_shm_id);
        push_bind(&mut out, registry_id, xdg_name, "xdg_wm_base", xdg_ver.min(2), xdg_wm_base_id);

        let wp_viewporter_id = if let Some((vp_name, _)) = viewporter_global {
            let id = next_id;
            next_id += 1;
            push_bind(&mut out, registry_id, vp_name, "wp_viewporter", 1, id);
            Some(id)
        } else {
            None
        };

        let mut outputs: Vec<OutputInfo> = Vec::new();
        for &(out_name, out_ver) in &output_globals {
            let id = next_id;
            next_id += 1;
            push_bind(&mut out, registry_id, out_name, "wl_output", out_ver.min(4), id);
            outputs.push(OutputInfo {
                global_name: out_name,
                obj_id: id,
                name: None,
                width: 0,
                height: 0,
            });
        }

        let sync2_id = next_id;
        next_id += 1;
        push_msg(&mut out, WL_DISPLAY_ID, 0, &[sync2_id]);
        send_bytes(fd, &out, None)?;

        let mut supports_rgb565 = false;
        let mut sync2_done = false;
        let deadline = Instant::now() + Duration::from_millis(800);
        while !sync2_done {
            if Instant::now() >= deadline {
                return Err("Wayland bind sync timed out".to_string());
            }
            if !recv_more(fd, &mut rx_buf, 100)? {
                continue;
            }
            while let Some((obj, opcode, payload)) = next_event(&mut rx_buf) {
                if obj == WL_DISPLAY_ID && opcode == 0 {
                    return Err("Wayland protocol error during bind".to_string());
                } else if obj == wl_shm_id && opcode == 0 && payload.len() >= 4 {
                    let fmt = u32::from_ne_bytes(payload[0..4].try_into().unwrap());
                    if fmt == WL_SHM_FORMAT_RGB565 {
                        supports_rgb565 = true;
                    }
                } else if obj == xdg_wm_base_id && opcode == 0 && payload.len() >= 4 {
                    let serial = u32::from_ne_bytes(payload[0..4].try_into().unwrap());
                    let mut pong = Vec::with_capacity(12);
                    push_msg(&mut pong, xdg_wm_base_id, 3, &[serial]);
                    let _ = send_bytes(fd, &pong, None);
                } else if obj == sync2_id && opcode == 0 {
                    sync2_done = true;
                } else if let Some(info) = outputs.iter_mut().find(|o| o.obj_id == obj) {
                    if opcode == 1 && payload.len() >= 16 {
                        // wl_output::mode(flags, width, height, refresh)
                        let flags = u32::from_ne_bytes(payload[0..4].try_into().unwrap());
                        let w = i32::from_ne_bytes(payload[4..8].try_into().unwrap());
                        let h = i32::from_ne_bytes(payload[8..12].try_into().unwrap());
                        if (flags & 1) != 0 || info.width == 0 {
                            info.width = w;
                            info.height = h;
                        }
                    } else if opcode == 4 {
                        // wl_output::name(name: string)
                        if let Some(s) = parse_wl_string(&payload) {
                            info.name = Some(s);
                        }
                    }
                }
            }
        }

        // Identify the HDMI output if named; reject headless NOOP-only sessions (stock labwc when unplugged).
        let hdmi_output = select_hdmi_output(&outputs, require_non_noop_output)?;
        let (target_output_id, hdmi_output_global, init_w, init_h) = match hdmi_output {
            Some(o) => (
                o.obj_id,
                Some(o.global_name),
                if o.width > 0 { o.width as usize } else { 1280 },
                if o.height > 0 { o.height as usize } else { 720 },
            ),
            None => (0, None, 1280, 720),
        };

        let wl_surface_id = next_id;
        next_id += 1;
        let xdg_surface_id = next_id;
        next_id += 1;
        let xdg_toplevel_id = next_id;
        next_id += 1;
        let wp_viewport_id = wp_viewporter_id.map(|_| {
            let id = next_id;
            next_id += 1;
            id
        });

        out.clear();
        // wl_compositor::create_surface(wl_surface_id) [opcode 0]
        push_msg(&mut out, wl_compositor_id, 0, &[wl_surface_id]);
        // xdg_wm_base::get_xdg_surface(xdg_surface_id, wl_surface_id) [opcode 2]
        push_msg(&mut out, xdg_wm_base_id, 2, &[xdg_surface_id, wl_surface_id]);
        // xdg_surface::get_toplevel(xdg_toplevel_id) [opcode 1]
        push_msg(&mut out, xdg_surface_id, 1, &[xdg_toplevel_id]);
        // xdg_toplevel::set_title("StickZero") [opcode 2]
        push_msg_str(&mut out, xdg_toplevel_id, 2, "StickZero");
        // xdg_toplevel::set_app_id("stickzero") [opcode 3]
        push_msg_str(&mut out, xdg_toplevel_id, 3, "stickzero");
        // xdg_toplevel::set_fullscreen(target_output_id) [opcode 11]
        push_msg(&mut out, xdg_toplevel_id, 11, &[target_output_id]);
        if let (Some(vp_id), Some(viewport_id)) = (wp_viewporter_id, wp_viewport_id) {
            // wp_viewporter::get_viewport(viewport_id, wl_surface_id) [opcode 1]
            push_msg(&mut out, vp_id, 1, &[viewport_id, wl_surface_id]);
        }
        // Initial empty wl_surface::commit() [opcode 6] to receive xdg_surface::configure
        push_msg(&mut out, wl_surface_id, 6, &[]);
        send_bytes(fd, &out, None)?;

        let mut cfg_w = init_w as i32;
        let mut cfg_h = init_h as i32;
        let mut configured = false;
        let deadline = Instant::now() + Duration::from_millis(800);
        while !configured {
            if Instant::now() >= deadline {
                break;
            }
            if !recv_more(fd, &mut rx_buf, 100)? {
                continue;
            }
            while let Some((obj, opcode, payload)) = next_event(&mut rx_buf) {
                if obj == WL_DISPLAY_ID && opcode == 0 {
                    return Err("Wayland protocol error waiting for xdg_surface::configure".to_string());
                } else if obj == xdg_wm_base_id && opcode == 0 && payload.len() >= 4 {
                    let serial = u32::from_ne_bytes(payload[0..4].try_into().unwrap());
                    let mut pong = Vec::with_capacity(12);
                    push_msg(&mut pong, xdg_wm_base_id, 3, &[serial]);
                    let _ = send_bytes(fd, &pong, None);
                } else if obj == xdg_toplevel_id && opcode == 0 && payload.len() >= 8 {
                    let w = i32::from_ne_bytes(payload[0..4].try_into().unwrap());
                    let h = i32::from_ne_bytes(payload[4..8].try_into().unwrap());
                    if w > 0 && h > 0 {
                        cfg_w = w;
                        cfg_h = h;
                    }
                } else if obj == xdg_surface_id && opcode == 0 && payload.len() >= 4 {
                    let serial = u32::from_ne_bytes(payload[0..4].try_into().unwrap());
                    let mut ack = Vec::with_capacity(12);
                    push_msg(&mut ack, xdg_surface_id, 4, &[serial]);
                    send_bytes(fd, &ack, None)?;
                    configured = true;
                }
            }
        }

        let width = (cfg_w.max(SCREEN_W as i32)) as usize;
        let height = (cfg_h.max(SCREEN_H as i32)) as usize;
        let use_viewporter = wp_viewport_id.is_some();
        let use_rgb565 = supports_rgb565;

        // With wp_viewporter, keep the shm buffer at native 320x170 and let the GPU scale it.
        // Without wp_viewporter, allocate an aspect-fit CPU-scaled buffer (capped at 1920x1080).
        let (buf_w, buf_h) = if use_viewporter {
            (SCREEN_W, SCREEN_H)
        } else {
            let cap_w = width.min(1920);
            let cap_h = height.min(1080);
            let (fw, fh, _, _) = compute_fit_rect(cap_w, cap_h);
            (fw.max(SCREEN_W), fh.max(SCREEN_H))
        };

        let bpp = if use_rgb565 { 2usize } else { 4usize };
        let buf_stride = buf_w * bpp;
        let buf_size = buf_stride * buf_h;
        let shm_len = buf_size * 2;
        let shm_fd = create_shm_fd(shm_len)?;
        let mapped = unsafe {
            mmap(
                ptr::null_mut(),
                shm_len,
                PROT_READ | PROT_WRITE,
                MAP_SHARED,
                shm_fd,
                0,
            )
        };
        if mapped == (!0usize as *mut c_void) || mapped.is_null() {
            unsafe { close(shm_fd) };
            return Err("Wayland shm mmap failed".to_string());
        }
        unsafe {
            ptr::write_bytes(mapped as *mut u8, 0, shm_len);
        }

        let pool_id = next_id;
        next_id += 1;
        let buf0_id = next_id;
        next_id += 1;
        let buf1_id = next_id;
        next_id += 1;

        let mut pool_msg = Vec::with_capacity(20);
        // wl_shm::create_pool(pool_id, fd, size) [opcode 0]
        push_msg(&mut pool_msg, wl_shm_id, 0, &[pool_id, shm_len as u32]);
        if let Err(e) = send_bytes(fd, &pool_msg, Some(shm_fd)) {
            unsafe {
                munmap(mapped, shm_len);
                close(shm_fd);
            }
            return Err(e);
        }

        let wl_fmt = if use_rgb565 {
            WL_SHM_FORMAT_RGB565
        } else {
            WL_SHM_FORMAT_XRGB8888
        };
        out.clear();
        push_msg(
            &mut out,
            pool_id,
            0,
            &[buf0_id, 0, buf_w as u32, buf_h as u32, buf_stride as u32, wl_fmt],
        );
        push_msg(
            &mut out,
            pool_id,
            0,
            &[
                buf1_id,
                buf_size as u32,
                buf_w as u32,
                buf_h as u32,
                buf_stride as u32,
                wl_fmt,
            ],
        );
        if let Some(vp_id) = wp_viewport_id {
            let (fit_w, fit_h, _, _) = compute_fit_rect(width, height);
            // wp_viewport::set_destination(width, height) [opcode 2]
            push_msg(&mut out, vp_id, 2, &[fit_w as u32, fit_h as u32]);
        }
        if let Err(e) = send_bytes(fd, &out, None) {
            unsafe {
                munmap(mapped, shm_len);
                close(shm_fd);
            }
            return Err(e);
        }

        let layout = if use_rgb565 {
            PixelLayout::Rgb565
        } else {
            PixelLayout::xrgb8888()
        };
        let blitter = Blitter::new(buf_w, buf_h, buf_stride, layout);

        Ok(Self {
            stream,
            next_id,
            wl_shm_id,
            wl_surface_id,
            xdg_wm_base_id,
            xdg_surface_id,
            xdg_toplevel_id,
            wp_viewport_id,
            hdmi_output_global,
            use_viewporter,
            use_rgb565,
            shm_fd,
            shm_ptr: mapped as *mut u8,
            shm_len,
            pool_id,
            buf_ids: [buf0_id, buf1_id],
            buf_busy: [false, false],
            src_w: SCREEN_W,
            src_h: SCREEN_H,
            buf_w,
            buf_h,
            buf_stride,
            buf_size,
            blitter,
            width,
            height,
            pending_cfg_w: cfg_w,
            pending_cfg_h: cfg_h,
            configured,
            alive: true,
            rx_buf,
        })
    }

    fn ensure_source_size(&mut self, src_w: usize, src_h: usize) -> bool {
        if src_w == self.src_w && src_h == self.src_h {
            return true;
        }
        self.src_w = src_w;
        self.src_h = src_h;

        if !self.use_viewporter {
            if self.blitter.ensure_source(src_w, src_h) && !self.shm_ptr.is_null() {
                unsafe {
                    ptr::write_bytes(self.shm_ptr, 0, self.shm_len);
                }
            }
            return true;
        }

        // Reallocate double-buffered wl_shm at the exact new source resolution so the GPU
        // (`wp_viewporter`) scales the sharp new framebuffer to HDMI with ~0.1 ms CPU cost.
        let bpp = if self.use_rgb565 { 2usize } else { 4usize };
        let new_stride = src_w * bpp;
        let new_buf_size = new_stride * src_h;
        let new_shm_len = new_buf_size * 2;

        let Ok(new_fd) = create_shm_fd(new_shm_len) else {
            self.alive = false;
            return false;
        };
        let new_mapped = unsafe {
            mmap(
                ptr::null_mut(),
                new_shm_len,
                PROT_READ | PROT_WRITE,
                MAP_SHARED,
                new_fd,
                0,
            )
        };
        if new_mapped == (!0usize as *mut c_void) || new_mapped.is_null() {
            unsafe { close(new_fd) };
            self.alive = false;
            return false;
        }
        unsafe {
            ptr::write_bytes(new_mapped as *mut u8, 0, new_shm_len);
        }

        let fd = self.stream.as_raw_fd();
        let mut destroy_msg = Vec::with_capacity(32);
        push_msg(&mut destroy_msg, self.buf_ids[0], 0, &[]);
        push_msg(&mut destroy_msg, self.buf_ids[1], 0, &[]);
        push_msg(&mut destroy_msg, self.pool_id, 1, &[]);
        if send_bytes(fd, &destroy_msg, None).is_err() {
            unsafe {
                munmap(new_mapped, new_shm_len);
                close(new_fd);
            }
            self.alive = false;
            return false;
        }

        let new_pool_id = self.next_id;
        self.next_id += 1;
        let new_buf0_id = self.next_id;
        self.next_id += 1;
        let new_buf1_id = self.next_id;
        self.next_id += 1;

        let mut pool_msg = Vec::with_capacity(20);
        push_msg(&mut pool_msg, self.wl_shm_id, 0, &[new_pool_id, new_shm_len as u32]);
        if send_bytes(fd, &pool_msg, Some(new_fd)).is_err() {
            unsafe {
                munmap(new_mapped, new_shm_len);
                close(new_fd);
            }
            self.alive = false;
            return false;
        }

        let wl_fmt = if self.use_rgb565 {
            WL_SHM_FORMAT_RGB565
        } else {
            WL_SHM_FORMAT_XRGB8888
        };
        let mut create_msg = Vec::with_capacity(96);
        push_msg(
            &mut create_msg,
            new_pool_id,
            0,
            &[new_buf0_id, 0, src_w as u32, src_h as u32, new_stride as u32, wl_fmt],
        );
        push_msg(
            &mut create_msg,
            new_pool_id,
            0,
            &[
                new_buf1_id,
                new_buf_size as u32,
                src_w as u32,
                src_h as u32,
                new_stride as u32,
                wl_fmt,
            ],
        );
        if let Some(vp_id) = self.wp_viewport_id {
            let (fit_w, fit_h, _, _) = compute_fit_rect_for(src_w, src_h, self.width, self.height);
            push_msg(&mut create_msg, vp_id, 2, &[fit_w as u32, fit_h as u32]);
        }
        if send_bytes(fd, &create_msg, None).is_err() {
            unsafe {
                munmap(new_mapped, new_shm_len);
                close(new_fd);
            }
            self.alive = false;
            return false;
        }

        if !self.shm_ptr.is_null() && self.shm_len > 0 {
            unsafe {
                munmap(self.shm_ptr as *mut c_void, self.shm_len);
            }
        }
        if self.shm_fd >= 0 {
            unsafe { close(self.shm_fd) };
        }

        let layout = if self.use_rgb565 {
            PixelLayout::Rgb565
        } else {
            PixelLayout::xrgb8888()
        };
        self.shm_fd = new_fd;
        self.shm_ptr = new_mapped as *mut u8;
        self.shm_len = new_shm_len;
        self.pool_id = new_pool_id;
        self.buf_ids = [new_buf0_id, new_buf1_id];
        self.buf_busy = [false, false];
        self.buf_w = src_w;
        self.buf_h = src_h;
        self.buf_stride = new_stride;
        self.buf_size = new_buf_size;
        self.blitter = Blitter::new_for_source(src_w, src_h, src_w, src_h, new_stride, layout);
        true
    }

    fn pump_events_nonblocking(&mut self) {
        let fd = self.stream.as_raw_fd();
        loop {
            let mut tmp = [0u8; 2048];
            let mut iov = Iovec {
                iov_base: tmp.as_mut_ptr() as *mut c_void,
                iov_len: tmp.len(),
            };
            let mut msg = Msghdr {
                msg_name: ptr::null_mut(),
                msg_namelen: 0,
                _pad0: 0,
                msg_iov: &mut iov,
                msg_iovlen: 1,
                msg_control: ptr::null_mut(),
                msg_controllen: 0,
                msg_flags: 0,
                _pad1: 0,
            };
            let n = unsafe { recvmsg(fd, &mut msg, MSG_DONTWAIT) };
            if n > 0 {
                self.rx_buf.extend_from_slice(&tmp[..n as usize]);
            } else if n == 0 {
                self.alive = false;
                return;
            } else {
                let err = std::io::Error::last_os_error();
                if err.kind() != ErrorKind::WouldBlock && err.kind() != ErrorKind::Interrupted {
                    self.alive = false;
                }
                break;
            }
        }

        let mut replies = Vec::new();
        while let Some((obj, opcode, payload)) = next_event(&mut self.rx_buf) {
            if obj == WL_DISPLAY_ID && opcode == 0 {
                self.alive = false;
                return;
            } else if obj == 2 && opcode == 1 && payload.len() >= 4 {
                // wl_registry::global_remove(name)
                let removed = u32::from_ne_bytes(payload[0..4].try_into().unwrap());
                if self.hdmi_output_global == Some(removed) {
                    self.alive = false;
                    return;
                }
            } else if obj == self.xdg_wm_base_id && opcode == 0 && payload.len() >= 4 {
                let serial = u32::from_ne_bytes(payload[0..4].try_into().unwrap());
                push_msg(&mut replies, self.xdg_wm_base_id, 3, &[serial]);
            } else if obj == self.xdg_toplevel_id && opcode == 0 && payload.len() >= 8 {
                let w = i32::from_ne_bytes(payload[0..4].try_into().unwrap());
                let h = i32::from_ne_bytes(payload[4..8].try_into().unwrap());
                if w > 0 && h > 0 {
                    self.pending_cfg_w = w;
                    self.pending_cfg_h = h;
                }
            } else if obj == self.xdg_toplevel_id && opcode == 1 {
                self.alive = false;
                return;
            } else if obj == self.xdg_surface_id && opcode == 0 && payload.len() >= 4 {
                let serial = u32::from_ne_bytes(payload[0..4].try_into().unwrap());
                push_msg(&mut replies, self.xdg_surface_id, 4, &[serial]);
                self.configured = true;
                let new_w = self.pending_cfg_w.max(SCREEN_W as i32) as usize;
                let new_h = self.pending_cfg_h.max(SCREEN_H as i32) as usize;
                if new_w != self.width || new_h != self.height {
                    self.width = new_w;
                    self.height = new_h;
                    if let Some(vp_id) = self.wp_viewport_id {
                        let (fit_w, fit_h, _, _) =
                            compute_fit_rect_for(self.src_w, self.src_h, new_w, new_h);
                        push_msg(&mut replies, vp_id, 2, &[fit_w as u32, fit_h as u32]);
                    }
                }
            } else if obj == self.buf_ids[0] && opcode == 0 {
                self.buf_busy[0] = false;
            } else if obj == self.buf_ids[1] && opcode == 0 {
                self.buf_busy[1] = false;
            }
        }

        if !replies.is_empty() && send_bytes(fd, &replies, None).is_err() {
            self.alive = false;
        }
    }

    pub fn present(&mut self, rgb565: &[u16], src_w: usize, src_h: usize) -> bool {
        if !self.alive {
            return false;
        }
        self.pump_events_nonblocking();
        if !self.alive {
            return false;
        }
        if !self.ensure_source_size(src_w, src_h) {
            return false;
        }

        let slot = if !self.buf_busy[0] {
            0
        } else if !self.buf_busy[1] {
            1
        } else {
            // Compositor is still scanning both buffers (e.g. 30 Hz TV): skip frame without blocking 60 Hz sim.
            return true;
        };

        let dst_ptr = unsafe { self.shm_ptr.add(slot * self.buf_size) };
        self.blitter
            .blit_rgb565(rgb565, src_w, src_h, dst_ptr, self.buf_size);

        let mut out = Vec::with_capacity(64);
        // wl_surface::attach(buf_id, 0, 0) [opcode 1]
        push_msg(&mut out, self.wl_surface_id, 1, &[self.buf_ids[slot], 0, 0]);
        // wl_surface::damage(0, 0, i32::MAX, i32::MAX) [opcode 2]
        push_msg(
            &mut out,
            self.wl_surface_id,
            2,
            &[0, 0, i32::MAX as u32, i32::MAX as u32],
        );
        // wl_surface::commit() [opcode 6]
        push_msg(&mut out, self.wl_surface_id, 6, &[]);

        if send_bytes(self.stream.as_raw_fd(), &out, None).is_err() {
            self.alive = false;
            return false;
        }
        self.buf_busy[slot] = true;
        true
    }

    #[allow(dead_code)]
    pub fn debug_info(&self) -> (bool, bool, usize, usize) {
        (self.use_viewporter, self.use_rgb565, self.buf_w, self.buf_h)
    }
}

impl Drop for WaylandDisplay {
    fn drop(&mut self) {
        let mut out = Vec::with_capacity(64);
        push_msg(&mut out, self.xdg_toplevel_id, 0, &[]);
        push_msg(&mut out, self.xdg_surface_id, 0, &[]);
        if let Some(vp_id) = self.wp_viewport_id {
            push_msg(&mut out, vp_id, 0, &[]);
        }
        push_msg(&mut out, self.wl_surface_id, 0, &[]);
        push_msg(&mut out, self.buf_ids[0], 0, &[]);
        push_msg(&mut out, self.buf_ids[1], 0, &[]);
        push_msg(&mut out, self.pool_id, 1, &[]);
        let _ = send_bytes(self.stream.as_raw_fd(), &out, None);

        if !self.shm_ptr.is_null() && self.shm_len > 0 {
            unsafe {
                munmap(self.shm_ptr as *mut c_void, self.shm_len);
            }
            self.shm_ptr = ptr::null_mut();
        }
        if self.shm_fd >= 0 {
            unsafe { close(self.shm_fd) };
            self.shm_fd = -1;
        }
        let _ = (self.next_id, self.wl_shm_id, self.buf_stride);
    }
}

fn select_hdmi_output(
    outputs: &[OutputInfo],
    require_non_noop: bool,
) -> Result<Option<OutputInfo>, String> {
    if outputs.is_empty() {
        return Ok(None);
    }
    // Prefer an explicitly named HDMI output (e.g. "HDMI-A-1" on labwc / wlroots).
    if let Some(o) = outputs.iter().find(|o| {
        o.name
            .as_deref()
            .map(|n| n.to_ascii_uppercase().contains("HDMI"))
            .unwrap_or(false)
    }) {
        return Ok(Some(o.clone()));
    }
    // If all outputs reported names and every one is a headless/NOOP output, decline Wayland.
    let all_named = outputs.iter().all(|o| o.name.is_some());
    let any_real = outputs.iter().any(|o| {
        o.name
            .as_deref()
            .map(|n| {
                let up = n.to_ascii_uppercase();
                !up.starts_with("NOOP") && !up.starts_with("HEADLESS")
            })
            .unwrap_or(true)
    });
    if require_non_noop && all_named && !any_real {
        return Err("Wayland compositor only has headless/NOOP outputs".to_string());
    }
    Ok(outputs.first().cloned())
}

fn create_shm_fd(size: usize) -> Result<i32, String> {
    let cname = CString::new("stickzero-wl").unwrap();
    let fd = unsafe { syscall(SYS_MEMFD_CREATE, cname.as_ptr(), MFD_CLOEXEC) as i32 };
    let fd = if fd >= 0 {
        fd
    } else {
        let seq = SHM_COUNTER.fetch_add(1, Ordering::Relaxed);
        let path = format!("/dev/shm/stickzero-wl-{}-{seq}", std::process::id());
        let cpath = CString::new(path).map_err(|_| "invalid shm path".to_string())?;
        let f = unsafe { open(cpath.as_ptr(), O_RDWR | O_CREAT | O_EXCL | O_CLOEXEC, 0o600) };
        if f < 0 {
            return Err("failed to create Wayland shm file".to_string());
        }
        unsafe { unlink(cpath.as_ptr()) };
        f
    };

    if unsafe { ftruncate(fd, size as i64) } < 0 {
        unsafe { close(fd) };
        return Err("ftruncate failed on Wayland shm fd".to_string());
    }
    Ok(fd)
}

fn push_msg(buf: &mut Vec<u8>, obj_id: u32, opcode: u16, args: &[u32]) {
    let size = (8 + args.len() * 4) as u32;
    buf.extend_from_slice(&obj_id.to_ne_bytes());
    buf.extend_from_slice(&((size << 16) | opcode as u32).to_ne_bytes());
    for &a in args {
        buf.extend_from_slice(&a.to_ne_bytes());
    }
}

fn push_wl_string(buf: &mut Vec<u8>, s: &str) {
    let bytes = s.as_bytes();
    let len_with_nul = (bytes.len() + 1) as u32;
    buf.extend_from_slice(&len_with_nul.to_ne_bytes());
    buf.extend_from_slice(bytes);
    buf.push(0);
    while buf.len() % 4 != 0 {
        buf.push(0);
    }
}

fn push_msg_str(buf: &mut Vec<u8>, obj_id: u32, opcode: u16, s: &str) {
    let start = buf.len();
    buf.extend_from_slice(&obj_id.to_ne_bytes());
    buf.extend_from_slice(&0u32.to_ne_bytes());
    push_wl_string(buf, s);
    let size = (buf.len() - start) as u32;
    let header = (size << 16) | opcode as u32;
    buf[start + 4..start + 8].copy_from_slice(&header.to_ne_bytes());
}

fn push_bind(
    buf: &mut Vec<u8>,
    registry_id: u32,
    global_name: u32,
    iface: &str,
    version: u32,
    new_id: u32,
) {
    let start = buf.len();
    buf.extend_from_slice(&registry_id.to_ne_bytes());
    buf.extend_from_slice(&0u32.to_ne_bytes());
    buf.extend_from_slice(&global_name.to_ne_bytes());
    push_wl_string(buf, iface);
    buf.extend_from_slice(&version.to_ne_bytes());
    buf.extend_from_slice(&new_id.to_ne_bytes());
    let size = (buf.len() - start) as u32;
    let header = size << 16; // opcode 0 = wl_registry::bind
    buf[start + 4..start + 8].copy_from_slice(&header.to_ne_bytes());
}

fn send_bytes(sock_fd: i32, data: &[u8], attach_fd: Option<i32>) -> Result<(), String> {
    let mut offset = 0usize;
    let mut pending_fd = attach_fd;
    while offset < data.len() {
        let mut iov = Iovec {
            iov_base: (data[offset..].as_ptr()) as *mut c_void,
            iov_len: data.len() - offset,
        };
        let mut cmsg = CmsgFd {
            hdr: Cmsghdr {
                cmsg_len: std::mem::size_of::<Cmsghdr>() + std::mem::size_of::<i32>(),
                cmsg_level: SOL_SOCKET,
                cmsg_type: SCM_RIGHTS,
            },
            fd: pending_fd.unwrap_or(-1),
            _pad: 0,
        };
        let (ctrl_ptr, ctrl_len) = if pending_fd.is_some() {
            (
                (&mut cmsg as *mut CmsgFd) as *mut c_void,
                std::mem::size_of::<CmsgFd>(),
            )
        } else {
            (ptr::null_mut(), 0)
        };
        let msg = Msghdr {
            msg_name: ptr::null_mut(),
            msg_namelen: 0,
            _pad0: 0,
            msg_iov: &mut iov,
            msg_iovlen: 1,
            msg_control: ctrl_ptr,
            msg_controllen: ctrl_len,
            msg_flags: 0,
            _pad1: 0,
        };
        let n = unsafe { sendmsg(sock_fd, &msg, MSG_NOSIGNAL) };
        if n > 0 {
            offset += n as usize;
            pending_fd = None;
        } else {
            let err = std::io::Error::last_os_error();
            if err.kind() == ErrorKind::Interrupted {
                continue;
            }
            return Err(format!("Wayland sendmsg failed: {err}"));
        }
    }
    Ok(())
}

fn recv_more(sock_fd: i32, rx_buf: &mut Vec<u8>, timeout_ms: i32) -> Result<bool, String> {
    let mut pfd = PollFd {
        fd: sock_fd,
        events: POLLIN,
        revents: 0,
    };
    let r = unsafe { poll(&mut pfd, 1, timeout_ms) };
    if r <= 0 {
        return Ok(false);
    }
    let mut tmp = [0u8; 4096];
    let mut iov = Iovec {
        iov_base: tmp.as_mut_ptr() as *mut c_void,
        iov_len: tmp.len(),
    };
    let mut msg = Msghdr {
        msg_name: ptr::null_mut(),
        msg_namelen: 0,
        _pad0: 0,
        msg_iov: &mut iov,
        msg_iovlen: 1,
        msg_control: ptr::null_mut(),
        msg_controllen: 0,
        msg_flags: 0,
        _pad1: 0,
    };
    let n = unsafe { recvmsg(sock_fd, &mut msg, MSG_DONTWAIT) };
    if n > 0 {
        rx_buf.extend_from_slice(&tmp[..n as usize]);
        Ok(true)
    } else if n == 0 {
        Err("Wayland compositor closed connection".to_string())
    } else {
        let err = std::io::Error::last_os_error();
        if err.kind() == ErrorKind::WouldBlock || err.kind() == ErrorKind::Interrupted {
            Ok(false)
        } else {
            Err(format!("Wayland recvmsg failed: {err}"))
        }
    }
}

fn next_event(rx_buf: &mut Vec<u8>) -> Option<(u32, u16, Vec<u8>)> {
    if rx_buf.len() < 8 {
        return None;
    }
    let obj_id = u32::from_ne_bytes(rx_buf[0..4].try_into().ok()?);
    let size_op = u32::from_ne_bytes(rx_buf[4..8].try_into().ok()?);
    let size = (size_op >> 16) as usize;
    let opcode = (size_op & 0xFFFF) as u16;
    if size < 8 || rx_buf.len() < size {
        return None;
    }
    let payload = rx_buf[8..size].to_vec();
    rx_buf.drain(0..size);
    Some((obj_id, opcode, payload))
}

fn parse_wl_string(data: &[u8]) -> Option<String> {
    if data.len() < 4 {
        return None;
    }
    let len = u32::from_ne_bytes(data[0..4].try_into().ok()?) as usize;
    if len == 0 || 4 + len > data.len() {
        return None;
    }
    let raw = &data[4..4 + len - 1];
    Some(String::from_utf8_lossy(raw).into_owned())
}

fn parse_registry_global(payload: &[u8]) -> Option<(u32, String, u32)> {
    if payload.len() < 12 {
        return None;
    }
    let name = u32::from_ne_bytes(payload[0..4].try_into().ok()?);
    let str_len = u32::from_ne_bytes(payload[4..8].try_into().ok()?) as usize;
    if str_len == 0 {
        return None;
    }
    let padded = (str_len + 3) & !3;
    if 8 + padded + 4 > payload.len() {
        return None;
    }
    let iface = String::from_utf8_lossy(&payload[8..8 + str_len - 1]).into_owned();
    let ver_off = 8 + padded;
    let ver = u32::from_ne_bytes(payload[ver_off..ver_off + 4].try_into().ok()?);
    Some((name, iface, ver))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;
    use std::os::unix::net::UnixListener;
    use std::thread;

    fn send_global(stream: &mut UnixStream, registry_id: u32, name: u32, iface: &str, ver: u32) {
        let mut msg = Vec::new();
        msg.extend_from_slice(&registry_id.to_ne_bytes());
        msg.extend_from_slice(&0u32.to_ne_bytes());
        msg.extend_from_slice(&name.to_ne_bytes());
        push_wl_string(&mut msg, iface);
        msg.extend_from_slice(&ver.to_ne_bytes());
        let size = msg.len() as u32;
        msg[4..8].copy_from_slice(&(size << 16).to_ne_bytes());
        stream.write_all(&msg).unwrap();
    }

    #[test]
    fn test_wayland_handshake_viewporter_and_present_with_mock_compositor() {
        let sock_path = PathBuf::from(format!("/tmp/stickzero-wl-test-{}.sock", std::process::id()));
        let _ = std::fs::remove_file(&sock_path);
        let listener = UnixListener::bind(&sock_path).unwrap();

        let server = thread::spawn(move || {
            let (mut conn, _) = listener.accept().unwrap();
            let fd = conn.as_raw_fd();
            let mut rx = Vec::new();

            // 1. Read get_registry + sync1
            let mut reg_id = 0u32;
            let mut sync1 = 0u32;
            while sync1 == 0 {
                recv_more(fd, &mut rx, 500).unwrap();
                while let Some((obj, op, payload)) = next_event(&mut rx) {
                    if obj == 1 && op == 1 {
                        reg_id = u32::from_ne_bytes(payload[0..4].try_into().unwrap());
                    } else if obj == 1 && op == 0 {
                        sync1 = u32::from_ne_bytes(payload[0..4].try_into().unwrap());
                    }
                }
            }

            // Advertise wl_compositor, wl_shm, xdg_wm_base, wp_viewporter, and HDMI-A-1 wl_output
            send_global(&mut conn, reg_id, 10, "wl_compositor", 4);
            send_global(&mut conn, reg_id, 11, "wl_shm", 1);
            send_global(&mut conn, reg_id, 12, "xdg_wm_base", 2);
            send_global(&mut conn, reg_id, 13, "wp_viewporter", 1);
            send_global(&mut conn, reg_id, 14, "wl_output", 4);
            let mut done = Vec::new();
            push_msg(&mut done, sync1, 0, &[0]);
            conn.write_all(&done).unwrap();

            // 2. Read binds + sync2
            let mut shm_id = 0u32;
            let mut out_id = 0u32;
            let mut sync2 = 0u32;
            while sync2 == 0 {
                recv_more(fd, &mut rx, 500).unwrap();
                while let Some((obj, op, payload)) = next_event(&mut rx) {
                    if obj == reg_id && op == 0 {
                        let gname = u32::from_ne_bytes(payload[0..4].try_into().unwrap());
                        let new_id = u32::from_ne_bytes(
                            payload[payload.len() - 4..].try_into().unwrap(),
                        );
                        if gname == 11 {
                            shm_id = new_id;
                        } else if gname == 14 {
                            out_id = new_id;
                        }
                    } else if obj == 1 && op == 0 {
                        sync2 = u32::from_ne_bytes(payload[0..4].try_into().unwrap());
                    }
                }
            }

            let mut ev = Vec::new();
            // wl_shm::format(RGB565)
            push_msg(&mut ev, shm_id, 0, &[WL_SHM_FORMAT_RGB565]);
            // wl_output::mode(CURRENT, 1920, 1080, 60000)
            push_msg(&mut ev, out_id, 1, &[1, 1920, 1080, 60000]);
            // wl_output::name("HDMI-A-1")
            push_msg_str(&mut ev, out_id, 4, "HDMI-A-1");
            // sync2::done
            push_msg(&mut ev, sync2, 0, &[0]);
            conn.write_all(&ev).unwrap();

            // 3. Wait for initial surface commit, send xdg_toplevel::configure + xdg_surface::configure
            let mut xdg_surf = 0u32;
            let mut xdg_top = 0u32;
            let mut saw_fullscreen = false;
            let mut saw_commit = false;
            while !saw_commit {
                recv_more(fd, &mut rx, 500).unwrap();
                while let Some((_obj, op, payload)) = next_event(&mut rx) {
                    if op == 2 && payload.len() == 8 && xdg_surf == 0 {
                        xdg_surf = u32::from_ne_bytes(payload[0..4].try_into().unwrap());
                    } else if op == 1 && payload.len() == 4 && xdg_surf != 0 && xdg_top == 0 {
                        xdg_top = u32::from_ne_bytes(payload[0..4].try_into().unwrap());
                    } else if op == 11 && payload.len() == 4 {
                        let fs_out = u32::from_ne_bytes(payload[0..4].try_into().unwrap());
                        assert_eq!(fs_out, out_id);
                        saw_fullscreen = true;
                    } else if op == 6 && payload.is_empty() {
                        saw_commit = true;
                    }
                }
            }
            assert!(saw_fullscreen);

            ev.clear();
            // xdg_toplevel::configure(1920, 1080, [])
            push_msg(&mut ev, xdg_top, 0, &[1920, 1080, 0]);
            // xdg_surface::configure(42)
            push_msg(&mut ev, xdg_surf, 0, &[42]);
            conn.write_all(&ev).unwrap();

            // 4. Verify client sends ack_configure(42), create_pool (with SCM_RIGHTS fd), create_buffer, attach, commit
            //    and then a second shm fd + attach when switching live to 640x360.
            let mut saw_ack = false;
            let mut attach_count = 0usize;
            let mut fd_count = 0usize;
            let deadline = Instant::now() + Duration::from_millis(1200);
            while !(saw_ack && attach_count >= 2 && fd_count >= 2) && Instant::now() < deadline {
                let mut tmp = [0u8; 4096];
                let mut iov = Iovec {
                    iov_base: tmp.as_mut_ptr() as *mut c_void,
                    iov_len: tmp.len(),
                };
                let mut cbuf = CmsgFd {
                    hdr: Cmsghdr::default(),
                    fd: -1,
                    _pad: 0,
                };
                let mut msg = Msghdr {
                    msg_name: ptr::null_mut(),
                    msg_namelen: 0,
                    _pad0: 0,
                    msg_iov: &mut iov,
                    msg_iovlen: 1,
                    msg_control: (&mut cbuf as *mut CmsgFd) as *mut c_void,
                    msg_controllen: std::mem::size_of::<CmsgFd>(),
                    msg_flags: 0,
                    _pad1: 0,
                };
                let n = unsafe { recvmsg(fd, &mut msg, 0) };
                if n <= 0 {
                    break;
                }
                if msg.msg_controllen >= std::mem::size_of::<Cmsghdr>() + 4
                    && cbuf.hdr.cmsg_level == SOL_SOCKET
                    && cbuf.hdr.cmsg_type == SCM_RIGHTS
                    && cbuf.fd >= 0
                {
                    fd_count += 1;
                    unsafe { close(cbuf.fd) };
                }
                rx.extend_from_slice(&tmp[..n as usize]);
                while let Some((obj, op, payload)) = next_event(&mut rx) {
                    if obj == xdg_surf && op == 4 && payload.len() == 4 {
                        let serial = u32::from_ne_bytes(payload[0..4].try_into().unwrap());
                        assert_eq!(serial, 42);
                        saw_ack = true;
                    } else if op == 1 && payload.len() == 12 {
                        attach_count += 1;
                    }
                }
            }
            assert!(saw_ack, "expected xdg_surface::ack_configure");
            assert!(fd_count >= 2, "expected initial + resized SCM_RIGHTS shm fds");
            assert!(attach_count >= 2, "expected wl_surface::attach for 320x170 and 640x360");
        });

        let mut disp = WaylandDisplay::connect_path(&sock_path, true).unwrap();
        assert_eq!((disp.width, disp.height), (1920, 1080));
        assert_eq!(disp.debug_info(), (true, true, SCREEN_W, SCREEN_H));

        let fb = [0xF800u16; SCREEN_W * SCREEN_H];
        assert!(disp.present(&fb, SCREEN_W, SCREEN_H));

        let fb_hd = vec![0x07E0u16; 640 * 360];
        assert!(disp.present(&fb_hd, 640, 360));
        assert_eq!(disp.debug_info(), (true, true, 640, 360));

        server.join().unwrap();
        drop(disp);
        let _ = std::fs::remove_file(&sock_path);
    }

    #[test]
    fn test_noop_only_wayland_output_is_rejected() {
        let outputs = vec![OutputInfo {
            global_name: 1,
            obj_id: 10,
            name: Some("NOOP-1".to_string()),
            width: 1280,
            height: 720,
        }];
        assert!(select_hdmi_output(&outputs, true).is_err());

        let outputs_hdmi = vec![
            outputs[0].clone(),
            OutputInfo {
                global_name: 2,
                obj_id: 11,
                name: Some("HDMI-A-1".to_string()),
                width: 1920,
                height: 1080,
            },
        ];
        let picked = select_hdmi_output(&outputs_hdmi, true).unwrap().unwrap();
        assert_eq!(picked.name.as_deref(), Some("HDMI-A-1"));
    }
}
