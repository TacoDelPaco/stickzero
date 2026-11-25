//! Zero-dependency Linux DRM/KMS dumb-buffer HDMI backend.
//!
//! Used automatically when HDMI is connected and no Wayland compositor owns DRM master
//! (e.g., CardputerZero with `lightdm` disabled, Raspberry Pi OS Lite, or console mode).
//! Prefers 1280x720@60Hz (exact 4x integer scale of 320x170 -> 1280x680) and native
//! 16-bit `DRM_FORMAT_RGB565` (with automatic 32-bit `XRGB8888` fallback), and cleanly
//! restores the previous CRTC state on exit or unplug.

use crate::display::{Blitter, PixelLayout, SCREEN_H, SCREEN_W};
use crate::linux_sys::{
    close, ioctl, mmap, munmap, open, DrmModeCardRes, DrmModeCreateDumb, DrmModeCrtc,
    DrmModeDestroyDumb, DrmModeFbCmd, DrmModeFbCmd2, DrmModeFbDirtyCmd, DrmModeGetConnector,
    DrmModeGetEncoder, DrmModeMapDumb, DrmModeModeInfo, DRM_FORMAT_RGB565,
    DRM_IOCTL_MODE_ADDFB, DRM_IOCTL_MODE_ADDFB2, DRM_IOCTL_MODE_CREATE_DUMB,
    DRM_IOCTL_MODE_DESTROY_DUMB, DRM_IOCTL_MODE_DIRTYFB, DRM_IOCTL_MODE_GETCONNECTOR,
    DRM_IOCTL_MODE_GETCRTC, DRM_IOCTL_MODE_GETENCODER, DRM_IOCTL_MODE_GETRESOURCES,
    DRM_IOCTL_MODE_MAP_DUMB, DRM_IOCTL_MODE_RMFB, DRM_IOCTL_MODE_SETCRTC, DRM_MODE_CONNECTED,
    DRM_MODE_CONNECTOR_HDMIA, DRM_MODE_CONNECTOR_HDMIB, DRM_MODE_TYPE_PREFERRED, MAP_SHARED,
    O_CLOEXEC, O_RDWR, PROT_READ, PROT_WRITE,
};
use std::ffi::CString;
use std::ptr;

struct DrmDumbBuf {
    handle: u32,
    fb_id: u32,
    pitch: u32,
    size: usize,
    ptr: *mut u8,
}

impl DrmDumbBuf {
    fn create(fd: i32, width: u32, height: u32, use_rgb565: bool) -> Result<Self, String> {
        let mut create = DrmModeCreateDumb {
            height,
            width,
            bpp: if use_rgb565 { 16 } else { 32 },
            ..Default::default()
        };
        if unsafe { ioctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &mut create) } < 0 {
            return Err("DRM_IOCTL_MODE_CREATE_DUMB failed".to_string());
        }

        let fb_id = if use_rgb565 {
            let mut cmd2 = DrmModeFbCmd2 {
                width,
                height,
                pixel_format: DRM_FORMAT_RGB565,
                handles: [create.handle, 0, 0, 0],
                pitches: [create.pitch, 0, 0, 0],
                ..Default::default()
            };
            if unsafe { ioctl(fd, DRM_IOCTL_MODE_ADDFB2, &mut cmd2) } < 0 {
                let mut destroy = DrmModeDestroyDumb {
                    handle: create.handle,
                };
                unsafe { ioctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &mut destroy) };
                return Err("DRM_IOCTL_MODE_ADDFB2(RGB565) failed".to_string());
            }
            cmd2.fb_id
        } else {
            let mut cmd = DrmModeFbCmd {
                width,
                height,
                pitch: create.pitch,
                bpp: 32,
                depth: 24,
                handle: create.handle,
                fb_id: 0,
            };
            if unsafe { ioctl(fd, DRM_IOCTL_MODE_ADDFB, &mut cmd) } < 0 {
                let mut destroy = DrmModeDestroyDumb {
                    handle: create.handle,
                };
                unsafe { ioctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &mut destroy) };
                return Err("DRM_IOCTL_MODE_ADDFB(XRGB8888) failed".to_string());
            }
            cmd.fb_id
        };

        let mut map = DrmModeMapDumb {
            handle: create.handle,
            ..Default::default()
        };
        if unsafe { ioctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &mut map) } < 0 {
            let mut fb = fb_id;
            let mut destroy = DrmModeDestroyDumb {
                handle: create.handle,
            };
            unsafe {
                ioctl(fd, DRM_IOCTL_MODE_RMFB, &mut fb);
                ioctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &mut destroy);
            }
            return Err("DRM_IOCTL_MODE_MAP_DUMB failed".to_string());
        }

        let size = create.size as usize;
        let mapped = unsafe {
            mmap(
                ptr::null_mut(),
                size,
                PROT_READ | PROT_WRITE,
                MAP_SHARED,
                fd,
                map.offset as i64,
            )
        };
        if mapped == (!0usize as *mut std::ffi::c_void) || mapped.is_null() {
            let mut fb = fb_id;
            let mut destroy = DrmModeDestroyDumb {
                handle: create.handle,
            };
            unsafe {
                ioctl(fd, DRM_IOCTL_MODE_RMFB, &mut fb);
                ioctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &mut destroy);
            }
            return Err("DRM mmap failed".to_string());
        }

        unsafe {
            ptr::write_bytes(mapped as *mut u8, 0, size);
        }

        Ok(Self {
            handle: create.handle,
            fb_id,
            pitch: create.pitch,
            size,
            ptr: mapped as *mut u8,
        })
    }

    fn destroy(&mut self, fd: i32) {
        if !self.ptr.is_null() && self.size > 0 {
            unsafe {
                munmap(self.ptr as *mut std::ffi::c_void, self.size);
            }
            self.ptr = ptr::null_mut();
        }
        if self.fb_id != 0 {
            let mut fb = self.fb_id;
            unsafe { ioctl(fd, DRM_IOCTL_MODE_RMFB, &mut fb) };
            self.fb_id = 0;
        }
        if self.handle != 0 {
            let mut destroy = DrmModeDestroyDumb {
                handle: self.handle,
            };
            unsafe { ioctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &mut destroy) };
            self.handle = 0;
        }
    }
}

pub struct DrmDisplay {
    fd: i32,
    connector_id: u32,
    crtc_id: u32,
    mode: DrmModeModeInfo,
    saved_crtc: Option<DrmModeCrtc>,
    bufs: [DrmDumbBuf; 2],
    front: usize,
    blitter: Blitter,
    pub width: usize,
    pub height: usize,
    pub alive: bool,
}

impl DrmDisplay {
    /// Probe `/dev/dri/card0..3` for a connected HDMI-A / HDMI-B connector and initialize
    /// double-buffered dumb scanout. Fails cleanly if another compositor holds DRM master.
    pub fn open_hdmi() -> Result<Self, String> {
        let mut last_err = "no connected HDMI DRM connector found".to_string();
        for idx in 0..4 {
            let path = format!("/dev/dri/card{idx}");
            match Self::try_open_card(&path) {
                Ok(disp) => return Ok(disp),
                Err(e) => last_err = e,
            }
        }
        Err(last_err)
    }

    fn try_open_card(path: &str) -> Result<Self, String> {
        let cpath = CString::new(path).map_err(|_| "invalid drm path".to_string())?;
        let fd = unsafe { open(cpath.as_ptr(), O_RDWR | O_CLOEXEC) };
        if fd < 0 {
            return Err(format!("cannot open {path}"));
        }

        let init = Self::init_on_fd(fd);
        if init.is_err() {
            unsafe { close(fd) };
        }
        init
    }

    fn init_on_fd(fd: i32) -> Result<Self, String> {
        let mut res = DrmModeCardRes::default();
        if unsafe { ioctl(fd, DRM_IOCTL_MODE_GETRESOURCES, &mut res) } < 0 {
            return Err("DRM_IOCTL_MODE_GETRESOURCES failed".to_string());
        }
        if res.count_connectors == 0 || res.count_crtcs == 0 {
            return Err("DRM card has no connectors or CRTCs".to_string());
        }

        let mut fbs = vec![0u32; res.count_fbs as usize];
        let mut crtcs = vec![0u32; res.count_crtcs as usize];
        let mut connectors = vec![0u32; res.count_connectors as usize];
        let mut encoders = vec![0u32; res.count_encoders as usize];
        res.fb_id_ptr = fbs.as_mut_ptr() as u64;
        res.crtc_id_ptr = crtcs.as_mut_ptr() as u64;
        res.connector_id_ptr = connectors.as_mut_ptr() as u64;
        res.encoder_id_ptr = encoders.as_mut_ptr() as u64;
        if unsafe { ioctl(fd, DRM_IOCTL_MODE_GETRESOURCES, &mut res) } < 0 {
            return Err("DRM_IOCTL_MODE_GETRESOURCES(2) failed".to_string());
        }

        for &conn_id in &connectors {
            let mut conn = DrmModeGetConnector {
                connector_id: conn_id,
                ..Default::default()
            };
            if unsafe { ioctl(fd, DRM_IOCTL_MODE_GETCONNECTOR, &mut conn) } < 0 {
                continue;
            }
            if conn.connection != DRM_MODE_CONNECTED
                || !is_hdmi_connector(conn.connector_type)
                || conn.count_modes == 0
            {
                continue;
            }

            let mut modes = vec![DrmModeModeInfo::default(); conn.count_modes as usize];
            let mut conn_encoders = vec![0u32; conn.count_encoders as usize];
            let mut props = vec![0u32; conn.count_props as usize];
            let mut prop_vals = vec![0u64; conn.count_props as usize];
            conn.modes_ptr = modes.as_mut_ptr() as u64;
            conn.encoders_ptr = conn_encoders.as_mut_ptr() as u64;
            conn.props_ptr = props.as_mut_ptr() as u64;
            conn.prop_values_ptr = prop_vals.as_mut_ptr() as u64;
            if unsafe { ioctl(fd, DRM_IOCTL_MODE_GETCONNECTOR, &mut conn) } < 0 {
                continue;
            }
            if modes.is_empty() {
                continue;
            }

            let crtc_id = match find_crtc_for_connector(fd, conn.encoder_id, &conn_encoders, &crtcs) {
                Some(id) => id,
                None => continue,
            };

            let mut saved = DrmModeCrtc {
                crtc_id,
                ..Default::default()
            };
            let saved_crtc = if unsafe { ioctl(fd, DRM_IOCTL_MODE_GETCRTC, &mut saved) } == 0 {
                Some(saved)
            } else {
                None
            };

            let mode = select_best_mode(&modes);
            let width = mode.hdisplay as u32;
            let height = mode.vdisplay as u32;

            // Prefer 16-bit RGB565 (half the memory bandwidth on Pi CM0 LPDDR2), fall back to XRGB8888.
            let (mut bufs, layout) = match (
                DrmDumbBuf::create(fd, width, height, true),
                DrmDumbBuf::create(fd, width, height, true),
            ) {
                (Ok(b0), Ok(b1)) => ([b0, b1], PixelLayout::Rgb565),
                (b0, b1) => {
                    if let Ok(mut b) = b0 {
                        b.destroy(fd);
                    }
                    if let Ok(mut b) = b1 {
                        b.destroy(fd);
                    }
                    let b0 = DrmDumbBuf::create(fd, width, height, false)?;
                    let b1 = match DrmDumbBuf::create(fd, width, height, false) {
                        Ok(b) => b,
                        Err(e) => {
                            let mut b0 = b0;
                            b0.destroy(fd);
                            return Err(e);
                        }
                    };
                    ([b0, b1], PixelLayout::xrgb8888())
                }
            };

            let mut set_conn = conn_id;
            let mut set_crtc = DrmModeCrtc {
                set_connectors_ptr: (&mut set_conn as *mut u32) as u64,
                count_connectors: 1,
                crtc_id,
                fb_id: bufs[0].fb_id,
                x: 0,
                y: 0,
                gamma_size: 0,
                mode_valid: 1,
                mode,
            };
            if unsafe { ioctl(fd, DRM_IOCTL_MODE_SETCRTC, &mut set_crtc) } < 0 {
                bufs[0].destroy(fd);
                bufs[1].destroy(fd);
                return Err("DRM_IOCTL_MODE_SETCRTC failed (DRM master busy?)".to_string());
            }

            let blitter = Blitter::new(width as usize, height as usize, bufs[0].pitch as usize, layout);

            return Ok(Self {
                fd,
                connector_id: conn_id,
                crtc_id,
                mode,
                saved_crtc,
                bufs,
                front: 0,
                blitter,
                width: width as usize,
                height: height as usize,
                alive: true,
            });
        }

        Err("no connected HDMI connector on DRM card".to_string())
    }

    pub fn present(&mut self, rgb565: &[u16; SCREEN_W * SCREEN_H]) -> bool {
        if !self.alive {
            return false;
        }
        let back = self.front ^ 1;
        let buf = &self.bufs[back];
        self.blitter.blit_rgb565(rgb565, buf.ptr, buf.size);

        let mut conn = self.connector_id;
        let mut crtc = DrmModeCrtc {
            set_connectors_ptr: (&mut conn as *mut u32) as u64,
            count_connectors: 1,
            crtc_id: self.crtc_id,
            fb_id: buf.fb_id,
            x: 0,
            y: 0,
            gamma_size: 0,
            mode_valid: 1,
            mode: self.mode,
        };
        if unsafe { ioctl(self.fd, DRM_IOCTL_MODE_SETCRTC, &mut crtc) } < 0 {
            self.alive = false;
            return false;
        }
        let mut dirty = DrmModeFbDirtyCmd {
            fb_id: buf.fb_id,
            ..Default::default()
        };
        unsafe { ioctl(self.fd, DRM_IOCTL_MODE_DIRTYFB, &mut dirty) };

        self.front = back;
        true
    }
}

impl Drop for DrmDisplay {
    fn drop(&mut self) {
        if let Some(mut saved) = self.saved_crtc {
            if saved.fb_id != 0 && saved.mode_valid != 0 {
                let mut conn = self.connector_id;
                saved.set_connectors_ptr = (&mut conn as *mut u32) as u64;
                saved.count_connectors = 1;
                unsafe { ioctl(self.fd, DRM_IOCTL_MODE_SETCRTC, &mut saved) };
            }
        }
        self.bufs[0].destroy(self.fd);
        self.bufs[1].destroy(self.fd);
        unsafe { close(self.fd) };
    }
}

pub fn is_hdmi_connector(connector_type: u32) -> bool {
    connector_type == DRM_MODE_CONNECTOR_HDMIA || connector_type == DRM_MODE_CONNECTOR_HDMIB
}

fn find_crtc_for_connector(
    fd: i32,
    current_encoder: u32,
    conn_encoders: &[u32],
    crtcs: &[u32],
) -> Option<u32> {
    if current_encoder != 0 {
        let mut enc = DrmModeGetEncoder {
            encoder_id: current_encoder,
            ..Default::default()
        };
        if unsafe { ioctl(fd, DRM_IOCTL_MODE_GETENCODER, &mut enc) } == 0 && enc.crtc_id != 0 {
            return Some(enc.crtc_id);
        }
    }
    for &enc_id in conn_encoders {
        let mut enc = DrmModeGetEncoder {
            encoder_id: enc_id,
            ..Default::default()
        };
        if unsafe { ioctl(fd, DRM_IOCTL_MODE_GETENCODER, &mut enc) } == 0 {
            if enc.crtc_id != 0 {
                return Some(enc.crtc_id);
            }
            for (bit, &crtc_id) in crtcs.iter().enumerate() {
                if bit < 32 && (enc.possible_crtcs & (1 << bit)) != 0 {
                    return Some(crtc_id);
                }
            }
        }
    }
    crtcs.first().copied()
}

/// Select the ideal HDMI display mode for 60 FPS FPV flight on CardputerZero (Pi CM0).
///
/// Prefers 1280x720@>=50Hz (exact 4x integer scale of 320x170 -> 1280x680 with 2.25x less
/// memory bandwidth than 1080p), then any >=50Hz preferred mode, then preferred mode,
/// then highest-refresh mode <=1080p.
pub fn select_best_mode(modes: &[DrmModeModeInfo]) -> DrmModeModeInfo {
    if let Some(m) = modes
        .iter()
        .find(|m| m.hdisplay == 1280 && m.vdisplay == 720 && m.vrefresh >= 50)
    {
        return *m;
    }
    if let Some(m) = modes
        .iter()
        .find(|m| (m.type_ & DRM_MODE_TYPE_PREFERRED) != 0 && m.vrefresh >= 50)
    {
        return *m;
    }
    if let Some(m) = modes
        .iter()
        .find(|m| (m.type_ & DRM_MODE_TYPE_PREFERRED) != 0)
    {
        return *m;
    }
    if let Some(m) = modes
        .iter()
        .filter(|m| m.hdisplay <= 1920 && m.vdisplay <= 1080)
        .max_by_key(|m| (m.vrefresh, m.hdisplay as u32 * m.vdisplay as u32))
    {
        return *m;
    }
    modes[0]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_mode(w: u16, h: u16, hz: u32, preferred: bool) -> DrmModeModeInfo {
        DrmModeModeInfo {
            hdisplay: w,
            vdisplay: h,
            vrefresh: hz,
            type_: if preferred { DRM_MODE_TYPE_PREFERRED } else { 0 },
            ..Default::default()
        }
    }

    #[test]
    fn test_drm_mode_selection_prefers_720p60_over_1080p30() {
        let modes = [
            make_mode(1920, 1080, 30, true),
            make_mode(1280, 720, 60, false),
            make_mode(640, 480, 60, false),
        ];
        let chosen = select_best_mode(&modes);
        assert_eq!((chosen.hdisplay, chosen.vdisplay, chosen.vrefresh), (1280, 720, 60));
    }

    #[test]
    fn test_drm_mode_selection_falls_back_to_preferred_when_no_720p60() {
        let modes = [
            make_mode(1920, 1080, 60, true),
            make_mode(800, 600, 60, false),
        ];
        let chosen = select_best_mode(&modes);
        assert_eq!((chosen.hdisplay, chosen.vdisplay, chosen.vrefresh), (1920, 1080, 60));
    }

    #[test]
    fn test_hdmi_connector_type_filter() {
        assert!(is_hdmi_connector(DRM_MODE_CONNECTOR_HDMIA));
        assert!(is_hdmi_connector(DRM_MODE_CONNECTOR_HDMIB));
        assert!(!is_hdmi_connector(1)); // VGA
        assert!(!is_hdmi_connector(16)); // DSI
    }
}
