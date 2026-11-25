//! Zero-dependency Linux `/dev/hidraw*` sidecar parser for recovering HID axes
//! silently dropped by the kernel `hid-input` driver.
//!
//! Specifically solves the ExpressLRS BLE Joystick (`ESP32-BLE-Gamepad`) Linux bug
//! (ExpressLRS issues #2136 & #3604):
//! - EdgeTX USB Classic exposes CH7 as `Slider` (`0x36` -> `ABS_THROTTLE` = 6) and
//!   CH8 as `Dial` (`0x37` -> `ABS_RUDDER` = 7, displayed as `A7+`).
//! - ExpressLRS BLE Joystick exposes BOTH CH7 and CH8 with the duplicate usage
//!   `Slider` (`0x36`). Linux `hid-input` maps the first `0x36` to `ABS_THROTTLE` (6)
//!   and drops the second `0x36` (CH8 / AUX 4 / SD switch) from `/dev/input/event*`,
//!   even though every BLE HID packet still carries CH8 intact on `/dev/hidraw*`.
//!
//! When a controller's HID Report Descriptor contains duplicate or unmapped Generic
//! Desktop absolute axes (`0x30..=0x38`), this sidecar opens the sibling `/dev/hidraw*`
//! node alongside `evdev` and maps the dropped second `Slider` (CH8) directly to
//! `ABS_RUDDER` (`7` = `A7+`) so AUX bindings work identically over USB and Bluetooth.

use crate::input::AxisCal;
use crate::linux_sys::*;
use std::ffi::CString;
use std::fs;
use std::path::Path;

const USAGE_PAGE_GENERIC_DESKTOP: u16 = 0x01;
const USAGE_GD_X: u16 = 0x30;
const USAGE_GD_SLIDER: u16 = 0x36;
const USAGE_GD_WHEEL: u16 = 0x38;

/// A single Generic Desktop absolute axis field parsed from a HID Report Descriptor.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct HidAxisField {
    pub report_id: u8,
    /// Bit offset within the report payload (excluding the optional 1-byte Report ID prefix).
    pub bit_offset: usize,
    pub bit_size: usize,
    pub logical_min: i32,
    pub logical_max: i32,
    /// Generic Desktop usage ID (`0x30..=0x38`).
    pub usage_id: u16,
    /// 0-based occurrence counter for this `usage_id` within the same report.
    pub occurrence: u8,
}

#[derive(Clone, Debug, Default)]
pub struct ParsedHidDescriptor {
    pub has_report_ids: bool,
    pub axes: Vec<HidAxisField>,
}

/// A recovered HID axis that is missing from Linux `evdev` and injected from `/dev/hidraw*`.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct RecoveredHidAxis {
    pub evdev_code: usize,
    pub report_id: u8,
    pub bit_offset: usize,
    pub bit_size: usize,
    pub logical_min: i32,
    pub logical_max: i32,
}

/// Live `/dev/hidraw*` sidecar attached to a `GamepadDevice` background reader thread.
pub struct HidrawSidecar {
    pub fd: i32,
    pub has_report_ids: bool,
    pub axes: Vec<RecoveredHidAxis>,
    /// Bitmask (`1u64 << code`) of `axes[]` slots driven by this hidraw sidecar.
    pub axis_mask: u64,
}

impl HidrawSidecar {
    /// Decode one raw HID input report and update all recovered axes in `target_axes`.
    /// Returns `true` if at least one recovered axis matched the report ID and was updated.
    pub fn apply_report(&self, report: &[u8], target_axes: &mut [AxisCal; ABS_MAX + 1]) -> bool {
        let mut updated = false;
        for ax in &self.axes {
            if let Some(val) = extract_field_value(report, self.has_report_ids, ax) {
                let slot = &mut target_axes[ax.evdev_code];
                slot.present = true;
                slot.min = ax.logical_min;
                slot.max = ax.logical_max;
                slot.value = val;
                updated = true;
            }
        }
        updated
    }
}

#[derive(Copy, Clone, Debug, Default)]
struct GlobalState {
    usage_page: u16,
    logical_min: i32,
    logical_max_u: u32,
    logical_max_s: i32,
    report_size: usize,
    report_id: u8,
    report_count: usize,
}

impl GlobalState {
    #[inline]
    fn effective_logical_max(&self) -> i32 {
        // Per HID 1.11 section 6.2.2.7: if Logical Minimum >= 0 and Logical Maximum has
        // its sign bit set in a 1- or 2-byte item (e.g. 0x26 0xFF 0xFF = 65535), treat
        // Logical Maximum as unsigned.
        if self.logical_min >= 0 && self.logical_max_s < 0 {
            self.logical_max_u.min(i32::MAX as u32) as i32
        } else {
            self.logical_max_s
        }
    }
}

#[inline]
fn read_le_unsigned(bytes: &[u8]) -> u32 {
    let mut v = 0u32;
    for (i, &b) in bytes.iter().take(4).enumerate() {
        v |= (b as u32) << (i * 8);
    }
    v
}

#[inline]
fn read_le_signed(bytes: &[u8]) -> i32 {
    match bytes.len() {
        0 => 0,
        1 => (bytes[0] as i8) as i32,
        2 => i16::from_le_bytes([bytes[0], bytes[1]]) as i32,
        4 => i32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]),
        _ => read_le_unsigned(bytes) as i32,
    }
}

/// Parse all Generic Desktop absolute axis fields (`0x30..=0x38`) from a raw HID Report Descriptor.
pub fn parse_hid_descriptor(desc: &[u8]) -> ParsedHidDescriptor {
    let mut out = ParsedHidDescriptor::default();
    let mut global = GlobalState::default();
    let mut stack: Vec<GlobalState> = Vec::new();

    let mut local_usages: Vec<u32> = Vec::new();
    let mut local_usage_min: Option<u32> = None;
    let mut local_usage_max: Option<u32> = None;

    // Bit offset tracked independently per Report ID
    let mut report_bits: Vec<(u8, usize)> = Vec::new();
    // Occurrence counter per (report_id, usage_id - 0x30)
    let mut usage_counts: Vec<(u8, [u8; 9])> = Vec::new();

    let mut i = 0usize;
    while i < desc.len() {
        let prefix = desc[i];
        i += 1;

        // Long item (0xFE)
        if prefix == 0xFE {
            if i >= desc.len() {
                break;
            }
            let data_len = desc[i] as usize;
            i = i.saturating_add(2 + data_len);
            continue;
        }

        let size_code = prefix & 0x03;
        let data_len = if size_code == 3 { 4 } else { size_code as usize };
        let item_type = (prefix >> 2) & 0x03;
        let tag = (prefix >> 4) & 0x0F;

        if i + data_len > desc.len() {
            break;
        }
        let data = &desc[i..i + data_len];
        i += data_len;

        match item_type {
            // Global items
            1 => match tag {
                0 => global.usage_page = read_le_unsigned(data) as u16,
                1 => global.logical_min = read_le_signed(data),
                2 => {
                    global.logical_max_u = read_le_unsigned(data);
                    global.logical_max_s = read_le_signed(data);
                }
                7 => global.report_size = read_le_unsigned(data) as usize,
                8 => {
                    global.report_id = read_le_unsigned(data) as u8;
                    if global.report_id != 0 {
                        out.has_report_ids = true;
                    }
                }
                9 => global.report_count = read_le_unsigned(data) as usize,
                10 => {
                    if stack.len() < 16 {
                        stack.push(global);
                    }
                }
                11 => {
                    if let Some(prev) = stack.pop() {
                        global = prev;
                    }
                }
                _ => {}
            },
            // Local items
            2 => match tag {
                0 => {
                    let u = if data_len <= 2 {
                        ((global.usage_page as u32) << 16) | (read_le_unsigned(data) & 0xFFFF)
                    } else {
                        read_le_unsigned(data)
                    };
                    local_usages.push(u);
                }
                1 => {
                    let u = if data_len <= 2 {
                        ((global.usage_page as u32) << 16) | (read_le_unsigned(data) & 0xFFFF)
                    } else {
                        read_le_unsigned(data)
                    };
                    local_usage_min = Some(u);
                }
                2 => {
                    let u = if data_len <= 2 {
                        ((global.usage_page as u32) << 16) | (read_le_unsigned(data) & 0xFFFF)
                    } else {
                        read_le_unsigned(data)
                    };
                    local_usage_max = Some(u);
                }
                _ => {}
            },
            // Main items
            0 => {
                if tag == 8 {
                    // Input item
                    let flags = read_le_unsigned(data);
                    let is_const = (flags & 0x01) != 0;
                    let is_var = (flags & 0x02) != 0;
                    let is_rel = (flags & 0x04) != 0;

                    let bit_off_entry = if let Some(entry) = report_bits.iter_mut().find(|(id, _)| *id == global.report_id) {
                        entry
                    } else {
                        report_bits.push((global.report_id, 0));
                        report_bits.last_mut().unwrap()
                    };

                    let counts_entry = if let Some(entry) = usage_counts.iter_mut().find(|(id, _)| *id == global.report_id) {
                        entry
                    } else {
                        usage_counts.push((global.report_id, [0u8; 9]));
                        usage_counts.last_mut().unwrap()
                    };

                    let logical_max = global.effective_logical_max();

                    for k in 0..global.report_count {
                        let full_usage = if !local_usages.is_empty() {
                            local_usages[k.min(local_usages.len() - 1)]
                        } else if let (Some(umin), Some(umax)) = (local_usage_min, local_usage_max) {
                            umin.saturating_add(k as u32).min(umax)
                        } else {
                            0
                        };

                        if !is_const && is_var && !is_rel && global.report_size > 0 && global.report_size <= 32 {
                            let page = (full_usage >> 16) as u16;
                            let usage_id = (full_usage & 0xFFFF) as u16;
                            if page == USAGE_PAGE_GENERIC_DESKTOP
                                && (USAGE_GD_X..=USAGE_GD_WHEEL).contains(&usage_id)
                            {
                                let idx = (usage_id - USAGE_GD_X) as usize;
                                let occ = counts_entry.1[idx];
                                counts_entry.1[idx] = occ.saturating_add(1);
                                out.axes.push(HidAxisField {
                                    report_id: global.report_id,
                                    bit_offset: bit_off_entry.1,
                                    bit_size: global.report_size,
                                    logical_min: global.logical_min,
                                    logical_max,
                                    usage_id,
                                    occurrence: occ,
                                });
                            }
                        }

                        bit_off_entry.1 = bit_off_entry.1.saturating_add(global.report_size);
                    }
                }

                // All Main items clear the local item state table per HID 1.11 section 6.2.2.8.
                local_usages.clear();
                local_usage_min = None;
                local_usage_max = None;
            }
            _ => {}
        }
    }

    out
}

/// Determine which HID axes from `parsed` were dropped or unmapped by Linux `evdev`
/// and assign them free evdev ABS codes.
///
/// Crucially, ExpressLRS BLE Joystick's duplicate 2nd `Slider` (`0x36`, CH8 / AUX 4 / SD)
/// is mapped to `ABS_RUDDER` (`7` = `A7+`) whenever `ABS_RUDDER` is not already claimed by
/// `evdev`, matching EdgeTX USB's `Dial` (`0x37` -> `ABS_RUDDER` = `A7+`) 1-to-1!
pub fn plan_recovered_axes(
    parsed: &ParsedHidDescriptor,
    evdev_axes: &[AxisCal; ABS_MAX + 1],
) -> Vec<RecoveredHidAxis> {
    let mut claimed = [false; ABS_MAX + 1];
    for code in 0..=ABS_MAX {
        if evdev_axes[code].present {
            claimed[code] = true;
        }
    }
    // Never steal hat axes
    claimed[ABS_HAT0X as usize] = true;
    claimed[ABS_HAT0Y as usize] = true;

    let mut recovered = Vec::new();

    for field in &parsed.axes {
        if field.logical_max <= field.logical_min || field.bit_size == 0 || field.bit_size > 32 {
            continue;
        }

        let base_code = (field.usage_id & 0x0F) as usize;
        if field.occurrence == 0 {
            // Primary occurrence: only recover if Linux evdev somehow failed to expose it.
            if base_code <= ABS_MAX && !claimed[base_code] {
                claimed[base_code] = true;
                recovered.push(RecoveredHidAxis {
                    evdev_code: base_code,
                    report_id: field.report_id,
                    bit_offset: field.bit_offset,
                    bit_size: field.bit_size,
                    logical_min: field.logical_min,
                    logical_max: field.logical_max,
                });
            }
            continue;
        }

        // Duplicate occurrence (e.g. ExpressLRS BLE's 2nd `USAGE (Slider)` 0x36 = CH8 / SD).
        // Prefer `ABS_RUDDER` (7) for the 2nd Slider so Bluetooth matches EdgeTX USB (`A7+`).
        let target_code = if field.usage_id == USAGE_GD_SLIDER
            && field.occurrence == 1
            && !claimed[ABS_RUDDER as usize]
        {
            Some(ABS_RUDDER as usize)
        } else {
            ((ABS_MISC as usize)..=ABS_MAX).find(|&c| !claimed[c])
        };

        if let Some(code) = target_code {
            claimed[code] = true;
            recovered.push(RecoveredHidAxis {
                evdev_code: code,
                report_id: field.report_id,
                bit_offset: field.bit_offset,
                bit_size: field.bit_size,
                logical_min: field.logical_min,
                logical_max: field.logical_max,
            });
        }
    }

    recovered
}

/// Extract a signed or unsigned little-endian bitfield from a raw HID report.
pub fn extract_field_value(
    report: &[u8],
    has_report_ids: bool,
    axis: &RecoveredHidAxis,
) -> Option<i32> {
    let payload = if has_report_ids {
        if report.is_empty() || report[0] != axis.report_id {
            return None;
        }
        &report[1..]
    } else {
        report
    };

    let end_bit = axis.bit_offset.checked_add(axis.bit_size)?;
    if end_bit > payload.len() * 8 || axis.bit_size == 0 || axis.bit_size > 32 {
        return None;
    }

    let mut raw = 0u32;
    for b in 0..axis.bit_size {
        let bit_idx = axis.bit_offset + b;
        let byte = payload[bit_idx / 8];
        let bit = ((byte >> (bit_idx % 8)) & 1) as u32;
        raw |= bit << b;
    }

    let val = if axis.logical_min < 0 && axis.bit_size < 32 {
        let sign_bit = 1u32 << (axis.bit_size - 1);
        if (raw & sign_bit) != 0 {
            let mask = !((1u32 << axis.bit_size) - 1);
            (raw | mask) as i32
        } else {
            raw as i32
        }
    } else {
        raw as i32
    };

    Some(val)
}

/// Locate the sibling `/dev/hidraw*` node corresponding to a Linux `/dev/input/eventN` path.
pub fn find_hidraw_for_event(event_path: &str) -> Option<String> {
    let event_name = Path::new(event_path).file_name()?.to_str()?;
    if !event_name.starts_with("event") {
        return None;
    }

    // Fast path: `/sys/class/input/eventN/device/device/hidraw/hidraw*`
    let direct_dir = format!("/sys/class/input/{event_name}/device/device/hidraw");
    if let Ok(entries) = fs::read_dir(&direct_dir) {
        let mut names: Vec<String> = entries
            .flatten()
            .filter_map(|e| {
                let n = e.file_name().to_string_lossy().to_string();
                if n.starts_with("hidraw") { Some(n) } else { None }
            })
            .collect();
        names.sort();
        if let Some(first) = names.into_iter().next() {
            return Some(format!("/dev/{first}"));
        }
    }

    // Fallback: scan `/sys/class/hidraw/hidraw*/device/input/input*/eventN`
    let hidraw_root = fs::read_dir("/sys/class/hidraw").ok()?;
    for entry in hidraw_root.flatten() {
        let hr_name = entry.file_name().to_string_lossy().to_string();
        if !hr_name.starts_with("hidraw") {
            continue;
        }
        let input_dir = entry.path().join("device").join("input");
        let Ok(inputs) = fs::read_dir(&input_dir) else { continue };
        for inp in inputs.flatten() {
            if inp.path().join(event_name).exists() {
                return Some(format!("/dev/{hr_name}"));
            }
        }
    }

    None
}

/// Probe the sibling `/dev/hidraw*` for `event_path` and open a sidecar ONLY if the HID
/// Report Descriptor contains duplicate/missing axes that Linux `evdev` dropped.
///
/// For normal USB radios and gamepads whose axes are all already exposed via `evdev`,
/// this closes the fd immediately and returns `None` (zero runtime overhead).
pub fn probe_hidraw_sidecar(
    event_path: &str,
    evdev_axes: &[AxisCal; ABS_MAX + 1],
) -> Option<HidrawSidecar> {
    let hidraw_path = find_hidraw_for_event(event_path)?;
    let cpath = CString::new(hidraw_path.as_str()).ok()?;
    let fd = unsafe { open(cpath.as_ptr(), O_RDONLY | O_NONBLOCK) };
    if fd < 0 {
        return None;
    }

    let mut desc_size: i32 = 0;
    if unsafe { ioctl(fd, hidiocgrdescsize(), &mut desc_size) } < 0
        || desc_size <= 0
        || (desc_size as usize) > HID_MAX_DESCRIPTOR_SIZE
    {
        unsafe { close(fd) };
        return None;
    }

    let mut desc = HidrawReportDescriptor::default();
    desc.size = desc_size as u32;
    if unsafe { ioctl(fd, hidiocgrdesc(), &mut desc) } < 0 {
        unsafe { close(fd) };
        return None;
    }

    let len = (desc.size as usize).min(HID_MAX_DESCRIPTOR_SIZE);
    let parsed = parse_hid_descriptor(&desc.value[..len]);
    let recovered = plan_recovered_axes(&parsed, evdev_axes);
    if recovered.is_empty() {
        unsafe { close(fd) };
        return None;
    }

    let mut axis_mask = 0u64;
    for ax in &recovered {
        if ax.evdev_code < 64 {
            axis_mask |= 1u64 << ax.evdev_code;
        }
    }

    Some(HidrawSidecar {
        fd,
        has_report_ids: parsed.has_report_ids,
        axes: recovered,
        axis_mask,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Exact HID Report Descriptor layout emitted by ExpressLRS BLE Joystick (`ESP32-BLE-Gamepad`):
    /// Report ID 3, 16 buttons, 8×16-bit axes with DUPLICATE `USAGE (Slider)` (`0x36`) for CH7 & CH8,
    /// plus 1-byte hat switch.
    fn elrs_ble_descriptor_single_input() -> Vec<u8> {
        vec![
            0x05, 0x01,       // USAGE_PAGE (Generic Desktop)
            0x09, 0x05,       // USAGE (Game Pad)
            0xA1, 0x01,       // COLLECTION (Application)
            0x85, 0x03,       //   REPORT_ID (3)
            // 16 buttons (2 bytes)
            0x05, 0x09,       //   USAGE_PAGE (Button)
            0x19, 0x01,       //   USAGE_MINIMUM (Button 1)
            0x29, 0x10,       //   USAGE_MAXIMUM (Button 16)
            0x15, 0x00,       //   LOGICAL_MINIMUM (0)
            0x25, 0x01,       //   LOGICAL_MAXIMUM (1)
            0x75, 0x01,       //   REPORT_SIZE (1)
            0x95, 0x10,       //   REPORT_COUNT (16)
            0x81, 0x02,       //   INPUT (Data,Var,Abs)
            // 8 axes × 16 bits (16 bytes), with duplicate 0x36 Slider for CH7 & CH8
            0x05, 0x01,       //   USAGE_PAGE (Generic Desktop)
            0x09, 0x01,       //   USAGE (Pointer)
            0xA1, 0x00,       //   COLLECTION (Physical)
            0x15, 0x00,       //     LOGICAL_MINIMUM (0)
            0x26, 0xFF, 0x7F, //     LOGICAL_MAXIMUM (32767)
            0x75, 0x10,       //     REPORT_SIZE (16)
            0x95, 0x08,       //     REPORT_COUNT (8)
            0x09, 0x30,       //     USAGE (X)
            0x09, 0x31,       //     USAGE (Y)
            0x09, 0x32,       //     USAGE (Z)
            0x09, 0x35,       //     USAGE (Rz)
            0x09, 0x33,       //     USAGE (Rx)
            0x09, 0x34,       //     USAGE (Ry)
            0x09, 0x36,       //     USAGE (Slider)  <-- CH7
            0x09, 0x36,       //     USAGE (Slider)  <-- CH8 (duplicate!)
            0x81, 0x02,       //     INPUT (Data,Var,Abs)
            0xC0,             //   END_COLLECTION
            // Hat switch (1 byte)
            0x05, 0x01,       //   USAGE_PAGE (Generic Desktop)
            0x09, 0x39,       //   USAGE (Hat switch)
            0x15, 0x01,       //   LOGICAL_MINIMUM (1)
            0x25, 0x08,       //   LOGICAL_MAXIMUM (8)
            0x75, 0x08,       //   REPORT_SIZE (8)
            0x95, 0x01,       //   REPORT_COUNT (1)
            0x81, 0x42,       //   INPUT (Data,Var,Abs,Null)
            0xC0,             // END_COLLECTION
        ]
    }

    #[test]
    fn test_elrs_ble_recovers_dropped_ch8_slider_to_abs_rudder_a7() {
        let desc = elrs_ble_descriptor_single_input();
        let parsed = parse_hid_descriptor(&desc);
        assert!(parsed.has_report_ids);
        assert_eq!(parsed.axes.len(), 8);
        // CH7 = 1st Slider at bit 112 (byte 14 of payload, byte 15 of report)
        assert_eq!(parsed.axes[6].usage_id, USAGE_GD_SLIDER);
        assert_eq!(parsed.axes[6].occurrence, 0);
        assert_eq!(parsed.axes[6].bit_offset, 112);
        // CH8 = 2nd Slider at bit 128 (byte 16 of payload, byte 17 of report)
        assert_eq!(parsed.axes[7].usage_id, USAGE_GD_SLIDER);
        assert_eq!(parsed.axes[7].occurrence, 1);
        assert_eq!(parsed.axes[7].bit_offset, 128);

        // Simulate Linux evdev for ELRS BLE: axes 0..=6 present, axis 7 (ABS_RUDDER) dropped!
        let mut evdev_axes = [AxisCal::default(); ABS_MAX + 1];
        for c in 0..=6 {
            evdev_axes[c] = AxisCal {
                present: true,
                min: 0,
                max: 32767,
                value: 16384,
            };
        }

        let recovered = plan_recovered_axes(&parsed, &evdev_axes);
        assert_eq!(recovered.len(), 1);
        assert_eq!(recovered[0].evdev_code, ABS_RUDDER as usize); // Axis 7 -> A7+!
        assert_eq!(recovered[0].report_id, 3);
        assert_eq!(recovered[0].bit_offset, 128);
        assert_eq!(recovered[0].logical_min, 0);
        assert_eq!(recovered[0].logical_max, 32767);

        // Real captured 20-byte ELRS BLE HID report (ExpressLRS #2136):
        // ReportID=3, Buttons=0x0000, X=16393, Y=16393, Z=19, Rz=19, Rx=19, Ry=16393,
        // Slider1=16613 (0x40E5), Slider2=16393 (0x4009), Hat=0
        let report_mid: [u8; 20] = [
            0x03, 0x00, 0x00, 0x09, 0x40, 0x09, 0x40, 0x13, 0x00, 0x13, 0x00, 0x13, 0x00, 0x09,
            0x40, 0xE5, 0x40, 0x09, 0x40, 0x00,
        ];
        assert_eq!(extract_field_value(&report_mid, true, &recovered[0]), Some(16393));

        // Now simulate SD button flipped HIGH (CH8 = 32767 = 0x7FFF) vs LOW (CH8 = 0)
        let mut report_sd_high = report_mid;
        report_sd_high[17] = 0xFF;
        report_sd_high[18] = 0x7F;
        let mut report_sd_low = report_mid;
        report_sd_low[17] = 0x00;
        report_sd_low[18] = 0x00;

        let sidecar = HidrawSidecar {
            fd: -1,
            has_report_ids: true,
            axes: recovered,
            axis_mask: 1u64 << ABS_RUDDER,
        };

        assert!(sidecar.apply_report(&report_sd_low, &mut evdev_axes));
        assert!(evdev_axes[ABS_RUDDER as usize].present);
        assert!(evdev_axes[ABS_RUDDER as usize].raw_norm() < -0.95);

        assert!(sidecar.apply_report(&report_sd_high, &mut evdev_axes));
        assert!(evdev_axes[ABS_RUDDER as usize].raw_norm() > 0.95);
    }

    #[test]
    fn test_elrs_4_1_separate_logical_collections_still_recovers_ch8() {
        // ExpressLRS 4.1+ / PR #3755 wraps Slider 1 and Slider 2 in separate Logical collections,
        // both still using usage 0x36 (Slider).
        let desc: Vec<u8> = vec![
            0x05, 0x01, 0x09, 0x05, 0xA1, 0x01, 0x85, 0x03,
            // 16 buttons
            0x05, 0x09, 0x19, 0x01, 0x29, 0x10, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x10, 0x81, 0x02,
            // 6 main axes
            0x05, 0x01, 0x16, 0x01, 0x80, 0x26, 0xFF, 0x7F, 0x75, 0x10, 0x95, 0x06,
            0x09, 0x30, 0x09, 0x31, 0x09, 0x32, 0x09, 0x33, 0x09, 0x34, 0x09, 0x35, 0x81, 0x02,
            // Logical collection 1: Slider 1
            0xA1, 0x02, 0x09, 0x36, 0x95, 0x01, 0x81, 0x02, 0xC0,
            // Logical collection 2: Slider 2 (CH8)
            0xA1, 0x02, 0x09, 0x36, 0x95, 0x01, 0x81, 0x02, 0xC0,
            0xC0,
        ];

        let parsed = parse_hid_descriptor(&desc);
        assert_eq!(parsed.axes.len(), 8);
        assert_eq!(parsed.axes[7].logical_min, -32767);
        assert_eq!(parsed.axes[7].logical_max, 32767);
        assert_eq!(parsed.axes[7].occurrence, 1);

        let mut evdev_axes = [AxisCal::default(); ABS_MAX + 1];
        for c in 0..=6 {
            evdev_axes[c] = AxisCal { present: true, min: -32767, max: 32767, value: 0 };
        }
        let recovered = plan_recovered_axes(&parsed, &evdev_axes);
        assert_eq!(recovered.len(), 1);
        assert_eq!(recovered[0].evdev_code, ABS_RUDDER as usize);

        // Signed -32767 (0x8001) at bytes 17..=18
        let mut report = [0u8; 19];
        report[0] = 3;
        report[17] = 0x01;
        report[18] = 0x80;
        assert_eq!(extract_field_value(&report, true, &recovered[0]), Some(-32767));
    }

    #[test]
    fn test_edgetx_usb_unique_slider_and_dial_needs_no_sidecar() {
        let desc: Vec<u8> = vec![
            0x05, 0x01, 0x09, 0x04, 0xA1, 0x01,
            0x15, 0x00, 0x26, 0xFF, 0x07, 0x75, 0x10, 0x95, 0x08,
            0x09, 0x30, 0x09, 0x31, 0x09, 0x32, 0x09, 0x33,
            0x09, 0x34, 0x09, 0x35, 0x09, 0x36, 0x09, 0x37,
            0x81, 0x02,
            0xC0,
        ];
        let parsed = parse_hid_descriptor(&desc);
        assert!(!parsed.has_report_ids);
        assert_eq!(parsed.axes.len(), 8);

        // EdgeTX USB already exposes all 8 axes (0..=7) in Linux evdev.
        let mut evdev_axes = [AxisCal::default(); ABS_MAX + 1];
        for c in 0..=7 {
            evdev_axes[c] = AxisCal { present: true, min: 0, max: 2047, value: 1024 };
        }
        let recovered = plan_recovered_axes(&parsed, &evdev_axes);
        assert!(recovered.is_empty(), "USB EdgeTX should not need hidraw recovery");
    }
}
