//! PowerSDR-style frequency readout: each digit tunes with the scroll wheel,
//! click upper/lower half to increment/decrement, or right-click to zero that
//! digit and everything to its right. Double-clicking a digit opens the same
//! type-in editor [`show_typed`] uses for compact layouts — the escape hatch
//! a per-digit nudge cannot offer once a source's valid ranges are not one
//! contiguous span (the RSR200's disjoint HF/VHF ranges are what exposed the
//! gap): jumping from deep in one band to deep in another crosses invalid
//! territory at every intermediate digit otherwise, and each rejected digit
//! change reverts before the next one can be reached. Compact layouts use
//! [`show_typed`] for the *resting* state too: the same dial, but any tap on
//! it opens the editor rather than nudging whichever place value was under
//! the finger — there is no room to hit one digit precisely on a phone.

use eframe::egui::{self, Color32, Label, RichText, Sense, Ui};
use sdroxide_types::WheelSettings;

/// The readout's design size, and the largest it is ever drawn at. Narrower
/// layouts fit smaller digits into the room they have — see `freq_module`.
pub const DIGIT_SIZE: f32 = 40.0;
/// The lit digits' amber, shared by both readouts and the type-in field.
///
/// The readout is the one number an operator reads from across the shack, so
/// it does not get to be low-contrast: on a light theme the same amber is
/// taken down to 6.3:1 rather than glowing at 1.7:1 on a white module face.
fn digit_ink() -> Color32 {
    if crate::theme::is_light() {
        Color32::from_rgb(0x8f, 0x50, 0x00)
    } else {
        Color32::from_rgb(255, 209, 66)
    }
}

/// Shows `hz` as a 10-digit tunable readout. Returns `Some(new_hz)` on change.
///
/// `wheel` supplies the operator's pointer preferences: `digit_wheel` turns the
/// per-digit scroll stepping off for anyone who would rather scroll the page,
/// and `invert` flips the wheel direction to match the panadapter.
///
/// `size` is the digit height; the caller fits it to the room it actually has
/// (see `freq_module`). [`DIGIT_SIZE`] is the design size a desktop uses.
///
/// `ink` overrides the lit digits' amber. `None` is the resting dial; the one
/// caller that passes a colour is the readout following the transmitter onto a
/// repeater's input, where the number on the front of the radio is no longer
/// the frequency it is listening on and has to say so.
pub fn show(
    ui: &mut Ui,
    id: egui::Id,
    hz: f64,
    wheel: WheelSettings,
    size: f32,
    ink: Option<Color32>,
) -> Option<f64> {
    // Mid-edit: the editor from a previous frame's double-click. Same shared
    // state and field as `show_typed`'s own, so there is one implementation
    // of "type a frequency" rather than two that could drift apart.
    let edit_id = id.with("edit");
    if let Some(text) = ui.data(|d| d.get_temp::<String>(edit_id)) {
        return edit_field(ui, id, edit_id, text, size);
    }

    let lit = ink.unwrap_or_else(digit_ink);
    let mut freq = hz.round().max(0.0) as i64;
    let orig = freq;
    let mut open_editor = false;

    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 1.0;
        for p in (0..10u32).rev() {
            if p < 9 && (p + 1) % 3 == 0 {
                ui.add(Label::new(
                    RichText::new(".").monospace().size(size).color(crate::theme::gray(110)),
                ));
            }

            let step = 10i64.pow(p);
            let digit = (freq / step) % 10;
            let leading_zero = p > 0 && freq < step;
            let color = if leading_zero { crate::theme::gray(70) } else { lit };

            let resp = ui
                .add(
                    Label::new(
                        RichText::new(format!("{digit}")).monospace().size(size).color(color),
                    )
                    .sense(Sense::click()),
                )
                .on_hover_cursor(egui::CursorIcon::ResizeVertical)
                .on_hover_text("Scroll or click to tune this digit — double-click to type a frequency");

            if resp.hovered() {
                ui.painter().hline(resp.rect.x_range(), resp.rect.bottom() - 1.0, (2.0, lit));
                let acc_id = id.with("acc").with(p);
                let mut acc = ui.data(|d| d.get_temp::<f32>(acc_id)).unwrap_or(0.0);
                let detents = if wheel.digit_wheel {
                    let n = crate::widgets::wheel_detents(ui, false, &mut acc);
                    if wheel.invert { -n } else { n }
                } else {
                    0.0
                };
                ui.data_mut(|d| d.insert_temp(acc_id, acc));
                freq = (freq + step * detents as i64).max(0);
            }

            // Double-click opens the type-in editor instead of nudging this
            // digit — checked ahead of the single-click bump below so a
            // double-click never also moves the dial by one step on its way
            // in (egui counts both clicks of a double-click as ordinary
            // `clicked()` events too).
            if resp.double_clicked() {
                open_editor = true;
            } else if resp.clicked()
                && let Some(pos) = resp.interact_pointer_pos()
            {
                if pos.y < resp.rect.center().y {
                    freq += step;
                } else {
                    freq = (freq - step).max(0);
                }
            }

            // Right-click: a quick way to reset the dial without walking every
            // digit down by hand. Zeros this digit and everything to its
            // right, so clicking the 5 in 7.090.054 gives 7.090.000 and
            // clicking the 4 gives 7.090.050.
            if resp.secondary_clicked() {
                freq = zero_from_digit(freq, step);
            }
        }
        ui.add(Label::new(RichText::new(" Hz").size(size * 0.3).color(crate::theme::gray(140))));
    });

    if open_editor {
        ui.data_mut(|d| {
            d.insert_temp(edit_id, format_mhz(hz));
            // Marks the editor's first frame, when it takes focus and selects
            // the prefill so typing replaces it — see `edit_field`.
            d.insert_temp(id.with("fresh"), true);
        });
        return None;
    }

    (freq != orig).then_some(freq as f64)
}

/// Floor `freq` to a multiple of `step * 10`: zeros the digit at `step`'s
/// place and every digit to its right, leaving everything above it alone.
fn zero_from_digit(freq: i64, step: i64) -> i64 {
    let modulus = step * 10;
    freq - freq % modulus
}

/// The readout for a compact layout: the same lit dial, but one click target.
///
/// Per-digit halves are mouse furniture. On the small screens the compact
/// tiers dress, a fingertip covers two or three digits, and a mis-tap tunes
/// the rig by whichever place value happened to be underneath — so here a tap
/// anywhere on the number opens a type-in field prefilled with the frequency
/// in MHz. Enter tunes, Escape or a tap elsewhere leaves the VFO alone.
/// Losing the per-digit targets is also what lets the compact boxes shrink
/// the digits below comfortable clicking size.
///
/// Returns `Some(new_hz)` when a typed frequency is committed. `ink` is
/// [`show`]'s.
pub fn show_typed(
    ui: &mut Ui,
    id: egui::Id,
    hz: f64,
    size: f32,
    ink: Option<Color32>,
) -> Option<f64> {
    let edit_id = id.with("edit");
    match ui.data(|d| d.get_temp::<String>(edit_id)) {
        Some(text) => edit_field(ui, id, edit_id, text, size),
        None => {
            show_dial(ui, id, edit_id, hz, size, ink);
            None
        }
    }
}

/// The dial in its resting state: the digits of [`show`], drawn without their
/// per-digit senses, behind one whole-row click target that opens the editor.
fn show_dial(
    ui: &mut Ui,
    id: egui::Id,
    edit_id: egui::Id,
    hz: f64,
    size: f32,
    ink: Option<Color32>,
) {
    let lit = ink.unwrap_or_else(digit_ink);
    let freq = hz.round().max(0.0) as i64;
    let row = ui
        .horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 1.0;
            for p in (0..10u32).rev() {
                if p < 9 && (p + 1) % 3 == 0 {
                    ui.add(Label::new(
                        RichText::new(".").monospace().size(size).color(crate::theme::gray(110)),
                    ));
                }
                let step = 10i64.pow(p);
                let digit = (freq / step) % 10;
                let leading_zero = p > 0 && freq < step;
                let color = if leading_zero { crate::theme::gray(70) } else { lit };
                ui.add(Label::new(
                    RichText::new(format!("{digit}")).monospace().size(size).color(color),
                ));
            }
            ui.add(Label::new(
                RichText::new(" Hz").size(size * 0.3).color(crate::theme::gray(140)),
            ));
        })
        .response;

    let resp = ui
        .interact(row.rect, id.with("dial"), Sense::click())
        .on_hover_cursor(egui::CursorIcon::Text)
        .on_hover_text("Tap to type a frequency");
    if resp.hovered() {
        ui.painter().hline(row.rect.x_range(), row.rect.bottom() - 1.0, (2.0, lit));
    }
    if resp.clicked() {
        ui.data_mut(|d| {
            d.insert_temp(edit_id, format_mhz(hz));
            // Marks the editor's first frame, when it takes focus and selects
            // the prefill so typing replaces it.
            d.insert_temp(id.with("fresh"), true);
        });
    }
}

/// The type-in field the dial turns into, in the dial's own font and ink.
fn edit_field(
    ui: &mut Ui,
    id: egui::Id,
    edit_id: egui::Id,
    mut text: String,
    size: f32,
) -> Option<f64> {
    let field_id = id.with("field");
    let mut out = None;
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 1.0;
        // Sized so the box the dial sits in does not move when it turns into
        // the editor: the field spans what the ten digits spanned.
        let digits_w = ui
            .painter()
            .layout_no_wrap(
                "0.000.000.000".to_owned(),
                egui::FontId::monospace(size),
                Color32::WHITE,
            )
            .size()
            .x;
        let resp = crate::chrome::field(
            ui,
            egui::TextEdit::singleline(&mut text)
                .id(field_id)
                .font(egui::FontId::monospace(size))
                .text_color(digit_ink())
                .margin(egui::Margin::ZERO)
                .desired_width(digits_w),
        );
        ui.add(Label::new(RichText::new(" MHz").size(size * 0.3).color(crate::theme::gray(140))));

        if ui.data_mut(|d| d.remove_temp::<bool>(id.with("fresh"))).is_some() {
            resp.request_focus();
            if let Some(mut state) = egui::TextEdit::load_state(ui.ctx(), field_id) {
                state.cursor.set_char_range(Some(egui::text::CCursorRange::two(
                    egui::text::CCursor::new(0),
                    egui::text::CCursor::new(text.chars().count()),
                )));
                state.store(ui.ctx(), field_id);
            }
        }

        // Enter, Escape and a tap elsewhere all drop focus and close the
        // editor; only Enter carrying a parsable number tunes anything.
        let done = resp.lost_focus();
        if done && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
            out = parse_mhz(&text);
        }
        if done {
            ui.data_mut(|d| d.remove_temp::<String>(edit_id));
        } else {
            ui.data_mut(|d| d.insert_temp(edit_id, text));
        }
    });
    out
}

/// `hz` as the MHz string the editor is prefilled with — "14.074", not
/// "14.074000", so the digits the operator is about to replace are the ones
/// that carry information.
fn format_mhz(hz: f64) -> String {
    let s = format!("{:.6}", hz.max(0.0) / 1e6);
    s.trim_end_matches('0').trim_end_matches('.').to_string()
}

/// A typed MHz value as Hz. Accepts the decimal comma half the world types.
/// `None` — nothing to tune to — for anything unparsable or out of all range.
fn parse_mhz(s: &str) -> Option<f64> {
    let mhz: f64 = s.trim().replace(',', ".").parse().ok()?;
    let hz = (mhz * 1e6).round();
    (hz.is_finite() && (0.0..1e10).contains(&hz)).then_some(hz)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn typed_frequencies_parse_as_mhz() {
        assert_eq!(parse_mhz("14.074"), Some(14_074_000.0));
        assert_eq!(parse_mhz(" 7,1 "), Some(7_100_000.0), "decimal comma");
        assert_eq!(parse_mhz("0.4776"), Some(477_600.0));
        assert_eq!(parse_mhz(""), None);
        assert_eq!(parse_mhz("ten"), None);
        assert_eq!(parse_mhz("-7.1"), None, "a negative frequency is a typo");
        assert_eq!(parse_mhz("1e7"), None, "10 THz is out of every range");
    }

    #[test]
    fn the_prefill_round_trips_and_sheds_trailing_zeros() {
        assert_eq!(format_mhz(14_074_000.0), "14.074");
        assert_eq!(format_mhz(7_000_000.0), "7");
        assert_eq!(format_mhz(477_612.0), "0.477612");
        assert_eq!(format_mhz(0.0), "0");
        for hz in [14_074_000.0, 7_000_000.0, 477_612.0] {
            assert_eq!(parse_mhz(&format_mhz(hz)), Some(hz), "prefill for {hz} did not round-trip");
        }
    }

    #[test]
    fn right_click_zeros_the_digit_and_everything_right_of_it() {
        // 7,090,054 Hz: right-clicking the tens digit ("5", step 10) clears
        // the tens and units, right-clicking the units digit ("4", step 1)
        // clears only itself.
        assert_eq!(zero_from_digit(7_090_054, 10), 7_090_000);
        assert_eq!(zero_from_digit(7_090_054, 1), 7_090_050);
        // A higher digit works the same way: zeroing the ten-thousands digit
        // ("9") clears everything below it too.
        assert_eq!(zero_from_digit(7_090_054, 10_000), 7_000_000);
        // Zeroing a digit that is already zero still clears what's to its
        // right.
        assert_eq!(zero_from_digit(7_090_054, 100), 7_090_000);
        // Zeroing a digit above the whole frequency just gives zero.
        assert_eq!(zero_from_digit(7_090_054, 10_000_000_000), 0);
    }
}
