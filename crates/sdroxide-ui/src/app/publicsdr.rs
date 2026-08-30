//! The "PUBLIC SDRS" window: browse the receivers other people have published
//! and open one as a radio.
//!
//! Built on the same bones as [`crate::app::spots`], because it is the same
//! problem — a long list from the internet, filter chips, a fuzzy search, and a
//! row that does something when it is clicked.
//!
//! # Where the list comes from
//!
//! Not from here. The window asks
//! [`sdroxide_types::DeviceProbe::PublicSdrs`], which is answered by the
//! machine the radio is attached to — the same lane the device enumerations
//! use. That is what makes this work in a browser: the web client has no HTTP
//! client at all, and could not read either directory across origins if it had.
//! It is also the right end to ask, because it is the end that will hold the
//! connection: a receiver this screen can reach is no use if the station
//! cannot.
//!
//! # Being a guest
//!
//! A receiver that cannot be used is shown greyed with the reason rather than
//! hidden. Two of them matter and neither is obvious from outside: a KiwiSDR
//! whose operator has not opened any channels to non-browser apps will refuse
//! sdroxide however many are free, and a receiver whose channels are all in use
//! is somebody else's for the moment. Being told which is far better than a
//! connection that fails for no visible reason.

use eframe::egui::{self, RichText};
use sdroxide_types::{Command, PublicSdrEntry, PublicSdrNetwork};

use crate::theme::ThemedScroll;
use crate::time::now_unix;

use crate::app::util::fmt_age;
use crate::app::{RadioTabRequest, SdroxideApp};

/// Rows drawn at once. The directories run to about eleven hundred receivers
/// between them and every row is a handful of laid-out labels; past this the
/// list is not a list any more, and the search box is the way through it.
const MAX_ROWS: usize = 300;

/// One receiver row: network badge, name, what it covers, how busy it is,
/// where it is, and the two ways to take it.
///
/// Columns are allocated rather than laid out by content, the way
/// [`crate::app::spots`]'s rows are: a directory is a table, and a name that
/// pushed the frequency column sideways would make it unreadable. The last one
/// takes whatever is left over so a wide window shows more of the place name
/// rather than more empty space.
///
/// Returns what the operator pressed, if anything.
fn entry_row(
    ui: &mut egui::Ui,
    e: &PublicSdrEntry,
    distance_km: Option<f64>,
) -> Option<PickAction> {
    let blocked = e.blocked_reason();
    let mut action = None;
    let dim = crate::theme::gray(if blocked.is_some() { 100 } else { 170 });
    let net_col = match e.network {
        PublicSdrNetwork::KiwiSdr => crate::theme::CYAN(),
        PublicSdrNetwork::SpyServer => crate::theme::PINK(),
    };
    /// What the buttons (or the refusal text) need on the right.
    const ACTIONS_W: f32 = 172.0;
    /// The five fixed columns ahead of the place, and the gaps between them.
    const FIXED_W: f32 = 60.0 + 176.0 + 80.0 + 46.0 + 4.0 * 6.0;
    /// The distance column, which is only drawn when a station grid is set.
    const DISTANCE_W: f32 = 56.0 + 6.0;
    /// Narrower than this and a place name is an ellipsis, which is worth less
    /// than the space. The row's tooltip has it either way.
    const PLACE_MIN_W: f32 = 74.0;

    egui::Frame::new()
        .fill(crate::theme::ROW_BG())
        .inner_margin(egui::Margin { left: 8, right: 6, top: 2, bottom: 2 })
        .show(ui, |ui| {
            let row_w = ui.available_width();
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = 6.0;
                let col = |ui: &mut egui::Ui, w: f32, lbl: egui::Label| {
                    let (rect, _) =
                        ui.allocate_exact_size(egui::vec2(w, 20.0), egui::Sense::hover());
                    ui.new_child(
                        egui::UiBuilder::new()
                            .max_rect(rect)
                            .layout(egui::Layout::left_to_right(egui::Align::Center)),
                    )
                    .add(lbl);
                };
                col(
                    ui,
                    60.0,
                    egui::Label::new(
                        RichText::new(e.network.label()).size(10.0).strong().color(net_col),
                    ),
                );
                let name_col = if blocked.is_some() {
                    crate::theme::gray(130)
                } else {
                    crate::theme::TEXT_STRONG()
                };
                col(
                    ui,
                    176.0,
                    egui::Label::new(RichText::new(&e.name).size(13.0).color(name_col)).truncate(),
                );
                col(
                    ui,
                    80.0,
                    egui::Label::new(RichText::new(e.range_label()).size(11.0).color(dim)),
                );
                // Users, and the one number that decides whether a receiver is
                // available at all.
                let busy = if e.max_users == 0 {
                    "—".to_string()
                } else {
                    format!("{}/{}", e.users, e.max_users)
                };
                let busy_col = match () {
                    _ if blocked.is_some() => crate::theme::ALERT(),
                    _ if e.max_users > 0 && e.users * 2 >= e.max_users => crate::theme::YELLOW(),
                    _ => crate::theme::GREEN(),
                };
                col(ui, 46.0, egui::Label::new(RichText::new(busy).size(11.0).color(busy_col)));
                // The distance stands in where the operator never wrote a
                // place, which on a SpyServer is most of them — it is the only
                // thing the directory knows about where that receiver is.
                // Its own column rather than appended to the place: a name
                // long enough to truncate would otherwise take the distance
                // with it, and on a SpyServer — where the operator usually
                // wrote no place at all — the distance is the only thing the
                // directory knows about where the receiver is.
                // Only where there is one to show: an operator who has not set
                // a grid would otherwise pay a column's width for a run of
                // blanks, and that width is the place name's.
                if let Some(km) = distance_km {
                    col(
                        ui,
                        56.0,
                        egui::Label::new(
                            RichText::new(if km >= 1000.0 {
                                format!("{:.1}k km", km / 1000.0)
                            } else {
                                format!("{km:.0} km")
                            })
                            .size(11.0)
                            .color(dim),
                        ),
                    );
                }
                // The one column that flexes, from the row's own width — which
                // has to be captured before anything is allocated out of it,
                // because `available_width` here describes what is left of the
                // frame rather than the row.
                //
                // On a narrow window it is dropped rather than squeezed: the
                // fixed columns come to more than a small window has, and a
                // place name rendered as a bare ellipsis is worth less than the
                // space it costs. Widen the window and it comes back.
                let used = FIXED_W + if distance_km.is_some() { DISTANCE_W } else { 0.0 };
                let place_w = (row_w - used - ACTIONS_W).min(340.0);
                if place_w >= PLACE_MIN_W {
                    col(
                        ui,
                        place_w,
                        egui::Label::new(RichText::new(&e.location).size(11.0).color(dim))
                            .truncate(),
                    );
                }

                ui.with_layout(
                    egui::Layout::right_to_left(egui::Align::Center),
                    |ui| match &blocked {
                        Some(why) => {
                            ui.add(
                                egui::Label::new(
                                    RichText::new(why).size(10.0).color(crate::theme::ALERT()),
                                )
                                .truncate(),
                            );
                        }
                        None => {
                            if crate::chrome::chip(ui, false, "+ TAB")
                                .on_hover_text("Open this receiver as a new radio, in its own tab")
                                .clicked()
                            {
                                action = Some(PickAction::NewRadio);
                            }
                            if crate::chrome::chip(ui, false, "USE")
                                .on_hover_text(
                                    "Point *this* radio at the receiver, replacing whatever \
                                     interface it is on now",
                                )
                                .clicked()
                            {
                                action = Some(PickAction::ThisRadio);
                            }
                        }
                    },
                );
            });
        })
        .response
        // Everything that did not earn a column of its own: the address to
        // connect to, the antenna, and what the receiver says it is.
        .on_hover_text(format!(
            "{}\n{}\nantenna: {}\n{}",
            e.address,
            e.device,
            if e.antenna.is_empty() { "not stated" } else { &e.antenna },
            match e.snr_db {
                Some(snr) => format!("noise-floor score {snr}"),
                None => format!("up to {:.0} kHz of I/Q", e.max_iq_rate / 1e3),
            },
        ));
    ui.add_space(1.0);
    action
}

/// What a row's buttons asked for.
#[derive(Clone, Copy, PartialEq)]
enum PickAction {
    NewRadio,
    ThisRadio,
}

impl SdroxideApp {
    /// Everything the search and the chips let through, ranked.
    fn public_sdr_rows<'a>(
        &self,
        entries: &'a [PublicSdrEntry],
        dial_hz: f64,
    ) -> Vec<(&'a PublicSdrEntry, i32)> {
        let query = self.public_sdr_search.trim();
        let visible: Vec<&PublicSdrEntry> = entries
            .iter()
            .filter(|e| {
                let net_on = match e.network {
                    PublicSdrNetwork::SpyServer => self.public_sdr_nets_shown[0],
                    PublicSdrNetwork::KiwiSdr => self.public_sdr_nets_shown[1],
                };
                net_on
                    && (!self.public_sdr_free_only || e.blocked_reason().is_none())
                    && (!self.public_sdr_in_band || e.covers(dial_hz))
            })
            .collect();
        let mut rows: Vec<(&PublicSdrEntry, i32)> = visible
            .iter()
            .filter_map(|e| crate::fuzzy::score_terms(&e.haystack(), query).map(|s| (*e, s)))
            .collect();
        if !query.is_empty() {
            rows.sort_by_key(|r| std::cmp::Reverse(r.1));
        }
        rows
    }

    /// Browse the public-SDR directories and open one as a radio.
    pub(in crate::app) fn public_sdrs_window(
        &mut self,
        ctx: &egui::Context,
        _cmds: &mut [Command],
    ) {
        if !self.show_public_sdrs {
            // Asked again the next time it opens, so a window reopened after an
            // hour is not showing an hour-old list.
            self.public_sdrs_asked = false;
            return;
        }
        // The cached answer on first open, so the window paints at once; the
        // ⟳ chip is what goes to the network.
        if !self.public_sdrs_asked {
            self.public_sdrs_asked = true;
            self.ask_device(ctx, sdroxide_types::DeviceProbe::PublicSdrs { refresh: false });
        }

        let dial_hz = self.state.active_freq_hz();
        let my_pos = sdroxide_types::grid_to_latlon(&self.my_grid());
        let directory = self.public_sdrs.clone();
        let mut open = self.show_public_sdrs;
        let mut refresh = false;
        let mut picked: Option<(PublicSdrEntry, PickAction)> = None;

        let resp = egui::Window::new("PUBLIC SDRS")
            .id(crate::layout::salted_id(ctx, "PUBLIC SDRS"))
            .open(&mut open)
            .frame(crate::chrome::window_frame())
            .resizable(true)
            .default_width(crate::layout::window_w(ctx, 980.0))
            .default_height(crate::layout::window_h(ctx, 520.0))
            .show(ctx, |ui| {
                crate::chrome::window_body_bg(ui);
                ui.horizontal(|ui| {
                    for (i, net) in PublicSdrNetwork::ALL.iter().enumerate() {
                        if crate::chrome::chip(ui, self.public_sdr_nets_shown[i], net.label())
                            .clicked()
                        {
                            self.public_sdr_nets_shown[i] = !self.public_sdr_nets_shown[i];
                        }
                    }
                    if crate::chrome::chip(ui, self.public_sdr_free_only, "AVAILABLE")
                        .on_hover_text(
                            "Hide receivers that are full, and the ones whose operator has \
                             not opened any channels to apps other than a browser",
                        )
                        .clicked()
                    {
                        self.public_sdr_free_only = !self.public_sdr_free_only;
                    }
                    if crate::chrome::chip(ui, self.public_sdr_in_band, "IN BAND")
                        .on_hover_text("Only receivers that cover the current dial frequency")
                        .clicked()
                    {
                        self.public_sdr_in_band = !self.public_sdr_in_band;
                    }
                    if crate::chrome::chip(ui, self.public_sdr_low_bw, "LOW BW")
                        .on_hover_text(
                            "Take a SpyServer in its low-bandwidth shape: a narrow I/Q window \
                             that follows the dial plus the server's own band view, instead of \
                             megabits of wideband I/Q. No effect on a KiwiSDR, which has only \
                             the one shape.",
                        )
                        .clicked()
                    {
                        self.public_sdr_low_bw = !self.public_sdr_low_bw;
                    }
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if crate::chrome::chip(ui, false, "⟳ REFRESH")
                            .on_hover_text("Fetch both directories again")
                            .clicked()
                        {
                            refresh = true;
                        }
                    });
                });
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 5.0;
                    ui.label(RichText::new("⌕").color(crate::theme::CYAN_DIM()).size(14.0));
                    crate::chrome::field(
                        ui,
                        egui::TextEdit::singleline(&mut self.public_sdr_search)
                            .desired_width(240.0)
                            .hint_text("name, place, antenna, band")
                            .text_color(crate::theme::TEXT_STRONG()),
                    );
                    if !self.public_sdr_search.trim().is_empty()
                        && ui.button("✕").on_hover_text("Clear the search").clicked()
                    {
                        self.public_sdr_search.clear();
                    }
                });

                let Some(dir) = &directory else {
                    ui.separator();
                    ui.add_space(8.0);
                    ui.label(
                        RichText::new(if self.probes_answered {
                            "Fetching the receiver lists…"
                        } else {
                            "The machine this radio is attached to does not answer \
                             device questions, so it cannot fetch the lists either."
                        })
                        .color(crate::theme::gray(140)),
                    );
                    return;
                };

                // Age and any source that failed, on one line: a directory that
                // is quietly an hour old looks exactly like a fresh one.
                ui.horizontal_wrapped(|ui| {
                    let age = if dir.fetched_unix > 0 {
                        fmt_age(now_unix() - dir.fetched_unix)
                    } else {
                        "—".to_string()
                    };
                    ui.label(
                        RichText::new(format!(
                            "{} receivers · {} SpyServer · {} KiwiSDR · fetched {age} ago",
                            dir.entries.len(),
                            dir.count(PublicSdrNetwork::SpyServer),
                            dir.count(PublicSdrNetwork::KiwiSdr),
                        ))
                        .size(11.0)
                        .color(crate::theme::gray(150)),
                    );
                    for note in &dir.notes {
                        ui.label(RichText::new(note).size(11.0).color(crate::theme::ALERT()));
                    }
                });
                ui.separator();

                let rows = self.public_sdr_rows(&dir.entries, dial_hz);
                if !self.public_sdr_search.trim().is_empty() || rows.len() > MAX_ROWS {
                    let shown = rows.len().min(MAX_ROWS);
                    let (text, colour) = match rows.len() {
                        0 => ("no match".to_string(), crate::theme::ALERT()),
                        n if n > MAX_ROWS => (
                            format!("showing {shown} of {n} — search to narrow it"),
                            crate::theme::YELLOW(),
                        ),
                        n => (format!("{n} match"), crate::theme::YELLOW()),
                    };
                    ui.label(RichText::new(text).color(colour).size(10.0));
                }

                egui::ScrollArea::vertical().auto_shrink([false, false]).show_themed(ui, |ui| {
                    for (e, _) in rows.iter().take(MAX_ROWS) {
                        let km = match (my_pos, e.lat, e.lon) {
                            (Some(me), Some(lat), Some(lon)) => Some(sdroxide_types::distance_km(
                                me,
                                (f64::from(lat), f64::from(lon)),
                            )),
                            _ => None,
                        };
                        if let Some(a) = entry_row(ui, e, km) {
                            picked = Some(((*e).clone(), a));
                        }
                    }
                    if rows.is_empty() {
                        ui.add_space(8.0);
                        ui.label(
                            RichText::new("nothing matches — try turning a filter chip back on")
                                .color(crate::theme::gray(120)),
                        );
                    }
                });
            });
        if let Some(r) = &resp {
            crate::chrome::paint_window_border(ctx, &r.response);
        }
        self.show_public_sdrs = open;

        if refresh {
            self.public_sdrs = None;
            self.ask_device(ctx, sdroxide_types::DeviceProbe::PublicSdrs { refresh: true });
        }
        if let Some((entry, action)) = picked {
            self.take_public_sdr(&entry, action);
        }
    }

    /// Act on a picked receiver.
    ///
    /// Both routes go through [`PublicSdrEntry::radio_config`], so the same
    /// receiver is configured identically however it was taken — otherwise
    /// "open it in a tab" and "use it here" would drift into two subtly
    /// different radios.
    fn take_public_sdr(&mut self, entry: &PublicSdrEntry, action: PickAction) {
        // Built on this radio's own configuration where there is one, so
        // pointing it at a receiver keeps its converter offset, its audio
        // devices and everything else the operator had set.
        let base = self.radio_cfg.clone().unwrap_or_default();
        let cfg = entry.radio_config(&base, self.public_sdr_low_bw);
        match action {
            PickAction::ThisRadio => {
                self.ctrl.set_radio_config(cfg);
                self.ctrl.reopen_source();
                self.show_notice(format!("Pointing this radio at {}…", entry.name));
                self.show_public_sdrs = false;
            }
            PickAction::NewRadio => {
                // A brand-new radio has none of this radio's settings, and
                // should not inherit them: it is a different receiver.
                let fresh = entry
                    .radio_config(&sdroxide_types::RadioConfig::default(), self.public_sdr_low_bw);
                self.radio_tab_requests.push(RadioTabRequest::Add {
                    station: self.station_key(),
                    preset: Some(Box::new(fresh)),
                });
                self.show_notice(format!("Opening {} as another radio…", entry.name));
            }
        }
    }
}
