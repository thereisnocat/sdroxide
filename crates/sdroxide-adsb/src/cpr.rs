//! Compact Position Reporting: turning the 34 bits a squitter carries into a
//! place on the earth.
//!
//! A position message does not contain a position. It contains 17 bits of
//! latitude and 17 of longitude *within a grid cell*, and which cell is left
//! for the receiver to work out — which is how a position good to about five
//! metres fits in half the bits it would otherwise need. There are two grids,
//! one for frames flagged even and one for odd, deliberately of slightly
//! different sizes, and there are two ways to resolve the cell:
//!
//! * [`global`] — an even and an odd frame together. The two grids drift out of
//!   step at a known rate, so the offset between the two readings says which
//!   pair of cells they are in. Needs both frames, and needs them close
//!   together in time.
//! * [`local`] — one frame plus a position already known to be within half a
//!   cell. Needs only one frame, which is why every fix after the first comes
//!   from here.
//!
//! Both are written from ICAO Doc 9871 (D.2.4.7). The zone counts are computed
//! from the closed form rather than stored as the 59-entry table the standard
//! also prints; [`nl`] is pinned against that table's shape by its own test.

/// Latitude zones between the equator and a pole.
const NZ: f64 = 15.0;
/// 2^17 — the CPR fields are 17 bits.
const CPR_MAX: f64 = 131_072.0;

/// The number of longitude zones at a latitude.
pub fn nl(lat: f64) -> f64 {
    let lat = lat.abs();
    if lat >= 87.0 {
        return 1.0;
    }
    if lat < 1e-9 {
        return 59.0;
    }
    let a = 1.0 - (std::f64::consts::PI / (2.0 * NZ)).cos();
    let b = (std::f64::consts::PI / 180.0 * lat).cos().powi(2);
    let x = 1.0 - a / b;
    (std::f64::consts::TAU / x.clamp(-1.0, 1.0).acos()).floor().max(1.0)
}

/// `x mod y`, always non-negative — the `MOD` of the standard's pseudocode,
/// which is not Rust's `%` once either side goes negative.
fn cpr_mod(x: f64, y: f64) -> f64 {
    let r = x % y;
    if r < 0.0 { r + y } else { r }
}

/// Resolve an even/odd pair into a position.
///
/// `even_first` says which of the two arrived later, because the answer is
/// reported in the grid of the *newer* frame — the aircraft is where it was
/// most recently, not where it was two frames ago.
///
/// Airborne frames only. There is no global surface decode here and that is not
/// an omission: a surface pair resolves only to one of four quadrants of the
/// earth, and choosing between them needs a reference position anyway, at which
/// point [`local_surface`] is the whole answer.
pub fn global(even: (u32, u32), odd: (u32, u32), even_is_newer: bool) -> Option<(f64, f64)> {
    let (lat_e, lon_e) = (f64::from(even.0) / CPR_MAX, f64::from(even.1) / CPR_MAX);
    let (lat_o, lon_o) = (f64::from(odd.0) / CPR_MAX, f64::from(odd.1) / CPR_MAX);

    // The latitude zone index, from how far the two readings have drifted apart.
    let j = (59.0 * lat_e - 60.0 * lat_o + 0.5).floor();
    let mut rlat_e = (360.0 / 60.0) * (cpr_mod(j, 60.0) + lat_e);
    let mut rlat_o = (360.0 / 59.0) * (cpr_mod(j, 59.0) + lat_o);
    if rlat_e >= 270.0 {
        rlat_e -= 360.0;
    }
    if rlat_o >= 270.0 {
        rlat_o -= 360.0;
    }
    if !(-90.0..=90.0).contains(&rlat_e) || !(-90.0..=90.0).contains(&rlat_o) {
        return None;
    }
    // Straddling a zone boundary the two frames disagree about which zone they
    // are in, and the longitude that follows would be wrong by a whole cell.
    // The answer is to wait for the next pair, not to pick one.
    if nl(rlat_e) != nl(rlat_o) {
        return None;
    }

    let (lat, lon_cpr, m_off) =
        if even_is_newer { (rlat_e, lon_e, 0.0) } else { (rlat_o, lon_o, 1.0) };
    let nlv = nl(lat);
    let ni = (nlv - m_off).max(1.0);
    let m = (lon_e * (nlv - 1.0) - lon_o * nlv + 0.5).floor();
    let mut lon = (360.0 / ni) * (cpr_mod(m, ni) + lon_cpr);
    if lon >= 180.0 {
        lon -= 360.0;
    }
    Some((lat, lon))
}

/// Resolve one airborne frame against a position already known.
///
/// `ref_lat`/`ref_lon` must be within about 180 NM — half a zone — of the truth,
/// which for a target already being tracked means its own previous fix.
/// Answers `None` when the result lands further from the reference than that,
/// which is the check that stops a stale reference or an undetected bit error
/// teleporting an aircraft into the next zone.
pub fn local(cpr: (u32, u32), odd: bool, ref_lat: f64, ref_lon: f64) -> Option<(f64, f64)> {
    decode(cpr, odd, ref_lat, ref_lon, 360.0)
}

/// The same for a **surface** frame, whose zones are a quarter the size.
///
/// The reference has to be within about 45 NM rather than 180, which is always
/// available in practice: an aircraft on the ground is at an airport, so either
/// its own last fix or the operator's own position will do.
pub fn local_surface(cpr: (u32, u32), odd: bool, ref_lat: f64, ref_lon: f64) -> Option<(f64, f64)> {
    decode(cpr, odd, ref_lat, ref_lon, 90.0)
}

/// `span` is 360 for an airborne frame and 90 for a surface one — the whole
/// difference between the two encodings.
fn decode(cpr: (u32, u32), odd: bool, ref_lat: f64, ref_lon: f64, span: f64) -> Option<(f64, f64)> {
    let i = if odd { 1.0 } else { 0.0 };
    let d_lat = span / (4.0 * NZ - i);
    let y = f64::from(cpr.0) / CPR_MAX;

    let j = (ref_lat / d_lat).floor() + (cpr_mod(ref_lat, d_lat) / d_lat - y + 0.5).floor();
    let lat = d_lat * (j + y);
    if !(-90.0..=90.0).contains(&lat) {
        return None;
    }

    let nlv = nl(lat) - i;
    let d_lon = if nlv > 0.0 { span / nlv } else { span };
    let x = f64::from(cpr.1) / CPR_MAX;
    let m = (ref_lon / d_lon).floor() + (cpr_mod(ref_lon, d_lon) / d_lon - x + 0.5).floor();
    let lon = wrap180(d_lon * (m + x));

    if (lat - ref_lat).abs() > d_lat / 2.0 || wrap180(lon - ref_lon).abs() > d_lon / 2.0 {
        return None;
    }
    Some((lat, lon))
}

fn wrap180(mut d: f64) -> f64 {
    while d > 180.0 {
        d -= 360.0;
    }
    while d < -180.0 {
        d += 360.0;
    }
    d
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::message::{Body, Es, decode as decode_msg};

    fn cpr_of(hex: &str) -> ((u32, u32), bool) {
        let bytes: Vec<u8> = (0..hex.len() / 2)
            .map(|i| u8::from_str_radix(&hex[i * 2..i * 2 + 2], 16).unwrap())
            .collect();
        let Body::Squitter(Es::Airborne { cpr, .. }) = decode_msg(&bytes, 0).body else {
            panic!("not an airborne position");
        };
        ((cpr.lat, cpr.lon), cpr.odd)
    }

    /// The pair published with the algorithm, and the position published with
    /// it: 52.2572 N, 3.9193 E. This is the one end-to-end check on the global
    /// decode that does not come from this crate's own arithmetic.
    #[test]
    fn the_published_pair_decodes_to_the_published_position() {
        let (even, _) = cpr_of("8D40621D58C382D690C8AC2863A7");
        let (odd, _) = cpr_of("8D40621D58C386435CC412692AD6");
        let (lat, lon) = global(even, odd, true).expect("the pair resolves");
        assert!((lat - 52.2572).abs() < 0.0002, "latitude {lat}");
        assert!((lon - 3.9193).abs() < 0.0002, "longitude {lon}");

        // Reported in the odd frame's grid instead. A different answer, and
        // that is the point: the two frames were sent from different places, so
        // "where is it" depends on which of them is the more recent. About a
        // kilometre apart here, which is what half a second at 500 knots is.
        let (olat, olon) = global(even, odd, false).expect("the pair resolves either way round");
        assert!((olat - 52.2658).abs() < 0.001, "latitude {olat}");
        assert!((olon - 3.9389).abs() < 0.001, "longitude {olon}");
        assert!(olat > lat, "the later frame is further along the track");
    }

    /// A local decode of either frame reproduces the global answer. This is
    /// what makes the local path trustworthy: it is checked against a
    /// completely different piece of arithmetic on the same bits.
    #[test]
    fn a_local_decode_reproduces_the_global_one() {
        let (even, _) = cpr_of("8D40621D58C382D690C8AC2863A7");
        let (odd, _) = cpr_of("8D40621D58C386435CC412692AD6");
        let (glat, glon) = global(even, odd, true).expect("the pair resolves");

        let (lat, lon) = local(even, false, glat, glon).expect("the even frame decodes locally");
        assert!((lat - glat).abs() < 1e-6, "latitude {lat} vs {glat}");
        assert!((lon - glon).abs() < 1e-6, "longitude {lon} vs {glon}");

        let (lat, _) = local(odd, true, glat, glon).expect("and so does the odd one");
        assert!((lat - glat).abs() < 0.01, "the odd frame is one report along: {lat}");
    }

    /// A local decode answers *near its reference*, always — that is what it
    /// is for, and it is also the hazard. Given a reference on the other side
    /// of the world it returns a position on the other side of the world, with
    /// no way to know it is wrong.
    ///
    /// So this pins the property rather than a guard that cannot exist: the
    /// answer is a function of the reference. What keeps a wrong one out is
    /// [`crate::track`], which only offers a fix it made in the last minute.
    #[test]
    fn a_local_decode_is_only_ever_as_good_as_its_reference() {
        let (odd, _) = cpr_of("8D40621D58C386435CC412692AD6");
        let near = local(odd, true, 52.2572, 3.9193).expect("a good reference");
        let far = local(odd, true, 52.2572, -160.0).expect("a bad one answers too");
        assert!((near.1 - 3.9389).abs() < 0.001);
        assert!((far.1 + 160.0).abs() < 6.0, "and it answers beside the bad reference: {far:?}");

        // Whatever it is given, what comes back is a place on the earth —
        // never a latitude past a pole or a longitude that has wrapped twice.
        let mut lat = -89.0;
        while lat <= 89.0 {
            let mut lon = -179.0;
            while lon <= 179.0 {
                if let Some((a, o)) = local(odd, true, lat, lon) {
                    assert!((-90.0..=90.0).contains(&a), "latitude {a} from {lat},{lon}");
                    assert!((-180.0..=180.0).contains(&o), "longitude {o} from {lat},{lon}");
                }
                lon += 7.0;
            }
            lat += 3.0;
        }
    }

    /// The zone count runs from 59 at the equator to 1 at the poles and never
    /// rises on the way. A break in that would shift fixes by a whole cell at
    /// some latitudes and not at others, which looks like an intermittent
    /// receiver rather than like arithmetic.
    #[test]
    fn the_longitude_zone_count_walks_from_fifty_nine_down_to_one() {
        assert_eq!(nl(0.0), 59.0);
        assert_eq!(nl(87.5), 1.0);
        assert_eq!(nl(-87.5), 1.0);
        let mut prev = 60.0;
        let mut lat = 0.0;
        while lat < 89.0 {
            let n = nl(lat);
            assert!(n <= prev, "zone count rose at {lat}: {n} after {prev}");
            prev = n;
            lat += 0.125;
        }
    }
}
