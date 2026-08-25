//! The ground the flat maps are drawn on.
//!
//! The FT8/WSPR panel map and the APRS map are dot matrices, and this is what
//! decides where a dot goes: the same Natural Earth rasters the 3D globe is
//! textured with (`assets/earth/`, rebuilt by `make_earth_maps.py`), decoded
//! once and kept as mip pyramids for the dot grid to sample. One set of
//! coastlines for every view in the program, so a QTH lands on the same
//! shoreline whichever one is open — and the flat maps get the country borders
//! and rivers the globe has had all along.
//!
//! Three things, in the form each one is actually good in:
//!
//! - [`land`] is a *coverage* raster — the fraction of each cell that is
//!   ground. A region is what a coverage field is good at: its ½ contour is the
//!   coastline, placed to a fraction of a cell rather than stepped along the
//!   grid, which is the same trick `solar_body.wgsl` plays on the globe.
//! - [`lines`] — the borders and the rivers — are the *polylines* they were
//!   digitised as. The globe is textured with rasters of them, but a flat map
//!   cannot use one: a border is a texel wide there, and once a map is zoomed
//!   in far enough for a texel to cover several dots, no threshold on the
//!   interpolated coverage gives a line back. Geometry has no such limit.
//! - [`cities`] is a table, because a flat map has room for a name and no
//!   texture can carry one.
//!
//! Everything is read on first use and never freed. That costs about 26 MB and
//! fifty milliseconds — nearly all of it the land pyramid — once, for a program
//! that then never touches the assets again, and only for a session that
//! actually opens a map. Fifty milliseconds is three dropped frames on the
//! frame that first draws one, which is exactly the frame somebody is looking
//! at, so a native session [`prime`]s it on a thread at startup instead.

use std::sync::OnceLock;

/// The world, as `assets/earth/make_earth_maps.py` leaves it.
///
/// Every view in the program reads these same bytes — the flat maps from the
/// pyramids below, the 3D globe and the login backdrop by uploading them to the
/// GPU — so they are named once here rather than `include_bytes!` in each place
/// and linked in three times. All are rasterised from Natural Earth 1:10m and
/// share one coordinate convention (x = −180°…180°, y = +90°…−90°), which is
/// what makes a QTH marker land on the same shoreline whichever view is open.
///
/// Land is 8192×4096 (1/22.75°, ~4.9 km) and holds *coverage* rather than a
/// 1-bit mask: the fraction of each texel that is land. Both the globe's shader
/// and [`land_ink`](crate::widgets::worldmap) draw the shoreline along that
/// field's ½ contour, which interpolation places to a fraction of a texel — so
/// the coast stays a clean curve however far it is zoomed into, instead of the
/// texel staircase a thresholded mask would give. The rest are 4320×2160
/// (1/12°): one-texel lines and small blobs rather than a filled region, so
/// there is no contour to sharpen and the extra grid would only cost memory.
pub(crate) const LAND_PNG: &[u8] = include_bytes!("../assets/earth/land.png");
pub(crate) const BORDER_PNG: &[u8] = include_bytes!("../assets/earth/borders.png");
pub(crate) const RIVER_PNG: &[u8] = include_bytes!("../assets/earth/rivers.png");
/// Built-up urban areas — the shape the globe lights its night side with. Not
/// sampled here: a flat map has room for a name, and uses [`cities`] instead.
pub(crate) const CITY_PNG: &[u8] = include_bytes!("../assets/earth/cities.png");
const CITY_BIN: &[u8] = include_bytes!("../assets/earth/cities.bin");
/// The borders and rivers as polylines rather than as pixels — see [`lines`].
const LINE_BIN: &[u8] = include_bytes!("../assets/earth/lines.bin");

/// The finest land grid kept on the CPU, as an edge length.
///
/// Natively the asset's full 8192×4096 — 1/22.75°, about 4.9 km — which is
/// what the globe uploads and what the flat maps now hold too: 22 MB of
/// nibble-packed pyramid, and the only raster the CPU keeps at all now that the
/// borders and the rivers are [`lines`] rather than pixels. It is the map's
/// resolution in the plainest sense, so it goes up whole.
///
/// The browser takes it one level down. Halving an edge quarters the memory,
/// and a tab that also holds the same asset on the GPU, decodes it on the one
/// thread it has and rarely shows a map more than a panel wide is the wrong
/// place to spend 22 MB on islands nobody is zoomed in on.
#[cfg(not(target_arch = "wasm32"))]
const MAX_DIM: usize = 8192;
#[cfg(target_arch = "wasm32")]
const MAX_DIM: usize = 4096;

/// Coverage as one nibble per cell, 0…15.
///
/// Sixteen levels because of what they are used for: the land contour's
/// position inside a cell (a sixteenth of a texel is far below what a four-point
/// dot grid can show) and a line's alpha (a dot is either drawn or not, and the
/// eye is not counting the sixteen steps in between). Half the memory of a byte
/// per cell, for a difference nothing on screen can resolve.
struct Level {
    w: usize,
    h: usize,
    cov: Vec<u8>,
}

impl Level {
    fn pack(w: usize, h: usize, gray: &[u8]) -> Level {
        let mut cov = vec![0u8; (w * h).div_ceil(2)];
        for (i, &g) in gray.iter().enumerate() {
            // Round to nearest of the sixteen levels rather than truncating,
            // so a fully covered texel comes back as 1.0 and not 15/16.
            let v = ((g as u16 * 15 + 127) / 255) as u8;
            if i % 2 == 0 {
                cov[i / 2] |= v << 4;
            } else {
                cov[i / 2] |= v;
            }
        }
        Level { w, h, cov }
    }

    #[inline]
    fn at(&self, col: usize, row: usize) -> f32 {
        let i = row * self.w + col;
        let byte = self.cov[i >> 1];
        let v = if i & 1 == 0 { byte >> 4 } else { byte & 0x0f };
        f32::from(v) * (1.0 / 15.0)
    }

    /// Coverage at a point, interpolated between the four cells around it.
    ///
    /// Longitude wraps — the world repeats sideways and a map straddling the
    /// date line has to interpolate across it — and latitude clamps, because
    /// past a pole there is nothing to blend with.
    fn bilinear(&self, lon: f64, lat: f64) -> f32 {
        let u = (lon + 180.0) / 360.0 * self.w as f64 - 0.5;
        let v = (90.0 - lat) / 180.0 * self.h as f64 - 0.5;
        let (u0, v0) = (u.floor(), v.floor());
        let (fu, fv) = ((u - u0) as f32, (v - v0) as f32);
        let x0 = u0.rem_euclid(self.w as f64) as usize;
        let x1 = if x0 + 1 == self.w { 0 } else { x0 + 1 };
        let y0 = v0.clamp(0.0, (self.h - 1) as f64) as usize;
        let y1 = (y0 + 1).min(self.h - 1);
        let top = self.at(x0, y0) + (self.at(x1, y0) - self.at(x0, y0)) * fu;
        let bot = self.at(x0, y1) + (self.at(x1, y1) - self.at(x0, y1)) * fu;
        top + (bot - top) * fv
    }
}

/// One equirectangular coverage raster as a mip pyramid.
pub struct Layer {
    levels: Vec<Level>,
}

impl Layer {
    fn build(label: &str, png: &[u8], max_dim: usize) -> Layer {
        // A checked-in asset failing to decode costs the layer, not the map;
        // an empty pyramid samples as zero everywhere and says so on stderr.
        let gray = match image::load_from_memory_with_format(png, image::ImageFormat::Png) {
            Ok(img) => img.to_luma8(),
            Err(e) => {
                eprintln!("sdroxide: decoding {label}: {e}");
                image::GrayImage::from_raw(1, 1, vec![0u8]).expect("1×1")
            }
        };
        let (mut w, mut h) = (gray.width() as usize, gray.height() as usize);
        let mut gray = gray.into_raw();
        while w > max_dim || h > max_dim {
            (w, h, gray) = halve(w, h, &gray);
        }
        let mut levels = vec![Level::pack(w, h, &gray)];
        while w > 1 || h > 1 {
            (w, h, gray) = halve(w, h, &gray);
            levels.push(Level::pack(w, h, &gray));
        }
        Layer { levels }
    }

    /// A sampler for a dot grid whose cells are `deg` of longitude wide.
    ///
    /// Which level that lands on is the usual mip choice — the one whose texels
    /// are about the size of a cell — and the two straddling levels are blended
    /// rather than switched between, because these maps *ease* their zoom and a
    /// level that changed in one frame would pop the whole coastline.
    pub fn sampler(&self, deg: f64) -> Sampler<'_> {
        let base = 360.0 / self.levels[0].w as f64;
        let lambda = (deg / base).max(1e-9).log2();
        let lo = (lambda.floor().max(0.0) as usize).min(self.levels.len() - 1);
        let hi = (lo + 1).min(self.levels.len() - 1);
        let t = if lambda <= lo as f64 { 0.0 } else { (lambda - lo as f64).min(1.0) as f32 };
        Sampler {
            lo: &self.levels[lo],
            hi: &self.levels[hi],
            t,
            mag: ((360.0 / self.levels[lo].w as f64) / deg) as f32,
        }
    }
}

/// A [`Layer`] bound to one zoom: the two levels it reads and how far the dot
/// grid is from them.
pub struct Sampler<'a> {
    lo: &'a Level,
    hi: &'a Level,
    t: f32,
    mag: f32,
}

impl Sampler<'_> {
    /// Coverage at (lon, lat), 0…1.
    pub fn at(&self, lon: f64, lat: f64) -> f32 {
        let a = self.lo.bilinear(lon, lat);
        if self.t < 0.02 { a } else { a + (self.hi.bilinear(lon, lat) - a) * self.t }
    }

    /// How many dot cells one texel of the level being read covers. Above 1 the
    /// dots have outrun the map data and a feature's edge is being drawn from
    /// interpolation rather than from anything measured.
    pub fn magnification(&self) -> f32 {
        self.mag
    }
}

/// One mip level down: a 2×2 box filter, with the odd row/column carried over
/// rather than dropped.
fn halve(w: usize, h: usize, src: &[u8]) -> (usize, usize, Vec<u8>) {
    let (nw, nh) = ((w / 2).max(1), (h / 2).max(1));
    let mut out = vec![0u8; nw * nh];
    for y in 0..nh {
        for x in 0..nw {
            let (x0, y0) = (x * 2, y * 2);
            let (x1, y1) = ((x0 + 1).min(w - 1), (y0 + 1).min(h - 1));
            let at = |x: usize, y: usize| u32::from(src[y * w + x]);
            let sum = at(x0, y0) + at(x1, y0) + at(x0, y1) + at(x1, y1);
            out[y * nw + x] = ((sum + 2) / 4) as u8;
        }
    }
    (nw, nh, out)
}

/// The land field, decoded on the first map frame and kept for the session.
///
/// The one raster the flat maps sample. Land is a *region*, and a region is
/// what a coverage field is good at: the ½ contour of it is the coastline, to a
/// fraction of a cell. The borders and the rivers are lines, which it is not —
/// see [`lines`].
pub fn land() -> &'static Layer {
    static LAND: OnceLock<Layer> = OnceLock::new();
    LAND.get_or_init(|| Layer::build("land.png", LAND_PNG, MAX_DIM))
}

// ── The line layers ─────────────────────────────────────────────────────────

/// One polyline: a run of points, with the box it lives in and how big a thing
/// it is.
pub struct Part {
    /// 0 for a border, 1…15 for a river by the width Natural Earth ranks it at.
    pub rank: u8,
    /// Bounds as a centre and a half-extent, in degrees, which is the form a
    /// wrapping longitude test wants: `wrap180(centre - view_centre)` against
    /// the two half-widths, with no cases to get wrong at the date line.
    pub mid: (f64, f64),
    pub half: (f64, f64),
    /// (lat, lon), in degrees.
    pub pts: Vec<(f32, f32)>,
}

/// The same lines at one level of detail.
pub struct LineLevel {
    /// How far a dropped vertex was allowed to sit from the line it was on,
    /// in degrees. What picks a level: the finest one under half a dot cell.
    pub eps: f64,
    pub parts: Vec<Part>,
}

/// A whole line layer — the borders, or the rivers — at every level.
pub struct LineLayer {
    /// Finest first.
    levels: Vec<LineLevel>,
}

impl LineLayer {
    /// The coarsest level still finer than half a `deg`-wide dot cell, so what
    /// was thrown away is always smaller than the grid it is drawn on.
    pub fn level(&self, deg: f64) -> &LineLevel {
        self.levels.iter().rev().find(|l| l.eps <= deg * 0.5).unwrap_or(&self.levels[0])
    }
}

/// The borders and the rivers, as the polylines they were digitised as.
pub struct Lines {
    pub borders: LineLayer,
    pub rivers: LineLayer,
}

/// Parsed on the first map frame and kept for the session (about 4 MB).
///
/// Lines rather than a raster because of what a flat map does with them. A
/// border is one texel wide in `borders.png`, and once a map is zoomed in far
/// enough for that texel to cover several dots, no threshold on the
/// interpolated coverage gives a line back: a low one draws the interpolation's
/// skirt as well and the border comes out a band, a high one breaks the same
/// border into fragments wherever it falls between two texels. Geometry has
/// neither problem — a line walked from its own vertices is one dot wide at
/// every zoom, and in the place it was digitised at.
pub fn lines() -> &'static Lines {
    static LINES: OnceLock<Lines> = OnceLock::new();
    LINES.get_or_init(|| parse_lines(LINE_BIN))
}

/// See `make_earth_maps.py`'s `build_lines`: a magic, then one layer after
/// another, each a run of levels, each a run of parts, each an absolute first
/// point and 16-bit steps from there.
fn parse_lines(blob: &'static [u8]) -> Lines {
    /// One step of a delta, in degrees — `LINE_STEP` in the baker.
    const STEP: f64 = 1e-4;
    let mut layers = Vec::new();
    let mut rest = match blob.strip_prefix(b"SDXLINE1") {
        Some(rest) => rest,
        None => {
            eprintln!("sdroxide: lines.bin is not a line table");
            &[]
        }
    };
    let take = |n: usize, rest: &mut &'static [u8]| -> &'static [u8] {
        let (head, tail) = rest.split_at_checked(n).unwrap_or((&[], &[]));
        *rest = tail;
        head
    };
    let n_layers = take(1, &mut rest).first().copied().unwrap_or(0);
    for _ in 0..n_layers {
        let n_levels = take(1, &mut rest).first().copied().unwrap_or(0);
        let mut levels = Vec::with_capacity(n_levels as usize);
        for _ in 0..n_levels {
            let head = take(8, &mut rest);
            if head.len() < 8 {
                break;
            }
            let eps = f32::from_le_bytes(head[0..4].try_into().expect("4 bytes"));
            let n_parts = u32::from_le_bytes(head[4..8].try_into().expect("4 bytes"));
            let mut parts = Vec::with_capacity(n_parts as usize);
            for _ in 0..n_parts {
                let head = take(11, &mut rest);
                if head.len() < 11 {
                    break;
                }
                let n = u16::from_le_bytes(head[1..3].try_into().expect("2 bytes")) as usize;
                let num =
                    |a: usize| i32::from_le_bytes(head[a..a + 4].try_into().expect("4 bytes"));
                let (mut lat, mut lon) = (f64::from(num(3)) / 1e5, f64::from(num(7)) / 1e5);
                let (mut lat0, mut lat1, mut lon0, mut lon1) = (lat, lat, lon, lon);
                let mut pts = Vec::with_capacity(n);
                pts.push((lat as f32, lon as f32));
                let steps = take(4 * n.saturating_sub(1), &mut rest);
                for step in steps.chunks_exact(4) {
                    let d = |a: usize| {
                        f64::from(i16::from_le_bytes(step[a..a + 2].try_into().expect("2 bytes")))
                    };
                    lat += d(0) * STEP;
                    lon += d(2) * STEP;
                    pts.push((lat as f32, lon as f32));
                    (lat0, lat1) = (lat0.min(lat), lat1.max(lat));
                    (lon0, lon1) = (lon0.min(lon), lon1.max(lon));
                }
                parts.push(Part {
                    rank: head[0],
                    mid: ((lat0 + lat1) * 0.5, (lon0 + lon1) * 0.5),
                    half: ((lat1 - lat0) * 0.5, (lon1 - lon0) * 0.5),
                    pts,
                });
            }
            levels.push(LineLevel { eps: f64::from(eps), parts });
        }
        layers.push(LineLayer { levels });
    }
    // A file that arrived short leaves empty layers rather than none at all,
    // so the map draws what it has instead of panicking on the way past.
    while layers.len() < 2 {
        layers.push(LineLayer { levels: vec![LineLevel { eps: 1.0, parts: Vec::new() }] });
    }
    let mut it = layers.into_iter();
    Lines { borders: it.next().expect("two layers"), rivers: it.next().expect("two layers") }
}

/// Start the decode on a thread of its own, so the first map frame does not
/// have to wait for it.
///
/// Fire-and-forget: whichever of the two threads reaches the data first does
/// the work and the other waits, which is all [`OnceLock`] is being asked for
/// here. In the browser there is no second thread to do it on, and the tab
/// pays for it once on the frame that needs it.
pub fn prime() {
    #[cfg(not(target_arch = "wasm32"))]
    std::thread::spawn(|| {
        land();
        lines();
        cities();
    });
}

/// True if the (lon, lat) point in degrees is over land.
///
/// Read off the finest grid kept, so it agrees with the coastline the maps
/// draw rather than approximating it from a coarser one.
pub fn is_land(lon: f64, lat: f64) -> bool {
    land().levels[0].bilinear(lon, lat) >= 0.5
}

// ── Cities ──────────────────────────────────────────────────────────────────

/// One populated place from Natural Earth.
pub struct City {
    pub lat: f64,
    pub lon: f64,
    /// The largest population figure Natural Earth carries for the place, or 0
    /// where it has none.
    pub pop: u32,
    /// A national capital, which a map is expected to mark whatever its size.
    pub capital: bool,
    /// The ASCII spelling. The UI ships one Latin font, and a name rendered as
    /// a row of empty boxes would be worse than a transliterated one.
    pub name: &'static str,
}

/// Every populated place, largest first.
///
/// The order is the whole interface: there is no room for seven thousand
/// labels, so a map walks this list, draws what falls inside its view and stops
/// when it has enough. Which cities show is then a consequence of how far in
/// the map is zoomed, with no threshold to pick and nothing to re-sort.
pub fn cities() -> &'static [City] {
    static CITIES: OnceLock<Vec<City>> = OnceLock::new();
    CITIES.get_or_init(|| parse_cities(CITY_BIN))
}

/// See `make_earth_maps.py`'s `build_places`: a magic, a count, then one
/// variable-length record per place.
fn parse_cities(blob: &'static [u8]) -> Vec<City> {
    let Some(rest) = blob.strip_prefix(b"SDXCITY1") else {
        eprintln!("sdroxide: cities.bin is not a city table");
        return Vec::new();
    };
    let Some((count, mut rest)) = rest.split_at_checked(4) else { return Vec::new() };
    let count = u32::from_le_bytes(count.try_into().expect("4 bytes")) as usize;
    let mut out = Vec::with_capacity(count);
    for _ in 0..count {
        let Some((head, tail)) = rest.split_at_checked(14) else { break };
        let n = head[13] as usize;
        let Some((name, tail)) = tail.split_at_checked(n) else { break };
        let num = |a: usize| i32::from_le_bytes(head[a..a + 4].try_into().expect("4 bytes"));
        out.push(City {
            lat: f64::from(num(0)) / 1e5,
            lon: f64::from(num(4)) / 1e5,
            pop: u32::from_le_bytes(head[8..12].try_into().expect("4 bytes")),
            capital: head[12] & 1 != 0,
            name: std::str::from_utf8(name).unwrap_or("?"),
        });
        rest = tail;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The mask has to have the continents where the continents are — this is
    /// what every marker on every flat map is placed against.
    #[test]
    fn land_is_where_land_is() {
        for (lon, lat) in [(-100.0, 40.0), (10.0, 50.0), (135.0, -25.0)] {
            assert!(is_land(lon, lat), "{lon},{lat} should be land");
        }
        for (lon, lat) in [(-140.0, 0.0), (-30.0, 30.0), (80.0, -40.0)] {
            assert!(!is_land(lon, lat), "{lon},{lat} should be sea");
        }
    }

    /// Every level of every pyramid is half the one above it, down to a single
    /// texel — a gap would make `sampler` read past the end at some zoom.
    #[test]
    fn the_pyramids_run_all_the_way_down() {
        for layer in [land()] {
            let mut prev: Option<&Level> = None;
            for l in &layer.levels {
                assert_eq!(l.cov.len(), (l.w * l.h).div_ceil(2));
                if let Some(p) = prev {
                    assert_eq!((l.w, l.h), ((p.w / 2).max(1), (p.h / 2).max(1)));
                }
                prev = Some(l);
            }
            let last = layer.levels.last().expect("a level");
            assert_eq!((last.w, last.h), (1, 1));
            assert!(layer.levels[0].w <= MAX_DIM);
        }
    }

    /// A sampler asks for the level whose texels match the dot grid, and never
    /// walks off either end of the pyramid.
    #[test]
    fn zoom_picks_a_level_and_stays_inside_the_pyramid() {
        let land = land();
        // Zoomed further in than the data goes: the base level, undiluted.
        let s = land.sampler(1e-6);
        assert!(std::ptr::eq(s.lo, &land.levels[0]));
        assert!(s.magnification() > 1.0);
        // ...and further out than the whole world.
        let s = land.sampler(720.0);
        assert!(std::ptr::eq(s.hi, land.levels.last().expect("a level")));
        assert!(s.magnification() < 1.0);
        // A dot the size of one base texel sits at the bottom of the pyramid.
        let s = land.sampler(360.0 / land.levels[0].w as f64);
        assert!(std::ptr::eq(s.lo, &land.levels[0]));
    }

    /// Coverage is a fraction: solid ground reads 1, open ocean 0, and a coast
    /// somewhere in between — that "in between" is what the maps draw their
    /// shoreline from.
    #[test]
    fn coverage_is_a_fraction() {
        let s = land().sampler(0.01);
        assert!(s.at(100.0, 45.0) > 0.99, "central Asia: {}", s.at(100.0, 45.0));
        assert!(s.at(-140.0, 0.0) < 0.01, "mid-Pacific: {}", s.at(-140.0, 0.0));
        // Longitude wraps rather than clamping, so ±180 is one place.
        let (a, b) = (s.at(-179.999, 65.0), s.at(179.999, 65.0));
        assert!((a - b).abs() < 0.35, "the date line is a seam: {a} vs {b}");
    }

    /// What the decoded world costs, held for the session.
    ///
    /// Asserted rather than measured in passing, because both numbers are
    /// decisions that are easy to change without noticing: `MAX_DIM` one level
    /// up is 45 MB of land pyramid rather than 22, and a finer line level is
    /// another megabyte of vertices — either would be handed to a browser tab
    /// as readily as to a workstation.
    #[test]
    fn the_world_fits_in_its_budget() {
        let raster: usize = land().levels.iter().map(|v| v.cov.len()).sum();
        assert!(raster < 32 << 20, "the land pyramid grew to {} MB", raster >> 20);
        let vectors: usize = [&lines().borders, &lines().rivers]
            .iter()
            .flat_map(|l| l.levels.iter())
            .flat_map(|l| l.parts.iter())
            .map(|p| p.pts.len() * std::mem::size_of::<(f32, f32)>())
            .sum();
        assert!(vectors < 8 << 20, "the line layers grew to {} MB", vectors >> 20);
    }

    /// The lines are the geometry they were digitised as: five levels of it,
    /// each coarser than the last, all of them on the planet.
    #[test]
    fn the_lines_are_lines() {
        for layer in [&lines().borders, &lines().rivers] {
            assert!(layer.levels.len() >= 4, "only {} levels", layer.levels.len());
            let mut prev = 0.0;
            for level in &layer.levels {
                assert!(level.eps > prev, "levels are not finest-first");
                prev = level.eps;
                assert!(!level.parts.is_empty());
                for part in &level.parts {
                    assert!(part.pts.len() >= 2, "a line needs two ends");
                    assert!(part.pts.iter().all(|p| (-90.0..=90.0).contains(&p.0)));
                    assert!(part.pts.iter().all(|p| (-180.5..=180.5).contains(&p.1)));
                }
            }
            // Simplification only ever drops vertices.
            let counts: Vec<usize> =
                layer.levels.iter().map(|l| l.parts.iter().map(|p| p.pts.len()).sum()).collect();
            assert!(counts.windows(2).all(|w| w[0] > w[1]), "{counts:?}");
        }
        // A border runs along the Rio Grande, so both layers have something
        // within a degree of it — which is the coarse test that the two layers
        // did not come back swapped or empty.
        for layer in [&lines().borders, &lines().rivers] {
            let near = layer.levels[0]
                .parts
                .iter()
                .any(|p| (p.mid.0 - 29.0).abs() < 3.0 && (p.mid.1 + 101.0).abs() < 4.0);
            assert!(near, "nothing on the Rio Grande");
        }
        // Rivers are ranked by size and borders are not.
        assert!(lines().borders.levels[0].parts.iter().all(|p| p.rank == 0));
        assert!(lines().rivers.levels[0].parts.iter().any(|p| p.rank > 8));
    }

    /// A dot grid picks the level that is finer than it, never a coarser one.
    #[test]
    fn the_level_follows_the_zoom() {
        let b = &lines().borders;
        let finest = b.levels[0].eps;
        assert!(b.level(finest).eps <= finest, "the finest grid gets the finest level");
        assert!(std::ptr::eq(b.level(1e-6), &b.levels[0]), "past the data, stay at the finest");
        let coarse = b.level(90.0);
        assert!(std::ptr::eq(coarse, b.levels.last().expect("a level")));
    }

    /// The table is sorted, spelled in ASCII, and has the places in it a
    /// world map is expected to name.
    #[test]
    fn the_city_table_is_a_city_table() {
        let cities = cities();
        assert!(cities.len() > 5000, "only {} cities", cities.len());
        assert!(cities.windows(2).all(|w| w[0].pop >= w[1].pop), "not sorted by population");
        assert!(cities.iter().all(|c| c.name.is_ascii() && !c.name.is_empty()));
        assert!(cities.iter().all(|c| (-90.0..=90.0).contains(&c.lat)));
        assert!(cities.iter().all(|c| (-180.0..=180.0).contains(&c.lon)));
        let tokyo = cities.iter().find(|c| c.name == "Tokyo").expect("Tokyo");
        assert!((tokyo.lat - 35.69).abs() < 0.2 && (tokyo.lon - 139.75).abs() < 0.2);
        assert!(cities.iter().take(60).any(|c| c.name == "London"));
        assert!(cities.iter().filter(|c| c.capital).count() > 150);
    }
}
