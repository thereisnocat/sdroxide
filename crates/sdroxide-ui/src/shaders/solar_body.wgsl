// Every solid body in the scene. One pipeline, one branch on `d.params.x` —
// the branch is uniform across a draw, so it costs nothing.
//
// Six bodies are drawn from real data: the Sun (live SDO imagery), the Earth
// (the Natural Earth coastline and border masks), and the Moon, Mars, Jupiter
// and Saturn from published spacecraft maps. Those are the ones a viewer can
// check against a photograph they have already seen, and a procedural stand-in
// for any of them reads as broken rather than as stylised.
//
// Everything else is procedural, which is a deliberate trade rather than a
// shortcut: a map per moon would be tens of megabytes of imagery for bodies
// that are a handful of pixels across in almost every frame. What the
// procedural surfaces get right is what is checkable at that size — Io is
// sulphur-yellow, Iapetus has one black hemisphere — and each of them turns
// with the body's real rotation.

struct Globals {
    view_proj: mat4x4<f32>,
    camera_pos: vec4<f32>,   // xyz eye, w near
    sun_pos: vec4<f32>,      // xyz centre, w rendered radius
    sun_to_earth: vec4<f32>, // xyz unit, w SDO disk radius as a fraction of the image
    solar_north: vec4<f32>,  // xyz unit, w Stonyhurst west sign
    viewport: vec4<f32>,     // w, h, 1/w, 1/h
    misc: vec4<f32>,         // x seconds, y photo blend, zw spare
};

struct DrawData {
    model: mat4x4<f32>,
    basis: mat4x4<f32>,
    tint: vec4<f32>,
    tint2: vec4<f32>,
    params: vec4<f32>,       // x mode, y half-angle, z alpha, w spare
    style: vec4<f32>,        // x style, y detail or map layer, z two-tone, w limb haze
};

@group(0) @binding(0) var<uniform> g: Globals;
@group(0) @binding(1) var land_tex: texture_2d<f32>;
@group(0) @binding(2) var sun_tex: texture_2d<f32>;
@group(0) @binding(3) var samp: sampler;
@group(1) @binding(0) var<uniform> d: DrawData;
@group(0) @binding(5) var border_tex: texture_2d<f32>;
@group(0) @binding(6) var body_maps: texture_2d_array<f32>;
@group(0) @binding(10) var river_tex: texture_2d<f32>;
@group(0) @binding(11) var city_tex: texture_2d<f32>;

// The FT8 map's own palette, so the globe reads as the same map (see
// widgets/worldmap.rs, land `#1c4458`).
const LAND_DAY  = vec3<f32>(0.109804, 0.266667, 0.345098); // #1c4458
const OCEAN_DAY = vec3<f32>(0.039216, 0.094118, 0.149020); // #0a1826
const COAST     = vec3<f32>(0.113725, 0.611765, 0.745098); // #1d9cbe  theme::CYAN_DIM
const ATMO      = vec3<f32>(0.000000, 0.815686, 0.956863); // #00d0f4  theme::CYAN
const RIVER     = vec3<f32>(0.109804, 0.372549, 0.560784); // #1c5f8f  the flat map's river
const CITY      = vec3<f32>(1.000000, 0.756863, 0.407843); // #ffc168  sodium, seen from orbit

const PI = 3.14159265;

/// Half-width of the coastline stroke, in pixels, before its one-pixel
/// antialiasing ramp. Deliberately thin: the coast is a *line drawing* over the
/// land fill, and anything wider stops resolving the estuaries and islands the
/// 1/22.75° map has in it.
const COAST_PX = 0.6;

/// Self-illumination for the coastline and the borders, added on the night
/// side.
///
/// The coast and the borders are a *line drawing* over the globe, not a
/// physical feature of it, and a line drawing has no business going dark
/// because the Sun set on it. Without this the whole night hemisphere is a
/// featureless slab and there is nowhere to put a QSO arc's far end, an
/// auroral oval or a satellite footprint — everything that happens at night is
/// the interesting half.
///
/// Deliberately small. This is a hint of the map showing through, not a second
/// daylit side: the terminator has to stay the most obvious thing on the globe,
/// and the borders keep their place below the coast in the hierarchy.
const COAST_GLOW  = 0.16;
const BORDER_GLOW = 0.085;

/// The night side's city lights, from the built-up urban areas.
///
/// The one place where the globe stops being a line drawing and shows
/// something photographic — and the reason it can afford to is that it is
/// *true*: the layer is Natural Earth's urban-area polygons, so the Ruhr is a
/// sprawl, the Nile is a thread and the Sahara is empty, exactly as the
/// photographs from orbit show them. Bright enough to read the continents by
/// on the dark half, nowhere near enough to compete with the terminator.
const CITY_GLOW = 0.55;

/// Rivers are a physical feature rather than an agreed line, so unlike the
/// borders they are *not* self-lit at night: after dark a river is as dark as
/// the ground it runs through, and the city lights beside it are what the eye
/// follows instead.
const RIVER_MIX = 0.55;

fn srgb_to_linear(c: vec3<f32>) -> vec3<f32> {
    let lo = c / 12.92;
    let hi = pow((c + vec3(0.055)) / 1.055, vec3(2.4));
    return select(hi, lo, c <= vec3(0.04045));
}

fn hash3(p: vec3<f32>) -> f32 {
    return fract(sin(dot(p, vec3(12.9898, 78.233, 37.719))) * 43758.5453);
}

fn vnoise(p: vec3<f32>) -> f32 {
    let i = floor(p);
    let f = fract(p);
    let u = f * f * (3.0 - 2.0 * f);
    let c000 = hash3(i + vec3(0.0, 0.0, 0.0));
    let c100 = hash3(i + vec3(1.0, 0.0, 0.0));
    let c010 = hash3(i + vec3(0.0, 1.0, 0.0));
    let c110 = hash3(i + vec3(1.0, 1.0, 0.0));
    let c001 = hash3(i + vec3(0.0, 0.0, 1.0));
    let c101 = hash3(i + vec3(1.0, 0.0, 1.0));
    let c011 = hash3(i + vec3(0.0, 1.0, 1.0));
    let c111 = hash3(i + vec3(1.0, 1.0, 1.0));
    let x00 = mix(c000, c100, u.x);
    let x10 = mix(c010, c110, u.x);
    let x01 = mix(c001, c101, u.x);
    let x11 = mix(c011, c111, u.x);
    return mix(mix(x00, x10, u.y), mix(x01, x11, u.y), u.z);
}

/// Four octaves of value noise, the workhorse behind every procedural surface
/// here. Sampled on the body-space normal, so it turns with the body instead of
/// swimming across it.
fn fbm(p: vec3<f32>) -> f32 {
    var v = 0.0;
    var amp = 0.5;
    var q = p;
    for (var i = 0; i < 4; i = i + 1) {
        v = v + amp * vnoise(q);
        q = q * 2.03;
        amp = amp * 0.5;
    }
    return v;
}

/// Ridged noise: the |·| fold turns smooth blobs into creases, which is what
/// reads as fractured ice or a crater rim rather than as cloud.
fn ridged(p: vec3<f32>) -> f32 {
    return 1.0 - abs(vnoise(p) * 2.0 - 1.0);
}

fn granulation(n: vec3<f32>) -> f32 {
    return vnoise(n * 34.0) * 0.55 + vnoise(n * 91.0) * 0.30 + vnoise(n * 210.0) * 0.15;
}

struct VsOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) uv: vec2<f32>,
    @location(1) nrm: vec3<f32>,
    @location(2) world: vec3<f32>,
    // The same normal in the *body* frame, which is where latitude, longitude
    // and every surface feature live.
    @location(3) body: vec3<f32>,
};

@vertex
fn vs(@location(0) pos: vec3<f32>, @location(1) uv: vec2<f32>) -> VsOut {
    var o: VsOut;
    let world = d.model * vec4(pos, 1.0);
    o.clip = g.view_proj * world;
    o.world = world.xyz;
    // The mesh is a unit sphere, so its position *is* its object-space normal.
    o.nrm = normalize((d.basis * vec4(pos, 0.0)).xyz);
    o.body = normalize(pos);
    o.uv = uv;
    return o;
}

/// Latitude and east longitude of a body-frame direction, degrees.
fn lat_lon(body: vec3<f32>) -> vec2<f32> {
    return vec2(degrees(asin(clamp(body.z, -1.0, 1.0))), degrees(atan2(body.y, body.x)));
}

fn sun_dir(in: VsOut) -> vec3<f32> {
    return normalize(g.sun_pos.xyz - in.world);
}

/// The day/night ramp every solid body shares. Softer than the geometric
/// terminator, because the Sun is half a degree wide.
fn daylight(n: vec3<f32>, to_sun: vec3<f32>) -> f32 {
    return smoothstep(-0.05, 0.12, dot(n, to_sun));
}

// ── The Moon's shadow ───────────────────────────────────────────────────────
//
// A solar eclipse, computed at TRUE scale whatever the exaggeration sliders
// say. The Moon the viewer sees may be drawn several times its size on a
// compressed orbit, but a shadow that swelled and wandered with the sliders
// would put the eclipse track over the wrong countries. So the occlusion test
// uses the real geometry — the Moon's true position and radius against the
// true Sun — and paints the result on the surface *by direction from the
// Earth's centre*, the one mapping that scaling the Earth itself preserves.
// The footprint is therefore geographically true, and follows the Earth's own
// exaggeration and nothing else's.

/// True radii, gigametres — `ephem::{SUN_R, EARTH_R, MOON_R}`.
const TRUE_SUN_R = 0.6957;
const TRUE_EARTH_R = 0.006371;
const TRUE_MOON_R = 0.0017374;

/// Fraction of the Sun's disc a blocker of radius `br` covers, as seen from a
/// point with (unnormalised) directions `to_sun` and `to_blocker`: 0 in the
/// open, 1 inside the umbra.
fn sun_covered(to_sun: vec3<f32>, to_blocker: vec3<f32>, br: f32) -> f32 {
    let su = normalize(to_sun);
    let bu = normalize(to_blocker);
    // Only a blocker on the Sun's side of the sky can stand in front of it.
    if (dot(su, bu) <= 0.0) {
        return 0.0;
    }
    // Angular radii and separation, small-angle throughout: everything here
    // is under two degrees. The separation comes from the cross product — the
    // acos of a dot this close to 1 would lose most of its bits in f32.
    let rs = TRUE_SUN_R / length(to_sun);
    let rb = br / length(to_blocker);
    let sep = length(cross(su, bu));
    // Fraction of the Sun's *diameter* covered, then of its area — the
    // smoothstep is a close fit to the lens-overlap area when the discs are
    // of comparable size. An annular eclipse falls out for free: rb < rs caps
    // the fraction below 1 and the shadow never quite goes dark.
    let f = clamp((rs + rb - sep) / (2.0 * rs), 0.0, 1.0);
    return f * f * (3.0 - 2.0 * f);
}

/// How much of the Sun's light reaches the Earth-surface point with world
/// normal `n`, with the Moon in the way: 1 in the open, 0 in the umbra.
/// `centre` is the Earth's world position and `moon_off` the Moon's TRUE
/// geocentric offset (`DrawData::tint2`); a zeroed offset means no Moon on
/// this draw.
fn eclipse_light(centre: vec3<f32>, moon_off: vec3<f32>, n: vec3<f32>) -> f32 {
    if (dot(moon_off, moon_off) < 1e-6) {
        return 1.0;
    }
    // The surface point, relative to the Earth's centre, at true scale. The
    // Moon's parallax across the Earth's disc is a degree — bigger than the
    // Sun itself — so the test is per point, not per planet.
    let p = n * TRUE_EARTH_R;
    return 1.0 - sun_covered((g.sun_pos.xyz - centre) - p, moon_off - p, TRUE_MOON_R);
}

/// The mirror case: how much sunlight reaches the Moon-surface point with
/// world normal `n`, with the Earth in the way — the lunar eclipse. Same true
/// scale, same reasoning: the shadow's sweep across the Moon must not care
/// how the sliders drew either body.
fn lunar_eclipse_light(n: vec3<f32>) -> f32 {
    // The Moon's true geocentric offset, as on the Earth draw; `w` is the
    // orbit compression, from which the Earth's centre is recovered out of
    // this draw's own (rendered, possibly compressed) position.
    let off = d.tint2.xyz;
    if (dot(off, off) < 1e-6) {
        return 1.0;
    }
    let earth = d.model[3].xyz - off * d.tint2.w;
    // The surface point at true scale, kept relative to the Earth so the
    // ~150 Gm world coordinate never meets the sub-Gm eclipse geometry in f32.
    let p = off + n * TRUE_MOON_R;
    // Danjon's enlargement: the Earth's air throws its shadow a couple of
    // per cent wider than the solid globe does.
    return 1.0 - sun_covered((g.sun_pos.xyz - earth) - p, -p, TRUE_EARTH_R * 1.02);
}

fn shade_earth(in: VsOut, n: vec3<f32>) -> vec3<f32> {
    let to_sun = sun_dir(in);
    // A soft terminator: the Sun is half a degree wide and the atmosphere
    // scatters well past the geometric line. The eclipse factor folds the
    // Moon's shadow into the same term, so inside the umbra the surface reads
    // as a patch of night — line glow and all.
    let day = smoothstep(-0.06, 0.16, dot(n, to_sun))
        * eclipse_light(d.model[3].xyz, d.tint2.xyz, n);

    let land = textureSample(land_tex, samp, in.uv).r;

    // The shoreline is the ½ contour of the land-coverage field, stroked to a
    // fixed width in *pixels*.
    //
    // `fwidth` is how much the field changes from one pixel to the next, so
    // dividing the distance-from-½ by it converts that distance into pixels
    // directly — no matter which mip is in play or how the sphere is
    // foreshortened. That is the whole trick: a fixed offset in *texels* (which
    // is what a gradient of neighbouring taps is) draws a hairline on a globe
    // 40 px across and a three-texel smear once the camera is down at the
    // surface, and it is that smear the eye reads as a thick, blurry coast.
    // This is the same hairline at both.
    let px = abs(land - 0.5) / max(fwidth(land), 1e-4);
    let coast = 1.0 - smoothstep(COAST_PX, COAST_PX + 1.0, px);

    // The FT8 map's palette is tuned for sparse dots on a dark panel; filling a
    // whole globe with it at 1× reads as almost black, so the daylit side is
    // lifted well above the flat colour while keeping its hue.
    var col = mix(srgb_to_linear(OCEAN_DAY), srgb_to_linear(LAND_DAY), land);
    col = col * (0.05 + 2.6 * day);
    // Night side: land stays faintly visible with a cyan glow, like a city map.
    col += srgb_to_linear(COAST) * land * (1.0 - day) * 0.045;
    col = mix(col, srgb_to_linear(COAST) * (0.35 + 0.9 * day), coast * (0.45 + 0.55 * day));

    // International borders, from the same Natural Earth data as the coastline
    // and drawn dimmer than it: on a globe the coast is the shape you navigate
    // by, and a border that competed with it would bury the map.
    let border = textureSample(border_tex, samp, in.uv).r;
    col = mix(col, srgb_to_linear(COAST) * (0.30 + 0.55 * day), border * 0.5 * (0.35 + 0.65 * day));

    // Rivers, from the same Natural Earth data, drawn under the borders and in
    // water's own colour: on a map with this much cyan on it, a line that is
    // meant to read as water has to be the one blue thing on the ground.
    let river = textureSample(river_tex, samp, in.uv).r;
    col = mix(col, srgb_to_linear(RIVER) * (0.25 + 2.2 * day), river * RIVER_MIX * day);

    // Both line layers glow faintly once the Sun is off them. Emitted rather
    // than mixed, and ramped in on the same soft terminator the shading uses,
    // so it arrives the way city lights do instead of at a hard edge.
    let night = 1.0 - day;
    col += srgb_to_linear(COAST) * night * (coast * COAST_GLOW + border * BORDER_GLOW);

    // ...and the cities light up, which is the one thing on this globe a
    // photograph would agree with. Squared, because that is roughly what the
    // eye does with a field of point sources seen from orbit: a dense core
    // reads far brighter than twice a sparse edge, and the ramp keeps the
    // suburbs from smearing every city into one blob.
    let city = textureSample(city_tex, samp, in.uv).r;
    col += srgb_to_linear(CITY) * night * city * city * CITY_GLOW;

    // Atmospheric limb. Brightest on the daylit edge, which is what gives the
    // globe its depth against a black background.
    let to_eye = normalize(g.camera_pos.xyz - in.world);
    let rim = pow(1.0 - clamp(dot(n, to_eye), 0.0, 1.0), 3.0);
    col += srgb_to_linear(ATMO) * rim * (0.06 + 0.55 * day);
    return col;
}

// ── The Moon ────────────────────────────────────────────────────────────────
//
// From a photograph, unlike everything else here that is not the Earth or the
// Sun: NASA's LRO Wide Angle Camera albedo mosaic (`assets/bodies/moon.jpg`).
// Procedural maria were tried and thrown away — the near side is the one
// surface in the solar system that every viewer already knows by heart, and
// noise-and-ellipses lands Imbrium in the wrong place in a way that reads as
// broken rather than as stylised.
//
// The frame the Moon is drawn in is tidally locked (`ephem::moon_basis`), so
// the mesh's own UVs index the map directly: u = 0.5 is the sub-Earth point,
// which is exactly where the map's 0° meridian is.

fn shade_moon(in: VsOut, n: vec3<f32>) -> vec3<f32> {
    let to_sun = sun_dir(in);
    let day = smoothstep(-0.03, 0.08, dot(n, to_sun));
    let sunlight = lunar_eclipse_light(n);
    let albedo = textureSample(body_maps, samp, in.uv, i32(d.style.y)).rgb;

    // The Moon is famously flat-looking at full — the regolith backscatters
    // straight at the light source — so the falloff is deliberately blunt
    // until close to the terminator, where the relief suddenly stands up.
    let mu = clamp(dot(n, to_sun), 0.0, 1.0);
    let backscatter = 0.55 + 0.45 * pow(mu, 0.35);
    // A little extra bite along the terminator, where real shadows are long.
    let grazing = 1.0 - smoothstep(0.0, 0.4, mu);
    let relief = 1.0 + (ridged(in.body * 90.0) - 0.45) * grazing * 0.9;
    // Earthshine on the night side: the same faint blue-grey the naked eye
    // sees on the dark limb of a crescent.
    let night = vec3<f32>(0.06, 0.075, 0.10) * (1.0 - day);
    // Inside the umbra the Moon is not black but copper: sunlight refracted
    // through the ring of the Earth's atmosphere, reddened the same way a
    // sunset is — the one fact about a total lunar eclipse everyone knows.
    // The cube keeps the colour out of the penumbra, which to the eye is only
    // a slight dimming of a still-full Moon.
    let umbra = pow(1.0 - sunlight, 3.0);
    return albedo * (1.35 * day * sunlight * backscatter * relief)
        + albedo * vec3<f32>(0.55, 0.16, 0.06) * (day * umbra * 0.35)
        + albedo * night;
}

// ── Everything else ─────────────────────────────────────────────────────────

/// Methane blue, nearly featureless: Uranus and Neptune.
fn shade_ice_giant(in: VsOut, n: vec3<f32>, to_sun: vec3<f32>) -> vec3<f32> {
    let ll = lat_lon(in.body);
    let band = sin((ll.x / 90.0 * 5.0 + (fbm(in.body * 4.0) - 0.5) * 0.4) * PI);
    var col = mix(d.tint2.rgb, d.tint.rgb, 0.5 + 0.5 * smoothstep(-0.8, 0.8, band));
    // A soft bright pole, which is what both actually show.
    col = mix(col, d.tint.rgb * 1.15, smoothstep(0.55, 1.0, abs(ll.x / 90.0)) * 0.35);
    let day = daylight(n, to_sun);
    // Deep atmosphere: strongly limb-darkened, which is most of what makes
    // these two read as balls of gas rather than as painted marbles.
    let to_eye = normalize(g.camera_pos.xyz - in.world);
    let mu = clamp(dot(n, to_eye), 0.0, 1.0);
    return col * (0.02 + 1.1 * day) * (0.55 + 0.45 * pow(mu, 0.5));
}

/// Airless, saturated with craters: Mercury, Ganymede, Callisto, the two
/// Martian rocks. `d.style.z` adds Iapetus's dark leading hemisphere.
fn shade_cratered(in: VsOut, n: vec3<f32>, to_sun: vec3<f32>) -> vec3<f32> {
    let craters = ridged(in.body * 30.0) * 0.5 + ridged(in.body * 85.0) * 0.32
                + ridged(in.body * 220.0) * 0.18;
    let basins = fbm(in.body * 3.5);
    var col = mix(d.tint2.rgb, d.tint.rgb, clamp(craters * 0.8 + basins * 0.5, 0.0, 1.0));

    // Iapetus: the leading hemisphere is nearly black, the trailing one is
    // clean ice. In a tidally locked frame the leading side is −y.
    if (d.style.z > 0.5) {
        let leading = clamp(-in.body.y, 0.0, 1.0);
        col = mix(col, col * 0.12, smoothstep(0.15, 0.8, leading));
    }
    let day = daylight(n, to_sun);
    let mu = clamp(dot(n, to_sun), 0.0, 1.0);
    // Rough regolith: bright to the terminator, then a hard edge.
    return col * (0.015 + 1.2 * day * (0.6 + 0.4 * pow(mu, 0.4)));
}

/// Ice, bright and fractured: Europa, Enceladus, the Saturnian and Uranian
/// moons, Triton.
fn shade_icy(in: VsOut, n: vec3<f32>, to_sun: vec3<f32>) -> vec3<f32> {
    // Long curved fractures, from ridged noise stretched along one axis.
    let cracks = pow(ridged(in.body * vec3(9.0, 9.0, 3.0)), 6.0)
               + pow(ridged(in.body * vec3(21.0, 6.0, 21.0)), 8.0) * 0.6;
    let mottle = 0.88 + 0.12 * fbm(in.body * 12.0);
    var col = d.tint.rgb * mottle;
    col = mix(col, d.tint2.rgb, clamp(cracks, 0.0, 1.0) * 0.55);
    let day = daylight(n, to_sun);
    return col * (0.02 + 1.3 * day);
}

/// Io: sulphur yellows over dark volcanic paterae.
fn shade_volcanic(in: VsOut, n: vec3<f32>, to_sun: vec3<f32>) -> vec3<f32> {
    let blotch = fbm(in.body * 7.0);
    let vents = smoothstep(0.62, 0.78, ridged(in.body * 17.0));
    var col = mix(d.tint.rgb, d.tint2.rgb, smoothstep(0.35, 0.7, blotch));
    col = mix(col, vec3(0.22, 0.10, 0.06), vents * 0.7);
    // Sulphur dioxide frost near the poles.
    col = mix(col, vec3(0.85, 0.83, 0.70), smoothstep(0.72, 1.0, abs(in.body.z)) * 0.4);
    let day = daylight(n, to_sun);
    return col * (0.02 + 1.25 * day);
}

/// An opaque atmosphere with nothing to see through it: Venus, Titan.
fn shade_cloud(in: VsOut, n: vec3<f32>, to_sun: vec3<f32>, haze: f32) -> vec3<f32> {
    // Very low contrast: the whole point of both bodies is that the surface is
    // not visible, and any strong feature here would be a lie about them.
    let swirl = fbm(in.body * vec3(2.5, 2.5, 6.0));
    var col = mix(d.tint2.rgb, d.tint.rgb, 0.35 + 0.65 * swirl);
    let day = daylight(n, to_sun);
    let to_eye = normalize(g.camera_pos.xyz - in.world);
    let mu = clamp(dot(n, to_eye), 0.0, 1.0);
    // The haze layer stands above the limb and glows there — Titan's is
    // detached and obvious, Venus's is a thin bright ring.
    let rim = pow(1.0 - mu, 3.0) * (0.15 + 0.85 * day);
    return col * (0.02 + 1.1 * day) * (0.7 + 0.3 * pow(mu, 0.4)) + d.tint.rgb * rim * haze;
}

/// A body drawn from a real map: `d.style.y` picks the layer.
///
/// Only the lighting is added here — the surface itself is a photograph, so
/// there is nothing to invent. Gas giants get the limb darkening a deep
/// atmosphere has, and Mars gets close enough to it from its own dust; the Moon
/// has its own branch because regolith does not behave like anything else here.
fn shade_mapped(in: VsOut, n: vec3<f32>) -> vec3<f32> {
    let to_sun = sun_dir(in);
    let albedo = textureSample(body_maps, samp, in.uv, i32(d.style.y)).rgb;
    let day = daylight(n, to_sun);
    let to_eye = normalize(g.camera_pos.xyz - in.world);
    let mu = clamp(dot(n, to_eye), 0.0, 1.0);
    var col = albedo * (0.02 + 1.15 * day) * (0.62 + 0.38 * pow(mu, 0.45));
    // A thin dusty atmosphere, for the bodies that have one: a faint warm limb,
    // nothing like the Earth's. `d.style.w` carries its strength, and the tint
    // is the body's own average colour, which is what the dust suspended in it
    // is made of.
    let rim = pow(1.0 - mu, 4.0);
    return col + d.tint.rgb * rim * day * d.style.w;
}

fn shade_body(in: VsOut, n: vec3<f32>) -> vec3<f32> {
    let to_sun = sun_dir(in);
    let style = d.style.x;
    if (style < 0.5) {
        return shade_cratered(in, n, to_sun);
    } else if (style < 1.5) {
        return shade_cloud(in, n, to_sun, 0.25);
    } else if (style < 2.5) {
        return shade_ice_giant(in, n, to_sun);
    } else if (style < 3.5) {
        return shade_icy(in, n, to_sun);
    } else if (style < 4.5) {
        return shade_volcanic(in, n, to_sun);
    } else if (style < 5.5) {
        return shade_cloud(in, n, to_sun, 0.75);
    }
    return shade_mapped(in, n);
}

fn shade_sun(in: VsOut, n: vec3<f32>) -> vec3<f32> {
    let e = g.sun_to_earth.xyz;
    // Solar north projected perpendicular to the view-from-Earth axis is "up"
    // in an SDO frame (the P angle is already removed from the browse images);
    // `rt` completes the pair, with the sign that puts heliographic west on the
    // right as seen from Earth.
    let up = normalize(g.solar_north.xyz - e * dot(g.solar_north.xyz, e));
    let rt = cross(up, e) * g.solar_north.w;
    // Facing fraction: >0 is the Earth-facing hemisphere SDO can see.
    let c = dot(n, e);

    // Limb darkening relative to the *camera*, which is what makes the sphere
    // read as a sphere from any viewpoint.
    let to_eye = normalize(g.camera_pos.xyz - in.world);
    let mu = clamp(dot(n, to_eye), 0.0, 1.0);
    let ld = 0.35 + 0.65 * pow(mu, 0.55);

    let base = d.tint.rgb * ld * (0.90 + 0.16 * granulation(n));

    // The SDO disk is an orthographic photograph of the Earth-facing side, so a
    // surface point's image coordinate is exactly its component along rt/up, in
    // solar radii — no perspective divide. It smears badly towards the limb
    // (infinite surface area compressed into no texture area), so it is faded
    // out well before the edge and the procedural surface takes over.
    let disk = g.sun_to_earth.w;
    let uv = vec2(0.5 + dot(n, rt) * disk, 0.5 - dot(n, up) * disk);
    let photo = textureSample(sun_tex, samp, uv).rgb;
    let w = g.misc.y * smoothstep(0.05, 0.35, c);
    return mix(base, photo * (0.55 + 0.45 * ld), w);
}

@fragment
fn fs(in: VsOut) -> @location(0) vec4<f32> {
    let n = normalize(in.nrm);
    let mode = d.params.x;
    var col: vec3<f32>;
    if (mode < 0.5) {
        col = shade_earth(in, n);
    } else if (mode < 1.5) {
        col = shade_moon(in, n);
    } else if (mode < 2.5) {
        col = shade_sun(in, n);
    } else {
        col = shade_body(in, n);
    }
    return vec4(col, 1.0);
}
