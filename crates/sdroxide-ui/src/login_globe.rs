//! The turning globe behind the sign-in screen.
//!
//! A backdrop, not a view: nothing here is data, nothing is interactive, and
//! the whole thing is drawn softened so the two boxes in front of it stay the
//! thing the eye lands on. What it is *for* is telling the operator, before
//! they have typed anything, that the machine on the other end of this socket
//! is a radio station.
//!
//! Three pieces:
//!
//! * [`Sky`] — the CPU half. Where the globe has turned to, and a handful of
//!   contacts being worked, each between two points that are actually on land.
//! * `shaders/login_globe.wgsl` — one fullscreen triangle, one analytic sphere,
//!   the same two Natural Earth maps the 3D view draws the Earth from.
//! * `shaders/login_blur.wgsl` — the softening, on the way into egui's pass.
//!
//! Resources are built on first use rather than at startup. Most sessions never
//! see this screen at all — every local one, and every remote one against a
//! server that asks for nothing — and decoding an 8192×4096 map for them would
//! be a tenth of a second spent on a picture nobody was ever shown.

use eframe::egui_wgpu::{CallbackResources, CallbackTrait, RenderState, ScreenDescriptor, wgpu};

use crate::basemap::{BORDER_PNG, LAND_PNG};

/// Live contacts at once. Also the array length in the shader — the two must
/// agree, which the test at the bottom of this file checks.
pub const MAX_ARCS: usize = 16;

/// How much smaller than the widget the scene is rendered. The blur's taps
/// reach this many times further in screen pixels than in texels, which is most
/// of where the softness comes from; the rest is the resample on the way back
/// up. Three is as far as this can go before the coastline stops reading as a
/// coastline.
const DOWNSCALE: u32 = 3;

/// The offscreen target is allocated in steps of this many pixels, so dragging
/// a window edge re-uses one allocation instead of making a new one every
/// frame. The UVs take up the slack.
const ALLOC_STEP: u32 = 64;

/// The maps are the 3D view's, re-uploaded rather than shared: that view builds
/// its resources when it is first opened, which on a client sitting at a
/// sign-in screen has not happened and should not be made to. Capped well below
/// the source resolution because this globe is blurred — 8192 texels of
/// coastline would be filtered straight back off.
const MAP_DIM: u32 = 2048;

/// Turns in this many seconds. A little over four times real time at the
/// equator, which is slow enough to read as drifting rather than spinning and
/// fast enough that the movement is visible while a password is typed.
const SPIN_PERIOD: f32 = 360.0;

/// Distance from the globe's centre, in radii.
const EYE_DIST: f32 = 2.6;

/// How much of the frame's half-height the globe's radius covers on a square
/// window. Over one on purpose: this is a *close* view, so the disc runs off
/// the top and bottom and the limb is what sweeps through the picture, rather
/// than a whole marble sitting in a field of black.
const FILL: f32 = 1.16;

/// How much further it grows on a wide one, per unit of aspect ratio, and how
/// far that growth is allowed to run.
///
/// A fixed vertical framing would leave a 16:9 desktop with the globe down the
/// middle and a third of the width black either side. Growing it sub-linearly
/// fills that without letting an ultrawide swallow the limb entirely — past
/// the cap the picture would be an unbroken wall of ocean.
const FILL_PER_ASPECT: f32 = 0.30;
const FILL_ASPECT_CAP: f32 = 1.6;

/// Where the eye sits off the axis, in radii. Pushes the globe down and a
/// little right, so the limb crosses the upper third and the open sky is above
/// — which is where a card centred on the window is not.
const EYE_OFFSET: [f32; 2] = [-0.10, 0.40];

/// Axial tilt, and the sun's bearing. Neither is the real one for today: this
/// is a backdrop, and a terminator that happens to fall straight down the
/// middle on an equinox afternoon would look like a rendering fault.
const TILT_DEG: f32 = 21.0;
const SUN_DIR: [f32; 3] = [0.42, 0.30, 0.86];

/// Arc line half-width and station dot radius, in radii and radians.
const ARC_WIDTH: f32 = 0.0075;
const DOT_RADIUS: f32 = 0.030;

// ── The CPU half ─────────────────────────────────────────────────────────────

/// One contact being worked: two points on the globe and how far along it is.
#[derive(Clone, Copy)]
struct Contact {
    /// Both ends as unit vectors in the globe's own frame, so the spin carries
    /// them without any per-arc work.
    a: [f32; 3],
    b: [f32; 3],
    /// Seconds since this one started.
    age: f32,
    /// How long it runs, start to gone.
    life: f32,
}

impl Contact {
    /// How much of the path has been drawn, and how brightly, at `age`.
    ///
    /// Deliberately shaped like a QSO rather than like a fade: the line reaches
    /// out, the pair sit there worked for a while, and then both let go.
    fn phase(&self) -> (f32, f32) {
        let reach = (self.life * 0.28).max(0.6);
        let drawn = (self.age / reach).clamp(0.0, 1.0);
        // Ease out, so it slows as it arrives instead of stopping dead.
        let drawn = 1.0 - (1.0 - drawn) * (1.0 - drawn);
        let fade_in = (self.age / 0.8).clamp(0.0, 1.0);
        let left = self.life - self.age;
        let fade_out = (left / 2.2).clamp(0.0, 1.0);
        (drawn.max(0.02), fade_in * fade_out)
    }
}

/// The backdrop's own state: where the globe has turned to, and who is working
/// whom. Cheap to keep — a few hundred bytes — and stepped from the frame time,
/// so it neither runs away on a fast machine nor stalls on a slow one.
pub struct Sky {
    contacts: [Contact; MAX_ARCS],
    /// Frame time of the last step, for the elapsed-seconds difference.
    last: Option<f64>,
    spin: f32,
    rng: u64,
}

impl Default for Sky {
    fn default() -> Self {
        // Seeded from the wall clock so two clients side by side are not
        // working the same list of contacts in lockstep. A fixed fallback
        // rather than a panic if there is no clock: identical arcs beat none.
        let seed = (crate::time::now_unix_f64() * 1000.0) as u64;
        let blank = Contact { a: [0.0, 0.0, 1.0], b: [0.0, 0.0, 1.0], age: 0.0, life: 1.0 };
        let mut sky = Sky { contacts: [blank; MAX_ARCS], last: None, spin: 0.0, rng: seed | 1 };
        for i in 0..MAX_ARCS {
            let mut c = sky.spawn();
            // Staggered, so the screen does not open with sixteen contacts
            // starting at once and then all ending together.
            c.age = sky.frac() * c.life;
            sky.contacts[i] = c;
        }
        sky
    }
}

impl Sky {
    /// xorshift64*. A backdrop needs unpredictable-looking, not unpredictable,
    /// and this is eight bytes of state with no dependency behind it.
    fn next_u64(&mut self) -> u64 {
        let mut x = self.rng;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.rng = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    fn frac(&mut self) -> f32 {
        (self.next_u64() >> 40) as f32 / (1u64 << 24) as f32
    }

    fn range(&mut self, lo: f32, hi: f32) -> f32 {
        lo + self.frac() * (hi - lo)
    }

    /// A point on land, as a unit vector in the globe's frame.
    ///
    /// Rejection-sampled against the same land mask the flat map is drawn from,
    /// so the stations sit where stations are. Latitude is drawn from a band
    /// rather than uniformly: the poles are land at one end and ice at the
    /// other, and a contact into either reads as a mistake.
    fn land_point(&mut self) -> [f32; 3] {
        for _ in 0..64 {
            let lon = self.range(-180.0, 180.0);
            let lat = self.range(-56.0, 68.0);
            if crate::basemap::is_land(lon as f64, lat as f64) {
                return ecef(lon, lat);
            }
        }
        // The mask has land on a third of its area, so sixty-four misses is
        // not something that happens; somewhere sensible beats a panic if it
        // ever does.
        ecef(self.range(-10.0, 30.0), self.range(35.0, 60.0))
    }

    /// A fresh contact: two stations far enough apart to be worth an arc.
    fn spawn(&mut self) -> Contact {
        let a = self.land_point();
        let mut b = self.land_point();
        for _ in 0..24 {
            let sep = dot(a, b).clamp(-1.0, 1.0).acos().to_degrees();
            // Under 20° is a local contact the arc cannot resolve; over 165°
            // is near-antipodal, where the great circle through the two is
            // barely defined and the path would swing anywhere.
            if (20.0..=165.0).contains(&sep) {
                break;
            }
            b = self.land_point();
        }
        Contact { a, b, age: 0.0, life: self.range(9.0, 17.0) }
    }

    /// Advance to frame time `now` (egui seconds). Idempotent within a frame
    /// and safe across a pause: a gap longer than a step is clamped rather than
    /// jumping the globe half a turn.
    pub fn step(&mut self, now: f64) {
        let dt = match self.last.replace(now) {
            Some(prev) => ((now - prev) as f32).clamp(0.0, 0.1),
            None => 0.0,
        };
        self.spin = (self.spin + dt / SPIN_PERIOD).fract();
        for i in 0..MAX_ARCS {
            self.contacts[i].age += dt;
            if self.contacts[i].age >= self.contacts[i].life {
                let fresh = self.spawn();
                self.contacts[i] = fresh;
            }
        }
    }
}

// ── Uniforms ─────────────────────────────────────────────────────────────────

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct GlobeUniform {
    rot: [[f32; 4]; 3],
    camera: [f32; 4],
    sun: [f32; 4],
    misc: [f32; 4],
    arcs: [[f32; 4]; MAX_ARCS * 2],
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct BlurUniform {
    uv_scale: [f32; 4],
    params: [f32; 4],
}

impl Sky {
    /// Everything the shader needs for this frame.
    fn uniform(&self, aspect: f32) -> GlobeUniform {
        let sun = norm(SUN_DIR);
        let mut u = GlobeUniform {
            rot: local_from_world(self.spin * std::f32::consts::TAU, TILT_DEG.to_radians()),
            camera: [EYE_OFFSET[0], EYE_OFFSET[1], EYE_DIST, focal(aspect)],
            sun: [sun[0], sun[1], sun[2], 0.45],
            misc: [aspect, ARC_WIDTH, DOT_RADIUS, 0.0],
            arcs: [[0.0; 4]; MAX_ARCS * 2],
        };
        for (i, c) in self.contacts.iter().enumerate() {
            let (drawn, alpha) = c.phase();
            u.arcs[i * 2] = [c.a[0], c.a[1], c.a[2], drawn];
            u.arcs[i * 2 + 1] = [c.b[0], c.b[1], c.b[2], alpha];
        }
        u
    }
}

// ── Small vector helpers ─────────────────────────────────────────────────────
//
// Written out rather than pulled from `solar3d::math`, which works in the
// scene's gigametre units and carries a projection this backdrop has no use
// for. Everything below is in globe radii, where the sphere has radius exactly
// 1 — the scale f32 resolves best, and the reason the spin here is smooth.

/// Lens focal length for a viewport of this aspect ratio, as the shader wants
/// it: 1/tan(½ vertical fov).
///
/// From the framing the other way round. The disc's angular radius is
/// `asin(1/D)`, so its projected radius in half-heights is
/// `tan(asin(1/D)) / tan(½fov)`, which is `focal / √(D²−1)`. Solving that for
/// the coverage [`FILL`] asks for is the line below.
fn focal(aspect: f32) -> f32 {
    let grow = 1.0 + FILL_PER_ASPECT * (aspect - 1.0).clamp(0.0, FILL_ASPECT_CAP);
    // Never large enough to put the corners inside the disc. On a tall narrow
    // window a fixed vertical framing does exactly that, and the backdrop
    // stops being a globe and becomes a blue wall with some lines on it — the
    // limb is the whole reason this reads as a planet.
    let corner = (1.0 + aspect * aspect).sqrt();
    let fill = (FILL * grow).min(corner * 0.92);
    fill * (EYE_DIST * EYE_DIST - 1.0).sqrt()
}

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn norm(v: [f32; 3]) -> [f32; 3] {
    let l = dot(v, v).sqrt().max(1e-9);
    [v[0] / l, v[1] / l, v[2] / l]
}

/// Degrees of longitude and latitude to the unit vector the maps are indexed
/// by: +X at (0°N, 0°E), +Z at the north pole — the convention
/// `solar3d::mesh::sphere` builds and `crate::basemap` is rasterised in.
fn ecef(lon_deg: f32, lat_deg: f32) -> [f32; 3] {
    let (lon, lat) = (lon_deg.to_radians(), lat_deg.to_radians());
    [lat.cos() * lon.cos(), lat.cos() * lon.sin(), lat.sin()]
}

/// The 3×3 that takes a world direction into the globe's own frame, as three
/// padded rows.
///
/// Built as the inverse of "spin about the pole, stand the pole up, then lean
/// it": `world_from_local = Rz(tilt) · B · Rz(spin)`, where `B` turns the
/// map's +Z pole into screen-up. Inverting a rotation is transposing it, and
/// the transpose of a product reverses it, which is the whole derivation.
fn local_from_world(spin: f32, tilt: f32) -> [[f32; 4]; 3] {
    // Rz(−spin) · Bᵀ · Rz(−tilt), multiplied out.
    let (ss, cs) = (-spin).sin_cos();
    let (st, ct) = (-tilt).sin_cos();
    // Bᵀ: world → the upright globe. B maps local +Z to world +Y and local +Y
    // to world −Z, so its transpose maps world +Y back to local +Z.
    let bt = [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]];
    let rz_tilt = [[ct, -st, 0.0], [st, ct, 0.0], [0.0, 0.0, 1.0]];
    let rz_spin = [[cs, -ss, 0.0], [ss, cs, 0.0], [0.0, 0.0, 1.0]];
    let m = mul3(&mul3(&rz_spin, &bt), &rz_tilt);
    [
        [m[0][0], m[0][1], m[0][2], 0.0],
        [m[1][0], m[1][1], m[1][2], 0.0],
        [m[2][0], m[2][1], m[2][2], 0.0],
    ]
}

fn mul3(a: &[[f32; 3]; 3], b: &[[f32; 3]; 3]) -> [[f32; 3]; 3] {
    let mut out = [[0.0f32; 3]; 3];
    for (r, row) in out.iter_mut().enumerate() {
        for (c, cell) in row.iter_mut().enumerate() {
            *cell = (0..3).map(|k| a[r][k] * b[k][c]).sum();
        }
    }
    out
}

// ── GPU resources ────────────────────────────────────────────────────────────

struct Target {
    alloc: (u32, u32),
    view: wgpu::TextureView,
    blur_bg: wgpu::BindGroup,
    _tex: wgpu::Texture,
}

pub struct GlobeResources {
    globe_pipeline: wgpu::RenderPipeline,
    globe_bg: wgpu::BindGroup,
    globe_buf: wgpu::Buffer,
    blur_pipeline: wgpu::RenderPipeline,
    blur_bgl: wgpu::BindGroupLayout,
    blur_buf: wgpu::Buffer,
    sampler: wgpu::Sampler,
    target: Option<Target>,
}

/// Colour format of the private target. Chosen rather than taken from the
/// surface: it is renderable and sampleable everywhere including WebGL2, and
/// being sRGB on both sides of the blit means the softening happens in the
/// space the eye reads, not in raw eighth-bit values.
const TARGET_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8UnormSrgb;

/// Build the pipelines and upload the maps, once.
///
/// Cheap to call every frame after the first — it checks and returns. Called
/// from the sign-in screen rather than from an app constructor, so a session
/// that never signs in never pays for it.
pub fn ensure(rs: &RenderState) {
    if rs.renderer.read().callback_resources.get::<GlobeResources>().is_some() {
        return;
    }
    let device = &rs.device;

    let globe_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("login-globe"),
        source: wgpu::ShaderSource::Wgsl(include_str!("shaders/login_globe.wgsl").into()),
    });
    let blur_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("login-blur"),
        source: wgpu::ShaderSource::Wgsl(include_str!("shaders/login_blur.wgsl").into()),
    });

    let limit = MAP_DIM.min(device.limits().max_texture_dimension_2d);
    let land = upload_map(device, &rs.queue, "login-land", LAND_PNG, limit);
    let border = upload_map(device, &rs.queue, "login-borders", BORDER_PNG, limit);

    let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
        label: Some("login-globe-sampler"),
        // The map wraps at the date line and clamps at the poles.
        address_mode_u: wgpu::AddressMode::Repeat,
        address_mode_v: wgpu::AddressMode::ClampToEdge,
        mag_filter: wgpu::FilterMode::Linear,
        min_filter: wgpu::FilterMode::Linear,
        mipmap_filter: wgpu::MipmapFilterMode::Linear,
        ..Default::default()
    });

    let uniform_entry = |binding: u32| wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Uniform,
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    };
    let texture_entry = |binding: u32| wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::FRAGMENT,
        ty: wgpu::BindingType::Texture {
            sample_type: wgpu::TextureSampleType::Float { filterable: true },
            view_dimension: wgpu::TextureViewDimension::D2,
            multisampled: false,
        },
        count: None,
    };
    let sampler_entry = |binding: u32| wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::FRAGMENT,
        ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
        count: None,
    };

    let globe_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("login-globe-bgl"),
        entries: &[uniform_entry(0), texture_entry(1), texture_entry(2), sampler_entry(3)],
    });
    let globe_buf = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("login-globe-uniform"),
        size: std::mem::size_of::<GlobeUniform>() as u64,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let globe_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("login-globe-bg"),
        layout: &globe_bgl,
        entries: &[
            wgpu::BindGroupEntry { binding: 0, resource: globe_buf.as_entire_binding() },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::TextureView(&land),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: wgpu::BindingResource::TextureView(&border),
            },
            wgpu::BindGroupEntry { binding: 3, resource: wgpu::BindingResource::Sampler(&sampler) },
        ],
    });

    let blur_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("login-blur-bgl"),
        entries: &[texture_entry(0), sampler_entry(1), uniform_entry(2)],
    });
    let blur_buf = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("login-blur-uniform"),
        size: std::mem::size_of::<BlurUniform>() as u64,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });

    let pipeline = |label: &str,
                    bgl: &wgpu::BindGroupLayout,
                    shader: &wgpu::ShaderModule,
                    format: wgpu::TextureFormat| {
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some(label),
            bind_group_layouts: &[Some(bgl)],
            immediate_size: 0,
        });
        device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some(label),
            layout: Some(&layout),
            vertex: wgpu::VertexState {
                module: shader,
                entry_point: Some("vs"),
                buffers: &[],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: shader,
                entry_point: Some("fs"),
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: Default::default(),
            depth_stencil: None,
            multisample: Default::default(),
            multiview_mask: None,
            cache: None,
        })
    };

    let resources = GlobeResources {
        globe_pipeline: pipeline("login-globe", &globe_bgl, &globe_shader, TARGET_FORMAT),
        globe_bg,
        globe_buf,
        blur_pipeline: pipeline("login-blur", &blur_bgl, &blur_shader, rs.target_format),
        blur_bgl,
        blur_buf,
        sampler,
        target: None,
    };
    rs.renderer.write().callback_resources.insert(resources);
}

impl GlobeResources {
    /// Make sure the private target is at least `size`, in steps.
    fn ensure_target(&mut self, device: &wgpu::Device, size: (u32, u32)) {
        let want =
            (size.0.div_ceil(ALLOC_STEP) * ALLOC_STEP, size.1.div_ceil(ALLOC_STEP) * ALLOC_STEP);
        if self.target.as_ref().is_some_and(|t| t.alloc.0 >= want.0 && t.alloc.1 >= want.1) {
            return;
        }
        let tex = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("login-globe-target"),
            size: wgpu::Extent3d { width: want.0, height: want.1, depth_or_array_layers: 1 },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: TARGET_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        });
        let view = tex.create_view(&Default::default());
        let blur_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("login-blur-bg"),
            layout: &self.blur_bgl,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&view),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::Sampler(&self.sampler),
                },
                wgpu::BindGroupEntry { binding: 2, resource: self.blur_buf.as_entire_binding() },
            ],
        });
        self.target = Some(Target { alloc: want, view, blur_bg, _tex: tex });
    }
}

// ── The egui callback ────────────────────────────────────────────────────────

/// One frame of the backdrop, handed to egui's painter.
///
/// Carries the finished uniform rather than the [`Sky`] it came from: the
/// callback outlives the frame that made it and is handed between threads by
/// the renderer, so it holds a block of plain data and no state anybody else
/// is still stepping.
pub struct GlobeCallback {
    uniform: GlobeUniform,
    /// Widget size in points; the pixel size follows from the frame's scaling.
    size_pt: [f32; 2],
    /// Faded in over the first second, so the screen does not snap from flat
    /// black to a lit globe the moment the pipelines finish building.
    alpha: f32,
}

/// The backdrop for this frame, at `size_pt` points.
pub fn callback(sky: &Sky, size_pt: [f32; 2], alpha: f32) -> GlobeCallback {
    let aspect = size_pt[0].max(1.0) / size_pt[1].max(1.0);
    GlobeCallback { uniform: sky.uniform(aspect), size_pt, alpha }
}

impl CallbackTrait for GlobeCallback {
    fn prepare(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        screen: &ScreenDescriptor,
        encoder: &mut wgpu::CommandEncoder,
        resources: &mut CallbackResources,
    ) -> Vec<wgpu::CommandBuffer> {
        let ppp = screen.pixels_per_point.max(0.1);
        let px = (
            ((self.size_pt[0] * ppp) as u32 / DOWNSCALE).clamp(16, 2048),
            ((self.size_pt[1] * ppp) as u32 / DOWNSCALE).clamp(16, 2048),
        );
        let Some(r) = resources.get_mut::<GlobeResources>() else { return Vec::new() };
        r.ensure_target(device, px);
        let Some(target) = r.target.as_ref() else { return Vec::new() };
        let alloc = target.alloc;

        queue.write_buffer(&r.globe_buf, 0, bytemuck::bytes_of(&self.uniform));
        queue.write_buffer(
            &r.blur_buf,
            0,
            bytemuck::bytes_of(&BlurUniform {
                uv_scale: [
                    px.0 as f32 / alloc.0 as f32,
                    px.1 as f32 / alloc.1 as f32,
                    1.0 / alloc.0 as f32,
                    1.0 / alloc.1 as f32,
                ],
                params: [1.15, self.alpha, 0.34, 0.0],
            }),
        );

        // The scene, into the private target. Scissored to the part of the
        // allocation actually in use, so the unused margin costs nothing.
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("login-globe-pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &target.view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color::TRANSPARENT),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_viewport(0.0, 0.0, px.0 as f32, px.1 as f32, 0.0, 1.0);
            pass.set_pipeline(&r.globe_pipeline);
            pass.set_bind_group(0, &r.globe_bg, &[]);
            pass.draw(0..3, 0..1);
        }
        Vec::new()
    }

    fn paint(
        &self,
        _info: eframe::egui::PaintCallbackInfo,
        pass: &mut wgpu::RenderPass<'static>,
        resources: &CallbackResources,
    ) {
        let Some(r) = resources.get::<GlobeResources>() else { return };
        let Some(target) = r.target.as_ref() else { return };
        pass.set_pipeline(&r.blur_pipeline);
        pass.set_bind_group(0, &target.blur_bg, &[]);
        pass.draw(0..3, 0..1);
    }
}

// ── Map upload ───────────────────────────────────────────────────────────────

/// Decode a map, reduce it to `max_dim`, and upload it with a mip chain.
///
/// The reduction is the point: the source is 8192×4096, this globe is blurred,
/// and filtering four thousand texels of coastline back off costs both the
/// upload and the memory for nothing.
fn upload_map(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    label: &str,
    png: &[u8],
    max_dim: u32,
) -> wgpu::TextureView {
    // A checked-in asset failing to decode costs the backdrop, not the window.
    let gray = match image::load_from_memory_with_format(png, image::ImageFormat::Png) {
        Ok(img) => img.to_luma8(),
        Err(e) => {
            eprintln!("sdroxide: decoding {label}: {e}");
            image::GrayImage::from_raw(1, 1, vec![0u8]).expect("1×1")
        }
    };
    let (sw, sh) = gray.dimensions();
    let scale = (max_dim as f32 / sw.max(sh) as f32).min(1.0);
    let gray = if scale < 1.0 {
        let (w, h) = (((sw as f32 * scale) as u32).max(1), ((sh as f32 * scale) as u32).max(1));
        image::imageops::resize(&gray, w, h, image::imageops::FilterType::Triangle)
    } else {
        gray
    };

    let (w, h) = gray.dimensions();
    let mut levels: Vec<(u32, u32, Vec<u8>)> = vec![(w, h, gray.into_raw())];
    while levels.last().is_some_and(|(w, h, _)| *w > 1 || *h > 1) {
        levels.push(halve(levels.last().expect("just checked")));
    }

    let tex = device.create_texture(&wgpu::TextureDescriptor {
        label: Some(label),
        size: wgpu::Extent3d { width: w, height: h, depth_or_array_layers: 1 },
        mip_level_count: levels.len() as u32,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::R8Unorm,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    for (level, (lw, lh, data)) in levels.iter().enumerate() {
        // `write_texture` wants rows aligned to 256 bytes, which no level here
        // happens to be.
        let stride = lw.div_ceil(256) * 256;
        let mut padded = vec![0u8; (stride * lh) as usize];
        for row in 0..*lh {
            let src = (row * lw) as usize;
            let dst = (row * stride) as usize;
            padded[dst..dst + *lw as usize].copy_from_slice(&data[src..src + *lw as usize]);
        }
        queue.write_texture(
            wgpu::TexelCopyTextureInfo {
                texture: &tex,
                mip_level: level as u32,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            &padded,
            wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(stride),
                rows_per_image: None,
            },
            wgpu::Extent3d { width: *lw, height: *lh, depth_or_array_layers: 1 },
        );
    }
    tex.create_view(&Default::default())
}

/// One mip level down: a 2×2 box filter, with the odd row/column carried over
/// rather than dropped.
fn halve((w, h, src): &(u32, u32, Vec<u8>)) -> (u32, u32, Vec<u8>) {
    let (nw, nh) = ((w / 2).max(1), (h / 2).max(1));
    let mut out = vec![0u8; (nw * nh) as usize];
    for y in 0..nh {
        for x in 0..nw {
            let (x0, y0) = (x * 2, y * 2);
            let (x1, y1) = ((x0 + 1).min(w - 1), (y0 + 1).min(h - 1));
            let at = |x: u32, y: u32| src[(y * w + x) as usize] as u32;
            let sum = at(x0, y0) + at(x1, y0) + at(x0, y1) + at(x1, y1);
            out[(y * nw + x) as usize] = ((sum + 2) / 4) as u8;
        }
    }
    (nw, nh, out)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The uniform blocks are written from Rust and read as WGSL structs, and
    /// nothing between the two checks that they agree — a field added on one
    /// side silently shifts every field after it on the other.
    ///
    /// Both are multiples of 16 as well, which WebGL2 requires of any uniform
    /// binding and which `tests/shaders.rs` checks from the shader's side.
    #[test]
    fn the_uniform_blocks_are_the_size_the_shaders_expect() {
        // 6 vec4s of camera and framing, then two per arc.
        assert_eq!(std::mem::size_of::<GlobeUniform>(), 16 * (6 + MAX_ARCS * 2));
        assert_eq!(std::mem::size_of::<BlurUniform>(), 32);
        assert_eq!(std::mem::size_of::<GlobeUniform>() % 16, 0);
    }

    /// The shader declares its own array length; a change on one side that is
    /// not made on the other writes arcs nobody draws, or reads uniforms nobody
    /// wrote.
    #[test]
    fn the_arc_count_matches_the_shader() {
        let src = include_str!("shaders/login_globe.wgsl");
        assert!(
            src.contains(&format!("const MAX_ARCS = {MAX_ARCS}u;")),
            "shaders/login_globe.wgsl does not declare MAX_ARCS = {MAX_ARCS}"
        );
        assert!(
            src.contains(&format!("array<vec4<f32>, {}>", MAX_ARCS * 2)),
            "the shader's arc array is not {} entries",
            MAX_ARCS * 2
        );
    }

    /// Both ends of every contact have to be on land, far enough apart to draw,
    /// and unit length — the shader takes all three on trust.
    #[test]
    fn contacts_are_between_real_places() {
        let mut sky = Sky::default();
        for _ in 0..200 {
            let c = sky.spawn();
            for p in [c.a, c.b] {
                let len = dot(p, p).sqrt();
                assert!((len - 1.0).abs() < 1e-4, "endpoint is not a unit vector: {len}");
                let lat = p[2].clamp(-1.0, 1.0).asin().to_degrees();
                let lon = p[1].atan2(p[0]).to_degrees();
                assert!(
                    crate::basemap::is_land(lon as f64, lat as f64),
                    "station at {lon:.1},{lat:.1} is in the sea"
                );
            }
            let sep = dot(c.a, c.b).clamp(-1.0, 1.0).acos().to_degrees();
            assert!((15.0..=170.0).contains(&sep), "contact spans {sep:.1}°");
            assert!(c.life > 0.0);
        }
    }

    /// A contact reaches out, sits worked, and lets go — and is never both
    /// invisible and fully drawn at the moment it starts.
    #[test]
    fn a_contact_draws_in_and_fades_out() {
        let c = Contact { a: ecef(0.0, 0.0), b: ecef(90.0, 0.0), age: 0.0, life: 12.0 };
        let (drawn0, alpha0) = Contact { age: 0.0, ..c }.phase();
        assert!(drawn0 < 0.1 && alpha0 < 0.01, "a contact starts unmade and unlit");
        let (drawn_mid, alpha_mid) = Contact { age: 6.0, ..c }.phase();
        assert!(drawn_mid > 0.99, "by half way the path is complete");
        assert!(alpha_mid > 0.99, "and fully lit");
        let (_, alpha_end) = Contact { age: 11.95, ..c }.phase();
        assert!(alpha_end < 0.05, "and it has let go by the end");
    }

    /// The globe turns, and keeps turning across a frame gap without jumping.
    #[test]
    fn the_spin_advances_smoothly_and_survives_a_stall() {
        let mut sky = Sky::default();
        sky.step(100.0);
        let start = sky.spin;
        sky.step(100.05);
        let after = sky.spin;
        assert!(after > start, "the globe did not turn");
        // A tab that was in the background for a minute must not snap round.
        sky.step(160.0);
        let step = sky.spin - after;
        assert!(step <= 0.1 / SPIN_PERIOD + 1e-6, "a stalled frame jumped the globe by {step}");
    }

    /// The framing has to hold on every window shape the client is opened at:
    /// a phone in landscape, a portrait tablet, a 16:9 desktop and an
    /// ultrawide. Two things must stay true — the disc covers the frame, and
    /// the limb stays somewhere in it.
    #[test]
    fn the_globe_frames_itself_at_every_window_shape() {
        for aspect in [0.42, 0.55, 0.75, 1.0, 1.33, 1.78, 2.37, 3.6] {
            // Projected radius of the disc, in half-heights.
            let r = focal(aspect) / (EYE_DIST * EYE_DIST - 1.0).sqrt();
            let short = aspect.min(1.0);
            assert!(
                r >= short * 0.98,
                "at {aspect}:1 the disc covers {r:.2} of the half-height, leaving a gap down \
                 the short side"
            );
            let corner = (1.0 + aspect * aspect).sqrt();
            assert!(
                r < corner,
                "at {aspect}:1 the disc ({r:.2}) reaches past the corner ({corner:.2}), so no \
                 limb is in shot"
            );
        }
    }

    /// Rotating a direction into the globe's frame and back has to be the
    /// identity — the map and the arcs are placed by this matrix, and a
    /// transposition in it would slide the coastline off the stations.
    #[test]
    fn the_frame_rotation_is_orthonormal() {
        for (spin, tilt) in [(0.0, 0.0), (1.2, 0.4), (-2.7, -0.37), (6.0, 0.41)] {
            let m = local_from_world(spin, tilt);
            let rows = [
                [m[0][0], m[0][1], m[0][2]],
                [m[1][0], m[1][1], m[1][2]],
                [m[2][0], m[2][1], m[2][2]],
            ];
            for (i, a) in rows.iter().enumerate() {
                assert!((dot(*a, *a) - 1.0).abs() < 1e-5, "row {i} is not unit length");
                for (j, b) in rows.iter().enumerate().skip(i + 1) {
                    assert!(dot(*a, *b).abs() < 1e-5, "rows {i} and {j} are not perpendicular");
                }
            }
        }
    }

    /// Screen-up has to be north-ish, or the globe lies on its side.
    #[test]
    fn the_pole_stands_up() {
        let m = local_from_world(0.0, 0.0);
        // World +Y (screen up) must come back as local +Z (the north pole).
        let up = [m[0][1], m[1][1], m[2][1]];
        assert!(up[2] > 0.99, "screen-up maps to {up:?}, which is not the pole");
    }
}
