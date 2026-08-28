//! The sdroxide GUI: an egui app that talks to any [`sdroxide_types::RadioController`].
//!
//! Compiles native and for wasm32. All custom wgpu rendering that the browser
//! build shares is written to WebGL2 downlevel limits (fragment-only, sampled
//! textures + uniforms). The one exception is [`solar3d`], which uses depth,
//! MSAA and vertex buffers — but does so entirely inside an offscreen pass of
//! its own, so the shared egui pass never sees them and the module runs on both
//! targets.

mod adsb_map;
mod app;
mod aprs_icons;
mod aprs_map;
/// The world the flat maps are drawn from — the globe's own land raster,
/// sampled on the CPU, plus the border and river geometry and the city table.
mod basemap;
pub mod chrome;
mod colormap;
mod digi_map;
mod download;
/// Running eframe on our own winit loop, so a Wayland session does not spin.
#[cfg(not(target_arch = "wasm32"))]
pub mod event_loop;
mod flags;
mod fuzzy;
mod hell;
mod help;
mod input;
/// Which layout the window wears — desktop strip, tablet menus, or the compact
/// phone strip — and the metrics that follow from it.
pub mod layout;
mod login;
mod login_globe;
/// Multi-radio shell: one window, one radio per tab. Native-only — the
/// browser client drives a single (remote) radio.
mod multi;
mod prop_map;
#[cfg(feature = "remote")]
mod remote;
/// Asking egui for the next frame at the rate actually asked for.
pub mod repaint;
mod rf_paint;
/// Solar-system 3D view. A second OS window natively, a second browser tab on
/// the web; the sole consumer of depth/MSAA/vertex buffers either way.
mod solar3d;
mod sstv;
pub mod theme;
mod time;
mod view;
pub mod waterfall_gpu;
mod wefax;
mod widgets;

pub use app::SdroxideApp;
pub use multi::{MultiApp, RadioFactory, RadioTab, RemoteFactory};
#[cfg(feature = "remote")]
pub use remote::{AudioBridge, RemoteController};
/// The solar-system view as a standalone app, for the browser tab the ☀ 3D
/// chip opens. Natively the same view is a child viewport of the main window
/// instead — see `solar3d::Solar3d`.
#[cfg(feature = "remote")]
pub use solar3d::SolarApp;

/// Wgpu access must go through this re-export so every crate agrees on the
/// wgpu version (project rule).
pub use eframe::egui_wgpu;

/// The wgpu setup every sdroxide window opens with.
///
/// eframe's default asks the device for `wgpu::Limits::default()` — the WebGPU
/// baseline — on every backend but GL. Real GPUs are allowed to sit *below*
/// that baseline, and the request then fails outright rather than degrading,
/// taking the window with it: a Raspberry Pi 5 (V3D) grants 15 inter-stage
/// shader variables where the baseline asks for 16, and eframe exits with
///
/// > Limit 'max_inter_stage_shader_variables' value 16 is better than allowed 15
///
/// Nothing here needs the baseline. The busiest shader in the tree passes three
/// varyings, and every texture is already built against `device.limits()` —
/// `solar3d` and `login_globe` pick the mip level that fits, and the
/// waterfall's history is sized against `max_texture_dimension_2d` too
/// ([`waterfall_gpu::auto_display_bins`]). So ask for exactly what the adapter
/// reports, a request no adapter can refuse, and let the drawing code adapt to
/// what it gets, which is what it already does.
///
/// This also *lifts* limits on the GL backend, where eframe asks for the WebGL2
/// downlevel defaults: a native GL context that can do better now says so, and
/// the globe gets its full-resolution maps instead of the 2048-pixel cap.
///
/// The same Pi is also the one machine where the backend itself is not left to
/// eframe — see [`v3dv_backends`].
pub fn wgpu_options() -> egui_wgpu::WgpuConfiguration {
    use egui_wgpu::wgpu;
    let mut setup = egui_wgpu::WgpuSetupCreateNew::without_display_handle();
    setup.device_descriptor =
        std::sync::Arc::new(|adapter: &wgpu::Adapter| wgpu::DeviceDescriptor {
            label: Some("sdroxide"),
            required_limits: adapter.limits(),
            ..Default::default()
        });
    // A slow OpenGL driver must not be able to kill the window — see
    // `gl_fence_behavior`. Inert on every other backend.
    setup.instance_descriptor.backend_options.gl.fence_behavior = gl_fence_behavior();
    #[cfg(not(target_arch = "wasm32"))]
    if let Some(backends) = v3dv_backends(setup.instance_descriptor.backends) {
        setup.instance_descriptor.backends = backends;
    }
    egui_wgpu::WgpuConfiguration {
        wgpu_setup: egui_wgpu::WgpuSetup::CreateNew(setup),
        ..Default::default()
    }
}

/// How much a fence is worth on the OpenGL backend.
///
/// wgpu waits for the GPU to go idle before it reconfigures a surface — which
/// happens at startup, when the window's real size arrives, and on every resize
/// after that — and a wait that does not finish is a fatal validation error:
///
/// > wgpu error: Validation Error
/// > In `Surface::configure`
/// > Failed to wait for GPU to come idle before reconfiguring the Surface
///
/// The wait wgpu asks for there is indefinite, and on Vulkan, Metal and D3D12
/// it really is. On OpenGL it is `glClientWaitSync`, and wgpu-hal passes that
/// call a timeout clamped to `i32::MAX` — **nanoseconds**, so "wait forever"
/// quietly becomes "wait 2.147 seconds". A GPU that needs longer to finish the
/// frame already in flight than that therefore does not merely stutter: the
/// window dies, and takes the process with it (issue #148, an Intel HD
/// Graphics 4000 on its 2013 OpenGL 4.0 driver, where the *first* frame — every
/// shader compiled, every texture allocated — did not land inside the 2.147 s).
/// Tearing down after that then walks into the queue's own shutdown wait, which
/// the same card misses as well, and *that* one panics inside a destructor —
/// so the process aborts instead of unwinding.
///
/// `AutoFinish` answers such a wait from the value the last submit recorded
/// rather than asking the driver, so a slow frame costs a slow frame and no
/// more. That is sound here in a way it would not be on the other backends:
/// OpenGL defers deleting anything the pipeline still refers to, so nothing is
/// freed under the GPU's feet, and the reason to wait in the first place — a
/// swapchain the application destroys itself — does not exist on OpenGL. It is
/// also what wgpu did on every OpenGL context until gfx-rs/wgpu#4589.
///
/// What the setting really costs is a truthful `Queue::on_completed_work_done`
/// and a truthful `poll(Wait)`, which matter to a reader mapping a buffer the
/// GPU has just written. Nothing in this tree reads back from the GPU — every
/// pass ends on the screen — so there is nothing to be early for.
///
/// `WGPU_GL_FENCE_BEHAVIOR=normal` puts the driver back in charge. (eframe
/// builds its backend options with `from_env_or_default`, which does not read
/// that variable, so honouring it is this function's job.)
fn gl_fence_behavior() -> egui_wgpu::wgpu::GlFenceBehavior {
    egui_wgpu::wgpu::GlFenceBehavior::AutoFinish.with_env()
}

#[cfg(test)]
mod gl_fence_tests {
    use super::{gl_fence_behavior, wgpu_options};
    use crate::egui_wgpu::{WgpuSetup, wgpu};

    /// The window must not be at the mercy of a 2.147-second `glClientWaitSync`
    /// on a slow OpenGL driver, so the configuration every window opens with
    /// carries `AutoFinish` into the GL backend.
    #[test]
    fn the_gl_backend_does_not_wait_on_the_driver() {
        assert_eq!(gl_fence_behavior(), wgpu::GlFenceBehavior::AutoFinish);
        let WgpuSetup::CreateNew(setup) = wgpu_options().wgpu_setup else {
            panic!("wgpu_options should be building a new setup");
        };
        assert!(setup.instance_descriptor.backend_options.gl.fence_behavior.is_auto_finish());
    }
}

/// What an adapter says about itself, cut down to the two strings and the
/// backend that [`prefer_gl_over_v3dv`] decides on.
///
/// wgpu's own `AdapterInfo` would do, but it carries a dozen fields and gains
/// more with each release, which would make the test below a chore to keep
/// compiling for no gain.
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug)]
struct AdapterSummary {
    backend: egui_wgpu::wgpu::Backend,
    /// e.g. `"V3D 7.1.10.2"`.
    name: String,
    /// e.g. `"V3DV Mesa"`.
    driver: String,
}

/// Whether the Raspberry Pi's GPU is here behind Mesa's Vulkan driver.
///
/// V3D is the 3D block in the Broadcom SoCs on the Pi 4 and Pi 5 (and so the
/// Pi 400/500). Mesa drives it two ways: **V3DV** for Vulkan and a GLES 3.1
/// driver for OpenGL, and both enumerate as adapters named `V3D <version>`,
/// which is why the backend has to be part of the test.
///
/// Rendering through V3DV flickers — on a Pi 500 under labwc badly enough to be
/// unusable — and it is not sdroxide-specific: the same driver flickers under
/// other wgpu and Vulkan applications (gfx-rs/wgpu#1467, warpdotdev/warp#4879).
/// The GLES path on the same GPU is steady, so that is what this picks, at a
/// cost of roughly one core of four (measured 237% CPU against 140% on an
/// RTL-SDR at 2.4 Msps).
#[cfg(not(target_arch = "wasm32"))]
fn prefer_gl_over_v3dv(adapters: &[AdapterSummary]) -> bool {
    adapters.iter().any(|a| {
        a.backend == egui_wgpu::wgpu::Backend::Vulkan
            && (a.driver.contains("V3DV") || a.name.to_ascii_uppercase().starts_with("V3D "))
    })
}

/// The backends to open the window with, or `None` to leave eframe's choice
/// alone — which is every machine but a Raspberry Pi.
///
/// Asking wgpu costs an instance and an enumeration, so this only asks where
/// the answer can be yes: V3D ships in Raspberry Pi silicon and nothing else,
/// so nothing but Linux on ARM pays for the probe. Enumerating Vulkan alone
/// keeps it to the library eframe is about to load anyway, and needs no display
/// handle (which does not exist yet at this point in startup, and which GLES
/// would want).
///
/// `WGPU_BACKEND` stays the override in both directions — it is read before
/// anything else here, so `WGPU_BACKEND=vulkan` puts the core back on a Pi for
/// anyone who would rather have the flicker.
#[cfg(not(target_arch = "wasm32"))]
fn v3dv_backends(configured: egui_wgpu::wgpu::Backends) -> Option<egui_wgpu::wgpu::Backends> {
    use egui_wgpu::wgpu;

    if !cfg!(all(target_os = "linux", any(target_arch = "aarch64", target_arch = "arm"))) {
        return None;
    }
    // Whatever the environment names, it means it.
    if wgpu::Backends::from_env().is_some() {
        return None;
    }
    // Neither half of the swap is on the table otherwise.
    if !configured.contains(wgpu::Backends::VULKAN) || !configured.contains(wgpu::Backends::GL) {
        return None;
    }

    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
        backends: wgpu::Backends::VULKAN,
        ..wgpu::InstanceDescriptor::new_without_display_handle()
    });
    // `enumerate_adapters` is a future only for the browser's sake; natively it
    // is already resolved by the time it is returned.
    let adapters: Vec<AdapterSummary> =
        pollster::block_on(instance.enumerate_adapters(wgpu::Backends::VULKAN))
            .iter()
            .map(|a| {
                let info = a.get_info();
                AdapterSummary { backend: info.backend, name: info.name, driver: info.driver }
            })
            .collect();

    if !prefer_gl_over_v3dv(&adapters) {
        return None;
    }
    eprintln!(
        "sdroxide: the Raspberry Pi's V3D through Mesa's Vulkan driver (V3DV) flickers, \
         so this window renders through OpenGL ES instead. \
         Set WGPU_BACKEND=vulkan to use Vulkan anyway — about one core cheaper, and it \
         may not flicker on your compositor."
    );
    Some(wgpu::Backends::GL)
}

#[cfg(all(test, not(target_arch = "wasm32")))]
mod v3dv_tests {
    use super::{AdapterSummary, prefer_gl_over_v3dv};
    use crate::egui_wgpu::wgpu::Backend;

    fn adapter(backend: Backend, name: &str, driver: &str) -> AdapterSummary {
        AdapterSummary { backend, name: name.to_owned(), driver: driver.to_owned() }
    }

    /// The three adapters a Pi 500 running Raspberry Pi OS bookworm reports,
    /// verbatim from the bug report (Mesa 24.2.8, kernel 6.12.96+rpt-rpi-2712).
    #[test]
    fn a_pi_500_asks_for_gl() {
        let pi = [
            adapter(Backend::Vulkan, "V3D 7.1.10.2", "V3DV Mesa"),
            adapter(Backend::Vulkan, "llvmpipe (LLVM 15.0.6, 128 bits)", "llvmpipe"),
            adapter(Backend::Gl, "V3D 7.1.10.2", ""),
        ];
        assert!(prefer_gl_over_v3dv(&pi));
    }

    /// The GLES driver names its adapter `V3D` too, so seeing that name is not
    /// on its own a reason to move: the swap has to be off Vulkan specifically,
    /// or a machine already on GL would be told to switch to where it is.
    #[test]
    fn the_gl_adapter_alone_is_not_a_reason() {
        let already_gl = [adapter(Backend::Gl, "V3D 7.1.10.2", "")];
        assert!(!prefer_gl_over_v3dv(&already_gl));
    }

    /// Every other machine keeps the renderer it had.
    #[test]
    fn other_gpus_are_left_alone() {
        let desktop = [
            adapter(Backend::Vulkan, "AMD Radeon RX 7900 XTX (RADV NAVI31)", "radv"),
            adapter(Backend::Vulkan, "NVIDIA GeForce RTX 4090", "NVIDIA"),
            adapter(Backend::Vulkan, "Intel(R) Arc(tm) A770", "Intel open-source Mesa driver"),
            adapter(Backend::Vulkan, "llvmpipe (LLVM 15.0.6, 128 bits)", "llvmpipe"),
            // An ARM board that is not a Pi.
            adapter(Backend::Vulkan, "NVIDIA Tegra Orin (nvgpu)", "NVIDIA"),
            adapter(Backend::Vulkan, "Mali-G610", "Mali-G610"),
        ];
        assert!(!prefer_gl_over_v3dv(&desktop));
    }

    /// A future Mesa that renames the driver string still gets caught by the
    /// adapter name, and one that renames the adapter by the driver string.
    #[test]
    fn either_half_of_the_name_is_enough() {
        assert!(prefer_gl_over_v3dv(&[adapter(Backend::Vulkan, "Broadcom V3D", "V3DV Mesa")]));
        assert!(prefer_gl_over_v3dv(&[adapter(Backend::Vulkan, "v3d 9.0.0.0", "Mesa")]));
    }
}

/// The application icon, for [`eframe::egui::ViewportBuilder::with_icon`].
///
/// This is what window managers show in the taskbar/dock and in alt-tab; the
/// desktop-menu entry gets its icon from the installed hicolor theme instead
/// (see `packaging/`). Both come from `packaging/icons/sdroxide.svg`.
#[cfg(not(target_arch = "wasm32"))]
pub fn app_icon() -> eframe::egui::IconData {
    const PNG: &[u8] = include_bytes!("../../../packaging/icons/sdroxide-256.png");
    // Decoding a 256x256 PNG once at startup; a failure here would only cost
    // the icon, so fall back to no icon rather than refusing to open a window.
    match image::load_from_memory_with_format(PNG, image::ImageFormat::Png) {
        Ok(img) => {
            let rgba = img.to_rgba8();
            let (width, height) = rgba.dimensions();
            eframe::egui::IconData { rgba: rgba.into_raw(), width, height }
        }
        Err(e) => {
            eprintln!("sdroxide: decoding the app icon: {e}");
            eframe::egui::IconData::default()
        }
    }
}
