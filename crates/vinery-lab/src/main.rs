use bevy::prelude::*;
use misina_lab::Generator;
use misina_lab::viewer::{Framing, start_from};
use vinery_lab::Vineyard;
use vinery_lab::elements::VineyardParams;

fn main() {
    let mut app = misina_lab::viewer::app::<Vineyard>();
    if cfg!(target_arch = "wasm32") {
        // The web build generates on one thread, so the playground starts on
        // a parcel just big enough for three rows, and so an alley either
        // side of the middle one, at the default headland and shortest row.
        let mut params = VineyardParams::default();
        (params.terrain.length, params.terrain.width) = (24.0, 18.0);
        start_from::<Vineyard>(&mut app, params);
        app.insert_resource(Framing {
            eye: Vec3::new(11.0, -11.0, 7.0),
            focus: Vec3::ZERO,
        });
    }
    // Viewer-only: gizmos need `GizmoPlugin`, which the headless path's
    // `MinimalPlugins` does not provide.
    app.add_plugins(vinery_lab::elements::util::parcel::debug_plugin);
    // The per-layer half of `misina_lab::perf`, under the same variable.
    if Vineyard::env("PERF").is_some() {
        app.add_plugins(vinery_lab::perf::plugin);
    }
    app.run();
}
