use misina_lab::Generator;
use vinery_lab::Vineyard;

fn main() {
    let mut app = misina_lab::viewer::app::<Vineyard>();
    // Viewer-only: gizmos need `GizmoPlugin`, which the headless path's
    // `MinimalPlugins` does not provide.
    app.add_plugins(vinery_lab::elements::util::parcel::debug_plugin);
    // The per-layer half of `misina_lab::perf`, under the same variable.
    if Vineyard::env("PERF").is_some() {
        app.add_plugins(vinery_lab::perf::plugin);
    }
    app.run();
}
