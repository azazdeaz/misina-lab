use bevy::prelude::*;
use greenhouse_lab::Greenhouse;
use greenhouse_lab::elements::GreenhouseParams;
use misina_lab::viewer::{Framing, start_from};

fn main() {
    let mut app = misina_lab::viewer::app::<Greenhouse>();
    let framing = if cfg!(target_arch = "wasm32") {
        // The web build generates on one thread, so the playground starts on
        // the smallest greenhouse, one row as short as the panel allows, seen
        // from the main path.
        let mut params = GreenhouseParams::default();
        (params.layout.rows, params.layout.row_length) = (1, 5.0);
        start_from::<Greenhouse>(&mut app, params);
        Framing {
            eye: Vec3::new(-6.0, -7.5, 4.0),
            focus: Vec3::new(2.5, 0.0, 2.0),
        }
    } else {
        // The default rows, seen from above the main path at the row head.
        Framing {
            eye: Vec3::new(-7.0, -8.0, 6.0),
            focus: Vec3::new(6.0, 0.0, 1.0),
        }
    };
    app.insert_resource(framing);
    app.run();
}
