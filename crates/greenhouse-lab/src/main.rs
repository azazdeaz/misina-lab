use bevy::prelude::*;
use greenhouse_lab::Greenhouse;
use misina_lab::viewer::Framing;

fn main() {
    let mut app = misina_lab::viewer::app::<Greenhouse>();
    // The default rows, seen from above the main path at the row head.
    app.insert_resource(Framing {
        eye: Vec3::new(-7.0, -8.0, 6.0),
        focus: Vec3::new(6.0, 0.0, 1.0),
    });
    app.run();
}
