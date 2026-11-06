use bevy::prelude::*;
use garden_lab::Garden;
use misina_lab::viewer::Framing;

fn main() {
    let mut app = misina_lab::viewer::app::<Garden>();
    // The default garden, seen from its south-west corner.
    app.insert_resource(Framing {
        eye: Vec3::new(-7.0, -9.0, 5.5),
        focus: Vec3::new(0.5, 0.5, 0.3),
    });
    app.run();
}
