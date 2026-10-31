//! Species — what every crop declares, and the registry the planting layer
//! reads them through.
//!
//! A species is one file under this module: a [`species!`] call declaring
//! its params struct, which is its schema and carries the knobs every
//! species has ([`Common`], and the mesh budget) beside its own, and the
//! marker type the layers are keyed on ([`Species`]); and a hand-written
//! [`Recipe`], how one plant of it grows, in the vocabulary of
//! [`prelude`]. [`species`] wires one into the app: its params resource,
//! [`publish`], which keeps its common knobs in [`Commons`] for the planting
//! layer, and the organ layers instantiated for it.
//!
//! Every plant of a species carries [`Of<S>`] and every leaf and fruit it
//! grows [`OrganOf<S>`], so a layer finds its species' things by type, and a
//! removal is attributed to a species, which `RemovedComponents` of a shared
//! config type could not do. The two markers are distinct so that a species'
//! plant layer, which replaces its organs, never wakes itself.

use std::collections::BTreeMap;
use std::marker::PhantomData;

use bevy::ecs::system::EntityCommands;
use bevy::prelude::*;

use crate::elements::planting::PlantConfig;
use crate::elements::{Grow, PhysicsParams, fruit, leaf, plant};
use crate::grow::builder::Builder;

/// Declares a species: its params struct with the common knobs appended,
/// their defaults, and the marker type with its [`Species`] impl.
///
/// ```ignore
/// species! {
///     /// What the crop is.
///     pub struct KaleParams for Kale as "kale" {
///         /// Height of a harvest-ready stem, in meters.
///         #[reflect(@Slider { min: 0.3, max: 1.0, step: 0.05 })]
///         pub height: f32 = 0.6,
///     }
///     common { share: 0.25, spacing: 0.3, sown_week: 0.0, bed: 0, variations: 12 }
/// }
/// ```
///
/// The attribute paths resolve where the macro is called, so a species file
/// starts with `use super::prelude::*`. The `Recipe` impl is written by hand
/// beside the call.
macro_rules! species {
    (
        $(#[$meta:meta])*
        pub struct $params:ident for $species:ident as $name:literal {
            $( $(#[$field_meta:meta])* pub $field:ident : $ty:ty = $default:expr ),* $(,)?
        }
        common {
            share: $share:expr,
            spacing: $spacing:expr,
            sown_week: $sown_week:expr,
            bed: $bed:expr,
            variations: $variations:expr $(,)?
        }
    ) => {
        $(#[$meta])*
        #[derive(::bevy::prelude::Resource, ::bevy::prelude::Reflect, Clone, Debug, PartialEq)]
        #[cfg_attr(
            feature = "python",
            ::pyo3::pyclass(get_all, set_all, skip_from_py_object)
        )]
        pub struct $params {
            $( $(#[$field_meta])* pub $field: $ty, )*
            /// Share of the garden's planting squares this crop gets, against
            /// the other crops'; 0 leaves it out.
            #[reflect(@Slider { min: 0.0, max: 1.0, step: 0.05 })]
            pub share: f32,
            /// Distance between plants, in meters, rounded to one, two, three
            /// or four per side of a 30 cm square.
            #[reflect(@Slider { min: 0.03, max: 1.0, step: 0.01 })]
            pub spacing: f32,
            /// The week it was sown, on the scene's calendar: a plant's age is
            /// the scene's week less this, and a bed is bare before it.
            #[reflect(@Slider { min: 0.0, max: 20.0, step: 1.0 })]
            pub sown_week: f32,
            /// The one bed it is confined to, counted from 1; 0 plants it
            /// anywhere.
            #[reflect(@Slider { min: 0.0, max: 24.0, step: 1.0 })]
            pub bed: u32,
            /// How many distinct meshes each of its organs, its leaves and its
            /// fruit, may share. A budget, not a count: the organs are
            /// clustered and this is how many representatives the clustering
            /// may keep.
            #[reflect(@Slider { min: 1.0, max: 64.0, step: 1.0 })]
            pub variations: u32,
        }

        impl Default for $params {
            fn default() -> Self {
                Self {
                    $( $field: $default, )*
                    share: $share,
                    spacing: $spacing,
                    sown_week: $sown_week,
                    bed: $bed,
                    variations: $variations,
                }
            }
        }

        pub struct $species;

        impl $crate::species::Species for $species {
            type Params = $params;
            const NAME: &'static str = $name;
            const PREFIX: &'static str = stringify!($species);

            fn common(p: &$params) -> $crate::species::Common {
                $crate::species::Common {
                    share: p.share,
                    spacing: p.spacing,
                    sown_week: p.sown_week,
                    bed: p.bed,
                }
            }

            fn variations(p: &$params) -> u32 {
                p.variations
            }
        }
    };
}

pub mod bush_bean;
pub mod carrot;
pub mod kale;
pub mod lettuce;
pub mod pole_bean;

/// What a species file writes its recipe in.
pub mod prelude {
    pub use bevy::prelude::*;
    pub use misina_lab::palette::{mix, shade, srgb};
    pub use misina_lab::params::{Choices, Label, Slider};

    pub use super::{Common, Recipe, Species};
    pub use crate::elements::fruit::FruitConfig;
    pub use crate::elements::leaf::LeafConfig;
    pub use crate::elements::planting::{PlantConfig, STRING_RADIUS};
    pub use crate::grow::axis::{Axis, Grown};
    pub use crate::grow::builder::{Builder, lerp, smooth};
    pub use crate::grow::phyllotaxis::{AxisNode, Rosette, Spiral};
}

/// The knobs every species carries, as the planting layer reads them.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Common {
    /// Share of the garden's planting squares; 0 leaves the species out.
    pub share: f32,
    /// Distance between plants, in meters.
    pub spacing: f32,
    /// The week it was sown: a plant's age is the scene's week less this.
    pub sown_week: f32,
    /// The one bed it is confined to, from 1; 0 for anywhere.
    pub bed: u32,
}

/// Declared by [`species!`]: a species' schema and its names.
pub trait Species: 'static + Send + Sync + Sized {
    type Params: Resource + Reflect + Clone + PartialEq + Default;
    /// What a plant's `garden_lab:species` attribute says, lower case.
    const NAME: &'static str;
    /// The stem of its part names: `LettuceLeaf_3`.
    const PREFIX: &'static str;
    /// The knobs the planting layer reads, off the params.
    fn common(params: &Self::Params) -> Common;
    /// How many distinct meshes each of its organ layers may hold.
    fn variations(params: &Self::Params) -> u32;
}

/// Written by hand: how one plant grows.
pub trait Recipe: Species {
    /// Sowing to harvest-ready, in weeks; [`PlantConfig::grown`] eases a
    /// plant's age over it.
    const HARVEST: f32;
    /// Whether it climbs: dealt the north squares of its bed, under a
    /// trellis, each plant given a string to the rail, which its recipe
    /// reads as [`PlantConfig::string`].
    const CLIMBS: bool = false;
    /// One plant, in its own frame, its foot at the origin on the soil: what
    /// it grows, drawn from the builder's stream.
    fn grow(params: &Self::Params, plant: &PlantConfig, b: &mut Builder);
}

/// Marks a plant of species `S`.
#[derive(Component)]
pub struct Of<S: Species>(PhantomData<S>);

impl<S: Species> Default for Of<S> {
    fn default() -> Self {
        Self(PhantomData)
    }
}

/// Marks a leaf or fruit a plant of species `S` grew.
#[derive(Component)]
pub struct OrganOf<S: Species>(PhantomData<S>);

impl<S: Species> Default for OrganOf<S> {
    fn default() -> Self {
        Self(PhantomData)
    }
}

/// One registered species: its common knobs, whether it climbs, and how to
/// mark a plant of it.
#[derive(Clone, Copy, Debug)]
pub struct Entry {
    pub common: Common,
    /// [`Recipe::CLIMBS`].
    pub climbs: bool,
    /// Inserts the species' [`Of`] marker on a plant being spawned.
    pub tag: fn(&mut EntityCommands),
}

impl PartialEq for Entry {
    /// The tag is a function of the species alone, so the knobs and the
    /// climbing are the entry.
    fn eq(&self, other: &Self) -> bool {
        self.common == other.common && self.climbs == other.climbs
    }
}

/// Every species wired into the app, by name, with its common knobs as
/// they stand: what the planting layer reads instead of naming N types.
/// Written only when an entry changes, so a system gated on it runs when a
/// species' common knobs move and not when its own do.
#[derive(Resource, Default, Debug, Clone, PartialEq)]
pub struct Commons(pub BTreeMap<&'static str, Entry>);

/// Keeps `S`'s entry in [`Commons`] current.
fn publish<S: Recipe>(params: Res<S::Params>, mut commons: ResMut<Commons>) {
    let entry = Entry {
        common: S::common(&params),
        climbs: S::CLIMBS,
        tag: |plant| {
            plant.insert(Of::<S>::default());
        },
    };
    if commons.0.get(S::NAME) != Some(&entry) {
        commons.0.insert(S::NAME, entry);
    }
}

/// Wires species `S` into the app: its params, its entry in [`Commons`],
/// and its plant, leaf and fruit layers.
pub fn species<S: Recipe>(app: &mut App) {
    app.init_resource::<S::Params>().add_systems(
        PreUpdate,
        (
            publish::<S>
                .run_if(resource_changed::<S::Params>)
                .in_set(Grow::Species),
            // Every edit of the species' params regrows its plants, whose
            // fresh organs wake the leaf and fruit layers below.
            plant::grow::<S>
                .run_if(
                    plant::changed::<S>
                        .or_eager(resource_changed::<S::Params>)
                        .or_eager(resource_changed::<PhysicsParams>),
                )
                .in_set(Grow::Plants),
            leaf::build::<S>
                .run_if(leaf::changed::<S>)
                .in_set(Grow::Leaves),
            fruit::build::<S>
                .run_if(fruit::changed::<S>)
                .in_set(Grow::Fruit),
        ),
    );
}
