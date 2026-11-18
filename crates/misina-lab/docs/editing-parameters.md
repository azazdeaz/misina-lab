# Editing parameters

> [!WARNING]
> **Under development.** This page may not reflect the current state of
> the source. We're working on it.

A parameter is one setting of a scene, such as the edge length of a box.
It is declared once, as a field of a **fragment**. A fragment is an
element's own struct of parameters
([generators.md](generators.md#the-params-aggregate)). An
[element](elements.md) is the code that builds one kind of thing in the
scene, such as the boxes. Every place that shows or sets the parameter is
made from that one field. So adding or changing a parameter means editing
the field and running one command.

Some of those places read the fragment's fields each time they run. The
others are text written into files by a command. In the table,
`{PACKAGE}` is the generator's Python package and `{name}` its `NAME` in
lower case ([generators.md](generators.md)). A **region** is a marked block
in a hand-written Python file that the command fills
([The generated files](#the-generated-files)). The **aggregate** is the
struct with one field per fragment, so it holds every parameter of the
scene.

| Where the parameter appears | How it gets there |
| --- | --- |
| the viewer's panel: caption, slider range, tooltip. The viewer is a window on the scene with a control for every parameter: a slider, a dropdown or a checkbox | [`src/ui.rs`](../src/ui.rs) reads the fragment's fields while the viewer runs |
| the snippet: Python code that constructs the Isaac Lab config with the current settings. The viewer's **Copy Isaac Lab cfg** button copies it | [`src/snippet.rs`](../src/snippet.rs) reads the fields when the button is pressed |
| the Python constructor, `BoxParams(size=0.5)` | [`src/python.rs`](../src/python.rs) sets the field named by each keyword, when the constructor is called |
| the type stub `python/{PACKAGE}/_core.pyi`, which editors and type checkers read | generated into the regions `fragments` and `aggregate`, the second inside `class {NAME}Params` |
| the Isaac Lab cfg classes, such as `BoxCfg`, in `python/{PACKAGE}/isaaclab/{name}_cfg.py`. A cfg class builds the scene the first time it is used, caches it as USD and spawns it ([isaac-lab.md](isaac-lab.md)) | generated into the regions `fragments` and `aggregate`, the second inside `class {NAME}Cfg` |
| the re-exports in `python/{PACKAGE}/__init__.py` | generated into the region `exports` |
| the re-exports in `python/{PACKAGE}/isaaclab/__init__.py` | generated into the regions `imports` and `all`, the second inside `__all__` |
| the generator's parameters page, `docs/parameters.md` by convention | generated whole |

The first three rows are compiled Rust that reads the field, so a new field
shows up the next time that code is built: `cargo run` builds the viewer,
and step 4 below rebuilds the Python module. The generated text is written
to files in the generator's crate by a command, and a test checks that it
is up to date.

> [!NOTE]
> **In Bevy terms.** Every fragment derives `Reflect`, so its field names,
> types, doc comments and `#[reflect(@...)]` attributes can be read at run
> time. [`src/params.rs`](../src/params.rs) reads them for all of the above,
> and [`src/codegen.rs`](../src/codegen.rs) writes the generated files.
> misina-lab turns on Bevy's `reflect_documentation` feature, which keeps
> the doc comments readable at run time.

## Change or add a parameter

1. Edit the field on its struct, in the element's file. A field needs a doc
   comment and a default. A number also needs a slider, and a string a
   list of choices ([What a field declares](#what-a-field-declares)):

   ```rust
   /// Edge length of a box, in meters. The default is the crate a market
   /// stall stacks.
   ///
   /// Anything after the first paragraph is for a reader of the Rust.
   #[reflect(@Slider { min: 0.1, max: 2.0, step: 0.1 })]
   pub size: f32,
   ```

   Then set the field's default value in the struct's `impl Default`.

2. Regenerate the Python files and the parameters page:

   ```bash
   cargo test regen_params -- --ignored
   ```

   `regen_params` is a test in the generator's own crate
   ([generators.md](generators.md#what-a-generators-tests-run)). It is
   marked `#[ignore]`, so a plain `cargo test` skips it, and `--ignored`
   runs it. It rewrites the generated files instead of checking them. In a
   Cargo workspace, name the crate:
   `cargo test -p <generator-crate> regen_params -- --ignored`.

3. Run `cargo test`, with `-p <generator-crate>` in a Cargo workspace. Two
   of the generator's tests check the result. One calls `codegen::stale`
   and fails while a generated file is out of date. The other calls
   `testing::check_params` and names the first field with no doc comment,
   with no slider or choice list, or with a default outside its slider
   range or choice list.

4. To use the change from Python, rebuild the extension module, the
   compiled Rust that Python imports. Neither the regeneration nor
   `cargo test` does that. In a project laid out as
   [outside-projects.md](outside-projects.md#pyprojecttoml) describes, the
   command is `uv sync --reinstall-package <package>`. Until then, a new
   field is in the generated Python files and not in the installed module.

The declaration needs nothing else. A new field has no effect until the
element's code reads it. If the element keeps the field in its configs,
the per-instance values its meshes are built from, its `reauthor` system
has to copy it too. An element that creates its own configs has no
`reauthor`: its `author` system sets the field
([elements.md](elements.md#rules)). Do not edit the generated regions by
hand: the next regeneration overwrites them.

## What a field declares

**The first paragraph of the doc comment is shown to users.** It is the
tooltip in the panel, the docstring in the generated Python files and the
field's row on the parameters page. So write it to make sense on its own.
Do not use rustdoc link syntax (`[`like this`]`) in it, because Python
would show the brackets as typed; `check_params` fails on it. Backticks are
fine: the tooltip drops them. Later paragraphs are for the reader of the
Rust. The panel, the generated Python files and the parameters page leave
them out. The compiled module's own docstrings, which Python's `help()`
shows, carry the whole doc comment.

A struct's doc comment is shown to users in full. It becomes the class
docstring in Python and the text under the fragment's heading on the
parameters page. So it says what the element is and how its fields
relate. `check_params` fails if any of its paragraphs uses rustdoc link
syntax.

**A number field has a `@Slider` attribute.** It sets the range the panel
offers and the step the slider moves by. The number of decimals shown
follows from the step. The range limits only the slider. Python accepts
any value, so the element's code must clamp a value it cannot build with.

**A string field has a `@Choices` attribute.** It points at the fixed list
of allowed names, an array of strings the element defines:
`@Choices(&Kind::NAMES)`. The panel offers the names in a dropdown. In the
stub, the field's type lists the names, so a type checker flags any other.
Python raises `ValueError` for a name that is not on the list when the
scene is built, not when the field is set.

**A `bool` has no attribute** and gets a checkbox.

**`@Label("...")` sets the caption.** Without it the caption is the field
name with underscores as spaces and a capital first letter:
`feature_size` shows as "Feature size". Use a label for units,
`@Label("Feature size (m)")`, or for other wording on screen. A label can
also go on a field of the aggregate. There it is the title of that
fragment's section in the panel and its heading on the parameters page:
`@Label("The boxes")`.

A field is an `f32`, `u32`, `u64`, `bool` or `String`; the code that
reads the structs panics on any other type. The order of declaration is
the order in the panel, the stub, the cfg classes and the parameters page.
By convention, the fields that change the shape come first. The fields
that trade quality against memory and build time come last: `variations`,
how many different meshes an element may build
([mesh-variations.md](mesh-variations.md)), and `detail`, how finely each
mesh is built.

## Add a fragment

A new element's struct of parameters is a new fragment.
[elements.md](elements.md) lists the steps for the element itself: its
module, its plugin and its stage. For the fragment:

1. Name the struct `<Stem>Params`. The stem names its classes: `CoverParams`
   is `CoverParams` in Python and `CoverCfg` in Isaac Lab. Derive
   `Resource, Reflect, Clone, Debug, PartialEq` and give it a `Default`,
   derived or written by hand. Add the line the other fragments have, which
   makes the struct a Python class when the `python` feature is on:

   ```rust
   #[cfg_attr(feature = "python", pyo3::pyclass(get_all, set_all, skip_from_py_object))]
   ```

   Give the struct a doc comment, and declare its fields as above.

2. Add one line to the generator's `misina_lab::generator!` call:
   `pub cover: cover::CoverParams,`. The macro reads that list and writes
   `apply`, which copies the params into the scene, `read`, which copies
   them back out, and the Python classes
   ([generators.md](generators.md#the-params-aggregate)). The generator's
   round-trip test takes `testing::nudged`, params with every field moved
   off its default, and checks that `apply` and `read` return them
   unchanged.

   A fragment that misina-lab, the framework, declares already has its
   Python constructor. One is `terrain::TerrainParams`. Write `as core`
   after its type, so the macro does not write a second one:
   `pub terrain: misina_lab::terrain::TerrainParams as core,`.

3. Regenerate, as above. The new fragment then appears in the stub, in the
   cfg classes, in the re-exports and in `FRAGMENTS`. `FRAGMENTS` is a
   tuple on the generator's cfg class that lists the names of its fragment
   fields. The cfg class turns those fields into params and builds its
   cache key from them
   ([isaac-lab.md](isaac-lab.md#the-contract)).

> [!NOTE]
> **In Bevy terms.** A `Resource` is a value the Bevy app stores once. In
> the running scene each fragment is its own resource, so an edit rebuilds
> only the elements that read it and the elements built on top of those
> ([generators.md](generators.md#the-params-aggregate)).

## The generated files

The Python files in the table are written by hand, except for their
regions, which the command fills. A region starts at a line
`# >>> generated: <region>` and ends at `# <<< generated: <region>`.
Everything outside the markers is left as it is. The table's paths are
relative to the generator's crate. If a file lacks the markers of a
region, `codegen::stale` panics and names the file and the region.

The tooltips are read from the doc comments while the viewer runs. The
docstrings in the generated Python files and the parameters page are
copied from the doc comments only when `regen_params` runs, so they keep
the old text until it does. The compiled module also takes its own
docstrings from the doc comments, each time it is built. If a description
reads wrong in the viewer, in an editor's hover or on the page, fix it on
the field and regenerate.
