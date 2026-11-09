---
name: readme-video
description: Re-records the READMEs' demo videos of the viewers — the vineyard's in the top README, the greenhouse's in its crate's — with docs/video/record.sh, publishes them with docs/video/publish.sh, and changes what they show — the camera moves, the panel edits, their timing — by editing the storyboards docs/video/vinery-lab.json and docs/video/greenhouse-lab.json. Use after a change to how a viewer, the panel or a scene looks, or when asked to update, re-record or re-cut a README video of the viewer. Not for the docs' still pictures, which are docs-shots, nor for the demos filmed in Isaac Sim.
---

# README videos

Two videos of a generator's viewer are recorded for a README each, from a
storyboard named after the generator's crate:

- the top README's **Try it** section, from `docs/video/vinery-lab.json`: the
  parcel from above while the shoots grow through a season and the terrain
  and the row layout are edited, the outer row from outside the parcel
  trained lower and given stray shoots, one leaf in wireframe at a finer
  mesh, lobed deeper and folded, the trunk below it grown old and cut back
  to one cordon, the vine going dormant, and a pull back over the winter
  vineyard.
- [greenhouse-lab's README](../../../crates/greenhouse-lab/README.md), beside
  the picture of the parameter editor, from
  `docs/video/greenhouse-lab.json`: the bare rows grown from planting through
  a season, a new seed, more rows further apart, the outer row's crop wall
  stripped higher and picked higher, one leaf in wireframe at a finer mesh
  and other shapes, the open flowers at the next plant's head swinging their
  petals forward, a truss given more and smaller fruit, hung apart down a
  longer rachis, plum-shaped and yellow, and a pull back to the whole house.

Each is one command away:

```bash
docs/video/record.sh
```

```bash
docs/video/record.sh greenhouse-lab
```

It builds the viewer, plays the storyboard in it one video frame per
rendered frame, and writes `target/video/{crate}.mp4` and, beside it,
`{crate}-sheet.png`: the frames at 0, 2, 4… seconds, four to a row. It needs
cargo and ffmpeg, and no display: the viewer opens no window, draws into an
image at the video's size, and quits when the video is done. Every frame of
a slider's slide rebuilds the scene, so most of a recording's several
minutes go to its slides' rebuilds.

## Re-record one

1. Run `docs/video/record.sh [crate]`.
2. Check the sheet, and the frames around every camera move, against the list
   below. The viewer's log, which the script prints, has a
   `tour: step N at T s` line for each step: `N` counts the entries of `steps`
   from 1, and `T` is seconds into the video.
   `ffmpeg -ss T -i target/video/vinery-lab.mp4 -frames:v 1 -update 1 frame.png`
   takes the frame at `T`. A video that fails one is not done: change the
   storyboard, never the video.
3. Run `docs/video/publish.sh` on the branch whose pull request changes the
   video, and commit the README with the change: with no arguments it
   publishes the vineyard's to the top README, and
   `docs/video/publish.sh target/video/greenhouse-lab.mp4 crates/greenhouse-lab/README.md`
   the greenhouse's. GitHub plays only video it hosts — its Markdown drops a
   `<video>` tag, and its pages load media from no other host — so a video is
   not published with the docs' pictures (the docs-shots skill): the script
   uploads the mp4 in a comment on that pull request, where reviewers see it
   too, and puts the URL GitHub gave it on the README's video line: the one
   `https://github.com/user-attachments/assets/…` line alone in its
   paragraph. A README without one has to be given one first, any asset URL
   in that form where the video goes. It needs gh 2.102 or later and push
   access, and it posts, so run it once the video is right.

A video is right when:

- its first frame shows the opening shot built and shaded. If shapes pop in,
  raise the storyboard's `warmup`, the wall-clock seconds the opening shot
  gets before recording starts (3 by default).
- no camera move passes through a plant, a post or a fixture. Sample each
  move every half second:
  `ffmpeg -i target/video/vinery-lab.mp4 -vf "select='between(t,12,16)*not(mod(n,15))',scale=480:-2,tile=4x2" -frames:v 1 -update 1 move.png`
- the panel shows each control, tooltip up, before it moves, and the scene has
  visibly answered an edit before the next one starts
- the panel's text is legible at the README's width, about 830 px
- the file is under 100 MB, the most GitHub takes on a paid plan; a free
  plan takes 10. `docs/video/encode.sh`, which the Isaac Sim demos' videos go
  through too, encodes at a constant quality, so the size follows the length
  and how busy the picture is: at 1080p the vineyard's 70 s are about 39 MB,
  the greenhouse's 78 s about 74.
  A video over the limit is cut shorter, not encoded worse.

## Change what it shows

A storyboard is the frame (`size`, and `scale`, which enlarges the panel:
it is 260 px wide before it, so 1.25 at 1280 × 720 and 1.875 at 1920 × 1080
give it the same share of the frame),
`warmup`, the opening shot (`start`, played before recording starts), and
`steps`. Its format is documented on `Tour`, `Step` and `Pose` in
[crates/misina-lab/src/tour.rs](../../../crates/misina-lab/src/tour.rs); read
those first. In short: everything in a step starts together and takes `secs`,
and the next step starts `wait` later, `secs` if left out — so `"wait": 0` runs
two steps at once, which is how the camera keeps moving under panel edits. A
camera move that starts while another is still under way blends out of it
without stopping, so overlapping two moves rounds the corner between them; one
that starts after the last has ended starts from rest. The
last step's `wait` is the closing hold: the video ends when it runs out.
`note` is a comment.

- **Parameters** are named `fragment.field`, as Python names them:
  `params.shoot.stray` is `"shoot.stray"`. Each generator's parameters page,
  [the vineyard's](../../../crates/vinery-lab/docs/parameters.md) and
  [the greenhouse's](../../../crates/greenhouse-lab/docs/parameters.md),
  lists every one with its slider's range. A storyboard is checked against
  the params when it loads, and by `cargo test --workspace`
  (`the_readme_tour_is_valid`, and the greenhouse's `the_shots_are_valid`,
  which a bare `cargo test` leaves out).
- **An edit** is two steps, one right after the other: `show` the field — its
  section opens, the others fold, the panel scrolls to it and its tooltip comes
  up — then `set` it. Any step between them that neither shows nor sets the
  field, a camera move included, takes the tooltip down. A number slides there
  over `secs` the way a drag would, and the scene is rebuilt at every frame of
  the slide — where a person's drag reaches it only once the slider stops —
  so the video shows the scene following the slider; a whole-number field
  rebuilds at each step. A flag or a name switches at once, and the scene
  answers 150 ms later. Edits persist: every later shot sees them.
- **View options** outside the params, such as the footer's Wireframe, are
  `toggle`d by their caption.
- **The camera** orbits a focus. `focus` is x and y in meters on the scene,
  then a height above the ground there — read when the step starts, so a
  camera step that starts before a terrain edit has settled aims at the old
  ground. `yaw` is 0 looking from −y and 90 from +x; `pitch` is above the
  horizon; `radius` is the distance. Parts left out stay as they are. The
  focus is drawn in the middle of the strip the panel leaves free.
- **Vineyard rows.** The parcel is centred on 0. With the rows at
  orientation θ and spacing s, a row runs through the origin and the others
  are s apart, so the alley next to it is centred on (−sin θ, cos θ) · s/2 —
  `[-0.38, 0.82]` for the 25° and 1.8 m the storyboard sets. The camera looks
  down it from yaw θ + 90 or θ − 90, and square across the rows from yaw θ
  or θ + 180. The outermost row on a side has nothing planted beyond it, so
  a camera out there is clear of the vines at any height, though not of the
  ground, which on a hill can rise a metre within a few: read its height
  under the camera (`Ground::height`) in the scratch test that finds the
  close-up's organ. At 25° and 1.8 m that row is `Row_000`, 12 m long in the
  corner around `[28.6, -16.5]`.
- **Greenhouse rows** run along +x from the row head at x = 0, gutters
  `pitch` apart and centred on y = 0, so with n rows the outer gutter,
  `Row_00`'s, is at y = −(n − 1)/2 · pitch, and the floor ends a pitch past
  it. The crop wall's leaves reach about 0.45 m out of its stems. A camera
  beyond the floor's edge on −y is clear of the crop at any height, and looks
  at the outer wall with the rest of the house behind it. A `layout` edit
  moves the outer row, so frame close-ups for the layout they are shot in.
  A small organ, such as a flower, is framed from outside the leaves' reach
  on a long lens, a small `fov`. Between two close-ups at different heights,
  back out past the leaves first: one move between them dips into the wall.
  A truss longer than a week's growth up the stem hangs into the truss
  below, as 12 fruit of 6 cm down a long rachis do: more fruit want smaller
  ones or a shorter rachis.
- **Keep the camera out of the plants.** Beside a vineyard row it has to be
  above the canopy, about 2 m. To get down into an alley, line up over it
  first, then change only `pitch` and `radius`, which keeps the camera in the
  alley's vertical plane. Once down, a `yaw` change moves the camera sideways
  by about `radius × sin(Δyaw)`, which has to stay well inside half the
  alley, 0.9 m in the storyboard. Stray shoots and dormant canes grow out
  into the alley and past the top wire: be up over the canopy before they
  appear. On steep hills the ground behind a low camera can rise above it.
- **Finding a shot**: write a scratch storyboard of cuts — steps with
  `"secs": 0` and `"wait": 2` after a first step of `"wait": 1`, so each of
  the sheet's frames falls a second into a cut, after its edits have built —
  and run
  `docs/video/record.sh vinery-lab scratch.json /tmp/scratch.mp4`.
- **A close-up of one organ** — a leaf, a truss — is framed on where that
  organ is: print its position from a scratch `#[ignore]` test that grows the
  storyboard's params at that point of it, the seed included, and reads the
  organ's entity (the greenhouse's `elements::in_scene`), and focus on it.
  The seed decides which organs there are, so a storyboard that edits it
  frames its close-ups on the seed it ends on.

`VINERY_LAB_TOUR=docs/video/vinery-lab.json cargo run --release` plays the
vineyard's storyboard on the wall clock, without recording: a preview,
rebuild stalls included. The greenhouse's is
`GREENHOUSE_LAB_TOUR=docs/video/greenhouse-lab.json cargo run --release -p greenhouse-lab`.

## Something no step can do

Add it to the tour as a field of `Step`, phrased as what a person would change —
the panel, the camera — rather than as something drawn over the video. The tour
is in the framework and knows nothing of vines or tomatoes; anything
generator-specific belongs in its storyboard.
