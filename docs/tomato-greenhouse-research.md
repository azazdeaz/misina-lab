# Tomato greenhouses: what a generator needs

The inside of a Western European high-wire tomato glasshouse, as the
numbers a generator can be built from, and the smallest set of variations
worth building first. It is the groundwork for a tomato greenhouse crate
beside `vinery-lab`, aimed first at truss-harvesting robots such as
eternal.ag's Harvester.

The scene is the crop and what a robot touches or sees up close in a row:
floor, pipe rail, gutter, slab, wires, hooks, trusses. The building (roof,
glass, columns, screens, top lights) comes from an HDR or a Gaussian splat
and is not covered here.

The numbers come from Dutch research compartments, supplier sheets and one
measured truss study. **(est.)** marks a value no source gives. Belgium,
Germany, France and the UK use the same substrate high-wire system (in
Flanders more than 96% of tomato area is on substrate). None of their
dimensions were found, though, so the layout here is Dutch.

## Where this leaves the generator

| Topic | In real glasshouses | Initial generator |
| --- | --- | --- |
| Row layout | One hanging gutter every 1.6 m; stems trained in a V to two wires; a pipe rail in every path; a concrete main path at the row head | The same, with pitch, gutter height, wire height and rail gauge as parameters |
| Robot surface | 51 mm heating pipes, 550–600 mm apart, double as rails; tilt at most 2° and the two rails at most 15–21 mm apart in height | A rail pair per path, gauge and pipe choices, and an unevenness knob scaled to that spec |
| Crop | Two-stem plants at 2.5 → 3.8 stems/m²; a truss every 3 leaves; 14–18 leaves kept; lowered bare stems along the gutter | Stems built plant by plant and solved to put the harvest band at a given height |
| Fruit | Dutch area: truss (TOV) about 53%, cherry 27%, loose 19% | A simple TOV truss first and a branched cherry truss second; no loose fruit |
| Ground truth | Robots cut at the peduncle base or at its bend, and skip trusses without clearance | Cut frames on every truss, ripeness on every fruit, and harvestable and clearance flags on every truss |

## The target robot

eternal.ag (EHT Eternal Horticulture Technologies GmbH, Cologne, founded in
2025) sells the **Harvester** for truss tomatoes. The figures below are
company claims ([product page][eternal], [HortiDaily][hd-sim],
[HortiDaily, Tomatoworld][hd-tw], [Universal Robots][ur]).

- It cuts the peduncle and never touches the fruit, with a patented end
  effector. Throughput is up to 120 trusses/h, for up to 22 h/day.
- Real-time vision reads plant height, canopy, truss position and ripeness.
  Before each cut the robot checks that there is clear space around the
  peduncle. It skips a truss that has no collision-free path, or that is
  "snagged between stems". Autonomous coverage is up to about 85%
  ([The Packer][packer], read through search excerpts only).
- The end effector is sterilized at the end of each row, against ToBRFV.
- It is "fully autonomous", with "autonomous lane switching on concrete
  floors". The company's Trolley product shares its omni-directional drive
  ([Fruitnet][fruitnet]).
- Universal Robots lists eternal.ag as a UR5e user, with the arm on a
  trolley. Press captions mention one arm.
- Development is simulation-first. The simulator is not named. A customer's
  greenhouse is rebuilt from its blueprint "within an hour", including
  layout, navigation paths and floor friction.
- The first customer is Van Noord Growers in Zeeland (NL), with 8.5 ha of
  tomatoes and cucumbers. One robot has run there since September 2025, and
  three were planned for summer 2026 ([HortiDaily][hd-vn]).

**Inferred, not confirmed:** in a Dutch row, the heating-pipe rail fills the
only floor between two gutters. A robot that switches lanes on concrete
therefore almost certainly rides that rail inside the row, as every
pipe-rail trolley does.

So a scene for this robot has to carry:

- a rail to ride, with its real gauge and its real unevenness;
- a concrete path to change rows on;
- trusses on peduncles with a cut point a policy can be scored against;
- the hooks, twine, clips, leaves and neighbouring stems that decide
  whether that cut point is reachable.

## A row, in numbers

Looking along a row, with Z up:

```text
 4.5 m  ─●──────────●───────────────────●──────────●──  crop wires, two per gutter
          ╲  heads ╱                     ╲        ╱
           ╲      ╱   ← canopy walls →    ╲      ╱      leaves kept above the harvest band
            ╲    ╱                         ╲    ╱
 1.2 m       ╲  ╱   ripe trusses, bare stem ╲  ╱        harvest band (est.)
 0.5 m     [gutter]                       [gutter]      hanging, slab and blocks on top
 0.15 m                  ◯       ◯                      pipe rail, 51 mm, 600 mm apart (height est.)
 0 ───────────────────────────────────────────────────  white film; concrete on the main path
           |<──────────── 1.6 m ────────────>|
```

### Layout and floor

| Quantity | Default | Range | Source |
| --- | --- | --- | --- |
| Gutter (row) pitch | 1.6 m | 1.5–1.6 m | [GTB-1139][gtb1139], [Lek/Habo][lekhabo] |
| Row length | 20 m for simulation | 91–115 m on farms | [WUR thesis][thesis] |
| Main concrete path | 3 m | 3–5 m | [WUR thesis][thesis] |
| Floor | White film under the crop; concrete on the main path | | |

### Pipe rail

| Quantity | Default | Range | Source |
| --- | --- | --- | --- |
| Pipe | 51 × 2.25 mm | 45 × 2 mm | [AgroArbo][agroarbo] |
| Gauge, centre to centre | 600 mm | 425, 500, 550, 600, 800 mm | [AgroArbo][agroarbo], [Royal Brinkman][rb-trolley] |
| Rail pairs | one per path | | [GTB-1139][gtb1139] (6 pairs per 9.6 m truss) |
| Height above floor | 0.15 m (est.) | | |
| Support spacing | 1.0 m | at most 1.0–1.25 m | [AgroArbo][agroarbo] |
| Tilt | | at most 2° | [AgroArbo][agroarbo] |
| Height difference between the rails | | at most 15, 17, 19, 21 mm at 425, 500, 550, 600 mm gauge | [AgroArbo][agroarbo] |
| Growth pipe | two 35 mm pipes, stacked, one pair per path | about 0.3 m above the slab; 0.6–0.8 m under the head with LED | [GTB-1139][gtb1139], [GTB-1177][gtb1177] |

### Gutter and substrate

| Quantity | Default | Range | Source |
| --- | --- | --- | --- |
| Gutter | hanging, 0.5 m above the floor | standing, on 0.26–0.43 m supports | [GTB-1139][gtb1139], [Hortitrade][hortitrade] |
| Gutter width | 0.2 m | 0.15–0.23 m | [Metazet][metazet], [Lek/Habo][lekhabo] |
| Gutter slope | 0.5% | | [WUR, Jordan guide][jordan] (unverified for NL) |
| Stone wool slab | 1.2 × 0.195 × 0.075 m | 1.0–1.2 long, 0.15–0.20 wide, 0.075–0.10 high | [GTB-1139][gtb1139], [GTB-1177][gtb1177] |
| Propagation block | 0.10 × 0.10 m | | [WUR 716057][wur716057] |
| Drip | one stake per plant, 2 L/h | | [GTB-1139][gtb1139] |

### Crop support

| Quantity | Default | Range | Source |
| --- | --- | --- | --- |
| Crop wire | 4.5 m above the floor | 4.5–4.6 m in WUR compartments; 3.1–3.8 m above the gutter on five full-LED farms | [GTB-1139][gtb1139], [GTB-1177][gtb1177], [WPR-1379][wpr1379] |
| V-training | two wires per gutter, alternate stems to each | wire offset ±0.35 m (est.) | [GTB-1139][gtb1139] |
| Hook | 0.22 m, 10–15 m of twine | 0.18 m in low houses | [Royal Brinkman][rb-hooks], [WUR thesis][thesis] |
| Stem clip | 23 mm | | [Bato][bato] |
| Truss support | arch of 5–6 mm wire, or a 12 cm truss hook | | [Royal Brinkman][rb-truss] |

### Plant and stem

| Quantity | Default | Range | Source |
| --- | --- | --- | --- |
| Stem density | 3.5 stems/m² | 2.5 at planting; 3.0–3.75 on five farms in winter; 3.8–4.1 late | [WPR-1379][wpr1379], [GTB-1139][gtb1139], [GTB-1177][gtb1177] |
| Plant | 2 stems, 0.36 m apart | 0.33–0.5 m | [GTB-1139][gtb1139] |
| Stem diameter | 16 mm | 12–20 mm | [Kondo et al.][kondo] |
| Leaves between trusses | 3 | | [Source.ag][sourceag] |
| Stem per truss | 0.25–0.35 m | from 20–35 cm/week growth at about one truss a week | [Kim & Kubota][kim], [Duijvestijn][duijvestijn] |
| Leaves kept | 16 | 14–18; 18–21 in the last month | [WUR thesis][thesis] |
| Leaf | 0.45 m, 7 or 9 leaflets | 0.35–0.6 m (est.); pressed leaves of 24–40 cm, with 3–4 lateral pairs and leaflets covering about 0.2 × length² | [Naturalis][naturalis] |
| Stem length, end of season | | up to 12 m | [Duijvestijn][duijvestijn] |
| Lowering | about 25 cm/week; hooks shifted along the wire; bare stem lies along the row | | [Duijvestijn][duijvestijn], [NL2000684C2][lowering-patent] |

### Truss and fruit

| Quantity | Default | Range | Source |
| --- | --- | --- | --- |
| Fruits per TOV truss | 6 | medium: 5–7 at 100–130 g; large: 4–6 at 160–220 g | [De Ruiter][deruiter], [Formoza][formoza] |
| Cherry truss | 15 fruits, branched (est.) | 12–20, 20–30 mm (est.) | |
| Fruit diameter | 52 mm on a medium truss | 43–63 mm | [Kondo et al.][kondo] |
| Peduncle, stem to first fruit | 79 mm | 33–132 mm | [Kondo et al.][kondo] |
| Peduncle with rachis | 148 mm | 98–227 mm | [Kondo et al.][kondo] |
| Peduncle diameter | 7 mm | 4.8–11.8 mm | [Kondo et al.][kondo] |
| Peduncle angle from the stem | 70° | 5–139° | [Kondo et al.][kondo] |
| Cutting force | | about 60 N for the thickest peduncle | [Kondo et al.][kondo] |
| Ripening | from the base of the truss to its tip; due when the terminal fruit is at least "turning" | | [AHPPEBot][ahppebot] |
| Colour stages | green, breaker, turning, pink, light red, red: coloured on up to 10%, 30%, 60% and 90% of the surface, and more | colour starts at the blossom end | [USDA][usda] |
| Fruit size by stage | full size at mature green | about 15% of it at fruit set (est.) | |
| Fruit shape, height over width | 0.9 for round TOV (est.) | beefsteak 0.6–0.75, plum 1.3–1.8 (est.) | |
| Lobes | 3–5 on round fruit, shallow ribs (est.) | beefsteak ribs about 15% of the radius deep (est.) | |
| Calyx | 5–6 narrow-lanceolate sepals | 15–25 mm long on a large TOV fruit (est.) | |

The Kondo figures were measured in a Dutch-style high-wire house in Japan,
on medium fruit. They are the only measured truss geometry found, so they
are defaults to sample around, not Dutch facts.

## The initial variation set

Each of these is a knob because it changes what the robot drives on, reaches
for or has to see past. The seed varies everything else within the ranges
above.

| Variation | Options or range | Default | Why it is in |
| --- | --- | --- | --- |
| Rail gauge and pipe | 425–800 mm; 45 or 51 mm | 600 mm, 51 mm | Sets the drive's track, and gauges differ between growers |
| Rail unevenness | 0 level; 1 at the AgroArbo limits (2°, 15–21 mm); 2 out of spec | 0.5 | What a rail-riding chassis and its arm calibration feel |
| Rows | pitch 1.5–1.8 m; 1–12 rows; length 5–120 m; main path 3–5 m | 1.6 m, 6 rows, 20 m, 3 m | The lane change on concrete, and the aisle the arm works in |
| Gutter | height 0.3–1.0 m; hanging or standing | 0.5 m, hanging | Moves the slab and the lowered stems relative to the rail |
| Wire height | 3.5–5.5 m | 4.5 m | How tall the crop stands |
| Harvest height, the lowest unharvested truss | 0.8–1.8 m; 6–12 trusses from there to the head | 1.2 m (est.); 9 (est.) | Where the arm has to reach. Stems are solved to put trusses here, so stem per truss follows: 0.34 m at the defaults, inside the measured growth range |
| Stem density | plant spacing 0.3–0.5 m; extra-stem share 0–0.5 | 0.36 m, 0 | Occlusion by neighbours, and trusses snagged between stems |
| Leaves | kept 10–21; length 0.35–0.6 m; an old leaf's droop 0–150° | 16, 0.45 m, 90° (est.) | The main occlusion knob, behind the ~15% of trusses the Harvester leaves |
| Truss | fruit count, fruit diameter, branched or simple | 6 × 60 mm, simple | What is cut; size drives occlusion and the gripper's approach. The TOV medium, TOV large and cherry presets are cfg values, not a choice field |
| Peduncle | length, diameter, angle; share turned to the path | the Kondo means; 0.8 (est.) | Where the cut point is. Trusses turned into the crop are the hard ones |
| Ripeness | ripe trusses per stem 0–3; colour spread within a truss | 1, 0.2 | Which trusses are due; the ready rule reads the terminal fruit |
| Fruit colour | red, orange, yellow, brown | red | Ripe must not mean red to the detector |
| Truss support | none, hook or arch | none | Plastic or wire right where the cut goes |
| Growth pipe | on or off; height above the slab | on, 0.3 m | A pipe at fruit height that blocks and occludes |
| Missing fruit | probability 0–0.3 | 0.05 | Cheap variety for perception |

The rest is fixed, with one good version and no knob: the slab, block and
drip look, the white film, hooks and twine, the two-wire V, stem taper, the
terminal, lateral and intercalary leaflet shapes, and the calyx.

A young crop needs no knob of its own. A lowered length of 0 with no ripe
trusses gives the crop before its first lowering.

## Left out, and why

- **The building.** Columns, roof, screens and top lights come from the
  background. One row per truss runs through a column ([Lek/Habo][lekhabo]),
  so a column-in-row option is the first addition if a splat and the
  generated rows have to line up.
- **Interlighting.** LED bars in the canopy are a minority practice, and no
  area share was found. Where they are used they are obstacles at fruit
  height.
- **Loose round, beefsteak and plum.** These are picked fruit by fruit and
  make up about 19% of Dutch area. A truss harvester does not cut them.
- **Cucumber.** It is eternal.ag's next crop, and would be a second plant
  element on the same layout.
- **Props.** These include bumblebee hives (20–60 cm above the ground on
  the path's south side, [Koppert][koppert]), trolleys and crates parked at
  the row head, CO2 lay-flat tubes, and leaf debris on the film. Each would
  be a cheap one-file element later.
- **Coco and soil.** Coco changes only the bag, and soil-grown tomato is
  under 3% of Flemish area.
- **Physics.** The first version is static meshes with simple colliders.
  The next step after it is the peduncle as a `Cable`, so that the
  framework's `Shears` can cut it with the truss hanging off it. Cutting a
  rod already works; fruit mass hanging off one is the new part.

## Ground truth

Every fruit, truss and peduncle is its own prim. Render-time labels, such
as instance masks and occlusion per camera, then come from Replicator off
the prim tree, while what is fixed at generation time is written on the
prims:

```text
/Greenhouse/Rows/Row_03
  /Rail                   two pipe capsules and their supports
  /Gutter  /Slab_00 …  /Wire_L  /Wire_R
  /Plant_017
    /Stem_0               Xform
      /Tube               the lowered run, the rise and the head
      /Hook  /Twine
      /Leaf_00 …          -> parts/Leaf_k
      /Truss_04           Xform, unique
        /Peduncle         -> parts/Truss_k: peduncle, rachis, pedicels
        /Fruit_00 …       -> parts/Fruit_k, by size, shape and stage
        /Cut              Xform at the peduncle base, +X along the peduncle
        /Bend             Xform at the peduncle's maximum curvature
```

`Cut` and `Bend` are [AHPPEBot][ahppebot]'s SP and CP keypoints. Cut-flush
grippers use the first; clamp-and-cut grippers use the second. Both are
frames, because the approach direction matters as much as the point.

Attributes:

| Prim | Attribute | Meaning |
| --- | --- | --- |
| `Fruit_NN` | `stage` | the growth stage or the USDA colour class |
| `Fruit_NN` | `ripeness` | red fraction, 0–1, from which any of the datasets' class schemes derives |
| `Truss_NN` | `harvestable` | terminal fruit at least "turning" |
| `Truss_NN` | `clearance` | distance from `Cut` to the nearest leaf, stem, wire, hook or other truss |
| `Truss_NN` | `snagged` | the truss crosses a stem other than its own |

`clearance` and `snagged` are the two checks the Harvester runs before a
cut. Exported per truss, they score a policy on reachable trusses without
rendering anything.

This needs one addition to the framework: `Node` in
[`scene/doc.rs`](../crates/misina-lab/src/scene/doc.rs) has no custom
attributes. A generic attribute map, written out as USD custom attributes,
fits misina-lab's rule that it knows nothing about any one generator.

## How it maps onto misina-lab

The same layered pipeline the vineyard uses, in build order:

1. **Terrain:** the core element, flat.
2. **Layout:** rows, paths and plant positions, solved like the vineyard's
   parcel. It owns no prims.
3. **Fixtures:** rail, growth pipe, gutter, slab, block, drip and wires, all
   cylinders and boxes, as in `pole` and `wire`.
4. **Stems:** one strand per stem, unique, since each stem's lowered run
   differs. A stem authors a leaf config at each node that keeps its leaf
   and a truss config in every third internode.
5. **Leaves:** a compound leaf
   ([leaf.rs](../crates/greenhouse-lab/src/elements/leaf.rs)) of leaflet
   outlines built from their veins
   ([leaflet.rs](../crates/greenhouse-lab/src/elements/leaflet.rs)), quantized.
6. **Trusses:** peduncle, rachis and pedicels as one strand mesh, quantized.
   A truss authors a fruit config at each pedicel tip.
7. **Fruit:** quantized by size, shape and stage, coloured per point.

Ripeness changes colour, and colour is baked per mesh. So fruit are
quantized on it, and every fruit stays its own prim so its own `ripeness`
is exact.

## Open questions

For eternal.ag or Van Noord Growers:

1. Does the Harvester ride the rail inside the row? What are the gauge,
   pipe and rail height at their sites?
2. How high above the floor do the ripe trusses hang, and how high are the
   wire and the gutter?
3. Is the cut made at the peduncle base or at its bend?
4. How far apart are the two V wires, and are the trusses turned to the
   path?
5. Where do the lowered stems lie (on stem supports along the gutter, or on
   the floor), and how long are they?
6. Which simulator do they use, and would a USD scene drop into it?

These are unsourced and should be calibrated from photos: rail height,
leaf length and leaflet count, the share of trusses on supports, and the
Dutch gutter slope.

[agroarbo]: https://www.agroarbo.nl/glastuinbouw/buisrailsysteem/
[ahppebot]: https://arxiv.org/html/2405.06959v1
[bato]: https://www.hortidaily.com/article/6029118/bato-optimises-23mm-clip/
[deruiter]: https://www.hortidaily.com/article/6024428/de-ruiter-offers-a-choice-in-middle-truss-tomatoes/
[duijvestijn]: https://duijvestijntomaten.nl/en/cultivation/
[eternal]: https://eternal.ag/en/products/harvester
[formoza]: https://www.freshplaza.com/north-america/article/9803762/formoza-ticks-all-the-boxes-for-large-tov-growers/
[fruitnet]: https://www.fruitnet.com/eurofruit/eternalag-launches-robot-ready-trolley/271371.article
[gtb1139]: https://edepot.wur.nl/191959
[gtb1177]: https://edepot.wur.nl/218567
[hd-sim]: https://www.hortidaily.com/article/9837382/we-can-rebuild-that-entire-greenhouse-within-an-hour-in-our-simulation-environment/
[hd-tw]: https://www.hortidaily.com/article/9881522/eternal-ag-s-reveals-new-tomato-harvesting-robot-at-tomatoworld/
[hd-vn]: https://www.hortidaily.com/article/9844042/eternal-ag-scales-fully-autonomous-harvesting-robots-with-first-customer-van-noord-growers/
[hortitrade]: https://www.hortitrade.com/nl/catalogus/mb0600008/teeltgoten-tomaten-20-cm
[jordan]: https://edepot.wur.nl/403806
[kim]: https://pmc.ncbi.nlm.nih.gov/articles/PMC12509216/
[kondo]: https://www.jstage.jst.go.jp/article/eaef/3/1/3_1_20/_pdf
[koppert]: https://www.koppert.ca/bumblebee-pollination/best-practices/optimal-placement-of-bumblebee-hives/
[lekhabo]: https://www.lekhabo.nl/lek-habo-groep/nieuws/395
[lowering-patent]: https://patents.google.com/patent/NL2000684C2/nl
[metazet]: https://metazet.com/teeltgoot/formflex-vg-teeltgoot/
[naturalis]: https://bioportal.naturalis.nl/en/specimen/L.4471084
[packer]: https://www.thepacker.com/news/packer-tech/precision-over-pressure-how-eternal-ag-solving-hardest-challenges-autonomous-tom
[rb-hooks]: https://royalbrinkman.com/knowledge-center/crop-rotation/twine-tomato-hooks
[rb-trolley]: https://royalbrinkman.com/mechanical-equipment/internal-transport/harvest-trolleys/
[rb-truss]: https://royalbrinkman.com/crop-rotation/clips-truss-support/truss-support/
[sourceag]: https://docs.source.ag/en/articles/96771-protocols-tomato-vegetative
[thesis]: https://edepot.wur.nl/176100
[ur]: https://www.universal-robots.com/case-stories/eternal/
[usda]: https://www.ecfr.gov/current/title-7/section-51.1860
[wpr1379]: https://edepot.wur.nl/685694
[wur716057]: https://edepot.wur.nl/716057
