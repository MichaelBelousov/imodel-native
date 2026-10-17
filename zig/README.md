# Zig build (Linux)

A replacement for BentleyBuild that builds the iModelCore libraries and the Node addon,
`imodeljs.node`, with the Zig build system and no access to `imodel-native-internal`.

## Requirements

- Zig 0.16
- libcurl and OpenSSL development packages (`libcurl4-openssl-dev`, `libssl-dev`), used by CloudSQLite
- [usd-brep](../../usd-brep) checked out next to this repo, for SMLib
- Node.js, only to run the tests

Everything else comes from this repo or is fetched by `zig build`: zlib, libpng, pugixml and the
BIS schemas from [iTwin/bis-schemas](https://github.com/iTwin/bis-schemas).

## Building

```sh
zig build addon                       # zig-out/imodeljs-linux-x64/{imodeljs.node,Assets}
zig build addon -Doptimize=ReleaseFast
zig build smoke                       # create an ECDb and an iModel with the built addon
zig build iTwinPlatform               # any single library, by its bmake name
zig build smkernel-test               # check the SMLib operations behind createBRepGeometry
```

`imodeljs-linux-x64/` has the layout `@bentley/imodeljs-native` expects next to its
`NativeLibrary.js`. To run `@itwin/core-backend` against it, copy the directory into that package,
or set `NativeLibrary._nativeLib` to the required addon before the backend starts, as
`brep-test.js` does:

```sh
node zig/brep-test.js zig-out/imodeljs-linux-x64/imodeljs.node path/to/node_modules/@itwin/core-backend
```

## Differences from Bentley's build

- **B-reps use SMLib instead of Parasolid** (`iModelCore/SMBRepGeometry`). Bodies are stored under
  the usual `ParasolidBRep` geometry opcode, tagged as SMLib data.
  - Supported: faceting with per-face symbology, volume and area properties, and every
    `createBRepGeometry` operation on b-reps, boxes, spheres, cones, cylinders, tori and planar
    regions. Sweep paths are line segments (with mitered corners) or a single smooth curve. A
    sheet subtracted from or intersected with a solid must be planar; it acts as a half space.
    Emboss works on solids and planar regions.
  - Parasolid b-reps in existing iModels (bare binary XT, schema 12006) are read by
    `ParasolidXT.cpp` and rebuilt as SMLib bodies by `SmParasolid.cpp`: planes, cylinders, cones,
    spheres, tori, B-surfaces, offset and swept surfaces, rolling-ball blends along circles and
    ellipses, and lines, arcs, B-curves, SP-curves and intersection curves. Not supported: spun
    surfaces, blends along intersection curves (such bodies restore as open sheets), text XT and
    other base schemas. `zig build parasolid-test` builds a harness for extracted blobs.
  - B-rep snapping reports failure.
- **Visualization is a replacement** (`iModelCore/Visualization`). It produces iMdl tiles for 2d and
  3d models and element graphics for single elements or dynamic geometry. Each element is faceted at
  the tile's chord tolerance, as `exportGraphics` does, simplified to that tolerance with
  [meshoptimizer](https://github.com/zeux/meshoptimizer), and clipped to the tile.
  - Supported: surfaces with crease edges and silhouettes, open curves, filled and unfilled regions,
    text, texture-mapped materials (pattern maps), and planar classifier trees.
  - Not supported: volume classifier trees, instancing of geometry parts (each instance is faceted
    separately), schedule animation node splitting, section cuts, line styles and area patterns,
    render material properties other than color and transparency, and a persistent tile cache.
  - `node zig/viz/dump-tiles.cjs` dumps tile trees and root tiles, e.g. to compare with Bentley's addon.
    `node zig/viz/make-test-imodel.cjs` writes a small iModel exercising each kind of graphic.
- **No crash reporting.** Crashpad isn't built; `iModelJsNodeAddon/CrashReportingNone.cpp` stands in
  for it.
- **The unit tests (gtest) aren't built.**

## Files

- `sources.zig`: the sources each library compiles, extracted from the `.mke`/`.mki` files.
- `schemas.zig`: the BIS schemas delivered to `Assets/ECSchemas`. Regenerate it with
  `python3 zig/genschemas.py <bis-schemas checkout>` after updating the `bis_schemas` dependency.
