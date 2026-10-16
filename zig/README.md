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
  - Supported: faceting with per-face symbology, volume and area properties, and
    `createBRepGeometry` for unite, subtract and intersect of b-reps, boxes, spheres, cones,
    cylinders and tori.
  - Not supported: reading Parasolid b-reps in existing iModels. They fail to restore, as on a
    platform without a solid kernel. The other `createBRepGeometry` operations and b-rep snapping
    also report failure.
- **Visualization is a stub** (`iModelCore/Visualization`). It produces no tiles, and element
  graphics requests resolve as `NoGeometry`.
- **No crash reporting.** Crashpad isn't built; `iModelJsNodeAddon/CrashReportingNone.cpp` stands in
  for it.
- **The unit tests (gtest) aren't built.**

## Files

- `sources.zig`: the sources each library compiles, extracted from the `.mke`/`.mki` files.
- `schemas.zig`: the BIS schemas delivered to `Assets/ECSchemas`. Regenerate it with
  `python3 zig/genschemas.py <bis-schemas checkout>` after updating the `bis_schemas` dependency.
