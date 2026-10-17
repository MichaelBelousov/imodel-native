// Dump tile tree JSON and tile content from the native addon for inspection, e.g. to compare the
// official Visualization module's output with ours.
// usage: node dump-tiles.cjs <file.bim> <outDir> [modelId...]
// Set IMODELJS_NODE to the path of an imodeljs.node to use instead of the installed one.
const fs = require("node:fs");
const path = require("node:path");
const { createRequire } = require("node:module");

const itwin = process.env.ITWINJS_CORE ?? path.join(process.env.HOME, "opensource/itwinjs-core");
const req = createRequire(path.join(itwin, "test-apps/display-test-app/package.json"));
const { IModelHost, SnapshotDb } = req("@itwin/core-backend");
const { iModelTileTreeIdToString, BatchType, defaultTileOptions, ImdlHeader, FeatureTableHeader, GltfHeader, TileFormat } = req("@itwin/core-common");
const { ByteStream, DbResult } = req("@itwin/core-bentley");
if (process.env.IMODELJS_NODE) {
  const backendReq = createRequire(req.resolve("@itwin/core-backend"));
  backendReq("@bentley/imodeljs-native").NativeLibrary._nativeLib = require(path.resolve(process.env.IMODELJS_NODE));
}

function describe(bytes) {
  const stream = ByteStream.fromUint8Array(bytes);
  const header = new ImdlHeader(stream);
  const out = {
    format: header.format === TileFormat.IModel ? "iMdl" : header.format,
    version: `${header.versionMajor}.${header.versionMinor}`,
    headerLength: header.headerLength,
    flags: header.flags,
    contentRange: header.contentRange.toJSON(),
    tolerance: header.tolerance,
    numElementsIncluded: header.numElementsIncluded,
    numElementsExcluded: header.numElementsExcluded,
    tileLength: header.tileLength,
    byteLength: bytes.byteLength,
    emptySubRanges: header.emptySubRanges,
  };
  const ftStart = stream.curPos;
  const ft = FeatureTableHeader.readFrom(stream);
  out.featureTable = { start: ftStart, length: ft.length, numSubCategories: ft.numSubCategories, count: ft.count };
  out.featureTableWords = Array.from(new Uint32Array(bytes.slice(ftStart + 12, ftStart + ft.length).buffer)).map((w) => `0x${w.toString(16)}`);
  stream.curPos = ftStart + ft.length;
  const gltfStart = stream.curPos;
  const gltf = new GltfHeader(stream);
  out.gltf = { start: gltfStart, version: gltf.version, gltfLength: gltf.gltfLength, scenePosition: gltf.scenePosition, sceneStrLength: gltf.sceneStrLength, binaryPosition: gltf.binaryPosition };
  out.rawGltfHeaderWords = Array.from(new Uint32Array(bytes.slice(gltfStart, gltfStart + 20).buffer));
  out.scene = JSON.parse(Buffer.from(bytes.slice(gltf.scenePosition, gltf.scenePosition + gltf.sceneStrLength)).toString("utf8"));
  return out;
}

(async () => {
  const [bim, outDir, ...modelArgs] = process.argv.slice(2);
  fs.mkdirSync(outDir, { recursive: true });
  await IModelHost.startup({ profileName: "dump-tiles" });
  const db = SnapshotDb.openFile(bim);

  let modelIds = modelArgs;
  if (modelIds.length === 0) {
    // eslint-disable-next-line deprecation/deprecation
    db.withPreparedStatement("SELECT ECInstanceId FROM bis.GeometricModel WHERE IsPrivate=false AND IsTemplate=false", (stmt) => {
      while (stmt.step() === DbResult.BE_SQLITE_ROW)
        modelIds.push(stmt.getValue(0).getId());
    });
  }

  const summary = [];
  for (const modelId of modelIds) {
    const treeId = iModelTileTreeIdToString(modelId, { type: BatchType.Primary, edges: process.env.EDGES ? { type: "compact", smooth: true } : false }, defaultTileOptions);
    try {
      const start = Date.now();
      const props = await db.tiles.requestTileTreeProps(treeId);
      fs.writeFileSync(path.join(outDir, `${modelId}.tree.json`), JSON.stringify(props, undefined, 2));
      const contentId = props.rootTile.contentId;
      const content = await db.tiles.requestTileContent(treeId, contentId);
      const bytes = content instanceof Uint8Array ? content : new Uint8Array(content.content ?? content);
      fs.writeFileSync(path.join(outDir, `${modelId}.root.imdl`), bytes);
      const desc = describe(bytes);
      fs.writeFileSync(path.join(outDir, `${modelId}.root.json`), JSON.stringify(desc, undefined, 2));
      summary.push({ modelId, treeId, rootContentId: contentId, bytes: bytes.byteLength, elements: desc.numElementsIncluded, flags: desc.flags, seconds: (Date.now() - start) / 1000 });
    } catch (err) {
      summary.push({ modelId, treeId, error: err?.message ?? String(err) });
    }
  }
  console.log(JSON.stringify(summary, undefined, 1));
  db.close();
  await IModelHost.shutdown();
})().catch((e) => { console.error(e); process.exit(1); });
