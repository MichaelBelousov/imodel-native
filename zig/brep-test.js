// End-to-end test of the SMLib b-rep admin through @itwin/core-backend: subtract a sphere from a
// box, store the result in an element, then measure it and export its mesh.
// Usage: node zig/brep-test.js <path to imodeljs.node> <path to an @itwin/core-backend package>
const path = require("path");
const os = require("os");
const fs = require("fs");

const assert = require("assert");
const [addonPath, backendDir] = process.argv.slice(2).map((p) => path.resolve(p));
const from = (name) => require(require.resolve(name, { paths: [backendDir] }));

// Use our addon instead of the published one.
const { NativeLibrary } = from("@bentley/imodeljs-native");
NativeLibrary._nativeLib = require(addonPath);

const backend = require(backendDir);
const common = from("@itwin/core-common");
const geom = from("@itwin/core-geometry");

async function main() {
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), "imodeljs-brep-"));
  await backend.IModelHost.startup({ cacheDir: dir });
  const db = backend.StandaloneDb.createEmpty(path.join(dir, "brep.bim"), { rootSubject: { name: "brep test" } });

  const box = geom.Box.createRange(geom.Range3d.createXYZXYZ(0, 0, 0, 2, 2, 2), true);
  const sphere = geom.Sphere.createCenterRadius(geom.Point3d.create(2, 2, 2), 1);
  const entryArray = [common.ElementGeometry.fromGeometryQuery(box), common.ElementGeometry.fromGeometryQuery(sphere)];
  let result;
  const status = db.createBRepGeometry({ operation: common.BRepGeometryOperation.Subtract, entryArray, onResult: (info) => { result = info; } });
  console.log("createBRepGeometry status", status, "entries", result?.entryArray.length, "opcode", result?.entryArray[0]?.opcode);
  if (!result)
    throw new Error("no result");

  const modelId = backend.PhysicalModel.insert(db, common.IModel.rootSubjectId, "model");
  const categoryId = backend.SpatialCategory.insert(db, common.IModel.dictionaryId, "category", new common.SubCategoryAppearance());
  const elementId = db.elements.insertElement({
    classFullName: "Generic:PhysicalObject",
    model: modelId,
    category: categoryId,
    code: common.Code.createEmpty(),
    placement: { origin: [0, 0, 0], angles: {} },
    elementGeometryBuilderParams: { entryArray: result.entryArray },
  });
  db.saveChanges();
  console.log("inserted", elementId, "range", JSON.stringify(db.elements.getElement(elementId).placement.calculateRange().toJSON()));

  const volume = await db.getMassProperties({ operation: common.MassPropertiesOperation.AccumulateVolumes, candidates: [elementId] });
  console.log("volume", volume.volume?.toFixed(4), "(expected", (8 - Math.PI / 6).toFixed(4) + ")", "area", volume.area?.toFixed(4), "centroid", JSON.stringify(volume.centroid));
  assert.ok(Math.abs(volume.volume - (8 - Math.PI / 6)) < 1e-3);

  for (const opts of [{}, { chordTol: 0.01, angleTol: 0.3 }, { angleTol: 0.05 }, { chordTol: 0.001 }]) {
    let triangles = 0, meshes = 0;
    const status = db.exportGraphics({
      elementIdArray: [elementId],
      ...opts,
      onGraphics: (info) => { meshes++; triangles += info.mesh.indices.length / 3; },
    });
    console.log("exportGraphics", JSON.stringify(opts), "status", status, "meshes", meshes, "triangles", triangles);
    assert.ok(meshes > 0 && triangles > 0);
  }

  db.close();
  await backend.IModelHost.shutdown();
  fs.rmSync(dir, { recursive: true, force: true });
}

main().catch((e) => { console.error(e); process.exit(1); });
