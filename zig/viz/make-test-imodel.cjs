// Create a small snapshot iModel exercising tile generation: a drawing with lines, arcs, filled and
// unfilled shapes and text, and a spatial model with an open curve, solids and a long slab.
// usage: node make-test-imodel.cjs <output.bim>
const fs = require("node:fs");
const path = require("node:path");
const { createRequire } = require("node:module");

const itwin = process.env.ITWINJS_CORE ?? path.join(process.env.HOME, "opensource/itwinjs-core");
const req = createRequire(path.join(itwin, "test-apps/display-test-app/package.json"));
const backend = req("@itwin/core-backend");
const zlib = require("node:zlib");
const { IModelHost, SnapshotDb, SpatialCategory, DrawingCategory, PhysicalModel, DocumentListModel, Drawing, ModelSelector,
  CategorySelector, DisplayStyle3d, DisplayStyle2d, SpatialViewDefinition, DrawingViewDefinition } = backend;
const { Code, ColorDef, GeometryStreamBuilder, TextString, FillDisplay, GeometryParams, IModel, RenderMode, ViewFlags, ImageSourceFormat } = req("@itwin/core-common");
const { Arc3d, Box, LineSegment3d, LineString3d, Loop, Point3d, Range3d, Sphere, Vector3d, YawPitchRollAngles, Angle } = req("@itwin/core-geometry");

// An RGB checkerboard PNG.
function checkerboardPng(size, cells) {
  const crcTable = Array.from({ length: 256 }, (_, n) => {
    let c = n;
    for (let k = 0; k < 8; k++)
      c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
    return c >>> 0;
  });
  const crc = (buf) => {
    let c = 0xffffffff;
    for (const b of buf)
      c = crcTable[(c ^ b) & 0xff] ^ (c >>> 8);
    return (c ^ 0xffffffff) >>> 0;
  };
  const chunk = (type, data) => {
    const len = Buffer.alloc(4);
    len.writeUInt32BE(data.length);
    const body = Buffer.concat([Buffer.from(type), data]);
    const sum = Buffer.alloc(4);
    sum.writeUInt32BE(crc(body));
    return Buffer.concat([len, body, sum]);
  };
  const rows = [];
  for (let y = 0; y < size; y++) {
    const row = Buffer.alloc(1 + size * 3);
    for (let x = 0; x < size; x++) {
      const on = (Math.floor(x * cells / size) + Math.floor(y * cells / size)) % 2 === 0;
      row.set(on ? [230, 120, 20] : [30, 60, 160], 1 + x * 3);
    }
    rows.push(row);
  }
  const ihdr = Buffer.alloc(13);
  ihdr.writeUInt32BE(size, 0);
  ihdr.writeUInt32BE(size, 4);
  ihdr.set([8, 2, 0, 0, 0], 8);
  return Buffer.concat([Buffer.from([0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a]), chunk("IHDR", ihdr),
    chunk("IDAT", zlib.deflateSync(Buffer.concat(rows))), chunk("IEND", Buffer.alloc(0))]);
}

function geometry(build) {
  const builder = new GeometryStreamBuilder();
  build(builder);
  return builder.geometryStream;
}

(async () => {
  const output = path.resolve(process.argv[2]);
  if (fs.existsSync(output))
    fs.unlinkSync(output);

  await IModelHost.startup({ profileName: "make-test-imodel" });
  const db = SnapshotDb.createEmpty(output, { rootSubject: { name: "Tile test" } });

  // Spatial content.
  const spatialCategory = SpatialCategory.insert(db, IModel.dictionaryId, "Solids", { color: ColorDef.from(180, 180, 200).toJSON() });
  const physicalModel = PhysicalModel.insert(db, IModel.rootSubjectId, "Physical");
  const addPhysical = (geom, origin = [0, 0, 0]) => db.elements.insertElement({
    classFullName: "Generic:PhysicalObject", model: physicalModel, category: spatialCategory, code: Code.createEmpty(),
    placement: { origin, angles: {} }, geom,
  });

  addPhysical(geometry((b) => b.appendGeometry(Box.createRange(Range3d.createXYZXYZ(0, 0, 0, 4, 3, 2), true))));
  addPhysical(geometry((b) => b.appendGeometry(Sphere.createCenterRadius(Point3d.create(8, 1.5, 1.5), 1.5))));
  addPhysical(geometry((b) => {
    const params = new GeometryParams(spatialCategory);
    params.lineColor = ColorDef.red;
    params.weight = 3;
    b.appendGeometryParamsChange(params);
    b.appendGeometry(LineString3d.create([[0, 5, 0], [3, 7, 1], [6, 5, 2], [9, 7, 1], [12, 5, 0]]));
    b.appendGeometry(Arc3d.createXY(Point3d.create(15, 6, 0), 1.5));
  }));
  // A texture-mapped box.
  const textureId = backend.Texture.insertTexture(db, IModel.dictionaryId, "Checker", ImageSourceFormat.Png, checkerboardPng(64, 8));
  const materialId = backend.RenderMaterialElement.insert(db, IModel.dictionaryId, "Checker material", {
    paletteName: "Test", patternMap: { TextureId: textureId, pattern_scale: [1, 1] },
  });
  addPhysical(geometry((b) => {
    const params = new GeometryParams(spatialCategory);
    params.materialId = materialId;
    b.appendGeometryParamsChange(params);
    b.appendGeometry(Box.createRange(Range3d.createXYZXYZ(11, -1, 0, 14, 2, 3), true));
  }));

  // A long thin slab, so deep tiles must clip it.
  addPhysical(geometry((b) => b.appendGeometry(Box.createRange(Range3d.createXYZXYZ(-50, -4, -0.5, 60, -2, 0), true))));

  // Drawing content.
  const drawingCategory = DrawingCategory.insert(db, IModel.dictionaryId, "Annotation", { color: ColorDef.black.toJSON() });
  const documents = DocumentListModel.insert(db, IModel.rootSubjectId, "Documents");
  const drawingModel = Drawing.insert(db, documents, "Drawing 1");
  const addDrawing = (geom) => db.elements.insertElement({
    classFullName: "BisCore:DrawingGraphic", model: drawingModel, category: drawingCategory, code: Code.createEmpty(),
    placement: { origin: [0, 0], angle: 0 }, geom,
  });

  addDrawing(geometry((b) => {
    b.appendGeometry(LineSegment3d.createXYXY(0, 0, 20, 0));
    b.appendGeometry(LineSegment3d.createXYXY(20, 0, 20, 10));
    b.appendGeometry(Arc3d.createXY(Point3d.create(10, 5), 3));
  }));
  addDrawing(geometry((b) => {
    b.appendGeometry(Loop.create(LineString3d.create([[25, 0], [35, 0], [35, 8], [25, 8], [25, 0]])));
  }));
  addDrawing(geometry((b) => {
    const params = new GeometryParams(drawingCategory);
    params.fillDisplay = FillDisplay.Always;
    params.fillColor = ColorDef.from(0, 150, 255);
    b.appendGeometryParamsChange(params);
    b.appendGeometry(Loop.create(LineString3d.create([[40, 0], [48, 0], [44, 7], [40, 0]])));
  }));
  const fontFile = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf";
  await db.fonts.embedFontFile({ file: backend.FontFile.createFromTrueTypeFileName(fontFile) });
  const font = db.fonts.findId({ name: "DejaVu Sans" }) ?? 0;
  addDrawing(geometry((b) => {
    b.appendTextString(new TextString({ text: "Hello tiles", font, height: 2, origin: { x: 0, y: 12, z: 0 }, rotation: {} }));
  }));

  // Views.
  const spatialExtents = Range3d.createXYZXYZ(-50, -5, -1, 60, 10, 4);
  db.updateProjectExtents(spatialExtents);
  const spatialStyle = DisplayStyle3d.insert(db, IModel.dictionaryId, "Spatial style", { viewFlags: ViewFlags.fromJSON({ renderMode: RenderMode.SmoothShade, visEdges: true }) });
  const spatialView = SpatialViewDefinition.insertWithCamera(db, IModel.dictionaryId, "Spatial view",
    ModelSelector.insert(db, IModel.dictionaryId, "Spatial models", [physicalModel]),
    CategorySelector.insert(db, IModel.dictionaryId, "Spatial categories", [spatialCategory]),
    spatialStyle, Range3d.createXYZXYZ(-2, -5, -1, 18, 9, 4));
  const drawingView = DrawingViewDefinition.insert(db, IModel.dictionaryId, "Drawing view", drawingModel,
    CategorySelector.insert(db, IModel.dictionaryId, "Drawing categories", [drawingCategory]),
    DisplayStyle2d.insert(db, IModel.dictionaryId, "Drawing style"), Range3d.createXYZXYZ(-2, -2, 0, 50, 16, 0));
  db.views.setDefaultViewId(process.env.DEFAULT_VIEW === "drawing" ? drawingView : spatialView);

  db.saveChanges();
  db.close();
  await IModelHost.shutdown();
  console.log(`Wrote ${output}`);
})().catch((err) => {
  console.error(err);
  process.exit(1);
});
