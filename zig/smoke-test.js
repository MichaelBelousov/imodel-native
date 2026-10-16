// Smoke test for a built imodeljs.node: create an ECDb and a standalone iModel and query them.
// Usage: node zig/smoke-test.js <path to imodeljs.node>
const path = require("path");
const os = require("os");
const fs = require("fs");
const assert = require("assert");

const addon = require(path.resolve(process.argv[2]));
const errors = [];
addon.logger = {
  minLevel: 3, // errors only
  categoryFilter: {},
  logTrace() {}, logInfo() {}, logWarning() {},
  logError: (category, message) => errors.push(`${category}: ${message}`),
};

const BE_SQLITE_ROW = 100;
const dir = fs.mkdtempSync(path.join(os.tmpdir(), "imodeljs-smoke-"));

function query(db, ecsql) {
  const stmt = new addon.ECSqlStatement();
  const status = stmt.prepare(db, ecsql);
  assert.strictEqual(status.status, 0, `prepare failed: ${status.message}`);
  const rows = [];
  while (stmt.step() === BE_SQLITE_ROW)
    rows.push(stmt.getValue(0).getString());
  stmt.dispose();
  return rows;
}

try {
  const ecdb = new addon.ECDb();
  assert.strictEqual(ecdb.createDb(path.join(dir, "test.ecdb")), 0, "createDb failed");
  const schemas = query(ecdb, "SELECT Name FROM meta.ECSchemaDef ORDER BY Name");
  assert.ok(schemas.includes("ECDbMeta"), `unexpected schemas: ${schemas}`);
  ecdb.closeDb();

  const imodel = new addon.DgnDb();
  imodel.createIModel(path.join(dir, "test.bim"), { rootSubject: { name: "smoke" } });
  const subjects = query(imodel, "SELECT CodeValue FROM bis.Subject");
  assert.deepStrictEqual(subjects, ["smoke"]);
  imodel.saveChanges();
  imodel.closeFile();

  assert.deepStrictEqual(errors, []);
  console.log(`imodeljs ${addon.version}: ok (${schemas.length} ECDb schemas, root subject "${subjects[0]}")`);
} finally {
  fs.rmSync(dir, { recursive: true, force: true });
}
