# Usage: python3 zig/genschemas.py <bis-schemas checkout>
#
# Picks the latest released version of each BIS schema the addon needs, as the BisSchemas part file
# does, and writes zig/schemas.zig.
import json, os, sys
root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
bis = sys.argv[1]
inv = json.load(open(f'{bis}/SchemaInventory.json'))
wanted = [
    ('Standard', True, ['SchemaUpgradeCustomAttributes', 'CoreCustomAttributes', 'BisCustomAttributes', 'SchemaLocalizationCustomAttributes', 'Units', 'Formats']),
    ('Dgn', False, ['BisCore', 'Generic']),
    ('Domain', False, ['Analytical', 'Functional', 'LinearReferencing', 'PhysicalMaterial', 'PresentationRules']),
]
out = ['//! Generated from bis-schemas SchemaInventory.json: the latest released version of each schema the', '//! addon loads, and where it goes under Assets/ECSchemas.', '', 'pub const Schema = struct { src: []const u8, dest: []const u8 };', '', 'pub const schemas = [_]Schema{']
for subdir, versioned, names in wanted:
    for n in names:
        rel = [e for e in inv[n] if e.get('released') and e.get('path')]
        best = max(rel, key=lambda e: tuple(int(x) for x in e['version'].split('.')))
        src = best['path'].replace('\\', '/')
        assert os.path.exists(f'{bis}/{src}'), src
        dest = f'{subdir}/{n}.{best["version"]}.ecschema.xml' if versioned else f'{subdir}/{n}.ecschema.xml'
        out.append(f'    .{{ .src = "{src}", .dest = "{dest}" }},')
out += ['};', '']
open(f'{root}/zig/schemas.zig', 'w').write('\n'.join(out))
