#!/usr/bin/env qjsm
/*
 * eagle-agent: reverse of eagle-materialize.js - reads a real, populated
 * EAGLE .sch (schematic only; eagle-materialize.js has no board/.brd
 * counterpart, so there's nothing for a .brd extraction to round-trip
 * into) and reconstructs the same parts+nets JSON netlist format
 * eagle-materialize.js consumes - library/deviceset/device/value/x/y/rot
 * per part (or a "gates" map for a part with >1 gate instance), and each
 * net as a list of "<ref>.<pin>" (or "<ref>.<gate>.<pin>" for a
 * multi-gate part) members, matching resolveMember()'s addressing scheme
 * exactly. Part/gate x/y are snapped to a 2.54mm (0.1") grid by default -
 * a real .sch's placements are usually already grid-aligned, but hand
 * tweaks or a different original grid can leave them a fraction off,
 * which then looks like deliberate off-grid placement in the emitted
 * netlist. Pass --grid <mm> for a different pitch, or --no-grid to emit
 * the exact real coordinates unchanged.
 */
import { Console } from 'console';
import { getOpt } from 'util';
import { EagleDocument } from './eagle.js';
import { ReadFile, WriteFile } from './io-helpers.js';
import { write as writeYAML } from 'yaml';

function extract(doc, grid) {
  if(doc.type != 'schematic') throw new Error(`'${doc.filename}' is a ${doc.type}, not a schematic - only .sch extraction is supported (eagle-materialize.js has no board counterpart to round-trip into)`);

  const snap = grid ? v => Math.round(v / grid) * grid : v => v;

  const parts = doc.schematic.parts;
  const sheets = [...doc.schematic.sheets.children];

  /* Every gate instance actually placed on any sheet, grouped by part -
   * a part with exactly one instance is single-gate for our purposes
   * (even if its deviceset technically defines just one gate under some
   * name other than "G$1", e.g. HCPL2730's "A"); more than one instance
   * means it needs the "gates" map + gate-qualified net members. */
  const instancesByPart = new Map();
  for(const sheet of sheets) {
    for(const inst of sheet.instances.children) {
      const partName = inst.getAttribute('part');
      if(!instancesByPart.has(partName)) instancesByPart.set(partName, []);
      instancesByPart.get(partName).push({
        gate: inst.getAttribute('gate'),
        x: snap(+inst.getAttribute('x')),
        y: snap(+inst.getAttribute('y')),
        rot: inst.getAttribute('rot') || 'R0',
      });
    }
  }

  const partsOut = {};
  for(const ref of Reflect.ownKeys(parts)) {
    const part = parts[ref];
    const spec = {
      library: part.getAttribute('library'),
      deviceset: part.getAttribute('deviceset'),
    };
    const device = part.getAttribute('device');
    if(device) spec.device = device;
    const value = part.getAttribute('value');
    if(value) spec.value = value;

    const instances = instancesByPart.get(ref) || [];
    if(instances.length > 1) {
      spec.gates = {};
      for(const inst of instances) {
        spec.gates[inst.gate] = { x: inst.x, y: inst.y };
        if(inst.rot != 'R0') spec.gates[inst.gate].rot = inst.rot;
      }
    } else if(instances.length == 1) {
      spec.x = instances[0].x;
      spec.y = instances[0].y;
      if(instances[0].rot != 'R0') spec.rot = instances[0].rot;
    }
    partsOut[ref] = spec;
  }

  const isMultiGate = ref => (instancesByPart.get(ref) || []).length > 1;
  const netsOut = {};
  for(const sheet of sheets) {
    for(const net of sheet.nets.children) {
      const netName = net.getAttribute('name');
      const members = netsOut[netName] || (netsOut[netName] = []);
      for(const seg of net.segments) {
        for(const pr of seg.pinrefs) {
          const ref = pr.getAttribute('part');
          const pin = pr.getAttribute('pin');
          members.push(isMultiGate(ref) ? `${ref}.${pr.getAttribute('gate')}.${pin}` : `${ref}.${pin}`);
        }
      }
    }
  }

  return { parts: partsOut, nets: netsOut };
}

function main(...args) {
  globalThis.console = new Console({ inspectOptions: { colors: true, depth: Infinity } });

  const params = getOpt(
    {
      output: [true, null, 'o'],
      grid: [true, null, 'g'],
      'no-grid': [false, null],
      '@': 'input',
    },
    args,
  );

  const file = (params['@'] || args)[0];
  if(!file) {
    console.log('usage: eagle-dematerialize.js <schematic.sch> [-o output.circuit.json|output.circuit.yaml] [--grid <mm>|--no-grid]');
    return;
  }

  const grid = params['no-grid'] ? null : params.grid != null ? +params.grid : 2.54;

  const doc = EagleDocument.open(file, ReadFile);
  const circuit = extract(doc, grid);

  const out = params.output;
  const asYAML = out && /\.ya?ml$/i.test(out);
  const text = asYAML ? writeYAML(circuit) : JSON.stringify(circuit, null, 2) + '\n';

  if(out) {
    WriteFile(out, text);
    console.log(`wrote '${out}'`);
  } else {
    console.log(text);
  }
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error.message}\n${error.stack}`);
}
