#!/usr/bin/env qjsm
/*
 * eagle-agent: deterministic netlist -> EAGLE .scr materializer
 * (doc/eagle-agent.md §2c). Takes a small JSON netlist (parts + nets) and
 * emits ADD/INVOKE/VALUE/NET commands with real computed coordinates.
 * Single-gate parts default to R0; give an explicit "rot" ("R90"/"R180"/
 * "R270") to rotate a part - e.g. a resistor feeding a vertically-drawn
 * LED needs "R90" itself to keep the connecting wire straight. Multi-gate
 * parts (spec.gates) aren't rotatable - each gate's placement is given
 * directly instead. Single-gate parts with no explicit "x"/"y" fall
 * back to a naive auto-incrementing single row; give explicit "x"/"y" (or,
 * for a multi-gate part, "gates": {gateName: {x, y, rot}}) for anything
 * that needs to sit in a real hand-designed layout instead. The SCR syntax
 * (ADD/INVOKE argument order, absolute library paths, device-suffix
 * concatenation) is confirmed against a live EAGLE 7.2.0 install - see
 * doc/eagle-agent.md §9. A part's "device" is normally picked automatically
 * (CURATED_DEVICE, or the deviceset's only/first device) - give an explicit
 * "device" in the netlist to pin an exact variant instead (this is what
 * eagle-dematerialize.js emits, so round-tripping a real .sch through
 * eagle-dematerialize.js -> eagle-materialize.js reproduces the same
 * device, not whatever pickDevice() would guess).
 */
import { Console } from 'console';
import { getOpt } from 'util';
import * as path from './lib/path.js';
import { EagleDocument } from './eagle.js';
import { ReadFile, WriteFile, ReadFd } from './io-helpers.js';

/* Fixed absolute path (not derived from this script's own location) - this
 * runs from arbitrary project directories (an-tronics, pictest, lc-meter)
 * AND from an installed location (/usr/local/bin, CMakeLists.txt), where
 * a path relative to the script's own dir would resolve wrong. Default is
 * the PIC/digital-oriented library set; override with --lib-dir for the
 * analog set (op-amps/OTAs/CMOS/RF) at /mnt/data/Projects/an-tronics/eagle/lbr
 * - see the eagle-circuit skill's "Analog designs" note. */
let LIB_DIR = '/mnt/data/Projects/pictest/eagle/lbr';
const SPACING = 20; // mm between parts on the placement row

/* R/L/C/CPOL: hand-curated horizontal default device per doc/eagle-agent.md
 * §1 "R/L/C/CPOL: a fixed curated set". Vertical orientation isn't wired
 * up in this first version - always places the horizontal device. */
const CURATED_DEVICE = {
  R: '0204/10',
  'R-H': '0204/10',
  L: '03/10',
  C: '050-025X075',
  CPOL: 'E2.5-5',
};

const openDocs = new Map();
function openLibrary(name) {
  let doc = openDocs.get(name);
  if(!doc) {
    doc = EagleDocument.open(`${LIB_DIR}/${name}.lbr`, ReadFile);
    openDocs.set(name, doc);
  }
  return doc;
}

function pickDevice(deviceset, devicesetName, explicit) {
  if(explicit != null) return explicit;

  if(deviceset.getAttribute('name') in CURATED_DEVICE || CURATED_DEVICE[devicesetName])
    return CURATED_DEVICE[devicesetName];

  /* deviceset.devices is a NamedMap proxy (lib/dom.js) whose ownKeys trap
   * drops the empty-string key entirely (isPropertyKey('') is false) -
   * Reflect.ownKeys() therefore can never see a blank-name default
   * device. Direct index access (deviceset.devices['']) does reach a
   * device, but NamedMap's get trap treats '' as numeric (loose '' == 0)
   * and falls back to a *positional* index-0 lookup when there's no
   * device literally named '' - so it must always be name-checked, never
   * just tested for truthiness, or every deviceset's first device gets
   * misread as the blank-name default. */
  /* .name (DeviceElement's getAttribute('name')) comes back as an empty
   * array-like, not the string '', when the XML attribute's value itself
   * is "" - only String(...) coercion normalizes that back to ''. */
  const blank = deviceset.devices[''];
  if(blank && String(blank.name) === '') return '';

  const names = Reflect.ownKeys(deviceset.devices);
  console.log(`WARNING: '${devicesetName}' has ${names.length} devices, no default - picking '${names[0]}' arbitrarily`);
  return names[0];
}

function resolvePart(ref, spec) {
  const doc = openLibrary(spec.library);
  const deviceset = doc.library.devicesets[spec.deviceset];
  if(!deviceset) {
    const names = Reflect.ownKeys(doc.library.devicesets);
    throw new Error(`part '${ref}': no deviceset '${spec.deviceset}' in ${spec.library}.lbr. Did you mean one of: ${names.slice(0, 10).join(', ')}${names.length > 10 ? ', ...' : ''}?`);
  }
  const device = pickDevice(deviceset, spec.deviceset, spec.device);

  /* Multi-gate parts (spec.gates given): a real device with >1 schematic
   * gate that needs each gate placed at its own spot - e.g. PICSTICK's
   * JP1/JP2 headers, modeled as one EAGLE device with gates G$1/G$2. Each
   * entry is {x, y} for that gate's placement point. Pin names can repeat
   * across gates (PICSTICK's G$1 and G$2 both have pins "1".."14"), so net
   * members referencing these parts must be gate-qualified
   * ("<ref>.<gate>.<pin>") - see resolvePin(). */
  if(spec.gates) {
    const gates = {};
    for(const gateName of Object.keys(spec.gates)) {
      const gate = deviceset.gates[gateName];
      if(!gate) {
        const names = Reflect.ownKeys(deviceset.gates);
        throw new Error(`part '${ref}': no gate '${gateName}' on deviceset '${spec.deviceset}'. Real gates: ${names.join(', ')}`);
      }
      const gateRot = spec.gates[gateName].rot || 'R0';
      const given = spec.gates[gateName];
      const residue = gridResidue(gate.symbol.pins, (dx, dy) => ROTATE[gateRot](dx, dy));
      const placement = snapToPinGrid(given.x, given.y, residue);
      if(placement.x != given.x || placement.y != given.y)
        console.log(`NOTE: part '${ref}' gate '${gateName}' placement nudged from (${given.x} ${given.y}) to (${placement.x} ${placement.y}) mm - its library symbol's pins sit on a half-grid offset, so this keeps every pin's absolute position on the 2.54mm grid`);
      gates[gateName] = { placement, symbol: gate.symbol, rot: gateRot };
    }
    return { ref, spec, deviceset: spec.deviceset, device, gates };
  }

  const gateName = Reflect.ownKeys(deviceset.gates)[0];
  const gate = deviceset.gates[gateName];
  return { ref, spec, deviceset: spec.deviceset, device, symbol: gate.symbol, rot: spec.rot || 'R0' };
}

/* Rotates a symbol-local pin offset (x, y) by an EAGLE orientation string:
 * "R0"/"R90"/"R180"/"R270" (counter-clockwise rotation), plus their
 * mirrored counterparts "MR0"/"MR90"/"MR180"/"MR270" - EAGLE mirrors about
 * the vertical (Y) axis first (x -> -x), then applies the rotation. Used
 * e.g. to mirror the PICSTICK header gates so their pins face outward
 * toward the harness wiring instead of into the part's own body. */
/* MR90/MR270 confirmed against a real EAGLE-saved .sch (not derived from
 * theory): a part instance in MIDI-Harness.sch has rot="MR270" and a real
 * wire terminating exactly on one of its pins - computing that pin's
 * absolute position with the formula below (and only this one; the
 * "mirror-then-rotate" formula this replaced gives the same result
 * rotated 180 degrees, i.e. the OLD MR90/MR270 entries were swapped)
 * reproduces the real wire endpoint exactly. MR0/MR180 are unaffected -
 * both compositions agree for a 0 or 180 degree rotation. */
const ROTATE = {
  R0: (x, y) => ({ x, y }),
  R90: (x, y) => ({ x: -y, y: x }),
  R180: (x, y) => ({ x: -x, y: -y }),
  R270: (x, y) => ({ x: y, y: -x }),
  MR0: (x, y) => ({ x: -x, y }),
  MR90: (x, y) => ({ x: y, y: x }),
  MR180: (x, y) => ({ x, y: -y }),
  MR270: (x, y) => ({ x: -y, y: -x }),
};

const STUB_LEN = 2.54; // 0.1" - see resolveMember()'s "dir" comment

/* Cardinal (never diagonal) direction a pin's lead points away from its
 * gate/part origin, from that pin's local offset (dx, dy). A real EAGLE
 * pin lead is always drawn strictly horizontal or vertical - never
 * diagonal - even on multi-pin parts (connectors, ICs, LCD modules) whose
 * pins sit at a diagonal offset from the part's origin (e.g. a pin row at
 * local y=-19.05 with varying x: each individual lead still exits due
 * south, even though the origin-to-pin vector for a non-center pin is
 * diagonal). So the correct "outward" direction is the dominant axis of
 * (dx, dy), not the true unit vector of the diagonal itself - normalizing
 * the diagonal (an earlier version of this function did, via Math.sqrt)
 * produces an irrational offset for any non-2-terminal part, which is
 * exactly what put a stub's coordinate visibly off the 2.54mm grid (found
 * live: a GND net's LCD1-side stub landed at a non-grid-multiple point).
 * Falls back to +X for the degenerate case of a pin placed exactly on its
 * own origin (offset (0,0)), which no real symbol does. */
function pinDir(dx, dy) {
  if(dx == 0 && dy == 0) return { dx: 1, dy: 0 };
  return Math.abs(dx) >= Math.abs(dy) ? { dx: Math.sign(dx) || 1, dy: 0 } : { dx: 0, dy: Math.sign(dy) };
}

const GRID = 254; // 2.54mm grid step, in integer hundredths-of-a-mm
const toCents = mm => Math.round(mm * 100);
const fromCents = c => c / 100;

/* Real EAGLE library symbols sometimes place a part's pins on the finer
 * 1.27mm half-grid rather than the 2.54mm sheet grid - not a bug, a real
 * footprint constraint (e.g. the NOKIA-5510 LCD module's pin row sits at
 * local y=-19.05mm = 7.5 x 2.54mm). If every one of a part's pins shares
 * the same offset-from-2.54-grid ("residue") on a given axis, shifting the
 * part's own placement point by that same residue brings every pin's
 * ABSOLUTE position exactly onto the 2.54mm grid (half-grid pin offset +
 * half-grid origin offset = whole-grid absolute position) - confirmed
 * live: this is what makes a wire terminating on such a pin show a clean
 * grid coordinate in EAGLE's properties dialog instead of an arbitrary
 * fraction. Returns null for an axis where the part's own pins disagree
 * (no single placement could satisfy all of them) - the caller then
 * leaves that axis un-adjusted rather than silently picking a value that
 * only works for some of the part's pins. Works in integer hundredths-of-
 * a-mm throughout specifically to avoid the float/mod interaction that
 * caused the pinDir bug above - 2.54 itself isn't exactly representable
 * in binary floating point, so `x % 2.54` on real mm values is exactly
 * the kind of computation that silently drifts. */
function gridResidue(pins, rotFn) {
  let rx = null,
    ry = null;
  for(const name of Reflect.ownKeys(pins)) {
    const p = pins[name];
    const r = rotFn(p.x, p.y);
    const cx = ((toCents(r.x) % GRID) + GRID) % GRID;
    const cy = ((toCents(r.y) % GRID) + GRID) % GRID;
    rx = rx == null ? cx : rx == cx ? rx : -1;
    ry = ry == null ? cy : ry == cy ? ry : -1;
  }
  return { rx: rx === -1 ? null : rx, ry: ry === -1 ? null : ry };
}

/* Nudges a part/gate placement point (mm) to the nearest position (0.01mm
 * steps) whose residue mod 2.54mm matches the pins' own required residue
 * per gridResidue() above, on each axis independently. */
function snapToPinGrid(x, y, residue) {
  const snapAxis = (v, r) => {
    if(r == null) return v;
    const c = toCents(v);
    const diff = (((c - r) % GRID) + GRID) % GRID;
    return fromCents(diff <= GRID / 2 ? c - diff : c + (GRID - diff));
  };
  return { x: snapAxis(x, residue.rx), y: snapAxis(y, residue.ry) };
}

function resolveMember(member, parts) {
  /* A net member is normally "<ref>.<pin>" (or "<ref>.<gate>.<pin>" for a
   * multi-gate part). It can also be a bare {x, y} waypoint - no part/pin,
   * just a routing point - to bend a wire around another wire it would
   * otherwise cross. EAGLE flags ANY two differently-named nets' wires
   * that geometrically cross (not just ones that share an endpoint) with
   * a "Merge net segment ... into given net ...?" dialog - confirmed
   * against a live install - and neither answer is safe to script past:
   * "Yes" electrically shorts the two nets together; "No" doesn't keep
   * both wires "crossing but unconnected" the way some other schematic
   * tools would - it silently drops the new segment instead, leaving a
   * real missing connection. So actual crossings have to be routed around
   * with waypoints, not auto-answered through. */
  if(typeof member == 'object') return { x: member.x, y: member.y };

  const firstDot = member.indexOf('.');
  if(firstDot < 0) throw new Error(`net member '${member}' is not '<part-ref>.<pin-name>' or '<part-ref>.<gate>.<pin-name>'`);
  const ref = member.slice(0, firstDot);
  const part = parts[ref];
  if(!part) throw new Error(`net member '${member}': no part '${ref}' declared in "parts"`);
  const rest = member.slice(firstDot + 1);

  if(part.gates) {
    const dot = rest.indexOf('.');
    if(dot < 0) throw new Error(`net member '${member}': '${ref}' is a multi-gate part (${Object.keys(part.gates).join(', ')}) - reference it as '${ref}.<gate>.<pin>'`);
    const gateName = rest.slice(0, dot);
    const pinName = rest.slice(dot + 1);
    const gate = part.gates[gateName];
    if(!gate) throw new Error(`net member '${member}': no gate '${gateName}' placed on '${ref}'. Placed gates: ${Object.keys(part.gates).join(', ')}`);
    const pin = gate.symbol.pins[pinName];
    if(!pin) {
      const names = Reflect.ownKeys(gate.symbol.pins);
      throw new Error(`net member '${member}': no pin '${pinName}' on '${ref}' gate '${gateName}'. Real pin names: ${names.join(', ')}`);
    }
    const rotated = ROTATE[gate.rot](pin.x, pin.y);
    return { x: gate.placement.x + rotated.x, y: gate.placement.y + rotated.y, dir: pinDir(rotated.x, rotated.y) };
  }

  const pinName = rest;
  const pin = part.symbol.pins[pinName];
  if(!pin) {
    const names = Reflect.ownKeys(part.symbol.pins);
    throw new Error(`net member '${member}': no pin '${pinName}' on '${ref}' (${part.deviceset}). Real pin names: ${names.join(', ')}`);
  }
  const rotated = ROTATE[part.rot](pin.x, pin.y);
  return { x: part.placement.x + rotated.x, y: part.placement.y + rotated.y, dir: pinDir(rotated.x, rotated.y) };
}

function materialize(netlist) {
  const parts = {};
  let x = 0;
  for(const [ref, spec] of Object.entries(netlist.parts)) {
    const part = resolvePart(ref, spec);
    /* Placement: explicit spec.x/spec.y (or per-gate spec.gates.*.x/y,
     * already baked into part.gates by resolvePart) wins when given - a
     * shared-center layout (multiple sub-circuits arranged around one
     * fixed part) needs manual placement everywhere, not just a single
     * auto row. Falls back to the naive auto-incrementing row (unchanged
     * default) only for single-gate parts with no explicit position. */
    if(!part.gates) {
      const given = spec.x != null && spec.y != null ? { x: spec.x, y: spec.y } : { x, y: 0 };
      const residue = gridResidue(part.symbol.pins, (dx, dy) => ROTATE[part.rot](dx, dy));
      part.placement = snapToPinGrid(given.x, given.y, residue);
      if(part.placement.x != given.x || part.placement.y != given.y)
        console.log(`NOTE: part '${ref}' placement nudged from (${given.x} ${given.y}) to (${part.placement.x} ${part.placement.y}) mm - its library symbol's pins sit on a half-grid offset, so this keeps every pin's absolute position on the 2.54mm grid`);
    }
    parts[ref] = part;
    if(spec.x == null || spec.y == null) x += SPACING;
  }

  /* SET WIRE_BEND 2 = "starting point - end (straight connection)" -
   * confirmed in the EAGLE 7.2.0 help reference (SET WIRE_BEND bend_nr).
   * Without this, EAGLE auto-routes every NET command's 2 points as an
   * L-shaped Manhattan path (horizontal at the start point's Y, then
   * vertical at the end point's X) - confirmed against a live install -
   * which is what caused this session's whole class of "Merge net
   * segment" collisions: dense layouts put multiple signals' Manhattan
   * legs on the same row/column. A direct point-to-point wire mostly
   * avoids that by construction (two unrelated wires sharing an exact
   * line is far rarer than sharing a row/column), at the cost of the
   * schematic being visually messier - acceptable since electrical
   * correctness, not routing aesthetics, is what a synthesized-then-
   * verified schematic needs. Pins that are already aligned (same part
   * placement x or y, e.g. a resistor stacked directly above its LED)
   * still draw a plain straight horizontal/vertical line under bend
   * style 2, same as before - only genuinely offset pins go diagonal. */
  const lines = ['GRID MM;', 'SET WIRE_BEND 2;'];
  for(const [ref, part] of Object.entries(parts)) {
    /* Real EAGLE ADD syntax, confirmed against a live EAGLE 7.2.0 install
     * (doc/eagle-agent.md §2c "headless verification"), is
     * `ADD device_name[@library_name] [name] [orientation] (x y);` - the
     * device/library token comes FIRST, the (quoted) instance name SECOND -
     * the reverse of what earlier versions of this line emitted. Two more
     * real findings baked in here:
     * - `device_name` must be the deviceset name with its specific device's
     *   name suffix appended directly (no separator) whenever the deviceset
     *   has no blank-name/default device - e.g. `R0204/10`, not `R` alone -
     *   that's exactly what `part.device` (resolved by pickDevice() above)
     *   is for, so it's now used here instead of being discarded.
     * - `@library_name` must be a full absolute path to the .lbr file, not
     *   a bare library short name - a bare name (even after `USE`) produced
     *   "Device not found" in live testing. */
    const deviceToken = `${part.deviceset}${part.device}@${LIB_DIR}/${part.spec.library}.lbr`;
    if(part.gates) {
      /* Multi-gate placement, confirmed against a live EAGLE 7.2.0 install:
       * `ADD` places the device's FIRST gate and creates the part instance;
       * every subsequent gate of that same (already-added) part needs
       * `INVOKE part_name gate_name orientation (x y);` instead - a second
       * `ADD` with the same instance name is rejected ("Part name '...'
       * already exists!"). */
      const gateNames = Object.keys(part.gates);
      for(let i = 0; i < gateNames.length; i++) {
        const gateName = gateNames[i];
        const { x: px, y: py } = part.gates[gateName].placement;
        const gateRot = part.gates[gateName].rot;
        lines.push(i == 0 ? `ADD ${deviceToken} '${ref}' ${gateName} ${gateRot} (${px} ${py});` : `INVOKE ${ref} ${gateName} ${gateRot} (${px} ${py});`);
      }
    } else {
      const { x: px, y: py } = part.placement;
      lines.push(`ADD ${deviceToken} '${ref}' ${part.rot} (${px} ${py});`);
    }
    /* VALUE name value; - unquoted, per the confirmed EAGLE help reference
     * (doc/eagle-agent.md Sources) - its own example is unquoted even
     * though ADD/NET quote their name arguments. */
    if(part.spec.value) lines.push(`VALUE ${ref} ${part.spec.value};`);
  }
  /* Two consecutive members landing on the exact same coordinate (e.g. a
   * part placed pin-to-pin touching the next) still connects them - pins at
   * the same point are the same node regardless of a drawn wire - but
   * emitting a zero-length NET line for it is degenerate: confirmed against
   * a live install, EAGLE doesn't treat it as "this net, zero length" but
   * instead creates a stray anonymous net (`N$n`) and then asks to merge
   * that into the real net name. Skip lines whose endpoints coincide. */
  const sameSpot = (a, b) => Math.abs(a.x - b.x) < 1e-6 && Math.abs(a.y - b.y) < 1e-6;

  for(const [netName, members] of Object.entries(netlist.nets)) {
    if(members.length < 2) throw new Error(`net '${netName}' has fewer than 2 members`);
    const points = members.map(m => resolveMember(m, parts));
    const n = points.length;

    /* A net chain's interior members (every member but the first and last)
     * have two wires meeting exactly at their point - if that point is a
     * real component pin, EAGLE draws the junction dot pinned to the pin
     * itself, which can't be dragged in the schematic editor afterward.
     * Chain endpoints don't have this problem (only one wire touches
     * them), and bare {x,y} waypoints have no "pin" to be pinned to, so
     * only interior members with a resolved pin direction get an anchor:
     * a short stub wire (STUB_LEN) extending the pin's own lead direction,
     * with the two chain wires meeting at the stub's far end instead of at
     * the pin - now a plain wire-to-wire junction, which IS draggable. */
    const anchors = points.map((p, i) => (i > 0 && i < n - 1 && p.dir ? { x: p.x + p.dir.dx * STUB_LEN, y: p.y + p.dir.dy * STUB_LEN } : { x: p.x, y: p.y }));

    for(let i = 1; i < n - 1; i++) {
      if(!points[i].dir) continue;
      const a = points[i], b = anchors[i];
      if(sameSpot(a, b)) continue;
      lines.push(`NET '${netName}' (${a.x} ${a.y}) (${b.x} ${b.y});`);
    }
    for(let i = 0; i < n - 1; i++) {
      const a = anchors[i], b = anchors[i + 1];
      if(sameSpot(a, b)) continue;
      lines.push(`NET '${netName}' (${a.x} ${a.y}) (${b.x} ${b.y});`);
    }
  }
  return lines.join('\n') + '\n';
}

/* Merges an eagle-tool.js `positions` dump (see that file's header
 * comment: {ref: {x,y,rot?}} or {ref: {gates: {gateName: {x,y,rot?}}}},
 * the exact shape of this netlist's own "parts.<ref>" placement fields)
 * into `netlist.parts`, in place - OVERRIDING whatever x/y/rot/gates the
 * netlist itself already gave that part. This is the point: it lets a
 * hand-tuned layout saved from a real .sch (via `eagle-tool.js positions`)
 * take priority over the netlist's own naive/placeholder coordinates,
 * without needing a separate MOVE+ROTATE follow-up script the way
 * eagle-tool.js's sch-from-sch works. A ref present in `positions` but
 * not in this netlist (a part since removed) or vice versa (a genuinely
 * new part with no old position to restore) is reported, not silently
 * dropped - matches eagle-tool.js's own NOTE-style diagnostics. A shape
 * mismatch (single-gate position given for a part this netlist declares
 * multi-gate, or vice versa) is reported and skipped rather than
 * corrupting the spec. */
function applyPositions(netlist, positions) {
  const missing = [],
    extra = [],
    mismatched = [];
  for(const ref of Reflect.ownKeys(positions)) {
    const spec = netlist.parts[ref];
    if(!spec) {
      extra.push(ref);
      continue;
    }
    const pos = positions[ref];
    const specIsMultiGate = !!spec.gates;
    const posIsMultiGate = !!pos.gates;
    if(specIsMultiGate != posIsMultiGate) {
      mismatched.push(ref);
      continue;
    }
    if(posIsMultiGate) {
      for(const gateName of Reflect.ownKeys(pos.gates)) {
        spec.gates[gateName] = { ...spec.gates[gateName], ...pos.gates[gateName] };
      }
    } else {
      spec.x = pos.x;
      spec.y = pos.y;
      if(pos.rot != null) spec.rot = pos.rot;
    }
  }
  for(const ref of Reflect.ownKeys(netlist.parts)) if(!(ref in positions)) missing.push(ref);

  if(missing.length) console.log(`NOTE: no saved position for these parts (using the netlist's own placement): ${missing.join(', ')}`);
  if(extra.length) console.log(`NOTE: saved positions for parts not in this netlist (ignored): ${extra.join(', ')}`);
  if(mismatched.length) console.log(`NOTE: saved position's single-gate/multi-gate shape doesn't match this netlist's part (ignored): ${mismatched.join(', ')}`);
}

function main(...args) {
  globalThis.console = new Console({ inspectOptions: { colors: true, depth: Infinity } });

  const params = getOpt(
    {
      output: [true, null, 'o'],
      positions: [true, null],
      'lib-dir': [true, null],
      '@': 'input',
    },
    args,
  );

  if(params['lib-dir']) LIB_DIR = params['lib-dir'];

  const file = (params['@'] || args)[0];
  if(!file) {
    console.log('usage: eagle-materialize.js <netlist.json> [-o output.scr] [--positions <file.json|->] [--lib-dir <path>]');
    return;
  }

  const netlist = JSON.parse(ReadFile(file));

  if(params.positions) {
    /* eagle-tool.js's `positions` command can also write YAML (-o
     * out.yaml), but the yaml module (doc/native/yaml.md) is write-only -
     * there's no read() to parse YAML back in, so only JSON is accepted
     * here regardless of the source file's extension. */
    const text = params.positions === '-' ? ReadFd(0) : ReadFile(params.positions);
    applyPositions(netlist, JSON.parse(text));
  }

  const scr = materialize(netlist);

  if(params.output) {
    WriteFile(params.output, scr);
    console.log(`wrote '${params.output}'`);
  } else {
    console.log(scr);
  }
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error.message}\n${error.stack}`);
}
