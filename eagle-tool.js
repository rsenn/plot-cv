#!/usr/bin/env qjsm
/*
 * eagle-agent: non-interactive position save/restore for EAGLE .sch/.brd
 * files - emits a MOVE(+ROTATE) .scr script that reproduces one
 * document's part/element positions when run against another. This is
 * the tool eagle-materialize.js's regenerate-then-lose-hand-tweaked-
 * layout problem needs: fine-tune a schematic's placement by hand in
 * EAGLE, then next time the netlist changes and eagle-materialize.js
 * regenerates the .scr from scratch, use `sch-from-sch` to carry the old
 * layout over onto the new one instead of starting from its naive
 * auto-placement again.
 *
 * Two real EAGLE scripting findings this relies on, both confirmed
 * against a live EAGLE 7.2.0 install (not assumed from general SPICE/
 * EAGLE folklore):
 * - `MOVE 'name' (x y);` (by part name) only works for a single-instance
 *   part. For a multi-gate part (e.g. PICSTICK's G$1/G$2 header gates,
 *   each its own <instance> sharing one part name) it errors "Unknown
 *   part: <name>" even though the part genuinely exists and ADD/INVOKE
 *   placed it fine moments earlier in the same script - EAGLE's MOVE
 *   apparently only resolves a bare name to a single-instance part.
 * - The real fix is coordinate-based selection instead of by-name:
 *   `MOVE (oldx oldy) (newx newy);` selects whatever instance currently
 *   sits at (oldx, oldy) - works identically for single- and multi-gate
 *   parts, so every command this tool emits uses that form exclusively
 *   (never the by-name form) to avoid the special case entirely.
 *   `ROTATE =<rot> (x y);` (coordinate-based, `=` for absolute rather
 *   than relative rotation) works the same way, selecting by the part's
 *   CURRENT (post-MOVE) position.
 *
 * Board elements have no gate concept (one physical footprint per part
 * regardless of how many schematic gates that part has), so board-side
 * lookups are keyed by part name alone; schematic-side lookups are keyed
 * "<part>.<gate>" to address each gate instance independently.
 */
import { Console } from 'console';
import { getOpt } from 'util';
import { EagleDocument } from './eagle.js';
import { ReadFile, WriteFile } from './io-helpers.js';
import { write as writeYAML } from 'yaml';

const toCents = mm => Math.round(mm * 100); // integer hundredths-of-mm - avoids float-equality bugs when comparing positions (see eagle-materialize.js's gridResidue() for the same reasoning)

/* Every schematic instance across all sheets, as a flat list - the shared
 * source both schInstances() (below, keyed for position-transfer lookups)
 * and positionsSpec() (this file's "positions" command, grouped back into
 * eagle-materialize.js's per-part netlist shape) build their own view of. */
function schRawInstances(doc) {
  if(doc.type != 'schematic') throw new Error(`'${doc.filename}' is a ${doc.type}, not a schematic`);
  const raw = [];
  for(const sheet of doc.schematic.sheets.children) {
    for(const inst of sheet.instances.children) {
      raw.push({
        part: inst.getAttribute('part'),
        gate: inst.getAttribute('gate'),
        x: +inst.getAttribute('x'),
        y: +inst.getAttribute('y'),
        rot: (inst.getAttribute('rot') || 'R0').replace(/^S/, ''), // 'S' prefix = "spun" text flag, not a real orientation
      });
    }
  }
  return raw;
}

/* Every schematic instance across all sheets, keyed by part name alone
 * for a single-instance part, or "<part>.<gate>" for a part with more
 * than one gate instance (matching eagle-dematerialize.js's own
 * isMultiGate convention) - only a genuinely multi-gate part needs gate-
 * qualification to disambiguate. A single-gate part's gate NAME is a
 * library implementation detail (e.g. the same physical diode symbol can
 * be "G$1" under one deviceset and "1" under another) that's irrelevant
 * to matching up that part's position between two documents - keying on
 * it too would spuriously treat the same part as unmatched any time its
 * deviceset changed, even though its position is still exactly what the
 * caller wants to carry over. */
function schInstances(doc) {
  const raw = schRawInstances(doc);
  const countByPart = new Map();
  for(const inst of raw) countByPart.set(inst.part, (countByPart.get(inst.part) || 0) + 1);
  const out = new Map();
  for(const inst of raw) out.set(countByPart.get(inst.part) > 1 ? `${inst.part}.${inst.gate}` : inst.part, inst);
  return out;
}

/* One entry per schematic part, in EXACTLY the shape eagle-materialize.js's
 * netlist "parts.<ref>" spec expects for placement - {x, y, rot?} for a
 * single-instance part, {gates: {<gateName>: {x, y, rot?}, ...}} for a
 * multi-gate one (mirrors eagle-dematerialize.js's own partsOut[ref]
 * convention exactly, minus the library/deviceset/device/value fields,
 * since this is meant to be merged into an existing circuit.json's own
 * part specs via eagle-materialize.js's --positions flag, not stand alone
 * as a full netlist). `rot` is omitted when it's the default R0. */
function positionsSpec(file) {
  const doc = EagleDocument.open(file, ReadFile);
  const byPart = new Map();
  for(const inst of schRawInstances(doc)) {
    if(!byPart.has(inst.part)) byPart.set(inst.part, []);
    byPart.get(inst.part).push(inst);
  }
  const out = {};
  for(const [part, insts] of byPart) {
    if(insts.length > 1) {
      const gates = {};
      for(const inst of insts) {
        gates[inst.gate] = { x: inst.x, y: inst.y };
        if(inst.rot != 'R0') gates[inst.gate].rot = inst.rot;
      }
      out[part] = { gates };
    } else {
      const inst = insts[0];
      out[part] = { x: inst.x, y: inst.y };
      if(inst.rot != 'R0') out[part].rot = inst.rot;
    }
  }
  return out;
}

/* Every board element, keyed by part name - see header comment on why
 * boards have no per-gate concept. */
function boardElements(doc) {
  if(doc.type != 'board') throw new Error(`'${doc.filename}' is a ${doc.type}, not a board`);
  const out = new Map();
  for(const name of Reflect.ownKeys(doc.board.elements)) {
    const el = doc.board.elements[name];
    out.set(name, {
      part: name,
      x: +el.getAttribute('x'),
      y: +el.getAttribute('y'),
      rot: (el.getAttribute('rot') || 'R0').replace(/^S/, ''),
    });
  }
  return out;
}

/* Collapses a schematic's per-gate instances down to one representative
 * position per part, for correlating against a board's one-element-per-
 * part model. A multi-gate part has no single "right" board position by
 * construction - this picks its alphabetically-first gate and reports
 * the part in `ambiguous` so the caller can warn about it, rather than
 * silently guessing something that might be wrong for that part's real
 * physical footprint. */
function schPositionsByPart(doc) {
  const byPart = new Map();
  for(const inst of schInstances(doc).values()) {
    if(!byPart.has(inst.part)) byPart.set(inst.part, []);
    byPart.get(inst.part).push(inst);
  }
  const positions = new Map();
  const ambiguous = [];
  for(const [part, insts] of byPart) {
    insts.sort((a, b) => (a.gate < b.gate ? -1 : a.gate > b.gate ? 1 : 0));
    if(insts.length > 1) ambiguous.push(part);
    positions.set(part, insts[0]);
  }
  return { positions, ambiguous };
}

/* Correlates a `target` position map (the document about to be modified -
 * its CURRENT positions are the MOVE/ROTATE selector) against a `source`
 * map (the desired final positions) by their shared keys, producing one
 * {key, oldX, oldY, oldRot, newX, newY, newRot} pair per target entry that
 * has a matching source entry. Keys present in only one side are reported
 * separately rather than silently dropped. */
function correlate(source, target) {
  const pairs = [];
  const missingInSource = [];
  for(const [key, t] of target) {
    const s = source.get(key);
    if(!s) {
      missingInSource.push(key);
      continue;
    }
    pairs.push({ key, oldX: t.x, oldY: t.y, oldRot: t.rot, newX: s.x, newY: s.y, newRot: s.rot });
  }
  const targetKeys = new Set(target.keys());
  const extraInSource = [...source.keys()].filter(k => !targetKeys.has(k));
  return { pairs, missingInSource, extraInSource };
}

function schFromSch(sourceFile, targetFile) {
  const source = schInstances(EagleDocument.open(sourceFile, ReadFile));
  const target = schInstances(EagleDocument.open(targetFile, ReadFile));
  return correlate(source, target);
}

function boardFromSchematic(sourceFile, targetFile) {
  const { positions: source, ambiguous } = schPositionsByPart(EagleDocument.open(sourceFile, ReadFile));
  const target = boardElements(EagleDocument.open(targetFile, ReadFile));
  return { ...correlate(source, target), ambiguous };
}

function boardFromBoard(sourceFile, targetFile) {
  const source = boardElements(EagleDocument.open(sourceFile, ReadFile));
  const target = boardElements(EagleDocument.open(targetFile, ReadFile));
  return correlate(source, target);
}

function snapGrid(file, gridMM) {
  const doc = EagleDocument.open(file, ReadFile);
  const instances = doc.type == 'schematic' ? schInstances(doc) : boardElements(doc);
  const snap = v => Math.round(v / gridMM) * gridMM;
  const pairs = [...instances.entries()].map(([key, inst]) => ({
    key,
    oldX: inst.x,
    oldY: inst.y,
    oldRot: inst.rot,
    newX: snap(inst.x),
    newY: snap(inst.y),
    newRot: inst.rot, // rotation untouched - only position is snapped
  }));
  return { pairs, missingInSource: [], extraInSource: [] };
}

/* Emits MOVE (and, where the rotation actually changed, ROTATE) lines for
 * every pair whose position or rotation differs - coordinate-selected
 * throughout (see header comment for why by-name MOVE isn't used). A
 * pair with no real change is skipped rather than emitting a no-op MOVE
 * to the same coordinate. */
function moveRotateScript(pairs) {
  const lines = ['GRID MM;'];
  let moved = 0,
    rotated = 0;
  for(const p of pairs) {
    const posChanged = toCents(p.oldX) != toCents(p.newX) || toCents(p.oldY) != toCents(p.newY);
    const rotChanged = p.newRot != p.oldRot;
    if(!posChanged && !rotChanged) continue;
    if(posChanged) {
      lines.push(`MOVE (${p.oldX} ${p.oldY}) (${p.newX} ${p.newY});`);
      moved++;
    }
    if(rotChanged) {
      lines.push(`ROTATE =${p.newRot} (${p.newX} ${p.newY});`);
      rotated++;
    }
  }
  return { script: lines.join('\n') + '\n', moved, rotated };
}

/* Every part/element name declared in a document, regardless of type -
 * schematic parts and board elements are both just "the set of reference
 * designators this document knows about" for diff-parts' purposes. */
function docPartNames(doc) {
  if(doc.type == 'schematic') return new Set(Reflect.ownKeys(doc.schematic.parts));
  if(doc.type == 'board') return new Set(Reflect.ownKeys(doc.board.elements));
  throw new Error(`'${doc.filename}' is a ${doc.type}, not a schematic or board`);
}

/* Bidirectional part-name diff between any two documents (schematic or
 * board, need not match) - a standalone consistency check, unlike
 * sch-from-sch/board-from-*'s missingInSource/extraInSource (which are a
 * side effect of computing a position transfer, not something you can
 * ask for on their own, and only ever compare like-with-like). */
function diffParts(fileA, fileB) {
  const a = docPartNames(EagleDocument.open(fileA, ReadFile));
  const b = docPartNames(EagleDocument.open(fileB, ReadFile));
  return {
    onlyInA: [...a].filter(n => !b.has(n)).sort(),
    onlyInB: [...b].filter(n => !a.has(n)).sort(),
    common: [...a].filter(n => b.has(n)).sort(),
  };
}

/* One row per schematic part: name/deviceset/device/value, straight from
 * the real .sch - catches a wrong device pick or forgotten value without
 * eyeballing raw XML or re-reading the .circuit.json that generated it
 * (which may not even be what's really in the file, if it was hand-edited
 * since). */
function bom(file) {
  const doc = EagleDocument.open(file, ReadFile);
  if(doc.type != 'schematic') throw new Error(`'${doc.filename}' is a ${doc.type}, not a schematic`);
  const parts = doc.schematic.parts;
  return Reflect.ownKeys(parts)
    .map(ref => {
      const part = parts[ref];
      return {
        name: ref,
        deviceset: part.getAttribute('deviceset'),
        device: part.getAttribute('device') || '',
        value: part.getAttribute('value') || '',
      };
    })
    .sort((a, b) => (a.name < b.name ? -1 : a.name > b.name ? 1 : 0));
}

/* A "bus" (harness) in circuit.json is descriptive metadata only - it does
 * NOT cause eagle-materialize.js to draw an EAGLE BUS wire (real BUS
 * objects add geometric complexity with no payoff for a point-to-point-
 * wired schematic). It records a physical multi-conductor interconnect -
 * a ribbon cable, a pin-header breakout - as it appears on the
 * CONTROLLER side (the MCU/eval-board/IC part), never on the destination
 * peripheral's own pinout (that's already fully described by the
 * netlist's own parts/nets - see SKILL.md's "Harnesses and pin-runs"
 * section for the full reasoning). One harness can bundle more than one
 * contiguous pin-run - e.g. a MIDI harness tying together RC6/RC7 on one
 * connector/gate with RB4-RB7 on another - so `runs` is a list, each with
 * its own `connector` and its own contiguous `pins` list. A pin slot with
 * no `net` (reserved, tied off, or simply not wired yet) still holds its
 * place, so a run stays a true physical picture of the interconnect even
 * where a conductor carries no signal today. */
function loadNetlist(file) {
  return JSON.parse(ReadFile(file));
}

function busMemberKey(connector, pin) {
  return connector.gate ? `${connector.part}.${connector.gate}.${pin}` : `${connector.part}.${pin}`;
}

function busNames(netlist) {
  return Reflect.ownKeys(netlist.buses || {});
}

function getBus(netlist, name) {
  const bus = netlist.buses?.[name];
  if(!bus) throw new Error(`no such bus '${name}' (known: ${busNames(netlist).join(', ') || 'none'})`);
  return bus;
}

/* Checks the two things harness metadata can drift out of sync with,
 * independently per run: each run's pin list must be a contiguous range
 * (that's the whole point of a pin-run - a "run" with gaps/duplicates
 * isn't one; contiguity is NOT required across different runs of the
 * same harness, since those are genuinely separate physical pin-runs
 * bundled together), and every pin that claims a net must actually
 * appear as that net's member at the claimed connector+pin in the
 * netlist's own `nets` section. */
function validateBus(netlist, name) {
  const bus = getBus(netlist, name);
  const problems = [];
  for(const run of bus.runs) {
    const label = run.connector.gate ? `${run.connector.part}.${run.connector.gate}` : run.connector.part;
    const sorted = [...run.pins].map(p => p.pin).sort((a, b) => a - b);
    for(let i = 1; i < sorted.length; i++) {
      if(sorted[i] == sorted[i - 1]) problems.push(`${label}: duplicate pin ${sorted[i]}`);
      else if(sorted[i] != sorted[i - 1] + 1) problems.push(`${label}: gap in pin range: ${sorted[i - 1]} -> ${sorted[i]} (not contiguous)`);
    }
    for(const p of run.pins) {
      if(p.net == null) continue;
      const members = netlist.nets?.[p.net];
      if(!members) {
        problems.push(`${label} pin ${p.pin}: net '${p.net}' not found in nets`);
        continue;
      }
      const expected = busMemberKey(run.connector, p.pin);
      if(!members.includes(expected)) problems.push(`${label} pin ${p.pin}: net '${p.net}' has no member '${expected}' (harness claims this pin, netlist disagrees)`);
    }
  }
  return problems;
}

function busRunTable(run) {
  return run.pins.map(p => ({ pin: p.pin, net: p.net ?? '', note: p.note ?? '' }));
}

function printTable(rows, columns) {
  const widths = columns.map((c, i) => Math.max(c.length, ...rows.map(r => String(r[c] ?? '').length)));
  const line = cells => columns.map((c, i) => cells[i].padEnd(widths[i])).join('  ');
  console.log(line(columns));
  for(const r of rows) console.log(line(columns.map(c => String(r[c] ?? ''))));
}

/* Real EAGLE 7.2.0 `PRINT` script syntax, confirmed against a live install:
 * this exact line renders every sheet of the CURRENTLY OPEN document to a
 * real PDF (the output path's extension alone selects PDF - no separate
 * format flag needed). Must be run by opening the file you want rendered
 * DIRECTLY (`eagle -N- -S<this-script> <existing-file>.sch`) - NOT through
 * eagle-verify.sh, which deletes its target file before opening it (right
 * for verifying a fresh .scr's output, wrong for rendering an existing
 * populated document you don't want touched). */
function renderScript(pdfPath) {
  return `PRINT landscape 0.8 -1 -0 -caption FILE '${pdfPath}' sheets all paper a4;\nQUIT;\n`;
}

function writeOrPrint(out, text) {
  if(out) {
    WriteFile(out, text);
    console.log(`wrote '${out}'`);
  } else {
    console.log(text);
  }
}

function usage() {
  console.log(
    [
      'usage: eagle-tool.js <command> [options] <arguments...>',
      '',
      'commands:',
      '  sch-from-sch <source.sch> <target.sch> [-o out.scr]',
      "      Emit a MOVE+ROTATE script (keyed by part+gate) that reproduces source.sch's",
      '      part/gate positions when run against target.sch.',
      '  board-from-schematic <source.sch> <target.brd> [-o out.scr]',
      "      Emit a MOVE+ROTATE script that arranges target.brd's elements to match",
      "      source.sch's part positions (one board element per part - a multi-gate part",
      "      uses its alphabetically-first gate's position).",
      '  board-from-board <source.brd> <target.brd> [-o out.scr]',
      "      Emit a MOVE+ROTATE script that reproduces source.brd's element positions",
      '      when run against target.brd.',
      '  snap-grid <file.sch|file.brd> [--grid <mm>] [-o out.scr]',
      '      Emit a MOVE script that snaps every part/gate (schematic) or element (board)',
      '      position onto a grid (default 1.27mm / half-grid). Rotation is left untouched.',
      '  diff-parts <a.sch|.brd> <b.sch|.brd>',
      '      Print the part/element names only in a, only in b, and common to both.',
      '  bom <file.sch>',
      '      Print a table of every part: name, deviceset, device, value.',
      '  render <file.sch|.brd> --pdf <output.pdf> [-o out.scr]',
      '      Emit a PRINT script that renders file to a real PDF when run against it',
      '      directly (see the script header comment - NOT via eagle-verify.sh).',
      '  positions <file.sch> [-o out.json|out.yaml]',
      "      Dump every part's x/y/rot (or a gates map, for a multi-gate part) as JSON",
      '      (YAML if -o ends .yaml/.yml) in eagle-materialize.js netlist part-spec shape.',
      '      Pipe into `eagle-materialize.js --positions -` to carry an old layout onto a',
      '      freshly regenerated .scr without a separate MOVE+ROTATE follow-up script.',
      '  buses <circuit.json> [<bus-name>] [--validate] [-o out.json|out.yaml]',
      "      Print a circuit.json's declared harnesses (\"buses\") - one or more",
      '      controller-side pin-runs each - as pin/net/note tables (all harnesses if no',
      '      name given). --validate checks each run is a contiguous pin range and that',
      '      every pin\'s claimed net really has that pin as a member in the netlist\'s',
      '      own `nets` section. With -o, writes the raw bus data (JSON, or YAML if the',
      '      path ends .yaml/.yml) instead of printing tables - a generated, always-',
      '      regenerable view, never a second hand-edited copy of the netlist.',
    ].join('\n'),
  );
}

function main(...args) {
  globalThis.console = new Console({ inspectOptions: { colors: true, depth: Infinity } });

  const command = args[0];
  if(!command) return usage();

  const params = getOpt({ output: [true, null, 'o'], grid: [true, null], pdf: [true, null], validate: [false, false], '@': 'args' }, args.slice(1));
  const rest = params['@'] || [];

  if(['sch-from-sch', 'board-from-schematic', 'board-from-board', 'snap-grid'].includes(command)) {
    let result;
    switch (command) {
      case 'sch-from-sch':
        if(rest.length < 2) return usage();
        result = schFromSch(rest[0], rest[1]);
        break;
      case 'board-from-schematic':
        if(rest.length < 2) return usage();
        result = boardFromSchematic(rest[0], rest[1]);
        break;
      case 'board-from-board':
        if(rest.length < 2) return usage();
        result = boardFromBoard(rest[0], rest[1]);
        break;
      case 'snap-grid':
        if(rest.length < 1) return usage();
        result = snapGrid(rest[0], params.grid != null ? +params.grid : 1.27);
        break;
    }

    if(result.ambiguous?.length) console.log(`NOTE: multi-gate parts in source (used their first gate's position as the board representative): ${result.ambiguous.join(', ')}`);
    if(result.missingInSource.length) console.log(`NOTE: no source position for these target keys (left unchanged): ${result.missingInSource.join(', ')}`);
    if(result.extraInSource.length) console.log(`NOTE: source has positions with no matching target key (ignored): ${result.extraInSource.join(', ')}`);

    const { script, moved, rotated } = moveRotateScript(result.pairs);
    console.log(`${moved} move(s), ${rotated} rotation(s)`);
    writeOrPrint(params.output, script);
    return;
  }

  switch (command) {
    case 'diff-parts': {
      if(rest.length < 2) return usage();
      const { onlyInA, onlyInB, common } = diffParts(rest[0], rest[1]);
      console.log(`${common.length} common part(s)`);
      console.log(onlyInA.length ? `only in '${rest[0]}': ${onlyInA.join(', ')}` : `nothing only in '${rest[0]}'`);
      console.log(onlyInB.length ? `only in '${rest[1]}': ${onlyInB.join(', ')}` : `nothing only in '${rest[1]}'`);
      break;
    }
    case 'bom': {
      if(rest.length < 1) return usage();
      printTable(bom(rest[0]), ['name', 'deviceset', 'device', 'value']);
      break;
    }
    case 'render': {
      if(rest.length < 1 || !params.pdf) return usage();
      writeOrPrint(params.output, renderScript(params.pdf));
      break;
    }
    case 'positions': {
      if(rest.length < 1) return usage();
      const data = positionsSpec(rest[0]);
      const asYAML = params.output && /\.ya?ml$/i.test(params.output);
      writeOrPrint(params.output, asYAML ? writeYAML(data) : JSON.stringify(data, null, 2) + '\n');
      break;
    }
    case 'buses': {
      if(rest.length < 1) return usage();
      const netlist = loadNetlist(rest[0]);
      const names = rest[1] ? [rest[1]] : busNames(netlist);
      if(!names.length) {
        console.log('no buses declared in this netlist');
        break;
      }
      let ok = true;
      const dump = {};
      for(const name of names) {
        const bus = getBus(netlist, name);
        if(!params.output) {
          console.log(bus.description ? `${name} - ${bus.description}` : name);
          for(const run of bus.runs) {
            const conn = run.connector.gate ? `${run.connector.part}.${run.connector.gate}` : run.connector.part;
            console.log(`  ${conn}:`);
            printTable(busRunTable(run), ['pin', 'net', 'note']);
          }
        }
        if(params.validate) {
          const problems = validateBus(netlist, name);
          if(problems.length) {
            ok = false;
            for(const p of problems) console.log(`${name}: PROBLEM: ${p}`);
          } else if(!params.output) {
            console.log('  OK');
          }
        }
        if(!params.output) console.log('');
        dump[name] = bus;
      }
      if(params.validate && !ok) throw new Error('bus validation failed');
      if(params.output) {
        const asYAML = /\.ya?ml$/i.test(params.output);
        writeOrPrint(params.output, asYAML ? writeYAML(dump) : JSON.stringify(dump, null, 2) + '\n');
      }
      break;
    }
    default:
      console.log(`unknown command '${command}'`);
      return usage();
  }
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error.message}\n${error.stack}`);
}
