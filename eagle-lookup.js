#!/usr/bin/env qjsm
/*
 * eagle-agent: live search over the real .lbr files, in place of a
 * pre-baked catalog (doc/eagle-agent.md - "Two more redundancy problems").
 * Prints real deviceset/gate/pin names so a skill can copy them exactly
 * into a netlist, instead of guessing or relying on a stale static dump.
 */
import { Console } from 'console';
import { getOpt, glob, GLOB_BRACE } from 'util';
import * as path from './lib/path.js';
import { EagleDocument } from './eagle.js';
import { ReadFile } from './io-helpers.js';

/* Fixed absolute path (not derived from this script's own location) - this
 * runs from arbitrary project directories (an-tronics, pictest, lc-meter)
 * AND from an installed location (/usr/local/bin, CMakeLists.txt), where
 * a path relative to the script's own dir would resolve wrong. Default is
 * the PIC/digital-oriented library set; override with --lib-dir for the
 * analog set (op-amps/OTAs/CMOS/RF) at /mnt/data/Projects/an-tronics/eagle/lbr
 * - see the eagle-circuit skill's "Analog designs" note. */
let LIB_DIR = '/mnt/data/Projects/pictest/eagle/lbr';

function search(pattern, files) {
  const re = new RegExp(pattern, 'i');
  for(const file of files) {
    const doc = EagleDocument.open(file, ReadFile);
    if(doc.type != 'library') continue;
    const libName = file.replace(/^.*\//, '');
    for(const name of Reflect.ownKeys(doc.library.devicesets)) {
      if(!re.test(name)) continue;
      const ds = doc.library.devicesets[name];
      console.log(`${libName} :: ${name}  (prefix=${ds.getAttribute('prefix') || '?'})`);
      const desc = ds.description?.textContent;
      if(desc) console.log(`  ${desc.replace(/<[^>]*>/g, '').trim().split('\n')[0]}`);
      for(const gn of Reflect.ownKeys(ds.gates)) {
        const gate = ds.gates[gn];
        const pins = Reflect.ownKeys(gate.symbol.pins).map(pn => {
          const p = gate.symbol.pins[pn];
          return `${pn}(${p.getAttribute('direction') || '?'})`;
        });
        console.log(`  gate ${gn}: ${pins.join(' ')}`);
      }
      const devNames = Reflect.ownKeys(ds.devices);
      console.log(`  devices: ${devNames.map(d => d || '(default)').join(', ')}`);
    }
  }
}

function main(...args) {
  globalThis.console = new Console({ inspectOptions: { colors: true, depth: Infinity } });

  const params = getOpt(
    {
      library: [true, null, 'l'],
      'lib-dir': [true, null],
      '@': 'input',
    },
    args,
  );

  if(params['lib-dir']) LIB_DIR = params['lib-dir'];

  const pattern = (params['@'] || args)[0];
  if(!pattern) {
    console.log('usage: eagle-lookup.js <pattern> [-l library.lbr] [--lib-dir <path>]');
    return;
  }

  const files = params.library
    ? glob(`${LIB_DIR}/${params.library}`, GLOB_BRACE)
    : glob(`${LIB_DIR}/*.lbr`, GLOB_BRACE);

  search(pattern, files);
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error.message}\n${error.stack}`);
}
