#!/usr/bin/env qjsm
/*
 * eagle-agent: prerequisite 3 (doc/eagle-agent.md) - given an exact list of
 * "library:deviceset" pairs, emit their gate/pin data as compact YAML,
 * in the shape scr-rules.md expects. This is the "make a small, exact,
 * copy-paste-ready YAML block" counterpart to eagle-lookup.js's console
 * search - use lookup to *find* names, then this to *emit* them.
 */
import { Console } from 'console';
import { getOpt } from 'util';
import * as path from './lib/path.js';
import { EagleDocument } from './eagle.js';
import { ReadFile } from './io-helpers.js';
import { write as writeYAML } from 'yaml';

/* Fixed absolute path (not derived from this script's own location) - this
 * runs from arbitrary project directories (an-tronics, pictest, lc-meter)
 * AND from an installed location (/usr/local/bin, CMakeLists.txt), where
 * a path relative to the script's own dir would resolve wrong. Default is
 * the PIC/digital-oriented library set; override with --lib-dir for the
 * analog set (op-amps/OTAs/CMOS/RF) at /mnt/data/Projects/an-tronics/eagle/lbr
 * - see the eagle-circuit skill's "Analog designs" note. */
let LIB_DIR = '/mnt/data/Projects/pictest/eagle/lbr';

/* Same curated defaults as eagle-materialize.js's CURATED_DEVICE - kept in
 * lockstep by hand (doc/eagle-agent.md §2c) since there's no shared module
 * between the two tools yet. This is the device-name SUFFIX that must be
 * concatenated directly (no separator) onto the deviceset name in an ADD
 * command whenever the deviceset has no blank-name/default device -
 * confirmed against a live EAGLE 7.2.0 install: a bare deviceset name with
 * no suffix (e.g. `R@...`) is rejected as "Device not found" unless that
 * exact deviceset happens to have an unnamed default device. */
const CURATED_DEVICE = {
  R: '0204/10',
  'R-H': '0204/10',
  L: '03/10',
  C: '050-025X075',
  CPOL: 'E2.5-5',
};

function pickDevice(deviceset, devicesetName) {
  if(CURATED_DEVICE[devicesetName]) return CURATED_DEVICE[devicesetName];

  const names = Reflect.ownKeys(deviceset.devices);
  if(names.includes('')) return '';
  console.log(`NOTE: '${devicesetName}' has ${names.length} devices, no default - '${names[0]}' picked arbitrarily (real choices: ${names.join(', ')})`);
  return names[0];
}

function extractGates(deviceset) {
  const gates = {};
  for(const name of Reflect.ownKeys(deviceset.gates)) {
    const gate = deviceset.gates[name];
    const pins = {};
    for(const pn of Reflect.ownKeys(gate.symbol.pins)) {
      const pin = gate.symbol.pins[pn];
      pins[pn] = { direction: pin.getAttribute('direction') || undefined, x: pin.x, y: pin.y };
    }
    gates[name] = { pins };
  }
  return gates;
}

function extract(libDeviceset) {
  const [library, devicesetName] = libDeviceset.split(':');
  if(!library || !devicesetName) throw new Error(`'${libDeviceset}' is not 'library:deviceset'`);

  const doc = EagleDocument.open(`${LIB_DIR}/${library}.lbr`, ReadFile);
  const deviceset = doc.library.devicesets[devicesetName];
  if(!deviceset) {
    const names = Reflect.ownKeys(doc.library.devicesets);
    throw new Error(`no deviceset '${devicesetName}' in ${library}.lbr. Real names: ${names.join(', ')}`);
  }

  return {
    library,
    prefix: deviceset.getAttribute('prefix') || undefined,
    device: pickDevice(deviceset, devicesetName),
    gates: extractGates(deviceset),
  };
}

function main(...args) {
  globalThis.console = new Console({ inspectOptions: { colors: true, depth: Infinity } });

  const params = getOpt({ 'lib-dir': [true, null], '@': 'input' }, args);
  if(params['lib-dir']) LIB_DIR = params['lib-dir'];
  const specs = params['@'] || args;

  if(specs.length == 0) {
    console.log("usage: eagle-extract.js <library:deviceset> ... [--lib-dir <path>]\n  e.g. eagle-extract.js r:R r:R-H l:L c:C c:CPOL");
    return;
  }

  const catalog = {};
  for(const spec of specs) {
    const [, devicesetName] = spec.split(':');
    catalog[devicesetName] = extract(spec);
  }

  console.log(writeYAML(catalog));
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error.message}\n${error.stack}`);
}
