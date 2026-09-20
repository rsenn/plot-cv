#!/usr/bin/env qjsm
/*
 * eagle-agent, stage 1 (doc/eagle-agent.md §1): walk every .lbr file's
 * devicesets/gates/symbols/pins/devices/packages/connects into a plain JS
 * tree, with NO condensation and NO filtering, and dump it as YAML for
 * inspection. Stage 2's condensation policy is now decided in the doc
 * (R/L/C/CPOL get a curated H/V device pair each, everything else drops
 * to deviceset+pins only) - this script still doesn't implement any of
 * that; it's Stage 3's job to apply it here.
 */
import { Console } from 'console';
import { getOpt, glob, GLOB_BRACE } from 'util';
import { EagleDocument } from './eagle.js';
import { ReadFile, WriteFile } from './io-helpers.js';
import { write as writeYAML } from 'yaml';

function collectPins(symbol) {
  const pins = {};
  for(const name of Reflect.ownKeys(symbol.pins)) {
    const pin = symbol.pins[name];
    pins[name] = {
      direction: pin.getAttribute('direction') || undefined,
      x: pin.x,
      y: pin.y,
    };
  }
  return pins;
}

function collectGates(deviceset) {
  const gates = {};
  for(const name of Reflect.ownKeys(deviceset.gates)) {
    const gate = deviceset.gates[name];
    gates[name] = {
      symbol: gate.getAttribute('symbol'),
      pins: collectPins(gate.symbol),
    };
  }
  return gates;
}

function collectDevices(deviceset) {
  const devices = {};
  for(const name of Reflect.ownKeys(deviceset.devices)) {
    const device = deviceset.devices[name];
    devices[name || '(default)'] = {
      package: device.getAttribute('package') || undefined,
      connects: [...(device.connects?.children ?? [])].map(c => ({
        gate: c.getAttribute('gate'),
        pin: c.getAttribute('pin'),
        pad: c.getAttribute('pad'),
      })),
    };
  }
  return devices;
}

function collectDevicesets(library) {
  const devicesets = {};
  for(const name of Reflect.ownKeys(library.devicesets)) {
    const deviceset = library.devicesets[name];
    devicesets[name] = {
      prefix: deviceset.getAttribute('prefix') || undefined,
      uservalue: deviceset.getAttribute('uservalue') || undefined,
      description: deviceset.description?.textContent || undefined,
      gates: collectGates(deviceset),
      devices: collectDevices(deviceset),
    };
  }
  return devicesets;
}

function collectLibrary(file) {
  const doc = EagleDocument.open(file, ReadFile);
  if(doc.type != 'library') throw new Error(`'${file}' is a ${doc.type}, not a library`);
  return collectDevicesets(doc.library);
}

function main(...args) {
  globalThis.console = new Console({ inspectOptions: { colors: true, depth: Infinity, compact: false } });

  const params = getOpt(
    {
      output: [true, null, 'o'],
      '@': 'input',
    },
    args,
  );

  const patterns = params['@']?.length ? params['@'] : ['../pictest/eagle/lbr/*.lbr'];
  const files = unique(patterns.flatMap(p => glob(p, GLOB_BRACE))).sort();

  console.log(`collecting ${files.length} .lbr file(s)...`);

  const catalog = {};
  for(const file of files) {
    const name = file.replace(/^.*\//, '');
    try {
      catalog[name] = collectLibrary(file);
    } catch(e) {
      console.log(`skipping '${file}': ${e.message}`);
    }
  }

  const out = writeYAML(catalog);

  if(params.output) {
    WriteFile(params.output, out);
    console.log(`wrote '${params.output}' (${out.length} bytes)`);
  } else {
    console.log(out);
  }
}

function unique(arr) {
  return [...new Set(arr)];
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error.message}\n${error.stack}`);
}
