import trkl from './lib/trkl.js';
import { sizeSync } from 'fs';
import { ReadJSON, WriteJSON } from './io-helpers.js';

export function AutoValue(filename = getConfFile('cache'), read_fn = ReadJSON, write_fn = WriteJSON) {
  let self, value;

  self = trkl(load());
  self.subscribe(newVal => {
    if(newVal !== value) {
      if(save(newVal)) value = newVal;
    }
  });
  self.merge = obj => {
    self({ ...value, ...obj });
    return self;
  };
  self.load = () => {
    let newVal;
    if((newVal = load()) === undefined) {
      throw new Error(`ERROR loading '${filename}'`);
    } else {
      value = newVal;
    }
    return self;
  };
  self.save = () => {
    save(value);
    let size = sizeSync(filename);
    //console.log(`Wrote ${size} bytes to  '${filename}'`);
    return self;
  };
  function load() {
    let retVal;
    try {
      retVal = read_fn(filename);
    } catch(e) {
      return undefined;
    }
    return retVal;
  }
  function save(newVal) {
    try {
      write_fn(filename, newVal);
    } catch(e) {
      return false;
    }
    return true;
  }
  return self;
}

export default AutoValue;
