import Util from './lib/util.js';
import { Mat } from 'mat.so';

const hr = Util.hrtime;

export class Pipeline extends Function {
  constructor(processors = [], callback) {
    let self;
    self = function(mat) {
      let i = 0;
      for(let processor of self.processors) {
        let start = hr();
        mat = processor.call(self, mat, self.images[i], i);
        if(mat) self.images[i] = mat;
        self.times[i] = hr(start);
        if(typeof callback == 'function')
          callback.call(self, self.images[i], i, self.processors.length);
        i++;
      }
      return mat;
    };
    Util.define(self, {
      processors,
      images: new Array(processors.length),
      callback
    });
    self.times = new Array(processors.length);
    return Object.setPrototypeOf(self, Pipeline.prototype);
    //return Object.assign(self, Pipeline.prototype);
  }

  get size() {
    return this.processors.length;
  }
  get names() {
    return this.processors.map((p) => p.name);
  }

  getProcessor(name_or_fn) {
    let index;
    if((index = this.processors.indexOf(name_or_fn)) != -1) return this.processors[index];
    return this.processors.find(processor => Util.fnName(processor) == name_or_fn);
  }

  processorIndex(fn) {
    if(typeof fn != 'function') fn = this.getProcessor(fn);
    return this.processors.indexOf(fn);
  }

  inputOf(processor) {
    let index = this.processorIndex(processor);
    return this.images[index];
  }
  outputOf(processor) {
    let index = this.processorIndex(processor);
    return this.images[index + 1];
  }
}

export function Processor(fn, ...args) {
  let self;
  let mapper = Util.weakMapper(() => {
    let mat = new Mat();
    //console.debug('New Mat', mat);
    return mat;
  });

  self = function(mat, out, i) {
    if(!out) {
      out = mapper(self);
    }

    fn.call(this, mat, out, ...args);
    return out;
  };
  Util.define(self, { name: Util.fnName(fn) });
  Object.setPrototypeOf(self, Processor.prototype);
  return self;
}
Object.setPrototypeOf(Processor.prototype, Function.prototype);
Object.assign(Pipeline.prototype, {
  setName(name) {
    this.name = name;
  }
});