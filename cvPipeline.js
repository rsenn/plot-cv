import Util from './lib/util.js';
import { Mat } from 'mat';

const hr = Util.hrtime;

export class Pipeline extends Function {
  constructor(processors = [], callback) {
    let self;
    self = function(mat) {
      let i = 0;
      for(let processor of self.processors) {
        let start = hr();
        self.currentProcessor = i;

        console.log(`Pipeline \x1b[38;5;112m#${i} \x1b[38;5;32m'${processor.name}'\x1b[m`);
        mat = processor.call(self, mat, self.images[i]);
        if(mat) self.images[i] = mat;
        self.times[i] = hr(start);
        if(typeof callback == 'function')
          callback.call(self, self.images[i], i, self.processors.length);
        i++;
      }
      self.currentProcessor = -1;
      return mat;
    };
    processors = processors.map(processor =>
      processor instanceof Processor ? processor : Processor(processor)
    );
    Util.define(self, {
      processors,
      currentProcessor: -1,
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
    return this.processors.find(processor => processor.name == name_or_fn);
  }

  processorIndex(fn) {
    if(typeof fn != 'function') fn = this.getProcessor(fn);
    return this.processors.indexOf(fn);
  }

  inputOf(processor) {
    let index = this.processorIndex(processor);
    return this.processors[index].in ?? this.images[index - 1];
  }
  outputOf(processor) {
    let index = this.processorIndex(processor);
    return this.processors[index].out ?? this.images[index];
  }
}

export function Processor(fn, ...args) {
  let self;
  let mapper = Util.weakMapper(() => {
    let mat = new Mat();
    //console.debug('New Mat', mat);
    return mat;
  });

  self = function(src, dst, i) {
    if(dst && mapper.get(self))
      throw new Error(`Duplicate output Mat for processor '${self.name}`);

    if(dst) mapper.set(self, dst);
    else if(!dst) dst = mapper(self);

    if(!('in' in self)) self.in = src;
    if(!('out' in self)) self.out = dst;

    fn.call(this, src, dst, ...args);
    return dst;
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
