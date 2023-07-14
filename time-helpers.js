import * as cv from 'opencv';

export function Profiler(name, ticks = () => cv.getTickCount(), freq = cv.getTickFrequency()) {
  let self,
    i = 0,
    prev,
    start = ticks();

  self = function(label = `#${i}`) {
    let t = ticks();
    let split = t - (prev || start);
    if(prev) console.log(`${name} ${printTime(split).padEnd(6)} ${label}`);
    i++;
    return (prev = t);
  };

  Define(self, {
    /* prettier-ignore */ get elapsed() {
      let t = ticks();
      return t - start;
    },
    /* prettier-ignore */ get lap() {
      let t = ticks();
      return t - (prev || start);
    }
  });

  function printTime(t) {
    let time, unit;
    if(t < freq * 1e-7) {
      time = t / (freq * 1e-9);
      unit = 'ns';
    } else if(t < freq * 1e-4) {
      time = t / (freq * 1e-6);
      unit = '\u00b5s';
    } else if(t < freq * 1e-1) {
      time = t / (freq * 1e-3);
      unit = 'ms';
    } else {
      time = t / freq;
      unit = 's';
    }
    return time.toFixed(4 - Math.max(1, Math.ceil(Math.log10(time)))) + unit;
  }

  return self;
}