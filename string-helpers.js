export function parseDegMinSec(s) {
  let matches = [...s.matchAll(/([0-9.]*)\s*([^\s0-9]+)/g)].map(([m, ...rest]) => rest);
  let r;

  if(matches && matches.length) {
    r = 0;
    for(let [value, unit] of matches) {
      if(value === '') value = 1;
      r = {
        deg: (v, r) => r + v,
        ["'"]: (v, r) => r + v / 60,
        ['"']: (v, r) => r + v / 3600,
        N: (v, r) => r,
        W: (v, r) => -r,
        S: (v, r) => -r,
        E: (v, r) => r
      }[unit](+value, r);
      // console.log('parseDegMinSec', {value,unit,r});
    }
  }
  return r;
}

export function parseGPSLocation(s) {
  let a = s.split(/,\s+/);
  if(a && a.length == 2) return a.map(parseDegMinSec);
}
