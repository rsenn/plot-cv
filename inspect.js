export const inspect = (o, pred = (v) => true) =>
  '{\n  ' +
  [
    [(o) => o, Object.keys],
    [Object.getPrototypeOf, Object.keys],
    [(o) => o, Object.getOwnPropertyNames],
    [Object.getPrototypeOf, Object.getOwnPropertyNames]
  ]
    .reduce((a, [proto, keys]) => (a.length ? a : [...a, ...keys(proto(o))]), [])
    .reduce((a, k) => (a.indexOf(k) == -1 ? [...a, k] : a), [])
    .map((k) => [k, o[k]])
    .map(([k, v]) => {
      if(pred(v, k) == false) return '';
      let s = v;
      if(typeof s != 'string') {
        try {
          if(typeof s == 'object') s = inspect(s, pred);
          else s = s + '';
        } catch(err) {
          s = typeof s;
        }
        if(typeof v == 'function') s = s.substring(0, s.indexOf('{')).trim();
      } else {
        s = '"' + s + '"';
      }
      if(s.length > 200) s = s.substring(0, 200) + '...';
      return k + ': ' + s.replace(/\n\s*/g, ' ');
    })
    //   .filter((item) => item != '')
    .join(',\n  ') +
  '\n}';

export default inspect;
