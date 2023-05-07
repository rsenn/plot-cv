import { define } from './lib/misc.js';

function padTrunc(...args) {
  let [len, s] = args;
  const end = len >= 0;
  len = Math.abs(len);
  if(args.length < 2) {
    return (s, pad = ' ') => {
      s = s + '';
      len ??= s.length;
      return s.length > len ? s.slice(0, len) : s['pad' + (end ? 'End' : 'Start')](len, pad);
    };
  } else {
    s = s + '';
    len ??= s.length;
    return s.length > len ? s.slice(0, len) : s['pad' + (end ? 'End' : 'Start')](len, ' ');
  }
}

export function Table(rows, keys, t = (cell, column) => (cell + '').replace(/\n.*/g, '').trim()) {
  let sizes = {};
  const names = keys || Object.keys(rows[0]);
  let getfn = k => (typeof k == 'function' ? k : row => row[k]);
  for(let row of rows) {
    for(let key of names) {
      const col = t(row[key] /*getfn(key)(row)*/, key);
      if((sizes[key] ?? 0) < col.length) sizes[key] = col.length;
      if((sizes[key] ?? 0) < key.length) sizes[key] = key.length;
    }
  }
  let width = names.reduce((acc, name) => (acc ? acc + 3 + sizes[name] : sizes[name]), 0);
  if(width > repl.termWidth) sizes['Params'] -= width - repl.termWidth;

  const trunc = names.map((name, i) => padTrunc(/*i == 0 ? -1 :*/ 1 * sizes[name]));
  const pad = (cols, space, sep) =>
    cols
      .map((s, col) => trunc[col](t(s, col), space))
      .join(sep ?? ' │ ')
      .trimEnd();

  if(!Array.isArray(rows[0])) rows = rows.map(cols => names.map((key, i) => getfn(key)(cols)));

  return define(
    {
      toString() {
        return [
          pad(this.names),
          pad(
            this.names.map(() => ' '),
            '─',
            '─┼─'
          )
        ]
          .concat([...this.rows].map(row => pad(row, ' ', ' │ ').slice(0, repl.columns)))
          .map(l => ` ${l} `)
          .join('\n');
      }
    },
    { rows, names, [Symbol.toStringTag]: 'Table', [Symbol.for('print')]: true }
  );
}
