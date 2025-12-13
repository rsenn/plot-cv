import { abbreviate, define } from './lib/misc.js';

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

export function Table(
  rows,
  keys,
  t = (cell, column) => abbreviate((cell === undefined ? '–' : cell + '').replace(/\n.*/g, ''))
) {
  let sizes = {};
  keys = keys || Object.keys(rows[0]);
  let getfn = k => (typeof k == 'function' ? k : row => row[k]);
  for(let row of rows) {
    for(let key of keys) {
      const col = t(row[key] /*getfn(key)(row)*/, key);
      if((sizes[key] ?? 0) < col.length) sizes[key] = col.length;
      if((sizes[key] ?? 0) < key.length) sizes[key] = key.length;
    }
  }
  let width = keys.reduce((acc, name) => (acc ? acc + 3 + sizes[name] : sizes[name]), 0);
  if(width > repl.termWidth) sizes['Params'] -= width - repl.termWidth;

  const trunc = keys.map((name, i) => padTrunc(/*i == 0 ? -1 :*/ 1 * sizes[name]));
  const pad = (cols, space, sep) =>
    cols
      .map((s, col) => trunc[col](t(s, col), space))
      .join(sep ?? ' │ ')
      .trimEnd();

  if(!Array.isArray(rows[0])) rows = rows.map(cols => keys.map((key, i) => getfn(key)(cols)));

  return define(
    {
      toString() {
        return [
          pad(this.keys),
          pad(
            this.keys.map(() => ''),
            '─',
            '─┼─'
          )
        ]
          .concat([...this.rows].map(row => pad(row, ' ', ' │ ').slice(0, repl.columns)))
          .map(l => ` ${l} `)
          .join('\n');
      }
    },
    { rows, keys, [Symbol.toStringTag]: 'Table', [Symbol.for('print')]: true }
  );
}

export function List(
  items,
  keys,
  t = (item, field) => (item === undefined ? '–' : item + '').replace(/[\r\n].*/g, '')
) {
  let sizes = {};
  keys = keys || Object.keys(items[0]);
  let getfn = k => (typeof k == 'function' ? k : item => item[k]);
  for(let item of items) {
    for(let key of keys) {
      const str = t(getfn(key)(item), key);
      if((sizes[key] ?? 0) < str.length) sizes[key] = str.length;
      if((sizes[key] ?? 0) < key.length) sizes[key] = key.length;
    }
  }
  let width = keys.reduce((acc, name) => (acc ? acc + 3 + sizes[name] : sizes[name]), 0);
  if(width > repl.termWidth) sizes['Params'] -= width - repl.termWidth;

  const trunc = keys.map((name, i) => padTrunc(1 * sizes[name]));
  const pad = (fields, space, sep) =>
    fields
      .map((s, str) => trunc[str](t(s, str), space))
      .join(sep ?? ' ')
      .trimEnd();

  if(!Array.isArray(items[0]))
    items = items.map(fields => keys.map((key, i) => getfn(key)(fields)));

  return define(
    {
      toString() {
        return [...this.items].map(item => pad(item, ' ', ' ').slice(0, repl.columns)).join('\n');
      }
    },
    { items, keys, [Symbol.toStringTag]: 'List', [Symbol.for('print')]: true }
  );
}