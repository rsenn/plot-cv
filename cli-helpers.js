export function Table(rows, keys) {
  let sizes = {};

  const names = keys;
  for(let row of rows) {
    for(let [j, i] of names.entries()) {
      const col = row[i] + '';
      if((sizes[i] ?? 0) < col.length) sizes[i] = col.length;
    }
  }
  let width = names.reduce((acc, name) => (acc ? acc + 3 + sizes[name] : sizes[name]), 0);
  if(width > repl.termWidth) sizes['Params'] -= width - repl.termWidth;

  const trunc = names.map((name, i) => Util.padTrunc((i == 0 ? -1 : 1) * sizes[name]));
  const pad = (cols, pad, sep) => {
    if(!Array.isArray(cols)) cols = names.map((key, i) => cols[key]);
    return cols
      .map((s, col) => trunc[col](s, pad))
      .join(sep ?? ' │ ')
      .trimEnd();
  };
  return (
    pad(names) +
    '\n' +
    pad(
      names.reduce((acc, n) => ({ ...acc, [n]: '' }), {}),
      '─',
      '─┼─'
    ) +
    '\n' +
    [...rows].reduce((acc, row) => {
      return acc + pad(row).slice(0, repl.columns) + '\n';
    }, '')
  );
}
