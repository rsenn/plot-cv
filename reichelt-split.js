(ns = ReadFile('reichelt3.txt').split(/\n/g)), (l = 11);
a = Array.from({ length: l })
  .map((n, i) => Array.from({ length: 6 }).map((m, j) => s[i + j * l]))
  .map(([value, name, whg, count, unit_price, price]) => ({ name, value, whg, count, unit_price, price }));
