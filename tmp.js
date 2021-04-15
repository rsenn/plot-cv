console.log(...cols.map((col, i) => (col + '').replaceAll('\n', '\\n').padEnd(colSizes[i])));
