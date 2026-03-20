import parseDiff from './lib/parse-diff.js';
import { readFileSync, writeFileSync } from 'fs';
import { getOpt } from 'util';
import { basename, dirname, extname } from 'path';

let noWhitespace = false;

function FormatName(name) {
  return name.replaceAll(/[\W]/g, '-');
}

function ChunkIsWhitespace(chunk) {
  const { content, changes } = chunk;

  for(const { type, content } of changes) {
    switch (type) {
      case 'add':
      case 'del': {
        if(content.slice(1).trim().length > 0) return false;
        break;
      }
    }
  }

  return true;
}

function Chunk(chunk) {
  const { content, changes } = chunk;
  let s = content + '\n';

  for(const { content } of changes) s += content + '\n';

  return s;
}

function ProcessDiff(d, filename) {
  const base = basename(filename, extname(filename));

  for(const diff of d) {
    const { chunks, from: src, to: dst } = diff;

    let i = 1,
      s = `diff --git a/${src} b/${dst}\n--- a/${src}\n+++ b/${dst}\n`;

    const n = Math.ceil(Math.log10(chunks.length) + 0.0001);

    for(const chunk of chunks) {
      if(noWhitespace && ChunkIsWhitespace(chunk)) continue;

      writeFileSync(base + '_' + FormatName(dst) + '_' + `${i++}`.padStart(n, '0') + `.diff`, s + Chunk(chunk));
    }
  }
}

function main(...args) {
  let params = (globalThis.params = getOpt(
    {
      verbose: [false, (a, v) => (v | 0) + 1, 'v'],
      'no-whitespace': [false, () => (noWhitespace = true), 'W'],
      '@': 'files',
    },
    args,
  ));

  for(const arg of params['@']) {
    const d = parseDiff(readFileSync(arg, 'utf-8'));

    console.log(arg, console.config({ compact: false }), d);

    ProcessDiff(d, arg);
  }
}

main(...scriptArgs.slice(1));
