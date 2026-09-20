#!/usr/bin/env qjsm
import { getOpt } from 'util';
import * as std from 'std';
import * as os from 'os';
import { JsonParser, JsonSerializer } from 'json';
import { TextDecoder } from 'textcode';

/* Column order of each "X" row, as emitted by mediathek-list. */
const FIELDS = [
  'Sender', 'Thema', 'Titel', 'Datum', 'Zeit', 'Dauer', 'Groesse', 'Beschreibung',
  'Url', 'Website', 'UrlUntertitel', 'UrlRTMP', 'UrlKlein', 'UrlRTMPKlein',
  'UrlHD', 'UrlRTMPHD', 'DatumL', 'UrlHistory', 'Geo', 'Neu',
];
const IDX = Object.fromEntries(FIELDS.map((name, i) => [name, i]));

/* Original German header row, re-emitted verbatim in place of the source's
 * own header line (which carries the same trailing-byte corruption as data
 * rows — see parseRow's doc comment — so it isn't safe to just pass through). */
const FIELDS_HEADER = [
  'Sender', 'Thema', 'Titel', 'Datum', 'Zeit', 'Dauer', 'Größe [MB]', 'Beschreibung',
  'Url', 'Website', 'Url Untertitel', 'Url RTMP', 'Url Klein', 'Url RTMP Klein',
  'Url HD', 'Url RTMP HD', 'DatumL', 'Url History', 'Geo', 'neu',
];

/* Wraps a file (or stdin, fd 0) as a JsonParser reader method, same as jsonpp.js:
 * pulls raw bytes on demand via os.read() instead of reading the whole file upfront. */
function fileReader(file) {
  const fd = file === '-' ? 0 : os.open(file, os.O_RDONLY);
  if(fd < 0) throw new Error(`cannot open '${file}'`);
  return {
    read(buf, len) {
      const n = os.read(fd, buf, 0, len);
      if(n < 0) throw new Error(`read '${file}' failed`);
      return n;
    },
    close() {
      if(file !== '-') os.close(fd);
    },
  };
}

function usage(exitCode) {
  std.puts(
    `Usage: ${scriptArgs[0]} [OPTIONS] [FILE]\n\n` +
      `Streaming filter for mediathek-list's JSON dump (one "X":[...] row per entry).\n` +
      `Keeps entries whose Dauer >= --min-length and whose Datum falls in\n` +
      `[--since, --until], dropping any whose Titel/Thema match an --exclude\n` +
      `keyword (default: Audiodeskription, Untertitel). No file means stdin.\n` +
      `Never buffers the whole document: reads and writes are both streamed.\n\n` +
      `  -s, --since EXPR      range start: "1y ago", "6mo ago", "2024-01-01" (default: 1y ago)\n` +
      `  -u, --until EXPR      range end: "now", "1w ago" (default: now)\n` +
      `  -l, --min-length DUR  minimum Dauer: "30m", "00:30:00", "1h" (default: 30m)\n` +
      `  -x, --exclude WORD    drop entries whose Titel/Thema contains WORD (repeatable)\n` +
      `  -i, --include WORD    keep only entries whose Titel/Thema/Beschreibung contains WORD (repeatable)\n` +
      `  -o, --output FILE     write to FILE instead of stdout\n` +
      `  -v, --verbose         print every corrupted row's parse error (default: one summary line)\n` +
      `  -h, --help            show this help\n\n` +
      `A minority of rows are corrupted in mediathek-list's own output (a known\n` +
      `field-splitting bug, not this script) and get silently dropped; a one-line\n` +
      `summary with the count goes to stderr at the end unless -v is given.\n`,
  );
  std.exit(exitCode);
}

/* --- date & duration parsing --- */

const DATE_UNITS = { y: 365.25 * 86400, mo: 30 * 86400, w: 7 * 86400, d: 86400, h: 3600 };

function parseDateExpr(str, now) {
  str = str.trim();
  let m;

  if(/^now$/i.test(str)) return now;

  if((m = /^(\d+(?:\.\d+)?)\s*(y|mo|w|d|h)(\s*ago)?$/i.exec(str)))
    return now - parseFloat(m[1]) * DATE_UNITS[m[2].toLowerCase()];

  if((m = /^(\d{1,2})\.(\d{1,2})\.(\d{4})$/.exec(str)))
    return new Date(+m[3], +m[2] - 1, +m[1]).getTime() / 1000;

  const t = Date.parse(str);
  if(!isNaN(t)) return t / 1000;

  throw new Error(`unrecognized date expression: '${str}'`);
}

function parseDuration(str) {
  str = str.trim();
  let m;

  if((m = /^(\d{1,3}):(\d{2}):(\d{2})$/.exec(str))) return +m[1] * 3600 + +m[2] * 60 + +m[3];

  if((m = /^(\d+(?:\.\d+)?)\s*(h|m|s)?$/i.exec(str)))
    return parseFloat(m[1]) * { h: 3600, m: 60, s: 1 }[(m[2] || 'm').toLowerCase()];

  throw new Error(`unrecognized duration: '${str}'`);
}

function durationField(dauer) {
  const m = /^(\d{1,3}):(\d{2}):(\d{2})$/.exec(dauer || '');
  return m ? +m[1] * 3600 + +m[2] * 60 + +m[3] : 0;
}

/* --- row predicate --- */

function passes(row, opts) {
  const titel = row[IDX.Titel] || '';
  const thema = row[IDX.Thema] || '';
  const hay = (titel + '\n' + thema).toLowerCase();

  for(const kw of opts.exclude) if(hay.includes(kw.toLowerCase())) return false;

  if(opts.include.length) {
    const hay2 = hay + '\n' + (row[IDX.Beschreibung] || '').toLowerCase();
    if(!opts.include.some(kw => hay2.includes(kw.toLowerCase()))) return false;
  }

  if(durationField(row[IDX.Dauer]) < opts.minLength) return false;

  const datumL = +row[IDX.DatumL];
  if(!isFinite(datumL) || datumL < opts.since || datumL > opts.until) return false;

  return true;
}

/* mediathek-list writes one "X":[ ... ] row per physical line. Line-buffered
 * reading over raw pulled bytes (never the whole document at once): split on
 * 0x0A (always a real newline, never a UTF-8 continuation byte) and decode
 * each completed line separately, so a stray invalid byte can't corrupt
 * decoding of the lines around it. */
function readLines(reader, onLine) {
  const CHUNK = 1 << 16;
  const chunk = new ArrayBuffer(CHUNK);
  const chunkView = new Uint8Array(chunk);
  let pending = new Uint8Array(0);

  /* A fresh TextDecoder per line, not one reused across the whole stream:
   * TextDecoder is incremental/buffered by design (a trailing invalid byte
   * is held back on the chance it starts a multi-byte sequence continued by
   * the next decode() call). Reused across lines, one corrupted line's
   * dangling byte(s) would silently bleed into the next line's decoded
   * text. Per-line decoders keep corruption contained to the line it's in. */
  function decodeLine(bytes) {
    /* TextDecoder throws (regardless of {stream:true}) when it hits a run of
     * genuinely invalid UTF-8 it can't resync within 4 bytes — expected on
     * mediathek-list's corrupted rows. Falling back to a lossy byte-for-byte
     * decode keeps line framing intact; the corrupted row it produces still
     * has to clear JsonParser in parseRow(), so garbage is caught there. */
    try {
      return new TextDecoder('utf-8').decode(bytes, { stream: true });
    } catch(error) {
      let s = '';
      for(let i = 0; i < bytes.length; i++) s += String.fromCharCode(bytes[i]);
      return s;
    }
  }

  for(;;) {
    const n = reader.read(chunk, CHUNK);
    if(n <= 0) break;

    const data = new Uint8Array(pending.length + n);
    data.set(pending);
    data.set(chunkView.subarray(0, n), pending.length);

    let start = 0;
    for(let i = 0; i < data.length; i++) {
      if(data[i] === 10) {
        onLine(decodeLine(data.subarray(start, i)));
        start = i + 1;
      }
    }
    pending = data.subarray(start);
  }
  if(pending.length) onLine(decodeLine(pending));
}

/* Pulls the field values out of one '"X":[ ... ]' line via JsonParser, scoped
 * to just the bracketed array text (a self-contained JSON value). Using a
 * fresh parser per row — rather than one instance streaming the whole
 * document — matters here: mediathek-list's output has real corrupted bytes
 * in some field values, and JsonParser's error-resync advances roughly one
 * character per thrown SyntaxError, which is fine for one bad line but
 * pathological across a multi-hundred-MB document with many bad spans. A
 * bad row now costs exactly one caught exception (the whole row is
 * dropped), instead of thousands of retries bleeding into later rows. */
function parseRow(line) {
  const open = line.indexOf('[');
  const close = line.lastIndexOf(']');
  if(open < 0 || close < open) return null;

  const parser = new JsonParser(line.slice(open, close + 1));
  const row = [];

  for(;;) {
    const tok = parser.parse();
    if(tok === 'NEED_DATA' || tok === 'NONE') return row;
    if(tok === 'STRING' || tok === 'NUMBER') row.push(parser.token);
  }
}

function filterStream(reader, filename, opts, put) {
  let first = true;
  let sawHeader = false;
  let total = 0;
  let corrupt = 0;
  let kept = 0;

  function emit(r) {
    if(!first) put(',\n');
    first = false;
    put('"X":');
    const s = new JsonSerializer(r);
    let chunk;
    while((chunk = s.read(256)) !== '') put(chunk);
  }

  put('{\n');
  emit(FIELDS_HEADER);

  let lineNo = 0;

  readLines(reader, line => {
    lineNo++;
    if(!/^\s*"X":\[/.test(line)) return;
    total++;

    let row;
    try {
      row = parseRow(line);
    } catch(error) {
      corrupt++;
      /* mediathek-list's own output has a real, known field-splitting bug
       * (see BUGS: mediathek-list-field-splitter-not-escape-aware) that
       * corrupts a visible minority of rows beyond repair; that's not
       * something wrong with THIS filter, so stay quiet about individual
       * rows by default and only report a count -x-v gets the detail.
       * error.message already has its own "1:col" from the per-line
       * JsonParser (col within THIS line, not a real file line) — swap in
       * the real line number so -v output points somewhere useful. */
      const col = /^1:(.*)$/.exec(error.message);
      if(opts.verbose) std.err.puts(`${filename}:${lineNo}:${col ? col[1] : error.message}\n`);
      return;
    }
    if(!row || !row.length) {
      corrupt++;
      if(opts.verbose) std.err.puts(`${filename}:${lineNo}: no "[...]" array found on this line\n`);
      return;
    }

    if(!sawHeader && row[0] === 'Sender' && row[2] === 'Titel') {
      sawHeader = true;
      total--;
      return;
    }

    if(passes(row, opts)) {
      kept++;
      emit(row);
    }
  });

  put('\n}\n');

  if(total)
    std.err.puts(
      `${filename}: ${kept} kept, ${total - corrupt - kept} filtered out, ` +
        `${corrupt} corrupted rows dropped (${((100 * corrupt) / total).toFixed(1)}% of ${total}) — ` +
        `known mediathek-list bug, not this filter; rerun with -v for per-row detail.\n`,
    );
}

function main(...args) {
  const now = Date.now() / 1000;

  const params = getOpt(
    {
      help: [false, () => usage(0), 'h'],
      since: [true, null, 's'],
      until: [true, null, 'u'],
      'min-length': [true, null, 'l'],
      exclude: [true, (v, prev) => [...(prev || []), v], 'x'],
      include: [true, (v, prev) => [...(prev || []), v], 'i'],
      output: [true, null, 'o'],
      verbose: [false, null, 'v'],
      '@': 'files',
    },
    args,
  );

  const opts = {
    since: parseDateExpr(params.since || '1y ago', now),
    until: parseDateExpr(params.until || 'now', now),
    minLength: parseDuration(params['min-length'] || '30m'),
    exclude: params.exclude || ['Audiodeskription', 'Untertitel'],
    include: params.include || [],
    verbose: !!params.verbose,
  };

  const files = params['@'].length ? params['@'] : ['-'];
  const out = params.output ? std.open(params.output, 'w+') : std.out;
  const put = s => out.puts(s);

  let failed = false;

  for(const file of files) {
    try {
      const reader = fileReader(file);
      try {
        filterStream(reader, file === '-' ? '<stdin>' : file, opts, put);
      } finally {
        reader.close();
      }
    } catch(error) {
      std.err.puts(`${file}: ${error.message}\n`);
      failed = true;
    }
  }

  out.flush();
  if(failed) std.exit(1);
}

main(...scriptArgs.slice(1));
