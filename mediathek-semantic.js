#!/usr/bin/env qjsm
/*
mediathek-semantic.js ranks a mediathek-filter.js candidate file by semantic
similarity to a --query sentence (sentence-transformer embeddings via
qjs-opencv), plus a learned tag bonus from past --learn feedback. It needs
two things to exist first: a raw mediathek-list dump, and a mediathek-filter.js
pass over that dump to produce the candidate pool it actually ranks.

Preparation (run once, re-run the first line occasionally to refresh):

mediathek-list -o ~/mediathek-filme.json

qjsm mediathek-filter.js -l 15m -s "2y ago" -i doku -i "Terra X" -i Wissen -i Geschichte -i Katastrophe -i Umwelt -i Natur -i Technik ~/mediathek-filme.json > ~/mediathek-doku-filtered.json

Four example semantic queries (topics this household actually asked for),
each piped straight into mediathek-parser to produce a ready-to-run
low-quality (-l) download script:

qjsm mediathek-semantic.js --query "Weltraum Raumfahrt Astronaut Mission Rakete Universum Planeten" --limit 3000 --top 8 ~/mediathek-doku-filtered.json | mediathek-parser -l -F wget -o ~/mediathek-space-download.sh

qjsm mediathek-semantic.js --query "Eine Dokumentation über eine Ölkatastrophe oder Umweltkatastrophe wie die Explosion der Bohrinsel Deepwater Horizon" --limit 6000 --top 8 ~/mediathek-doku-filtered.json | mediathek-parser -l -F wget -o ~/mediathek-disaster-download.sh

qjsm mediathek-semantic.js --query "Eine Dokumentation über die Geschichte der Wissenschaft und berühmte Wissenschaftler und ihre großen Entdeckungen" --limit 6000 --top 8 ~/mediathek-doku-filtered.json | mediathek-parser -l -F wget -o ~/mediathek-science-download.sh

qjsm mediathek-semantic.js --query "Eine Geschichte der Kryptographie und schlauer Köpfe im Zweiten Weltkrieg" --limit 6000 --top 10 ~/mediathek-doku-filtered.json | mediathek-parser -l -F wget -o ~/mediathek-krypto-download.sh

After watching something from one of those scripts, teach the ranker what
you liked (bumps a tag-weight file used by every future query):

qjsm mediathek-semantic.js --learn "https://...the entry's Url field..." ~/mediathek-doku-filtered.json
*/
import { getOpt } from 'util';
import * as std from 'std';
import * as os from 'os';
import { JsonParser, JsonSerializer } from 'json';
import { TextDecoder } from 'textcode';
import * as cv from 'opencv';

/* Column order of each "X" row, as emitted by mediathek-list (see mediathek-filter.js). */
const FIELDS = [
  'Sender', 'Thema', 'Titel', 'Datum', 'Zeit', 'Dauer', 'Groesse', 'Beschreibung',
  'Url', 'Website', 'UrlUntertitel', 'UrlRTMP', 'UrlKlein', 'UrlRTMPKlein',
  'UrlHD', 'UrlRTMPHD', 'DatumL', 'UrlHistory', 'Geo', 'Neu',
];
const IDX = Object.fromEntries(FIELDS.map((name, i) => [name, i]));

/* import.meta.url resolves the model directory relative to this script's own
 * location, not the caller's cwd, so it runs the same from anywhere. No
 * global URL/std.dirname in this qjs build, so strip 'file://' and the
 * trailing filename by hand. */
const SCRIPT_PATH = import.meta.url.replace(/^file:\/\//, '');
const SCRIPT_DIR = SCRIPT_PATH.slice(0, SCRIPT_PATH.lastIndexOf('/'));
const MODELS_DIR = SCRIPT_DIR + '/qjs-opencv/examples/models';

const PREFS_FILE = std.getenv('HOME') + '/.mediathek-preferences.json';
const CACHE_FILE_TMPL = model => std.getenv('HOME') + `/.mediathek-embeddings.${model}.json`;

function usage(exitCode) {
  std.puts(
    `Usage: ${scriptArgs[0]} [OPTIONS] [FILE]\n\n` +
      `Semantic search + preference learning on top of mediathek-filter.js's output.\n` +
      `Ranks entries by cosine similarity of a sentence embedding against --query,\n` +
      `plus a learned tag bonus from past --learn feedback. No file means stdin.\n\n` +
      `  -q, --query TEXT      rank by semantic similarity to TEXT (required unless --learn)\n` +
      `      --model NAME      embedding model short-name (default: minilm-multilingual)\n` +
      `      --limit N         cap candidates considered, most recent first (default: 3000)\n` +
      `      --top N           print only the top N results (default: 20)\n` +
      `      --learn URL       record positive feedback for the entry with this Url and exit\n` +
      `      --weight W        tag weight bump for --learn (default: 0.15)\n` +
      `  -o, --output FILE     write ranked "X":[...] rows to FILE (for piping into mediathek-parser)\n` +
      `  -h, --help            show this help\n\n` +
      `Input is mediathek-filter.js's streamed JSON ("X":[...] rows). Embeddings are\n` +
      `cached per-model in ~/.mediathek-embeddings.<model>.json, keyed by Url, so\n` +
      `repeat runs only embed new/changed entries.\n\n` +
      `--query works best as a natural sentence ("Eine Dokumentation ueber ..."),\n` +
      `not a bag of keywords -- this is a sentence-transformer model, trained on\n` +
      `full sentences, and a keyword list ranks noticeably worse in practice.\n`,
  );
  std.exit(exitCode);
}

/* --- input streaming (same framing as mediathek-filter.js: one JSON row per line) --- */

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

function readLines(reader, onLine) {
  const CHUNK = 1 << 16;
  const chunk = new ArrayBuffer(CHUNK);
  const chunkView = new Uint8Array(chunk);
  let pending = new Uint8Array(0);

  function decodeLine(bytes) {
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

function readRows(file) {
  const rows = [];
  const reader = fileReader(file);
  try {
    readLines(reader, line => {
      if(!/^\s*"X":\[/.test(line)) return;
      let row;
      try {
        row = parseRow(line);
      } catch(error) {
        return; // corrupted row, see mediathek-filter.js/BUGS -- silently skip here too
      }
      if(!row || !row.length) return;
      if(row[0] === 'Sender' && row[2] === 'Titel') return; // header row
      rows.push(row);
    });
  } finally {
    reader.close();
  }
  return rows;
}

/* --- Unigram/SentencePiece tokenizer (validated against Python ground truth) --- */

function loadUnigramTokenizer(path) {
  const text = std.loadFile(path);
  if(!text) throw new Error(`cannot read tokenizer file '${path}'`);
  const raw = JSON.parse(text);
  const vocab = raw.model.vocab;
  const unkId = raw.model.unk_id;
  const pieceToId = new Map();
  const scores = new Float64Array(vocab.length);
  for(let i = 0; i < vocab.length; i++) {
    pieceToId.set(vocab[i][0], i);
    scores[i] = vocab[i][1];
  }
  const bosId = pieceToId.get('<s>');
  const eosId = pieceToId.get('</s>');
  const UNK_PENALTY = -100;

  function segmentWord(word) {
    const n = word.length;
    const best = new Float64Array(n + 1).fill(-Infinity);
    const backPos = new Int32Array(n + 1).fill(-1);
    const backId = new Int32Array(n + 1).fill(-1);
    best[0] = 0;

    for(let i = 0; i < n; i++) {
      if(best[i] === -Infinity) continue;
      for(let j = i + 1; j <= n; j++) {
        const piece = word.slice(i, j);
        const id = pieceToId.get(piece);
        if(id !== undefined) {
          const s = best[i] + scores[id];
          if(s > best[j]) {
            best[j] = s;
            backPos[j] = i;
            backId[j] = id;
          }
        }
      }
      if(backPos[i + 1] === -1) {
        const s = best[i] + UNK_PENALTY;
        if(s > best[i + 1]) {
          best[i + 1] = s;
          backPos[i + 1] = i;
          backId[i + 1] = unkId;
        }
      }
    }

    const ids = [];
    let pos = n;
    while(pos > 0) {
      ids.push(backId[pos]);
      pos = backPos[pos];
    }
    ids.reverse();
    return ids;
  }

  function encode(text) {
    const normalized = text.normalize('NFKC');
    const words = normalized.split(/\s+/).filter(w => w.length);
    const ids = [bosId];
    for(const w of words) ids.push(...segmentWord('▁' + w));
    ids.push(eosId);
    return ids;
  }

  return { encode };
}

/* --- model loading & embedding --- */

function loadModel(name) {
  const dir = `${MODELS_DIR}/${name}`;
  const manifestText = std.loadFile(`${dir}/manifest.json`);
  if(!manifestText) throw new Error(`no such model '${name}' (expected ${dir}/manifest.json)`);
  const manifest = JSON.parse(manifestText);
  const tokenizer = loadUnigramTokenizer(`${dir}/tokenizer.json`);
  const net = cv.readNetFromONNX(`${dir}/${manifest.onnx}`);
  if(net.empty) throw new Error(`${name}: failed to load ${manifest.onnx}`);
  return { manifest, tokenizer, net, seqLen: manifest.seqLen, dim: manifest.dim };
}

/* Runs one forward pass and mean-pools + L2-normalizes to a single sentence
 * embedding. The 64*384-element mean-pool stays plain JS -- it's ~24k ops,
 * already sub-millisecond, and routing it through cv.Mat would only add
 * complexity; the actual payoff of using cv.Mat (BLAS-backed matmul) is in
 * rankByCosine() below, over N candidates at once. */
function embed(model, text) {
  const { tokenizer, net, seqLen, dim } = model;
  let ids = tokenizer.encode(text);
  if(ids.length > seqLen) ids = ids.slice(0, seqLen - 1).concat(ids[ids.length - 1]); // keep EOS

  const idsArr = new Int32Array(seqLen);
  const maskArr = new Int32Array(seqLen);
  const ttypeArr = new Int32Array(seqLen); // all zero: single-segment input
  for(let i = 0; i < ids.length; i++) {
    idsArr[i] = ids[i];
    maskArr[i] = 1;
  }

  const idsMat = new cv.Mat(1, seqLen, cv.CV_32S, idsArr.buffer);
  const maskMat = new cv.Mat(1, seqLen, cv.CV_32S, maskArr.buffer);
  const ttypeMat = new cv.Mat(1, seqLen, cv.CV_32S, ttypeArr.buffer);

  net.setInput(idsMat, 'input_ids');
  net.setInput(maskMat, 'attention_mask');
  net.setInput(ttypeMat, 'token_type_ids');
  const out = net.forward('last_hidden_state'); // shape [1, seqLen, dim], row-major token-major
  const data = out.data32F;

  const pooled = new Float64Array(dim);
  let maskSum = 0;
  for(let t = 0; t < seqLen; t++) {
    if(!maskArr[t]) continue;
    maskSum++;
    const base = t * dim;
    for(let d = 0; d < dim; d++) pooled[d] += data[base + d];
  }
  if(maskSum === 0) maskSum = 1;

  let norm = 0;
  for(let d = 0; d < dim; d++) {
    pooled[d] /= maskSum;
    norm += pooled[d] * pooled[d];
  }
  norm = Math.sqrt(norm) || 1;
  const vec = new Float32Array(dim);
  for(let d = 0; d < dim; d++) vec[d] = pooled[d] / norm;
  return vec;
}

/* --- embedding cache (keyed by Url, per model) --- */

function loadCache(model) {
  const raw = std.loadFile(CACHE_FILE_TMPL(model)); // null if the file doesn't exist (std.loadFile doesn't throw)
  if(!raw) return {};
  try {
    return JSON.parse(raw);
  } catch(error) {
    return {};
  }
}

function saveCache(model, cache) {
  const f = std.open(CACHE_FILE_TMPL(model), 'w');
  f.puts(JSON.stringify(cache));
  f.close();
}

function embedRows(model, modelName, rows, put) {
  const cache = loadCache(modelName);
  let computed = 0;
  const vectors = new Array(rows.length);

  for(let i = 0; i < rows.length; i++) {
    const url = rows[i][IDX.Url];
    let entry = cache[url];
    if(!entry) {
      const text = [rows[i][IDX.Thema], rows[i][IDX.Titel], rows[i][IDX.Beschreibung]].filter(Boolean).join(' - ');
      const vec = embed(model, text);
      entry = Array.from(vec);
      cache[url] = entry;
      computed++;
      if(computed % 200 === 0) std.err.puts(`embedding ${computed}/${rows.length}...\n`);
    }
    vectors[i] = entry;
  }

  if(computed) saveCache(modelName, cache);
  std.err.puts(`embedded ${computed} new entries (${rows.length - computed} from cache)\n`);
  return vectors;
}

/* --- cosine ranking via cv.Mat (per the user's ask to use qjs-opencv for the
 * vector math rather than hand-rolled JS). Note: Mat.mul() in this binding
 * (like opencv.js's own Mat.mul()) is cv::Mat::mul() -- elementwise, NOT a
 * real matrix multiply -- see BUGS: mat-mul-is-elementwise-not-matrix-
 * multiplication. Real matmul is cv.gemm(src1, src2, alpha, src3, beta, dst,
 * flags), the same free function opencv.js itself uses for it. */

function rankByCosine(vectors, queryVec, dim) {
  const n = vectors.length;
  const flat = new Float32Array(n * dim);
  for(let i = 0; i < n; i++) flat.set(vectors[i], i * dim);
  const candidatesMat = new cv.Mat(n, dim, cv.CV_32F, flat.buffer); // rows already L2-normalized
  const queryCol = new cv.Mat(dim, 1, cv.CV_32F, queryVec.buffer);  // also L2-normalized

  const sums = new cv.Mat(n, 1, cv.CV_32F);
  cv.gemm(candidatesMat, queryCol, 1, undefined, 0, sums); // (n x dim) * (dim x 1) = n x 1

  return Array.from(sums.data32F);
}

/* --- preference learning --- */

function loadPrefs() {
  const raw = std.loadFile(PREFS_FILE);
  if(!raw) return {};
  try {
    return JSON.parse(raw);
  } catch(error) {
    return {};
  }
}

function savePrefs(prefs) {
  const f = std.open(PREFS_FILE, 'w');
  f.puts(JSON.stringify(prefs, null, 2));
  f.close();
}

const STOPWORDS = new Set([
  'der', 'die', 'das', 'den', 'dem', 'des', 'ein', 'eine', 'einer', 'eines', 'und', 'oder',
  'von', 'im', 'in', 'am', 'an', 'zu', 'zum', 'zur', 'auf', 'fuer', 'für', 'mit', 'aus', 'bei',
]);

function tagsForRow(row) {
  const words = ((row[IDX.Thema] || '') + ' ' + (row[IDX.Titel] || ''))
    .split(/[^\p{L}]+/u)
    .filter(w => w.length >= 4 && !STOPWORDS.has(w.toLowerCase()));
  return [...new Set(words)];
}

function learn(url, weightBump, files) {
  let row = null;
  for(const file of files) {
    for(const r of readRows(file)) {
      if(r[IDX.Url] === url) { row = r; break; }
    }
    if(row) break;
  }
  if(!row) {
    std.err.puts(`--learn: no entry with Url '${url}' found in input\n`);
    std.exit(1);
  }

  const prefs = loadPrefs();
  const tags = tagsForRow(row);
  for(const tag of tags) prefs[tag] = (prefs[tag] || 0) + weightBump;
  savePrefs(prefs);
  std.err.puts(`learned from '${row[IDX.Titel]}': ${tags.join(', ')} (+${weightBump} each)\n`);
}

function tagBonus(row, prefs) {
  const hay = ((row[IDX.Thema] || '') + ' ' + (row[IDX.Titel] || '') + ' ' + (row[IDX.Beschreibung] || '')).toLowerCase();
  let bonus = 0;
  for(const tag in prefs) if(hay.includes(tag.toLowerCase())) bonus += prefs[tag];
  return bonus;
}

/* --- output --- */

function emitRow(put, first, r) {
  if(!first) put(',\n');
  put('"X":');
  const s = new JsonSerializer(r);
  let chunk;
  while((chunk = s.read(256)) !== '') put(chunk);
}

function main(...args) {
  const params = getOpt(
    {
      help: [false, () => usage(0), 'h'],
      query: [true, null, 'q'],
      model: [true, null],
      limit: [true, null],
      top: [true, null],
      learn: [true, null],
      weight: [true, null],
      output: [true, null, 'o'],
      '@': 'files',
    },
    args,
  );

  const modelName = params.model || 'minilm-multilingual';

  if(params.learn) {
    if(!params['@'].length) {
      // --learn has to scan a candidate file for the matching Url; unlike --query it's normally
      // run interactively (not piped), so silently defaulting to stdin here would just hang forever
      // waiting for input that's never coming.
      std.err.puts('mediathek-semantic.js: --learn needs an explicit candidate FILE argument (no stdin default)\n\n');
      usage(1);
    }
    learn(params.learn, params.weight ? parseFloat(params.weight) : 0.15, params['@']);
    return;
  }

  const files = params['@'].length ? params['@'] : ['-'];

  if(!params.query) {
    std.err.puts('mediathek-semantic.js: --query is required (or use --learn)\n\n');
    usage(1);
  }

  const limit = params.limit ? parseInt(params.limit) : 3000;
  const top = params.top ? parseInt(params.top) : 20;

  std.err.puts(`loading model '${modelName}'...\n`);
  const model = loadModel(modelName);

  std.err.puts('reading candidates...\n');
  let rows = [];
  for(const file of files) rows = rows.concat(readRows(file)); // not push(...readRows(file)): spread onto push blows QuickJS's argument-count limit for a full unfiltered dump
  std.err.puts(`${rows.length} candidate rows\n`);

  rows.sort((a, b) => +b[IDX.DatumL] - +a[IDX.DatumL]);
  if(rows.length > limit) rows = rows.slice(0, limit);
  std.err.puts(`ranking ${rows.length} (capped by --limit ${limit})\n`);

  const vectors = embedRows(model, modelName, rows, null);
  const queryVec = embed(model, params.query);
  const cosine = rankByCosine(vectors, queryVec, model.dim);

  const prefs = loadPrefs();
  const ranked = rows.map((row, i) => ({ row, score: cosine[i] + tagBonus(row, prefs) }));
  ranked.sort((a, b) => b.score - a.score);
  const results = ranked.slice(0, top);

  std.err.puts(`\ntop ${results.length} results for "${params.query}":\n\n`);
  for(const { row, score } of results) {
    std.err.puts(
      `${score.toFixed(3)}  [${row[IDX.Sender]}] ${row[IDX.Thema]} -- ${row[IDX.Titel]} ` +
        `(${row[IDX.Dauer]}, ${row[IDX.Datum]})\n`,
    );
  }

  const out = params.output ? std.open(params.output, 'w+') : std.out;
  const put = s => out.puts(s);
  put('{\n');
  emitRow(put, true, FIELDS);
  for(const { row } of results) emitRow(put, false, row);
  put('\n}\n');
  out.flush();
}

main(...scriptArgs.slice(1));
