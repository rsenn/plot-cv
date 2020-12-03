import { EagleDocument, EagleProject } from './lib/eagle.js';
import PortableFileSystem from './lib/filesystem.js';
import { LineList, Rect } from './lib/geom.js';
import { toXML } from './lib/json.js';
import Util from './lib/util.js';
import deep from './lib/deep.js';
import { Graph } from './lib/fd-graph.js';
import ptr from './lib/json-ptr.js';
import LogJS from './lib/log.js';
import ConsoleSetup from './lib/consoleSetup.js';
import tXml from './lib/tXml.js';
import PortableChildProcess, { SIGTERM, SIGKILL, SIGSTOP, SIGCONT } from './lib/childProcess.js';
import { Reader, ReadAll } from './lib/stream/utils.js';
import { Repeater } from './lib/repeater/repeater.js';

let filesystem,
  childProcess,
  documents = [];

function WriteFile(name, data) {
  if(Util.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;

  filesystem.writeFile(name, data + '\n');

  console.log(`Wrote ${name}: ${data.length} bytes`);
}

class Location {
  static primary = null;
  static contents = Util.memoize(file => filesystem.readFile(file).toString());

  constructor(file, begin, end) {
    let data = Location.contents(file);
    this.line = begin.line;
    this.col = begin.col;
    //  console.log('end:', end);
    this.string = data.substring(+begin, +end + end.tokLen);
  }

  static from(obj) {
    const { file, line, col, offset, ...rest } = obj;
    let ret = Object.setPrototypeOf(/*Util.define({  file, line, col, offset }, { ...obj })*/ obj, Location.prototype);
    ret.string = ret.token;
    if(!('line' in ret))
      ret.line = Location.contents(file || Location.primary)
        .substring(0, obj.offset)
        .split(/\n/g).length;
    if(!('file' in ret)) ret.file = Location.primary;

    return ret;
  }

  get token() {
    const { file, offset, tokLen } = this;
    let data = Location.contents(file || Location.primary);
    //console.log("data:", data);
    return data.substring(offset, offset + tokLen);
  }

  valueOf() {
    return this.offset;
  }

  [Symbol.toPrimitive](hint) {
    if(hint == 'string') return this.toString();
    return this.offset;
  }

  static equal(a, b) {
    return a.file == b.file && a.offset == b.offset && a.tokLen == b.tokLen;
  }

  toString() {
    return `${this.file}:${this.line}:${this.col}`;
  }
}

function NodeToString(node, locKey = 'expansionLoc' || 'spellingLoc', startOffset) {
  if(node.value) return node.value;
  let flat = deep.flatten(node,
    new Map(),
    (v, k) => Util.isObject(v),
    (k, v) => [k, v]
  );
  let locations = [];
  for(let [k, v] of flat) {
    if(Util.isObject(v) && 'offset' in v && typeof v.offset == 'number') {
      const loc = Location.from(deep.get(node, k));
      // if(k[k.length - 1] == locKey) continue;
      if(loc.file == Location.primary || !loc.file) locations.push([k, loc]);
      deep.set(node, k, loc);
    }
  }
  let { begin, end } = node.range || {};
  if(begin && begin[locKey]) begin = begin[locKey];
  if(end && end[locKey]) end = end[locKey];
  locations = Util.unique(locations.sort((a, b) => a[1].offset - b[1].offset),
    (a, b) => Location.equal(a[1], b[1])
  );
  if(!locations.length && node.name) return node.name;
  if(locations.length > 1) {
    begin = locations[0][1];
    end = locations[locations.length - 1][1];
  }
  const loc = new Location(begin.file, begin, end);
  return loc.string;
}

function *GetRanges(tree) {
  for(let [node,path] of deep.iterate(tree, v => Util.isObject(v) && v.begin))
    yield [path.join('.'),node];
}

function *GetLocations(tree) {
  for(let [node,path] of deep.iterate(tree, v => Util.isObject(v) && typeof(v.offset) == 'number'))
    yield [path.join('.'),node];
}

function *GetNodes(tree, pred = n => true) {
  for(let [node,path] of deep.iterate(tree, (v,p) => Util.isObject(v) && typeof(v.kind) == 'string' && pred(v, p)))
    yield [path.join('.'),node];
}
function  GetLocation(node) {
  let loc = node.range || node.loc;

if(!Util.isObject(loc)) {
  console.log("node:", node);
  return {};
}

  if(loc.begin)
    loc = loc.begin;

  if(loc.expansionLoc)
    loc = loc.expansionLoc;

return loc;
}

Array.prototype.contains = function(arg) {
  return this.indexOf(arg) != -1;
};

async function DumpAst(source) {
  let stderr = filesystem.open('ast.err', 'w+');
  let proc = childProcess('clang', ['-Xclang', '-ast-dump=json', '-fsyntax-only', source], {
    block: false,
    stdio: [null, 'pipe', stderr]
  });

  //        await proc.wait();
  //  console.log('proc.stdout:', proc.stdout);
  let data = await ReadAll(proc.stdout);
  filesystem.close(stderr);
  WriteFile('ast.json', data);
  return JSON.parse(data);
}

function processCallExpr(loc, func, ...args) {
  const fmtIndex = args.findIndex(a => a.startsWith('"'));

  if(fmtIndex == -1) return;
  const fmtStr = args[fmtIndex];
  const fmtArgs = args.slice(fmtIndex + 1);

  let matches = [...Util.matchAll(/(%([-#0 +'I]?)([0-9.]*)[diouxXeEfFgGaAcspnm%](hh|h|l|ll|q|L|j|z|Z|t|))/g, fmtStr)];
  let ranges = [];
  let last = 0;
  for(let match of matches) {
    if(match.index > last) ranges.push([last, match.index]);
    ranges.push([match.index, (last = match.index + match[0].length)]);
  }
  if(last < fmtStr.length) ranges.push([last, fmtStr.length]);
  let parts = ranges.map(r => fmtStr.substring(...r));

  console.log('ranges:', ranges);
  console.log('parts:', parts);
}


async function main(...args) {
  const cols = await Util.getEnv('COLUMNS');
  console.log('cols:', cols, process.env.COLUMNS);
  await ConsoleSetup({ colors: true, depth: 8, breakLength: 138 });
  await PortableFileSystem(fs => (filesystem = fs));
  await PortableChildProcess(cp => (childProcess = cp));

  if(args.length == 0) args.unshift('/home/roman/Sources/c-proxy/tsproxy/transockproxy.c');

  for(let arg of args) {
    Location.primary = arg;
    let ast = await DumpAst(arg);
    let flat = deep.flatten(ast,
      new Map(),
      (v, k) => k.indexOf('range') == -1,
      (k, v) => [k, v]
    );
    let locations = deep.flatten(ast, new Map(), (v, k) => Util.isObject(v) && typeof v.col == 'number').values();
    let line;
    /* for(let loc of locations) {
      if(typeof(loc.line) == 'number')
        line = loc.line;
      else
        loc.line = line;
    }*/

    //console.log({v,k}) /*[k.split('.'), v]*/);

    const re = /^(__fbufsize|__flbf|__fpending|__fpurge|__freadable|__freading|__fwritable|clearerr|clearerr_unlocked|fclose|fdopen|feof|feof_unlocked|ferror|ferror_unlocked|fflush|fflush_unlocked|fgetc|fgetc_unlocked|fgetpos|fgets|fgets_unlocked|fileno|fileno_unlocked|fopen|fprintf|fputc|fputc_unlocked|fputs|fputs_unlocked|fread|fread_unlocked|freopen|fscanf|fseek|fseeko|fsetpos|ftell|ftello|fwrite|fwrite_unlocked|printf|putchar|puts|scanf|setvbuf|tmpfile|ungetc|vfprintf|vfscanf|vprintf|vscanf)$/;
    let allFunctions = [...flat].filter(([k, v]) => Util.isObject(v) && v.kind == 'CallExpr');
    let formattedFunctions = [...flat]
      .filter(([k, v]) => typeof v == 'string' && re.test(v))
      .map(([k, v]) => k.slice(0, -1));
    console.log('allFunctions:',
      allFunctions.map(([k, v]) => NodeToString(v, 'expansionLoc'))
    );

    let formattedCalls = formattedFunctions
      .map(k => {
        for(let i = k.length - 1; i > 0; i--) {
          let l = k.slice(0, i);
          let n = deep.get(ast, l);

          if(n.kind == 'CallExpr') return l;
        }
        return null;
      })
      .filter(k => !!k)
      .map(k => [k, Util.clone(deep.get(ast, k))]);
    //console.log('formattedCalls:', formattedCalls);
    for(let [key, value] of formattedCalls) {
      const loc = value.range.begin.spellingLoc || value.range.begin;

      const call = NodeToString(value, 'spellingLoc');
      let flatMap = deep.flatten(value,
        new Map(),
        (v, k) => Util.isObject(v) && k.length < 10,
        (k, v) => [k, v]
      );
      let inner = value.inner.map((n, i) => NodeToString(n, 'spellingLoc', i > 0 ? loc.offset : 0));

      processCallExpr(loc, ...inner);

      console.log('inner:', /*value.*/ inner);
      //  console.log('nodes:', nodes.map(([k, n]) => [k.join('.'), n.kind , NodeToString(n)]));
    }
  await ConsoleSetup({ colors: true, depth: 4, breakLength: 138 });

    console.log("ranges:", [...GetNodes(ast, n => GetLocation(n).file == arg)]);
  }
}

Util.callMain(main, true);
