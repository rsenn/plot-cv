import fs from 'fs';
import { spawn } from 'child_process';
import deep from './lib/deep.js';
import { ReadAll } from './lib/stream/utils.js';
import { ImmutablePath } from './lib/json.js';
import { memoize, define } from './lib/misc.js';
import * as path from './lib/path.js';

let documents = [];

function WriteFile(name, data) {
  if(Array.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;

  fs.writeFileSync(name, data + '\n');

  console.log(`Wrote ${name}: ${data.length} bytes`);
}

class Location {
  static primary = null;
  static contents = memoize(file => fs.readFileSync(file).toString());

  constructor(file, begin, end) {
    let data = Location.contents(file);
    this.line = begin.line;
    this.col = begin.col;
    this.string = data.substring(+begin, +end + end.tokLen);
  }

  static from(obj) {
    const { file, line, col, offset, ...rest } = obj;
    let ret = Object.setPrototypeOf(obj, Location.prototype);
    ret.string = ret.token;
    if(!('line' in ret))
      ret.line = Location.contents(file || Location.primary)
        .substring(0, obj.offset)
        .split(/\n/g).length;
    if(!('file' in ret)) ret.file = Location.primary;
    ret.file = path.relative(process.cwd(), ret.file);
    if('includedFrom' in ret) ret.includedFrom = path.relative(process.cwd(), ret.includedFrom);

    return ret;
  }

  /* prettier-ignore */ get token() {
    const { file, offset, tokLen } = this;
    let data = Location.contents(file || Location.primary);
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

  static isLocation(obj) {
    return isObject(obj) && typeof obj.offset == 'number';
  }

  toString() {
    const { file = Location.primary, line, col, offset } = this;

    let ret = [];
    /* console.log('file:', file);
    console.log('process.cwd():', process.cwd());*/
    if(typeof file == 'string') ret.unshift(file);
    if(line && col) ret = ret.concat([line, col]);
    else ret.push(offset);
    return ret.join(':');
  }
}

function NodeToString(node, locKey = 'expansionLoc' || 'spellingLoc', startOffset) {
  if(node.value) return node.value;
  let flat = deep.flatten(
    node,
    new Map(),
    (v, k) => isObject(v),
    (k, v) => [k, v],
  );
  let l = [];
  for(let [k, v] of flat) {
    if(isObject(v) && 'offset' in v && typeof v.offset == 'number') {
      const loc = Location.from(deep.get(node, k));
      // if(k[k.length - 1] == locKey) continue;
      if(loc.file == Location.primary || !loc.file) l.push([k, loc]);
      deep.set(node, k, loc);
    }
  }
  let { begin, end } = node.range || {};
  if(begin && begin[locKey]) begin = begin[locKey];
  if(end && end[locKey]) end = end[locKey];
  l = unique(
    l.sort((a, b) => a[1].offset - b[1].offset),
    (a, b) => Location.equal(a[1], b[1]),
  );
  if(!l.length && node.name) return node.name;
  if(l.length > 1) {
    begin = l[0][1];
    end = l[l.length - 1][1];
  }
  const loc = new Location(begin.file, begin, end);
  return loc.string;
}

function* GetRanges(tree) {
  for(let [node, path] of deep.iterate(tree, v => isObject(v) && v.begin)) yield [path.join('.'), node];
}

function* GetLocations(tree) {
  for(let [node, path] of deep.iterate(tree, v => isObject(v) && typeof v.offset == 'number'))
    yield [path.join('.'), node];
}

function* GetNodes(tree, pred = n => true) {
  for(let [node, path] of deep.iterate(
    tree,
    (v, p) => isObject(v) && typeof v.kind == 'string' && v.kind != 'TranslationUnitDecl' && pred(v, p),
  ))
    yield [path.join('.'), node];
}

function GetLocation(node) {
  let loc = node.range || node.loc;
  if(!isObject(loc)) {
    loc = {};
  }
  if(loc.begin) loc = loc.begin;
  if(loc.expansionLoc) loc = loc.expansionLoc;
  if(isEmpty(loc)) return null;
  return loc;
}

function IsStruct(node) {
  if(Array.isArray(node.inner)) {
    if(node.inner.some(child => child.kind == 'FieldDecl')) return true;
  }
}

function ContainsDecls(node) {
  if(Array.isArray(node.inner)) {
    if(node.inner.some(child => /Decl/.test(child.kind + ''))) return true;
  }
}

function GetProperty(ast, key, pred) {
  let k = [...key];
  do {
    let node = deep.get(ast, k);
    let value = pred(node);
    if(value) return value;

    k.pop();
  } while(k.length > 0);
}

function GetParent(ast, key) {
  return deep.get(ast, key.slice(0, -1));
}

function GetOwned(ast, key) {
  key = [...key];

  if(key[key.length - 1] == 'ownedTagDecl') key = key.slice(0, -1);
  return [key, deep.get(ast, key)];
}

function GetHeight(key) {
  return key.filter(prop => prop == 'inner').length;
}

function GetDepth(node) {
  let maxLen = 0;
  for(let [v, k] of deep.iterate(node, v => isObject(v))) {
    let n = k.filter(prop => prop == 'inner').length;
    if(n > maxLen) maxLen = n;
  }
  return maxLen;
}

function GetKey([k, v]) {
  return k;
}

function GetValue([k, v]) {
  return v;
}

function GetValueKey([v, k]) {
  return [k, v];
}

function RelativeTo(to, k) {
  if(k.startsWith(to)) return k.slice(to.length);
  return k;
}

function GetNodeProps([k, v]) {
  let props = [...getMemberNames(v)].map(n => [n, v[n]]).filter(([n, v]) => !isObject(v) && v != '');

  if(props.filter(([prop, value]) => prop != 'kind').length) return Object.fromEntries(props);
}

function GetNodeTypes(ast, [k, v]) {
  let ret = [];
  let prev = [];
  for(let i = 1; i <= k.length; i++) {
    let key = k.slice(0, i);
    let node = deep.get(ast, key);

    if(isObject(node) && typeof node.kind == 'string') {
      ret = ret.concat([new ImmutablePath(RelativeTo(prev, key)), GetNodeProps([key, node]) || node.kind]);
      prev = key;
    }
  }
  return ret;
}

function GetNodeChildren(ast, [k, v]) {
  let children = [...deep.iterate(v, (v, p) => isObject(v))].map(GetValueKey);
  children = children.filter(([key, child]) => typeof child.kind == 'string' && child.kind != '');

  return children.reduce(
    (acc, [key, child]) => [...acc, new ImmutablePath(key), GetNodeProps([key, child]) || child],
    [],
  );
}

function GetNameOrId(ast, [key, node], pred = id => id != '') {
  let [k, n] = GetOwned(ast, key);
  let paths = [['name']];
  if(node.ownedTagDecl) paths = [['ownedTagDecl', 'name'], ['ownedTagDecl', 'id'], ...paths];
  for(let prop of [...paths, ['id']]) {
    let value = deep.get(n, prop);
    if(typeof value == 'string' && pred(value)) return [[...k, ...prop], value];
  }
  let names = [...deep.iterate(n, (v, p) => p[p.length - 1] == 'name' && typeof v == 'string' && v != '', [...k])].map(
    GetValueKey,
  );

  let ids = [...deep.iterate(n, (v, p) => p[p.length - 1] == 'id' && v, [...k])].map(GetValueKey);

  return [...names, ...ids].filter(([k, v]) => pred(v))[0];
}

function GetTypeStr(node) {
  if(isObject(node.type)) {
    if(node.type.qualType) return node.type.qualType;
    return node.type;
  }
  if(node.isImplicit) return Symbol.for('implicit');
}

function GetRecord(node) {
  if(isObject(node) && Array.isArray(node.inner))
    return new Map(
      node.inner.map(field => [field.name, ContainsDecls(field) ? GetRecord(field) : GetTypeStr(field) || field]),
    );
}

define(Array.prototype, {
  contains(arg) {
    return this.indexOf(arg) != -1;
  },
  startsWith(other) {
    if(other.length > this.length) return false;
    for(let i = 0; i < other.length; i++) if(this[i] != other[i]) return false;
    return true;
  },
  endsWith(other) {
    if(other.length > this.length) return false;
    let start = this.length - other.length;
    for(let i = 0; i < other.length; i++) if(this[i + start] != other[i]) return false;
    return true;
  },
});

async function DumpAst(source) {
  let outfile = path.basename(source) + '.ast.json';
  let stat = { in: fs.stat(source), out: fs.stat(outfile) };
  let data;

  if(stat.in.mtimeMs > stat.out.mtimeMs) {
    console.log(`Generating '${outfile}' ...`);

    let stderr = fs.open('ast.err', 'w+');
    let proc = spawn('clang', ['-Xclang', '-ast-dump=json', '-fsyntax-only', source], {
      block: false,
      stdio: [null, 'pipe', stderr],
    });
    data = await ReadAll(proc.stdout);
    fs.close(stderr);
    WriteFile(outfile, data);
  } else {
    console.log(`Reading '${outfile}' ...`);
    data = fs.readFileSync(outfile);
  }
  return JSON.parse(data);
}

function processCallExpr(loc, func, ...args) {
  const fmtIndex = args.findIndex(a => a.startsWith('"'));
  if(fmtIndex == -1) return;
  const fmtStr = args[fmtIndex];
  const fmtArgs = args.slice(fmtIndex + 1);
  let matches = [...matchAll(/(%([-#0 +'I]?)([0-9.]*)[diouxXeEfFgGaAcspnm%](hh|h|l|ll|q|L|j|z|Z|t|))/g, fmtStr)];
  let ranges = [];
  let last = 0;
  for(let match of matches) {
    if(match.index > last) ranges.push([last, match.index]);
    ranges.push([match.index, (last = match.index + match[0].length)]);
  }
  if(last < fmtStr.length) ranges.push([last, fmtStr.length]);
  let parts = ranges.map(r => fmtStr.substring(...r));

  //console.log('ranges:', ranges);
  //console.log('parts:', parts);
}

const typeRe =
  /^(array|buffer|build_type_t|config_t|dirs_t|dir_t|exts_t|fd_t|HMAP_DB|int64|intptr_t|lang_type|machine_type|MAP_NODE_T|MAP_PAIR_T|MAP_T|os_type|range|rdir_t|set_iterator_t|set_t|sighandler_t_ref|sigset_t|sourcedir|sourcefile|ssize_t|stralloc|strarray|strlist|system_type|target|tools_t|TUPLE|uint32|uint64)$/;

async function main(...args) {
  const cols = await getEnv('COLUMNS');
  // console.log('cols:', cols, process.env.COLUMNS);

  if(args.length == 0) args.unshift('/home/roman/Sources/c-utils/genmakefile.c');

  for(let arg of args) {
    Location.primary = arg;
    let ast = await DumpAst(arg);
    let flat;
    const generateFlat = () =>
      (flat = deep.flatten(
        ast,
        new Map(),
        (v, k) => k.indexOf('range') == -1 && isObject(v),
        (k, v) => [k, v],
      ));
    generateFlat();
    let l = deep.flatten(ast, new Map(), (v, k) => isObject(v) && typeof v.col == 'number').values();
    let line;
    const re =
      /^(__fbufsize|__flbf|__fpending|__fpurge|__freadable|__freading|__fwritable|clearerr|clearerr_unlocked|fclose|fdopen|feof|feof_unlocked|ferror|ferror_unlocked|fflush|fflush_unlocked|fgetc|fgetc_unlocked|fgetpos|fgets|fgets_unlocked|fileno|fileno_unlocked|fopen|fprintf|fputc|fputc_unlocked|fputs|fputs_unlocked|fread|fread_unlocked|freopen|fscanf|fseek|fseeko|fsetpos|ftell|ftello|fwrite|fwrite_unlocked|printf|putchar|puts|scanf|setvbuf|tmpfile|ungetc|vfprintf|vfscanf|vprintf|vscanf)$/;
    let allf = [...flat].filter(([k, v]) => isObject(v) && v.kind == 'CallExpr');
    let allst = [...flat].filter(
      ([k, v]) => isObject(v) && IsStruct(v) && typeRe.test(GetProperty(ast, k, v => v.name)),
    );
    let incfrom = [...flat].filter(([k, v]) => k[k.length - 1] == 'includedFrom');

    incfrom.forEach(([k, v]) => deep.set(ast, k, v.file));
    //incfrom.forEach(([k,v])  => flat.set( k, v.file));
    //

    let refIds = [...flat].filter(([k, v]) => isObject(v) && typeof v.id == 'string').map(([k, v]) => [k, v.id, v]);

    refIds.sort((a, b) => a[1] - b[1]);

    let ids = refIds.filter(([k, id, v]) => ContainsDecls(v)).map(([k, id, v]) => id);
    ids = unique(ids);
    let idLists = new Map(
      ids
        .map(id => [
          id,
          refIds
            .filter(([k, other, v]) => id == other)
            .map(([k, other, v]) => [GetHeight(k), ...GetOwned(ast, k)])
            .map(([h, k, v]) => [
              h,
              GetDepth(v),
              new ImmutablePath(k),
              GetNameOrId(ast, [k, v], id2 => id2 != '' && id != id2),
              GetRecord(v) || v,
              GetNodeTypes(ast, [k, v]),
              GetNodeChildren(ast, [k, v]),
            ])
            .filter(([h, d, k, name, v]) => !/FunctionDecl/.test(v.kind + '')),
        ])
        .filter(([id, keys]) => keys.length > 1),
    );

    console.log('ids:', ids);
    console.log('idLists:', idLists);

    //
    generateFlat();

    /*    let locs = [...flat].filter(([k, v]) => Location.isLocation(v));*/
    let locMap = new WeakMap();
    let node, loc, prop;

    for(let [k, v] of flat) {
      if(Location.isLocation(v)) {
        loc = Object.setPrototypeOf(v, Location.prototype);
        deep.unset(ast, k);
        //  flat.delete(k);
        let key = [...k];

        while(key.length >= 1) {
          prop = key.pop();
          node = deep.get(ast, key);
          //  if(locMap.get(node)) break;
          locMap.set(node, loc);
          deep.unset(ast, [...key, prop]);
          delete node[prop];
        }
      } /* else if(!locMap.get(v) && loc) {
        locMap.set(v, loc);
      }*/
    }

    generateFlat();

    //  incfrom = [...flat].filter(([k, v]) => k[k.length - 1] == 'includedFrom');
    //   console.log('incfrom:', incfrom);
    let localfiles = [...flat].map(([k, v]) => [k, locMap.get(v)]).filter(([k, v]) => !!v);

    // console.log('localfiles:', localfiles);
    localfiles = localfiles.filter(([k, l]) => (typeof l.file == 'string' ? l.file.startsWith('/home') : true));
    localfiles = localfiles.map(([k, l]) => [k, flat.get(k)]);

    //console.log('localfiles:', localfiles);
    let keys = localfiles.map(([k, v]) => k.slice(0, k.indexOf('loc')));

    for(let k of flat.keys()) {
      if(!keys.some(m => k.startsWith(m))) {
        flat.delete(k);
        //  console.log("delete:",k);}
      }
    }
    let localAst = deep.unflatten(flat);

    /// console.log('locs:', [...flat].map(([k, v]) => [k, locMap.get(v)]).filter(([k,v]) => !! v));

    let types = [...flat].filter(([k, v]) => isObject(v) && (('name' in v && typeRe.test(v.name)) || IsStruct(v)));
    let typeKeys = types.map(([k, v]) => k);

    console.log(
      'types:',
      types
        .filter(([k, v]) => IsStruct(v))
        .map(([k, v]) => [k, GetProperty(ast, k, v => v.name) || v.id, IsStruct(v) ? GetRecord(v) : v, locMap.get(v)]),
    );
    //    console.log('types keys:', typeKeys);
    /* console.log('type fields: ',
      [...flat].filter(([k, v]) => typeKeys.some(j => k.startsWith(j))
        ).filter(([k, v]) => /Field/.test(v.kind + ''))
    );*/

    let fmtfns = [...flat].filter(([k, v]) => typeof v == 'string' && re.test(v)).map(([k, v]) => k.slice(0, -1));
    //console.log('allf:', allf.map(([k, v]) =>   NodeToString(v, 'expansionLoc')));
    console.log(
      'allst:',
      allst.map(([k, v]) => [
        v.name,
        k.join('.'),
        deep.get(ast, k.slice(0, 4)),
      ]) /*.filter(([loc,st]) => loc.file.startsWith(process.cwd))*/,
    );
    let fmtc = fmtfns
      .map(k => {
        for(let i = k.length - 1; i > 0; i--) {
          let l = k.slice(0, i);
          let n = deep.get(ast, l);
          if(n.kind == 'CallExpr') return l;
        }
        return null;
      })
      .filter(k => !!k)
      .map(k => [k, clone(deep.get(ast, k))]);
    for(let [key, value] of fmtc) {
      const loc = value.range.begin.spellingLoc || value.range.begin;
      const call = NodeToString(value, 'spellingLoc');
      let flat = deep.flatten(
        value,
        new Map(),
        (v, k) => isObject(v) && k.length < 10,
        (k, v) => [k, v],
      );
      let inner = value.inner.map((n, i) => NodeToString(n, 'spellingLoc', i > 0 ? loc.offset : 0));
      processCallExpr(loc, ...inner);
      //    console.log('inner:',  inner);
    }
    //console.log('ranges:', [...GetNodes(ast, (n, k) => n.kind != 'TranslationUnitDecl' && GetLocation(n) && !GetLocation(n).includedFrom && [undefined, null, arg].contains(GetLocation(n).file))]);
  }
}

main(...scriptArgs.slice(1));
