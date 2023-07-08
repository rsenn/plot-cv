import filesystem from 'fs';
import PortableChildProcess from './lib/childProcess.js';
import * as path from './lib/path.js';
let filesystem,
  childProcess,
  documents = [];

function WriteFile(name, data) {
  if(Array.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;
  filesystem.writeFile(name, data + '\n');
  console.log(`Wrote ${name}: ${data.length} bytes`);
}

Util.define(Array.prototype, {
  contains(arg) {
    return this.indexOf(arg) != -1;
  }
});

function DummyPreproc(source) {
  let data = filesystem.readFileSync(source);

  let lines = data.split(/\n/g).map((line, i) => [i + 1, line]);
  let pp = lines.filter(([no, str]) => /^\s*#/.test(str));
  const includeRegex = /^\s*#\s*include\s+/g;
  let includeDirectives = pp.filter(([no, str]) => includeRegex.test(str));

  const removeQuotes = /^[<"](.*)[">]$/;
  let includeFiles = includeDirectives.map(([lineno, str]) => [lineno, str.replace(includeRegex, '')]);
  return includeFiles.map(([lineno, file]) => [lineno, file.replace(removeQuotes, '$1'), file.startsWith('<')]);
}

async function DumpDeps(sources, includeDirs = []) {
  //let stderr = filesystem.open('deps.err', 'w+');$
  let includes = includeDirs.reduce((acc, dir) => (/^\/opt/.test(dir) ? [...acc, '-isystem', dir] : [...acc, `-I${dir}`]), []);
  let argv = ['-MM', ...includes, '-c', ...sources];
  console.log('argv:', argv.join(' '));
  let proc = childProcess('g++', argv, {
    block: false,
    stdio: [null, 'pipe', 'pipe']
  });
  let out = filesystem.readAll(proc.stdout);
  let err = filesystem.readAll(proc.stderr);
  filesystem.close(proc.stdout);
  filesystem.close(proc.stderr);
  out = out.replace(/\\\n\s*/g, '');
  WriteFile('deps.mk', out);
  let i = 0;
  let ret = [];
  let lines = out.split(/\n/g);
  for(let line of lines) {
    let source = sources[i++];
    let idx = line.indexOf(': ');
    if(idx == -1) continue;
    let obj = line.substring(0, idx - 2);
    let deps = line.substring(idx + 2).split(/\s+/g);
    if(deps[0] == source) deps.shift();
    deps.sort();
    deps = deps.map(dep => path.normalize(dep));
    ret.push([source, deps]);
  }
  return new Map(ret);
}

async function main(...sources) {
  await PortableChildProcess(cp => (childProcess = cp));

  const env = await Util.env;
  const includes = ['quickjs', 'src', '/opt/opencv-4.5.0/include', '/opt/opencv-4.5.0/include/opencv4'];

  if(sources.length == 0) {
    let files = filesystem.readdir('src').map(file => `src/${file}`);
    sources = files.filter(path => /\.[ch]/.test(path));
    sources.sort();
  }

  console.log(`DumpDeps(`, sources, `)`);

  let deps = await DumpDeps(sources, includes);
  console.log(`DumpDeps() = `, deps);

  for(let [source, depfiles] of deps) {
    let pp = DummyPreproc(source);

    console.log(`${source} deps:`, depfiles);
    console.log(`${source} pp:`, pp);
  }
}

main(...scriptArgs.slice(1));