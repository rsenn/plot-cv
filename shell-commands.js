#!/usr/bin/env qjsm
import { getOpt } from 'util';
import * as std from 'std';
import * as os from 'os';
import { JsonParser } from 'json';
import { basename } from 'path';

/* The special builtins (B_SPECIAL in shish's builtin_table.c): errors in them abort a non-interactive shell. */
const SPECIAL = new Set(['.', ':', 'break', 'continue', 'eval', 'exit', 'export', 'set', 'shift', 'readonly', 'return', 'source', 'times', 'unset', 'trap']);

/* The POSIX utilities as listed in POSIX's Shell & Utilities volume; `colon` and `dot` are the
 * volume's page names for `:` and `.`, so those are added by their real names. */
const POSIX = new Set(
  (
    '[,admin,alias,ar,asa,at,awk,basename,batch,bc,bg,break,builtins-redirector,c17,cal,cat,cd,cflow,chgrp,chmod,chown,cksum,cmp,colon,comm,command,compress,contents,continue,cp,crontab,csplit,ctags,cut,cxref,date,dd,delta,df,diff,dirname,dot,du,echo,ed,env,eval,ex,exec,exit,expand,export,expr,false,fc,fg,file,find,fold,fuser,gencat,get,getconf,getopts,gettext,grep,hash,head,iconv,id,ipcrm,ipcs,jobs,join,kill,lex,link,ln,locale,localedef,logger,logname,lp,ls,m4,mailx,make,man,mesg,mkdir,mkfifo,more,msgfmt,mv,newgrp,ngettext,nice,nl,nm,nohup,od,paste,patch,pathchk,pax,pr,printf,prs,ps,pwd,read,readlink,readonly,realpath,renice,return,rm,rmdel,rmdir,sact,sccs,sed,set,sh,shift,sleep,sort,split,strings,strip,stty,tabs,tail,talk,tee,test,time,timeout,times,toc,touch,tput,tr,trap,true,tsort,tty,type,ulimit,umask,unalias,uname,uncompress,unexpand,unget,uniq,unlink,unset,uucp,uudecode,uuencode,uustat,uux,val,vi,wait,wc,what,who,write,xargs,xgettext,yacc,zcat' +
    ',:,.'
  ).split(','),
);

/* Names of the executables in the PATH directories: the only commands classified as external. */
function pathCommands() {
  const names = new Set();

  for(const dir of (std.getenv('PATH') || '').split(':')) {
    if(!dir) continue;

    const [entries, err] = os.readdir(dir);
    if(err !== 0) continue;

    for(const name of entries) {
      const [st, statErr] = os.stat(`${dir}/${name}`);

      if(statErr === 0 && (st.mode & os.S_IFMT) === os.S_IFREG && st.mode & 0o111) names.add(name);
    }
  }

  return names;
}

/* Names of shish's builtins, taken from the synopsis lines of `shish -c help`: one "name args"
 * entry per line when the terminal is narrow, two per line when it is wide. Entries contain single
 * spaces themselves, so column two is found by its start offset, which is the same on every row
 * that has one, instead of by splitting on whitespace. */
function shishBuiltins(help) {
  if(help === undefined) {
    const proc = std.popen('shish -c help', 'r');
    help = proc.readAsString();
    proc.close();
  }

  const lines = help.split('\n').filter(line => line.trim());
  const starts = new Map();

  for(const line of lines) {
    const gap = / {2,}(?=\S)/.exec(line);
    if(gap) bump(starts, gap.index + gap[0].length, 1);
  }

  const column2 = [...starts].sort((a, b) => b[1] - a[1])[0]?.[0] ?? Infinity;
  const names = new Set();

  for(const line of lines)
    for(const entry of [line.slice(0, column2), line.slice(column2)]) {
      const name = entry.trim().split(/\s+/)[0];
      if(name) names.add(name);
    }

  return names;
}

function classify(name, builtins, externals) {
  if(name === '[[') return 'keyword';
  if(SPECIAL.has(name)) return 'special';
  if(builtins.has(name)) return 'builtin';

  return externals.has(name) ? 'external' : 'notfound';
}

function shellQuote(s) {
  return `'${s.replace(/'/g, `'\\''`)}'`;
}

/* Bigger scripts are mostly self-extracting installers with a binary payload appended, which
 * take forever to fail to parse. */
const MAX_SCRIPT_BYTES = 1024 * 1024;

/* Tool caches (.cargo, .cache, .git, ...) hold vendored copies of scripts, not the user's own. */
const HIDDEN_DIR = /\/\./;

/* Test suites (git's t/, coreutils' tests/, the POSIX suite in OpenGroup) call every builtin and
 * utility on purpose, and the kernel header trees are dozens of near-identical selftest/build
 * scripts: both skew the ranking away from how scripts are really written. */
const TEST_PATH = /\/(tests?|testsuites?|test-suite|t|tset|selftests?|regress(ion)?|testdata|autotest|unittests?|OpenGroup)\//i;
const TEST_NAME = /\/(t\d{3,}[-_.][^/]*|test[-_][^/]*|[^/]*[-_]test)\.sh$/i;
const KERNEL_TREE = /^\/usr\/src\/linux-/;

/* Without a shell shebang a .sh file is a sourced fragment or a template, not a script that runs on
 * its own. Autoconf inputs are shell mixed with m4 and never have one. */
const SHEBANG = /^#!\s*\S*\/(env\s+(-\S+\s+)*)?(ba|da|k|z|a)?sh\b/;
const AUTOCONF_INPUT = /\/configure\.(in|ac)$/;

/* Generated configure scripts are the same tens of thousands of lines of autoconf boilerplate in
 * every project; counting each distinct command once keeps one of them from outweighing the rest. */
const GENERATED_CONFIGURE = /\/configure$/;

function fnv(s) {
  let h = 0x811c9dc5;
  for(let i = 0; i < s.length; i++) h = Math.imul(h ^ s.charCodeAt(i), 0x01000193) >>> 0;
  return `${s.length}:${h}`;
}

/* `seen` holds the hashes of scripts already accepted, so vendored copies count once. */
function isWanted(path, excludes, seen) {
  if(HIDDEN_DIR.test(path) || TEST_PATH.test(path) || TEST_NAME.test(path) || KERNEL_TREE.test(path)) return false;
  if(excludes.some(re => re.test(path))) return false;

  const [st, err] = os.stat(path);
  if(err !== 0 || (st.mode & os.S_IFMT) !== os.S_IFREG || st.size === 0 || st.size > MAX_SCRIPT_BYTES) return false;

  const text = std.loadFile(path);
  if(text === null || (!AUTOCONF_INPUT.test(path) && !SHEBANG.test(text.split('\n', 1)[0]))) return false;

  const hash = fnv(text);
  if(seen.has(hash)) return false;

  seen.add(hash);
  return true;
}

/* Real ASTs are far below this; shparse2ast can emit output without end (see BUGS). */
const MAX_AST_BYTES = 64 * 1024 * 1024;

/* Builds the value from JsonParser tokens instead of using JsonPushParser: qjs-modules builds from
 * before jread's go_stack became growable segfault on nesting deeper than 64, which long elif
 * chains in real scripts exceed. */
function readAst(proc) {
  let total = 0;
  const parser = new JsonParser({
    read(buf, len) {
      const n = proc.read(buf, 0, len);
      if((total += n) > MAX_AST_BYTES) throw new Error('AST too large');
      return n;
    },
  });
  const stack = [];
  let key;
  let root;

  const put = value => {
    const top = stack[stack.length - 1];
    if(!top) root = value;
    else if(Array.isArray(top)) top.push(value);
    else top[key] = value;
  };

  for(;;) {
    const tok = parser.parse();

    switch (tok) {
      case 'NEED_DATA':
      case 'NONE':
        return stack.length ? null : root;
      case 'OBJECT':
      case 'ARRAY': {
        const container = tok === 'OBJECT' ? {} : [];
        put(container);
        stack.push(container);
        break;
      }
      case 'OBJECT_END':
      case 'ARRAY_END':
        stack.pop();
        break;
      case 'KEY':
        key = parser.token;
        break;
      case 'STRING':
        put(parser.token);
        break;
      case 'NUMBER':
        put(+parser.token);
        break;
      case 'TRUE':
      case 'FALSE':
      case 'NULL':
        put(tok === 'TRUE' ? true : tok === 'FALSE' ? false : null);
        break;
    }
  }
}

/* Returns the parsed AST of a shell script, or null if shparse2ast rejects it. */
function parseScript(file) {
  const dir = file.slice(0, file.lastIndexOf('/')) || '/';
  const base = file.slice(file.lastIndexOf('/') + 1);

  /* shparse2ast given an absolute path crashes or loops forever on scripts defining functions
   * (see BUGS), so it runs from the script's directory on a relative path. */
  const proc = std.popen(`cd ${shellQuote(dir)} && timeout 30 shparse2ast -P ${shellQuote('./' + base)} 2>/dev/null`, 'r');
  let ast;

  try {
    ast = readAst(proc);
  } catch(e) {
    ast = null;
  }

  return proc.close() === 0 ? (ast ?? null) : null;
}

/* A word made of exactly one plain string node has a statically known text. */
function literal(word) {
  const list = word.list;
  return list && list.length === 1 && list[0].kind === 'string' ? list[0].stra : null;
}

const KINDS = new Set(['simple_command', 'function_definition']);

/* Visits every member, not just .cmds: simple commands also hide inside
 * if/while/for bodies, functions, subshells and $(...) expansions. Uses an explicit
 * stack because recursive generators overflow QuickJS's stack on long elif chains. */
function* nodesOfKind(root, kinds) {
  const stack = [root];

  while(stack.length) {
    const node = stack.pop();
    if(node === null || typeof node !== 'object') continue;

    if(kinds.has(node.kind)) yield node;

    const children = Array.isArray(node) ? node : Object.values(node);
    for(let i = children.length - 1; i >= 0; i--) stack.push(children[i]);
  }
}

function bump(counts, key, n) {
  counts.set(key, (counts.get(key) ?? 0) + n);
}

const COLORS = { special: '\x1b[92m', builtin: '\x1b[92m', external: '\x1b[91m' };
const colorize = os.isatty(1);

/* Colors the command name only, the first word after the 10-column class field of a key. */
function paint(key) {
  const color = colorize && COLORS[key.slice(0, 10).trim()];
  const name = color && /^.{10}\S+/.exec(key);

  return name ? name[0].slice(0, 10) + color + name[0].slice(10) + '\x1b[0m' + key.slice(name[0].length) : key;
}

function printHistogram(counts) {
  for(const [key, count] of [...counts].sort((a, b) => b[1] - a[1] || (a[0] < b[0] ? -1 : 1))) std.puts(`${count}\t${paint(key)}\n`);
}

function usage(exitCode) {
  std.puts(
    `Usage: ${scriptArgs[0]} [OPTIONS]\n\n` +
      `Lists the commands invoked by the *.sh, configure.in and configure.ac files known\n` +
      `to locate(1), classified as\n` +
      `special (special builtin), builtin (any other builtin listed by \`shish -c help\`), keyword ([[),\n` +
      `external (an executable in a PATH directory) or notfound. Commands whose name is\n` +
      `not a plain literal (e.g. "$CC") are skipped, and so are calls to functions the same\n` +
      `script defines.\n\n` +
      `Only real-world scripts count: files under hidden directories, test suites and\n` +
      `/usr/src/linux-* are skipped, as are files over 1 MiB, .sh files without a shell shebang\n` +
      `and files whose content was already seen. Each distinct command in a generated\n` +
      `configure script counts once.\n\n` +
      `  -a, --all         print the whole command line (literal words only) instead of the name\n` +
      `  -H, --histogram   instead of one line per command, print "count class name" sorted by count:\n` +
      `                    per script right after it is parsed, then a total over all scripts\n` +
      `  -i, --identifier  use only valid identifiers\n` +
      `  -e, --exists      commands must exist\n` +
      `  -p, --posix       only show POSIX utilities\n` +
      `  -x, --exclude RE  skip scripts whose path matches the regexp RE (repeatable)\n` +
      `  -h, --help        show this help\n`,
  );

  std.exit(exitCode);
}

function main(...args) {
  let all = false;
  let histogram = false;
  let limit = Infinity;

  const params = getOpt(
    {
      help: [false, () => usage(0), 'h'],
      all: [false, () => (all = true), 'a'],
      histogram: [false, () => (histogram = true), 'H'],
      exclude: [true, (v, prev) => [...(prev || []), v], 'x'],
      limit: [true, v => (limit = +v), 'n'],
      exists: [false, null, 'e'],
      identifier: [false, null, 'i'],
      posix: [false, null, 'p'],
    },
    args,
  );

  const excludes = (params.exclude || []).map(re => new RegExp(re));
  const builtins = shishBuiltins();
  const externals = pathCommands();
  const counts = new Map();
  const seen = new Set();
  let scripts = 0;
  let failed = 0;
  let skipped = 0;

  const locate = std.popen(`locate -e --regex '\\.sh$|/configure\\.(in|ac)$'`, 'r');
  let file;
  let count = 0;

  while((file = locate.getline()) !== null) {
    if(!file) continue;

    file = file.replace(/\.(in|ac)$/gi, '');

    if(!isWanted(file, excludes, seen)) {
      skipped++;
      continue;
    }

    const ast = parseScript(file);

    if(!ast) {
      failed++;
      continue;
    }

    scripts++;

    const fileCounts = new Map();
    const onlyOnce = GENERATED_CONFIGURE.test(file) ? new Set() : null;

    const commands = [];
    const functions = new Set();

    for(const node of nodesOfKind(ast, KINDS)) {
      if(node.kind === 'simple_command') commands.push(node);
      else functions.add(node.name);
    }

    for(const cmd of commands) {
      if(!cmd.args || !cmd.args.length) continue;

      const word = literal(cmd.args[0]);
      if(word === null) continue;

      const name = basename(word);
      if(functions.has(name)) continue;

      //if(/\//.test(name)) name = name.replaceAll(/.*\//g, '');

      if(params.identifier && !/^[\[\]-_A-Za-z0-9]+$/.test(name)) continue;
      if(params.posix && !POSIX.has(name)) continue;

      const text = all
        ? cmd.args
            .map(literal)
            .filter(s => s !== null)
            .join(' ')
        : name;
      const cl = classify(name, builtins, externals);

      if(params.exists && cl == 'notfound') continue;

      const key = cl.padEnd(10, ' ') + text;

      if(onlyOnce) {
        if(onlyOnce.has(key)) continue;
        onlyOnce.add(key);
      }

      if(histogram) bump(fileCounts, key, 1);
      else std.puts(paint(key) + '\n');
    }

    if(histogram && fileCounts.size) {
      std.puts(`# ${file}\n`);
      printHistogram(fileCounts);
      std.puts('\n');
      std.out.flush();

      for(const [key, count] of fileCounts) bump(counts, key, count);
    }

    if(++count >= limit) break;
  }

  locate.close();

  if(histogram) {
    std.puts('# total\n');
    printHistogram(counts);
  }

  std.err.puts(`${scripts} scripts parsed, ${failed} failed, ${skipped} skipped\n`);
}

main(...scriptArgs.slice(1));
