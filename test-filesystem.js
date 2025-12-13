import { assert, assertEquals, run, default as TinyTest } from './lib/tinyTest.js';

let tmpdir;
let buffer, buffer2;
let handle;
let data = 'TEST\nabcdefg\n123456789';
let data2;

const tests = {
  'filesystem.buffer': () => {
    buffer = filesystem.buffer(4096);
    assert(buffer);
  },
  'filesystem.bufferSize': () => {
    assertEquals(filesystem.bufferSize(buffer), 4096);
  },
  'filesystem.bufferFrom': () => {
    buffer2 = filesystem.bufferFrom('abcdefg\n');
    assertEquals(filesystem.bufferSize(buffer2), 8);

    data2 = filesystem.bufferFrom(data);

    assertEquals(filesystem.bufferSize(data2), data.length);
  },
  'filesystem.bufferToString': () => {
    assertEquals(filesystem.bufferToString(buffer2), 'abcdefg\n');
  },
  'filesystem.mkdir': () => {
    assert(!(filesystem.mkdir(tmpdir, 0o1777) < 0), `mkdir("${tmpdir}", 0o1777) < 0`);
  },
  'filesystem.exists': () => {
    assert(filesystem.exists(tmpdir));
  },
  'filesystem.open': () => {
    assert((handle = filesystem.open(tmpdir + '/rdwr', 'w+')) != null, `open("${tmpdir}/rdwr", "w+") == null`);
  },
  'filesystem.write': () => {
    assertEquals(filesystem.write(handle, data), data.length);
  },
  'filesystem.seek': () => {
    assertEquals(filesystem.seek(handle, 5, SEEK_SET), 5);
  },
  'filesystem.tell': () => {
    assertEquals(filesystem.tell(handle), 5);
  },
  'filesystem.read': () => {
    let ret, str;
    for(let str of ['abcdefg\n', '123456789']) {
      ret = filesystem.read(handle, buffer, 0, str.length);
      assertEquals(ret, str.length);
      assertEquals(filesystem.bufferToString(buffer).slice(0, ret), str);
    }
  },
  'filesystem.close': () => {
    assertEquals(filesystem.close(handle), 0);
  },
  'filesystem.readFile': () => {
    assertEquals(filesystem.readFileSync(tmpdir + '/rdwr'), data);
  },
  'filesystem.writeFile': () => {
    let name = tmpdir + '/wrf';
    let ret = filesystem.writeFile(name, data);
    let d = filesystem.readFileSync(name, null);

    assertEquals(filesystem.bufferToString(d), data);
  },
  'filesystem.size': () => {
    assertEquals(filesystem.size(tmpdir + '/wrf'), data.length);
  },
  'filesystem.symlink': () => {
    assertEquals(filesystem.symlink('..', tmpdir + '/link'), 0);
    assertEquals(filesystem.symlink('wrf', tmpdir + '/file'), 0);
  },
  'filesystem.readlink': () => {
    assertEquals(filesystem.readlink(tmpdir + '/link'), '..');
  },
  'filesystem.realpath': () => {
    assertEquals(filesystem.realpath(tmpdir + '/link'), '/tmp');
  },
  'filesystem.chdir': () => {
    assertEquals(filesystem.chdir(tmpdir), 0);
  },
  'filesystem.getcwd': () => {
    assertEquals(filesystem.getcwd(), tmpdir);
  },
  'filesystem.readdir': () => {
    assertEquals(filesystem.readdir('.').sort().join(','), '.,..,file,link,rdwr,wrf');
  },
  'filesystem.rename': () => {
    assertEquals(filesystem.rename('link', 'link2'), 0);
    assert(filesystem.exists('link2'));
  },
  'filesystem.stat': () => {
    let st;
    assert(filesystem.stat(tmpdir).isDirectory());
    st = filesystem.stat(tmpdir + '/file');
    // console.log("st:", inspect(st));
    assert(st.isFile());
  },
  'filesystem.lstat': () => {
    assert(filesystem.lstat(tmpdir + '/file').isSymbolicLink());
  },
  'filesystem.unlink': () => {
    for(let file of filesystem.readdir(tmpdir)) {
      if(file[0] == '.') continue;
      let path = `${tmpdir}/${file}`;
      assertEquals(filesystem.unlink(path), 0);
      assert(!filesystem.exists(path));
    }
    assertEquals(filesystem.unlink(tmpdir), 0);
    assert(!filesystem.exists(tmpdir));
  },
  'filesystem.isatty': () => {
    let fd = filesystem.open('/dev/stderr');
    //console.log('errstr:', filesystem.errstr);
    assertEquals(filesystem.isatty(fd || filesystem.open('/dev/tty')), true);
    assert(!filesystem.isatty(filesystem.open('/dev/null')));
  },
  'filesystem.mkdtemp': () => {
    let tmp = filesystem.mkdtemp('/tmp/aaaa');
    console.log('tmp:', tmp);
    assert(filesystem.exists(tmp));
    assert(filesystem.stat(tmp).isDirectory());
    assertEquals(filesystem.unlink(tmp), 0);
    assert(!filesystem.exists(tmp));
  },
  'filesystem.stdin': () => {
    let file = filesystem.stdin;
    assertEquals(filesystem.fileno(file), 0);
  },
  'filesystem.stdout': () => {
    let file = filesystem.stdout;
    assertEquals(filesystem.fileno(file), 1);
  },
  'filesystem.stderr': () => {
    let file = filesystem.stderr;
    assertEquals(filesystem.fileno(file), 2);
  }
};

async function main(...args) {
  //  globalThis.console = {};

  console.log('Console:', Object.getPrototypeOf(console));
  console.log('log:', Object.getPrototypeOf(console).log);

  console.log(
    'ARGS:',
    new Map([
      ['a', 1],
      ['b', 2]
    ]),
    { u: undefined, n: null, args: scriptArgs, filesystem }
  );
  tmpdir = `/tmp/${Util.randStr(10)}`;
  TinyTest.run(filter(tests, t => t));
  return;
  console.log(
    Util.getMethodNames(filesystem)
      .map(n => `  'filesystem.${n}': null,`)
      .join('\n')
  );
}

main(...scriptArgs.slice(1));