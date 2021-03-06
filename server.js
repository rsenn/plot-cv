import express from 'express';
import path from './lib/path.js';
import Util from './lib/util.js';
import bodyParser from 'body-parser';
import expressWs from 'express-ws';
import { Alea } from './lib/alea.js';
import crypto from 'crypto';
import fetch from 'isomorphic-fetch';
import { exec } from 'promisify-child-process';
import fs, { promises as fsPromises } from 'fs';
import ConsoleSetup from './lib/consoleSetup.js';
import SerialPort from 'serialport';
import SerialStream from '@serialport/stream';
import SerialBinding from '@serialport/bindings';
import Socket from './webSocket.js';
import WebSocket from 'ws';
import PortableFileSystem from './lib/filesystem.js';
import PortableChildProcess, { SIGTERM, SIGKILL, SIGSTOP, SIGCONT } from './lib/childProcess.js';
import { Repeater } from './lib/repeater/repeater.js';
import { Message } from './message.js';

SerialStream.Binding = SerialBinding;

let filesystem, childProcess;
const port = process.env.PORT || 3000;

const hash = crypto.createHash('sha1');

const prng = new Alea();
prng.seed(Date.now());
console.log('random:', prng.uint32());
console.log('randStr:', Util.randStr(8, null, prng));

let app = express();
expressWs(app, null, { perMessageDeflate: false });
const p = path.join(path.dirname(process.argv[1]), '.');

let mountDirs = [
  'data',
  '../an-tronics/eagle',
  '../insider/eagle',
  '../lc-meter/eagle',
  '../pictest/eagle'
];
let tmpDir = './tmp';

async function waitChild(proc) {
  const { pid, stdout, stderr, wait } = proc;
  console.log('Process ID =', pid);
  let ret = await wait();
  console.log('wait() =', ret);
  return ret;
}

async function runMount(dirsIterator) {
  for await(let dirs of await dirsIterator) {
    console.log(`runMount`, dirs);
    console.debug(`Mount ${dirs} to tmp/`);

    let proc = childProcess('./mount-tmp.sh', ['-f', ...Util.unique(dirs || [])], {
      env: { OPTS: 'auto_unmount,atomic_o_trunc,big_writes,kernel_cache' }
    });
    async function readData(output, callback = d => {}) {
      try {
        for await(let data of new Repeater((push, stop) => {
          output.on('data', chunk => push(chunk.toString()));
          proc.on('exit', stop);
          //          output.on('close', () => push(null));
        })) {
          if(data === null) {
            console.log('output EOF');
            //return;
          }
          if(typeof data == 'string') data.split(/\n/g).forEach(line => callback(line));
        }
      } catch(e) {
        return e;
      }
    }
    readData(proc.stdout);
    readData(proc.stderr, data =>
      console.log('stderr data:', Util.abbreviate(Util.escape(data), Util.getEnv('COLUMNS') || 120))
    );
    let exitCode = await waitChild(proc);
    console.log('exitCode:', exitCode);
    return exitCode;
  }
}

async function RequestContours(req, res) {
  const { body } = req;
  const { contours, frame, width, height } = body;
  console.log(`${req.url}`, { contours, frame, width, height });

  res.status(200).send('OK');

  Socket.sendAll({ type: 'CONTOURS', origin: '*', recipient: '*', body });

  //  res.json({ status: 'OK' });
}

//console.log('Serving from', p);

async function main() {
  await ConsoleSetup({
    breakLength: 120,
    maxStringLength: Infinity,
    maxArrayLength: Infinity
  });
  await PortableFileSystem(fs => (filesystem = fs));
  await PortableChildProcess(cp => (childProcess = cp));

  Socket.timeoutCycler();

  let mounter = runMount(new Repeater(async (push, stop) => {
      while(true) await push(mountDirs);
    })
  ).then(exitCode => {
    console.log('runMount', { exitCode });
    if(exitCode == 127) {
      Util.exit(127);
    }
    return exitCode;
  });

  app.use(express.text({ type: 'application/xml', limit: '16384kb' }));

  app.use(bodyParser.json({ limit: '200mb' }));
  app.use(bodyParser.raw({ type: 'text/plain;charset=UTF-8', limit: '524288kb' }));
  app.use(bodyParser.raw({ type: 'text/plain', limit: '524288kb' }));
  app.use(bodyParser.raw({ type: 'application/octet-stream', limit: '524288kb' }));
  app.use(bodyParser.raw({ type: 'multipart/mixed', limit: '16384kb' }));

  app.use((req, res, next) => {
    res.append('Access-Control-Allow-Origin', `https://api.github.com, http://127.0.0.1:${port}`);
    res.append('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS');
    res.append('Access-Control-Allow-Headers', 'Content-Type, Accept, Authorization');
    res.append('Access-Control-Allow-Credentials', 'true');
    next();
  });

  function SendRaw(res, file, data, type = 'application/octet-stream') {
    res.setHeader('Content-Disposition', `attachment; filename="${path.basename(file)}"`);

    if(type) res.setHeader('Content-Type', type);
    if(data) return res.send(data);
    else if(file && typeof file == 'string') {
      console.log('sendFile', { file });
      return res.sendFile(file, { root: process.cwd() });
    }
  }

  const convertToGerber = async (boardFile, opts = {}) => {
    console.log('convertToGerber', { boardFile, opts });
    let {
      layers = opts.side == 'outline'
        ? ['Measures']
        : opts.drill
        ? ['Drills', 'Holes']
        : [opts.front ? 'Top' : 'Bottom', 'Pads', 'Vias'],
      format = opts.drill ? 'EXCELLON' : 'GERBER_RS274X',
      data,
      fetch = false,
      front,
      back
    } = opts;
    const base = path.basename(boardFile, '.brd');
    const formatToExt = (layers, format) => {
      if(opts.drill ||
        format.startsWith('EXCELLON') ||
        layers.indexOf('Drills') != -1 ||
        layers.indexOf('Holes') != -1
      )
        return 'TXT';
      if(layers.indexOf('Bottom') != -1 || format.startsWith('GERBER'))
        return opts.side == 'outline' ? 'GKO' : front ? 'GTL' : 'GBL';

      return 'rs274x';
    };
    const gerberFile = `./tmp/${base}.${formatToExt(layers, format)}`;
    const cmd = `eagle -X -d ${format} -o "${gerberFile}" "${boardFile}" ${layers.join(' ')}`;
    console.log(`executing '${cmd}'`);
    const child = exec(`${cmd} 2>&1 0</dev/null`, {});
    // do whatever you want with `child` here - it's a ChildProcess instance just
    // with promise-friendly `.then()` & `.catch()` functions added to it!
    let output = '';
    child.stdout.on('data', data => (output += data));
    child.stderr.on('data', data => (output += data));
    const { stdout, stderr, code, signal } = await child;
    console.log(`code: ${code}`);
    //  console.log(`output: ${output}`);
    if(code !== 0) throw new Error(output);
    if(output) output = output.replace(/\s*\r*\n/g, '\n');
    let result = { code, output };
    if(opts.fetch) result.data = await (await fsPromises.readFile(gerberFile)).toString();
    result.file = gerberFile;
    console.log('convertToGerber result =', result);
    return result;
  };

  const gerberEndpoint = async (req, res) => {
    const { body } = req;
    let { board, save, file: filename, raw, ...opts } = body;
    let result;
    console.log('Request /gerber', { board, save, opts });
    try {
      result = await convertToGerber(board, opts);
      if(save) {
        filename = filename || typeof save == 'string' ? save : null;
        filename = `tmp/` + filename.replace(/.*\/([^\/])*\.[^\/.]*$/g, '$1');
        await fsPromises
          .writeFile(filename, result.data)
          .then(res => console.log('Wrote file:', res));
      }
    } catch(error) {
      result = { error };
    }
    console.log('Response /gerber', Util.filterOutKeys(result, /(output|data)/));

    if(/get/i.test(req.method) || raw) {
      const { file } = result;
      return SendRaw(res, file, result.data);
    }
    res.json(result);
  };

  app.get(/^\/gerber/, gerberEndpoint);
  app.post(/^\/gerber/, gerberEndpoint);

  const gerberToGcode = async (gerberFile, allOpts = {}) => {
    const basename = gerberFile.replace(/.*\//g, '').replace(/\.[^.]*$/, '');
    let { fetch, data, raw, ...opts } = allOpts;
    opts = {
      basename,
      zsafe: '1mm',
      zchange: '2mm',
      zwork: '-1mm',
      zdrill: '-2mm',
      zcut: '-2mm',
      'cutter-diameter': '1mm',
      'drill-feed': 1000,
      'drill-speed': 10000,
      'mill-feed': 600,
      'mill-speed': 16000,
      'cut-feed': 200,
      'cut-speed': 10000,
      'cut-infeed': '1mm',

      'output-dir': './tmp/',
      ...opts
    };
    if(opts.front == undefined && opts.back == undefined && opts.drill == undefined)
      opts.back = gerberFile;
    let sides = [];

    for(let side of ['front', 'back', 'drill', 'outline'])
      if(side in opts) {
        if(typeof opts[side] != 'string') opts[side] = gerberFile;
        sides.push(side);
      }

    if(opts.voronoi && !opts.vectorial) opts.vectorial = 1;

    console.debug(`gerberToGcode`, opts);
    function makePath(ext, side, base = basename) {
      return path.join(opts['output-dir'], `${base}_${side}.${ext}`);
    }

    const params = [...Object.entries(opts)]
      .filter(([k, v]) =>
          typeof v == 'string' || typeof v == 'number' || (typeof v == 'boolean' && v === true)
      )
      .map(([k, v]) => `--${k}${typeof v != 'boolean' && v != '' ? '=' + v : ''}`);
    console.log('Request /gcode', { gerberFile, fetch, raw });
    //console.warn(`gerberToGcode`, Util.abbreviate(gerberFile), { gcodeFile, opts });

    let wait;
    try {
      const cmd = `pcb2gcode ${params.join(' ')} 2>&1`;
      console.warn(`executing '${cmd}'`);
      const child = exec(cmd, {});
      // do whatever you want with `child` here - it's a ChildProcess instance just
      // with promise-friendly `.then()` & `.catch()` functions added to it!
      let output = '';
      child.stdout.on('data', data => (output += data));
      child.stderr.on('data', data => (output += data));
      wait = await child.catch(error => ({ code: -1, error }));

      const { stdout, stderr, code, signal } = wait;
      if(output) output = Util.abbreviate(output.replace(/\s*\r*\n/g, '\n'), 200);
      console.log('Response /gcode', { stdout, output, sides });

      //   if(code !== 0) throw new Error(output);

      const gcodeFile = makePath('ngc', sides[0]);
      const svgFile = makePath('svg', sides[0], 'processed');

      for(let [file, to] of sides.map(side => [
        makePath('svg', side, 'processed'),
        makePath('svg', side)
      ]))
        if(fs.existsSync(file)) fs.renameSync(file, to);

      let files = sides
        .map(side => [side, makePath('ngc', side)])
        .filter(([side, file]) => fs.existsSync(file));
      console.log('Response /gcode', { files });

      let result = { code, output, cmd };
      if(fetch) {
        for(let [side, file] of files)
          result[side] = await (await fsPromises.readFile(file)).toString();
      }
      if(/*/get/i.test(req.method) || */ raw) {
        const { file } = result;
        return SendRaw(res, file, result.data);
      }
      result.files = Object.fromEntries(files);
      console.log('Response /gcode', Util.filterOutKeys(result, /(Xoutput|data)/));
      return result;
    } catch(error) {
      Util.putError(error);
    }
  };

  let gcodeEndpoint = async (req, res) => {
    const { body } = req;
    let { file, ...opts } = body;
    let result;

    try {
      result = await gerberToGcode(file, opts).catch(error => ({ error }));
    } catch(error) {
      result = { error };
    } finally {
      res.json(result);
    }
  };
  app.post(/^\/gcode/, gcodeEndpoint);
  app.get(/^\/gcode/, gcodeEndpoint);

  const GithubListContents = async (owner, repo, dir, filter) => {
    const url = `https://api.github.com/repos/${owner}/${repo}/contents/${dir}`;
    console.log(`GITHUB list`, { owner, repo, dir, filter });
    let response = await fetch(url);
    let result = JSON.parse(await response.text());
    console.log('result', result);
    if(filter) {
      const re = new RegExp(filter, 'g');
      result = result.filter(({ name }) => re.test(name));
    }
    return result;
  };

  app.use(async (req, res, next) => {
    if(!/overrides\//.test(req.path)) {
      let relativePath = path.join('.', req.path);
      let overridePath = path.join('overrides', req.path);
      let isFile = false;

      await fsPromises
        .stat(relativePath)
        .then(st => (isFile = st.isFile()))
        .catch(err => {});

      let override = false;

      if(isFile)
        await fsPromises
          .access(overridePath, fs.constants.F_OK)
          .then(() => (override = true))
          .catch(err => {});

      if(override) {
        console.log('Static request:', { overridePath, override, res });

        return res.redirect('/' + overridePath);
      }
    }
    if(/lib\/preact.js/.test(req.url)) req.url = '/lib/preact.mjs';

    if(!/lib\//.test(req.url)) {
      const { path, url, method, headers, query, body } = req;
      false &&
        console.log('Static request:', { path, url, method, headers, query, body } /* Object.keys(req), */,
          ...Util.if(Util.filterOutKeys(
              req.headers,
              /(^sec|^accept|^cache|^dnt|-length|^host$|^if-|^connect|^user-agent|-type$|^origin$|^referer$)/
            ),
            () => [],
            value => ['headers: ', value],
            Util.isEmpty
          )
        );
    }

    next();
  });

  app.use('/static', express.static(path.join(p, 'static')));
  app.use('/modules', express.static(path.join(p, 'node_modules')));
  app.use('/htm', express.static(path.join(p, 'htm')));
  app.use('/node_modules', express.static(path.join(p, 'node_modules')));
  app.use('/overrides', express.static(path.join(p, 'overrides')));
  app.use('/components', express.static(path.join(p, 'components')));
  app.use('/lib', express.static(path.join(p, 'lib')));
  app.use('/tmp', express.static(path.join(p, 'tmp')));
  app.use('/', express.static(p));

  app.get('/favicon.ico', (req, res) =>
    res.sendFile(path.join(p, 'lib/eagle/icon/eagleicon.ico'), {
      headers: {
        'Content-Type': 'image/x-icon'
      }
    })
  );
  app.get(/\/[^\/]*\.js$/, async (req, res) => res.sendFile(path.join(p, req.path)));

  //app.get('/components.js', async (req, res) => res.sendFile(path.join(p, 'components.js')));

  app.get(/\/[^\/]*\.css$/, async (req, res) =>
    res.sendFile(path.join(p, 'style.css'), {
      headers: { 'Content-Type': 'text/css', cacheControl: false }
    })
  );

  async function getDescription(file) {
    let str = await fs.promises.readFile(file).then(r => r.toString());
    let r = [...Util.matchAll('<(/)?(board|schematic|library)[ >]', str)]
      .map(m => m.index)
      .sort((a, b) => a - b)
      .slice(0, 2);
    let chunk = str.substring(...r);
    let a = ['<description>', '</description>'];
    let indexes = a
      .map(s => new RegExp(s))
      .map(re => re.exec(chunk))
      .map(m => m && m.index);
    let d = chunk.substring(...indexes);
    if(d.startsWith('<description')) return Util.decodeHTMLEntities(d.substring(a[0].length));
    return '';
  }

  const descMap = Util.weakMapper(getDescription, new Map());

  async function GetFilesList(dir = './tmp', opts = {}) {
    let { filter = '.*\\.(brd|sch|lbr|GBL|GTL|GKO|ngc)$', descriptions = false, names } = opts;
    const re = new RegExp(filter, 'i');
    const f = ent => re.test(ent);

    console.log('GetFilesList()', { filter, descriptions }, ...(names ? [names.length] : []));

    if(!names) names = [...(await fs.promises.readdir(dir))].filter(f);

    return Promise.all(names
        .map(entry => `${dir}/${entry}`)
        .reduce((acc, file) => {
          let description = descriptions ? descMap(file) : descMap.get(file);
          //   console.log('descMap:', util.inspect(descMap, { depth: 1 }));
          let obj = {
            name: file
          };
          if(typeof description == 'string') obj.description = description;

          acc.push(fs.promises
              .stat(file)
              .then(({ ctime, mtime, mode, size }) =>
                Object.assign(obj, {
                  mtime: Util.toUnixTime(mtime),
                  time: Util.toUnixTime(ctime),
                  mode: `0${(mode & 0x09ff).toString(8)}`,
                  size
                })
              )
              .catch(err => {})
          );
          return acc;
        }, [])
    ).then(a => a.filter(i => i != null));
  }

  function FilesURLs(list) {
    const base_url = list[0].replace(/\/[^\/]*$/, '');
    const files = list.map(url => url.replace(/.*\//g, ''));
    return { base_url, files };
  }
  //app.use("/serialport", remoteSerialPort.http({ verbose: true }));

  app.get(/\/list-serial/, async (req, res) => {
    const list = await SerialPort.list();

    res.json(list.filter(port => ['manufacturer', 'pnpId', 'vendorId', 'productId'].some(key => port[key]))
    );
  });

  app.ws('/serial', async (ws, req) => {
    const { port } = req.body;
    console.debug('Object.keys(req)', Object.keys(req));

    const duplex = WebSocket.createWebSocketStream(ws, { encoding: 'utf8' });

    let serial = new SerialStream(port || '/dev/tnt1');

    duplex.on('data', async data => {
      data = data + '' + '\r\n';

      console.debug('ws -> serial:', escape(data));

      serial.write(data);
      serial.flush();
    });

    serial.on('data', async data => {
      data = (data + '').replace(/\r?\n?$/, '');

      console.debug('serial -> ws:', escape(data));
      duplex.write(data);
    });

    console.debug('websocket:', Util.getMethods(ws, Infinity, 0));
  });

  app.post(/\/serial/, async (req, res) => {
    const { body } = req;
    const { port } = body;
  });
  const configFile = 'config.json';
  const safeStat = Util.tryFunction(f => filesystem.stat(f),
    st => st,
    () => {}
  );

  app.get('/config', async (req, res) => {
    let str = '',
      data = {},
      time = 0;
    Util.tryCatch(() => filesystem.readFile(configFile),
      c => {
        str = c;
        let stat = safeStat(configFile);
        console.log('stat:', stat);
        if(Util.isObject(stat.mtime)) time = stat.mtime.getTime();
      },
      () => (str = '{}')
    );
    let config = Util.tryCatch(() => JSON.parse(str),
      o => o,
      () => ({})
    );
    console.log('config:', config);

    res.json({ config, time, hash: Util.hashString(str) });
  });
  app.post('/config', async (req, res) => {
    const { body } = req;
    let text = body.toString();
    console.log('text:', text);
    let ret = filesystem.writeFile(configFile, text);
    console.log('ret:', ret);
    let stat = safeStat(configFile);
    res.json({
      size: ret,
      time: stat.mtime.getTime(),
      hash: Util.hashString(text)
    });
  });

  app.get(/\/github/, async (req, res) => {
    Util.tryCatch(async () => {
        const { body } = req;
        const url = Util.parseURL(req.url);
        const { location, query } = url;
        let args = location.split(/\//g).filter(p => !/(^github$|^$)/.test(p));
        let options = { ...query, ...body };

        if(args.length > 0) {
          const [owner, repo, dir, filter] = args;
          Util.weakAssign(options, { owner, repo, dir, filter });
        }

        console.log(`GET ${location}`, { args, query, options });

        let result;
        const { owner, repo, dir, filter, tab, after } = options;

        if(owner && repo && dir)
          result = await GithubListContents(owner, repo, dir, filter && new RegExp(filter, 'g'));
        else if(owner && (tab || after)) {
          let proxyUrl = Util.makeURL({
            ...url,
            protocol: 'https',
            host: 'github.com',
            location: ['', ...args].join('/')
          });
          console.log(`PROXY ${proxyUrl}`);

          let response = await fetch(proxyUrl);
          let type = response.headers['content-type'];

          console.log(`RESPONSE`, response.url, type);
          let data = await response.text();
          res.send(data);
          //, 200, { headers: { 'content-type': type }});
          return;
        }

        res.json(FilesURLs(result.map(file => file.download_url)));
      },
      () => {},
      Util.putError
    );
  });

  app.post(/\/github.*/, async (req, res) => {
    const { body } = req;
    let result;
    const { owner, repo, dir, filter } = body;
    console.log('POST github', { owner, repo, dir, filter });

    res.json(await GithubListContents(owner, repo, dir, filter && new RegExp(filter, 'g'))
        .then(result => FilesURLs(result.map(file => file.download_url)))
        .catch(error => ({ error }))
    );
  });

  app.get(/^\/files/, async (req, res) => res.json({ files: await GetFilesList() }));
  app.post(/^\/(files|list).html/, async (req, res) => {
    const { body } = req;
    let { filter, descriptions, names } = body;

    if(names !== undefined) {
      if(typeof names == 'string') names = names.split(/\n/g);
      if(Util.isArray(names)) names = names.map(name => name.replace(/.*\//g, ''));
    }

    res.json({
      files: await GetFilesList('tmp', { filter, descriptions, names })
    });
  });

  app.get('/index.html', async (req, res) => {
    let data = await fs.promises.readFile(path.join(p, 'index.html'));
    res.send(data.toString().replace(/<\?TS\?>/g, Util.unixTime() + ''));
  });
  app.get('/contours', RequestContours);
  app.post('/contours', RequestContours);

  app.post('/save', async (req, res, next) => {
    //   const filename = (req.headers['content-disposition']||'').replace(new RegExp('.*"([^"]*)".*','g'), '$1') || 'output.svg';
    /*    const filename = path.join(process.cwd(), 'tmp', 'upload-' + Util.toUnixTime(Date.now()) + '.txt');
    let output = fs.createWriteStream(filename, { autoClose: true, emitClose: true });
    let s = req.pipe(output);
    console.log('s', Util.className(s));
    let data;
    s.on('close', () => Util.waitFor(1000).then(() => end()));

    //  req.on('end', () => Util.waitFor(500).then(() => end()));

    function end() {
      data = fs.readFileSync(filename).toString();
      console.log('req end', { data });
      res.end(data + '\n\nUpload complete');
      next();
    }
*/

    const { body } = req;
    console.log('req.headers:', req.headers);
    console.log('body:', body, Util.className(body), Util.inspect(body));
    console.log('save body:', typeof body == 'string' ? Util.abbreviate(body, 100) : body);
    let st,
      err,
      filename =
        (req.headers['content-disposition'] || '').replace(new RegExp('.*"([^"]*)".*', 'g'),
          '$1'
        ) || 'output.svg';
    filename = 'tmp/' + filename.replace(/^tmp\//, '');
    await fs.promises
      .writeFile(filename, body, { mode: 0x0180, flag: 'w' })
      .then(() => (st = fs.statSync(filename)))
      .catch(error => (err = error));

    if(err) {
      console.log('save error:', err);
      res.json(err);
    } else {
      console.log('saved:', filename, `${st.size} bytes`);
      res.json({ size: st.size, filename });
    }
  });

  app.ws('/ws', Socket.endpoint);
  app.ws('/.websocket', Socket.endpoint);
  app.ws('/ws/.websocket', Socket.endpoint);

  app.get('/', (req, res) => {
    res.redirect(302, '/index.html');
  });

  app.listen(port, () => {
    console.log(`Ready at http://127.0.0.1:${port}`);
  });
}

try {
  await main();
} catch(err) {
  Util.putError(err);
}
//Util.callMain(main, true);
