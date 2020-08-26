import express from 'express';
import path from './lib/path.js';
import ConsoleSetup from './consoleSetup.js';
import Util from './lib/util.js';
import tXml from './lib/tXml.js';
import bodyParser from 'body-parser';
import expressWs from 'express-ws';
import { Alea } from './lib/alea.js';
import { Message } from './message.js';
import crypto from 'crypto';
import fetch from 'isomorphic-fetch';
import { exec, spawn, fork, execFile } from 'promisify-child-process';
import fs, { promises as fsPromises } from 'fs';

const port = process.env.PORT || 3000;

const hash = crypto.createHash('sha1');

const prng = new Alea();
prng.seed(Date.now());
console.log('random:', prng.uint32());
console.log('randStr:', Util.randStr(8, null, prng));

let app = express();
expressWs(app);
const p = path.join(path.dirname(process.argv[1]), '.');

//console.log('Serving from', p);

app.use(express.text({ type: 'application/xml', limit: '16384kb' }));

app.use(bodyParser.json());
app.use(bodyParser.raw({ type: 'application/octet-stream', limit: '16384kb' }));
app.use(bodyParser.raw({ type: 'multipart/mixed', limit: '16384kb' }));

app.use((req, res, next) => {
  res.append('Access-Control-Allow-Origin', `https://api.github.com, http://127.0.0.1:${port}`);
  res.append('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS');
  res.append('Access-Control-Allow-Headers', 'Content-Type, Accept, Authorization');
  res.append('Access-Control-Allow-Credentials', 'true');
  next();
});

let sockets = [];

const removeItem = (arr, item, key = 'ws') => {
  let i = arr.findIndex((e) => e[key] === item);
  if(i != -1) arr.splice(i, 1);

  return arr;
};

const convertToGerber = async (boardFile, opts = {}) => {
  const { layers = ['Bottom', 'Pads', 'Vias'], format = 'GERBER_RS274X', data = false } = opts;
  const base = path.basename(boardFile, '.brd');
  const formatToExt = (layers, format) => {
    if(layers.indexOf('Bottom') != -1 || format.startsWith('GERBER')) return 'GBL';
    if(format.startsWith('EXCELLON') || layers.indexOf('Drills') != -1 || layers.indexOf('Holes') != -1) return 'txt';
    return 'rs274x';
  };
  const gerberFile = `./tmp/${base}.${formatToExt(layers, format)}`;
  const cmd = `eagle -X -d ${format} -o "${gerberFile}" "${boardFile}" ${layers.join(' ')}`;

  console.log(`executing '${cmd}'`);
  const child = exec(cmd, {});
  // do whatever you want with `child` here - it's a ChildProcess instance just
  // with promise-friendly `.then()` & `.catch()` functions added to it!
  let output = '';
  child.stdout.on('data', (data) => (output += data));
  child.stderr.on('data', (data) => (output += data));
  const { stdout, stderr, code, signal } = await child;

  console.log(`code: ${code}`);
  console.log(`output: ${output}`);

  if(code !== 0) throw new Error(output);

  if(output) output = output.replace(/ *\r*\n/g, '\n');

  let result = { code, output };

  if(data) result.data = await (await fsPromises.readFile(gerberFile)).toString();
  else result.file = gerberFile;

  return result;
};

const gerberToGcode = async (gerberFile, allOpts = {}) => {
  const basename = path.basename(gerberFile.replace(/\.[A-Za-z0-9]*$/, ''));
  let { fetch, data, ...opts } = allOpts;
  opts = {
    basename,
    zsafe: '1mm',
    zchange: '2mm',
    zwork: '-1mm',
    'mill-feed': 30,
    'mill-speed': 16000,
    'output-dir': './tmp/',
    ...opts
  };
  let side = 'front' in opts ? 'front' : 'back' in opts ? 'back' : '';
  if(side == '') {
    side = 'back';
    opts.back = '';
  }

  const gcodeFile = path.join(opts['output-dir'], `${basename}_${side}.ngc`);
  const params = [...Object.entries(opts)].filter(([k, v]) => typeof v == 'string' || typeof v == 'number').map(([k, v]) => `--${k}${typeof v != 'boolean' && v != '' ? ' ' + v : ''}`);
  console.warn(`gerberToGcode`, { gerberFile, gcodeFile, opts });

  const cmd = `pcb2gcode ${params.join(' ')}  '${gerberFile}'`;
  console.warn(`executing '${cmd}'`);
  const child = exec(cmd, {});
  // do whatever you want with `child` here - it's a ChildProcess instance just
  // with promise-friendly `.then()` & `.catch()` functions added to it!
  let output = '';
  child.stdout.on('data', (data) => (output += data));
  child.stderr.on('data', (data) => (output += data));
  const { stdout, stderr, code, signal } = await child;

  console.log(`code: ${code}`);
  console.log(`output: ${output}`);

  if(code !== 0) throw new Error(output);

  if(output) output = output.replace(/ *\r*\n/g, '\n');

  let result = { code, output };

  if(fetch || data) result.data = await (await fsPromises.readFile(gcodeFile)).toString();
  else result.file = gcodeFile;

  return result;
};

const ListGithubRepo = async (owner, repo, dir, filter) => {
  const url = `https://api.github.com/repos/${owner}/${repo}/contents/${dir}`;
  let response = await fetch(url);
  let result = JSON.parse(await response.text());
  console.log('result', result);
  if(filter) {
    const re = new RegExp(filter, 'g');
    result = result.filter(({ name }) => re.test(name));
  }
  return result;
};

function Socket(ws, info) {
  Object.assign(this, { ...info, ws });
  this.id = Util.randStr(10, '0123456789abcdef', prng);
  return this;
}

Socket.prototype.toString = function () {
  return `${this.address}:${this.port}`;
};

app.ws('/ws', async (ws, req) => {
  const { connection, client, upgrade, query, socket, headers, trailers, params, res, route, body } = req;
  const { path, protocol, ip, cookies, hostname } = req;
  const { remoteAddress, remotePort, localAddress, localPort } = client;
  const { _host, _peername } = connection;
  let { address, port } = _peername;
  const { cookie } = headers;

  console.log('WebSocket connected:', path, headers);

  if(address == '::1') address = 'localhost';

  address = address.replace(/^::ffff:/, '');
  let s = new Socket(ws, {
    address,
    port,
    remoteAddress,
    remotePort,
    localAddress,
    localPort,
    cookie
  });
  let i = sockets.length;

  const sendTo = (sock, msg, ...args) => {
    if(args.length > 0) msg = new Message(msg, ...args);
    if(msg instanceof Message) msg = msg.data;

    Util.tryCatch(
      (ws) => client.writable,
      (ok, ws, data) => ws.send(data),
      (err, ws, data) => (console.log('socket:', sock.info, ' error:', (err + '').replace(/\n.*/g, '')), false),
      sock.ws,
      msg
    );
  };

  sendTo(s, JSON.stringify(sockets.map((s) => s.id)), null, null, 'USERS');

  sockets.push(s);

  const sendMany = (except, msg, ...args) => {
    if(args.length > 0) msg = new Message(msg, ...args);
    if(msg instanceof Message) msg = msg.data;

    for(let sock of sockets) {
      if(sock == except || sock.id == except || sock.ws == except) continue;
      sendTo(sock, msg);
    }
  };

  sendMany(s, '', s.id, null, 'JOIN');

  //console.log('sockets:', sockets.map(s => Util.filterKeys(s, /^(address|port|id)/)));

  ws.on('close', () => {
    console.log(`socket close ${s.toString()} (${s.id})`);
    removeItem(sockets, ws, 'ws');
    sendMany(s, '', s.id, null, 'QUIT');
  });

  ws.on('message', (data) => {
    s.lastMessage = Date.now();
    let msg = new Message(data, s.id);
    if(msg.type == 'INFO') {
      const id = sockets.findIndex((s) => s.id == msg.body);
      if(id != -1) {
        const sock = sockets[id];
        sendTo(s, JSON.stringify(Util.filterOutKeys(sock, ['ws', 'id'])), sock.id, s.id, 'INFO');
      }
      return;
    }
    console.log(`message from ${s.toString()}${msg.recipient ? ' to ' + msg.recipient : ''} (${s.id}): '${msg.body}'`);
    if(msg.recipient) {
      let rId = sockets.findIndex((s) => s.id == msg.recipient);
      if(rId == -1) {
        console.error(`No such recipient: '${msg.recipient}'`);
        return;
      }
    }
    let i = -1;
    for(let sock of sockets) {
      if(sock.ws === ws) continue;
      if(msg.recipient && sock.id != msg.recipient) continue;
      console.log(`Sending[${++i}/${sockets.length}] to ${sock.id}`);
      sendTo(sock, msg.data);
    }
  });
});

app.use(async (req, res, next) => {
  if(!/overrides\//.test(req.path)) {
    let relativePath = path.join('.', req.path);
    let overridePath = path.join('overrides', req.path);
    let isFile = false;

    await fsPromises
      .stat(relativePath)
      .then((st) => (isFile = st.isFile()))
      .catch((err) => {});

    let override = false;

    if(isFile)
      await fsPromises
        .access(overridePath, fs.constants.F_OK)
        .then(() => (override = true))
        .catch((err) => {});

    if(override) {
      console.log('Request:', { overridePath, override });

      return res.redirect('/' + overridePath);
    }
  }

  if(!/lib\//.test(req.url)) console.log('Request:', req.url, ' path:', req.path);

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
app.use('/static', express.static(p));

app.get('/favicon.ico', (req, res) =>
  res.sendFile(path.join(p, 'lib/eagle/icon/eagleicon.ico'), {
    headers: {
      'Content-Type': 'image/x-icon'
    }
  })
);
app.get(/\/[^/]*\.js$/, async (req, res) => res.sendFile(path.join(p, req.path)));

//app.get('/components.js', async (req, res) => res.sendFile(path.join(p, 'components.js')));

app.get('/style.css', async (req, res) =>
  res.sendFile(path.join(p, 'style.css'), {
    headers: { 'Content-Type': 'text/css', cacheControl: false }
  })
);

async function getDescription(file) {
  let str = await fs.promises.readFile(file).then((r) => r.toString());
  let r = [...Util.matchAll('<(/)?(board|schematic|library)[ >]', str)]
    .map((m) => m.index)
    .sort((a, b) => a - b)
    .slice(0, 2);
  let chunk = str.substring(...r);
  let a = ['<description>', '</description>'];
  let indexes = a
    .map((s) => new RegExp(s))
    .map((re) => re.exec(chunk))
    .map((m) => m && m.index);
  let d = chunk.substring(...indexes);
  if(d.startsWith('<description')) return Util.decodeHTMLEntities(d.substring(a[0].length));
  return '';
}

const descMap = Util.weakMapper(getDescription, new Map());

const GetFilesList = async (dir = './tmp', opts = {}) => {
  let { filter = '.*\\.(brd|sch|lbr)$', descriptions = false, names } = opts;
  const re = new RegExp(filter, 'i');
  const f = (ent) => re.test(ent);

  console.log('GetFilesList()', { filter, descriptions, names });

  if(!names) names = [...(await fs.promises.readdir(dir))].filter(f);

  return Promise.all(
    names
      .map((entry) => `${dir}/${entry}`)
      .reduce((acc, file) => {
        let description = descriptions ? descMap(file) : descMap.get(file);
        //   console.log('descMap:', util.inspect(descMap, { depth: 1 }));
        let obj = {
          name: file
        };
        if(typeof description == 'string') obj.description = description;

        acc.push(
          fs.promises
            .stat(file)
            .then(({ ctime, mtime, mode, size }) =>
              Object.assign(obj, {
                mtime: Util.toUnixTime(mtime),
                time: Util.toUnixTime(ctime),
                mode: `0${(mode & 0o4777).toString(8)}`,
                size
              })
            )
            .catch((err) => {})
        );
        return acc;
      }, [])
  ).then((a) => a.filter((i) => i != null));
};

/*app.param(['owner', 'repo','dir'], function (req, res, next, value) {
  console.log('CALLED ONLY ONCE with', value)
  next()
})
*/
function FilesURLs(list) {
  const base_url = list[0].replace(/\/[^\/]*$/, '');
  const files = list.map((url) => url.replace(/.*\//g, ''));
  return { base_url, files };
}

app.get(/\/github/, async (req, res) => {
  const url = Util.parseURL(req.url);
  const { query } = url;
  let result;
  const { owner, repo, dir, filter } = query;
  result = await ListGithubRepo(owner, repo, dir, filter && new RegExp(filter, 'g'));
  res.json(FilesURLs(result.map((file) => file.download_url)));
});

app.post(/\/github.*/, async (req, res) => {
  const { body } = req;
  let result;
  const { owner, repo, dir, filter } = body;
  result = await ListGithubRepo(owner, repo, dir, filter && new RegExp(filter, 'g'));
  res.json(FilesURLs(result.map((file) => file.download_url)));
});

app.post(/^\/gerber/, async (req, res) => {
  const { body } = req;
  let { board, save, filename, ...opts } = body;

  let result;
  console.log('gerber', { board, save, opts });

  try {
    result = await convertToGerber(board, opts);

    if(save) {
      filename = filename || typeof save == 'string' ? save : null;
      filename = `tmp/` + filename.replace(/.*\/([^/])*\.[^/.]*$/g, '$1');

      await fsPromises.writeFile(filename, result.data).then((res) => console.log('Wrote file:', res));
    }
  } catch(error) {
    result = { error };
  }

  res.json(result);
});

app.post(/^\/gcode/, async (req, res) => {
  const { body } = req;
  let { gerber, ...opts } = body;

  let result;

  console.log('gcode', { gerber, opts });

  try {
    result = await gerberToGcode(gerber, opts);
  } catch(error) {
    result = { error };
  }

  res.json(result);
});
app.get(/^\/files/, async (req, res) => res.json({ files: await GetFilesList() }));
app.post(/^\/(files|list).html/, async (req, res) => {
  const { body } = req;
  let { filter, descriptions, names } = body;

  if(names !== undefined) {
    if(typeof names == 'string') names = names.split(/\n/g);
    if(Util.isArray(names)) names = names.map((name) => name.replace(/.*\//g, ''));
  }

  res.json({ files: await GetFilesList('tmp', { filter, descriptions, names }) });
});

app.get('/index.html', async (req, res) => {
  let data = await fs.promises.readFile(path.join(p, 'index.html'));
  res.send(data.toString().replace(/<\?TS\?>/g, Util.unixTime() + ''));
});

app.post('/save', async (req, res) => {
  const { body } = req;
  //console.log('req.headers:', req.headers);
  console.log('save body:', typeof body == 'string' ? Util.abbreviate(body, 100) : body);
  const filename = 'tmp/' + req.headers['content-disposition'].replace(/.*"([^"]*)".*/, '$1') || 'output.svg';
  await fs.promises.writeFile(filename, body, { mode: 0o600, flag: 'w' });
  let st = await fs.promises.stat(filename);

  console.log('saved:', filename, `${st.size} bytes`);
  res.json({ size: st.size, filename });
});

app.get('/', (req, res) => {
  res.redirect(302, '/index.html');
});
app.listen(port, () => {
  //console.log(`Ready at http://127.0.0.1:${port}`);
});
