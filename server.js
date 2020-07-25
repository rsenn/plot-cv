import express from 'express';
import path from 'path';
import fs, { promises as fsPromises } from 'fs';
import util from 'util';
import Util from './lib/util.js';
import tXml from './lib/tXml.js';
import bodyParser from 'body-parser';
import expressWs from 'express-ws';
import { Alea } from './lib/alea.js';

import { Console } from 'console';

const prng = new Alea();
prng.seed(Date.now());
console.log('random:', prng.uint32());
console.log('randStr:', Util.randStr(8, null, prng));

global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 2, colors: true }
});

let app = express();
expressWs(app);
const p = path.join(path.dirname(process.argv[1]), '.');

//console.log('Serving from', p);

app.use(express.text({ type: 'application/xml' }));

app.use(bodyParser.json());

let sockets = [];

const removeItem = (arr, item, key = 'ws') => {
  let i = arr.findIndex(e => e[key] === item);
  if(i != -1) arr.splice(i, 1);
  return arr;
};

function Socket(ws, info) {
  Object.assign(this, { ...info, ws });
  this.id = Util.randStr(10, '0123456789abcdef', prng);
  return this;
}

Socket.prototype.toString = function() {
  return `${this.address}:${this.port}`;
};

app.ws('/ws', async (ws, req) => {
  const { connection, client, upgrade, query, socket, headers, trailers, params, res, route, body } = req;
  const { path, protocol, ip, cookies, hostname } = req;
  const { remoteAddress, remotePort, localAddress, localPort } = client;
  const { _host, _peername } = connection;
  let { address, port } = _peername;
  const { cookie } = headers;

  console.log('WebSocket connected:', path);
  if(address == '::1') address = 'localhost';

  address = address.replace(/^::ffff:/, '');
  let s = new Socket(ws, {
    address,
    port,
    /*remoteAddress, remotePort,*/ localAddress,
    localPort,
    cookie
  });
  let i = sockets.length;
  sockets.push(s);

  console.log('headers:', headers);
  console.log('cookie:', cookie);

  console.log(
    's:',
    Util.filterKeys(s, k => k != 'ws')
  );
  let j = sockets.findIndex(e => e.ws === ws);

  //  console.log('socket', { i, j });
  console.log(
    'sockets:',
    sockets.map(s => Util.filterKeys(s, /^(address|port|id)/))
  );
  ws.on('close', () => {
    console.log(`socket close ${s.toString()} (${s.id})`);
    removeItem(sockets, ws, 'ws');
  });

  ws.on('message', msg => {
    console.log(`message from ${s.toString()} (${s.id}): '${msg}'`);
    const data = `|${s.id}|${msg}`;
    console.log('data:', data);
    let i = -1;
    for(let sock of sockets) {
      if(sock.ws === ws) continue;

      //   console.log('sock:', Util.filterKeys(sock, /^(address|port|cookies)/));
      console.log(`Sending[${++i}/${sockets.length}] to ${sock.id}`);

      let r = Util.tryCatch(
        () => client.writable,
        () => sock.ws.send(data),
        err => (console.log('socket:', sock.info, ' error:', (err + '').replace(/\n.*/g, '')), false),
        null
      );
      if(!r) removeItem(sockets, sock.ws, 'ws');
    }
  });
});

app.use((req, res, next) => {
  if(!/lib\//.test(req.url)) console.log('Request:', req.url);
  next();
});

app.use('/modules', express.static(path.join(p, 'node_modules')));
app.use('/htm', express.static(path.join(p, 'htm')));
app.use('/node_modules', express.static(path.join(p, 'node_modules')));
app.use('/components', express.static(path.join(p, 'components')));
app.use('/lib', express.static(path.join(p, 'lib')));
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

app.get('/style.css', async (req, res) => res.sendFile(path.join(p, 'style.css'), { headers: { 'Content-Type': 'text/css' } }));

function getDescription(file) {
  return parseDocument(fs.readFileSync(file));
  function parseDocument(data) {
    const xml = tXml(data.toString());
    let s = [a => a[0], a => a.children && a.children[0], a => a.children && a.children[0], a => a.children, a => a.find(e => /(board|schematic|library)/.test(e.tagName)), a => a.children, a => a.find(e => e.tagName == 'description'), a => a.children && a.children[0]].reduce((a, p) => a && p(a), xml);

    if(typeof s == 'string') {
      s = Util.decodeHTMLEntities(s);
      s = Util.stripXML(s);
      s = Util.decodeEscapes(s);
      s = Util.stripNonPrintable(s);
      s = s.trim();
      s = s.split(/\n/g).filter(l => l != '')[0];
    }
    return s || '';
  }
}
const descMap = Util.weakMapper(getDescription, new Map());

const GetFilesList = async (dir = './tmp', opts = {}) => {
  const { filter = '.*', descriptions = false } = opts;
  const re = new RegExp(filter);
  const f = ent => /\.(brd|sch|lbr)$/i.test(ent) && re.test(ent);

  return [...(await fs.promises.readdir(dir))]
    .filter(f)
    .map(entry => `${dir}/${entry}`)
    .map(file => {
      let description = descriptions ? descMap(file) : descMap.get(file);
      //   console.log('descMap:', util.inspect(descMap, { depth: 1 }));

      const { ctime, mtime, mode, size } = fs.statSync(file);
      let obj = {
        name: file,
        mtime: '' + Util.unixTime(mtime),
        time: '' + Util.unixTime(ctime),
        mode: `0${(mode & 0o4777).toString(8)}`,
        size: '' + size
      };
      if(typeof description == 'string') obj.description = description;
      return obj;
    });
};

app.get(/^\/files/, async (req, res) => res.json({ files: await GetFilesList() }));
app.post(/^\/(files|list).html/, async (req, res) => {
  const { body } = req;
  const { filter, descriptions } = body;
  console.log('body:', body);
  res.json({ files: await GetFilesList('tmp', { filter, descriptions }) });
});

app.get('/index.html', (req, res) => {
  res.sendFile(path.join(p, 'index.html'));
});

app.post('/save', async (req, res) => {
  const { body } = req;
  //console.log('req.headers:', req.headers);
  //console.log('save body:', body.substring(0, 100), '...');
  const filename = req.headers['content-disposition'].replace(/.*"([^"]*)".*/, '$1') || 'output.svg';
  let result = await fs.promises.writeFile(filename, body, { mode: 0o600, flag: 'w' });
  res.json({ result });
});

app.get('/', (req, res) => {
  res.redirect(302, '/index.html');
});
const port = process.env.PORT || 3000;

app.listen(port, () => {
  //console.log(`Ready at http://127.0.0.1:${port}`);
});
