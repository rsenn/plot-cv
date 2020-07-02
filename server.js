import express from 'express';
import path from 'path';
import fs from 'fs';
import Util from './lib/util.js';

import expressWs from 'express-ws';

let app = express();
expressWs(app);
const p = path.join(path.dirname(process.argv[1]), '.');

//console.log('Serving from', p);

app.use(express.text({ type: 'application/xml' }));

let sockets = [];

const removeItem = (arr, item) => {
  let i = arr.indexOf(item);
  if(i != -1) arr.splice(i, 1);
  return arr;
};

function Socket(ws, info) {
  Object.assign(this, { ...info, ws });
  return this;
}

Socket.prototype.toString = function() {
  return `${this.address}:${this.port}`;
};

app.ws('/ws', async (ws, req) => {
  const { connection, client, upgrade, query, socket, headers, trailers, params, res, route, body } = req;
  const { path, protocol, ip, cookies, hostname, host } = req;
  const { remoteAddress, remotePort, localAddress, localPort } = client;
  const { _host, _peername } = connection;
  let { address, port } = _peername;
  const { cookie } = headers;

  //console.log('WebSocket connected:', path);
  if(address == '::1') address = 'localhost';

  address = address.replace(/^::ffff:/, '');
  let s = new Socket(ws, {
    address,
    port,
    /*remoteAddress, remotePort,*/ localAddress,
    localPort,
    cookie
  });
  sockets.push(s);

  //console.log('headers:', headers);
  //console.log('cookie:', cookie);

  //console.log(
    's:',
    Util.filterKeys(s, k => k != 'ws')
  );

  ws.on('message', msg => {
    //console.log(`message from ${s.toString()}:`, msg);

    for(let sock of sockets) {
      if(sock.ws === ws) continue;

      //console.log('sock:', Util.filterKeys(sock, /^(address|port|cookies)/));

      let r =
        client.writable &&
        Util.tryCatch(
          ws => ws.send(msg),
          true,
          err => {
            //console.log('socket:', sock.info, ' error:', (err + '').replace(/\n.*/g, ''));
            return false;
          },
          sock.ws
        );
      if(!r) removeItem(sockets, sock);
    }
  });
});

app.use((req, res, next) => {
  if(!/lib\//.test(req.url)) console.log('Request:', req.url);
  next();
});

app.use('/static', express.static(p));
app.use('/modules', express.static(path.join(p, 'node_modules')));
app.use('/node_modules', express.static(path.join(p, 'node_modules')));
app.use('/components', express.static(path.join(p, 'components')));
app.use('/lib', express.static(path.join(p, 'lib')));

app.get('/favicon.ico', (req, res) =>
  res.sendFile(path.join(p, 'lib/eagle/icon/eagleicon.ico'), {
    headers: {
      'Content-Type': 'image/x-icon'
    }
  })
);
app.get('/main.js', async (req, res) => res.sendFile(path.join(p, 'main.js')));
app.get('/style.css', async (req, res) => res.sendFile(path.join(p, 'style.css'), { headers: { 'Content-Type': 'text/css' } }));

app.get('/files.html', async (req, res) => {
  let files = [...(await fs.promises.readdir('.'))].filter(entry => /\.(brd|sch)$/.test(entry));

  files = files.map(file => {
    const stat = fs.statSync(file);
    const { ctime, mtime, mode, size } = stat;
    return {
      name: file,
      mtime: '' + Util.unixTime(mtime),
      time: '' + Util.unixTime(ctime),
      mode: `0${(mode & 0o4777).toString(8)}`,
      size: '' + size
    };
  });

  //console.log("files:", files);

  res.type('json');
  res.json({ files });
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
