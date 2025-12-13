import { Alea } from './lib/alea.js';
import { define, once, randStr, tryCatch, weakMapper } from './lib/misc.js';
import * as Timers from './lib/repeater/timers.js';
import { Message } from './message.js';

const prng = new Alea();
prng.seed(Date.now());

let sockets = [];
let client;
const enqueue = (q, ...items) => [...(q ? q : []), ...items];

const removeItem = (arr, item, key = 'ws') => {
  let i = arr.findIndex(e => e[key] === item);
  if(i != -1) return arr.splice(i, 1);
};

const sendBuf = client => {
  let self;
  let lines = [];
  self = async function(sock, fn) {
    let ret = await fn.call(sock, (...args) => self.send.call(sock, ...args));
    self.flush(sock);
    return ret;
  };

  self.send = function(msg, ...args) {
    if(Array.isArray(msg)) {
      msg.forEach(m => self(m));
      return;
    }

    if(!(args.length == 0 && typeof msg == 'string')) msg = new Message(msg, ...args);
    if(msg instanceof Message) msg = msg.data;
    console.debug(`[${this.id}] send '${msg}'`);
    lines.push(msg);
  };
  self.flush = async sock => {
    if(lines.length > 0) await sendTo.call(client, sock, lines.join('\n'));
    lines.splice(0, lines.length);
  };
  self.buf = lines;
  return self;
};

function sendTo(sock, msg, ...args) {
  if(Array.isArray(msg)) {
    let lines = msg.map(m => new Message(m).data);
    return sendTo(sock, lines.join('\n'));
  }

  if(!(args.length == 0 && typeof msg == 'string')) msg = new Message(msg, ...args);
  if(msg instanceof Message) msg = msg.data;

  const { writable } = this || { writable: true };
  // console.debug(`[${sock.id}] sendTo '${msg.replace(/\n/g, '\\n')}'`);

  return tryCatch(
    async () => {
      if(writable) await sock.ws.send(msg);
      else throw new Error(`${sock.id} not writable`);
    },
    async ret => (sock.bytesWritten = (sock.bytesWritten || 0) + msg.length),
    err => console.log(`[${sock.id}] error:`, (err + '').replace(/\n.*/g, ''))
  );
  /*else
sock.queue = enqueue(sock.queue, msg);*/
}

function sendMany(except, msg, ...args) {
  if(!(args.length == 0 && typeof msg == 'string')) {
    msg = new Message(msg, ...args);
    msg = msg.data;
  }
  return Promise.all(sockets.filter(sock => !(sock == except || sock.id == except || sock.ws == except)).map(sock => sendTo.call(this, sock, msg)));
}

export class Socket {
  static map = weakMapper((ws, info, client) => new Socket(ws, info, client));
  handlers = new Map();

  constructor(ws, info, props) {
    Object.assign(this, { info });
    define(this, { ws, ...props });

    this.id = randStr(10, '0123456789abcdef', prng);
  }

  on(event, fn) {
    if(this.handlers.has(event)) return;
    const handler = fn; /* async (...args) => { this.send = sendBuf(this.client);
      let r = await fn.call(this, ...args);
      await this.send.flush(this);
      this.send = null;
      return r;
    };*/
    this.handlers.set(event, handler);
    this.ws.on(event, handler);
  }

  async processLine(line, send) {
    this.lastMessage = Date.now();

    this.bytesRead = (this.bytesRead || 0) + line.length;
    // console.debug(`[${this.id}] message '${line}'`);

    let msg = new Message(line);
    if(msg.type == 'INFO') {
      const id = sockets.findIndex(s => s.id == msg.body);
      if(id != -1) {
        const sock = sockets[id];
        return await send({ ...this.info, idle: Date.now() - this.lastMessage }, sock.id, null, 'INFO');
      }
    } else if(msg.type == 'PING') {
      return await send(msg.body, null, msg.origin, 'PONG');
    } else if(msg.type == 'QUIT') {
      return await this.closeConnection(msg.body);
    }

    // console.log(`[${this.id}] message ${msg.origin ? `from '${msg.origin}'` : ''}${msg.recipient ? ' to ' + msg.recipient : ''}: `, msg);
    if(msg.recipient) {
      let rId = sockets.findIndex(s => s.id == msg.recipient);
      if(rId == -1) {
        console.error(`No such recipient: '${msg.recipient}'`);
        return;
      }
    }
    let i = -1;
    for(let sock of sockets) {
      if(sock.ws === this.ws) continue;
      if(msg.recipient && sock.id != msg.recipient) continue;
      console.log(`Sending[${++i}/${sockets.length}] to ${sock.id}:`, msg.line);
      await send(msg.line);
    }
  }

  /*  async flush() {
    return await this.send.flush(this);
  }*/
  static timeoutCycler = once(async () => {
    let timer = Timers.interval(1000);
    for await(let t of timer) {
      let now = Date.now();
      for(let s of sockets) {
        const idle = now - s.lastMessage;
        const seconds = Math.floor(idle / 1000);
        //  console.debug(`timeoutCycler`, s.id, seconds);
      }
    }
  });

  static async endpoint(ws, req) {
    const { connection, client, headers } = req;
    const { path } = req;
    let { remoteAddress, remotePort, localAddress, localPort } = client;
    const { _host, _peername } = connection;
    let { address, port } = _peername;
    const { cookie } = headers;
    if(localAddress == '::1') localAddress = 'localhost';
    if(remoteAddress == '::1') remoteAddress = 'localhost';
    let s = Socket.map(
      ws,
      {
        local: localAddress.replace(/^::ffff:/, '') + ':' + localPort,
        remote: remoteAddress.replace(/^::ffff:/, '') + ':' + remotePort,
        cookie,
        userAgent: headers['user-agent'],
        path
      },
      { client, connection }
    );
    // console.log('WebSocket connected:', s, headers);
    let i = sockets.length;
    Object.assign(client, { sendTo, sendMany });
    s.closeConnection = async function closeConnection(reason) {
      console.debug(`[${this.id}] closeConnection:`, reason);
      await this.ws.close();
      if(removeItem(sockets, this.ws, 'ws')) await client.sendMany(this, reason || 'closed', this.id, null, 'QUIT');
    };
    s.lastMessage = Date.now();
    sockets.push(s);
    s.on('close', async function(arg) {
      console.log(`[${s.id}] close`);
      await s.closeConnection();
    });
    s.on('error', async function(arg) {
      console.log(`[${s.id}] error`, arg);
      await s.closeConnection('error');
    });
    s.on('message', async function(data) {
      await sendBuf(client)(s, async function(send) {
        for(let line of data.split(/\n/g)) await this.processLine(line, send);
      });
    });
    await client.sendMany(s, { origin: s.id, type: 'JOIN' });
    await sendBuf(client)(s, function(send) {
      this.lastMessage = Date.now();
      send({ type: 'HELLO', body: this.id });
      if(sockets.length)
        send({
          type: 'USERS',
          body: sockets.map(s => s.id).filter(s => typeof s == 'string')
        });
    });
  }

  static sendAll = async (...args) => await sendMany(null, ...args);
}

export default Socket;