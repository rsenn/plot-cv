import { toString, escape } from 'util';
import { Console } from 'console';
import { Socket, AsyncSocket, SockAddr, AF_INET, SOCK_STREAM, IPPROTO_TCP } from 'sockets';

class TCPSocket extends AsyncSocket {
  constructor() {
    super(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    this.ndelay(true);
  }

  connect(address, port) {
    return super.connect(new SockAddr(AF_INET, address, port));
  }

  bind(address, port) {
    return super.bind(new SockAddr(AF_INET, address, port));
  }
}

async function main(...args) {
  globalThis.console = new Console({
    inspectOptions: { compact: 2, customInspect: true }
  });

  let sock = new TCPSocket(); // new AsyncSocket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

  /*console.log('new Socket() =', sock);
  console.log('sock:', Object.getOwnPropertyNames(sock.__proto__).reduce((acc,k) => sock[k] === undefined ? acc : ({...acc, [k]: sock[k] }), {}));
  console.log('sock.nonblock:', sock.nonblock);*/

  let ret = await sock.connect(new SockAddr(...(args.length ? args : ['192.168.178.23', 22])));

  console.log('connect() =', ret);

  let buf = new ArrayBuffer(1024);
  ret = await sock.recv(buf);

  console.log('sock.recv() =', ret);
  console.log('buf =', escape(toString(buf, 0, ret)));

  sock.close();
}

main(...scriptArgs.slice(1)).catch(err => console.log('error:', err.message, err.stack));
