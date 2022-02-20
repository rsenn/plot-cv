import { client, setLog } from 'net';
import { Console } from 'console';

function main(...args) {
  globalThis.console = new Console(std.err, {
    inspectOptions: { compact: 0, customInspect: true }
  });

  const debug = false;

  setLog(((debug ? net.LLL_DEBUG : net.LLL_WARN) << 1) - 1, (level, msg) => {
    let p =
      ['ERR', 'WARN', 'NOTICE', 'INFO', 'DEBUG', 'PARSER', 'HEADER', 'EXT', 'CLIENT', 'LATENCY', 'MINNET', 'THREAD'][
        level && Math.log2(level)
      ] ?? level + '';
    msg = msg.replace(/\n/g, '\\n').replace(/\r/g, '\\r');

    if(!/POLL/.test(msg) && /MINNET/.test(p))
      if(debug && /(client|http|read|write)/i.test(msg)) console.log(p.padEnd(8), msg);
  });

  let url = args[0] ?? 'tcp://127.0.0.1:6999';

  client(url, {
    binary: true,
    onConnect(ws, req) {
      console.log('onConnect', { ws, req });
    },
    onClose(ws, status, reason, error) {
      console.log('onClose', { ws, status, reason, error });
    },
    onHttp(req, resp) {
      console.log('onHttp', { req, resp });
    },
    onFd(fd, rd, wr) {
      //console.log('onFd', fd, rd, wr);
      os.setReadHandler(fd, rd);
      os.setWriteHandler(fd, wr);
    },
    onMessage(ws, msg) {
      console.log('onMessage', { ws, msg });
    },
    onError(ws, error) {
      console.log('onError', ws, error);
    }
  });
}

main(...scriptArgs.slice(1));
