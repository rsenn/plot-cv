import ProxyList from '/home/roman/.nvm/versions/node/v14.3.0/lib/node_modules/free-proxy/index.js';
import ProxyLists from '/home/roman/.nvm/versions/node/v14.3.0/lib/node_modules/proxy-lists/index.js';
import Util from './lib/util.js';

import repeater from '@repeaterjs/repeater';
const { Repeater } = repeater;
import { Console } from 'console';
import net from 'net';
import fs, { promises as fsPromises } from 'fs';

global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 0, colors: true, breakLength: Number.Infinity, compact: true }
});

function Proxy(obj) {
  const p = this instanceof Proxy ? this : {};

  for(let prop in obj) {
    const v = obj[prop];

    if(Util.isIpAddress(v)) {
      p.ip = v;
    } else if(Util.isPortNumber(v)) {
      p.port = +v;
    } else if(/proto/i.test(prop)) {
      p.protocol = Util.isArray(v) ? v[0] : v;
      if(/https/.test(p.protocol)) p.protocol = 'http';
    } else if(/(country|source)/i.test(prop)) {
      p[prop] = v;
    }
  }  const propNames = ['protocol', 'ip', 'port', 'country', 'source'];
  let i = propNames.findIndex(prop => p[prop] === undefined);
  if(i != -1) {
    throw new Error(`Property '${propNames[i]}' missing on: ` + Util.toSource(p));
  }
  return p;}
Proxy.prototype.defaultTimeout = 5000;
Proxy.prototype.valueOf = function() {
  return this.time;
};
Proxy.prototype.toSource = function() {
  const { protocol, ip, port, country, time, source } = this;
  return Util.toSource({ protocol, ip, port, country, time, source });
};
Proxy.prototype.toString = function() {
  const { protocol, ip, port } = this;
  return `${protocol} ${ip} ${port}`;
};
Proxy.prototype.ping = function() {
  const proxy = this;
  const { protocol, ip, port } = proxy;
  return new Promise((resolve, reject) => {
    const tcp = new net.Socket();
    const start = Date.now();
    tcp.setTimeout(proxy.defaultTimeout);
    tcp.setNoDelay(true);
    //console.log(`Connecting to ${ip}:${port} ...`);
    tcp
      .connect(port, ip, () => finish(`connected to ${ip}:${port}`, start))
      .on('close', () => finish(null, start))
      .on('error', err => finish(err, -1));
    function finish(msg, start = -1, end = Date.now()) {
      proxy.time = start >= 0 ? end - start : Number.Infinity;
      tcp.destroy();
      if(msg) console.error(msg);
      start < 0 ? reject(msg) : resolve(proxy);
    }
  });
};

function main() {
  const proxies = [
    new Repeater(async (push, stop) => {
      try {
        const proxyList = new ProxyList();
        for(const p of await proxyList.getByCountryCode('DE')) {
          let proxy = new Proxy({ source: 'free-proxy', ...p });
          await proxy.ping();
          console.log('\nPROXY:', proxy, '\n');
          push(proxy);
        }
      } catch(error) {
        stop(new Error(error));
      }
    }),
    new Repeater(async (push,stop) => {
      proxynova(['de', 'at'], 1000, (err, proxies) => {
        for(let p of proxies)
          await new Proxy(p).ping().then(push).catch(console.error);
        }
  //...
});
    })
    new Repeater(async (push, stop) => {
      ProxyLists.getProxies({
        countries: ['de' /*, 'at', 'nl'*/],
           requestQueue: {
        concurrency: 5,
        delay: 50
    },
      })
        .on('data', async proxies => {
          console.error('got some proxies', proxies.length);
          for(let p of proxies) {
            console.error('got proxy', p);

            let proxy = new Proxy(p);
            await proxy
              .ping()
              .then(push)
              .catch(err => console.error('err:', err));
          }
        })
        .on('error', function(error) {
          console.error('error!', (error + '').split(/\n/g)[0]);
        })
        .once('end', function() {
          stop();
        });
    })
  ];

  async function writeResults(results, format = 'txt', outputName = 'proxies') {
    let filename = outputName + '.' + format;
let tempfile = filename+'.'+Util.randStr(6);
    let output = await fsPromises.open(tempfile, 'w');
    let method = { txt: r => r.map(p => p.toString()).join('\n'), json: r => `[\n${r.map(p => '  ' + Util.toSource(p)).join(',\n')}\n]` }[format];

    await output.write(method(results) + '\n');
    await output.close();

    await fsPromises.unlink(filename);
    await fsPromises.link(tempfile, filename);
    await fsPromises.unlink(tempfile);
  }

  (async () => {
    let results = [];

    try {
      let i = 0;
      for await (const proxy of Repeater.merge(proxies.slice(0,1))) {
        console.error(`Proxy #${++i}:`, proxy); // 1, 2

        Util.insertSorted(results, proxy);
        console.log(proxy);
        await writeResults(results, 'txt');
        await writeResults(results, 'json');
      }
    } catch(err) {
      console.error(err); // TimeoutError: 1000 ms elapsed
    }

    //proxies.sort((a, b) => a.time - b.time);
  })();
}

try {
  main();
} catch(err) {
  console.log('Top-level error:', err);
}
