import ProxyList from '/home/roman/.nvm/versions/node/v14.3.0/lib/node_modules/free-proxy/index.js';
import ProxyLists from '/home/roman/.nvm/versions/node/v14.3.0/lib/node_modules/proxy-lists/index.js';
import proxynova from '/home/roman/.nvm/versions/node/v14.3.0/lib/node_modules/proxynova/index.js';
import Util from './lib/util.js';
import { Repeater } from './lib/repeater/repeater.js';
import net from 'net';
import ConsoleSetup from './consoleSetup.js';
import fetch from 'isomorphic-fetch';
import deep from './lib/deep.js';
import { promises as fsPromises } from 'fs';

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
  }
  const propNames = ['protocol', 'ip', 'port', 'country', 'source'];
  let i = propNames.findIndex(prop => p[prop] === undefined);
  if(i != -1) {
    throw new Error(`Property '${propNames[i]}' missing on: ` + Util.toSource(p));
  }
  //console.log('new proxy:', p);
  return p;
}
Proxy.prototype.defaultTimeout = 30000;
Proxy.prototype.valueOf = function() {
  return this.time;
};
Proxy.prototype.toSource = function() {
  const { protocol, ip, port, country, time, source } = this;
  return Util.toSource({
    protocol,
    ip,
    port,
    country,
    time,
    source
  });
};
Proxy.prototype.toString = function() {
  const { protocol, ip, port } = this;
  return `${protocol} ${ip} ${port}`;
};
Proxy.prototype.check = function(url) {
  return Check(this, url);
};

Proxy.prototype.ping = function() {
  const proxy = this;
  const { protocol, ip, port } = proxy;
  return new Promise((resolve, reject) => {
    const tcp = new net.Socket();
    const start = Date.now();
    tcp.setTimeout(proxy.defaultTimeout);
    tcp.setNoDelay(true);
    console.log(`Connecting to ${ip}:${port} ...`);
    tcp.connect(port, ip, () => finish(`Connected to ${ip}:${port}`, start));
    tcp.on('close', () => finish(null, start));
    tcp.on('error', err => finish(err, -1));
    tcp.on('timeout', () => finish('timeout', -1));

    function finish(msg, start = -1, end = Date.now()) {
      proxy.time = start >= 0 ? end - start : Number.Infinity;
      tcp.destroy();
      if(msg) console.error(msg);
      start < 0 ? reject(msg) : resolve(proxy);
    }
  });
};
Proxy.prototype[Symbol.for('nodejs.util.inspect.custom')] = function() {
  const coloring = Util.coloring(!Util.isBrowser());
};

async function main(country = 'de') {
  await ConsoleSetup({ depth: 2, breakLength: 120 });

  console.log(`Searching proxies in country '${country}'`);
  const proxies = [
    new Repeater(async (push, stop) => {
      try {
        const proxyList = new ProxyList();
        for(const p of await proxyList.getByCountryCode(country.toUpperCase())) {
          let proxy = new Proxy({ source: 'free-proxy', ...p });
          let check = await Check(proxy);
          console.log('\nPROXY:', proxy, check, '\n');
          push(proxy);
        }
      } catch(error) {
        stop(new Error(error));
      }
    }),
    new Repeater(async (push, stop) => {
      proxynova([country], 1000, async (err, proxies) => {
        for(let p of proxies)
          await new Proxy(p)
            //.ping()
            .then(push)
            .catch(console.log);
      });
    }),

    new Repeater(async (push, stop) => {
      ProxyLists.getProxies({
        countries: [country],
        requestQueue: {
          concurrency: 5,
          delay: 50
        }
      })
        .on('data', async proxies => {
          console.log('got some proxies', proxies.length);
          for(let p of proxies) {
            console.log('got proxy', p);
            let proxy = new Proxy(p);
            await proxy
              .ping()
              .then(push)
              .catch(err => console.error('err:', err));
          }
        })
        .on('error', error => {
          console.error('error!', (error + '').split(/\n/g)[0]);
        })
        .once('end', () => {
          stop();
        });
    }) /*,
    new Repeater(async (push, stop) => {
    try {    
      let response = await fetch('https://sunny9577.github.io/proxy-scraper/proxies.json').catch(stop);

        let json = await response.json();
        //console.log("json:", json);

        const flat = deep.flatten(json, new Map(), (v, k) => k.length > 1 && typeof v == 'object', (k, v) => [k.join('.'), v]
        );
        console.log('flat:', flat.values());

        for(let entry of flat.values()) {
          console.log('proxy:', entry);
          const { ip, port, country, anonymity, type } = entry;
          if(!/germany/i.test(country)) continue;
          const protocol = type.split(/[^A-Za-z0-9]+/g)[0].toLowerCase();

          console.log('proxy:', { ip, port, country, anonymity, protocol });
          let proxy = new Proxy({ ip, port, country, protocol, source: 'proxy-scraper' });
          await proxy
            .ping()
            .then(push)
            .catch(err => console.error('err:', err));
        }
      } catch(err) {}
    })*/
  ];
  (async () => {
    let results = [];
    try {
      console.log(`Start`);
      let i = 0;
      for await (const proxy of Repeater.merge(proxies.slice(-1))) {
        const { host, port, type } = proxy;
        console.log(`Proxy #${++i}:`, proxy); // 1, 2
        Util.insertSorted(results, proxy);
        console.log(proxy.toString());
        //  let response = await Check(proxy);

        await writeResults(results, 'txt');
        await writeResults(results, 'json');
      }
    } catch(err) {
      console.log('ERROR:', err); // TimeoutError: 1000 ms elapsed
    }

    //proxies.sort((a, b) => a.time - b.time);
  })();
}
Util.callMain(main, true);

async function writeResults(results, format = 'txt', outputName = 'proxies') {
  let filename = outputName + '.' + format;
  let tempfile = filename + '.' + Util.randStr(6);
  let ret;
  try {
    let output = await fsPromises.open(tempfile, 'w');
    let method = {
      txt: r => r.map(p => p.toString()).join('\n'),
      json: r => `[\n${r.map(p => '  ' + Util.toSource(p)).join(',\n')}\n]`
    }[format];

    ret = await output.write(method(results) + '\n');
    await output.close();

    await fsPromises.unlink(filename).catch(err => {});
    await fsPromises.link(tempfile, filename);
    await fsPromises.unlink(tempfile);
  } catch(err) {
    console.log(`ERROR writing '${filename}':`, err);
    throw err;
  }
  const { bytesWritten } = ret;
  console.log(`Wrote '${filename}':`, bytesWritten);
}
