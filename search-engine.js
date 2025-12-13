import { client, LLL_USER, logLevels, setLog, URL } from 'net';
import * as os from 'os';
import { escape } from 'util';
import { Node, Parser } from './quickjs/qjs-modules/lib/dom.js';
import Console from 'console';
import { get, select, RETURN_PATH_VALUE } from 'deep';
import extendArray from 'extendArray';
extendArray(Array.prototype);

const SearchEngines = [
  'https://www.findtubes.com/search/%s?filter%5Border_by%5D=popular&filter%5Badvertiser_publish_date%5D%5Bmin%5D=&filter%5Bduration%5D%5Bmin%5D=600&filter%5Bquality%5D=hd&filter%5Bvirtual_reality%5D=&filter%5Badvertiser_site%5D=&filter%5Btag_list%5D%5Borientation%5D=straight&filter%5Btag_list%5D%5Bpricing%5D=',
  'https://www.nudevista.com/?q=%s+length%3E15:00',
  'https://www.pornmd.com/straight/%s?qualities=hd&min_duration=600',
  'https://rarbgmirror.org/torrents.php?search=%s&order=seeders&by=DESC',
  'https://sxyprn.com/%s.html?sm=orgasmic'
];

function CreateDocument(xml, filename) {
  let parser = new Parser();
  let doc = parser.parseFromString(xml, filename, { tolerant: true });
  return doc;
}

let clients = new Set();

function* Search(query, fn) {
  //return  new Repeater(async (push, stop) => {

  for(let engine of SearchEngines) {
    let uri = engine.replace(/%s/g, query);
    console.log('URI', uri);
    let url = new URL(uri);

    let { host, port, protocol } = url;
    //    console.log('Search', { query, host, port, protocol });
    let cli;

    yield new Promise((resolve, reject) => {
      cli = client(uri, {
        block: false,
        onConnect(ws, req) {
          console.log('onConnect', console.config({ compact: 1 }), { ws, req });
        },
        onRequest(req, resp) {
          //console.log('onRequest', console.config({ compact: 0 }), { req, resp });
          let { body } = resp;
          resp.text().then(data => {
            console.log('data', console.config({ maxStringLength: 100 }), { data: escape(data) });
            fn(data, url);
            resolve([url, data]);
          });
        },
        onClose() {
          reject();
        },
        onError() {
          reject();
        },
        onFd(fd, rd, wr) {
          // console.log('onFd', fd, rd, wr);
          os.setReadHandler(fd, rd);
          os.setWriteHandler(fd, wr);
        }
      });
      clients.add(cli);
      // console.log('Search clients', [...clients]);

      return cli;
    });
  }
}

function ProcessDocument(resp) {
  let doc = CreateDocument(resp);
  console.log('ProcessDocument', { doc });

  let raw = Node.raw(doc.body);
  console.log('ProcessDocument', console.config({ compact: 10, maxArrayLength: Number.MAX_SAFE_INTEGER, depth: Infinity, customInspect: true }), { raw });

  let entries = select(
    raw,
    (value, path, key) => {
      if(['src', 'href'].contains(path.last)) return true;
      if(['src', 'href'].contains(key)) return true;
      if(typeof value == 'string' && /:\/\//.test(value)) return true;
    },
    RETURN_PATH_VALUE
  ).map(([p, v]) => {
    if(!/:\/\//.test(v)) v = resp.url + v;

    return [p, v];
  });

  console.log('entries', console.config({ maxArrayLength: Number.MAX_SAFE_INTEGER }), entries);

  let elements = entries.map(([p, v]) => get(doc.body, p));
  console.log('elements', console.config({ maxArrayLength: Number.MAX_SAFE_INTEGER }), elements);
  return elements;
}

/*
async function* SearchQuery(arg) {
  let ita = [...Search(arg)];
  console.log('ita', ita);

   for await(let resp of await Repeater.race(ita)) {
    console.log('response', resp);
   yield ProcessDocument(resp);
  
  }
 }*/

async function main(...args) {
  globalThis.console = new Console({
    inspectOptions: {
      maxStringLength: 200,
      maxArrayLength: Infinity,
      compact: 0,
      depth: Infinity
    }
  });
  //setLog(0, () => {});
  setLog(LLL_USER, (level, message) => {
    if(/COMPLETED_CLIENT_HTTP/.test(message)) console.log(logLevels[level].padEnd(16), message);
  });
  for(let arg of args) {
    let promises = [
      ...Search(arg, (data, url) => {
        //console.log('result', { url, data });
      })
    ].map(promise =>
      promise.then(([url, data]) => {
        //console.log('data',console.config({ maxStringLength: 255 }),  { url, data });

        let elements = ProcessDocument(data);
        console.log('elements', elements);
      })
    );
    await Promise.all(promises);
  }

  //await Promise.all([...clients]);
}

main(...scriptArgs.slice(1)).catch(err => {
  console.log('ERROR:', err.message, err.stack);
  std.exit(1);
});