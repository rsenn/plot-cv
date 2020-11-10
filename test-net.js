import Util from './lib/util.js';
import ConsoleSetup from './lib/consoleSetup.js';
import { client, server, fetch } from 'net';
import { Contour } from 'contour';
import * as cv from 'cv';
import { Point } from 'point';
import { PointIterator } from 'point-iterator';
import { PointList } from './lib/geom.js';
import { Alea } from './lib/alea.js';
import * as os from 'os';

let prng = new Alea(1337);

async function main(...args) {
  await ConsoleSetup({ breakLength: 120, maxStringLength: 200, maxArrayLength: 20 });

  function connect() {
    print('CLIENT');
    net.client({
      port: 3001,
      server: 'localhost',
      onConnect: socket => {
        print('Connected to server');
        socket.send('hello');
      },
      onMessage: (socket, msg) => {
        print('Received from server: ', msg);
      },
      onClose: why => {
        print('Disconnected from server. Reason: ', why);
      },
      onPong: (socket, data) => {
        print('Pong: ', data);
      }
    });
  }
  console.log('globalThis.cv:', globalThis.cv);
  //console.log('global.cv:', global.cv);

  const getTicks = (() => {
    const f = cv.getTickFrequency();
    const ns = f / 10000000;
    const ms = f / 1000;
    const s = f;

    return () => {
      const ticks = cv.getTickCount();
      return [Math.floor(ticks / s), Math.floor(ticks / ns)];
    };
  })();

  console.log('Symbol.iterator:', Symbol.iterator);
  console.log('fetch:', fetch);
  console.log('cv:', cv);
  console.log('cv.getTickCount:', cv.getTickCount);
  console.log('cv.getTickCount():', cv.getTickCount());
  console.log('getTicks():', getTicks());

  const hr = Util.hrtime;
  const times = Util.repeat(4, undefined);

  times[0] = hr();
  os.sleep(10);
  times[1] = hr();
  os.sleep(100);
  times[2] = hr();
  os.sleep(1000);
  times[3] = hr();

  for(let i = 0; i < times.length; i++) console.log(`times[${i}] =`, times[i]);

  function randContour() {
    let pl = new PointList();
    Util.repeat(Util.randInt(10, 100, prng), () => pl.push(Util.randInt(0, 1024, prng), Util.randInt(0, 1024, prng)));
    let ctr = pl.centroid();
    let bb = pl.bbox();
    pl.translate(-ctr.x, -ctr.y);

    let polar = pl.toPolar((x, y) => ({ x: (x * 180) / Math.PI, y: y }));
    polar.sort((a, b) => a.x - b.x);

    pl = polar.fromPolar((x, y) => ({ x: (x * Math.PI) / 180, y: y }));
    pl.translate(ctr.x, ctr.y);
    pl.round(1);

    console.log('pl:', pl + '');
    let c = new Contour(pl);
    return c;
  }

  let contours = Util.repeat(4, () => randContour());
  let contourStr = contours.map(c => c.toString(Contour.FORMAT_NOBRACKET | Contour.FORMAT_SPACE | Contour.FORMAT_01));
  //console.log('contours:', contours.map(contour => [...contour]));

  let body; // = Util.encodeQuery({ data: Util.repeat(10, 'TEST\n').join(''), num: 1234 });

  body = JSON.stringify({ contours: contourStr, frame: 0 });

  let response = await fetch('http://127.0.0.1:3001/contours', {
    method: 'post',
    headers: {
      'Content-Type': 'application/json',
      //'User-Agent': 'meep!meep!',
      'user-agent': 'meep!meep!'
    },
    body
  });
  console.log('response:', response);
  console.log('response:', Util.getMemberNames(response));
  return 'done';
}
Util.callMain(main, true);
