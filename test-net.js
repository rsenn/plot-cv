import Util from './lib/util.js';
import ConsoleSetup from './lib/consoleSetup.js';
import { client, server, fetch } from 'net';
import { Contour } from 'contour';
import { Point } from 'point';
import { PointIterator } from 'point-iterator';
import { PointList } from './lib/geom.js';
import { Alea } from './lib/alea.js';

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

  console.log('Symbol.iterator:', Symbol.iterator);
  console.log('fetch:', fetch);

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
    /*for(let i = 0; i < pl.length; i++)
    delete pl[i];*/
    return c;
  }

  let contours = Util.repeat(4, () => randContour());

  let contourStr = contours.map(c => c.toString(Contour.FORMAT_NOBRACKET | Contour.FORMAT_SPACE | Contour.FORMAT_01));
  //  let contourSource = contours.map(c => c.toSource());

  console.log('contours:',
    //contours.map(contour => contour.toArray())
    contours.map(contour => [...contour])
    //contours.map(contour => contour.toSource())
  );
  /*  console.log('contours:',
    contours.map(contour => {
      let r = [];
      for(let i = 0; i < contour.length; i++) r.push({ x: contour.get(i).x, y: contour.get(i) });
      return r;
    })
  );*/

  let body = Util.encodeQuery({ data: Util.repeat(10, 'TEST\n').join(''), num: 1234 });

  body = JSON.stringify({ data: contourStr, num: 1234 });

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
