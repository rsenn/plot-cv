import Util from './lib/util.js';
import ConsoleSetup from './lib/consoleSetup.js';
import { client, server, fetch } from 'net.so';
import { Contour } from 'contour.so';
import * as cv from 'cv.so';
import { Point } from 'point.so';
import { PointIterator } from 'point-iterator.so';
import { VideoCapture } from 'video-capture.so';
import { Mat } from 'mat.so';
import { PointList } from './lib/geom.js';
import { Alea } from './lib/alea.js';
import * as os from 'os';

let prng = new Alea(Date.now());
const hr = Util.hrtime;

async function main(...args) {
  let start;
  let begin = hr();
  await ConsoleSetup({
    breakLength: 120,
    maxStringLength: 200,
    maxArrayLength: 20
  });

  /*  function connect() {
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
*/
  console.log('Setup duration:', hr(begin));

  let raw = new Mat();
  let cap = new VideoCapture();
  cap.open(0, cv.CAP_V4L2);

  cap.read(raw);
  const width = raw.cols;
  const height = raw.rows;

  console.log('raw', raw.cols, raw.rows);

  console.log('Preamble duration:', hr(begin));

  start = hr();

  function randContour() {
    let pl = new PointList();
    Util.repeat(Util.randInt(10, 100, prng), () =>
      pl.push(Util.randInt(0, width, prng), Util.randInt(0, height, prng))
    );
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
  let body;
  body = JSON.stringify({ contours: contourStr, frame: 0, width, height });
  console.log('Prepare duration:', hr(start));

  start = hr();

  let response = await fetch('http://127.0.0.1:3001/contours', {
    method: 'post',
    headers: {
      'Content-Type': 'application/json',
      'User-Agent': 'meep!meep!'
    },
    body
  });

  console.log('Request duration:', hr(start));

  console.log('response:', response);
  console.log('response:', Util.getMemberNames(response));

  console.log('Total duration:', hr(begin));
}

Util.callMain(main, true);
