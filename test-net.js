import { fetch } from 'net.so';
import { Alea } from './lib/alea.js';
let prng = new Alea(Date.now());

function main(...args) {
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
  /* console.log('Setup duration:', hr(begin));

  let raw = new Mat();
  let cap = new VideoCapture();
  cap.open(0, CAP_V4L2);

  cap.read(raw);
  const width = raw.cols;
  const height = raw.rows;

  console.log('raw', raw.cols, raw.rows);

  console.log('Preamble duration:', hr(begin));

  start = hr();

  function randContour() {
    let pl = new PointList();
    Util.repeat(Util.randInt(10, 100, prng), () => pl.push(Util.randInt(0, width, prng), Util.randInt(0, height, prng)));
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
  body = JSON.stringify({ contours: contourStr, frame: 0, width, height });
  console.log('Prepare duration:', hr(start));

  start = hr();*/
  let body;

  let response = fetch('http://www.google.com/', {
    method: 'get',
    headers: {
      // 'Content-Type': 'application/json',
      'User-Agent': 'meep!meep!'
    },
    body
  });

  // console.log('Request duration:', hr(start));

  console.log('response:', response);
  // console.log('response:', Util.getMemberNames(response));

  // console.log('Total duration:', hr(begin));
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error.message}\n${error.stack}`);
  std.exit(1);
} finally {
  //console.log('SUCCESS');
}