import * as net from 'net';
import PortableConsole from './lib/consoleSetup.js';
import inspect from 'inspect';
import { Contour } from 'opencv';

const { client, server, fetch } = net;

PortableConsole(console => console.log('net: ', { client, server, fetch }));

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

let frame = 0;
let g = globalThis;

g.process = async function process(contours, hier, ...args) {
  //let size = new Size(imgOriginal.cols, imgOriginal.rows);
  const time = Date.now();
  const [images, ...rest] = args;
  const { imgBlurred, imgCanny, imgGrayscale, imgMorphology, imgOriginal, imgRaw, imgTemp, imgVector } = images;
  console.log('images: ', Object.keys(images));
  let m = images.imgVector;
  /*  console.log('Images: ', typeof images);
  console.log('Images: ', Object.keys(images));
  console.log('Images: ', inspect(images));
  console.log('rest.length: ', rest.length);
  console.log('rest: ', ...rest);*/
  const { cols, rows } = m;

  console.log('m: ', m);
  console.log(`Video resolution: ${cols}x${rows}`);
  console.log('Num contours: ', contours.length);
  console.log('Contours: ', Util.className(contours));
  console.log('Contours[0]: ', Util.className(contours[0]));

  // console.log('contours[0]: ', contours[0]);
  console.log('contours[0][0]: ', contours[0][0]);

  let data = {
    width: cols,
    height: rows,
    frame,
    time,
    contours: contours
      .filter(c => c.length >= 4)
      .map(c => c.toString(Contour.FORMAT_NOBRACKET | Contour.FORMAT_SPACE | Contour.FORMAT_01))
      .join('|')
  };
  let body = JSON.stringify(data);
  console.log('body: ', Util.abbreviate(body));

  frame++;

  let response = await fetch('http://127.0.0.1:3001/contours', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body
  });

  console.log('response: ', response);
  return response;
};

/*let args = process.argv;
console.log('args:', args);

if(args.length >= 1) process([new Contour()], [], {});
*/
//client();