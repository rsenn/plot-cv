import * as net from 'net.so';
import contour from 'contour.so';
import PortableConsole from './lib/consoleSetup.js';
import Util from './lib/util.js';
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

globalThis.process = async function process(contours, hier, ...args) {
  //let size = new Size(imgOriginal.cols, imgOriginal.rows);
  const [images, ...rest] = args;
  const {
    cols,
    rows,
    imgBlurred,
    imgCanny,
    imgGrayscale,
    imgMorphology,
    imgOriginal,
    imgRaw,
    imgTemp,
    imgVector
  } = images;
  console.log('console.log: ', console.log);
  let m = imgRaw;
  /*  console.log('Images: ', typeof images);
  console.log('Images: ', Object.keys(images));
  console.log('Images: ', Util.inspect(images));
  console.log('rest.length: ', rest.length);
  console.log('rest: ', ...rest);*/

  console.log('imgRaw: ', Util.className(m));
  console.log(`Video resolution: ${cols}x${rows}`);
  console.log('Num contours: ', contours.length);
  console.log('Contours: ', Util.className(contours));
  console.log('Contours[0]: ', Util.className(contours[0]));

  // console.log('contours[0]: ', contours[0]);
  console.log('contours[0][0]: ', contours[0][0]);

  let body = contours.map(contour => [...contour].map(({ x, y }) => `{x:${x},y:${y}}`)).join(',');

  console.log('body: ', body);

  let response = await fetch('http://127.0.0.1:3000/contours', { method: 'POST', body });
  console.log('response: ', response);
};

//client();
