import * as net from "net.so"
import PortableConsole from "./lib/consoleSetup.js"
const { client, server, fetch } = net;


PortableConsole(console => 
  console.log('net: ', { client, server, fetch } ));

function connect() {
    print("CLIENT")
    net.client({
        port: 3001,
        server: "localhost",
        onConnect: (socket) => {
            print("Connected to server")
            socket.send("hello")
        },
        onMessage: (socket, msg) => {
            print("Received from server: ", msg)
        },
        onClose: (why) => {
            print("Disconnected from server. Reason: ", why)
        },
        onPong: (socket, data) => {
            print("Pong: ", data)
        }
    });
}

function process(contours, hier, ...args) {
  //let size = new Size(imgOriginal.cols, imgOriginal.rows);
  const [images, ...rest] = args;
  const { imgBlurred, imgCanny, imgGrayscale, imgMorphology, imgOriginal, imgRaw, imgTemp, imgVector } = images;

  let m = imgOriginal;
  console.log('Images: ', typeof images);
  console.log('Images: ', Object.keys(images));
  console.log('rest.length: ', rest.length);
  console.log('rest: ', ...rest);

  console.log(`Video resolution: ${m.cols}x${m.rows}`);
  imgBlurred, imgCanny, imgGrayscale, imgMorphology, imgOriginal, imgRaw, imgTemp, imgVector;

  console.log('Num contours: ', contours.length);
}


//client();