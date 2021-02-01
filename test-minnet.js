import { client, server, fetch } from 'net.so';
import Util from './lib/util.js';
import ConsoleSetup from './lib/consoleSetup.js';

function CreateServer() {
  print('SERVER');
  server({
    port: 3300,
    onConnect: socket => {
      print('Client connected');
      print('Socket: ' + socket);
      socket.send('Hello from server');
    },
    onMessage: (socket, msg) => {
      print('Received: ', msg);
    },
    onClose: why => {
      print('Client disconnected. Reason: ', why);
    },
    onPong: (socket, data) => {
      print('Pong: ', data);
    }
  });
}

function CreateClient() {
  print('CLIENT');
  client({
    port: 7981,
    server: 'localhost',
    onConnect: socket => {
      print('Connected to server');
      socket.send('Hello from client');
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

function getJSON() {
  console.log('getJSON');
  const res = fetch('https://api.github.com/repos/rsenn/plot-cv', {
    method: 'head'
  });
  const { ok, status, type } = res;
  console.log('res:', { ok, status, type });

  const json = res.json();
  console.log('json:', json);

  const data = new Map(Object.entries(json));
  console.log('data:', data);
  return data;
}

async function main(...args) {
  await ConsoleSetup({ depth: 10 });

  switch (args[0]) {
    case 's':
      CreateServer();
      break;
    case 'c':
      CreateClient();
      break;
    case 'f':
      getJSON();
      break;
  }
}

Util.callMain(main, true);
