import { client, server, fetch } from 'net';

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

function getDownloadCount() {
  const res = fetch('https://api.github.com/repos/khanhas/spicetify-cli/releases');
  const dl_count = res.json().reduce((total, tag) => {
    return (total += tag.assets.reduce((tag_total, asset) => {
      return (tag_total += asset.download_count);
    }, 0));
  }, 0);
  print('Fetch:', res.url);
  print('STATUS: ', res.status, 'OK: ', res.ok, 'TYPE: ', res.type);
  print('RESULT: ', dl_count);
  return `${dl_count}`;
}

switch (scriptArgs[1]) {
  case 's':
    CreateServer();
    break;
  case 'c':
    CreateClient();
    break;
  case 'f':
    getDownloadCount();
    break;
}
