net.client({
  host: '127.0.0.1',
  port: 22,
  raw: true,
  onMessage(ws, msg) {
    console.log('onMessage', ws, msg);
  }
});
