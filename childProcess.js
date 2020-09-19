import { Message } from './message.js';
import Util from './lib/util.js';
import { Alea } from './lib/alea.js';
import Stream, { Readable, Writable, PassThrough, Duplex } from 'stream';
import Timers, { TimeoutError } from './lib/repeater/timers.js';
import { spawn } from 'child_process';
import { AsyncWrite, AsyncRead, PipeToRepeater, LineReader, WritableRepeater, WriteIterator, ReadFromIterator } from './lib/stream/utils.js';
import { Repeater } from './lib/repeater/repeater.js';
import ConsoleSetup from './consoleSetup.js';
1;
const prng = new Alea();
async function CreateWebSocket(ws, req) {
  const { connection, client, headers } = req;
  const { path } = req;
  let { remoteAddress, remotePort, localAddress, localPort } = client;
  const { _host, _peername } = connection;
  let { address, port } = _peername;
  const { cookie } = headers;

  if(localAddress == '::1') localAddress = 'localhost';
  if(remoteAddress == '::1') remoteAddress = 'localhost';

  let s = Socket.map(ws,
    {
      local: localAddress.replace(/^::ffff:/, '') + ':' + localPort,
      remote: remoteAddress.replace(/^::ffff:/, '') + ':' + remotePort,
      cookie,
      userAgent: headers['user-agent'],
      path
    },
    { client, connection }
  );
  console.log('WebSocket connected:', s, headers);
}

function execStream(cmd, args, options) {
  var AB = new Stream.Duplex({ ...options, readableHighWaterMark: 1, writableHighWaterMark: 1 });
  var A = new Stream.PassThrough();
  var B = new Stream.PassThrough();
  // console.log = function() {}
  AB._write = function(chunk, encoding, cb) {
    return A.write(chunk, encoding, cb);
  };
  AB.on('finish', function() {
    A.end();
    A.emit('end');
  });
  AB._read = function(n) {};
  B.on('readable', function() {
    AB.push(B.read());
  });
  B.on('end', function() {
    AB.end();
    AB.emit('end');
  });
  var proc = spawn(cmd, args, options);
  A.pipe(proc.stdin);
  proc.stdout.pipe(B);
  return AB;
}

function exec(cmd, args = [], options = { stdio: 'pipe' }) {
  let child = spawn(cmd, args, options);
  const { stdin, stdout, stderr } = child;

  child.stdout.on('data', data => {
    console.log(`stdout: ${data}`);
  });

  child.stderr.on('data', data => {
    console.error(`stderr: ${data}`);
  });

  child.on('close', code => {
    console.log(`child process exited with code ${code}`);
  });
  return [stdin, stdout, stderr];
}

async function main(...args) {
  await ConsoleSetup({ depth: 0, breakLength: 200, maxStringLength: 200, maxArrayLength: 20 });

  Util.getGlobalObject().ReadableStream = Readable;
  Util.getGlobalObject().WritableStream = Writable;

  const [input, output, error] = exec('grbl_sim', []);
  for(let [name, stream] of Object.entries({ input, output, error })) console.debug(`${name}:`, Util.getPrototypeChain(stream));
  console.debug('process.stdin:', process.stdin);

  const stdout = await PipeToRepeater(output);
  /*
const stderr  = await PipeToRepeater(error);

const outStream = Repeater.merge([stdout, stderr]);
*/
  (async () => {
    for await (let data of stdout) {
      console.debug('data:', (await data).toString());
    }
  })();
  let st = new ReadFromIterator(async function* () {
    yield '$\n';
    yield '$$\n';
  });

  st.pipe(input);

  for await (let data of await AsyncRead(error)) console.debug('data:', (await data).toString());

  //process.stdin.pipe(input);
}
Util.callMain(main, true);
