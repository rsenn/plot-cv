import { client, LLL_DEBUG, LLL_WARN, setLog } from 'net';
import * as os from 'os';
import { MIDIEvent, MIDIStream } from './lib/midi.js';
import { quote } from './lib/misc.js';
import { Console } from 'console';
import * as std from 'std';
/*const MIDI_NOTE_OFF = 0x80;

const MIDI_NOTE_ON = 0x90;

const MIDI_POLYPHONIC_KEY_PRESSURE = 0xA0;
const MIDI_CONTROL_CHANGE = 0xB0;
const MIDI_PROGRAM_CHANGE = 0xC0;
const MIDI_CHANNEL_PRESSURE = 0xD0;
const MIDI_PITCH_BEND = 0xE0;
const MIDI_SYSEX = 0xf0;
const MIDI_SONG_POSITION = 0xf2;
const MIDI_SONG_SELECT = 0xf3;
const MIDI_TUNE_REQUEST = 0xf6;
const MIDI_EOX = 0xf7;
const MIDI_TIMING_CLOCK = 0xf8;
const MIDI_START = 0xfa;
const MIDI_CONTINUE = 0xfb;
const MIDI_STOP = 0xfc;
const MIDI_ACTIVE_SENSING = 0xfe;
const MIDI_RESET = 0xff
*/
/* MIDIMessageLength -- how many bytes in a message? */
function MIDIMessageLength(byte) {
  byte &= 0xff;
  if(byte < 0x80) {
    return 0;
  } else if(byte < 0xf0) {
    const length = [3, 3, 3, 3, 2, 2, 3];
    return length[(byte - 0x80) >> 4];
  } else {
    const length = [-1, 2, 3, 2, 0, 0, 1, -1, 1, 0, 1, 1, 1, 0, 1, 1];
    return length[byte - 0xf0];
  }
}

class MIDIMessage extends ArrayBuffer {
  constructor(bytes) {
    super(bytes.length);
    let u8 = new Uint8Array(this);
    for(let i = 0; i < bytes.length; i++) u8[i] = bytes[i];
  }

  /* prettier-ignore */ get command() { return new Uint8Array(this)[0] & 0xf0; }
  /* prettier-ignore */ set command(value) { let a=new Uint8Array(this)[0]; a[0] = (a[0] & 0x0f) | (value & 0xf0); }

  /* prettier-ignore */ get channel() { return new Uint8Array(this)[0] & 0x0f; }
  /* prettier-ignore */ set channel(value) { let a=new Uint8Array(this)[0]; a[0] = (a[0] & 0xf0) | (value & 0x0f); }

  /* prettier-ignore */ get data() { return new Uint8Array(this, 1); }

  [Symbol.inspect](depth, opts) {
    return `MIDIMessage ` + inspect([...new Uint8Array(this)], { numberBase: 16 });
  }
}

function MIDIMessageRead(byteArr) {
  let len;
  while(byteArr.length && !(len = MIDIMessageLength(byteArr[0]))) byteArr.shift();
  if(len && byteArr.length >= len) {
    console.log('MIDIMessageRead', { len });
    return new MIDIMessage(byteArr.splice(0, len));
  }
  return null;
}

function MIDIMessageDecode(byteArr) {
  const command = byteArr[0] & 0xf0;
  const channel = byteArr[0] & 0x0f;
  const len = MIDIMessageLength(byteArr[0]);
  const data = byteArr.slice(1, len);
  return { command, channel, data };
}

function TCPClient(url) {
  let recvBuf = [];
  let status = [];
  return client(url, {
    binary: true,
    onConnect(ws, req) {
      console.log('onConnect', { ws, req });
    },
    onClose(ws, reason) {
      console.log('onClose', { ws, reason });
    },
    onRequest(req, resp) {
      console.log('onRequest', { req, resp });
    },
    onFd(fd, rd, wr) {
      //console.log('onFd', fd, rd, wr);
      os.setReadHandler(fd, rd);
      os.setWriteHandler(fd, wr);
    },
    onMessage(ws, data) {
      //console.log('onMessage', { ws, data });
      try {
        let stream = new MIDIStream(data);
        let event = MIDIEvent.read(stream, status);
        console.log('onMessage', { event });
      } catch(e) {
        console.log('onMessage.exception', e.message, e.stack);
      }
    },

    onError(ws, error) {
      console.log('onError', ws, error);
    }
  });
}

function main(...args) {
  globalThis.console = new Console(std.err, {
    inspectOptions: { compact: 2, customInspect: true /*, numberBase: 16*/ }
  });
  console.log('midi', Object.keys({ MIDIEvent, MIDIStream }));

  const debug = false;

  setLog(((debug ? LLL_DEBUG : LLL_WARN) << 1) - 1, (level, msg) => {
    let p = ['ERR', 'WARN', 'NOTICE', 'INFO', 'DEBUG', 'PARSER', 'HEADER', 'EXT', 'CLIENT', 'LATENCY', 'MINNET', 'THREAD'][level && Math.log2(level)] ?? level + '';
    msg = msg.replace(/\n/g, '\\n').replace(/\r/g, '\\r');

    if(!/POLL/.test(msg) && /MINNET/.test(p)) if (debug && /(client|http|read|write)/i.test(msg)) console.log(p.padEnd(8), msg);
  });

  let url = args[0] ?? 'tcp://127.0.0.1:6999';

  os.signal(os.SIGINT, undefined);
  os.ttySetRaw(0);
  os.setReadHandler(0, () => {
    let b = new ArrayBuffer(1);
    let r = os.read(0, b, 0, 1);
    if(r == 1) {
      let [num] = new Uint8Array(b);
      let ch = String.fromCharCode(num);
      console.log('Read byte ', num, ` ${quote(ch, "'")}`);

      if(num == 4) std.exit(0);
    }
  });

  TCPClient(url);
}

main(...scriptArgs.slice(1));