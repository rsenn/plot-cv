import { fetch, LLL_USER, logLevels, setLog } from 'net';
import * as os from 'os';
import { concat, define, memoize, toArrayBuffer, toString, waitFor } from 'util';
import Console from 'console';
import inspect from 'inspect';
import * as std from 'std';

const PutsFunction = outFn => str => {
  let b = toArrayBuffer(str);
  return outFn(b, b.byteLength);
};

const FileWriter = file => {
  let fd = os.open(file, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o644);
  return define(FdWriter(fd, file), {
    close: () => os.close(fd),
  });
};

function FdWriter(fd, name) {
  let fn;
  fn = (buf, len) => {
    if(typeof buf == 'string') buf = toArrayBuffer(buf);
    len ??= buf.byteLength;
    let result = os.write(fd, buf, 0, len);
    return result;
  };
  define(fn, {
    fd,
    name,
    puts: PutsFunction(fn),
    write: fn,
    close: () => {},
    seek: (whence, offset) => os.seek(fd, whence, offset),
    [Symbol.toStringTag]: `FileWriter< ${fd} >`,
    inspect() {
      return inspect({ fd }) ?? this[Symbol.toStringTag];
    },
  });
  return fn;
}

async function main(...args) {
  globalThis.console = new Console({
    inspectOptions: {
      maxStringLength: 200,
      maxArrayLength: Infinity,
      compact: 1,
      depth: Infinity,
    },
  });
  setLog(/*(LLL_DEBUG-1)|*/ LLL_USER, (level, message) => console.log(logLevels[level].padEnd(16), message));
  //
  function DailyPhase(t) {
    t -= t % (21600 * 1000);
    return Math.floor(t);
  }

  function PhaseFile(t) {
    let date = new Date(t);
    let str = date.toISOString().replace(/T.*/g, '');

    let phaseStr = ((date / (21600 * 1000)) % 4) + 1 + '';

    let file = str + '-' + phaseStr + '.txt';

    /* os.setTimeout(() => {
      file.close();
    },   (t+ (21600 * 1000)) - date + 1000);
*/
    return file;
  }

  const files = memoize(phase => PhaseFile(phase));
  const keys = [
    'icao24',
    'callsign',
    'origin_country',
    'time_position',
    'last_contact',
    'longitude',
    'latitude',
    'baro_altitude',
    'on_ground',
    'velocity',
    'true_track',
    'vertical_rate',
    'sensors',
    'geo_altitude',
    'squawk',
    'spi',
    'position_source',
  ];
  //
  for(;;) {
    let t = Date.now();
    let msecs = 10000 - (t % 10000);

    await waitFor(msecs);

    t = Date.now();

    let phase = DailyPhase(t);
    console.log('phase', new Date(phase).toISOString());

    let file = std.open(PhaseFile(phase), 'a');

    let response = await fetch('https://opensky-network.org/api/states/all?lamin=45.8389&lomin=5.9962&lamax=47.8229&lomax=10.5226', {
      sslCert: 'localhost.crt',
      sslPrivateKey: 'localhost.key',
      sslCA: '/etc/ssl/certs/ca-certificates.crt',
      onMessage(req, res) {
        console('URL:', req.url);
        let buf;
        for(let chunk of res) {
          buf = buf ? concat(buf, chunk) : chunk;
        }
        //let obj = (new Function('return '+body+';'))();
      },
    });

    function ProcessResponse(buf) {
      /*  let decoder = new TextDecoder('utf-8');
          let body = decoder.decode(buf).trimEnd();*/
      let body = toString(buf);
      console.log('ProcessResponse', body);
      if(body.codePointAt(body.length - 1) == 0) body = body.slice(0, -1);

      file.puts(body + '\n');
      file.flush();
      file.close();

      let obj = JSON.parse(body);

      obj.states = obj.states.map(item =>
        item.reduce(
          (acc, field, i) => ({
            ...acc,
            [keys[i]]: ['time_position', 'last_contact'].indexOf(keys[i]) != -1 ? new Date(field * 1000) : field,
          }),
          {},
        ),
      );
      obj.states.sort((a, b) => b.baro_altitude - a.baro_altitude);

      console.log('onMessage', console.config({ compact: 1, depth: Infinity, maxArrayLength: Infinity }), obj);
    }

    console.log('response', response);

    ProcessResponse(response.body);
  }
}

main(...scriptArgs.slice(1)).catch(err => {
  console.log('ERROR:', err.message, err.stack);
  std.exit(1);
});