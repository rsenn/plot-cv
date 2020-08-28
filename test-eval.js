const code = `
 function _processChunk(chunk, flushFlag, cb) {
      var availInBefore = chunk && chunk.length;
      var availOutBefore = this._chunkSize - this._offset;
      var inOff = 0;

      var self = this;

      var async = typeof cb === 'function';

      if(!async) {
        var buffers = [];
        var nread = 0;

        var error;
        this.on('error', function (er) {
          error = er;
        });

        assert(this._handle, 'zlib binding closed');
        do {
          var res = this._handle.writeSync(flushFlag,
            chunk, // in
            inOff, // in_off
            availInBefore, // in_len
            this._buffer, // out
            this._offset, //out_off
            availOutBefore
          ); // out_len
        } while(!this._hadError && callback(res[0], res[1]));

        if(this._hadError) {
          throw error;
        }

        if(nread >= kMaxLength) {
          _close(this);
          throw new RangeError(kRangeErrorMessage);
        }

        var buf = Buffer.concat(buffers, nread);
        _close(this);

        return buf;
      }

      assert(this._handle, 'zlib binding closed');
      var req = this._handle.write(flushFlag,
        chunk, // in
        inOff, // in_off
        availInBefore, // in_len
        this._buffer, // out
        this._offset, //out_off
        availOutBefore
      ); // out_len

      req.buffer = chunk;
      req.callback = callback;

      function callback(availInAfter, availOutAfter) {
        // When the callback is used in an async write, the callback's
        // context is the req object that was created. The req object
        // is === this._handle, and that's why it's important to null
        // out the values after they are done being used. this._handle
        // can stay in memory longer than the callback and buffer are needed.
        if(this) {
          this.buffer = null;
          this.callback = null;
        }

        if(self._hadError) return;

        var have = availOutBefore - availOutAfter;
        assert(have >= 0, 'have should not go down');

        if(have > 0) {
          var out = self._buffer.slice(self._offset, self._offset + have);
          self._offset += have;
          // serve some output to the consumer.
          if(async) {
            self.push(out);
          } else {
            buffers.push(out);
            nread += out.length;
          }
        }

        // exhausted the output buffer, or used all the input create a new one.
        if(availOutAfter === 0 || self._offset >= self._chunkSize) {
          availOutBefore = self._chunkSize;
          self._offset = 0;
          self._buffer = Buffer.allocUnsafe(self._chunkSize);
        }

        if(availOutAfter === 0) {
          // Not actually done.  Need to reprocess.
          // Also, update the availInBefore to the availInAfter value,
          // so that if we have to hit it a third (fourth, etc.) time,
          // it'll have the correct byte counts.
          inOff += availInBefore - availInAfter;
          availInBefore = availInAfter;

          if(!async) return true;

          var newReq = self._handle.write(flushFlag, chunk, inOff, availInBefore, self._buffer, self._offset, self._chunkSize);
          newReq.callback = callback; // this same function
          newReq.buffer = chunk;
          return;
        }

        if(!async) return false;

        // finished with the chunk.
        cb();
      }
    };
    const Zlib = { _chunkSize: 3, _offset: 0, _processChunk, _handle: {} };

    Zlib._processChunk('abcdefg', false, () => {});

    function test(a,b) {
      return a+b;
    }
    let c = test(10,5);
    console.log("Test = "+c);
    c;
 `;

import { Environment, ECMAScriptParser, Printer } from './lib/ecmascript.js';
import Util from './lib/util.js';
import PortableFileSystem from './lib/filesystem.js';

let filesystem;

async function main(args) {
  filesystem = await PortableFileSystem();

  let file = 'lib/geom/point.js';

  let data = code || filesystem.readFile(file).toString();

  let parser = new ECMAScriptParser(data, code ? process.argv[1].replace(/.*\//g, '') : file, true);
  let ast = parser.parseProgram();
  let printer = new Printer({ indent: 2 });

  parser.addCommentsToNodes(ast);

  let env = new Environment([
    {
      Symbol: { species: Symbol.for('species') },
      console: {
        log(...args) {
          console.debug('console.log(', ...args.map((arg) => `'${arg}'`).reduce((acc, arg) => (acc ? [...acc, ',', arg] : [arg]), null), ')');
        }
      }
    }
  ]);

  Util.safeCall(() => {
    let iter = env.generate(ast);
    console.log('iter:', iter);

    for(let it of iter()) console.info('it:', it);
  });

  let output = printer.print(ast);

  let outputFile = 'output.es';
  console.log(`wrote '${outputFile}'`, await filesystem.writeFile('output.es', output));
}

Util.callMain(main, true);
