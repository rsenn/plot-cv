import { readAllSync } from 'fs';
import { Worker, pipe } from 'os';
import { Console } from 'console';
import inspect from 'inspect';
import * as std from 'std';
import { spawn } from 'child_process';
import { reader, write, writeFileSync, existsSync } from 'fs';
import { basename, extname } from 'path';
import { toString } from 'util';

globalThis.console = new Console(std.open('upload-worker.log', 'a+'), {
  inspectOptions: {
    compact: 2,
    customInspect: true,
    maxArrayLength: 200,
    prefix: '\x1b[38;5;220mWORKER\x1b[0m'
  }
});

const worker = Worker.parent;

console.log('worker started!');

worker.onmessage = async ({ data }) => {
  const [id, msg] = data;

  const fn = {
    ReadExiftool,
    HeifConvert,
    MagickResize,
    async PostUpload(sha1, filename, storage, address) {
      let exif = await ReadExiftool(storage);
      let ext = extname(storage);
      let base = storage.slice(0, -ext.length);

      let obj = {
        filename,
        storage,
        uploaded: Date.now(),
        address,
        exif
      };

      console.log('PostUpload', obj);

      if(/\.hei[cf]$/gi.test(filename)) {
        await HeifConvert(storage, base + '.jpg');

        if(existsSync(base + '.jpg')) obj.jpg = base + '.jpg';
      }
      let width = '',
        height = '256';

      if(exif) {
        const { ImageSize, ImageHeight, ImageWidth } = exif;
        let aspect = ImageWidth / ImageHeight;
        if(aspect >= 1) {
          width = 256;
          height = width / aspect;
        } else {
          /* height = 256;
           width = height * aspect;*/
        }
      }

      MagickResize(obj.jpg ?? storage, base + '.thumb.jpg', obj.exif?.Rotation ?? 0, width, height);
      if(existsSync(base + '.thumb.jpg')) obj.thumbnail = base + '.thumb.jpg';

      writeFileSync(base + '.json', JSON.stringify(obj));
      obj.json = base + '.json';
      return obj;
    }
  }[msg.command];

  let result = await fn(...msg.args);
  console.log('worker.onmessage', { id, result });

  worker.postMessage([id, false, result]);
};

async function ReadExiftool(file) {
  const [rd, wr] = pipe();
  const child = spawn('exiftool', ['-S', '-ee', file], {
    block: false,
    stdio: ['inherit', wr, wr]
  });

  let output = '';
  for await(let chunk of reader(rd)) {
    output += toString(chunk);
  }

  console.log('ReadExiftool', { file, child, output });

  await child.wait();

  return Object.fromEntries(
    output
      .split(/\r?\n/g)
      .filter(l => l != '')
      .map(line => [line, line.indexOf(': ')])
      .map(([line, idx]) => [line.slice(0, idx), line.slice(idx + 2)])
  );
}

async function HeifConvert(src, dst, quality = 100) {
  console.log('HeifConvert', src, dst);

  let child = spawn('heif-convert', ['-q', quality + '', src, dst], {
    block: false,
    stdio: ['inherit', 'inherit', 'inherit']
  });

  console.log('HeifConvert', child);
  await child.wait();
  return child.exitcode;
}

async function MagickResize(src, dst, rotate = 0, width, height) {
  console.log('MagickResize', { width, height, dst, rotate });
  let child = spawn('convert', [src, '-resize', width + 'x' + height, ...(rotate ? ['-rotate', '-' + rotate] : []), dst], { block: false, stdio: ['inherit', 'inherit', 'inherit'] });

  console.log('MagickResize', { child });
  await child.wait();
  return child.exitcode;
}