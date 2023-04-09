import PortableFileSystem from './lib/filesystem.js';
import { WritableStream } from './lib/stream/writableStream.js';

let filesystem;

async function main() {
  await PortableFileSystem(fs => (filesystem = fs));

  let s = await filesystem.open('tmp/7seg-2.54.brd');
  let r = await filesystem.read(s);

  ws = await filesystem.open('tmp/test.txt', 'w');
}

main(...scriptArgs.slice(1));
