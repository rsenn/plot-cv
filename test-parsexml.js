import parse from './lib/xml/parse.js';
import ConsoleSetup from './lib/consoleSetup.js';
import PortableFileSystem from './lib/filesystem.js';
import tXml from './lib/tXml.js';

async function main(...args) {
  await ConsoleSetup({ depth: 20, colors: true, breakLength: 80 });
  await PortableFileSystem();

  let data = filesystem.readFile(args[0] ?? 'BreadboardContacts.out.xml');

  let result = parse(data);
  console.log('result:', result);
}

Util.callMain(main, true);
