import parse from './lib/xml/parse.js';
import ConsoleSetup from './lib/consoleSetup.js';
import PortableFileSystem from './lib/filesystem.js';
import tXml from './lib/tXml.js';
import { toXML } from './lib/json.js';

async function main(...args) {
  await ConsoleSetup({ depth: 20, colors: true, breakLength: 80 });
  await PortableFileSystem();

  let data = filesystem.readFile(args[0] ?? 'BreadboardContacts.out.xml', null);
  console.log('data:', data);

  let result = parse(new Uint8Array(data));
  console.log('result:', result);

  let xml = toXML(result);
  console.log('xml:', xml);
}

Util.callMain(main, true);
