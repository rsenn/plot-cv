import { parse2 } from './lib/xml/parse.js';
import ConsoleSetup from './lib/consoleSetup.js';
import PortableFileSystem from './lib/filesystem.js';
import { toXML } from './lib/json.js';
import Util from './lib/util.js';

async function main(...args) {
  await ConsoleSetup({ depth: 20, colors: true, breakLength: 80 });
  await PortableFileSystem();

  let data = filesystem.readFile(args[0] ?? 'BreadboardContacts.out.xml',
    'utf-8'
  );
  console.log('data:', data);

  // let result = parse2(Util.bufferToString(data));
  let result = parse2(data);
  console.log('result:', Util.typeOf(result));
  console.log('result.length:', result.length);
  console.log('result[0]:', result[0]);
  console.log('result:', result);

  let xml = toXML(result, { depth: Infinity, quote: '"', indent: '' });
  console.log('xml:', xml);
}

Util.callMain(main, true);
