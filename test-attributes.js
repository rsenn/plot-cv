import tXml from './lib/tXml.js';
import { XMLIterator } from './lib/xml/util.js';
import inspect from 'inspect';
//prettier-ignore

function readXML(filename) {
  //console.log('readXML', filename);
  let data = filesystem.readFileSync(filename);
  let xml = tXml(data);
  //console.log('xml:', xml);
  return xml;
}

//TODO: Test with tmScheme (XML) and ColorMap

const push_back = (arr, ...items) => [...(arr || []), ...items];

const push_front = (arr, ...items) => [...items, ...(arr || [])];

async function main(...args) {
  console.log('args:\n  ' + args.join('\n  '));

  const varNames = [
    'CLASSPATH',
    'COLORTERM',
    'DEFAULTS_PATH',
    'DESKTOP_SESSION',
    'DISPLAY',
    'EDITOR',
    'HOME',
    'LANG',
    'LANGUAGE',
    'LESS',
    'LOGNAME',
    'OLDPWD',
    'PAGER',
    //  'PATH',
    'PROMPT_COMMAND',
    'PWD',
    'SESSION_MANAGER',
    'SHELL',
    'SHLVL',
    'STY',
    'TERM',
    'USER',
    'VISUAL',
    'WINDOW'
  ];
  let envEntries = Util.chunkArray(await Promise.all(varNames.reduce((acc, n) => [...acc, n, Util.getEnv(n)], [])), 2);
  let envMap = new Map(envEntries);
  //console.log('Environment:', Util.toSource(envEntries, { quote: '"'}).replaceAll('\n', "\\n"));
  console.log('Environment:', inspect(envMap));

  console.log('OK');
  let colors, keys;
  let attributes = new Map();
  let numeric = new Set();
  const printSet = set => [...set.values()].map(n => "'" + n + "'").join(', ');

  const setAttributes = (tag, attrs) => {
    if(!attributes.has(tag)) attributes.set(tag, new Set());

    let s = attributes.get(tag);

    for(let name in attrs) {
      s.add(name);

      const value = attrs[name];

      if(Util.isNumeric(value)) numeric.add(name);
    }
  };

  try {
    for(let filename of args) {
      let xml = readXML(filename);

      for(let [element, path] of XMLIterator.iterate(xml[0])) {
        //console.log('element:',element.tagName, element.attributes);
        setAttributes(element.tagName, element.attributes);
      }
    }
  } catch(err) {
    Util.putError(err);
  }

  for(let [tag, attr] of attributes) {
    let names = [...attr.values()];
    if(names.length == 0) continue;

    console.log(` ${tag}: [${printSet(attr)}],`);
  }

  console.log('numeric: ' + printSet([...numeric.values()].sort()));
}

main(...scriptArgs.slice(1));
//Util.callMain(main);