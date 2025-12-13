import * as bjson from 'bjson';
function WriteFile(name, data) {
  if(Array.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;

  filesystem.writeFile(name, data + '\n');

  console.log(`Wrote ${name}: ${data.length} bytes`);
}

async function main(...args) {
  console.log('bjson:', bjson);
  let ffi = await import('ffi');
  console.log('ffi:', ffi);
  return;
}

main(...scriptArgs.slice(1));