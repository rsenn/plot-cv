let filesystem, globalThis;

main(...scriptArgs.slice(1));

async function main(...args) {

  console.log('main args =', args);


  globalThis = globalThis;

  throw new Error('This is an error');
}
