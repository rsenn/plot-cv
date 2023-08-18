let filesystem;
let globalThis;

main(...scriptArgs.slice(1));
  let { message, stack } = error;

  console.log('ERROR message =', message);
  console.log('ERROR stack:\n' + stack);
});

async function main(...args) {

  console.log('main args =', args);


  globalThis = globalThis;

  throw new Error('This is an error');
}
