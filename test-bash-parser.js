import filesystem from 'fs';
import parse from 'bash-parser';

const consoleOpts = {
  colors: true,
  depth: Infinity,
  compact: 5,
  hideKeys: ['pos']
};

async function main(...args) {

  console.options = consoleOpts;

  const data = filesystem.readFileSync('../cfg.sh');

  const ast = parse(data ?? 'echo ciao;');
  console.log('ast:', ast);
}
main(...scriptArgs.slice(1));
