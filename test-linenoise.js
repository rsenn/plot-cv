import linenoise from './linenoise.js';
function main(...args) {
  linenoise.HistoryLoad('history.txt'); /* Load the history at startup */
  let running = true;
  while(running) {
    let line = linenoise.linenoise('Test> ');
    if(line === 0 || line == 'quit') running = false;
    if(line && line.trim().length) {
      linenoise.HistoryAdd(line);
      linenoise.HistorySave('history.txt');
      console.log('line:', line);
    }
  }
}

main();