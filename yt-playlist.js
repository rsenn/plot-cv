import {spawnSync, spawn } from 'child_process';
import { reader, readerSync } from 'fs';

 function main(...args) {
  for(let arg of args) {
    let child = spawnSync('yt-dlp', ['-f', 'best', '--yes-playlist', '--skip-download', '-J', arg], {
      block: false,
      stdio: ['inherit', 'pipe', 'pipe'],
    });

    const { stdio } = child;

    console.log('child', { child, stdio, result });

    let result =  child.wait();

  }
}

try {
   main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error?.message ?? error}\n${error?.stack}`);
} finally {
  //console.log('SUCCESS');
}
