import { spawnSync, spawn, WNOHANG } from 'child_process';
import { reader, readerSync, readAll } from 'fs';
import { TextDecoder } from 'textcode';
import { quote, abbreviate } from 'util';

function main(...args) {
  for(let arg of args) {
    const child = spawnSync('yt-dlp', ['-f', 'best', '--yes-playlist', '--skip-download', '-J', arg], {
      stdio: ['inherit', 'pipe', 'pipe'],
    });
    console.log('child', child);

    const { stdout, stderr, status } = child;
    console.log('stdout', abbreviate(stdout));
    console.log('stderr', abbreviate(stderr));

    /*    let result = child.waitSync(0);
    console.log('result',  result ); */
  }
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error?.message ?? error}\n${error?.stack}`);
} finally {
  //console.log('SUCCESS');
}
