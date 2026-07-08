import { spawnSync, spawn, WNOHANG } from 'child_process';
import { writeFileSync, reader, readerSync, readAll } from 'fs';
import { TextDecoder } from 'textcode';
import { quote, abbreviate } from 'util';

function GetYouTubeJSONSync(url) {
  const child = spawnSync('yt-dlp', ['-f', 'b[ext=mp4]', '--yes-playlist', '--skip-download', '--cookies', '/home/roman/cookies.txt', '-J', '--download-archive', 'archive.txt', url], {
    stdio: ['inherit', 'pipe', 'inherit'],
  });
  const { stdout, stderr, exitcode, exited } = child;
  if(exitcode) throw new Error(`yt-dlp outputted: ${stderr}`);
  return JSON.parse(stdout);
}

async function GetYouTubeJSON(url) {
  const child = spawn('yt-dlp', ['-f', 'b', '--yes-playlist', '--skip-download', '--cookies', '/home/roman/cookies.txt', '-j', '--download-archive', 'archive.txt', '--force-write-archive', url], {
    stdio: ['inherit', 'pipe', 'inherit'],
  });

  const { stdio } = child;

  const [stdin, stdout, stderr] = stdio;

  console.log('GetYouTubeJSON', { child, stdio });

  const output = await readAll(stdout);
  console.log('stdout', { output });

  return output;
}

async function main(...args) {
  for(const arg of args) {
    const data = await GetYouTubeJSON(arg);

    console.log('stdout', console.config({ compact: false, depth: 3, maxStringLength: Infinity, maxArrayLength: Infinity }), data);
    writeFileSync('out.json', JSON.stringify(data, null, 2));
  }
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error?.message ?? error}\n${error?.stack}`);
}
