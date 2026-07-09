import { spawnSync, spawn, WNOHANG } from 'child_process';
import { writeFileSync, reader, readerSync, readAll, gets, waitRead } from 'fs';
import { TextDecoder } from 'textcode';
import { quote, abbreviate } from 'util';
import { read, write } from 'json';

/*function GetYouTubeJSONSync(url) {
  const child = spawnSync(
    'yt-dlp',
    [
      '-f',
      'sb0
      '--yes-playlist',
      '--skip-download',
      '--cookies',
      '/home/roman/cookies.txt',
      '-j',
      '--download-archive',
      'archive.txt',
      url,
    ],
    {
      stdio: ['inherit', 'pipe', 'inherit'],
    },
  );
  const { stdout, stderr, exitcode, exited } = child;
  if(exitcode) throw new Error(`yt-dlp outputted: ${stderr}`);
  return JSON.parse(stdout);
}*/

async function GetYouTubeJSON(url) {
  const args = [
    'yt-dlp',
    '-f',
    'sb0',
    '--yes-playlist',
    '--skip-download',
    '--cookies',
    '/home/roman/cookies.txt',
    '-j',
    //'--download-archive', 'archive.txt', '--force-write-archive',
    //'--playlist-items', '1,2',
    url,
  ];

  console.log(`Executing ${args.map(arg => (/[&]/.test(arg) ? "'" + arg + "'" : arg)).join(' ')}`);

  const child = spawn(args[0], args.slice(1), {
    stdio: ['inherit', 'pipe', 'inherit'],
  });

  const [stdin, stdout, stderr] = child.stdio;

  console.log('GetYouTubeJSON', { child });

  let output = '';

  for(;;) {
    await waitRead(stdout);
    const line = gets(stdout);

    console.log('line',  abbreviate(line,120));

    output += line + '\n';
    child.wait(WNOHANG);

    if(child.exited) break;
  }

  return output
    .trimEnd()
    .split(/\n/g)
    .map(s => read(s));
}

async function main(...args) {
  for(const arg of args) {
    const data = await GetYouTubeJSON(arg);

    console.log(
      'data',
      console.config({
        compact: false,
        depth: 3,
        maxStringLength: Infinity,
        maxArrayLength: Infinity,
      }),
      data,
    );
    writeFileSync('out.json', write(data, 2));
  }
}

try {
  await main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error?.message ?? error}\n${error?.stack}`);
}
