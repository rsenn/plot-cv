import * as os from 'os';
import { ReadFd } from './io-helpers.js';
function main(...args) {
  let [rd, wr] = os.pipe();

  os.exec(
    [
      'curl',
      '-v',
      'https://service.post.ch/vgkklp/info/informationen?lang=en&service=vgk-infoint&shortcut=info-int',
      '-H',
      'user-agent: Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/98.0.4758.66 Safari/537.36',
      '--compressed'
    ],
    { stdout: wr, block: false }
  );
  os.close(wr);
  let html = ReadFd(rd);

  console.log('html', html);
}

main(...scriptArgs.slice(1));