import { spawn, WNOHANG } from 'child_process';
import * as path from 'path';
import { glob } from 'util';
import { LoadConfig } from './config.js';
import { FetchURL, ReadFile, WriteJSON } from './io-helpers.js';
import Console from 'console';
function ReadCallback(fd, fn = data => {}) {
  let buf = new ArrayBuffer(1024);
  setReadHandler(fd, () => {
    let r = read(fd, buf, 0, 1024);
    if(r <= 0) {
      close(fd);
      setReadHandler(fd, null);
      return;
    }
    let data = buf.slice(0, r);
    data = toString(data);
    fn(data);
  });
}

function FetchURL(url, options = {}) {
  let { headers, proxy, cookies = 'cookies.txt', range, body, version = '1.1', tlsv, 'user-agent': userAgent } = options;

  let args = Object.entries(headers ?? {})
    .reduce((acc, [k, v]) => acc.concat(['-H', `${k}: ${v}`]), [])
    .concat(Array.isArray(url) ? url : [url]);

  args.push('--compressed');
  args.unshift('-L', '-k');

  if(body) args.unshift('-d', body);
  if(version) args.unshift('--http' + version);
  if(tlsv) args.unshift('--tlsv' + tlsv);
  if(userAgent) args.unshift('-A', userAgent);
  if(range) args.unshift('-r', range);
  if(cookies) args.unshift('-c', cookies);
  if(proxy) args.unshift('-x', proxy);

  args.unshift('--tcp-fastopen', '--tcp-nodelay');

  console.log('FetchURL', console.config({ maxArrayLength: Infinity, compact: false }), { args });

  let child = spawn('curl', args, {
    block: false,
    stdio: ['inherit', 'pipe', 'pipe']
  });

  let [, out, err] = child.stdio;

  console.log('child', { out, err });
  let output = '',
    errors = '';

  ReadCallback(out, data => (output += data));
  ReadCallback(err, data => ((errors += data), err.puts(data), err.flush()));

  child.wait(WNOHANG);
  let status;

  status = child.wait();
  console.log('FetchURL', { status });

  return output;
}

async function main(...args) {
  globalThis.console = new Console({
    colors: true,
    depth: 3,
    maxArrayLength: 30,
    compact: 1
  });
  let config = LoadConfig();
  let orderId = '8369022-364';

  let result;
  let orders = new Map();

  let addressExpr = /<\/?address[^>]*>/;
  let totalExpr = /<strong>\s*Total/;

  if(args.length == 0) args = glob('tmp/discogs/*').filter(n => !/\.json$/.test(n));

  for(let file of args) {
    //    if(/\.json$/.test(file)) continue;

    let data,
      id = path.basename(file);
    console.log('fetching order: ' + id);

    data = await dl(id);

    console.log('Order:   ' + ColorStr([24, 160, 255], 'https://www.discogs.com/sell/order/' + id));

    continue;

    data = ReadFile(file);

    let re = new RegExp('<div class="thread_content">', 'g');

    let address = data.substring(data.search(addressExpr) + 1);
    address = address.substring(address.indexOf('>') + 1, address.search(addressExpr));
    address = address.replace(/<br>/g, '\n');
    address = address.trim();
    address = address.replace(/\n\s*\n.*/g, '');

    address = address.substring(0, address.indexOf('Paypal address:'));

    console.log('Addresse:\n\t ' + ColorStr([192, 255, 0], address).replace(/\n/g, '\n\t '));
    function ColorStr(c, str) {
      return str;
      return `\x1b[38;2;${c.join(';')}m${str}\x1b[0m`;
    }

    let total = data.substring(data.search(totalExpr) + 1);

    total = total.substring(total.indexOf('<span'), total.indexOf('</span'));

    total = total.replace(/<[^>]+>/g, '');
    total = total.trim();

    console.log('Total:\t ' + ColorStr([255, 64, 0], total));

    data = data.replace(/\n\s+/gm, '\n');
    let parts = data.split(re).slice(1, -1);
    parts = parts.map(p => p.trim());
    //parts = parts.map(p => p.split(/<\/?[-0-9A-Za-z]+>/g));

    parts = parts.map(p =>
      p
        .split(/<[^>]+>/g)
        .map(str => str.trim())
        .filter(str => str.trim() != '')
    );
    let columns = [20, 20, 16, 0];

    parts = parts.map(part => {
      const [date, ago, who, ...what] = part;

      return {
        date,
        ago,
        who,
        what: what.join('\n'),

        [Symbol.inspect](depth, options) {
          return '\n\t ' + [date, ago, who, what.join(' ')].reduce((line, field, i) => line + field.padEnd(columns[i]), '');
        }
      };

      return [date, ago, who, what.join('\n')].reduce((line, field, i) => line + field.padEnd(columns[i]), '');
    });
    console.log(`${parts.length} Nachrichten`, console.config({ maxArrayLength: 2, compact: false, stringBreakNewline: true }), parts);

    WriteJSON(file + '.json', parts);

    // console.log(`order #${id}`, obj);
    orders.set(id, parts);
    console.log('\n' + '─'.repeat(80) + '\n');
  }

  //console.log('orders', orders);

  let entries = [...orders.entries()];

  WriteJSON('discogs.json', Object.fromEntries(entries));
}

main(...scriptArgs.slice(1));

async function dl(orderId) {
  result = await FetchURL('https://www.discogs.com/sell/order/' + orderId, {
    headers: {
      authority: 'www.discogs.com',
      'sec-ch-ua': '"(Not(A:Brand";v="8", "Chromium";v="98", "Google Chrome";v="98"',
      'sec-ch-ua-mobile': '?0',
      'sec-ch-ua-platform': '"Linux"',
      'upgrade-insecure-requests': '1',
      'user-agent': 'Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/98.0.4758.74 Safari/537.36',
      accept: 'text/html,application/xhtml+xml,application/xml;q=0.9,image/avif,image/webp,image/apng,*/*;q=0.8,application/signed-exchange;v=b3;q=0.9',
      'sec-fetch-site': 'same-origin',
      'sec-fetch-mode': 'navigate',
      'sec-fetch-user': '?1',
      'sec-fetch-dest': 'document',
      referer: 'https://www.discogs.com/sell/orders',
      'accept-language': 'en-US,en;q=0.9',
      cookie:
        'cookie: sid=deb14330f89995598b4cd37ecd8f0c3d; language2=en; mp_session=ed5700f25fac3c643b872191; OptanonConsent=isIABGlobal=false&datestamp=Mon+Feb+14+2022+09%3A15%3A59+GMT%2B0100+(Central+European+Standard+Time)&version=6.20.0&hosts=&consentId=16f6b226-a0fd-429d-ba34-0bdad57d38f1&interactionCount=1&landingPath=https%3A%2F%2Fwww.discogs.com%2Fsell%2Fundefined&groups=C0001%3A1%2CC0004%3A1%2CC0003%3A1%2CC0002%3A1%2CSTACK8%3A0; currency=USD; ck_username=diskosenn; ppc_onboard_prompt=seen; session="5V0o/D1Lm2v3OYz32dQNvkTeAkE=?_expires=MTY2MjM3NDY4MQ==&auth_token=IktCZ0tWaWdxWkp3cWdubzZkY0RoMXpEb09EIg==&created_at=IjIwMjItMDMtMDlUMTA6NDQ6NDEuMjc3MDkxIg==&idp%3Auser_id=ODM2OTAyMg=="; __cf_bm=87380KFs8sEkEzylVP4kiA2ULlZ5mStrXUTZ7KpJE88-1647469525-0-ASnGsOYRSM4SnY6xqBsIgusJAOyZdwJ/ANWVSqv+VTXPWaTp4+7KNp9xihb9nHpr8Vkcam6hWHB7ESQ4lXD/7E4='
    }
  });
  console.log('result', result);

  for await(let chunk of result) {
    console.log('chunk', chunk);
  }
}