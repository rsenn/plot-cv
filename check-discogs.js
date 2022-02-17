import * as cv from 'opencv';
import * as path from 'path';
import Console from 'console';
import { glob } from 'util';
import { LoadConfig } from './config.js';
import { FetchURL, ReadFile, ReadXML, WriteJSON } from './io-helpers.js';

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
          return (
            '\n\t ' + [date, ago, who, what.join(' ')].reduce((line, field, i) => line + field.padEnd(columns[i]), '')
          );
        }
      };

      return [date, ago, who, what.join('\n')].reduce((line, field, i) => line + field.padEnd(columns[i]), '');
    });
    console.log(
      `${parts.length} Nachrichten`,
      console.config({ maxArrayLength: 2, compact: false, stringBreakNewline: true }),
      parts
    );

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
      'user-agent':
        'Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/98.0.4758.74 Safari/537.36',
      accept:
        'text/html,application/xhtml+xml,application/xml;q=0.9,image/avif,image/webp,image/apng,*/*;q=0.8,application/signed-exchange;v=b3;q=0.9',
      'sec-fetch-site': 'same-origin',
      'sec-fetch-mode': 'navigate',
      'sec-fetch-user': '?1',
      'sec-fetch-dest': 'document',
      referer: 'https://www.discogs.com/sell/orders',
      'accept-language': 'en-US,en;q=0.9',
      cookie:
        'sid=deb14330f89995598b4cd37ecd8f0c3d; language2=en; ck_username=diskosenn; session="gZ7nTDR5jpQkxXO7q6F2MuG6Ejw=?_expires=MTY1NjI3NzgxOQ==&auth_token=IktCZ0tWaWdxWkp3cWdubzZkY0RoMXpEb09EIg==&created_at=IjIwMjEtMTItMjhUMjE6MTA6MTkuNDMxNDQzIg==&idp%3Auser_id=ODM2OTAyMg=="; mp_session=ed5700f25fac3c643b872191; ppc_onboard_prompt=seen; __cf_bm=rU7Wrl49zidnmNz1cfYZks5MHHblFsaUHRqkk.3_ehg-1644783026-0-AeEfVFvIDNWHI+ooHhTwSJc44cwl2UZS/5UTnL3Cgew4zlS9OzC3p5rWl/bZMIE35CxnxbE1U8EAbyrRXlpC2+Y='
    }
  });
  console.log('result', result);

  for await(let chunk of result) {
    console.log('chunk', chunk);
  }
}