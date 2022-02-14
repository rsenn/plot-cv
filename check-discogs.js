import * as cv from 'opencv';
import Console from 'console';
import { LoadConfig } from './config.js';
import { FetchURL } from './io-helpers.js';

async function main(...args) {
  globalThis.console = new Console({
    colors: true,
    depth: 3,
    maxArrayLength: 30,
    compact: 3
  });
  let config = LoadConfig();
  let orderId = '8369022-364';

  let result;

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

main(...scriptArgs.slice(1));
