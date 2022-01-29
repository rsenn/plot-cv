import * as os from 'os';
import { ReadFd } from './io-helpers.js';

function main(...args) {
  let [rd, wr] = os.pipe();

  os.exec(
    [
      'curl',
      'https://service.post.ch/vgkklp/info/informationen/ProdukteAnzeigen/',
      '-H',
      'authority: service.post.ch',
      '-H',
      'cache-control: max-age=0',
      '-H',
      'sec-ch-ua: "(Not(A:Brand";v="8", "Chromium";v="98", "Google Chrome";v="98"',
      '-H',
      'sec-ch-ua-mobile: ?0',
      '-H',
      'sec-ch-ua-platform: "Linux"',
      '-H',
      'upgrade-insecure-requests: 1',
      '-H',
      'origin: https://service.post.ch',
      '-H',
      'content-type: application/x-www-form-urlencoded',
      '-H',
      'user-agent: Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/98.0.4758.66 Safari/537.36',
      '-H',
      'accept: text/html,application/xhtml+xml,application/xml;q=0.9,image/avif,image/webp,image/apng,*/*;q=0.8,application/signed-exchange;v=b3;q=0.9',
      '-H',
      'sec-fetch-site: same-origin',
      '-H',
      'sec-fetch-mode: navigate',
      '-H',
      'sec-fetch-user: ?1',
      '-H',
      'sec-fetch-dest: document',
      '-H',
      'referer: https://service.post.ch/',
      '-H',
      'accept-language: en-US,en;q=0.9',
      '-H',
      'cookie: user_profile_profileInfo_loginStatusCookie=undefined; x-unblu-device="k30u5TFMRXe9cHJ5rcRpyw"; user_profile_attributes_primarySegment=unknown; NPKlpipSession=602c11ac1791LF1rN3eEqekbIxVBEXoaoOjR97w8lmLHQe1YxJ6xjDAGyK; SC_ANALYTICS_GLOBAL_COOKIE=569e3db2b91441a69bc8369627d7f9ff|False; utag_main=v_id:017ea31ae00a001c4099fb7fac3105068005f06000bd0$_sn:1$_ss:0$_st:1643415994047$ses_id:1643413954571%3Bexp-session$_pn:13%3Bexp-session',
      '--data-raw',
      'f0d8546538d522=2fb31f55353a4b7e8a394126ece70c20&bb386538d52269=0&f7dc99bdac2883=1&InfoInt.DestinationGewaehlt=128559483&InfoInt.Gewicht=1.000',
      '--compressed'
    ],
    { stdout: wr, block: false }
  );
  os.close(wr);
  let html = ReadFd(rd);

  console.log('html', html);
}

main(...scriptArgs.slice(1));
