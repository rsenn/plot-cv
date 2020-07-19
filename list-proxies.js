import ProxyList from '/home/roman/.nvm/versions/node/v14.3.0/lib/node_modules/free-proxy/index.js';
import ProxyLists from '/home/roman/.nvm/versions/node/v14.3.0/lib/node_modules/proxy-lists/index.js';

import { Repeater } from '@repeaterjs/repeater';

const proxies = [
  new Repeater(async (push, stop) => {
    try {
      const proxyList = new ProxyList();
      await push((await proxyList.getByCountryCode('DE')).map(({ ip, port, country }) => [ip, port, country]));
    } catch(error) {
      stop(new Error(error));
    }
  }),
  new Repeater(async (push, stop) => {
    // `getProxies` returns an event emitter.
    ProxyLists.getProxies({
      // options
      countries: ['de', 'at', 'nl']
    })
      .on('data', function(proxies) {
        // Received some proxies.
        console.log('got some proxies');
        console.log(proxies);
        push(proxies);
      })
      .on('error', function(error) {
        // Some error has occurred.
        console.log('error!', error);
        stop(error);
      })
      .once('end', function() {
        // Done getting proxies.
        console.log('end!');
        stop();
      });
  })
];

main();
