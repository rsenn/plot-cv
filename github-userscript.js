// ==UserScript==
// @name         GitHub
// @namespace    https://github.com/
// @version      0.1
// @description  try to take over the world!
// @author       You
// @match        https://github.com/
// @icon         https://www.google.com/s2/favicons?sz=64&domain=github.com
// @grant        none
// ==/UserScript==

(function () {
  'use strict';

  window.getRepos = () =>
    [...document.querySelectorAll('#user-repositories-list > ul > li > div.col-10.col-lg-9.d-inline-block')].map(e => [
      e.querySelector('a').href,
      e.querySelector('h3').nextElementSibling?.querySelector('a')?.href
    ]);
  window.parseXML = xmlStr => {
    const parser = new DOMParser();
    return parser.parseFromString(xmlStr, 'text/html');
  };
  window.fetchURL = url =>
    fetch(url, {
      headers: {
        accept: 'text/html, application/xhtml+xml',
        'accept-language': 'en-US,en;q=0.9',
        'cache-control': 'no-cache',
        pragma: 'no-cache',
        'sec-ch-ua': '"Chromium";v="118", "Brave";v="118", "Not=A?Brand";v="99"',
        'sec-ch-ua-mobile': '?0',
        'sec-ch-ua-platform': '"Linux"',
        'sec-fetch-dest': 'empty',
        'sec-fetch-mode': 'cors',
        'sec-fetch-site': 'same-origin',
        'sec-gpc': '1',
        'turbo-frame': 'user-profile-frame'
      },
      referrer: 'https://github.com/rsenn?tab=repositories',
      referrerPolicy: 'strict-origin-when-cross-origin',
      body: null,
      method: 'GET',
      mode: 'cors',
      credentials: 'include'
    }).then(r => r.text());

  // Your code here...
})();
