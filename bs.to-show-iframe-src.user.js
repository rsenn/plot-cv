// ==UserScript==
// @name         BS.to show iframe video link
// @namespace    http://tampermonkey.net/
// @version      0.1
// @description  try to take over the world!
// @author       You
// @match        https://bs.to/serie/*
// @icon         https://bs.to/favicon.ico
// @grant        none
// ==/UserScript==

(function () {
  'use strict';

  window.addEventListener('load', e => {
    console.log('Userscript loaded');

    let url, tid;

    tid = setInterval(() => {
      let iframe;

      if((iframe = document.querySelector('iframe[height="100%"]'))) {
        let newurl = iframe.src;

        if(newurl != url) {
          ShowLink(newurl, iframe.parentElement);

          url = newurl;

          clearInterval(tid);
        }
      } else {
        console.log('periodic check');
      }
    }, 500);
  });

  function ShowLink(url, container) {
    let e = document.createElement('a');

    e.innerHTML = url;

    container.appendChild(e);

    e.href = url;
    e.target = '_blank';
    e.style.padding = '1em';
    e.style.marginTop = '0';
  }

  Object.assign(globalThis, { ShowLink });
})();
