// ==UserScript==
// @name         Soundcloud
// @namespace    http://soundcloud.com/
// @version      2025-04-12
// @description  try to take over the world!
// @author       You
// @match        https://soundcloud.com/*
// @icon         https://soundcloud.com/favicon.ico
// @grant        none
// ==/UserScript==

(function () {
  'use strict';

  console.log('SoundCloud grabber loaded');

  window.addEventListener('load', e => {
    //window.url = Object.assign(new URL(window.location + ''), { pathname: '' });

    window.a = GetTracks();
  });
})();

Object.assign(window, {
  GetTracks
});

function GetTracks() {
  const re = /^[^:]*:\/\/[^/]*\//g;
  return Object.assign(
    [...document.querySelectorAll('a.soundTitle__title')].map(e => [e.href.replace(re, ''), e.innerText]),
    {
      toString() {
        const { length } = this;
        let maxlen = 0;
        for(let i = 0; i < length; ++i) {
          const len = this[i][0].length;
          if(len > maxlen) maxlen = len;
        }
        return this.map(([url, title]) => url.padEnd(maxlen) + ' ' + title).join('\n');
      }
    }
  );
}
