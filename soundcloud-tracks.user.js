// ==UserScript==
// @name         SoundCloud grabber
// @namespace    http://soundcloud.com/
// @version      2025-04-12
// @description  Get tracks from soundcloud
// @author       Roman Senn <roman.l.senn@gmail.com>
// @match        https://soundcloud.com/*
// @match        https://www.youtube.com/*
// @icon         https://soundcloud.com/favicon.ico
// @grant        none
// @downloadURL  https://github.com/rsenn/plot-cv/raw/refs/heads/main/soundcloud-tracks.user.js
// @updateURL    https://github.com/rsenn/plot-cv/raw/refs/heads/main/soundcloud-tracks.user.js
// ==/UserScript==

(function () {
  'use strict';

  console.log('SoundCloud grabber loaded');

  window.addEventListener('load', e => {
    window.a = GetTracks();
  });
})();

Object.assign(window, {
  GetTracks,
});

function GetTracks() {
  const re = /^[^:]*:\/\/[^/]*\//g;
  const yt = /youtube/.test(window.location + '');

  return Object.assign(
    [...document.querySelectorAll('a.soundTitle__title, a.ytd-video-renderer')]
      .map(yt ? e => [e.href, e.innerText, e] : e => [e.href, e.innerText])
      .filter(yt ? ([url, title, e]) => /watch/.test(url) && !/<div\s/.test(e.innerHTML) : ([url, title]) => url != '')
      .map(
        yt
          ? ([url, ...rest]) => ['watch?v=' + url.replace(/.*v=/g, '').replace(/[\&&?].*/g, ''), ...rest]
          : ([url, ...rest]) => [url.replace(re, ''), ...rest],
      )
      .filter(([url, title]) => url != '' && title != ''),
    {
      toString(pad = false) {
        let maxlen = 0;
        if(pad) {
          const { length } = this;
          for(let i = 0; i < length; ++i) {
            const len = this[i][0].length;
            if(len > maxlen) maxlen = len;
          }
        }
        return this.map(([url, title]) => url.padEnd(maxlen) + ' ' + title).join('\n');
      },
    },
  );
}
