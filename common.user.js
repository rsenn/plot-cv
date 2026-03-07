// ==UserScript==
// @name         Common Userscript
// @namespace    http://tampermonkey.net/
// @version      2026-03-07
// @description  try to take over the world!
// @author       You
// @match        *://*/*
// @icon         https://images.prismic.io/only-alien/ZnyUh5bWFbowe5x5_favicon.ico
// @grant        none
// @downloadURL  https://github.com/rsenn/plot-cv/raw/refs/heads/main/common.user.js
// @updateURL    https://github.com/rsenn/plot-cv/raw/refs/heads/main/common.user.js
// ==/UserScript==

(function () {
  'use strict';

   console.log('Common Userscript LOADED');

  Object.assign(window, {
    waitFor: ms => new Promise((r) => setTimeout(r, ms)),
    Q: (s, d = document) => d.querySelector(s),
    QA: (s, d = document) => [...(d.querySelectorAll(s) ?? [])],
  });
})();
