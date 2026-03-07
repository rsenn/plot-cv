// ==UserScript==
// @name         Common Userscript
// @namespace    http://tampermonkey.net/
// @version      V1.0.1
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
    concat: (...args) => Array.prototype.concat.call(...args),
    applyGen: (it, fn) => fn(...it),
    *protoChain(o, t = a => a) {
      do yield t(o);
      while((o = Object.getPrototypeOf(o)));
    },
    waitFor: ms => new Promise(r => setTimeout(r, ms)),
    Q: (s, d = document) => d.querySelector(s),
    QA: (s, d = document) => [...(d.querySelectorAll(s) ?? [])],
    elementInViewport2(el) {
      const width = el.offsetWidth,
        height = el.offsetHeight;
      let top = el.offsetTop,
        left = el.offsetLeft;

      while(el.offsetParent) {
        el = el.offsetParent;
        top += el.offsetTop;
        left += el.offsetLeft;
      }

      return top < window.pageYOffset + window.innerHeight && left < window.pageXOffset + window.innerWidth && top + height > window.pageYOffset && left + width > window.pageXOffset;
    },
  });
})();
