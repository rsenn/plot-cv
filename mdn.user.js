// ==UserScript==
// @name         MDN
// @namespace    http://tampermonkey.net/
// @version      20250907
// @description  try to take over the world!
// @author       You
// @match        https://developer.mozilla.org/*
// @icon         https://www.google.com/s2/favicons?sz=64&domain=developer.mozilla.org
// @grant        none
// @downloadURL  https://github.com/rsenn/plot-cv/raw/refs/heads/main/mdn.user.js
// @updateURL    https://github.com/rsenn/plot-cv/raw/refs/heads/main/mdn.user.js
// ==/UserScript==

(function (g) {
  'use strict';

  const isRegex = arg => typeof arg != 'function' && typeof arg == 'object' && arg != null && arg instanceof RegExp;

  Array.prototype.match = function(f) {
    if(isRegex(f)) {
      const re = f;
      f = (item, i, arr) => re.test(item);
    }
    return this.filter(f);
  };

  function* Map(gen, fn = a => a) {
    let i = 0;
    for(const item of gen) yield fn(item, i++, gen);
  }

  function toArray(gen) {
    return [...gen];
  }

  console.log('MDN userscript loaded');

  window.addEventListener('load', e => console.log('document loaded'));

  /*const CFN = fns => (...args) => fns.every(fn => fn(...args));*/

  const ANDFN =
    (a, b) =>
    (...args) =>
      a(...args) && b(...args);

  const M = (s, fn = () => true) => {
    for(const m of [...(s.matchAll(/([^.]+|\..*)/g) ?? [])].map(m => m[0])) {
      if(m.startsWith('.')) {
        const cl = new RegExp(`\\b${m.slice(1)}\\b`);
        fn = ANDFN(fn, e => cl.test(e.className));
      } else fn = ANDFN(fn, e => e.tagName == m);
    }
    return fn;
  };

  const FD = async url => await (await fetch(/:\/\//.test(url) ? url : 'https://developer.mozilla.org/en-US/docs/' + url)).text();

  const GD = async s => (typeof s == 'string' ? (s = PD(await FD(s))) : s) ?? window.document;

  const PD = s => new DOMParser().parseFromString(s, 'text/html');

  const Q = (s, d = document) => d.querySelector(s);

  const QA = (s, d = document) => [...(d.querySelectorAll(s) ?? [])];

  const QM = (o, fn) => Object.entries(o).map(([name, value]) => fn(name, value));

  const W = prop =>
    function* (e, incl) {
      if(incl) yield e;
      while((e = e[prop])) yield e;
    };

  const WU = W('parentElement');
  const WR = W('previousElementSibling');

  const ED = e => toArray(WU(e, true)).length;

  const Until = (it, cond) => {
    if(typeof cond == 'string') cond = M(cond);
    else if(Array.isArray(cond)) cond = new RegExp(`^(${cond.join('|')})$`, 'i');

    if(isRegex(cond)) {
      const re = cond;
      cond = e => re.test(e.tagName);
    }
    if(!cond) return [...it];
    for(const e of it) if(cond(e)) return e;
  };

  for(const proto of [Object.getPrototypeOf(WU()), Object.getPrototypeOf(WR())]) {
    Object.assign(proto, {
      map(fn) {
        return Map(this, fn);
      },
      toArray() {
        return toArray(this);
      },
      until(cond) {
        return Until(this, cond);
      },
    });
  }

  const WebAPI = async () => QA('section.content-section:nth-child(3) > div:nth-child(3) > ul > li > a:nth-child(1)', await GD('Web/API')).map(e => e.href);

  const GlobalObjects = async () => QA('ul li a code', await GD('Web/JavaScript/Reference/Global_Objects')).map(e => e.parentElement.href);

  Object.assign(g, { Q, QA, QM, FD, PD, GD, W, WU, WR, ED, WC, Map, toArray, Until, WebAPI, GlobalObjects });

  // Your code here...
})(window ?? this);
