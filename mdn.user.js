// ==UserScript==
// @name         MDN
// @namespace    http://tampermonkey.net/
// @version      2025-05-12
// @description  try to take over the world!
// @author       You
// @match        https://developer.mozilla.org/*
// @icon         https://www.google.com/s2/favicons?sz=64&domain=developer.mozilla.org
// @grant        none
// ==/UserScript==

(function(g) {
    'use strict';

    console.log('MDN userscript loaded');
    window.addEventListener('load', e => console.log('document loaded'));

    const Q = s => document.querySelector(s);
    const QA = s => { const r = document.querySelectorAll(s); return r ? [...r] : []; };
    const QM = (o, fn) => { return Object.entries(o).map(([name, value]) => fn(name, value)); }

    Object.assign(g, { Q, QA, QM });

    // Your code here...
})(globalThis ?? window ?? this);
