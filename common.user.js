// ==UserScript==
// @name         Common Userscript
// @namespace    http://tampermonkey.net/
// @version      V1.0.1
// @description  try to take over the world!
// @author       You
// @match        *
// @icon         https://images.prismic.io/only-alien/ZnyUh5bWFbowe5x5_favicon.ico
// @grant        none
// @downloadURL  https://github.com/rsenn/plot-cv/raw/refs/heads/main/common.user.js
// @updateURL    https://github.com/rsenn/plot-cv/raw/refs/heads/main/common.user.js
// ==/UserScript==

(function () {
  'use strict';

  console.log('Common Userscript LOADED');

  Object.assign(window, {
    define,
    nonenumerable,
    declare,
    concat: (...args) => Array.prototype.concat.call(...args),
    applyGen: (it, fn) => fn(...it),
    *protoChain(o, t = a => a) {
      do yield t(o);
      while((o = Object.getPrototypeOf(o)));
    },
    waitFor: ms => new Promise(r => setTimeout(r, ms)),
    Q: (s, d = document) => d.querySelector(s),
    QA: (s, d = document) => [...(d.querySelectorAll(s) ?? [])],
    isVScrollable: e => e.offsetHeight && e.scrollHeight > e.offsetHeight,
    isHScrollable: e => e.offsetWidth && e.scrollWidth > e.offsetWidth,
    elementInViewport(e) {
      const w = e.offsetWidth,
        h = e.offsetHeight;
      let y = e.offsetTop,
        x = e.offsetLeft;
      while((e = e.offsetParent)) {
        y += e.offsetTop;
        x += e.offsetLeft;
      }
      return (
        y < window.pageYOffset + window.innerHeight &&
        x < window.pageXOffset + window.innerWidth &&
        y + h > window.pageYOffset &&
        x + w > window.pageXOffset
      );
    },
    quote: (s, rpl) =>
      s.replaceAll(
        /([ \t]+|[\r\n])/g,
        rpl ??
          ((match, p1, offset, string, groups) =>
            /^[ \t]+$/.test(match)
              ? ' '
              : ({
                  '\n': '\\n',
                  '\r': '\\r',
                  '\t': '\\t',
                }[match] ?? match)),
      ),
    extractTable(t, tfn = e => quote(e.innerText, () => ' ').trimEnd()) {
      const rows = [...t.querySelectorAll('tr')].map(e =>
        [...e.children].map(tfn),
      );
      const max = Array.from({ length: rows[0].length }, e => 0);

      for(let row of rows) {
        let i = 0;
        for(let col of row) {
          max[i] = Math.max(max[i], col.length);
          i++;
        }
      }

      return Object.assign(rows, {
        max,
        rows: rows.length,
        cols: max.length,
        at(row, col) {
          if(col == this.cols - 1) return this[row][col];
          return this[row][col].padEnd(
            max[col] + (col == this.cols - 1 ? 0 : 1),
            ' ',
          );
        },
        row(row) {
          const r = [];
          for(let i = 0; i < this.max.length; ++i) r[i] = this.at(row, i);
          return Object.assign(r, {
            toString() {
              let s = '';
              for(let j = 0; j < this.length; j++) s += this[j];
              return s;
            },
          });
        },
        toString() {
          let s = '';
          for(let i = 0; i < this.length; i++)
            s += this.row(i).toString() + '\n';
          return s;
        },
        toJSON(...args) {
          return JSON.stringify([...this], ...args);
        },
      });
    },
  });

  function define(obj, ...args) {
    for(const props of args) {
      const desc = Object.getOwnPropertyDescriptors(props);
      const keys = Object.getOwnPropertyNames(props).concat(
        Object.getOwnPropertySymbols(props),
      );
      for(const prop of keys) {
        try {
          delete obj[prop];
        } catch(e) {}
        if(prop == '__proto__') Object.setPrototypeOf(obj, props[prop]);
        else Object.defineProperty(obj, prop, desc[prop]);
      }
    }

    return obj;
  }

  function nonenumerable(props, obj = Object.create(null)) {
    const desc = Object.getOwnPropertyDescriptors(props);
    return Object.defineProperties(
      obj,
      Object.fromEntries(
        Object.entries(desc).map(([k, v]) => (delete v.enumerable, [k, v])),
      ),
    );
  }
  function declare(obj, ...props) {
    return define(obj, ...props.map(o => nonenumerable(o)));
  }
})();
