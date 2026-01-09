// ==UserScript==
// @name         YouTube Playlist script
// @namespace    http://tampermonkey.net/
// @version      2025-02-17
// @description  extract youtube playlists
// @author       You
// @match        https://www.youtube.com/playlist*
// @icon         https://www.google.com/s2/favicons?sz=64&domain=youtube.com
// @grant        none
// ==/UserScript==

(function () {
  'use strict';

  const globalThis = window;

  function scrollToBottom() {
    const scrollElem = document.documentElement;
    const newScrollPos = scrollElem.scrollHeight - window.innerHeight;
    const scrollOffset = newScrollPos - scrollElem.scrollTop;

    if(scrollOffset > 0) {
      console.log('TamperMonkey script: scrolling by', scrollOffset);
      scrollElem.scrollTop += scrollOffset;
    }

    return scrollOffset;
  }

  const interceptXHR = (function () {
    let h,
      requests = [];
    return once(
      (handler = e => console.log('loadend event fired')) => {
        h = handler;
        const originalXMLHttpRequest = XMLHttpRequest;
        window.XMLHttpRequest = function() {
          const req = new originalXMLHttpRequest();
          const originalOpen = req.open;
          req.open = function(method, url) {
            console.log('XMLHttpRequest', { method, url });
            requests.push([method, url]);
            return originalOpen.call(this, method, url);
          };
          req.addEventListener('loadend', event => h(event, requests));
          return req;
        };
        window.XMLHttpRequest.prototype = originalXMLHttpRequest.prototype;
      },
      null,
      newHandler => (h = newHandler),
    );
  })();

  const interceptFetch = once((handler = (req, options, response) => console.log('fetch', { url: req + '', options, response })) => {
    const oldFetch = fetch;
    window.fetch = async function(req, options) {
      const response = await oldFetch(req, options);
      handler(req, options, response);
      return response;
    };
  });

  Object.assign(globalThis, {
    getVideos: () =>
      Object.assign(
        [...document.querySelectorAll('h3 a')].map(e => [e.getAttribute('href').replace(/.*\bv=([^&]*).*/g, 'https://www.youtube.com/watch?v=$1'), e.getAttribute('title')]),
        {
          toString() {
            return this.map(l => l.join(' ')).join('\n');
          },
        },
      ),
    once,
    debounceAsync,
  });

  window.addEventListener('keydown', e => {
    if(e.key == 'End' || e.key == 'PageDown') {
      console.log('TamperMonkey script: Installing XHR intercept handler');
      interceptXHR((e, requests) => {
        if(e.target.responseURL.indexOf('/browse') != -1) setTimeout(scrollToBottom, 100);
        requests.splice(0, requests.length);
      });
    }
    if(e.key == 'Home' || e.key == 'PageUp') {
      console.log('TamperMonkey script: Removing XHR intercept handler');
      interceptXHR(() => {});
    }
  });

  function once(fn, thisArg, memoFn) {
    let ret,
      ran = false;
    return function(...args) {
      if(!ran) {
        ran = true;
        ret = fn.apply(thisArg || this, args);
      } else if(typeof memoFn == 'function') {
        ret = memoFn(...args);
      }
      return ret;
    };
  }

  function debounceAsync(fn, wait = 0, options = {}) {
    let lastCallAt,
      d,
      timer,
      pendingArgs = [];
    const callFn = (thisObj, args) => fn.call(thisObj, ...args);
    return function debounced(...args) {
      const currentWait = getWait(wait);
      const currentTime = new Date().getTime();
      const isCold = !lastCallAt || currentTime - lastCallAt > currentWait;
      lastCallAt = currentTime;
      if(isCold && options.leading) return options.accumulate ? Promise.resolve(callFn(this, [args])).then(result => result[0]) : Promise.resolve(callFn(this, args));
      if(d) clearTimeout(timer);
      else d = defer();
      pendingArgs.push(args);
      timer = setTimeout(flush.bind(this), currentWait);
      if(options.accumulate) {
        const argsIndex = pendingArgs.length - 1;
        return d.promise.then(results => results[argsIndex]);
      }
      return d.promise;
    };
    function defer() {
      const d = {};
      d.promise = new Promise((resolve, reject) => {
        d.resolve = resolve;
        d.reject = reject;
      });
      return d;
    }
    function getWait(wait) {
      return typeof wait === 'function' ? wait() : wait;
    }
    function flush() {
      const thisDeferred = d;
      clearTimeout(timer);
      Promise.resolve(options.accumulate ? callFn(this, [pendingArgs]) : callFn(this, pendingArgs[pendingArgs.length - 1])).then(thisDeferred.resolve, thisDeferred.reject);
      pendingArgs = [];
      d = null;
    }
  }

  console.log('YouTube scrolling script enabled');
})();
