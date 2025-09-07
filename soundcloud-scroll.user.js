//==UserScript==
// @name         Scroll to bottom
// @namespace    https://soundcloud.com/
// @version      20250907
// @description  Scrolls a soundcloud page to the bottom
// @author       Roman Senn
// @match        https://soundcloud.com/*
// @icon         https://soundcloud.com/favicon.ico
// @grant        none
// @updateURL    https://github.com/rsenn/plot-cv/raw/refs/heads/main/soundcloud-scroll.user.js
//==/UserScript==

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
  let h;
  const requests = [];
  return once(
    (handler = e => console.log('loadend event fired')) => {
      h = handler;
      const originalXMLHttpRequest = XMLHttpRequest;
      window.XMLHttpRequest = function() {
        const req = new originalXMLHttpRequest();
        const originalOpen = req.open;
        req.open = function(method, url) {
          //console.log('XMLHttpRequest.open()', {method,url});
          requests.push([method, url]);
          return originalOpen.call(this, method, url);
        };
        req.addEventListener('loadend', event => {
          h(event, requests);
          // requests.splice(0, requests.length);
        });
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

Object.assign(window, { interceptXHR, interceptFetch, once, debounceAsync });

window.addEventListener('keydown', e => {
  if(e.key == 'End') {
    console.log('TamperMonkey script: Installing XHR intercept handler');
    interceptXHR((e, requests) => {
      if(e.target.responseURL.indexOf('/tracks/') != -1) {
        setTimeout(scrollToBottom, 100);
      }
      requests.splice(0, requests.length);
    });
  }
  if(e.key == 'Home') {
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

  function defer() {
    const d = {};
    d.promise = new Promise((resolve, reject) => {
      d.resolve = resolve;
      d.reject = reject;
    });
    return d;
  }

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
