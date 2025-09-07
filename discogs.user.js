// ==UserScript==
// @name         discogs.js
// @namespace    discogs
// @version      20250907
// @description  discogs.js
// @author       You
// @match        *://*/*
// @exclude      none
// @icon         https://www.google.com/s2/favicons?sz=64&domain=discogs.com
// @grant        none
// @run-at       document-end
// @downloadURL  https://github.com/rsenn/plot-cv/raw/refs/heads/main/discogs.user.js
// @updateURL    https://github.com/rsenn/plot-cv/raw/refs/heads/main/discogs.user.js
// ==/UserScript==

(function () {
  'use strict';
  //Util.timeit(DiscogsHelpers);
  DiscogsHelpers();
})();

globalThis.orderDocuments = {};

async function DiscogsHelpers(g = globalThis) {
  let iframe, overlay, order, thread;

  const once = (fn, thisArg, memoFn) => {
    let ran = false;
    let ret;
    return function(...args) {
      if(!ran) {
        ran = true;
        ret = fn.call(thisArg || this, ...args);
      } else if(typeof memoFn == 'function') {
        ret = memoFn(ret);
      }
      return ret;
    };
  };

  const makeOverlay = once(src => {
    overlay = Element.create('div', { position: 'fixed', width: '100vw', height: '100vh', pointerEvents: 'none', zIndex: 1000000000000 }, document.body);
    overlay = Element.create('div', {}, overlay);

    Element.setCSS(overlay, {
      display: 'inline-flex',
      flex: '0 0 10%',
      flexFlow: 'column nowrap',
      justifyContent: 'stretch',
      alignItems: 'flex-end',
      position: 'absolute',
      bottom: '0',
      right: '0',
      width: '50vw',
      height: '100vh'
    });
    return overlay;
  });

  const makeText = text => {
    let e = Element.create('pre', {}, []);
    e.appendChild(document.createTextNode(text));
    makeOverlay().appendChild(e);
    Element.setCSS(e, {
      margin: '4px',
      padding: '0.5em',
      border: 'black',
      backgroundColor: 'hsl(50,80%,60%)',
      boxShadow: '2px 2px 2px black',
      color: 'black',
      fontSize: '14pt',
      fontFamily: 'fixed-width'
    });
    return e;
  };

  const scaleIFrame = f => Element.setCSS(iframe, { width: Math.floor(w * f) + 'px', height: Math.floor(h * f) + 'px' });

  const makeIFrame = once(src => {
    iframe = Element.create('iframe', { src }, makeOverlay());
    Element.setCSS(iframe, { width: '100vw', height: '100vh', boxShadow: '2px 2px 10px', userSelect: 'none' }, makeOverlay());
    scaleIFrame(0.25);
    return iframe;
  });
  let w = window.innerWidth,
    h = window.innerHeight;

  const storeValue = name => {
    let t,
      s = localStorage.getItem(name);
    t = trkl(s != null ? JSON.parse(s) : s);
    t.subscribe(value => localStorage.setItem(JSON.stringify(value)));
    return t;
  };

  const transformOverlay = transform => Element.setCSS(overlay, { transform });

  const navigableList = d => Element.findAll('.shortcut_navigable', d);
  const ordersGetList = () => navigableList().filter(e => e.id.startsWith('order'));
  const orderIdCells = () => navigableList('.shortcut_navigable').map(row => row.children[0]);

  const tableGetRows = t => Element.findAll('.table_block tr, table tr', t);

  const ordersGetIds = () => [...ordersGetList()].map(r => r.id.replace(/order/, '')).filter(id => /[0-9]-/.test(id));
  const ordersGetURLs = () => ordersGetIds().map(id => 'https://www.discogs.com/sell/order/' + id);

  const orderId = elem => (typeof elem == 'string' ? elem.replace(/.*\//g, '') : elem.getAttribute('id').replace(/order/, ''));
  const toURL = item => item.url;
  const toText = async resp => await (await resp).text();

  const pageGetRequests = async () => await (await caches.open('fetch')).keys();
  const pageGetResponse = async key => await caches.match(key);
  const pageFetch = globalThis.CachedFetch
    ? await CachedFetch(fetch, caches)
    : async url => {
        let resp;
        try {
          if((resp = await caches.match(key))) return resp;
        } catch(e) {}
        resp = await fetch(url);
        return resp;
      };

  const pageParse = x => {
    let parser = new DOMParser();
    return (order = parser.parseFromString(x, 'text/html'));
  };

  const ordersLoad = async (orders = ordersGetURLs()) => {
    for await(let order of orders) {
      let html = await pageFetch(order).then(toText);
      let doc = await pageParse(html);

      orderDocuments[orderId(order)] = doc;
      let messages = messageGetItems(doc);

      console.log(`order: ${orderId(order)}\nmessages:`, getTextArray(messages));
    }
  };

  const frameLoad = src => {
    makeIFrame(src);
    if(iframe.getAttribute('src') != src) iframe.setAttribute('src', src);
    return iframe;
  };

  const cacheLoad = async src => await toText(await pageGetResponse(src));

  const cacheParse = async src => {
    let x = await cacheLoad(src);
    return (order = pageParse(x));
  };

  const hasText = obj => (obj.children ?? []).some(child => typeof child == 'text');
  const getTextChildren = obj => (obj.children ?? []).reduce((acc, child) => acc.concat(typeof child == 'string' ? [child] : [getTextChildren(child)]), []);
  const getTextArray = obj => ('innerText' in obj ? obj.innerText.split(/\n/g) : getTextChildren(obj).flat());
  const getTextFlat = obj => getTextArray(obj).join('\n');

  const messageGetThread = doc => Element.find('ul.thread', doc ?? iframe?.contentDocument ?? order);

  const frameShowThread = thr => {
    thread ??= messageGetThread(iframe);
    let iframeDoc = thread.ownerDocument;
    thread.parentElement.removeChild(thread);
    iframeDoc.body.innerHTML = '';
    iframeDoc.body.appendChild(thread);
    return thread;
  };

  const messageGetItems = doc => Element.findAll('li', messageGetThread(doc));
  const messageGetObj = doc => Element.toObject(messageGetThread(doc));
  const messageGetList = (doc, reducer = getTextArray) => messageGetItems(doc).map(getTextArray);

  const showElement = (e, state) => e.style.setProperty('display', state === true ? 'block' : state === false ? 'none' : state);

  const hideElement = e => e.style.setProperty('display', 'none');
  const styleElement = (e, styles) => Element.css(e, styles);
  //frameLoad(u=(await pageGetRequests())[2].url)

  const idFields = () => ordersGetList().map(row => row.firstElementChild.nextElementSibling);

  const idToURL = id => 'https://www.discogs.com/sell/order/' + id;

  Object.assign(g, {
    once,
    makeOverlay,
    makeText,
    scaleIFrame,
    makeIFrame,
    storeValue,
    transformOverlay,
    navigableList,
    ordersGetList,
    tableGetRows,
    ordersGetIds,
    ordersGetURLs,
    ordersLoad,
    orderId,
    toURL,
    toText,
    pageGetRequests,
    pageGetResponse,
    pageFetch,
    pageParse,
    frameLoad,
    cacheLoad,
    cacheParse,
    hasText,
    getTextChildren,
    getTextArray,
    getTextFlat,
    messageGetThread,
    frameShowThread,
    messageGetItems,
    messageGetObj,
    messageGetList,
    showElement,
    hideElement,
    styleElement,
    orderIdCells,
    idFields,
    idToURL
  });
}
