// ==UserScript==

// @name         discogs.es
// @namespace    discogs
// @version      0.4
// @description  discogs.es
// @author       You
// @match        *://*/*
// @exclude      *://127.0.0.1*/*
// @updateURL    http://127.0.0.1:3000/discogs.es
// @grant        none
// @run-at       document-end
// ==/UserScript==

(function () {
  'use strict';

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
    overlay = Element.create(
      'div',
      { position: 'fixed', width: '100vw', height: '100vh', pointerEvents: 'none', zIndex: 1000000000000 },
      document.body
    );
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

  const scaleIFrame = f =>
    Element.setCSS(iframe, { width: Math.floor(w * f) + 'px', height: Math.floor(h * f) + 'px' });

  const makeIFrame = once(src => {
    iframe = Element.create('iframe', { src }, makeOverlay());
    Element.setCSS(
      iframe,
      { width: '100vw', height: '100vh', boxShadow: '2px 2px 10px', userSelect: 'none' },
      makeOverlay()
    );
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

  const tableGetRows = t => Element.findAll('.table_block tr', t);

  const ordersGetIds = () => [...ordersGetList()].map(r => r.id.replace(/order/, '')).filter(id => /[0-9]-/.test(id));
  const ordersGetURLs = () => ordersGetIds().map(id => 'https://www.discogs.com/sell/order/' + id);

  const getId = elem => elem.getAttribute('id').replace(/order/, '');
  const toURL = item => item.url;
  const toText = async resp => await (await resp).text();

  const pageGetRequests = async () => await (await caches.open('fetch')).keys();
  const pageGetResponse = async key => await caches.match(key);

  const frameLoad = src => {
    makeIFrame(src);
    if(iframe.getAttribute('src') != src) iframe.setAttribute('src', src);
    return iframe;
  };
  const cacheLoad = async src => await toText(await pageGetResponse(src));
  const cacheParse = async src => {
    let x = await cacheLoad(src);
    let parser = new DOMParser();
    return (order = parser.parseFromString(x, 'text/html'));
  };

  const hasText = obj => (obj.children ?? []).some(child => typeof child == 'text');
  const getTextChildren = obj =>
    (obj.children ?? []).reduce(
      (acc, child) => acc.concat(typeof child == 'string' ? [child] : [getTextChildren(child)]),
      []
    );
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

  const messageGetItems = () => Element.findAll('li', messageGetThread());
  const messageGetObj = () => Element.toObject(messageGetThread());
  const messageGetList = () => messageGetItems().map(getTextArray);

  const showElement = (e, state) =>
    e.style.setProperty('display', state === true ? 'block' : state === false ? 'none' : state);

  const hideElement = e => e.style.setProperty('display', 'none');
  const styleElement = (e, styles) => Element.css(e, styles);
  //frameLoad(u=(await pageGetRequests())[2].url)

  const orderIdCells = () => navigableList('.shortcut_navigable').map(row => row.children[0]);

  const idFields = () => ordersGetList().map(row => row.firstElementChild.nextElementSibling);

  const idToURL = id => 'https://www.discogs.com/sell/order/' + id;

  Object.assign(globalThis, {
    scaleIFrame,
    ordersGetList,
    tableGetRows,
    ordersGetIds,
    ordersGetURLs,
    toURL,
    toText,
    pageGetRequests,
    pageGetResponse,
    frameLoad,
    frameShowThread,
    cacheLoad,
    cacheParse,
    messageGetThread,
    messageGetItems,
    messageGetObj,
    messageGetList,
    hasText,
    getTextChildren,
    getTextFlat,
    showElement,
    hideElement,
    styleElement,
    makeText,
    transformOverlay
  });
})();
