// ==UserScript==

// @name         discogs.es
// @namespace    discogs
// @version      0.2
// @description  discogs.es
// @author       You
// @match        *://*/*
// @exclude      *://127.0.0.1*/*
// @updateURL    http://127.0.0.1:3000/discogs.es
// @grant        none
// @run-at       document-end
// ==/UserScript==

(function() {
    'use strict';

   const getRows= () =>  [...document.querySelectorAll('.table_block  tr')];
        const getOrderIds= () => [...getRows()].map(r => r.id.replace(/order/,'')).filter(id => /[0-9]-/.test(id));
     const   getOrderURLs= () => getOrderIds().map(id => 'https://www.discogs.com/sell/order/'+id);

const getCacheKeys= () =>   caches.get('fetch').then(c =>c.keys().then(requests => globalThis.requests=requests));
const matchCache = url => caches.match(url).then(reponse => globalThis.response = response);


    
    
    
    Object.assign(globalThis, {getRows,getOrderIds,getOrderURLs,getCacheKeys,matchCache
    });




    // Your code here...
})();
