// ==UserScript==
// @name         YouTube Playlist script
// @namespace    http://tampermonkey.net/
// @version      2025-02-17
// @description  extract youtube playlists
// @author       You
// @match        https://www.youtube.com/playlist
// @icon         https://www.google.com/s2/favicons?sz=64&domain=youtube.com
// @grant        none
// ==/UserScript==

(function() {
    'use strict';

    window.getVideos = () => Object.assign([...document.querySelectorAll("h3 a")].map(e => [ e.getAttribute('href').replace(/.*\bv=([^&]*).*/g, 'https://www.youtube.com/watch?v=$1'), e.getAttribute('title')]), { toString() { return this.map(l => l.join(' ')).join('\n'); } });
    // Your code here...
})();
