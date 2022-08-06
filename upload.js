import React, { h, html, render, Fragment, Component, useState, useLayoutEffect, useRef } from './lib/dom/preactComponent.js';
import { randStr } from './lib/misc.js';
import { isElement, createElement } from './dom-helpers.js';
import { DragArea, DropArea, Card, List, RUG } from './lib/upload.js';

const MakeUUID = (rng = Math.random) => [8, 4, 4, 4, 12].map(n => util.randStr(n, '0123456789abcdef'), rng).join('-');

let uuid;

Object.assign(globalThis, { isElement, createElement, React });
Object.assign(globalThis, { DragArea, DropArea, Card, List, RUG });

window.addEventListener('load', e => {
  console.log('upload.js loaded!');

  let form = document.querySelector('form');

  form.addEventListener('submit', e => {
    console.log('form.submit', e);
    e.preventDefault();

    UploadFiles();
    return false;
  });

  globalThis.ws = CreateWS();
});

function UploadFiles() {
  let input = document.querySelector('#file');

  // for(let file of input.files)
  UploadFile(input.files);
}

// upload JPEG files
function UploadFile(files) {
  //file??=   document.querySelector('#file');

  const formData = new FormData();
  formData.append('uuid', uuid);

  for(let file of files) formData.append('file', file);

  fetch('upload'/*+'.html'*/, { method: 'POST', body: formData }).then(response => {
    console.log('response', response);
  });
}

function CreateWS() {
  let ws = new WebSocket(document.location.href.replace(/\/[^/]*$/, '/uploads').replace(/^[^:]*/, 'ws'));
  ws.onmessage = e => {
    const { data } = e;
    let command = JSON.parse(data);
    console.log('onmessage', command);
    switch (command.type) {
      case 'uuid':
        uuid = command.data;
        break;
    }
  };

  return ws;
}

function XHRUpload(formData) {
  let form = document.querySelector('form');
  var xhr = new XMLHttpRequest();

  if(xhr.upload) {
    xhr.upload.addEventListener(
      'progress',
      function(e) {
        var pc = parseInt(100 - (e.loaded / e.total) * 100);

        console.log('pc', pc);
      },
      false
    );

    xhr.open('POST', form.action, true);
    xhr.setRequestHeader('Content-Type', 'multipart/form-data');
    xhr.send(formData);
  }
}

// file selection
function FileSelectHandler(e) {
  // cancel event and hover styling
  FileDragHover(e);

  // fetch FileList object
  var files = e.target.files || e.dataTransfer.files;

  // process all File objects
  for(var i = 0, f; (f = files[i]); i++) {
    ParseFile(f);
    UploadFile(f);
  }
}
