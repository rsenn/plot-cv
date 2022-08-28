import React, { h, html, render, Fragment, Component, useState, useLayoutEffect, useRef } from './lib/dom/preactComponent.js';
import { randStr, assert, lazyProperties, define, isObject, memoize, unique } from './lib/misc.js';

import { isElement, createElement } from './dom-helpers.js';
import { DragArea, DropArea, Card, List, RUG } from './lib/upload.js';
import * as dom from './lib/dom.js';
import * as geom from './lib/geom.js';
import * as transformation from './lib/geom/transformation.js';

const MakeUUID = (rng = Math.random) => [8, 4, 4, 4, 12].map(n => randStr(n, '0123456789abcdef'), rng).join('-');

let uuid, input;

let fileList = [],
  progress = 0,
  uploads = (globalThis.uploads = []);

Object.assign(globalThis, { isElement, createElement, React, dom, geom, transformation });
Object.assign(globalThis, { DragArea, DropArea, Card, List, RUG, FileAction });

function setLabel(text) {
  globalThis.uploadLabel ??= document.querySelector('form label');
  globalThis.uploadLabel.innerText = text;
}
window.addEventListener('load', e => {
  console.log('upload.js loaded!');
  input ??= document.querySelector('input[type=file]');
  let form = document.querySelector('form');
  let drop = document.querySelector('#drop-area');

  drop.addEventListener('click', e => {
    console.log('drop.click', e);

    input.click(e);
  });

  input.addEventListener('change', e => {
    const { target } = e;
    let { files } = target;
    fileList = globalThis.fileList = files = [...files];
    e.preventDefault();
    setLabel(`${fileList.length} files selected`);

    UploadFiles(files)
      .catch(err => {
        console.log('UploadFiles ERROR:', err);
      })
      .then(resp => {
        console.log('UploadFiles response:', resp);
      });
    return false;
  });

  form.addEventListener('submit', e => {
    console.log('form.submit', e);
    e.preventDefault();

    UploadFiles();
    return false;
  });

  input ??= document.querySelector('input[type=file]');
  /*  input.addEventListener('change', e => {
    const { srcElement, target } = e;
    console.log('input.change', e);
    console.log('target.files', target.files);
    console.log('srcElement.value', srcElement.value);
  });*/

  CreateWS();
});

function FileAction(cmd, file, contents) {
  return fetch('file', {
    method: 'POST',
    body: JSON.stringify({ action: cmd, file, contents }),
    headers: { ['Content-Type']: 'application/json' }
  }).then(r => r.text());
}

function UploadFiles(files) {
  let input = document.querySelector('#file');
  console.log('UploadFiles', files);
  files ??= input.files;

  // for(let file of input.files)
  return UploadFile(files);
}

// upload JPEG files
function UploadFile(files) {
  //file??=   document.querySelector('#file');

  const formData = new FormData();

  uuid ??= MakeUUID();
  formData.append('uuid', uuid);

  for(let file of files) formData.append('file', file);

  return fetch('upload' /*+'.html' */, { method: 'POST', body: formData }).then(response => {
    console.log('response', response);
    return response.text();
  });
}

function CreateWS() {
  let ws = (globalThis.ws ??= new WebSocket(
    document.location.href.replace(/\/[^/]*$/, '/uploads').replace(/^http/, 'ws')
  ));
  console.log('CreateWS', ws);
  let tid;
  const restart = (delay = 10) => {
    tid ??= setTimeout(() => {
      CreateWS();

      tid = undefined;
    }, delay);
  };
  ws.onmessage = e => {
    const { data } = e;

    if(typeof data == 'string') {
      let command = JSON.parse(data);
      switch (command.type) {
        case 'uuid':
          uuid = command.data;
          break;
        case 'progress':
          const { done, total } = command;
          progress = done;
          break;
        case 'upload':
          const { address, thumbnail, uploaded, filename, exif, storage } = command;
          let upload = { address, thumbnail, uploaded, filename, exif, storage };
          console.log('upload', upload);

          uploads.push(upload);
          break;
        default:
          console.log('onmessage', command);

          break;
      }
    }
  };
  ws.onopen = e => {
    console.log('onopen', e);
  };
  ws.onclose = e => {
    globalThis.ws = null;
    console.log('onclose', e);
    restart();
  };
  ws.onerror = e => {
    globalThis.ws = null;
    console.log('onerror', e);
    restart();
  };
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
