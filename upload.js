import React, { h, html, render, Fragment, Component, useState, useLayoutEffect, useRef } from './lib/dom/preactComponent.js';
import { randStr, assert, lazyProperties, define, isObject, memoize, unique } from './lib/misc.js';
import { isElement, createElement } from './dom-helpers.js';
import { DragArea, DropArea, Card, List, RUG } from './lib/upload.js';
import * as dom from './lib/dom.js';
import * as geom from './lib/geom.js';
import * as transformation from './lib/geom/transformation.js';
import { useTrkl } from './lib/hooks/useTrkl.js';
import trkl from './lib/trkl.js';

const MakeUUID = (rng = Math.random) => [8, 4, 4, 4, 12].map(n => randStr(n, '0123456789abcdef'), rng).join('-');

let uuid, input, drop;

let fileList = (globalThis.fileList = trkl([])),
  progress = 0,
  uploads = (globalThis.uploads = []);

Object.assign(globalThis, { isElement, createElement, React, dom, geom, transformation });
Object.assign(globalThis, { DragArea, DropArea, Card, List, RUG, FileAction });

export function prioritySort(arr, predicates=[]){
  const matchPred = item => predicates.findIndex(p => p(item));
 return [...arr].sort((a,b) => matchPred(a) - matchPred(b));
}

function setLabel(text) {
  globalThis.uploadLabel ??= document.querySelector('form label');
  globalThis.uploadLabel.innerHTML = text;
}

const Table = ({ rows }) => {
  return h(
    'table',
    { cellspacing: 0, cellpadding: 0 },
    rows.map(row =>
      h(
        'tr', 
        {},
        row.map(cell => h('td', {}, [cell]))
      )
    )
  );
};

const PropertyList = ({ data, filter, ...props }) => {
  /*let filter= useTrkl(props.filter);*/
  let rows = Array.isArray(data) ? data : data.entries ? [...data.entries()] : Object.entries(data);
  if(filter) rows = rows.filter(filter);
rows=prioritySort(rows, [
  ([k,v]) => /GPSPos/.test(k),
  ([k,v]) => /GPS/.test(k),
  ([k,v]) => /Orientation/.test(k),
  ([k,v]) => /ImageSize/.test(k),
  ([k,v]) => /FileSize/.test(k),
  ([k,v]) => /Model/.test(k),
  ([k,v]) => /Megapixels/.test(k),
  ([k,v]) => /.*/.test(k)
  ]);
  return h('div', { class: 'property-list' }, [h(Table, { rows })]);
};

const FileItem = ({ file, ...props }) => {
  const { name, lastModified, size, type } = file;
  let upload = useTrkl(file.upload);
  //console.log('FileItem', file);
  return h('li', {}, [
    h('h2', {}, [name]),
    h(PropertyList, {
      data: upload?.exif ?? {},
      filter: ([k, v]) =>
        /*        /^(Make|Model|GPS|Date|Created|FileSize|Flash|Focal|Distance|ISO|Exposure|Lens|Shutter|White|FNumber|Aperture|Megapixels)/.test(
         */ /^(Orientation|ImageSize|Model|GPS(Position|DestBearing|GPSSpeed|GPSSpeedRef|ImgDir)|DateTimeOriginal|FileSize|Flash$|Distance|ISO|ExposureTime|Lens(Info)|FocalLength$|ShutterSpeed|ApertureValue|Megapixels)/.test(
          k
        )
    }) /*,
    h(Table, { rows: Object.entries(upload,exit ?? {}) })
    */,
    h('img', upload?.thumbnail ? { src: upload.thumbnail } : {})
  ]);
};

const FileList = ({ files }) => {
  return h(
    'ul',
    {},
    useTrkl(files).map(file => h(FileItem, { file }))
  );
};

window.addEventListener('load', e => {
  console.log('upload.js loaded!');
  input ??= document.querySelector('input[type=file]');
  let form = document.querySelector('form');
  drop = document.querySelector('#drop-area');
  let preact = document.querySelector('#preact');

  render(h(FileList, { files: fileList }, []), preact);

  drop.addEventListener('click', e => {
    console.log('drop.click', e);

    input.click(e);
  });

  input.addEventListener('change', e => {
    const { target } = e;
    let { files } = target;
    fileList((files = [...files]));
    e.preventDefault();
    setLabel(`${fileList.length} Fotos ausgew&auml;hlt`);

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

  if(files.length > 0) return UploadFile(files);
}

// upload JPEG files
function UploadFile(files) {
  //file??=   document.querySelector('#file');

  const formData = new FormData();

  uuid ??= MakeUUID();
  formData.append('uuid', uuid);

  for(let file of files) formData.append('file', file);

  fileList([...files].map(f => ((f.upload = trkl(null)), f)));

  return fetch('upload' /*+'.html' */, { method: 'POST', body: formData }).catch(err => {
    console.log('POST done!');
  }) /*.then(response => {
    console.log('response', response);
    return response.text();
  })*/;
}

function UploadDone(upload) {
  let list = fileList();
  let found = list.findIndex(({ name }) => name == upload.filename);

  uploads.push(upload);

  if(found != -1) {
    list[found].upload(upload);

    // fileList([...list]);
  }
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
          input.disabled = false;
          drop.style.filter = '';
          drop.style.opacity = '';

          console.log('UUID', uuid);
          break;
        case 'progress':
          const { done, total } = command;
          progress = done;
          break;
        case 'upload':
          const { address, thumbnail, uploaded, filename, exif, storage } = command;
          let upload = { address, thumbnail, uploaded, filename, exif, storage };
          console.log('UPLOAD', upload);
          UploadDone(upload);
          break;
        default:
          console.log('UNHANDLED', command);

          break;
      }
    }
  };
  ws.onopen = e => {
    //  console.log('onopen', e);
  };
  ws.onclose = e => {
    globalThis.ws = null;
    //   console.log('onclose', e);
    restart();
  };
  ws.onerror = e => {
    globalThis.ws = null;
    //  console.log('onerror', e);
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
