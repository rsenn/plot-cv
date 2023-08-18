import { createElement, isElement } from './dom-helpers.js';
import * as dom from './lib/dom.js';
import { createRef, h, render, default as React } from './lib/dom/preactComponent.js';
import * as geom from './lib/geom.js';
import * as transformation from './lib/geom/transformation.js';
import { useTrkl } from './lib/hooks/useTrkl.js';
import { randStr } from './lib/misc.js';
import trkl from './lib/trkl.js';
import { Card, DragArea, DropArea, List, RUG } from './lib/upload.js';
import { parseDegMinSec, parseGPSLocation } from './string-helpers.js';

const MakeUUID = (rng = Math.random) => [8, 4, 4, 4, 12].map(n => randStr(n, '0123456789abcdef'), rng).join('-');

let uuid, input, drop;

let fileList = (globalThis.fileList = trkl([])),
  progress = 0,
  uploads = (globalThis.uploads = []);

Object.assign(globalThis, { isElement, createElement, React, dom, geom, transformation });
Object.assign(globalThis, {
  /*  ParseCoordinates,
  TransformCoordinates,
  Coordinate,
  Pin,
  Markers,
  OpenlayersMap,*/
  DragArea,
  DropArea,
  Card,
  List,
  RUG,
  FileAction,
  parseGPSLocation,
  parseDegMinSec,
  ListFiles
});

export function prioritySort(arr, predicates = []) {
  const matchPred = item => predicates.findIndex(p => p(item));
  return [...arr].sort((a, b) => matchPred(a) - matchPred(b));
}

function setLabel(text) {
  globalThis.uploadLabel ??= document.querySelector('form label');
  globalThis.uploadLabel.innerHTML = text;
}

function isComponent(obj) {
  if(typeof obj == 'object' && obj !== null) return 'props' in obj;
}

const Table = ({ children, rows, ...props }) => {
  return h(
    'table',
    { cellspacing: 0, cellpadding: 0, ...props },
    (rows ?? children).map(row =>
      Array.isArray(row)
        ? h(
            'tr',
            {},
            row.map(cell => (isComponent(cell) ? cell : h('td', {}, [cell])))
          )
        : row
    )
  );
};

const PropertyList = ({ data, filter, ...props }) => {
  /*let filter= useTrkl(props.filter);*/
  let rows = Array.isArray(data) ? data : data.entries ? [...data.entries()] : Object.entries(data);
  if(filter) rows = rows.filter(filter);
  rows = prioritySort(rows, [
    ([k, v]) => /GPSPos/.test(k),
    ([k, v]) => /GPS/.test(k),
    ([k, v]) => /Orientation/.test(k),
    ([k, v]) => /ImageSize/.test(k),
    ([k, v]) => /FileSize/.test(k),
    ([k, v]) => /Model/.test(k),
    ([k, v]) => /Megapixels/.test(k),
    ([k, v]) => /.*/.test(k)
  ]);
  return h('div', { class: 'property-list' }, [h(Table, { rows })]);
};

const FileItem = ({ file, ref, ...props }) => {
  const { name, lastModified, size, type } = file;
  let upload = useTrkl(file.upload);
  ref ??= trkl();
  /*  ref.subscribe(v => {
    console.log('ref', v);
    let rect = (file.rect = dom.Element.rect(v));
    console.log('rect', rect);
  });
*/
  return h('li', { ref }, [
    h('h2', {}, [name]),
    h(PropertyList, {
      data: upload?.exif ?? {},
      filter: ([k, v]) =>
        /*        /^(Make|Model|GPS|Date|Created|FileSize|Flash|Focal|Distance|ISO|Exposure|Lens|Shutter|White|FNumber|Aperture|Megapixels)/.test(
         */ /^(Orientation|ImageSize|Model|GPS(Position|DestBearing|GPSSpeed|GPSSpeedRef|ImgDir)|DateTimeOriginal|FileSize|Flash$|Distance|ISO|ExposureTime|Lens(Info)|FocalLength$|ShutterSpeed|ApertureValue|Megapixels)/.test(
          k
        )
    }),
    /*,
     */ h('img', upload?.thumbnail ? { src: `file/${upload.thumbnail}` } : {}),
    upload?.exif?.GPSPosition ? h(Table, { class: 'gps' }, [...parseGPSLocation(upload.exif.GPSPosition).map((coord, i) => [i ? 'longitude' : 'latitude', coord])]) : null
  ]);
};

const FileList = ({ files, ref, ...props }) => {
  let list = useTrkl(files);
  return h(
    'ul',
    { ref },
    list.map(file => h(FileItem, { file }))
  );
};

window.addEventListener('load', e => {
  console.log('upload.js loaded!');
  input ??= document.querySelector('input[type=file]');
  let form = document.querySelector('form');
  drop = document.querySelector('#drop-area');
  let preact = document.querySelector('#preact');

  try {
    render(h(FileList, { files: fileList, ref: (globalThis.listElem = createRef()) }, []), preact);
  } catch(e) {
    console.log('Render ERROR:', e.message);
  }
  try {
    drop.addEventListener('click', e => {
      //console.log('drop.click', e);
      globalThis.e = e;

      if(input) input.click(e);
    });
    drop.addEventListener('drop', e => {
      console.log('drop.drop', e);
      e.preventDefault();
      return false;
    });
  } catch(e) {
    console.log('drop ERROR:', e.message);
  }
  try {
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
  } catch(e) {
    console.log('input ERROR:', e.message);
  }
  try {
    form.addEventListener('submit', e => {
      console.log('form.submit', e);
      e.preventDefault();

      UploadFiles();
      return false;
    });
  } catch(e) {
    console.log('submit ERROR:', e.message);
  }
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

async function ListFiles() {
  let resp = await fetch('uploads?limit=0,10&pretty=1').then(r => r.json());

  //console.log('resp', resp);

  return (
    resp
      .map(upload => ({ name: upload.filename, ...upload }))
      .sort((a, b) => a.name.localeCompare(b.name))
      // .filter(r => r.upload?.exif?.GPSPosition)
      .map(file => {
        let pos;
        if(file.upload?.exif?.GPSPosition) {
          pos = parseGPSLocation(file.upload?.exif?.GPSPosition);
        }
        if(Array.isArray(pos) && !(pos[0] == 0 && pos[1] == 0))
          try {
            file.position = new Coordinate(...pos);
          } catch(e) {}
        return file;
      })
    //.filter(file => 'position' in file)
  );
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
  let ws = (globalThis.ws ??= new WebSocket(document.location.href.replace(/\/[^/]*$/, '/uploads').replace(/^http/, 'ws')));
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
          if(input) input.disabled = false;
          if(drop) {
            drop.style.filter = '';
            drop.style.opacity = '';
          }
          console.log('UUID', uuid);
          break;
        case 'progress':
          const { done, total } = command;
          console.log('PROGRESS', command);
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