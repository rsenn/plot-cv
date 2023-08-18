import { h, render } from './lib/dom/preactComponent.js';
import extendArray from './quickjs/qjs-modules/lib/extendArray.js';
extendArray();

const Table = props =>
  h(
    'table',
    { cellspacing: 0, cellpadding: 0 },
    (props.rows ?? props.children).map(row =>
      Array.isArray(row)
        ? h(
            'tr',
            {},
            row.map(cell => (isComponent(cell) ? cell : h('td', {}, [cell])))
          )
        : row
    )
  );

const FileTable = props => {
  /*let data = useAsyncGenerator(() => ConcatDir(ListFiles()));

 console.log('data', data);

return h(Table, {}, Array.isArray(data) ? data : []);*/
};

async function* ReadIterator(st) {
  let d,
    rd = await st.getReader();
  for(;;) {
    d = await rd.read();
    if(d.done) break;
    yield d.value;
  }
  rd.releaseLock();
  return d.value;
}

async function* LineIterator(it) {
  let i,
    j,
    s = '';
  for await(let chunk of await it) {
    s += chunk;
    for(j = 0; (i = s.indexOf('\n', j)) != -1; j += i + 1) yield s.substring(j, i).trimEnd();
    if(j > 0) {
      s = s.slice(j);
      j = 0;
    }
  }
  if(s !== '') yield s.trimEnd();
}

async function* ConcatDir(it) {
  let dir;
  for await(let line of await it) {
    if(line.endsWith(':')) {
      dir = line.slice(0, -1);
      continue;
    }
    yield dir + '/' + line;
  }
}

async function MapDir(it) {
  let a,
    map = new Map();
  for await(let line of await it) {
    if(line.endsWith(':')) {
      map.set(line.slice(0, -1), (a = []));
      continue;
    }
    a.insert(line);
  }
  return map;
}

async function* ListFiles(filter = '@(*.sch|*.brd|*.lbr)') {
  let response = await fetch(`files?root=*/eagle&filter=${filter}`);

  let { writable, readable } = new TextDecoderStream();
  response.body.pipeTo(writable);

  yield* await LineIterator(ReadIterator(readable));

  return response;
}

async function Accumulate(gen) {
  let a = [];
  for await(let item of await gen) {
    a.insert(item);
  }
  return a;
}

window.addEventListener('load', e => {
  console.log('Loaded.');

  let element = document.querySelector('#preact');

  render(h(FileTable, {}, []), element);

  if(document.forms[0]) {
    let filterInput = document.forms[0].elements.filter;

    filterInput.addEventListener('change', e => {
      const { target, currentTarget } = e;
      console.log('Changed:', target.value);
    });
    filterInput.addEventListener('input', e => {
      const { target, currentTarget } = e;
      console.log('Input:', target.value);
    });
  }
});

function Pattern2Regexp(pattern, flags = 'i') {
  pattern = pattern.replace(/\./g, '\\.');
  pattern = pattern.replace(/\*/g, '.*');
  pattern = pattern.replace(/\?/g, '.');
  pattern = '^' + pattern + '$';

  return new RegExp(pattern, flags);
}

function GetTable() {
  return document.querySelector('table');
}

function GetTableRows() {
  return [...GetTable().rows];
}

function GetRow(element) {
  return Object.assign(
    [...element.children].map(e => e.innerText),
    { element }
  );
}

function GetField(row, field) {
  let columns = [...row.children];

  let idx = columns.findIndex(c => c.getAttribute('class') == field);
  if(idx != -1) return columns[idx].innerText;
}

function FilterTable(re) {
  let rows = GetTableRows().slice(1);

  if(typeof re == 'string') re = new RegExp(re);

  if(typeof re != 'function') {
    let re2 = re;
    re = str => re2.test(str);
  }

  console.log('FilterTable', { re });

  for(let row of rows) {
    let file = GetField(row, 'file');

    let fn = re(file) ? Show : Hide;

    fn(row);
  }
}

function Hide(e) {
  //e.classList.add('hidden');
  e.style.display = 'none';
}

function Show(e) {
  e.style.removeProperty('display');
}

function RemoveRow(row) {
  let parent = row.parentElement;
  parent.removeChild(row);
  return row;
}

Object.assign(globalThis, {
  GetTable,
  GetTableRows,
  GetRow,
  GetField,
  FilterTable,
  RemoveRow,
  Pattern2Regexp,
  ListFiles,
  Accumulate,
  ConcatDir,
  MapDir
});