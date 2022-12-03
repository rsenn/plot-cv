import React, { Fragment, h, render, Component } from './lib/dom/preactComponent.js';

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

window.addEventListener('load', e => {
  console.log('Loaded.');

  let element = document.querySelector('#preact');

  render(h(Table, {}, []), element);

  let filterInput = document.forms[0].elements.filter;

  filterInput.addEventListener('change', e => {
    const { target, currentTarget } = e;
    console.log('Changed:', target.value);
  });
  filterInput.addEventListener('input', e => {
    const { target, currentTarget } = e;
    console.log('Input:', target.value);
  });
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
  Pattern2Regexp
});
1;
