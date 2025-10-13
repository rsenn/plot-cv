import { httpGet } from './net-helpers.js';
import { parseHTML, isElement, createElement, upTo, parents, iterateTree, domConsoleLog } from './dom-helpers.js';
import { readFileSync } from 'fs';

async function url2doc(url) {
  const s = await httpGet(url);
  return parseHTML(s);
}

function cleanString(s) {
  return s.replaceAll(/\u2013/g, '-');
}

function parseYear(s) {
  const result = s.split(/\s+-\s+/g).map(s => (isNaN(+s) ? s : +s));
  /* const result = [...s.matchAll(/\d+/g)].map(([m]) => +m);*/

  while(result.length < 2) result.push(undefined);
  return result;
}

function processDocument(doc, url) {
  const e = doc.querySelector('.serie');
  const infos = doc.querySelector('.infos');

  const a = [...infos.children].map(getTitleAndContent);

  //console.log('processDocument(1)',a);

  function getTitleAndContent(e, i, arr) {
    const [title, ...rest] = [...e.children];
    let str = cleanString(rest.map(e => e.innerText).join('\n'));

    if(/jahr/gi.test(title.innerText)) str = parseYear(str);
    else if(/(Genre)/i.test(title.innerText)) str = str.split(/\s+/g);
    else if(/(Hauptdarsteller|Regisseure|Autoren|Produzenten)/i.test(title.innerText)) str = str.split(/(?:\s*,\s+|\s+und\s+)/g);

    return [
      { Titel: 'title', Produzenten: 'producers', Hauptdarsteller: 'actors', Regisseure: 'directors', Genres: 'genres', Produktionsjahre: 'years', Autoren: 'autors' }[title.innerText] ??
        title.innerText,
      str,
    ];
  }

  a.unshift(['title', cleanString(doc.querySelector('title').innerText).replace(/(\s+\(1\)|) - Burning Series.*/g, '')]);
  a.unshift(['url', url]);

  return Object.fromEntries(a);
}

async function processURL(url, result) {
  verbose(`Fetching '${url}'...`);

  const doc = await url2doc(url);

  verbose(`Fetched '${url}':`, doc);

  const obj = processDocument(doc, url);

  verbose('obj', obj);

  result.push(obj);
}

async function main(file) {
  file ||= 'sci-fi.txt';

  console.log(`Processing '${file}'...`);

  const result = [],
    urls = readFileSync(file, 'utf-8').trim().split('\n');

  Object.assign(globalThis, { urls, result });

  for(const url of urls) {
    try {
      await processURL(url, result);

      //console.log('Results:', result.length);
    } catch(error) {
      console.log('ERROR', error);
      throw error;
    }
  }
}

main(scriptArgs[1]);

function verbose(...args) {
  if(process.env.DEBUG) console.log(console.config({ compact: true }), ...args);
}

function interactive() {
  os.kill(os.getpid(), os.SIGUSR1);
}

interactive();
