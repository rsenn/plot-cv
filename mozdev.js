import { Parser } from 'dom';
import { urlGet } from 'std';

export function FetchClass(url) {
  const doc = new Parser().parseFromString(urlGet(url));

  const summaries = [...doc.querySelectorAll('summary')].map(e => [e.innerText, [...e.nextSibling.querySelectorAll('li')].map(e => [e.innerText, e.querySelector('a').getAttribute('href')])]);

  return summaries;
}
