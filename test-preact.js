import { Component, Fragment, cloneElement, createContext, createElement, createRef, h, hydrate, isValidElement, options, render, toChildArray } from './lib/preact.js';

import { html } from './lib/htm.js';
import { renderToStaticMarkup, renderToString, shallowRender } from './lib/renderToString.js';

const TestComponent = props =>
  html`
    <div id="test" ...${props}>test text</div>
  `;

const str = renderToString(
  html`
    <${TestComponent} class="text" />
  `
);

//console.log('str:', str);
