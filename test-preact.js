import { Component } from './lib/preact.js';

import { html } from './lib/htm.js';
import { renderToString } from './lib/renderToString.js';

const TestComponent = props => html` <div id="test" ...${props}>test text</div> `;

const str = renderToString(html` <${TestComponent} class="text" /> `);

//console.log('str:', str);
