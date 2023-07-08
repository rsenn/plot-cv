import { h } from './lib/preact.module.js';
import { renderToString } from './lib/renderToString.js';

const TestComponent = props => h('div', { id: 'test' }, ['test']);

const str = renderToString(h(TestComponent, { class: 'text' }));

console.log('str:', str);