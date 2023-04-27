import { forwardRef, h, Fragment, React, default as preactComponent, ReactComponent, Portal, toChildArray, isComponent } from './lib/dom/preactComponent.js';
import { renderToString } from './lib/renderToString.js';

const TestComponent = props => h('div', { id: 'test' }, ['test']);

const str = renderToString(h(TestComponent, { class: 'text' }));

console.log('str:', str);
