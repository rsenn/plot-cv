import { h } from './lib/dom/preactComponent.js';
import { render } from './lib/dom/preactComponent.js';
window.addEventListener('load', () => {
  render(h('div'), document.body);
});