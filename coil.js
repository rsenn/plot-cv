import {
  h,
  Fragment,
  html,
  render,
  Component,
  useState,
  useEffect,
  useRef,
  useCallback,
  Portal,
  ReactComponent
} from './lib/dom/preactComponent.js';
import { Element } from './lib/dom.js';

window.addEventListener('load', () => {
  render(h('div'), document.body);
});
