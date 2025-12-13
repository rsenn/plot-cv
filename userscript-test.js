import { Element } from './lib/dom/element.js';
(function () {
  'use strict';

  Object.assign(globalThis.Element, Element);
})();