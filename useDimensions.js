import { h, html, render, Component, createContext, useState, useReducer, useEffect, useLayoutEffect, useRef, useImperativeHandle, useMemo, useCallback, useContext, useDebugValue } from './lib/dom/preactComponent.js';

function getDimensionObject(node) {
  if(typeof node == 'object' && node != null && node.base) node = node.base;
  //  console.log('getDimensionObject', node);

  let rect = node.getBoundingClientRect();
  return {
    width: rect.width,
    height: rect.height,
    top: 'x' in rect ? rect.x : rect.top,
    left: 'y' in rect ? rect.y : rect.left,
    x: 'x' in rect ? rect.x : rect.left,
    y: 'y' in rect ? rect.y : rect.top,
    right: rect.right,
    bottom: rect.bottom
  };
}

export function useDimensions() {
  let _ref = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : {},
    _ref$liveMeasure = _ref.liveMeasure,
    liveMeasure = _ref$liveMeasure === undefined ? true : _ref$liveMeasure;

  let _useState = useState({}),
    dimensions = _useState[0],
    setDimensions = _useState[1];

  let _useState2 = useState(null),
    node = _useState2[0],
    setNode = _useState2[1];

  let ref = useCallback((node) => {
    setNode(node);
  }, []);
  useLayoutEffect(() => {
    if(node) {
      let measure = function measure() {
        return window.requestAnimationFrame(() => setDimensions(getDimensionObject(node)));
      };
      measure();
      if(liveMeasure) {
        window.addEventListener('resize', measure);
        window.addEventListener('scroll', measure);
        return function () {
          window.removeEventListener('resize', measure);
          window.removeEventListener('scroll', measure);
        };
      }
    }
  }, [node]);
  return [ref, dimensions, node];
}
export default useDimensions;
//# sourceMappingURL=index.js.map
