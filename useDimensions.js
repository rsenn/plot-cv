import { Component, useState, useLayoutEffect, useCallback } from './lib/dom/preactComponent.js';

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

export function useDimensions(arg = {}) {
  if(typeof arg == 'function') arg = arg();

  let liveRef = arg._ref$liveMeasure;
  let _ref$liveMeasure = liveRef === undefined ? true : liveRef;
  const [dimensions, setDimensions] = useState({});
  const [node, setNode] = useState(null);

  let ref = useCallback((node) => {
    setNode(node);
  }, []);
  useLayoutEffect(() => {
    if(node) {
      let measure = function measure() {
        return window.requestAnimationFrame(() => setDimensions(getDimensionObject(node)));
      };
      measure();
      if(_ref$liveMeasure) {
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
