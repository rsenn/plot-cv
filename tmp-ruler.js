import { h, useRef, useState } from './lib/dom/preactComponent.js';

export function Ruler() {
  const refRuler = useRef();
  const [value, setValue] = useState(null);

  const pressingDown = () => refRuler.current.pressingDown();
  const pressingUp = () => refRuler.current.pressingUp();
  const stopPressing = () => refRuler.current.stopPressing();
  const onChanged = value => setValue(value);

  return h(RulerDraggable, {}, [
    h(
      'button',
      {
        onMouseDown: pressingDown,
        onMouseUp: stopPressing
      },
      [Down]
    ),
    h(
      'button',
      {
        onMouseDown: pressingUp,
        onMouseUp: stopPressing
      },
      [Up]
    ),
    h('div', {}, [value]),
    h(
      Ruler,
      {
        ref: refRuler,
        defaultValue: 50,
        onChanged,
        longLength: 300,
        shortLength: 60,
        horizontal: true
      },
      []
    )
  ]);
}