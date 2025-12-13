import { classNames } from './lib/classNames.js';
import { Element } from './lib/dom/element.js';
import { useDimensions, useElement, useEvent, usePanZoom } from './lib/hooks.js';
import { useTrkl } from './lib/hooks/useTrkl.js';
import { isObject, roundTo, tryCatch } from './lib/misc.js';
import * as path from './lib/path.js';
import { useValue } from './lib/repeater/react-hooks.js';
import { trkl } from './lib/trkl.js';
import RulerDraggable from './ruler-draggable.js';
import { wordWrap } from './string-helpers.js';
import { h, Fragment, html, useState, useEffect, useRef, useCallback, Portal, ReactComponent, toChildArray } from './lib/dom/preactComponent.js';

export const Ruler = ({ handleChange, style = {}, class: className }) => {
  const refRuler = useRef();
  const [value, setValue] = useState(null);

  /*console.log('Ruler refRuler: ', refRuler);
  //console.log('Ruler value: ', value);*/

  const handlers = trkl(null);
  let commands;

  handlers.subscribe(value => {
    if(!commands && isObject(value)) commands = value;
    // ========================================================================== //
    //     /*console.log('trkl handlers value =', value);P

    //console.log('commands =', commands);*/
  });

  const pressingDown = () => commands.pressingDown();
  const pressingUp = () => commands.pressingUp();
  const stopPressing = () => commands.stopPressing();
  let onChanged = value => setValue(value);

  if(typeof handleChange == 'function') {
    const prevHandler = onChanged;
    onChanged = value => {
      prevHandler(value);
      handleChange(value);
    };
  }

  return h('div', { style, class: className }, [
    h(
      'button',
      {
        onMouseDown: pressingDown,
        onMouseUp: stopPressing,
      },
      ['Down'],
    ),
    h(
      'button',
      {
        onMouseDown: pressingUp,
        onMouseUp: stopPressing,
      },
      ['Up'],
    ),
    h('div', {}, [roundTo(value, 0.001, 3)]),
    h(
      RulerDraggable,
      {
        ref: refRuler,
        handlers,
        defaultValue: 50,
        onChanged,
        longLength: 600,
        shortLength: 60,
        horizontal: false,
      },
      [],
    ),
  ]);
};

export const ClickHandler = callback => e => {
  if(e.type) {
    const press = e.type.endsWith('down');
    callback(e, press);
  }
};

export const ToggleHandler = (callback, getState, setState) => e => {
  if(e.type) {
    const press = e.type.endsWith('down');

    if(press) {
      let state = !getState();
      setState(state);
      callback(e, state);
    }
  }
};

export const MouseEvents = h => ({
  onMouseDown: h,

  /*  onBlur: h,*/
  onMouseOut: h,
  onMouseUp: h,
});

export const Overlay = ({ className = 'overlay', title, tooltip, active = true, enable = trkl(true), visible = trkl(true), toggle, state, onPush, text, children, ...props }) => {
  const [pushed, setPushed] = typeof state == 'function' ? [useTrkl(state), state] : useState(false);
  const enabled = useTrkl(enable);
  const events = enabled
    ? MouseEvents(
        (toggle ? ToggleHandler : ClickHandler)(
          (e, state) => {
            const prev = pushed;
            if(e.buttons && e.buttons != 1) return;
            if(!e.type.endsWith('down') && !e.type.endsWith('up')) return;
            let ret;
            // console.log("MouseEvent", {type: e.type, state});
            if(typeof onPush == 'function') ret = onPush(e, state);
            if(ret === true || ret === false) state = ret;
            setPushed(state);
          },
          () => pushed,
          setPushed,
        ),
      )
    : {};
  if(typeof title == 'string' && title.length > 0) props.title = title;
  if(typeof tooltip == 'string' && tooltip.length > 0) props['data-tooltip'] = tooltip;
  return h(
    'div',
    {
      className: classNames(className, pushed && 'pushed', active ? 'active' : 'inactive', !visible && 'hidden'),
      ...props,
      ...events,
    },
    children,
  );
};

export const Container = ({ className = 'panel', tag = 'div', children, ...props }) => {
  // useCallback(() => console.debug('re-render panel'));
  return h(tag, { className, ...props }, children);
};

export const Button = allProps => {
  let {
    className,
    caption,
    image,
    fn,
    state,
    enable = trkl(true),
    visible = trkl(true),
    onPush = state => (state && typeof fn == 'function' ? fn(state) : undefined),
    style = {},
    ...props
  } = allProps;
  const enabled = useTrkl(enable);
  const show = useTrkl(visible);

  if(!props.children) props.children = [];
  if(typeof image == 'string') image = h('img', { src: image });
  else if(typeof image == 'function') image = image(allProps);

  props.children.unshift(image);

  //console.log('Button', caption ?? image, { enabled, show });

  return h(Overlay, {
    className: classNames('button', className, !enabled && 'disabled'),
    text: caption,
    toggle: true,
    state,
    onPush,
    style,
    enable,
    visible,
    ...props,
  });
};

export const Toggle = ({ className, images, fn, state, style = {}, ...props }) => {
  const pushed = useTrkl(state);
  const image = images[pushed | 0];
  state.subscribe(value => {
    //console.log('Toggled:', value);
  });
  return h(Button, {
    className,
    fn,
    state,
    image,
    style,
    ...props,
  });
};

export const ButtonGroup = ({ className, index = trkl(), children, onChange = () => {}, ...props }) => {
  const active = useTrkl(index);
  const buttons = [
    ...children.map((component, i) =>
      typeof component == 'function'
        ? component({
            ...props,
            className: classNames('button', className),
            state: active === i /*,
            onPush*/,
          })
        : component,
    ),
  ];

  function onPush(event) {
    let { currentTarget, target } = event;

    const tag = target?.tagName?.toLowerCase();
    let siblingCounter = 0;
    if(tag == 'img') {
      do {
        target = target.previousElementSibling;
        siblingCounter++;
      } while(target);
      console.log('ButtonGroup.event:', siblingCounter);
      let j = siblingCounter - 1;
      if(j != active) {
        index(j);
        onChange(event);
      }
    }
  }

  console.log('ButtonGroup.buttons:', buttons);

  return h(
    'div',
    {
      className: 'button-group',
      onMouseDown: onPush,
    },
    buttons,
  );
};

/*html`
  <${Overlay} className="button" text=${caption} onPush=${state => (state ? fn(state) : undefined)} ${...props} />
`;
*/
export const FloatingPanel = ({ children, className, onSize, onHide, style = {}, ...props }) => {
  const [ref, { x, y, width, height }] = useDimensions();

  //console.log('FloatingPanel.dimensions:', { x, y, width, height });

  const [size, setSize] = useState(onSize ? onSize() : {});
  const [hidden, setHidden] = useState(onHide ? onHide() : false);

  let noUpdate = false;

  const hasOnSize = typeof onSize == 'function' && typeof onSize.subscribe == 'function';

  function updateSize(value) {
    if(!noUpdate) setSize(value);
  }
  useEffect(() => {
    hasOnSize && onSize.subscribe(updateSize);
    return () => hasOnSize && onSize.unsubscribe(updateSize);
  }, []);

  if(hasOnSize) {
    const tmpSize = onSize();
    noUpdate = true;
    // if(tmpSize.width != width || tmpSize.height != height)
    if(isObject(tmpSize) && (tmpSize.width === undefined || tmpSize.height === undefined)) if (width !== undefined && height !== undefined) onSize({ width, height });
    noUpdate = false;
  }
  const hasOnHide = typeof onHide == 'function' && typeof onHide.subscribe == 'function';

  function updateHide(value) {
    setHidden(value);
  }
  useEffect(() => {
    hasOnHide && onHide.subscribe(updateHide);
    return () => hasOnHide && onHide.unsubscribe(updateHide);
  }, []);

  if(size) {
    if(!isNaN(+size.width)) style.width = `${size.width}px`;
    if(!isNaN(+size.height)) style.height = `${size.height}px`;
    //    console.log('FloatingPanel size:', size);
  }

  if(hidden) style.display = 'none';

  return h(
    Overlay,
    {
      ref,
      className: classNames('floating', hidden && 'hidden', className),
      ...props,
      style,
    },
    children,
  );
};

export const Label = ({ className, text, title, tooltip, children, ...props }) => {
  if(typeof title == 'string' && title.length > 0) props.title = title;
  if(typeof tooltip == 'string' && tooltip.length > 0) props['data-tooltip'] = tooltip;
  return h('div', { className: classNames('caption', className), ...props }, (text ? [text] : []).concat(children));
};

export const DynamicLabel = ({ caption, title, children, ...props }) => {
  const [text, setText] = useState(caption());

  caption.subscribe(value => setText(value));

  return h(Label, { ...props, text }, []);
};

export const Item = ({ className = 'item', title, tooltip, label, icon, children, ...props }) => {
  if(typeof title == 'string' && title.length > 0) props.title = title;
  if(typeof tooltip == 'string' && tooltip.length > 0) props['data-tooltip'] = tooltip;
  //console.log('Item', props);
  return h(Overlay, { className, ...props }, h(Label, { text: icon }, label));
};

export const Icon = ({ className = 'icon', caption, image, ...props }) => h(Container, { className, ...props }, h('img', { src: image }));

export const Progress = ({ className, percent, ...props }) => {
  let p;

  if(!{ string: true, number: true }[typeof percent]) p = useTrkl(percent);
  else p = percent;

  return h(
    Overlay,
    {
      className: classNames('progress', 'center', className),
      text: p + '%',
      style: {
        position: 'relative',
        height: '1em',
        border: '1px solid black',
        padding: 0,
        textAlign: 'center',
        zIndex: '99',
        overflow: 'hidden',
      },
    },
    h(Fragment, {}, [
      h('div', {
        className: classNames('progress-bar', 'fill'),
        style: {
          width: p + '%',
          position: 'absolute',
          background: 'hsla(210, 100%,50%,0.5)',
          zIndex: '98',
        },
        innerHTML: '&nbsp;',
      }),
      h(
        'div',
        {
          className: classNames('progress-bar', 'number'),
          style: {
            position: 'absolute',
            width: '100%',

            zIndex: '98',
            textShadow: 'white 1px 1px 2px',
          },
        },
        [p + '%'],
      ),
    ]),
  );
};

/*export const BrowseIcon = (props) => html`<svg xmlns="http://www.w3.org/2000/svg" height="40" width="40" viewBox="131 -131 731.429 731.429">
  <path d="M240.714 39.414v390.6h432.4l79.6-244.1h-435.6l-48.8 145.7 33.1-170.2h378v-48.8h-243.4l-47.2-73.2z" fill="#fff"/>
</svg>
`;*/

export const SchematicIcon = props => html`
  <svg viewBox="0 0 40 40" xmlns="http://www.w3.org/2000/svg">
    <defs />
    <path
      d="M32.02 3.594c-21.347 24.27-10.673 12.135 0 0zm3.439 26.494a2.727 2.727 0 01-.7 1.675l-7.165 7.505a2.375 2.375 0 01-1.706.732H12.766c-3.32 0-5.978-2.8-5.978-6.245V6.25c0-1.735.675-3.302 1.754-4.433C9.629.702 11.108 0 12.766 0h16.708A5.86 5.86 0 0133.7 1.817 6.422 6.422 0 0135.46 6.25z"
      fill="#fff"
    />
    <path
      d="M33.079 6.25a3.82 3.82 0 00-1.059-2.656 3.481 3.481 0 00-2.547-1.091H12.765c-.968 0-1.868.375-2.538 1.09A3.785 3.785 0 009.175 6.25v27.505c0 2.06 1.612 3.742 3.59 3.742h13.143V29.82h7.17zm2.38 23.838a2.728 2.728 0 01-.7 1.675l-7.165 7.505a2.375 2.375 0 01-1.706.733H12.766c-3.319 0-5.978-2.802-5.978-6.245V6.25c0-1.735.675-3.303
    1.754-4.434C9.63.702 11.108 0 12.766 0h16.708A5.86 5.86 0 0133.7 1.817a6.421 6.421 0 011.76 4.434z"
      fill="#444443"
    />
    <path d="M33.074 24.821H4.535v-17.5h28.539z" fill="#b4b3b3" />
    <path
      d="M13.695 18.812c.287.371.501.701.501 1.19 0 1.075-.817 1.621-1.806 1.621H9.234c-.546 0-1.019-.1-1.45-.46-.388-.315-.674-.831-.674-1.334 0-.458.258-.86.76-.86.416 0 .675.343.675.73 0 .432.229.49.631.49h3.113c.115 0 .472.043.472-.175 0-.112-.2-.328-.257-.415l-4.921-6.325c-.3-.387-.473-.675-.473-1.178 0-.574.273-1.062.76-1.375.401-.26.731-.26 1.19-.26h3.043c.545 0 1.003.115 1.42.475.375.328.675.86.675 1.362 0 .459-.23.831-.747.831-.43 0-.69-.272-.69-.673 0-.488-.242-.56-.687-.56H9.033c-.158 0-.488-.043-.488.2 0 .13.158.302.214.388l4.937 6.327zm4.933 2.811c-1.077 0-1.693-.43-2.153-1.364l-1.319-2.667c-.245-.488-.387-.946-.387-1.506 0-.632.112-1.02.4-1.593l1.32-2.655c.488-.989 1.106-1.376 2.21-1.376h2.437c.316 0 .72.171.72.718 0 .516-.389.717-.719.717h-2.395c-.617 0-.717.057-.99.602l-1.25 2.497c-.185.386-.3.645-.3 1.09 0 .402.13.645.3.99l1.22 2.48c.288.588.433.633 1.075.633h2.34c.315 0 .72.172.72.717 0 .516-.39.717-.72.717zm6.17-4.864v4.147c0 .46-.2.717-.717.717s-.717-.257-.717-.717v-9.728c0-.459.2-.717.717-.717.517 0 .717.258.717.717v4.146h4.218v-4.146c0-.459.2-.717.717-.717s.717.258.717.717v9.728c0 .46-.201.717-.717.717s-.717-.257-.717-.717v-4.147z"
      fill="#bf272f"
    />
  </svg>
`;

export const BoardIcon = props => html`
  <svg viewBox="0 0 40 40" xmlns="http://www.w3.org/2000/svg">
    <defs />
    <path
      d="M32.02 3.594c-21.347 24.27-10.673 12.135 0 0zm3.439 26.494a2.727 2.727 0 01-.7 1.675l-7.165 7.505a2.375 2.375 0 01-1.706.732H12.766c-3.32 0-5.978-2.8-5.978-6.245V6.25c0-1.735.675-3.302 1.754-4.433C9.629.702 11.108 0 12.766 0h16.708A5.86 5.86 0 0133.7 1.817 6.422 6.422 0 0135.46 6.25z"
      fill="#fff"
    />
    <path
      d="M33.079 6.25a3.82 3.82 0 00-1.059-2.656 3.481 3.481 0 00-2.548-1.091H12.765c-.968 0-1.868.374-2.538 1.09A3.785 3.785 0 009.175 6.25v27.505c0 2.06 1.612 3.742 3.591 3.742H25.91V29.82h7.17V6.25zm2.38 23.837a2.728 2.728 0 01-.7 1.675l-7.165 7.505a2.375 2.375 0 01-1.707.733H12.765c-3.319 0-5.978-2.801-5.978-6.245V6.25a6.41 6.41 0 011.754-4.434C9.63.701 11.107 0 12.765 0h16.707A5.86 5.86 0 0133.7 1.816a6.421 6.421 0 011.759 4.434v23.837z"
      fill="#444443"
    />
    <path d="M33.079 24.795H4.54v-17.5H33.08v17.5z" fill="#b4b3b3" />
    <path
      d="M8.738 20.206h2.608c1.044 0 1.653-.462 1.653-1.565 0-1.217-.333-1.9-1.653-1.9H8.737v3.466zm0-4.914h2.608c1.044 0 1.653-.465 1.653-1.566 0-1.217-.333-1.899-1.653-1.899H8.737v3.465zm-1.45-4.915h4.058c2.132 0 3.103 1.305 3.103 3.35 0 .84-.188 1.608-.956 2.29.824.782.956 1.55.956 2.624 0 1.9-1.261 3.014-3.103 3.014H7.287V10.377zm10.15 3.712h3.217c.406 0 1.045-.029 1.045-.609v-1c0-.58-.595-.653-1.03-.653h-3.232v2.262zm0 6.842c0 .463-.204.724-.725.724-.523 0-.726-.261-.726-.724V10.377h4.872c1.146 0 2.291.813 2.291 2.103v1c0 1.364-1.13 2.059-2.291 2.059h-1.334l3.261 4.669c.159.216.363.49.363.767 0 .405-.334.68-.739.68-.29 0-.479-.173-.637-.405l-4.002-5.711h-.333v5.392zm9.63-.725h.276c.61 0 .754-.072 1.015-.607l1.275-2.582c.162-.333.275-.58.275-.943 0-.493-.159-.768-.361-1.188l-1.204-2.45c-.305-.624-.434-.609-1.1-.609h-.175v8.38zM25.62 11.83h-.537c-.48 0-.884-.175-.884-.725 0-.523.391-.725.84-.725h2.421c1.131 0 1.695.462 2.188 1.45l1.277 2.55c.288.595.435 1.015.435 1.682 0 .579-.159 1.073-.42 1.594l-1.321 2.637c-.505 1.032-1.16 1.363-2.29 1.363h-2.404c-.379 0-.725-.201-.725-.725 0-.52.346-.723.725-.723h.696v-8.38z"
      fill="#42ab3d"
    />
  </svg>
`;

export const LibraryIcon = props =>
  html` <svg viewBox="0 0 40 40" xmlns="http://www.w3.org/2000/svg">
    <g>
      <path
        d="M3.397 6.488v26.938c0 3.085 2.13 5.675 5.013 6.407V.105a6.561 6.561 0 00-5.012 6.38M30.122-.096H11.645v40.105h18.477a6.6 6.6 0 006.59-6.581V6.488a6.582 6.582 0 00-6.59-6.583"
        fill="#444443"
      />
      <path d="M30.313 19.791h-12.5v-12.5h12.5v12.5z" fill="#dedd00" />
    </g>
  </svg>`;

export const GCodeIcon = props =>
  html`<svg viewBox="0 0 40 40" xmlns="http://www.w3.org/2000/svg">
    <path
      d="M32.02 3.594c-21.347 24.27-10.673 12.135 0 0zm3.439 26.494a2.727 2.727 0 01-.7 1.675l-7.165 7.505a2.375 2.375 0 01-1.706.732H12.766c-3.32 0-5.978-2.8-5.978-6.245V6.25c0-1.735.675-3.302 1.754-4.433C9.629.702 11.108 0 12.766 0h16.708A5.86 5.86 0 0133.7 1.817a6.422 6.422 0 011.76 4.433z"
      fill="#fff"
    />
    <path
      d="M33.079 6.25c-22.053 22.5-11.026 11.25 0 0zm2.38 23.837a2.728 2.728 0 01-.7 1.675l-7.165 7.505a2.375 2.375 0 01-1.707.733H12.765c-3.319 0-5.978-2.801-5.978-6.245V6.25a6.41 6.41 0 011.754-4.434C9.63.701 11.107 0 12.765 0h16.707A5.86 5.86 0 0133.7 1.816a6.421 6.421 0 011.759 4.434z"
      fill="#fff"
    />
    <path
      d="M33.079 6.25a3.82 3.82 0 00-1.059-2.656 3.481 3.481 0 00-2.548-1.091H12.765c-.968 0-1.868.374-2.538 1.09A3.785 3.785 0 009.175 6.25v27.505c0 2.06 1.612 3.742 3.591 3.742H25.91V29.82h7.17V6.25zm2.38 23.837a2.728 2.728 0 01-.7 1.675l-7.165 7.505a2.375 2.375 0 01-1.707.733H12.765c-3.319 0-5.978-2.801-5.978-6.245V6.25a6.41 6.41 0 011.754-4.434C9.63.701 11.107 0 12.765 0h16.707A5.86 5.86 0 0133.7 1.816a6.421 6.421 0 011.759 4.434v23.837z"
      fill="#444443"
    />
    <path
      d="M6.757 22.75c-.583-.583-.624-1.04-.624-6.87s.041-6.288.624-6.87c.606-.606 1.041-.625 14.366-.625s13.76.018 14.366.624c.583.583.624 1.041.624 6.87 0 5.83-.041 6.288-.624 6.871-.606.606-1.041.625-14.366.625s-13.76-.019-14.366-.625z"
      fill="#fff"
    />
    <path
      d="M6.757 22.75c-.583-.583-.624-1.04-.624-6.87s.041-6.288.624-6.87c.606-.606 1.041-.625 14.366-.625s13.76.018 14.366.624c.583.583.624 1.041.624 6.87 0 5.83-.041 6.288-.624 6.871-.606.606-1.041.625-14.366.625s-13.76-.019-14.366-.625zm5.622-5.777c0-1.041-.122-1.406-.469-1.406-.312 0-.468.313-.468.937 0 1.022-.463 1.197-1.385.522-.724-.529-.736-2.067-.021-2.59.314-.23.928-.323 1.444-.22.764.153.899.081.899-.483 0-.579-.181-.664-1.406-.664-.989 0-1.59.185-2.03.625-.44.44-.624 1.04-.624 2.03 0 1.891.763 2.654 2.654 2.654h1.406zm4.996.741c0-.564-.134-.636-.898-.483-1.172.234-1.99-.385-1.99-1.507 0-1.123.818-1.742 1.99-1.507.764.152.898.08.898-.484 0-.571-.182-.664-1.307-.664-1.827 0-2.752.91-2.752 2.709 0 1.833.783 2.6 2.654 2.6 1.225 0 1.405-.085 1.405-.664zm5.211-.015c.974-.915.997-2.893.044-3.907-.915-.974-2.893-.996-3.907-.044-.974.915-.997 2.893-.044 3.907.915.975 2.893.997 3.907.044zm-2.603-.59c-1.185-.439-1.104-2.54.11-2.857.844-.22 1.856.233 2.033.91.372 1.423-.765 2.456-2.143 1.946zm7.912.59c.975-.915.997-2.893.044-3.907-.528-.562-1.014-.723-2.186-.723h-1.507V18.378h1.463c1.073 0 1.657-.181 2.186-.679zm-2.712-1.975v-1.786l.859.212c1.136.281 1.405.582 1.405 1.574 0 .991-.269 1.292-1.405 1.573l-.86.212zm7.807 2.186c0-.339-.347-.469-1.249-.469-1.04 0-1.25-.104-1.25-.624 0-.506.21-.625 1.094-.625.763 0 1.093-.141 1.093-.468 0-.328-.33-.469-1.093-.469-.885 0-1.093-.119-1.093-.624 0-.506.208-.625 1.093-.625.763 0 1.093-.141 1.093-.468 0-.354-.382-.469-1.561-.469h-1.562V18.378h1.718c1.318 0 1.717-.109 1.717-.468z"
      fill="#59f"
    />
  </svg> `;

export const FileIcons = {
  sch: SchematicIcon,
  brd: BoardIcon,
  lbr: LibraryIcon,
  ngc: GCodeIcon,
};

export const Conditional = ({ initialState, component = Fragment, className, children, signal, ...props }) => {
  const [show, setShown] = useState(initialState !== undefined ? initialState : signal());

  if(signal) signal.subscribe(value => setShown(value));

  return show ? h(component, { className, ...props }, children) : h(Fragment, {});
};

export const ShowHide = ({ initialState, component, className, children, signal, ...props }) => {
  const [hidden, setHidden] = useState(initialState !== undefined ? initialState : !signal());

  if(signal) signal.subscribe(value => setHidden(!value));

  return h(component, { className: classNames(className, hidden && 'hidden'), ...props }, children);
};

export const EditBox = ({ value = '', type = 'div', className, hidden = false, current, focus, ...props }) => {
  if(typeof current == 'function') props.ref = input => current(input);

  const outerProps = { className: classNames(className, hidden && 'hidden') };

  if(type == 'form') outerProps.onSubmit = event => event.preventDefault();

  return h(
    type,
    outerProps,
    h('input', {
      type: 'text',
      value,
      className: classNames(className, hidden && 'hidden'),
      ...props,
    }),
  );
};

export const File = ({ label, name, description, i, key, className = 'file', onPush, signal, data, doc, ...props }) => {
  //console.log('File', props);
  const [loaded, setLoaded] = useState(NaN);
  if(signal) signal.subscribe(data => setLoaded(data.percent));
  onPush =
    onPush ||
    (async state => {
      await load(name);
    });
  let id = key || i;
  let iconProps = { style: { minWidth: '20px' /*, width: '40px', height: '40px'*/ } };
  let icon = /brd$/i.test(id + className)
    ? h(BoardIcon, { ...iconProps, className: 'icon' })
    : /sch$/i.test(id + className)
      ? h(SchematicIcon, { ...iconProps, className: 'icon' })
      : /lbr$/i.test(id + className)
        ? h(LibraryIcon, { ...iconProps, className: 'icon' })
        : undefined;
  let fileExt = path.extname(name).replace(/^\./, '');
  if(!icon && FileIcons[fileExt]) icon = h(FileIcons[fileExt], { ...iconProps, className: 'icon' });
  if(id) {
    id = isNaN(+id) ? i : id;
    id = 'file-' + id;
  }
  label = label.replace(/([^\s])-([^\s])/g, '$1 $2');
  //if(icon) label = label.replace(/\.[^.]*$/, '');
  label = h(Label, { text: wordWrap(label, 50, '\n') });
  if(description) {
    let s = multiParagraphWordWrap(stripXML(decodeHTMLEntities(description)), 60, '\n');
    let d = s.split(/\n/g).slice(0, 1);
    label = h('div', {}, [
      label,
      h(
        'div',
        { className: 'description' },
        d.map(line => h('pre', { className: 'description' }, [line])),
      ),
    ]);
  }
  icon = h('div', { ...iconProps, className: 'icon' }, icon);
  return h(
    Item,
    { className, id, 'data-filename': name, label, onPush, icon, ...props },
    h(Progress, {
      className: !isNaN(loaded) ? 'visible' : 'hidden',
      percent: loaded,
    }),
  );
};

export const Chooser = ({ className = 'list', itemClass = 'item', tooltip = () => '', itemComponent = Overlay, itemFilter, items, sortCompare, onChange = () => {}, onPush = () => {}, ...props }) => {
  const [active, setActive] = useState(-1);
  const [filter, setFilter] = useState('*');
  const [list, setList] = /*useState(items);*/ trkl.is(items) ? useState(items()) : [items];

  if(trkl.is(items)) items.subscribe(setList);

  //if(typeof items == 'function') console.error('items():', items());

  const pushHandler = i => (e, state) => {
    const prev = active;
    state == true && setActive(i);
    if(i != prev && e.type.endsWith('down')) onChange(e, list[i], i);
    onPush(e, i, state);
  };

  if(itemFilter) {
    setFilter(itemFilter());
    itemFilter.subscribe(value => setFilter(value));
  }
  const list2re = list => list.map(part => tryCatch(() => new RegExp(part.trim().replace(/\./g, '\\.').replace(/\*/g, '.*'), 'i'))).filter(r => r !== null);
  const bar = html``;
  const preFilter = filter
    .replace(/\|/g, ' | ')
    .replace(/\+/, ' +')
    .split(/\s+/g)
    .filter(p => !/:\/\//.test(p));
  const plus = list2re(preFilter.filter(p => p.startsWith('+')).map(p => p.replace(/\+/g, '')));
  const rest = preFilter.filter(p => !p.startsWith('+')).join(' ');
  //console.log('filter', { plus, rest });
  const reList = rest
    .split(/\|/g)
    .map(p => p.trim())
    .filter(p => p != '')
    .map(p => list2re(p.split(/\s\s*/g)));
  //Sconsole.debug('regex:', ...reList);
  const pred = name => !reList.every(c => !c.every(re => re.test(name))) && plus.every(re => re.test(name));
  const other = list.filter(({ name }) => !pred(name)).map(i => i.name);
  const displayList = [...list].sort(sortCompare);
  //console.log('displayList', { displayList, sortCompare });
  const children = displayList
    .filter(({ name }) => pred(name))
    .map((value, key) => {
      let { name, description /*= ''*/, i, title, number, data, ...item } = value;
      data = data || number;
      i = i === undefined ? number : i;
      //console.log(`Chooser item #${i}:`, { keys: Object.keys(item), data });

      return h(itemComponent, {
        key: i,
        i,
        className: typeof itemClass == 'function' ? itemClass(value) : classNames(itemClass || className + '-item', (name + '').replace(/.*\./, '')),
        active: i == active,
        onPush: pushHandler(i),
        label: name.replace(new RegExp('.*/'), ''),
        tooltip: tooltip({
          title,
          name,
          description,
          i,
          number,
          data,
          ...item,
        }),
        name,
        description,
        ...item,
      });
    });

  return h(Container, { className: classNames('panel', 'no-select', className), ...props }, children);
};

const ToolTipFn = ({ name, data, ...item }) => {
  let tooltip = `name\t${name.replace(new RegExp('.*/', 'g'), '')}`;

  for(let field of ['type', 'size', 'sha', 'path']) if(item[field] !== undefined) tooltip += `\n${field}\t${item[field]}`;
  if(typeof data == 'object' && data != null) {
    //console.log('data',data);
    data = Object.entries(data)
      .filter(([name, value]) => !isNaN(value) && value != null)
      .map(([name, value]) => `${name}: ${value}`)
      .join('\n');
  } else data = abbreviate(data);
  if(data) tooltip += `\ndata\t${data}`;
  return tooltip;
};

export const FileList = ({
  files,
  onChange,
  onActive,
  filter,
  showSearch,
  focusSearch,
  currentInput,
  changeInput,
  tag = 'div',
  listTag = 'div',
  sortKey,
  sortOrder,
  makeSortCompare,
  children,
  ...props
}) => {
  const [active, setActive] = useState(true);
  const [items, setItems] = useState(files());
  const key = useTrkl(sortKey);
  const order = useTrkl(sortOrder);

  const sortFunction = makeSortCompare(key);
  const sortCompare = (a, b) => sortFunction(a, b) * (order | 1);

  files.subscribe(value => setItems(value));

  onActive.subscribe(value => setActive(value));
  const className = classNames('sidebar', active ? 'active' : 'inactive');

  return h(tag, { className }, [
    ...children,
    h(Conditional, {
      component: EditBox,
      type: 'form',
      className: 'search',
      autofocus: true,
      name: 'query',
      id: 'search',
      placeholder: 'Search',
      signal: showSearch,
      focus: focusSearch,
      current: currentInput,
      onChange: changeInput,
      onInput: changeInput,
      value: filter(),
    }),
    h(Chooser, {
      tag: listTag,
      className: 'list',
      itemComponent: File,
      itemClass: item => classNames('file', 'hcenter', item.name.replace(/.*\./g, '')),
      itemFilter: filter,
      sortCompare,
      items,
      tooltip: ToolTipFn,
      onChange: (...args) => onChange(...args),
      ...props,
    }),
  ]);
};

export const Panel = ({ className, children, ...props }) => h(Container, { className: classNames('panel', className), ...props }, children);

export const WrapInAspectBox = (enable, { width = '100%', aspect = 1, className }, children) =>
  enable
    ? h(
        SizedAspectRatioBox,
        {
          className,
          width,
          aspect,
          style: { overflow: 'visible' },
        },
        children,
      )
    : h(
        'div',
        {
          className,
          style: { width },
        },
        children,
      );

export const AspectRatioBox = (
  {
    aspect = 1.0,
    children,
    insideClassName,
    outsideClassName,
    outsideProps = {},
    style,
    ...props
  } /* console.debug('AspectRatioBox ', { props, aspect, children, insideClassName, outsideClassName, style });*/,
) =>
  h(Fragment, {}, [
    h(
      'div',
      {
        className: classNames('aspect-ratio-box', outsideClassName),
        style: {
          height: 0,
          paddingBottom: (1.0 / aspect) * 100 + '%',
          ...style,
        },
      },
      [
        h(
          'div',
          {
            className: classNames('aspect-ratio-box-inside', insideClassName),
          },
          children,
        ),
      ],
    ),
  ]);

export const SizedAspectRatioBox = ({
  id,
  width,
  height,
  style,
  className,
  children,
  outsideClassName,
  insideClassName,
  insideProps,
  outsideProps = {},
  sizeClassName,
  sizeProps = {},
  onClick,
  ...props
}) =>
  h(
    'div',
    {
      className: classNames('aspect-ratio-box-size', className && className + '-size', sizeClassName),
      style: { position: 'relative', width, height, ...style },
      onClick,
      id,
    },
    [
      h(
        AspectRatioBox,
        {
          outsideClassName: classNames('aspect-ratio-box-outside', className && className + '-outside', outsideClassName),
          outsideProps,
          insideClassName: insideClassName || className,
          onClick,
          ...props,
        },
        children,
      ),
    ],
  );

export const TransformedElement = ({ type = 'div', id, aspect, listener, style = { position: 'relative' }, className, children = [], ...props }) => {
  /*  const [transform, setTransform] = useState(new TransformationList());
  //console.debug('TransformedElement:', { aspect });
  //
  if(listener && listener.subscribe)
    listener.subscribe(value => {
     //console.log('TransformedElement setValue', value);
      if(value !== undefined) setTransform(value + '');
    });*/
  let transform = useTrkl(listener);
  return h(
    type,
    {
      id,
      className: classNames('transformed-element', className && className + '-size'),
      style: {
        position: 'relative',
        ...style,
        transform: transform.toString('px'),
      },
      aspect,
      ...props,
    },
    children,
  );
};

export const Slider = ({ min = 0, max = 100, value: initialValue = 0, step = 1, name = 'slider', orient = 'horizontal', label, onChange = value => {}, style = {}, length, ...props }) => {
  const [value, setValue] = useState(initialValue);
  const onInput = e => {
    const { target } = e;

    setValue(target.value);
    onChange(target.value);
  };
  label = label || name;
  let dim = length ? { [orient == 'horizontal' ? 'width' : 'height']: length } : {};

  return h(
    'div',
    {
      style: {
        display: 'inline-flex',
        flexBasis: '100%',
        flexFlow: orient == 'horizontal' ? 'row' : 'column',
        justifyContent: 'stretch',
        alignItems: 'center',
        fontSize: '0.8em',
        ...style,
      },
    },
    [
      //h('label', { for: name }, label),
      label,
      h('input', {
        name,
        type: 'range',
        min,
        max,
        orient,
        style: {
          WebkitAppearance: `slider-${orient}`,
          display: 'inline',
          width: 40,
          flex: '1 1 auto',
          ...dim,
        },
        ...props,
        value,
        onInput,
      }),
      h('input', {
        name,
        type: 'number',
        min,
        max,

        orient,
        style: {
          width: (+max + '').length * 10 + 'px',
          MozAppearance: `textfield`,
          WebkitAppearance: 'none',
          margin: 0,
          display: 'inline',
          textAlign: 'center',
          flex: '0 1 auto',
        },
        ...props,
        value,
        onInput,
      }),
    ],
  );
};

export const Canvas = ({ onInit, ...props }) => {
  const [drawing, setDrawing] = useState(false);
  const [width, setWidth] = useState(props.width);
  const [height, setHeight] = useState(props.height);
  const canvasRef = useRef();
  const ctx = useRef();

  useEffect(() => {
    //console.debug('canvasRef.current', canvasRef.current);
    ctx.current = canvasRef.current.getContext('2d');
    //console.debug('ctx.current', ctx.current);
    const { offsetLeft: x, offsetTop: y } = canvasRef.current;

    if(typeof onInit == 'function') onInit(ctx.current, canvasRef.current, { width, height, x, y });
  }, []);

  /*  const [windowWidth, windowHeight] = useWindowSize(() => {
    setWidth(window.innerWidth)
    setHeight(window.innerHeight)
  })*/

  function handleMouseMove(e) {
    //actual coordinates
    const coords = [e.clientX - canvasRef.current.offsetLeft, e.clientY - canvasRef.current.offsetTop];
    if(drawing) {
      ctx.current.lineTo(...coords);
      ctx.current.stroke();
    }
    if(props.handleMouseMove) {
      props.handleMouseMove(...coords);
    }
  }

  function startDrawing(e) {
    ctx.current.lineJoin = 'round';
    ctx.current.lineCap = 'round';
    ctx.current.lineWidth = 1;
    ctx.current.strokeStyle = props.color;
    ctx.current.beginPath();
    //actual coordinates
    ctx.current.moveTo(e.clientX - canvasRef.current.offsetLeft, e.clientY - canvasRef.current.offsetTop);
    setDrawing(true);
  }

  function stopDrawing() {
    ctx.current.closePath();
    setDrawing(false);
  }

  return h(
    'canvas',
    {
      ref: canvasRef,
      width,
      height,
      onMouseDown: startDrawing,
      onMouseUp: stopDrawing,
      onMouseOut: stopDrawing,
      onMouseMove: handleMouseMove,
    },
    [],
  );
};

export const ColorWheel = ({ radius = 50, ...props }) => {
  let data;
  let left, top;
  return h(Canvas, {
    width: radius * 2,
    height: radius * 2,
    onInit: (ctx, canvas, size) => {
      left = size.x;
      top = size.y;

      drawCircle();
      ctx.lineWidth = 0.75;
      ctx.beginPath();
      ctx.arc(50, 50, 50 - ctx.lineWidth / 2, 0, Math.ceil(2 * Math.PI));
      ctx.stroke();

      function drawCircle() {
        let radius = 50;
        let image = ctx.createImageData(2 * radius, 2 * radius);
        data = image.data;

        for(let x = -radius; x < radius; x++) {
          for(let y = -radius; y < radius; y++) {
            let [r, phi] = xy2polar(x, y);

            if(r > radius) {
              //skip all (x,y) coordinates that are outside of the circle
              continue;
            }

            let deg = rad2deg(phi);

            //Figure out the starting index of this pixel in the image data array.
            let rowLength = 2 * radius;
            let adjustedX = x + radius; //convert x from [-50, 50] to [0, 100] (the coordinates of the image data array) let adjustedY = y + radius; //convert y from [-50, 50] to [0, 100] (the coordinates of the image data array) let pixelWidth = 4; //each pixel requires 4 slots in the data array let index = (adjustedX + adjustedY * rowLength) * pixelWidth;

            let hue = deg;
            let saturation = r / radius;
            let value = 1.0;

            let [red, green, blue] = hsv2rgb(hue, saturation, value);
            let alpha = 255;

            data[index] = red;
            data[index + 1] = green;
            data[index + 2] = blue;
            data[index + 3] = alpha;
          }
        }

        ctx.putImageData(image, 0, 0);
      }

      function xy2polar(x, y) {
        let r = Math.sqrt(x * x + y * y);
        let phi = Math.atan2(y, x);
        return [r, phi];
      }

      //rad in [-π, π] range
      //return degree in [0, 360] range
      function rad2deg(rad) {
        return ((rad + Math.PI) / (2 * Math.PI)) * 360;
      }

      //hue in range [0, 360]
      //saturation, value in range [0,1]
      //return [r,g,b] each in range [0,255]
      //See: https://en.wikipedia.org/wiki/HSL_and_HSV#From_HSV
      function hsv2rgb(hue, saturation, value) {
        let chroma = value * saturation;
        let hue1 = hue / 60;
        let x = chroma * (1 - Math.abs((hue1 % 2) - 1));
        let r1, g1, b1;
        if(hue1 >= 0 && hue1 <= 1) {
          [r1, g1, b1] = [chroma, x, 0];
        } else if(hue1 >= 1 && hue1 <= 2) {
          [r1, g1, b1] = [x, chroma, 0];
        } else if(hue1 >= 2 && hue1 <= 3) {
          [r1, g1, b1] = [0, chroma, x];
        } else if(hue1 >= 3 && hue1 <= 4) {
          [r1, g1, b1] = [0, x, chroma];
        } else if(hue1 >= 4 && hue1 <= 5) {
          [r1, g1, b1] = [x, 0, chroma];
        } else if(hue1 >= 5 && hue1 <= 6) {
          [r1, g1, b1] = [chroma, 0, x];
        }

        let m = value - chroma;
        let [r, g, b] = [r1 + m, g1 + m, b1 + m];

        //Change r,g,b values from [0,1] to [0,255]
        return [255 * r, 255 * g, 255 * b];
      }
    },
  });
};

export const CrossHair = ({ position, show, radius = 20, ...props }) => {
  const [pos, setPos] = useState(position());
  const [visible, setVisible] = useState(show());

  position.subscribe(value => setPos(value));
  show.subscribe(value => setVisible(value));

  return h(
    'div',
    {
      style: {
        position: 'fixed',
        left: `${pos.x - radius}px`,
        top: `${pos.y - radius}px`,
        ...(visible ? {} : { display: 'none' }),
        zIndex: 100000,
      },
    },
    h(
      'svg',
      {
        viewBox: '0 0 22 22',
        style: {
          position: 'relative',
          width: `${radius * 2}px`,
          height: `${radius * 2}px`,
        },
      },
      h('path', {
        d: 'M11.004 22c-.007-9.184-.013-12.816 0-22M0 11.005c9.183-.007 12.816-.013 22 0M15.27 11A4.271 4.271 0 0111 15.27 4.271 4.271 0 016.729 11 4.271 4.271 0 0111 6.729a4.271 4.271 0 014.271 4.27zm3.645.015a7.932 7.931 0 01-7.932 7.932 7.932 7.931 0 01-7.931-7.932 7.932 7.931 0 017.931-7.931 7.932 7.931 0 017.932 7.931z',
        fill: 'none',
        stroke: '#000',
        strokeWidth: 1,
      }),
    ),
  );
};

export const MoveCursor = props =>
  h(
    'svg',
    {
      height: '22',
      width: '22',
      xmlns: 'http://www.w3.org/2000/svg',
    },
    [
      h('defs', {}),
      h('path', {
        d: 'M19.173 11.722l-2.393 2.393 1.044 1.044L22 10.983l-4.176-4.177L16.78 7.85l3.132 3.133zM5.221 14.115l-3.132-3.132L5.22 7.851 4.177 6.806 0 10.983l4.177 4.177zm6.535-11.288l2.398 2.394 1.044-1.044L11.018 0 6.84 4.177 7.885 5.22l3.132-3.133zm-1.473 16.345L7.885 16.78l-1.044 1.044L11.017 22l4.181-4.177-1.044-1.044-3.136 3.132zm-7.455-7.45h16.345l.739-.74-.74-.739H2.829l-.74.74zm8.928-8.895l-.739-.739-.734.74v7.415h1.473zm-1.473 16.345l.734.74.74-.74v-7.455h-1.474z',
      }),
    ],
  );

export const DropDown = ({ children, into /* = 'body'*/, isOpen = trkl(false), ...props }) => {
  let [button, overlay] = ReactComponent.toChildArray(children);
  const [open, setOpen] = useState(isOpen());
  isOpen.subscribe(value => setOpen(value));
  const [ref, rect, element] = useDimensions();
  const oref = useElement(element => {
    if(rect) {
      const xy = new Rect(rect).toPoints()[3];
      Element.setCSS(element, xy.toCSS());
    }
  });
  const event = useCallback(
    trkl().subscribe((e, prev) => {
      const { x, y, buttons, button, timeStamp } = e;
      const orect = Element.rect(oref.current);
      const timeStep = timeStamp - (prev ? prev.timeStamp : NaN);
      const points = [new Point(prev), new Point(e)];
      const diff = Point.diff(...points);
      const dist = Point.distance(...points);
      const inside = orect && orect.inside({ x, y });
      //   console.debug(e.type, diff, dist, {timeStep, orect, inside, x, y, buttons, button, timeStamp });
      if(e.button == 2) {
        e.preventDefault();
        return false;
      }
      if(oref.current && open && !inside) {
        isOpen(false);
        return false;
      }
      return true;
    }),
  );

  useEvent('mousedown', event);

  if(typeof button == 'function') button = button({ ref: ref, ...props });
  if(typeof overlay == 'function') overlay = overlay({ ref: oref, onMouseWheel });
  //console.log('DropDown open=', { open });

  function onMouseWheel(e) {
    const { deltaY, wheelDelta, wheelDeltaX, wheelDeltaY } = e;
    //console.log(e.type, ': ', { deltaY, wheelDelta, wheelDeltaX, wheelDeltaY });
    e.stopPropagation();
    return false;
  }

  return h(Fragment, {}, open ? [button, into ? h(Portal, { into }, overlay) : overlay] : button);
};

export const Fence = ({ children, style = {}, sizeListener, aspectListener, ...props }) => {
  const [dimensions, setDimensions] = useState(sizeListener());
  const [aspect, setAspect] = useState(aspectListener());
  if(sizeListener && sizeListener.subscribe) sizeListener.subscribe(value => setDimensions(value));
  if(aspectListener && aspectListener.subscribe) aspectListener.subscribe(value => setAspect(value));
  //console.debug('Fence dimensions:', dimensions);
  return h(
    TransformedElement,
    {
      id: 'fence',
      type: 'div' || SizedAspectRatioBox,
      'data-aspect': aspect,
      // listener: transform,
      style: {
        position: 'relative',
        minWidth: '100px',
        top: 0,
        left: 0,
        ...style,
        ...dimensions,
      },
      ...props,
    },
    children,
  );
};

export const Zoomable = ({ type = 'div', style, children, ...props }) => {
  const { transform, panZoomHandlers, setContainer, setPan, setZoom } = usePanZoom({
    zoomSensitivity: 0.001,
    minZoom: 1,
    initialZoom: 4,
    //minX: 0, minY: 0, //maxX: window.innerWidth, maxY: window.innerHeight,
    onPan,
    onZoom,
  });
  function onPan(arg) {
    //console.log('onPan', arg);
  }
  function onZoom(arg) {
    //console.log('onZoom', arg);
  }
  // console.log('Zoomable transform:', transform);
  let inner = trkl();
  const ref = el => {
    /*  if(el && typeof el.getBoundingClientRect == 'function')
          //console.log('Zoomable.container:', el.getBoundingClientRect());

        if(inner())
          //console.log('Zoomable.inner:', inner().getBoundingClientRect());*/
    setContainer(el);
  };
  return h(
    type,
    {
      ...props,
      ...panZoomHandlers,
    },
    h(type, { ref, style: { ...style, transform } }, children),
  );
};

export const DisplayList = ({ data, ...props }) => {
  let [items, setItems] = useState([]);

  let itemData = useValue(async function* () {
    for await(let item of data.repeater) {
      //console.log('DisplayList.item:', item);
      yield item;
    }
  });

  if(itemData) setItems(itemData);

  //console.log('itemData:', itemData);

  return h(Fragment, {}, ['DisplayList', ...items]);
};
/*
export const Conditional = ({ trkl, children, ...props }) => {
  const [cond, setCond] = useState(trkl());

  trkl.subscribe(setCond);+

  return h(Fragment, {}, cond ? children : []);
};
*/
export default {
  Overlay,
  Container,
  Chooser,
  Button,
  Label,
  Item,
  Icon,
  Progress,
  BoardIcon,
  File,
  FileList,
  Panel,
  AspectRatioBox,
  WrapInAspectBox,
  SizedAspectRatioBox,
  Canvas,
  ColorWheel,
  Slider,
  CrossHair,
  MoveCursor,
  Fence,
};