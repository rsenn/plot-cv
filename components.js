import { h, html, render, Component, createContext, useState, useReducer, useEffect, useLayoutEffect, useRef, useImperativeHandle, useMemo, useCallback, useContext, useDebugValue } from './lib/dom/preactComponent.js';

import { trkl } from './lib/trkl.js';

//import React from '../modules/preact/dist/preact.mjs';

export function classNames() {
  let classes = [];

  for(let i = 0; i < arguments.length; i++) {
    let arg = arguments[i];
    if(!arg) continue;

    let argType = typeof arg;

    if(argType === 'string' || argType === 'number') {
      classes.push(arg);
    } else if(Array.isArray(arg) && arg.length) {
      let inner = classNames.apply(null, arg);
      if(inner) {
        classes.push(inner);
      }
    } else if(argType === 'object') {
      for(let key in arg) {
        if(hasOwn.call(arg, key) && arg[key]) {
          classes.push(key);
        }
      }
    }
  }

  return classes.join(' ');
}

export const MouseHandler = callback => e => {
  if(e.type) {
    const pressed = e.type.endsWith('down');
    callback(e, pressed);
  }
};

export const MouseEvents = h => ({
  onMouseDown: h,

  /*  onBlur: h,*/
  onMouseOut: h,
  onMouseUp: h
});

export const Overlay = ({ className = 'overlay', active = false, onPush, text, children, ...props }) => {
  const [pushed, setPushed] = useState(false);
  const events = MouseEvents(
    MouseHandler((e, state) => {
      const prev = pushed;
      if(!e.type.endsWith('down') && !e.type.endsWith('up')) return;
      setPushed(state);
      //console.log(`overlay pushed=${pushed} active=${active}:`, e.target);
      return typeof onPush == 'function' ? onPush(e, state) : null;
    })
  );

  return html`
    <div className=${classNames(className, pushed && 'pushed', active ? 'active' : 'inactive')} ...${props} ...${events}>
      ${text} ${children}
    </div>
  `;
};

export const Container = ({ className = 'panel', children, ...props }) => {
  useCallback(() => console.log('re-render panel'));
  return html`
    <div className=${className} ...${props}>${children}</div>
  `;
};

export const Button = ({ caption, fn }) => html`
  <${Overlay} className="button" text=${caption} onPush=${state => (state ? fn(state) : undefined)} />
`;

export const Label = ({ className = 'label', text, children, ...props }) =>
  html`
    <div className=${className} ...${props}>${text}${children}</div>
  `;

export const Item = ({ className = 'item', label, icon, children, ...props }) => html`
            <${Overlay} className=${className}  ...${props}>
                <${Label} text=${icon} ...${props}>${label}</${Label}>
            </${Overlay}>
          `;

export const Icon = ({ className = 'icon', caption, image, ...props }) => html`
  <div className=${className} ...${props}>${caption}<img src=${image} /></div>
`;

export const Progress = ({ className, percent, ...props }) =>
  html`<${Overlay} className=${classNames('progress', 'center', className)} text=${percent + '%'} style=${{
    position: 'relative',
    width: '100%',
    height: '1.5em',
    border: '1px solid black',
    textAlign: 'center',
    zIndex: '99'
  }}><div className=${classNames('progress-bar', 'fill')} style=${{
    width: percent + '%',
    position: 'absolute',
    left: '0px',
    top: '0px',
    zIndex: '98'
  }}></div></${Overlay}>`;

const SchematicIcon = props => html`
  <svg viewBox="0 0 40 40" xmlns="http://www.w3.org/2000/svg">
    <defs>
      <clipPath id="Sch_svg__a">
        <path d="M0 32h32V0H0v32z" />
      </clipPath>
    </defs>
    <g clipPath="url(#Sch_svg__a)" transform="matrix(1.25 0 0 -1.25 0 40)">
      <path
        d="M26.463 27c0 .817-.308 1.555-.847 2.125a2.785 2.785 0 01-2.038.873H10.212c-.774 0-1.494-.3-2.03-.873A3.028 3.028 0 017.34 27V4.996c0-1.648 1.29-2.994 2.873-2.994H20.727v6.143h5.736V27zm1.904-19.07a2.182 2.182 0 00-.56-1.34L22.075.586A1.9 1.9 0 0020.71 0H10.212C7.557 0 5.43 2.241 5.43 4.996V27c0 1.388.54 2.642 1.403 3.547.87.892 2.053 1.453 3.379 1.453h13.366a4.688 4.688 0 003.382-1.453A5.137 5.137 0 0028.367 27V7.93z"
        fill="#444443"
      />
      <path d="M26.459 12.143H3.628v14h22.831v-14z" fill="#b4b3b3" />
      <path
        d="M10.956 16.95c.23-.297.401-.561.401-.952 0-.86-.654-1.297-1.445-1.297H7.387c-.437 0-.815.08-1.16.368-.31.252-.539.665-.539 1.067 0 .367.206.689.608.689.333 0 .54-.275.54-.585 0-.345.183-.391.505-.391h2.49c.092 0 .378-.035.378.139 0 .09-.16.263-.206.332l-3.937 5.06c-.24.31-.378.54-.378.943 0 .459.218.849.608 1.1.321.208.585.208.952.208h2.434c.436 0 .803-.092 1.136-.38.3-.263.54-.688.54-1.09 0-.367-.184-.665-.597-.665-.345 0-.552.218-.552.539 0 .39-.194.448-.55.448H7.226c-.126 0-.39.034-.39-.16 0-.104.126-.242.171-.311l3.95-5.061zM14.902 14.701c-.861 0-1.354.344-1.722 1.091l-1.055 2.134c-.196.39-.31.757-.31 1.205 0 .505.09.815.32 1.274l1.057 2.124c.39.791.884 1.101 1.767 1.101h1.95c.253 0 .576-.137.576-.574 0-.413-.311-.574-.575-.574h-1.916c-.494 0-.574-.046-.792-.482l-1-1.997c-.148-.309-.24-.516-.24-.872 0-.322.104-.516.24-.792l.976-1.985c.23-.47.346-.506.86-.506h1.872c.252 0 .575-.137.575-.573 0-.413-.311-.574-.575-.574h-2.008zM19.838 18.592v-3.317c0-.368-.16-.574-.573-.574-.413 0-.574.206-.574.574v7.782c0 .367.16.574.574.574.413 0 .573-.207.573-.574V19.74h3.375v3.317c0 .367.16.574.573.574.413 0 .574-.207.574-.574v-7.782c0-.368-.161-.574-.574-.574-.413 0-.573.206-.573.574v3.317h-3.375z"
        fill="#bf272f"
      />
    </g>
  </svg>
`;

export const BoardIcon = props =>
  h(
    'svg',
    {
      viewBox: '0 0 40 40',
      xmlns: 'http://www.w3.org/2000/svg'
    },
    [
      h(
        'defs',
        {},
        h(
          'clipPath',
          {
            id: 'Brd_svg__a'
          },
          h('path', {
            d: 'M0 32h32V0H0v32z'
          })
        )
      ),
      h(
        'g',
        {
          clipPath: 'url(#Brd_svg__a)',
          transform: 'matrix(1.25 0 0 -1.25 0 40)'
        },
        [
          h('path', {
            d:
              'M26.463 27c0 .817-.308 1.555-.847 2.125a2.785 2.785 0 01-2.038.873H10.212c-.774 0-1.494-.3-2.03-.873A3.028 3.028 0 017.34 27V4.996c0-1.648 1.29-2.994 2.873-2.994H20.727v6.143h5.736V27zm1.904-19.07a2.182 2.182 0 00-.56-1.34L22.075.586A1.9 1.9 0 0020.71 0H10.212C7.557 0 5.43 2.241 5.43 4.996V27c0 1.388.54 2.642 1.403 3.547.87.892 2.053 1.453 3.379 1.453h13.366a4.688 4.688 0 003.382-1.453A5.137 5.137 0 0028.367 27V7.93z',
            fill: '#444443'
          }),
          h('path', {
            d: 'M26.463 12.164H3.633v14h22.83v-14z',
            fill: '#b4b3b3'
          }),
          h('path', {
            d:
              'M6.99 15.835h2.087c.835 0 1.322.37 1.322 1.252 0 .974-.266 1.52-1.322 1.52H6.99v-2.773zm0 3.931h2.087c.835 0 1.322.372 1.322 1.253 0 .974-.266 1.519-1.322 1.519H6.99v-2.772zm-1.16 3.932h3.247c1.705 0 2.482-1.044 2.482-2.68 0-.671-.15-1.286-.765-1.831.66-.626.765-1.241.765-2.1 0-1.52-1.009-2.411-2.482-2.411H5.83v9.022zM13.95 20.729h2.574c.325 0 .836.023.836.487v.8c0 .464-.476.522-.824.522H13.95v-1.809zm0-5.474c0-.37-.163-.579-.58-.579-.418 0-.58.209-.58.579v8.443h3.897c.917 0 1.833-.65 1.833-1.682v-.8c0-1.091-.904-1.647-1.833-1.647H15.62l2.609-3.735c.127-.173.29-.393.29-.614 0-.324-.267-.544-.591-.544-.232 0-.383.138-.51.324l-3.201 4.569h-.267v-4.314zM21.654 15.835h.221c.488 0 .603.058.812.486l1.02 2.065c.13.267.22.465.22.755 0 .394-.127.614-.289.95l-.963 1.96c-.244.499-.347.487-.88.487h-.14v-6.703zm-1.158 6.702h-.43c-.383 0-.707.14-.707.58 0 .418.313.58.672.58h1.937c.905 0 1.356-.37 1.75-1.16l1.022-2.04c.23-.476.348-.812.348-1.346 0-.463-.127-.858-.336-1.275l-1.057-2.11c-.404-.825-.928-1.09-1.832-1.09H19.94c-.303 0-.58.161-.58.58 0 .416.277.578.58.578h.557v6.704z',
            fill: '#42ab3d'
          })
        ]
      )
    ]
  );

export const Conditional = ({ initialState, component, className, children, signal, ...props }) => {
  const [show, setShown] = useState(initialState !== undefined ? initialState : signal());

  if(signal) signal.subscribe(value => setShown(value));

  return show ? h(component, { className, ...props }, children) : [];
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
      ...props
    })
  );
};

export const File = ({ label, i, key, className = 'file', onPush, signal, data, doc, ...props }) => {
  const [loaded, setLoaded] = useState(NaN);
  if(signal) signal.subscribe(data => setLoaded(data.percent));
  onPush =
    onPush ||
    (async state => {
      //console.log(`loading "${name}"...`);
      await load(name);
    });
  let name;
  let id = label || key || i;
  let style = { minWidth: '40px', width: '40px', height: '40px' };
  let icon = /brd$/i.test(id || className) ? h(BoardIcon, { style }) : h(SchematicIcon, {});
  icon = h('div', { style }, icon);
  if(id) {
    name = id;
    id = (id + '').replace(/[^._A-Za-z0-9]/g, '-');
  }
  label = label.replace(/\.[^.]*$/, '').replace(/([^\s])-([^\s])/g, '$1 $2');
  //data = signal();
  /*console.log(`File`, { name, id,  label });
  //console.log(`File`, props);*/

  return html`
              <${Item} className=${className} id=${id} data-filename="${name}" onPush=${onPush} label=${label} icon=${icon} ...${props}>
              <${Progress} className=${!isNaN(loaded) ? 'visible' : 'hidden'} percent=${loaded} />
              </${Item}>
            `;
};

export const Chooser = ({ className = 'list', itemClass = 'item', itemComponent = Overlay, itemFilter, items, onChange = () => {}, onPush = () => {}, ...props }) => {
  const [active, setActive] = useState(-1);
  const [filter, setFilter] = useState('.*');
  const pushHandler = i => (e, state) => {
    const prev = active;
    state == true && setActive(i);
    if(i != prev && e.type.endsWith('down')) onChange(e, items[i], i);
    onPush(e, i, state);
  };

  if(itemFilter) {
    setFilter(itemFilter());
    itemFilter.subscribe(value => setFilter(value));
  }

  const bar = html``;
  const reList = filter
    .split(/\s\s*/g)
    .map(part => Util.tryCatch(() => new RegExp(part.replace(/\./g, '\\.').replace(/\*/g, '.*'), 'i')))
    .filter(r => r !== null);
  const pred = name => reList.every(re => re.test(name));
  const other = items.filter(({ name }) => !pred(name)).map(i => i.name);
  //console.log(reList);
  const children = items
    .filter(({ name }) => pred(name))
    .map(({ name, i, data, ...item }, key) =>
      //console.log(`Chooser item #${i}:`, { name, data, item });
      h(itemComponent, {
        key: key || i,
        className: classNames(itemClass || className + '-item', (name + '').replace(/.*\./, '')),
        active: i == active,
        onPush: pushHandler(i),
        label: name.replace(/.*\//, '') /*,
      ...item*/
      })
    );
  return html`<${Container} className=${classNames('panel', className)} ...${props}>${children}</${Container}>`;
};

export const FileList = ({ files, onChange, onActive, filter, showSearch, focusSearch, currentInput, changeInput, ...props }) => {
  const [active, setActive] = useState(true);
  const [items, setItems] = useState(files());

  files.subscribe(value => setItems(value));

  onActive.subscribe(value => setActive(value));

  return html`
    <div className=${classNames('sidebar', active ? 'active' : 'inactive')}>
      <${Conditional} component=${EditBox} type="form" className="search" autofocus name=${'query'} id="search" placeholder="Search" signal=${showSearch} focus=${focusSearch} current=${currentInput} onChange=${changeInput} onInput=${changeInput} value=${filter()} />
      <${Chooser}
        className="list"
        itemComponent=${File}
        itemClass="file hcenter"
        itemFilter=${filter}
        items=${items}
        onChange=${(...args) => {
          onChange(...args);
        }}
        ...${props}
      />
    </div>
  `;
};

export const Panel = (name, children) => html`<${Container} className="${name}">${children}</${Container}>`;

export const WrapInAspectBox = (enable, { width = '100%', aspect = 1, className }, children) =>
  enable
    ? h(
        SizedAspectRatioBox,
        {
          className,
          width,
          aspect,
          style: { overflow: 'visible' }
        },
        children
      )
    : h(
        'div',
        {
          className,
          style: { width }
        },
        children
      );

export const AspectRatioBox = ({ aspect = 1.0, children, insideClassName, outsideClassName, outsideProps = {}, style, ...props } /* console.log('AspectRatioBox ', { props, aspect, children, insideClassName, outsideClassName, style });*/) =>
  h(React.Fragment, {}, [
    h(
      'div',
      {
        className: classNames('aspect-ratio-box', outsideClassName),
        style: { height: 0, paddingBottom: (1.0 / aspect) * 100 + '%', ...style }
      },
      [
        h(
          'div',
          {
            className: classNames('aspect-ratio-box-inside', insideClassName)
          },
          children
        )
      ]
    )
  ]);

export const SizedAspectRatioBox = ({ width, height, style, className, children, outsideClassName, insideClassName, insideProps, outsideProps = {}, sizeClassName, sizeProps = {}, onClick, ...props }) =>
  h(
    'div',
    {
      className: classNames('aspect-ratio-box-size', className && className + '-size', sizeClassName),
      style: { position: 'relative', width, height, ...style },
      onClick
    },
    [
      h(
        AspectRatioBox,
        {
          outsideClassName: classNames('aspect-ratio-box-outside', className && className + '-outside', outsideClassName),
          outsideProps,
          insideClassName: insideClassName || className,
          onClick,
          ...props
        },
        children
      )
    ]
  );

export const TransformedElement = ({ type = 'div', aspect, listener, style = { position: 'relative' }, className, children = [], ...props }) => {
  const [transform, setTransform] = useState(new TransformationList());

  //console.log('TransformedElement:', { aspect });
  if(listener && listener.subscribe)
    listener.subscribe(value => {
      //console.log('TransformedElement setValue', value+'');
      if(value !== undefined) setTransform(value);
    });

  return h(
    type,
    {
      className: classNames('transformed-element', className && className + '-size'),
      style: { position: 'relative', ...style, transform },
      aspect
    },
    children
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
        ...style
      }
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
          ...dim
        },
        ...props,
        value,
        onInput
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
          flex: '0 1 auto'
        },
        ...props,
        value,
        onInput
      })
    ]
  );
};

export const Canvas = ({ onInit, ...props }) => {
  const [drawing, setDrawing] = useState(false);
  const [width, setWidth] = useState(props.width);
  const [height, setHeight] = useState(props.height);
  const canvasRef = useRef();
  const ctx = useRef();

  useEffect(() => {
    //console.log('canvasRef.current', canvasRef.current);
    ctx.current = canvasRef.current.getContext('2d');
    //console.log('ctx.current', ctx.current);
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
      onMouseMove: handleMouseMove
    },
    []
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
            let adjustedX = x + radius; //convert x from [-50, 50] to [0, 100] (the coordinates of the image data array)
            let adjustedY = y + radius; //convert y from [-50, 50] to [0, 100] (the coordinates of the image data array)
            let pixelWidth = 4; //each pixel requires 4 slots in the data array
            let index = (adjustedX + adjustedY * rowLength) * pixelWidth;

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
    }
  });
};

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
  Slider
};
