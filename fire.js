import { crosskit, CANVAS } from './lib/crosskit.js';
import { RGBA, HSLA } from './lib/color.js';
import { WebSocketIterator, WebSocketURL, CreateWebSocket, ReconnectingWebSocket, StreamReadIterator } from './lib/async/websocket.js';
import { once, streamify, throttle, distinct, subscribe } from './lib/async/events.js';
import { memoize, define, isUndefined, properties, keys, unique } from './lib/misc.js';
import { isStream, AcquireReader, AcquireWriter, ArrayWriter, readStream, PipeTo, WritableRepeater, WriteIterator, AsyncWrite, AsyncRead, ReadFromIterator, WriteToRepeater, LogSink, StringReader, LineReader, DebugTransformStream, CreateWritableStream, CreateTransformStream, RepeaterSource, RepeaterSink, LineBufferStream, TextTransformStream, ChunkReader, ByteReader, PipeToRepeater, Reader, ReadAll, default as utils } from './lib/stream/utils.js';
import { Intersection, Matrix, isRect, Rect, Point, Line, TransformationList, Vector } from './lib/geom.js';
import { Element, isElement } from './lib/dom/element.js';

function NewWS() {
  let url = WebSocketURL('/ws', { mirror: currentFile });
  return CreateWebSocket(url, 'lws-mirror-protocol');
}

function main() {
  define(globalThis, { crosskit, RGBA, HSLA, Util, Matrix, TransformationList });
  define(globalThis, { WebSocketIterator, WebSocketURL, CreateWebSocket, NewWS, ReconnectingWebSocket });
  define(globalThis, { define, isUndefined, properties, keys });
  define(globalThis, { once, streamify, throttle, distinct, subscribe });
  define(globalThis, { Intersection, Matrix, isRect, Rect, Point, Line, TransformationList, Vector });
  define(globalThis, {
    isStream,
    AcquireReader,
    AcquireWriter,
    ArrayWriter,
    readStream,
    PipeTo,
    WritableRepeater,
    WriteIterator,
    AsyncWrite,
    AsyncRead,
    ReadFromIterator,
    WriteToRepeater,
    LogSink,
    StringReader,
    LineReader,
    DebugTransformStream,
    CreateWritableStream,
    CreateTransformStream,
    RepeaterSource,
    RepeaterSink,
    LineBufferStream,
    TextTransformStream,
    ChunkReader,
    ByteReader,
    PipeToRepeater,
    Reader,
    ReadAll,
    StreamReadIterator
  });

  define(
    globalThis,
    properties(
      {
        currentURL: () => new URL(import.meta.url),
        currentFile: () => globalThis.currentURL.pathname.replace(/^\//, '')
      },
      { memoize: true }
    )
  );

  const w = 320;
  const h = 200;
  const parent = document.body;

  crosskit.init({
    renderer: CANVAS,
    parent,
    w,
    h,
    alpha: false
  });

  function Reparent(canvas = document.getElementsByTagName('canvas')[0]) {
    canvas.parentElement.removeChild(canvas);
    document.getElementById('canvas').appendChild(canvas);
  }

  function GetElementMatrix(element) {
    let { transform } = Element.getCSS(divElement);

    return Matrix.fromCSS(transform);
  }

  function SetCrosshair(pos) {
    let ch = Element.find('#crosshair');
    let rect = Element.getRect(ch);

    console.log('SetCrosshair', { ch, rect, pos });
    rect.x = pos.x - rect.width / 2;
    rect.y = pos.y - rect.height / 2;

    Element.setRect(ch, rect);
  }

  function EventPositions(eventOrTouch) {
    let positions = unique(
      keys(eventOrTouch, 2)
        .filter(n => typeof n == 'string' && /[XY]$/.test(n))
        .map(n => n.slice(0, -1))
    );

    return positions.reduce((acc, key) => {
      acc[key] = new Point(eventOrTouch[key + 'X'], eventOrTouch[key + 'Y']);
      return acc;
    }, {});
  }

  function ProcessPosition(pos) {
    let vertical = canvasRect.aspect() < 1;
    let ret;
    let { width: xres, height: yres } = canvasElement;
    let fx, fy;

    if(canvasRect.inside(pos)) {
      let dx = pos.x - canvasRect.x;
      let dy = pos.y - canvasRect.y;

      ret = new Point(dx, dy);

      if(!vertical) {
        fx = xres / canvasRect.width;
        fy = yres / canvasRect.height;
      } else {
        fx = xres / canvasRect.height;
        fy = yres / canvasRect.width;
      }

      ret.mul(new Point(fx, fy));
    }

    return ret;
  }

  async function* MouseIterator(element) {
    for(;;) {
      yield await once(element, 'mousedown', 'touchstart');

      for await(let event of streamify(['mouseup', 'mousemove', 'touchend', 'touchmove'], element)) {
        if('touches' in event) {
          if(event.touches.length) {
            globalThis.mouseEvent = event;

            yield* [...event.touches].map(EventPositions);
            //.map(({ client }) => client)
          }
          /*const{clientX:x,clientY:y}=touch;
          yield {x,y};
         } */
        } else {
          const { type, clientX: x, clientY: y } = event;
          yield { type, x, y };
        }
        if(/(up|end)$/.test(event.type)) break;
      }
    }
  }
  Reparent();

  crosskit.clear();
  crosskit.rect({
    x: 0,
    y: 0,
    w,
    h,
    fill: 'black',
    stroke: 'black',
    angle: 0
  });

  const buffer = new ArrayBuffer(w * (h + 2));
  const palette = CreatePalette();
  const paletteHSL = CreatePaletteHSL();

  const pixels = Array.from({ length: h + 2 }).map((v, i) => new Uint8ClampedArray(buffer, i * w, w));
  const { context } = crosskit;
  const image = context.createImageData(w, h);

  const { now, waitFor, animationFrame } = Util;
  const fps = 50;
  const matrix = new Matrix().translate(160, 100).scale(0.5);

  Object.assign(globalThis, {
    buffer,
    palette,
    paletteHSL,
    pixels,
    context,
    image,
    fps,
    matrix,
    Reparent,
    dom: { Element },
    geom: { Rect },
    MouseIterator,
    MouseHandler,
    GetElementMatrix,
    SetCrosshair,
    EventPositions,
    ProcessPosition
  });

  async function Loop() {
    const delay = 1000 / fps;
    const log = (t, name) => globalThis.doLog && console.log(`${name} timing: ${t.toFixed(3)}ms`);
    const fire = (...args) => Fire(...args);
    const redraw = (...args) => Redraw(...args);

    await once(window, 'load');

    Init();

    for(;;) {
      fire();
      redraw();
      await animationFrame(delay);
    }
  }

  function Fire() {
    for(let x = 0; x < w; x++) {
      pixels[h][x] = 255 - (RandomByte() % 128);
      pixels[h + 1][x] = 255 - (RandomByte() % 128);
    }

    for(let y = 0; y < h; y++) {
      for(let x = 0; x < w; x++) {
        const sum = [
          pixels[y + 1][Modulo(x - 1, w)],
          pixels[y + 1][x],
          pixels[y + 1][Modulo(x + 1, w)],
          pixels[y + 2][x]
        ].reduce((a, p) => a + (p | 0), 0);

        pixels[y][x] = (sum * 15) >>> 6;
      }
    }
  }

  async function Redraw() {
    const { data } = image;

    let i = 0;
    let t = [...matrix];

    for(let y = 0; y < h; y++) {
      for(let x = 0; x < w; x++) {
        const c = palette[pixels[y][x]];
        data[i++] = c.r;
        data[i++] = c.g;
        data[i++] = c.b;
        data[i++] = c.a;
      }
    }

    context.putImageData(image, 0, 0);
  }

  function CreatePalette() {
    const colors = new Array(256);

    for(let i = 0; i < 64; i++) {
      const value = i * 4;

      colors[i] = new RGBA(value, 0, 0);
      colors[i + 64] = new RGBA(255, value, 0);
      colors[i + 128] = new RGBA(255, 255, value);
      colors[i + 192] = new RGBA(255, 255, 255);
    }
    return colors;
  }

  function CreatePaletteHSL() {
    const colors = new Array(256);

    const hues = [
      new HSLA(0, 100, 0),
      new HSLA(0, 100, 50),
      new HSLA(30, 100, 50),
      new HSLA(60, 100, 50),
      new HSLA(60, 100, 100),
      new HSLA(60, 100, 100)
    ];

    const breakpoints = [0, 51, 80, 154, 205, 256];
    console.log('breakpoints:', breakpoints);

    for(let i = 0; i < 256; i++) {
      const hue = (v => (v == -1 ? () => hues.length - 2 : v => v))(breakpoints.findIndex(b => i < b));
      const range = breakpoints[hue] - 1 - breakpoints[hue - 1];

      colors[i] = HSLA.blend(hues[hue - 1], hues[hue], (i - breakpoints[hue - 1]) / range).toRGBA();
    }
    return colors;
  }

  // For random numbers, use "x = 181 * x + 359" from
  // Tom Dickens "Random Number Generator for Microcontrollers"
  // https://web.archive.org/web/20170323204917/http://home.earthlink.net/~tdickens/68hc11/random/68hc11random.html
  let scratch = 0;

  function RandomByte() {
    const value = 181 * scratch + 359;
    scratch = value >>> 0;
    return (value >>> 8) & 0xff;
  }

  function Modulo(n, m) {
    return ((n % m) + m) % m;
  }

  let element, rect, rc;

  function Blaze(x, y) {
    for(let ty = y - 1; ty < y + 1; ty++) {
      for(let tx = x - 1; tx < x + 1; tx++) {
        pixels[ty][tx] = rc;
      }
    }

    pixels[y + 1][x] = rc;
  }

  function MouseHandler(e) {
    let { target, buttons, type } = e;

    if('touches' in e) {
      for(let touch of [...e.touches]) {
        const { clientX, clientY } = touch;
        MouseHandler({
          type,
          target,
          buttons,
          offsetX: Math.trunc(clientX) - rect.x,
          offsetY: Math.trunc(clientY) - rect.y
        });
      }
      return;
    }

    globalThis.pointerEvent = e;

    const x = Math.round((e.offsetX * w) / rect.width);
    const y = Math.round((e.offsetY * h) / rect.height);

    try {
      if(/(down|start)$/.test(type)) rc = pixels[y][x] > 0x30 ? 0 : RandomByte() | 0x80;

      Blaze(x, y);
    } catch(e) {}
  }

  function ResizeHandler(e) {
    rect = element.getBoundingClientRect();
    console.log('ResizeHandler', { e, rect });
  }

  function OrientationChange(e) {
    rect = element.getBoundingClientRect();
    console.log('OrientationChange', { e, rect });
  }

  Object.assign(globalThis, { RandomByte });

  function Init() {
    window.canvas = element = document.querySelector('canvas');

    rect = element.getBoundingClientRect();

    define(
      globalThis,
      properties({
        canvasElement: () => Element.find('canvas'),
        divElement: () => Element.find('body > div:first-child')
      }),
      properties({
        windowRect: () => new Rect(window.innerWidth, window.innerHeight),
        bodyRect: () => Element.rect('body').round(0),
        canvasRect: () => Element.rect('canvas').round(0),
        divRect: () => Element.rect('body > div:first-child').round(0)
      }),
      {
        getRect
      },
      properties({
        transform: [
          () => new TransformationList(Element.getCSS('body > div:first-child').transform),
          value => Element.setCSS('body > div:first-child', { transform: value + '' })
        ]
      })
    );

    (async function() {
      for await(let event of streamify(['orientationchange', 'resize'], document)) {
        console.log(event.type, event);
        globalThis[event.type] = event;
      }
    })();

    window.addEventListener('resize', ResizeHandler, true);
    window.addEventListener('orientationchange', OrientationChange, true);

    const handler = MouseHandler;

    subscribe(MouseIterator(element), handler);
  }

  Loop();
}

main();

function getRect(elem) {
  return new Rect((elem ?? divElement).getBoundingClientRect()).round(1);
}
