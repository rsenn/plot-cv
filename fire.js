import { crosskit, CANVAS } from './lib/crosskit.js';
import { RGBA, HSLA } from './lib/color.js';
import { Matrix } from './lib/geom/matrix.js';
//import { Element } from './lib/dom/element.js';
//import { Transformation, Rotation, Translation, Scaling, MatrixTransformation, TransformationList } from './lib/geom/transformation.js';
import { streamify, once, subscribe } from './lib/async/events.js';

function main() {
  Object.assign(globalThis, { crosskit, RGBA, HSLA, Util, Matrix });

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
  /*const paletteX = palette.map(color => color.hex());
  const palette32 = Uint32Array.from(palette, c => +c);*/
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
    matrix
  });

  async function Loop() {
    const delay = 1000 / fps;
    const log = (t, name) => globalThis.doLog && console.log(`${name} timing: ${t.toFixed(3)}ms`);
    const fire = (...args) => Fire(...args); //Util.instrument(Fire, log);
    const redraw = (...args) => Redraw(...args); //Util.instrument(Redraw, log);

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

    /* context.transform(...t);
    console.log("t:", context.currentTransform);
  */
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

      colors[i] = new RGBA(value, 0, 0); // black to red
      colors[i + 64] = new RGBA(255, value, 0); // red to yellow
      colors[i + 128] = new RGBA(255, 255, value); // yellow to white
      colors[i + 192] = new RGBA(255, 255, 255); // all white
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
    ]; /*.map(hsla => hsla.toRGBA())*/

    const breakpoints = [0, 51, 80, 154, 205, 256];
    console.log('breakpoints:', breakpoints);

    for(let i = 0; i < 256; i++) {
      const hue = (v => (v == -1 ? () => hues.length - 2 : v => v))(breakpoints.findIndex(b => i < b));
      const range = breakpoints[hue] - 1 - breakpoints[hue - 1];
      //console.log("hue:", {i,hue, range});

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
      //console.log(`${e.type}`, ...e.touches);
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

    //console.log(`${e.type} @ ${x},${y}`);

    try {
      if(/(down|start)$/.test(type)) rc = pixels[y][x] > 0x30 ? 0 : RandomByte() | 0x80;

      Blaze(x, y);
    } catch(e) {}
  }

  async function* MouseIterator() {
    for(;;) {
      yield await once(element, 'mousedown', 'touchstart');

      for await(let event of streamify(['mouseup', 'mousemove', 'touchend', 'touchmove'], element)) {
        yield event;
        if(/(up|end)$/.test(event.type)) break;
      }
    }
  }

  function ResizeHandler(e) {
    rect = element.getBoundingClientRect();
    console.log('rect', rect);
  }

  Object.assign(globalThis, { RandomByte });

  function Init() {
    window.canvas = element = document.querySelector('canvas');

    rect = element.getBoundingClientRect();

    window.addEventListener('resize', ResizeHandler, true);

    const handler =
      MouseHandler; /*|| Util.instrument(MouseHandler, (duration, name, args, ret) => console.log(`handler time: ${duration}`))*/

    subscribe(MouseIterator(), handler);
  }

  Loop();
}

main();
