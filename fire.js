import { crosskit, CANVAS } from './lib/crosskit.js';
import { RGBA } from './lib/color.js';
import Util from './lib/util.js';

Object.assign(globalThis, { RGBA, Util });

const w = 320;
const h = 200;

crosskit.init({
  renderer: CANVAS,
  w,
  h
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
const palette = CreatePalette().map(color => color.toCSS());
//console.log('Create palette:', palette);

const pixels = Array.from({ length: h + 2 }).map((v, i) => new Uint8ClampedArray(buffer, i * w, w)
);
const { now, waitFor, animationFrame } = Util;
const fps = 10;

Object.assign(globalThis, { buffer, palette, pixels, fps });

async function Loop() {
  const delay = 1000 / fps;
  let prev = 0;

  for(;;) {
    Fire();
    await animationFrame(delay);
  }
}

function Fire() {
  for(let x = 0; x < w; x++) {
    pixels[h][x] = (RandomByte() & 0b10111111) | 0x40;
    pixels[h + 1][x] = (RandomByte() & 0b10111111) | 0x40;
  }

  for(let y = h / 2; y < h; y++) {
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

  for(let y = 0; y < h; y++) {
    for(let x = 0; x < w; x++) {
      crosskit.pixel({
        x,
        y,
        color: palette[pixels[y][x]]
      });
    }
  }
}

function CreatePalette() {
  let colors = new Array(256);

  for(let i = 0; i < 64; i++) {
    const value = i * 4;

    colors[i] = new RGBA(value, 0, 0); // black to red
    colors[i + 64] = new RGBA(255, value, 0); // red to yellow
    colors[i + 128] = new RGBA(255, 255, value); // yellow to white
    colors[i + 192] = new RGBA(255, 255, 255); // all white
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

let element, rect;

function MouseHandler(e) {
  const { offsetX, offsetY, target, buttons, type } = e;

  if(type != 'click' && buttons == 0) return;

  const x = Math.round((offsetX * w) / rect.width);
  const y = Math.round((offsetY * h) / rect.height);

  //console.log(`${e.type} @ ${x},${y}`);

  const rc = RandomByte() | 0x80;

  for(let ty = y - 1; ty < y + 1; ty++)
    for(let tx = x - 1; tx < x + 1; tx++)
      pixels[ty][tx] = rc;
  
  pixels[y + 1][x] = rc;
}

function ResizeHandler(e) {
  rect = element.getBoundingClientRect();
}

Object.assign(globalThis, { RandomByte });

window.addEventListener('load', () => {
  element = document.querySelector('canvas');

  rect = element.getBoundingClientRect();

  const handler =
    MouseHandler ||
    Util.instrument(MouseHandler, (duration, name, args, ret) =>
      console.log(`handler time: ${duration}`)
    );

  element.addEventListener('click', handler);
  element.addEventListener('mousemove', handler);
});

Loop();
