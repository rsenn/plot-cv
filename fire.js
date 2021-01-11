import { crosskit, CANVAS } from './lib/crosskit.js';
import { RGBA } from './lib/color.js';

const w = 640;
const h = 480;

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

const buffer = new ArrayBuffer(w * (h + 1));
const palette = CreatePalette().map(color => color.toCSS());
//console.log('Create palette:', palette);

function Fire() {
  const pixels = Array.from({ length: h + 1 }).map((v, i) => new Uint8ClampedArray(buffer, (h - i) * w, w));

  for(let x = 0; x < w; x++) {
    pixels[0][x] = RandomByte() | 0xc0;
  }

  for(let y = 1; y <= h; y++) {
    for(let x = 0; x < w; x++) {
      const value = [pixels[y - 1][x - 1], pixels[y - 1][x], pixels[y - 1][x + 1], pixels[y][x]].reduce((a, p) => a + (p | 0),
        0
      );

      pixels[y][x] = value >>> 2;
    }
  }

  for(let y = 0; y < h; y++) {
    for(let x = 0; x < w; x++) {
      crosskit.pixel({
        x,
        y,
        color: palette[pixels[h - y][x]]
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

function RandomByte() {
  return Math.round(Math.random() * 255);
}
