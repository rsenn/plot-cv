import { RGBA } from './lib/color/rgba.js';
let args = scriptArgs;
let colorStr = args.shift() || '#00f';
let color = new RGBA(colorStr);
let hsla = color.toHSLA();

/*//console.log(color);
//console.log(hsla.toString());*/

const colors = ['#ffffff', '#4b4ba5', '#4ba54b', '#4ba5a5', '#a54b4b', '#a54ba5', '#a5a54b', '#e6e6e6', '#4b4bff', '#4bff4b', '#4bffff', '#ff4b4b', '#ff4bff', '#ffff4b', '#4b4b4b', '#a5a5a5'];

let out = colors.map(hex => {
  let rgba = RGBA.fromHex(hex);
  let hsla = rgba.toHSLA();

  return rgba.toString(',');
});

console.log(out.map(c => `"${c}"`).join(', '));