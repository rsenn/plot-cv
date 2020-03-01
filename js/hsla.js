import { RGBA } from "./rgba.js";

/**
 * @brief [brief description]
 * @param h  hue value 0-360
 * @param s  saturation 0-100%
 * @param l  luminance 0-100%
 * @param a  alpha 0-1.0
 *
 * @return [description]
 */
export function HSLA(h = 0, s = 0, l = 0, a = 1.0) {
  const args = [...arguments];
  let c = [];
  let ret = this instanceof HSLA ? this : {};
  /*  if(!this) return Object.assign({}, HSLA.prototype, { h, s, l, a });*/

  if(args.length >= 3) {
    ret.h = Math.round(h);
    ret.s = s;
    ret.l = l;
    ret.a = a;
  } else {
    const arg = args[0];
    if(typeof arg === "string") {
      var matches = /hsla\(\s*([0-9.]+)\s*,\s*([0-9.]+%?)\s*,\s*([0-9.]+%?),\s*([0-9.]+)\s*\)/g.exec(arg) || /hsl\(\s*([0-9.]+)\s*,\s*([0-9.]+%?)\s*,\s*([0-9.]+%?)\s*\)/g.exec(arg);

      if(matches != null) matches = [...matches].slice(1);
    }
    ret.h = c[0];
    ret.s = c[1];
    ret.l = c[2];
    ret.a = c[3] !== undefined ? c[3] : 1.0;

    ["h", "s", "l", "a"].forEach(channel => {
      if(String(ret[channel]).endsWith("%")) ret[channel] = parseFloat(ret[channel].slice(0, ret[channel].length - 1));
      else ret[channel] = parseFloat(ret[channel]);
    });
  }

  //console.log('HSLA ', { c, ret, args });
  if(!(ret instanceof HSLA)) return ret;
}

HSLA.prototype.properties = ["h", "s", "l", "a"];

//export const isHSLA = obj => HSLA.properties.every(prop => obj.hasOwnProperty(prop));

HSLA.prototype.css = function() {
  const hsla = HSLA.clamp(HSLA.round(this));
  return HSLA.setcss(hsla)();
};
HSLA.prototype.toHSL = function() {
  const { h, s, l } = this;
  return new HSLA(h, s, l, 1.0);
};

HSLA.prototype.clamp = function() {
  this.h = this.h % 360;
  this.s = Math.min(Math.max(this.s, 0), 100);
  this.l = Math.min(Math.max(this.l, 0), 100);
  this.a = Math.min(Math.max(this.a, 0), 1);
  return this;
};
HSLA.prototype.round = function() {
  this.h = Math.round(this.h);
  this.s = Math.round(this.s);
  this.l = Math.round(this.l);
  this.a = Math.round(this.a);
  return this;
};

HSLA.prototype.hex = function() {
  return RGBA.prototype.hex.call(HSLA.prototype.toRGBA.call(this));
};

HSLA.prototype.toRGBA = function() {
  var { h, s, l, a } = this;

  var r, g, b, m, c, x;

  if(!isFinite(h)) h = 0;
  if(!isFinite(s)) s = 0;
  if(!isFinite(l)) l = 0;

  h /= 60;
  if(h < 0) h = 6 - (-h % 6);
  h %= 6;

  s = Math.max(0, Math.min(1, s / 100));
  l = Math.max(0, Math.min(1, l / 100));

  c = (1 - Math.abs(2 * l - 1)) * s;
  x = c * (1 - Math.abs((h % 2) - 1));

  if(h < 1) {
    r = c;
    g = x;
    b = 0;
  } else if(h < 2) {
    r = x;
    g = c;
    b = 0;
  } else if(h < 3) {
    r = 0;
    g = c;
    b = x;
  } else if(h < 4) {
    r = 0;
    g = x;
    b = c;
  } else if(h < 5) {
    r = x;
    g = 0;
    b = c;
  } else {
    r = c;
    g = 0;
    b = x;
  }

  m = l - c / 2;
  r = Math.round((r + m) * 255);
  g = Math.round((g + m) * 255);
  b = Math.round((b + m) * 255);
  a = Math.round(a * 255);

  return new RGBA(r, g, b, a);
};

HSLA.prototype.toString = function() {
  if(this.a == 1) return `hsl(${this.h},${this.s}%,${this.l}%)`;
  return `hsla(${this.h},${this.s}%,${this.l}%,${this.a})`;
};
