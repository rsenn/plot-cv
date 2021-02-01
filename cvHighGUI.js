import * as cv from 'cv.so';
import * as draw from 'draw.so';
import { Size } from 'size.so';

import Util from './lib/util.js';

export const MouseEvents = [
  'EVENT_MOUSEMOVE',
  'EVENT_LBUTTONDOWN',
  'EVENT_RBUTTONDOWN',
  'EVENT_MBUTTONDOWN',
  'EVENT_LBUTTONUP',
  'EVENT_RBUTTONUP',
  'EVENT_MBUTTONUP',
  'EVENT_LBUTTONDBLCLK',
  'EVENT_RBUTTONDBLCLK',
  'EVENT_MBUTTONDBLCLK',
  'EVENT_MOUSEWHEEL',
  'EVENT_MOUSEHWHEEL'
].reduce((acc, name) => ({ ...acc, [cv[name]]: name }), {});

export const MouseFlags = [
  'EVENT_FLAG_LBUTTON',
  'EVENT_FLAG_RBUTTON',
  'EVENT_FLAG_MBUTTON',
  'EVENT_FLAG_CTRLKEY',
  'EVENT_FLAG_SHIFTKEY',
  'EVENT_FLAG_ALTKEY'
].reduce((acc, name) => ({ ...acc, [name]: cv[name] }), {});

export const Mouse = {
  printEvent: (() => {
    return event => MouseEvents[event].replace(/EVENT_/, '');
  })(),
  printFlags: (() => {
    const toks = Util.bitsToNames(MouseFlags, name => name.replace(/EVENT_FLAG_/, ''));

    return flags => [...toks(flags)];
  })()
};

export class Window {
  constructor(name, flags = cv.WINDOW_NORMAL) {
    this.name = name;
    this.flags = flags;

    cv.namedWindow(this.name, this.flags);
  }

  move(x, y) {
    cv.moveWindow(this.name, x, y);
  }

  resize(...args) {
    let size = new Size(...args);
    cv.resizeWindow(this.name, ...size);
  }

  get imageRect() {
    return cv.getWindowImageRect(this.name);
  }

  get(propId) {
    return cv.getWindowProperty(this.name, propId);
  }
  set(propId, value) {
    cv.setWindowProperty(this.name, propId, value);
  }

  setTitle(title) {
    this.title = title;
    cv.setWindowTitle(this.name, title);
  }

  setMouseCallback(fn) {
    cv.setMouseCallback(this.name, (event, x, y, flags) => {
      //console.log("MouseCallback", {event,x,y,flags});
      fn.call(this, event, x, y, flags);
    });
  }

  show(mat) {
    cv.imshow(this.name, mat);
  }

  valueOf() {
    return this.name;
  }
}

export function TextStyle(fontFace = cv.FONT_HERSHEY_PLAIN, fontScale = 1.0, thickness = 1) {
  Object.assign(this, { fontFace, fontScale, thickness });
}

Object.assign(TextStyle.prototype, {
  size(text) {
    const { fontFace, fontScale, thickness } = this;
    let baseY;
    let size = new Size(...draw.textSize(text, fontFace, fontScale, thickness, y => (baseY = y)));

    size.y = baseY;
    return size;
  },

  draw(mat, text, pos, color, lineThickness, lineType) {
    const { fontFace, fontScale, thickness } = this;
    draw.text(mat,
      text,
      pos,
      fontFace,
      fontScale,
      color ?? 0xffffff,
      lineThickness ?? thickness,
      lineType ?? cv.LINE_AA
    );
  }
});
