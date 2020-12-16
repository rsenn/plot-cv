import cv from 'cv';
import Util from './lib/util.js';

export const MouseEvents = ["EVENT_MOUSEMOVE", "EVENT_LBUTTONDOWN", "EVENT_RBUTTONDOWN", "EVENT_MBUTTONDOWN", "EVENT_LBUTTONUP", "EVENT_RBUTTONUP", "EVENT_MBUTTONUP", "EVENT_LBUTTONDBLCLK", "EVENT_RBUTTONDBLCLK", "EVENT_MBUTTONDBLCLK", "EVENT_MOUSEWHEEL", "EVENT_MOUSEHWHEEL"
].reduce((acc,name) => ({ ...acc, [name]: cv[name] }), {});

export const MouseFlags =   ["EVENT_FLAG_LBUTTON", "EVENT_FLAG_RBUTTON", "EVENT_FLAG_MBUTTON", "EVENT_FLAG_CTRLKEY", "EVENT_FLAG_SHIFTKEY", "EVENT_FLAG_ALTKEY"
].reduce((acc,name) => ({ ...acc, [name]: cv[name] }), {});

export const Mouse = {
  printFlags: Util.bitsToNames(MouseFlags),
  printEvents: Util.bitsToNames(MouseEvents),

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

  resize(width, height) {
    cv.resizeWindow(this.name, width, height);
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
}
