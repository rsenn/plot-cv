import * as cv from 'cv';

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
    cv.setMouseCallback(this.name, (event, x, y, flags) => fn.call(this, event, x, y, flags));
  }

  show(mat) {
    cv.imshow(this.name, mat);
  }
}
