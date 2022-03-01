import { CONTEXT_VERSION_MAJOR, CONTEXT_VERSION_MINOR, OPENGL_CORE_PROFILE, OPENGL_FORWARD_COMPAT, OPENGL_PROFILE, RESIZABLE, SAMPLES, Window, Size, Position, context, poll } from 'glfw';
export { Window, Size, Position } from 'glfw';
import * as nvg from 'nanovg';
import { RGBA } from './lib/color.js';
import { glClear, glClearColor, glViewport, GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT, GL_STENCIL_BUFFER_BIT } from './gl.js';

export function GLFW(width, height, options = {}) {
  let resolution = new Size(width, height);
  const {
    resizable = false,
    samples = 4,
    contextVersionMajor = 3,
    contextVersionMinor = 2,
    openglProfile = OPENGL_CORE_PROFILE,
    openglForwardCompat = true,
    title = 'OpenGL',
    ...handlers
  } = options;
  const hints = [
    [CONTEXT_VERSION_MAJOR, contextVersionMajor],
    [CONTEXT_VERSION_MINOR, contextVersionMinor],
    [OPENGL_PROFILE, openglProfile],
    [OPENGL_FORWARD_COMPAT, openglForwardCompat],
    [RESIZABLE, resizable],
    [SAMPLES, samples]
  ];

  for(let [prop, value] of hints) Window.hint(prop, value);

  let window = (context.current = new Window(resolution.width, resolution.height, title));

  Object.assign(window, { ...GLFW.defaultCallbacks, ...handlers });

  nvg.CreateGL3(nvg.STENCIL_STROKES | nvg.ANTIALIAS | nvg.DEBUG);

  return Object.assign(this, { resolution, title, window });
}

GLFW.defaultCallbacks = {
  handlePos(x, y) {
    console.log('handlePos', { x, y });
  },
  handleSize(width, height) {
    console.log('handleSize', { width, height });
  },
  handleClose(window = this) {
    console.log('handleClose', { window });
  },
  handleRefresh(window = this) {
    //console.log('handleRefresh', { window });
  },
  handleFocus(focused) {
    console.log('handleFocus', { focused });
  },
  handleIconify(iconified) {
    console.log('handleIconify', { iconified });
  },
  handleFramebufferSize(width, height) {
    //console.log('handleFramebufferSize', { width, height });
  },
  handleMouseButton(button, action, mods) {
    console.log('handleMouseButton', { button, action, mods });
  },
  handleCursorPos(x, y) {
    console.log('handleCursorPos', { x, y });
  },
  handleCursorEnter(cur) {
    console.log('handleCursorEnter', { cur });
  },
  handleScroll(xoffset, yoffset) {
    console.log('handleScroll', { xoffset, yoffset });
  },
  handleKey(key, scancode, action, mods) {
    console.log('handleKey', { key, scancode, action, mods });
  },
  handleChar(c) {
    console.log('handleChar', { c });
  },
  handleCharMods(c, mods) {
    console.log('handleCharMods', { c, mods });
  },
  handleDrop(argc, argv) {
    console.log('handleDrop', { argc, argv });
  }
};

Object.defineProperties(GLFW.prototype, {
  size: {
    get() {
      return this.window.size;
    }
  },
  position: {
    get() {
      return this.window.position;
    }
  }
});

Object.assign(GLFW.prototype, {
  poll,
  move(...args) {
    const { window } = this;
    window.position = new Position(...args);
  },
  resize(...args) {
    const { window } = this;
    window.size = new Size(...args);
  },
  setTitle(title) {
    const { window } = this;
    window.title = this.title = title;
  },
  begin(clearColor = new RGBA(0, 0, 0, 255)) {
    const { resolution } = this;
    glViewport(0, 0, resolution.width, resolution.height);

    if(clearColor instanceof RGBA) clearColor = clearColor.normalize();

    glClearColor(...clearColor);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
  },
  end() {
    const { window } = this;
    window.swapBuffers();
    poll();
  }
});

export function Mat2Image(mat) {
  return nvg.CreateImageRGBA(mat.cols, mat.rows, 0, mat.buffer);
}

export function DrawImage(image, pos) {
  const size = nvg.ImageSize(image);
  nvg.Save();
  if(pos) nvg.Translate(...pos);
  nvg.BeginPath();
  nvg.Rect(0, 0, ...size);
  nvg.FillPaint(nvg.ImagePattern(0, 0, ...size, 0, image, 1));
  nvg.Fill();
  nvg.Restore();
}

export function DrawCircle(pos, radius) {
  nvg.Save();
  nvg.Translate(...pos);
  nvg.BeginPath();
  nvg.StrokeColor(nvg.RGB(255, 255, 255));
  nvg.StrokeWidth(5);
  nvg.FillColor(nvg.RGBA(255, 0, 0, 96));
  nvg.Circle(0, 0, radius);
  nvg.Fill();
  nvg.Stroke();
  nvg.Restore();
}
