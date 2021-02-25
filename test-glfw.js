import { poll, context, CONTEXT_VERSION_MAJOR, CONTEXT_VERSION_MINOR, OPENGL_PROFILE, OPENGL_CORE_PROFILE, OPENGL_FORWARD_COMPAT, RESIZABLE, SAMPLES, Window, Monitor } from 'glfw.so';
import Util from './lib/util.js';

import { dlopen, define, dlsym, RTLD_NOW, call } from 'ffi.so';

function library(file) {
  let handle = dlopen(file, RTLD_NOW);

  return function bind(name, ret, ...args) {
    let fp = dlsym(handle, name);
    define(name, fp, null, ret, ...args);
    globalThis[name] = (...args) => call(name, ...args);
    return globalThis[name];
  };
}

let gl = library('libGL.so');

gl('glClear', 'void', 'unsigned int');
gl('glClearColor', 'void', 'float', 'float', 'float', 'float');
gl('glViewport', 'void', 'int', 'int', 'int', 'int');

const GL_COLOR_BUFFER_BIT = 0x00004000;

async function main(...args) {
  Window.hint(CONTEXT_VERSION_MAJOR, 3);
  Window.hint(CONTEXT_VERSION_MINOR, 2);
  Window.hint(OPENGL_PROFILE, OPENGL_CORE_PROFILE);
  Window.hint(OPENGL_FORWARD_COMPAT, true);
  Window.hint(RESIZABLE, false);
  Window.hint(SAMPLES, 4);

  const window = new Window(800, 600, 'OpenGL');
  context.current = window;

  const { position, size } = window;
  const { width, height } = size;
  const { x, y } = position;

  console.log(`width: ${width}, height: ${height}, x: ${x}, y: ${y}`);

  while(!window.shouldClose) {
    glViewport(0, 0, width, height); //set the viewport of where we want to draw
    glClearColor(0, Math.random(), Math.random(), Math.random()); //clear color (R,G,B,A) for example blue
    glClear(GL_COLOR_BUFFER_BIT); //clears the window to the color you want.

    window.swapBuffers();
    poll();
  }
}
Util.callMain(main, true);
