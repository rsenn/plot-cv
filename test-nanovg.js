import * as glfw from 'glfw.so';
import Util from './lib/util.js';
import ConsoleSetup from './lib/consoleSetup.js';
import { glFlush, glBegin, glBindTexture, glClear, glClearColor, glEnable, glEnd, glGenTextures, glTexCoord2f, glTexParameterf, glTexImage2D, glVertex3f, glViewport, GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT, GL_STENCIL_BUFFER_BIT, GL_LINEAR, GL_QUADS, GL_REPEAT, GL_RGB, GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_TEXTURE_MIN_FILTER, GL_TEXTURE_WRAP_S, GL_TEXTURE_WRAP_T, GL_UNSIGNED_BYTE, glDisable, glLoadIdentity, glMatrixMode, glOrtho, glPushMatrix, glPopMatrix, GL_LIGHTING, GL_MODELVIEW, GL_PROJECTION } from './gl.js';
import { RGBA, HSLA } from './lib/color.js';
import { Mat } from 'mat.so';
import * as cv from 'cv.so';
import * as nvg from 'nanovg.so';

function Mat2Texture(texture_cv) {
  let texture = new Uint32Array(1);
  console.log('Mat2Texture', { texture, texture_cv });
  glGenTextures(1, texture.buffer); // Create The Texture
  // console.log('Mat2Texture texture_cv.buffer', texture_cv.buffer);

  glBindTexture(GL_TEXTURE_2D, texture[0]);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

  console.log('Mat2Texture texture', [...texture]);
  /// console.log('Mat2Texture texture_cv.buffer', texture_cv.buffer);
  console.log('Mat2Texture texture_cv.size', texture_cv.size);

  glTexImage2D(GL_TEXTURE_2D,
    0,
    3,
    texture_cv.cols,
    texture_cv.rows,
    0,
    GL_RGB,
    GL_UNSIGNED_BYTE,
    texture_cv.buffer
  );
  console.log('Mat2Texture texture[0]', texture[0]);

  return texture[0];
}

async function main(...args) {
  await ConsoleSetup({
    maxStringLength: 200,
    maxArrayLength: 10,
    breakLength: 100,
    compact: 0
  });
  console.log('glfw:', glfw);

  glfw.Window.hint(glfw.CONTEXT_VERSION_MAJOR, 3);
  glfw.Window.hint(glfw.CONTEXT_VERSION_MINOR, 2);
  glfw.Window.hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE);
  glfw.Window.hint(glfw.OPENGL_FORWARD_COMPAT, true);
  glfw.Window.hint(glfw.RESIZABLE, false);
  glfw.Window.hint(glfw.SAMPLES, 4);

  const window = new glfw.Window(800, 600, 'OpenGL');
  glfw.context.current = window;

  const { position, size } = window;
  const { width, height } = size;
  const { x, y } = position;

  console.log(`width: ${width}, height: ${height}, x: ${x}, y: ${y}`);

  nvg.CreateGL3(nvg.STENCIL_STROKES | nvg.ANTIALIAS | nvg.DEBUG);
  /*
  const vg = new nvg.Paint();

  console.log("vg:", vg);
*/

  let image = cv.imread('9b16290d7d9c8f1aca810b6702070189_20170331_112428.jpg');
  let image2 = cv.imread('796e7bfab61dd66b5f12c3f929f75d9c_Mhleberg_5.jpg');
  console.log('image:', image);
  let texture = Mat2Texture(image.clone());
  let texture2 = Mat2Texture(image2);
  console.log('texture:', texture);
  console.log('texture2:', texture2);
  console.log('image.buffer:', image.buffer);

  while(true) {
    if(window.shouldClose) {
      console.log('window.shouldClose:', window.shouldClose);
      break;
    }
    glViewport(0, 0, width, height);

    /*    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0.0, width, 0.0, height, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    glLoadIdentity();
    glDisable(GL_LIGHTING);
*/
    let time = +new Date() / 1000;
    let index = Math.floor((time * 360) / 30);
    let color = new HSLA(index % 360, 100, 50 + 25 * Math.sin(time * 2 * Math.PI)).toRGBA();
    //console.log("color", ...color.normalize());

    glClearColor(...color.normalize());
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT); //clears the window to the color you want.

    nvg.BeginFrame(width, height, 1);

    nvg.BeginPath();
    nvg.Rect(10, 10, 200, 200);
    nvg.StrokeColor(nvg.RGB(255, 128, 0));
    nvg.StrokeWidth(4);
    nvg.Stroke();
    nvg.FillColor(nvg.RGB(0, 0, 0));
    nvg.Fill();

    nvg.EndFrame();

    window.swapBuffers();
    glfw.poll();
  }
}
Util.callMain(main, true);
