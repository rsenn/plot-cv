import { GL_COLOR_BUFFER_BIT } from './gl.js';
import { GL_LIGHTING } from './gl.js';
import { GL_LINEAR } from './gl.js';
import { GL_MODELVIEW } from './gl.js';
import { GL_PROJECTION } from './gl.js';
import { GL_QUADS } from './gl.js';
import { GL_REPEAT } from './gl.js';
import { GL_RGB } from './gl.js';
import { GL_TEXTURE_2D } from './gl.js';
import { GL_TEXTURE_MAG_FILTER } from './gl.js';
import { GL_TEXTURE_MIN_FILTER } from './gl.js';
import { GL_TEXTURE_WRAP_S } from './gl.js';
import { GL_TEXTURE_WRAP_T } from './gl.js';
import { GL_UNSIGNED_BYTE } from './gl.js';
import { glBegin } from './gl.js';
import { glBindTexture } from './gl.js';
import { glClear } from './gl.js';
import { glClearColor } from './gl.js';
import { glDisable } from './gl.js';
import { glEnable } from './gl.js';
import { glEnd } from './gl.js';
import { glFlush } from './gl.js';
import { glGenTextures } from './gl.js';
import { glLoadIdentity } from './gl.js';
import { glMatrixMode } from './gl.js';
import { glOrtho } from './gl.js';
import { glPopMatrix } from './gl.js';
import { glPushMatrix } from './gl.js';
import { glTexCoord2f } from './gl.js';
import { glTexImage2D } from './gl.js';
import { glTexParameterf } from './gl.js';
import { glVertex3f } from './gl.js';
import { glViewport } from './gl.js';
import { HSLA } from './lib/color.js';
import { range } from './lib/misc.js';
import Console from 'console';
import { context } from 'glfw';
import { CONTEXT_VERSION_MAJOR } from 'glfw';
import { CONTEXT_VERSION_MINOR } from 'glfw';
import { OPENGL_CORE_PROFILE } from 'glfw';
import { OPENGL_FORWARD_COMPAT } from 'glfw';
import { OPENGL_PROFILE } from 'glfw';
import { poll } from 'glfw';
import { RESIZABLE } from 'glfw';
import { SAMPLES } from 'glfw';
import { Window } from 'glfw';
import { imread } from 'opencv';
function Mat2Texture(texture_cv) {
  console.log('texture_cv', texture_cv);
  const { buffer } = texture_cv;
  console.log('texture_cv.buffer', buffer);
  let texture = new Uint32Array(1);
  console.log('texture', texture);
  glGenTextures(1, texture.buffer); // Create The Texture

  glBindTexture(GL_TEXTURE_2D, texture[0]);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

  glTexImage2D(GL_TEXTURE_2D, 0, 3, texture_cv.cols, texture_cv.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, buffer);
  return texture[0];
}

function main(...args) {
  globalThis.console = new Console({
    inspectOptions: { colors: true, depth: 1, maxArrayLength: Infinity, maxStringLength: 100 }
  });
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
  let textures = [];

  console.log(`width: ${width}, height: ${height}, x: ${x}, y: ${y}`);

  if(args.length == 0) args.push('Muehleberg.png');

  while(args.length > 0) {
    console.log('args[0]:', args[0]);

    let image = imread(args[0]);
    console.log('image:', image);
    console.log('image.buffer:', image.buffer);
    let texture = Mat2Texture(image);

    args.shift();
    textures.push(texture);
  }
  shuffle(textures);
  console.log('textures', textures);

  let hues = range(0, 359, 360 / 16)
    .map(h => new HSLA(h, 100, 50))
    .map(hsla => hsla.toRGBA());

  const clamp = (n, min, max) => Math.min(Math.max(min, n), max);
  const interpolate = (x, y, sigma) => (Array.isArray(x) ? x.map((xx, i) => interpolate(xx, y[i], sigma)) : x * (1.0 - sigma) + y * sigma);

  console.log(
    'hues',
    hues.map(c => [...c].slice(0, 3))
  );

  while(!window.shouldClose) {
    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0.0, width, 0.0, height, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    glLoadIdentity();
    glDisable(GL_LIGHTING);

    let time = +new Date() / 1000;
    let index = Math.floor((time * 360) / 300);

    let sine = Math.sin(time * 2 * Math.PI);

    let color = [...hues[index % 16]]; //.normalize();

    color = interpolate(color, sine >= 0 ? [255, 255, 255, 255] : [0, 0, 0, 255], Math.abs(sine) * 0.3).map(Math.round);

    glClearColor(...color.map(n => n / 255));
    glClear(GL_COLOR_BUFFER_BIT); //clears the window to the color you want.

    glEnable(GL_TEXTURE_2D);
    // console.log('textures[0]', textures[0]);
    glBindTexture(GL_TEXTURE_2D, textures[0]);

    // Draw a textured quad
    glBegin(GL_QUADS);
    glTexCoord2f(0, 0);
    glVertex3f(0, 0, 0);
    glTexCoord2f(0, 1);
    glVertex3f(0, 100, 0);
    glTexCoord2f(1, 1);
    glVertex3f(100, 100, 0);
    glTexCoord2f(1, 0);
    glVertex3f(100, 0, 0);
    glEnd();

    glDisable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, 0);

    glPopMatrix();

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

    glMatrixMode(GL_MODELVIEW);

    glFlush();

    window.swapBuffers();
    poll();
  }
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error.message}\n${error.stack}`);
  std.exit(1);
} finally {
  console.log('SUCCESS');
}