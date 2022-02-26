import { poll, context, CONTEXT_VERSION_MAJOR, CONTEXT_VERSION_MINOR, OPENGL_PROFILE, OPENGL_CORE_PROFILE, OPENGL_FORWARD_COMPAT, RESIZABLE, SAMPLES, Window } from 'glfw';
import Util from './lib/util.js';
import Console from 'console';
import { glFlush, glBegin, glBindTexture, glClear, glClearColor, glEnable, glEnd, glGenTextures, glTexCoord2f, glTexParameterf, glTexImage2D, glVertex3f, glViewport, GL_COLOR_BUFFER_BIT, GL_LINEAR, GL_QUADS, GL_REPEAT, GL_RGB, GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_TEXTURE_MIN_FILTER, GL_TEXTURE_WRAP_S, GL_TEXTURE_WRAP_T, GL_UNSIGNED_BYTE, glDisable, glLoadIdentity, glMatrixMode, glOrtho, glPushMatrix, glPopMatrix, GL_LIGHTING, GL_MODELVIEW, GL_PROJECTION } from './gl.js';
import { HSLA } from './lib/color.js';
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
    inspectOptions: { colors: true, depth: 1, maxArrayLength: 30, maxStringLength: 100 }
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

  if(args.length==0)
    args.push('Muehleberg.png');

  while(args.length > 0) {
    console.log('args[0]:', args[0]);

    let image = imread(args[0]);
    console.log('image:', image);
    console.log('image.buffer:', image.buffer);
    let texture = Mat2Texture(image);

    args.shift();
    textures.push(texture);
  }
  Util.shuffle(textures);
  console.log('textures', textures);

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
    let index = Math.floor((time * 360) / 30);
    let color = new HSLA(index % 360, 100, 50 + 25 * Math.sin(time * 2 * Math.PI)).toRGBA();
    //console.log("color", ...color.normalize());

    glClearColor(...color.normalize());
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
