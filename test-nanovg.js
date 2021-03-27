import * as glfw from 'glfw.so';
import Util from './lib/util.js';
import ConsoleSetup from './lib/consoleSetup.js';
import { glFlush, glBegin, glBindTexture, glClear, glClearColor, glEnable, glEnd, glGenTextures, glTexCoord2f, glTexParameterf, glTexImage2D, glVertex3f, glViewport, GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT, GL_STENCIL_BUFFER_BIT, GL_LINEAR, GL_QUADS, GL_REPEAT, GL_RGB, GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_TEXTURE_MIN_FILTER, GL_TEXTURE_WRAP_S, GL_TEXTURE_WRAP_T, GL_UNSIGNED_BYTE, glDisable, glLoadIdentity, glMatrixMode, glOrtho, glPushMatrix, glPopMatrix, GL_LIGHTING, GL_MODELVIEW, GL_PROJECTION } from './gl.js';
import { RGBA, HSLA } from './lib/color.js';
import { Mat } from 'mat.so';
import * as cv from 'cv.so';
import * as nvg from 'nanovg.so';

async function main(...args) {
  await ConsoleSetup({
    maxStringLength: 200,
    maxArrayLength: 10,
    breakLength: 100,
    compact: 0
  });
  //console.log('glfw:', glfw);

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

  let image = cv.imread('796e7bfab61dd66b5f12c3f929f75d9c_Mhleberg_5.jpg');
  let image2 = cv.imread('9b16290d7d9c8f1aca810b6702070189_20170331_112428.jpg');

  cv.cvtColor(image2, image2, cv.CV_RGB2RGBA);
  console.log('image:', image);
  console.log('image.buffer:', image.buffer);
  let imgId = nvg.CreateImage('796e7bfab61dd66b5f12c3f929f75d9c_Mhleberg_5.jpg', 0);
  console.log('imgId:', imgId);
  let img2Id = nvg.CreateImageRGBA(image2.cols, image2.rows, 0, image2.buffer);
  console.log('img2Id:', img2Id);
  let img2Sz = nvg.ImageSize(img2Id);
  let imgSz = nvg.ImageSize(imgId);
  console.log('nvg.ImageSize():', imgSz);
  console.log('nvg.ImageSize():', img2Sz);
  let i = 0;

  while(true) {
    if(window.shouldClose) {
      console.log('window.shouldClose:', window.shouldClose);
      break;
    }
    glViewport(0, 0, width, height);

    let time = +new Date() / 1000;
    let index = Math.floor((time * 360) / 30);
    let color = new HSLA(index % 360, 100, 50 + 25 * Math.sin(time * 0.1 * Math.PI)).toRGBA();
    //console.log("color", ...color.normalize());

    glClearColor(...color.normalize());
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT); //clears the window to the color you want.

    nvg.BeginFrame(width, height, 1);

    let m = nvg.CurrentTransform();
    let t = nvg.TransformTranslate([], 10, 20);
    let s = nvg.TransformScale([], 3, 3);

    let p = nvg.TransformMultiply(nvg.TransformMultiply(m, t), s);

    /*   console.log('t:', t);
    console.log('p:', p);
    console.log('nvg.TransformPoint:', nvg.TransformPoint(p, 40, 100));*/
    let pattern = nvg.ImagePattern(0, 0, ...img2Sz, 0, img2Id, 1);
    // console.log('pattern:', pattern);

    nvg.Scale(0.4, 0.4);

    /* let  sx = Mat.cos((i % 360) * Math.PI /360);
    let  sy = Mat.sin((i % 360) * Math.PI /360);

    nvg.Translate(50+sx*50,50+sy*50);*/
    let phi = ((i % 360) / 180) * Math.PI;
    let vec = [Math.cos(phi), Math.sin(phi)].map(n => n * 100);

    vec = vec.map(n => n + 150);
    //    nvg.Translate((i % 50) * 4, 10);
    nvg.Translate(...vec);

    nvg.BeginPath();
    nvg.Rect(0, 0, ...imgSz);
    nvg.StrokeColor(nvg.RGB(255, 128, 0));
    nvg.StrokeWidth(4);
    nvg.Stroke();
    nvg.FillPaint(pattern);
    // nvg.FillColor(nvg.RGB(0, 0, 0));
    nvg.Fill();

    nvg.EndFrame();

    window.swapBuffers();
    glfw.poll();
    i++;
  }
}
Util.callMain(main, true);