import * as glfw from 'glfw';
import Util from './lib/util.js';
import { glFlush, glBegin, glBindTexture, glClear, glClearColor, glEnable, glEnd, glGenTextures, glTexCoord2f, glTexParameterf, glTexImage2D, glVertex3f, glViewport, GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT, GL_STENCIL_BUFFER_BIT, GL_LINEAR, GL_QUADS, GL_REPEAT, GL_RGB, GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_TEXTURE_MIN_FILTER, GL_TEXTURE_WRAP_S, GL_TEXTURE_WRAP_T, GL_UNSIGNED_BYTE, glDisable, glLoadIdentity, glMatrixMode, glOrtho, glPushMatrix, glPopMatrix, GL_LIGHTING, GL_MODELVIEW, GL_PROJECTION } from './gl.js';
import { RGBA, HSLA } from './lib/color.js';
import { Mat, Size, Point, Rect } from 'opencv';
import * as cv from 'opencv';
import * as nvg from 'nanovg';
import Console from 'console';

function GLFW(...args) {
  let resolution = new Size(...args);
  const { Window } = glfw;
  const hints = [
    [glfw.CONTEXT_VERSION_MAJOR, 3],
    [glfw.CONTEXT_VERSION_MINOR, 2],
    [glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE],
    [glfw.OPENGL_FORWARD_COMPAT, true],
    [glfw.RESIZABLE, false],
    [glfw.SAMPLES, 4]
  ];

  for(let [prop, value] of hints) Window.hint(prop, value);

  let window = (glfw.context.current = new Window(resolution.width, resolution.height, 'OpenGL'));
  let size = new Size(window.size);
  let position = new Point(window.position);
  nvg.CreateGL3(nvg.STENCIL_STROKES | nvg.ANTIALIAS | nvg.DEBUG);
  return Object.assign(this, { resolution, window, size, position });
}

function main(...args) {
  globalThis.console = new Console({
    inspectOptions: {
      maxStringLength: 200,
      maxArrayLength: 10,
      breakLength: 100,
      compact: 1,
      depth: 10
    }
  });

  const { position, size, window } = new GLFW(1280, 900);
  const { width, height } = size;
  const { x, y } = position;

  console.log(`width: ${width}, height: ${height}, x: ${x}, y: ${y}`);

  let mat = new Mat(size, cv.CV_8UC4);

  mat.setTo([11, 22, 33, 255]);
  //cv.rectangle(mat, new Point(0,0), new Point(800,600), [255,0,0,0],4, cv.LINE_8);
  cv.line(mat,
    new Point(10, 10),
    new Point(size.width - 10, size.height - 10),
    [255, 255, 0, 255],
    4,
    cv.LINE_AA
  );
  cv.line(mat,
    new Point(size.width - 10, 10),
    new Point(10, size.height - 10),
    [255, 0, 0, 255],
    4,
    cv.LINE_AA
  );

  console.log('mat:', mat);
  console.log('mat.size:', mat.size);

  let image2 = cv.imread('Architektur.png');

  //image2.copyTo(mat);

  console.log('mat:', mat);
  console.log('mat.elemSize', mat.elemSize);
  let { buffer } = mat;
  console.log('mat.buffer:', buffer);
  let pixels;
  console.log('mat.buffer.byteLength/mat.elemSize:', (pixels = buffer.byteLength / mat.elemSize));
  console.log('pixels / mat.cols', pixels / mat.cols);

  let imgId = nvg.CreateImageRGBA(mat.cols, mat.rows, 0, mat.buffer);
  let img2Id = nvg.CreateImage('Muehleberg.png', 0);
  console.log('imgId:', imgId);
  console.log('img2Id:', img2Id);
  let img2Sz = nvg.ImageSize(img2Id);
  let imgSz = nvg.ImageSize(imgId);
  console.log('nvg.ImageSize(img2Id)', img2Sz + '');
  console.log('nvg.ImageSize(imgId)', imgSz + '');
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

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error.message}\n${error.stack}`);
  std.exit(1);
} finally {
  console.log('SUCCESS');
}
