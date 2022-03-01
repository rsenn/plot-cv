import * as glfw from 'glfw';
import { glClear, glClearColor, glViewport, GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT, GL_STENCIL_BUFFER_BIT } from './gl.js';
import { RGBA, HSLA } from './lib/color.js';
import { Mat, Size, Point, Rect } from 'opencv';
import * as cv from 'opencv';
import * as nvg from 'nanovg';
import Console from 'console';

function GLFW(...args) {
  let resolution = new glfw.Size(...args);
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
  let { size, position } = window;
  nvg.CreateGL3(nvg.STENCIL_STROKES | nvg.ANTIALIAS | nvg.DEBUG);
  return Object.assign(this, { resolution, window, size, position });
}

function DrawImage(image, pos) {
  const size = nvg.ImageSize(image);
  nvg.Save();
  if(pos) nvg.Translate(...pos);
  nvg.BeginPath();
  nvg.Rect(0, 0, ...size);
  nvg.FillPaint(nvg.ImagePattern(0, 0, ...size, 0, image, 1));
  nvg.Fill();
  nvg.Restore();
}

function DrawCircle(pos, radius) {
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

function main(...args) {
  globalThis.console = new Console({
    inspectOptions: {
      maxStringLength: 200,
      maxArrayLength: 10,
      breakLength: Infinity,
      compact: 2,
      depth: 10
    }
  });

  const { position, size, window } = new GLFW(1280, 900);
  const { width, height } = size;
  const { x, y } = position;

  Object.assign(window, {
    handleKey(charCode) {
      let char = String.fromCodePoint(charCode);
      //console.log(`handleKey`, { charCode,char });
    },
    handleCharMods(char, mods) {
      console.log(`handleCharMods`, { char, mods });
    },
    handleMouseButton(button, action, mods) {
      //console.log(`handleMouseButton`, { button, action, mods });
    },
    handleCursorPos(x, y) {
      //console.log(`handleCursorPos`, { x, y });
    }
  });

  console.log(`width: ${width}, height: ${height}, x: ${x}, y: ${y}`);

  let mat = new Mat(size, cv.CV_8UC4);

  mat.setTo([11, 22, 33, 255]);
  //cv.rectangle(mat, new glfw.Position(0,0), new glfw.Position(800,600), [255,0,0,0],4, cv.LINE_8);
  cv.line(mat, new Point(10, 10), new Point(size.width - 10, size.height - 10), [255, 255, 0, 255], 4, cv.LINE_AA);
  cv.line(mat, new Point(size.width - 10, 10), new Point(10, size.height - 10), [255, 0, 0, 255], 4, cv.LINE_AA);

  console.log('glfw.Position', glfw.Position);
  console.log('mat:', mat);
  console.log('mat.size:', mat.size);

  let image2 = cv.imread('Architektur.png');

  //image2.copyTo(mat);

  console.log('mat:', mat);
  console.log('mat.elemSize', mat.elemSize);
  let { buffer } = mat;
  //console.log('mat.buffer:', buffer);
  let pixels;
  console.log('mat.buffer.byteLength/mat.elemSize:', (pixels = buffer.byteLength / mat.elemSize));
  console.log('pixels / mat.cols', pixels / mat.cols);

  let imgId = nvg.CreateImageRGBA(mat.cols, mat.rows, 0, mat.buffer);
  let img2Id = nvg.CreateImage('Muehleberg.png', 0);
  console.log('imgId:', imgId);
  console.log('img2Id:', img2Id);
  let img2Sz = new glfw.Size(nvg.ImageSize(img2Id));
  let imgSz = new glfw.Size(nvg.ImageSize(imgId));
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

    let pattern = nvg.ImagePattern(0, 0, ...img2Sz, 0, img2Id, 1);

    let center = new glfw.Position(size.width / 2, size.height / 2);
    let imgSz_2 = new glfw.Position(img2Sz.width * -0.5, img2Sz.height * -0.5);
    /*console.log("size",size);
  console.log("center",center);
  console.log("img2Sz",img2Sz);
  console.log("img2Sz.mul(-1)",img2Sz.mul(-1));
  console.log("img2Sz.div(2)",img2Sz.div(2));
   console.log("imgSz_2",imgSz_2);*/

    nvg.Save();

    //
    nvg.Translate(...center);
    nvg.Scale(0.5, 0.5);
    nvg.Translate(...imgSz_2);

    let phi = ((i % 360) / 180) * Math.PI;
    let vec = [Math.cos(phi), Math.sin(phi)].map(n => n * 100);

    DrawImage(img2Id, vec);
    nvg.Translate(imgSz_2.width * -1, imgSz_2.height * -1);
    DrawCircle(new glfw.Position(0, 0), 40);

    nvg.Restore();

    DrawCircle(center, 100);

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
