import * as glfw from 'glfw.so';
import { glClear, glClearColor, glViewport, GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT, GL_STENCIL_BUFFER_BIT } from './gl.js';
import { HSLA } from './lib/color.js';
import { Mat, Point } from 'opencv.so';
import * as cv from 'opencv.so';
import * as nvg from 'nanovg.so';
import Console from 'console';
import { GLFW, Mat2Image, DrawImage, DrawCircle, Position } from './draw-utils.js';

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

  let i = 0;
  let running = true;

  let context;
  const { position, size, window } = (context = new GLFW(1280, 900, {
    title: scriptArgs[0],
    resizable: true,
    handleSize(width, height) {
      console.log('resized', { width, height });
    },
    handleKey(keyCode) {
      let charCode = keyCode & 0xff;
      console.log(`handleKey`, { keyCode: '0x' + keyCode.toString(16), charCode, char: String.fromCharCode(charCode) });
      let char = String.fromCodePoint(charCode);

      let handler = { '\x00': () => (running = false), Q: () => (running = false) }[char];
      if(handler) handler();
    },
    handleCharMods(char, mods) {
      console.log(`handleCharMods`, { char, mods });
    },
    handleMouseButton(button, action, mods) {
      console.log(`handleMouseButton`, { button, action, mods });
    },
    handleCursorPos(x, y) {
      //console.log(`handleCursorPos`, { x, y });
    }
  }));

  nvg.CreateGL3(nvg.STENCIL_STROKES | nvg.ANTIALIAS | nvg.DEBUG);

  const { width, height } = size;
  const { x, y } = position;

  //console.log(`width: ${width}, height: ${height}, x: ${x}, y: ${y}`);

  let mat = new Mat(size, cv.CV_8UC4);

  mat.setTo([11, 22, 33, 255]);

  cv.drawLine(mat, new Point(10, 10), new Point(size.width - 10, size.height - 10), [255, 255, 0, 255], 4, cv.LINE_AA);
  cv.drawLine(mat, new Point(size.width - 10, 10), new Point(10, size.height - 10), [255, 0, 0, 255], 4, cv.LINE_AA);

  let image2 = cv.imread('Architektur.png');

  let { buffer } = mat;

  let pixels;
  let imgId = Mat2Image(mat);
  let img2Id = nvg.CreateImage('Muehleberg.png', 0);

  console.log(``, { imgId, img2Id });

  let img2Sz = nvg.ImageSize(img2Id);
  let imgSz = nvg.ImageSize(imgId);

  while((running &&= !window.shouldClose)) {
    let time = +new Date() / 1000;
    let index = Math.floor((time * 360) / 30);
    let color = new HSLA(index % 360, 100, 50 + 25 * Math.sin(time * 0.1 * Math.PI)).toRGBA();

    context.begin(color);

    nvg.BeginFrame(width, height, 1);

    let m = nvg.CurrentTransform();
    let t = nvg.TransformTranslate([], 10, 20);
    let s = nvg.TransformScale([], 3, 3);

    let p = nvg.TransformMultiply(nvg.TransformMultiply(m, t), s);

    // let pattern = nvg.ImagePattern(0, 0, ...img2Sz, 0, img2Id, 1);

    let center = new Position(size.width / 2, size.height / 2);
    let imgSz_2 = new Position(img2Sz.width * -0.5, img2Sz.height * -0.5);

    nvg.Save();

    nvg.Translate(...center);
    nvg.Scale(0.5, 0.5);
    nvg.Translate(...imgSz_2);

    let phi = ((i % 360) / 180) * Math.PI;
    let vec = [Math.cos(phi), Math.sin(phi)].map(n => n * 100);

    DrawImage(img2Id, vec);
    nvg.Translate(imgSz_2.width * -1, imgSz_2.height * -1);
    DrawCircle(new Position(0, 0), 40);

    nvg.Restore();

    DrawCircle(center, 100);

    nvg.EndFrame();

    context.end();
    /*window.swapBuffers();
    glfw.poll();*/
    i++;
  }
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(error ? `FAIL: ${error.message}\n${error.stack}` : `FAIL: ${error}`);
  std.exit(1);
}
