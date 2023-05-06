import * as glfw from 'glfw';
import { context, poll, Position, Window, CONTEXT_VERSION_MAJOR, CONTEXT_VERSION_MINOR, OPENGL_PROFILE, OPENGL_CORE_PROFILE, OPENGL_FORWARD_COMPAT, RESIZABLE, SAMPLES } from 'glfw';
import { glClear, glClearColor, glViewport, GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT, GL_STENCIL_BUFFER_BIT } from './gl.js';
import { HSLA } from './lib/color.js';
import { Mat, Point, Size, Rect,imread } from 'opencv';
import * as cv from 'opencv';
import * as nvg from 'nanovg';
import Console from 'console';
import { GLFW, Mat2Image, DrawImage, DrawCircle } from './draw-utils.js';
import * as ImGui from 'imgui';

function Clear(color = nvg.RGB(0, 0, 0)) {
  const { size } = context.current;

  nvg.Save();
  nvg.BeginPath();
  nvg.Rect(0, 0, ...size);
  nvg.FillColor(color);
  nvg.Fill();
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

  let i = 0;
  let running = true;

  let context, window, position, size;

  if(true) {
    for(let [prop, value] of [
      [CONTEXT_VERSION_MAJOR, 3],
      [CONTEXT_VERSION_MINOR, 2],
      [OPENGL_PROFILE, OPENGL_CORE_PROFILE],
      [OPENGL_FORWARD_COMPAT, true],
      [RESIZABLE, true],
      [SAMPLES, 4]
    ])
      Window.hint(prop, value);

    window = glfw.context.current = new Window(1280, 900, 'ImGui test');

    context = {
      begin() {},
      end() {
        window.swapBuffers();
        poll();
      }
    };

     window.handleCharMods  = (char, mods) => {
        console.log(`handleCharMods`, { char, mods });
      };

  } else {
    context = new GLFW(1280, 900, {
      title: scriptArgs[0],
      resizable: true,
      handleSize(width, height) {
        console.log('resized', { width, height });
      },
      handleKey(keyCode) {
        let charCode = keyCode & 0xff;
        console.log(`handleKey`, {
          keyCode: '0x' + keyCode.toString(16),
          charCode,
          char: String.fromCharCode(charCode)
        });
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
    });
    window = context.window;
  }

  position = window.position;
  size = window.size;

  nvg.CreateGL3(nvg.STENCIL_STROKES | nvg.ANTIALIAS | nvg.DEBUG);

  ImGui.Init(ImGui.ImplGlfw, ImGui.ImplOpenGL3);
  ImGui.CreateContext(window);

  const { width, height } = size;
  const { x, y } = position;

  //console.log(`width: ${width}, height: ${height}, x: ${x}, y: ${y}`);

  let mat = new Mat(new Size(width, height), cv.CV_8UC4);


  mat.setTo([11, 22, 33, 255]);


  let image2 = cv.imread('Architektur.png');

cv.cvtColor(image2, image2, cv.COLOR_BGR2BGRA);
 
  image2.copyTo(mat(new Rect(image2.size)));


  cv.drawLine(mat, new Point(10, 10), new Point(size.width - 10, size.height - 10), [255, 255, 0, 255], 4, cv.LINE_AA);
  cv.drawLine(mat, new Point(size.width - 10, 10), new Point(10, size.height - 10), [255, 0, 0, 255], 4, cv.LINE_AA);

  let { buffer } = mat;

  let pixels;
  let imgId = Mat2Image(mat);
  let img2Id = nvg.CreateImage('Muehleberg.png', 0);

  console.log(``, { imgId, img2Id });

  let img2Sz = nvg.ImageSize(img2Id);
  let imgSz = nvg.ImageSize(imgId);

  const timer = {
    ticks(rate = 1000) {
      const t = Date.now();

      return ((t - (this.start ??= t)) * rate) / 1000;
    },
    frame() {
      const t = Date.now();
      const { prev = this.start } = this;
      this.prev = t;
      return (this.fps = +(1000 / (t - prev)).toFixed(1));
    }
  };

  console.log('FPS:');

  let floatValue=0.0,alphaValue=127;


  while((running &&= !window.shouldClose)) {
    let index = Math.floor(timer.ticks(360) / 30);

    let color = new HSLA(index % 360, 100, 50 + 25 * Math.sin(timer.ticks(0.1) * Math.PI)).toRGBA();

    context.begin(color);

    nvg.BeginFrame(width, height, 1);

    Clear(nvg.RGB(...color));

    let m;
    nvg.CurrentTransform((m = []));

    let p, a;
 p=   nvg.TransformIdentity();

    //   nvg.Scale(2,1);

    nvg.TransformMultiply(
      p,
      nvg.TransformTranslate(0, 100),
      nvg.TransformRotate(((timer.ticks(200) % 360) * Math.PI) / 180),
      nvg.TransformRotate(nvg.DegToRad(45)),
      nvg.TransformScale(2, 0.5),
      nvg.TransformRotate(nvg.DegToRad(-45)),
    );
    //console.log('Transform', p);

    nvg.TransformPoint((a = []), p, 0, 0);

    // let pattern = nvg.ImagePattern(0, 0, ...img2Sz, 0, img2Id, 1);

    let center = new Position(size.width / 2, size.height / 2);
    let imgSz_2 = new Position(img2Sz.width * -0.5, img2Sz.height * -0.5);

    nvg.Save();

    nvg.Translate(...center);
    nvg.Scale(0.5, 0.5);
    nvg.Translate(...imgSz_2);

    let phi = ((timer.ticks(60) % 360) / 180) * Math.PI;
    let vec = [Math.cos(phi), Math.sin(phi)].map(n => n * 100);

    DrawImage(imgId, vec);
    //  nvg.Translate(imgSz_2.width * -1, imgSz_2.height * -1);
    nvg.Restore();
    nvg.Save();

    nvg.Translate(size.width / 4, size.height / 4);
    /*   nvg.Scale(2,1);
    nvg.Rotate(((timer.ticks(180) % 360) * Math.PI) / 180);
  nvg.Scale(0.5,1);
 */
    DrawCircle(a, 40, 2, [0, 0, 0], [255, 255, 0, 192]);

    nvg.Restore();

    DrawCircle(center, 100);

    nvg.EndFrame();

    ImGui.NewFrame();

    ImGui.Begin('Parameters', null, ImGui.WindowFlags.MenuBar);

    ImGui.SetWindowSize([400, 300]);
    ImGui.PushItemWidth(ImGui.GetFontSize() * -12);

    ImGui.Text('Adjust values for this processing step:');

    ImGui.DragFloat('Value', val => (val === undefined ? floatValue : (floatValue = val)), 0.0, 1.0, "%.3f");
    ImGui.SliderFloat('Value', val => (val === undefined ? floatValue : (floatValue = val)), 0.0, 1.0, "%.3f");
    ImGui.SliderInt('Alpha', val => (val === undefined ? Math.floor(alphaValue) : (alphaValue = Math.floor(val))),0, 255);


    ImGui.End();

    ImGui.Render();

    let data = ImGui.GetDrawData();

    ImGui.RenderDrawData(data);

    /* let { id,Valid, CmdListsCount, TotalIdxCount, TotalVtxCount, DisplayPos, DisplaySize, FramebufferScale } = data;
    console.log('data', id, {Valid, CmdListsCount, TotalIdxCount, TotalVtxCount, DisplayPos, DisplaySize, FramebufferScale });
*/
    context.end();
    /*window.swapBuffers();
    glfw.poll();*/
    i++;

    timer.frame();
  }
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(error ? `FAIL: ${error.message}\n${error.stack}` : `FAIL: ${error}`);
  std.exit(1);
}
