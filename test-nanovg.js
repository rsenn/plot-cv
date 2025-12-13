import { DrawCircle, DrawImage, GLFW, Mat2Image } from './draw-utils.js';
import { HSLA } from './lib/color.js';
import Console from 'console';
import { className } from 'util';
import * as glfw from 'glfw';
import * as ImGui from 'imgui';
import * as nanovg from 'nanovg';
import { imread, copyTo, log, CV_8UC4, COLOR_BGR2BGRA, LINE_AA, cvtColor, drawLine, Mat, Point, Rect } from 'opencv';

function Clear(color = nanovg.RGB(0, 0, 0)) {
  const { size } = glfw.context.current;

  nanovg.Save();
  nanovg.BeginPath();
  nanovg.Rect(0, 0, ...size);
  nanovg.FillColor(color);
  nanovg.Fill();
  nanovg.Restore();
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
      [glfw.CONTEXT_VERSION_MAJOR, 3],
      [glfw.CONTEXT_VERSION_MINOR, 2],
      [glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE],
      [glfw.OPENGL_FORWARD_COMPAT, true],
      [glfw.RESIZABLE, true],
      [glfw.SAMPLES, 4]
    ])
      glfw.Window.hint(prop, value);

    window = glfw.context.current = new glfw.Window(1280, 900, 'ImGui test');

    context = {
      begin() {},
      end() {
        window.swapBuffers();
        glfw.poll();
      }
    };

    window.handleCharMods = (char, mods) => {
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

  nanovg.CreateGL3(nanovg.STENCIL_STROKES | nanovg.ANTIALIAS | nanovg.DEBUG);

  ImGui.Init(ImGui.ImplGlfw, ImGui.ImplOpenGL3);
  ImGui.CreateContext(window);

  const { width, height } = size;
  const { x, y } = position;

  //console.log(`width: ${width}, height: ${height}, x: ${x}, y: ${y}`);

  let mat = new Mat(height, width, CV_8UC4);

  mat.setTo([11, 22, 33, 255]);

  let image2 = imread('Architektur.png');

  cvtColor(image2, image2, COLOR_BGR2BGRA);

  image2.copyTo(mat(new Rect(image2.size)));

  drawLine(mat, new Point(10, 10), new Point(size.width - 10, size.height - 10), [255, 255, 0, 255], 4, LINE_AA);
  drawLine(mat, new Point(size.width - 10, 10), new Point(10, size.height - 10), [255, 0, 0, 255], 4, LINE_AA);

  let { buffer } = mat;

  let pixels;
  let imgId = Mat2Image(mat);
  let img2Id = nanovg.CreateImage('Muehleberg.png', 0);

  console.log(``, { imgId, img2Id });

  let img2Sz = nanovg.ImageSize(img2Id);
  let imgSz = nanovg.ImageSize(imgId);

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

  let floatValue = 0.0,
    alphaValue = 127;

  while((running &&= !window.shouldClose)) {
    let index = Math.floor(timer.ticks(360) / 30);

    let color = new HSLA(index % 360, 100, 50 + 25 * Math.sin(timer.ticks(0.1) * Math.PI)).toRGBA();

    context.begin(color);

    nanovg.BeginFrame(width, height, 1);

    Clear(nanovg.RGB(...color));

    let m;
    nanovg.CurrentTransform((m = []));

    let p, a;
    p = nanovg.TransformIdentity();

    //   nanovg.Scale(2,1);

    nanovg.TransformMultiply(
      p,
      nanovg.TransformTranslate(0, 100),
      nanovg.TransformRotate(((timer.ticks(200) % 360) * Math.PI) / 180),
      nanovg.TransformRotate(nanovg.DegToRad(45)),
      nanovg.TransformScale(2, 0.5),
      nanovg.TransformRotate(nanovg.DegToRad(-45))
    );
    //console.log('Transform', p);

    nanovg.TransformPoint((a = []), p, 0, 0);

    // let pattern = nanovg.ImagePattern(0, 0, ...img2Sz, 0, img2Id, 1);

    let center = new glfw.Position(size.width / 2, size.height / 2);
    let imgSz_2 = new glfw.Position(img2Sz.width * -0.5, img2Sz.height * -0.5);

    nanovg.Save();

    nanovg.Translate(...center);
    nanovg.Scale(0.5, 0.5);
    nanovg.Translate(...imgSz_2);

    let phi = ((timer.ticks(60) % 360) / 180) * Math.PI;
    let vec = [Math.cos(phi), Math.sin(phi)].map(n => n * 100);

    DrawImage(imgId, vec);
    //  nanovg.Translate(imgSz_2.width * -1, imgSz_2.height * -1);
    nanovg.Restore();
    nanovg.Save();

    nanovg.Translate(size.width / 4, size.height / 4);
    /*   nanovg.Scale(2,1);
    nanovg.Rotate(((timer.ticks(180) % 360) * Math.PI) / 180);
  nanovg.Scale(0.5,1);
 */
    DrawCircle(a, 40, 2, [0, 0, 0], [255, 255, 0, 192]);

    nanovg.Restore();

    DrawCircle(center, 100);

    nanovg.EndFrame();

    ImGui.NewFrame();

    ImGui.Begin('Parameters', null, ImGui.WindowFlags.MenuBar);

    ImGui.SetWindowSize([400, 300]);
    ImGui.PushItemWidth(ImGui.GetFontSize() * -12);

    ImGui.Text('Adjust values for this processing step:');

    /* ImGui.DragFloat('Value', val => (val === undefined ? floatValue : (floatValue = val)), 0.0, 1.0, '%.3f');
    ImGui.SliderFloat('Value', val => (val === undefined ? floatValue : (floatValue = val)), 0.0, 1.0, '%.3f');
    ImGui.SliderInt('Alpha', val => (val === undefined ? Math.floor(alphaValue) : (alphaValue = Math.floor(val))), 0, 255);
*/
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
  console.log(`Exception (${className(error)}): `, error);
  console.log('FAIL: ' + error.message + '\n' + error.stack);
  //os.kill(process.pid, os.SIGUSR1);
  std.exit(1);
}