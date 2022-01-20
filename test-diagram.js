import { AxisPoints, DrawAxis, Origin, DrawCross, GetRect, Max, X, Y, Flip } from './diagram.js';
import { Console } from 'console';
import { Point, Size, Rect, Line, TickMeter, Mat, Draw, imwrite, CV_8UC3, FILLED, CV_RGB } from 'opencv';
import { TextStyle, DrawText } from './qjs-opencv/js/cvHighGUI.js';

function main(...args) {
  globalThis.console = new Console({
    inspectOptions: { maxArrayLength: 100, colors: true, depth: Infinity, compact: 0, customInspect: true }
  });

  let fontFile = 'qjs-opencv/MiscFixedSC613.ttf',
    fontSize = 12;

  let font = new TextStyle(fontFile, fontSize);

  let size = font.size('X');

  let mat = new Mat([1024, 800], CV_8UC3);
  let area = new Rect(0, 0, ...mat.size).inset(20);
  let diagramMat = mat(area);

  console.log('area', area);

  Draw.rectangle(diagramMat, new Point(0, 0), new Point(...diagramMat.size), [255, 255, 255], FILLED);

  let axes = {
    x: AxisPoints(100, 20, X, font, diagramMat.size),
    y: AxisPoints(100, 20, Y, font, diagramMat.size)
  };

  let rect = GetRect(diagramMat, axes.x, axes.y, font);
  console.log('rect', rect);

  let origin = Origin(diagramMat, axes.x, axes.y);
  // DrawCross(diagramMat, origin);

  const { tl, br } = Flip(diagramMat, rect);
  console.log('rect', { tl, br });

  tl.x += 1;
  tl.y -= 1;

  //Draw.rectangle(diagramMat, tl, br, CV_RGB(255, 0, 0), FILLED, 8);

  DrawAxis(diagramMat, axes.x, rect, font);
  DrawAxis(diagramMat, axes.y, rect, font);

  console.log('origin', origin);

  imwrite('diagram.png', mat);
}

main(...scriptArgs);
