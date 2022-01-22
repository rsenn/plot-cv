import { AxisPoints, DrawAxis, Origin, DrawCross, GetRect, Max, X, Y, Flip } from './diagram.js';
import { Console } from 'console';
import { Point, Size, Rect, Line, TickMeter, Mat, Draw, imwrite, CV_8UC3, CV_8UC1, CV_16UC1, CV_32FC1, CV_64FC1, FILLED, CV_RGB, LineIterator } from 'opencv';
import { TextStyle, DrawText } from './qjs-opencv/js/cvHighGUI.js';

function main(...args) {
  globalThis.console = new Console({
    inspectOptions: { maxArrayLength: 100, colors: true, depth: Infinity, compact: 0, customInspect: true }
  });

  let fontFile = 'qjs-opencv/MiscFixedSC613.ttf',
    fontSize = 12;

  let font = new TextStyle(fontFile, fontSize);

  let size = font.size('X');

  let mat = new Mat([640, 480], CV_8UC3);
  let area = new Rect(0, 0, ...mat.size).inset(20);
  let diagramMat = mat(area);

  console.log('area', area);

  Draw.rectangle(diagramMat, new Point(0, 0), new Point(...diagramMat.size), [255, 255, 255], FILLED);

  let axes = {
    x: AxisPoints(100, 10, X, font, diagramMat.size),
    y: AxisPoints(100, 10, Y, font, diagramMat.size)
  };

  let rect = GetRect(diagramMat, axes.x, axes.y, font);
  //console.log('rect', rect);

  let origin = Origin(diagramMat, axes.x, axes.y);
  // DrawCross(diagramMat, origin);

  const { tl, br } = Flip(diagramMat, rect);
  //console.log('rect', { tl, br });

  tl.x += 1;
  tl.y -= 1;

  //Draw.rectangle(diagramMat, tl, br, CV_RGB(255, 0, 0), FILLED, 8);

  DrawAxis(diagramMat, axes.x, rect, font);
  DrawAxis(diagramMat, axes.y, rect, font);

  console.log('origin', origin);

  let mat8 = new Mat(mat.size, CV_8UC1);
  let mat16 = new Mat(mat.size, CV_16UC1);
  let mat32 = new Mat(mat.size, CV_32FC1);
  let mat64 = new Mat(mat.size, CV_64FC1);

  function MakeLineIterator(mat) {
    let li = new LineIterator(mat, new Point(0, 0), new Point(mat.cols - 1, mat.rows - 1));
    console.log('LineIterator.err', li.err);
    const { ptr, ptr0 } = li;

    li.postIncr();
    //  li.preIncr();
    /*  li.preIncr();
  li.preIncr();
  console.log('li.postIncr()', li.postIncr());
*/
    li.postIncr();
    li.postIncr();
    li.postIncr();
    const { count, elemSize, err, minusDelta, minusShift, minusStep, p, plusDelta, plusShift, plusStep, ptmode, step } = li;

    console.log('LineIterator', { count, elemSize, err, minusDelta, minusShift, minusStep, p, plusDelta, plusShift, plusStep, ptmode, ptr, ptr0, step });
    console.log('li.ptr - li.ptr0', li.ptr - li.ptr0);
    console.log('mat.ptr(0)', mat.ptr(0));
  }

  MakeLineIterator(mat8);
  MakeLineIterator(mat16);
  MakeLineIterator(mat32);
  MakeLineIterator(mat64);

  imwrite('diagram.png', mat);
}

main(...scriptArgs);
