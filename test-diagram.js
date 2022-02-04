import { AxisPoints, AxisRange, DrawAxis, Origin, GetRect, X, Y, Flip, DrawRect, ClientRect, ClientArea, ClientMatrix } from './diagram.js';
import { Console } from 'console';
import { Point, Rect, Mat, Draw, Contour, transform, CV_8UC3, CV_64FC1, CV_64FC2, FILLED, LINE_AA, LINE_8, imshow, imwrite, waitKey } from 'opencv';
import { TextStyle } from './qjs-opencv/js/cvHighGUI.js';
import { range, srand, randInt } from 'util';

function main(...args) {
  globalThis.console = new Console({
    inspectOptions: { maxArrayLength: 100, colors: true, depth: Infinity, compact: 1, customInspect: true }
  });

  srand(Date.now());

  let fontFile = 'qjs-opencv/MiscFixedSC613.ttf',
    fontSize = 12;

  let font = new TextStyle(fontFile, fontSize);

  let size = font.size('X');

  let mat = new Mat([640, 480], CV_8UC3);
  let diagramRect = new Rect(0, 0, ...mat.size).inset(20);
  let diagramMat = mat(diagramRect);

  console.log('diagramRect', diagramRect);

  Draw.rectangle(diagramMat, new Point(0, 0), new Point(...diagramMat.size), [255, 255, 255], FILLED);

  let axes = {
    x: AxisPoints(100, 10, X, font, diagramMat.size),
    y: AxisPoints(100, 10, Y, font, diagramMat.size)
  };
  let ranges = {
    x: AxisRange(axes.x),
    y: AxisRange(axes.y)
  };
  console.log('', { ranges });

  let rect = GetRect(diagramMat, axes.x, axes.y, font);
  let origin = Origin(diagramMat, axes.x, axes.y);

  console.log('', { rect, origin });
  let area = ClientArea(diagramMat, axes.x, axes.y, font);

  //Draw.rectangle(area, new Point(0, 0), new Point(area.cols - 1, area.rows - 1), [255, 0, 255], 1, LINE_8);

  let contour = new Contour(...range(0, 100, 10).map(x => new Point(x, randInt(100))));
  console.log('contour', console.config({ compact: false }), contour);

  let matrix = ClientMatrix(diagramMat, axes.x, axes.y, font);

  let contour2 = new Mat(1, contour.length, CV_64FC2);

  transform(contour.getMat(), contour2, matrix);

  let c = new Contour(...[...contour2].map(a => new Point(...a)));
  console.log('c', console.config({ compact: false }), c);

  console.log('Draw.polylines', Draw.polylines);

  Draw.polylines(area, [c?? [...contour2]], false, [1, 220, 90], 1, LINE_AA);

  //  Draw.line(area, [0,0],[100,100], [0,255,0], 1, LINE_8);

  DrawAxis(diagramMat, axes.x, rect, font);
  DrawAxis(diagramMat, axes.y, rect, font);

  imwrite('diagram.png', mat);

  imshow(args[0], mat);
  waitKey(-1);
}

main(...scriptArgs);
