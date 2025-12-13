import { HSLA } from './lib/color/hsla.js';
import { RGBA } from './lib/color/rgba.js';
import { Line } from './lib/geom/line.js';
import { Matrix } from './lib/geom/matrix.js';
import { PointList } from './lib/geom/pointList.js';
import * as cv from 'opencv';

const lib = { Point, Size, Line, Rect, PointList, RGBA, HSLA, Matrix };

Point.prototype.atan2 = function() {
  return Math.atan2(this.x, this.y);
};
Object.defineProperty(Point.prototype, 'distance', {
  get() {
    return Math.sqrt(this.x * this.x + this.y * this.y);
  },
  enumerable: true
});

async function main(...args) {
  function testPointVector() {
    let pv = new Contour();
    let poly = new Contour();
    let hull = new Contour();
    let s = new Size(320, 200);
    let mat = new Mat(s, Mat.CV_32FC2);
    let mat2 = new Mat(200, 320, Mat.CV_32FC2);
    //console.log('s =', s);
    //console.log('mat.rows =', mat.rows);
    //console.log('mat.cols =', mat.cols);
    //console.log('mat.type =', mat.type);
    //console.log('mat.channels =', mat.channels);
    //console.log('mat2.rows =', mat2.rows);
    //console.log('mat2.cols =', mat2.cols);
    pv.push(0, 0);
    pv.push({ x: 10, y: 0 });
    pv.push({ x: 10, y: 20 });
    pv.push({ x: 0, y: 20 });
    pv.push({ x: 0, y: 0 });

    pv.approxPolyDP(poly, 2.0, true);
    poly.convexHull(hull, true, true);

    let circle = pv.minEnclosingCircle();
    let triangle = pv.minEnclosingTriangle();
    //console.log("circle.center: ", circle.center);
    //console.log('circle.radius: ', circle.radius);
    //console.log('triangle: ', triangle);
    //console.log('Mat.CV_8UC4 ', Mat.CV_8UC4);
    //console.log('Mat.CV_32FC1 ', Mat.CV_32FC1);
    /*  let a = pv.get(1);
  let b = pv.get(2);
*/
    //console.log("a.cross(b): ", a.cross(b));
    //console.log("a.dot(b): ", a.dot(b));
    //console.log("a.atan2(): ", a.atan2());
    //console.log("a.length(): ", a.distance);

    //console.log('pv.pointPolygonTest: ', pv.pointPolygonTest(new Point(10, 10)));
    //console.log('pv.pointPolygonTest: ', pv.pointPolygonTest(new Point(200, 200)));

    //console.log('pv.pointPolygonTest: ', pv.pointPolygonTest(new Point(10, 10), true));
    //console.log('pv.pointPolygonTest: ', pv.pointPolygonTest(new Point(200, 200), true));

    //console.log('poly.length: ', poly.length);
    //console.log('hull.length: ', hull.length);
    //console.log("poly.boundingRect(): ", poly.boundingRect());
    //console.log("pv.minAreaRect(): ", pv.minAreaRect());
    //console.log("pv.fitEllipse(): ", pv.fitEllipse());
    //console.log("pv.fitLine(): ", pv.fitLine());
    //console.log("hull.boundingRect(): ", hull.boundingRect());
    //console.log("pv.length: ", pv.length);
    //console.log("pv.get(0): ", pv.get(0));
    //console.log("pv.get(1): ", pv.get(1));
    //console.log("pv.area: ", pv.area);

    /*
  let it = pv[Symbol.iterator]();
    //console.log("it: ", it);

  let arr = [...it];
  //console.log("arr: ", arr);
*/
  }

  function detectEdges(src, dst, thres1 = 10, thres2 = 20) {
    cv.Canny(src, dst, thres1, thres2);

    // cv.imwrite('canny.png', dst);

    cv.imshow('canny', dst);

    console.log('src:', src + '');

    //  detectLines(dst);
  }

  let matTypes = [
    'CV_8U',
    'CV_8S',
    'CV_16U',
    'CV_16S',
    'CV_32S',
    'CV_32F',
    'CV_64F',
    'CV_8UC1',
    'CV_8UC2',
    'CV_8UC3',
    'CV_8UC4',
    'CV_8SC1',
    'CV_8SC2',
    'CV_8SC3',
    'CV_8SC4',
    'CV_16UC1',
    'CV_16UC2',
    'CV_16UC3',
    'CV_16UC4',
    'CV_16SC1',
    'CV_16SC2',
    'CV_16SC3',
    'CV_16SC4',
    'CV_32SC1',
    'CV_32SC2',
    'CV_32SC3',
    'CV_32SC4',
    'CV_32FC1',
    'CV_32FC2',
    'CV_32FC3',
    'CV_32FC4',
    'CV_64FC1',
    'CV_64FC2',
    'CV_64FC3',
    'CV_64FC4'
  ];
  let matTypeEntries = matTypes.map(n => [n, cv[n]]);
  console.log(new Map(matTypeEntries.map(([n, v]) => [n, ('0000000' + v.toString(16)).slice(-8)])));

  console.log(
    'mask: ',
    matTypeEntries
      .filter(([n, v]) => !/C[0-9]$/.test(n))
      .reduce((acc, [n, v]) => acc | v, 0)
      .toString(2)
  );

  let input = cv.imread(args[0] ?? '3daaffa22db97a9394054e9e9bdb6837_20170930_120917.jpg');
  let output = new Mat();

  detectEdges(input, output);

  const { HIER_PREVIOUS, HIER_NEXT, HIER_PARENT, HIER_FIRSTCHILD } = cv;

  {
    let contours = [],
      hier;

    cv.findContours(output, contours, h => (hier = h));

    console.log('cv.findContours', { contours, hier });

    let obj = {
      contours,
      hier,
      index(i) {
        return i === -1 ? null : this.contours[i];
      },
      previous(i) {
        return this.index(this.hier[i][HIER_PREVIOUS]);
      },
      next(i) {
        return this.index(this.hier[i][HIER_NEXT]);
      },
      parent(i) {
        return this.index(this.hier[i][HIER_PARENT]);
      },
      firstChild(i) {
        return this.index(this.hier[i][HIER_FIRSTCHILD]);
      }
    };

    console.log('contours.length=', contours.length);
    console.log('hier.length=', hier.length);
    //   console.log('hier=', hier);
    console.log('obj.prev=', obj.previous(1));
    console.log('obj.next=', obj.next(1));
    console.log('obj.parent=', obj.parent(1));
    console.log('obj.firstChild=', obj.firstChild(1));
  }

  return;
  globalThis.test_array = [1, 2, 3, 4, 5, 6];
  globalThis.process = function(contours, hier) {
    let areas = [];

    let outlines = {
      contours,
      hier
    };
    contours = contours.filter(c => c.length > 3);

    let c = contours[0];

    let orig = imgOriginal;

    /*  let hull = c.convexHull(false, false);
  let defects = null; //c.convexityDefects(hull);

  //console.log(typeof c);
  //console.log("contour: ", c);
  c.rotatePoints(2);
  //console.log("contour: ", c);
  //console.log("convexHull: ", hull);
  //console.log("convexityDefects: ", defects);
  //console.log("simplifyNthPoint: ", c.simplifyNthPoint(2));
  //console.log("simplifyRadialDistance: ", c.simplifyRadialDistance(10));
  //console.log("simplifyPerpendicularDistance: ", c.simplifyPerpendicularDistance(20));
  //console.log("simplifyReumannWitkam: ", c.simplifyReumannWitkam());
  //console.log("simplifyOpheim: ", c.simplifyOpheim());
  //console.log("simplifyLang: ", c.simplifyLang());
  //console.log("simplifyDouglasPeucker: ", c.simplifyDouglasPeucker());
  */
    //console.log("center: ", c.center);

    /*  let a = c.toArray();
  //console.log("toArray: ", a);
  //console.log("imgRaw: ", imgRaw);
  */

    /*
  //console.log("orig.rows =", orig.rows);
  //console.log("orig.cols =", orig.cols);*/

    /*  console.log("contours: ", globalThis.contours[globalThis.contours.length - 1]);
  //console.log("hier: ", globalThis.hier[globalThis.contours.length - 1]);*/

    function dumpContour(c) {
      //console.log(`contour #${c.id} length=${(c.length + '').padStart(5, ' ')} bbox=`, c.bbox, ' rect:', c.rect, ' area=', c.area);
    }

    function processContours(contours) {
      contours.sort((a, b) => a.length - b.length);
      contours = contours.filter(c => c.length >= 4);
      for(let i = 0; i < contours.length; i++) {
        const [next, prev, child, parent] = hier[i];
        let list = new PointList(contours[i]);
        let bbox = list.bbox();
        let rect = new Rect(bbox);
        contours[i].area = rect.area;
        contours[i].id = i;
        contours[i].bbox = bbox;
        contours[i].rect = rect;
        areas.push(rect.area);
        list = list.map(p => {
          p.x += 2;
          p.y += 2;
          return p;
        });
        drawContour(list, new RGBA(255, 0, 255), 8, false);
      }
      contours.sort((a, b) => b.area - a.area);
      areas = contours.map(c => c.area);
      dumpContour(contours[0]);
      drawContour(contours[0], new RGBA(255, 0, 0), 20, false);
      return contours;
    }

    let polygons = [
      new PointList([
        { x: 0, y: 0 },
        { x: 320, y: 0 },
        { x: 320, y: 240 },
        { x: 0, y: 240 },
        { x: 0, y: 0 }
      ])
    ];

    polygons.push(polygons[0].sum({ x: 320, y: 0 }));
    polygons.push(polygons[0].sum({ x: 320, y: 240 }));
    polygons.push(polygons[0].sum({ x: 0, y: 240 }));

    /*
  drawPolygon(polygons[0], new RGBA(255, 255, 0), 3);
  drawPolygon(polygons[1], new RGBA(0, 255, 0), 3);
  drawPolygon(polygons[2], new RGBA(0, 0, 255), 3);
  drawPolygon(polygons[3], new RGBA(255, 0, 255), 3);
*/
    /*  drawCircle(new Point(300, 150), 110, new RGBA(1, 1, 1), -1);
  drawCircle(new Point(300, 150), 100, new RGBA(255, 255, 0), -1);
*/
    const do_log = false;

    if(do_log) {
      //console.log(`polygons: [\n  ${polygons.join(',\n  ')}\n]`);
      //console.log('PROCESS contours: ', contours.map(c => '[' + c.map(pt => `{x:${pt.x},y:${pt.y}}`).join(', ') + ']').join(', '));
      //console.log('PROCESS hier: ', '[' + hier.map(h => `[${h.join(',')}]`).join(', '));
    }
  };
  let ctor = Point.prototype.constructor;
  //console.log('Classes: ', inspect(lib));
  //console.log('Point: ', inspect(Point));
  //console.log('Contour: ' + inspect(Contour));
  //console.log('typeof(Point.prototype.constructor): ', typeof Point.prototype.constructor == 'function');
  //console.log('typeof(Point): ', typeof Point);
  //console.log('ctor.name: ', ctor.name);

  /*console.log("Point.prototype: ", Point.prototype);
//console.log("Point.prototype.constructor: ", Point.prototype.constructor);
*/
  let points = [new Point(0, 0), new Point(50, 0), new Point(100, 0), new Point(100, 50), new Point(100, 100), new Point(100, 200)];
  //console.log('points[0]: ', points[0]);
  //console.log('points[last]: ', points[points.length - 1]);
  //console.log('points: ', points.map(p => `{x:${p.x},y:${p.y}}`).join(', '));
}

main(...scriptArgs.slice(1));