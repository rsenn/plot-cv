//import { Size } from './size.js';
//import { PointList } from './pointList.js';
//import Point from './rect.js';
/**
 * Class for rectangle.
 *
 * @class      Rect (name)
 */
function Rect(arg) {
  let obj = this instanceof Rect ? this : {}; // this === Rect || !this ? {} : this;
  let args = arg instanceof Array ? arg : [...arguments];
  let ret;
  if(typeof args[0] == 'number') arg = args;
  else if(args[0].length !== undefined) arg = args.shift();
  ['x', 'y', 'width', 'height'].forEach(field => {
    if(typeof obj[field] != 'number') obj[field] = 0;
  });
  if(arg && arg.x1 !== undefined && arg.y1 !== undefined && arg.x2 !== undefined && arg.y2 !== undefined) {
    const { x1, y1, x2, y2 } = arg;
    obj.x = x1;
    obj.y = y1;
    obj.width = x2 - x1;
    obj.height = y2 - y1;
    ret = 1;
  } else if(arg && arg.x !== undefined && arg.y !== undefined && arg.x2 !== undefined && arg.y2 !== undefined) {
    const { x, y, x2, y2 } = arg;
    obj.x = x;
    obj.y = y;
    obj.width = x2 - x;
    obj.height = y2 - y;
    ret = 1;
  } else if(Point.isPoint(arg) && arg.y !== undefined && arg.width !== undefined && arg.height !== undefined) {
    obj.x = parseFloat(arg.x);
    obj.y = parseFloat(arg.y);
    obj.width = parseFloat(arg.width);
    obj.height = parseFloat(arg.height);
    ret = 1;
  } else if(arg && arg.length >= 4 && arg.slice(0, 4).every(arg => !isNaN(parseFloat(arg)))) {
    let x = arg.shift();
    let y = arg.shift();
    let w = arg.shift();
    let h = arg.shift();
    obj.x = typeof x === 'number' ? x : parseFloat(x);
    obj.y = typeof y === 'number' ? y : parseFloat(y);
    obj.width = typeof w === 'number' ? w : parseFloat(w);
    obj.height = typeof h === 'number' ? h : parseFloat(h);
    ret = 4;
  } else if(arg && arg.length >= 2 && arg.slice(0, 2).every(arg => !isNaN(parseFloat(arg)))) {
    obj.width = typeof x === 'number' ? x : parseFloat(x);
    obj.height = typeof y === 'number' ? y : parseFloat(y);
    ret = 2;
  } else if(arg instanceof Array) {
    let argc;
    let argi = 0;
    if(arg.length >= 4) {
      argc = typeof x == 'number' ? 2 : 1;
      Point.apply(obj, arg.slice(0, argc));
      argi = argc;
    }
    argc = typeof arg[argi] == 'number' ? 2 : 1;
    Size.apply(obj, arg.slice(argi, argc));
    ret = argi + argc;
  }
  if(obj.round === undefined) {
    Object.defineProperty(obj, 'round', {
      value: function() {
        return Rect.round(this);
      },
      enumerable: true,
      writable: false
    });
  }
  //  if(!isRect(obj)) throw new Error('ERROR: is not a rect: '  + Util.inspect(obj));
  if(!(this instanceof Rect)) {
    /*    if(this === Rect && obj)*/ return obj;
    return ret;
  }
}
Rect.aspect = Size.aspect;
//export const isRect = rect => isPoint(rect) && isSize(rect);
Rect.isRect = rect => isPoint(rect) && isSize(rect);
Rect.fromPoints = (p1, p2) => {
  return new Rect(Math.min(p1.x, p2.x), Math.min(p1.y, p2.y), Math.abs(p1.x - p2.x), Math.abs(p1.y - p2.y));
};
Rect.clone = function clone(arg) {
  const { x, y, width, height } = this === Rect ? arg : this;
  return this === Rect ? { x, y, width, height } : new Rect(x, y, width, height);
};
Rect.prototype.clone = function() {
  return new Rect(this.x, this.y, this.width, this.height);
};
Rect.corners = (rect = this) => [
  { x: rect.x, y: rect.y }, // top left
  { x: rect.x + rect.width, y: rect.y }, // top right
  { x: rect.x + rect.width, y: rect.y + rect.height }, // bottom right
  { x: rect.x, y: rect.y + rect.height } // bottom left
];
Rect.centered = (point, rx, ry = rx) => new Rect(point.x - rx * 0.5, point.y - ry * 0.5, rx * 2, ry * 2);
Rect.bbrect = () => {
  const ex = Rect.extrema(arguments);
  return {
    x: ex.x[0],
    y: ex.y[0],
    w: ex.x[1] - ex.x[0],
    h: ex.y[1] - ex.y[0]
  };
};
Rect.extrema = () => {
  let args = Array.from(arguments);
  const f = args.shift();
  return args.reduce(
    function(ex, pt) {
      return {
        x: [Math.min(ex.x[0], pt.x), Math.max(ex.x[1], pt.x + (pt.width || 0))],
        y: [Math.min(ex.y[0], pt.y), Math.max(ex.y[1], pt.y + (pt.height || 0))]
      };
    },
    { x: [f.x, f.x], y: [f.y, f.y] }
  );
};
/*
Rect.move = (rect, point) => {
  let args = [...arguments];
  args.shift();
  let to = Point.call(Point, args);
  rect.x += to.x;
  rect.y += to.y;
  return rect;
};
Rect.move_to = (rect, point) => {
  let args = [...arguments];
  args.shift();
  let to = Point.call(Point, args);
  rect.x = to.x;
  rect.y = to.y;
  return rect;
};*/
Rect.resize = () => {
  let args = Array.from(arguments);
  let rect = this == Rect ? args.shift() : this;
  Size.apply(rect, args);
  return rect;
};
Object.assign(Rect.prototype, Point.prototype);
Object.assign(Rect.prototype, Size.prototype);
if (Rect.prototype.isSquare === undefined) {
  Rect.prototype.isSquare = function() {
    return Math.abs(this.width - this.height) < 1;
  };
}
Rect.prototype.constructor = Rect;
Rect.prototype.area = function() {
  return this.width * this.height;
};
Rect.prototype.toString = function() {
  return this.x + ',' + this.y + ' ' + this.width + 'x' + this.height;
};
Rect.toString = rect => rect.x + ',' + rect.y + ' ' + rect.width + 'x' + rect.height;
Rect.toSource = (r = this) => '{x:' + r.x + ',y:' + r.y + ',width:' + r.width + ',height:' + r.height + '}';
Rect.prototype.toSource = function() {
  return 'new Rect(' + (this ? this.x + ',' + this.y + ',' + this.width + ',' + this.height : '') + ')';
};
Rect.equal = (a, b) => a.x == b.x && a.y == b.y && a.width == b.width && a.height == b.height;
Rect.indexOf = (r, a, skip = () => false) =>
  a.reduce((accu, item, i) => {
    return accu == -1 && !skip(item) && Rect.equal(r, item) ? i : accu;
  }, -1);
Rect.lastIndexOf = (r, a, skip = () => false) =>
  a.reduceRight((accu, item, i) => {
    return accu == -1 && !skip(item) && Rect.equal(r, item) ? i : accu;
  }, -1);
Rect.onlyUnique = (item, i, arr, keep = () => true) => Rect.indexOf(item, arr, elem => !keep(elem)) == i;
Rect.uniq = (arr, keep = elem => false) => arr.filter((elem, i, arr) => keep(elem) || Rect.onlyUnique(elem, i, arr));
Rect.x2 = rect => rect.x + rect.width;
Rect.y2 = rect => rect.y + rect.height;
Rect.round = r =>
  new Rect({
    x: Math.round(r.x),
    y: Math.round(r.y),
    x2: Math.round(Rect.x2(r)),
    y2: Math.round(Rect.y2(r))
  });
Rect.prototype.round = function(precision = 1.0) {
  let { x, y, x2, y2 } = this;
  this.x = +x.toFixed(precision);
  this.y = +y.toFixed(precision);
  this.width = +x2.toFixed(precision) - this.x;
  this.height = +y2.toFixed(precision) - this.y;
  return this;
};
Rect.union = (a, b) => {
  let ret = {
    x: a.x < b.x ? a.x : b.x,
    y: a.y < b.y ? a.y : b.y
  };
  ret.width = Math.max(Rect.x2(a), Rect.x2(b)) - ret.x;
  ret.height = Math.max(Rect.y2(a), Rect.y2(b)) - ret.y;
  return Rect.round(ret);
};
Object.defineProperty(
  Rect.prototype,
  'x1', {
  get: function() {
    return this.x;
  },
  set: function(value) {
    const extend = this.x - value;
    this.width += extend;
    this.x -= extend;
  },
  enumerable: true}
);
Object.defineProperty(Rect.prototype, 'x2', {
  get: function() {
    return this.x + this.width;
  },
  set: function(value) {
    this.width = value - this.x;
  },
  enumerable: true
});
Object.defineProperty(Rect.prototype, 'y1', {
  get: function() {
    return this.y;
  },
  set: function(value) {
    const extend = this.y - value;
    this.height += extend;
    this.y -= extend;
  }
});
Object.defineProperty(Rect.prototype, 'y2', {
  get: function() {
    return this.y + this.height;
  },
  set: function(value) {
    this.height = value - this.y;
  }
});
Rect.area = r => r.width * r.height;
Object.defineProperty(Rect.prototype, 'area', {
  get: function() {
    return Rect.area(this);
  }
});
Rect.center = r => new Point(r.x + r.width / 2, r.y + r.height / 2);
//Rect.prototype.center = function() { return new Point(this.x + this.width / 2, this.y + this.height / 2); }
Object.defineProperty(Rect.prototype, 'center', {
  get: function() {
    return Rect.center(this);
  }
});
Rect.path = (r, clockwise = true) => ({
  d: clockwise ? 'M' + r.x + ' ' + r.y + ' h' + r.width + ' v' + r.height + ' h-' + r.width + 'z' : 'M' + r.x + ' ' + r.y + ' v' + r.height + ' h' + r.width + ' v-' + r.height + 'z'
});
Object.assign(Rect.prototype, {
  get points() {
    const c = Rect.corners(this);
    return new PointList(c);
  }
});
/*
get x2() {
  return this.x + this.width;
}
get y2() {
  return this.y + this.height;
}
get round() {
  const points = this.points;
  return new Rect(points[0].round, points[1].round);
}
*/
Rect.toCSS = function() {
  const rect = Rect.apply(Rect, arguments);
  return { ...Point.toCSS(rect), ...Size.toCSS(rect) };
};
Rect.prototype.toCSS = Rect.toCSS;
Rect.set = function(rect, element) {
  let e = typeof element === 'string' ? Element.find(element) : element;
  Element.move(e, rect);
  Element.resize(e, rect);
  return rect;
};
Rect.outset = function(rect, trbl) {
  if(typeof trbl == 'number') trbl = new TRBL(trbl, trbl, trbl, trbl);
  rect.x -= trbl.left;
  rect.y -= trbl.top;
  rect.width += trbl.left + trbl.right;
  rect.height += trbl.top + trbl.bottom;
  return rect;
};
Rect.prototype.outset = function(trbl) {
  if(typeof trbl == 'number') trbl = new TRBL(trbl, trbl, trbl, trbl);
  this.x -= trbl.left;
  this.y -= trbl.top;
  this.width += trbl.left + trbl.right;
  this.height += trbl.top + trbl.bottom;
  return this;
};
Rect.inset = function(rect, trbl) {
  if(typeof trbl == 'number') trbl = new TRBL(trbl, trbl, trbl, trbl);
  rect.x += trbl.left;
  rect.y += trbl.top;
  rect.width -= trbl.left + trbl.right;
  rect.height -= trbl.top + trbl.bottom;
  return rect;
};
Rect.prototype.inset = function(trbl) {
  if(typeof trbl == 'number') trbl = new TRBL(trbl, trbl, trbl, trbl);
  if(trbl.left + trbl.right < this.width && trbl.top + trbl.bottom < this.height) {
    this.x += trbl.left;
    this.y += trbl.top;
    this.width -= trbl.left + trbl.right;
    this.height -= trbl.top + trbl.bottom;
  }
  return this;
};
Rect.align = function(rect, align_to, a = 0) {
  const xdiff = align_to.width - rect.width;
  const ydiff = align_to.height - rect.height;
  let oldx = rect.x;
  let oldy = rect.y;
  switch (Align.horizontal(a)) {
    case Align.LEFT:
      rect.x = align_to.x;
      break;
    case Align.RIGHT:
      rect.x = align_to.x + xdiff;
      break;
    default:
      rect.x = align_to.x + xdiff / 2;
      break;
  }
  switch (Align.vertical(a)) {
    case Align.TOP:
      rect.y = align_to.y;
      break;
    case Align.BOTTOM:
      rect.y = align_to.y + ydiff;
      break;
    default:
      rect.y = align_to.y + ydiff / 2;
      break;
  }
  rect.tx = rect.x - oldx;
  rect.ty = rect.y - oldy;
  return rect;
};
Rect.inside = (rect, point) => point.x >= rect.x && point.x <= rect.x + rect.width && point.y >= rect.y && point.y <= rect.y + rect.height;
Rect.prototype.inside = function(point) {
  return Rect.inside(this, point);
};
Rect.toTRBL = rect => ({
  top: rect.y,
  right: rect.x + rect.width,
  bottom: rect.y + rect.height,
  left: rect.x
});
Rect.prototype.toPoints = function() {
  var list = new PointList();
  list.push(new Point(this.x, this.y));
  list.push(new Point(this.x, this.y2));
  list.push(new Point(this.x2, this.y2));
  list.push(new Point(this.x2, this.y));
  return list;
};
Rect.prototype.pointFromCenter = function(point) {
  Point.sub(point, this.center);
  point.x /= this.width;
  point.y /= this.height;
  return point;
};
Rect.transform = function(rect1, rect2) {
  let translate = Point.diff(rect1, rect2);
  let scale = { x: rect1.width / rect2.width, y: rect1.height / rect2.height };
  let matrix = Matrix.identity();
  // matrix = Matrix.scale(matrix, scale.x, scale.y);
  matrix.translate_self(translate.x, translate.y);
  matrix.scale_self(scale.x, scale.y);
  return matrix;
};
