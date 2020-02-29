/**
 * Class for line.
 *
 * @class      Line (name)
 */
function Line(x1, y1, x2, y2) {
  let obj = this instanceof Line ? this : {};
  let arg;
  let args = [...arguments];
  let ret;
  if(args.length >= 4 && args.every(arg => !isNaN(parseFloat(arg)))) {
    arg = { x1, y1, x2, y2 };
  } else if(args.length == 1) {
    arg = args[0];
  }
  if(arg && arg.x1 !== undefined && arg.y1 !== undefined && arg.x2 !== undefined && arg.y2 !== undefined) {
    const { x1, y1, x2, y2 } = arg;
    obj.x1 = parseFloat(x1);
    obj.y1 = parseFloat(y1);
    obj.x2 = parseFloat(x2);
    obj.y2 = parseFloat(y2);
    ret = 1;
  } else if(Point.isPoint(args[0]) && isPoint(args[1])) {
    obj.x1 = parseFloat(args[0].x);
    obj.y1 = parseFloat(args[0].y);
    obj.x2 = parseFloat(args[1].x);
    obj.y2 = parseFloat(args[1].y);
    ret = 2;
  } else if(arg && arg.length >= 4 && arg.slice(0, 4).every(arg => !isNaN(parseFloat(arg)))) {
    obj.x1 = typeof x === 'number' ? x : parseFloat(x);
    obj.y1 = typeof y === 'number' ? y : parseFloat(y);
    obj.x2 = typeof w === 'number' ? w : parseFloat(w);
    obj.y2 = typeof h === 'number' ? h : parseFloat(h);
    ret = 4;
  } else {
    ret = 0;
  }
  if(!isLine(obj)) console.log('ERROR: is not a line: ', [...arguments]);
  if(!(this instanceof Line)) return obj;
}
//export const isLine = obj => ['x1', 'y1', 'x2', 'y2'].every(prop => obj[prop] !== undefined);
Line.intersect = (a, b) => {
  const ma = (a[0].y - a[1].y) / (a[0].x - a[1].x); // slope of line 1
  const mb = (b[0].y - b[1].y) / (b[0].x - b[1].x); // slope of line 2
  if(ma - mb < Number.EPSILON) return undefined;
  return new Point({
    x: (ma * a[0].x - mb * b[0].x + b[0].y - a[0].y) / (ma - mb),
    y: (ma * mb * (b[0].x - a[0].x) + mb * a[0].y - ma * b[0].y) / (mb - ma)
  });
};
Object.defineProperty(
  Line.prototype,
  'x1', {
  get: function() {
    return this.a && this.a.x;
  },
  set: function(v) {
    if(!this.a) this.a = new Point();
    this.a.x = v;
  },
  enumerable: true }
);
Object.defineProperty(
  Line.prototype,
  'y1', {
  get: function() {
    return this.a && this.a.y;
  },
  set: function(v) {
    if(!this.a) this.a = new Point();
    this.a.y = v;
  },
  enumerable: true }
);
Object.defineProperty(
  Line.prototype,
  'x2', {
  get: function() {
    return this.b && this.b.x;
  },
  set: function(v) {
    if(!this.b) this.b = new Point();
    this.b.x = v;
  },
  enumerable: true }
);
Object.defineProperty(
  Line.prototype,
  'y2', {
  get: function() {
    return this.b && this.b.y;
  },
  set: function(v) {
    if(!this.b) this.b = new Point();
    this.b.y = v;
  },
  enumerable: true }
);
Line.prototype.direction = function() {
  var dist = Point.distance(this.a, this.b);
  return Point.diff(this.a, this.b) / dist;
};
Line.prototype.slope = function() {
  return Point.diff(this.a, this.b);
};
Line.prototype.angle = function() {
  return Point.angle(this.a, this.b);
};
Line.prototype.length = function() {
  return Point.distance(this.a, this.b);
};
Line.prototype.pointAt = function(pos) {
  return new Point(pos * (this.x2 - this.x1) + this.x1, pos * (this.y2 - this.y1) + this.y1);
};
Line.prototype.transform = function(m) {
  this.a = this.a.transform(m);
  this.b = this.b.transform(m);
  return this;
};
Line.transform = (line, matrix) => {
  const a = Point.transform({ x: line.x1, y: line.y1 }, matrix);
  const b = Point.transform({ x: line.x2, y: line.y2 }, matrix);
  return {
    x1: a.x,
    y1: a.y,
    x2: b.x,
    y2: b.y
  };
};
Line.bbox = line => BBox.fromPoints(Line.points(line));
Line.prototype.bbox = function() {
  return BBox.fromPoints(this.points());
};
Line.points = line => {
  const { a, b } = line;
  return [a, b];
};
Line.prototype.points = function() {
  return Line.points(this);
};
Line.prototype.inspect = function() {
  const { x1, y1, x2, y2 } = this;
  return 'Line{ ' + inspect({ x1, y1, x2, y2 }) + ' }';
};
Line.prototype.toString = function() {
  let { a, b } = this;
  if(a.x > b.x) {
    let tmp = this.b;
    this.b = this.a;
    this.a = tmp;
  }
  return Point.prototype.toString.call(this.a) + ' -> ' + Point.prototype.toString.call(this.b);
};
