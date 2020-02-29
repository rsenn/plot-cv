
/**
 * Class for point.
 *
 * @class      Point (name)
 */
export function Point(arg) {
  let args = arg instanceof Array ? arg : [...arguments];
  let p = !this || this === Point ? {} : this;

  arg = args.shift();

  if(typeof arg === 'number') {
    p.x = parseFloat(arg);
    p.y = parseFloat(args.shift());
  } else if(typeof arg === 'string') {
    const matches = [...arg.matchAll(new RegExp('/([-+]?d*.?d+)(?:[eE]([-+]?d+))?/g'))];
    p.x = parseFloat(matches[0]);
    p.y = parseFloat(matches[1]);
  } else if(typeof arg == 'object' && arg !== null && (arg.x !== undefined || arg.y !== undefined)) {
    p.x = arg.x;
    p.y = arg.y;
  } else if(typeof arg == 'object' && arg !== null && arg.length > 0 && x !== undefined && y !== undefined) {
    p.x = parseFloat(arg.shift());
    p.y = parseFloat(arg.shift());
  } else if(typeof args[0] === 'number' && typeof args[1] === 'number') {
    p.x = args[0];
    p.y = args[1];
    args.shift(2);
  } else {
    p.x = 0;
    p.y = 0;
  }
  if(isNaN(p.x)) p.x = undefined;
  if(isNaN(p.y)) p.y = undefined;

  if(!this || this === Point) {
    if(p.prototype == Object) p.prototype = Point.prototype;
    else Object.assign(p, Point.prototype);
    return p;
  }
}

Point.move_to = (p, x, y) => {
  let args = [...arguments];
  /*p =*/ args.shift();
  let other = Point.call(Point, args);
  p.x = other.x;
  p.y = other.y;
  return p;
};
Point.move = (p, x, y) => {
  p.x += x;
  p.y += y;
  return p;
};

Point.prototype.move_to = function(x, y) {
  this.x = x;
  this.y = y;
  return this;
};
Point.prototype.clear = function(x, y) {
  this.x = 0;
  this.y = 0;
  return this;
};
Point.prototype.move = function(x, y) {
  this.x += x;
  this.y += y;
  return this;
};

Point.set = (p, fn) => {
  if(typeof fn != 'function') return Point.apply(p, [...arguments]);

  return fn(p.x, p.y);
};
Point.prototype.set = function(fn) {
  if(typeof fn != 'function') {
    Point.apply(this, [...arguments]);
    return this;
  }
  return fn(this.x, this.y);
};

Point.clone = ({ x, y }) => Point({ x, y });
Point.prototype.clone = function() {
  return new Point({ x: this.x, y: this.y });
};

Point.sum = (sum, pt) => new Point({ x: sum.x + pt.x, y: sum.y + pt.y });
Point.prototype.sum = function(other) {
  return new Point(this.x + other.x, this.y + other.y);
};
Point.prototype.add = function(other) {
  this.x += other.x;
  this.y += other.y;
  return this;
};
Point.add = (p, other) => Point.prototype.add.call(p, other);

Point.diff = (diff, pt) => Point({ x: diff.x - pt.x, y: diff.y - pt.y });
Point.prototype.diff = function(other) {
  return new Point(this.x - other.x, this.y - other.y);
};
Point.prototype.sub = function(other) {
  this.x -= other.x;
  this.y -= other.y;
  return this;
};
Point.sub = (p, other) => Point.prototype.sub.call(p, other);

Point.prod = (p, f) => {
  const o = Point.isPoint(f) ? f : { x: f, y: f };
  return Point({ x: p.x * o.x, y: p.y * o.y });
};

Point.prototype.prod = function(f) {
  const o = Point.isPoint(f) ? f : { x: f, y: f };
  return new Point(this.x * o.x, this.y * o.y);
};
Point.mul = (p, f) => {
  const o = Point.isPoint(f) ? f : { x: f, y: f };
  p.x *= o.x;
  p.y *= o.y;
  return p;
};
Point.prototype.mul = function(f) {
  const o = Point.isPoint(f) ? f : { x: f, y: f };
  this.x *= o.x;
  this.y *= o.y;
  return this;
};

Point.quot = (p, f) => Point({ x: p.x / f, y: p.y / f });
Point.prototype.quot = function(other) {
  return new Point(this.x / other.x, this.y / other.y);
};
Point.prototype.div = function(f) {
  this.x /= f;
  this.y /= f;
  return this;
};

Point.comp = p => Point({ x: -p.x, y: -p.y });
Point.prototype.neg = function() {
  this.x *= -1;
  this.y *= -1;
  return this;
};

Point.distance = (p1, p2) => {
  if(!p1) p1 = { x: 0, y: 0 };
  if(!p2) p2 = { x: 0, y: 0 };

  return Math.sqrt((p1.y - p2.y) * (p1.y - p2.y) + (p1.x - p2.x) * (p1.x - p2.x));
};

Point.prototype.distance = function(other = { x: 0, y: 0 }) {
  return Point.distance(this, other);
};

Point.equal = (a, b) => a.x == b.x && a.y == b.y;

Point.prototype.equal = function(other) {
  return Point.equal(this, other);
};

Point.round = (p, precision = 1.0) => {
  p.x = Math.round(p.x / precision) * precision;
  p.y = Math.round(p.y / precision) * precision;
  return p;
};

Point.prototype.round = function(precision = 1.0) {
  this.x = Math.round(this.x / precision) * precision;
  this.y = Math.round(this.y / precision) * precision;
  return this;
};

Point.sides = p => ({
  top: p.y,
  right: p.x + p.w1idth,
  bottom: p.y + p.height,
  left: p.x
});

Point.dot = (a, b) => a.x * b.x + a.y * b.y;
Point.prototype.dot = function(other) {
  return Point.dot(this, other);
};

Point.fromAngle = (angle, dist = 1.0) =>
  new Point({
    x: Math.cos(angle) * dist,
    y: Math.sin(angle) * dist
  });

Point.toAngle = (p, deg = false) => Math.atan2(p.x, p.y)  * (deg ? 180 / Math.PI : 1);
Point.prototype.toAngle = function(deg = false) {
  return Point.toAngle(this, deg);
};

Point.angle = (p1, p2 = { x: 0, y: 0 }) => Point.toAngle(Point.diff(p1, p2));
Point.prototype.angle = function(p2 = { x: 0, y: 0 }) {
  return Point.angle(this, p2);
};

Point.dimension = p => [p.width, p.height];

Point.toString = (point, prec) => {
  if(point instanceof Array) return `[${point[0].toFixed(3)},${point[1].toFixed(3)}]`;
  if(prec !== undefined) return point.x.toFixed(prec) + ',' + point.y.toFixed(prec);

  return `${point.x},${point.y}`;
  //  return Point.prototype.toString.call(point);
};
Point.prototype.toString = function(asArray = false) {
  if(asArray)
    return `[${this.x},${this.y}]`;
  return `{x:${this.x},y:${this.y}}`;
};
Point.prototype.toSource = function() {
  return '{x:' + this.x.toFixed(3) + ',y:' + this.y.toFixed(3) + '}';
};

Point.toCSS = function() {
  const point = this instanceof Point ? this : Point.apply(Point, arguments);
  return {
    left: point.x + 'px',
    top: point.y + 'px'
  };
};
Point.prototype.toCSS = Point.toCSS;

Point.match = new RegExp('/([0-9.]+,[0-9.]+)/');

Point.inside = (p, rect) => p.x >= rect.x && p.x < rect.x + rect.width && p.y >= rect.y && p.y < rect.y + rect.height;

Point.transform = (p, m) => Point.prototype.transform.call(p, m);

Point.prototype.transform = function(m) {
  Matrix.prototype.transform_point.call(m, this);
  return this;
};

Point.prototype.normalize = function(minmax) {
  return new Point({
    x: (this.x - minmax.x1) / (minmax.x2 - minmax.x1),
    y: (this.y - minmax.y1) / (minmax.y2 - minmax.y1)
  });
};

Point.isPoint = o => o && ((o.x !== undefined && o.y !== undefined) || ((o.left !== undefined || o.right !== undefined) && (o.top !== undefined || o.bottom !== undefined)));

