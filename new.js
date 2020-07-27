class Node {
  static parents(node) {
    return (function*() {
      var n = node;

      do {
        if(n) yield n;
      } while(n && (n = n.parentNode));
    })();
  }

  static depth(node) {
    let r = 0;

    while(node && node.parentNode) {
      r++;
      node = node.parentNode;
    }

    return r;
  }

  static attrs(node) {
    return node.attributes && node.attributes.length > 0 ? Array.from(node.attributes).reduce((acc, attr) => ({ ...acc, [[attr.name]]: isNaN(parseFloat(attr.value)) ? attr.value : parseFloat(attr.value) }), {}) : {};
  }

  static map = map;
}
Node;

const SymSpecies = Util.tryCatch(
  () => Symbol,
  sym => sym.species
);

const CTOR = obj => {
  if(obj.SymSpecies) return obj.SymSpecies;
  let p = Object.getPrototypeOf(obj);
  if(p.SymSpecies) return p.SymSpecies;
  return p.constructor;
};

function Point(arg) {
  let args = arg instanceof Array ? arg : [...arguments];
  let p = this instanceof Point ? this : null;
  arg = args.shift();

  if(p === null) {
    if(arg instanceof Point) return arg;
    p = {};
  }

  if(typeof arg === 'undefined') {
    p.x = arg;
    p.y = args.shift();
  } else if(typeof arg === 'number') {
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

  if(p.x === undefined) p.x = 0;
  if(p.y === undefined) p.y = 0;
  if(isNaN(p.x)) p.x = undefined;
  if(isNaN(p.y)) p.y = undefined;

  if(!this || this === Point) {
    if(p.prototype == Object) p.prototype = Point.prototype;
    else Object.assign(p, Point.prototype);
    return p;
  }
}

Object.defineProperties(Point.prototype, {
  X: {
    get() {
      return this.x;
    }
  },
  Y: {
    get() {
      return this.y;
    }
  }
});

Point.prototype.move = function(x, y) {
  this.x += x;
  this.y += y;
  return this;
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

Point.prototype.set = function(fn) {
  if(typeof fn != 'function') {
    Point.apply(this, [...arguments]);
    return this;
  }

  return fn(this.x, this.y);
};

Point.prototype.clone = function() {
  const ctor = this[Symbol.species] || this.constructor[Symbol.species];
  return new ctor({ x: this.x, y: this.y });
};

Point.prototype.sum = function(...args) {
  const p = new Point(...args);
  let r = new Point(this.x, this.y);
  r.x += p.x;
  r.y += p.y;
  return r;
};

Point.prototype.add = function(...args) {
  const other = new Point(...args);
  this.x += other.x;
  this.y += other.y;
  return this;
};

Point.prototype.diff = function(...args) {
  const p = new Point(...args);
  let r = new Point(this.x, this.y);
  r.x -= p.x;
  r.y -= p.y;
  return r;
};

Point.prototype.sub = function(...args) {
  const other = new Point(...args);
  this.x -= other.x;
  this.y -= other.y;
  return this;
};

Point.prototype.prod = function(f) {
  const o = isPoint(f) ? f : { x: f, y: f };
  return new Point(this.x * o.x, this.y * o.y);
};

Point.prototype.mul = function(f) {
  const o = isPoint(f) ? f : { x: f, y: f };
  this.x *= o.x;
  this.y *= o.y;
  return this;
};

Point.prototype.quot = function(other) {
  other = isPoint(other) ? other : { x: other, y: other };
  return new Point(this.x / other.x, this.y / other.y);
};

Point.prototype.div = function(other) {
  other = isPoint(other) ? other : { x: other, y: other };
  this.x /= other.x;
  this.y /= other.y;
  return this;
};

Point.prototype.comp = function() {
  return new Point({ x: -this.x, y: -this.y });
};

Point.prototype.neg = function() {
  this.x *= -1;
  this.y *= -1;
  return this;
};

Point.prototype.distanceSquared = function(other = { x: 0, y: 0 }) {
  return (other.y - this.y) * (other.y - this.y) + (other.x - this.x) * (other.x - this.x);
};

Point.prototype.distance = function(other = { x: 0, y: 0 }) {
  return Math.sqrt(Point.prototype.distanceSquared.call(this, other));
};

Point.prototype.equals = function(other) {
  return +this.x == +other.x && +this.y == +other.y;
};

Point.prototype.round = function(precision = (0.001, digits, (type = 'round'))) {
  let { x, y } = this;
  this.x = Util.roundTo(x, precision, digits, type);
  this.y = Util.roundTo(y, precision, digits, type);
  return this;
};

Point.prototype.sides = function() {
  return { top: this.y, right: this.x + this.w1idth, bottom: this.y + this.height, left: this.x };
};

Point.prototype.dot = function(other) {
  return this.x * other.x + this.y * other.y;
};

Point.prototype.fromAngle = function(angle, dist = 1.0) {
  this.x = Math.cos(angle) * dist;
  this.y = Math.sin(angle) * dist;
  return this;
};

Point.prototype.toAngle = function(deg = false) {
  return Math.atan2(this.x, this.y) * (deg ? 180 / Math.PI : 1);
};

Point.prototype.angle = function(other, deg = false) {
  other = other || { x: 0, y: 0 };
  return Point.prototype.diff.call(this, other).toAngle(deg);
};

Point.prototype.rotate = function(angle, origin = { x: 0, y: 0 }) {
  this.x -= origin.x;
  this.y -= origin.y;
  let c = Math.cos(angle),
    s = Math.sin(angle);
  let xnew = this.x * c - this.y * s;
  let ynew = this.x * s + this.y * c;
  this.x = xnew;
  this.y = ynew;
  return this;
};

Point.prototype.dimension = function() {
  return [this.width, this.height];
};

Util.defineGetter(Point.prototype, Symbol.iterator, function() {
  const { x, y } = this;
  let a = [x, y];
  return a[Symbol.iterator].bind(a);
});

Point.prototype.toString = function(opts = {}) {
  const { precision, unit, separator, left, right } = opts;
  const x = Util.roundTo(this.x, precision);
  const y = Util.roundTo(this.y, precision);
  return `${left}${x}${unit}${separator}${y}${unit}${right}`;
};

Util.defineGetterSetter(
  Point.prototype,
  Symbol.toStringTag,
  function() {
    return `Point{ ${Point.prototype.toSource.call(this)}`;
  },
  () => {},
  false
);

Point.prototype.toSource = function(opts = {}) {
  const { asArray, pad, showNew } = opts;
  let x = pad(this.x + '');
  let y = pad(this.y + '');
  if(typeof this != 'object' || this === null) return '';
  if(asArray) return `[${x},${y}]`;
  return `${Util.colorText(showNew ? 'new ' : '', 1, 31)}${Util.colorText('Point', 1, 33)}${Util.colorText('(', 1, 36)}${Util.colorText(x, 1, 32)}${Util.colorText(',', 1, 36)}${Util.colorText(y, 1, 32)}${Util.colorText(')', 1, 36)}`;
};

Point.prototype.toObject = function() {
  const { x, y } = this;
  const obj = { x, y };
  Object.setPrototypeOf(obj, Point.prototype);
  return obj;
};

Point.prototype.toCSS = function(precision = 0.001) {
  return { left: Util.roundTo(this.x, precision) + 'px', top: Util.roundTo(this.y, precision) + 'px' };
};

Point.prototype.toFixed = function(digits) {
  return new Point(+this.x.toFixed(digits), +this.y.toFixed(digits));
};

Point.prototype.isNull = function() {
  return this.x == 0 && this.y == 0;
};

Point.prototype.inside = function(rect) {
  return this.x >= rect.x && this.x < rect.x + rect.width && this.y >= rect.y && this.y < rect.y + rect.height;
};

Point.prototype.normalize = function(minmax) {
  return new Point({ x: (this.x - minmax.x1) / (minmax.x2 - minmax.x1), y: (this.y - minmax.y1) / (minmax.y2 - minmax.y1) });
};
Point.move = (point, x, y) => Point.prototype.move.call(point, x, y);
Point.angle = (point, other, deg = false) => Point.prototype.angle.call(point, other, deg);
Point.inside = (point, rect) => Point.prototype.inside.call(point, rect);
Point.sub = (point, other) => Point.prototype.sub.call(point, other);
Point.prod = (a, b) => Point.prototype.prod.call(a, b);
Point.quot = (a, b) => Point.prototype.quot.call(a, b);

Point.equals = (a, b) => {
  let ret = Point.prototype.equals.call(a, b);
  return ret;
};
Point.round = (point, prec) => Point.prototype.round.call(point, prec);
Point.fromAngle = (angle, f) => Point.prototype.fromAngle.call(new Point(0, 0), angle, f);

for(let name of ['clone', 'comp', 'neg', 'sides', 'dimension', 'toString', 'toCSS', 'sub', 'diff', 'add', 'sum', 'distance']) {
  Point.name = (point, ...args) => Point.prototype.name.call(point || new Point(point), ...args);
}
Point.toSource = point => `{ x:${point.x}, y: ${point.y} }`;
const isPoint = o => o && ((o.x !== undefined && o.y !== undefined) || ((o.left !== undefined || o.right !== undefined) && (o.top !== undefined || o.bottom !== undefined)) || o instanceof Point || Object.getPrototypeOf(o).constructor === Point);
Point.isPoint = isPoint;
Util.defineInspect(Point.prototype, 'x', 'y');

Point.bind = (o, p, gen) => {
  const [x, y] = p || ['x', 'y'];
  if(!gen) gen = k => v => (v === undefined ? o.k : (o.k = v));
  return Util.bindProperties(new Point(0, 0), o, { x, y }, gen);
};
Point;

Util.defineGetter(Point, Symbol.species, function() {
  return this;
});
const ImmutablePoint = Util.immutableClass(Point);

Util.defineGetter(ImmutablePoint, Symbol.species, function() {
  return ImmutablePoint;
});

class BBox {
  static fromPoints(pts) {
    let pt = pts.shift();
    let bb = new BBox(pt.x, pt.y, pt.x, pt.y);
    bb.update(pts);
    return bb;
  }

  constructor(x1, y1, x2, y2) {
    if(x1 !== undefined && y1 !== undefined && x2 !== undefined && y2 !== undefined) {
      this.x1 = Math.min(x1, x2);
      this.y1 = Math.min(y1, y2);
      this.x2 = Math.max(x1, x2);
      this.y2 = Math.max(y1, y2);
    } else {
      this.x1 = undefined;
      this.y1 = undefined;
      this.x2 = undefined;
      this.y2 = undefined;
    }

    Util.define(this, 'objects', {});
  }

  getObjects() {
    return new Map(Object.entries(this.objects));
  }

  updateList(list, offset = 0.0) {
    for(let arg of list) this.update(arg, offset);
    return this;
  }

  update(arg, offset = (0.0, (obj = null))) {
    if(Util.isArray(arg)) return this.updateList(arg, offset);
    if(arg.x !== undefined && arg.y != undefined) this.updateXY(arg.x, arg.y, offset, name => (this.objects.name = obj || arg));
    if(arg.x1 !== undefined && arg.y1 != undefined) this.updateXY(arg.x1, arg.y1, 0, name => (this.objects.name = obj || arg));
    if(arg.x2 !== undefined && arg.y2 != undefined) this.updateXY(arg.x2, arg.y2, 0, name => (this.objects.name = obj || arg));
    return this;
  }

  updateXY(x, y, offset = (0, (set = () => {}))) {
    let updated = {};

    if(this.x1 === undefined || this.x1 > x - offset) {
      this.x1 = x - offset;
      set('x1');
    }

    if(this.x2 === undefined || this.x2 < x + offset) {
      this.x2 = x + offset;
      set('x2');
    }

    if(this.y1 === undefined || this.y1 > y - offset) {
      this.y1 = y - offset;
      set('y1');
    }

    if(this.y2 === undefined || this.y2 < y + offset) {
      this.y2 = y + offset;
      set('y2');
    }

    return this;
  }

  get center() {
    return new Point({ x: this.x + this.width / 2, y: this.y + this.height / 2 });
  }

  relative_to(x, y) {
    return new BBox(this.x1 - x, this.y1 - y, this.x2 - x, this.y2 - y);
  }

  get x() {
    return this.x1;
  }

  get width() {
    return this.x2 - this.x1;
  }

  get y() {
    return this.y1 < this.y2 ? this.y1 : this.y2;
  }

  get height() {
    return this.y2 - this.y1;
  }

  set;
  x(x) {
    let ix = x - this.x1;
    this.x1 += ix;
    this.x2 += ix;
  }

  set;
  width(w) {
    this.x2 = this.x1 + w;
  }

  set;
  y(y) {
    let iy = y - this.y1;
    this.y1 += iy;
    this.y2 += iy;
  }

  set;
  height(h) {
    this.y2 = this.y1 + h;
  }

  get rect() {
    const { x1, y1, x2, y2 } = this;
    return { x: x1, y: y1, width: x2 - x1, height: y2 - y1 };
  }

  toObject() {
    const { x1, y1, x2, y2 } = this;
    let obj = Object.create(null);
    obj.x1 = x1;
    obj.y1 = y1;
    obj.x2 = x2;
    obj.y2 = y2;
    return obj;
  }

  toString() {
    return `${this.x1} ${this.y1} ${this.x2} ${this.y2}`;
  }

  transform(fn = (arg => arg, out)) {
    if(!out) out = this;

    for(let prop of ['x1', 'y1', 'x2', 'y2']) {
      const v = this.prop;
      out.prop = fn(v);
    }

    return this;
  }

  round(fn = arg => Math.round(arg)) {
    let ret = new BBox();
    this.transform(fn, ret);
    return ret;
  }

  move(x, y) {
    this.x1 += x;
    this.y1 += y;
    this.x2 += x;
    this.y2 += y;
    return this;
  }

  static from(iter, tp = p => p) {
    if(typeof iter == 'object' && iter[Symbol.iterator]) iter = iter[Symbol.iterator]();
    let r = new BBox();
    let result = iter.next();
    let p;

    if(result.value) {
      p = tp(result.value);
      r.x1 = p.x;
      r.x2 = p.x;
      r.y1 = p.y;
      r.y2 = p.y;
    }

    while(true) {
      result = iter.next();
      if(!result.value) break;
      p = tp(result.value);
      r.update(p);
    }

    return r;
  }
}

function Line(x1, y1, x2, y2) {
  let obj;
  let arg;
  let args = [...arguments];
  let ret;

  if(args.length >= 4 && args.every(arg => !isNaN(parseFloat(arg)))) {
    arg = { x1, y1, x2, y2 };
  } else if(args.length == 1) {
    arg = args[0];
  }

  obj = this || { ...arg };
  if(obj === null) obj = Object.create(Line.prototype);
  if(Object.getPrototypeOf(obj) !== Line.prototype) Object.setPrototypeOf(obj, Line.prototype);

  if(arg && arg.x1 !== undefined && arg.y1 !== undefined && arg.x2 !== undefined && arg.y2 !== undefined) {
    const { x1, y1, x2, y2 } = arg;
    obj.x1 = parseFloat(x1);
    obj.y1 = parseFloat(y1);
    obj.x2 = parseFloat(x2);
    obj.y2 = parseFloat(y2);
    ret = 1;
  } else if(isPoint(args[0]) && isPoint(args[1])) {
    obj.x1 = args[0].x;
    obj.y1 = args[0].y;
    obj.x2 = args[1].x;
    obj.y2 = args[1].y;
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

  if(!('a' in obj) || obj.a === undefined) Object.defineProperty(obj, 'a', { value: new Point(obj.x1, obj.y1), enumerable: false });
  if(!('b' in obj) || obj.b === undefined) Object.defineProperty(obj, 'b', { value: new Point(obj.x2, obj.y2), enumerable: false });
  if(!isLine(obj)) {
  }
  return obj;
}
const isLine = obj => ['x1', 'y1', 'x2', 'y2'].every(prop => obj.prop !== undefined) || ['a', 'b'].every(prop => isPoint(obj.prop));

Line.prototype.intersect = function(other) {
  const ma = (this[0].y - this[1].y) / (this[0].x - this[1].x);
  const mb = (other[0].y - other[1].y) / (other[0].x - other[1].x);
  if(ma - mb < Number.EPSILON) return undefined;
  return new Point({ x: (ma * this[0].x - mb * other[0].x + other[0].y - this[0].y) / (ma - mb), y: (ma * mb * (other[0].x - this[0].x) + mb * this[0].y - ma * other[0].y) / (mb - ma) });
};

Object.defineProperty(Line.prototype, 0, {
  get() {
    return this.a;
  },
  set(v) {
    this.a.x = v.x;
    this.a.y = v.y;
  },
  enumerable: false
});

Object.defineProperty(Line.prototype, 1, {
  get() {
    return this.b;
  },
  set(v) {
    this.b.x = v.x;
    this.b.y = v.y;
  },
  enumerable: false
});

Object.defineProperty(Line.prototype, 'x1', {
  get() {
    return this.a && this.a.x;
  },
  set(v) {
    if(!this.a) Object.defineProperty(this, 'a', { value: new Point(), enumerable: false });
    this.a.x = v;
  },
  enumerable: true
});

Object.defineProperty(Line.prototype, 'y1', {
  get() {
    return this.a && this.a.y;
  },
  set(v) {
    if(!this.a) Object.defineProperty(this, 'a', { value: new Point(), enumerable: false });
    this.a.y = v;
  },
  enumerable: true
});

Object.defineProperty(Line.prototype, 'x2', {
  get() {
    return this.b && this.b.x;
  },
  set(v) {
    if(!this.b) Object.defineProperty(this, 'b', { value: new Point(), enumerable: false });
    this.b.x = v;
  },
  enumerable: true
});

Object.defineProperty(Line.prototype, 'y2', {
  get() {
    return this.b && this.b.y;
  },
  set(v) {
    if(!this.b) Object.defineProperty(this, 'b', { value: new Point(), enumerable: false });
    this.b.y = v;
  },
  enumerable: true
});

Line.prototype.direction = function() {
  var dist = Point.prototype.distance.call(this.a, this.b);
  return Point.prototype.quot.call(Line.prototype.getSlope.call(this), dist);
};

Line.prototype.getVector = function() {
  return { x: this.x2 - this.x1, y: this.y2 - this.y1 };
};
Object.defineProperty(Line.prototype, 'vector', { get: Line.prototype.getVector });

Line.prototype.getSlope = function() {
  return (this.y2 - this.y1) / (this.x2 - this.x1);
};
Object.defineProperty(Line.prototype, 'slope', { get: Line.prototype.getSlope });

Line.prototype.yIntercept = function() {
  let v = Line.prototype.getVector.call(this);

  if(v.x !== 0) {
    let slope = v.y / v.x;
    return [this.a.y - this.a.x * slope, slope || 0];
  }
};

Line.prototype.xIntercept = function() {
  let v = Line.prototype.getVector.call(this);

  if(v.y !== 0) {
    let slope = v.x / v.y;
    return [this.a.x - this.a.y * slope, slope || 0];
  }
};

Line.prototype.isHorizontal = function() {
  return Line.prototype.getVector.call(this).y === 0;
};

Line.prototype.isVertical = function() {
  return Line.prototype.getVector.call(this).x === 0;
};

Line.prototype.isNull = function() {
  return this.x1 == 0 && this.y1 == 0 && this.x2 == 0 && this.y2 == 0;
};

Line.prototype.equations = function() {
  let intercept = { y: Line.prototype.yIntercept.call(this), x: Line.prototype.xIntercept.call(this) };
  let equations = [];

  for(let axis in intercept) {
    if(intercept.axis) {
      let [c0, m] = intercept.axis;
      let rhs = `${c0}`;
      if(m !== 0) rhs += ` + ${m} * ${axis == 'y' ? 'x' : 'y'}`;
      equations.push(`${axis} = ${rhs}`);
    }
  }

  return equations;
};

Line.prototype.functions = function() {
  let i;
  let fns = {};

  if((i = Line.prototype.yIntercept.call(this))) {
    let [y0, myx] = i;
    fns.y = x => y0 + myx * x;
  } else {
    let { y } = this.a;
    fns.y = new Function('x', `return ${y}`);
  }

  if((i = Line.prototype.xIntercept.call(this))) {
    let [x0, mxy] = i;
    fns.x = y => x0 + mxy * y;
  } else {
    let { x } = this.a;
    fns.x = new Function('y', `return ${x}`);
  }

  return fns;
};

Line.prototype.angle = function() {
  return Point.prototype.angle.call(Line.prototype.getVector.call(this));
};

Line.prototype.getLength = function() {
  const { a, b } = this;
  const { x1, y1, x2, y2 } = this;
  return Point.prototype.distance.call(a, b);
};

Line.prototype.endpointDist = function(point) {
  return Math.min(point.distance(this.a), point.distance(this.b));
};

Line.prototype.matchEndpoints = function(arr) {
  const { a, b } = this;
  return [...arr.entries()].filter(([i, otherLine]) => !Line.prototype.equals.call(this, otherLine) && (Point.prototype.equals.call(a, otherLine.a) || Point.prototype.equals.call(b, otherLine.b) || Point.prototype.equals.call(b, otherLine.a) || Point.prototype.equals.call(a, otherLine.b)));
};

Line.prototype.distanceToPointSquared = function(p) {
  const { a, b } = this;
  var l2 = Point.prototype.distanceSquared.call(a, b);
  if(l2 === 0) return Point.prototype.distanceSquared.call(p, a);
  var t = ((p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y)) / l2;
  t = Math.max(0, Math.min(1, t));
  return Point.prototype.distanceSquared.call(p, new Point(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)));
};

Line.prototype.distanceToPoint = function(p) {
  return Math.sqrt(Line.prototype.distanceToPointSquared.call(this, p));
};
Object.defineProperty(Line.prototype, 'len', { get: Line.prototype.getLength });

Object.defineProperty(Line.prototype, 'cross', {
  get() {
    const { x1, x2, y1, y2 } = this;
    return x1 * y2 - y1 * x2;
  }
});

Object.defineProperty(Line.prototype, 'dot', {
  get() {
    const { x1, x2, y1, y2 } = this;
    return x1 * x2 + y1 * y2;
  }
});

Line.prototype.pointAt = function(pos) {
  return new Point(pos * (this.x2 - this.x1) + this.x1, pos * (this.y2 - this.y1) + this.y1);
};

Line.prototype.transform = function(m) {
  this.a = this.a.transform(m);
  this.b = this.b.transform(m);
  return this;
};

Line.prototype.bbox = function() {
  const { x1, y1, x2, y2 } = this;
  return new BBox(x1, y1, x2, y2);
};

Line.prototype.add = function(other) {
  const { x1, y1, x2, y2 } = Line(...arguments);
  this.x1 += x1;
  this.y1 += y1;
  this.x2 += x2;
  this.y2 += y2;
  return this;
};

Line.prototype.points = function() {
  const { a, b } = this;
  return [a, b];
};

Line.prototype.diff = function(other) {
  other = Line(...arguments);
  return new Line(Point.diff(this.a, other.a), Point.diff(this.b, other.b));
};

Line.prototype.inspect = function() {
  const { x1, y1, x2, y2 } = this;
  return 'Line{ ' + inspect({ x1, y1, x2, y2 }) + ' }';
};

Line.prototype.toString = function() {
  const { x1, y1, x2, y2 } = this;
  return Point.toString(this.a || Point(x1, y1)) + ' -> ' + Point.toString(this.b || Point(x2, y2));
};

Line.prototype.toSource = function() {
  let { a, b } = this;
  return `new Line(${a.x},${a.y},${b.x},${b.y})`;
};

Line.prototype.reverse = function() {
  const { a, b } = this;
  return new Line(b, a);
};

Line.prototype.toObject = function(t = num => num) {
  const { x1, y1, x2, y2 } = this;
  const obj = { x1: t(x1), y1: t(y1), x2: t(x2), y2: t(y2) };
  Object.setPrototypeOf(obj, Line.prototype);
  return obj;
};

Line.prototype.clone = function() {
  const ctor = this.constructor[Symbol.species];
  const { x1, y1, x2, y2 } = this;
  return new ctor(x1, y1, x2, y2);
};

Line.prototype.round = function(precision = (0.001, digits)) {
  let { x1, y1, x2, y2 } = this;
  this.a.x = Util.roundTo(x1, precision, digits);
  this.a.y = Util.roundTo(y1, precision, digits);
  this.b.x = Util.roundTo(x2, precision, digits);
  this.b.y = Util.roundTo(y2, precision, digits);
  return this;
};

Line.prototype.some = function(pred) {
  return pred(this.a) || pred(this.b);
};

Line.prototype.every = function(pred) {
  return pred(this.a) && pred(this.b);
};

Line.prototype.includes = function(point) {
  return Point.prototype.equals.call(this.a, point) || Point.prototype.equals.call(this.b, point);
};

Line.prototype.equals = function(other) {
  other = Line(other);
  if(Point.equals(this.a, other.a) && Point.equals(this.b, other.b)) return 1;
  if(Point.equals(this.a, other.b) && Point.equals(this.b, other.a)) return -1;
  return false;
};

Line.prototype.indexOf = function(point) {
  let i = 0;

  for(let p of [this.a, this.b]) {
    if(Point.prototype.equals.call(p, point)) return i;
    i++;
  }

  return -1;
};

Line.prototype.lastIndexOf = function(point) {
  let i = 0;

  for(let p of [this.b, this.a]) {
    if(Point.prototype.equals.call(p, point)) return i;
    i++;
  }

  return -1;
};

Line.prototype.map = function(fn) {
  let i = 0;
  let r = [];

  for(let p of [this.a, this.b]) {
    r.push(fn(p, i, this));
    i++;
  }

  return new Line(...r);
};

Line.prototype.swap = function(fn) {
  return new Line(this.a, this.b).reverse();
};

Line.prototype.toPoints = function() {
  const { x1, y1, x2, y2 } = this;
  var list = new PointList();
  list.push(new Point(x1, y1));
  list.push(new Point(x2, y1));
  list.push(new Point(x2, y2));
  list.push(new Point(x1, y2));
  return list;
};

for(let name of ['direction', 'round', 'slope', 'angle', 'bbox', 'points', 'inspect', 'toString', 'toObject', 'toSource', 'distanceToPointSquared', 'distanceToPoint']) {
  Line.name = (line, ...args) => Line.prototype.name.call(line || new Line(line), ...args);
}
Util.defineInspect(Line.prototype, 'x1', 'y1', 'x2', 'y2');

Line.bind = (o, p, gen) => {
  const [x1, y1, x2, y2] = p || ['x1', 'y1', 'x2', 'y2'];
  if(!gen) gen = k => v => (v === undefined ? o.k : (o.k = v));
  let proxy = { a: Point.bind(o, [x1, y1], gen), b: Point.bind(o, [x2, y2], gen) };
  return Object.setPrototypeOf(proxy, Line.prototype);
};

Util.defineGetter(Line, Symbol.species, function() {
  return this;
});
const ImmutableLine = Util.immutableClass(Line);

Util.defineGetter(ImmutableLine, Symbol.species, function() {
  return ImmutableLine;
});

function Size(arg) {
  let obj = this instanceof Size ? this : {};
  let args = [...arguments];

  if(args.length == 1 && Util.isObject(args[0]) && args[0].length !== undefined) {
    args = args[0];
    arg = args[0];
  }

  if(typeof arg == 'object') {
    if(arg.width !== undefined || arg.height !== undefined) {
      arg = args.shift();
      obj.width = arg.width;
      obj.height = arg.height;
    } else if(arg.x2 !== undefined && arg.y2 !== undefined) {
      arg = args.shift();
      obj.width = arg.x2 - arg.x;
      obj.height = arg.y2 - arg.y;
    } else if(arg.bottom !== undefined && arg.right !== undefined) {
      arg = args.shift();
      obj.width = arg.right - arg.left;
      obj.height = arg.bottom - arg.top;
    }
  } else {
    while(typeof arg == 'object' && (arg instanceof Array || 'length' in arg)) {
      args = [...arg];
      arg = args[0];
    }

    if(args && args.length >= 2) {
      let w = args.shift();
      let h = args.shift();
      if(typeof w == 'object' && 'baseVal' in w) w = w.baseVal.value;
      if(typeof h == 'object' && 'baseVal' in h) h = h.baseVal.value;
      obj.width = typeof w == 'number' ? w : parseFloat(w.replace(/[^-.0-9]*$/, ''));
      obj.height = typeof h == 'number' ? h : parseFloat(h.replace(/[^-.0-9]*$/, ''));
      Object.defineProperty(obj, 'units', { value: { width: typeof w == 'number' ? 'px' : w.replace(obj.width.toString(), ''), height: typeof h == 'number' ? 'px' : h.replace(obj.height.toString(), '') }, enumerable: false });
    }
  }

  if(isNaN(obj.width)) obj.width = undefined;
  if(isNaN(obj.height)) obj.height = undefined;
  if(!(obj instanceof Size)) return obj;
}
Size.prototype.width = NaN;
Size.prototype.height = NaN;
Size.prototype.units = null;

Size.prototype.convertUnits = function(w = 'window' in global ? window : null) {
  if(w === null) return this;
  const view = { vw: w.innerWidth, vh: w.innerHeight, vmin: w.innerWidth < w.innerHeight ? w.innerWidth : w.innerHeight, vmax: w.innerWidth > w.innerHeight ? w.innerWidth : w.innerHeight };

  if(view[this.units.width] !== undefined) {
    this.width = (this.width / 100) * view[this.units.width];
    delete this.units.width;
  }

  if(view[this.units.height] !== undefined) {
    this.height = (this.height / 100) * view[this.units.height];
    delete this.units.height;
  }

  return size;
};

Size.prototype.aspect = function() {
  return this.width / this.height;
};

Size.prototype.toCSS = function(units) {
  let ret = {};
  units = typeof units == 'string' ? { width: units, height: units } : units || this.units || { width: 'px', height: 'px' };
  if(this.width !== undefined) ret.width = this.width + (units.width || 'px');
  if(this.height !== undefined) ret.height = this.height + (units.height || 'px');
  return ret;
};

Size.prototype.transform = function(m) {
  this.width = m.xx * this.width + m.yx * this.height;
  this.height = m.xy * this.width + m.yy * this.height;
  return this;
};

Size.prototype.isSquare = function() {
  return Math.abs(this.width - this.height) < 1;
};

Size.prototype.area = function() {
  return this.width * this.height;
};

Size.prototype.resize = function(width, height) {
  this.width = width;
  this.height = height;
  return this;
};

Size.prototype.sum = function(other) {
  return new Size(this.width + other.width, this.height + other.height);
};

Size.prototype.add = function() {
  for(let other of [...arguments]) {
    this.width += other.width;
    this.height += other.height;
  }

  return this;
};

Size.prototype.diff = function(other) {
  return new Size(this.width - other.width, this.height - other.height);
};

Size.prototype.sub = function() {
  for(let other of [...arguments]) {
    this.width -= other.width;
    this.height -= other.height;
  }

  return this;
};

Size.prototype.prod = function(f) {
  const o = isSize(f) ? f : isPoint(f) ? { width: f.x, height: f.y } : { width: f, height: f };
  return new Size(this.width * o.width, this.height * o.height);
};

Size.prototype.mul = function(...args) {
  for(let f of args) {
    const o = isSize(f) ? f : isPoint(f) ? { width: f.x, height: f.y } : { width: f, height: f };
    this.width *= o.width;
    this.height *= o.height;
  }

  return this;
};

Size.prototype.quot = function(other) {
  return new Size(this.width / other.width, this.height / other.height);
};

Size.prototype.inverse = function(other) {
  return new Size(1 / this.width, 1 / this.height);
};

Size.prototype.div = function(f) {
  for(let f of [...arguments]) {
    this.width /= f;
    this.height /= f;
  }

  return this;
};

Size.prototype.round = function(precision = (0.001, digits)) {
  let { width, height } = this;
  this.width = Util.roundTo(width, precision, digits);
  this.height = Util.roundTo(height, precision, digits);
  return this;
};

Size.prototype.bounds = function(other) {
  let w = [Math.min(this.width, other.width), Math.max(this.width, other.width)];
  let h = [Math.min(this.height, other.height), Math.max(this.height, other.height)];
  let scale = h / this.height;
  this.mul(scale);
  return this;
};

Size.prototype.fit = function(size) {
  size = new Size(size);
  let factors = Size.prototype.fitFactors.call(this, size);
  let ret = [Size.prototype.prod.call(this, factors[0]), Size.prototype.prod.call(this, factors[1])];
  return ret;
};

Size.prototype.fitHeight = function(other) {
  other = new Size(other);
  let scale = other.height / this.height;
  this.mul(scale);
  return [this.width, other.width];
};

Size.prototype.fitWidth = function(other) {
  other = new Size(other);
  let scale = other.width / this.width;
  this.mul(scale);
  return [this.height, other.height];
};

Size.prototype.fitFactors = function(other) {
  const hf = other.width / this.width;
  const vf = other.height / this.height;
  return [hf, vf];
};

Size.prototype.toString = function(opts = {}) {
  const { unit, separator, left, right } = opts;
  const { width, height } = this;
  return `${left}${width}${unit}${separator}${height}${unit}${right}`;
};
Size.area = sz => Size.prototype.area.call(sz);
Size.aspect = sz => Size.prototype.aspect.call(sz);

Size.bind = (o, p, gen) => {
  const [width, height] = p || ['width', 'height'];
  if(!gen) gen = k => v => (v === undefined ? o.k : (o.k = v));
  return Util.bindProperties(new Size(0, 0), o, { width, height }, gen);
};
for(let method of Util.getMethodNames(Size.prototype)) Size.method = (size, ...args) => Size.prototype.method.call(size || new Size(size), ...args);
const isSize = o => o && ((o.width !== undefined && o.height !== undefined) || (o.x !== undefined && o.x2 !== undefined && o.y !== undefined && o.y2 !== undefined) || (o.left !== undefined && o.right !== undefined && o.top !== undefined && o.bottom !== undefined));

for(let name of ['toCSS', 'isSquare', 'round', 'sum', 'add', 'diff', 'sub', 'prod', 'mul', 'quot', 'div']) {
  Size.name = (size, ...args) => Size.prototype.name.call(size || new Size(size), ...args);
}

Util.defineGetter(Size, Symbol.species, function() {
  return this;
});
const ImmutableSize = Util.immutableClass(Size);

Util.defineGetter(ImmutableSize, Symbol.species, function() {
  return ImmutableSize;
});

function Align(arg) {}
Align.CENTER = 0;
Align.LEFT = 1;
Align.RIGHT = 2;
Align.MIDDLE = 0;
Align.TOP = 4;
Align.BOTTOM = 8;
Align.horizontal = alignment => alignment & (Align.LEFT | Align.RIGHT);
Align.vertical = alignment => alignment & (Align.TOP | Align.BOTTOM);
const Anchor = Align;

function Rect(arg) {
  let obj = this instanceof Rect ? this : {};
  let args = arg instanceof Array ? arg : [...arguments];
  let ret;
  if(typeof args[0] == 'number') arg = args;
  else if(Util.isObject(args[0]) && args[0].length !== undefined) arg = args.shift();

  ['x', 'y', 'width', 'height'].forEach(field => {
    if(typeof obj.field != 'number') obj.field = 0;
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
  } else if(isPoint(arg) && arg.y !== undefined && arg.width !== undefined && arg.height !== undefined) {
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
    obj.x = 0;
    obj.y = 0;
    obj.width = typeof arg[0] === 'number' ? arg[0] : parseFloat(arg[0]);
    obj.height = typeof arg[1] === 'number' ? arg[1] : parseFloat(arg[1]);
    ret = 2;
  } else if(arg instanceof Array) {
    let argc;
    let argi = 0;

    if(arg.length >= 4) {
      argc = typeof x == 'number' ? 2 : 1;
      Point.apply(obj, arg.slice(0, argc));
      argi = argc;
    }

    argc = typeof arg.argi == 'number' ? 2 : 1;
    Size.apply(obj, arg.slice(argi, argc));
    ret = argi + argc;
  }

  if(typeof obj.x != 'number' || isNaN(obj.x)) obj.x = 0;
  if(typeof obj.y != 'number' || isNaN(obj.y)) obj.y = 0;
  if(typeof obj.width != 'number' || isNaN(obj.width)) obj.width = 0;
  if(typeof obj.height != 'number' || isNaN(obj.height)) obj.height = 0;
  return obj;
  if(!(this instanceof Rect) || new.target === undefined) return obj;
}
Rect.prototype = { ...Size.prototype, ...Point.prototype, ...Rect.prototype };

Rect.prototype.clone = function(fn) {
  const ctor = this.constructor[Symbol.species];
  let ret = new ctor(this.x, this.y, this.width, this.height);
  if(fn) fn(ret);
  return ret;
};

Rect.prototype.corners = function() {
  const rect = this;
  return [
    { x: rect.x, y: rect.y },
    { x: rect.x + rect.width, y: rect.y },
    { x: rect.x + rect.width, y: rect.y + rect.height },
    { x: rect.x, y: rect.y + rect.height }
  ];
};

if(Rect.prototype.isSquare === undefined) {
  Rect.prototype.isSquare = function() {
    return Math.abs(this.width - this.height) < 1;
  };
}
Rect.prototype.constructor = Rect;

Rect.prototype.getArea = function() {
  return this.width * this.height;
};

Rect.prototype.toString = function(opts = {}) {
  if(typeof opts == 'string') opts = { separator: opts };
  const { precision, unit, separator, left, right } = opts;
  return left + Point.prototype.toString.call(this, opts) + separator + Size.prototype.toString.call(this, opts) + right;
};

Rect.prototype.toSource = function(opts = {}) {
  const { color } = opts;
  const c = Util.coloring(color);
  const { x, y, width, height } = this;
  return c.concat(c.text('new', 1, 31), c.text('Rect', 1, 33), `(${x},${y},${width},${height})`);
};

Object.defineProperty(Rect.prototype, 'x1', {
  get() {
    return this.x;
  },
  set(value) {
    const extend = this.x - value;
    this.width += extend;
    this.x -= extend;
  },
  enumerable: true
});

Object.defineProperty(Rect.prototype, 'x2', {
  get() {
    return this.x + this.width;
  },
  set(value) {
    this.width = value - this.x;
  },
  enumerable: true
});

Object.defineProperty(Rect.prototype, 'y1', {
  get() {
    return this.y;
  },
  set(value) {
    const extend = this.y - value;
    this.height += extend;
    this.y -= extend;
  }
});

Object.defineProperty(Rect.prototype, 'y2', {
  get() {
    return this.y + this.height;
  },
  set(value) {
    this.height = value - this.y;
  }
});

Object.defineProperty(Rect.prototype, 'area', {
  get() {
    return Rect.prototype.getArea.call(this);
  }
});

Object.defineProperty(Rect.prototype, 'center', {
  get() {
    return Rect.center(this);
  }
});

Object.defineProperty(Rect.prototype, 'size', {
  get() {
    const rect = this;
    const size = new Size(rect.width, rect.height);

    Object.defineProperties(size, {
      width: {
        get() {
          return rect.width;
        },
        set(value) {
          return (rect.width = +value);
        },
        enumerable: true
      },
      height: {
        get() {
          return rect.height;
        },
        set(value) {
          return (rect.height = +value);
        },
        enumerable: true
      }
    });

    return size;
  }
});

Rect.prototype.points = function(ctor = items => Array.from(items)) {
  const c = this.corners();
  return ctor(c);
};
Rect.prototype.toCSS = Rect.toCSS;

Rect.prototype.scale = function(factor) {
  let width = this.width * factor;
  let height = this.height * factor;
  this.x += (width - this.width) / 2;
  this.y += (height - this.height) / 2;
  this.width = width;
  this.height = height;
  return this;
};

Rect.prototype.mul = function(...args) {
  Point.prototype.mul.call(this, ...args);
  Size.prototype.mul.call(this, ...args);
  return this;
};

Rect.prototype.div = function(...args) {
  Point.prototype.div.call(this, ...args);
  Size.prototype.div.call(this, ...args);
  return this;
};

Rect.prototype.outset = function(trbl) {
  if(typeof trbl == 'number') trbl = { top: trbl, right: trbl, bottom: trbl, left: trbl };
  this.x -= trbl.left;
  this.y -= trbl.top;
  this.width += trbl.left + trbl.right;
  this.height += trbl.top + trbl.bottom;
  return this;
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

Rect.prototype.inside = function(point) {
  return Rect.inside(this, point);
};
Rect.CONTAIN = 16;
Rect.COVER = 32;

Rect.prototype.fit = function(other, align = Align.CENTER | Align.MIDDLE | Rect.CONTAIN) {
  let factors = Size.prototype.fitFactors.call(this, new Size(other)).sort((a, b) => a - b);

  let rects = factors.reduce((acc, factor) => {
    let rect = new Rect(0, 0, this.width, this.height);
    rect.mul(factor);
    rect.align(other, align & 0x0f);
    acc.push(rect);
    return acc;
  }, []);

  return rects;
};

Rect.prototype.pointFromCenter = function(point) {
  Point.prototype.sub.call(point, this.center);
  point.x /= this.width;
  point.y /= this.height;
  return point;
};

Rect.prototype.toCSS = function() {
  return { ...Point.prototype.toCSS.call(this), ...Size.prototype.toCSS.call(this) };
};

Rect.prototype.toTRBL = function() {
  return { top: this.y, right: this.x + this.width, bottom: this.y + this.height, left: this.x };
};

Rect.prototype.toArray = function() {
  const { x, y, width, height } = this;
  return [x, y, width, height];
};

Rect.prototype.toPoints = function(ctor = points => Array.from(points)) {
  const { x, y, width, height } = this;
  return ctor([new Point(x, y), new Point(x + width, y), new Point(x + width, y + height), new Point(x, y + height)]);
};

Rect.prototype.toLines = function(ctor = lines => Array.from(lines, points => new Line(...points))) {
  let [a, b, c, d] = Rect.prototype.toPoints.call(this);
  return ctor([
    [a, b],
    [b, c],
    [c, d],
    [d, a]
  ]);
};

Rect.prototype.align = function(align_to, a = 0) {
  const xdiff = (align_to.width || 0) - this.width;
  const ydiff = (align_to.height || 0) - this.height;
  let oldx = this.x;
  let oldy = this.y;

  switch (Align.horizontal(a)) {
    case Align.LEFT:
      this.x = align_to.x;
      break;

    case Align.RIGHT:
      this.x = align_to.x + xdiff;
      break;

    default:
      this.x = align_to.x + xdiff / 2;
      break;
  }

  switch (Align.vertical(a)) {
    case Align.TOP:
      this.y = align_to.y;
      break;

    case Align.BOTTOM:
      this.y = align_to.y + ydiff;
      break;

    default:
      this.y = align_to.y + ydiff / 2;
      break;
  }

  return this;
};

Rect.prototype.round = function(precision = (0.001, digits, (type = 'round'))) {
  let { x1, y1, x2, y2 } = this.toObject(true);
  let a = new Point(x1, y1).round(precision, digits, type);
  let b = new Point(x2, y2).round(precision, null, type);
  this.x = a.x;
  this.y = a.y;
  this.width = +(b.x - this.x).toFixed(digits);
  this.height = +(b.y - this.y).toFixed(digits);
  return this;
};

Rect.prototype.toObject = function(bb = false) {
  if(bb) {
    const { x1, y1, x2, y2 } = this;
    return { x1, y1, x2, y2 };
  }

  const { x, y, width, height } = this;
  return { x, y, width, height };
};
Rect.round = rect => Rect.prototype.round.call(rect);
Rect.align = (rect, align_to, a = 0) => Rect.prototype.align.call(rect, align_to, a);
Rect.toCSS = rect => Rect.prototype.toCSS.call(rect);
Rect.inset = (rect, trbl) => Rect.prototype.inset.call(rect, trbl);
Rect.outset = (rect, trbl) => Rect.prototype.outset.call(rect, trbl);
Rect.center = rect => new Point(rect.x + rect.width / 2, rect.y + rect.height / 2);

Rect.bind = rect => {
  let obj = new Rect();
};

Rect.inside = (rect, point) => {
  return point.x >= rect.x && point.x <= rect.x + rect.width && point.y >= rect.y && point.y <= rect.y + rect.height;
};

Rect.from = function(obj) {
  const fn = (v1, v2) => [Math.min(v1, v2), Math.max(v1, v2)];
  const h = fn(obj.x1, obj.x2);
  const v = fn(obj.y1, obj.y2);
  const [x1, x2, y1, y2] = [...h, ...v];
  return new Rect({ x1, y1, x2, y2 });
};

Rect.fromCircle = function(...args) {
  const { x, y } = Point(args);
  const radius = args.shift();
  return new Rect(x - radius, y - radius, radius * 2, radius * 2);
};

for(let name of ['clone', 'corners', 'isSquare', 'getArea', 'toString', 'points', 'toCSS', 'toTRBL', 'toPoints']) {
  Rect.name = (rect, ...args) => Rect.prototype.name.call(rect || new Rect(rect), ...args);
}

Rect.toSource = (rect, opts = {}) => {
  const { sep, inner, spc, colon } = opts;
  let props = `x${colon}${spc}${rect.x}${sep}y${colon}${spc}${rect.y}${sep}width${colon}${spc}${rect.width}${sep}height${colon}${spc}${rect.height}`;
  if(inner) return props;
  return `{${sep}${props}${sep}}`;
};

Rect.bind = (o, p, gen) => {
  const [x, y, width, height] = p || ['x', 'y', 'width', 'height'];
  if(!gen) gen = k => v => (v === undefined ? o.k : (o.k = v));
  let pt = Point.bind(o, [x, y], gen);
  let sz = Size.bind(o, [width, height], gen);
  let proxy = new Rect(pt, sz);
  return proxy;
};
Rect.scale = Util.curry((rect, sx, sy) => Matrix.scale(sx, sy).transform_rect(rect));

Rect.resize = Util.curry((rect, width, height) => {
  rect.width = width;
  rect.height = height;
  return rect;
});
Rect.translate = Util.curry((rect, x, y) => Matrix.translate(f, f).transform_rect(rect));

for(let f of ['scale', 'resize', 'translate']) {
  Rect.prototype.f = function(...args) {
    Rect.f(this, ...args);
    return this;
  };
}
Util.defineInspect(Rect.prototype, 'x', 'y', 'width', 'height');
const isRect = rect => isPoint(rect) && isSize(rect);

Util.defineGetter(Rect, Symbol.species, function() {
  return this;
});
const ImmutableRect = Util.immutableClass(Rect);
delete ImmutableRect[Symbol.species];

Util.defineGetter(ImmutableRect, Symbol.species, function() {
  return ImmutableRect;
});

function TRBL(arg) {
  let ret = this instanceof TRBL ? this : {};
  let args = [...arguments];

  if(typeof arg === 'object' && !arg instanceof Array) {
    Object.keys(arg).forEach(k => {
      const matches = /(top|right|bottom|left)/i.exec(k);
      ret[matches[0].toLowerCase()] = parseInt(arg.k);
    });
  } else if(arg) {
    if(args.length > 1) arg = args;
    if(typeof arg === 'string') arg = [...arg.matchAll(/^[0-9.]+(|px|em|rem|pt|cm|mm)$/g)];
    else if(arg.length == 4) arg = arg.map(v => parseInt(v));
    ret.top = arg[0];
    ret.right = arg[1];
    ret.bottom = arg[2];
    ret.left = arg[3];
  }

  if(isNaN(ret.top)) ret.top = 0;
  if(isNaN(ret.right)) ret.right = 0;
  if(isNaN(ret.bottom)) ret.bottom = 0;
  if(isNaN(ret.left)) ret.left = 0;
  if(!this || this === TRBL) return Object.assign(ret, TRBL.prototype);
}

TRBL.prototype.null = function() {
  return this.top == 0 && this.right == 0 && this.bottom == 0 && this.left == 0;
};
TRBL.null = trbl => TRBL.prototype.null.call(trbl);
TRBL.neg = (trbl = this) => ({ top: -trbl.top, right: -trbl.right, bottom: -trbl.bottom, left: -trbl.left });

TRBL.prototype.isNaN = function() {
  return isNaN(this.top) || isNaN(this.right) || isNaN(this.bottom) || isNaN(this.left);
};

Object.defineProperty(TRBL.prototype, 'inset', {
  get() {
    return rect => Rect.inset(rect, this);
  }
});

Object.defineProperty(TRBL.prototype, 'outset', {
  get() {
    return rect => Rect.outset(rect, this);
  }
});

TRBL.prototype.add = function(other) {
  this.top += other.top;
  this.right += other.right;
  this.bottom += other.bottom;
  this.left += other.left;
};

TRBL.prototype.union = function(other) {
  this.top = other.top < this.top ? other.top : this.top;
  this.right = other.right > this.right ? other.right : this.right;
  this.bottom = other.bottom > this.bottom ? other.bottom : this.bottom;
  this.left = other.left < this.left ? other.left : this.left;
};

TRBL.prototype.toRect = function() {
  return new Rect({ x: this.left, y: this.top, width: this.right - this.left, height: this.bottom - this.top });
};

TRBL.prototype.toRect = function() {
  return new Rect({ x: this.left, y: this.top, width: this.right - this.left, height: this.bottom - this.top });
};
TRBL.union = (trbl, other) => ({ top: other.top < trbl.top ? other.top : trbl.top, right: other.right > trbl.right ? other.right : trbl.right, bottom: other.bottom > trbl.bottom ? other.bottom : trbl.bottom, left: other.left < trbl.left ? other.left : trbl.left });
TRBL.toRect = trbl => new Rect(trbl.left, trbl.top, trbl.right - trbl.left, trbl.bottom - trbl.top);

TRBL.prototype.toString = function(unit = 'px') {
  return '' + this.top + '' + unit + ' ' + this.right + '' + unit + ' ' + this.bottom + '' + unit + ' ' + this.left + unit;
};

TRBL.prototype.toSource = function() {
  return '{top:' + this.top + ',right:' + this.right + ',bottom:' + this.bottom + ',left:' + this.left + '}';
};

for(let name of ['null', 'isNaN', 'outset', 'toRect', 'toSource']) {
  TRBL.name = points => TRBL.prototype.name.call(points);
}

function isTRBL(obj) {
  return top in obj && right in obj && bottom in obj && left in obj;
}

Util.defineGetter(TRBL, Symbol.species, function() {
  return this;
});
const ImmutableTRBL = Util.immutableClass(TRBL);

Util.defineGetter(ImmutableTRBL, Symbol.species, function() {
  return ImmutableTRBL;
});

const SymSpecies = Util.tryCatch(
  () => Symbol,
  sym => sym.species
);

const CTOR = obj => {
  if(obj.SymSpecies) return obj.SymSpecies;
  let p = Object.getPrototypeOf(obj);
  if(p.SymSpecies) return p.SymSpecies;
  return p.constructor;
};

function Point(arg) {
  let args = arg instanceof Array ? arg : [...arguments];
  let p = this instanceof Point ? this : null;
  arg = args.shift();

  if(p === null) {
    if(arg instanceof Point) return arg;
    p = {};
  }

  if(typeof arg === 'undefined') {
    p.x = arg;
    p.y = args.shift();
  } else if(typeof arg === 'number') {
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

  if(p.x === undefined) p.x = 0;
  if(p.y === undefined) p.y = 0;
  if(isNaN(p.x)) p.x = undefined;
  if(isNaN(p.y)) p.y = undefined;

  if(!this || this === Point) {
    if(p.prototype == Object) p.prototype = Point.prototype;
    else Object.assign(p, Point.prototype);
    return p;
  }
}

Object.defineProperties(Point.prototype, {
  X: {
    get() {
      return this.x;
    }
  },
  Y: {
    get() {
      return this.y;
    }
  }
});

Point.prototype.move = function(x, y) {
  this.x += x;
  this.y += y;
  return this;
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

Point.prototype.set = function(fn) {
  if(typeof fn != 'function') {
    Point.apply(this, [...arguments]);
    return this;
  }

  return fn(this.x, this.y);
};

Point.prototype.clone = function() {
  const ctor = this[Symbol.species] || this.constructor[Symbol.species];
  return new ctor({ x: this.x, y: this.y });
};

Point.prototype.sum = function(...args) {
  const p = new Point(...args);
  let r = new Point(this.x, this.y);
  r.x += p.x;
  r.y += p.y;
  return r;
};

Point.prototype.add = function(...args) {
  const other = new Point(...args);
  this.x += other.x;
  this.y += other.y;
  return this;
};

Point.prototype.diff = function(...args) {
  const p = new Point(...args);
  let r = new Point(this.x, this.y);
  r.x -= p.x;
  r.y -= p.y;
  return r;
};

Point.prototype.sub = function(...args) {
  const other = new Point(...args);
  this.x -= other.x;
  this.y -= other.y;
  return this;
};

Point.prototype.prod = function(f) {
  const o = isPoint(f) ? f : { x: f, y: f };
  return new Point(this.x * o.x, this.y * o.y);
};

Point.prototype.mul = function(f) {
  const o = isPoint(f) ? f : { x: f, y: f };
  this.x *= o.x;
  this.y *= o.y;
  return this;
};

Point.prototype.quot = function(other) {
  other = isPoint(other) ? other : { x: other, y: other };
  return new Point(this.x / other.x, this.y / other.y);
};

Point.prototype.div = function(other) {
  other = isPoint(other) ? other : { x: other, y: other };
  this.x /= other.x;
  this.y /= other.y;
  return this;
};

Point.prototype.comp = function() {
  return new Point({ x: -this.x, y: -this.y });
};

Point.prototype.neg = function() {
  this.x *= -1;
  this.y *= -1;
  return this;
};

Point.prototype.distanceSquared = function(other = { x: 0, y: 0 }) {
  return (other.y - this.y) * (other.y - this.y) + (other.x - this.x) * (other.x - this.x);
};

Point.prototype.distance = function(other = { x: 0, y: 0 }) {
  return Math.sqrt(Point.prototype.distanceSquared.call(this, other));
};

Point.prototype.equals = function(other) {
  return +this.x == +other.x && +this.y == +other.y;
};

Point.prototype.round = function(precision = (0.001, digits, (type = 'round'))) {
  let { x, y } = this;
  this.x = Util.roundTo(x, precision, digits, type);
  this.y = Util.roundTo(y, precision, digits, type);
  return this;
};

Point.prototype.sides = function() {
  return { top: this.y, right: this.x + this.w1idth, bottom: this.y + this.height, left: this.x };
};

Point.prototype.dot = function(other) {
  return this.x * other.x + this.y * other.y;
};

Point.prototype.fromAngle = function(angle, dist = 1.0) {
  this.x = Math.cos(angle) * dist;
  this.y = Math.sin(angle) * dist;
  return this;
};

Point.prototype.toAngle = function(deg = false) {
  return Math.atan2(this.x, this.y) * (deg ? 180 / Math.PI : 1);
};

Point.prototype.angle = function(other, deg = false) {
  other = other || { x: 0, y: 0 };
  return Point.prototype.diff.call(this, other).toAngle(deg);
};

Point.prototype.rotate = function(angle, origin = { x: 0, y: 0 }) {
  this.x -= origin.x;
  this.y -= origin.y;
  let c = Math.cos(angle),
    s = Math.sin(angle);
  let xnew = this.x * c - this.y * s;
  let ynew = this.x * s + this.y * c;
  this.x = xnew;
  this.y = ynew;
  return this;
};

Point.prototype.dimension = function() {
  return [this.width, this.height];
};

Util.defineGetter(Point.prototype, Symbol.iterator, function() {
  const { x, y } = this;
  let a = [x, y];
  return a[Symbol.iterator].bind(a);
});

Point.prototype.toString = function(opts = {}) {
  const { precision, unit, separator, left, right } = opts;
  const x = Util.roundTo(this.x, precision);
  const y = Util.roundTo(this.y, precision);
  return `${left}${x}${unit}${separator}${y}${unit}${right}`;
};

Util.defineGetterSetter(
  Point.prototype,
  Symbol.toStringTag,
  function() {
    return `Point{ ${Point.prototype.toSource.call(this)}`;
  },
  () => {},
  false
);

Point.prototype.toSource = function(opts = {}) {
  const { asArray, pad, showNew } = opts;
  let x = pad(this.x + '');
  let y = pad(this.y + '');
  if(typeof this != 'object' || this === null) return '';
  if(asArray) return `[${x},${y}]`;
  return `${Util.colorText(showNew ? 'new ' : '', 1, 31)}${Util.colorText('Point', 1, 33)}${Util.colorText('(', 1, 36)}${Util.colorText(x, 1, 32)}${Util.colorText(',', 1, 36)}${Util.colorText(y, 1, 32)}${Util.colorText(')', 1, 36)}`;
};

Point.prototype.toObject = function() {
  const { x, y } = this;
  const obj = { x, y };
  Object.setPrototypeOf(obj, Point.prototype);
  return obj;
};

Point.prototype.toCSS = function(precision = 0.001) {
  return { left: Util.roundTo(this.x, precision) + 'px', top: Util.roundTo(this.y, precision) + 'px' };
};

Point.prototype.toFixed = function(digits) {
  return new Point(+this.x.toFixed(digits), +this.y.toFixed(digits));
};

Point.prototype.isNull = function() {
  return this.x == 0 && this.y == 0;
};

Point.prototype.inside = function(rect) {
  return this.x >= rect.x && this.x < rect.x + rect.width && this.y >= rect.y && this.y < rect.y + rect.height;
};

Point.prototype.normalize = function(minmax) {
  return new Point({ x: (this.x - minmax.x1) / (minmax.x2 - minmax.x1), y: (this.y - minmax.y1) / (minmax.y2 - minmax.y1) });
};
Point.move = (point, x, y) => Point.prototype.move.call(point, x, y);
Point.angle = (point, other, deg = false) => Point.prototype.angle.call(point, other, deg);
Point.inside = (point, rect) => Point.prototype.inside.call(point, rect);
Point.sub = (point, other) => Point.prototype.sub.call(point, other);
Point.prod = (a, b) => Point.prototype.prod.call(a, b);
Point.quot = (a, b) => Point.prototype.quot.call(a, b);

Point.equals = (a, b) => {
  let ret = Point.prototype.equals.call(a, b);
  return ret;
};
Point.round = (point, prec) => Point.prototype.round.call(point, prec);
Point.fromAngle = (angle, f) => Point.prototype.fromAngle.call(new Point(0, 0), angle, f);

for(let name of ['clone', 'comp', 'neg', 'sides', 'dimension', 'toString', 'toCSS', 'sub', 'diff', 'add', 'sum', 'distance']) {
  Point.name = (point, ...args) => Point.prototype.name.call(point || new Point(point), ...args);
}
Point.toSource = point => `{ x:${point.x}, y: ${point.y} }`;
const isPoint = o => o && ((o.x !== undefined && o.y !== undefined) || ((o.left !== undefined || o.right !== undefined) && (o.top !== undefined || o.bottom !== undefined)) || o instanceof Point || Object.getPrototypeOf(o).constructor === Point);
Point.isPoint = isPoint;
Util.defineInspect(Point.prototype, 'x', 'y');

Point.bind = (o, p, gen) => {
  const [x, y] = p || ['x', 'y'];
  if(!gen) gen = k => v => (v === undefined ? o.k : (o.k = v));
  return Util.bindProperties(new Point(0, 0), o, { x, y }, gen);
};
Point;

Util.defineGetter(Point, Symbol.species, function() {
  return this;
});
const ImmutablePoint = Util.immutableClass(Point);

Util.defineGetter(ImmutablePoint, Symbol.species, function() {
  return ImmutablePoint;
});

function Rect(arg) {
  let obj = this instanceof Rect ? this : {};
  let args = arg instanceof Array ? arg : [...arguments];
  let ret;
  if(typeof args[0] == 'number') arg = args;
  else if(Util.isObject(args[0]) && args[0].length !== undefined) arg = args.shift();

  ['x', 'y', 'width', 'height'].forEach(field => {
    if(typeof obj.field != 'number') obj.field = 0;
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
  } else if(isPoint(arg) && arg.y !== undefined && arg.width !== undefined && arg.height !== undefined) {
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
    obj.x = 0;
    obj.y = 0;
    obj.width = typeof arg[0] === 'number' ? arg[0] : parseFloat(arg[0]);
    obj.height = typeof arg[1] === 'number' ? arg[1] : parseFloat(arg[1]);
    ret = 2;
  } else if(arg instanceof Array) {
    let argc;
    let argi = 0;

    if(arg.length >= 4) {
      argc = typeof x == 'number' ? 2 : 1;
      Point.apply(obj, arg.slice(0, argc));
      argi = argc;
    }

    argc = typeof arg.argi == 'number' ? 2 : 1;
    Size.apply(obj, arg.slice(argi, argc));
    ret = argi + argc;
  }

  if(typeof obj.x != 'number' || isNaN(obj.x)) obj.x = 0;
  if(typeof obj.y != 'number' || isNaN(obj.y)) obj.y = 0;
  if(typeof obj.width != 'number' || isNaN(obj.width)) obj.width = 0;
  if(typeof obj.height != 'number' || isNaN(obj.height)) obj.height = 0;
  return obj;
  if(!(this instanceof Rect) || new.target === undefined) return obj;
}
Rect.prototype = { ...Size.prototype, ...Point.prototype, ...Rect.prototype };

Rect.prototype.clone = function(fn) {
  const ctor = this.constructor[Symbol.species];
  let ret = new ctor(this.x, this.y, this.width, this.height);
  if(fn) fn(ret);
  return ret;
};

Rect.prototype.corners = function() {
  const rect = this;
  return [
    { x: rect.x, y: rect.y },
    { x: rect.x + rect.width, y: rect.y },
    { x: rect.x + rect.width, y: rect.y + rect.height },
    { x: rect.x, y: rect.y + rect.height }
  ];
};

if(Rect.prototype.isSquare === undefined) {
  Rect.prototype.isSquare = function() {
    return Math.abs(this.width - this.height) < 1;
  };
}
Rect.prototype.constructor = Rect;

Rect.prototype.getArea = function() {
  return this.width * this.height;
};

Rect.prototype.toString = function(opts = {}) {
  if(typeof opts == 'string') opts = { separator: opts };
  const { precision, unit, separator, left, right } = opts;
  return left + Point.prototype.toString.call(this, opts) + separator + Size.prototype.toString.call(this, opts) + right;
};

Rect.prototype.toSource = function(opts = {}) {
  const { color } = opts;
  const c = Util.coloring(color);
  const { x, y, width, height } = this;
  return c.concat(c.text('new', 1, 31), c.text('Rect', 1, 33), `(${x},${y},${width},${height})`);
};

Object.defineProperty(Rect.prototype, 'x1', {
  get() {
    return this.x;
  },
  set(value) {
    const extend = this.x - value;
    this.width += extend;
    this.x -= extend;
  },
  enumerable: true
});

Object.defineProperty(Rect.prototype, 'x2', {
  get() {
    return this.x + this.width;
  },
  set(value) {
    this.width = value - this.x;
  },
  enumerable: true
});

Object.defineProperty(Rect.prototype, 'y1', {
  get() {
    return this.y;
  },
  set(value) {
    const extend = this.y - value;
    this.height += extend;
    this.y -= extend;
  }
});

Object.defineProperty(Rect.prototype, 'y2', {
  get() {
    return this.y + this.height;
  },
  set(value) {
    this.height = value - this.y;
  }
});

Object.defineProperty(Rect.prototype, 'area', {
  get() {
    return Rect.prototype.getArea.call(this);
  }
});

Object.defineProperty(Rect.prototype, 'center', {
  get() {
    return Rect.center(this);
  }
});

Object.defineProperty(Rect.prototype, 'size', {
  get() {
    const rect = this;
    const size = new Size(rect.width, rect.height);

    Object.defineProperties(size, {
      width: {
        get() {
          return rect.width;
        },
        set(value) {
          return (rect.width = +value);
        },
        enumerable: true
      },
      height: {
        get() {
          return rect.height;
        },
        set(value) {
          return (rect.height = +value);
        },
        enumerable: true
      }
    });

    return size;
  }
});

Rect.prototype.points = function(ctor = items => Array.from(items)) {
  const c = this.corners();
  return ctor(c);
};
Rect.prototype.toCSS = Rect.toCSS;

Rect.prototype.scale = function(factor) {
  let width = this.width * factor;
  let height = this.height * factor;
  this.x += (width - this.width) / 2;
  this.y += (height - this.height) / 2;
  this.width = width;
  this.height = height;
  return this;
};

Rect.prototype.mul = function(...args) {
  Point.prototype.mul.call(this, ...args);
  Size.prototype.mul.call(this, ...args);
  return this;
};

Rect.prototype.div = function(...args) {
  Point.prototype.div.call(this, ...args);
  Size.prototype.div.call(this, ...args);
  return this;
};

Rect.prototype.outset = function(trbl) {
  if(typeof trbl == 'number') trbl = { top: trbl, right: trbl, bottom: trbl, left: trbl };
  this.x -= trbl.left;
  this.y -= trbl.top;
  this.width += trbl.left + trbl.right;
  this.height += trbl.top + trbl.bottom;
  return this;
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

Rect.prototype.inside = function(point) {
  return Rect.inside(this, point);
};
Rect.CONTAIN = 16;
Rect.COVER = 32;

Rect.prototype.fit = function(other, align = Align.CENTER | Align.MIDDLE | Rect.CONTAIN) {
  let factors = Size.prototype.fitFactors.call(this, new Size(other)).sort((a, b) => a - b);

  let rects = factors.reduce((acc, factor) => {
    let rect = new Rect(0, 0, this.width, this.height);
    rect.mul(factor);
    rect.align(other, align & 0x0f);
    acc.push(rect);
    return acc;
  }, []);

  return rects;
};

Rect.prototype.pointFromCenter = function(point) {
  Point.prototype.sub.call(point, this.center);
  point.x /= this.width;
  point.y /= this.height;
  return point;
};

Rect.prototype.toCSS = function() {
  return { ...Point.prototype.toCSS.call(this), ...Size.prototype.toCSS.call(this) };
};

Rect.prototype.toTRBL = function() {
  return { top: this.y, right: this.x + this.width, bottom: this.y + this.height, left: this.x };
};

Rect.prototype.toArray = function() {
  const { x, y, width, height } = this;
  return [x, y, width, height];
};

Rect.prototype.toPoints = function(ctor = points => Array.from(points)) {
  const { x, y, width, height } = this;
  return ctor([new Point(x, y), new Point(x + width, y), new Point(x + width, y + height), new Point(x, y + height)]);
};

Rect.prototype.toLines = function(ctor = lines => Array.from(lines, points => new Line(...points))) {
  let [a, b, c, d] = Rect.prototype.toPoints.call(this);
  return ctor([
    [a, b],
    [b, c],
    [c, d],
    [d, a]
  ]);
};

Rect.prototype.align = function(align_to, a = 0) {
  const xdiff = (align_to.width || 0) - this.width;
  const ydiff = (align_to.height || 0) - this.height;
  let oldx = this.x;
  let oldy = this.y;

  switch (Align.horizontal(a)) {
    case Align.LEFT:
      this.x = align_to.x;
      break;

    case Align.RIGHT:
      this.x = align_to.x + xdiff;
      break;

    default:
      this.x = align_to.x + xdiff / 2;
      break;
  }

  switch (Align.vertical(a)) {
    case Align.TOP:
      this.y = align_to.y;
      break;

    case Align.BOTTOM:
      this.y = align_to.y + ydiff;
      break;

    default:
      this.y = align_to.y + ydiff / 2;
      break;
  }

  return this;
};

Rect.prototype.round = function(precision = (0.001, digits, (type = 'round'))) {
  let { x1, y1, x2, y2 } = this.toObject(true);
  let a = new Point(x1, y1).round(precision, digits, type);
  let b = new Point(x2, y2).round(precision, null, type);
  this.x = a.x;
  this.y = a.y;
  this.width = +(b.x - this.x).toFixed(digits);
  this.height = +(b.y - this.y).toFixed(digits);
  return this;
};

Rect.prototype.toObject = function(bb = false) {
  if(bb) {
    const { x1, y1, x2, y2 } = this;
    return { x1, y1, x2, y2 };
  }

  const { x, y, width, height } = this;
  return { x, y, width, height };
};
Rect.round = rect => Rect.prototype.round.call(rect);
Rect.align = (rect, align_to, a = 0) => Rect.prototype.align.call(rect, align_to, a);
Rect.toCSS = rect => Rect.prototype.toCSS.call(rect);
Rect.inset = (rect, trbl) => Rect.prototype.inset.call(rect, trbl);
Rect.outset = (rect, trbl) => Rect.prototype.outset.call(rect, trbl);
Rect.center = rect => new Point(rect.x + rect.width / 2, rect.y + rect.height / 2);

Rect.bind = rect => {
  let obj = new Rect();
};

Rect.inside = (rect, point) => {
  return point.x >= rect.x && point.x <= rect.x + rect.width && point.y >= rect.y && point.y <= rect.y + rect.height;
};

Rect.from = function(obj) {
  const fn = (v1, v2) => [Math.min(v1, v2), Math.max(v1, v2)];
  const h = fn(obj.x1, obj.x2);
  const v = fn(obj.y1, obj.y2);
  const [x1, x2, y1, y2] = [...h, ...v];
  return new Rect({ x1, y1, x2, y2 });
};

Rect.fromCircle = function(...args) {
  const { x, y } = Point(args);
  const radius = args.shift();
  return new Rect(x - radius, y - radius, radius * 2, radius * 2);
};

for(let name of ['clone', 'corners', 'isSquare', 'getArea', 'toString', 'points', 'toCSS', 'toTRBL', 'toPoints']) {
  Rect.name = (rect, ...args) => Rect.prototype.name.call(rect || new Rect(rect), ...args);
}

Rect.toSource = (rect, opts = {}) => {
  const { sep, inner, spc, colon } = opts;
  let props = `x${colon}${spc}${rect.x}${sep}y${colon}${spc}${rect.y}${sep}width${colon}${spc}${rect.width}${sep}height${colon}${spc}${rect.height}`;
  if(inner) return props;
  return `{${sep}${props}${sep}}`;
};

Rect.bind = (o, p, gen) => {
  const [x, y, width, height] = p || ['x', 'y', 'width', 'height'];
  if(!gen) gen = k => v => (v === undefined ? o.k : (o.k = v));
  let pt = Point.bind(o, [x, y], gen);
  let sz = Size.bind(o, [width, height], gen);
  let proxy = new Rect(pt, sz);
  return proxy;
};
Rect.scale = Util.curry((rect, sx, sy) => Matrix.scale(sx, sy).transform_rect(rect));

Rect.resize = Util.curry((rect, width, height) => {
  rect.width = width;
  rect.height = height;
  return rect;
});
Rect.translate = Util.curry((rect, x, y) => Matrix.translate(f, f).transform_rect(rect));

for(let f of ['scale', 'resize', 'translate']) {
  Rect.prototype.f = function(...args) {
    Rect.f(this, ...args);
    return this;
  };
}
Util.defineInspect(Rect.prototype, 'x', 'y', 'width', 'height');
const isRect = rect => isPoint(rect) && isSize(rect);

Util.defineGetter(Rect, Symbol.species, function() {
  return this;
});
const ImmutableRect = Util.immutableClass(Rect);
delete ImmutableRect[Symbol.species];

Util.defineGetter(ImmutableRect, Symbol.species, function() {
  return ImmutableRect;
});

function Size(arg) {
  let obj = this instanceof Size ? this : {};
  let args = [...arguments];

  if(args.length == 1 && Util.isObject(args[0]) && args[0].length !== undefined) {
    args = args[0];
    arg = args[0];
  }

  if(typeof arg == 'object') {
    if(arg.width !== undefined || arg.height !== undefined) {
      arg = args.shift();
      obj.width = arg.width;
      obj.height = arg.height;
    } else if(arg.x2 !== undefined && arg.y2 !== undefined) {
      arg = args.shift();
      obj.width = arg.x2 - arg.x;
      obj.height = arg.y2 - arg.y;
    } else if(arg.bottom !== undefined && arg.right !== undefined) {
      arg = args.shift();
      obj.width = arg.right - arg.left;
      obj.height = arg.bottom - arg.top;
    }
  } else {
    while(typeof arg == 'object' && (arg instanceof Array || 'length' in arg)) {
      args = [...arg];
      arg = args[0];
    }

    if(args && args.length >= 2) {
      let w = args.shift();
      let h = args.shift();
      if(typeof w == 'object' && 'baseVal' in w) w = w.baseVal.value;
      if(typeof h == 'object' && 'baseVal' in h) h = h.baseVal.value;
      obj.width = typeof w == 'number' ? w : parseFloat(w.replace(/[^-.0-9]*$/, ''));
      obj.height = typeof h == 'number' ? h : parseFloat(h.replace(/[^-.0-9]*$/, ''));
      Object.defineProperty(obj, 'units', { value: { width: typeof w == 'number' ? 'px' : w.replace(obj.width.toString(), ''), height: typeof h == 'number' ? 'px' : h.replace(obj.height.toString(), '') }, enumerable: false });
    }
  }

  if(isNaN(obj.width)) obj.width = undefined;
  if(isNaN(obj.height)) obj.height = undefined;
  if(!(obj instanceof Size)) return obj;
}
Size.prototype.width = NaN;
Size.prototype.height = NaN;
Size.prototype.units = null;

Size.prototype.convertUnits = function(w = 'window' in global ? window : null) {
  if(w === null) return this;
  const view = { vw: w.innerWidth, vh: w.innerHeight, vmin: w.innerWidth < w.innerHeight ? w.innerWidth : w.innerHeight, vmax: w.innerWidth > w.innerHeight ? w.innerWidth : w.innerHeight };

  if(view[this.units.width] !== undefined) {
    this.width = (this.width / 100) * view[this.units.width];
    delete this.units.width;
  }

  if(view[this.units.height] !== undefined) {
    this.height = (this.height / 100) * view[this.units.height];
    delete this.units.height;
  }

  return size;
};

Size.prototype.aspect = function() {
  return this.width / this.height;
};

Size.prototype.toCSS = function(units) {
  let ret = {};
  units = typeof units == 'string' ? { width: units, height: units } : units || this.units || { width: 'px', height: 'px' };
  if(this.width !== undefined) ret.width = this.width + (units.width || 'px');
  if(this.height !== undefined) ret.height = this.height + (units.height || 'px');
  return ret;
};

Size.prototype.transform = function(m) {
  this.width = m.xx * this.width + m.yx * this.height;
  this.height = m.xy * this.width + m.yy * this.height;
  return this;
};

Size.prototype.isSquare = function() {
  return Math.abs(this.width - this.height) < 1;
};

Size.prototype.area = function() {
  return this.width * this.height;
};

Size.prototype.resize = function(width, height) {
  this.width = width;
  this.height = height;
  return this;
};

Size.prototype.sum = function(other) {
  return new Size(this.width + other.width, this.height + other.height);
};

Size.prototype.add = function() {
  for(let other of [...arguments]) {
    this.width += other.width;
    this.height += other.height;
  }

  return this;
};

Size.prototype.diff = function(other) {
  return new Size(this.width - other.width, this.height - other.height);
};

Size.prototype.sub = function() {
  for(let other of [...arguments]) {
    this.width -= other.width;
    this.height -= other.height;
  }

  return this;
};

Size.prototype.prod = function(f) {
  const o = isSize(f) ? f : isPoint(f) ? { width: f.x, height: f.y } : { width: f, height: f };
  return new Size(this.width * o.width, this.height * o.height);
};

Size.prototype.mul = function(...args) {
  for(let f of args) {
    const o = isSize(f) ? f : isPoint(f) ? { width: f.x, height: f.y } : { width: f, height: f };
    this.width *= o.width;
    this.height *= o.height;
  }

  return this;
};

Size.prototype.quot = function(other) {
  return new Size(this.width / other.width, this.height / other.height);
};

Size.prototype.inverse = function(other) {
  return new Size(1 / this.width, 1 / this.height);
};

Size.prototype.div = function(f) {
  for(let f of [...arguments]) {
    this.width /= f;
    this.height /= f;
  }

  return this;
};

Size.prototype.round = function(precision = (0.001, digits)) {
  let { width, height } = this;
  this.width = Util.roundTo(width, precision, digits);
  this.height = Util.roundTo(height, precision, digits);
  return this;
};

Size.prototype.bounds = function(other) {
  let w = [Math.min(this.width, other.width), Math.max(this.width, other.width)];
  let h = [Math.min(this.height, other.height), Math.max(this.height, other.height)];
  let scale = h / this.height;
  this.mul(scale);
  return this;
};

Size.prototype.fit = function(size) {
  size = new Size(size);
  let factors = Size.prototype.fitFactors.call(this, size);
  let ret = [Size.prototype.prod.call(this, factors[0]), Size.prototype.prod.call(this, factors[1])];
  return ret;
};

Size.prototype.fitHeight = function(other) {
  other = new Size(other);
  let scale = other.height / this.height;
  this.mul(scale);
  return [this.width, other.width];
};

Size.prototype.fitWidth = function(other) {
  other = new Size(other);
  let scale = other.width / this.width;
  this.mul(scale);
  return [this.height, other.height];
};

Size.prototype.fitFactors = function(other) {
  const hf = other.width / this.width;
  const vf = other.height / this.height;
  return [hf, vf];
};

Size.prototype.toString = function(opts = {}) {
  const { unit, separator, left, right } = opts;
  const { width, height } = this;
  return `${left}${width}${unit}${separator}${height}${unit}${right}`;
};
Size.area = sz => Size.prototype.area.call(sz);
Size.aspect = sz => Size.prototype.aspect.call(sz);

Size.bind = (o, p, gen) => {
  const [width, height] = p || ['width', 'height'];
  if(!gen) gen = k => v => (v === undefined ? o.k : (o.k = v));
  return Util.bindProperties(new Size(0, 0), o, { width, height }, gen);
};
for(let method of Util.getMethodNames(Size.prototype)) Size.method = (size, ...args) => Size.prototype.method.call(size || new Size(size), ...args);
const isSize = o => o && ((o.width !== undefined && o.height !== undefined) || (o.x !== undefined && o.x2 !== undefined && o.y !== undefined && o.y2 !== undefined) || (o.left !== undefined && o.right !== undefined && o.top !== undefined && o.bottom !== undefined));

for(let name of ['toCSS', 'isSquare', 'round', 'sum', 'add', 'diff', 'sub', 'prod', 'mul', 'quot', 'div']) {
  Size.name = (size, ...args) => Size.prototype.name.call(size || new Size(size), ...args);
}

Util.defineGetter(Size, Symbol.species, function() {
  return this;
});
const ImmutableSize = Util.immutableClass(Size);

Util.defineGetter(ImmutableSize, Symbol.species, function() {
  return ImmutableSize;
});

function Align(arg) {}
Align.CENTER = 0;
Align.LEFT = 1;
Align.RIGHT = 2;
Align.MIDDLE = 0;
Align.TOP = 4;
Align.BOTTOM = 8;
Align.horizontal = alignment => alignment & (Align.LEFT | Align.RIGHT);
Align.vertical = alignment => alignment & (Align.TOP | Align.BOTTOM);
const Anchor = Align;

function iterator() {
  let done = false;
  let events = [];
  let resolve;
  let promise;
  defer();
  return [read(), write, end];

  function defer() {
    promise = new Promise(r => (resolve = r));
  }

  async function* read() {
    await promise;
    yield* events.splice(0);

    if(!done) yield* read();
  }

  function write(event) {
    events.push(event);
    resolve();
    defer();
  }

  function end() {
    done = true;
    resolve();
  }
}

function eventIterator(element, ...events) {
  let [iter, push, end] = iterator();
  events.forEach(name => element.addEventListener(name, push));

  iter.stop = () => {
    events.forEach(name => element.removeEventListener(name, push));
    end();
  };

  return iter;
}
iterator;

class Element extends Node {
  static EDGES = { upperLeft: 0, upperCenter: 0.5, upperRight: 1, centerRight: 1.5, lowerRight: 2, lowerCenter: 2.5, lowerLeft: 3, centerLeft: 3.5 };

  static edges = arg => Element.getEdgesXYWH(Element.rect(arg));
  static Axis = { H: 0, V: 2 };

  static margin = element => Element.getTRBL(element, 'margin');
  static padding = element => Element.getTRBL(element, 'padding');
  static border = element => Element.getTRBL(element, 'border');
  static wrap(e) {
    if(!this.methods) this.methods = Util.static({}, this, this, (k, fn) => k != 'wrap' && fn.length > 0);
    if(typeof e == 'string') e = Element.find(e);
    return Util.extend(e, this.methods);
  }

  static create() {
    let args = [...arguments];
    let { tagName, ns, children, ...props } = typeof args[0] == 'object' ? args.shift() : { tagName: args.shift(), ...args.shift() };
    let parent = args.shift();
    parent = typeof parent == 'string' ? Element.find(parent) : parent;
    let d = document || window.document;
    let e = ns ? d.createElementNS(ns, tagName) : d.createElement(tagName);

    for(let k in props) {
      const value = props.k;

      if(k == 'parent') {
        parent = props.k;
        continue;
      } else if(k == 'className') k = 'class';

      if(k == 'style' && typeof value === 'object') Element.setCSS(e, value);
      else if(k.startsWith('on') || k.startsWith('inner')) e.k = value;
      else e.setAttribute(k, value);
    }

    if(children && children.length) children.forEach(obj => Element.create(obj, e));
    if(parent && parent.appendChild) parent.appendChild(e);
    return e;
  }

  static walkUp(elem, pred = e => true) {
    if(typeof elem == 'string') elem = Element.find(elem);
    var depth = 0;

    if(typeof pred == 'number') {
      var n = pred;
      pred = (e, d) => d == n;
    }

    let ret = [];

    while(elem) {
      if(pred(elem, depth)) ret.push(elem);
      elem = elem.parentElement;
      depth++;
    }

    return ret.length ? ret : null;
  }

  static *skip(elem, fn = (e, next) => next(e.parentElement)) {
    elem = typeof elem == 'string' ? Element.find(elem) : elem;
    let emit = n => (elem = n);

    while(elem) {
      yield elem;

      fn(elem, emit);
    }
  }

  static walk(elem, fn, accu = {}) {
    if(typeof elem == 'string') elem = Element.find(elem);
    const root = elem;
    let depth = 0;

    while(elem) {
      accu = fn(this.wrap(elem), accu, root, depth);
      if(elem.firstElementChild) depth++;

      elem =
        elem.firstElementChild ||
        elem.nextElementSibling ||
        (function() {
          do {
            if(!(elem = elem.parentElement)) break;
            depth--;
          } while(depth > 0 && !elem.nextElementSibling);

          return elem && elem != root ? elem.nextElementSibling : null;
        })();
    }

    return accu;
  }

  static *iterator(elem, predicate = ((e, d, r) => true, getProp)) {
    if(getProp == 'node') getProp = (obj, prop) => obj[prop.replace(/Element$/, 'Node').replace(/Element/, '')];
    if(!getProp) getProp = (obj, prop) => obj.prop;
    if(typeof elem == 'string') elem = Element.find(elem);
    const root = elem;
    let depth = 0;

    while(elem) {
      if(predicate(elem, depth, root)) yield this.wrap(elem);

      if(getProp(elem, 'firstElementChild')) depth++;

      elem =
        getProp(elem, 'firstElementChild') ||
        getProp(elem, 'nextElementSibling') ||
        (function() {
          do {
            if(!(elem = getProp(elem, 'parentElement'))) break;
            depth--;
          } while(depth > 0 && !getProp(elem, 'nextElementSibling'));

          return elem && elem != root ? getProp(elem, 'nextElementSibling') : null;
        })();
    }
  }

  static *childIterator(elem) {
    if(elem.firstElementChild) {
      for(let c = elem.firstElementChild; c; c = c.nextElementSibling) yield c;
    } else {
      let children = [...elem.children];

      for(let i = 0; i < children.length; i++) yield children.i;
    }
  }

  static toObject(elem, opts = { children: true }) {
    elem = Element.find(elem);
    let children = [];

    if(opts.children) {
      [...this.childIterator(elem)].forEach(c => (Util.isObject(c) && 'tagName' in c ? children.push(Element.toObject(c, elem)) : (c.textContent + '').trim() != '' ? children.push(c.textContent) : undefined));
    }

    let attributes = (opts ? opts.namespaceURI : document.body.namespaceURI) != elem.namespaceURI ? { ns: elem.namespaceURI } : {};
    let a = 'length' in elem.attributes ? Element.attr(elem) : elem.attributes;
    for(let key in a) attributes.key = '' + a.key;
    return { tagName: /[a-z]/.test(elem.tagName) ? elem.tagName : elem.tagName.toLowerCase(), ...attributes, ...(children.length > 0 ? { children } : {}) };
  }

  static toCommand(elem, opts = {}) {
    let { parent, varName, recursive, cmd, quote } = opts;
    let o = Element.toObject(elem, { children: false });
    let s = '';
    let { tagName, ns, children, ...attributes } = o;
    let v = '';
    s = Object.keys(ns ? { ns, ...attributes } : attributes)
      .map(k => `${k}:${quote}${attributes[k]}${quote}`)
      .join(', ');
    s = `${cmd}('${tagName}', {${s}}`;
    let c = elem.children;
    if(c.length >= 1) s = `${s}, [\n  ${c.map(e => Element.toCommand(e, opts).replace(/\n/g, '\n  ')).join(',\n  ')}\n]`;
    s += parent ? ', ' + parent : '';

    if(elem.firstElementChild && varName) {
      v = parent ? String.fromCharCode(parent.charCodeAt(0) + 1) : varName;
      s = `${v} = ${s}`;
    }

    return s.replace(new RegExp(';*$', 'g'), '');
  }

  static find(arg, parent, globalObj = Util.getGlobalObject()) {
    if(typeof parent == 'string') parent = Element.find(parent);
    if(!parent && globalObj.document) parent = globalObj.document;
    if(typeof arg != 'string') throw new Error(arg + '');

    if(arg.startsWith('/')) arg = arg.substring(1).replace(/\//g, ' > ');
    return parent.querySelector(arg);
  }

  static findAll(arg, parent) {
    parent = typeof parent == 'string' ? Element.find(parent) : parent;
    return [...(parent && parent.querySelectorAll ? parent.querySelectorAll(arg) : document.querySelectorAll(arg))];
  }

  static attr(e, attrs_or_name) {
    const elem = typeof e === 'string' ? Element.find(e) : e;

    if(!Util.isArray(attrs_or_name) && typeof attrs_or_name === 'object' && elem) {
      for(let key in attrs_or_name) {
        const name = Util.decamelize(key, '-');
        const value = attrs_or_name.key;
        if(key.startsWith('on') && !/svg/.test(elem.namespaceURI)) elem.key = value;
        else if(elem.setAttribute) elem.setAttribute(name, value);
        else elem.key = value;
      }

      return elem;
    }

    if(typeof attrs_or_name === 'function') {
      attrs_or_name(elem.attributes, elem);
      return elem;
    } else if(typeof attrs_or_name === 'string') {
      attrs_or_name = [attrs_or_name];
    } else if(Util.isObject(elem) && 'getAttributeNames' in elem) {
      attrs_or_name = elem.getAttributeNames();
    } else {
      attrs_or_name = [];

      if(Util.isObject(elem) && Util.isArray(elem.attributes)) for(let i = 0; i < elem.attributes.length; i++) attrs_or_name.push(elem.attributes.i.name);
    }

    let ret = attrs_or_name.reduce((acc, name) => {
      const key = name;
      const value = elem && elem.getAttribute ? elem.getAttribute(name) : elem.key;
      acc.key = /^-?[0-9]*\.[0-9]\+$/.test(value) ? parseFloat(value) : value;
      return acc;
    }, {});

    if(typeof arguments[1] == 'string') return ret[attrs_or_name[0]];
    return ret;
  }

  static getRect(elem) {
    let e = elem;

    while(e) {
      if(e.style) {
        if(e.style.position == '') e.style.position = 'relative';
        if(e.style.left == '') e.style.left = '0px';
        if(e.style.top == '') e.style.top = '0px';
      }

      e = e.offsetParent || e.parentNode;
    }

    const bbrect = elem.getBoundingClientRect();
    return { x: bbrect.left + window.scrollX, y: bbrect.top + window.scrollY, width: bbrect.right - bbrect.left, height: bbrect.bottom - bbrect.top };
  }

  static rect(elem, options = {}) {
    let args = [...arguments];
    let element = args.shift();
    if(args.length > 0 && (isRect(args) || isRect(args[0]))) return Element.setRect.apply(Element, arguments);
    let { round, relative_to, relative, scroll_offset } = options;
    const e = typeof element === 'string' ? Element.find(element) : element;

    if(!e || !e.getBoundingClientRect) {
      return null;
    }

    const bb = e.getBoundingClientRect();
    let r = TRBL.toRect(bb);
    if(relative) relative_to = e.parentElement;

    if(relative_to && relative_to !== null) {
      const off = Element.rect(relative_to);
      r.x -= off.x;
      r.y -= off.y;
    }

    if(options.border) {
      const border = Element.border(e);
      Rect.outset(r, border);
    }

    const { scrollTop, scrollY } = window;

    if(scroll_offset) {
      r.y += scrollY;
    }

    r = new Rect(round ? Rect.round(r) : r);
    return r;
  }

  static setRect(element, rect, opts = {}) {
    let { anchor, unit, scale } = opts;
    const e = typeof element === 'string' ? Element.find(element) : element;

    if(typeof anchor == 'string') {
      e.style.position = anchor;
      anchor = 0;
    }

    if(scale) Rect.scale(rect, scale, scale);
    anchor = anchor || Anchor.LEFT | Anchor.TOP;
    const position = element.style && element.style.position;
    const pelement = position == 'fixed' ? e.documentElement || document.body : e.parentNode;
    const prect = Element.rect(pelement, { round: false });
    const ptrbl = Rect.toTRBL(prect);
    const trbl = Rect.toTRBL(rect);
    let css = {};
    let remove;

    switch (Anchor.horizontal(anchor)) {
      case Anchor.LEFT:
      default:
        css.left = Math.round(trbl.left) + unit;
        remove = 'right';
        break;

      case Anchor.RIGHT:
        css.right = Math.round(trbl.right - ptrbl.right) + unit;
        remove = 'left';
        break;
    }

    switch (Anchor.vertical(anchor)) {
      case Anchor.TOP:
      default:
        css.top = Math.round(trbl.top) + unit;
        remove = 'bottom';
        break;

      case Anchor.BOTTOM:
        css.bottom = Math.round(trbl.bottom - ptrbl.bottom) + unit;
        remove = 'top';
        break;
    }

    if(e.style) {
      if(e.style.removeProperty) e.style.removeProperty(remove);
      else e.style.remove = undefined;
    }

    css.width = Math.round(rect.width) + (unit || unit);
    css.height = Math.round(rect.height) + (unit || unit);
    Element.setCSS(e, css);
    return e;
  }

  static position(element, pos = 'absolute') {
    if(typeof element == 'string') element = Element.find(element);
    const { x, y } = element.getBoundingClientRect();
    return new Point({ x, y });
  }

  static move(element, point, pos) {
    let [e, ...rest] = [...arguments];
    let { x, y } = new Point(rest);
    let to = { x, y };
    let position = rest.shift() || Element.getCSS(element, 'position') || 'relative';
    let off;

    const getValue = prop => {
      const property = Element.getCSS(element, prop);
      if(property === undefined) return undefined;
      const matches = /([-0-9.]+)(.*)/.exec(property) || [];
      return parseFloat(matches[1]);
    };

    const current = new Point({ x: getValue('left') || 0, y: getValue('top') || 0 });
    off = new Point(Element.rect(element, { round: false }));
    Point.add(current, Point.diff(to, off));
    let css = Point.toCSS(current);
    Element.setCSS(element, { ...css, position });
    return element;
  }

  static moveRelative(element, to, position) {
    var e = typeof element == 'string' ? Element.find(element) : element;
    var pos = Object.freeze(new Rect(to || Element.rect(e)));

    function move(x, y) {
      let rect = new Rect(pos.x + x, pos.y + y, pos.width, pos.height);
      move.last = rect;
      return Element.move(e, rect, position);
    }

    move.pos = pos;
    move.cancel = () => move(0, 0);
    move.jump = () => Element.moveRelative(e);
    return move;
  }

  static resize(element, ...dimensions) {
    let e = typeof element == 'string' ? Element.find(element) : element;
    let size = new Size(...dimensions);
    const css = Size.toCSS(size);
    Element.setCSS(e, css);
    return e;
  }

  static getEdgesXYWH({ x, y, w, h }) {
    return [
      { x, y },
      { x: x + w, y },
      { x: x + w, y: y + h },
      { x, y: y + h }
    ];
  }

  static getEdge({ x, y, w, h }, which) {
    return [
      { x, y },
      { x: x + w / 2, y },
      { x: x + w, y },
      { x: x + w, y: y + h / 2 },
      { x: x + w, y: y + h },
      { x: x + w / 2, y: y + h },
      { x, y: y + h },
      { x, y: y + h / 2 }
    ][Math.floor(which * 2)];
  }

  static getPointsXYWH({ x, y, w, h }) {
    return [
      { x, y },
      { x: x + w, y: y + h }
    ];
  }

  static cumulativeOffset(element, relative_to = null) {
    if(typeof element == 'string') element = Element.find(element);
    let p = { x: 0, y: 0 };

    do {
      p.y += element.offsetTop || 0;
      p.x += element.offsetLeft || 0;
    } while((element = element.offsetParent) && element != relative_to);

    return p;
  }

  static getTRBL(element, prefix = '') {
    const names = ['Top', 'Right', 'Bottom', 'Left'].map(pos => prefix + (prefix == '' ? pos.toLowerCase() : pos + (prefix == 'border' ? 'Width' : '')));
    return new TRBL(Element.getCSS(element, names));
  }

  static setTRBL(element, trbl, prefix = 'margin') {
    const attrs = ['Top', 'Right', 'Bottom', 'Left'].reduce((acc, pos) => {
      const name = prefix + (prefix == '' ? pos.toLowerCase() : pos);
      return { ...acc, [[name]]: trbl[pos.toLowerCase()] };
    }, {});

    return Element.setCSS(element, attrs);
  }

  static setCSS(element, prop, value) {
    if(typeof element == 'string') element = Element.find(element);
    if(!isElement(element)) return false;
    if(typeof prop == 'string' && typeof value == 'string') prop = { [[prop]]: value };

    for(let key in prop) {
      let value = prop.key;
      const propName = Util.decamelize(key);

      if(typeof value == 'function') {
        if('subscribe' in value) {
          value.subscribe = newval => element.style.setProperty(propName, newval);
          value = value();
        }
      }

      if(element.style) {
        if(element.style.setProperty) element.style.setProperty(propName, value);
        else element.style[Util.camelize(propName)] = value;
      }
    }

    return element;
  }

  static getCSS(element, property = (undefined, (receiver = null))) {
    element = typeof element == 'string' ? Element.find(element) : element;
    const w = window !== undefined ? window : global.window;
    const d = document !== undefined ? document : global.document;
    let parent = Util.isObject(element) ? element.parentElement || element.parentNode : null;
    let estyle = Util.tryPredicate(() => (Util.isObject(w) && w.getComputedStyle ? w.getComputedStyle(element) : d.getComputedStyle(element)), null);
    let pstyle = Util.tryPredicate(() => (parent && parent.tagName ? (w && w.getComputedStyle ? w.getComputedStyle(parent) : d.getComputedStyle(parent)) : {}), null);
    if(!estyle || !pstyle) return null;
    let style = Util.tryPredicate(() => Util.removeEqual(estyle, pstyle), null);
    if(!style) return null;
    let keys = Object.keys(style).filter(k => !/^__/.test(k));
    let ret = {};

    if(receiver == null) {
      receiver = result => {
        if(typeof result == 'object') {
          try {
            Object.defineProperty(result, 'cssText', {
              get() {
                return Object.entries(this)
                  .map(([k, v]) => `${Util.decamelize(k, '-')}: ${v};\n`)
                  .join('');
              },
              enumerable: false
            });
          } catch(err) {}
        }

        return result;
      };
    }

    if(property !== undefined) {
      ret =
        typeof property === 'string'
          ? style.property
          : property.reduce((ret, key) => {
              ret.key = style.key;
              return ret;
            }, {});
    } else {
      for(let i = 0; i < keys.length; i++) {
        const stylesheet = keys.i;
        const key = Util.camelize(stylesheet);
        const val = style.stylesheet || style.key;
        if(val && val.length > 0 && val != 'none') ret.key = val;
      }
    }

    return receiver(ret);
  }

  static xpath(elt, relative_to = null) {
    let path = '';
    for(let e of this.skip(elt, (e, next) => next(e.parentElement !== relative_to && e.parentElement))) path = '/' + Element.unique(e) + path;
    return path;
  }

  static selector(elt, opts = {}) {
    const { relative_to, use_id } = opts;
    let sel = '';

    for(; elt && elt.nodeType == 1; elt = elt.parentNode) {
      if(sel != '') sel = ' > ' + sel;
      let xname = Element.unique(elt, { idx: false, use_id });
      if(use_id === false) xname = xname.replace(/#.*/g, '');
      sel = xname + sel;
      if(elt == relative_to) break;
    }

    return sel;
  }

  static depth(elem, relative_to = document.body) {
    let count = 0;
    while(elem != relative_to && (elem = elem.parentNode)) count++;
    return count;
  }

  static dump(elem) {
    let str = '';

    function dumpElem(child, accu, root, depth) {
      const rect = Rect.round(Element.rect(child, elem));
      accu += '  '.repeat((depth > 0 ? depth : 0) + 1) + ' ' + Element.xpath(child, child);
      [...child.attributes].forEach(attr => (accu += ' ' + attr.name + "='" + attr.value + "'"));
      if(Rect.area(rect) > 0) accu += ' ' + Rect.toString(rect);

      ['margin', 'border', 'padding'].forEach(name => {
        let trbl = Element.getTRBL(elem, 'margin');
        if(!trbl.null()) accu += ' ' + name + ': ' + trbl + '';
      });

      return accu;
    }

    str = dumpElem(elem, '');

    str = Element.walk(
      elem.firstElementChild,
      (e, a, r, d) => {
        if(e && e.attributes) return dumpElem(e, a + '\n', r, d);
        return null;
      },
      str
    );

    return str;
  }

  static skipper(fn, pred = (a, b) => a.tagName == b.tagName) {
    return function(elem) {
      let next = fn(elem);

      for(; next; next = fn(next)) if(pred(elem, next)) return next;

      return null;
    };
  }

  static prevSibling(sib) {
    return sib.previousElementSibling;
  }

  static nextSibling(sib) {
    return sib.nextElementSibling;
  }

  static idx(elt) {
    let count = 1;
    let sib = elt.previousElementSibling;

    for(; sib; sib = sib.previousElementSibling) {
      if(sib.tagName == elt.tagName) count++;
    }

    return count;
  }

  static name(elem) {
    let name = elem.tagName.toLowerCase();
    if(elem.id && elem.id.length) name += '#' + elem.id;
    else if(elem.class && elem.class.length) name += '.' + elem.class;
    return name;
  }

  static unique(elem, opts = {}) {
    const { idx, use_id } = opts;
    let name = elem.tagName.toLowerCase();
    if(use_id && elem.id && elem.id.length) return name + '#' + elem.id;
    const classNames = [...elem.classList];

    for(let i = 0; i < classNames.length; i++) {
      let res = document.getElementsByClassName(classNames.i);
      if(res && res.length === 1) return name + '.' + classNames.i;
    }

    if(idx) {
      if(elem.nextElementSibling || elem.previousElementSibling) {
        return name + '[' + Element.idx(elem) + ']';
      }
    }

    return name;
  }

  static factory(delegate = ({}, (parent = null))) {
    let root = parent;

    if(root === null) {
      if(typeof delegate.append_to !== 'function') {
        root = delegate;
        delegate = {};
      } else {
        root = 'body';
      }
    }

    const { append_to, create, setattr, setcss } = delegate;
    if(typeof root === 'string') root = Element.find(root);
    if(!delegate.root) delegate.root = root;

    if(!delegate.append_to) {
      delegate.append_to = function(elem, parent) {
        if(!parent) parent = root;
        if(parent) parent.appendChild(elem);
        if(!this.root) this.root = elem;
      };
    }

    if(!delegate.create) delegate.create = tag => document.createElement(tag);

    if(!delegate.setattr) {
      delegate.setattr = (elem, attr, value) => {
        elem.setAttribute(attr, value);
      };
    }

    if(!delegate.setcss) delegate.setcss = (elem, css) => Object.assign(elem.style, css);

    delegate.bound_factory = (tag, attr = ({}, (parent = null))) => {
      if(typeof tag == 'object') {
        const { tagName, ...a } = tag;
        attr = a;
        tag = tagName;
      }

      const { style, children, className, innerHTML, ...props } = attr;
      let elem = delegate.create(tag);
      if(style) delegate.setcss(elem, style);

      if(children && children.length) {
        for(let i = 0; i < children.length; i++) {
          if(typeof children.i === 'string') {
            elem.innerHTML += children.i;
          } else {
            const { tagName, parent, ...childProps } = children.i;
            delegate.bound_factory(tagName, childProps, elem);
          }
        }
      }

      if(innerHTML) elem.innerHTML += innerHTML;

      if(className && elem) {
        if(elem.classList) elem.classList.add(className);
        else if(elem.attributes['class']) elem.attributes['class'].value += ' ' + className;
      }

      for(let k in props) delegate.setattr(elem, k, props.k);
      if(delegate.append_to) delegate.append_to(elem, parent);
      return elem;
    };

    delegate.bound_factory.delegate = delegate;
    return delegate.bound_factory;
  }

  static remove(element) {
    const e = typeof element === 'string' ? Element.find(element) : element;

    if(e && e.parentNode) {
      const parent = e.parentNode;
      parent.removeChild(e);
      return true;
    }

    return false;
  }

  static isat(e, x, y, options) {
    let args = [...arguments];
    let element = args.shift();
    let point = Point(args);
    const o = args[0] || { round: false };
    const rect = Element.rect(element, o);
    return Rect.inside(rect, point);
  }

  static at(x, y, options) {
    if(isElement(x)) return Element.isat.apply(Element, arguments);
    let args = [...arguments];
    const p = Point(args);
    const w = global.window;
    const d = w.document;

    const s = o.all
      ? e => {
          if(ret == null) ret = [];
          ret.push(e);
        }
      : (e, depth) => {
          e.depth = depth;
          if(ret === null || depth >= ret.depth) ret = e;
        };

    let ret = null;

    return new Promise((resolve, reject) => {
      let element = null;

      Element.walk(d.body, (e, accu, root, depth) => {
        const r = Element.rect(e, { round: true });
        if(Rect.area(r) == 0) return;
        if(Rect.inside(r, p)) s(e, depth);
      });

      if(ret !== null) resolve(ret);
      else reject();
    });
  }

  static transition(element, css, time, easing = ('linear', (callback = null))) {
    let args = [...arguments];
    const e = typeof element === 'string' ? Element.find(args.shift()) : args.shift();
    let a = [];
    const t = typeof time == 'number' ? `${time}ms` : time;
    let ctx = { e, t, from: {}, to: {}, css };
    args.shift();
    args.shift();
    easing = typeof args[0] == 'function' ? 'linear' : args.shift();
    callback = args.shift();

    for(let prop in css) {
      const name = Util.decamelize(prop);
      a.push(`${name} ${t} ${easing}`);
      ctx.from.prop = e.style.getProperty ? e.style.getProperty(name) : e.style.prop;
      ctx.to.name = css.prop;
    }

    const tlist = a.join(', ');
    var cancel;

    let ret = new Promise((resolve, reject) => {
      var trun = function(e) {
        this.event = e;
        callback(this);
      };

      var tend = function(e) {
        this.event = e;
        this.e.removeEventListener('transitionend', this);
        this.e.style.setProperty('transition', '');
        delete this.cancel;
        resolve(this);
      };

      e.addEventListener('transitionend', (ctx.cancel = tend).bind(ctx));
      if(typeof callback == 'function') e.addEventListener('transitionrun', (ctx.run = trun).bind(ctx));
      cancel = () => ctx.cancel();
      if(e.style && e.style.setProperty) e.style.setProperty('transition', tlist);
      else e.style.transition = tlist;
      Object.assign(e.style, css);
    });

    ret.cancel = cancel;
    return ret;
  }

  static toString(e) {
    let o = e.__proto__ === Object.prototype ? e : Element.toObject(e);
    const { tagName, ns, children, ...a } = o;
    let s = `<${tagName}`;
    s += Object.entries(a)
      .map(([name, value]) => ` ${name}="${value}"`)
      .join('');
    s += children.length ? `>` : ` />`;
    if(children.length) s += children.map(Element.toString).join('') + `</${tagName}`;
    return s;
  }

  static clipboardCopy = text =>
    new Promise((resolve, reject) => {
      if(navigator.clipboard) {
        return navigator.clipboard
          .writeText(text)
          .then(() => resolve(true))
          .catch(err => reject(err !== undefined ? err : new DOMException('The request is not allowed', 'NotAllowedError')));
      }

      let ok = false;

      try {
        const d = document,
          w = window;
        let e = d.createElement('e');
        e.textContent = text;
        e.style.whiteSpace = 'pre';
        d.body.appendChild(e);
        let s = w.getSelection();
        let r = w.d.createRange();
        s.removeAllRanges();
        r.selectNode(e);
        s.addRange(r);

        try {
          ok = w.d.execCommand('copy');
        } catch(err) {
          console.log('error', err);
        }

        s.removeAllRanges();
        w.d.body.removeChild(e);
      } catch(err) {}

      if(ok) resolve(ok);
      else reject(new DOMException('The request is not allowed', 'NotAllowedError'));
    });
}

Element.children = function*(elem, tfn = e => e) {
  if(typeof elem == 'string') elem = Element.find(elem);

  for(let e = elem.firstElementChild; e; e = e.nextElementSibling) yield tfn(e);
};

Element.recurse = function*(elem, tfn = e => e) {
  if(typeof elem == 'string') elem = Element.find(elem);
  let root = elem;

  do {
    elem =
      elem.firstElementChild ||
      elem.nextElementSibling ||
      (function() {
        do {
          if(!(elem = elem.parentElement)) break;
        } while(!elem.nextSibling);

        return elem && elem != root ? elem.nextElementSibling : null;
      })();

    if(elem !== null) yield tfn(elem);
  } while(elem);
};

function isElement(e) {
  return Util.isObject(e) && e.tagName !== undefined;
}
