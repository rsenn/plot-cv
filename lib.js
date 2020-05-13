export function Point(arg) {
  let args = arg instanceof Array ? arg : [...arguments];
  let p = !this || this === Point ? {} : this;
  arg = args.shift();
  if(typeof arg === "number") {
    p.x = parseFloat(arg);
    p.y = parseFloat(args.shift());
  } else if(typeof arg === "string") {
    const matches = [...arg.matchAll(new RegExp("/([-+]?d*.?d+)(?:[eE]([-+]?d+))?/g"))];
    p.x = parseFloat(matches[0]);
    p.y = parseFloat(matches[1]);
  } else if(typeof arg == "object" && arg !== null && (arg.x !== undefined || arg.y !== undefined)) {
    p.x = arg.x;
    p.y = arg.y;
  } else if(typeof arg == "object" && arg !== null && arg.length > 0 && x !== undefined && y !== undefined) {
    p.x = parseFloat(arg.shift());
    p.y = parseFloat(arg.shift());
  } else if(typeof args[0] === "number" && typeof args[1] === "number") {
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
Point.prototype.move = function(x, y) {
  this.x += x;
  this.y += y;
  return this;
};
Point.prototype.set = function(fn) {
  if(typeof fn != "function") {
    Point.apply(this, [...arguments]);
    return this;
  }
  return fn(this.x, this.y);
};
Point.prototype.clone = function() {
  return new Point({ x: this.x, y: this.y });
};
Point.prototype.sum = function(other) {
  return new Point(this.x + other.x, this.y + other.y);
};
Point.prototype.add = function(other) {
  this.x += other.x;
  this.y += other.y;
  return this;
};
Point.prototype.diff = function(other) {
  return new Point(this.x - other.x, this.y - other.y);
};
Point.prototype.sub = function(other) {
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
  return new Point(this.x / other.x, this.y / other.y);
};
Point.prototype.div = function(f) {
  this.x /= f;
  this.y /= f;
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
Point.prototype.distance = function(other = { x: 0, y: 0 }) {
  return Math.sqrt((other.y - this.y) * (other.y - this.y) + (other.x - this.x) * (other.x - this.x));
};
Point.prototype.equal = function(other) {
  return this.x == other.x && this.y == other.y;
};
Point.prototype.round = function(precision = 1.0) {
  this.x = Math.round(this.x / precision) * precision;
  this.y = Math.round(this.y / precision) * precision;
  return this;
};
Point.prototype.sides = function() {
  return {
    top: this.y,
    right: this.x + this.w1idth,
    bottom: this.y + this.height,
    left: this.x
  };
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
Point.prototype.angle = function(p2 = { x: 0, y: 0 }) {
  return Point.prototype.diff.call(p1, p2).toAngle();
};
Point.prototype.dimension = function() {
  return [this.width, this.height];
};
Point.prototype.toString = function(asArray = false) {
  if(typeof this != "object" || this === null) return "";
  if(asArray) return `[${this.x},${this.y}]`;
  return `{x:${this.x},y:${this.y}}`;
};
Point.prototype.toSource = function() {
  return "{x:" + this.x.toFixed(3) + ",y:" + this.y.toFixed(3) + "}";
};
Point.prototype.toCSS = function() {
  return {
    left: this.x + "px",
    top: this.y + "px"
  };
};
Point.prototype.inside = function(rect) {
  return this.x >= rect.x && this.x < rect.x + rect.width && this.y >= rect.y && this.y < rect.y + rect.height;
};
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
export const isPoint = o => o && ((o.x !== undefined && o.y !== undefined) || ((o.left !== undefined || o.right !== undefined) && (o.top !== undefined || o.bottom !== undefined)));
Point.isPoint = isPoint;

export function Size(arg) {
  let obj = this instanceof Size ? this : {};
  let args = [...arguments];
  if(args.length == 1 && args[0].length !== undefined) {
    args = args[0];
    arg = args[0];
  }
  if(typeof arg == "object") {
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
    while(typeof arg == "object" && (arg instanceof Array || "length" in arg)) {
      args = [...arg];
      arg = args[0];
    }
    if(args && args.length >= 2) {
      let w = args.shift();
      let h = args.shift();
      if(typeof w == "object" && "baseVal" in w) w = w.baseVal.value;
      if(typeof h == "object" && "baseVal" in h) h = h.baseVal.value;
      obj.width = typeof w == "number" ? w : parseFloat(w.replace(/[^-.0-9]*$/, ""));
      obj.height = typeof h == "number" ? h : parseFloat(h.replace(/[^-.0-9]*$/, ""));
      obj.units = {
        width: typeof w == "number" ? "px" : w.replace(obj.width.toString(), ""),
        height: typeof h == "number" ? "px" : h.replace(obj.height.toString(), "")
      };
    }
  }
  if(isNaN(obj.width)) obj.width = undefined;
  if(isNaN(obj.height)) obj.height = undefined;
  if(!(obj instanceof Size)) return obj;
}
Size.convertUnits = (size, w = "window" in global ? window : null) => {
  if(w === null) return size;
  const view = {
    vw: w.innerWidth,
    vh: w.innerHeight,
    vmin: w.innerWidth < w.innerHeight ? w.innerWidth : w.innerHeight,
    vmax: w.innerWidth > w.innerHeight ? w.innerWidth : w.innerHeight
  };
  if(view[size.units.width] !== undefined) {
    size.width = (size.width / 100) * view[size.units.width];
    delete size.units.width;
  }
  if(view[size.units.height] !== undefined) {
    size.height = (size.height / 100) * view[size.units.height];
    delete size.units.height;
  }
  return size;
};
Size.aspect = size => {
  size = this instanceof Size ? this : size;
  return size.width / size.height;
};
Size.prototype.aspect = function() {
  return Size.aspect(this);
};
Size.toCSS = function(arg) {
  const size = arg && arg.width !== undefined ? arg : this;
  let ret = {};
  if(size.width !== undefined) ret.width = size.width + (size.units && "width" in size.units ? size.units.width : "px");
  if(size.height !== undefined) ret.height = size.height + (size.units && "height" in size.units ? size.units.height : "px");
  return ret;
};
Size.prototype.toCSS = Size.toCSS;
Size.transform = (s, m) => ({
  width: m.xx * s.width + m.yx * s.height,
  height: m.xy * s.width + m.yy * s.height
});
Size.prototype.transform = function(m) {
  const t = Size.transform(this, m);
  this.width = t.width;
  this.height = t.height;
  return this;
};
Size.prototype.isSquare = function() {
  return Math.abs(this.width - this.height) < 1;
};
Size.prototype.area = function() {
  return this.width * this.height;
};
Size.area = size => {
  return size.width * size.height;
};
export function Line(x1, y1, x2, y2) {
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
  } else if(isPoint(args[0]) && isPoint(args[1])) {
    obj.x1 = parseFloat(args[0].x);
    obj.y1 = parseFloat(args[0].y);
    obj.x2 = parseFloat(args[1].x);
    obj.y2 = parseFloat(args[1].y);
    ret = 2;
  } else if(arg && arg.length >= 4 && arg.slice(0, 4).every(arg => !isNaN(parseFloat(arg)))) {
    obj.x1 = typeof x === "number" ? x : parseFloat(x);
    obj.y1 = typeof y === "number" ? y : parseFloat(y);
    obj.x2 = typeof w === "number" ? w : parseFloat(w);
    obj.y2 = typeof h === "number" ? h : parseFloat(h);
    ret = 4;
  } else {
    ret = 0;
  }
  if(!isLine(obj)) console.log("ERROR: is not a line: ", [...arguments]);
  if(!(this instanceof Line)) return obj;
}
export const isLine = obj => ["x1", "y1", "x2", "y2"].every(prop => obj[prop] !== undefined);
Line.isLine = isLine;
Line.intersect = (a, b) => {
  const ma = (a[0].y - a[1].y) / (a[0].x - a[1].x);
  const mb = (b[0].y - b[1].y) / (b[0].x - b[1].x);
  if(ma - mb < Number.EPSILON) return undefined;
  return new Point({
    x: (ma * a[0].x - mb * b[0].x + b[0].y - a[0].y) / (ma - mb),
    y: (ma * mb * (b[0].x - a[0].x) + mb * a[0].y - ma * b[0].y) / (mb - ma)
  });
};
Object.defineProperty(Line.prototype, "x1", {
  get: function() {
    return this.a && this.a.x;
  },
  set: function(v) {
    if(!this.a) this.a = new Point();
    this.a.x = v;
  },
  enumerable: true
});
Object.defineProperty(Line.prototype, "y1", {
  get: function() {
    return this.a && this.a.y;
  },
  set: function(v) {
    if(!this.a) this.a = new Point();
    this.a.y = v;
  },
  enumerable: true
});
Object.defineProperty(Line.prototype, "x2", {
  get: function() {
    return this.b && this.b.x;
  },
  set: function(v) {
    if(!this.b) this.b = new Point();
    this.b.x = v;
  },
  enumerable: true
});
Object.defineProperty(Line.prototype, "y2", {
  get: function() {
    return this.b && this.b.y;
  },
  set: function(v) {
    if(!this.b) this.b = new Point();
    this.b.y = v;
  },
  enumerable: true
});
Line.prototype.direction = function() {
  var dist = Point.prototype.distance.call(this.a, this.b);
  return Point.prototype.diff.call(this.a, this.b) / dist;
};
Line.prototype.slope = function() {
  return Point.prototype.diff.call(this.a, this.b);
};
Line.prototype.angle = function() {
  return Point.prototype.angle.call(Line.prototype.slope.call(this));
};
Line.prototype.length = function() {
  return Point.prototype.distance.call(this.a, this.b);
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
  return "Line{ " + inspect({ x1, y1, x2, y2 }) + " }";
};
Line.prototype.toString = function() {
  let { a, b } = this;
  if(a.x > b.x) {
    let tmp = this.b;
    this.b = this.a;
    this.a = tmp;
  }
  return Point.prototype.toString.call(this.a) + " -> " + Point.prototype.toString.call(this.b);
};
export function Rect(arg) {
  let obj = this instanceof Rect ? this : {};
  let args = arg instanceof Array ? arg : [...arguments];
  let ret;
  if(typeof args[0] == "number") arg = args;
  else if(args[0].length !== undefined) arg = args.shift();
  ["x", "y", "width", "height"].forEach(field => {
    if(typeof obj[field] != "number") obj[field] = 0;
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
    obj.x = typeof x === "number" ? x : parseFloat(x);
    obj.y = typeof y === "number" ? y : parseFloat(y);
    obj.width = typeof w === "number" ? w : parseFloat(w);
    obj.height = typeof h === "number" ? h : parseFloat(h);
    ret = 4;
  } else if(arg && arg.length >= 2 && arg.slice(0, 2).every(arg => !isNaN(parseFloat(arg)))) {
    obj.width = typeof x === "number" ? x : parseFloat(x);
    obj.height = typeof y === "number" ? y : parseFloat(y);
    ret = 2;
  } else if(arg instanceof Array) {
    let argc;
    let argi = 0;
    if(arg.length >= 4) {
      argc = typeof x == "number" ? 2 : 1;
      Point.apply(obj, arg.slice(0, argc));
      argi = argc;
    }
    argc = typeof arg[argi] == "number" ? 2 : 1;
    Size.apply(obj, arg.slice(argi, argc));
    ret = argi + argc;
  }
  if(obj.round === undefined) {
    Object.defineProperty(obj, "round", {
      value: function() {
        return Rect.round(this);
      },
      enumerable: true,
      writable: false
    });
  }
  if(!(this instanceof Rect)) {
    return obj;
    return ret;
  }
}
Rect.prototype.clone = function() {
  return new Rect(this.x, this.y, this.width, this.height);
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
Object.assign(Rect.prototype, Point.prototype);
Object.assign(Rect.prototype, Size.prototype);
if(Rect.prototype.isSquare === undefined) {
  Rect.prototype.isSquare = function() {
    return Math.abs(this.width - this.height) < 1;
  };
}
Rect.prototype.constructor = Rect;
Rect.prototype.area = function() {
  return this.width * this.height;
};
Rect.prototype.toString = function() {
  return this.x + "," + this.y + " " + this.width + "x" + this.height;
};
Rect.prototype.toSource = function() {
  return "new Rect(" + (this ? this.x + "," + this.y + "," + this.width + "," + this.height : "") + ")";
};
Object.defineProperty(Rect.prototype, "x1", {
  get: function() {
    return this.x;
  },
  set: function(value) {
    const extend = this.x - value;
    this.width += extend;
    this.x -= extend;
  },
  enumerable: true
});
Object.defineProperty(Rect.prototype, "x2", {
  get: function() {
    return this.x + this.width;
  },
  set: function(value) {
    this.width = value - this.x;
  },
  enumerable: true
});
Object.defineProperty(Rect.prototype, "y1", {
  get: function() {
    return this.y;
  },
  set: function(value) {
    const extend = this.y - value;
    this.height += extend;
    this.y -= extend;
  }
});
Object.defineProperty(Rect.prototype, "y2", {
  get: function() {
    return this.y + this.height;
  },
  set: function(value) {
    this.height = value - this.y;
  }
});
Object.defineProperty(Rect.prototype, "area", {
  get: function() {
    return Rect.area(this);
  }
});
Object.defineProperty(Rect.prototype, "center", {
  get: function() {
    return Rect.center(this);
  }
});
Rect.prototype.points = function() {
  const c = this.corners();
  return new PointList(c);
};
Rect.prototype.toCSS = Rect.toCSS;
Rect.prototype.outset = function(trbl) {
  if(typeof trbl == "number") trbl = new TRBL(trbl, trbl, trbl, trbl);
  this.x -= trbl.left;
  this.y -= trbl.top;
  this.width += trbl.left + trbl.right;
  this.height += trbl.top + trbl.bottom;
  return this;
};
Rect.prototype.inset = function(trbl) {
  if(typeof trbl == "number") trbl = new TRBL(trbl, trbl, trbl, trbl);
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
Rect.prototype.pointFromCenter = function(point) {
  Point.prototype.sub.call(point, this.center);
  point.x /= this.width;
  point.y /= this.height;
  return point;
};
function PointList(points) {
  let args = [...arguments];
  let ret = this instanceof PointList ? this : [];
  if(args.length == 1 && args[0] instanceof Array) args = args[0];
  if(typeof points === "string") {
    const matches = [...points.matchAll(/[-.0-9,]+/g)];
    for(let i = 0; i < matches.length; i++) {
      const coords = String(matches[i]).split(",");
      ret.push(Point(coords));
    }
  } else if(args[0] && args[0].length == 2) {
    for(let i = 0; i < args.length; i++) ret.push(this instanceof PointList ? new Point(args[i]) : Point(args[i]));
  } else if(isPoint(args[0])) {
    for(let i = 0; i < args.length; i++) ret.push(this instanceof PointList ? new Point(args[i]) : Point(args[i]));
  }
  if(!(this instanceof PointList)) {
    return ret;
  }
}
PointList.prototype = new Array();
PointList.prototype.push = function() {
  const args = [...arguments];
  args.forEach(arg => {
    if(!(arg instanceof Point)) arg = new Point(arg);
    Array.prototype.push.call(this, arg);
  });
};
PointList.push = (plist, points) => {
  let args = [...arguments];
  args.shift();
  return PointList.prototype.push.apply(plist, args);
};
PointList.prototype.splice = function() {
  let args = [...arguments];
  const start = args.shift();
  const remove = args.shift();
  return Array.prototype.splice.apply(this, [start, remove, ...args.map(arg => (arg instanceof Point ? arg : new Point(arg)))]);
};
PointList.splice = (plist, start, remove, points) => {
  let args = [...arguments];
  args.shift();
  return PointList.prototype.splice.apply(plist, args);
};
PointList.prototype.removeSegment = function(index) {
  let indexes = [PointList.prototype.getLineIndex.call(this, index - 1), PointList.prototype.getLineIndex.call(this, index), PointList.prototype.getLineIndex.call(this, index + 1)];
  let lines = indexes.map(i => PointList.prototype.getLine.call(this, i));
  let point = Line.intersect(lines[0], lines[2]);
  if(point) {
    PointList.prototype.splice.call(this, 0, 2, new Point(point));
  }
};
PointList.prototype.toPath = function(options = {}) {
  const { relative = false, close = false } = options;
  let out = "";
  for(let i = 0; i < this.length; i++) {
    out += (i == 0 ? "M" : "L") + this[i].x.toFixed(3) + "," + this[i].y.toFixed(3) + " ";
  }
  if(close) out += "Z";
  return out;
};
PointList.prototype.clone = function() {
  let ret = new PointList();
  ret.splice.apply(ret, [0, ret.length, ...this.map(p => new Point(p.x, p.y))]);
  return ret;
};
PointList.copy = plist => PointList.prototype.clone.call(plist);
PointList.prototype.toPolar = function(tfn) {
  let ret = new PointList();
  let t = typeof tfn == "function" ? tfn : (x, y) => ({ x: (x * 180) / Math.PI, y });
  ret.splice.apply(ret, [
    0,
    ret.length,
    ...this.map(p => {
      const angle = Point.prototype.toAngle.call(p);
      return t(angle, Point.prototype.distance.call(p));
    })
  ]);
  return ret;
};
PointList.toPolar = plist => PointList.prototype.toPolar.call(plist);
PointList.prototype.fromPolar = function(tfn) {
  let ret = new PointList();
  let t = typeof tfn == "function" ? tfn : (x, y) => ({ x: (x * Math.PI) / 180, y });
  ret.splice.apply(ret, [
    0,
    ret.length,
    ...this.map(p => {
      let r = t(p.x, p.y);
      return Point.prototype.fromAngle.call(r.x, r.y);
    })
  ]);
  return ret;
};
PointList.fromPolar = plist => PointList.prototype.toPolar.call(plist);
PointList.prototype.draw = function(ctx, close = false) {
  ctx.to(this[0].x, this[0].y);
  for(let i = 1; i < this.length; i++) {
    ctx.line(this[i].x, this[i].y);
  }
  if(close) ctx.line(this[0].x, this[0].y);
  return this;
};
PointList.prototype.area = function() {
  var area = 0;
  var i;
  var j;
  var point1;
  var point2;
  for(i = 0, j = this.length - 1; i < this.length; j = i, i += 1) {
    point1 = this[i];
    point2 = this[j];
    area += point1.x * point2.y;
    area -= point1.y * point2.x;
  }
  area /= 2;
  return area;
};
PointList.prototype.centroid = function() {
  var x = 0;
  var y = 0;
  var i;
  var j;
  var f;
  var point1;
  var point2;
  for(i = 0, j = this.length - 1; i < this.length; j = i, i += 1) {
    point1 = this[i];
    point2 = this[j];
    f = point1.x * point2.y - point2.x * point1.y;
    x += (point1.x + point2.x) * f;
    y += (point1.y + point2.y) * f;
  }
  f = this.area() * 6;
  return new Point(x / f, y / f);
};
PointList.prototype.avg = function() {
  var ret = this.reduce((acc, p) => acc.add(p), new Point());
  return ret.div(this.length);
};
PointList.prototype.minmax = function() {
  if(!this.length) return {};
  var ret = {
    x1: this[0].x,
    x2: this[0].x,
    y1: this[0].y,
    y2: this[0].y,
    toString: function() {
      return `x ${this.x1.toFixed(3)}->${this.x2.toFixed(3)} y ${this.y1.toFixed(3)}->${this.y2.toFixed(3)}`;
    }
  };
  for(let i = 1; i < this.length; i++) {
    const x = this[i].x;
    const y = this[i].y;
    if(x < ret.x1) ret.x1 = x;
    if(x > ret.x2) ret.x2 = x;
    if(y < ret.y1) ret.y1 = y;
    if(y > ret.y2) ret.y2 = y;
  }
  return ret;
};
PointList.minmax = list => PointList.prototype.minmax.call(list);
PointList.rect = list => new Rect(PointList.minmax(list));
PointList.prototype.rect = function() {
  return PointList.rect(this);
};
PointList.prototype.xrange = function() {
  const minmax = this.minmax();
  return [minmax.x1, minmax.x2];
};
PointList.prototype.normalizeX = function(newVal = x => x) {
  const xrange = PointList.prototype.xrange.call(this);
  const xdiff = xrange[1] - xrange[0];
  this.forEach((p, i, l) => {
    l[i].x = newVal((l[i].x - xrange[0]) / xdiff);
  });
  return this;
};
PointList.prototype.yrange = function() {
  const minmax = this.minmax();
  return [minmax.y1, minmax.y2];
};
PointList.prototype.normalizeY = function(newVal = y => y) {
  const yrange = PointList.prototype.yrange.call(this);
  const ydiff = yrange[1] - yrange[0];
  this.forEach((p, i, l) => {
    l[i].y = newVal((l[i].y - yrange[0]) / ydiff);
  });
  return this;
};
PointList.prototype.boundingRect = function() {
  return new Rect(this.minmax());
};
PointList.prototype.translate = function(x, y) {
  for(let i = 0; i < this.length; i++) Point.prototype.move.call(this[i], x, y);
  return this;
};
PointList.prototype.transform = function(arg) {
  const fn = typeof arg == "function" ? arg : p => Point.prototype.transform.call(p, arg);
  for(let i = 0; i < this.length; i++) {
    const p = fn(this[i]);
    this[i].x = p.x;
    this[i].y = p.y;
  }
  return this;
};
PointList.prototype.filter = function(pred) {
  let ret = new PointList();
  this.forEach((p, i, l) => pred(p, i, l) && ret.push(new Point(l[i])));
  return ret;
};
PointList.prototype.getLineIndex = function(index) {
  return (index < 0 ? this.length + index : index) % this.length;
};
PointList.prototype.getLine = function(index) {
  let a = PointList.prototype.getLineIndex.call(this, index);
  let b = PointList.prototype.getLineIndex.call(this, index + 1);
  return [this[a], this[b]];
};
PointList.prototype.lines = function(closed = false) {
  const points = this;
  const n = points.length - (closed ? 0 : 1);
  const iterableObj = {
    [Symbol.iterator]() {
      let step = 0;
      return {
        next() {
          let value;
          let done = step >= n;
          if(!done) {
            value = new Line(points[step], points[(step + 1) % points.length]);
            step++;
          }
          return { value, done };
        }
      };
    }
  };
  return iterableObj;
};
PointList.prototype.toString = function(prec) {
  return this.map(point => Point.prototype.toString.call(point, prec)).join(" ");
};
PointList.toString = pointList => {
  return "[" + [...pointList].map(p => `[${p.x || p[0]},${p.y || p[1]}]`).join(",") + "]";
};
PointList.prototype.rotateRight = function(n) {
  return Util.rotateRight(this, n);
};
const Classes = { Point, Line, Rect, Size };
export default Classes;
