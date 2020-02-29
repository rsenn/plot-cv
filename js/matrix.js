export function Matrix(arg) {
  let ret = this instanceof Matrix ? this : [undefined, 0, 0, undefined, 0, 0, undefined, 0, 0];
  if(typeof arg === "string") {
    if(/matrix\([^)]*\)/.test(arg)) {
      let [xx, xy, x0, yx, yy, y0] = [...arg.matchAll(/[-.0-9]+/g)].map(m => parseFloat(m[0]));
      ret[0] = xx;
      ret[1] = xy;
      ret[2] = x0;
      ret[3] = yx;
      ret[4] = yy;
      ret[5] = y0;
    }
  } else if(arg && typeof arg == "object") {
    if(arg.xx !== undefined && arg.yx !== undefined && arg.xy !== undefined && arg.yy !== undefined && arg.x0 !== undefined && arg.y0 !== undefined) {
      ret[0] = arg.xx;
      ret[1] = arg.xy;
      ret[2] = arg.x0;
      ret[3] = arg.yx;
      ret[4] = arg.yy;
      ret[5] = arg.y0;
    } else if(arg.a !== undefined && arg.b !== undefined && arg.c !== undefined && arg.d !== undefined && arg.e !== undefined && arg.f !== undefined) {
      ret[0] = arg.a;
      ret[3] = arg.b;
      ret[1] = arg.c;
      ret[4] = arg.d;
      ret[2] = arg.e;
      ret[5] = arg.f;
    }
  } else {
    ret[0] = 1;
    ret[1] = 0;
    ret[2] = 0;
    ret[3] = 0;
    ret[4] = 1;
    ret[5] = 0;
  }
  if(ret[0] === undefined) Matrix.prototype.set_row.call(ret, 0, 1, 0, 0);
  if(ret[3] === undefined) Matrix.prototype.set_row.call(ret, 1, 0, 1, 0);
  if(ret[6] === undefined) Matrix.prototype.set_row.call(ret, 2, 0, 0, 1);

  if(!(this instanceof Matrix)) return Object.assign(ret, Matrix.prototype);
}

Matrix.prototype = [1, 0, 0, 0, 1, 0, 0, 0, 1];

Matrix.prototype.keys = ["xx", "xy", "x0", "yx", "yy", "y0"];
Matrix.prototype.keySeq = ["xx", "yx", "xy", "yy", "x0", "y0"];
Matrix.prototype.keyIndex = { xx: 0, a: 0, xy: 1, c: 1, x0: 2, tx: 2, e: 2, yx: 3, b: 3, yy: 4, d: 4, y0: 5, ty: 5, f: 5 };

Matrix.prototype.at = function(key) {
  return this[Matrix.prototype.keyIndex[key]];
};

export const MatrixProps = Object.keys(Matrix.prototype.keyIndex).reduce((acc, k) => {
  const i = Matrix.prototype.keyIndex[k];
  return {
    ...acc,
    [k]: {
      get: function() {
        return this[i];
      },
      set: function(v) {
        this[i] = v;
      },
      enumerable: true
    }
  };
}, {});

// prettier-ignore
Object.defineProperties(Matrix.prototype, {
  xx: {get: function() {return this[0]; }, set: function(v) {this[0] = v; }, enumerable: true },
  xy: {get: function() {return this[1]; }, set: function(v) {this[1] = v; }, enumerable: true },
  x0: {get: function() {return this[2]; }, set: function(v) {this[2] = v; }, enumerable: true },
  yx: {get: function() {return this[3]; }, set: function(v) {this[3] = v; }, enumerable: true },
  yy: {get: function() {return this[4]; }, set: function(v) {this[4] = v; }, enumerable: true },
  y0: {get: function() {return this[5]; }, set: function(v) {this[5] = v; }, enumerable: true }
});

Matrix.prototype.row = function(row) {
  let i = row * 3;
  return Array.prototype.slice.call(this, i, i + 3);
};

Matrix.prototype.init = function() {
  let args = [...arguments];
  if(args.length == 6) args.push(0);
  if(args.length == 7) args.push(0);
  if(args.length == 8) args.push(1);
  for(let i = 0; i < args.length; i++) this[i] = args[i];
  return this;
};
Matrix.prototype.set_row = function() {
  let args = [...arguments];
  let row = args.shift();
  let start = row * 3;
  for(let i = 0; i < args.length; i++) this[start + i] = args[i];
  return this;
};
Matrix.prototype.rows = function() {
  let ret = [];
  for(let i = 0; i < this.length; i += 3) {
    let row = [];
    for(let j = 0; j < 3; j++) {
      row.push(this[i + j]);
    }
    ret.push(row);
  }
  return ret;
};
Matrix.prototype.multiply = function(m) {
  const r = [
    this[0] * m[0] + this[1] * m[3],
    this[0] * m[1] + this[1] * m[4],
    this[0] * m[2] + this[1] * m[5] + this[2],
    this[3] * m[0] + this[4] * m[3],
    this[3] * m[1] + this[4] * m[4],
    this[3] * m[2] + this[4] * m[5] + this[5]
  ];
  return this.init.apply(this, r);
};

Matrix.prototype.product = function(m) {
  if(!(m instanceof Matrix)) m = new Matrix(m);
  return new Matrix({
    xx: this[0] * m[0] + this[1] * m[3],
    xy: this[0] * m[1] + this[1] * m[4],
    x0: this[0] * m[2] + this[1] * m[5] + this[2],
    yx: this[3] * m[0] + this[4] * m[3],
    yy: this[3] * m[1] + this[4] * m[4],
    y0: this[3] * m[2] + this[4] * m[5] + this[5]
  });
};

Matrix.prototype.scalar_product = function(f) {
  return new Matrix({
    xx: this[0] * f,
    xy: this[1] * f,
    x0: this[2] * f,
    yx: this[3] * f,
    yy: this[4] * f,
    y0: this[5] * f
  });
};

Matrix.prototype.translate = function(tx, ty) {
  const m = new Matrix({ xx: 1, xy: 0, x0: tx, yx: 0, yy: 1, y0: ty });
  return Matrix.prototype.multiply.call(this, m);
};

Matrix.prototype.scale = function(sx, sy) {
  const m = new Matrix({ xx: sx, xy: 0, x0: 0, yx: 0, yy: sy, y0: 0 });
  return Matrix.prototype.multiply.call(this, m);
};

Matrix.prototype.rotate = function(rad) {
  let m = new Matrix({ xx: 1, xy: 0, x0: 0, yx: 0, yy: 1, y0: 0 });
  Matrix.prototype.init_rotate.call(m, rad);
  return Matrix.prototype.multiply.call(this, m);
};

Matrix.prototype.toArray = function() {
  let k;
  let arr = [];
  for(k = 0; k < Matrix.prototype.keys.length; k++) {
    let key = Matrix.prototype.keys[k];
    arr.push(this[key] || this[k]);
  }
  return arr;
};

Matrix.prototype.toString = function() {
  let rows = Matrix.prototype.rows.call(this);
  return "[\n  " + rows.map(row => "[" + row.join(", ") + "]").join(",\n  ") + "\n]";
};

Matrix.prototype.toSVG = function() {
  return "matrix(" + ["a", "b", "c", "d", "e", "f"].map(k => this[Matrix.prototype.keyIndex[k]]).join(",") + ")";
};

Matrix.prototype.init_identity = function() {
  Matrix.prototype.set_row.call(this, 0, 1, 0, 0);
  Matrix.prototype.set_row.call(this, 1, 0, 1, 0);
  Matrix.prototype.set_row.call(this, 2, 0, 0, 1);
  return this;
};

Matrix.prototype.init_translate = function(tx, ty) {
  Matrix.prototype.set_row.call(this, 0, 1, 0, tx);
  Matrix.prototype.set_row.call(this, 1, 0, 1, ty);
  Matrix.prototype.set_row.call(this, 2, 0, 0, 1);
  return this;
};

Matrix.prototype.init_scale = function(sx, sy) {
  Matrix.prototype.set_row.call(this, 0, sx, 0, 0);
  Matrix.prototype.set_row.call(this, 1, 0, sy, 0);
  Matrix.prototype.set_row.call(this, 2, 0, 0, 1);
  return this;
};

Matrix.prototype.init_rotate = function(rad) {
  const s = Math.sin(rad);
  const c = Math.cos(rad);
  Matrix.prototype.set_row.call(this, 0, c, -s, 0);
  Matrix.prototype.set_row.call(this, 1, s, c, 0);
  Matrix.prototype.set_row.call(this, 2, 0, 0, 1);
  return this;
};

Matrix.prototype.transform_distance = function(p) {
  const x = this.xx * p.x + this.xy * p.y;
  const y = this.yx * p.x + this.yy * p.y;
  p.x = x;
  p.y = y;
  return p;
};

Matrix.prototype.transform_point = function(p) {
  const x = this[0] * p.x + this[1] * p.y + this[2];
  const y = this[3] * p.x + this[4] * p.y + this[5];
  p.x = x;
  p.y = y;
  return p;
};

Matrix.prototype.transform_size = function(s) {
  const w = this[0] * s.width + this[1] * s.height;
  const h = this[3] * s.width + this[4] * s.height;
  s.width = w;
  s.height = h;
  return s;
};

Matrix.prototype.transform_rect = function(rect) {
  Matrix.prototype.transform_point.call(this, rect);
  Matrix.prototype.transform_size.call(this, rect);
  return rect;
};

Matrix.prototype.point_transformer = function() {
  const m = this;
  return function(p) {
    var matrix = m;
    return matrix.transform_point(p);
  };
};

Matrix.prototype.decompose = function() {
  let translate = {
    toString: function() {
      return `translate(${this.x.toFixed(3)} ${this.y.toFixed(3)})`;
    },
    x: this[2],
    y: this[5]
  };
  let scale = {
    toString: function() {
      return `scale(${this.x.toFixed(6)} ${this.y.toFixed(6)})`;
    },
    x: Math.sign(this[0]) * Math.sqrt(Math.pow(this[0], 2) + Math.pow(this[3], 2)),
    y: Math.sign(this[4]) * Math.sqrt(Math.pow(this[1], 2) + Math.pow(this[4], 2))
  };
  let rotate = {
    toString() {
      return `rotate(${this.deg.toFixed(2)}deg)`;
    },
    get deg() {
      return (this.rad * 180) / Math.PI;
    }
  };
  rotate.rad = Math.atan2(-this[1] / scale.y, this[0] / scale.x);
  return { translate, scale, rotate };
};

Matrix.prototype.getAffineTransform = (a, b) => {
  var xx, yx, xy, yy, tx, ty;
  if(typeof a == "object" && a.toPoints !== undefined) a = a.toPoints();
  if(typeof b == "object" && b.toPoints !== undefined) b = b.toPoints();
  xx =
    (b[0].x * a[1].y + b[1].x * a[2].y + b[2].x * a[0].y - b[0].x * a[2].y - b[1].x * a[0].y - b[2].x * a[1].y) /
    (a[0].x * a[1].y + a[1].x * a[2].y + a[2].x * a[0].y - a[0].x * a[2].y - a[1].x * a[0].y - a[2].x * a[1].y);
  yx =
    (b[0].y * a[1].y + b[1].y * a[2].y + b[2].y * a[0].y - b[0].y * a[2].y - b[1].y * a[0].y - b[2].y * a[1].y) /
    (a[0].x * a[1].y + a[1].x * a[2].y + a[2].x * a[0].y - a[0].x * a[2].y - a[1].x * a[0].y - a[2].x * a[1].y);
  xy =
    (a[0].x * b[1].x + a[1].x * b[2].x + a[2].x * b[0].x - a[0].x * b[2].x - a[1].x * b[0].x - a[2].x * b[1].x) /
    (a[0].x * a[1].y + a[1].x * a[2].y + a[2].x * a[0].y - a[0].x * a[2].y - a[1].x * a[0].y - a[2].x * a[1].y);
  yy =
    (a[0].x * b[1].y + a[1].x * b[2].y + a[2].x * b[0].y - a[0].x * b[2].y - a[1].x * b[0].y - a[2].x * b[1].y) /
    (a[0].x * a[1].y + a[1].x * a[2].y + a[2].x * a[0].y - a[0].x * a[2].y - a[1].x * a[0].y - a[2].x * a[1].y);
  tx =
    (a[0].x * a[1].y * b[2].x + a[1].x * a[2].y * b[0].x + a[2].x * a[0].y * b[1].x - a[0].x * a[2].y * b[1].x - a[1].x * a[0].y * b[2].x - a[2].x * a[1].y * b[0].x) /
    (a[0].x * a[1].y + a[1].x * a[2].y + a[2].x * a[0].y - a[0].x * a[2].y - a[1].x * a[0].y - a[2].x * a[1].y);
  ty =
    (a[0].x * a[1].y * b[2].y + a[1].x * a[2].y * b[0].y + a[2].x * a[0].y * b[1].y - a[0].x * a[2].y * b[1].y - a[1].x * a[0].y * b[2].y - a[2].x * a[1].y * b[0].y) /
    (a[0].x * a[1].y + a[1].x * a[2].y + a[2].x * a[0].y - a[0].x * a[2].y - a[1].x * a[0].y - a[2].x * a[1].y);
  this.set_row.call(this, 0, xx, xy, tx);
  this.set_row.call(this, 1, yx, yy, ty);
  this.set_row.call(this, 2, 0, 0, 1);
  return this;
};
