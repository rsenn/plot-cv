import { Point } from "point.js";
import { Size } from "size.js";

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
Rect.prototype = {
  ...Point.prototype,
  ...Size.prototype,
  ...Rect.prototype
};
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

if (Rect.prototype.isSquare === undefined) {
  Rect.prototype.isSquare = function() {
    return Math.abs(this.width - this.height) < 1;
  };
}
Rect.prototype.constructor = Rect;
Rect.prototype.getArea = function() {
  return this.width * this.height;
};
Rect.prototype.toString = function() {
  return (
    (this.x + "").padStart(4, " ") +
    "," +
    (this.y + "").padEnd(4, " ") +
    " " +
    (this.width + "").padStart(4, " ") +
    "x" +
    (this.height + "").padEnd(4, " ")
  );
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
    return Rect.prototype.getArea.call(this);
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
