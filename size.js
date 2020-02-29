export function Size(arg) {
  let obj = this instanceof Size ? this : {};
  let args = [...arguments];
  if (args.length == 1 && args[0].length !== undefined) {
    args = args[0];
    arg = args[0];
  }
  if (typeof arg == "object") {
    if (arg.width !== undefined || arg.height !== undefined) {
      arg = args.shift();
      obj.width = arg.width;
      obj.height = arg.height;
    } else if (arg.x2 !== undefined && arg.y2 !== undefined) {
      arg = args.shift();
      obj.width = arg.x2 - arg.x;
      obj.height = arg.y2 - arg.y;
    } else if (arg.bottom !== undefined && arg.right !== undefined) {
      arg = args.shift();
      obj.width = arg.right - arg.left;
      obj.height = arg.bottom - arg.top;
    }
  } else {
    while (typeof arg == "object" && (arg instanceof Array || "length" in arg)) {
      args = [...arg];
      arg = args[0];
    }
    if (args && args.length >= 2) {
      let w = args.shift();
      let h = args.shift();
      if (typeof w == "object" && "baseVal" in w) w = w.baseVal.value;
      if (typeof h == "object" && "baseVal" in h) h = h.baseVal.value;
      obj.width = typeof w == "number" ? w : parseFloat(w.replace(/[^-.0-9]*$/, ""));
      obj.height = typeof h == "number" ? h : parseFloat(h.replace(/[^-.0-9]*$/, ""));
      obj.units = {
        width: typeof w == "number" ? "px" : w.replace(obj.width.toString(), ""),
        height: typeof h == "number" ? "px" : h.replace(obj.height.toString(), "")
      };
    }
  }
  if (isNaN(obj.width)) obj.width = undefined;
  if (isNaN(obj.height)) obj.height = undefined;
  if (!(obj instanceof Size)) return obj;
}
Size.prototype.convertUnits = function(w = "window" in global ? window : null) {
  if (w === null) return this;
  const view = {
    vw: w.innerWidth,
    vh: w.innerHeight,
    vmin: w.innerWidth < w.innerHeight ? w.innerWidth : w.innerHeight,
    vmax: w.innerWidth > w.innerHeight ? w.innerWidth : w.innerHeight
  };
  if (view[this.units.width] !== undefined) {
    this.width = (this.width / 100) * view[this.units.width];
    delete this.units.width;
  }
  if (view[this.units.height] !== undefined) {
    this.height = (this.height / 100) * view[this.units.height];
    delete this.units.height;
  }
  return size;
};

Size.prototype.aspect = function() {
  return this.width / this.height;
};
Size.prototype.toCSS = function() {
  let ret = {};
  if (this.width !== undefined) ret.width = this.width + (this.units && "width" in this.units ? this.units.width : "px");
  if (this.height !== undefined) ret.height = this.height + (this.units && "height" in this.units ? this.units.height : "px");
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
