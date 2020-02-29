/**
 * Class for size.
 *
 * @class      Size (w, h)
 */
function Size(arg) {
  let obj = this instanceof Size ? this : {};
  let args = [...arguments];
  if(args.length == 1 && args[0].length !== undefined) {
    args = args[0];
    arg = args[0];
  }
  //console.log.apply(console, ['Size(', ...args, ')']);
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
    //console.log('Size ', { arg, args });
    /*
    if(typeof arg === 'string')
      arg = [...args.join(' ').matchAll(/[0-9.]+/g)].slice(-2).map(a => parseFloat(a));
    else if(typeof arg !== 'object') arg = arguments;
*/
    if(args && args.length >= 2) {
      let w = args.shift();
      let h = args.shift();
      if(typeof w == 'object' && 'baseVal' in w) w = w.baseVal.value;
      if(typeof h == 'object' && 'baseVal' in h) h = h.baseVal.value;
      obj.width = typeof w == 'number' ? w : parseFloat(w.replace(/[^-.0-9]*$/, ''));
      obj.height = typeof h == 'number' ? h : parseFloat(h.replace(/[^-.0-9]*$/, ''));
      obj.units = {
        width: typeof w == 'number' ? 'px' : w.replace(obj.width.toString(), ''),
        height: typeof h == 'number' ? 'px' : h.replace(obj.height.toString(), '')
      };
    }
  }
  if(isNaN(obj.width)) obj.width = undefined;
  if(isNaN(obj.height)) obj.height = undefined;
  //console.log('Size', obj, ' arg=', arg);
  if(!(obj instanceof Size)) return obj;
}
Size.convertUnits = (size, w = 'window' in global ? window : null) => {
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
  if(size.width !== undefined) ret.width = size.width + (size.units && 'width' in size.units ? size.units.width : 'px');
  if(size.height !== undefined) ret.height = size.height + (size.units && 'height' in size.units ? size.units.height : 'px');
  return ret;
};
Size.prototype.toCSS = Size.toCSS;
//export const isSize = o => o && ((o.width !== undefined && o.height !== undefined) || (o.x !== undefined && o.x2 !== undefined && o.y !== undefined && o.y2 !== undefined) || (o.left !== undefined && o.right !== undefined && o.top !== undefined && o.bottom !== undefined));
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
