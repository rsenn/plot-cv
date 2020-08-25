/*
 * concatenanted lib/geom/align.js
 */

export function Align(arg) {}
Align.CENTER = 0;
Align.LEFT = 1;
Align.RIGHT = 2;
Align.MIDDLE = 0;
Align.TOP = 4;
Align.BOTTOM = 8;
Align.horizontal = (alignment) => alignment & (Align.LEFT | Align.RIGHT);
Align.vertical = (alignment) => alignment & (Align.TOP | Align.BOTTOM);
const Anchor = Align;

/*
 * concatenanted lib/geom/bbox.js
 */

export class BBox {
  static fromPoints(pts) {
    let pt = pts.shift();
    let bb = new BBox(pt.x, pt.y, pt.x, pt.y);
    bb.update(pts);
    return bb;
  }

  static fromRect(rect) {
    return new BBox(rect.x, rect.y, rect.x + rect.width, rect.y + rect.height);
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

  updateList(list, offset = (0.0, (objFn = ((item) => item, (t = (a) => a))))) {
    for(let arg of list) this.update(t(arg), offset, objFn(arg));
    return this;
  }

  update(arg, offset = (0.0, (obj = null))) {
    if(Util.isArray(arg)) return this.updateList(arg, offset);
    else if(Util.isObject(arg)) {
      if(typeof arg.bbox == 'function') {
        arg = arg.bbox();
      } else {
        if(arg.x !== undefined && arg.y != undefined) this.updateXY(arg.x, arg.y, offset, (name) => (this.objects.name = obj || arg));
        if(arg.x1 !== undefined && arg.y1 != undefined) this.updateXY(arg.x1, arg.y1, 0, (name) => (this.objects.name = obj || arg));
        if(arg.x2 !== undefined && arg.y2 != undefined) this.updateXY(arg.x2, arg.y2, 0, (name) => (this.objects.name = obj || arg));
      }
    }

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

  toRect(proto) {
    let r = this.rect;
    return Object.setPrototypeOf(r, proto || Object.prototype);
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

  transform(fn = ((arg) => arg, out)) {
    if(!out) out = this;

    for(let prop of ['x1', 'y1', 'x2', 'y2']) {
      const v = this.prop;
      out.prop = fn(v);
    }

    return this;
  }

  round(fn = (arg) => Math.round(arg)) {
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

  static from(iter, tp = (p) => p) {
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

  [Symbol.iterator] = [Symbol.iterator];
}

/*
 * concatenanted lib/geom/graph.js
 */

export class Graph {
  constructor() {
    this.nodes = [];
    this.connections = [];
  }

  checkForDuplicateNodes() {
    for(let i = 0; i < this.nodes.length; i++) {
      for(let j = i + 1; j < this.nodes.length; j++) {
        if(this.nodes.i.equals(this.nodes.j)) {
        }
      }
    }
  }

  checkForDuplicateConnections() {
    for(let i = 0; i < this.connections.length; i++) {
      for(let j = i + 1; j < this.connections.length; j++) {
        if(this.connections.i.equals(this.connections.j)) {
        }
      }
    }
  }

  getAdjacencyList() {
    let adjacencyList = [];

    for(let i = 0; i < this.nodes.length; i++) {
      adjacencyList.i = [];
      let node = this.nodes.i;
      node.id = i;
      let connectedNodes = this.getConnectedNodes(node);

      connectedNodes.forEach((node) => {
        adjacencyList.i.push(this.nodes.indexOf(node));
      });
    }

    return adjacencyList;
  }

  getConnectedNodes(node) {
    let myConnections = this.connections.filter((connection) => connection.node1 === node || connection.node2 === node);
    let connectedNodes = new Set();

    myConnections.forEach((connection) => {
      connectedNodes.add(connection.node1).add(connection.node2);
    });

    connectedNodes.delete(node);
    return connectedNodes;
  }

  getConnectionsFromNode(node) {
    return this.connections.filter((connection) => connection.node1 === node || connection.node2 === node);
  }

  getNeighboringNodes(node) {
    let nodeConnections = this.getConnectionsFromNode(node);
    let neighboringNodes = [];

    nodeConnections.forEach((nodeConnection) => {
      if(nodeConnection.node1 !== node) {
        neighboringNodes.push(nodeConnection.node1);
      } else if(nodeConnection.node2 !== node) {
        neighboringNodes.push(nodeConnection.node2);
      }
    });

    return neighboringNodes;
  }

  isConnected(node1, node2) {
    return this.connections.some((connection) => {
      return (connection.node1 === node1 && connection.node2 === node2) || (connection.node2 === node1 && connection.node1 === node2);
    });
  }

  addConnection(node1, node2) {
    if(!node1 || !node2) {
      return false;
    }

    if(node1.equals(node2)) {
      return false;
    }

    let node1Matches = this.nodes.filter((node) => Point.equals(node.point, node1.point));
    let node2Matches = this.nodes.filter((node) => Point.equals(node.point, node2.point));

    if(node1Matches.length > 1) {
      return false;
    }

    if(node2Matches.length > 1) {
      return false;
    }

    if(node1Matches.length === 0) {
      return;
    }

    if(node2Matches.length === 0) {
      return;
    }

    let newConnection = new Graph.Connection(node1Matches[0], node2Matches[0]);
    let duplicateConnections = this.connections.filter((connection) => connection.equals(newConnection));

    if(duplicateConnections.length > 1) {
    } else if(duplicateConnections.length === 1) {
      return;
    } else {
      this.connections.push(newConnection);
    }
  }

  addNode(newNode) {
    let duplicateNodes = this.nodes.filter((node) => Point.equals(newNode.point, node.point));
    if(duplicateNodes.length > 1) {
    }

    if(duplicateNodes.length === 0) {
      this.nodes.push(newNode);
    }
  }

  findMinimumCyclesFromSource(adjacencyListSourceIndex) {
    const adjacencyList = this.getAdjacencyList();
    var neighbors = adjacencyList.adjacencyListSourceIndex;
    let paths = [];

    for(let i = 0; i < neighbors.length; i++) {
      let path = [adjacencyListSourceIndex];
      let startingNeighborIndex = neighbors.i;
      let tmpAdjacencyList = { ...adjacencyList };
      tmpAdjacencyList.startingNeighborIndex = tmpAdjacencyList.startingNeighborIndex.filter((nodeIndex) => nodeIndex !== adjacencyListSourceIndex);
      path = path.concat(Graph.findShortestPath(tmpAdjacencyList, startingNeighborIndex, adjacencyListSourceIndex));
      paths.push(path);
    }

    return paths;
  }

  findMinimumCycles() {
    const adjacencyList = this.getAdjacencyList();
    let cycles = [];

    for(let i = 0; i < adjacencyList.length; i++) {
      let paths = this.findMinimumCyclesFromSource(i);
      cycles = cycles.concat(paths);
    }

    let cyclesToRemove = [];
    let uniqueCycles = [];

    for(let i = 0; i < cycles.length; i++) {
      if(uniqueCycles.filter((cycle) => Graph.doArraysContainSameElements(cycle, cycles.i)).length === 0) {
        uniqueCycles.push(cycles.i);
      }
    }

    let edgePairs = [];
    let edgePairCount = {};

    for(let i = 0; i < uniqueCycles.length; i++) {
      let cycle = uniqueCycles.i;

      for(let j = 1; j < cycle.length; j++) {
        let edgePair = [];

        if(cycle[j - 1] < cycle.j) {
          edgePair = [cycle[j - 1], cycle.j];
        } else {
          edgePair = [cycle.j, cycle[j - 1]];
        }

        if(edgePairCount[edgePair[0] + ',' + edgePair[1]]) {
          edgePairCount[edgePair.join(',')]++;
        } else {
          edgePairCount[edgePair.join(',')] = 1;
        }

        edgePairs.push(edgePair);
      }
    }

    let edgesOnlyUsedOnce = [];

    edgePairs.forEach((edgePair) => {
      if(edgePairCount[edgePair.join(',')] == 1) {
        edgesOnlyUsedOnce.push(edgePair);
      }
    });

    let leftoverAdjacencyList = [];

    edgesOnlyUsedOnce.forEach((edge) => {
      if(leftoverAdjacencyList[edge[0]]) {
        leftoverAdjacencyList[edge[0]].push(edge[1]);
      } else {
        leftoverAdjacencyList[edge[0]] = [edge[1]];
      }

      if(leftoverAdjacencyList[edge[1]]) {
        leftoverAdjacencyList[edge[1]].push(edge[0]);
      } else {
        leftoverAdjacencyList[edge[1]] = [edge[0]];
      }
    });

    let extraPaths = [];

    for(let i = 0; i < leftoverAdjacencyList.length; i++) {
      let neighbors = leftoverAdjacencyList.i;

      if(neighbors) {
        for(let j = 0; j < neighbors.length; j++) {
          let path = [i];
          let startingNeighborIndex = neighbors.j;
          let tmpAdjacencyList = { ...leftoverAdjacencyList };
          tmpAdjacencyList.startingNeighborIndex = tmpAdjacencyList.startingNeighborIndex.filter((nodeIndex) => nodeIndex !== i);
          path = path.concat(Graph.findShortestPath(tmpAdjacencyList, startingNeighborIndex, i));
          extraPaths.push(path);
        }
      }
    }

    let leftoverCycles = [];

    for(let i = 0; i < extraPaths.length; i++) {
      if(leftoverCycles.filter((cycle) => Graph.doArraysContainSameElements(cycle, extraPaths.i)).length === 0) {
        leftoverCycles.push(extraPaths.i);
      }
    }

    let longestCycleLength = -1;
    let longestCycleIndex = -1;

    for(let i = 0; i < leftoverCycles.length; i++) {
      if(leftoverCycles.i.length > longestCycleLength) {
        longestCycleIndex = i;
        longestCycleLength = leftoverCycles.i.length;
      }
    }

    leftoverCycles.splice(longestCycleIndex, 1);
    uniqueCycles = leftoverCycles.concat(uniqueCycles);
    return uniqueCycles;
  }

  static doArraysContainSameElements(array1, array2) {
    if(array1.length !== array2.length) {
      return false;
    } else {
      for(let i = 0; i < array1.length; i++) {
        if(array2.includes(array1.i) === false) {
          return false;
        }
      }
    }

    return true;
  }

  static findShortestPath(adjacencyList, source, target) {
    if(source == target) {
      print('SOURCE AND PATH ARE SAME');
      return [target];
    }

    let visitQueue = [source];
    let visitedStatusList = { source: true };
    let predecessorList = {};
    let nextInQueue = 0;

    while(nextInQueue < visitQueue.length) {
      let node = visitQueue[nextInQueue++];
      let neighbors = adjacencyList.node;

      for(let i = 0; i < neighbors.length; i++) {
        var neighbor = neighbors.i;

        if(!visitedStatusList.neighbor) {
          visitedStatusList.neighbor = true;

          if(neighbor === target) {
            let path = [target];

            while(node !== source) {
              path.push(node);
              node = predecessorList.node;
            }

            path.push(node);
            path.reverse();
            return path;
          }

          predecessorList.neighbor = node;
          visitQueue.push(neighbor);
        }
      }
    }

    print('there is no path from ' + source + ' to ' + target);
  }
}

Graph.Node = class {
  constructor(point, connections) {
    this.point = point;
  }

  equals(node) {
    return Point.equals(node.point, this.point);
  }
};

Graph.Connection = class {
  constructor(node1, node2) {
    this.node1 = node1;
    this.node2 = node2;
  }

  equals(connection) {
    return (this.node1.equals(connection.node1) && this.node2.equals(connection.node2)) || (this.node2.equals(connection.node1) && this.node1.equals(connection.node2));
  }
};

/*
 * concatenanted lib/geom/point.js
 */

const SymSpecies = Util.tryCatch(
  () => Symbol,
  (sym) => sym.species
);

const CTOR = (obj) => {
  if(obj.SymSpecies) return obj.SymSpecies;
  let p = Object.getPrototypeOf(obj);
  if(p.SymSpecies) return p.SymSpecies;
  return p.constructor;
};

export function Point(arg) {
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
    const matches = [...arg.matchAll(/([-+]?d*.?d+)(?:[eE]([-+]?d+))?/g)];
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

Point.prototype.move = function (x, y) {
  this.x += x;
  this.y += y;
  return this;
};

Point.prototype.moveTo = function (x, y) {
  this.x = x;
  this.y = y;
  return this;
};

Point.prototype.clear = function (x, y) {
  this.x = 0;
  this.y = 0;
  return this;
};

Point.prototype.set = function (fn) {
  if(typeof fn != 'function') {
    Point.apply(this, [...arguments]);
    return this;
  }

  return fn(this.x, this.y);
};

Point.prototype.clone = function () {
  const ctor = this[Symbol.species] || this.constructor[Symbol.species];
  return new ctor({ x: this.x, y: this.y });
};

Point.prototype.sum = function (...args) {
  const p = new Point(...args);
  let r = new Point(this.x, this.y);
  r.x += p.x;
  r.y += p.y;
  return r;
};

Point.prototype.add = function (...args) {
  const other = new Point(...args);
  this.x += other.x;
  this.y += other.y;
  return this;
};

Point.prototype.diff = function (arg) {
  let { x, y } = this;

  var fn = function (other) {
    let r = new Point(x, y);
    return r.sub(other);
  };

  if(arg) return fn(arg);
  return fn;
};

Point.prototype.sub = function (...args) {
  const other = new Point(...args);
  this.x -= other.x;
  this.y -= other.y;
  return this;
};

Point.prototype.prod = function (f) {
  const o = isPoint(f) ? f : { x: f, y: f };
  return new Point(this.x * o.x, this.y * o.y);
};

Point.prototype.mul = function (f) {
  const o = isPoint(f) ? f : { x: f, y: f };
  this.x *= o.x;
  this.y *= o.y;
  return this;
};

Point.prototype.quot = function (other) {
  other = isPoint(other) ? other : { x: other, y: other };
  return new Point(this.x / other.x, this.y / other.y);
};

Point.prototype.div = function (other) {
  other = isPoint(other) ? other : { x: other, y: other };
  this.x /= other.x;
  this.y /= other.y;
  return this;
};

Point.prototype.comp = function () {
  return new Point({ x: -this.x, y: -this.y });
};

Point.prototype.neg = function () {
  this.x *= -1;
  this.y *= -1;
  return this;
};

Point.prototype.distanceSquared = function (other = { x: 0, y: 0 }) {
  return (other.y - this.y) * (other.y - this.y) + (other.x - this.x) * (other.x - this.x);
};

Point.prototype.distance = function (other = { x: 0, y: 0 }) {
  return Math.sqrt(Point.prototype.distanceSquared.call(this, other));
};

Point.prototype.equals = function (other) {
  return +this.x == +other.x && +this.y == +other.y;
};

Point.prototype.round = function (precision = (0.001, digits, (type = 'round'))) {
  let { x, y } = this;
  this.x = Util.roundTo(x, precision, digits, type);
  this.y = Util.roundTo(y, precision, digits, type);
  return this;
};

Point.prototype.ceil = function () {
  let { x, y } = this;
  this.x = Math.ceil(x);
  this.y = Math.ceil(y);
  return this;
};

Point.prototype.floor = function () {
  let { x, y } = this;
  this.x = Math.floor(x);
  this.y = Math.floor(y);
  return this;
};

Point.prototype.dot = function (other) {
  return this.x * other.x + this.y * other.y;
};

Point.prototype.values = function () {
  return [this.x, this.y];
};

Point.prototype.fromAngle = function (angle, dist = 1.0) {
  this.x = Math.cos(angle) * dist;
  this.y = Math.sin(angle) * dist;
  return this;
};

Point.prototype.toAngle = function (deg = false) {
  return Math.atan2(this.x, this.y) * (deg ? 180 / Math.PI : 1);
};

Point.prototype.angle = function (other, deg = false) {
  other = other || { x: 0, y: 0 };
  return Point.prototype.diff.call(this, other).toAngle(deg);
};

Point.prototype.rotate = function (angle, origin = { x: 0, y: 0 }) {
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

Util.defineGetter(Point.prototype, Symbol.iterator, function () {
  const { x, y } = this;
  let a = [x, y];
  return a[Symbol.iterator].bind(a);
});

Point.prototype.toString = function (opts = {}) {
  const { precision, unit, separator, left, right } = opts;
  const x = Util.roundTo(this.x, precision);
  const y = Util.roundTo(this.y, precision);
  return `${left}${x}${unit}${separator}${y}${unit}${right}`;
};

Util.defineGetterSetter(
  Point.prototype,
  Symbol.toStringTag,
  function () {
    return `Point{ ${Point.prototype.toSource.call(this)}`;
  },
  () => {},
  false
);

Point.prototype.toSource = function (opts = {}) {
  const { asArray, plainObj, pad, showNew } = opts;
  let x = pad(this.x + '');
  let y = pad(this.y + '');
  let c = (t) => t;
  if(typeof this != 'object' || this === null) return '';
  if(asArray) return `[${x},${y}]`;
  if(plainObj) return `{x:${x},y:${y}}`;
  return `${c(showNew ? 'new ' : '', 1, 31)}${c('Point', 1, 33)}${c('(', 1, 36)}${c(x, 1, 32)}${c(',', 1, 36)}${c(y, 1, 32)}${c(')', 1, 36)}`;
};

Point.prototype.toObject = function (proto = Point.prototype) {
  const { x, y } = this;
  const obj = { x, y };
  Object.setPrototypeOf(obj, proto);
  return obj;
};

Point.prototype.toCSS = function (precision = (0.001, (edges = ['left', 'top']))) {
  return { [[edges[0]]]: Util.roundTo(this.x, precision) + 'px', [[edges[1]]]: Util.roundTo(this.y, precision) + 'px' };
};

Point.prototype.toFixed = function (digits) {
  return new Point(+this.x.toFixed(digits), +this.y.toFixed(digits));
};

Point.prototype.isNull = function () {
  return this.x == 0 && this.y == 0;
};

Point.prototype.inside = function (rect) {
  return this.x >= rect.x && this.x < rect.x + rect.width && this.y >= rect.y && this.y < rect.y + rect.height;
};

Point.prototype.transform = function (m) {
  if(Util.isObject(m) && typeof m.toMatrix == 'function') m = m.toMatrix();
  if(Util.isObject(m) && typeof m.transform_point == 'function') return m.transform_point(this);
  const x = m[0] * this.x + m[1] * this.y + m[2];
  const y = m[3] * this.x + m[4] * this.y + m[5];
  this.x = x;
  this.y = y;
  return this;
};

Point.prototype.scaleTo = function (minmax) {
  return new Point({ x: (this.x - minmax.x1) / (minmax.x2 - minmax.x1), y: (this.y - minmax.y1) / (minmax.y2 - minmax.y1) });
};

Point.prototype.normal = function () {
  let d = Point.prototype.distance.call(this);
  return new Point({ x: this.x / d, y: this.y / d });
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
Point.toSource = (point, { space = ' ', padding = ' ', separator = ',' }) => `{${padding}x:${space}${point.x}${separator}y:${space}${point.y}${padding}}`;
const isPoint = (o) => o && ((o.x !== undefined && o.y !== undefined) || ((o.left !== undefined || o.right !== undefined) && (o.top !== undefined || o.bottom !== undefined)) || o instanceof Point || Object.getPrototypeOf(o).constructor === Point);
Point.isPoint = isPoint;
Util.defineInspect(Point.prototype, 'x', 'y');

Point.bind = (o, p, gen) => {
  const [x, y] = p || ['x', 'y'];
  if(!gen) gen = (k) => (v) => (v === undefined ? o.k : (o.k = v));
  return Util.bindProperties(new Point(0, 0), o, { x, y }, gen);
};
export default Point;

Util.defineGetter(Point, Symbol.species, function () {
  return this;
});
const ImmutablePoint = Util.immutableClass(Point);

Util.defineGetter(ImmutablePoint, Symbol.species, function () {
  return ImmutablePoint;
});

/*
 * concatenanted lib/geom/line.js
 */

export function Line(x1, y1, x2, y2) {
  let obj;
  let arg;
  let args = [...arguments];
  let ret;

  if(args.length >= 4 && args.every((arg) => !isNaN(parseFloat(arg)))) {
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
  } else if(arg && arg.length >= 4 && arg.slice(0, 4).every((arg) => !isNaN(parseFloat(arg)))) {
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
const isLine = (obj) => (Util.isObject(obj) && ['x1', 'y1', 'x2', 'y2'].every((prop) => obj.prop !== undefined)) || ['a', 'b'].every((prop) => isPoint(obj.prop));

Line.prototype.intersect = function (other) {
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

Line.prototype.direction = function () {
  var dist = Point.prototype.distance.call(this.a, this.b);
  return Point.prototype.quot.call(Line.prototype.getSlope.call(this), dist);
};

Line.prototype.getVector = function () {
  return { x: this.x2 - this.x1, y: this.y2 - this.y1 };
};
Object.defineProperty(Line.prototype, 'vector', { get: Line.prototype.getVector });

Line.prototype.getSlope = function () {
  return (this.y2 - this.y1) / (this.x2 - this.x1);
};
Object.defineProperty(Line.prototype, 'slope', { get: Line.prototype.getSlope });

Line.prototype.yIntercept = function () {
  let v = Line.prototype.getVector.call(this);

  if(v.x !== 0) {
    let slope = v.y / v.x;
    return [this.a.y - this.a.x * slope, slope || 0];
  }
};

Line.prototype.xIntercept = function () {
  let v = Line.prototype.getVector.call(this);

  if(v.y !== 0) {
    let slope = v.x / v.y;
    return [this.a.x - this.a.y * slope, slope || 0];
  }
};

Line.prototype.isHorizontal = function () {
  return Line.prototype.getVector.call(this).y === 0;
};

Line.prototype.isVertical = function () {
  return Line.prototype.getVector.call(this).x === 0;
};

Line.prototype.isNull = function () {
  return this.x1 == 0 && this.y1 == 0 && this.x2 == 0 && this.y2 == 0;
};

Line.prototype.equations = function () {
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

Line.prototype.functions = function () {
  let i;
  let fns = {};

  if((i = Line.prototype.yIntercept.call(this))) {
    let [y0, myx] = i;
    fns.y = (x) => y0 + myx * x;
  } else {
    let { y } = this.a;
    fns.y = new Function('x', `return ${y}`);
  }

  if((i = Line.prototype.xIntercept.call(this))) {
    let [x0, mxy] = i;
    fns.x = (y) => x0 + mxy * y;
  } else {
    let { x } = this.a;
    fns.x = new Function('y', `return ${x}`);
  }

  return fns;
};

Line.prototype.angle = function () {
  return Point.prototype.angle.call(Line.prototype.getVector.call(this));
};

Line.prototype.getLength = function () {
  const { a, b } = this;
  const { x1, y1, x2, y2 } = this;
  return Point.prototype.distance.call(a, b);
};

Line.prototype.endpointDist = function (point) {
  return Math.min(point.distance(this.a), point.distance(this.b));
};

Line.prototype.matchEndpoints = function (arr) {
  const { a, b } = this;
  return [...arr.entries()].filter(([i, otherLine]) => !Line.prototype.equals.call(this, otherLine) && (Point.prototype.equals.call(a, otherLine.a) || Point.prototype.equals.call(b, otherLine.b) || Point.prototype.equals.call(b, otherLine.a) || Point.prototype.equals.call(a, otherLine.b)));
};

Line.prototype.distanceToPointSquared = function (p) {
  const { a, b } = this;
  var l2 = Point.prototype.distanceSquared.call(a, b);
  if(l2 === 0) return Point.prototype.distanceSquared.call(p, a);
  var t = ((p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y)) / l2;
  t = Math.max(0, Math.min(1, t));
  return Point.prototype.distanceSquared.call(p, new Point(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)));
};

Line.prototype.distanceToPoint = function (p) {
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

Line.prototype.pointAt = function (pos) {
  return new Point(pos * (this.x2 - this.x1) + this.x1, pos * (this.y2 - this.y1) + this.y1);
};

Line.prototype.transform = function (m) {
  this.a = this.a.transform(m);
  this.b = this.b.transform(m);
  return this;
};

Line.prototype.bbox = function () {
  const { x1, y1, x2, y2 } = this;
  return new BBox(x1, y1, x2, y2);
};

Line.prototype.add = function (other) {
  const { x1, y1, x2, y2 } = Line(...arguments);
  this.x1 += x1;
  this.y1 += y1;
  this.x2 += x2;
  this.y2 += y2;
  return this;
};

Line.prototype.points = function () {
  const { a, b } = this;
  return [a, b];
};

Line.prototype.diff = function (other) {
  other = Line(...arguments);
  return new Line(Point.diff(this.a, other.a), Point.diff(this.b, other.b));
};

Line.prototype.inspect = function () {
  const { x1, y1, x2, y2 } = this;
  return 'Line{ ' + inspect({ x1, y1, x2, y2 }) + ' }';
};

Line.prototype.toString = function () {
  const { x1, y1, x2, y2 } = this;
  return Point.toString(this.a || Point(x1, y1)) + ' -> ' + Point.toString(this.b || Point(x2, y2));
};

Line.prototype.toSource = function () {
  let { a, b } = this;
  return `new Line(${a.x},${a.y},${b.x},${b.y})`;
};

Line.prototype.reverse = function () {
  const { a, b } = this;
  return new Line(b, a);
};

Line.prototype.toObject = function (t = (num) => num) {
  const { x1, y1, x2, y2 } = this;
  const obj = { x1: t(x1), y1: t(y1), x2: t(x2), y2: t(y2) };
  return obj;
};

Line.prototype.clone = function () {
  const ctor = this.constructor[Symbol.species];
  const { x1, y1, x2, y2 } = this;
  return new ctor(x1, y1, x2, y2);
};

Line.prototype.round = function (precision = (0.001, digits)) {
  let { x1, y1, x2, y2 } = this;
  this.a.x = Util.roundTo(x1, precision, digits);
  this.a.y = Util.roundTo(y1, precision, digits);
  this.b.x = Util.roundTo(x2, precision, digits);
  this.b.y = Util.roundTo(y2, precision, digits);
  return this;
};

Line.prototype.some = function (pred) {
  return pred(this.a) || pred(this.b);
};

Line.prototype.every = function (pred) {
  return pred(this.a) && pred(this.b);
};

Line.prototype.includes = function (point) {
  return Point.prototype.equals.call(this.a, point) || Point.prototype.equals.call(this.b, point);
};

Line.prototype.equals = function (other) {
  other = Line(other);
  if(Point.equals(this.a, other.a) && Point.equals(this.b, other.b)) return 1;
  if(Point.equals(this.a, other.b) && Point.equals(this.b, other.a)) return -1;
  return false;
};

Line.prototype.indexOf = function (point) {
  let i = 0;

  for(let p of [this.a, this.b]) {
    if(Point.prototype.equals.call(p, point)) return i;
    i++;
  }

  return -1;
};

Line.prototype.lastIndexOf = function (point) {
  let i = 0;

  for(let p of [this.b, this.a]) {
    if(Point.prototype.equals.call(p, point)) return i;
    i++;
  }

  return -1;
};

Line.prototype.map = function (fn) {
  let i = 0;
  let r = [];

  for(let p of [this.a, this.b]) {
    r.push(fn(p, i, this));
    i++;
  }

  return new Line(...r);
};

Line.prototype.swap = function (fn) {
  return new Line(this.a, this.b).reverse();
};

Line.prototype.toPoints = function () {
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
  if(!gen) gen = (k) => (v) => (v === undefined ? o.k : (o.k = v));
  let proxy = { a: Point.bind(o, [x1, y1], gen), b: Point.bind(o, [x2, y2], gen) };
  return Object.setPrototypeOf(proxy, Line.prototype);
};

Util.defineGetter(Line, Symbol.species, function () {
  return this;
});
const ImmutableLine = Util.immutableClass(Line);

Util.defineGetter(ImmutableLine, Symbol.species, function () {
  return ImmutableLine;
});

/*
 * concatenanted lib/geom/intersection.js
 */

export class Intersection {
  constructor(line1, line2, intersectionPoint) {
    this.line1 = line1;
    this.line2 = line2;
    this.point = intersectionPoint;
  }

  static findIntersection(line1, line2) {
    let denominator, a, b, numerator1, numerator2;
    let result = new Intersection(line1, line2, {});
    denominator = (line2.y2 - line2.y1) * (line1.x2 - line1.x1) - (line2.x2 - line2.x1) * (line1.y2 - line1.y1);
    if(denominator == 0) return null;
    a = line1.y1 - line2.y1;
    b = line1.x1 - line2.x1;
    numerator1 = (line2.x2 - line2.x1) * a - (line2.y2 - line2.y1) * b;
    numerator2 = (line1.x2 - line1.x1) * a - (line1.y2 - line1.y1) * b;
    a = numerator1 / denominator;
    b = numerator2 / denominator;
    result.point.x = line1.x1 + a * (line1.x2 - line1.x1);
    result.point.y = line1.y1 + a * (line1.y2 - line1.y1);
    result.line1 = line1;
    result.line2 = line2;

    if(a > 0 && a < 1 && b > 0 && b < 1) {
      return result;
    } else {
      return null;
    }
  }

  static equals(intersection1, intersection2) {
    return Point.equals(intersection1.point, intersection2.point) && ((Line.equals(intersection1.line1, intersection2.line1) && Line.equals(intersection1.line2, intersection2.line2)) || (Line.equals(intersection1.line1, intersection2.line2) && Line.equals(intersection1.line2, intersection2.line1)));
  }
}

/*
 * concatenanted lib/geom/line.js
 */

export function Line(x1, y1, x2, y2) {
  let obj;
  let arg;
  let args = [...arguments];
  let ret;

  if(args.length >= 4 && args.every((arg) => !isNaN(parseFloat(arg)))) {
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
  } else if(arg && arg.length >= 4 && arg.slice(0, 4).every((arg) => !isNaN(parseFloat(arg)))) {
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
const isLine = (obj) => (Util.isObject(obj) && ['x1', 'y1', 'x2', 'y2'].every((prop) => obj.prop !== undefined)) || ['a', 'b'].every((prop) => isPoint(obj.prop));

Line.prototype.intersect = function (other) {
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

Line.prototype.direction = function () {
  var dist = Point.prototype.distance.call(this.a, this.b);
  return Point.prototype.quot.call(Line.prototype.getSlope.call(this), dist);
};

Line.prototype.getVector = function () {
  return { x: this.x2 - this.x1, y: this.y2 - this.y1 };
};
Object.defineProperty(Line.prototype, 'vector', { get: Line.prototype.getVector });

Line.prototype.getSlope = function () {
  return (this.y2 - this.y1) / (this.x2 - this.x1);
};
Object.defineProperty(Line.prototype, 'slope', { get: Line.prototype.getSlope });

Line.prototype.yIntercept = function () {
  let v = Line.prototype.getVector.call(this);

  if(v.x !== 0) {
    let slope = v.y / v.x;
    return [this.a.y - this.a.x * slope, slope || 0];
  }
};

Line.prototype.xIntercept = function () {
  let v = Line.prototype.getVector.call(this);

  if(v.y !== 0) {
    let slope = v.x / v.y;
    return [this.a.x - this.a.y * slope, slope || 0];
  }
};

Line.prototype.isHorizontal = function () {
  return Line.prototype.getVector.call(this).y === 0;
};

Line.prototype.isVertical = function () {
  return Line.prototype.getVector.call(this).x === 0;
};

Line.prototype.isNull = function () {
  return this.x1 == 0 && this.y1 == 0 && this.x2 == 0 && this.y2 == 0;
};

Line.prototype.equations = function () {
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

Line.prototype.functions = function () {
  let i;
  let fns = {};

  if((i = Line.prototype.yIntercept.call(this))) {
    let [y0, myx] = i;
    fns.y = (x) => y0 + myx * x;
  } else {
    let { y } = this.a;
    fns.y = new Function('x', `return ${y}`);
  }

  if((i = Line.prototype.xIntercept.call(this))) {
    let [x0, mxy] = i;
    fns.x = (y) => x0 + mxy * y;
  } else {
    let { x } = this.a;
    fns.x = new Function('y', `return ${x}`);
  }

  return fns;
};

Line.prototype.angle = function () {
  return Point.prototype.angle.call(Line.prototype.getVector.call(this));
};

Line.prototype.getLength = function () {
  const { a, b } = this;
  const { x1, y1, x2, y2 } = this;
  return Point.prototype.distance.call(a, b);
};

Line.prototype.endpointDist = function (point) {
  return Math.min(point.distance(this.a), point.distance(this.b));
};

Line.prototype.matchEndpoints = function (arr) {
  const { a, b } = this;
  return [...arr.entries()].filter(([i, otherLine]) => !Line.prototype.equals.call(this, otherLine) && (Point.prototype.equals.call(a, otherLine.a) || Point.prototype.equals.call(b, otherLine.b) || Point.prototype.equals.call(b, otherLine.a) || Point.prototype.equals.call(a, otherLine.b)));
};

Line.prototype.distanceToPointSquared = function (p) {
  const { a, b } = this;
  var l2 = Point.prototype.distanceSquared.call(a, b);
  if(l2 === 0) return Point.prototype.distanceSquared.call(p, a);
  var t = ((p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y)) / l2;
  t = Math.max(0, Math.min(1, t));
  return Point.prototype.distanceSquared.call(p, new Point(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)));
};

Line.prototype.distanceToPoint = function (p) {
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

Line.prototype.pointAt = function (pos) {
  return new Point(pos * (this.x2 - this.x1) + this.x1, pos * (this.y2 - this.y1) + this.y1);
};

Line.prototype.transform = function (m) {
  this.a = this.a.transform(m);
  this.b = this.b.transform(m);
  return this;
};

Line.prototype.bbox = function () {
  const { x1, y1, x2, y2 } = this;
  return new BBox(x1, y1, x2, y2);
};

Line.prototype.add = function (other) {
  const { x1, y1, x2, y2 } = Line(...arguments);
  this.x1 += x1;
  this.y1 += y1;
  this.x2 += x2;
  this.y2 += y2;
  return this;
};

Line.prototype.points = function () {
  const { a, b } = this;
  return [a, b];
};

Line.prototype.diff = function (other) {
  other = Line(...arguments);
  return new Line(Point.diff(this.a, other.a), Point.diff(this.b, other.b));
};

Line.prototype.inspect = function () {
  const { x1, y1, x2, y2 } = this;
  return 'Line{ ' + inspect({ x1, y1, x2, y2 }) + ' }';
};

Line.prototype.toString = function () {
  const { x1, y1, x2, y2 } = this;
  return Point.toString(this.a || Point(x1, y1)) + ' -> ' + Point.toString(this.b || Point(x2, y2));
};

Line.prototype.toSource = function () {
  let { a, b } = this;
  return `new Line(${a.x},${a.y},${b.x},${b.y})`;
};

Line.prototype.reverse = function () {
  const { a, b } = this;
  return new Line(b, a);
};

Line.prototype.toObject = function (t = (num) => num) {
  const { x1, y1, x2, y2 } = this;
  const obj = { x1: t(x1), y1: t(y1), x2: t(x2), y2: t(y2) };
  return obj;
};

Line.prototype.clone = function () {
  const ctor = this.constructor[Symbol.species];
  const { x1, y1, x2, y2 } = this;
  return new ctor(x1, y1, x2, y2);
};

Line.prototype.round = function (precision = (0.001, digits)) {
  let { x1, y1, x2, y2 } = this;
  this.a.x = Util.roundTo(x1, precision, digits);
  this.a.y = Util.roundTo(y1, precision, digits);
  this.b.x = Util.roundTo(x2, precision, digits);
  this.b.y = Util.roundTo(y2, precision, digits);
  return this;
};

Line.prototype.some = function (pred) {
  return pred(this.a) || pred(this.b);
};

Line.prototype.every = function (pred) {
  return pred(this.a) && pred(this.b);
};

Line.prototype.includes = function (point) {
  return Point.prototype.equals.call(this.a, point) || Point.prototype.equals.call(this.b, point);
};

Line.prototype.equals = function (other) {
  other = Line(other);
  if(Point.equals(this.a, other.a) && Point.equals(this.b, other.b)) return 1;
  if(Point.equals(this.a, other.b) && Point.equals(this.b, other.a)) return -1;
  return false;
};

Line.prototype.indexOf = function (point) {
  let i = 0;

  for(let p of [this.a, this.b]) {
    if(Point.prototype.equals.call(p, point)) return i;
    i++;
  }

  return -1;
};

Line.prototype.lastIndexOf = function (point) {
  let i = 0;

  for(let p of [this.b, this.a]) {
    if(Point.prototype.equals.call(p, point)) return i;
    i++;
  }

  return -1;
};

Line.prototype.map = function (fn) {
  let i = 0;
  let r = [];

  for(let p of [this.a, this.b]) {
    r.push(fn(p, i, this));
    i++;
  }

  return new Line(...r);
};

Line.prototype.swap = function (fn) {
  return new Line(this.a, this.b).reverse();
};

Line.prototype.toPoints = function () {
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
  if(!gen) gen = (k) => (v) => (v === undefined ? o.k : (o.k = v));
  let proxy = { a: Point.bind(o, [x1, y1], gen), b: Point.bind(o, [x2, y2], gen) };
  return Object.setPrototypeOf(proxy, Line.prototype);
};

Util.defineGetter(Line, Symbol.species, function () {
  return this;
});
const ImmutableLine = Util.immutableClass(Line);

Util.defineGetter(ImmutableLine, Symbol.species, function () {
  return ImmutableLine;
});

/*
 * concatenanted lib/geom/lineList.js
 */

export class LineList extends Array {
  constructor(lines) {
    super();

    if(Util.isArray(lines) || Util.isGenerator(lines)) {
      for(let line of lines) {
        if(!(line instanceof Line)) line = Util.isArray(line) ? new Line(...line) : new Line(line);
        this.push(line);
      }
    }
  }

  bbox() {
    let bb = new BBox();
    for(let line of this) bb.update(Line.prototype.toObject.call(line));
    return bb;
  }
}

if(!Util.isBrowser()) {
  let c = Util.coloring();
  const sym = Symbol.for('nodejs.util.inspect.custom');

  LineList.prototype.sym = function () {
    return `${c.text('LineList', 1, 31)}${c.text('(', 1, 36)}${c.text(this.length, 1, 35) + c.code(1, 36)}) [\n  ${this.map((line) => line.sym()).join(',\n  ')}\n${c.text(']', 1, 36)}`;
  };
}

Util.defineGetter(LineList, Symbol.species, function () {
  return this;
});
const ImmutableLineList = Util.immutableClass(LineList);

Util.defineGetter(ImmutableLineList, Symbol.species, function () {
  return ImmutableLineList;
});

/*
 * concatenanted lib/geom/matrix.js
 */

export function Matrix(arg) {
  let ret = this instanceof Matrix || new.target === Matrix ? this : [undefined, 0, 0, undefined, 0, 0, undefined, 0, 0];
  const isObj = Util.isObject(arg);

  if(isObj && arg.xx !== undefined && arg.yx !== undefined && arg.xy !== undefined && arg.yy !== undefined && arg.x0 !== undefined && arg.y0 !== undefined) {
    ret[0] = arg.xx;
    ret[1] = arg.xy;
    ret[2] = arg.x0;
    ret[3] = arg.yx;
    ret[4] = arg.yy;
    ret[5] = arg.y0;
  } else if(isObj && arg.a !== undefined && arg.b !== undefined && arg.c !== undefined && arg.d !== undefined && arg.e !== undefined && arg.f !== undefined) {
    ret[0] = arg.a;
    ret[1] = arg.c;
    ret[2] = arg.e;
    ret[3] = arg.b;
    ret[4] = arg.d;
    ret[5] = arg.f;
  } else if(arg instanceof Array) {
    Matrix.prototype.init.call(ret, arg);
  } else if(typeof arg === 'number') {
    Matrix.prototype.init.apply(ret, arguments);
  } else if(typeof arg === 'string' && /matrix\([^)]*\)/.test(arg)) {
    let [xx, xy, x0, yx, yy, y0] = [...arg.matchAll(/[-.0-9]+/g)].map((m) => parseFloat(m[0]));
    ret[0] = xx;
    ret[1] = xy;
    ret[2] = x0;
    ret[3] = yx;
    ret[4] = yy;
    ret[5] = y0;
  } else {
    Array.prototype.splice.call(ret, 0, ret.length, 1, 0, 0, 0, 1, 0, 0, 0, 1);
  }

  for(let i = 0; i < 9; i++) if(ret.i === undefined) ret.i = [1, 0, 0, 0, 1, 0, 0, 0, 1].i;

  if(!(this instanceof Matrix)) return ret;
}

Object.defineProperty(Matrix, Symbol.species, {
  get() {
    return Matrix;
  }
});

Matrix.prototype[Symbol.toStringTag] = function () {
  return Matrix.prototype.toString.apply(this, arguments);
};
Matrix.prototype[Symbol.isConcatSpreadable] = false;
Object.defineProperty(Matrix.prototype, 'length', { value: 6, enumerable: false, writable: true, configurable: false });
Matrix.prototype.keys = ['xx', 'xy', 'x0', 'yx', 'yy', 'y0'];
Matrix.prototype.keySeq = ['xx', 'yx', 'xy', 'yy', 'x0', 'y0'];
const keyIndexes = { xx: 0, a: 0, xy: 1, c: 1, x0: 2, tx: 2, e: 2, yx: 3, b: 3, yy: 4, d: 4, y0: 5, ty: 5, f: 5 };

Matrix.prototype.at = function (col, row = 0) {
  return this[row * 3 + col];
};

Matrix.prototype.get = function (field) {
  if(typeof field == 'number' && field < this.length) return this.field;
  if((field = keyIndexes.field)) return this.field;
};

const MatrixProps = (obj = {}) =>
  Object.entries(keyIndexes).reduce(
    (acc, [k, i]) => ({
      ...acc,
      [[k]]: {
        get() {
          return this.i;
        },
        set(v) {
          this.i = v;
        },
        enumerable: true
      }
    }),
    obj
  );
Object.defineProperties(Matrix.prototype, MatrixProps());
Matrix.propDescriptors = MatrixProps;

Matrix.prototype.init = function (...args) {
  if(args.length == 1) args = args[0];
  if(args.length < 9) args = args.concat(Array.prototype.slice.call(Matrix.IDENTITY, args.length));
  Array.prototype.splice.call(this, 0, this.length, ...args);
  return this;
};

Matrix.prototype.set_row = function (...args) {
  const start = args.shift() * 3;
  const end = Math.max(3, args.length);

  for(let i = 0; i < end; i++) this[start + i] = args.i;

  return this;
};

Matrix.prototype.multiply = function (...args) {
  return this.clone().multiplySelf(...args);
};

Matrix.prototype.multiplySelf = function (...args) {
  for(let arg of args) {
    if(!(arg instanceof Matrix)) throw new Error('Not a Matrix: ' + arg.constructor);

    this.init([this[0] * arg[0] + this[1] * arg[3], this[0] * arg[1] + this[1] * arg[4], this[0] * arg[2] + this[1] * arg[5] + this[2], this[3] * arg[0] + this[4] * arg[3], this[3] * arg[1] + this[4] * arg[4], this[3] * arg[2] + this[4] * arg[5] + this[5]]);
  }

  return this;
};

Matrix.prototype.multiply_self = function (...args) {
  for(let m of args) {
    if(!(m instanceof Matrix)) m = new Matrix(m);
    Matrix.prototype.init.call(this, this[0] * m[0] + this[1] * m[3], this[0] * m[1] + this[1] * m[4], this[0] * m[2] + this[1] * m[5] + this[2], this[3] * m[0] + this[4] * m[3], this[3] * m[1] + this[4] * m[4], this[3] * m[2] + this[4] * m[5] + this[5]);
  }

  return this;
};

Matrix.prototype.toObject = function () {
  const { xx, xy, x0, yx, yy, y0 } = this;
  return { xx, xy, x0, yx, yy, y0 };
};

Matrix.prototype.entries = function () {
  return Object.entries(Matrix.prototype.toObject.call(this));
};

Matrix.prototype.clone = function () {
  const ctor = this.constructor[Symbol.species];
  return new ctor(Array.from(this));
};

Matrix.prototype.row = function (row) {
  let i = row * 3;
  return Array.prototype.slice.call(this, i, i + 3);
};

Matrix.prototype.rows = function () {
  let ret = [];

  for(let i = 0; i < 9; i += 3) ret.push([this[i + 0], this[i + 1], this[i + 2]]);

  return ret;
};

Matrix.prototype.toArray = function () {
  return Array.from(this);
};

Matrix.prototype.isIdentity = function () {
  return Util.equals(this, Matrix.IDENTITY);
};

Matrix.prototype.determinant = function () {
  return this[0] * (this[4] * this[8] - this[5] * this[7]) + this[1] * (this[5] * this[6] - this[3] * this[8]) + this[2] * (this[3] * this[7] - this[4] * this[6]);
};

Matrix.prototype.invert = function () {
  const det = Matrix.prototype.determinant.call(this);
  return new Matrix([
    (this[4] * this[8] - this[5] * this[7]) / det,
    (this[2] * this[7] - this[1] * this[8]) / det,
    (this[1] * this[5] - this[2] * this[4]) / det,
    (this[5] * this[6] - this[3] * this[8]) / det,
    (this[0] * this[8] - this[2] * this[6]) / det,
    (this[2] * this[3] - this[0] * this[5]) / det,
    (this[3] * this[7] - this[4] * this[6]) / det,
    (this[6] * this[1] - this[0] * this[7]) / det,
    (this[0] * this[4] - this[1] * this[3]) / det
  ]);
};

Matrix.prototype.scalar_product = function (f) {
  return new Matrix({ xx: this[0] * f, xy: this[1] * f, x0: this[2] * f, yx: this[3] * f, yy: this[4] * f, y0: this[5] * f });
};

Matrix.prototype.toSource = function (construct = (false, (multiline = true))) {
  const nl = multiline ? '\n' : '';
  const rows = Matrix.prototype.rows.call(this);
  const src = `${rows.map((row) => row.join(',')).join(multiline ? ',\n ' : ',')}`;
  return construct ? `new Matrix([${nl}${src}${nl}])` : `[${src}]`;
};

Matrix.prototype.toString = function (separator = ' ') {
  let rows = Matrix.prototype.rows.call(this);
  let name = rows[0].length == 3 ? 'matrix' : 'matrix3d';

  if(rows[0].length == 3) {
    rows = [['a', 'b', 'c', 'd', 'e', 'f'].map((k) => this[keyIndexes.k])];
  }

  return `${name}(` + rows.map((row) => row.join(',' + separator)).join(',' + separator) + ')';
};

Matrix.prototype.toSVG = function () {
  return 'matrix(' + ['a', 'b', 'c', 'd', 'e', 'f'].map((k) => this[keyIndexes.k]).join(',') + ')';
};

Matrix.prototype.toDOM = function (ctor = DOMMatrix) {
  const rows = Matrix.prototype.rows.call(this);
  const [a, c, e] = rows[0];
  const [b, d, f] = rows[1];
  return new ctor([a, b, c, d, e, f]);
};

Matrix.prototype.toJSON = function () {
  const rows = Matrix.prototype.rows.call(this);
  const [a, c, e] = rows[0];
  const [b, d, f] = rows[1];
  return { a, b, c, d, e, f };
};

Matrix.fromJSON = (obj) => {
  return new Matrix(obj);
};

Matrix.fromDOM = (matrix) => {
  const { a, b, c, d, e, f } = matrix;
  return new Matrix([a, c, e, b, d, f]);
};

Matrix.prototype.equals = function (other) {
  return Array.prototype.every.call((n, i) => other.i == n);
};

Matrix.prototype.transform_distance = function (d) {
  const k = 'x' in d && 'y' in d ? ['x', 'y'] : 'width' in d && 'height' in d ? ['width', 'height'] : [0, 1];
  const x = this[0] * d[k[0]] + this[2] * d[k[1]];
  const y = this[1] * d[k[0]] + this[3] * d[k[1]];
  d[k[0]] = x;
  d[k[1]] = y;
  return d;
};

Matrix.prototype.transform_xy = function (x, y) {
  const m0 = this.row(0);
  const m1 = this.row(1);
  return [m0[0] * x + m0[1] * y + m0[2], m1[0] * x + m1[1] * y + m0[2]];
};

Matrix.prototype.transform_point = function (p) {
  const k = 'x' in p && 'y' in p ? ['x', 'y'] : [0, 1];
  const m0 = this.row(0);
  const m1 = this.row(1);
  const x = m0[0] * p[k[0]] + m0[1] * p[k[1]] + m0[2];
  const y = m1[0] * p[k[0]] + m1[1] * p[k[1]] + m1[2];
  p[k[0]] = x;
  p[k[1]] = y;
  return p;
};

Matrix.prototype.transformGenerator = function (what = 'point') {
  const matrix = Object.freeze(this.clone());

  return function* (list) {
    const method = Matrix.prototype['transform_' + what] || (typeof what == 'function' && what) || Matrix.prototype.transform_xy;
    for(let item of list) yield item instanceof Array ? method.apply(matrix, [...item]) : method.call(matrix, { ...item });
  };
};

Matrix.prototype.transform_points = function* (list) {
  for(let i = 0; i < list.length; i++) yield Matrix.prototype.transform_point.call(this, { ...list.i });
};

Matrix.prototype.transform_wh = function (width, height) {
  const w = this[0] * width + this[1] * height;
  const h = this[3] * width + this[4] * height;
  return [w, h];
};

Matrix.prototype.transform_size = function (s) {
  const w = this[0] * s.width + this[1] * s.height;
  const h = this[3] * s.width + this[4] * s.height;
  s.width = w;
  s.height = h;
  return s;
};

Matrix.prototype.transform_xywh = function (x, y, width, height) {
  return [...Matrix.prototype.transform_xy.call(this, x, y), ...Matrix.prototype.transform_wh.call(this, width, height)];
};

Matrix.prototype.transform_rect = function (rect) {
  let { x1, y1, x2, y2 } = rect;
  [x1, y1] = Matrix.prototype.transform_xy.call(this, x1, y1);
  [x2, y2] = Matrix.prototype.transform_xy.call(this, x2, y2);
  let xrange = [x1, x2];
  let yrange = [y1, y2];
  [x1, x2] = [Math.min, Math.max].map((fn) => fn(...xrange));
  [y1, y2] = [Math.min, Math.max].map((fn) => fn(...yrange));
  Object.assign(rect, { x1, x2, y1, y2 });
  return rect;
};

Matrix.prototype.point_transformer = function () {
  const matrix = this;
  return (point) => matrix.transform_point(point);
};

Matrix.prototype.transformer = function () {
  const matrix = this;
  return { point: (point) => matrix.transform_point(point), xy: (x, y) => matrix.transform_xy(x, y), size: (s) => matrix.transform_size(s), wh: (w, h) => matrix.transform_wh(w, h), rect: (rect) => matrix.transform_rect(rect), points: (list) => matrix.transform_points(list), distance: (d) => matrix.transform_distance(d) };
};

Matrix.prototype.scale_sign = function () {
  return this[0] * this[4] < 0 || this[1] * this[3] > 0 ? -1 : 1;
};

Matrix.prototype.affine_transform = function (a, b) {
  var xx, yx, xy, yy, tx, ty;
  if(typeof a == 'object' && a.toPoints !== undefined) a = a.toPoints();
  if(typeof b == 'object' && b.toPoints !== undefined) b = b.toPoints();
  xx = (b[0].x * a[1].y + b[1].x * a[2].y + b[2].x * a[0].y - b[0].x * a[2].y - b[1].x * a[0].y - b[2].x * a[1].y) / (a[0].x * a[1].y + a[1].x * a[2].y + a[2].x * a[0].y - a[0].x * a[2].y - a[1].x * a[0].y - a[2].x * a[1].y);
  yx = (b[0].y * a[1].y + b[1].y * a[2].y + b[2].y * a[0].y - b[0].y * a[2].y - b[1].y * a[0].y - b[2].y * a[1].y) / (a[0].x * a[1].y + a[1].x * a[2].y + a[2].x * a[0].y - a[0].x * a[2].y - a[1].x * a[0].y - a[2].x * a[1].y);
  xy = (a[0].x * b[1].x + a[1].x * b[2].x + a[2].x * b[0].x - a[0].x * b[2].x - a[1].x * b[0].x - a[2].x * b[1].x) / (a[0].x * a[1].y + a[1].x * a[2].y + a[2].x * a[0].y - a[0].x * a[2].y - a[1].x * a[0].y - a[2].x * a[1].y);
  yy = (a[0].x * b[1].y + a[1].x * b[2].y + a[2].x * b[0].y - a[0].x * b[2].y - a[1].x * b[0].y - a[2].x * b[1].y) / (a[0].x * a[1].y + a[1].x * a[2].y + a[2].x * a[0].y - a[0].x * a[2].y - a[1].x * a[0].y - a[2].x * a[1].y);
  tx = (a[0].x * a[1].y * b[2].x + a[1].x * a[2].y * b[0].x + a[2].x * a[0].y * b[1].x - a[0].x * a[2].y * b[1].x - a[1].x * a[0].y * b[2].x - a[2].x * a[1].y * b[0].x) / (a[0].x * a[1].y + a[1].x * a[2].y + a[2].x * a[0].y - a[0].x * a[2].y - a[1].x * a[0].y - a[2].x * a[1].y);
  ty = (a[0].x * a[1].y * b[2].y + a[1].x * a[2].y * b[0].y + a[2].x * a[0].y * b[1].y - a[0].x * a[2].y * b[1].y - a[1].x * a[0].y * b[2].y - a[2].x * a[1].y * b[0].y) / (a[0].x * a[1].y + a[1].x * a[2].y + a[2].x * a[0].y - a[0].x * a[2].y - a[1].x * a[0].y - a[2].x * a[1].y);
  this.set_row.call(this, 0, xx, xy, tx);
  this.set_row.call(this, 1, yx, yy, ty);
  this.set_row.call(this, 2, 0, 0, 1);
  return this;
};

Matrix.getAffineTransform = (a, b) => {
  var matrix = new Matrix();
  matrix.affine_transform(a, b);
  return matrix;
};

Matrix.prototype.decompose = function (degrees = (false, (useLU = true))) {
  var a = this[0],
    b = this[3],
    c = this[1],
    d = this[4];
  var translate = { x: this[2], y: this[5] },
    rotation = 0,
    scale = { x: 1, y: 1 },
    skew = { x: 0, y: 0 };
  var determ = a * d - b * c,
    r,
    s;

  const calcFromValues = (r1, m1, r2, m2) => {
    if(!isFinite(r1)) return r2;
    else if(!isFinite(r2)) return r1;
    (m1 = Math.abs(m1)), (m2 = Math.abs(m2));
    return Util.roundTo((m1 * r1 + m2 * r2) / (m1 + m2), 0.0001);
  };

  let sign = Matrix.prototype.scale_sign.call(this);
  rotation = (Math.atan2(this[3], this[4]) + Math.atan2(-sign * this[1], sign * this[0])) / 2;
  const cos = Math.cos(rotation),
    sin = Math.sin(rotation);
  scale = { x: calcFromValues(this[0] / cos, cos, -this[1] / sin, sin), y: calcFromValues(this[4] / cos, cos, this[3] / sin, sin) };
  return { translate, rotate: degrees === true ? Util.roundTo(Matrix.rad2deg(rotation), 0.1) : rotation, scale, skew };
};

Matrix.prototype.init_identity = function () {
  return Matrix.prototype.init.call(this, 1, 0, 0, 0, 1, 0, 0, 0, 1);
};

Matrix.prototype.is_identity = function () {
  return Matrix.prototype.equals.call(this, [1, 0, 0, 0, 1, 0, 0, 0, 1]);
};

Matrix.prototype.init_translate = function (tx, ty) {
  return Matrix.prototype.init.call(this, 1, 0, tx, 0, 1, ty);
};

Matrix.prototype.init_scale = function (sx, sy) {
  if(sy === undefined) sy = sx;
  return Matrix.prototype.init.call(this, sx, 0, 0, 0, sy, 0);
};

Matrix.prototype.init_rotate = function (angle, deg = false) {
  const rad = deg ? Matrix.deg2rad(angle) : angle;
  const s = Math.sin(rad);
  const c = Math.cos(rad);
  return Matrix.prototype.init.call(this, c, -s, 0, s, c, 0);
};

Matrix.prototype.init_skew = function (x, y, deg = false) {
  const ax = Math.tan(deg ? Matrix.deg2rad(x) : x);
  const ay = Math.tan(deg ? Matrix.deg2rad(y) : y);
  return Matrix.prototype.init.call(this, 1, ax, 0, ay, 1, 0);
};
Matrix.identity = () => new Matrix([1, 0, 0, 0, 1, 0, 0, 0, 1]);
Matrix.IDENTITY = Object.freeze(Matrix.identity());
Matrix.rad2deg = (radians) => (radians * 180) / Math.PI;
Matrix.deg2rad = (degrees) => (degrees * Math.PI) / 180;

for(let name of ['toObject', 'init', 'toArray', 'isIdentity', 'determinant', 'invert', 'multiply', 'scalar_product', 'toSource', 'toString', 'toSVG', 'equals', 'init_identity', 'is_identity', 'init_translate', 'init_scale', 'init_rotate', 'scale_sign', 'decompose', 'transformer']) {
  Matrix.name = (matrix, ...args) => Matrix.prototype.name.call(matrix || new Matrix(matrix), ...args);
}

for(let name of ['translate', 'scale', 'rotate', 'skew']) {
  Matrix.name = (...args) => Matrix.prototype['init_' + name].call(new Matrix(), ...args);
}

for(let name of ['translate', 'scale', 'rotate', 'skew']) {
  Matrix.prototype.name = function (...args) {
    return Matrix.prototype.multiply.call(this, new Matrix()['init_' + name](...args));
  };

  Matrix.prototype[name + '_self'] = function (...args) {
    return Matrix.prototype.multiply_self.call(this, new Matrix()['init_' + name](...args));
  };
}

for(let name of ['transform_distance', 'transform_xy', 'transform_point', 'transform_points', 'transform_wh', 'transform_size', 'transform_rect', 'affine_transform']) {
  const method = Matrix.prototype.name;

  if(method.length == 2) {
    Matrix.name = Util.curry((m, a, b) => Matrix.prototype.name.call(m || new Matrix(m), a, b));
  } else if(method.length == 1) {
    Matrix.name = Util.curry((m, a) => Matrix.prototype.name.call(m || new Matrix(m), a));
  }
}

Util.defineGetter(Matrix, Symbol.species, function () {
  return this;
});
const isMatrix = (m) => Util.isObject(m) && (m instanceof Matrix || (m.length !== undefined && (m.length == 6 || m.length == 9) && m.every((el) => typeof el == 'number')));
const ImmutableMatrix = Util.immutableClass(Matrix);

Util.defineGetter(ImmutableMatrix, Symbol.species, function () {
  return ImmutableMatrix;
});

/*
 * concatenanted lib/geom/point.js
 */

const SymSpecies = Util.tryCatch(
  () => Symbol,
  (sym) => sym.species
);

const CTOR = (obj) => {
  if(obj.SymSpecies) return obj.SymSpecies;
  let p = Object.getPrototypeOf(obj);
  if(p.SymSpecies) return p.SymSpecies;
  return p.constructor;
};

export function Point(arg) {
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
    const matches = [...arg.matchAll(/([-+]?d*.?d+)(?:[eE]([-+]?d+))?/g)];
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

Point.prototype.move = function (x, y) {
  this.x += x;
  this.y += y;
  return this;
};

Point.prototype.moveTo = function (x, y) {
  this.x = x;
  this.y = y;
  return this;
};

Point.prototype.clear = function (x, y) {
  this.x = 0;
  this.y = 0;
  return this;
};

Point.prototype.set = function (fn) {
  if(typeof fn != 'function') {
    Point.apply(this, [...arguments]);
    return this;
  }

  return fn(this.x, this.y);
};

Point.prototype.clone = function () {
  const ctor = this[Symbol.species] || this.constructor[Symbol.species];
  return new ctor({ x: this.x, y: this.y });
};

Point.prototype.sum = function (...args) {
  const p = new Point(...args);
  let r = new Point(this.x, this.y);
  r.x += p.x;
  r.y += p.y;
  return r;
};

Point.prototype.add = function (...args) {
  const other = new Point(...args);
  this.x += other.x;
  this.y += other.y;
  return this;
};

Point.prototype.diff = function (arg) {
  let { x, y } = this;

  var fn = function (other) {
    let r = new Point(x, y);
    return r.sub(other);
  };

  if(arg) return fn(arg);
  return fn;
};

Point.prototype.sub = function (...args) {
  const other = new Point(...args);
  this.x -= other.x;
  this.y -= other.y;
  return this;
};

Point.prototype.prod = function (f) {
  const o = isPoint(f) ? f : { x: f, y: f };
  return new Point(this.x * o.x, this.y * o.y);
};

Point.prototype.mul = function (f) {
  const o = isPoint(f) ? f : { x: f, y: f };
  this.x *= o.x;
  this.y *= o.y;
  return this;
};

Point.prototype.quot = function (other) {
  other = isPoint(other) ? other : { x: other, y: other };
  return new Point(this.x / other.x, this.y / other.y);
};

Point.prototype.div = function (other) {
  other = isPoint(other) ? other : { x: other, y: other };
  this.x /= other.x;
  this.y /= other.y;
  return this;
};

Point.prototype.comp = function () {
  return new Point({ x: -this.x, y: -this.y });
};

Point.prototype.neg = function () {
  this.x *= -1;
  this.y *= -1;
  return this;
};

Point.prototype.distanceSquared = function (other = { x: 0, y: 0 }) {
  return (other.y - this.y) * (other.y - this.y) + (other.x - this.x) * (other.x - this.x);
};

Point.prototype.distance = function (other = { x: 0, y: 0 }) {
  return Math.sqrt(Point.prototype.distanceSquared.call(this, other));
};

Point.prototype.equals = function (other) {
  return +this.x == +other.x && +this.y == +other.y;
};

Point.prototype.round = function (precision = (0.001, digits, (type = 'round'))) {
  let { x, y } = this;
  this.x = Util.roundTo(x, precision, digits, type);
  this.y = Util.roundTo(y, precision, digits, type);
  return this;
};

Point.prototype.ceil = function () {
  let { x, y } = this;
  this.x = Math.ceil(x);
  this.y = Math.ceil(y);
  return this;
};

Point.prototype.floor = function () {
  let { x, y } = this;
  this.x = Math.floor(x);
  this.y = Math.floor(y);
  return this;
};

Point.prototype.dot = function (other) {
  return this.x * other.x + this.y * other.y;
};

Point.prototype.values = function () {
  return [this.x, this.y];
};

Point.prototype.fromAngle = function (angle, dist = 1.0) {
  this.x = Math.cos(angle) * dist;
  this.y = Math.sin(angle) * dist;
  return this;
};

Point.prototype.toAngle = function (deg = false) {
  return Math.atan2(this.x, this.y) * (deg ? 180 / Math.PI : 1);
};

Point.prototype.angle = function (other, deg = false) {
  other = other || { x: 0, y: 0 };
  return Point.prototype.diff.call(this, other).toAngle(deg);
};

Point.prototype.rotate = function (angle, origin = { x: 0, y: 0 }) {
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

Util.defineGetter(Point.prototype, Symbol.iterator, function () {
  const { x, y } = this;
  let a = [x, y];
  return a[Symbol.iterator].bind(a);
});

Point.prototype.toString = function (opts = {}) {
  const { precision, unit, separator, left, right } = opts;
  const x = Util.roundTo(this.x, precision);
  const y = Util.roundTo(this.y, precision);
  return `${left}${x}${unit}${separator}${y}${unit}${right}`;
};

Util.defineGetterSetter(
  Point.prototype,
  Symbol.toStringTag,
  function () {
    return `Point{ ${Point.prototype.toSource.call(this)}`;
  },
  () => {},
  false
);

Point.prototype.toSource = function (opts = {}) {
  const { asArray, plainObj, pad, showNew } = opts;
  let x = pad(this.x + '');
  let y = pad(this.y + '');
  let c = (t) => t;
  if(typeof this != 'object' || this === null) return '';
  if(asArray) return `[${x},${y}]`;
  if(plainObj) return `{x:${x},y:${y}}`;
  return `${c(showNew ? 'new ' : '', 1, 31)}${c('Point', 1, 33)}${c('(', 1, 36)}${c(x, 1, 32)}${c(',', 1, 36)}${c(y, 1, 32)}${c(')', 1, 36)}`;
};

Point.prototype.toObject = function (proto = Point.prototype) {
  const { x, y } = this;
  const obj = { x, y };
  Object.setPrototypeOf(obj, proto);
  return obj;
};

Point.prototype.toCSS = function (precision = (0.001, (edges = ['left', 'top']))) {
  return { [[edges[0]]]: Util.roundTo(this.x, precision) + 'px', [[edges[1]]]: Util.roundTo(this.y, precision) + 'px' };
};

Point.prototype.toFixed = function (digits) {
  return new Point(+this.x.toFixed(digits), +this.y.toFixed(digits));
};

Point.prototype.isNull = function () {
  return this.x == 0 && this.y == 0;
};

Point.prototype.inside = function (rect) {
  return this.x >= rect.x && this.x < rect.x + rect.width && this.y >= rect.y && this.y < rect.y + rect.height;
};

Point.prototype.transform = function (m) {
  if(Util.isObject(m) && typeof m.toMatrix == 'function') m = m.toMatrix();
  if(Util.isObject(m) && typeof m.transform_point == 'function') return m.transform_point(this);
  const x = m[0] * this.x + m[1] * this.y + m[2];
  const y = m[3] * this.x + m[4] * this.y + m[5];
  this.x = x;
  this.y = y;
  return this;
};

Point.prototype.scaleTo = function (minmax) {
  return new Point({ x: (this.x - minmax.x1) / (minmax.x2 - minmax.x1), y: (this.y - minmax.y1) / (minmax.y2 - minmax.y1) });
};

Point.prototype.normal = function () {
  let d = Point.prototype.distance.call(this);
  return new Point({ x: this.x / d, y: this.y / d });
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
Point.toSource = (point, { space = ' ', padding = ' ', separator = ',' }) => `{${padding}x:${space}${point.x}${separator}y:${space}${point.y}${padding}}`;
const isPoint = (o) => o && ((o.x !== undefined && o.y !== undefined) || ((o.left !== undefined || o.right !== undefined) && (o.top !== undefined || o.bottom !== undefined)) || o instanceof Point || Object.getPrototypeOf(o).constructor === Point);
Point.isPoint = isPoint;
Util.defineInspect(Point.prototype, 'x', 'y');

Point.bind = (o, p, gen) => {
  const [x, y] = p || ['x', 'y'];
  if(!gen) gen = (k) => (v) => (v === undefined ? o.k : (o.k = v));
  return Util.bindProperties(new Point(0, 0), o, { x, y }, gen);
};
export default Point;

Util.defineGetter(Point, Symbol.species, function () {
  return this;
});
const ImmutablePoint = Util.immutableClass(Point);

Util.defineGetter(ImmutablePoint, Symbol.species, function () {
  return ImmutablePoint;
});

/*
 * concatenanted lib/geom/size.js
 */

export function Size(arg) {
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

Size.prototype.convertUnits = function (w = 'window' in global ? window : null) {
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

Size.prototype.aspect = function () {
  return this.width / this.height;
};

Size.prototype.toCSS = function (units) {
  let ret = {};
  units = typeof units == 'string' ? { width: units, height: units } : units || this.units || { width: 'px', height: 'px' };
  if(this.width !== undefined) ret.width = this.width + (units.width || 'px');
  if(this.height !== undefined) ret.height = this.height + (units.height || 'px');
  return ret;
};

Size.prototype.transform = function (m) {
  this.width = m.xx * this.width + m.yx * this.height;
  this.height = m.xy * this.width + m.yy * this.height;
  return this;
};

Size.prototype.isSquare = function () {
  return Math.abs(this.width - this.height) < 1;
};

Size.prototype.area = function () {
  return this.width * this.height;
};

Size.prototype.resize = function (width, height) {
  this.width = width;
  this.height = height;
  return this;
};

Size.prototype.sum = function (other) {
  return new Size(this.width + other.width, this.height + other.height);
};

Size.prototype.add = function () {
  for(let other of [...arguments]) {
    this.width += other.width;
    this.height += other.height;
  }

  return this;
};

Size.prototype.diff = function (other) {
  return new Size(this.width - other.width, this.height - other.height);
};

Size.prototype.sub = function () {
  for(let other of [...arguments]) {
    this.width -= other.width;
    this.height -= other.height;
  }

  return this;
};

Size.prototype.prod = function (f) {
  const o = isSize(f) ? f : isPoint(f) ? { width: f.x, height: f.y } : { width: f, height: f };
  return new Size(this.width * o.width, this.height * o.height);
};

Size.prototype.mul = function (...args) {
  for(let f of args) {
    const o = isSize(f) ? f : isPoint(f) ? { width: f.x, height: f.y } : { width: f, height: f };
    this.width *= o.width;
    this.height *= o.height;
  }

  return this;
};

Size.prototype.quot = function (other) {
  return new Size(this.width / other.width, this.height / other.height);
};

Size.prototype.inverse = function (other) {
  return new Size(1 / this.width, 1 / this.height);
};

Size.prototype.div = function (f) {
  for(let f of [...arguments]) {
    this.width /= f;
    this.height /= f;
  }

  return this;
};

Size.prototype.round = function (precision = (0.001, digits)) {
  let { width, height } = this;
  this.width = Util.roundTo(width, precision, digits);
  this.height = Util.roundTo(height, precision, digits);
  return this;
};

Size.prototype.bounds = function (other) {
  let w = [Math.min(this.width, other.width), Math.max(this.width, other.width)];
  let h = [Math.min(this.height, other.height), Math.max(this.height, other.height)];
  let scale = h / this.height;
  this.mul(scale);
  return this;
};

Size.prototype.fit = function (size) {
  size = new Size(size);
  let factors = Size.prototype.fitFactors.call(this, size);
  let ret = [Size.prototype.prod.call(this, factors[0]), Size.prototype.prod.call(this, factors[1])];
  return ret;
};

Size.prototype.fitHeight = function (other) {
  other = new Size(other);
  let scale = other.height / this.height;
  this.mul(scale);
  return [this.width, other.width];
};

Size.prototype.fitWidth = function (other) {
  other = new Size(other);
  let scale = other.width / this.width;
  this.mul(scale);
  return [this.height, other.height];
};

Size.prototype.fitFactors = function (other) {
  const hf = other.width / this.width;
  const vf = other.height / this.height;
  return [hf, vf];
};

Size.prototype.toString = function (opts = {}) {
  const { unit, separator, left, right } = opts;
  const { width, height, units } = this;
  return `${left}${width}${units.width || ''}${separator}${height}${units.height || ''}${right}`;
};
Size.area = (sz) => Size.prototype.area.call(sz);
Size.aspect = (sz) => Size.prototype.aspect.call(sz);

Size.bind = (o, p, gen) => {
  const [width, height] = p || ['width', 'height'];
  if(!gen) gen = (k) => (v) => (v === undefined ? o.k : (o.k = v));
  return Util.bindProperties(new Size(0, 0), o, { width, height }, gen);
};
for(let method of Util.getMethodNames(Size.prototype)) Size.method = (size, ...args) => Size.prototype.method.call(size || new Size(size), ...args);
const isSize = (o) => o && ((o.width !== undefined && o.height !== undefined) || (o.x !== undefined && o.x2 !== undefined && o.y !== undefined && o.y2 !== undefined) || (o.left !== undefined && o.right !== undefined && o.top !== undefined && o.bottom !== undefined));

for(let name of ['toCSS', 'isSquare', 'round', 'sum', 'add', 'diff', 'sub', 'prod', 'mul', 'quot', 'div']) {
  Size.name = (size, ...args) => Size.prototype.name.call(size || new Size(size), ...args);
}

Util.defineGetter(Size, Symbol.species, function () {
  return this;
});
const ImmutableSize = Util.immutableClass(Size);

Util.defineGetter(ImmutableSize, Symbol.species, function () {
  return ImmutableSize;
});

/*
 * concatenanted lib/geom/trbl.js
 */

export function TRBL(arg) {
  let ret = this instanceof TRBL ? this : {};
  let args = [...arguments];

  if(typeof arg === 'object' && !Util.isArray(arg)) {
    Object.keys(arg).forEach((k) => {
      const matches = /(top|right|bottom|left)/i.exec(k);
      ret[matches[0].toLowerCase()] = parseInt(arg.k);
    });
  } else if(arg) {
    if(args.length > 1) arg = args;
    if(typeof arg === 'string') arg = [...arg.matchAll(/^[0-9.]+(|px|em|rem|pt|cm|mm)$/g)];
    else if(arg.length == 4) arg = arg.map((v) => parseInt(v.replace(/[a-z]*$/g, '')));
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

TRBL.prototype.null = function () {
  return this.top == 0 && this.right == 0 && this.bottom == 0 && this.left == 0;
};
TRBL.null = (trbl) => TRBL.prototype.null.call(trbl);
TRBL.neg = (trbl = this) => ({ top: -trbl.top, right: -trbl.right, bottom: -trbl.bottom, left: -trbl.left });

TRBL.prototype.isNaN = function () {
  return isNaN(this.top) || isNaN(this.right) || isNaN(this.bottom) || isNaN(this.left);
};

Object.defineProperty(TRBL.prototype, 'inset', {
  get() {
    return (rect) => Rect.inset(rect, this);
  }
});

Object.defineProperty(TRBL.prototype, 'outset', {
  get() {
    return (rect) => Rect.outset(rect, this);
  }
});

TRBL.prototype.add = function (other) {
  this.top += other.top;
  this.right += other.right;
  this.bottom += other.bottom;
  this.left += other.left;
};

TRBL.prototype.union = function (other) {
  this.top = other.top < this.top ? other.top : this.top;
  this.right = other.right > this.right ? other.right : this.right;
  this.bottom = other.bottom > this.bottom ? other.bottom : this.bottom;
  this.left = other.left < this.left ? other.left : this.left;
};

TRBL.prototype.toRect = function () {
  return new Rect({ x: this.left, y: this.top, width: this.right - this.left, height: this.bottom - this.top });
};

TRBL.prototype.toRect = function () {
  return new Rect({ x: this.left, y: this.top, width: this.right - this.left, height: this.bottom - this.top });
};
TRBL.union = (trbl, other) => ({ top: other.top < trbl.top ? other.top : trbl.top, right: other.right > trbl.right ? other.right : trbl.right, bottom: other.bottom > trbl.bottom ? other.bottom : trbl.bottom, left: other.left < trbl.left ? other.left : trbl.left });
TRBL.toRect = (trbl) => new Rect(trbl.left, trbl.top, trbl.right - trbl.left, trbl.bottom - trbl.top);

TRBL.prototype.toString = function (unit = 'px') {
  return '' + this.top + '' + unit + ' ' + this.right + '' + unit + ' ' + this.bottom + '' + unit + ' ' + this.left + unit;
};

TRBL.prototype.toSource = function () {
  return '{top:' + this.top + ',right:' + this.right + ',bottom:' + this.bottom + ',left:' + this.left + '}';
};

for(let name of ['null', 'isNaN', 'outset', 'toRect', 'toSource']) {
  TRBL.name = (points) => TRBL.prototype.name.call(points);
}

export function isTRBL(obj) {
  return top in obj && right in obj && bottom in obj && left in obj;
}

Util.defineGetter(TRBL, Symbol.species, function () {
  return this;
});
const ImmutableTRBL = Util.immutableClass(TRBL);

Util.defineGetter(ImmutableTRBL, Symbol.species, function () {
  return ImmutableTRBL;
});

/*
 * concatenanted lib/geom/rect.js
 */

export function Rect(arg) {
  let obj = this instanceof Rect ? this : {};
  let args = arg instanceof Array ? arg : [...arguments];
  let ret;
  if(typeof args[0] == 'number') arg = args;
  else if(Util.isObject(args[0]) && args[0].length !== undefined) arg = args.shift();

  ['x', 'y', 'width', 'height'].forEach((field) => {
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
  } else if(arg && arg.length >= 4 && arg.slice(0, 4).every((arg) => !isNaN(parseFloat(arg)))) {
    let x = arg.shift();
    let y = arg.shift();
    let w = arg.shift();
    let h = arg.shift();
    obj.x = typeof x === 'number' ? x : parseFloat(x);
    obj.y = typeof y === 'number' ? y : parseFloat(y);
    obj.width = typeof w === 'number' ? w : parseFloat(w);
    obj.height = typeof h === 'number' ? h : parseFloat(h);
    ret = 4;
  } else if(arg && arg.length >= 2 && arg.slice(0, 2).every((arg) => !isNaN(parseFloat(arg)))) {
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

Rect.prototype.clone = function (fn) {
  const ctor = this.constructor[Symbol.species] || this.constructor;
  let ret = new ctor(this.x, this.y, this.width, this.height);
  if(fn) fn(ret);
  return ret;
};

Rect.prototype.corners = function () {
  const rect = this;
  return [
    { x: rect.x, y: rect.y },
    { x: rect.x + rect.width, y: rect.y },
    { x: rect.x + rect.width, y: rect.y + rect.height },
    { x: rect.x, y: rect.y + rect.height }
  ];
};

if(Rect.prototype.isSquare === undefined) {
  Rect.prototype.isSquare = function () {
    return Math.abs(this.width - this.height) < 1;
  };
}
Rect.prototype.constructor = Rect;

Rect.prototype.getArea = function () {
  return this.width * this.height;
};

Rect.prototype.toSource = function (opts = {}) {
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

Rect.prototype.points = function (ctor = (items) => Array.from(items)) {
  const c = this.corners();
  return ctor(c);
};
Rect.prototype.toCSS = Rect.toCSS;

Rect.prototype.scale = function (factor) {
  let width = this.width * factor;
  let height = this.height * factor;
  this.x += (width - this.width) / 2;
  this.y += (height - this.height) / 2;
  this.width = width;
  this.height = height;
  return this;
};

Rect.prototype.mul = function (...args) {
  Point.prototype.mul.call(this, ...args);
  Size.prototype.mul.call(this, ...args);
  return this;
};

Rect.prototype.div = function (...args) {
  Point.prototype.div.call(this, ...args);
  Size.prototype.div.call(this, ...args);
  return this;
};

Rect.prototype.outset = function (trbl) {
  if(typeof trbl == 'number') trbl = { top: trbl, right: trbl, bottom: trbl, left: trbl };
  this.x -= trbl.left;
  this.y -= trbl.top;
  this.width += trbl.left + trbl.right;
  this.height += trbl.top + trbl.bottom;
  return this;
};

Rect.prototype.inset = function (trbl) {
  if(typeof trbl == 'number') trbl = new TRBL(trbl, trbl, trbl, trbl);

  if(trbl.left + trbl.right < this.width && trbl.top + trbl.bottom < this.height) {
    this.x += trbl.left;
    this.y += trbl.top;
    this.width -= trbl.left + trbl.right;
    this.height -= trbl.top + trbl.bottom;
  }

  return this;
};

Rect.prototype.inside = function (point) {
  return Rect.inside(this, point);
};
Rect.CONTAIN = 16;
Rect.COVER = 32;

Rect.prototype.fit = function (other, align = Align.CENTER | Align.MIDDLE | Rect.CONTAIN) {
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

Rect.prototype.pointFromCenter = function (point) {
  Point.prototype.sub.call(point, this.center);
  point.x /= this.width;
  point.y /= this.height;
  return point;
};

Rect.prototype.toCSS = function () {
  return { ...Point.prototype.toCSS.call(this), ...Size.prototype.toCSS.call(this) };
};

Rect.prototype.toTRBL = function () {
  return { top: this.y, right: this.x + this.width, bottom: this.y + this.height, left: this.x };
};

Rect.prototype.toArray = function () {
  const { x, y, width, height } = this;
  return [x, y, width, height];
};

Rect.prototype.toPoints = function (...args) {
  let ctor = Util.isConstructor(args[0])
    ? (() => {
        let arg = args.shift();
        return (points) => new arg(points);
      })()
    : (points) => Array.from(points);

  let num = typeof args[0] == 'number' ? args.shift() : 4;
  const { x, y, width, height } = this;
  let a = num == 2 ? [new Point(x, y), new Point(x + width, y + height)] : [new Point(x, y), new Point(x + width, y), new Point(x + width, y + height), new Point(x, y + height)];
  return ctor(a);
};

Rect.prototype.toLines = function (ctor = (lines) => Array.from(lines, (points) => new Line(...points))) {
  let [a, b, c, d] = Rect.prototype.toPoints.call(this);
  return ctor([
    [a, b],
    [b, c],
    [c, d],
    [d, a]
  ]);
};

Rect.prototype.align = function (align_to, a = 0) {
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

Rect.prototype.round = function (precision = (0.001, digits, (type = 'round'))) {
  let { x1, y1, x2, y2 } = this.toObject(true);
  let a = new Point(x1, y1).round(precision, digits, type);
  let b = new Point(x2, y2).round(precision, null, type);
  this.x = a.x;
  this.y = a.y;
  this.width = +(b.x - this.x).toFixed(digits);
  this.height = +(b.y - this.y).toFixed(digits);
  return this;
};

Rect.prototype.toObject = function (bb = false) {
  if(bb) {
    const { x1, y1, x2, y2 } = this;
    return { x1, y1, x2, y2 };
  }

  const { x, y, width, height } = this;
  return { x, y, width, height };
};

Rect.prototype.bbox = function () {
  return this.toObject(true);
};

Rect.prototype.transform = function (m) {
  if(Util.isObject(m) && typeof m.toMatrix == 'function') m = m.toMatrix();
  Matrix.prototype.transform_rect.call(m, this);
  return this;
};

Rect.prototype[Symbol.iterator] = function* () {
  let { x, y, width, height } = this;
  for(let prop of [x, y, width, height]) yield prop;
};
Rect.round = (rect) => Rect.prototype.round.call(rect);
Rect.align = (rect, align_to, a = 0) => Rect.prototype.align.call(rect, align_to, a);
Rect.toCSS = (rect) => Rect.prototype.toCSS.call(rect);
Rect.inset = (rect, trbl) => Rect.prototype.inset.call(rect, trbl);
Rect.outset = (rect, trbl) => Rect.prototype.outset.call(rect, trbl);
Rect.center = (rect) => new Point(rect.x + rect.width / 2, rect.y + rect.height / 2);

Rect.bind = (rect) => {
  let obj = new Rect();
};

Rect.inside = (rect, point) => {
  return point.x >= rect.x && point.x <= rect.x + rect.width && point.y >= rect.y && point.y <= rect.y + rect.height;
};

Rect.from = function (obj) {
  const fn = (v1, v2) => [Math.min(v1, v2), Math.max(v1, v2)];
  const h = fn(obj.x1, obj.x2);
  const v = fn(obj.y1, obj.y2);
  const [x1, x2, y1, y2] = [...h, ...v];
  return new Rect(x1, y1, x2 - x1, y2 - y1);
};

Rect.fromCircle = function (...args) {
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
  if(!gen) gen = (k) => (v) => (v === undefined ? o.k : (o.k = v));
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
  Rect.prototype.f = function (...args) {
    Rect.f(this, ...args);
    return this;
  };
}
Util.defineInspect(Rect.prototype, 'x', 'y', 'width', 'height');
const isRect = (rect) => isPoint(rect) && isSize(rect);

Util.defineGetter(Rect, Symbol.species, function () {
  return this;
});
const ImmutableRect = Util.immutableClass(Rect);
delete ImmutableRect[Symbol.species];

Util.defineGetter(ImmutableRect, Symbol.species, function () {
  return ImmutableRect;
});

Rect.prototype.toString = function (opts = {}) {
  if(typeof opts == 'string') opts = { separator: opts };
  const { precision, unit, separator, left, right } = opts;
  let { x, y, width, height } = this;
  let props = [x, y, width, height];
  return left + props.map((p) => p + unit).join(' ') + right;
};

/*
 * concatenanted lib/geom/circle.js
 */

export function Circle(x, y, radius) {
  let obj = this || null;
  let arg;
  let args = [...arguments];
  let ret;

  if(args.length >= 3 && args.every((arg) => !isNaN(parseFloat(arg)))) {
    arg = { x: +args[0], y: +args[1], radius: +args[2] };
  } else if(args.length == 1) {
    arg = args[0];
    obj.x = +arg.x;
    obj.y = +arg.y;
    obj.radius = +arg.radius;
  }

  if(obj === null) obj = Object.create(Circle.prototype);
  if(Object.getPrototypeOf(obj) !== Circle.prototype) Object.setPrototypeOf(obj, Circle.prototype);

  if(arg && arg.x !== undefined && arg.y !== undefined && arg.radius !== undefined) {
    obj.x = +arg.x;
    obj.y = +arg.y;
    obj.radius = +arg.radius;
    ret = 1;
  } else if(isPoint(args[0]) && typeof args[1] == 'number') {
    obj.x = +args[0].x;
    obj.y = +args[0].y;
    obj.radius = +args[1];
    ret = 2;
  } else if(arg && arg.length >= 3 && arg.slice(0, 3).every((arg) => !isNaN(parseFloat(arg)))) {
    obj.x = +arg[0];
    obj.y = +arg[1];
    obj.radius + arg[2];
    ret = 3;
  } else {
    obj.x = 0;
    obj.y = 0;
    obj.radius = 0;
    ret = 0;
  }

  if(!isCircle(obj)) {
  }
  return obj;
}
const isCircle = (obj) => ['x', 'y', 'radius'].every((prop) => obj.prop !== undefined);
Object.defineProperty(Circle.prototype, 'x', { value: 0, enumerable: true, writable: true });
Object.defineProperty(Circle.prototype, 'y', { value: 0, enumerable: true, writable: true });
Object.defineProperty(Circle.prototype, 'radius', { value: 0, enumerable: true, writable: true });

Object.defineProperty(Circle.prototype, 'center', {
  get() {
    return Point.bind(this, null, (value) => {
      if(value === undefined) return new Point(this.x, this.y);
      this.x = value.x;
      this.y = value.y;
    });
  }
});

Circle.prototype.bbox = function (width = 0) {
  const { x, y, radius } = this;
  let distance = radius + width;
  return new Rect({ x1: x - distance, x2: x + distance, y1: y - distance, y2: y + distance });
};

Circle.prototype.transform = function (m) {
  if(Util.isObject(m) && typeof m.toMatrix == 'function') m = m.toMatrix();
  Matrix.prototype.transform_point.call(m, this);
  this.radius = Matrix.prototype.transform_wh.call(m, this.radius, this.radius)[0];
  return this;
};
Util.defineInspect(Circle.prototype, 'x', 'y', 'radius');

Circle.bind = (o, p, gen) => {
  const [x, y, radius] = p || ['x', 'y', 'radius'];
  if(!gen) gen = (k) => (v) => (v === undefined ? o.k : (o.k = v));
  return Util.bindProperties(new Circle(0, 0, 0), o, { x, y, radius }, gen);
};

/*
 * concatenanted lib/geom/pointList.js
 */

export class PointList extends Array {
  constructor(points, tfn = (...args) => new Point(...args)) {
    super();
    const base = Array;
    let args = [...arguments];
    let ret = this instanceof PointList ? this : [];
    if(Util.isArray(args[0]) || Util.isGenerator(args[0])) args = [...args[0]];

    if(typeof points === 'string') {
      const matches = [...points.matchAll(/[-.0-9,]+/g)];

      for(let i = 0; i < matches.length; i++) {
        const coords = (matches.i[0] + '').split(/,/g).map((n) => +n);
        ret.push(tfn(...coords));
      }
    } else if(args[0] && args[0].length == 2) {
      for(let i = 0; i < args.length; i++) ret.push(this instanceof PointList ? new Point(args.i) : Point(args.i));
    } else if(args.length !== undefined) {
      for(let i = 0; i < args.length; i++) {
        ret.push(args.i instanceof Point ? args.i : tfn(args.i));
      }
    }

    let proto = PointList.prototype;
    Object.setPrototypeOf(ret, proto);
    if(!(this instanceof PointList)) return ret;
  }
}

PointList.prototype[Symbol.toStringTag] = function () {
  return PointList.prototype.toString.apply(this, arguments);
};
Util.defineGetter(PointList, Symbol.species, () => PointList);
PointList.prototype[Symbol.isConcatSpreadable] = true;

PointList.prototype.rotateRight = function (n) {
  this.unshift(...this.splice(n % this.length, this.length));
  return this;
};

PointList.prototype.rotateLeft = function (n) {
  return this.rotateRight(this.length - (n % this.length));
};

PointList.prototype.rotate = function (n) {
  if(n < 0) return this.rotateLeft(-n);
  if(n > 0) return this.rotateRight(n);
  return this;
};

PointList.prototype.push = function (...args) {
  while(args.length > 0) Array.prototype.push.call(this, new Point(args));
  return this;
};

PointList.prototype.unshift = function (...args) {
  let points = [];
  while(args.length > 0) points.push(new Point(args));
  Array.prototype.splice.call(this, 0, 0, ...points);
  return this;
};
PointList.prototype.length = 0;

PointList.prototype.getLength = function () {
  return this.length;
};

Object.defineProperty(PointList.prototype, 'size', {
  get() {
    return PointList.prototype.getLength.call(this);
  }
});

PointList.prototype.at = function (index) {
  return this[+index];
};

PointList.prototype.splice = function () {
  let args = [...arguments];
  const start = args.shift();
  const remove = args.shift();
  return Array.prototype.splice.apply(this, [start, remove, ...args.map((arg) => (arg instanceof Point ? arg : new Point(arg)))]);
};
PointList.prototype.slice = Array.prototype.slice;

PointList.prototype.removeSegment = function (index) {
  let indexes = [PointList.prototype.getLineIndex.call(this, index - 1), PointList.prototype.getLineIndex.call(this, index), PointList.prototype.getLineIndex.call(this, index + 1)];
  let lines = indexes.map((i) => PointList.prototype.getLine.call(this, i));
  let point = Line.intersect(lines[0], lines[2]);

  if(point) {
    PointList.prototype.splice.call(this, 0, 2, new Point(point));
  }
};

PointList.prototype.clone = function () {
  const ctor = this.constructor[Symbol.species];
  let points = PointList.prototype.map.call(this, (p) => Point.prototype.clone.call(p));
  return new ctor(points);
};

PointList.prototype.toPolar = function (tfn) {
  let ret = new PointList();
  let t = typeof tfn == 'function' ? tfn : (x, y) => ({ x, y });

  ret.splice.apply(ret, [
    0,
    ret.length,
    ...PointList.prototype.map.call(this, (p) => {
      const angle = Point.prototype.toAngle.call(p);
      return t(angle, Point.prototype.distance.call(p));
    })
  ]);

  return ret;
};

PointList.prototype.fromPolar = function (tfn) {
  let ret = new PointList();
  let t = typeof tfn == 'function' ? tfn : (x, y) => ({ x, y });

  ret.splice.apply(ret, [
    0,
    ret.length,
    ...PointList.prototype.map.call(this, (p) => {
      let r = t(p.x, p.y);
      return Point.prototype.fromAngle.call(r.x, r.y);
    })
  ]);

  return ret;
};

PointList.prototype.draw = function (ctx, close = false) {
  const first = PointList.prototype.at.call(this, 0);
  const len = PointList.prototype.getLength.call(this);
  ctx.to(first.x, first.y);

  for(let i = 1; i < len; i++) {
    const { x, y } = PointList.prototype.at.call(this, i);
    ctx.line(x, y);
  }

  if(close) ctx.line(first.x, first.y);
  return this;
};

PointList.prototype.area = function () {
  var area = 0;
  var i;
  var j;
  var point1;
  var point2;
  const len = PointList.prototype.getLength.call(this);

  for(i = (0, (j = len - 1)); i < len; j = (i, (i += 1))) {
    point1 = PointList.prototype.at.call(this, i);
    point2 = PointList.prototype.at.call(this, j);
    area += point1.x * point2.y;
    area -= point1.y * point2.x;
  }

  area /= 2;
  return area;
};

PointList.prototype.centroid = function () {
  var x = 0;
  var y = 0;
  var i;
  var j;
  var f;
  var point1;
  var point2;
  const len = PointList.prototype.getLength.call(this);

  for(i = (0, (j = len - 1)); i < len; j = (i, (i += 1))) {
    point1 = PointList.prototype.at.call(this, i);
    point2 = PointList.prototype.at.call(this, j);
    f = point1.x * point2.y - point2.x * point1.y;
    x += (point1.x + point2.x) * f;
    y += (point1.y + point2.y) * f;
  }

  f = PointList.prototype.area.call(this) * 6;
  return new Point(x / f, y / f);
};

PointList.prototype.avg = function () {
  var ret = PointList.prototype.reduce.call(this, (acc, p) => acc.add(p), new Point());
  return ret.div(PointList.prototype.getLength.call(this));
};

PointList.prototype.bbox = function () {
  const len = PointList.prototype.getLength.call(this);
  if(!len) return {};
  const first = PointList.prototype.at.call(this, 0);

  var ret = {
    x1: first.x,
    x2: first.x,
    y1: first.y,
    y2: first.y,
    toString() {
      return `{x1:${(this.x1 + '').padStart(4, ' ')},x2:${(this.x2 + '').padStart(4, ' ')},y1:${(this.y1 + '').padStart(4, ' ')},y2:${(this.y2 + '').padStart(4, ' ')}}`;
    }
  };

  for(let i = 1; i < len; i++) {
    const { x, y } = PointList.prototype.at.call(this, i);
    if(x < ret.x1) ret.x1 = x;
    if(x > ret.x2) ret.x2 = x;
    if(y < ret.y1) ret.y1 = y;
    if(y > ret.y2) ret.y2 = y;
  }

  return ret;
};

PointList.prototype.rect = function () {
  return new Rect(PointList.prototype.bbox.call(this));
};

PointList.prototype.xrange = function () {
  const bbox = PointList.prototype.bbox.call(this);
  return [bbox.x1, bbox.x2];
};

PointList.prototype.normalizeX = function (newVal = (x) => x) {
  const xrange = PointList.prototype.xrange.call(this);
  const xdiff = xrange[1] - xrange[0];

  PointList.prototype.forEach.call(this, (p, i, l) => {
    l.i.x = newVal((l.i.x - xrange[0]) / xdiff);
  });

  return this;
};

PointList.prototype.yrange = function () {
  const bbox = PointList.prototype.bbox.call(this);
  return [bbox.y1, bbox.y2];
};

PointList.prototype.normalizeY = function (newVal = (y) => y) {
  const yrange = PointList.prototype.yrange.call(this);
  const ydiff = yrange[1] - yrange[0];

  PointList.prototype.forEach.call(this, (p, i, l) => {
    l.i.y = newVal((l.i.y - yrange[0]) / ydiff);
  });

  return this;
};

PointList.prototype.boundingRect = function () {
  return new Rect(PointList.prototype.bbox.call(this));
};

PointList.prototype.translate = function (x, y) {
  PointList.prototype.forEach.call(this, (it) => Point.prototype.move.call(it, x, y));
  return this;
};

PointList.prototype.transform = function (m) {
  if(Util.isObject(m) && typeof m.toMatrix == 'function') m = m.toMatrix();

  if(Util.isObject(m) && typeof m.transform_point == 'function') {
    this.forEach((p) => m.transform_point(p));
    return this;
  }

  for(let i = 0; i < this.length; i++) Point.prototype.transform.call(this.i, m);

  return this;
};

PointList.prototype.filter = function (pred) {
  let ret = new PointList();
  PointList.prototype.forEach.call(this, (p, i, l) => pred(p, i, l) && ret.push(new Point(l.i)));
  return ret;
};

PointList.prototype.getLineIndex = function (index) {
  const len = PointList.prototype.getLength.call(this);
  return (index < 0 ? len + index : index) % len;
};

PointList.prototype.getLine = function (index) {
  let a = PointList.prototype.getLineIndex.call(this, index);
  let b = PointList.prototype.getLineIndex.call(this, index + 1);
  return [PointList.prototype.at.call(this, a), PointList.prototype.at.call(this, b)];
};

PointList.prototype.lines = function (closed = false) {
  const points = this;
  const n = points.length - (closed ? 0 : 1);

  const iterableObj = {
    [[Symbol.iterator]]() {
      let step = 0;

      return {
        next() {
          let value;
          let done = step >= n;

          if(!done) {
            value = new Line(points.step, points[(step + 1) % points.length]);
            step++;
          }

          return { value, done };
        }
      };
    }
  };

  return iterableObj;
};

PointList.prototype.sort = function (pred) {
  return Array.prototype.sort.call(this, pred || ((a, b) => Point.prototype.valueOf.call(a) - Point.prototype.valueOf.call(b)));
};

PointList.prototype.toString = function (sep = (',', prec)) {
  return Array.prototype.map.call(this, (point) => (Point.prototype.toString ? Point.prototype.toString.call(point, prec, sep) : point + '')).join(' ');
};

PointList.prototype.toPath = function () {
  return Array.prototype.map.call(this, (point, i) => `${i > 0 ? 'L' : 'M'}${point}`).join(' ');
  return Array.prototype.reduce.call(this, (acc, point, i) => (acc ? acc + ' ' : '') + `${acc ? 'L' : 'M'}${point}`);
};

PointList.prototype.toSource = function (opts = {}) {
  if(opts.asString) return `new PointList("${this.toString(opts)}")`;
  let fn = opts.asArray ? (p) => `[${p.x},${p.y}]` : opts.plainObj ? (p) => Point.toSource(p, { space: '', padding: ' ', separator: ',' }) : (point) => Point.prototype.toSource.call(point, { ...opts, plainObj: true });
  return 'new PointList([' + PointList.prototype.map.call(this, fn).join(',') + '])';
};

PointList.prototype.add = function (pt) {
  if(!(pt instanceof Point)) pt = new Point(...arguments);
  PointList.prototype.forEach.call(this, (it) => Point.prototype.add.call(it, pt));
  return this;
};

PointList.prototype.sum = function (pt) {
  let ret = PointList.prototype.clone.call(this);
  return PointList.prototype.add.apply(ret, arguments);
};

PointList.prototype.sub = function (pt) {
  if(!(pt instanceof Point)) pt = new Point(...arguments);
  PointList.prototype.forEach.call(this, (it) => Point.prototype.sub.call(it, pt));
  return this;
};

PointList.prototype.diff = function (pt) {
  let ret = PointList.prototype.clone.call(this);
  return PointList.prototype.sub.apply(ret, arguments);
};

PointList.prototype.mul = function (pt) {
  if(typeof pt == 'number') pt = new Point({ x: pt, y: pt });
  if(!(pt instanceof Point)) pt = new Point(...arguments);
  PointList.prototype.forEach.call(this, (it) => Point.prototype.mul.call(it, pt));
  return this;
};

PointList.prototype.prod = function (pt) {
  let ret = PointList.prototype.clone.call(this);
  return PointList.prototype.mul.apply(ret, arguments);
};

PointList.prototype.div = function (pt) {
  if(typeof pt == 'number') pt = new Point({ x: pt, y: pt });
  if(!(pt instanceof Point)) pt = new Point(...arguments);
  PointList.prototype.forEach.call(this, (it) => Point.prototype.div.call(it, pt));
  return this;
};

PointList.prototype.quot = function (pt) {
  let ret = PointList.prototype.clone.call(this);
  return PointList.prototype.div.apply(ret, arguments);
};

PointList.prototype.round = function (prec) {
  PointList.prototype.forEach.call(this, (it) => Point.prototype.round.call(it, prec));
  return this;
};

PointList.prototype.ceil = function (prec) {
  PointList.prototype.forEach.call(this, (it) => Point.prototype.ceil.call(it, prec));
  return this;
};

PointList.prototype.floor = function (prec) {
  PointList.prototype.forEach.call(this, (it) => Point.prototype.floor.call(it, prec));
  return this;
};

PointList.prototype.toMatrix = function () {
  return Array.prototype.map.call(this, ({ x, y }) => Object.freeze([x, y]));
};

if(!Util.isBrowser()) {
  let c = Util.coloring();
  let sym = Symbol.for('nodejs.util.inspect.custom');

  PointList.prototype.sym = function () {
    return `${c.text('PointList', 1, 31)}${c.text('(', 1, 36)}${c.text(this.getLength(), 1, 35) + c.code(1, 36)}) [\n  ${this.map(({ x, y }) => Util.toString({ x, y }, { multiline: false, spacing: ' ' })).join(',\n  ')}\n${c.text(']', 1, 36)}`;
  };
}

for(let name of ['push', 'splice', 'clone', 'area', 'centroid', 'avg', 'bbox', 'rect', 'xrange', 'yrange', 'boundingRect']) {
  PointList.name = (points) => PointList.prototype.name.call(points);
}

export function Polyline(lines) {
  let ret = this instanceof Polyline ? this : new PointList();

  const addUnique = (point) => {
    const ok = ret.length > 0 ? !Point.equals(ret[ret.length - 1], point) : true;
    if(ok) Array.prototype.push.call(ret, Point.clone(point));
    return ok;
  };

  let prev;

  for(let i = 0; i < lines.length; i++) {
    const line = lines.shift();

    if(isPoint(line)) {
      addUnique(line);
      prev = line;
    } else if(isLine(line)) {
      if(i > 0) {
        const eq = [Point.equals(prev, line.a)];
        if(!eq[0] && !Point.equals(prev, line.b)) break;
      } else {
        addUnique(line.a);
      }

      addUnique(line.b);
      prev = line.b;
    }
  }

  return ret;
}
Polyline.prototype = new PointList();

Polyline.prototype.toSVG = function (factory, attrs = ({}, (parent = (null, prec)))) {
  return factory('polyline', { points: PointList.prototype.toString.call(this), ...attrs }, parent, prec);
};

Polyline.prototype.push = function (...args) {
  const last = this[this.length - 1];

  for(let arg of args) {
    if(last && Point.equals(arg, last)) continue;
    PointList.prototype.push.call(this, arg);
  }

  return this;
};

Polyline.prototype.inside = function (point) {
  var i,
    j,
    c = false,
    nvert = this.length;

  for(i = (0, (j = nvert - 1)); i < nvert; j = i++) {
    if(this.i.y > point.y !== this.j.y > point.y && point.x < ((this.j.x - this.i.x) * (point.y - this.i.y)) / (this.j.y - this.i.y) + this.i.x) {
      c = !c;
    }
  }

  return c;
};

Polyline.inside = function (a, b) {
  return a.every((point) => b.inside(point));
};

Polyline.prototype.isClockwise = function () {
  var sum = 0;

  for(var i = 0; i < this.length - 1; i++) {
    var cur = this.i,
      next = this[i + 1];
    sum += (next.x - cur.x) * (next.y + cur.y);
  }

  return sum > 0;
};

Util.defineGetter(Polyline.prototype, 'clockwise', function () {
  let ret = new (this[Symbol.species] || this.constructor)().concat(this);
  return Polyline.prototype.isClockwise.call(this) ? ret : ret.reverse();
});

Util.defineGetter(Polyline.prototype, 'counterClockwise', function () {
  let ret = new (this[Symbol.species] || this.constructor)().concat(this);
  return Polyline.prototype.isClockwise.call(this) ? ret.reverse() : ret;
});

Polyline.isClockwise = function isClockwise(poly) {
  var sum = 0;

  for(var i = 0; i < poly.length - 1; i++) {
    var cur = poly.i,
      next = poly[i + 1];
    sum += (next.x - cur.x) * (next.y + cur.y);
  }

  return sum > 0;
};

Util.define(PointList, {
  get [[Symbol.species]]() {
    return PointList;
  }
});
const ImmutablePointList = Util.immutableClass(PointList);

Util.defineGetter(ImmutablePointList, Symbol.species, function () {
  return ImmutablePointList;
});

/*
 * concatenanted lib/geom/polygonFinder.js
 */

export class PolygonFinder {
  static buildGraphFromSegments(segments) {
    let graph = new Graph();
    let intersections = PolygonFinder.findAllIntersectionsInSegments(segments);
    let connectedSegments = [];
    let connectedIntersections = [];

    segments.forEach((segment) => {
      let intersectionsOnSegment = intersections.filter((intersection) => intersection.line1 === segment || intersection.line2 === segment);

      if(intersectionsOnSegment.length > 1) {
        intersectionsOnSegment.forEach((intersection) => {
          if(!connectedIntersections.includes(intersection)) {
            connectedIntersections.push(intersection);
          }
        });

        connectedSegments.push(segment);
      }
    });

    connectedSegments.forEach((segment) => {
      let intersectionsOnSegment = connectedIntersections.filter((intersection) => intersection.line1 === segment || intersection.line2 === segment);

      let nearestNeighborTrios = intersectionsOnSegment.map((intersection, index, intersections) => {
        let nearestNeighborPair = [null, null];
        let minimumDistancePair = [Infinity, Infinity];
        let possibleNeighbors = intersections.filter((possibleNeighborIntersection) => intersection != possibleNeighborIntersection);

        possibleNeighbors.forEach((possibleNeighbor) => {
          let comparisonProperty = '';
          let distanceBetween = dist(intersection.point.x, intersection.point.y, possibleNeighbor.point.x, possibleNeighbor.point.y);

          if(possibleNeighbor.point.x !== intersection.point.x) {
            comparisonProperty = 'x';
          } else if(possibleNeighbor.point.y !== intersection.point.y) {
            comparisonProperty = 'y';
          } else {
            return null;
          }

          if(possibleNeighbor.point.comparisonProperty < intersection.point.comparisonProperty) {
            if(nearestNeighborPair[0] == null || distanceBetween < minimumDistancePair[0]) {
              nearestNeighborPair[0] = possibleNeighbor;
              minimumDistancePair[0] = distanceBetween;
            }
          } else if(possibleNeighbor.point.comparisonProperty > intersection.point.comparisonProperty) {
            if(nearestNeighborPair[1] == null || distanceBetween < minimumDistancePair[1]) {
              nearestNeighborPair[1] = possibleNeighbor;
              minimumDistancePair[1] = distanceBetween;
            }
          }
        });

        if(nearestNeighborPair[0] === null) {
          nearestNeighborPair[0] = intersection;
        }

        if(nearestNeighborPair[1] === null) {
          nearestNeighborPair[1] = intersection;
        }

        return [intersection, nearestNeighborPair[0], nearestNeighborPair[1]];
      });

      nearestNeighborTrios.forEach((trio) => {
        let nodes = [];

        trio.forEach((intersection) => {
          let newNode = new Graph.Node(intersection.point);
          nodes.push(newNode);
          graph.addNode(newNode);
        });

        nodes.forEach((node) => {
          graph.addNode(node);
        });

        graph.addConnection(nodes[0], nodes[1]);
        graph.addConnection(nodes[0], nodes[2]);
      });
    });

    graph.checkForDuplicateNodes();
    graph.checkForDuplicateConnections();
    return graph;
  }

  static polygonsFromCycles(cycles, graph) {
    let polygons = [];

    cycles.forEach((cycle) => {
      let points = cycle.map((node) => {
        return graph.nodes.node.point;
      });

      polygons.push(points);
    });

    return polygons;
  }

  static polygonsFromSegments(segments) {
    let graph = PolygonFinder.buildGraphFromSegments(segments);
    let cycles = graph.findMinimumCycles();
    return PolygonFinder.polygonsFromCycles(cycles, graph);
  }

  static findAllIntersectionsInSegments(segmentSet) {
    let intersections = [];

    for(let i = 0; i < segmentSet.length; i++) {
      for(let j = i + 1; j < segmentSet.length; j++) {
        let intersection = Intersection.findIntersection(segmentSet.i, segmentSet.j);

        if(intersection !== null) {
          let alreadyInSet = false;

          for(let k = 0; k < intersections.length; k++) {
            if(Intersection.equals(intersections.k, intersection)) {
              alreadyInSet = true;
            }
          }

          if(!alreadyInSet) {
            intersections.push(intersection);
          }
        }
      }
    }

    return intersections;
  }
}

/*
 * concatenanted lib/geom/polygon.js
 */

const Polygon = function Polygon() {};

Polygon.area = (polygon) => {
  var area = 0;
  var j = polygon.length - 1;
  var p1;
  var p2;

  for(var k = 0; k < polygon.length; j = k++) {
    p1 = polygon.k;
    p2 = polygon.j;

    if(p1.x !== undefined && p2.x !== undefined) {
      area += p1.x * p2.y;
      area -= p1.y * p2.x;
    } else {
      area += p1[0] * p2[1];
      area -= p1[1] * p2[0];
    }
  }

  area = area / 2;
  return area;
};

Polygon.center = (polygon) => {
  var x = 0;
  var y = 0;
  var f;
  var j = polygon.length - 1;
  var p1;
  var p2;

  for(var k = 0; k < polygon.length; j = k++) {
    p1 = polygon.k;
    p2 = polygon.j;

    if(p1.x !== undefined && p2.x !== undefined) {
      f = p1.x * p2.y - p2.x * p1.y;
      x += (p1.x + p2.x) * f;
      y += (p1.y + p2.y) * f;
    } else {
      f = p1[0] * p2[1] - p2[0] * p1[1];
      x += (p1[0] + p2[0]) * f;
      y += (p1[1] + p2[1]) * f;
    }
  }

  f = area(polygon) * 6;
  return [x / f, y / f];
};

Polygon.approxCircle = (radius, npoints) => {
  var ret = [];

  for(var k = 0; k < npoints; k++) {
    var theta = (Math.PI * 2 * k) / npoints;
    var x = Math.sin(theta) * radius;
    var y = Math.cos(theta) * radius;
    ret.push({ x, y });
  }

  return ret;
};

Polygon.toPath = (polygon, relative = true) => {
  var prevx = 0;
  var prevy = 0;
  var path = '';

  for(var k = 0; k < polygon.length; k++) {
    let x = polygon.k.x !== undefined ? polygon.k.x : polygon.k[0];
    let y = polygon.k.y !== undefined ? polygon.k.y : polygon.k[1];

    if(relative) {
      x -= prevx;
      y -= prevy;
    }

    let cmd = k == 0 ? 'M' : 'L';
    if(relative) cmd = cmd.toLowerCase();
    path += `${cmd}${x},${y}`;
  }

  path += 'z';
  return path;
};

Polygon.fromLine = (arg, offset, steps = 3) => {
  let line = new Line(arg);
  const PI2 = Math.PI * 0.5;
  const step = Util.range(0, steps - 1).map((i) => (i * Math.PI) / (steps - 1));
  const a = line.angle();
  let vl = new PointList();
  vl = vl.concat(step.map((va) => Point.fromAngle(a - PI2 - va, offset).sum(line.a)));
  vl = vl.concat(step.map((va) => Point.fromAngle(a + PI2 - va, offset).sum(line.b)));
  return vl;
};
export default Polygon;

/*
 * concatenanted lib/geom/rect.js
 */

export function Rect(arg) {
  let obj = this instanceof Rect ? this : {};
  let args = arg instanceof Array ? arg : [...arguments];
  let ret;
  if(typeof args[0] == 'number') arg = args;
  else if(Util.isObject(args[0]) && args[0].length !== undefined) arg = args.shift();

  ['x', 'y', 'width', 'height'].forEach((field) => {
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
  } else if(arg && arg.length >= 4 && arg.slice(0, 4).every((arg) => !isNaN(parseFloat(arg)))) {
    let x = arg.shift();
    let y = arg.shift();
    let w = arg.shift();
    let h = arg.shift();
    obj.x = typeof x === 'number' ? x : parseFloat(x);
    obj.y = typeof y === 'number' ? y : parseFloat(y);
    obj.width = typeof w === 'number' ? w : parseFloat(w);
    obj.height = typeof h === 'number' ? h : parseFloat(h);
    ret = 4;
  } else if(arg && arg.length >= 2 && arg.slice(0, 2).every((arg) => !isNaN(parseFloat(arg)))) {
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

Rect.prototype.clone = function (fn) {
  const ctor = this.constructor[Symbol.species] || this.constructor;
  let ret = new ctor(this.x, this.y, this.width, this.height);
  if(fn) fn(ret);
  return ret;
};

Rect.prototype.corners = function () {
  const rect = this;
  return [
    { x: rect.x, y: rect.y },
    { x: rect.x + rect.width, y: rect.y },
    { x: rect.x + rect.width, y: rect.y + rect.height },
    { x: rect.x, y: rect.y + rect.height }
  ];
};

if(Rect.prototype.isSquare === undefined) {
  Rect.prototype.isSquare = function () {
    return Math.abs(this.width - this.height) < 1;
  };
}
Rect.prototype.constructor = Rect;

Rect.prototype.getArea = function () {
  return this.width * this.height;
};

Rect.prototype.toSource = function (opts = {}) {
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

Rect.prototype.points = function (ctor = (items) => Array.from(items)) {
  const c = this.corners();
  return ctor(c);
};
Rect.prototype.toCSS = Rect.toCSS;

Rect.prototype.scale = function (factor) {
  let width = this.width * factor;
  let height = this.height * factor;
  this.x += (width - this.width) / 2;
  this.y += (height - this.height) / 2;
  this.width = width;
  this.height = height;
  return this;
};

Rect.prototype.mul = function (...args) {
  Point.prototype.mul.call(this, ...args);
  Size.prototype.mul.call(this, ...args);
  return this;
};

Rect.prototype.div = function (...args) {
  Point.prototype.div.call(this, ...args);
  Size.prototype.div.call(this, ...args);
  return this;
};

Rect.prototype.outset = function (trbl) {
  if(typeof trbl == 'number') trbl = { top: trbl, right: trbl, bottom: trbl, left: trbl };
  this.x -= trbl.left;
  this.y -= trbl.top;
  this.width += trbl.left + trbl.right;
  this.height += trbl.top + trbl.bottom;
  return this;
};

Rect.prototype.inset = function (trbl) {
  if(typeof trbl == 'number') trbl = new TRBL(trbl, trbl, trbl, trbl);

  if(trbl.left + trbl.right < this.width && trbl.top + trbl.bottom < this.height) {
    this.x += trbl.left;
    this.y += trbl.top;
    this.width -= trbl.left + trbl.right;
    this.height -= trbl.top + trbl.bottom;
  }

  return this;
};

Rect.prototype.inside = function (point) {
  return Rect.inside(this, point);
};
Rect.CONTAIN = 16;
Rect.COVER = 32;

Rect.prototype.fit = function (other, align = Align.CENTER | Align.MIDDLE | Rect.CONTAIN) {
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

Rect.prototype.pointFromCenter = function (point) {
  Point.prototype.sub.call(point, this.center);
  point.x /= this.width;
  point.y /= this.height;
  return point;
};

Rect.prototype.toCSS = function () {
  return { ...Point.prototype.toCSS.call(this), ...Size.prototype.toCSS.call(this) };
};

Rect.prototype.toTRBL = function () {
  return { top: this.y, right: this.x + this.width, bottom: this.y + this.height, left: this.x };
};

Rect.prototype.toArray = function () {
  const { x, y, width, height } = this;
  return [x, y, width, height];
};

Rect.prototype.toPoints = function (...args) {
  let ctor = Util.isConstructor(args[0])
    ? (() => {
        let arg = args.shift();
        return (points) => new arg(points);
      })()
    : (points) => Array.from(points);

  let num = typeof args[0] == 'number' ? args.shift() : 4;
  const { x, y, width, height } = this;
  let a = num == 2 ? [new Point(x, y), new Point(x + width, y + height)] : [new Point(x, y), new Point(x + width, y), new Point(x + width, y + height), new Point(x, y + height)];
  return ctor(a);
};

Rect.prototype.toLines = function (ctor = (lines) => Array.from(lines, (points) => new Line(...points))) {
  let [a, b, c, d] = Rect.prototype.toPoints.call(this);
  return ctor([
    [a, b],
    [b, c],
    [c, d],
    [d, a]
  ]);
};

Rect.prototype.align = function (align_to, a = 0) {
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

Rect.prototype.round = function (precision = (0.001, digits, (type = 'round'))) {
  let { x1, y1, x2, y2 } = this.toObject(true);
  let a = new Point(x1, y1).round(precision, digits, type);
  let b = new Point(x2, y2).round(precision, null, type);
  this.x = a.x;
  this.y = a.y;
  this.width = +(b.x - this.x).toFixed(digits);
  this.height = +(b.y - this.y).toFixed(digits);
  return this;
};

Rect.prototype.toObject = function (bb = false) {
  if(bb) {
    const { x1, y1, x2, y2 } = this;
    return { x1, y1, x2, y2 };
  }

  const { x, y, width, height } = this;
  return { x, y, width, height };
};

Rect.prototype.bbox = function () {
  return this.toObject(true);
};

Rect.prototype.transform = function (m) {
  if(Util.isObject(m) && typeof m.toMatrix == 'function') m = m.toMatrix();
  Matrix.prototype.transform_rect.call(m, this);
  return this;
};

Rect.prototype[Symbol.iterator] = function* () {
  let { x, y, width, height } = this;
  for(let prop of [x, y, width, height]) yield prop;
};
Rect.round = (rect) => Rect.prototype.round.call(rect);
Rect.align = (rect, align_to, a = 0) => Rect.prototype.align.call(rect, align_to, a);
Rect.toCSS = (rect) => Rect.prototype.toCSS.call(rect);
Rect.inset = (rect, trbl) => Rect.prototype.inset.call(rect, trbl);
Rect.outset = (rect, trbl) => Rect.prototype.outset.call(rect, trbl);
Rect.center = (rect) => new Point(rect.x + rect.width / 2, rect.y + rect.height / 2);

Rect.bind = (rect) => {
  let obj = new Rect();
};

Rect.inside = (rect, point) => {
  return point.x >= rect.x && point.x <= rect.x + rect.width && point.y >= rect.y && point.y <= rect.y + rect.height;
};

Rect.from = function (obj) {
  const fn = (v1, v2) => [Math.min(v1, v2), Math.max(v1, v2)];
  const h = fn(obj.x1, obj.x2);
  const v = fn(obj.y1, obj.y2);
  const [x1, x2, y1, y2] = [...h, ...v];
  return new Rect(x1, y1, x2 - x1, y2 - y1);
};

Rect.fromCircle = function (...args) {
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
  if(!gen) gen = (k) => (v) => (v === undefined ? o.k : (o.k = v));
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
  Rect.prototype.f = function (...args) {
    Rect.f(this, ...args);
    return this;
  };
}
Util.defineInspect(Rect.prototype, 'x', 'y', 'width', 'height');
const isRect = (rect) => isPoint(rect) && isSize(rect);

Util.defineGetter(Rect, Symbol.species, function () {
  return this;
});
const ImmutableRect = Util.immutableClass(Rect);
delete ImmutableRect[Symbol.species];

Util.defineGetter(ImmutableRect, Symbol.species, function () {
  return ImmutableRect;
});

Rect.prototype.toString = function (opts = {}) {
  if(typeof opts == 'string') opts = { separator: opts };
  const { precision, unit, separator, left, right } = opts;
  let { x, y, width, height } = this;
  let props = [x, y, width, height];
  return left + props.map((p) => p + unit).join(' ') + right;
};

/*
 * concatenanted lib/geom/size.js
 */

export function Size(arg) {
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

Size.prototype.convertUnits = function (w = 'window' in global ? window : null) {
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

Size.prototype.aspect = function () {
  return this.width / this.height;
};

Size.prototype.toCSS = function (units) {
  let ret = {};
  units = typeof units == 'string' ? { width: units, height: units } : units || this.units || { width: 'px', height: 'px' };
  if(this.width !== undefined) ret.width = this.width + (units.width || 'px');
  if(this.height !== undefined) ret.height = this.height + (units.height || 'px');
  return ret;
};

Size.prototype.transform = function (m) {
  this.width = m.xx * this.width + m.yx * this.height;
  this.height = m.xy * this.width + m.yy * this.height;
  return this;
};

Size.prototype.isSquare = function () {
  return Math.abs(this.width - this.height) < 1;
};

Size.prototype.area = function () {
  return this.width * this.height;
};

Size.prototype.resize = function (width, height) {
  this.width = width;
  this.height = height;
  return this;
};

Size.prototype.sum = function (other) {
  return new Size(this.width + other.width, this.height + other.height);
};

Size.prototype.add = function () {
  for(let other of [...arguments]) {
    this.width += other.width;
    this.height += other.height;
  }

  return this;
};

Size.prototype.diff = function (other) {
  return new Size(this.width - other.width, this.height - other.height);
};

Size.prototype.sub = function () {
  for(let other of [...arguments]) {
    this.width -= other.width;
    this.height -= other.height;
  }

  return this;
};

Size.prototype.prod = function (f) {
  const o = isSize(f) ? f : isPoint(f) ? { width: f.x, height: f.y } : { width: f, height: f };
  return new Size(this.width * o.width, this.height * o.height);
};

Size.prototype.mul = function (...args) {
  for(let f of args) {
    const o = isSize(f) ? f : isPoint(f) ? { width: f.x, height: f.y } : { width: f, height: f };
    this.width *= o.width;
    this.height *= o.height;
  }

  return this;
};

Size.prototype.quot = function (other) {
  return new Size(this.width / other.width, this.height / other.height);
};

Size.prototype.inverse = function (other) {
  return new Size(1 / this.width, 1 / this.height);
};

Size.prototype.div = function (f) {
  for(let f of [...arguments]) {
    this.width /= f;
    this.height /= f;
  }

  return this;
};

Size.prototype.round = function (precision = (0.001, digits)) {
  let { width, height } = this;
  this.width = Util.roundTo(width, precision, digits);
  this.height = Util.roundTo(height, precision, digits);
  return this;
};

Size.prototype.bounds = function (other) {
  let w = [Math.min(this.width, other.width), Math.max(this.width, other.width)];
  let h = [Math.min(this.height, other.height), Math.max(this.height, other.height)];
  let scale = h / this.height;
  this.mul(scale);
  return this;
};

Size.prototype.fit = function (size) {
  size = new Size(size);
  let factors = Size.prototype.fitFactors.call(this, size);
  let ret = [Size.prototype.prod.call(this, factors[0]), Size.prototype.prod.call(this, factors[1])];
  return ret;
};

Size.prototype.fitHeight = function (other) {
  other = new Size(other);
  let scale = other.height / this.height;
  this.mul(scale);
  return [this.width, other.width];
};

Size.prototype.fitWidth = function (other) {
  other = new Size(other);
  let scale = other.width / this.width;
  this.mul(scale);
  return [this.height, other.height];
};

Size.prototype.fitFactors = function (other) {
  const hf = other.width / this.width;
  const vf = other.height / this.height;
  return [hf, vf];
};

Size.prototype.toString = function (opts = {}) {
  const { unit, separator, left, right } = opts;
  const { width, height, units } = this;
  return `${left}${width}${units.width || ''}${separator}${height}${units.height || ''}${right}`;
};
Size.area = (sz) => Size.prototype.area.call(sz);
Size.aspect = (sz) => Size.prototype.aspect.call(sz);

Size.bind = (o, p, gen) => {
  const [width, height] = p || ['width', 'height'];
  if(!gen) gen = (k) => (v) => (v === undefined ? o.k : (o.k = v));
  return Util.bindProperties(new Size(0, 0), o, { width, height }, gen);
};
for(let method of Util.getMethodNames(Size.prototype)) Size.method = (size, ...args) => Size.prototype.method.call(size || new Size(size), ...args);
const isSize = (o) => o && ((o.width !== undefined && o.height !== undefined) || (o.x !== undefined && o.x2 !== undefined && o.y !== undefined && o.y2 !== undefined) || (o.left !== undefined && o.right !== undefined && o.top !== undefined && o.bottom !== undefined));

for(let name of ['toCSS', 'isSquare', 'round', 'sum', 'add', 'diff', 'sub', 'prod', 'mul', 'quot', 'div']) {
  Size.name = (size, ...args) => Size.prototype.name.call(size || new Size(size), ...args);
}

Util.defineGetter(Size, Symbol.species, function () {
  return this;
});
const ImmutableSize = Util.immutableClass(Size);

Util.defineGetter(ImmutableSize, Symbol.species, function () {
  return ImmutableSize;
});

/*
 * concatenanted lib/geom/sweepLine.js
 */

export class SweepLineClass {
  constructor() {
    this.objectNodeMap = new Map();
    this.queueHead = null;
  }

  add(object, loVal, hiVal) {
    let hiNode = new SweepLineClass.NodeClass(this, object, SweepLineClass._HI, hiVal);
    let loNode = new SweepLineClass.NodeClass(this, object, SweepLineClass._LO, loVal);
    this.objectNodeMap.set(object, { loNode, hiNode });
  }

  update(object, loVal, hiVal) {
    let n = this.objectNodeMap.get(object);

    if(n) {
      n.hiNode.x = hiVal || n.hiNode.x;
      this.sortNode(n.hiNode);
      n.loNode.x = loVal || n.loNode.x;
      this.sortNode(n.loNode);
    }
  }

  del(object) {
    n = this.objectNodeMap.get(object);

    if(n) {
      this.deleteNode(n.hiNode);
      this.deleteNode(n.loNode);
      this.objectNodeMap.delete(object);
    }
  }

  sortNode(node) {
    function moveNode() {
      this.deleteNode(node);

      if(newLocation === null) {
        node.prev = null;
        node.next = this.queueHead;
        this.queueHead = node;
      } else {
        node.prev = newLocation;
        node.next = newLocation.next;
        if(newLocation.next) newLocation.next.prev = node;
        newLocation.next = node;
      }
    }

    let newLocation = node.prev;

    while(newLocation && node.x < newLocation.x) {
      newLocation = newLocation.prev;
    }

    if(newLocation !== node.prev) moveNode.call(this);
    newLocation = node;

    while(newLocation.next && newLocation.next.x < node.x) {
      newLocation = newLocation.next;
    }

    if(newLocation !== node) moveNode.call(this);
  }

  deleteNode(node) {
    if(node.prev === null) this.queueHead = node.next;
    if(node.prev) node.prev.next = node.next;
    if(node.next) node.next.prev = node.prev;
  }

  findCollisions(collisionFunction) {
    let collision = [];
    let activeObjects = new Set();
    var node = this.queueHead;

    while(node) {
      if(node.loHi === SweepLineClass._LO) {
        let object = node.object;

        for(let ao of activeObjects) {
          if(collisionFunction(object, ao)) {
            collision.push([object, ao]);
          }
        }

        activeObjects.add(object);
      } else {
        activeObjects.delete(node.object);
      }

      node = node.next;
    }

    return collision;
  }

  print(printFunction) {
    var n = this.queueHead;

    while(n) {
      printFunction(n.object, n.loHi, n.x);
      n = n.next;
    }
  }
}
SweepLineClass._LO = false;
SweepLineClass._HI = true;

SweepLineClass.NodeClass = class {
  constructor(sweepLine, object, loHi, x) {
    this.object = object;
    this.parent = sweepLine;
    this.loHi = loHi;
    this.x = x;
    this.prev = null;
    this.next = null;
    if(sweepLine.queueHead) sweepLine.queueHead.prev = this;
    this.next = sweepLine.queueHead;
    sweepLine.queueHead = this;
    sweepLine.sortNode(this);
  }
};

/*
 * concatenanted lib/geom/transformation.js
 */

export class Transformation {
  constructor(typeName) {
    return this;
  }

  get type() {
    let type =
      this.typeName ||
      Util.className(this)
        .toLowerCase()
        .replace(/transform(ation)?/, '')
        .replace(/(ion|ing)$/, 'e');
    return type;
  }

  get [[Symbol.isConcatSpreadable]]() {
    return this.constructor === TransformationList || Object.getPrototypeOf(this) == TransformationList.prototype || Object.getPrototypeOf(this).constructor == TransformationList;
  }

  get axes() {
    return this.axis !== undefined ? [this.axis] : ['x', 'y', 'z'].filter((axis) => axis in this);
  }

  get props() {
    return this.axes.concat(['axis', 'angle'].filter((key) => key in this));
  }

  has(axis) {
    if(this.axis !== undefined) return axis === this.axis;
    return axis in this;
  }

  get is3D() {
    return this.has('z');
  }

  entries() {
    return this.props.map((prop) => [prop, this.prop]);
  }

  toJSON() {
    return Object.fromEntries(this.entries());
  }

  vector(unit) {
    unit = this.unit || unit;
    return (this.is3D ? ['x', 'y', 'z'] : ['x', 'y']).map(unit ? (axis) => this.axis + unit : (axis) => this.axis);
  }

  toString(tUnit) {
    return `${this.type}${this.is3D ? '3d' : ''}(${this.vector(tUnit).join(', ')})`;
  }

  clone() {
    let desc = Object.getOwnPropertyDescriptors(this);
    let props = this.props.reduce((acc, prop) => ({ ...acc, [[prop]]: desc.prop }), {});
    return Object.create(Object.getPrototypeOf(this), props);
  }

  static fromString(arg) {
    let cmdLen = arg.indexOf('(');
    let argStr = arg.slice(cmdLen + 1, arg.indexOf(')'));
    let args = argStr.split(/[,\s\ ]+/g);
    let cmd = arg.substring(0, cmdLen);
    let t;
    let unit;

    args = args
      .filter((arg) => /^[-+0-9.]+[a-z]*$/.test(arg))
      .map((arg) => {
        if(/[a-z]$/.test(arg)) {
          unit = arg.replace(/[-+0-9.]*/g, '');
          arg = arg.replace(/[a-z]*$/g, '');
        }

        return +arg;
      });

    if(cmd.startsWith('rotat')) {
      const axis = cmd.slice(6);
      args = axis != '' ? [args[0], axis] : args;
      t = new Rotation(...args);
    } else if(cmd.startsWith('translat')) {
      const axis = cmd.slice(9);
      args = axis != '' ? [args[0], axis] : args;
      t = new Translation(...args);
    } else if(cmd.startsWith('scal')) {
      const axis = cmd.slice(5);
      args = axis != '' ? [args[0], axis] : args;
      t = new Scaling(...args);
    } else if(cmd.startsWith('matrix')) {
      t = new MatrixTransformation(...args);
    }

    if(unit) t.unit = unit;
    return t;
  }

  [[Symbol.toPrimitive]](hint) {
    if(hint == 'string') return this.toString();
    return this.toString() != '';
  }

  static get rotation() {
    return Rotation;
  }

  static get translation() {
    return Translation;
  }

  static get scaling() {
    return Scaling;
  }

  static get matrix() {
    return MatrixTransformation;
  }
}

Object.defineProperty(Transformation, Symbol.hasInstance, {
  value(inst) {
    return [Transformation, MatrixTransformation, Rotation, Translation, Scaling, TransformationList].some((ctor) => Object.getPrototypeOf(inst) == ctor.prototype);
  }
});
const ImmutableTransformation = Util.immutableClass(Transformation);

export class Rotation extends Transformation {
  angle = 0;
  static RAD2DEG = 180 / Math.PI;
  static DEG2RAD = 1 / Rotation.RAD2DEG;
  constructor(angle, axis) {
    super('rotate');
    if(typeof axis == 'string' && ['x', 'y', 'z'].indexOf(axis.toLowerCase()) != -1) this.axis = axis.toLowerCase();
    this.angle = angle;
  }

  invert() {
    return new Rotation(-this.angle, this.axis);
  }

  get values() {
    return { [[this.axis || 'z']]: this.angle };
  }

  get is3D() {
    return this.axis == 'z';
  }

  isZero() {
    return this.angle == 0;
  }

  toString(rUnit) {
    rUnit = rUnit || this.unit || '';
    const axis = this.axis !== undefined ? this.axis.toUpperCase() : '';
    const angle = this.constructor.convertAngle(this.angle, rUnit);
    return `rotate${this.is3D ? axis : ''}(${angle}${rUnit})`;
  }

  toSource() {
    let o = Util.colorText('new ', 1, 31) + Util.colorText(Util.className(this), 1, 33) + Util.colorText('(' + this.angle + ')', 1, 36);
    return o;
  }

  toMatrix() {
    return Matrix.rotate(Rotation.DEG2RAD * this.angle);
  }

  accumulate(other) {
    if(this.type !== other.type && this.axis !== other.axis) throw new Error(Util.className(this) + ': accumulate mismatch');

    return new Rotation(this.angle + other.angle, this.axis);
  }

  static convertAngle(angle, unit) {
    switch (unit) {
      case 'deg':
        return angle;
      case 'rad':
        return this.DEG2RAD * angle;
      case 'turn':
        return angle / 360;
      default:
        return angle;
    }
  }
}
const ImmutableRotation = Util.immutableClass(Rotation);

export class Translation extends Transformation {
  x = 0;
  y = 0;
  constructor(...args) {
    super('translate');

    if(typeof args[1] == 'string' && ['x', 'y', 'z'].indexOf(args[1].toLowerCase()) != -1) {
      const axis = args[1].toLowerCase();
      this.axis = args[0];
    } else {
      const [x, y, z] = args;
      this.x = x;
      this.y = y;
      if(z !== undefined) this.z = z;
    }
  }

  get values() {
    const { x, y, z } = this;
    return 'z' in this ? { x, y, z } : { x, y };
  }

  isZero() {
    const { x, y, z } = this;
    return 'z' in this ? x == 0 && y == 0 && z == 0 : x == 0 && y == 0;
  }

  toMatrix(matrix = Matrix.IDENTITY) {
    return matrix.translate(this.x, this.y, this.z);
  }

  invert() {
    const { x, y, z } = this;
    return z !== undefined ? new Translation(-x, -y, -z) : new Translation(Math.abs(x) == 0 ? 0 : -x, Math.abs(y) == 0 ? 0 : -y);
  }

  accumulate(other) {
    if(this.type !== other.type) throw new Error(Util.className(this) + ': accumulate mismatch');

    if(this.is3D) return new Translation(this.x + other.x, this.y + other.y, this.z + other.z);
    return new Translation(this.x + other.x, this.y + other.y);
  }
}
const ImmutableTranslation = Util.immutableClass(Translation);

export class Scaling extends Transformation {
  x = 1;
  y = 1;
  constructor(...args) {
    super('scale');

    if(typeof args[1] == 'string' && ['x', 'y', 'z'].indexOf(args[1].toLowerCase()) != -1) {
      const axis = args[1].toLowerCase();
      this.axis = args[0];
    } else {
      const [x, y, z] = args;
      this.x = x;
      this.y = y === undefined ? x : y;
      if(z !== undefined) this.z = z;
    }
  }

  get values() {
    const { x, y, z } = this;
    return 'z' in this ? { x, y, z } : { x, y };
  }

  toMatrix(matrix = Matrix.IDENTITY) {
    return matrix.scale(this.x, this.y, this.z);
  }

  isZero() {
    const { x, y, z } = this;
    return 'z' in this ? x == 0 && y == 0 && z == 0 : x == 0 && y == 0;
  }

  invert() {
    const { x, y, z } = this;
    return z !== undefined ? new Scaling(1 / x, 1 / y, 1 / z) : new Scaling(1 / x, 1 / y);
  }

  accumulate(other) {
    if(this.type !== other.type) throw new Error(Util.className(this) + ': accumulate mismatch');

    if(this.is3D) return new Scaling(this.x * other.x, this.y * other.y, this.z * other.z);
    return new Scaling(this.x * other.x, this.y * other.y);
  }
}
const ImmutableScaling = Util.immutableClass(Scaling);

export class MatrixTransformation extends Transformation {
  matrix = Matrix.IDENTITY;
  constructor(init) {
    super('matrix');
    if(init instanceof Matrix) this.matrix = init;
    else if(isMatrix(init)) this.matrix = new Matrix(init);
    else this.matrix = new Matrix(...arguments);
  }

  get values() {
    return this.matrix.values();
  }

  toMatrix() {
    return this.matrix.clone();
  }

  toString() {
    return this.matrix.toString('');
  }

  invert() {
    return new MatrixTransformation(this.matrix.invert());
  }

  isZero() {
    return this.matrix.isIdentity();
  }

  accumulate(other) {
    if(this.type !== other.type) throw new Error(Util.className(this) + ': accumulate mismatch');

    return new MatrixTransformation(this.matrix.multiply(other.matrix));
  }
}
const ImmutableMatrixTransformation = Util.immutableClass(MatrixTransformation);

export class TransformationList extends Array {
  constructor(init, ...rest) {
    super();
    if(init !== undefined) this.initialize(init, ...rest);
    return this;
  }

  initialize(init, ...args) {
    if(typeof init == 'number') while(this.length < init) this.push(undefined);
    else if(typeof init == 'string') TransformationList.prototype.fromString.call(this, init);
    else if(init instanceof Array) TransformationList.prototype.fromArray.call(this, init);
    else throw new Error('No such initialization: ' + init);

    return this;
  }

  get [[Symbol.isConcatSpreadable]]() {
    return true;
  }

  static get [[Symbol.species]]() {
    return TransformationList;
  }

  fromString(str) {
    let n,
      a = [];

    for(let i = 0; i < str.length; i += n) {
      let s = str.slice(i);
      n = s.indexOf(')') + 1;
      if(n == 0) n = str.length;
      s = s.slice(0, n).trim();
      if(s != '') a.push(s);
    }

    return this.fromArray(a);
  }

  fromArray(arr) {
    for(let i = 0; i < arr.length; i++) {
      const arg = arr.i;
      if(arg instanceof Transformation) this.push(arg);
      else if(typeof arg == 'string') this.push(Transformation.fromString(arg));
      else throw new Error('No such transformation: ' + arg);
    }

    return this;
  }

  static fromString(str) {
    return new TransformationList().fromString(str);
  }

  static fromArray(arr) {
    return new TransformationList().fromArray(arr);
  }

  static fromMatrix(matrix) {
    matrix = matrix instanceof Matrix ? matrix : new Matrix(matrix);
    const transformations = Matrix.decompose(matrix, true);

    Util.extend(transformations.scale, {
      toArray() {
        return [this.x, this.y];
      }
    });

    Util.extend(transformations.translate, {
      toArray() {
        return [this.x, this.y];
      }
    });

    let ret = new TransformationList();
    ret.translate(...transformations.translate.toArray());
    ret.rotate(transformations.rotate);
    ret.scale(...transformations.scale.toArray());
    return ret;
  }

  push(...args) {
    for(let arg of args) {
      if(typeof arg == 'string') arg = Transformation.fromString(arg);
      else if(isMatrix(arg)) arg = new MatrixTransformation(arg);
      Array.prototype.push.call(this, arg);
    }

    return this;
  }

  clone() {
    return this.map((t) => t.clone());
  }

  unshift(...args) {
    for(let arg of args.reverse()) {
      if(typeof arg == 'string') arg = Transformation.fromString(arg);
      Array.prototype.unshift.call(this, arg);
    }

    return this;
  }

  rotate(...args) {
    Array.prototype.push.call(this, new Rotation(...args));
    return this;
  }

  translate(x, y) {
    let trans = this.filter((t) => !t.type.startsWith('translat'));
    let vec = new Point(x, y);
    vec = vec.round(0.00001, 5);
    if(Math.abs(vec.x) != 0 || Math.abs(vec.y) != 0) Array.prototype.push.call(this, new Translation(vec.x, vec.y));
    return this;
  }

  scale(...args) {
    Array.prototype.push.call(this, new Scaling(...args));
    return this;
  }

  matrix(...args) {
    Array.prototype.push.call(this, new MatrixTransformation(...args));
    return this;
  }

  toString(tUnit, rUnit) {
    return this.map((t) => t.toString(t.type.startsWith('scal') ? '' : t.type.startsWith('rotat') ? rUnit : tUnit)).join(' ');
  }

  [[Symbol.toStringTag]]() {
    return this.toString();
  }

  toSource() {
    let s = Util.colorText('new ', 1, 31) + Util.colorText(Util.className(this), 1, 33) + Util.colorText('([', 1, 36);
    s += this.map((t) => t.toSource()).join(', ');
    return s + Util.colorText('])', 1, 36);
  }

  toMatrices() {
    return Array.prototype.map.call(this, (t) => t.toMatrix());
  }

  toMatrix() {
    let matrix = Matrix.IDENTITY;
    for(let other of this.toMatrices()) matrix = matrix.multiply(other);
    return matrix;
  }

  undo() {
    let ret = new TransformationList();

    for(let i = this.length - 1; i >= 0; i--) Array.prototype.push.call(ret, this.i.invert());

    return ret;
  }

  merge(...args) {
    for(let arg of args) {
      if(typeof arg == 'string') arg = TransformationList.fromString(arg);
      TransformationList.prototype.push.apply(this, arg);
    }

    return this;
  }

  decompose(degrees = true) {
    let matrix = this.toMatrix();
    const { translate, rotate, scale } = matrix.decompose(degrees);
    let ret = { translate, rotate, scale };

    ret.scale.toArray = ret.translate.toArray = function toArray() {
      return [this.x, this.y];
    };

    return ret;
  }

  findLast(predicate) {
    for(let i = this.length - 1; i >= 0; --i) {
      const x = this.i;
      if(predicate(x)) return x;
    }

    return null;
  }

  get rotation() {
    return this.findLast((item) => item.type.startsWith('rotat'));
  }

  get scaling() {
    return this.findLast((item) => item.type.startsWith('scal'));
  }

  get translation() {
    return this.findLast((item) => item.type.startsWith('translat'));
  }

  get last() {
    return this.at(-1);
  }

  get first() {
    return this.at(0);
  }

  at(pos) {
    if(pos < 0) pos += this.length;
    return this.pos;
  }

  collapse() {
    let ret = new TransformationList();

    for(let i = 0; i < this.length; i++) {
      let item = this.i;

      if(i + 1 < this.length && this[i + 1].type == this.i.type) {
        item = item.accumulate(this[i + 1]);
        i++;
      } else {
        item = item.clone();
      }

      Array.prototype.push.call(ret, item);
    }

    return ret;
  }

  collapseAll() {
    return TransformationList.fromMatrix(this.toMatrix());
  }

  invert() {
    return new TransformationList(this.reduceRight((acc, t) => [...acc, t.invert()], []));
  }

  join(sep = ' ') {
    return Array.prototype.join.call(this, sep);
  }

  clear() {
    return this.splice(0, this.length);
  }
}
const { concat, copyWithin, find, findIndex, lastIndexOf, pop, push, shift, unshift, slice, splice, includes, indexOf, entries, filter, map, every, some, reduce, reduceRight } = Array.prototype;

Util.inherit(
  TransformationList.prototype,
  { concat, copyWithin, find, findIndex, lastIndexOf, pop, shift, slice, splice, includes, indexOf, entries, filter, map, every, some, reduce, reduceRight },
  {
    [[Symbol.iterator]]() {
      return Array.prototype[Symbol.iterator];
    },
    [[Symbol.isConcatSpreadable]]() {
      return true;
    }
  }
);
const ImmutableTransformationList = Util.immutableClass(TransformationList);

Util.defineGetter(ImmutableTransformationList, Symbol.species, function () {
  return ImmutableTransformationList;
});

/*
 * concatenanted lib/geom/trbl.js
 */

export function TRBL(arg) {
  let ret = this instanceof TRBL ? this : {};
  let args = [...arguments];

  if(typeof arg === 'object' && !Util.isArray(arg)) {
    Object.keys(arg).forEach((k) => {
      const matches = /(top|right|bottom|left)/i.exec(k);
      ret[matches[0].toLowerCase()] = parseInt(arg.k);
    });
  } else if(arg) {
    if(args.length > 1) arg = args;
    if(typeof arg === 'string') arg = [...arg.matchAll(/^[0-9.]+(|px|em|rem|pt|cm|mm)$/g)];
    else if(arg.length == 4) arg = arg.map((v) => parseInt(v.replace(/[a-z]*$/g, '')));
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

TRBL.prototype.null = function () {
  return this.top == 0 && this.right == 0 && this.bottom == 0 && this.left == 0;
};
TRBL.null = (trbl) => TRBL.prototype.null.call(trbl);
TRBL.neg = (trbl = this) => ({ top: -trbl.top, right: -trbl.right, bottom: -trbl.bottom, left: -trbl.left });

TRBL.prototype.isNaN = function () {
  return isNaN(this.top) || isNaN(this.right) || isNaN(this.bottom) || isNaN(this.left);
};

Object.defineProperty(TRBL.prototype, 'inset', {
  get() {
    return (rect) => Rect.inset(rect, this);
  }
});

Object.defineProperty(TRBL.prototype, 'outset', {
  get() {
    return (rect) => Rect.outset(rect, this);
  }
});

TRBL.prototype.add = function (other) {
  this.top += other.top;
  this.right += other.right;
  this.bottom += other.bottom;
  this.left += other.left;
};

TRBL.prototype.union = function (other) {
  this.top = other.top < this.top ? other.top : this.top;
  this.right = other.right > this.right ? other.right : this.right;
  this.bottom = other.bottom > this.bottom ? other.bottom : this.bottom;
  this.left = other.left < this.left ? other.left : this.left;
};

TRBL.prototype.toRect = function () {
  return new Rect({ x: this.left, y: this.top, width: this.right - this.left, height: this.bottom - this.top });
};

TRBL.prototype.toRect = function () {
  return new Rect({ x: this.left, y: this.top, width: this.right - this.left, height: this.bottom - this.top });
};
TRBL.union = (trbl, other) => ({ top: other.top < trbl.top ? other.top : trbl.top, right: other.right > trbl.right ? other.right : trbl.right, bottom: other.bottom > trbl.bottom ? other.bottom : trbl.bottom, left: other.left < trbl.left ? other.left : trbl.left });
TRBL.toRect = (trbl) => new Rect(trbl.left, trbl.top, trbl.right - trbl.left, trbl.bottom - trbl.top);

TRBL.prototype.toString = function (unit = 'px') {
  return '' + this.top + '' + unit + ' ' + this.right + '' + unit + ' ' + this.bottom + '' + unit + ' ' + this.left + unit;
};

TRBL.prototype.toSource = function () {
  return '{top:' + this.top + ',right:' + this.right + ',bottom:' + this.bottom + ',left:' + this.left + '}';
};

for(let name of ['null', 'isNaN', 'outset', 'toRect', 'toSource']) {
  TRBL.name = (points) => TRBL.prototype.name.call(points);
}

export function isTRBL(obj) {
  return top in obj && right in obj && bottom in obj && left in obj;
}

Util.defineGetter(TRBL, Symbol.species, function () {
  return this;
});
const ImmutableTRBL = Util.immutableClass(TRBL);

Util.defineGetter(ImmutableTRBL, Symbol.species, function () {
  return ImmutableTRBL;
});

/*
 * concatenanted lib/geom/vector.js
 */

export function Vector(init, n, base = Int32Array) {
  if(n === undefined) n = init.length;
  let buf = new ArrayBuffer(base.BYTES_PER_ELEMENT * n);
  let vec = new base(buf);

  for(let i = 0; i < n; i++) vec.set([init.i], i);

  return vec;
}

/*
 * concatenanted lib/geom/align.js
 */

export function Align(arg) {}
Align.CENTER = 0;
Align.LEFT = 1;
Align.RIGHT = 2;
Align.MIDDLE = 0;
Align.TOP = 4;
Align.BOTTOM = 8;
Align.horizontal = (alignment) => alignment & (Align.LEFT | Align.RIGHT);
Align.vertical = (alignment) => alignment & (Align.TOP | Align.BOTTOM);
const Anchor = Align;

/*
 * concatenanted lib/geom/bbox.js
 */

export class BBox {
  static fromPoints(pts) {
    let pt = pts.shift();
    let bb = new BBox(pt.x, pt.y, pt.x, pt.y);
    bb.update(pts);
    return bb;
  }

  static fromRect(rect) {
    return new BBox(rect.x, rect.y, rect.x + rect.width, rect.y + rect.height);
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

  updateList(list, offset = (0.0, (objFn = ((item) => item, (t = (a) => a))))) {
    for(let arg of list) this.update(t(arg), offset, objFn(arg));
    return this;
  }

  update(arg, offset = (0.0, (obj = null))) {
    if(Util.isArray(arg)) return this.updateList(arg, offset);
    else if(Util.isObject(arg)) {
      if(typeof arg.bbox == 'function') {
        arg = arg.bbox();
      } else {
        if(arg.x !== undefined && arg.y != undefined) this.updateXY(arg.x, arg.y, offset, (name) => (this.objects.name = obj || arg));
        if(arg.x1 !== undefined && arg.y1 != undefined) this.updateXY(arg.x1, arg.y1, 0, (name) => (this.objects.name = obj || arg));
        if(arg.x2 !== undefined && arg.y2 != undefined) this.updateXY(arg.x2, arg.y2, 0, (name) => (this.objects.name = obj || arg));
      }
    }

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

  toRect(proto) {
    let r = this.rect;
    return Object.setPrototypeOf(r, proto || Object.prototype);
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

  transform(fn = ((arg) => arg, out)) {
    if(!out) out = this;

    for(let prop of ['x1', 'y1', 'x2', 'y2']) {
      const v = this.prop;
      out.prop = fn(v);
    }

    return this;
  }

  round(fn = (arg) => Math.round(arg)) {
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

  static from(iter, tp = (p) => p) {
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

  [Symbol.iterator] = [Symbol.iterator];
}

/*
 * concatenanted lib/geom/graph.js
 */

export class Graph {
  constructor() {
    this.nodes = [];
    this.connections = [];
  }

  checkForDuplicateNodes() {
    for(let i = 0; i < this.nodes.length; i++) {
      for(let j = i + 1; j < this.nodes.length; j++) {
        if(this.nodes.i.equals(this.nodes.j)) {
        }
      }
    }
  }

  checkForDuplicateConnections() {
    for(let i = 0; i < this.connections.length; i++) {
      for(let j = i + 1; j < this.connections.length; j++) {
        if(this.connections.i.equals(this.connections.j)) {
        }
      }
    }
  }

  getAdjacencyList() {
    let adjacencyList = [];

    for(let i = 0; i < this.nodes.length; i++) {
      adjacencyList.i = [];
      let node = this.nodes.i;
      node.id = i;
      let connectedNodes = this.getConnectedNodes(node);

      connectedNodes.forEach((node) => {
        adjacencyList.i.push(this.nodes.indexOf(node));
      });
    }

    return adjacencyList;
  }

  getConnectedNodes(node) {
    let myConnections = this.connections.filter((connection) => connection.node1 === node || connection.node2 === node);
    let connectedNodes = new Set();

    myConnections.forEach((connection) => {
      connectedNodes.add(connection.node1).add(connection.node2);
    });

    connectedNodes.delete(node);
    return connectedNodes;
  }

  getConnectionsFromNode(node) {
    return this.connections.filter((connection) => connection.node1 === node || connection.node2 === node);
  }

  getNeighboringNodes(node) {
    let nodeConnections = this.getConnectionsFromNode(node);
    let neighboringNodes = [];

    nodeConnections.forEach((nodeConnection) => {
      if(nodeConnection.node1 !== node) {
        neighboringNodes.push(nodeConnection.node1);
      } else if(nodeConnection.node2 !== node) {
        neighboringNodes.push(nodeConnection.node2);
      }
    });

    return neighboringNodes;
  }

  isConnected(node1, node2) {
    return this.connections.some((connection) => {
      return (connection.node1 === node1 && connection.node2 === node2) || (connection.node2 === node1 && connection.node1 === node2);
    });
  }

  addConnection(node1, node2) {
    if(!node1 || !node2) {
      return false;
    }

    if(node1.equals(node2)) {
      return false;
    }

    let node1Matches = this.nodes.filter((node) => Point.equals(node.point, node1.point));
    let node2Matches = this.nodes.filter((node) => Point.equals(node.point, node2.point));

    if(node1Matches.length > 1) {
      return false;
    }

    if(node2Matches.length > 1) {
      return false;
    }

    if(node1Matches.length === 0) {
      return;
    }

    if(node2Matches.length === 0) {
      return;
    }

    let newConnection = new Graph.Connection(node1Matches[0], node2Matches[0]);
    let duplicateConnections = this.connections.filter((connection) => connection.equals(newConnection));

    if(duplicateConnections.length > 1) {
    } else if(duplicateConnections.length === 1) {
      return;
    } else {
      this.connections.push(newConnection);
    }
  }

  addNode(newNode) {
    let duplicateNodes = this.nodes.filter((node) => Point.equals(newNode.point, node.point));
    if(duplicateNodes.length > 1) {
    }

    if(duplicateNodes.length === 0) {
      this.nodes.push(newNode);
    }
  }

  findMinimumCyclesFromSource(adjacencyListSourceIndex) {
    const adjacencyList = this.getAdjacencyList();
    var neighbors = adjacencyList.adjacencyListSourceIndex;
    let paths = [];

    for(let i = 0; i < neighbors.length; i++) {
      let path = [adjacencyListSourceIndex];
      let startingNeighborIndex = neighbors.i;
      let tmpAdjacencyList = { ...adjacencyList };
      tmpAdjacencyList.startingNeighborIndex = tmpAdjacencyList.startingNeighborIndex.filter((nodeIndex) => nodeIndex !== adjacencyListSourceIndex);
      path = path.concat(Graph.findShortestPath(tmpAdjacencyList, startingNeighborIndex, adjacencyListSourceIndex));
      paths.push(path);
    }

    return paths;
  }

  findMinimumCycles() {
    const adjacencyList = this.getAdjacencyList();
    let cycles = [];

    for(let i = 0; i < adjacencyList.length; i++) {
      let paths = this.findMinimumCyclesFromSource(i);
      cycles = cycles.concat(paths);
    }

    let cyclesToRemove = [];
    let uniqueCycles = [];

    for(let i = 0; i < cycles.length; i++) {
      if(uniqueCycles.filter((cycle) => Graph.doArraysContainSameElements(cycle, cycles.i)).length === 0) {
        uniqueCycles.push(cycles.i);
      }
    }

    let edgePairs = [];
    let edgePairCount = {};

    for(let i = 0; i < uniqueCycles.length; i++) {
      let cycle = uniqueCycles.i;

      for(let j = 1; j < cycle.length; j++) {
        let edgePair = [];

        if(cycle[j - 1] < cycle.j) {
          edgePair = [cycle[j - 1], cycle.j];
        } else {
          edgePair = [cycle.j, cycle[j - 1]];
        }

        if(edgePairCount[edgePair[0] + ',' + edgePair[1]]) {
          edgePairCount[edgePair.join(',')]++;
        } else {
          edgePairCount[edgePair.join(',')] = 1;
        }

        edgePairs.push(edgePair);
      }
    }

    let edgesOnlyUsedOnce = [];

    edgePairs.forEach((edgePair) => {
      if(edgePairCount[edgePair.join(',')] == 1) {
        edgesOnlyUsedOnce.push(edgePair);
      }
    });

    let leftoverAdjacencyList = [];

    edgesOnlyUsedOnce.forEach((edge) => {
      if(leftoverAdjacencyList[edge[0]]) {
        leftoverAdjacencyList[edge[0]].push(edge[1]);
      } else {
        leftoverAdjacencyList[edge[0]] = [edge[1]];
      }

      if(leftoverAdjacencyList[edge[1]]) {
        leftoverAdjacencyList[edge[1]].push(edge[0]);
      } else {
        leftoverAdjacencyList[edge[1]] = [edge[0]];
      }
    });

    let extraPaths = [];

    for(let i = 0; i < leftoverAdjacencyList.length; i++) {
      let neighbors = leftoverAdjacencyList.i;

      if(neighbors) {
        for(let j = 0; j < neighbors.length; j++) {
          let path = [i];
          let startingNeighborIndex = neighbors.j;
          let tmpAdjacencyList = { ...leftoverAdjacencyList };
          tmpAdjacencyList.startingNeighborIndex = tmpAdjacencyList.startingNeighborIndex.filter((nodeIndex) => nodeIndex !== i);
          path = path.concat(Graph.findShortestPath(tmpAdjacencyList, startingNeighborIndex, i));
          extraPaths.push(path);
        }
      }
    }

    let leftoverCycles = [];

    for(let i = 0; i < extraPaths.length; i++) {
      if(leftoverCycles.filter((cycle) => Graph.doArraysContainSameElements(cycle, extraPaths.i)).length === 0) {
        leftoverCycles.push(extraPaths.i);
      }
    }

    let longestCycleLength = -1;
    let longestCycleIndex = -1;

    for(let i = 0; i < leftoverCycles.length; i++) {
      if(leftoverCycles.i.length > longestCycleLength) {
        longestCycleIndex = i;
        longestCycleLength = leftoverCycles.i.length;
      }
    }

    leftoverCycles.splice(longestCycleIndex, 1);
    uniqueCycles = leftoverCycles.concat(uniqueCycles);
    return uniqueCycles;
  }

  static doArraysContainSameElements(array1, array2) {
    if(array1.length !== array2.length) {
      return false;
    } else {
      for(let i = 0; i < array1.length; i++) {
        if(array2.includes(array1.i) === false) {
          return false;
        }
      }
    }

    return true;
  }

  static findShortestPath(adjacencyList, source, target) {
    if(source == target) {
      print('SOURCE AND PATH ARE SAME');
      return [target];
    }

    let visitQueue = [source];
    let visitedStatusList = { source: true };
    let predecessorList = {};
    let nextInQueue = 0;

    while(nextInQueue < visitQueue.length) {
      let node = visitQueue[nextInQueue++];
      let neighbors = adjacencyList.node;

      for(let i = 0; i < neighbors.length; i++) {
        var neighbor = neighbors.i;

        if(!visitedStatusList.neighbor) {
          visitedStatusList.neighbor = true;

          if(neighbor === target) {
            let path = [target];

            while(node !== source) {
              path.push(node);
              node = predecessorList.node;
            }

            path.push(node);
            path.reverse();
            return path;
          }

          predecessorList.neighbor = node;
          visitQueue.push(neighbor);
        }
      }
    }

    print('there is no path from ' + source + ' to ' + target);
  }
}

Graph.Node = class {
  constructor(point, connections) {
    this.point = point;
  }

  equals(node) {
    return Point.equals(node.point, this.point);
  }
};

Graph.Connection = class {
  constructor(node1, node2) {
    this.node1 = node1;
    this.node2 = node2;
  }

  equals(connection) {
    return (this.node1.equals(connection.node1) && this.node2.equals(connection.node2)) || (this.node2.equals(connection.node1) && this.node1.equals(connection.node2));
  }
};

/*
 * concatenanted lib/geom/intersection.js
 */

export class Intersection {
  constructor(line1, line2, intersectionPoint) {
    this.line1 = line1;
    this.line2 = line2;
    this.point = intersectionPoint;
  }

  static findIntersection(line1, line2) {
    let denominator, a, b, numerator1, numerator2;
    let result = new Intersection(line1, line2, {});
    denominator = (line2.y2 - line2.y1) * (line1.x2 - line1.x1) - (line2.x2 - line2.x1) * (line1.y2 - line1.y1);
    if(denominator == 0) return null;
    a = line1.y1 - line2.y1;
    b = line1.x1 - line2.x1;
    numerator1 = (line2.x2 - line2.x1) * a - (line2.y2 - line2.y1) * b;
    numerator2 = (line1.x2 - line1.x1) * a - (line1.y2 - line1.y1) * b;
    a = numerator1 / denominator;
    b = numerator2 / denominator;
    result.point.x = line1.x1 + a * (line1.x2 - line1.x1);
    result.point.y = line1.y1 + a * (line1.y2 - line1.y1);
    result.line1 = line1;
    result.line2 = line2;

    if(a > 0 && a < 1 && b > 0 && b < 1) {
      return result;
    } else {
      return null;
    }
  }

  static equals(intersection1, intersection2) {
    return Point.equals(intersection1.point, intersection2.point) && ((Line.equals(intersection1.line1, intersection2.line1) && Line.equals(intersection1.line2, intersection2.line2)) || (Line.equals(intersection1.line1, intersection2.line2) && Line.equals(intersection1.line2, intersection2.line1)));
  }
}

/*
 * concatenanted lib/geom/line.js
 */

export function Line(x1, y1, x2, y2) {
  let obj;
  let arg;
  let args = [...arguments];
  let ret;

  if(args.length >= 4 && args.every((arg) => !isNaN(parseFloat(arg)))) {
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
  } else if(arg && arg.length >= 4 && arg.slice(0, 4).every((arg) => !isNaN(parseFloat(arg)))) {
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
const isLine = (obj) => (Util.isObject(obj) && ['x1', 'y1', 'x2', 'y2'].every((prop) => obj.prop !== undefined)) || ['a', 'b'].every((prop) => isPoint(obj.prop));

Line.prototype.intersect = function (other) {
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

Line.prototype.direction = function () {
  var dist = Point.prototype.distance.call(this.a, this.b);
  return Point.prototype.quot.call(Line.prototype.getSlope.call(this), dist);
};

Line.prototype.getVector = function () {
  return { x: this.x2 - this.x1, y: this.y2 - this.y1 };
};
Object.defineProperty(Line.prototype, 'vector', { get: Line.prototype.getVector });

Line.prototype.getSlope = function () {
  return (this.y2 - this.y1) / (this.x2 - this.x1);
};
Object.defineProperty(Line.prototype, 'slope', { get: Line.prototype.getSlope });

Line.prototype.yIntercept = function () {
  let v = Line.prototype.getVector.call(this);

  if(v.x !== 0) {
    let slope = v.y / v.x;
    return [this.a.y - this.a.x * slope, slope || 0];
  }
};

Line.prototype.xIntercept = function () {
  let v = Line.prototype.getVector.call(this);

  if(v.y !== 0) {
    let slope = v.x / v.y;
    return [this.a.x - this.a.y * slope, slope || 0];
  }
};

Line.prototype.isHorizontal = function () {
  return Line.prototype.getVector.call(this).y === 0;
};

Line.prototype.isVertical = function () {
  return Line.prototype.getVector.call(this).x === 0;
};

Line.prototype.isNull = function () {
  return this.x1 == 0 && this.y1 == 0 && this.x2 == 0 && this.y2 == 0;
};

Line.prototype.equations = function () {
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

Line.prototype.functions = function () {
  let i;
  let fns = {};

  if((i = Line.prototype.yIntercept.call(this))) {
    let [y0, myx] = i;
    fns.y = (x) => y0 + myx * x;
  } else {
    let { y } = this.a;
    fns.y = new Function('x', `return ${y}`);
  }

  if((i = Line.prototype.xIntercept.call(this))) {
    let [x0, mxy] = i;
    fns.x = (y) => x0 + mxy * y;
  } else {
    let { x } = this.a;
    fns.x = new Function('y', `return ${x}`);
  }

  return fns;
};

Line.prototype.angle = function () {
  return Point.prototype.angle.call(Line.prototype.getVector.call(this));
};

Line.prototype.getLength = function () {
  const { a, b } = this;
  const { x1, y1, x2, y2 } = this;
  return Point.prototype.distance.call(a, b);
};

Line.prototype.endpointDist = function (point) {
  return Math.min(point.distance(this.a), point.distance(this.b));
};

Line.prototype.matchEndpoints = function (arr) {
  const { a, b } = this;
  return [...arr.entries()].filter(([i, otherLine]) => !Line.prototype.equals.call(this, otherLine) && (Point.prototype.equals.call(a, otherLine.a) || Point.prototype.equals.call(b, otherLine.b) || Point.prototype.equals.call(b, otherLine.a) || Point.prototype.equals.call(a, otherLine.b)));
};

Line.prototype.distanceToPointSquared = function (p) {
  const { a, b } = this;
  var l2 = Point.prototype.distanceSquared.call(a, b);
  if(l2 === 0) return Point.prototype.distanceSquared.call(p, a);
  var t = ((p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y)) / l2;
  t = Math.max(0, Math.min(1, t));
  return Point.prototype.distanceSquared.call(p, new Point(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)));
};

Line.prototype.distanceToPoint = function (p) {
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

Line.prototype.pointAt = function (pos) {
  return new Point(pos * (this.x2 - this.x1) + this.x1, pos * (this.y2 - this.y1) + this.y1);
};

Line.prototype.transform = function (m) {
  this.a = this.a.transform(m);
  this.b = this.b.transform(m);
  return this;
};

Line.prototype.bbox = function () {
  const { x1, y1, x2, y2 } = this;
  return new BBox(x1, y1, x2, y2);
};

Line.prototype.add = function (other) {
  const { x1, y1, x2, y2 } = Line(...arguments);
  this.x1 += x1;
  this.y1 += y1;
  this.x2 += x2;
  this.y2 += y2;
  return this;
};

Line.prototype.points = function () {
  const { a, b } = this;
  return [a, b];
};

Line.prototype.diff = function (other) {
  other = Line(...arguments);
  return new Line(Point.diff(this.a, other.a), Point.diff(this.b, other.b));
};

Line.prototype.inspect = function () {
  const { x1, y1, x2, y2 } = this;
  return 'Line{ ' + inspect({ x1, y1, x2, y2 }) + ' }';
};

Line.prototype.toString = function () {
  const { x1, y1, x2, y2 } = this;
  return Point.toString(this.a || Point(x1, y1)) + ' -> ' + Point.toString(this.b || Point(x2, y2));
};

Line.prototype.toSource = function () {
  let { a, b } = this;
  return `new Line(${a.x},${a.y},${b.x},${b.y})`;
};

Line.prototype.reverse = function () {
  const { a, b } = this;
  return new Line(b, a);
};

Line.prototype.toObject = function (t = (num) => num) {
  const { x1, y1, x2, y2 } = this;
  const obj = { x1: t(x1), y1: t(y1), x2: t(x2), y2: t(y2) };
  return obj;
};

Line.prototype.clone = function () {
  const ctor = this.constructor[Symbol.species];
  const { x1, y1, x2, y2 } = this;
  return new ctor(x1, y1, x2, y2);
};

Line.prototype.round = function (precision = (0.001, digits)) {
  let { x1, y1, x2, y2 } = this;
  this.a.x = Util.roundTo(x1, precision, digits);
  this.a.y = Util.roundTo(y1, precision, digits);
  this.b.x = Util.roundTo(x2, precision, digits);
  this.b.y = Util.roundTo(y2, precision, digits);
  return this;
};

Line.prototype.some = function (pred) {
  return pred(this.a) || pred(this.b);
};

Line.prototype.every = function (pred) {
  return pred(this.a) && pred(this.b);
};

Line.prototype.includes = function (point) {
  return Point.prototype.equals.call(this.a, point) || Point.prototype.equals.call(this.b, point);
};

Line.prototype.equals = function (other) {
  other = Line(other);
  if(Point.equals(this.a, other.a) && Point.equals(this.b, other.b)) return 1;
  if(Point.equals(this.a, other.b) && Point.equals(this.b, other.a)) return -1;
  return false;
};

Line.prototype.indexOf = function (point) {
  let i = 0;

  for(let p of [this.a, this.b]) {
    if(Point.prototype.equals.call(p, point)) return i;
    i++;
  }

  return -1;
};

Line.prototype.lastIndexOf = function (point) {
  let i = 0;

  for(let p of [this.b, this.a]) {
    if(Point.prototype.equals.call(p, point)) return i;
    i++;
  }

  return -1;
};

Line.prototype.map = function (fn) {
  let i = 0;
  let r = [];

  for(let p of [this.a, this.b]) {
    r.push(fn(p, i, this));
    i++;
  }

  return new Line(...r);
};

Line.prototype.swap = function (fn) {
  return new Line(this.a, this.b).reverse();
};

Line.prototype.toPoints = function () {
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
  if(!gen) gen = (k) => (v) => (v === undefined ? o.k : (o.k = v));
  let proxy = { a: Point.bind(o, [x1, y1], gen), b: Point.bind(o, [x2, y2], gen) };
  return Object.setPrototypeOf(proxy, Line.prototype);
};

Util.defineGetter(Line, Symbol.species, function () {
  return this;
});
const ImmutableLine = Util.immutableClass(Line);

Util.defineGetter(ImmutableLine, Symbol.species, function () {
  return ImmutableLine;
});

/*
 * concatenanted lib/geom/lineList.js
 */

export class LineList extends Array {
  constructor(lines) {
    super();

    if(Util.isArray(lines) || Util.isGenerator(lines)) {
      for(let line of lines) {
        if(!(line instanceof Line)) line = Util.isArray(line) ? new Line(...line) : new Line(line);
        this.push(line);
      }
    }
  }

  bbox() {
    let bb = new BBox();
    for(let line of this) bb.update(Line.prototype.toObject.call(line));
    return bb;
  }
}

if(!Util.isBrowser()) {
  let c = Util.coloring();
  const sym = Symbol.for('nodejs.util.inspect.custom');

  LineList.prototype.sym = function () {
    return `${c.text('LineList', 1, 31)}${c.text('(', 1, 36)}${c.text(this.length, 1, 35) + c.code(1, 36)}) [\n  ${this.map((line) => line.sym()).join(',\n  ')}\n${c.text(']', 1, 36)}`;
  };
}

Util.defineGetter(LineList, Symbol.species, function () {
  return this;
});
const ImmutableLineList = Util.immutableClass(LineList);

Util.defineGetter(ImmutableLineList, Symbol.species, function () {
  return ImmutableLineList;
});

/*
 * concatenanted lib/geom/polygon.js
 */

const Polygon = function Polygon() {};

Polygon.area = (polygon) => {
  var area = 0;
  var j = polygon.length - 1;
  var p1;
  var p2;

  for(var k = 0; k < polygon.length; j = k++) {
    p1 = polygon.k;
    p2 = polygon.j;

    if(p1.x !== undefined && p2.x !== undefined) {
      area += p1.x * p2.y;
      area -= p1.y * p2.x;
    } else {
      area += p1[0] * p2[1];
      area -= p1[1] * p2[0];
    }
  }

  area = area / 2;
  return area;
};

Polygon.center = (polygon) => {
  var x = 0;
  var y = 0;
  var f;
  var j = polygon.length - 1;
  var p1;
  var p2;

  for(var k = 0; k < polygon.length; j = k++) {
    p1 = polygon.k;
    p2 = polygon.j;

    if(p1.x !== undefined && p2.x !== undefined) {
      f = p1.x * p2.y - p2.x * p1.y;
      x += (p1.x + p2.x) * f;
      y += (p1.y + p2.y) * f;
    } else {
      f = p1[0] * p2[1] - p2[0] * p1[1];
      x += (p1[0] + p2[0]) * f;
      y += (p1[1] + p2[1]) * f;
    }
  }

  f = area(polygon) * 6;
  return [x / f, y / f];
};

Polygon.approxCircle = (radius, npoints) => {
  var ret = [];

  for(var k = 0; k < npoints; k++) {
    var theta = (Math.PI * 2 * k) / npoints;
    var x = Math.sin(theta) * radius;
    var y = Math.cos(theta) * radius;
    ret.push({ x, y });
  }

  return ret;
};

Polygon.toPath = (polygon, relative = true) => {
  var prevx = 0;
  var prevy = 0;
  var path = '';

  for(var k = 0; k < polygon.length; k++) {
    let x = polygon.k.x !== undefined ? polygon.k.x : polygon.k[0];
    let y = polygon.k.y !== undefined ? polygon.k.y : polygon.k[1];

    if(relative) {
      x -= prevx;
      y -= prevy;
    }

    let cmd = k == 0 ? 'M' : 'L';
    if(relative) cmd = cmd.toLowerCase();
    path += `${cmd}${x},${y}`;
  }

  path += 'z';
  return path;
};

Polygon.fromLine = (arg, offset, steps = 3) => {
  let line = new Line(arg);
  const PI2 = Math.PI * 0.5;
  const step = Util.range(0, steps - 1).map((i) => (i * Math.PI) / (steps - 1));
  const a = line.angle();
  let vl = new PointList();
  vl = vl.concat(step.map((va) => Point.fromAngle(a - PI2 - va, offset).sum(line.a)));
  vl = vl.concat(step.map((va) => Point.fromAngle(a + PI2 - va, offset).sum(line.b)));
  return vl;
};
export default Polygon;

/*
 * concatenanted lib/geom/matrix.js
 */

export function Matrix(arg) {
  let ret = this instanceof Matrix || new.target === Matrix ? this : [undefined, 0, 0, undefined, 0, 0, undefined, 0, 0];
  const isObj = Util.isObject(arg);

  if(isObj && arg.xx !== undefined && arg.yx !== undefined && arg.xy !== undefined && arg.yy !== undefined && arg.x0 !== undefined && arg.y0 !== undefined) {
    ret[0] = arg.xx;
    ret[1] = arg.xy;
    ret[2] = arg.x0;
    ret[3] = arg.yx;
    ret[4] = arg.yy;
    ret[5] = arg.y0;
  } else if(isObj && arg.a !== undefined && arg.b !== undefined && arg.c !== undefined && arg.d !== undefined && arg.e !== undefined && arg.f !== undefined) {
    ret[0] = arg.a;
    ret[1] = arg.c;
    ret[2] = arg.e;
    ret[3] = arg.b;
    ret[4] = arg.d;
    ret[5] = arg.f;
  } else if(arg instanceof Array) {
    Matrix.prototype.init.call(ret, arg);
  } else if(typeof arg === 'number') {
    Matrix.prototype.init.apply(ret, arguments);
  } else if(typeof arg === 'string' && /matrix\([^)]*\)/.test(arg)) {
    let [xx, xy, x0, yx, yy, y0] = [...arg.matchAll(/[-.0-9]+/g)].map((m) => parseFloat(m[0]));
    ret[0] = xx;
    ret[1] = xy;
    ret[2] = x0;
    ret[3] = yx;
    ret[4] = yy;
    ret[5] = y0;
  } else {
    Array.prototype.splice.call(ret, 0, ret.length, 1, 0, 0, 0, 1, 0, 0, 0, 1);
  }

  for(let i = 0; i < 9; i++) if(ret.i === undefined) ret.i = [1, 0, 0, 0, 1, 0, 0, 0, 1].i;

  if(!(this instanceof Matrix)) return ret;
}

Object.defineProperty(Matrix, Symbol.species, {
  get() {
    return Matrix;
  }
});

Matrix.prototype[Symbol.toStringTag] = function () {
  return Matrix.prototype.toString.apply(this, arguments);
};
Matrix.prototype[Symbol.isConcatSpreadable] = false;
Object.defineProperty(Matrix.prototype, 'length', { value: 6, enumerable: false, writable: true, configurable: false });
Matrix.prototype.keys = ['xx', 'xy', 'x0', 'yx', 'yy', 'y0'];
Matrix.prototype.keySeq = ['xx', 'yx', 'xy', 'yy', 'x0', 'y0'];
const keyIndexes = { xx: 0, a: 0, xy: 1, c: 1, x0: 2, tx: 2, e: 2, yx: 3, b: 3, yy: 4, d: 4, y0: 5, ty: 5, f: 5 };

Matrix.prototype.at = function (col, row = 0) {
  return this[row * 3 + col];
};

Matrix.prototype.get = function (field) {
  if(typeof field == 'number' && field < this.length) return this.field;
  if((field = keyIndexes.field)) return this.field;
};

const MatrixProps = (obj = {}) =>
  Object.entries(keyIndexes).reduce(
    (acc, [k, i]) => ({
      ...acc,
      [[k]]: {
        get() {
          return this.i;
        },
        set(v) {
          this.i = v;
        },
        enumerable: true
      }
    }),
    obj
  );
Object.defineProperties(Matrix.prototype, MatrixProps());
Matrix.propDescriptors = MatrixProps;

Matrix.prototype.init = function (...args) {
  if(args.length == 1) args = args[0];
  if(args.length < 9) args = args.concat(Array.prototype.slice.call(Matrix.IDENTITY, args.length));
  Array.prototype.splice.call(this, 0, this.length, ...args);
  return this;
};

Matrix.prototype.set_row = function (...args) {
  const start = args.shift() * 3;
  const end = Math.max(3, args.length);

  for(let i = 0; i < end; i++) this[start + i] = args.i;

  return this;
};

Matrix.prototype.multiply = function (...args) {
  return this.clone().multiplySelf(...args);
};

Matrix.prototype.multiplySelf = function (...args) {
  for(let arg of args) {
    if(!(arg instanceof Matrix)) throw new Error('Not a Matrix: ' + arg.constructor);

    this.init([this[0] * arg[0] + this[1] * arg[3], this[0] * arg[1] + this[1] * arg[4], this[0] * arg[2] + this[1] * arg[5] + this[2], this[3] * arg[0] + this[4] * arg[3], this[3] * arg[1] + this[4] * arg[4], this[3] * arg[2] + this[4] * arg[5] + this[5]]);
  }

  return this;
};

Matrix.prototype.multiply_self = function (...args) {
  for(let m of args) {
    if(!(m instanceof Matrix)) m = new Matrix(m);
    Matrix.prototype.init.call(this, this[0] * m[0] + this[1] * m[3], this[0] * m[1] + this[1] * m[4], this[0] * m[2] + this[1] * m[5] + this[2], this[3] * m[0] + this[4] * m[3], this[3] * m[1] + this[4] * m[4], this[3] * m[2] + this[4] * m[5] + this[5]);
  }

  return this;
};

Matrix.prototype.toObject = function () {
  const { xx, xy, x0, yx, yy, y0 } = this;
  return { xx, xy, x0, yx, yy, y0 };
};

Matrix.prototype.entries = function () {
  return Object.entries(Matrix.prototype.toObject.call(this));
};

Matrix.prototype.clone = function () {
  const ctor = this.constructor[Symbol.species];
  return new ctor(Array.from(this));
};

Matrix.prototype.row = function (row) {
  let i = row * 3;
  return Array.prototype.slice.call(this, i, i + 3);
};

Matrix.prototype.rows = function () {
  let ret = [];

  for(let i = 0; i < 9; i += 3) ret.push([this[i + 0], this[i + 1], this[i + 2]]);

  return ret;
};

Matrix.prototype.toArray = function () {
  return Array.from(this);
};

Matrix.prototype.isIdentity = function () {
  return Util.equals(this, Matrix.IDENTITY);
};

Matrix.prototype.determinant = function () {
  return this[0] * (this[4] * this[8] - this[5] * this[7]) + this[1] * (this[5] * this[6] - this[3] * this[8]) + this[2] * (this[3] * this[7] - this[4] * this[6]);
};

Matrix.prototype.invert = function () {
  const det = Matrix.prototype.determinant.call(this);
  return new Matrix([
    (this[4] * this[8] - this[5] * this[7]) / det,
    (this[2] * this[7] - this[1] * this[8]) / det,
    (this[1] * this[5] - this[2] * this[4]) / det,
    (this[5] * this[6] - this[3] * this[8]) / det,
    (this[0] * this[8] - this[2] * this[6]) / det,
    (this[2] * this[3] - this[0] * this[5]) / det,
    (this[3] * this[7] - this[4] * this[6]) / det,
    (this[6] * this[1] - this[0] * this[7]) / det,
    (this[0] * this[4] - this[1] * this[3]) / det
  ]);
};

Matrix.prototype.scalar_product = function (f) {
  return new Matrix({ xx: this[0] * f, xy: this[1] * f, x0: this[2] * f, yx: this[3] * f, yy: this[4] * f, y0: this[5] * f });
};

Matrix.prototype.toSource = function (construct = (false, (multiline = true))) {
  const nl = multiline ? '\n' : '';
  const rows = Matrix.prototype.rows.call(this);
  const src = `${rows.map((row) => row.join(',')).join(multiline ? ',\n ' : ',')}`;
  return construct ? `new Matrix([${nl}${src}${nl}])` : `[${src}]`;
};

Matrix.prototype.toString = function (separator = ' ') {
  let rows = Matrix.prototype.rows.call(this);
  let name = rows[0].length == 3 ? 'matrix' : 'matrix3d';

  if(rows[0].length == 3) {
    rows = [['a', 'b', 'c', 'd', 'e', 'f'].map((k) => this[keyIndexes.k])];
  }

  return `${name}(` + rows.map((row) => row.join(',' + separator)).join(',' + separator) + ')';
};

Matrix.prototype.toSVG = function () {
  return 'matrix(' + ['a', 'b', 'c', 'd', 'e', 'f'].map((k) => this[keyIndexes.k]).join(',') + ')';
};

Matrix.prototype.toDOM = function (ctor = DOMMatrix) {
  const rows = Matrix.prototype.rows.call(this);
  const [a, c, e] = rows[0];
  const [b, d, f] = rows[1];
  return new ctor([a, b, c, d, e, f]);
};

Matrix.prototype.toJSON = function () {
  const rows = Matrix.prototype.rows.call(this);
  const [a, c, e] = rows[0];
  const [b, d, f] = rows[1];
  return { a, b, c, d, e, f };
};

Matrix.fromJSON = (obj) => {
  return new Matrix(obj);
};

Matrix.fromDOM = (matrix) => {
  const { a, b, c, d, e, f } = matrix;
  return new Matrix([a, c, e, b, d, f]);
};

Matrix.prototype.equals = function (other) {
  return Array.prototype.every.call((n, i) => other.i == n);
};

Matrix.prototype.transform_distance = function (d) {
  const k = 'x' in d && 'y' in d ? ['x', 'y'] : 'width' in d && 'height' in d ? ['width', 'height'] : [0, 1];
  const x = this[0] * d[k[0]] + this[2] * d[k[1]];
  const y = this[1] * d[k[0]] + this[3] * d[k[1]];
  d[k[0]] = x;
  d[k[1]] = y;
  return d;
};

Matrix.prototype.transform_xy = function (x, y) {
  const m0 = this.row(0);
  const m1 = this.row(1);
  return [m0[0] * x + m0[1] * y + m0[2], m1[0] * x + m1[1] * y + m0[2]];
};

Matrix.prototype.transform_point = function (p) {
  const k = 'x' in p && 'y' in p ? ['x', 'y'] : [0, 1];
  const m0 = this.row(0);
  const m1 = this.row(1);
  const x = m0[0] * p[k[0]] + m0[1] * p[k[1]] + m0[2];
  const y = m1[0] * p[k[0]] + m1[1] * p[k[1]] + m1[2];
  p[k[0]] = x;
  p[k[1]] = y;
  return p;
};

Matrix.prototype.transformGenerator = function (what = 'point') {
  const matrix = Object.freeze(this.clone());

  return function* (list) {
    const method = Matrix.prototype['transform_' + what] || (typeof what == 'function' && what) || Matrix.prototype.transform_xy;
    for(let item of list) yield item instanceof Array ? method.apply(matrix, [...item]) : method.call(matrix, { ...item });
  };
};

Matrix.prototype.transform_points = function* (list) {
  for(let i = 0; i < list.length; i++) yield Matrix.prototype.transform_point.call(this, { ...list.i });
};

Matrix.prototype.transform_wh = function (width, height) {
  const w = this[0] * width + this[1] * height;
  const h = this[3] * width + this[4] * height;
  return [w, h];
};

Matrix.prototype.transform_size = function (s) {
  const w = this[0] * s.width + this[1] * s.height;
  const h = this[3] * s.width + this[4] * s.height;
  s.width = w;
  s.height = h;
  return s;
};

Matrix.prototype.transform_xywh = function (x, y, width, height) {
  return [...Matrix.prototype.transform_xy.call(this, x, y), ...Matrix.prototype.transform_wh.call(this, width, height)];
};

Matrix.prototype.transform_rect = function (rect) {
  let { x1, y1, x2, y2 } = rect;
  [x1, y1] = Matrix.prototype.transform_xy.call(this, x1, y1);
  [x2, y2] = Matrix.prototype.transform_xy.call(this, x2, y2);
  let xrange = [x1, x2];
  let yrange = [y1, y2];
  [x1, x2] = [Math.min, Math.max].map((fn) => fn(...xrange));
  [y1, y2] = [Math.min, Math.max].map((fn) => fn(...yrange));
  Object.assign(rect, { x1, x2, y1, y2 });
  return rect;
};

Matrix.prototype.point_transformer = function () {
  const matrix = this;
  return (point) => matrix.transform_point(point);
};

Matrix.prototype.transformer = function () {
  const matrix = this;
  return { point: (point) => matrix.transform_point(point), xy: (x, y) => matrix.transform_xy(x, y), size: (s) => matrix.transform_size(s), wh: (w, h) => matrix.transform_wh(w, h), rect: (rect) => matrix.transform_rect(rect), points: (list) => matrix.transform_points(list), distance: (d) => matrix.transform_distance(d) };
};

Matrix.prototype.scale_sign = function () {
  return this[0] * this[4] < 0 || this[1] * this[3] > 0 ? -1 : 1;
};

Matrix.prototype.affine_transform = function (a, b) {
  var xx, yx, xy, yy, tx, ty;
  if(typeof a == 'object' && a.toPoints !== undefined) a = a.toPoints();
  if(typeof b == 'object' && b.toPoints !== undefined) b = b.toPoints();
  xx = (b[0].x * a[1].y + b[1].x * a[2].y + b[2].x * a[0].y - b[0].x * a[2].y - b[1].x * a[0].y - b[2].x * a[1].y) / (a[0].x * a[1].y + a[1].x * a[2].y + a[2].x * a[0].y - a[0].x * a[2].y - a[1].x * a[0].y - a[2].x * a[1].y);
  yx = (b[0].y * a[1].y + b[1].y * a[2].y + b[2].y * a[0].y - b[0].y * a[2].y - b[1].y * a[0].y - b[2].y * a[1].y) / (a[0].x * a[1].y + a[1].x * a[2].y + a[2].x * a[0].y - a[0].x * a[2].y - a[1].x * a[0].y - a[2].x * a[1].y);
  xy = (a[0].x * b[1].x + a[1].x * b[2].x + a[2].x * b[0].x - a[0].x * b[2].x - a[1].x * b[0].x - a[2].x * b[1].x) / (a[0].x * a[1].y + a[1].x * a[2].y + a[2].x * a[0].y - a[0].x * a[2].y - a[1].x * a[0].y - a[2].x * a[1].y);
  yy = (a[0].x * b[1].y + a[1].x * b[2].y + a[2].x * b[0].y - a[0].x * b[2].y - a[1].x * b[0].y - a[2].x * b[1].y) / (a[0].x * a[1].y + a[1].x * a[2].y + a[2].x * a[0].y - a[0].x * a[2].y - a[1].x * a[0].y - a[2].x * a[1].y);
  tx = (a[0].x * a[1].y * b[2].x + a[1].x * a[2].y * b[0].x + a[2].x * a[0].y * b[1].x - a[0].x * a[2].y * b[1].x - a[1].x * a[0].y * b[2].x - a[2].x * a[1].y * b[0].x) / (a[0].x * a[1].y + a[1].x * a[2].y + a[2].x * a[0].y - a[0].x * a[2].y - a[1].x * a[0].y - a[2].x * a[1].y);
  ty = (a[0].x * a[1].y * b[2].y + a[1].x * a[2].y * b[0].y + a[2].x * a[0].y * b[1].y - a[0].x * a[2].y * b[1].y - a[1].x * a[0].y * b[2].y - a[2].x * a[1].y * b[0].y) / (a[0].x * a[1].y + a[1].x * a[2].y + a[2].x * a[0].y - a[0].x * a[2].y - a[1].x * a[0].y - a[2].x * a[1].y);
  this.set_row.call(this, 0, xx, xy, tx);
  this.set_row.call(this, 1, yx, yy, ty);
  this.set_row.call(this, 2, 0, 0, 1);
  return this;
};

Matrix.getAffineTransform = (a, b) => {
  var matrix = new Matrix();
  matrix.affine_transform(a, b);
  return matrix;
};

Matrix.prototype.decompose = function (degrees = (false, (useLU = true))) {
  var a = this[0],
    b = this[3],
    c = this[1],
    d = this[4];
  var translate = { x: this[2], y: this[5] },
    rotation = 0,
    scale = { x: 1, y: 1 },
    skew = { x: 0, y: 0 };
  var determ = a * d - b * c,
    r,
    s;

  const calcFromValues = (r1, m1, r2, m2) => {
    if(!isFinite(r1)) return r2;
    else if(!isFinite(r2)) return r1;
    (m1 = Math.abs(m1)), (m2 = Math.abs(m2));
    return Util.roundTo((m1 * r1 + m2 * r2) / (m1 + m2), 0.0001);
  };

  let sign = Matrix.prototype.scale_sign.call(this);
  rotation = (Math.atan2(this[3], this[4]) + Math.atan2(-sign * this[1], sign * this[0])) / 2;
  const cos = Math.cos(rotation),
    sin = Math.sin(rotation);
  scale = { x: calcFromValues(this[0] / cos, cos, -this[1] / sin, sin), y: calcFromValues(this[4] / cos, cos, this[3] / sin, sin) };
  return { translate, rotate: degrees === true ? Util.roundTo(Matrix.rad2deg(rotation), 0.1) : rotation, scale, skew };
};

Matrix.prototype.init_identity = function () {
  return Matrix.prototype.init.call(this, 1, 0, 0, 0, 1, 0, 0, 0, 1);
};

Matrix.prototype.is_identity = function () {
  return Matrix.prototype.equals.call(this, [1, 0, 0, 0, 1, 0, 0, 0, 1]);
};

Matrix.prototype.init_translate = function (tx, ty) {
  return Matrix.prototype.init.call(this, 1, 0, tx, 0, 1, ty);
};

Matrix.prototype.init_scale = function (sx, sy) {
  if(sy === undefined) sy = sx;
  return Matrix.prototype.init.call(this, sx, 0, 0, 0, sy, 0);
};

Matrix.prototype.init_rotate = function (angle, deg = false) {
  const rad = deg ? Matrix.deg2rad(angle) : angle;
  const s = Math.sin(rad);
  const c = Math.cos(rad);
  return Matrix.prototype.init.call(this, c, -s, 0, s, c, 0);
};

Matrix.prototype.init_skew = function (x, y, deg = false) {
  const ax = Math.tan(deg ? Matrix.deg2rad(x) : x);
  const ay = Math.tan(deg ? Matrix.deg2rad(y) : y);
  return Matrix.prototype.init.call(this, 1, ax, 0, ay, 1, 0);
};
Matrix.identity = () => new Matrix([1, 0, 0, 0, 1, 0, 0, 0, 1]);
Matrix.IDENTITY = Object.freeze(Matrix.identity());
Matrix.rad2deg = (radians) => (radians * 180) / Math.PI;
Matrix.deg2rad = (degrees) => (degrees * Math.PI) / 180;

for(let name of ['toObject', 'init', 'toArray', 'isIdentity', 'determinant', 'invert', 'multiply', 'scalar_product', 'toSource', 'toString', 'toSVG', 'equals', 'init_identity', 'is_identity', 'init_translate', 'init_scale', 'init_rotate', 'scale_sign', 'decompose', 'transformer']) {
  Matrix.name = (matrix, ...args) => Matrix.prototype.name.call(matrix || new Matrix(matrix), ...args);
}

for(let name of ['translate', 'scale', 'rotate', 'skew']) {
  Matrix.name = (...args) => Matrix.prototype['init_' + name].call(new Matrix(), ...args);
}

for(let name of ['translate', 'scale', 'rotate', 'skew']) {
  Matrix.prototype.name = function (...args) {
    return Matrix.prototype.multiply.call(this, new Matrix()['init_' + name](...args));
  };

  Matrix.prototype[name + '_self'] = function (...args) {
    return Matrix.prototype.multiply_self.call(this, new Matrix()['init_' + name](...args));
  };
}

for(let name of ['transform_distance', 'transform_xy', 'transform_point', 'transform_points', 'transform_wh', 'transform_size', 'transform_rect', 'affine_transform']) {
  const method = Matrix.prototype.name;

  if(method.length == 2) {
    Matrix.name = Util.curry((m, a, b) => Matrix.prototype.name.call(m || new Matrix(m), a, b));
  } else if(method.length == 1) {
    Matrix.name = Util.curry((m, a) => Matrix.prototype.name.call(m || new Matrix(m), a));
  }
}

Util.defineGetter(Matrix, Symbol.species, function () {
  return this;
});
const isMatrix = (m) => Util.isObject(m) && (m instanceof Matrix || (m.length !== undefined && (m.length == 6 || m.length == 9) && m.every((el) => typeof el == 'number')));
const ImmutableMatrix = Util.immutableClass(Matrix);

Util.defineGetter(ImmutableMatrix, Symbol.species, function () {
  return ImmutableMatrix;
});

/*
 * concatenanted lib/geom/point.js
 */

const SymSpecies = Util.tryCatch(
  () => Symbol,
  (sym) => sym.species
);

const CTOR = (obj) => {
  if(obj.SymSpecies) return obj.SymSpecies;
  let p = Object.getPrototypeOf(obj);
  if(p.SymSpecies) return p.SymSpecies;
  return p.constructor;
};

export function Point(arg) {
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
    const matches = [...arg.matchAll(/([-+]?d*.?d+)(?:[eE]([-+]?d+))?/g)];
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

Point.prototype.move = function (x, y) {
  this.x += x;
  this.y += y;
  return this;
};

Point.prototype.moveTo = function (x, y) {
  this.x = x;
  this.y = y;
  return this;
};

Point.prototype.clear = function (x, y) {
  this.x = 0;
  this.y = 0;
  return this;
};

Point.prototype.set = function (fn) {
  if(typeof fn != 'function') {
    Point.apply(this, [...arguments]);
    return this;
  }

  return fn(this.x, this.y);
};

Point.prototype.clone = function () {
  const ctor = this[Symbol.species] || this.constructor[Symbol.species];
  return new ctor({ x: this.x, y: this.y });
};

Point.prototype.sum = function (...args) {
  const p = new Point(...args);
  let r = new Point(this.x, this.y);
  r.x += p.x;
  r.y += p.y;
  return r;
};

Point.prototype.add = function (...args) {
  const other = new Point(...args);
  this.x += other.x;
  this.y += other.y;
  return this;
};

Point.prototype.diff = function (arg) {
  let { x, y } = this;

  var fn = function (other) {
    let r = new Point(x, y);
    return r.sub(other);
  };

  if(arg) return fn(arg);
  return fn;
};

Point.prototype.sub = function (...args) {
  const other = new Point(...args);
  this.x -= other.x;
  this.y -= other.y;
  return this;
};

Point.prototype.prod = function (f) {
  const o = isPoint(f) ? f : { x: f, y: f };
  return new Point(this.x * o.x, this.y * o.y);
};

Point.prototype.mul = function (f) {
  const o = isPoint(f) ? f : { x: f, y: f };
  this.x *= o.x;
  this.y *= o.y;
  return this;
};

Point.prototype.quot = function (other) {
  other = isPoint(other) ? other : { x: other, y: other };
  return new Point(this.x / other.x, this.y / other.y);
};

Point.prototype.div = function (other) {
  other = isPoint(other) ? other : { x: other, y: other };
  this.x /= other.x;
  this.y /= other.y;
  return this;
};

Point.prototype.comp = function () {
  return new Point({ x: -this.x, y: -this.y });
};

Point.prototype.neg = function () {
  this.x *= -1;
  this.y *= -1;
  return this;
};

Point.prototype.distanceSquared = function (other = { x: 0, y: 0 }) {
  return (other.y - this.y) * (other.y - this.y) + (other.x - this.x) * (other.x - this.x);
};

Point.prototype.distance = function (other = { x: 0, y: 0 }) {
  return Math.sqrt(Point.prototype.distanceSquared.call(this, other));
};

Point.prototype.equals = function (other) {
  return +this.x == +other.x && +this.y == +other.y;
};

Point.prototype.round = function (precision = (0.001, digits, (type = 'round'))) {
  let { x, y } = this;
  this.x = Util.roundTo(x, precision, digits, type);
  this.y = Util.roundTo(y, precision, digits, type);
  return this;
};

Point.prototype.ceil = function () {
  let { x, y } = this;
  this.x = Math.ceil(x);
  this.y = Math.ceil(y);
  return this;
};

Point.prototype.floor = function () {
  let { x, y } = this;
  this.x = Math.floor(x);
  this.y = Math.floor(y);
  return this;
};

Point.prototype.dot = function (other) {
  return this.x * other.x + this.y * other.y;
};

Point.prototype.values = function () {
  return [this.x, this.y];
};

Point.prototype.fromAngle = function (angle, dist = 1.0) {
  this.x = Math.cos(angle) * dist;
  this.y = Math.sin(angle) * dist;
  return this;
};

Point.prototype.toAngle = function (deg = false) {
  return Math.atan2(this.x, this.y) * (deg ? 180 / Math.PI : 1);
};

Point.prototype.angle = function (other, deg = false) {
  other = other || { x: 0, y: 0 };
  return Point.prototype.diff.call(this, other).toAngle(deg);
};

Point.prototype.rotate = function (angle, origin = { x: 0, y: 0 }) {
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

Util.defineGetter(Point.prototype, Symbol.iterator, function () {
  const { x, y } = this;
  let a = [x, y];
  return a[Symbol.iterator].bind(a);
});

Point.prototype.toString = function (opts = {}) {
  const { precision, unit, separator, left, right } = opts;
  const x = Util.roundTo(this.x, precision);
  const y = Util.roundTo(this.y, precision);
  return `${left}${x}${unit}${separator}${y}${unit}${right}`;
};

Util.defineGetterSetter(
  Point.prototype,
  Symbol.toStringTag,
  function () {
    return `Point{ ${Point.prototype.toSource.call(this)}`;
  },
  () => {},
  false
);

Point.prototype.toSource = function (opts = {}) {
  const { asArray, plainObj, pad, showNew } = opts;
  let x = pad(this.x + '');
  let y = pad(this.y + '');
  let c = (t) => t;
  if(typeof this != 'object' || this === null) return '';
  if(asArray) return `[${x},${y}]`;
  if(plainObj) return `{x:${x},y:${y}}`;
  return `${c(showNew ? 'new ' : '', 1, 31)}${c('Point', 1, 33)}${c('(', 1, 36)}${c(x, 1, 32)}${c(',', 1, 36)}${c(y, 1, 32)}${c(')', 1, 36)}`;
};

Point.prototype.toObject = function (proto = Point.prototype) {
  const { x, y } = this;
  const obj = { x, y };
  Object.setPrototypeOf(obj, proto);
  return obj;
};

Point.prototype.toCSS = function (precision = (0.001, (edges = ['left', 'top']))) {
  return { [[edges[0]]]: Util.roundTo(this.x, precision) + 'px', [[edges[1]]]: Util.roundTo(this.y, precision) + 'px' };
};

Point.prototype.toFixed = function (digits) {
  return new Point(+this.x.toFixed(digits), +this.y.toFixed(digits));
};

Point.prototype.isNull = function () {
  return this.x == 0 && this.y == 0;
};

Point.prototype.inside = function (rect) {
  return this.x >= rect.x && this.x < rect.x + rect.width && this.y >= rect.y && this.y < rect.y + rect.height;
};

Point.prototype.transform = function (m) {
  if(Util.isObject(m) && typeof m.toMatrix == 'function') m = m.toMatrix();
  if(Util.isObject(m) && typeof m.transform_point == 'function') return m.transform_point(this);
  const x = m[0] * this.x + m[1] * this.y + m[2];
  const y = m[3] * this.x + m[4] * this.y + m[5];
  this.x = x;
  this.y = y;
  return this;
};

Point.prototype.scaleTo = function (minmax) {
  return new Point({ x: (this.x - minmax.x1) / (minmax.x2 - minmax.x1), y: (this.y - minmax.y1) / (minmax.y2 - minmax.y1) });
};

Point.prototype.normal = function () {
  let d = Point.prototype.distance.call(this);
  return new Point({ x: this.x / d, y: this.y / d });
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
Point.toSource = (point, { space = ' ', padding = ' ', separator = ',' }) => `{${padding}x:${space}${point.x}${separator}y:${space}${point.y}${padding}}`;
const isPoint = (o) => o && ((o.x !== undefined && o.y !== undefined) || ((o.left !== undefined || o.right !== undefined) && (o.top !== undefined || o.bottom !== undefined)) || o instanceof Point || Object.getPrototypeOf(o).constructor === Point);
Point.isPoint = isPoint;
Util.defineInspect(Point.prototype, 'x', 'y');

Point.bind = (o, p, gen) => {
  const [x, y] = p || ['x', 'y'];
  if(!gen) gen = (k) => (v) => (v === undefined ? o.k : (o.k = v));
  return Util.bindProperties(new Point(0, 0), o, { x, y }, gen);
};
export default Point;

Util.defineGetter(Point, Symbol.species, function () {
  return this;
});
const ImmutablePoint = Util.immutableClass(Point);

Util.defineGetter(ImmutablePoint, Symbol.species, function () {
  return ImmutablePoint;
});

/*
 * concatenanted lib/geom/circle.js
 */

export function Circle(x, y, radius) {
  let obj = this || null;
  let arg;
  let args = [...arguments];
  let ret;

  if(args.length >= 3 && args.every((arg) => !isNaN(parseFloat(arg)))) {
    arg = { x: +args[0], y: +args[1], radius: +args[2] };
  } else if(args.length == 1) {
    arg = args[0];
    obj.x = +arg.x;
    obj.y = +arg.y;
    obj.radius = +arg.radius;
  }

  if(obj === null) obj = Object.create(Circle.prototype);
  if(Object.getPrototypeOf(obj) !== Circle.prototype) Object.setPrototypeOf(obj, Circle.prototype);

  if(arg && arg.x !== undefined && arg.y !== undefined && arg.radius !== undefined) {
    obj.x = +arg.x;
    obj.y = +arg.y;
    obj.radius = +arg.radius;
    ret = 1;
  } else if(isPoint(args[0]) && typeof args[1] == 'number') {
    obj.x = +args[0].x;
    obj.y = +args[0].y;
    obj.radius = +args[1];
    ret = 2;
  } else if(arg && arg.length >= 3 && arg.slice(0, 3).every((arg) => !isNaN(parseFloat(arg)))) {
    obj.x = +arg[0];
    obj.y = +arg[1];
    obj.radius + arg[2];
    ret = 3;
  } else {
    obj.x = 0;
    obj.y = 0;
    obj.radius = 0;
    ret = 0;
  }

  if(!isCircle(obj)) {
  }
  return obj;
}
const isCircle = (obj) => ['x', 'y', 'radius'].every((prop) => obj.prop !== undefined);
Object.defineProperty(Circle.prototype, 'x', { value: 0, enumerable: true, writable: true });
Object.defineProperty(Circle.prototype, 'y', { value: 0, enumerable: true, writable: true });
Object.defineProperty(Circle.prototype, 'radius', { value: 0, enumerable: true, writable: true });

Object.defineProperty(Circle.prototype, 'center', {
  get() {
    return Point.bind(this, null, (value) => {
      if(value === undefined) return new Point(this.x, this.y);
      this.x = value.x;
      this.y = value.y;
    });
  }
});

Circle.prototype.bbox = function (width = 0) {
  const { x, y, radius } = this;
  let distance = radius + width;
  return new Rect({ x1: x - distance, x2: x + distance, y1: y - distance, y2: y + distance });
};

Circle.prototype.transform = function (m) {
  if(Util.isObject(m) && typeof m.toMatrix == 'function') m = m.toMatrix();
  Matrix.prototype.transform_point.call(m, this);
  this.radius = Matrix.prototype.transform_wh.call(m, this.radius, this.radius)[0];
  return this;
};
Util.defineInspect(Circle.prototype, 'x', 'y', 'radius');

Circle.bind = (o, p, gen) => {
  const [x, y, radius] = p || ['x', 'y', 'radius'];
  if(!gen) gen = (k) => (v) => (v === undefined ? o.k : (o.k = v));
  return Util.bindProperties(new Circle(0, 0, 0), o, { x, y, radius }, gen);
};

/*
 * concatenanted lib/geom/pointList.js
 */

export class PointList extends Array {
  constructor(points, tfn = (...args) => new Point(...args)) {
    super();
    const base = Array;
    let args = [...arguments];
    let ret = this instanceof PointList ? this : [];
    if(Util.isArray(args[0]) || Util.isGenerator(args[0])) args = [...args[0]];

    if(typeof points === 'string') {
      const matches = [...points.matchAll(/[-.0-9,]+/g)];

      for(let i = 0; i < matches.length; i++) {
        const coords = (matches.i[0] + '').split(/,/g).map((n) => +n);
        ret.push(tfn(...coords));
      }
    } else if(args[0] && args[0].length == 2) {
      for(let i = 0; i < args.length; i++) ret.push(this instanceof PointList ? new Point(args.i) : Point(args.i));
    } else if(args.length !== undefined) {
      for(let i = 0; i < args.length; i++) {
        ret.push(args.i instanceof Point ? args.i : tfn(args.i));
      }
    }

    let proto = PointList.prototype;
    Object.setPrototypeOf(ret, proto);
    if(!(this instanceof PointList)) return ret;
  }
}

PointList.prototype[Symbol.toStringTag] = function () {
  return PointList.prototype.toString.apply(this, arguments);
};
Util.defineGetter(PointList, Symbol.species, () => PointList);
PointList.prototype[Symbol.isConcatSpreadable] = true;

PointList.prototype.rotateRight = function (n) {
  this.unshift(...this.splice(n % this.length, this.length));
  return this;
};

PointList.prototype.rotateLeft = function (n) {
  return this.rotateRight(this.length - (n % this.length));
};

PointList.prototype.rotate = function (n) {
  if(n < 0) return this.rotateLeft(-n);
  if(n > 0) return this.rotateRight(n);
  return this;
};

PointList.prototype.push = function (...args) {
  while(args.length > 0) Array.prototype.push.call(this, new Point(args));
  return this;
};

PointList.prototype.unshift = function (...args) {
  let points = [];
  while(args.length > 0) points.push(new Point(args));
  Array.prototype.splice.call(this, 0, 0, ...points);
  return this;
};
PointList.prototype.length = 0;

PointList.prototype.getLength = function () {
  return this.length;
};

Object.defineProperty(PointList.prototype, 'size', {
  get() {
    return PointList.prototype.getLength.call(this);
  }
});

PointList.prototype.at = function (index) {
  return this[+index];
};

PointList.prototype.splice = function () {
  let args = [...arguments];
  const start = args.shift();
  const remove = args.shift();
  return Array.prototype.splice.apply(this, [start, remove, ...args.map((arg) => (arg instanceof Point ? arg : new Point(arg)))]);
};
PointList.prototype.slice = Array.prototype.slice;

PointList.prototype.removeSegment = function (index) {
  let indexes = [PointList.prototype.getLineIndex.call(this, index - 1), PointList.prototype.getLineIndex.call(this, index), PointList.prototype.getLineIndex.call(this, index + 1)];
  let lines = indexes.map((i) => PointList.prototype.getLine.call(this, i));
  let point = Line.intersect(lines[0], lines[2]);

  if(point) {
    PointList.prototype.splice.call(this, 0, 2, new Point(point));
  }
};

PointList.prototype.clone = function () {
  const ctor = this.constructor[Symbol.species];
  let points = PointList.prototype.map.call(this, (p) => Point.prototype.clone.call(p));
  return new ctor(points);
};

PointList.prototype.toPolar = function (tfn) {
  let ret = new PointList();
  let t = typeof tfn == 'function' ? tfn : (x, y) => ({ x, y });

  ret.splice.apply(ret, [
    0,
    ret.length,
    ...PointList.prototype.map.call(this, (p) => {
      const angle = Point.prototype.toAngle.call(p);
      return t(angle, Point.prototype.distance.call(p));
    })
  ]);

  return ret;
};

PointList.prototype.fromPolar = function (tfn) {
  let ret = new PointList();
  let t = typeof tfn == 'function' ? tfn : (x, y) => ({ x, y });

  ret.splice.apply(ret, [
    0,
    ret.length,
    ...PointList.prototype.map.call(this, (p) => {
      let r = t(p.x, p.y);
      return Point.prototype.fromAngle.call(r.x, r.y);
    })
  ]);

  return ret;
};

PointList.prototype.draw = function (ctx, close = false) {
  const first = PointList.prototype.at.call(this, 0);
  const len = PointList.prototype.getLength.call(this);
  ctx.to(first.x, first.y);

  for(let i = 1; i < len; i++) {
    const { x, y } = PointList.prototype.at.call(this, i);
    ctx.line(x, y);
  }

  if(close) ctx.line(first.x, first.y);
  return this;
};

PointList.prototype.area = function () {
  var area = 0;
  var i;
  var j;
  var point1;
  var point2;
  const len = PointList.prototype.getLength.call(this);

  for(i = (0, (j = len - 1)); i < len; j = (i, (i += 1))) {
    point1 = PointList.prototype.at.call(this, i);
    point2 = PointList.prototype.at.call(this, j);
    area += point1.x * point2.y;
    area -= point1.y * point2.x;
  }

  area /= 2;
  return area;
};

PointList.prototype.centroid = function () {
  var x = 0;
  var y = 0;
  var i;
  var j;
  var f;
  var point1;
  var point2;
  const len = PointList.prototype.getLength.call(this);

  for(i = (0, (j = len - 1)); i < len; j = (i, (i += 1))) {
    point1 = PointList.prototype.at.call(this, i);
    point2 = PointList.prototype.at.call(this, j);
    f = point1.x * point2.y - point2.x * point1.y;
    x += (point1.x + point2.x) * f;
    y += (point1.y + point2.y) * f;
  }

  f = PointList.prototype.area.call(this) * 6;
  return new Point(x / f, y / f);
};

PointList.prototype.avg = function () {
  var ret = PointList.prototype.reduce.call(this, (acc, p) => acc.add(p), new Point());
  return ret.div(PointList.prototype.getLength.call(this));
};

PointList.prototype.bbox = function () {
  const len = PointList.prototype.getLength.call(this);
  if(!len) return {};
  const first = PointList.prototype.at.call(this, 0);

  var ret = {
    x1: first.x,
    x2: first.x,
    y1: first.y,
    y2: first.y,
    toString() {
      return `{x1:${(this.x1 + '').padStart(4, ' ')},x2:${(this.x2 + '').padStart(4, ' ')},y1:${(this.y1 + '').padStart(4, ' ')},y2:${(this.y2 + '').padStart(4, ' ')}}`;
    }
  };

  for(let i = 1; i < len; i++) {
    const { x, y } = PointList.prototype.at.call(this, i);
    if(x < ret.x1) ret.x1 = x;
    if(x > ret.x2) ret.x2 = x;
    if(y < ret.y1) ret.y1 = y;
    if(y > ret.y2) ret.y2 = y;
  }

  return ret;
};

PointList.prototype.rect = function () {
  return new Rect(PointList.prototype.bbox.call(this));
};

PointList.prototype.xrange = function () {
  const bbox = PointList.prototype.bbox.call(this);
  return [bbox.x1, bbox.x2];
};

PointList.prototype.normalizeX = function (newVal = (x) => x) {
  const xrange = PointList.prototype.xrange.call(this);
  const xdiff = xrange[1] - xrange[0];

  PointList.prototype.forEach.call(this, (p, i, l) => {
    l.i.x = newVal((l.i.x - xrange[0]) / xdiff);
  });

  return this;
};

PointList.prototype.yrange = function () {
  const bbox = PointList.prototype.bbox.call(this);
  return [bbox.y1, bbox.y2];
};

PointList.prototype.normalizeY = function (newVal = (y) => y) {
  const yrange = PointList.prototype.yrange.call(this);
  const ydiff = yrange[1] - yrange[0];

  PointList.prototype.forEach.call(this, (p, i, l) => {
    l.i.y = newVal((l.i.y - yrange[0]) / ydiff);
  });

  return this;
};

PointList.prototype.boundingRect = function () {
  return new Rect(PointList.prototype.bbox.call(this));
};

PointList.prototype.translate = function (x, y) {
  PointList.prototype.forEach.call(this, (it) => Point.prototype.move.call(it, x, y));
  return this;
};

PointList.prototype.transform = function (m) {
  if(Util.isObject(m) && typeof m.toMatrix == 'function') m = m.toMatrix();

  if(Util.isObject(m) && typeof m.transform_point == 'function') {
    this.forEach((p) => m.transform_point(p));
    return this;
  }

  for(let i = 0; i < this.length; i++) Point.prototype.transform.call(this.i, m);

  return this;
};

PointList.prototype.filter = function (pred) {
  let ret = new PointList();
  PointList.prototype.forEach.call(this, (p, i, l) => pred(p, i, l) && ret.push(new Point(l.i)));
  return ret;
};

PointList.prototype.getLineIndex = function (index) {
  const len = PointList.prototype.getLength.call(this);
  return (index < 0 ? len + index : index) % len;
};

PointList.prototype.getLine = function (index) {
  let a = PointList.prototype.getLineIndex.call(this, index);
  let b = PointList.prototype.getLineIndex.call(this, index + 1);
  return [PointList.prototype.at.call(this, a), PointList.prototype.at.call(this, b)];
};

PointList.prototype.lines = function (closed = false) {
  const points = this;
  const n = points.length - (closed ? 0 : 1);

  const iterableObj = {
    [[Symbol.iterator]]() {
      let step = 0;

      return {
        next() {
          let value;
          let done = step >= n;

          if(!done) {
            value = new Line(points.step, points[(step + 1) % points.length]);
            step++;
          }

          return { value, done };
        }
      };
    }
  };

  return iterableObj;
};

PointList.prototype.sort = function (pred) {
  return Array.prototype.sort.call(this, pred || ((a, b) => Point.prototype.valueOf.call(a) - Point.prototype.valueOf.call(b)));
};

PointList.prototype.toString = function (sep = (',', prec)) {
  return Array.prototype.map.call(this, (point) => (Point.prototype.toString ? Point.prototype.toString.call(point, prec, sep) : point + '')).join(' ');
};

PointList.prototype.toPath = function () {
  return Array.prototype.map.call(this, (point, i) => `${i > 0 ? 'L' : 'M'}${point}`).join(' ');
  return Array.prototype.reduce.call(this, (acc, point, i) => (acc ? acc + ' ' : '') + `${acc ? 'L' : 'M'}${point}`);
};

PointList.prototype.toSource = function (opts = {}) {
  if(opts.asString) return `new PointList("${this.toString(opts)}")`;
  let fn = opts.asArray ? (p) => `[${p.x},${p.y}]` : opts.plainObj ? (p) => Point.toSource(p, { space: '', padding: ' ', separator: ',' }) : (point) => Point.prototype.toSource.call(point, { ...opts, plainObj: true });
  return 'new PointList([' + PointList.prototype.map.call(this, fn).join(',') + '])';
};

PointList.prototype.add = function (pt) {
  if(!(pt instanceof Point)) pt = new Point(...arguments);
  PointList.prototype.forEach.call(this, (it) => Point.prototype.add.call(it, pt));
  return this;
};

PointList.prototype.sum = function (pt) {
  let ret = PointList.prototype.clone.call(this);
  return PointList.prototype.add.apply(ret, arguments);
};

PointList.prototype.sub = function (pt) {
  if(!(pt instanceof Point)) pt = new Point(...arguments);
  PointList.prototype.forEach.call(this, (it) => Point.prototype.sub.call(it, pt));
  return this;
};

PointList.prototype.diff = function (pt) {
  let ret = PointList.prototype.clone.call(this);
  return PointList.prototype.sub.apply(ret, arguments);
};

PointList.prototype.mul = function (pt) {
  if(typeof pt == 'number') pt = new Point({ x: pt, y: pt });
  if(!(pt instanceof Point)) pt = new Point(...arguments);
  PointList.prototype.forEach.call(this, (it) => Point.prototype.mul.call(it, pt));
  return this;
};

PointList.prototype.prod = function (pt) {
  let ret = PointList.prototype.clone.call(this);
  return PointList.prototype.mul.apply(ret, arguments);
};

PointList.prototype.div = function (pt) {
  if(typeof pt == 'number') pt = new Point({ x: pt, y: pt });
  if(!(pt instanceof Point)) pt = new Point(...arguments);
  PointList.prototype.forEach.call(this, (it) => Point.prototype.div.call(it, pt));
  return this;
};

PointList.prototype.quot = function (pt) {
  let ret = PointList.prototype.clone.call(this);
  return PointList.prototype.div.apply(ret, arguments);
};

PointList.prototype.round = function (prec) {
  PointList.prototype.forEach.call(this, (it) => Point.prototype.round.call(it, prec));
  return this;
};

PointList.prototype.ceil = function (prec) {
  PointList.prototype.forEach.call(this, (it) => Point.prototype.ceil.call(it, prec));
  return this;
};

PointList.prototype.floor = function (prec) {
  PointList.prototype.forEach.call(this, (it) => Point.prototype.floor.call(it, prec));
  return this;
};

PointList.prototype.toMatrix = function () {
  return Array.prototype.map.call(this, ({ x, y }) => Object.freeze([x, y]));
};

if(!Util.isBrowser()) {
  let c = Util.coloring();
  let sym = Symbol.for('nodejs.util.inspect.custom');

  PointList.prototype.sym = function () {
    return `${c.text('PointList', 1, 31)}${c.text('(', 1, 36)}${c.text(this.getLength(), 1, 35) + c.code(1, 36)}) [\n  ${this.map(({ x, y }) => Util.toString({ x, y }, { multiline: false, spacing: ' ' })).join(',\n  ')}\n${c.text(']', 1, 36)}`;
  };
}

for(let name of ['push', 'splice', 'clone', 'area', 'centroid', 'avg', 'bbox', 'rect', 'xrange', 'yrange', 'boundingRect']) {
  PointList.name = (points) => PointList.prototype.name.call(points);
}

export function Polyline(lines) {
  let ret = this instanceof Polyline ? this : new PointList();

  const addUnique = (point) => {
    const ok = ret.length > 0 ? !Point.equals(ret[ret.length - 1], point) : true;
    if(ok) Array.prototype.push.call(ret, Point.clone(point));
    return ok;
  };

  let prev;

  for(let i = 0; i < lines.length; i++) {
    const line = lines.shift();

    if(isPoint(line)) {
      addUnique(line);
      prev = line;
    } else if(isLine(line)) {
      if(i > 0) {
        const eq = [Point.equals(prev, line.a)];
        if(!eq[0] && !Point.equals(prev, line.b)) break;
      } else {
        addUnique(line.a);
      }

      addUnique(line.b);
      prev = line.b;
    }
  }

  return ret;
}
Polyline.prototype = new PointList();

Polyline.prototype.toSVG = function (factory, attrs = ({}, (parent = (null, prec)))) {
  return factory('polyline', { points: PointList.prototype.toString.call(this), ...attrs }, parent, prec);
};

Polyline.prototype.push = function (...args) {
  const last = this[this.length - 1];

  for(let arg of args) {
    if(last && Point.equals(arg, last)) continue;
    PointList.prototype.push.call(this, arg);
  }

  return this;
};

Polyline.prototype.inside = function (point) {
  var i,
    j,
    c = false,
    nvert = this.length;

  for(i = (0, (j = nvert - 1)); i < nvert; j = i++) {
    if(this.i.y > point.y !== this.j.y > point.y && point.x < ((this.j.x - this.i.x) * (point.y - this.i.y)) / (this.j.y - this.i.y) + this.i.x) {
      c = !c;
    }
  }

  return c;
};

Polyline.inside = function (a, b) {
  return a.every((point) => b.inside(point));
};

Polyline.prototype.isClockwise = function () {
  var sum = 0;

  for(var i = 0; i < this.length - 1; i++) {
    var cur = this.i,
      next = this[i + 1];
    sum += (next.x - cur.x) * (next.y + cur.y);
  }

  return sum > 0;
};

Util.defineGetter(Polyline.prototype, 'clockwise', function () {
  let ret = new (this[Symbol.species] || this.constructor)().concat(this);
  return Polyline.prototype.isClockwise.call(this) ? ret : ret.reverse();
});

Util.defineGetter(Polyline.prototype, 'counterClockwise', function () {
  let ret = new (this[Symbol.species] || this.constructor)().concat(this);
  return Polyline.prototype.isClockwise.call(this) ? ret.reverse() : ret;
});

Polyline.isClockwise = function isClockwise(poly) {
  var sum = 0;

  for(var i = 0; i < poly.length - 1; i++) {
    var cur = poly.i,
      next = poly[i + 1];
    sum += (next.x - cur.x) * (next.y + cur.y);
  }

  return sum > 0;
};

Util.define(PointList, {
  get [[Symbol.species]]() {
    return PointList;
  }
});
const ImmutablePointList = Util.immutableClass(PointList);

Util.defineGetter(ImmutablePointList, Symbol.species, function () {
  return ImmutablePointList;
});

/*
 * concatenanted lib/geom/polygonFinder.js
 */

export class PolygonFinder {
  static buildGraphFromSegments(segments) {
    let graph = new Graph();
    let intersections = PolygonFinder.findAllIntersectionsInSegments(segments);
    let connectedSegments = [];
    let connectedIntersections = [];

    segments.forEach((segment) => {
      let intersectionsOnSegment = intersections.filter((intersection) => intersection.line1 === segment || intersection.line2 === segment);

      if(intersectionsOnSegment.length > 1) {
        intersectionsOnSegment.forEach((intersection) => {
          if(!connectedIntersections.includes(intersection)) {
            connectedIntersections.push(intersection);
          }
        });

        connectedSegments.push(segment);
      }
    });

    connectedSegments.forEach((segment) => {
      let intersectionsOnSegment = connectedIntersections.filter((intersection) => intersection.line1 === segment || intersection.line2 === segment);

      let nearestNeighborTrios = intersectionsOnSegment.map((intersection, index, intersections) => {
        let nearestNeighborPair = [null, null];
        let minimumDistancePair = [Infinity, Infinity];
        let possibleNeighbors = intersections.filter((possibleNeighborIntersection) => intersection != possibleNeighborIntersection);

        possibleNeighbors.forEach((possibleNeighbor) => {
          let comparisonProperty = '';
          let distanceBetween = dist(intersection.point.x, intersection.point.y, possibleNeighbor.point.x, possibleNeighbor.point.y);

          if(possibleNeighbor.point.x !== intersection.point.x) {
            comparisonProperty = 'x';
          } else if(possibleNeighbor.point.y !== intersection.point.y) {
            comparisonProperty = 'y';
          } else {
            return null;
          }

          if(possibleNeighbor.point.comparisonProperty < intersection.point.comparisonProperty) {
            if(nearestNeighborPair[0] == null || distanceBetween < minimumDistancePair[0]) {
              nearestNeighborPair[0] = possibleNeighbor;
              minimumDistancePair[0] = distanceBetween;
            }
          } else if(possibleNeighbor.point.comparisonProperty > intersection.point.comparisonProperty) {
            if(nearestNeighborPair[1] == null || distanceBetween < minimumDistancePair[1]) {
              nearestNeighborPair[1] = possibleNeighbor;
              minimumDistancePair[1] = distanceBetween;
            }
          }
        });

        if(nearestNeighborPair[0] === null) {
          nearestNeighborPair[0] = intersection;
        }

        if(nearestNeighborPair[1] === null) {
          nearestNeighborPair[1] = intersection;
        }

        return [intersection, nearestNeighborPair[0], nearestNeighborPair[1]];
      });

      nearestNeighborTrios.forEach((trio) => {
        let nodes = [];

        trio.forEach((intersection) => {
          let newNode = new Graph.Node(intersection.point);
          nodes.push(newNode);
          graph.addNode(newNode);
        });

        nodes.forEach((node) => {
          graph.addNode(node);
        });

        graph.addConnection(nodes[0], nodes[1]);
        graph.addConnection(nodes[0], nodes[2]);
      });
    });

    graph.checkForDuplicateNodes();
    graph.checkForDuplicateConnections();
    return graph;
  }

  static polygonsFromCycles(cycles, graph) {
    let polygons = [];

    cycles.forEach((cycle) => {
      let points = cycle.map((node) => {
        return graph.nodes.node.point;
      });

      polygons.push(points);
    });

    return polygons;
  }

  static polygonsFromSegments(segments) {
    let graph = PolygonFinder.buildGraphFromSegments(segments);
    let cycles = graph.findMinimumCycles();
    return PolygonFinder.polygonsFromCycles(cycles, graph);
  }

  static findAllIntersectionsInSegments(segmentSet) {
    let intersections = [];

    for(let i = 0; i < segmentSet.length; i++) {
      for(let j = i + 1; j < segmentSet.length; j++) {
        let intersection = Intersection.findIntersection(segmentSet.i, segmentSet.j);

        if(intersection !== null) {
          let alreadyInSet = false;

          for(let k = 0; k < intersections.length; k++) {
            if(Intersection.equals(intersections.k, intersection)) {
              alreadyInSet = true;
            }
          }

          if(!alreadyInSet) {
            intersections.push(intersection);
          }
        }
      }
    }

    return intersections;
  }
}

/*
 * concatenanted lib/geom/rect.js
 */

export function Rect(arg) {
  let obj = this instanceof Rect ? this : {};
  let args = arg instanceof Array ? arg : [...arguments];
  let ret;
  if(typeof args[0] == 'number') arg = args;
  else if(Util.isObject(args[0]) && args[0].length !== undefined) arg = args.shift();

  ['x', 'y', 'width', 'height'].forEach((field) => {
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
  } else if(arg && arg.length >= 4 && arg.slice(0, 4).every((arg) => !isNaN(parseFloat(arg)))) {
    let x = arg.shift();
    let y = arg.shift();
    let w = arg.shift();
    let h = arg.shift();
    obj.x = typeof x === 'number' ? x : parseFloat(x);
    obj.y = typeof y === 'number' ? y : parseFloat(y);
    obj.width = typeof w === 'number' ? w : parseFloat(w);
    obj.height = typeof h === 'number' ? h : parseFloat(h);
    ret = 4;
  } else if(arg && arg.length >= 2 && arg.slice(0, 2).every((arg) => !isNaN(parseFloat(arg)))) {
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

Rect.prototype.clone = function (fn) {
  const ctor = this.constructor[Symbol.species] || this.constructor;
  let ret = new ctor(this.x, this.y, this.width, this.height);
  if(fn) fn(ret);
  return ret;
};

Rect.prototype.corners = function () {
  const rect = this;
  return [
    { x: rect.x, y: rect.y },
    { x: rect.x + rect.width, y: rect.y },
    { x: rect.x + rect.width, y: rect.y + rect.height },
    { x: rect.x, y: rect.y + rect.height }
  ];
};

if(Rect.prototype.isSquare === undefined) {
  Rect.prototype.isSquare = function () {
    return Math.abs(this.width - this.height) < 1;
  };
}
Rect.prototype.constructor = Rect;

Rect.prototype.getArea = function () {
  return this.width * this.height;
};

Rect.prototype.toSource = function (opts = {}) {
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

Rect.prototype.points = function (ctor = (items) => Array.from(items)) {
  const c = this.corners();
  return ctor(c);
};
Rect.prototype.toCSS = Rect.toCSS;

Rect.prototype.scale = function (factor) {
  let width = this.width * factor;
  let height = this.height * factor;
  this.x += (width - this.width) / 2;
  this.y += (height - this.height) / 2;
  this.width = width;
  this.height = height;
  return this;
};

Rect.prototype.mul = function (...args) {
  Point.prototype.mul.call(this, ...args);
  Size.prototype.mul.call(this, ...args);
  return this;
};

Rect.prototype.div = function (...args) {
  Point.prototype.div.call(this, ...args);
  Size.prototype.div.call(this, ...args);
  return this;
};

Rect.prototype.outset = function (trbl) {
  if(typeof trbl == 'number') trbl = { top: trbl, right: trbl, bottom: trbl, left: trbl };
  this.x -= trbl.left;
  this.y -= trbl.top;
  this.width += trbl.left + trbl.right;
  this.height += trbl.top + trbl.bottom;
  return this;
};

Rect.prototype.inset = function (trbl) {
  if(typeof trbl == 'number') trbl = new TRBL(trbl, trbl, trbl, trbl);

  if(trbl.left + trbl.right < this.width && trbl.top + trbl.bottom < this.height) {
    this.x += trbl.left;
    this.y += trbl.top;
    this.width -= trbl.left + trbl.right;
    this.height -= trbl.top + trbl.bottom;
  }

  return this;
};

Rect.prototype.inside = function (point) {
  return Rect.inside(this, point);
};
Rect.CONTAIN = 16;
Rect.COVER = 32;

Rect.prototype.fit = function (other, align = Align.CENTER | Align.MIDDLE | Rect.CONTAIN) {
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

Rect.prototype.pointFromCenter = function (point) {
  Point.prototype.sub.call(point, this.center);
  point.x /= this.width;
  point.y /= this.height;
  return point;
};

Rect.prototype.toCSS = function () {
  return { ...Point.prototype.toCSS.call(this), ...Size.prototype.toCSS.call(this) };
};

Rect.prototype.toTRBL = function () {
  return { top: this.y, right: this.x + this.width, bottom: this.y + this.height, left: this.x };
};

Rect.prototype.toArray = function () {
  const { x, y, width, height } = this;
  return [x, y, width, height];
};

Rect.prototype.toPoints = function (...args) {
  let ctor = Util.isConstructor(args[0])
    ? (() => {
        let arg = args.shift();
        return (points) => new arg(points);
      })()
    : (points) => Array.from(points);

  let num = typeof args[0] == 'number' ? args.shift() : 4;
  const { x, y, width, height } = this;
  let a = num == 2 ? [new Point(x, y), new Point(x + width, y + height)] : [new Point(x, y), new Point(x + width, y), new Point(x + width, y + height), new Point(x, y + height)];
  return ctor(a);
};

Rect.prototype.toLines = function (ctor = (lines) => Array.from(lines, (points) => new Line(...points))) {
  let [a, b, c, d] = Rect.prototype.toPoints.call(this);
  return ctor([
    [a, b],
    [b, c],
    [c, d],
    [d, a]
  ]);
};

Rect.prototype.align = function (align_to, a = 0) {
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

Rect.prototype.round = function (precision = (0.001, digits, (type = 'round'))) {
  let { x1, y1, x2, y2 } = this.toObject(true);
  let a = new Point(x1, y1).round(precision, digits, type);
  let b = new Point(x2, y2).round(precision, null, type);
  this.x = a.x;
  this.y = a.y;
  this.width = +(b.x - this.x).toFixed(digits);
  this.height = +(b.y - this.y).toFixed(digits);
  return this;
};

Rect.prototype.toObject = function (bb = false) {
  if(bb) {
    const { x1, y1, x2, y2 } = this;
    return { x1, y1, x2, y2 };
  }

  const { x, y, width, height } = this;
  return { x, y, width, height };
};

Rect.prototype.bbox = function () {
  return this.toObject(true);
};

Rect.prototype.transform = function (m) {
  if(Util.isObject(m) && typeof m.toMatrix == 'function') m = m.toMatrix();
  Matrix.prototype.transform_rect.call(m, this);
  return this;
};

Rect.prototype[Symbol.iterator] = function* () {
  let { x, y, width, height } = this;
  for(let prop of [x, y, width, height]) yield prop;
};
Rect.round = (rect) => Rect.prototype.round.call(rect);
Rect.align = (rect, align_to, a = 0) => Rect.prototype.align.call(rect, align_to, a);
Rect.toCSS = (rect) => Rect.prototype.toCSS.call(rect);
Rect.inset = (rect, trbl) => Rect.prototype.inset.call(rect, trbl);
Rect.outset = (rect, trbl) => Rect.prototype.outset.call(rect, trbl);
Rect.center = (rect) => new Point(rect.x + rect.width / 2, rect.y + rect.height / 2);

Rect.bind = (rect) => {
  let obj = new Rect();
};

Rect.inside = (rect, point) => {
  return point.x >= rect.x && point.x <= rect.x + rect.width && point.y >= rect.y && point.y <= rect.y + rect.height;
};

Rect.from = function (obj) {
  const fn = (v1, v2) => [Math.min(v1, v2), Math.max(v1, v2)];
  const h = fn(obj.x1, obj.x2);
  const v = fn(obj.y1, obj.y2);
  const [x1, x2, y1, y2] = [...h, ...v];
  return new Rect(x1, y1, x2 - x1, y2 - y1);
};

Rect.fromCircle = function (...args) {
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
  if(!gen) gen = (k) => (v) => (v === undefined ? o.k : (o.k = v));
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
  Rect.prototype.f = function (...args) {
    Rect.f(this, ...args);
    return this;
  };
}
Util.defineInspect(Rect.prototype, 'x', 'y', 'width', 'height');
const isRect = (rect) => isPoint(rect) && isSize(rect);

Util.defineGetter(Rect, Symbol.species, function () {
  return this;
});
const ImmutableRect = Util.immutableClass(Rect);
delete ImmutableRect[Symbol.species];

Util.defineGetter(ImmutableRect, Symbol.species, function () {
  return ImmutableRect;
});

Rect.prototype.toString = function (opts = {}) {
  if(typeof opts == 'string') opts = { separator: opts };
  const { precision, unit, separator, left, right } = opts;
  let { x, y, width, height } = this;
  let props = [x, y, width, height];
  return left + props.map((p) => p + unit).join(' ') + right;
};

/*
 * concatenanted lib/geom/size.js
 */

export function Size(arg) {
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

Size.prototype.convertUnits = function (w = 'window' in global ? window : null) {
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

Size.prototype.aspect = function () {
  return this.width / this.height;
};

Size.prototype.toCSS = function (units) {
  let ret = {};
  units = typeof units == 'string' ? { width: units, height: units } : units || this.units || { width: 'px', height: 'px' };
  if(this.width !== undefined) ret.width = this.width + (units.width || 'px');
  if(this.height !== undefined) ret.height = this.height + (units.height || 'px');
  return ret;
};

Size.prototype.transform = function (m) {
  this.width = m.xx * this.width + m.yx * this.height;
  this.height = m.xy * this.width + m.yy * this.height;
  return this;
};

Size.prototype.isSquare = function () {
  return Math.abs(this.width - this.height) < 1;
};

Size.prototype.area = function () {
  return this.width * this.height;
};

Size.prototype.resize = function (width, height) {
  this.width = width;
  this.height = height;
  return this;
};

Size.prototype.sum = function (other) {
  return new Size(this.width + other.width, this.height + other.height);
};

Size.prototype.add = function () {
  for(let other of [...arguments]) {
    this.width += other.width;
    this.height += other.height;
  }

  return this;
};

Size.prototype.diff = function (other) {
  return new Size(this.width - other.width, this.height - other.height);
};

Size.prototype.sub = function () {
  for(let other of [...arguments]) {
    this.width -= other.width;
    this.height -= other.height;
  }

  return this;
};

Size.prototype.prod = function (f) {
  const o = isSize(f) ? f : isPoint(f) ? { width: f.x, height: f.y } : { width: f, height: f };
  return new Size(this.width * o.width, this.height * o.height);
};

Size.prototype.mul = function (...args) {
  for(let f of args) {
    const o = isSize(f) ? f : isPoint(f) ? { width: f.x, height: f.y } : { width: f, height: f };
    this.width *= o.width;
    this.height *= o.height;
  }

  return this;
};

Size.prototype.quot = function (other) {
  return new Size(this.width / other.width, this.height / other.height);
};

Size.prototype.inverse = function (other) {
  return new Size(1 / this.width, 1 / this.height);
};

Size.prototype.div = function (f) {
  for(let f of [...arguments]) {
    this.width /= f;
    this.height /= f;
  }

  return this;
};

Size.prototype.round = function (precision = (0.001, digits)) {
  let { width, height } = this;
  this.width = Util.roundTo(width, precision, digits);
  this.height = Util.roundTo(height, precision, digits);
  return this;
};

Size.prototype.bounds = function (other) {
  let w = [Math.min(this.width, other.width), Math.max(this.width, other.width)];
  let h = [Math.min(this.height, other.height), Math.max(this.height, other.height)];
  let scale = h / this.height;
  this.mul(scale);
  return this;
};

Size.prototype.fit = function (size) {
  size = new Size(size);
  let factors = Size.prototype.fitFactors.call(this, size);
  let ret = [Size.prototype.prod.call(this, factors[0]), Size.prototype.prod.call(this, factors[1])];
  return ret;
};

Size.prototype.fitHeight = function (other) {
  other = new Size(other);
  let scale = other.height / this.height;
  this.mul(scale);
  return [this.width, other.width];
};

Size.prototype.fitWidth = function (other) {
  other = new Size(other);
  let scale = other.width / this.width;
  this.mul(scale);
  return [this.height, other.height];
};

Size.prototype.fitFactors = function (other) {
  const hf = other.width / this.width;
  const vf = other.height / this.height;
  return [hf, vf];
};

Size.prototype.toString = function (opts = {}) {
  const { unit, separator, left, right } = opts;
  const { width, height, units } = this;
  return `${left}${width}${units.width || ''}${separator}${height}${units.height || ''}${right}`;
};
Size.area = (sz) => Size.prototype.area.call(sz);
Size.aspect = (sz) => Size.prototype.aspect.call(sz);

Size.bind = (o, p, gen) => {
  const [width, height] = p || ['width', 'height'];
  if(!gen) gen = (k) => (v) => (v === undefined ? o.k : (o.k = v));
  return Util.bindProperties(new Size(0, 0), o, { width, height }, gen);
};
for(let method of Util.getMethodNames(Size.prototype)) Size.method = (size, ...args) => Size.prototype.method.call(size || new Size(size), ...args);
const isSize = (o) => o && ((o.width !== undefined && o.height !== undefined) || (o.x !== undefined && o.x2 !== undefined && o.y !== undefined && o.y2 !== undefined) || (o.left !== undefined && o.right !== undefined && o.top !== undefined && o.bottom !== undefined));

for(let name of ['toCSS', 'isSquare', 'round', 'sum', 'add', 'diff', 'sub', 'prod', 'mul', 'quot', 'div']) {
  Size.name = (size, ...args) => Size.prototype.name.call(size || new Size(size), ...args);
}

Util.defineGetter(Size, Symbol.species, function () {
  return this;
});
const ImmutableSize = Util.immutableClass(Size);

Util.defineGetter(ImmutableSize, Symbol.species, function () {
  return ImmutableSize;
});

/*
 * concatenanted lib/geom/sweepLine.js
 */

export class SweepLineClass {
  constructor() {
    this.objectNodeMap = new Map();
    this.queueHead = null;
  }

  add(object, loVal, hiVal) {
    let hiNode = new SweepLineClass.NodeClass(this, object, SweepLineClass._HI, hiVal);
    let loNode = new SweepLineClass.NodeClass(this, object, SweepLineClass._LO, loVal);
    this.objectNodeMap.set(object, { loNode, hiNode });
  }

  update(object, loVal, hiVal) {
    let n = this.objectNodeMap.get(object);

    if(n) {
      n.hiNode.x = hiVal || n.hiNode.x;
      this.sortNode(n.hiNode);
      n.loNode.x = loVal || n.loNode.x;
      this.sortNode(n.loNode);
    }
  }

  del(object) {
    n = this.objectNodeMap.get(object);

    if(n) {
      this.deleteNode(n.hiNode);
      this.deleteNode(n.loNode);
      this.objectNodeMap.delete(object);
    }
  }

  sortNode(node) {
    function moveNode() {
      this.deleteNode(node);

      if(newLocation === null) {
        node.prev = null;
        node.next = this.queueHead;
        this.queueHead = node;
      } else {
        node.prev = newLocation;
        node.next = newLocation.next;
        if(newLocation.next) newLocation.next.prev = node;
        newLocation.next = node;
      }
    }

    let newLocation = node.prev;

    while(newLocation && node.x < newLocation.x) {
      newLocation = newLocation.prev;
    }

    if(newLocation !== node.prev) moveNode.call(this);
    newLocation = node;

    while(newLocation.next && newLocation.next.x < node.x) {
      newLocation = newLocation.next;
    }

    if(newLocation !== node) moveNode.call(this);
  }

  deleteNode(node) {
    if(node.prev === null) this.queueHead = node.next;
    if(node.prev) node.prev.next = node.next;
    if(node.next) node.next.prev = node.prev;
  }

  findCollisions(collisionFunction) {
    let collision = [];
    let activeObjects = new Set();
    var node = this.queueHead;

    while(node) {
      if(node.loHi === SweepLineClass._LO) {
        let object = node.object;

        for(let ao of activeObjects) {
          if(collisionFunction(object, ao)) {
            collision.push([object, ao]);
          }
        }

        activeObjects.add(object);
      } else {
        activeObjects.delete(node.object);
      }

      node = node.next;
    }

    return collision;
  }

  print(printFunction) {
    var n = this.queueHead;

    while(n) {
      printFunction(n.object, n.loHi, n.x);
      n = n.next;
    }
  }
}
SweepLineClass._LO = false;
SweepLineClass._HI = true;

SweepLineClass.NodeClass = class {
  constructor(sweepLine, object, loHi, x) {
    this.object = object;
    this.parent = sweepLine;
    this.loHi = loHi;
    this.x = x;
    this.prev = null;
    this.next = null;
    if(sweepLine.queueHead) sweepLine.queueHead.prev = this;
    this.next = sweepLine.queueHead;
    sweepLine.queueHead = this;
    sweepLine.sortNode(this);
  }
};

/*
 * concatenanted lib/geom/transformation.js
 */

export class Transformation {
  constructor(typeName) {
    return this;
  }

  get type() {
    let type =
      this.typeName ||
      Util.className(this)
        .toLowerCase()
        .replace(/transform(ation)?/, '')
        .replace(/(ion|ing)$/, 'e');
    return type;
  }

  get [[Symbol.isConcatSpreadable]]() {
    return this.constructor === TransformationList || Object.getPrototypeOf(this) == TransformationList.prototype || Object.getPrototypeOf(this).constructor == TransformationList;
  }

  get axes() {
    return this.axis !== undefined ? [this.axis] : ['x', 'y', 'z'].filter((axis) => axis in this);
  }

  get props() {
    return this.axes.concat(['axis', 'angle'].filter((key) => key in this));
  }

  has(axis) {
    if(this.axis !== undefined) return axis === this.axis;
    return axis in this;
  }

  get is3D() {
    return this.has('z');
  }

  entries() {
    return this.props.map((prop) => [prop, this.prop]);
  }

  toJSON() {
    return Object.fromEntries(this.entries());
  }

  vector(unit) {
    unit = this.unit || unit;
    return (this.is3D ? ['x', 'y', 'z'] : ['x', 'y']).map(unit ? (axis) => this.axis + unit : (axis) => this.axis);
  }

  toString(tUnit) {
    return `${this.type}${this.is3D ? '3d' : ''}(${this.vector(tUnit).join(', ')})`;
  }

  clone() {
    let desc = Object.getOwnPropertyDescriptors(this);
    let props = this.props.reduce((acc, prop) => ({ ...acc, [[prop]]: desc.prop }), {});
    return Object.create(Object.getPrototypeOf(this), props);
  }

  static fromString(arg) {
    let cmdLen = arg.indexOf('(');
    let argStr = arg.slice(cmdLen + 1, arg.indexOf(')'));
    let args = argStr.split(/[,\s\ ]+/g);
    let cmd = arg.substring(0, cmdLen);
    let t;
    let unit;

    args = args
      .filter((arg) => /^[-+0-9.]+[a-z]*$/.test(arg))
      .map((arg) => {
        if(/[a-z]$/.test(arg)) {
          unit = arg.replace(/[-+0-9.]*/g, '');
          arg = arg.replace(/[a-z]*$/g, '');
        }

        return +arg;
      });

    if(cmd.startsWith('rotat')) {
      const axis = cmd.slice(6);
      args = axis != '' ? [args[0], axis] : args;
      t = new Rotation(...args);
    } else if(cmd.startsWith('translat')) {
      const axis = cmd.slice(9);
      args = axis != '' ? [args[0], axis] : args;
      t = new Translation(...args);
    } else if(cmd.startsWith('scal')) {
      const axis = cmd.slice(5);
      args = axis != '' ? [args[0], axis] : args;
      t = new Scaling(...args);
    } else if(cmd.startsWith('matrix')) {
      t = new MatrixTransformation(...args);
    }

    if(unit) t.unit = unit;
    return t;
  }

  [[Symbol.toPrimitive]](hint) {
    if(hint == 'string') return this.toString();
    return this.toString() != '';
  }

  static get rotation() {
    return Rotation;
  }

  static get translation() {
    return Translation;
  }

  static get scaling() {
    return Scaling;
  }

  static get matrix() {
    return MatrixTransformation;
  }
}

Object.defineProperty(Transformation, Symbol.hasInstance, {
  value(inst) {
    return [Transformation, MatrixTransformation, Rotation, Translation, Scaling, TransformationList].some((ctor) => Object.getPrototypeOf(inst) == ctor.prototype);
  }
});
const ImmutableTransformation = Util.immutableClass(Transformation);

export class Rotation extends Transformation {
  angle = 0;
  static RAD2DEG = 180 / Math.PI;
  static DEG2RAD = 1 / Rotation.RAD2DEG;
  constructor(angle, axis) {
    super('rotate');
    if(typeof axis == 'string' && ['x', 'y', 'z'].indexOf(axis.toLowerCase()) != -1) this.axis = axis.toLowerCase();
    this.angle = angle;
  }

  invert() {
    return new Rotation(-this.angle, this.axis);
  }

  get values() {
    return { [[this.axis || 'z']]: this.angle };
  }

  get is3D() {
    return this.axis == 'z';
  }

  isZero() {
    return this.angle == 0;
  }

  toString(rUnit) {
    rUnit = rUnit || this.unit || '';
    const axis = this.axis !== undefined ? this.axis.toUpperCase() : '';
    const angle = this.constructor.convertAngle(this.angle, rUnit);
    return `rotate${this.is3D ? axis : ''}(${angle}${rUnit})`;
  }

  toSource() {
    let o = Util.colorText('new ', 1, 31) + Util.colorText(Util.className(this), 1, 33) + Util.colorText('(' + this.angle + ')', 1, 36);
    return o;
  }

  toMatrix() {
    return Matrix.rotate(Rotation.DEG2RAD * this.angle);
  }

  accumulate(other) {
    if(this.type !== other.type && this.axis !== other.axis) throw new Error(Util.className(this) + ': accumulate mismatch');

    return new Rotation(this.angle + other.angle, this.axis);
  }

  static convertAngle(angle, unit) {
    switch (unit) {
      case 'deg':
        return angle;
      case 'rad':
        return this.DEG2RAD * angle;
      case 'turn':
        return angle / 360;
      default:
        return angle;
    }
  }
}
const ImmutableRotation = Util.immutableClass(Rotation);

export class Translation extends Transformation {
  x = 0;
  y = 0;
  constructor(...args) {
    super('translate');

    if(typeof args[1] == 'string' && ['x', 'y', 'z'].indexOf(args[1].toLowerCase()) != -1) {
      const axis = args[1].toLowerCase();
      this.axis = args[0];
    } else {
      const [x, y, z] = args;
      this.x = x;
      this.y = y;
      if(z !== undefined) this.z = z;
    }
  }

  get values() {
    const { x, y, z } = this;
    return 'z' in this ? { x, y, z } : { x, y };
  }

  isZero() {
    const { x, y, z } = this;
    return 'z' in this ? x == 0 && y == 0 && z == 0 : x == 0 && y == 0;
  }

  toMatrix(matrix = Matrix.IDENTITY) {
    return matrix.translate(this.x, this.y, this.z);
  }

  invert() {
    const { x, y, z } = this;
    return z !== undefined ? new Translation(-x, -y, -z) : new Translation(Math.abs(x) == 0 ? 0 : -x, Math.abs(y) == 0 ? 0 : -y);
  }

  accumulate(other) {
    if(this.type !== other.type) throw new Error(Util.className(this) + ': accumulate mismatch');

    if(this.is3D) return new Translation(this.x + other.x, this.y + other.y, this.z + other.z);
    return new Translation(this.x + other.x, this.y + other.y);
  }
}
const ImmutableTranslation = Util.immutableClass(Translation);

export class Scaling extends Transformation {
  x = 1;
  y = 1;
  constructor(...args) {
    super('scale');

    if(typeof args[1] == 'string' && ['x', 'y', 'z'].indexOf(args[1].toLowerCase()) != -1) {
      const axis = args[1].toLowerCase();
      this.axis = args[0];
    } else {
      const [x, y, z] = args;
      this.x = x;
      this.y = y === undefined ? x : y;
      if(z !== undefined) this.z = z;
    }
  }

  get values() {
    const { x, y, z } = this;
    return 'z' in this ? { x, y, z } : { x, y };
  }

  toMatrix(matrix = Matrix.IDENTITY) {
    return matrix.scale(this.x, this.y, this.z);
  }

  isZero() {
    const { x, y, z } = this;
    return 'z' in this ? x == 0 && y == 0 && z == 0 : x == 0 && y == 0;
  }

  invert() {
    const { x, y, z } = this;
    return z !== undefined ? new Scaling(1 / x, 1 / y, 1 / z) : new Scaling(1 / x, 1 / y);
  }

  accumulate(other) {
    if(this.type !== other.type) throw new Error(Util.className(this) + ': accumulate mismatch');

    if(this.is3D) return new Scaling(this.x * other.x, this.y * other.y, this.z * other.z);
    return new Scaling(this.x * other.x, this.y * other.y);
  }
}
const ImmutableScaling = Util.immutableClass(Scaling);

export class MatrixTransformation extends Transformation {
  matrix = Matrix.IDENTITY;
  constructor(init) {
    super('matrix');
    if(init instanceof Matrix) this.matrix = init;
    else if(isMatrix(init)) this.matrix = new Matrix(init);
    else this.matrix = new Matrix(...arguments);
  }

  get values() {
    return this.matrix.values();
  }

  toMatrix() {
    return this.matrix.clone();
  }

  toString() {
    return this.matrix.toString('');
  }

  invert() {
    return new MatrixTransformation(this.matrix.invert());
  }

  isZero() {
    return this.matrix.isIdentity();
  }

  accumulate(other) {
    if(this.type !== other.type) throw new Error(Util.className(this) + ': accumulate mismatch');

    return new MatrixTransformation(this.matrix.multiply(other.matrix));
  }
}
const ImmutableMatrixTransformation = Util.immutableClass(MatrixTransformation);

export class TransformationList extends Array {
  constructor(init, ...rest) {
    super();
    if(init !== undefined) this.initialize(init, ...rest);
    return this;
  }

  initialize(init, ...args) {
    if(typeof init == 'number') while(this.length < init) this.push(undefined);
    else if(typeof init == 'string') TransformationList.prototype.fromString.call(this, init);
    else if(init instanceof Array) TransformationList.prototype.fromArray.call(this, init);
    else throw new Error('No such initialization: ' + init);

    return this;
  }

  get [[Symbol.isConcatSpreadable]]() {
    return true;
  }

  static get [[Symbol.species]]() {
    return TransformationList;
  }

  fromString(str) {
    let n,
      a = [];

    for(let i = 0; i < str.length; i += n) {
      let s = str.slice(i);
      n = s.indexOf(')') + 1;
      if(n == 0) n = str.length;
      s = s.slice(0, n).trim();
      if(s != '') a.push(s);
    }

    return this.fromArray(a);
  }

  fromArray(arr) {
    for(let i = 0; i < arr.length; i++) {
      const arg = arr.i;
      if(arg instanceof Transformation) this.push(arg);
      else if(typeof arg == 'string') this.push(Transformation.fromString(arg));
      else throw new Error('No such transformation: ' + arg);
    }

    return this;
  }

  static fromString(str) {
    return new TransformationList().fromString(str);
  }

  static fromArray(arr) {
    return new TransformationList().fromArray(arr);
  }

  static fromMatrix(matrix) {
    matrix = matrix instanceof Matrix ? matrix : new Matrix(matrix);
    const transformations = Matrix.decompose(matrix, true);

    Util.extend(transformations.scale, {
      toArray() {
        return [this.x, this.y];
      }
    });

    Util.extend(transformations.translate, {
      toArray() {
        return [this.x, this.y];
      }
    });

    let ret = new TransformationList();
    ret.translate(...transformations.translate.toArray());
    ret.rotate(transformations.rotate);
    ret.scale(...transformations.scale.toArray());
    return ret;
  }

  push(...args) {
    for(let arg of args) {
      if(typeof arg == 'string') arg = Transformation.fromString(arg);
      else if(isMatrix(arg)) arg = new MatrixTransformation(arg);
      Array.prototype.push.call(this, arg);
    }

    return this;
  }

  clone() {
    return this.map((t) => t.clone());
  }

  unshift(...args) {
    for(let arg of args.reverse()) {
      if(typeof arg == 'string') arg = Transformation.fromString(arg);
      Array.prototype.unshift.call(this, arg);
    }

    return this;
  }

  rotate(...args) {
    Array.prototype.push.call(this, new Rotation(...args));
    return this;
  }

  translate(x, y) {
    let trans = this.filter((t) => !t.type.startsWith('translat'));
    let vec = new Point(x, y);
    vec = vec.round(0.00001, 5);
    if(Math.abs(vec.x) != 0 || Math.abs(vec.y) != 0) Array.prototype.push.call(this, new Translation(vec.x, vec.y));
    return this;
  }

  scale(...args) {
    Array.prototype.push.call(this, new Scaling(...args));
    return this;
  }

  matrix(...args) {
    Array.prototype.push.call(this, new MatrixTransformation(...args));
    return this;
  }

  toString(tUnit, rUnit) {
    return this.map((t) => t.toString(t.type.startsWith('scal') ? '' : t.type.startsWith('rotat') ? rUnit : tUnit)).join(' ');
  }

  [[Symbol.toStringTag]]() {
    return this.toString();
  }

  toSource() {
    let s = Util.colorText('new ', 1, 31) + Util.colorText(Util.className(this), 1, 33) + Util.colorText('([', 1, 36);
    s += this.map((t) => t.toSource()).join(', ');
    return s + Util.colorText('])', 1, 36);
  }

  toMatrices() {
    return Array.prototype.map.call(this, (t) => t.toMatrix());
  }

  toMatrix() {
    let matrix = Matrix.IDENTITY;
    for(let other of this.toMatrices()) matrix = matrix.multiply(other);
    return matrix;
  }

  undo() {
    let ret = new TransformationList();

    for(let i = this.length - 1; i >= 0; i--) Array.prototype.push.call(ret, this.i.invert());

    return ret;
  }

  merge(...args) {
    for(let arg of args) {
      if(typeof arg == 'string') arg = TransformationList.fromString(arg);
      TransformationList.prototype.push.apply(this, arg);
    }

    return this;
  }

  decompose(degrees = true) {
    let matrix = this.toMatrix();
    const { translate, rotate, scale } = matrix.decompose(degrees);
    let ret = { translate, rotate, scale };

    ret.scale.toArray = ret.translate.toArray = function toArray() {
      return [this.x, this.y];
    };

    return ret;
  }

  findLast(predicate) {
    for(let i = this.length - 1; i >= 0; --i) {
      const x = this.i;
      if(predicate(x)) return x;
    }

    return null;
  }

  get rotation() {
    return this.findLast((item) => item.type.startsWith('rotat'));
  }

  get scaling() {
    return this.findLast((item) => item.type.startsWith('scal'));
  }

  get translation() {
    return this.findLast((item) => item.type.startsWith('translat'));
  }

  get last() {
    return this.at(-1);
  }

  get first() {
    return this.at(0);
  }

  at(pos) {
    if(pos < 0) pos += this.length;
    return this.pos;
  }

  collapse() {
    let ret = new TransformationList();

    for(let i = 0; i < this.length; i++) {
      let item = this.i;

      if(i + 1 < this.length && this[i + 1].type == this.i.type) {
        item = item.accumulate(this[i + 1]);
        i++;
      } else {
        item = item.clone();
      }

      Array.prototype.push.call(ret, item);
    }

    return ret;
  }

  collapseAll() {
    return TransformationList.fromMatrix(this.toMatrix());
  }

  invert() {
    return new TransformationList(this.reduceRight((acc, t) => [...acc, t.invert()], []));
  }

  join(sep = ' ') {
    return Array.prototype.join.call(this, sep);
  }

  clear() {
    return this.splice(0, this.length);
  }
}
const { concat, copyWithin, find, findIndex, lastIndexOf, pop, push, shift, unshift, slice, splice, includes, indexOf, entries, filter, map, every, some, reduce, reduceRight } = Array.prototype;

Util.inherit(
  TransformationList.prototype,
  { concat, copyWithin, find, findIndex, lastIndexOf, pop, shift, slice, splice, includes, indexOf, entries, filter, map, every, some, reduce, reduceRight },
  {
    [[Symbol.iterator]]() {
      return Array.prototype[Symbol.iterator];
    },
    [[Symbol.isConcatSpreadable]]() {
      return true;
    }
  }
);
const ImmutableTransformationList = Util.immutableClass(TransformationList);

Util.defineGetter(ImmutableTransformationList, Symbol.species, function () {
  return ImmutableTransformationList;
});

/*
 * concatenanted lib/geom/trbl.js
 */

export function TRBL(arg) {
  let ret = this instanceof TRBL ? this : {};
  let args = [...arguments];

  if(typeof arg === 'object' && !Util.isArray(arg)) {
    Object.keys(arg).forEach((k) => {
      const matches = /(top|right|bottom|left)/i.exec(k);
      ret[matches[0].toLowerCase()] = parseInt(arg.k);
    });
  } else if(arg) {
    if(args.length > 1) arg = args;
    if(typeof arg === 'string') arg = [...arg.matchAll(/^[0-9.]+(|px|em|rem|pt|cm|mm)$/g)];
    else if(arg.length == 4) arg = arg.map((v) => parseInt(v.replace(/[a-z]*$/g, '')));
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

TRBL.prototype.null = function () {
  return this.top == 0 && this.right == 0 && this.bottom == 0 && this.left == 0;
};
TRBL.null = (trbl) => TRBL.prototype.null.call(trbl);
TRBL.neg = (trbl = this) => ({ top: -trbl.top, right: -trbl.right, bottom: -trbl.bottom, left: -trbl.left });

TRBL.prototype.isNaN = function () {
  return isNaN(this.top) || isNaN(this.right) || isNaN(this.bottom) || isNaN(this.left);
};

Object.defineProperty(TRBL.prototype, 'inset', {
  get() {
    return (rect) => Rect.inset(rect, this);
  }
});

Object.defineProperty(TRBL.prototype, 'outset', {
  get() {
    return (rect) => Rect.outset(rect, this);
  }
});

TRBL.prototype.add = function (other) {
  this.top += other.top;
  this.right += other.right;
  this.bottom += other.bottom;
  this.left += other.left;
};

TRBL.prototype.union = function (other) {
  this.top = other.top < this.top ? other.top : this.top;
  this.right = other.right > this.right ? other.right : this.right;
  this.bottom = other.bottom > this.bottom ? other.bottom : this.bottom;
  this.left = other.left < this.left ? other.left : this.left;
};

TRBL.prototype.toRect = function () {
  return new Rect({ x: this.left, y: this.top, width: this.right - this.left, height: this.bottom - this.top });
};

TRBL.prototype.toRect = function () {
  return new Rect({ x: this.left, y: this.top, width: this.right - this.left, height: this.bottom - this.top });
};
TRBL.union = (trbl, other) => ({ top: other.top < trbl.top ? other.top : trbl.top, right: other.right > trbl.right ? other.right : trbl.right, bottom: other.bottom > trbl.bottom ? other.bottom : trbl.bottom, left: other.left < trbl.left ? other.left : trbl.left });
TRBL.toRect = (trbl) => new Rect(trbl.left, trbl.top, trbl.right - trbl.left, trbl.bottom - trbl.top);

TRBL.prototype.toString = function (unit = 'px') {
  return '' + this.top + '' + unit + ' ' + this.right + '' + unit + ' ' + this.bottom + '' + unit + ' ' + this.left + unit;
};

TRBL.prototype.toSource = function () {
  return '{top:' + this.top + ',right:' + this.right + ',bottom:' + this.bottom + ',left:' + this.left + '}';
};

for(let name of ['null', 'isNaN', 'outset', 'toRect', 'toSource']) {
  TRBL.name = (points) => TRBL.prototype.name.call(points);
}

export function isTRBL(obj) {
  return top in obj && right in obj && bottom in obj && left in obj;
}

Util.defineGetter(TRBL, Symbol.species, function () {
  return this;
});
const ImmutableTRBL = Util.immutableClass(TRBL);

Util.defineGetter(ImmutableTRBL, Symbol.species, function () {
  return ImmutableTRBL;
});

/*
 * concatenanted lib/geom/vector.js
 */

export function Vector(init, n, base = Int32Array) {
  if(n === undefined) n = init.length;
  let buf = new ArrayBuffer(base.BYTES_PER_ELEMENT * n);
  let vec = new base(buf);

  for(let i = 0; i < n; i++) vec.set([init.i], i);

  return vec;
}

/*
 * concatenanted lib/geom/voronoi.js
 */

export class Voronoi {
  constructor() {
    this.vertices = null;
    this.edges = null;
    this.cells = null;
    Util.define(this, { toRecycle: null, beachsectionJunkyard: [], circleEventJunkyard: [], vertexJunkyard: [], edgeJunkyard: [], cellJunkyard: [] });
  }

  reset() {
    if(!this.beachline) this.beachline = new this.RBTree();

    if(this.beachline.root) {
      var beachsection = this.beachline.getFirst(this.beachline.root);

      while(beachsection) {
        this.beachsectionJunkyard.push(beachsection);
        beachsection = beachsection.rbNext;
      }
    }

    this.beachline.root = null;
    if(!this.circleEvents) this.circleEvents = new this.RBTree();
    this.circleEvents.root = this.firstCircleEvent = null;
    this.vertices = [];
    this.edges = [];
    this.cells = [];
  }

  get sqrt() {
    return Math.sqrt;
  }

  get abs() {
    return Math.abs;
  }

  get [['ε']]() {
    return (Voronoi['ε'] = 1e-9);
  }

  get [['invε']]() {
    return (Voronoi.invε = 1.0 / Voronoi.ε);
  }

  equalWithEpsilon(a, b) {
    return this.abs(a - b) < 1e-9;
  }

  greaterThanWithEpsilon(a, b) {
    return a - b > 1e-9;
  }

  greaterThanOrEqualWithEpsilon(a, b) {
    return b - a < 1e-9;
  }

  lessThanWithEpsilon(a, b) {
    return b - a > 1e-9;
  }

  lessThanOrEqualWithEpsilon(a, b) {
    return a - b < 1e-9;
  }

  get RBTree() {
    return RBTree;
  }

  get Diagram() {
    return Diagram;
  }

  get Cell() {
    return Cell;
  }

  get Vertex() {
    return Vertex;
  }

  get Edge() {
    return Edge;
  }

  get Halfedge() {
    return Halfedge;
  }

  createVertex(x, y) {
    var v = this.vertexJunkyard.pop();

    if(!v) {
      v = new this.Vertex(x, y);
    } else {
      v.x = x;
      v.y = y;
    }

    this.vertices.push(v);
    return v;
  }

  createEdge(lSite, rSite, va, vb) {
    var edge = this.edgeJunkyard.pop();

    if(!edge) {
      edge = new this.Edge(lSite, rSite);
    } else {
      edge.lSite = lSite;
      edge.rSite = rSite;
      edge.va = edge.vb = null;
    }

    this.edges.push(edge);

    if(va) {
      this.setEdgeStartpoint(edge, lSite, rSite, va);
    }

    if(vb) {
      this.setEdgeEndpoint(edge, lSite, rSite, vb);
    }

    this.cells[lSite.voronoiId].halfedges.push(this.createHalfedge(edge, lSite, rSite));
    this.cells[rSite.voronoiId].halfedges.push(this.createHalfedge(edge, rSite, lSite));
    return edge;
  }

  createBorderEdge(lSite, va, vb) {
    var edge = this.edgeJunkyard.pop();

    if(!edge) {
      edge = new this.Edge(lSite, null);
    } else {
      edge.lSite = lSite;
      edge.rSite = null;
    }

    edge.va = va;
    edge.vb = vb;
    this.edges.push(edge);
    return edge;
  }

  setEdgeStartpoint(edge, lSite, rSite, vertex) {
    if(!edge.va && !edge.vb) {
      edge.va = vertex;
      edge.lSite = lSite;
      edge.rSite = rSite;
    } else if(edge.lSite === rSite) {
      edge.vb = vertex;
    } else {
      edge.va = vertex;
    }
  }

  setEdgeEndpoint(edge, lSite, rSite, vertex) {
    this.setEdgeStartpoint(edge, rSite, lSite, vertex);
  }

  get Beachsection() {
    return this.getBeachsection();
  }

  getBeachsection = Util.memoize(() => function Beachsection() {});
  createBeachsection(site) {
    let beachsection = this.beachsectionJunkyard.pop() || new this.Beachsection();
    beachsection.site = site;
    return beachsection;
  }

  leftBreakPoint(arc, directrix) {
    var site = arc.site,
      rfocx = site.x,
      rfocy = site.y,
      pby2 = rfocy - directrix;
    if(!pby2) return rfocx;
    var lArc = arc.rbPrevious;
    if(!lArc) return -Infinity;
    site = lArc.site;
    var lfocx = site.x,
      lfocy = site.y,
      plby2 = lfocy - directrix;
    if(!plby2) return lfocx;
    var hl = lfocx - rfocx,
      aby2 = 1 / pby2 - 1 / plby2,
      b = hl / plby2;
    if(aby2) return (-b + this.sqrt(b * b - 2 * aby2 * ((hl * hl) / (-2 * plby2) - lfocy + plby2 / 2 + rfocy - pby2 / 2))) / aby2 + rfocx;
    return (rfocx + lfocx) / 2;
  }

  rightBreakPoint(arc, directrix) {
    var rArc = arc.rbNext;
    if(rArc) return this.leftBreakPoint(rArc, directrix);
    var site = arc.site;
    return site.y === directrix ? site.x : Infinity;
  }

  detachBeachsection(beachsection) {
    this.detachCircleEvent(beachsection);
    this.beachline.rbRemoveNode(beachsection);
    this.beachsectionJunkyard.push(beachsection);
  }

  removeBeachsection(beachsection) {
    var circle = beachsection.circleEvent,
      x = circle.x,
      y = circle.ycenter,
      vertex = this.createVertex(x, y),
      previous = beachsection.rbPrevious,
      next = beachsection.rbNext,
      disappearingTransitions = [beachsection],
      abs_fn = Math.abs;
    this.detachBeachsection(beachsection);
    var lArc = previous;

    while(lArc.circleEvent && abs_fn(x - lArc.circleEvent.x) < 1e-9 && abs_fn(y - lArc.circleEvent.ycenter) < 1e-9) {
      previous = lArc.rbPrevious;
      disappearingTransitions.unshift(lArc);
      this.detachBeachsection(lArc);
      lArc = previous;
    }

    disappearingTransitions.unshift(lArc);
    this.detachCircleEvent(lArc);
    var rArc = next;

    while(rArc.circleEvent && abs_fn(x - rArc.circleEvent.x) < 1e-9 && abs_fn(y - rArc.circleEvent.ycenter) < 1e-9) {
      next = rArc.rbNext;
      disappearingTransitions.push(rArc);
      this.detachBeachsection(rArc);
      rArc = next;
    }

    disappearingTransitions.push(rArc);
    this.detachCircleEvent(rArc);
    var nArcs = disappearingTransitions.length,
      iArc;

    for(iArc = 1; iArc < nArcs; iArc++) {
      rArc = disappearingTransitions.iArc;
      lArc = disappearingTransitions[iArc - 1];
      this.setEdgeStartpoint(rArc.edge, lArc.site, rArc.site, vertex);
    }

    lArc = disappearingTransitions[0];
    rArc = disappearingTransitions[nArcs - 1];
    rArc.edge = this.createEdge(lArc.site, rArc.site, undefined, vertex);
    this.attachCircleEvent(lArc);
    this.attachCircleEvent(rArc);
  }

  addBeachsection(site) {
    var x = site.x,
      directrix = site.y;
    var lArc,
      rArc,
      dxl,
      dxr,
      node = this.beachline.root;

    while(node) {
      dxl = this.leftBreakPoint(node, directrix) - x;

      if(dxl > 1e-9) {
        node = node.rbLeft;
      } else {
        dxr = x - this.rightBreakPoint(node, directrix);

        if(dxr > 1e-9) {
          if(!node.rbRight) {
            lArc = node;
            break;
          }

          node = node.rbRight;
        } else {
          if(dxl > -1e-9) {
            lArc = node.rbPrevious;
            rArc = node;
          } else if(dxr > -1e-9) {
            lArc = node;
            rArc = node.rbNext;
          } else {
            lArc = rArc = node;
          }

          break;
        }
      }
    }

    var newArc = this.createBeachsection(site);
    this.beachline.rbInsertSuccessor(lArc, newArc);

    if(!lArc && !rArc) {
      return;
    }

    if(lArc === rArc) {
      this.detachCircleEvent(lArc);
      rArc = this.createBeachsection(lArc.site);
      this.beachline.rbInsertSuccessor(newArc, rArc);
      newArc.edge = rArc.edge = this.createEdge(lArc.site, newArc.site);
      this.attachCircleEvent(lArc);
      this.attachCircleEvent(rArc);
      return;
    }

    if(lArc && !rArc) {
      newArc.edge = this.createEdge(lArc.site, newArc.site);
      return;
    }

    if(lArc !== rArc) {
      this.detachCircleEvent(lArc);
      this.detachCircleEvent(rArc);
      var lSite = lArc.site,
        ax = lSite.x,
        ay = lSite.y,
        bx = site.x - ax,
        by = site.y - ay,
        rSite = rArc.site,
        cx = rSite.x - ax,
        cy = rSite.y - ay,
        d = 2 * (bx * cy - by * cx),
        hb = bx * bx + by * by,
        hc = cx * cx + cy * cy,
        vertex = this.createVertex((cy * hb - by * hc) / d + ax, (bx * hc - cx * hb) / d + ay);
      this.setEdgeStartpoint(rArc.edge, lSite, rSite, vertex);
      newArc.edge = this.createEdge(lSite, site, undefined, vertex);
      rArc.edge = this.createEdge(site, rSite, undefined, vertex);
      this.attachCircleEvent(lArc);
      this.attachCircleEvent(rArc);
      return;
    }
  }

  get CircleEvent() {
    return this.getCircleEvent();
  }

  getCircleEvent = Util.memoize(
    () =>
      class CircleEvent {
        constructor() {
          this.arc = null;
          this.rbLeft = null;
          this.rbNext = null;
          this.rbParent = null;
          this.rbPrevious = null;
          this.rbRed = false;
          this.rbRight = null;
          this.site = null;
          this.x = this.y = this.ycenter = 0;
        }
      }
  );
  attachCircleEvent(arc) {
    var lArc = arc.rbPrevious,
      rArc = arc.rbNext;

    if(!lArc || !rArc) {
      return;
    }

    var lSite = lArc.site,
      cSite = arc.site,
      rSite = rArc.site;

    if(lSite === rSite) {
      return;
    }

    var bx = cSite.x,
      by = cSite.y,
      ax = lSite.x - bx,
      ay = lSite.y - by,
      cx = rSite.x - bx,
      cy = rSite.y - by;
    var d = 2 * (ax * cy - ay * cx);

    if(d >= -2e-12) {
      return;
    }

    var ha = ax * ax + ay * ay,
      hc = cx * cx + cy * cy,
      x = (cy * ha - ay * hc) / d,
      y = (ax * hc - cx * ha) / d,
      ycenter = y + by;
    var circleEvent = this.circleEventJunkyard.pop();

    if(!circleEvent) {
      circleEvent = new this.CircleEvent();
    }

    circleEvent.arc = arc;
    circleEvent.site = cSite;
    circleEvent.x = x + bx;
    circleEvent.y = ycenter + this.sqrt(x * x + y * y);
    circleEvent.ycenter = ycenter;
    arc.circleEvent = circleEvent;
    var predecessor = null,
      node = this.circleEvents.root;

    while(node) {
      if(circleEvent.y < node.y || (circleEvent.y === node.y && circleEvent.x <= node.x)) {
        if(node.rbLeft) {
          node = node.rbLeft;
        } else {
          predecessor = node.rbPrevious;
          break;
        }
      } else {
        if(node.rbRight) {
          node = node.rbRight;
        } else {
          predecessor = node;
          break;
        }
      }
    }

    this.circleEvents.rbInsertSuccessor(predecessor, circleEvent);

    if(!predecessor) {
      this.firstCircleEvent = circleEvent;
    }
  }

  detachCircleEvent(arc) {
    var circleEvent = arc.circleEvent;

    if(circleEvent) {
      if(!circleEvent.rbPrevious) {
        this.firstCircleEvent = circleEvent.rbNext;
      }

      this.circleEvents.rbRemoveNode(circleEvent);
      this.circleEventJunkyard.push(circleEvent);
      arc.circleEvent = null;
    }
  }

  connectEdge(edge, bbox) {
    var vb = edge.vb;

    if(!!vb) {
      return true;
    }

    var va = edge.va,
      xl = bbox.xl,
      xr = bbox.xr,
      yt = bbox.yt,
      yb = bbox.yb,
      lSite = edge.lSite,
      rSite = edge.rSite,
      lx = lSite.x,
      ly = lSite.y,
      rx = rSite.x,
      ry = rSite.y,
      fx = (lx + rx) / 2,
      fy = (ly + ry) / 2,
      fm,
      fb;
    this.cells[lSite.voronoiId].closeMe = true;
    this.cells[rSite.voronoiId].closeMe = true;

    if(ry !== ly) {
      fm = (lx - rx) / (ry - ly);
      fb = fy - fm * fx;
    }

    if(fm === undefined) {
      if(fx < xl || fx >= xr) {
        return false;
      }

      if(lx > rx) {
        if(!va || va.y < yt) {
          va = this.createVertex(fx, yt);
        } else if(va.y >= yb) {
          return false;
        }

        vb = this.createVertex(fx, yb);
      } else {
        if(!va || va.y > yb) {
          va = this.createVertex(fx, yb);
        } else if(va.y < yt) {
          return false;
        }

        vb = this.createVertex(fx, yt);
      }
    } else if(fm < -1 || fm > 1) {
      if(lx > rx) {
        if(!va || va.y < yt) {
          va = this.createVertex((yt - fb) / fm, yt);
        } else if(va.y >= yb) {
          return false;
        }

        vb = this.createVertex((yb - fb) / fm, yb);
      } else {
        if(!va || va.y > yb) {
          va = this.createVertex((yb - fb) / fm, yb);
        } else if(va.y < yt) {
          return false;
        }

        vb = this.createVertex((yt - fb) / fm, yt);
      }
    } else {
      if(ly < ry) {
        if(!va || va.x < xl) {
          va = this.createVertex(xl, fm * xl + fb);
        } else if(va.x >= xr) {
          return false;
        }

        vb = this.createVertex(xr, fm * xr + fb);
      } else {
        if(!va || va.x > xr) {
          va = this.createVertex(xr, fm * xr + fb);
        } else if(va.x < xl) {
          return false;
        }

        vb = this.createVertex(xl, fm * xl + fb);
      }
    }

    edge.va = va;
    edge.vb = vb;
    return true;
  }

  clipEdge(edge, bbox) {
    var ax = edge.va.x,
      ay = edge.va.y,
      bx = edge.vb.x,
      by = edge.vb.y,
      t0 = 0,
      t1 = 1,
      dx = bx - ax,
      dy = by - ay;
    var q = ax - bbox.xl;

    if(dx === 0 && q < 0) {
      return false;
    }

    var r = -q / dx;

    if(dx < 0) {
      if(r < t0) {
        return false;
      }

      if(r < t1) {
        t1 = r;
      }
    } else if(dx > 0) {
      if(r > t1) {
        return false;
      }

      if(r > t0) {
        t0 = r;
      }
    }

    q = bbox.xr - ax;

    if(dx === 0 && q < 0) {
      return false;
    }

    r = q / dx;

    if(dx < 0) {
      if(r > t1) {
        return false;
      }

      if(r > t0) {
        t0 = r;
      }
    } else if(dx > 0) {
      if(r < t0) {
        return false;
      }

      if(r < t1) {
        t1 = r;
      }
    }

    q = ay - bbox.yt;

    if(dy === 0 && q < 0) {
      return false;
    }

    r = -q / dy;

    if(dy < 0) {
      if(r < t0) {
        return false;
      }

      if(r < t1) {
        t1 = r;
      }
    } else if(dy > 0) {
      if(r > t1) {
        return false;
      }

      if(r > t0) {
        t0 = r;
      }
    }

    q = bbox.yb - ay;

    if(dy === 0 && q < 0) {
      return false;
    }

    r = q / dy;

    if(dy < 0) {
      if(r > t1) {
        return false;
      }

      if(r > t0) {
        t0 = r;
      }
    } else if(dy > 0) {
      if(r < t0) {
        return false;
      }

      if(r < t1) {
        t1 = r;
      }
    }

    if(t0 > 0) {
      edge.va = this.createVertex(ax + t0 * dx, ay + t0 * dy);
    }

    if(t1 < 1) {
      edge.vb = this.createVertex(ax + t1 * dx, ay + t1 * dy);
    }

    if(t0 > 0 || t1 < 1) {
      this.cells[edge.lSite.voronoiId].closeMe = true;
      this.cells[edge.rSite.voronoiId].closeMe = true;
    }

    return true;
  }

  clipEdges(bbox) {
    var edges = this.edges,
      iEdge = edges.length,
      edge,
      abs_fn = Math.abs;

    while(iEdge--) {
      edge = edges.iEdge;

      if(!this.connectEdge(edge, bbox) || !this.clipEdge(edge, bbox) || (abs_fn(edge.va.x - edge.vb.x) < 1e-9 && abs_fn(edge.va.y - edge.vb.y) < 1e-9)) {
        edge.va = edge.vb = null;
        edges.splice(iEdge, 1);
      }
    }
  }

  closeCells(bbox) {
    var xl = bbox.xl,
      xr = bbox.xr,
      yt = bbox.yt,
      yb = bbox.yb,
      cells = this.cells,
      iCell = cells.length,
      cell,
      iLeft,
      halfedges,
      nHalfedges,
      edge,
      va,
      vb,
      vz,
      lastBorderSegment,
      abs_fn = Math.abs;

    while(iCell--) {
      cell = cells.iCell;

      if(!cell.prepareHalfedges()) {
        continue;
      }

      if(!cell.closeMe) {
        continue;
      }

      halfedges = cell.halfedges;
      nHalfedges = halfedges.length;
      iLeft = 0;

      while(iLeft < nHalfedges) {
        va = halfedges.iLeft.getEndpoint();
        vz = halfedges[(iLeft + 1) % nHalfedges].getStartpoint();

        if(abs_fn(va.x - vz.x) >= 1e-9 || abs_fn(va.y - vz.y) >= 1e-9) {
          switch (true) {
            case this.equalWithEpsilon(va.x, xl) && this.lessThanWithEpsilon(va.y, yb):
              lastBorderSegment = this.equalWithEpsilon(vz.x, xl);
              vb = this.createVertex(xl, lastBorderSegment ? vz.y : yb);
              edge = this.createBorderEdge(cell.site, va, vb);
              iLeft++;
              halfedges.splice(iLeft, 0, this.createHalfedge(edge, cell.site, null));
              nHalfedges++;

              if(lastBorderSegment) {
                break;
              }

              va = vb;

            case this.equalWithEpsilon(va.y, yb) && this.lessThanWithEpsilon(va.x, xr):
              lastBorderSegment = this.equalWithEpsilon(vz.y, yb);
              vb = this.createVertex(lastBorderSegment ? vz.x : xr, yb);
              edge = this.createBorderEdge(cell.site, va, vb);
              iLeft++;
              halfedges.splice(iLeft, 0, this.createHalfedge(edge, cell.site, null));
              nHalfedges++;

              if(lastBorderSegment) {
                break;
              }

              va = vb;

            case this.equalWithEpsilon(va.x, xr) && this.greaterThanWithEpsilon(va.y, yt):
              lastBorderSegment = this.equalWithEpsilon(vz.x, xr);
              vb = this.createVertex(xr, lastBorderSegment ? vz.y : yt);
              edge = this.createBorderEdge(cell.site, va, vb);
              iLeft++;
              halfedges.splice(iLeft, 0, this.createHalfedge(edge, cell.site, null));
              nHalfedges++;

              if(lastBorderSegment) {
                break;
              }

              va = vb;

            case this.equalWithEpsilon(va.y, yt) && this.greaterThanWithEpsilon(va.x, xl):
              lastBorderSegment = this.equalWithEpsilon(vz.y, yt);
              vb = this.createVertex(lastBorderSegment ? vz.x : xl, yt);
              edge = this.createBorderEdge(cell.site, va, vb);
              iLeft++;
              halfedges.splice(iLeft, 0, this.createHalfedge(edge, cell.site, null));
              nHalfedges++;

              if(lastBorderSegment) {
                break;
              }

              va = vb;
              lastBorderSegment = this.equalWithEpsilon(vz.x, xl);
              vb = this.createVertex(xl, lastBorderSegment ? vz.y : yb);
              edge = this.createBorderEdge(cell.site, va, vb);
              iLeft++;
              halfedges.splice(iLeft, 0, this.createHalfedge(edge, cell.site, null));
              nHalfedges++;

              if(lastBorderSegment) {
                break;
              }

              va = vb;
              lastBorderSegment = this.equalWithEpsilon(vz.y, yb);
              vb = this.createVertex(lastBorderSegment ? vz.x : xr, yb);
              edge = this.createBorderEdge(cell.site, va, vb);
              iLeft++;
              halfedges.splice(iLeft, 0, this.createHalfedge(edge, cell.site, null));
              nHalfedges++;

              if(lastBorderSegment) {
                break;
              }

              va = vb;
              lastBorderSegment = this.equalWithEpsilon(vz.x, xr);
              vb = this.createVertex(xr, lastBorderSegment ? vz.y : yt);
              edge = this.createBorderEdge(cell.site, va, vb);
              iLeft++;
              halfedges.splice(iLeft, 0, this.createHalfedge(edge, cell.site, null));
              nHalfedges++;

              if(lastBorderSegment) {
                break;
              }

            default:
              throw 'Voronoi.closeCells() > this makes no sense!';
          }
        }

        iLeft++;
      }

      cell.closeMe = false;
    }
  }

  dumpBeachline(y) {
    console.log('Voronoi.dumpBeachline(%f) > Beachsections, from left to right:', y);

    if(!this.beachline) {
      console.log('  None');
    } else {
      var bs = this.beachline.getFirst(this.beachline.root);

      while(bs) {
        console.log('  site %d: xl: %f, xr: %f', bs.site.voronoiId, this.leftBreakPoint(bs, y), this.rightBreakPoint(bs, y));
        bs = bs.rbNext;
      }
    }
  }

  quantizeSites(sites) {
    var ε = this.ε,
      n = sites.length,
      site;

    while(n--) {
      site = sites.n;
      site.x = Math.floor(site.x / ε) * ε;
      site.y = Math.floor(site.y / ε) * ε;
    }
  }

  recycle(diagram) {
    if(diagram) {
      if(diagram instanceof this.Diagram) {
        this.toRecycle = diagram;
      } else {
        throw 'Voronoi.recycleDiagram() > Need a Diagram object.';
      }
    }
  }

  compute(sites, bbox) {
    var startTime = new Date();
    this.reset();

    if(this.toRecycle) {
      this.vertexJunkyard = this.vertexJunkyard.concat(this.toRecycle.vertices);
      this.edgeJunkyard = this.edgeJunkyard.concat(this.toRecycle.edges);
      this.cellJunkyard = this.cellJunkyard.concat(this.toRecycle.cells);
      this.toRecycle = null;
    }

    var siteEvents = sites.slice(0);

    siteEvents.sort(function (a, b) {
      var r = b.y - a.y;

      if(r) {
        return r;
      }

      return b.x - a.x;
    });

    var site = siteEvents.pop(),
      siteid = 0,
      xsitex,
      xsitey,
      cells = this.cells,
      circle;

    for(;;) {
      circle = this.firstCircleEvent;

      if(site && (!circle || site.y < circle.y || (site.y === circle.y && site.x < circle.x))) {
        if(site.x !== xsitex || site.y !== xsitey) {
          cells.siteid = this.createCell(site);
          site.voronoiId = siteid++;
          this.addBeachsection(site);
          xsitey = site.y;
          xsitex = site.x;
        }

        site = siteEvents.pop();
      } else if(circle) {
        this.removeBeachsection(circle.arc);
      } else {
        break;
      }
    }

    this.clipEdges(bbox);
    this.closeCells(bbox);
    var stopTime = new Date();
    var diagram = new this.Diagram();
    diagram.cells = this.cells;
    diagram.edges = this.edges;
    diagram.vertices = this.vertices;
    diagram.execTime = stopTime.getTime() - startTime.getTime();
    this.reset();
    return diagram;
  }

  createCell(site) {
    var cell = this.cellJunkyard.pop();

    if(cell) {
      return cell.init(site);
    }

    return new this.Cell(site);
  }

  createHalfedge(edge, lSite, rSite) {
    return new this.Halfedge(edge, lSite, rSite);
  }
}

export class Diagram {
  constructor(site) {
    this.site = site;
  }
}

export class RBTree {
  constructor() {
    this.root = null;
  }

  rbInsertSuccessor(node, successor) {
    var parent;

    if(node) {
      successor.rbPrevious = node;
      successor.rbNext = node.rbNext;

      if(node.rbNext) {
        node.rbNext.rbPrevious = successor;
      }

      node.rbNext = successor;

      if(node.rbRight) {
        node = node.rbRight;

        while(node.rbLeft) {
          node = node.rbLeft;
        }

        node.rbLeft = successor;
      } else {
        node.rbRight = successor;
      }

      parent = node;
    } else if(this.root) {
      node = this.getFirst(this.root);
      successor.rbPrevious = null;
      successor.rbNext = node;
      node.rbPrevious = successor;
      node.rbLeft = successor;
      parent = node;
    } else {
      successor.rbPrevious = successor.rbNext = null;
      this.root = successor;
      parent = null;
    }

    successor.rbLeft = successor.rbRight = null;
    successor.rbParent = parent;
    successor.rbRed = true;
    var grandpa, uncle;
    node = successor;

    while(parent && parent.rbRed) {
      grandpa = parent.rbParent;

      if(parent === grandpa.rbLeft) {
        uncle = grandpa.rbRight;

        if(uncle && uncle.rbRed) {
          parent.rbRed = uncle.rbRed = false;
          grandpa.rbRed = true;
          node = grandpa;
        } else {
          if(node === parent.rbRight) {
            this.rbRotateLeft(parent);
            node = parent;
            parent = node.rbParent;
          }

          parent.rbRed = false;
          grandpa.rbRed = true;
          this.rbRotateRight(grandpa);
        }
      } else {
        uncle = grandpa.rbLeft;

        if(uncle && uncle.rbRed) {
          parent.rbRed = uncle.rbRed = false;
          grandpa.rbRed = true;
          node = grandpa;
        } else {
          if(node === parent.rbLeft) {
            this.rbRotateRight(parent);
            node = parent;
            parent = node.rbParent;
          }

          parent.rbRed = false;
          grandpa.rbRed = true;
          this.rbRotateLeft(grandpa);
        }
      }

      parent = node.rbParent;
    }

    this.root.rbRed = false;
  }

  rbRemoveNode(node) {
    if(node.rbNext) {
      node.rbNext.rbPrevious = node.rbPrevious;
    }

    if(node.rbPrevious) {
      node.rbPrevious.rbNext = node.rbNext;
    }

    node.rbNext = node.rbPrevious = null;
    var parent = node.rbParent,
      left = node.rbLeft,
      right = node.rbRight,
      next;

    if(!left) {
      next = right;
    } else if(!right) {
      next = left;
    } else {
      next = this.getFirst(right);
    }

    if(parent) {
      if(parent.rbLeft === node) {
        parent.rbLeft = next;
      } else {
        parent.rbRight = next;
      }
    } else {
      this.root = next;
    }

    var isRed;

    if(left && right) {
      isRed = next.rbRed;
      next.rbRed = node.rbRed;
      next.rbLeft = left;
      left.rbParent = next;

      if(next !== right) {
        parent = next.rbParent;
        next.rbParent = node.rbParent;
        node = next.rbRight;
        parent.rbLeft = node;
        next.rbRight = right;
        right.rbParent = next;
      } else {
        next.rbParent = parent;
        parent = next;
        node = next.rbRight;
      }
    } else {
      isRed = node.rbRed;
      node = next;
    }

    if(node) {
      node.rbParent = parent;
    }

    if(isRed) {
      return;
    }

    if(node && node.rbRed) {
      node.rbRed = false;
      return;
    }

    var sibling;

    do {
      if(node === this.root) {
        break;
      }

      if(node === parent.rbLeft) {
        sibling = parent.rbRight;

        if(sibling.rbRed) {
          sibling.rbRed = false;
          parent.rbRed = true;
          this.rbRotateLeft(parent);
          sibling = parent.rbRight;
        }

        if((sibling.rbLeft && sibling.rbLeft.rbRed) || (sibling.rbRight && sibling.rbRight.rbRed)) {
          if(!sibling.rbRight || !sibling.rbRight.rbRed) {
            sibling.rbLeft.rbRed = false;
            sibling.rbRed = true;
            this.rbRotateRight(sibling);
            sibling = parent.rbRight;
          }

          sibling.rbRed = parent.rbRed;
          parent.rbRed = sibling.rbRight.rbRed = false;
          this.rbRotateLeft(parent);
          node = this.root;
          break;
        }
      } else {
        sibling = parent.rbLeft;

        if(sibling.rbRed) {
          sibling.rbRed = false;
          parent.rbRed = true;
          this.rbRotateRight(parent);
          sibling = parent.rbLeft;
        }

        if((sibling.rbLeft && sibling.rbLeft.rbRed) || (sibling.rbRight && sibling.rbRight.rbRed)) {
          if(!sibling.rbLeft || !sibling.rbLeft.rbRed) {
            sibling.rbRight.rbRed = false;
            sibling.rbRed = true;
            this.rbRotateLeft(sibling);
            sibling = parent.rbLeft;
          }

          sibling.rbRed = parent.rbRed;
          parent.rbRed = sibling.rbLeft.rbRed = false;
          this.rbRotateRight(parent);
          node = this.root;
          break;
        }
      }

      sibling.rbRed = true;
      node = parent;
      parent = parent.rbParent;
    } while(!node.rbRed);

    if(node) {
      node.rbRed = false;
    }
  }

  rbRotateLeft(node) {
    var p = node,
      q = node.rbRight,
      parent = p.rbParent;

    if(parent) {
      if(parent.rbLeft === p) {
        parent.rbLeft = q;
      } else {
        parent.rbRight = q;
      }
    } else {
      this.root = q;
    }

    q.rbParent = parent;
    p.rbParent = q;
    p.rbRight = q.rbLeft;

    if(p.rbRight) {
      p.rbRight.rbParent = p;
    }

    q.rbLeft = p;
  }

  rbRotateRight(node) {
    var p = node,
      q = node.rbLeft,
      parent = p.rbParent;

    if(parent) {
      if(parent.rbLeft === p) {
        parent.rbLeft = q;
      } else {
        parent.rbRight = q;
      }
    } else {
      this.root = q;
    }

    q.rbParent = parent;
    p.rbParent = q;
    p.rbLeft = q.rbRight;

    if(p.rbLeft) {
      p.rbLeft.rbParent = p;
    }

    q.rbRight = p;
  }

  getFirst(node) {
    while(node.rbLeft) {
      node = node.rbLeft;
    }

    return node;
  }

  getLast(node) {
    while(node.rbRight) {
      node = node.rbRight;
    }

    return node;
  }
}

export class Cell {
  constructor(site) {
    this.site = site;
    this.halfedges = [];
    this.closeMe = false;
  }

  init(site) {
    this.site = site;
    this.halfedges = [];
    this.closeMe = false;
    return this;
  }

  prepareHalfedges() {
    var halfedges = this.halfedges,
      iHalfedge = halfedges.length,
      edge;

    while(iHalfedge--) {
      edge = halfedges.iHalfedge.edge;

      if(!edge.vb || !edge.va) {
        halfedges.splice(iHalfedge, 1);
      }
    }

    halfedges.sort(function (a, b) {
      return b.angle - a.angle;
    });

    return halfedges.length;
  }

  getNeighborIds() {
    var neighbors = [],
      iHalfedge = this.halfedges.length,
      edge;

    while(iHalfedge--) {
      edge = this.halfedges.iHalfedge.edge;

      if(edge.lSite !== null && edge.lSite.voronoiId != this.site.voronoiId) {
        neighbors.push(edge.lSite.voronoiId);
      } else if(edge.rSite !== null && edge.rSite.voronoiId != this.site.voronoiId) {
        neighbors.push(edge.rSite.voronoiId);
      }
    }

    return neighbors;
  }

  getBbox() {
    var halfedges = this.halfedges,
      iHalfedge = halfedges.length,
      xmin = Infinity,
      ymin = Infinity,
      xmax = -Infinity,
      ymax = -Infinity,
      v,
      vx,
      vy;

    while(iHalfedge--) {
      v = halfedges.iHalfedge.getStartpoint();
      vx = v.x;
      vy = v.y;

      if(vx < xmin) {
        xmin = vx;
      }

      if(vy < ymin) {
        ymin = vy;
      }

      if(vx > xmax) {
        xmax = vx;
      }

      if(vy > ymax) {
        ymax = vy;
      }
    }

    return { x: xmin, y: ymin, width: xmax - xmin, height: ymax - ymin };
  }

  pointIntersection(x, y) {
    var halfedges = this.halfedges,
      iHalfedge = halfedges.length,
      halfedge,
      p0,
      p1,
      r;

    while(iHalfedge--) {
      halfedge = halfedges.iHalfedge;
      p0 = halfedge.getStartpoint();
      p1 = halfedge.getEndpoint();
      r = (y - p0.y) * (p1.x - p0.x) - (x - p0.x) * (p1.y - p0.y);

      if(!r) {
        return 0;
      }

      if(r > 0) {
        return -1;
      }
    }

    return 1;
  }
}

export class Vertex {
  constructor(x, y) {
    this.x = x;
    this.y = y;
  }
}

export class Edge {
  constructor(lSite, rSite) {
    this.lSite = lSite;
    this.rSite = rSite;
    this.va = this.vb = null;
  }
}

export class Halfedge {
  constructor(edge, lSite, rSite) {
    this.site = lSite;
    this.edge = edge;

    if(rSite) {
      this.angle = Math.atan2(rSite.y - lSite.y, rSite.x - lSite.x);
    } else {
      var va = edge.va,
        vb = edge.vb;
      this.angle = edge.lSite === lSite ? Math.atan2(vb.x - va.x, va.y - vb.y) : Math.atan2(va.x - vb.x, vb.y - va.y);
    }
  }

  getStartpoint() {
    return this.edge.lSite === this.site ? this.edge.va : this.edge.vb;
  }

  getEndpoint() {
    return this.edge.lSite === this.site ? this.edge.vb : this.edge.va;
  }
}
export default Voronoi;

/*
 * concatenanted lib/geom/simplify.js
 */

function getSqDist(p1, p2) {
  var dx = p1[0] - p2[0],
    dy = p1[1] - p2[1];
  return dx * dx + dy * dy;
}

function getSqSegDist(p, p1, p2) {
  var x = p1[0],
    y = p1[1],
    dx = p2[0] - x,
    dy = p2[1] - y,
    t;

  if(dx !== 0 || dy !== 0) {
    t = ((p[0] - x) * dx + (p[1] - y) * dy) / (dx * dx + dy * dy);

    if(t > 1) {
      x = p2[0];
      y = p2[1];
    } else if(t > 0) {
      x += dx * t;
      y += dy * t;
    }
  }

  dx = p[0] - x;
  dy = p[1] - y;
  return dx * dx + dy * dy;
}

export function simplifyRadialDist(points, sqTolerance) {
  var prevPoint = [points[0], points[1]],
    newPoints = prevPoint,
    i = 2,
    n = points.length,
    point;

  for(i; i < n; i += 2) {
    point = [points.i, points[i + 1]];

    if(getSqDist(point, prevPoint) > sqTolerance) {
      newPoints.push(point[0], point[1]);
      prevPoint = point;
    }
  }

  if(prevPoint !== point) {
    newPoints.push(point[0], point[1]);
  }

  return newPoints;
}

export function simplifyDPStep(points, first, last, sqTolerance, simplified) {
  var maxSqDist = sqTolerance,
    i = first + 2,
    sqDist,
    index;

  for(i; i < last; i += 2) {
    sqDist = getSqSegDist([points.i, points[i + 1]], [points.first, points[first + 1]], [points.last, points[last + 1]]);

    if(sqDist > maxSqDist) {
      index = i;
      maxSqDist = sqDist;
    }
  }

  if(maxSqDist > sqTolerance) {
    if(index - first > 1) simplifyDPStep(points, first, index, sqTolerance, simplified);
    simplified.push(points.index, points[index + 1]);
    if(last - index > 1) simplifyDPStep(points, index, last, sqTolerance, simplified);
  }
}

export function simplifyDouglasPeucker(points, sqTolerance) {
  var last = points.length - 2;
  var simplified = [points[0], points[1]];
  simplifyDPStep(points, 0, last, sqTolerance, simplified);
  simplified.push(points.last, points[last + 1]);
  return simplified;
}

export function simplify(points, tolerance, highestQuality) {
  if(points.length <= 4) return points;
  var sqTolerance = tolerance !== undefined ? tolerance * tolerance : 1;
  points = highestQuality ? points : simplifyRadialDist(points, sqTolerance);
  points = simplifyDouglasPeucker(points, sqTolerance);
  return points;
}
export default simplify;

/*
 * concatenanted lib/geom/simplify.js
 */

function getSqDist(p1, p2) {
  var dx = p1[0] - p2[0],
    dy = p1[1] - p2[1];
  return dx * dx + dy * dy;
}

function getSqSegDist(p, p1, p2) {
  var x = p1[0],
    y = p1[1],
    dx = p2[0] - x,
    dy = p2[1] - y,
    t;

  if(dx !== 0 || dy !== 0) {
    t = ((p[0] - x) * dx + (p[1] - y) * dy) / (dx * dx + dy * dy);

    if(t > 1) {
      x = p2[0];
      y = p2[1];
    } else if(t > 0) {
      x += dx * t;
      y += dy * t;
    }
  }

  dx = p[0] - x;
  dy = p[1] - y;
  return dx * dx + dy * dy;
}

export function simplifyRadialDist(points, sqTolerance) {
  var prevPoint = [points[0], points[1]],
    newPoints = prevPoint,
    i = 2,
    n = points.length,
    point;

  for(i; i < n; i += 2) {
    point = [points.i, points[i + 1]];

    if(getSqDist(point, prevPoint) > sqTolerance) {
      newPoints.push(point[0], point[1]);
      prevPoint = point;
    }
  }

  if(prevPoint !== point) {
    newPoints.push(point[0], point[1]);
  }

  return newPoints;
}

export function simplifyDPStep(points, first, last, sqTolerance, simplified) {
  var maxSqDist = sqTolerance,
    i = first + 2,
    sqDist,
    index;

  for(i; i < last; i += 2) {
    sqDist = getSqSegDist([points.i, points[i + 1]], [points.first, points[first + 1]], [points.last, points[last + 1]]);

    if(sqDist > maxSqDist) {
      index = i;
      maxSqDist = sqDist;
    }
  }

  if(maxSqDist > sqTolerance) {
    if(index - first > 1) simplifyDPStep(points, first, index, sqTolerance, simplified);
    simplified.push(points.index, points[index + 1]);
    if(last - index > 1) simplifyDPStep(points, index, last, sqTolerance, simplified);
  }
}

export function simplifyDouglasPeucker(points, sqTolerance) {
  var last = points.length - 2;
  var simplified = [points[0], points[1]];
  simplifyDPStep(points, 0, last, sqTolerance, simplified);
  simplified.push(points.last, points[last + 1]);
  return simplified;
}

export function simplify(points, tolerance, highestQuality) {
  if(points.length <= 4) return points;
  var sqTolerance = tolerance !== undefined ? tolerance * tolerance : 1;
  points = highestQuality ? points : simplifyRadialDist(points, sqTolerance);
  points = simplifyDouglasPeucker(points, sqTolerance);
  return points;
}
export default simplify;

/*
 * concatenanted lib/geom.js
 */

export default {
  Align,
  Anchor,
  BBox,
  Graph,
  Intersection,
  isLine,
  isMatrix,
  isPoint,
  isRect,
  isSize,
  isTRBL,
  Line,
  LineList,
  Matrix,
  MatrixTransformation,
  Point,
  PointList,
  PolygonFinder,
  Polyline,
  Rect,
  Rotation,
  Scaling,
  Size,
  SweepLineClass,
  Transformation,
  TransformationList,
  Translation,
  TRBL,
  Vector,
  Polygon,
  ImmutableLine,
  ImmutableMatrix,
  ImmutableMatrixTransformation,
  ImmutablePoint,
  ImmutablePointList,
  ImmutableRect,
  ImmutableRotation,
  ImmutableScaling,
  ImmutableSize,
  ImmutableTransformation,
  ImmutableTransformationList,
  ImmutableTranslation,
  ImmutableTRBL,
  simplifyRadialDist,
  simplifyDPStep,
  simplifyDouglasPeucker,
  simplify
};
