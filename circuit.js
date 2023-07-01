import { fopen, gets, closeSync } from 'fs';
import { define } from 'util';

export class Comment {
  constructor(s) {
    this.s = s;
  }
  concat(other) {
    return new Comment((this.s ? this.s + '\n' : '') + other);
  }
  toString() {
    return this.s;
  }
  [Symbol.inspect](depth, opts) {
    return `\x1b[1;31mComment\x1b[0m '\x1b[1;32m${this.s.replace(/\n/g, '\\n')}\x1b[0m'`;
  }
}

Comment.prototype[Symbol.toStringTag] = 'Comment';

class Empty {
  constructor(s) {
    this.s = s;
  }
  concat(other) {
    return new Empty((this.s ? this.s + '\n' : '') + other);
  }
  toString() {
    return this.s;
  }
  [Symbol.inspect](depth, opts) {
    return `\x1b[1;31mEmpty\x1b[0m '\x1b[1;32m${this.s.replace(/\n/g, '\\n')}\x1b[0m'`;
  }
}

Empty.prototype[Symbol.toStringTag] = 'Empty';

export class Via {
  constructor(x, y) {
    this.x = x;
    this.y = y;
  }

  add(other) {
    return new Via(other.x + this.x, other.y + this.y);
  }

  toString() {
    const { x, y } = this;
    return `${x},${y}`;
  }
}

Via.prototype[Symbol.toStringTag] = 'Via';

export class Board {
  constructor(w, h) {
    this.width = w;
    this.height = h;
  }
}

Board.prototype[Symbol.toStringTag] = 'Board';

export class Package extends Array {
  constructor(name, pos) {
    super();
    this.name = name;
    for(let p of pos) this.push(p);
  }

  [Symbol.inspect](depth, opts) {
    let pins = '';
    for(let v of this) pins += ` ${v.x},${v.y}`;
    return `\x1b[1;31mPackage\x1b[0m \x1b[1;33m${this.name}\x1b[0m' [${pins} ]`;
  }
}

Package.prototype[Symbol.toStringTag] = 'Package';

export class Component {
  constructor(packageName, pin0AbsPos) {
    this.name = undefined;
    this.packageName = packageName;
    this.pin0AbsPos = pin0AbsPos;

    //define(this, { dontCarePinIdxSet: new Set() });
    Object.assign(this, { dontCarePinIdxSet: new Set() });
  }
}

Component.prototype[Symbol.toStringTag] = 'Component';

export class ConnectionPoint {
  constructor(componentName, pinIdx) {
    this.componentName = componentName;
    this.pinIdx = pinIdx;
  }

  toString() {
    return this.componentName + '.' + (1 + this.pinIdx);
  }
}

ConnectionPoint.prototype[Symbol.toStringTag] = 'ConnectionPoint';

export class Connection {
  constructor(start, end) {
    this.start = start;
    this.end = end;
  }
}

Connection.prototype[Symbol.toStringTag] = 'Connection';

export class Circuit {
  constructor() {
    this.packageToPosMap = new Map();
    this.components = new Map();
    this.connections = [];
    this.board = null;
    this.offset = new Via(0, 0);
    define(this, { elements: [] });
  }
}

export class CircuitFileWriter {
  constructor(circuit) {
    this.circuit = circuit;
  }

  write(output) {
    const { elements, comments } = this.circuit;

    for(let element of elements) {
      let comment = comments.get(element);
      if(comment) output(comment.s + '\n');
      let { name } = element.constructor;
      if(!this['write' + name]) throw new Error(`no method ${'write' + name}`);
      this['write' + name](output, element);
    }
  }

  writeBoard(output, board) {
    output(`board\t${board.width},${board.height}\n`);
  }

  writePackage(output, pkg) {
    let pins = [...pkg].map(via => `${via.x},${via.y}`).join(' ');

    output(`${pkg.name}\t${pins}\n`);
  }

  writeComponent(output, component) {
    let { name, packageName, pin0AbsPos } = component;
    output(`${name.padEnd(9)} ${packageName.padEnd(16)} ${pin0AbsPos}\n`);
  }

  writeConnection(output, connection) {
    let { start, end } = connection;
    output(`${(start + '').padEnd(9)} ${end}\n`);
  }

  writeSet(output, set) {
    let { name } = set;
    output(`${name}\t${[...set]}\n`);
  }

  writeEmpty(output, empty) {
    output(`${empty.s}\n`);
  }
}

export class CircuitFileParser {
  constructor() {
    this.circuit = new Circuit();
    this.offset = new Via(0, 0);
    this.lines = [];
  }

  parse(circuitFilePath) {
    let file = fopen(circuitFilePath);
    let obj,
      line,
      no = 0,
      comments = new Map(),
      comment = new Comment('');
    while((line = gets(file))) {
      line = line.trim();
      line = line.replace(/\s+/g, ' ');
      line = line.replace(/\s*,\s*/g, ',');
      if(!(obj = this.parseLine(line, ++no))) throw new Error(`Parse error on ${circuitFilePath}:${no}`);

      if(obj instanceof Comment) comment = comment.concat(obj);

      this.lines.push(obj);

      if(!(obj instanceof Comment)) {
        comments.set(obj, comment.s != '' ? comment : undefined);
        comment = new Comment('');

        this.circuit.elements.push(obj);
      }
    }

    closeSync(file);

    define(this.circuit, { comments });

    return this.circuit;
  }

  parseLine(line, lineno) {
    let ret;
    if((ret = this.parseCommentOrEmpty(line))) return ret;
    if((ret = this.parseConnection(line))) return ret;
    if((ret = this.parseBoard(line))) return ret;
    if((ret = this.parseOffset(line))) return ret;
    if((ret = this.parsePackage(line))) return ret;
    if((ret = this.parseComponent(line))) return ret;
    if((ret = this.parseDontCare(line))) return ret;

    throw new Error(`Invalid line #${lineno}: '${line}'`);
  }

  parseCommentOrEmpty(line) {
    if(/^\s*#.*$/.test(line)) return new Comment(line);
    if(/^\s*$/.test(line)) return new Empty(line);
  }

  parseBoard(line) {
    let match = /^board (\d+),(\d+)$/.exec(line);
    if(!match) return false;
    let [gridW, gridH] = match.slice(1).map(n => +n);
    let board = new Board(gridW, gridH);
    this.circuit.board = board;
    return board;
  }

  parseOffset(line) {
    let match = /^offset (-?\d+),(-?\d+)$/.exec(line);
    if(!match) return false;
    let offset = new Via(+m[1], +m[2]);
    console.log('parseOffset', { offset });
    this.circuit.offset = offset;
    return offset;
  }

  parsePackage(line) {
    const pkgNameRx = /^(\w+)\s(.*)/,
      pkgSepRx = /\s+/;
    let m;
    if(!(m = pkgNameRx.exec(line))) return false;
    let pkgName = m[1],
      pkgPos = m[2],
      v = [];

    let pos = pkgPos.split(pkgSepRx);
    if(pos.length < 2) return false;

    for(let s of pos) {
      const pkgPosRx = /(-?\d+),(-?\d+)/g;
      if((m = pkgPosRx.exec(s))) {
        let via = new Via(...m.slice(1).map(n => +n));
        v.push(via);
      } else {
        return false;
      }
    }

    //console.log('parsePackage', { pkgName, pkgPos });
    let pkg = new Package(pkgName, v);
    this.circuit.packageToPosMap.set(pkgName, pkg);
    return pkg;
  }

  parseComponent(line) {
    const comFull = /^(\w+) (\w+) ?(\d+),(\d+)$/;
    let m;
    if(!(m = comFull.exec(line))) return false;
    let componentName = m[1],
      packageName = m[2],
      x = +m[3],
      y = +m[4];

    if(!this.circuit.packageToPosMap.has(packageName)) throw new Error(`Unknown package: ${packageName}`);

    let p = new Via(x, y).add(this.offset);
    let i = 0;
    for(let v of this.circuit.packageToPosMap.get(packageName)) {
      if(p.x + v.x < 0 || p.x + v.x >= this.circuit.board.width || p.y + v.y < 0 || p.y + v.y >= this.circuit.board.height)
        throw new Error(`Component pin outside of board: ${componentName}.${i + 1}`);

      ++i;
    }
    let component = new Component(packageName, p);
    this.circuit.components.set(componentName, component);

    //console.log('parseComponent', { componentName, component });
    component.name = componentName;
    return component;
  }

  parseDontCare(line) {
    const fullRx = /^(\w+) (\d+(,|$))+/,
      pinIdxRx = /(\d+)(,|$)/g;
    let m;
    if(!(m = fullRx.exec(line))) return false;

    let componentName = m[1];

    if(!this.circuit.components.has(componentName)) throw new Error(`Unknown component: ${componentName}`);
    let component = this.circuit.components.get(componentName);

    let packagePosVec = this.circuit.packageToPosMap.get(component.packageName);

    while((m = pinIdxRx.exec(line))) {
      let dontCarePinIdx = +m[1];

      if(dontCarePinIdx < 1 || dontCarePinIdx > packagePosVec.length)
        throw new Error(`Invalid "Don't Care" pin number for ${componentName}: ${dontCarePinIdx}. Must be between 1 and ${packagePosVec.length} (including)`);
      component.dontCarePinIdxSet.add(--dontCarePinIdx);
    }
    component.dontCarePinIdxSet.name = componentName;
    //console.log('parseDontCare', { componentName });
    return component.dontCarePinIdxSet;
  }

  parseConnection(line) {
    const comFull = /^(\w+)\.(\d+) (\w+)\.(\d+)$/;
    let m;
    if(!(m = comFull.exec(line))) return false;
    let start = new ConnectionPoint(m[1], +m[2] - 1),
      end = new ConnectionPoint(m[3], +m[4] - 1);

    this.checkConnectionPoint(start);
    this.checkConnectionPoint(end);

    if(start.componentName == end.componentName && start.pinIdx == end.pinIdx) {
      console.log('parseConnection', { start, end });
      return true;
    }

    let connection = new Connection(start, end);
    this.circuit.connections.push(connection);

    return connection;
  }

  checkConnectionPoint(connectionPoint) {
    if(!this.circuit.components.has(connectionPoint.componentName)) throw new Error(`Unknown component: ${connectionPoint.componentName}`);

    let component = this.circuit.components.get(connectionPoint.componentName);
    let packagePosVec = this.circuit.packageToPosMap.get(component.packageName);
    let pinIdx1Base = connectionPoint.pinIdx + 1;
    if(pinIdx1Base < 1 || pinIdx1Base > packagePosVec.length)
      throw new Error(`Invalid pin number for ${connectionPoint.componentName}.${pinIdx1Base}. Must be between 1 and ${packagePosVec.length} (including)`);
    if(component.dontCarePinIdxSet.has(connectionPoint.pinIdx)) throw new Error(`Invalid pin number for {connectionPoint.componentName}.{pinIdx1Base}. Pin has been set as "Don't Care"`);
  }
}
