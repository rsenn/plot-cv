import { Point, Feature, VectorLayer, VectorSource, LineString, transform } from './lib/ol.js';
import { ObjectWrapper, BiDirMap } from './object-helpers.js';
import { define, isObject, isFunction, isInstanceOf, ArrayFacade } from './lib/misc.js';
import { add, closestOnCircle, closestOnSegment, createStringXY, degreesToStringHDMS, format, equals, rotate, scale, squaredDistance, distance, squaredDistanceToSegment, toStringHDMS, toStringXY, wrapX, getWorldsAway } from './openlayers/src/ol/coordinate.js';

export function TransformCoordinates(...args) {
  if(args.length == 2) return transform(args, 'EPSG:4326', 'EPSG:3857');
  if(args.length == 4) {
    let extent = [args.splice(0, 2), args.splice(0, 2)];
    return extent.reduce((acc, coord) => acc.concat(TransformCoordinates(...coord)), []);
  }

  if(typeof args[0] == 'string') return TransformCoordinates(args[0].split(',').map(n => +n));
}

export class Coordinate {
  static obj2ol = new WeakMap();

  static from(arg) {
    let coord;
    try {
      if(arg.getGeometry) arg = arg.getGeometry();
      if(arg.getCoordinates) arg = arg.getCoordinates();
      if(Array.isArray(arg)) coord = new Coordinate(...arg, 'EPSG:3857');
    } catch(e) {
      try {
        coord = new Coordinate(...arg);
      } catch(e) {
        try {
          coord = new Coordinate(arg.lon, arg.lat, arg.type);
        } catch(e) {}
      }
    }
    if(typeof arg == 'object' && arg != null) Coordinate.obj2ol.set(coord, arg);
    return coord;
  }

  constructor(lon, lat, type) {
    this.lon = lon;
    this.lat = lat;

    if(typeof type == 'string') this.type = type;
  }

  get [0]() {
    return this.convertTo('EPSG:3857')[0];
  }
  get [1]() {
    return this.convertTo('EPSG:3857')[1];
  }

  get length() {
    return 2;
  }

  convertTo(destType) {
    return transform([this.lon, this.lat], this.type, destType);
  }

  distanceTo(other) {
    var line = new LineString([[...this], [...other]]);
    return Math.round(line.getLength() * 100) / 100;
  }

  *[Symbol.iterator]() {
    yield* this.convertTo('EPSG:3857');
  }
  get [Symbol.toStringTag]() {
    return `Coordinate${this.type ? '[' + this.type + ']' : ''} ${this.lon},${this.lat}`;
  }
  toString() {
    return `${this.lon},${this.lat}`;
  }
  toLonLat() {
    return this.convertTo('EPSG:4326');
  }
  toPoint() {
    return new Point([...this]);
  }
}

Coordinate.prototype.type = 'EPSG:4326';
Coordinate.prototype.slice = Array.prototype.slice;

export class Pin {
  static from = ObjectWrapper((...args) => new Pin(...args), Pin.prototype);

  static create(...args) {
    let feature;
    if(isInstanceOf(args[0], Feature)) {
      feature = args[0];
    } else {
      const [properties, style, position] = args;
      if(typeof properties == 'string') properties = { name: properties };
      feature = new Feature({ ...properties, geometry: new Point([...position]) });
      feature.setStyle(style);
    }
    return Pin.from(feature);
  }

  get feature() {
    return Pin.from.unwrap(this);
  }

  get position() {
    const { feature } = this;
    return Coordinate.from(feature);
  }

  set position(value) {
    const { feature } = this;
    let point = value.toPoint();
    feature.setGeometry(point);
  }
}

export class Markers extends ArrayFacade {
  static from = ObjectWrapper((...args) => Markers.create(...args), Markers.prototype);

  length = 0;

  static create(map) {
    let source = new VectorSource({ features: [] });
    let layer = new VectorLayer({ source });
    let obj = { source, layer };
    Object.setPrototypeOf(obj, Markers.prototype);
    try {
      map.addLayer(layer);
    } catch(e) {}
    obj = new Proxy(obj, {
      get: (target, prop, receiver) => {
        if(prop == 'length') return source.getFeatures().length;
        if(typeof prop == 'number' || (typeof prop == 'string' && /^\d+$/.test(prop))) return obj.at(+prop);
        return Reflect.get(target, prop, receiver);
      }
    });
    Markers.from.map.set(layer, obj);
    return obj;
  }

  at(pos) {
    const { source } = this;
    const feature = source.getFeatures()[pos];
    return feature ? Pin.from(feature) : undefined;
  }

  add(...items) {
    const { source } = this;

    for(let item of items) {
      let pin = Pin.from(item);
      console.log('Markers.add', { pin });
      console.log('pin.feature', pin.feature);
      source.addFeature(pin.feature);
    }
  }
}
