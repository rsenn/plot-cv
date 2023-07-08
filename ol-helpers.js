import { Layer as HTMLLayer } from './lib/dom/layer.js';
import { Fragment } from './lib/dom/preactComponent.js';
import { h } from './lib/dom/preactComponent.js';
import { toChildArray } from './lib/dom/preactComponent.js';
import { ArrayFacade } from './lib/misc.js';
import { define } from './lib/misc.js';
import { isInstanceOf } from './lib/misc.js';
import { Feature } from './lib/ol.js';
import { LineString } from './lib/ol.js';
import { Link } from './lib/ol.js';
import { OLMap } from './lib/ol.js';
import { Overlay } from './lib/ol.js';
import { Point } from './lib/ol.js';
import { TileLayer } from './lib/ol.js';
import { transform } from './lib/ol.js';
import { VectorLayer } from './lib/ol.js';
import { VectorSource } from './lib/ol.js';
import { View } from './lib/ol.js';
import { XYZ } from './lib/ol.js';
import { ZoomSlider } from './lib/ol.js';
import { ObjectWrapper } from './object-helpers.js';
import { Zoom } from './openlayers/src/ol/control.js';
import { add } from './openlayers/src/ol/coordinate.js';
import { distance } from './openlayers/src/ol/coordinate.js';
import { parseGPSLocation } from './string-helpers.js';
import LayerSwitcher /* , { BaseLayerOptions, GroupLayerOptions }*/ from './lib/ol-layerswitcher.js';

export function TransformCoordinates(...args) {
  if(args.length == 2) return transform(args, 'EPSG:4326', 'EPSG:3857');
  if(args.length == 4) {
    let extent = [args.splice(0, 2), args.splice(0, 2)];
    return extent.reduce((acc, coord) => acc.concat(TransformCoordinates(...coord)), []);
  }

  if(typeof args[0] == 'string') return TransformCoordinates(args[0].split(',').map(n => +n));
}

export function ParseCoordinates(str) {
  return new Coordinate(...parseGPSLocation(str).reverse(), 'EPSG:4326');
  // return transform(parseGPSLocation(str) ?? str, 'EPSG:4326', 'EPSG:3857');
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
      let [properties, style, position] = args.length == 1 ? [, , args[0]] : args;
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

  get length() {
    const { source } = this;
    return source.getFeatures().length;
  }

  add(...items) {
    const { source } = this;

    for(let item of items) {
      let pin = Pin.from(item);
      /* console.log('Markers.add', { pin });
      console.log('pin.feature', pin.feature);*/
      source.addFeature(pin.feature);
    }
  }
  remove(...items) {
    const { source } = this;

    for(let item of items) {
      let pin = Pin.from(item);
      source.removeFeature(pin.feature);
    }
  }
}

export class Popup {
  static from = ObjectWrapper((...args) => new Popup(...args), Popup.prototype);

  static create(content, offset = [0, 0], position, positioning = 'center-center') {
    let layer = new HTMLLayer('div', { class: 'ol-popup' });
    let { elm: element } = layer;

    let component = h(Fragment, {}, [
      h('a', {
        href: '#',
        id: 'popup-closer',
        class: 'ol-popup-closer'
      }),
      h(
        'div',
        {
          class: 'popup-content'
        },
        toChildArray(content)
      )
    ]);

    layer.render(component);

    let overlay = new Overlay({ element, offset, position, positioning });

    if(globalThis.map) globalThis.map.addOverlay(overlay);

    return Object.assign(Popup.from(overlay), { layer, component, content });
  }

  get overlay() {
    return Popup.from.unwrap(this);
  }

  get position() {
    const { overlay } = this;
    return overlay?.getPosition();
  }
  get position() {
    const { overlay } = this;
    return overlay?.getPosition();
  }
}

export class OpenlayersMap {
  //  static from = ObjectWrapper((...args) => OpenlayersMap.create(...args), OpenlayersMap.prototype);

  static create(target = 'mapdiv') {
    let map = new OLMap({
      target,
      controls: [
        new ZoomSlider(),
        new LayerSwitcher({
          reverse: true,
          groupSelectStyle: 'group'
        })
      ],
      layers: [
        new TileLayer({
          title: 'OSM',
          type: 'base',
          visible: false,
          source: new XYZ({
            url: 'https://{a-c}.tile.openstreetmap.org/{z}/{x}/{y}.png'
          })
        }),
        new TileLayer({
          title: 'Satellit',
          type: 'base',
          visible: true,
          source: new XYZ({
            url: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
            maxZoom: 19
          })
        }),
        new TileLayer({
          title: 'Strassenkarte',
          type: 'base',
          visible: false,
          source: new XYZ({
            url: 'http://server.arcgisonline.com/ArcGIS/rest/services/World_Street_Map/MapServer/tile/{z}/{y}/{x}',
            maxZoom: 20
          })
        }),
        new TileLayer({
          title: 'Topographie',
          type: 'base',
          visible: false,
          source: new XYZ({
            url: 'http://server.arcgisonline.com/ArcGIS/rest/services/World_Topo_Map/MapServer/tile/{z}/{y}/{x}',
            maxZoom: 20
          })
        }),
        new TileLayer({
          title: 'National Geographic',
          type: 'base',
          visible: false,
          source: new XYZ({
            url: 'http://server.arcgisonline.com/ArcGIS/rest/services/NatGeo_World_Map/MapServer/tile/{z}/{y}/{x}',
            maxZoom: 12
          })
        })
      ],
      view: new View({
        center: transform([7.454281, 46.96453], 'EPSG:4326', 'EPSG:3857'),
        zoom: 11,
        minZoom: 3,
        maxZoom: 20,
        extent: TransformCoordinates([5.9962, 45.8389, 10.5226, 47.8229])
      })
    });
    map.addInteraction(new Link({ animate: true, prefix: 'm' }));

    return map;
  }
}