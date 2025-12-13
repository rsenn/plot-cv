import { CurrentFile, DailyPhase, DateToUnix, PhaseFile, Time, TimeToStr } from './adsb-common.js';
import { memoize } from './lib/misc.js';
import PlainDraggable from './lib/plain-draggable.js';
import { ZoomSlider } from './openlayers/src/ol/control.js';
import Feature from './openlayers/src/ol/Feature.js';
import Geolocation from './openlayers/src/ol/Geolocation.js';
import LineString from './openlayers/src/ol/geom/LineString.js';
import MultiPoint from './openlayers/src/ol/geom/MultiPoint.js';
import Point from './openlayers/src/ol/geom/Point.js';
import Layer from './openlayers/src/ol/layer/Layer.js';
import TileLayer from './openlayers/src/ol/layer/Tile.js';
import VectorLayer from './openlayers/src/ol/layer/Vector.js';
import OLMap from './openlayers/src/ol/Map.js';
import Overlay from './openlayers/src/ol/Overlay.js';
import { addCoordinateTransforms, addProjection, fromLonLat, transform } from './openlayers/src/ol/proj.js';
import Projection from './openlayers/src/ol/proj/Projection.js';
import OSM from './openlayers/src/ol/source/OSM.js';
import VectorSource from './openlayers/src/ol/source/Vector.js';
import XYZ from './openlayers/src/ol/source/XYZ.js';
import { Circle as CircleStyle, Fill, RegularShape, Stroke, Style, Text as TextStyle } from './openlayers/src/ol/style.js';
import Icon from './openlayers/src/ol/style/Icon.js';
import { composeCssTransform } from './openlayers/src/ol/transform.js';
import View from './openlayers/src/ol/View.js';
import extendArray from './quickjs/qjs-modules/lib/extendArray.js';
extendArray(Array.prototype);

let data = (globalThis.data = []);
let center = (globalThis.center = transform([7.454281, 46.96453], 'EPSG:4326', 'EPSG:3857'));
let extent = [5.9962, 45.8389, 10.5226, 47.8229];
let topLeft = [5.9962, 47.8229],
  topRight = [10.5226, 47.8229],
  bottomLeft = [5.9962, 45.8389],
  bottomRight = [10.5226, 45.8389];
let states = (globalThis.states = []);
let phases = (globalThis.phases = new Set());

function GetPhases() {
  return [...phases].sort();
}

function InsertSorted(entries, ...values) {
  let [key] = values[0];
  let at = entries.findIndex(([k, v]) => k > key);

  if(at != -1) entries.splice(at, 0, ...values);
  else entries.push(...values);
}

async function FetchFile(file, done = data => data) {
  let response = await fetch(file);
  let text = await response.text();
  return done(text);
}

async function FetchJSON(file, done = data => data) {
  let text = await FetchFile(file);
  return done(JSON.parse(text));
}

function TransformCoordinates(...args) {
  if(args.length == 2) return transform(args, 'EPSG:4326', 'EPSG:3857');
  if(args.length == 4) {
    let extent = [args.splice(0, 2), args.splice(0, 2)];
    return extent.reduce((acc, coord) => acc.concat(TransformCoordinates(...coord)), []);
  }

  //return [...TransformCoordinates(args.slice(0,2)), ...TransformCoordinates(args.slice(2,4))];
  if(typeof args[0] == 'string') return TransformCoordinates(args[0].split(',').map(n => +n));
}

class Coordinate {
  constructor(lon, lat) {
    this.lon = lon;
    this.lat = lat;
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

  *[Symbol.iterator]() {
    yield* this.convertTo('EPSG:3857');
  }
  get [Symbol.toStringTag]() {
    return `Coordinate ${this.lon},${this.lat}`;
  }
  toString() {
    return `${this.lon},${this.lat}`;
  }
}

Coordinate.prototype.type = 'EPSG:4326';
Coordinate.prototype.slice = Array.prototype.slice;

const cities = {
  bern: new Coordinate(7.4458, 46.95),
  zuerich: new Coordinate(8.545094, 47.373878),
  winterthur: new Coordinate(8.729869, 47.500954),
  genf: new Coordinate(6.143158, 46.204391),
  zug: new Coordinate(8.515495, 47.166168),
  basel: new Coordinate(7.588576, 47.559601),
  winterthur: new Coordinate(8.737565, 47.49995),
};

const tryFunction = (fn, resolve = a => a, reject = () => null) => {
  if(typeof resolve != 'function') {
    let rval = resolve;
    resolve = () => rval;
  }
  if(typeof reject != 'function') {
    let cval = reject;
    reject = () => cval;
  }
  return function(...args) {
    let ret;
    try {
      ret = fn(...args);
    } catch(err) {
      return reject(err, ...args);
    }
    return resolve(ret, ...args);
  };
};

const tryCatch = (fn, resolve = a => a, reject = () => null, ...args) => {
  return tryFunction(fn, resolve, reject)(...args);
};

function Refresh() {
  //map.render();
  //
  source.clear();
  if(states.length) {
    const [time, list] = states.last;
    console.log('refresh', list.length);

    SetTime(time);

    for(let state of list) {
      let obj = Aircraft.fromState(state);
      let style = Aircraft.style(obj.icao24);

      source.addFeature(obj);
    }
  }
}

function Connection(port, onConnect = () => {}) {
  port ??= 12001;
  let ws = (globalThis.ws = new WebSocket('wss://transistorisiert.ch:' + port));
  return Object.assign(ws, {
    sendCommand(cmd, args = []) {
      return ws.sendMessage({ cmd, args });
    },
    sendMessage(obj) {
      return ws.send(JSON.stringify(obj));
    },
    onopen: e => onConnect(ws),
    onerror(e) {
      console.log('ERROR:', e);
    },
    onclose(e) {
      console.log('CLOSED:', e);
    },
    onmessage(e) {
      let response;
      try {
        response = JSON.parse(e.data); //tryCatch(() => JSON.parse(e.data), d=>d, err => err);
        console.log('onmessage', response);

        if(response.type && /^[A-Z]/.test(response.type[0])) {
          console.log('return value', response.value);
          switch (response.type) {
            case 'StatePhases': {
              for(let phase of response.value) {
                phases.add(phase);
              }
              break;
            }
          }
        } else if(response.type == 'list') {
          console.log('times', response.times);
        } else if(response.type == 'update' || 'time' in response) {
          // console.log('update', response.states);
          InsertSorted(states, [response.time, response.states]);
          Refresh();
        } else if(response.type == 'array') {
          let arr = response.array;

          arr = arr.map(([time, obj]) => [new Date(+time * 1e3), obj]);

          console.log('arr', arr);

          data.splice(0, data.length, ...arr.map(([time, states]) => ({ time, states /*: states.map(StateToObject)*/ })));
          if(arr[0]) InsertSorted(states, ...arr);

          console.log('data.length', data.length);
          Refresh();
        } else if(response.type == 'error') {
          console.log('ERROR response', response.error);
        } else {
          throw new Error(`Invalid response: ${e.data}`);
        }
      } catch(error) {
        console.log('onmessage ERROR:', error.message);
        console.log('onmessage ERROR data:', e.data);
        console.log('onmessage ERROR stack:', error.stack);
      }
      console.log('states.length', states.length);
    },
  });
}

function SetFenceColor(color) {
  vector.setStyle(new OpenLayers.Style({ stroke: new OpenLayers.Stroke({ color, width: 3, lineDash: [2, 4] }) }));
}

function SetTimescaleArea(width, offset) {
  let ruler = document.querySelector('.time-ruler');
  let scale = document.querySelector('.time-scale');
  let area = document.querySelector('.time-area');

  scale.style.setProperty('width', `${width}`);
  /*ruler.style.setProperty('width', '100vw');*/
  ruler.scrollTo(offset, 0);
}

function SetTime(t) {
  let str = TimeToStr(t);

  let disp = document.querySelector('.time-display');
  let row = document.querySelector('.time-row');

  disp.innerText = str.split(' ').join(' ');

  row.style.setProperty('transition', '');
  //row.style.setProperty('border', '2px dashed red');
  row.style.setProperty('box-shadow', '0 0 4px red');
  /* disp.style.setProperty('transition', 'all 1s ease-out');
  disp.style.setProperty('border-bottom', 'none');*/
  setTimeout(() => {
    row.style.setProperty('transition', 'all 1s ease-in'); //row.style.setProperty('border', '2px dashed white');
    row.style.setProperty('box-shadow', 'none');
  }, 1000);
}

function FlyTo(location, done = () => {}) {
  console.log('FlyTo', { location, done });

  if(typeof location == 'string') location = cities[location];

  const duration = 2000;
  const zoom = view.getZoom();
  let parts = 2;
  let called = false;
  function callback(complete) {
    --parts;
    if(called) {
      return;
    }
    if(parts === 0 || !complete) {
      called = true;
      done(complete);
    }
  }
  view.animate(
    {
      center: location,
      duration: duration,
    },
    callback,
  );
  view.animate(
    {
      zoom: zoom - 1,
      duration: duration / 2,
    },
    {
      zoom: zoom,
      duration: duration / 2,
    },
    callback,
  );
}

/*
onClick('fly-to-bern', function() {
  FlyTo(bern, function() {});
});
*/
const styles = [
  new Style({
    stroke: new Stroke({
      color: 'blue',
      width: 3,
    }),
    fill: new Fill({
      color: 'rgba(0, 0, 255, 0.1)',
    }),
  }),
  new Style({
    image: new CircleStyle({
      radius: 5,
      fill: new Fill({
        color: 'orange',
      }),
    }),
    geometry: function(feature) {
      // return the coordinates of the first ring of the polygon
      const coordinates = feature.getGeometry().getCoordinates()[0];
      return new MultiPoint(coordinates);
    },
  }),
];

const geojsonObject = {
  type: 'FeatureCollection',
  crs: {
    type: 'name',
    properties: {
      name: 'EPSG:3857',
    },
  },
  features: [
    {
      type: 'Feature',
      geometry: {
        type: 'Polygon',
        coordinates: [
          [
            [25.801, 5.054],
            [25.801, 18.943],
            [44.931, 32.158],
            [44.882, 35.452],
            [25.802, 27.867],
            [25.802, 39.085],
            [33.192, 45.149],
            [32.749, 47.847],
            [23.716, 44.385],
            [14.864, 47.738],
            [14.317, 44.946],
            [17.857, 42.272],
            [21.289, 39.253],
            [21.289, 27.867],
            [3.325, 35.13],
            [3.085, 32.392],
            [21.29, 18.942],
            [21.34, 5.004],
            [21.872, 2.544],
            [23.378, 0.15],
            [25.182, 2.56],
            [25.801, 5.053],
          ].map(([x, y]) => [x * 1e-3, y * 1e-3]),
        ],
      },
    },
    {
      type: 'Feature',
      geometry: {
        type: 'Polygon',
        coordinates: [
          [
            [-2000000, 6e6],
            [-2e6, 8e6],
            [0, 8e6],
            [0, 6e6],
            [-2e6, 6e6],
          ].map(([x, y]) => [x * 1e-3, y * 1e-3]),
        ],
      },
    },
    {
      type: 'Feature',
      geometry: {
        type: 'Polygon',
        coordinates: [
          [
            [500000, 6000000],
            [500000, 7000000],
            [1500000, 7000000],
            [1500000, 6000000],
            [500000, 6000000],
          ],
        ],
      },
    },
    {
      type: 'Feature',
      geometry: {
        type: 'Polygon',
        coordinates: [
          [
            [-2000000, -1000000],
            [-1000000, 1000000],
            [0, -1000000],
            [-2000000, -1000000],
          ],
        ],
      },
    },
  ],
};

const source = (globalThis.source = new VectorSource({
  features: [], // new GeoJSON().readFeatures(geojsonObject)
}));

const vectorLayer = new VectorLayer({
  source: source,
  style: styles,
});

const iconMarkerStyle = new Style({
  image: new Icon({
    src: 'static/svg/plane.svg',
    //size: [100, 100],
    offset: [0, 0],
    opacity: 1,
    scale: 0.35,
    //color: [10, 98, 240, 1]
  }),
});

function CreateMap() {
  const view = new View({
    center,
    zoom: 11,
    minZoom: 3,
    maxZoom: 20,
    extent: TransformCoordinates(extent),
  });

  const positionFeature = new Feature();
  positionFeature.setStyle(
    new Style({
      image: new CircleStyle({
        radius: 6,
        fill: new Fill({
          color: '#3399CC',
        }),
        stroke: new Stroke({
          color: '#fff',
          width: 2,
        }),
      }),
    }),
  );
  let extentVector = [topLeft, topRight, bottomRight, bottomLeft, topLeft].map(a => TransformCoordinates(...a));
  let lineString = new LineString(extentVector);

  let feature = new Feature({
    geometry: lineString,
  });

  let stroke = new Stroke({
    color: '#ffd705',
    width: 4,
    lineDash: [4, 8],
  });
  const vector = new VectorLayer({
    source: new VectorSource({
      features: [feature, positionFeature],
      wrapX: false,
    }),
    style: new Style({
      stroke,
    }),
  });

  const geolocation = new Geolocation({
    // enableHighAccuracy must be set to true to have the heading value.
    trackingOptions: {
      enableHighAccuracy: true,
    },
    projection: view.getProjection(),
  });
  const tileLayer = new TileLayer({
    source: new XYZ({
      url: 'https://{a-c}.tile.openstreetmap.org/{z}/{x}/{y}.png',
    }),
  });
  const rasterLayer = new TileLayer({
    source: new XYZ({
      url: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
      maxZoom: 19,
    }),
  });
  /*  tileLayer.on('postrender', function(event) {
    const vectorContext = getVectorContext(event);
 
    console.log('tileLayer.postrender', event);
    if(states.length) {
      const [time, list] = states.last;
      for(let state of list) {
        let obj = Aircraft.fromState(state);
        let style = Aircraft.style(obj.icao24);
        console.log('obj', { obj, style });

  
        vectorContext.drawFeature(obj, style);
      }
    }
   });*/

  /*const feature2 = new Feature({
  geometry: new Point(fromLonLat([-30, 10])),
  name: 'Fish.2 Island',
});*/

  let map = new OLMap({
    target: 'mapdiv',
    layers: [tileLayer, rasterLayer /*,vector*/],
    view,
  });

  const zoomslider = new ZoomSlider();
  map.addControl(zoomslider);

  Object.assign(globalThis, {
    tileLayer,
    rasterLayer,
    view,
    vector,
    map,
    extentVector,
    lineString,
    feature,
    stroke,
    positionFeature,
  });
  Object.defineProperties(globalThis, {
    zoom: {
      get() {
        return view.getZoom();
      },
      set(value) {
        view.setZoom(value);
      },
    },
  });

  const svgContainer = (globalThis.svgContainer = document.createElement('div'));
  /*const xhr = new XMLHttpRequest();
xhr.open('GET', 'data/world.svg');
xhr.addEventListener('load', function() {
  const svg = xhr.responseXML.documentElement;
  svgContainer.ownerDocument.importNode(svg);
  svgContainer.appendChild(svg);
});
xhr.send();*/
  let el = document.querySelector('.' + map.getAllLayers()[0].getClassName());

  const width = /*el.offsetWidth  ||document.body.clientWidth||*/ window.innerWidth;
  const height = /* el.offsetHeight||document.body.clientHeight||*/ window.innerHeight;

  const svgResolution = 360 / width;
  svgContainer.style.width = width + 'px';
  svgContainer.style.height = height + 'px';
  svgContainer.style.transformOrigin = 'top left';
  svgContainer.className = 'svg-layer';
  console.log('svgResolution', svgResolution);
  map.addLayer(
    new Layer({
      render(frameState) {
        const scale = svgResolution / frameState.viewState.resolution;
        const center = frameState.viewState.center;
        const size = frameState.size;
        const cssTransform = composeCssTransform(size[0] / 2, size[1] / 2, scale, scale, frameState.viewState.rotation, -center[0] / svgResolution - width / 2, center[1] / svgResolution - height / 2);
        svgContainer.style.transform = cssTransform;
        svgContainer.style.opacity = this.getOpacity();
        return svgContainer;
      },
    }),
  );
  map.addLayer(vectorLayer);
  return map;
}

function CreateSlider() {
  let element = document.querySelector('.time-point');
  let scale = document.querySelector('.time-scale');
  console.log('CreateSlider', element);

  let draggable = new PlainDraggable(element, {
    snap: 5,
    onDrag(position) {
      const { left } = position;
      console.log('onDrag', left);
      return true;
      return !!position.snapped; // It is moved only when it is snapped.
    },
  });
  draggable.snap = { step: 40 };
  draggable.containment = { left: 0, top: 20, width: window.offsetWidth, height: 0 };
  draggable.autoScroll = { target: document.querySelector('.time-area') };
  Object.assign(globalThis, { draggable });

  scale.addEventListener('click', e => {
    const { clientX } = e;
    console.log('scale clicked', clientX);
    draggable.left = clientX - element.offsetWidth / 2;
  });
  return draggable;
}

Object.assign(globalThis, {
  center,
  PlainDraggable,
  GetPhases,
  OpenLayers: {
    Map,
    View,
    TileLayer,
    Layer,
    Point,
    Overlay,
    XYZ,
    OSM,
    Feature,
    Projection,
    VectorLayer,
    VectorSource,
    LineString,
    Fill,
    RegularShape,
    Stroke,
    Style,
    ZoomSlider,
    addCoordinateTransforms,
    addProjection,
    transform,
    fromLonLat,
  },
  Connection,
  Time,
  TimeToStr,
  DateToUnix,
  PhaseFile,
  CurrentFile,
  tryCatch,
  tryFunction,
  TransformCoordinates,
  SetFenceColor,
  SetTimescaleArea,
  FlyTo,
  Coordinate,
  cities,
  FetchFile,
  FetchJSON,
});

let planes = (globalThis.planes = []);
let d = Date.parse('2022-04-19T14:59:00Z');

const keys = [
  'icao24',
  'callsign',
  'origin_country',
  'time_position',
  'last_contact',
  'longitude',
  'latitude',
  'baro_altitude',
  'on_ground',
  'velocity',
  'true_track',
  'vertical_rate',
  'sensors',
  'geo_altitude',
  'squawk',
  'spi',
  'position_source',
];

function StateToObject(item) {
  return item.reduce(
    (acc, field, i) => ({
      ...acc,
      [keys[i]]: ['time_position', 'last_contact'].indexOf(keys[i]) != -1 ? new Date(field * 1000) : field,
    }),
    {},
  );
}

CreateMap();

window.addEventListener('load', () => {
  ws = Connection(null, ws => {
    console.log('Connected');
    ws.sendCommand('StatePhases');
  });
  ///CreateSlider();
  SetTime(DateToUnix());
  if(/Android|webOS|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(navigator.userAgent)) {
    for(let q of ['#timeline', '.time-display' /*,'.time-scale','.time-row'*/]) {
      let e = document.querySelector(q);
      //e.style.setProperty('height', '2em');
      e.style.setProperty('font-size', '2em');
    }
  }

  return;

  let filename = PhaseFile(DailyPhase(d));
  fetch(filename).then(response => {
    response.text().then(text => {
      data = globalThis.data = text
        .split(/\n/g)
        .map(line => {
          try {
            let obj = JSON.parse(line);
            obj.states = obj.states.map(item =>
              item.reduce(
                (acc, field, i) => ({
                  ...acc,
                  [keys[i]]: ['time_position', 'last_contact'].indexOf(keys[i]) != -1 ? new Date(field * 1000) : field,
                }),
                {},
              ),
            );
            obj.states.sort((a, b) => b.baro_altitude - a.baro_altitude);

            return obj;
          } catch(e) {
            return null;
          }
        })
        .filter(obj => !!obj);

      data[0].states.forEach(state => new Plane(state));

      console.log('text', text);
    });
  });
});

class Plane extends Overlay {
  constructor(obj = {}) {
    const { longitude, latitude } = obj;
    obj = {
      ...obj,
      element: (() => {
        let div = document.createElement('div');
        div.innerHTML = `<svg xmlns="http://www.w3.org/2000/svg" xml:space="preserve" width="48" height="48"><defs><linearGradient xmlns="http://www.w3.org/2000/svg" id="a" x1="0" x2="0" y1="0" y2="1"><stop offset="0" stop-color="#f00"/><stop offset=".333" stop-color="#ff0"/><stop offset=".5" stop-color="#0f0"/><stop offset=".666" stop-color="#0ff"/><stop offset=".833" stop-color="#00f"/><stop offset="1" stop-color="#f0f"/></linearGradient></defs><path fill="url(#a)" d="M24.39 1.054c.77.852 1.229 2.587 1.411 4 .068 4.567.042 9.327 0 13.889a776.135 776.135 0 0 1 19.13 13.215l-.007 1.644c-.003.902-.019 1.647-.042 1.65l-19.08-7.585v11.218c.646.47 7.185 5.853 7.39 6.064.242.525-.322 2.067-.443 2.698l-9.033-3.462-8.27 3.114-.582.239-.33-1.353c-.221-.927-.293-1.378-.217-1.439.065-.047.978-.74 2.027-1.533 4.718-3.591 4.89-4.085 4.945-4.16V27.867L12.2 31.54c-4.848 1.96-8.832 3.588-8.875 3.59-.08-.006-.315-2.677-.24-2.738 6.486-4.657 11.317-8.303 18.205-13.45-.058-4.09-.256-8.299 0-12.158.094-1.287-.095-2.83.23-3.728.296-.825.969-1.955 1.525-2.548l.333-.357.318.241c.18.13.488.427.696.662z" style="stroke:#000;stroke-width:.5;stroke-miterlimit:4;stroke-dasharray:none"/></svg>`;
        div.style.setProperty('position', 'relative');
        div.style.setProperty('width', '48px');
        div.style.setProperty('height', '48px');
        //div.style.setProperty('border', '1px dotted black');

        return div;
      })(),
      position: transform([longitude, latitude], 'EPSG:4326', 'EPSG:3857'),
      //offset: [-24,-24],
      positioning: 'center-center',
    };
    super(obj);
    Object.assign(this, obj);

    map.addOverlay(this);
    planes.push(this);
  }

  rotate(deg) {
    const { element } = this;
    element.firstElementChild.setAttributeNS(null, 'transform', `rotate(${deg})`);
  }
}

let aircraftMap = (globalThis.aircraftMap = new Map());

class Aircraft extends Feature {
  //static list = memoize(obj => new Aircraft(obj.icao24, [obj.longitude,obj.latitude], obj.true_track), aircraftMap);
  static style = name =>
    new Style({
      image: new Icon({
        anchor: [0.5, 0.5],
        src: 'static/svg/plane.svg',
        crossOrigin: '',

        rotation: Math.PI / 4,
      }),
      text: new TextStyle({
        text: name,
        scale: [0, 0],
        rotation: Math.PI / 4,
        textAlign: 'center',
        textBaseline: 'top',
      }),
    });

  static fromState(state) {
    let obj = StateToObject(state);
    const { icao24, callsign, origin_country, time_position, last_contact, longitude, latitude, baro_altitude, on_ground, velocity, true_track, vertical_rate, sensors, geo_altitude, squawk, spi } =
      obj;

    let aircraft;

    if((aircraft = aircraftMap.get(obj.icao24))) {
      aircraft.position = [longitude, latitude];
    } else {
      aircraft = new Aircraft(icao24, [longitude, latitude], true_track);
      aircraftMap.set(icao24, aircraft);
    }

    aircraft.setId(icao24);

    return aircraft;
  }

  constructor(name, coord, heading = 0) {
    if(!(coord instanceof Coordinate)) coord = new Coordinate(...coord);

    super({ geometry: new Point(coord), name });

    let scale = 0.8;
    if(/Android|webOS|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(navigator.userAgent)) {
      scale = 2;
    }

    this.setStyle(
      new Style({
        image: new Icon({
          src: 'static/svg/plane.svg',
          size: [100, 100],
          offset: [0, 0],
          opacity: 1,
          scale,
          rotation: (heading * Math.PI) / 180,
        }),
      }),
    );
  }
}

const getElement = obj => (typeof obj == 'object' && obj != null && 'elm' in obj ? obj.elm : obj);

const getElementFn =
  fn =>
  (elm, ...args) =>
    fn(getElement(elm), ...args);

function findElements(q = '*') {
  let ret = document.querySelectorAll(q);
  if(ret) ret = [...ret];
  return ret;
}

function getNumber(value) {
  let ret;
  ret = parseFloat(value);
  return ret;
}

let styleMaps = new WeakMap();
let styleMapper = memoize(elm => window.getComputedStyle(elm));

const getStyle = getElementFn(function getStyle(elm, prop) {
  return styleMapper(elm)[prop];
});

const getStyles = getElementFn(function getStyles(elm) {
  let names = Object.getOwnPropertyNames(elm.style);

  let wm = {},
    styles = styleMapper(elm);

  for(let name of names) if(isNaN(+name) && styles[name] !== '') wm[name] = styles[name];

  return wm;
});

const setStyle = getElementFn(function setStyle(elm, prop, value) {
  let wm = styleMapper(elm);
  wm[prop] = elm.style[prop] = value;

  return (wm[prop] = window.getComputedStyle(elm)[prop]);
});

const setStyles = getElementFn(function setStyles(elm, styles) {
  Object.assign(elm.style, styles);
  let wm;
  styleMaps.set(elm, (wm = window.getComputedStyle(elm)));
  return wm;
});

const setAttributes = getElementFn(function setAttributes(elm, attrs) {
  for(let name in attrs) elm.setAttribute(name, attrs[name]);
});

let layers = (globalThis.layers = new Set());

class HTMLLayer {
  static maxZ = 10000;

  constructor(type = 'div', attributes = {}, parent = document.body) {
    let element = (this.elm = document.createElement(type));
    const { style, ...attrs } = attributes;

    let zIndex = HTMLLayer.maxZ++;

    setAttributes(element, attrs);
    setStyles(element, { zIndex });

    parent.appendChild(element);
    let obj = this;
    let ret = new Proxy(obj, {
      get(target, prop, receiver) {
        switch (prop) {
          case 'top':
          case 'left':
          case 'bottom':
          case 'right':
          case 'width':
          case 'height': {
            return getNumber(getStyle(element, prop));
            break;
          }
          case 'style': {
            return getStyles(element);
          }
        }
        if(element.hasAttribute(prop)) return element.getAttribute(prop);

        return Reflect.get(target, prop, receiver);
      },
      set(target, prop, value) {
        switch (prop) {
          case 'top':
          case 'left':
          case 'bottom':
          case 'right':
          case 'width':
          case 'height': {
            setStyle(element, prop, value);
            return;
          }
        }
        if(element.hasAttribute(prop)) return element.setAttribute(prop, value);

        throw new Error(`No such property '${prop}'`);
        //   return Reflect.set(target, prop, receiver);
      },
      ownKeys(target) {
        return [...element.getAttributeNames()] /*.concat(Reflect.ownKeys(target))*/;
      },
    });
    layers.add(ret);
    return ret;
  }

  static get list() {
    return [...layers];
  }
}

HTMLLayer.prototype.elm = null;

Object.assign(globalThis, {
  Aircraft,
  HTMLLayer,
  findElements,
  getStyle,
  getStyles,
  setStyle,
  setStyles,
  setAttributes,
});