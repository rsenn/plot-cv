import Map from './openlayers/src/ol/Map.js';
import View from './openlayers/src/ol/View.js';
import TileLayer from './openlayers/src/ol/layer/Tile.js';
import Layer from './openlayers/src/ol/layer/Layer.js';
import Point from './openlayers/src/ol/geom/Point.js';
import Overlay from './openlayers/src/ol/Overlay.js';
import XYZ from './openlayers/src/ol/source/XYZ.js';
import OSM from './openlayers/src/ol/source/OSM.js';
import Projection from './openlayers/src/ol/proj/Projection.js';
import { addCoordinateTransforms, addProjection, transform } from './openlayers/src/ol/proj.js';

function Connection(port = 12001) {
  let ws = (globalThis.ws = new WebSocket('wss://transistorisiert.ch:' + port));
  ws.onerror = e => {
    console.log('ERROR:', e);
  };
  return ws;
}

function Time(t, offset = 0) {
  let dt = t ? new Date(t * 1e3) : new Date();
  return Math.floor(+dt * 1e-3 + offset);
}
function TimeToStr(t, offset = 0) {
  if(typeof t == 'object' && t != null && !(t instanceof Date)) {
    let obj = {};
    for(let [name, value] of Object.entries(t)) {
      obj[name] = TimeToStr(value);
    }
    return obj;
  }
  let dt = new Date(Time(t, offset) * 1000);
  return dt.toISOString();
}
function DailyPhase(t) {
  t -= t % (21600 * 1000);
  return Math.floor(t);
}

function PhaseFile(t) {
  let date = new Date(t);
  let str = date.toISOString().replace(/T.*/g, '');

  let phaseStr = ((date / (21600 * 1000)) % 4) + 1 + '';

  let file = str + '-' + phaseStr + '.txt';

  return file;
}

let center = transform([7.454281, 46.96453], 'EPSG:4326', 'EPSG:3857');
let map = (globalThis.map = new Map({
  target: 'mapdiv',
  layers: [
    new TileLayer({
      source: new XYZ({
        url: 'https://{a-c}.tile.openstreetmap.org/{z}/{x}/{y}.png'
      })
    })
  ],
  view: new View({
    center,
    zoom: 8
  })
}));

Object.assign(globalThis, {
  center,
  OpenLayers: {
    Overlay,
    Map,
    View,
    TileLayer,
    Layer,
    Point,
    XYZ,
    OSM,
    Projection,
    addCoordinateTransforms,
    addProjection,
    transform
  },
  Connection,
  Time,
  TimeToStr
});

let planes = (globalThis.planes = []);
let d = Date.parse('2022-04-19T14:59:00Z');
let data;
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
  'position_source'
];

false &&
  window.addEventListener('load', () => {
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
                    [keys[i]]: ['time_position', 'last_contact'].indexOf(keys[i]) != -1 ? new Date(field * 1000) : field
                  }),
                  {}
                )
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
      positioning: 'center-center'
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
