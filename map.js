/*map = new OpenLayers.Map('mapdiv');
map.addLayer(new OpenLayers.Layer.OSM());

var pois = new OpenLayers.Layer.Text('My Points', { location: './textfile.txt', projection: map.displayProjection });
map.addLayer(pois);
// create layer switcher widget in top right corner of map.
var layer_switcher = new OpenLayers.Control.LayerSwitcher({});
map.addControl(layer_switcher);
//Set start centrepoint and zoom
var lonLat = new OpenLayers.LonLat(9.5788, 48.9773).transform(
  new OpenLayers.Projection('EPSG:4326'), // transform from WGS 1984
  map.getProjectionObject() // to Spherical Mercator Projection
);
var zoom = 11;
map.setCenter(lonLat, zoom);
*/
import Map from './openlayers/src/ol/Map.js';
import View from './openlayers/src/ol/View.js';
import TileLayer from './openlayers/src/ol/layer/Tile.js';
import Layer from './openlayers/src/ol/layer/Layer.js';
import Point from './openlayers/src/ol/geom/Point.js';
import Overlay from './openlayers/src/ol/Overlay.js';
import XYZ from './openlayers/src/ol/source/XYZ.js';
import OSM from './openlayers/src/ol/source/OSM.js';
import Projection from './openlayers/src/ol/proj/Projection.js';
import {
  addCoordinateTransforms,
  addProjection,
  transform,
} from './openlayers/src/ol/proj.js';


let center = transform([ 7.454281, 46.964530], 'EPSG:4326', 'EPSG:3857');
let map =globalThis.map= new Map({
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
});

Object.assign(globalThis,  { center ,OpenLayers: { Overlay, Map,View,TileLayer,Layer,Point,XYZ,OSM,Projection,addCoordinateTransforms, addProjection, transform }});

//Set start centrepoint and zoom
/*var lonLat = new Point(7.454281, 46.964530 );
var zoom = 11;
console.log('map',map);
map.setCenter(lonLat, zoom);*/
//map.addLayer(new OSM());


/*import GeoJSON from './openlayers/src/ol/format/GeoJSON.js';
import Map from './openlayers/src/ol/Map.js';
import View from './openlayers/src/ol/View.js';
import {Circle as CircleStyle, Fill, Stroke, Style} from './openlayers/src/ol/style.js';
import {OSM, Vector as VectorSource} from './openlayers/src/ol/source.js';
import {Tile as TileLayer, Vector as VectorLayer} from './openlayers/src/ol/layer.js';

const source = new VectorSource({
  url: 'data/geojson/switzerland.geojson',
  format: new GeoJSON(),
});
const style = new Style({
  fill: new Fill({
    color: 'rgba(255, 255, 255, 0.6)',
  }),
  stroke: new Stroke({
    color: '#319FD3',
    width: 1,
  }),
  image: new CircleStyle({
    radius: 5,
    fill: new Fill({
      color: 'rgba(255, 255, 255, 0.6)',
    }),
    stroke: new Stroke({
      color: '#319FD3',
      width: 1,
    }),
  }),
});
const vectorLayer = new VectorLayer({
  source: source,
  style: style,
});
const view = new View({
  center: [0, 0],
  zoom: 1,
});
const map = new Map({
  layers: [
    new TileLayer({
      source: new OSM(),
    }),
    vectorLayer,
  ],
  target: 'map',
  view: view,
});

const zoomtoswitzerland = document.getElementById('zoomtoswitzerland');
zoomtoswitzerland.addEventListener(
  'click',
  function () {
    const feature = source.getFeatures()[0];
    const polygon = feature.getGeometry();
    view.fit(polygon, {padding: [170, 50, 30, 150]});
  },
  false
);

const zoomtolausanne = document.getElementById('zoomtolausanne');
zoomtolausanne.addEventListener(
  'click',
  function () {
    const feature = source.getFeatures()[1];
    const point = feature.getGeometry();
    view.fit(point, {padding: [170, 50, 30, 150], minResolution: 50});
  },
  false
);

const centerlausanne = document.getElementById('centerlausanne');
centerlausanne.addEventListener(
  'click',
  function () {
    const feature = source.getFeatures()[1];
    const point = feature.getGeometry();
    const size = map.getSize();
    view.centerOn(point.getCoordinates(), size, [570, 500]);
  },
  false
);*/
