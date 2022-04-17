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
import XYZ from './openlayers/src/ol/source/XYZ.js';

new Map({
  target: 'map',
  layers: [
    new TileLayer({
      source: new XYZ({
        url: 'https://{a-c}.tile.openstreetmap.org/{z}/{x}/{y}.png'
      })
    })
  ],
  view: new View({
    center: [0, 0],
    zoom: 2
  })
});
