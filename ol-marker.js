import { Point } from './openlayers/src/ol/geom.js';
import { Feature } from './openlayers/src/ol/index.js';
import { fromLonLat } from './openlayers/src/ol/proj.js';
import { Vector } from './openlayers/src/ol/source.js';
import { Icon, Style } from './openlayers/src/ol/style.js';

function CreateMarker(coord) {
  const [lon, lat] = coord;
  /*
    Create Icon
*/
  let MarkerIcon = new Feature({
    geometry: new Point(proj.fromLonLat([lon, lat])),
    name: 'Marker text',
    desc: '<label>Details</label> <br> Latitude: ' + lat + ' Longitude: ' + lon
  });
  // Add icon style
  MarkerIcon.setStyle(
    new Style({
      image: new Icon({
        anchor: [0.5, 50],
        anchorXUnits: 'fraction',
        anchorYUnits: 'pixels',
        src: 'img/marker-64.png'
        // ,scale: 0.4
      })
    })
  );

  /*
    Create map source
*/
  var MapSource = new Vector({
    features: [MarkerIcon]
  });
  // Create map layer
  var MapLayer = new Vector({
    source: MapSource
  });
  // Set layer z-index
  MapLayer.setZIndex(999);
  // Add marker to layer
  map.addLayer(MapLayer);
}