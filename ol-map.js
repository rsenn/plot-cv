import { OLMap, View, TileLayer, Layer, Point, Overlay, XYZ, OSM, Feature, Projection, VectorLayer, VectorSource, MultiPoint, Polygon, LineString, Geolocation, GeoJSON, composeCssTransform, Icon, Fill, fromLonLat, ZoomSlider, addCoordinateTransforms, getVectorContext, transform, Style, Stroke, CircleStyle, RegularShape, addProjection, LayerGroup } from './lib/ol.js';

import LayerSwitcher /* , { BaseLayerOptions, GroupLayerOptions }*/ from './lib/ol-layerswitcher.js';
import { assert, lazyProperties, define, isObject, memoize, unique } from './lib/misc.js';
import { Element } from './lib/dom.js';

let data = (globalThis.data = []);
let center = (globalThis.center = transform([7.454281, 46.96453], 'EPSG:4326', 'EPSG:3857'));
let extent = [5.9962, 45.8389, 10.5226, 47.8229];
let topLeft = [5.9962, 47.8229],
  topRight = [10.5226, 47.8229],
  bottomLeft = [5.9962, 45.8389],
  bottomRight = [10.5226, 45.8389];

lazyProperties(globalThis, {
  locationDisplay: () =>
    document.body.appendChild(
      Element.create(
        'div',
        {
          style: {
            position: 'fixed',
            bottom: '20px',
            right: '20px',
            minWidth: '200px',
            height: '50px',
            background: 'white',
            border: '1px solid black'
          }
        },
        []
      )
    )
});

function TransformCoordinates(...args) {
  if(args.length == 2) return transform(args, 'EPSG:4326', 'EPSG:3857');
  if(args.length == 4) {
    let extent = [args.splice(0, 2), args.splice(0, 2)];
    return extent.reduce((acc, coord) => acc.concat(TransformCoordinates(...coord)), []);
  }

  if(typeof args[0] == 'string') return TransformCoordinates(args[0].split(',').map(n => +n));
}

class Coordinate {
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
  winterthur: new Coordinate(8.737565, 47.49995)
};

function Refresh() {
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
function SetFenceColor(color) {
  vector.setStyle(new OpenLayers.Style({ stroke: new OpenLayers.Stroke({ color, width: 3, lineDash: [2, 4] }) }));
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
      duration: duration
    },
    callback
  );
  view.animate(
    {
      zoom: zoom - 1,
      duration: duration / 2
    },
    {
      zoom: zoom,
      duration: duration / 2
    },
    callback
  );
}

function CreateMap() {
  const view = new View({
    center,
    zoom: 11,
    minZoom: 3,
    maxZoom: 20,
    extent: TransformCoordinates(extent)
  });

  const positionFeature = new Feature();
  positionFeature.setStyle(
    new Style({
      image: new CircleStyle({
        radius: 6,
        fill: new Fill({
          color: '#3399CC'
        }),
        stroke: new Stroke({
          color: '#fff',
          width: 2
        })
      })
    })
  );
  let extentVector = [topLeft, topRight, bottomRight, bottomLeft, topLeft].map(a => TransformCoordinates(...a));

  const vector = new VectorLayer({
    source: new VectorSource({
      features: [
        new Feature({
          geometry: new LineString(extentVector)
        }),
        positionFeature
      ],
      wrapX: false
    }),
    style: new Style({
      stroke: new Stroke({
        color: '#ffd705',
        width: 4,
        lineDash: [4, 8]
      })
    })
  });

  const geolocation = new Geolocation({
    trackingOptions: {
      enableHighAccuracy: true
    },
    projection: view.getProjection()
  });

  geolocation.on('change', function(e) {
    locationDisplay.innerHTML = geolocation.getPosition();
  });

  const tileLayer = new TileLayer({
    title: 'OSM',
    type: 'base',
    visible: false,
    source: new XYZ({
      url: 'https://{a-c}.tile.openstreetmap.org/{z}/{x}/{y}.png'
    })
  });
  const rasterLayer = new TileLayer({
    title: 'Satellit',
    type: 'base',
    visible: true,
    source: new XYZ({
      url: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
      maxZoom: 19
    })
  });
  const worldStreetMap = new TileLayer({
    title: 'Strassenkarte',
    type: 'base',
    visible: false,
    source: new XYZ({
      url: 'http://server.arcgisonline.com/ArcGIS/rest/services/World_Street_Map/MapServer/tile/{z}/{y}/{x}',
      maxZoom: 20
    })
  });
  const worldTopoMap = new TileLayer({
    title: 'Topographie',
    type: 'base',
    visible: false,
    source: new XYZ({
      url: 'http://server.arcgisonline.com/ArcGIS/rest/services/World_Topo_Map/MapServer/tile/{z}/{y}/{x}',
      maxZoom: 20
    })
  });
  const natGeoWorldMap = new TileLayer({
    title: 'National Geographic',
    type: 'base',
    visible: false,
    source: new XYZ({
      url: 'http://server.arcgisonline.com/ArcGIS/rest/services/NatGeo_World_Map/MapServer/tile/{z}/{y}/{x}',
      maxZoom: 12
    })
  });
  let map = new OLMap({
    target: 'mapdiv',
    layers: [tileLayer, rasterLayer, worldStreetMap, worldTopoMap, natGeoWorldMap].reverse(),
    view
  });

  const zoomslider = new ZoomSlider();
  map.addControl(zoomslider);
  const layerSwitcher = new LayerSwitcher({
    reverse: true,
    groupSelectStyle: 'group'
  });
  map.addControl(layerSwitcher);

  Object.assign(globalThis, {
    tileLayer,
    rasterLayer,
    view,
    vector,
    map,
    extentVector,
    positionFeature,
    layerSwitcher,
    geolocation
  });
  Object.defineProperties(globalThis, {
    zoom: {
      get() {
        return view.getZoom();
      },
      set(value) {
        view.setZoom(value);
      }
    }
  });
  return map;
}

Object.assign(globalThis, {
  OpenLayers: {
    OLMap,
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
    MultiPoint,
    Polygon,
    LineString,
    Geolocation,
    GeoJSON,
    composeCssTransform,
    Icon,
    Fill,
    fromLonLat,
    ZoomSlider,
    addCoordinateTransforms,
    getVectorContext,
    transform,
    Style,
    Stroke,
    CircleStyle,
    RegularShape,
    addProjection,
    LayerGroup
  },

  FlyTo,
  Coordinate,
  cities
});

CreateMap();
