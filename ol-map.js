import { OLMap, View, TileLayer, Layer, Point, Overlay, XYZ, OSM, Feature, Projection, VectorLayer, VectorSource, MultiPoint, Polygon, LineString, Geolocation, GeoJSON, composeCssTransform, Icon, Fill, fromLonLat, ZoomSlider, addCoordinateTransforms, getVectorContext, transform, Style, Stroke, CircleStyle, RegularShape, addProjection, LayerGroup } from './lib/ol.js';

import LayerSwitcher /* , { BaseLayerOptions, GroupLayerOptions }*/ from './lib/ol-layerswitcher.js';
import { assert, lazyProperties, define, isObject, memoize, unique, arrayFacade } from './lib/misc.js';
import { Element } from './lib/dom.js';
import { TransformCoordinates, Coordinate, Pin, Markers } from './ol-helpers.js';
import { ObjectWrapper, BiDirMap } from './object-helpers.js';

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

const cities = {
  bern: new Coordinate(7.4458, 46.95),
  zuerich: new Coordinate(8.545094, 47.373878),
  winterthur: new Coordinate(8.729869, 47.500954),
  genf: new Coordinate(6.143158, 46.204391),
  zug: new Coordinate(8.515495, 47.166168),
  basel: new Coordinate(7.588576, 47.559601),
  winterthur: new Coordinate(8.737565, 47.49995),
  hinterkappelen: new Coordinate(7.37736, 46.96792),
  wankdorf: new Coordinate(7.46442, 46.96662)
};

/*function Refresh() {
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
*/
function SetFenceColor(color) {
  vector.setStyle(new Style({ stroke: new Stroke({ color, width: 3, lineDash: [2, 4] }) }));
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

function CreateMarkerLayer(features) {
  const iconStyle = new Style({
    image: new Icon({
      anchor: [1, 1],
      anchorXUnits: 'fraction',
      anchorYUnits: 'fraction',
      src: 'static/svg/map-pin.svg',
      scale: 1
    })
  });

  globalThis.features = features;

  features.forEach(f => f.setStyle(iconStyle));
  globalThis.markers = new Markers(map);

  let vectorLayer = new VectorLayer({
    source: new VectorSource({ features })
  });

  Object.assign(globalThis, { features, vectorLayer, iconStyle });

  return vectorLayer;
}

function Get(feature) {
  let g = feature.getGeometry();
  return g.getCoordinates();
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

  /* const markerLayer = CreateMarkerLayer(
    Object.entries(cities).map(([name, geometry]) => new Feature({ name, geometry: new Point([...geometry]) }))
  );*/
  let map = new OLMap({
    target: 'mapdiv',
    layers: [tileLayer, rasterLayer, worldStreetMap, worldTopoMap, natGeoWorldMap, vector],
    view
  });

  globalThis.markers = Markers.create(map);

  globalThis.pins = Object.entries(cities).map(([name, geometry]) =>
    Pin.create(
      name,
      new Style({
        image: new Icon({
          anchor: [1, 1],
          anchorXUnits: 'fraction',
          anchorYUnits: 'fraction',
          src: 'static/svg/map-pin.svg',
          scale: 1
        })
      }),
      geometry
    )
  );

  markers.add(...pins);

  //map.addLayer(markerLayer);

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
  cities,
  Get,
  fromLonLat,
  ObjectWrapper,
  BiDirMap,
  Markers,
  Pin,
  assert,
  lazyProperties,
  define,
  isObject,
  memoize,
  unique,
  arrayFacade
});

CreateMap();
