import { Layer as HTMLLayer } from './lib/dom/layer.js';
import { forwardRef, Fragment, h, Portal, React, ReactComponent, toChildArray } from './lib/dom/preactComponent.js';
import { assert, define, isObject, lazyProperties, memoize, unique } from './lib/misc.js';
import { addCoordinateTransforms, addProjection, CircleStyle, composeCssTransform, Feature, Fill, fromLonLat, GeoJSON, Geolocation, getVectorContext, Icon, Layer, LayerGroup, LineString, MultiPoint, OLMap, OSM, Overlay, Point, Polygon, Projection, RegularShape, Stroke, Style, TileLayer, transform, VectorLayer, VectorSource, View, XYZ, ZoomSlider, } from './lib/ol.js';
import { BiDirMap, ObjectWrapper } from './object-helpers.js';
import { Coordinate, Markers, OpenlayersMap, ParseCoordinates, Pin, Popup, TransformCoordinates } from './ol-helpers.js';
import { add, scale, toStringHDMS, wrapX } from './openlayers/src/ol/coordinate.js';
import { toLonLat } from './openlayers/src/ol/proj.js';
import { parseDMS } from './string-helpers.js';

let data = (globalThis.data = []);
let center = (globalThis.center = transform([7.454281, 46.96453], 'EPSG:4326', 'EPSG:3857'));
let extent = [5.9962, 45.8389, 10.5226, 47.8229];
let topLeft = [5.9962, 47.8229],
  topRight = [10.5226, 47.8229],
  bottomLeft = [5.9962, 45.8389],
  bottomRight = [10.5226, 45.8389];

/*lazyProperties(globalThis, {
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
});*/

const cities = {
  bern: new Coordinate(7.4458, 46.95),
  zuerich: new Coordinate(8.545094, 47.373878),
  winterthur: new Coordinate(8.729869, 47.500954),
  genf: new Coordinate(6.143158, 46.204391),
  zug: new Coordinate(8.515495, 47.166168),
  basel: new Coordinate(7.588576, 47.559601),
  winterthur: new Coordinate(8.737565, 47.49995),
  hinterkappelen: new Coordinate(7.37736, 46.96792),
  wankdorf: new Coordinate(7.46442, 46.96662),
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
    if(called) return;

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

function Get(feature) {
  let g = feature.getGeometry();
  return g.getCoordinates();
}

function CreateMap() {
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

  const vector = new VectorLayer({
    source: new VectorSource({
      features: [
        new Feature({
          geometry: new LineString(extentVector),
        }),
        positionFeature,
      ],
      wrapX: false,
    }),
    style: new Style({
      stroke: new Stroke({
        color: '#ffd705',
        width: 4,
        lineDash: [4, 8],
      }),
    }),
  });

  let map = OpenlayersMap.create();
  let view = map.getView();

  const geolocation = new Geolocation({
    trackingOptions: {
      enableHighAccuracy: true,
    },
    projection: view.getProjection(),
  });

  geolocation.on('change', function(e) {
    locationDisplay.innerHTML = geolocation.getPosition();
  });

  globalThis.markers = Markers.create(map);
  globalThis.mapPinStyle ??= new Style({
    image: new Icon({
      anchor: [1, 1],
      anchorXUnits: 'fraction',
      anchorYUnits: 'fraction',
      src: 'static/svg/map-pin.svg',
      scale: 1,
    }),
  });
  globalThis.pins = Object.entries(cities).map(([name, geometry]) => Pin.create(name, mapPinStyle, geometry));

  //markers.add(...pins);

  const hereMarker = Pin.create(
    'You are here',
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
    [...new Coordinate(7.45425, 46.96483)],
  );

  Object.assign(globalThis, { hereMarker });

  markers.add(hereMarker);
  //map.addLayer(markerLayer);

  Object.assign(globalThis, {
    view,
    map,
    extentVector,
    positionFeature,
    geolocation,
    parseDMS,
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

  let popup;
  // = Popup.create(h(Fragment, {}, ['blah']), [0, 0], cities.hinterkappelen);

  /**
   * Add a click handler to the map to render the popup.
   */
  map.on('singleclick', function(evt) {
    const coordinate = evt.coordinate;
    const hdms = toStringHDMS(toLonLat(coordinate));
    globalThis.evt = evt;

    let lonlat = new Coordinate(...coordinate, 'EPSG:3857').toLonLat();

    console.log('Clicked at', lonlat);

    let features = (globalThis.features = map.getFeaturesAtPixel(evt.pixel_));

    if(features && features.length) {
      console.log('features', features);

      if(features.length == 1) {
        let [feature] = features;

        globalThis.feature = feature;
      }
    }
    //content.innerHTML = '<p>You clicked here:</p><code>' + hdms + '</code>';

    if(popup?.overlay) popup.overlay.setPosition(coordinate);
  });

  fetch('uploads').then(async response => {
    let json = await response.json();
    globalThis.uploads = json
      .filter(u => u?.exif?.GPSPosition)
      .map(u => {
        u.position = ParseCoordinates(u.exif.GPSPosition);
        u.pin = Pin.create(u.filename, mapPinStyle, u.position);

        //if(u.filename != '') u.popup =Popup.create(u.filename??'<no filename>', [0,0], u.position);

        markers.add(u.pin);

        return u;
      });
    console.log('uploads', globalThis.uploads);
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
    LayerGroup,
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
  Popup,
  assert,
  lazyProperties,
  define,
  isObject,
  memoize,
  unique,
  HTMLLayer,
  h,
  forwardRef,
  Fragment,
  React,
  ReactComponent,
  Portal,
  toChildArray,
  ParseCoordinates,
});

CreateMap();
