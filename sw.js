/*import Util from './lib/util.js';
import { Repeater } from './lib/repeater/repeater.js';
import { streamify, oncePromise, filter, map, throttle, distinct, subscribe } from './lib/async/events.js';
*/
const log = (() => {}) || console.debug.bind(null, 'WORKER:');

log('self:', self);

log('executing.');

/* A version number is useful when updating the worker logic,
   allowing you to remove outdated cache entries during the update.
*/
var version = 'v1::';

var requested = new Set();

/* These resources will be downloaded and cached by the service worker
   during the installation process. If any resource fails to be downloaded,
   then the service worker won't be installed either.
*/
var offlineFundamentals = [
  '',
  'config',
  'commands.js',
  'components.js',
  'events.js',
  'favicon.ico',
  'index.html',
  'lib/alea.js',
  'lib/async/debounce.js',
  'lib/async/helpers.js',
  'lib/autoStore.js',
  'lib/classNames.js',
  'lib/clipper.js',
  'lib/clipper-lib.js',
  'lib/color/coloredText.js',
  'lib/color/hsla.js',
  'lib/color.js',
  'lib/color/rgba.js',
  'lib/container/binaryTree.js',
  'lib/container/hashList.js',
  'lib/container/multiMap.js',
  'lib/deep-diff.js',
  'lib/deep.js',
  'lib/devtools.js',
  'lib/dom/cache.js',
  'lib/dom/cacheStorage.js',
  'lib/dom/container.js',
  'lib/dom/css.js',
  'lib/dom/element.js',
  'lib/dom/elementRect.js',
  'lib/dom/event.js',
  'lib/dom/isEditable.js',
  'lib/dom/iterator.js',
  'lib/dom.js',
  'lib/dom/keysim.js',
  'lib/dom/layer.js',
  'lib/dom/node.js',
  'lib/dom/preactComponent.js',
  'lib/dom/select.js',
  'lib/dom/svg.js',
  'lib/dom/timer.js',
  'lib/dom/tree.js',
  'lib/dom/xpath.js',
  'lib/draw/colorMap.js',
  'lib/eagle/boardRenderer.js',
  'lib/eagle/common.js',
  'lib/eagle/components/arc.js',
  'lib/eagle/components/background.js',
  'lib/eagle/components/circle.js',
  'lib/eagle/components/cross.js',
  'lib/eagle/components/dimension.js',
  'lib/eagle/components/drawing.js',
  'lib/eagle/components/element.js',
  'lib/eagle/components/frame.js',
  'lib/eagle/components/grid.js',
  'lib/eagle/components/hole.js',
  'lib/eagle/components/instance.js',
  'lib/eagle/components.js',
  'lib/eagle/components/origin.js',
  'lib/eagle/components/package.js',
  'lib/eagle/components/pad.js',
  'lib/eagle/components/pin.js',
  'lib/eagle/components/polygon.js',
  'lib/eagle/components/rectangle.js',
  'lib/eagle/components/svg.js',
  'lib/eagle/components/symbol.js',
  'lib/eagle/components/textElement.js',
  'lib/eagle/components/text.js',
  'lib/eagle/components/wire.js',
  'lib/eagle/components/wirePath.js',
  'lib/eagle/document.js',
  'lib/eagle/element.js',
  'lib/eagle.js',
  'lib/eagle/libraryRenderer.js',
  'lib/eagle/node.js',
  'lib/eagle/nodeList.js',
  'lib/eagle/nodeMap.js',
  'lib/eagle/project.js',
  'lib/eagle/ref.js',
  'lib/eagle/renderer.js',
  'lib/eagle/renderUtils.js',
  'lib/eagle/schematicRenderer.js',
  'lib/eagle/svgRenderer.js',
  'lib/ecmascript/environment.js',
  'lib/ecmascript/estree.js',
  'lib/ecmascript/interpreter.js',
  'lib/ecmascript.js',
  'lib/ecmascript/lexer.js',
  'lib/ecmascript/parser.js',
  'lib/ecmascript/printer.js',
  'lib/ecmascript/token.js',
  'lib/eventEmitter.js',
  'lib/filesystem.js',
  'lib/functional.js',
  'lib/gcode/gcode-interp.js',
  'lib/gcode/gcodetogeometry.js',
  'lib/gcode/gcodeToObject.js',
  'lib/gcode/interp.js',
  'lib/gcode/interpreter.js',
  'lib/gcode.js',
  'lib/gcode/objectToGcode.js',
  'lib/gcode/parser.js',
  'lib/geom/align.js',
  'lib/geom/arc.js',
  'lib/geom/bbox.js',
  'lib/geom/circle.js',
  'lib/geom/graph.js',
  'lib/geom/intersection.js',
  'lib/geom.js',
  'lib/geom/line.js',
  'lib/geom/lineList.js',
  'lib/geom/matrix.js',
  'lib/geom/point.js',
  'lib/geom/pointList.js',
  'lib/geom/polygonFinder.js',
  'lib/geom/polygon.js',
  'lib/geom/polyline.js',
  'lib/geom/rect.js',
  'lib/geom/simplify.js',
  'lib/geom/size.js',
  'lib/geom/sweepLine.js',
  'lib/geom/transformation.js',
  'lib/geom/trbl.js',
  'lib/geom/vector.js',
  'lib/geom/voronoi.js',
  'lib/gerber/parser.js',
  'lib/github.js',
  'lib/hooks.js',
  'lib/hooks/useActive.js',
  'lib/hooks/useClickOut.js',
  'lib/hooks/useDimensions.js',
  'lib/hooks/useDoubleClick.js',
  'lib/hooks/useElement.js',
  'lib/hooks/useEvent.js',
  'lib/hooks/useFocus.js',
  'lib/hooks/useForceUpdate.js',
  'lib/hooks/useGesture.js',
  'lib/hooks/useGetSet.js',
  'lib/hooks/useHover.js',
  'lib/hooks/useMousePosition.js',
  'lib/hooks/usePanZoom.js',
  'lib/hooks/useTrkl.js',
  'lib/hooks/utils.js',
  'lib/iterator.js',
  'lib/json/diff.js',
  'lib/json.js',
  'lib/json/json2xml.js',
  'lib/json/path.js',
  'lib/json/pathMapper.js',
  'lib/json/treeObserver.js',
  'lib/json/util.js',
  'lib/json/xml2json.js',
  'lib/KolorWheel.js',
  'lib/lazyInitializer.js',
  'lib/log.js',
  'lib/lscache.js',
  'lib/net/websocket-async.js',
  'lib/path.js',
  'lib/proxy/observableMembrane.js',
  'lib/repeater/react-hooks.js',
  'lib/repeater/repeater.js',
  'lib/repeater/timers.js',
  'lib/scrollHandler.js',
  'lib/stream.js',
  'lib/stream/textDecodeStream.js',
  'lib/stream/textEncodeStream.js',
  'lib/stream/transformStream.js',
  'lib/stream/utils.js',
  'lib/stream/writableStream.js',
  'lib/svg/path.js',
  'lib/svg/path-parser.js',
  'lib/svg/pathReverse.js',
  'lib/tlite.js',
  'lib/touchHandler.js',
  'lib/trkl.js',
  'lib/tXml.js',
  'lib/util.js',
  'lib/xml.js',
  'lib/xml/util.js',
  'lib/xml/xpath.js',
  'main.js',
  'message.js',
  'node_modules/htm/preact/standalone.mjs',
  'serial.js',
  'slots.js',
  'static/fonts.css',
  'static/style.css',
  'static/tlite.css'
];

/* The install event fires when the service worker is first installed.
   You can use this event to prepare the service worker to be able to serve
   files while visitors are offline.
*/
self.addEventListener('install', event => {
  log('install event in progress.');
  /* Using event.waitUntil(p) blocks the installation process on the provided
     promise. If the promise is rejected, the service worker won't be installed.
  */
  event.waitUntil(
    /* The caches built-in is a promise-based API that helps you cache responses,
       as well as finding and deleting them.
    */
    caches
      /* You can open a cache by name, and this method returns a promise. We use
         a versioned cache name here so that we can remove old cache entries in
         one fell swoop later, when phasing out an older service worker.
      */
      .open(version + 'fundamentals')
      .then(cache =>
        /* After the cache is opened, we can fill it with the offline fundamentals.
           The method below will add all resources in `offlineFundamentals` to the
           cache, after making requests for them.
        */
        cache.addAll(offlineFundamentals)
      )
      .then(() => log('install completed'))
  );
});
let messagePort;

self.addEventListener('message', event => {
  const { data } = event;

  if(data) {
    const { type } = data;

    if(type == 'INIT_PORT') {
      messagePort = event.ports[0];
    } else if(data == 'REQUESTED') {
      messagePort.postMessage({ requested });
    }

    console.debug('message', { type, data, messagePort });
  }
});

/* The fetch event fires whenever a page controlled by this service worker requests
   a resource. This isn't limited to `fetch` or even XMLHttpRequest. Instead, it
   comprehends even the request for the HTML page on first load, as well as JS and
   CSS resources, fonts, any images, etc.
*/
self.addEventListener('fetch', event => {
  const { request } = event;
  const { url } = request;
  const location = url.replace(/^[^\/]*:\/\/[^\/]*/, '');

  log('fetch event in progress.', location);

  /* We should only cache GET requests, and deal with the rest of method in the
     client-side, by handling failed POST,PUT,PATCH,etc. requests.
  */
  if(event.request.method !== 'GET') {
    /* If we don't block the event as shown below, then the request will go to
       the network as usual. */
    log('fetch event ignored.', event.request.method, location);
    return;
  }

  requested.add(location);

  /* Similar to event.waitUntil in that it blocks the fetch event on a promise.
     Fulfillment result will be used as the response, and rejection will end in a
     HTTP response indicating failure.
  */
  event.respondWith(
    caches
      /* This method returns a promise that resolves to a cache entry matching
         the request. Once the promise is settled, we can then provide a response
         to the fetch request.
      */
      .match(event.request)
      .then(cached => {
        /* Even if the response is in our cache, we go to the network as well.
           This pattern is known for producing "eventually fresh" responses,
           where we return cached responses immediately, and meanwhile pull
           a network response and store that in the cache.
           Read more:
           https://ponyfoo.com/articles/progressive-networking-serviceworker
        */
        var networked = fetch(event.request)
          // We handle the network request with success and failure scenarios.
          .then(fetchedFromNetwork, unableToResolve)
          // We should catch errors on the fetchedFromNetwork handler as well.
          .catch(unableToResolve);

        /* We return the cached response immediately if there is one, and fall
           back to waiting on the network as usual.
        */
        log('fetch event', cached ? '(cached)' : '(network)', location);
        return cached || networked;

        function fetchedFromNetwork(response) {
          /* We copy the response before replying to the network request.
             This is the response that will be stored on the ServiceWorker cache.
          */
          var cacheCopy = response.clone();

          log('fetch response from network.', location);

          caches
            // We open a cache to store the response for this request.
            .open(version + 'pages')
            .then(cache =>
              /* We store the response for this request. It'll later become
                 available to caches.match(event.request) calls, when looking
                 for cached responses.
              */
              cache.put(event.request, cacheCopy)
            )
            .then(() => log('fetch response stored in cache.', location));

          // Return the response so that the promise is settled in fulfillment.
          return response;
        }

        /* When this method is called, it means we were unable to produce a response
           from either the cache or the network. This is our opportunity to produce
           a meaningful response even when all else fails. It's the last chance, so
           you probably want to display a "Service Unavailable" view or a generic
           error response. */
        function unableToResolve() {
          /* There's a couple of things we can do here.
             - Test the Accept header and then return one of the `offlineFundamentals`
               e.g: `return caches.match('/some/cached/image.png')`
             - You should also consider the origin. It's easier to decide what
               "unavailable" means for requests against your origins than for requests
               against a third party, such as an ad provider.
             - Generate a Response programmaticaly, as shown below, and return that. */

          log('fetch request failed in both cache and network.');

          /* Here we're creating a response programmatically. The first parameter is the
             response body, and the second one defines the options for the response. */
          return new Response('<h1>Service Unavailable</h1>', {
            status: 503,
            statusText: 'Service Unavailable',
            headers: new Headers({
              'Content-Type': 'text/html'
            })
          });
        }
      })
  );
});

/* The activate event fires after a service worker has been successfully installed.
   It is most useful when phasing out an older version of a service worker, as at
   this point you know that the new worker was installed correctly. In this example,
   we delete old caches that don't match the version in the worker we just finished
   installing. */
self.addEventListener('activate', event => {
  /* Just like with the install event, event.waitUntil blocks activate on a promise.
     Activation will fail unless the promise is fulfilled. */
  log('activate event in progress.');

  event.waitUntil(
    caches
      /* This method returns a promise which will resolve to an array of available
         cache keys.  */
      .keys()
      // We return a promise that settles when all outdated caches are deleted.
      .then(keys =>
        Promise.all(
          keys
            // Filter by keys that don't start with the latest version prefix.
            .filter(key => !key.startsWith(version))
            .map(key => caches.delete(key))
        )
      )
      .then(() => log('activate completed.'))
  );
});
