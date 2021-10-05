import { WebSocketClient } from './lib/net/websocket-async.js';
import Util from './lib/util.js';
import path from './lib/path.js';
import { DebuggerProtocol } from './debuggerprotocol.js';
import { toString, toArrayBuffer, btoa as Base64Encode, atob as Base64Decode } from './lib/misc.js';
import React, { h, html, render, Fragment, Component, useState, useLayoutEffect, useRef } from './lib/dom/preactComponent.js';
import { ECMAScriptParser, Lexer } from './lib/ecmascript/parser.js';
import { EventEmitter, EventTarget } from './lib/events.js';
import { DroppingBuffer, FixedBuffer, MAX_QUEUE_LENGTH, Repeater, RepeaterOverflowError, SlidingBuffer } from './lib/repeater/repeater.js';
import { useAsyncIter, useRepeater, useResult, useValue } from './lib/repeater/react-hooks.js';
import { TimeoutError, delay, interval, timeout } from './lib/repeater/timers.js';
import { InMemoryPubSub } from './lib/repeater/pubsub.js';
import { semaphore, throttler } from './lib/repeater/limiters.js';
import { trkl } from './lib/trkl.js';
import { HSLA, RGBA, Point, isPoint, Size, isSize, Line, isLine, Rect, isRect, PointList, Polyline, Matrix, isMatrix, BBox, TRBL, Timer, Tree, Node, XPath, Element, isElement, CSS, SVG, Container, Layer, Renderer, Select, ElementPosProps, ElementRectProps, ElementRectProxy, ElementSizeProps, ElementWHProps, ElementXYProps, Align, Anchor, dom, isNumber, Unit, ScalarValue, ElementTransformation, CSSTransformSetters, Transition, TransitionList, RandomColor } from './lib/dom.js';
import { useActive, useClickout, useConditional, useDebouncedCallback, useDebounce, useDimensions, useDoubleClick, useElement, EventTracker, useEvent, useFocus, useForceUpdate, useGetSet, useHover, useMousePosition, useToggleButtonGroupState, useTrkl, useFetch } from './lib/hooks.js';
import { clamp, identity, noop, compose, maybe, snd, toPair, getOffset, getPositionOnElement, isChildOf } from './lib/hooks.js';
import CodeEditor from './react-simple-code-editor.js'
import Prism from './prism-core.js';
import './react-simple-code-editor/node_modules/prismjs/components/prism-clike.js';
import './react-simple-code-editor/node_modules/prismjs/components/prism-javascript.js';


const {highlight,languages} = Prism;

globalThis.addEventListener('load', async e => {
  let url = Util.parseURL();
  let socketURL = Util.makeURL({
    location: url.location + '/ws',
    protocol: url.protocol == 'https' ? 'wss' : 'ws'
  });

  globalThis.ws = await CreateSocket(socketURL);
  console.log(`Loaded`, { socketURL, ws });
});

async function LoadSource(filename) {
  try {
    let response = await fetch(filename);
    return await response.text();
  } catch(e) {}
}

Object.assign(globalThis, { DebuggerProtocol, LoadSource, Util, toString, toArrayBuffer, Base64Encode, Base64Decode, React, h, html, render, Fragment, Component, useState, useLayoutEffect, useRef, ECMAScriptParser, Lexer, EventEmitter, EventTarget, Element, isElement, path, CreateSource, Start, GetVariables, SendRequest, Step, Next, Continue, StackTrace });
Object.assign(globalThis, { DroppingBuffer, FixedBuffer, MAX_QUEUE_LENGTH, Repeater, RepeaterOverflowError, SlidingBuffer, useAsyncIter, useRepeater, useResult, useValue, TimeoutError, delay, interval, timeout, InMemoryPubSub, semaphore, throttler, trkl });

Object.assign(globalThis, { HSLA, RGBA, Point, isPoint, Size, isSize, Line, isLine, Rect, isRect, PointList, Polyline, Matrix, isMatrix, BBox, TRBL, Timer, Tree, Node, XPath, Element, isElement, CSS, SVG, Container, Layer, Renderer, Select, ElementPosProps, ElementRectProps, ElementRectProxy, ElementSizeProps, ElementWHProps, ElementXYProps, Align, Anchor, dom, isNumber, Unit, ScalarValue, ElementTransformation, CSSTransformSetters, Transition, TransitionList, RandomColor });
Object.assign(globalThis, { useActive, useClickout, useConditional, useDebouncedCallback, useDebounce, useDimensions, useDoubleClick, useElement, EventTracker, useEvent, useFocus, useForceUpdate, useGetSet, useHover, useMousePosition, useToggleButtonGroupState, useTrkl, useFetch });
Object.assign(globalThis, { clamp, identity, noop, compose, maybe, snd, toPair, getOffset, getPositionOnElement, isChildOf });
Object.assign(globalThis, {CodeEditor ,  highlight, languages});

function Start(args, address = '127.0.0.1:9901') {
  return ws.send(
    JSON.stringify({
      command: 'start',
      start: { connect: false, args, address }
    })
  );
}
let cwd = '.';

async function CreateSocket(url) {
  let ws = (globalThis.ws = new WebSocketClient());

  console.log('ws', ws);
  await ws.connect(url);

  (async function ReadSocket() {
    for await(let msg of ws) {
      let data;
      try {
        data = JSON.parse(msg.data);
      } catch(e) {
        console.log('WS ERROR parsing', msg.data);
      }
      globalThis.response = data;
      if(data) {
        console.log('WS', data);
        const { response } = data;
        if(response) {
          const { command } = response;

          if(command == 'file') {
            const { path, data } = response;
            CreateSource(data, path);
          } else if(command == 'start') {
            //const { cwd, args } = response;
            cwd = response.cwd;
            // Object.assign(globalThis, { cwd, args });
            console.log('start', response);
          }
        }
      }
    }
  })();

  await Start(['test-ecmascript2.js']);

  ws.sendMessage = function(msg) {
    return this.send(JSON.stringify(msg));
  };

  return ws;
}
let seq = 0;

function GetVariables(ref = 0) {
  SendRequest('variables', { variablesReference: ref });
}

function Step() {
  SendRequest('step');
}
function Next() {
  SendRequest('next');
}

function Continue() {
  SendRequest('continue');
}

function StackTrace() {
  SendRequest('stackTrace');
}

function SendRequest(command, args = {}) {
  ws.sendMessage({ type: 'request', request: { request_seq: ++seq, command, args } });
}

const SourceLine = ({ lineno, text }) => h(Fragment, {}, [h('pre', { class: 'lineno' }, [lineno + 1 + '']), h('pre', { class: 'text' }, [text])]);
const SourceFile = ({ filename }) => {
  const url = path.relative(cwd, filename);
  let text = useFetch(url) ?? '';
  return h(
    'div',
    { class: 'container' },
    [ h('div', { }, []), h('div', {class: 'header'}, [url])].concat(text.split('\n').map((text, lineno) => h(SourceLine, { lineno, text })))
  );
};

async function CreateSource(filename) {
  const component = h(SourceFile, { filename });
  const { body } = document;
  let r = render(component, body);
}
