import { WebSocketClient } from './lib/net/websocket-async.js';
import Util from './lib/util.js';
import path from './lib/path.js';
import { DebuggerProtocol } from './debuggerprotocol.js';
/* prettier-ignore */ import { toString, toArrayBuffer, extendArray } from './lib/misc.js';
/* prettier-ignore */ import React, {h, html, render, Fragment, Component, useState, useLayoutEffect, useRef } from './lib/dom/preactComponent.js';
/* prettier-ignore */ import { EventEmitter, EventTarget } from './lib/events.js';
/* prettier-ignore */ import { DroppingBuffer, FixedBuffer, MAX_QUEUE_LENGTH, Repeater, RepeaterOverflowError, SlidingBuffer } from './lib/repeater/repeater.js';
/* prettier-ignore */ import { useAsyncIter, useRepeater, useResult, useValue } from './lib/repeater/react-hooks.js';
/* prettier-ignore */ import { TimeoutError, delay, interval, timeout } from './lib/repeater/timers.js';
/* prettier-ignore */ import { InMemoryPubSub } from './lib/repeater/pubsub.js';
/* prettier-ignore */ import { semaphore, throttler } from './lib/repeater/limiters.js';
import { trkl } from './lib/trkl.js';
import { classNames } from './lib/classNames.js';
/* prettier-ignore */ import { HSLA, RGBA, Point, isPoint, Size, isSize, Line, isLine, Rect, isRect, PointList, Polyline, Matrix, isMatrix, BBox, TRBL, Timer, Tree, Node, XPath, Element, isElement, CSS, SVG, Container, Layer, Renderer, Select, ElementPosProps, ElementRectProps, ElementRectProxy, ElementSizeProps, ElementWHProps, ElementXYProps, Align, Anchor, dom, isNumber, Unit, ScalarValue, ElementTransformation, CSSTransformSetters, Transition, TransitionList, RandomColor } from './lib/dom.js';
/* prettier-ignore */ import { useIterable, useIterator, useAsyncGenerator, useAsyncIterable, useAsyncIterator, useGenerator, useActive, useClickout, useConditional, useDebouncedCallback, useDebounce, useDimensions, useDoubleClick, useElement, EventTracker, useEvent, useFocus, useForceUpdate, useGetSet, useHover, useMousePosition, useToggleButtonGroupState, useTrkl, useFetch } from './lib/hooks.js';
/* prettier-ignore */ import { clamp, identity, noop, compose, maybe, snd, toPair, getOffset, getPositionOnElement, isChildOf } from './lib/hooks.js';
import { JSLexer, Location } from './lib/jslexer.js';

let cwd = '.';
let responses = {};
let currentSource = trkl();
let currentLine = trkl();

window.addEventListener('load', e => {
  let url = Util.parseURL();
  let socketURL = Util.makeURL({
    location: url.location + '/ws',
    protocol: url.protocol == 'https' ? 'wss' : 'ws'
  });

  (async () => {
    globalThis.ws = await CreateSocket(socketURL);
    console.log(`Loaded`, { socketURL, ws });
  })();
});

globalThis.addEventListener('keypress', e => {
  const handler = {
    KeyN: Next,
    KeyI: StepIn,
    KeyO: StepOut,
    KeyC: Continue,
    KeyP: Pause
  }[e.code];
  //console.log('keypress', e, handler);

  if(handler) handler();
});

async function LoadSource(filename) {
  try {
    let response = await fetch(filename);
    return await response.text();
  } catch(e) {}
}

/* prettier-ignore */ Object.assign(globalThis, { DebuggerProtocol, LoadSource, Util, toString, toArrayBuffer, extendArray, React, h, html, render, Fragment, Component, useState, useLayoutEffect, useRef, EventEmitter, EventTarget, Element, isElement, path });
/* prettier-ignore */ Object.assign(globalThis, { DroppingBuffer, FixedBuffer, MAX_QUEUE_LENGTH, Repeater, RepeaterOverflowError, SlidingBuffer, useAsyncIter, useRepeater, useResult, useValue, TimeoutError, delay, interval, timeout, InMemoryPubSub, semaphore, throttler, trkl });
/* prettier-ignore */ Object.assign(globalThis, { HSLA, RGBA, Point, isPoint, Size, isSize, Line, isLine, Rect, isRect, PointList, Polyline, Matrix, isMatrix, BBox, TRBL, Timer, Tree, Node, XPath, Element, isElement, CSS, SVG, Container, Layer, Renderer, Select, ElementPosProps, ElementRectProps, ElementRectProxy, ElementSizeProps, ElementWHProps, ElementXYProps, Align, Anchor, dom, isNumber, Unit, ScalarValue, ElementTransformation, CSSTransformSetters, Transition, TransitionList, RandomColor });
/* prettier-ignore */ Object.assign(globalThis, {   useIterable, useIterator, useAsyncGenerator, useAsyncIterable, useAsyncIterator, useGenerator, useActive, useClickout, useConditional, useDebouncedCallback, useDebounce, useDimensions, useDoubleClick, useElement, EventTracker, useEvent, useFocus, useForceUpdate, useGetSet, useHover, useMousePosition, useToggleButtonGroupState, useTrkl, useFetch });
/* prettier-ignore */ Object.assign(globalThis, { clamp, identity, noop, compose, maybe, snd, toPair, getOffset, getPositionOnElement, isChildOf });
/* prettier-ignore */ Object.assign(globalThis, { ShowSource, Start, GetVariables, SendRequest, StepIn,StepOut, Next, Continue, Pause, Evaluate, StackTrace });
/* prettier-ignore */ Object.assign(globalThis, {JSLexer, Location});

function Start(args, address = '127.0.0.1:9901') {
  return ws.send(
    JSON.stringify({
      command: 'start',
      start: { connect: false, args, address }
    })
  );
}

const tokenColors = {
  comment: new RGBA(0, 255, 0),
  regexpLiteral: new RGBA(255, 0, 255),
  templateLiteral: new RGBA(0, 255, 255),
  punctuator: new RGBA(0, 255, 255),
  numericLiteral: new RGBA(0, 255, 255),
  stringLiteral: new RGBA(0, 255, 255),
  booleanLiteral: new RGBA(255, 255, 0),
  nullLiteral: new RGBA(255, 0, 255),
  keyword: new RGBA(255, 0, 0),
  identifier: new RGBA(255, 255, 0),
  privateIdentifier: new RGBA(255, 255, 0),
  whitespace: new RGBA(255, 255, 255)
};

function* TokenizeJS(data, filename) {
  let lex = new JSLexer();
  lex.setInput(data, filename);

  let { tokens } = lex;
  let colors = Object.entries(tokenColors).reduce(
    (acc, [type, c]) => ({ ...acc, [tokens.indexOf(type) + 1]: c.hex() }),
    {}
  );
  let prev = {};
  yield '<pre>';

  for(let { id, lexeme } of lex) {
    const color = colors[id];
    const { line } = lex.loc;

    if(prev.color != color || prev.line != line) yield '</pre>';

    if(prev.line != line) {
      yield `<pre class="lineno"><a name="line-${line}">${line}</a></pre>`;
    }
    if(prev.color != color || prev.line != line) {
      yield `<pre style="color: ${color};">`;
    }
    console.log('tok', { lexeme, color, line });
    yield lexeme;

    prev.color = color;
    prev.line = line;
  }
  yield '</pre>';
}

Object.assign(globalThis, { responses, currentLine, currentSource, TokenizeJS });

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
        const { response, request_seq } = data;
        if(response) {
          const { command } = response;

          /* if(command == 'file') {
            const { path, data } = response;
            CreateSource(data, path);
            continue;
          } else*/ if(command == 'start') {
            cwd = response.cwd;
            console.log('start', response);
            ShowSource(response.args[0]);
            continue;
          }
        }
        if(responses[request_seq]) responses[request_seq](data);
      } else {
        console.log('WS', data);
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
  return SendRequest('variables', { variablesReference: ref });
}

async function UpdatePosition() {
  const stack = await StackTrace();
  //console.log('stack', stack);

  const { filename, line, name } = stack[0];

  ShowSource(filename);
  currentLine(line);

  window.location.hash = `#line-${line}`;
}

async function StepIn() {
  await SendRequest('stepIn');
  await UpdatePosition();
}

async function StepOut() {
  await SendRequest('stepOut');
  await UpdatePosition();
}

async function Next() {
  await SendRequest('next');
  await UpdatePosition();
}

async function Continue() {
  return SendRequest('continue');
}

async function Pause() {
  await SendRequest('pause');
  await UpdatePosition();
}

async function Evaluate(expression) {
  return SendRequest('evaluate', { expression });
}

async function StackTrace() {
  let { body } = await SendRequest('stackTrace');
  return body;
}

/*
  {
    "type": "breakpoints",
    "breakpoints": {
      "path": "lib/ecmascript/parser2.js",
      "breakpoints": [ { "line": 470, "column": 0 }, { "line": 2151, "column": 0 }, { "line": 2401, "column": 0 } ]
    }
  }
*/

function SendRequest(command, args = {}) {
  const request_seq = ++seq;
  ws.sendMessage({ type: 'request', request: { request_seq, command, args } });
  return new Promise((resolve, reject) => (responses[request_seq] = resolve));
}

const SourceLine = ({ lineno, text, active }) =>
  h(Fragment, {}, [
    h('pre', { class: 'lineno' }, h('a', { name: `line-${lineno}` }, [lineno + ''])),
    h('pre', { class: classNames('text', active && 'active') }, [text])
  ]);

const SourceText = ({ text, filename }) => {
  const activeLine = useTrkl(currentLine);
  let tokens = TokenizeJS(text, filename);

let data=[...tokens];
  console.log('useIterable',data);
  return h(
    Fragment,
    {},
    text.split('\n').map((text, i) => h(SourceLine, { lineno: i + 1, text, active: activeLine == i + 1 }))
  );
};

const SourceFile = ({ filename }) => {
  let text = useFetch(filename) ?? '';

  return h('div', { class: 'container' }, [
    h('div', {}, []),
    h('div', { class: 'header' }, [filename]),
    h(SourceText, { text, filename })
  ]);
};

async function ShowSource(sourceFile) {
  if(currentSource() != sourceFile) {
    currentSource(sourceFile);
    const component = h(SourceFile, { filename: path.relative(cwd, sourceFile, cwd) });
    const { body } = document;
    let r = render(component, body);
  }
}
