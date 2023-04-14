import { crosskit, CANVAS } from './lib/crosskit.js';
import { RGBA, HSLA } from './lib/color.js';
import { timer } from './lib/async/helpers.js';
import { WebSocketIterator, WebSocketURL, CreateWebSocket, ReconnectingWebSocket, StreamReadIterator } from './lib/async/websocket.js';
import { once, streamify, throttle, distinct, subscribe } from './lib/async/events.js';
import { memoize, define, isUndefined, properties, keys, unique, randStr, randInt } from './lib/misc.js';
import { isStream, AcquireReader, AcquireWriter, ArrayWriter, readStream, PipeTo, WritableRepeater, WriteIterator, AsyncWrite, AsyncRead, ReadFromIterator, WriteToRepeater, LogSink, StringReader, LineReader, DebugTransformStream, CreateWritableStream, CreateTransformStream, RepeaterSource, RepeaterSink, LineBufferStream, TextTransformStream, ChunkReader, ByteReader, PipeToRepeater, Reader, ReadAll, default as utils } from './lib/stream/utils.js';
import { Intersection, Matrix, isRect, Rect, Size, Point, Line, TransformationList, Vector } from './lib/geom.js';
import { Element, isElement, SVG } from './lib/dom.js';
import React, { h, html, render, Fragment, Component, createRef, useState, useLayoutEffect, useRef, toChildArray } from './lib/dom/preactComponent.js';
import { fire } from './fire/build/fire-debug.js';

function main() {
  define(
    globalThis,
    {fire},
    properties(
      {
        currentURL: () => new URL(import.meta.url),
        currentFile: () => globalThis.currentURL.pathname.replace(/^\//, '')
      },
      { memoize: true }
    )
  );

  const width = 320;
  const height = 200;
  const parent = document.body;

  crosskit.init({
    renderer: CANVAS,
    parent,
    w: width,
    h: height,
    alpha: false
  });

  function Reparent(canvas = document.getElementsByTagName('canvas')[0]) {
    canvas.parentElement.removeChild(canvas);
    document.getElementById('canvas').appendChild(canvas);
  }

  function GetElementMatrix(element) {
    let { transform } = Element.getCSS(divElement);

    return Matrix.fromCSS(transform);
  }

  function SetCrosshair(pos) {
    let ch = Element.find('#crosshair');
    let rect = Element.getRect(ch);

    console.log('SetCrosshair', { ch, rect, pos });
    rect.x = pos.x - rect.width / 2;
    rect.y = pos.y - rect.height / 2;

    Element.setRect(ch, rect);
  }

  function EventPositions(eventOrTouch) {
    let positions = unique(
      keys(eventOrTouch, 2)
        .filter(n => typeof n == 'string' && /[XY]$/.test(n))
        .map(n => n.slice(0, -1))
    );

    return positions.reduce((acc, key) => {
      acc[key] = new Point(eventOrTouch[key + 'X'], eventOrTouch[key + 'Y']);
      return acc;
    }, {});
  }

  function PositionMatrix(canvas = canvasElement, rect = canvasRect) {
    let vertical = rect.aspect() < 1;
    let topLeft = rect.toPoints()[vertical ? 1 : 0];
    let m = Matrix.identity().scale(canvas.width, canvas.height);

    if(vertical) m = m.rotate(-Math.PI / 2);

    m = m.scale(1 / rect.width, 1 / rect.height);
    m = m.translate(-topLeft.x, -topLeft.y);

    return m;
  }

  function PositionProcessor(canvas = canvasElement, rect = canvasRect) {
    let m = PositionMatrix(canvas, rect);
    return pos => new Point(...m.transformPoint(new Point(pos))).round(1);
  }

  function ProcessPosition(pos) {
    return PositionProcessor()(pos);
  }

  async function* MovementIterator(element) {
    let ev = await once(element, 'mousedown', 'touchstart');
    let type = ev.type.slice(0, 5);
    yield ev;
    console.log(type + ' start');
    for await(let event of streamify(['mouseup', 'mousemove', 'touchend', 'touchmove'], element)) {
      //event.preventDefault();
      if('touches' in event) {
        if(event.touches.length) {
          globalThis.mouseEvent = event;

          yield* [...event.touches].map(EventPositions);
        }
      } else {
        const { type, clientX: x, clientY: y } = event;
        let obj = { type, x, y };
        yield obj;
      }
      if(/(up|end)$/.test(event.type)) {
        console.log(type + ' end');
        break;
      }
    }
  }

  Reparent();

  crosskit.clear();
  crosskit.rect({
    x: 0,
    y: 0,
    width,
    height,
    fill: 'black',
    stroke: 'black',
    angle: 0
  });

  const buffer = new ArrayBuffer(width * (height + 2));
  const palette = CreatePalette();
  const paletteHSL = CreatePaletteHSL();

  const pixels = Array.from({ length: height + 2 }).map((v, i) => new Uint8ClampedArray(buffer, i * width, width));
  const { context } = crosskit;
  const image = context.createImageData(width, height);

  const { now, waitFor, animationFrame } = Util;
  const fps = 50;
  const matrix = new Matrix().translate(160, 100).scale(0.5);

  Object.assign(globalThis, {
    buffer,
    palette,
    paletteHSL,
    pixels,
    context,
    image,
    fps,
    matrix,
    Reparent,
    dom: { Element },
    geom: { Rect },
    MovementIterator,
    MouseHandler,
    GetElementMatrix,
    SetCrosshair,
    EventPositions,
    PositionProcessor,
    PositionMatrix,
    ProcessPosition
  });

  async function Loop() {
    const delay = 1000 / fps;
    const log = (t, name) => globalThis.doLog && console.log(`${name} timing: ${t.toFixed(3)}ms`);
    const fire = (...args) => Fire(...args);
    const redraw = (...args) => Redraw(...args);

    await once(window, 'load');

    Init();

    for(;;) {
      fire();
      redraw();
      await animationFrame(delay);
    }
  }

  function Fire() {
    for(let x = 0; x < width; x++) {
      pixels[height][x] = 255 - (RandomByte() % 128);
      pixels[height + 1][x] = 255 - (RandomByte() % 128);
    }

    for(let y = 0; y < height; y++) {
      for(let x = 0; x < width; x++) {
        const sum = [
          pixels[y + 1][Modulo(x - 1, width)],
          pixels[y + 1][x],
          pixels[y + 1][Modulo(x + 1, width)],
          pixels[y + 2][x]
        ].reduce((a, p) => a + (p | 0), 0);

        pixels[y][x] = (sum * 15) >>> 6;
      }
    }
  }

  async function Redraw() {
    const { data } = image;

    let i = 0;
    let t = [...matrix];

    for(let y = 0; y < height; y++) {
      for(let x = 0; x < width; x++) {
        const c = palette[pixels[y][x]];
        data[i++] = c.r;
        data[i++] = c.g;
        data[i++] = c.b;
        data[i++] = c.a;
      }
    }

    context.putImageData(image, 0, 0);
  }

  function CreatePalette() {
    const colors = new Array(256);

    for(let i = 0; i < 64; i++) {
      const value = i * 4;

      colors[i] = new RGBA(value, 0, 0);
      colors[i + 64] = new RGBA(255, value, 0);
      colors[i + 128] = new RGBA(255, 255, value);
      colors[i + 192] = new RGBA(255, 255, 255);
    }
    return colors;
  }

  function CreatePaletteHSL() {
    const colors = new Array(256);

    const hues = [
      new HSLA(0, 100, 0),
      new HSLA(0, 100, 50),
      new HSLA(30, 100, 50),
      new HSLA(60, 100, 50),
      new HSLA(60, 100, 100),
      new HSLA(60, 100, 100)
    ];

    const breakpoints = [0, 51, 80, 154, 205, 256];
    console.log('breakpoints:', breakpoints);

    for(let i = 0; i < 256; i++) {
      const hue = (v => (v == -1 ? () => hues.length - 2 : v => v))(breakpoints.findIndex(b => i < b));
      const range = breakpoints[hue] - 1 - breakpoints[hue - 1];

      colors[i] = HSLA.blend(hues[hue - 1], hues[hue], (i - breakpoints[hue - 1]) / range).toRGBA();
    }
    return colors;
  }

  // For random numbers, use "x = 181 * x + 359" from
  // Tom Dickens "Random Number Generator for Microcontrollers"
  // https://web.archive.org/web/20170323204917/http://home.earthlink.net/~tdickens/68hc11/random/68hc11random.html
  let scratch = 0;

  function RandomByte() {
    const value = 181 * scratch + 359;
    scratch = value >>> 0;
    return (value >>> 8) & 0xff;
  }

  function Modulo(n, m) {
    return ((n % m) + m) % m;
  }

  let element, rect, rc, mouseTransform;

  function Blaze(x, y) {
    for(let ty = y - 1; ty < y + 1; ty++) {
      for(let tx = x - 1; tx < x + 1; tx++) {
        pixels[ty][tx] = rc;
      }
    }

    pixels[y + 1][x] = rc;
  }

  function MouseHandler(e) {
    let { target, buttons, type } = e;

    if('touches' in e) {
      for(let touch of [...e.touches]) {
        const { clientX, clientY } = touch;
        MouseHandler({
          type,
          target,
          buttons,
          offsetX: Math.trunc(clientX) - rect.x,
          offsetY: Math.trunc(clientY) - rect.y
        });
      }
      return;
    }

    globalThis.pointerEvent = e;

    const x = Math.round((e.offsetX * width) / rect.width);
    const y = Math.round((e.offsetY * height) / rect.height);

    try {
      if(/(down|start)$/.test(type)) rc = pixels[y][x] > 0x30 ? 0 : RandomByte() | 0x80;

      Blaze(x, y);
    } catch(e) {}
  }

  function ResizeHandler(e) {
    rect = canvasRect;
    mouseTransform = PositionProcessor();
    console.log('ResizeHandler', { e, rect });
  }

  function OrientationChange(e) {
    rect = canvasRect;
    mouseTransform = PositionProcessor();
    console.log('OrientationChange', { e, rect });
  }

  Object.assign(globalThis, { RandomByte });

  function Init() {
    window.canvas = element = document.querySelector('canvas');

    define(
      globalThis,
      properties({
        canvasElement: () => Element.find('canvas'),
        divElement: () => Element.find('body > div:first-child')
      }),
      properties({
        windowRect: () => new Rect(window.innerWidth, window.innerHeight),
        bodyRect: () => Element.rect('body').round(0),
        canvasRect: () => Element.rect('canvas').round(0),
        divRect: () => Element.rect('body > div:first-child').round(0)
      }),
      {
        getRect
      },
      properties({
        transform: [
          () => new TransformationList(Element.getCSS('body > div:first-child').transform),
          value => Element.setCSS('body > div:first-child', { transform: value + '' })
        ]
      })
    );

    const SVGComponent = props => {
      const rect = Element.rect(document.body).round(1);
      const { center, width, height } = rect;

      return h(
        'svg',
        { version: '1.1', xmlns: 'http://www.w3.org/2000/svg', viewBox: [...rect].join(' '), width, height },
        [
          h('circle', {
            cx: center.x,
            cy: center.y,
            r: 250,
            stroke: '#0f0',
            'stroke-width': 1,
            fill: `rgba(80,80,80,0.3)`
          })
        ]
      );
    };

    let svgContainer = Element.create('div', {}, document.body);
    Element.setCSS(svgContainer, { position: 'absolute', top: 0, left: 0, zIndex: 99999999, pointerEvents: 'none' });

    /*render(h(SVGComponent), svgContainer);*/

    rect = canvasRect;
    mouseTransform = PositionProcessor();

    (async function() {
      for await(let event of streamify(['orientationchange', 'resize'], document)) {
        console.log(event.type, event);
        globalThis[event.type] = event;
      }
    })();

    window.addEventListener('resize', ResizeHandler, true);
    window.addEventListener('orientationchange', OrientationChange, true);

    /*   const handler = MouseHandler;

    subscribe(MovementIterator(element), handler);*/
  }

  globalThis.ws = (globalThis.rws ??= NewWS()).ws;

  (async function() {
    let trail = [],
      start,
      last;
    for(;;) {
      let prev, pt;

      for await(let ev of MovementIterator(window)) {
        if(!globalThis.cid) {
          globalThis.cid ??= MakeClientID();

          ws.send(JSON.stringify({ type: 'hello', cid }));
        }

        if(rect.inside(ev)) {
          let pt = (globalThis.mousePos = mouseTransform(ev));
          let diff = pt,
            t;
          if(prev) diff = pt.diff(prev);

          start ??= t = Date.now();

          trail.push({ ...pt, time: Date.now() - start });
          // trail.push(t - start + '/' + (prev && diff.x > 0 ? '+' : '') + diff.x + ',' + (prev && diff.y > 0 ? '+' : '') + diff.y);

          last = t;
          try {
            //console.log('blaze', ...pt);
            rc = pixels[pt.y][pt.x] > 0x30 ? 0 : RandomByte() | 0x80;

            Blaze(pt.x, pt.y);
          } catch(e) {}
          prev = pt;
        }
      }

      SendTrail();

      function SendTrail() {
        if(trail.length) {
          const payload = { type: 'blaze', start, trail: [...trail] };
          console.log('SendTrail', payload);
          ws.send(JSON.stringify(payload));
        }
        trail.splice(0, trail.length);
        start = undefined;
        prev = undefined;
      }
    }
  })();

  Loop();
}

function getRect(elem) {
  return new Rect((elem ?? divElement).getBoundingClientRect()).round(1);
}
function NewWS() {
  let url = WebSocketURL('/ws', { mirror: currentFile });
  let ws = new ReconnectingWebSocket(url, 'lws-mirror-protocol');
  (async function() {
    for await(let chunk of ws) {
      let data = JSON.parse(chunk);

      if(data.cid != globalThis.cid) console.log('WS receive:', data);
    }
  })();
  return (globalThis.ws = ws);
}

function MakeUUID(rng = Math.random) {
  return [8, 4, 4, 4, 12].map(n => randStr(n, '0123456789abcdef'), rng).join('-');
}
function MakeClientID(rng = Math.random) {
  return [4, 4, 4, 4]
    .map(n => randStr(n, ['ABCDEFGHIJKLMNOPQRSTUVWXYZ', 'abcdefghijklmnopqrstuvwxyz', '.-$'][randInt(0, 3)]), rng)
    .join('');
}

define(globalThis, { crosskit, RGBA, HSLA, Util, Matrix, TransformationList });
define(globalThis, { WebSocketIterator, WebSocketURL, CreateWebSocket, NewWS, ReconnectingWebSocket });
define(globalThis, { define, isUndefined, properties, keys });
define(globalThis, { once, streamify, throttle, distinct, subscribe });
define(globalThis, { Intersection, Matrix, isRect, Rect, Size, Point, Line, TransformationList, Vector });
define(globalThis, {
  timer,
  MakeUUID,
  MakeClientID,
  isStream,
  AcquireReader,
  AcquireWriter,
  ArrayWriter,
  readStream,
  PipeTo,
  WritableRepeater,
  WriteIterator,
  AsyncWrite,
  AsyncRead,
  ReadFromIterator,
  WriteToRepeater,
  LogSink,
  StringReader,
  LineReader,
  DebugTransformStream,
  CreateWritableStream,
  CreateTransformStream,
  RepeaterSource,
  RepeaterSink,
  LineBufferStream,
  TextTransformStream,
  ChunkReader,
  ByteReader,
  PipeToRepeater,
  Reader,
  ReadAll,
  StreamReadIterator
});

define(globalThis, {
  Element,
  isElement,
  SVG,
  React,
  h,
  html,
  render,
  Fragment,
  Component,
  createRef,
  useState,
  useLayoutEffect,
  useRef,
  toChildArray
});

main();
