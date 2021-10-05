import { WebSocketClient } from './lib/net/websocket-async.js';
import Util from './lib/util.js';
import path from './lib/path.js';
import { DebuggerProtocol } from './debuggerprotocol.js';
import { toString, toArrayBuffer, btoa as Base64Encode, atob as Base64Decode } from './lib/misc.js';
import React, { h, html, render, Fragment, Component, useState, useLayoutEffect, useRef } from './lib/dom/preactComponent.js';
import { ECMAScriptParser, Lexer } from './lib/ecmascript/parser.js';
import { EventEmitter, EventTarget } from './lib/events.js';
import { Element, isElement } from './lib/dom.js';

window.addEventListener('load', async e => {
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

Object.assign(globalThis, {
  DebuggerProtocol,
  LoadSource,
  Util,
  toString,
  toArrayBuffer,
  Base64Encode,
  Base64Decode,
  React,
  h,
  html,
  render,
  Fragment,
  Component,
  useState,
  useLayoutEffect,
  useRef,
  ECMAScriptParser,
  Lexer,
  EventEmitter,
  EventTarget,
  Element,
  isElement,
  path,
  CreateSource,
  Start,
  GetVariables
});

function Start(args, address = '127.0.0.1:9901') {
  return ws.send(
    JSON.stringify({
      command: 'start',
      start: { connect: false, args, address }
    })
  );
}

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
            const { cwd, args } = response;

            Object.assign(globalThis, { cwd, args });
            console.log('start', { cwd, args });
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
  ws.sendMessage({ type: 'request', request: { request_seq: ++seq, command: 'variables', args: { variablesReference: ref } } });
}

async function CreateSource(filename) {
  if(path.isAbsolute(filename)) filename = path.relative(cwd, filename);

  let response = await fetch(filename);
  let text = await response.text();
  const lines = text.split('\n');

  const container = Element.create('div', { class: 'container', 'data-filename': filename });

  lines.forEach((line, i) => {
    //  let e=Element.create('div', { class: 'line' }, container);
    Element.create('pre', { class: 'lineno', innerText: i + 1 + '' }, container);
    Element.create('pre', { class: 'text', innerText: line }, container);
  });

  const { body } = document;
  let elm, next;
  for(elm = body.firstElementChild; elm; elm = next) {
    next = elm.nextElementSibling;
    let data = elm.getAttribute('data-filename');
    if(data == filename) body.removeChild(elm);
    else elm.style.setProperty('display', 'none');
  }

  body.appendChild(container);
  return container;
}
