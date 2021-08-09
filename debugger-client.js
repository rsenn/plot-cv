import { WebSocketClient } from './lib/net/websocket-async.js';
import Util from './lib/util.js';
import { DebuggerProtocol } from './debuggerprotocol.js';
import { toString as ArrayBufferToString, toArrayBuffer as StringToArrayBuffer, btoa as Base64Encode, atob as Base64Decode } from './lib/misc.js';
import React, { h, html, render, Fragment, Component, useState, useLayoutEffect, useRef } from './lib/dom/preactComponent.js';
import { ECMAScriptParser, Lexer } from './lib/ecmascript/parser2.js';
import { EventEmitter, EventTarget } from './lib/events.js';
import * as dom from './lib/dom.js';
import rpc from './quickjs/net/rpc.js';

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
  ArrayBufferToString,
  StringToArrayBuffer,
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
  EventEmitter,EventTarget,
  dom,
  rpc
});

async function CreateSocket(url) {
  let ws = (Util.getGlobalObject().ws = new WebSocketClient());

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
      if(data) console.log('WS', data);
    }
  })();

  await ws.send(
    JSON.stringify({
      command: 'start',
      start: { connect: false, args: ['test-ecmascript2.js'], address: '127.0.0.1:9901' }
    })
  );

  ws.sendMessage = function(msg) {
    return this.send(JSON.stringify(msg));
  };

  return ws;
}
