import { Align, Container, CSSTransformSetters, Element, ElementPosProps, ElementTransformation, isNumber, Line, Matrix, Point, PointList, RandomColor, ScalarValue, Size, Timer, Transition, TransitionList, Tree, Unit, XPath } from './lib/dom.js';

window.addEventListener('load', async () => {
  console.log('midi-client.js loaded');

  let access = (globalThis.access = await navigator.requestMIDIAccess());

  let inputs = (globalThis.inputs = new Map(access.inputs.entries())),
    outputs = (globalThis.outputs = new Map(access.outputs.entries()));
});
