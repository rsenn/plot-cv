
window.addEventListener('load', async () => {
  console.log('midi-client.js loaded');

  let access = (globalThis.access = await navigator.requestMIDIAccess());

  let inputs = (globalThis.inputs = new Map(access.inputs.entries())),
    outputs = (globalThis.outputs = new Map(access.outputs.entries()));
});