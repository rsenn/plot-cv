import PinkNoiseNode from './pinkNoise.js';
globalThis.PinkNoiseNode = PinkNoiseNode;

var audioCtx = (globalThis.audioCtx = new (window.AudioContext || window.webkitAudioContext)());

// Create an empty three-second stereo buffer at the sample rate of the AudioContext
var buf = audioCtx.createBuffer(2, audioCtx.sampleRate * 3, audioCtx.sampleRate);

// Fill the buffer with white noise; just random values between -1.0 and 1.0
for(var channel = 0; channel < buf.numberOfChannels; channel++) {
  var nowBuffering = buf.getChannelData(channel);
  for(var i = 0; i < buf.length; i++) nowBuffering[i] = Math.random() * 2 - 1;
}

let b = (globalThis.b = document.querySelector('button'));
let form = document.querySelector('form');

form.onsubmit = () => false;

// Get an AudioBufferSourceNode.  This is the AudioNode to use when we want to play an AudioBuffer
//var source = audioCtx.createBufferSource();

function NewPinkNoise() {
  globalThis.noise = new PinkNoiseNode(audioCtx);

  noise.connect(audioCtx.destination);
  noise.loop = false;
  noise.loopEnd = 4;
}

NewPinkNoise();

let runs = false;

b.addEventListener('click', () => {
  if(!b.disabled) {
    noise.start();
    runs = true;
    b.disabled = true;
    noise.onended = () => {
      b.disabled = false;
      runs = false;
      NewPinkNoise();
    };
  } else {
    runs = false;
  }
});