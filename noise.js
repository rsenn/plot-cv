var audioCtx = new (window.AudioContext || window.webkitAudioContext)();

// Create an empty three-second stereo buffer at the sample rate of the AudioContext
var buf = audioCtx.createBuffer(2, audioCtx.sampleRate * 3, audioCtx.sampleRate);

// Fill the buffer with white noise;
// just random values between -1.0 and 1.0
for(var channel = 0; channel < buf.numberOfChannels; channel++) {
  // This gives us the actual array that contains the data
  var nowBuffering = buf.getChannelData(channel);
  for(var i = 0; i < buf.length; i++) {
    // Math.random() is in [0; 1.0]
    // audio needs to be in [-1.0; 1.0]
    nowBuffering[i] = Math.random() * 2 - 1;
  }
}

// Get an AudioBufferSourceNode.
// This is the AudioNode to use when we want to play an AudioBuffer
var source = audioCtx.createBufferSource();

// set the buffer in the AudioBufferSourceNode
source.buffer = buf;

// connect the AudioBufferSourceNode to the
// destination so we can hear the sound
source.connect(audioCtx.destination);

// start the source playing
source.start();
