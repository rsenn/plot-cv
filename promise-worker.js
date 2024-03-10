'use strict';

var messageIds = 0;

function onMessage(self, e) {
  var message = e.data;
  console.log('onMessage', message);

  if(!Array.isArray(message) || message.length < 2) {
    // Ignore - this message is not for us.
    return;
  }
  var messageId = message[0];
  var error = message[1];
  var result = message[2];

  console.log('onMessage', { messageId, error, result });

  var callback = self._callbacks[messageId];

  if(!callback) {
    // Ignore - user might have created multiple PromiseWorkers.
    // This message is not for us.
    return;
  }

  delete self._callbacks[messageId];
  callback(error, result);
}

export function PromiseWorker(worker) {
  var self = this;
  self._worker = worker;
  self._callbacks = {};

  worker.onmessage = function(e) {
    onMessage(self, e);
  };
}

PromiseWorker.prototype.postMessage = function(userMessage) {
  var self = this;
  var messageId = messageIds++;

  var messageToSend = [messageId, userMessage];

  return new Promise(function (resolve, reject) {
    self._callbacks[messageId] = function(error, result) {
      if(error) {
        return reject(new Error(error.message));
      }
      resolve(result);
    };

    /* istanbul ignore if */
    if(0 && typeof self._worker.controller !== 'undefined') {
      // service worker, use MessageChannels because e.source is broken in Chrome < 51:
      // https://bugs.chromium.org/p/chromium/issues/detail?id=543198
      var channel = new MessageChannel();
      channel.port1.onmessage = function(e) {
        onMessage(self, e);
      };
      self._worker.controller.postMessage(messageToSend, [channel.port2]);
    } else {
      // web worker
      self._worker.postMessage(messageToSend);
    }
  });
};
