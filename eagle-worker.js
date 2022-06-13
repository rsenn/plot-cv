if (false && 'serviceWorker' in navigator) {
  window.addEventListener('load', async function () {
    const script = '/sw.js';
    const { serviceWorker } = navigator;
    window.sw = serviceWorker;
    console.log('register service worker', serviceWorker);
    let result = await serviceWorker.register(script);
    console.log('Service Worker Registered', serviceWorker);
    const { controller } = serviceWorker;
    console.log('Service Worker controller', controller);
    window.swc = controller;
    controller.addEventListener('message', (event) => {
      console.debug('index.html MESSAGE:', event.data);
    });
    console.log('controller.postMessage', controller.postMessage);
    const messageChannel = new MessageChannel();
    controller.postMessage(
      {
        type: 'INIT_PORT'
      },
      [messageChannel.port2]
    );
    messageChannel.port1.onmessage = (event) => {
      console.log('Message from serviceWorker:', event.data);
    };
    controller.postMessage({
      type: 'REQUESTED'
    });
    let registration = await serviceWorker.ready;
    console.log('Service Worker ready', registration);
  });
}
