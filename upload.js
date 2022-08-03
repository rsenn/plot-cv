window.addEventListener('load', e => {
  console.log('upload.js loaded!');

  let form = document.querySelector('form');

  form.addEventListener('submit', e => {
    console.log('form.submit', e);
    e.preventDefault();

    UploadFiles();
    return false;
  });

  globalThis.ws = CreateWS();
});

function UploadFiles() {
  let input = document.querySelector('#file');

  // for(let file of input.files)
  UploadFile(input.files);
}

// upload JPEG files
function UploadFile(files) {
  //file??=   document.querySelector('#file');

  const formData = new FormData();

  for(let file of files) formData.append('file', file);

  fetch('upload.html', { method: 'POST', body: formData }).then(response => {
    console.log('response', response);
  });
}

function CreateWS() {
  let ws = new WebSocket(document.location.href.replace(/\/[^/]*$/, '/uploads').replace(/^[^:]*/, 'ws'));
  ws.onmessage = e => {
    console.log('onmessage', e);
  };

  return ws;
}

function XHRUpload(formData) {
  let form = document.querySelector('form');
  var xhr = new XMLHttpRequest();

  if(xhr.upload) {
    xhr.upload.addEventListener(
      'progress',
      function(e) {
        var pc = parseInt(100 - (e.loaded / e.total) * 100);

        console.log('pc', pc);
      },
      false
    );

    xhr.open('POST', form.action, true);
    xhr.setRequestHeader('Content-Type', 'multipart/form-data');
    xhr.send(formData);
  }
}

// file selection
function FileSelectHandler(e) {
  // cancel event and hover styling
  FileDragHover(e);

  // fetch FileList object
  var files = e.target.files || e.dataTransfer.files;

  // process all File objects
  for(var i = 0, f; (f = files[i]); i++) {
    ParseFile(f);
    UploadFile(f);
  }
}
