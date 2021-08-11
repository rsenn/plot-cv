window.addEventListener('load', () => {
  var canvas = document.getElementById('c');
  var gl = canvas.getContext('webgl');

  // Note: createProgramFromScripts will call bindAttribLocation
  // based on the index of the attibute names we pass to it.
  var program = twgl.createProgramFromScripts(gl, ['vshader', 'fshader'], ['a_position', 'a_textureIndex']);
  gl.useProgram(program);
  var imageLoc = gl.getUniformLocation(program, 'u_image');
  var paletteLoc = gl.getUniformLocation(program, 'u_palette');
  // tell it to use texture units 0 and 1 for the image and palette
  gl.uniform1i(imageLoc, 0);
  gl.uniform1i(paletteLoc, 1);

  // Setup a unit quad
  var positions = [1, 1, -1, 1, -1, -1, 1, 1, -1, -1, 1, -1];
  var vertBuffer = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, vertBuffer);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(positions), gl.STATIC_DRAW);
  gl.enableVertexAttribArray(0);
  gl.vertexAttribPointer(0, 2, gl.FLOAT, false, 0, 0);

  // Setup a palette.
  var palette = new Uint8Array(256 * 4);

  // I'm lazy so just setting 4 colors in palette
  function setPalette(index, r, g, b, a) {
    palette[index * 4 + 0] = r;
    palette[index * 4 + 1] = g;
    palette[index * 4 + 2] = b;
    palette[index * 4 + 3] = a;
  }
  setPalette(1, 255, 0, 0, 255); // red
  setPalette(2, 0, 255, 0, 255); // green
  setPalette(3, 0, 0, 255, 255); // blue

  for(let i = 0; i < 256; i++) setPalette(i, i, i, i, 255);

  // make palette texture and upload palette
  gl.activeTexture(gl.TEXTURE1);
  var paletteTex = gl.createTexture();
  gl.bindTexture(gl.TEXTURE_2D, paletteTex);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
  gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, 256, 1, 0, gl.RGBA, gl.UNSIGNED_BYTE, palette);

  // Make image. Just going to make something 8x8
  var width = 256;
  var height = 256;
  var image = makeImage((x, y) => x ^ y); /*new Uint8Array([
    0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 2, 0, 0, 2, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 3, 3, 3, 3, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0
  ]);*/

  console.log('image:', image);

  // make image textures and upload image
  gl.activeTexture(gl.TEXTURE0);
  var imageTex = gl.createTexture();
  gl.bindTexture(gl.TEXTURE_2D, imageTex);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
  gl.texImage2D(gl.TEXTURE_2D, 0, gl.ALPHA, width, height, 0, gl.ALPHA, gl.UNSIGNED_BYTE, image);

  var frameCounter = 0;
  function render() {
    ++frameCounter;

    // skip 3 of 4 frames so the animation is not too fast
    if((frameCounter & 3) == 0) {
      //      rotatePalette(palette);
      reuploadImage();

      gl.drawArrays(gl.TRIANGLES, 0, positions.length / 2);
    }
    requestAnimationFrame(render);
  }

  function rotatePalette(p) {
    // rotate the 3 p colors
    var tempR = p[4 + 0];
    var tempG = p[4 + 1];
    var tempB = p[4 + 2];
    var tempA = p[4 + 3];
    setPalette(1, p[2 * 4 + 0], p[2 * 4 + 1], p[2 * 4 + 2], p[2 * 4 + 3]);
    setPalette(2, p[3 * 4 + 0], p[3 * 4 + 1], p[3 * 4 + 2], p[3 * 4 + 3]);
    setPalette(3, tempR, tempG, tempB, tempA);

    // re-upload p
    gl.activeTexture(gl.TEXTURE1);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, 256, 1, 0, gl.RGBA, gl.UNSIGNED_BYTE, p);
  }

  function reuploadImage() {
    // re-upload image
    gl.activeTexture(gl.TEXTURE0);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.ALPHA, width, height, 0, gl.ALPHA, gl.UNSIGNED_BYTE, image);
  }

  globalThis.makeImage = makeImage;

  function makeImage(fn) {
    if(typeof fn == 'string') fn = new Function('x', 'y', `return ${fn}`);
    console.log('fn:', fn);
    let i = 0;
    let a = new Uint8Array(width * height);
    for(let y = 0; y < height; y++) {
      for(let x = 0; x < width; x++) {
        a[i++] = fn(x, y) & 0xff;
      }
    }
    return (image = a);
  }

  render();
});
