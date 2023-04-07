/* ---------------------- start of './lib/crosskit.js' ---------------------- */

//Crosskit,Portable And Lightweight Basic And Simple Cross Rendering Engine
//Cross Between DOM,WEBGL,CANVAS,SVG
//By Rabia Alhaffar,Built In 11/April/2020
//3rd Party Libraries Only Used: webgl-2d.js
//For Crosskit Code,Start From Line 1310
/**
 *  WebGL-2D.js - HTML5 Canvas2D API in a WebGL context
 *
 *  Created by Corban Brook <corbanbrook@gmail.com> on 2011-03-02.
 *  Amended to by Bobby Richter <secretrobotron@gmail.com> on 2011-03-03
 *  CubicVR.js by Charles Cliffe <cj@cubicproductions.com> on 2011-03-03
 *
 */

/*
 *  Copyright (c) 2011 Corban Brook
 *
 *  Permission is hereby granted, free of charge, to any person obtaining
 *  a copy of this software and associated documentation files (the
 *  "Software"), to deal in the Software without restriction, including
 *  without limitation the rights to use, copy, modify, merge, publish,
 *  distribute, sublicense, and/or sell copies of the Software, and to
 *  permit persons to whom the Software is furnished to do so, subject to
 *  the following conditions:
 *
 *  The above copyright notice and this permission notice shall be
 *  included in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 *  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 *  LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 *  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 *  WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

/**
 * Usage:
 *
 *    var cvs = document.getElementById("myCanvas");
 *
 *    WebGL2D.enable(cvs);  //adds "webgl-2d" to cvs
 *
 *    cvs.getContext("webgl-2d");
 *
 */

//Vector & Matrix libraries from CubicVR.js
let M_PI = 3.1415926535897932384626433832795028841968;
let M_TWO_PI = 2.0 * M_PI;
let M_HALF_PI = M_PI / 2.0;

function isPOT(value) {
  return value > 0 && ((value - 1) & value) === 0;
}

let vec3 = {
  length(pt) {
    return Math.sqrt(pt[0] * pt[0] + pt[1] * pt[1] + pt[2] * pt[2]);
  },
  normalize(pt) {
    let d = Math.sqrt(pt[0] * pt[0] + pt[1] * pt[1] + pt[2] * pt[2]);
    if(d === 0) {
      return [0, 0, 0];
    }
    return [pt[0] / d, pt[1] / d, pt[2] / d];
  },
  dot(v1, v2) {
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
  },
  angle(v1, v2) {
    return Math.acos((v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]) / (Math.sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2]) * Math.sqrt(v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2])));
  },
  cross(vectA, vectB) {
    return [vectA[1] * vectB[2] - vectB[1] * vectA[2], vectA[2] * vectB[0] - vectB[2] * vectA[0], vectA[0] * vectB[1] - vectB[0] * vectA[1]];
  },
  multiply(vectA, constB) {
    return [vectA[0] * constB, vectA[1] * constB, vectA[2] * constB];
  },
  add(vectA, vectB) {
    return [vectA[0] + vectB[0], vectA[1] + vectB[1], vectA[2] + vectB[2]];
  },
  subtract(vectA, vectB) {
    return [vectA[0] - vectB[0], vectA[1] - vectB[1], vectA[2] - vectB[2]];
  },
  equal(a, b) {
    let epsilon = 0.0000001;
    if(a === undefined && b === undefined) {
      return true;
    }
    if(a === undefined || b === undefined) {
      return false;
    }
    return Math.abs(a[0] - b[0]) < epsilon && Math.abs(a[1] - b[1]) < epsilon && Math.abs(a[2] - b[2]) < epsilon;
  }
};

let mat3 = {
  identity: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],

  multiply(m1, m2) {
    let m10 = m1[0],
      m11 = m1[1],
      m12 = m1[2],
      m13 = m1[3],
      m14 = m1[4],
      m15 = m1[5],
      m16 = m1[6],
      m17 = m1[7],
      m18 = m1[8],
      m20 = m2[0],
      m21 = m2[1],
      m22 = m2[2],
      m23 = m2[3],
      m24 = m2[4],
      m25 = m2[5],
      m26 = m2[6],
      m27 = m2[7],
      m28 = m2[8];

    m2[0] = m20 * m10 + m23 * m11 + m26 * m12;
    m2[1] = m21 * m10 + m24 * m11 + m27 * m12;
    m2[2] = m22 * m10 + m25 * m11 + m28 * m12;
    m2[3] = m20 * m13 + m23 * m14 + m26 * m15;
    m2[4] = m21 * m13 + m24 * m14 + m27 * m15;
    m2[5] = m22 * m13 + m25 * m14 + m28 * m15;
    m2[6] = m20 * m16 + m23 * m17 + m26 * m18;
    m2[7] = m21 * m16 + m24 * m17 + m27 * m18;
    m2[8] = m22 * m16 + m25 * m17 + m28 * m18;
  },
  vec2_multiply(m1, m2) {
    let mOut = [];
    mOut[0] = m2[0] * m1[0] + m2[3] * m1[1] + m2[6];
    mOut[1] = m2[1] * m1[0] + m2[4] * m1[1] + m2[7];
    return mOut;
  },
  transpose(m) {
    return [m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]];
  }
}; //mat3

//Transform library from CubicVR.js
function Transform(mat) {
  return this.clearStack(mat);
}

let STACK_DEPTH_LIMIT = 16;

Transform.prototype.clearStack = function(init_mat) {
  this.m_stack = [];
  this.m_cache = [];
  this.c_stack = 0;
  this.valid = 0;
  this.result = null;

  for(let i = 0; i < STACK_DEPTH_LIMIT; i++) {
    this.m_stack[i] = this.getIdentity();
  }

  if(init_mat !== undefined) {
    this.m_stack[0] = init_mat;
  } else {
    this.setIdentity();
  }
}; //clearStack

Transform.prototype.setIdentity = function() {
  this.m_stack[this.c_stack] = this.getIdentity();
  if(this.valid === this.c_stack && this.c_stack) {
    this.valid--;
  }
};

Transform.prototype.getIdentity = function() {
  return [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
};

Transform.prototype.getResult = function() {
  if(!this.c_stack) {
    return this.m_stack[0];
  }

  let m = mat3.identity;

  if(this.valid > this.c_stack - 1) {
    this.valid = this.c_stack - 1;
  }

  for(let i = this.valid; i < this.c_stack + 1; i++) {
    m = mat3.multiply(this.m_stack[i], m);
    this.m_cache[i] = m;
  }

  this.valid = this.c_stack - 1;

  this.result = this.m_cache[this.c_stack];

  return this.result;
};

Transform.prototype.pushMatrix = function() {
  this.c_stack++;
  this.m_stack[this.c_stack] = this.getIdentity();
};

Transform.prototype.popMatrix = function() {
  if(this.c_stack === 0) {
    return;
  }
  this.c_stack--;
};

let translateMatrix = Transform.prototype.getIdentity();

Transform.prototype.translate = function(x, y) {
  translateMatrix[6] = x;
  translateMatrix[7] = y;

  mat3.multiply(translateMatrix, this.m_stack[this.c_stack]);

  /*
      if(this.valid === this.c_stack && this.c_stack) {
        this.valid--;
      }
      */
};

let scaleMatrix = Transform.prototype.getIdentity();

Transform.prototype.scale = function(x, y) {
  scaleMatrix[0] = x;
  scaleMatrix[4] = y;

  mat3.multiply(scaleMatrix, this.m_stack[this.c_stack]);

  /*
      if(this.valid === this.c_stack && this.c_stack) {
        this.valid--;
      }
      */
};

let rotateMatrix = Transform.prototype.getIdentity();

Transform.prototype.rotate = function(ang) {
  let sAng, cAng;

  sAng = Math.sin(-ang);
  cAng = Math.cos(-ang);

  rotateMatrix[0] = cAng;
  rotateMatrix[3] = sAng;
  rotateMatrix[1] = -sAng;
  rotateMatrix[4] = cAng;

  mat3.multiply(rotateMatrix, this.m_stack[this.c_stack]);

  /*
      if(this.valid === this.c_stack && this.c_stack) {
        this.valid--;
      }
      */
};

let WebGL2D = /*this.WebGL2D =*/ function WebGL2D(canvas, options) {
  this.canvas = canvas;
  this.options = options || {};
  this.gl = undefined;
  this.fs = undefined;
  this.vs = undefined;
  this.shaderProgram = undefined;
  this.transform = new Transform();
  this.shaderPool = [];
  this.maxTextureSize = undefined;

  //Save a reference to the WebGL2D instance on the canvas object
  canvas.gl2d = this;

  //Store getContext function for later use
  canvas.$getContext = canvas.getContext;

  //Override getContext function with "webgl-2d" enabled version
  canvas.getContext = (function (gl2d) {
    return function(context) {
      if((gl2d.options.force || context === 'webgl-2d') && !(canvas.width === 0 || canvas.height === 0)) {
        if(gl2d.gl) {
          return gl2d.gl;
        }

        let gl = (gl2d.gl = gl2d.canvas.$getContext('experimental-webgl'));

        gl2d.initShaders();
        gl2d.initBuffers();

        //Append Canvas2D API features to the WebGL context
        gl2d.initCanvas2DAPI();

        gl.viewport(0, 0, gl2d.canvas.width, gl2d.canvas.height);

        //Default white background
        gl.clearColor(1, 1, 1, 1);
        gl.clear(gl.COLOR_BUFFER_BIT); //| gl.DEPTH_BUFFER_BIT);

        //Disables writing to dest-alpha
        gl.colorMask(1, 1, 1, 0);

        //Depth options
        //gl.enable(gl.DEPTH_TEST);
        //gl.depthFunc(gl.LEQUAL);

        //Blending options
        gl.enable(gl.BLEND);
        gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);

        gl2d.maxTextureSize = gl.getParameter(gl.MAX_TEXTURE_SIZE);

        return gl;
      }
      return gl2d.canvas.$getContext(context);
    };
  })(this);

  this.postInit();
};

//Enables WebGL2D on your canvas
WebGL2D.enable = function(canvas, options) {
  return canvas.gl2d || new WebGL2D(canvas, options);
};

//Shader Pool BitMasks, i.e. sMask = (shaderMask.texture+shaderMask.stroke)
let shaderMask = {
  texture: 1,
  crop: 2,
  path: 4
};

//Fragment shader source
WebGL2D.prototype.getFragmentShaderSource = function getFragmentShaderSource(sMask) {
  let fsSource = [
    '#ifdef GL_ES',
    'precision highp float;',
    '#endif',
    '#define hasTexture ' + (sMask & shaderMask.texture ? '1' : '0'),
    '#define hasCrop ' + (sMask & shaderMask.crop ? '1' : '0'),
    'varying vec4 vColor;',
    '#if hasTexture',
    'varying vec2 vTextureCoord;',
    'uniform sampler2D uSampler;',
    '#if hasCrop',
    'uniform vec4 uCropSource;',
    '#endif',
    '#endif',
    'void main(void) {',
    '#if hasTexture',
    '#if hasCrop',
    'gl_FragColor = texture2D(uSampler, vec2(vTextureCoord.x * uCropSource.z, vTextureCoord.y * uCropSource.w) + uCropSource.xy);',
    '#else',
    'gl_FragColor = texture2D(uSampler, vTextureCoord);',
    '#endif',
    '#else',
    'gl_FragColor = vColor;',
    '#endif',
    '}'
  ].join('\n');

  return fsSource;
};

WebGL2D.prototype.getVertexShaderSource = function getVertexShaderSource(stackDepth, sMask) {
  let w = 2 / this.canvas.width,
    h = -2 / this.canvas.height;

  stackDepth = stackDepth || 1;

  let vsSource = [
    '#define hasTexture ' + (sMask & shaderMask.texture ? '1' : '0'),
    'attribute vec4 aVertexPosition;',
    '#if hasTexture',
    'varying vec2 vTextureCoord;',
    '#endif',
    'uniform vec4 uColor;',
    'uniform mat3 uTransforms[' + stackDepth + '];',
    'varying vec4 vColor;',
    'const mat4 pMatrix = mat4(' + w + ',0,0,0, 0,' + h + ',0,0, 0,0,1.0,1.0, -1.0,1.0,0,0);',
    'mat3 crunchStack(void) {',
    'mat3 result = uTransforms[0];',
    'for (int i = 1; i < ' + stackDepth + '; ++i) {',
    'result = uTransforms[i] * result;',
    '}',
    'return result;',
    '}',
    'void main(void) {',
    'vec3 position = crunchStack() * vec3(aVertexPosition.x, aVertexPosition.y, 1.0);',
    'gl_Position = pMatrix * vec4(position, 1.0);',
    'vColor = uColor;',
    '#if hasTexture',
    'vTextureCoord = aVertexPosition.zw;',
    '#endif',
    '}'
  ].join('\n');
  return vsSource;
};

//Initialize fragment and vertex shaders
WebGL2D.prototype.initShaders = function initShaders(transformStackDepth, sMask) {
  let gl = this.gl;

  transformStackDepth = transformStackDepth || 1;
  sMask = sMask || 0;
  let storedShader = this.shaderPool[transformStackDepth];

  if(!storedShader) {
    storedShader = this.shaderPool[transformStackDepth] = [];
  }
  storedShader = storedShader[sMask];

  if(storedShader) {
    gl.useProgram(storedShader);
    this.shaderProgram = storedShader;
    return storedShader;
  }
  let fs = (this.fs = gl.createShader(gl.FRAGMENT_SHADER));
  gl.shaderSource(this.fs, this.getFragmentShaderSource(sMask));
  gl.compileShader(this.fs);

  if(!gl.getShaderParameter(this.fs, gl.COMPILE_STATUS)) {
    throw 'fragment shader error: ' + gl.getShaderInfoLog(this.fs);
  }

  let vs = (this.vs = gl.createShader(gl.VERTEX_SHADER));
  gl.shaderSource(this.vs, this.getVertexShaderSource(transformStackDepth, sMask));
  gl.compileShader(this.vs);

  if(!gl.getShaderParameter(this.vs, gl.COMPILE_STATUS)) {
    throw 'vertex shader error: ' + gl.getShaderInfoLog(this.vs);
  }

  let shaderProgram = (this.shaderProgram = gl.createProgram());
  shaderProgram.stackDepth = transformStackDepth;
  gl.attachShader(shaderProgram, fs);
  gl.attachShader(shaderProgram, vs);
  gl.linkProgram(shaderProgram);

  if(!gl.getProgramParameter(shaderProgram, gl.LINK_STATUS)) {
    throw 'Could not initialise shaders.';
  }

  gl.useProgram(shaderProgram);

  shaderProgram.vertexPositionAttribute = gl.getAttribLocation(shaderProgram, 'aVertexPosition');
  gl.enableVertexAttribArray(shaderProgram.vertexPositionAttribute);

  shaderProgram.uColor = gl.getUniformLocation(shaderProgram, 'uColor');
  shaderProgram.uSampler = gl.getUniformLocation(shaderProgram, 'uSampler');
  shaderProgram.uCropSource = gl.getUniformLocation(shaderProgram, 'uCropSource');

  shaderProgram.uTransforms = [];
  for(let i = 0; i < transformStackDepth; ++i) {
    shaderProgram.uTransforms[i] = gl.getUniformLocation(shaderProgram, 'uTransforms[' + i + ']');
  } //for
  this.shaderPool[transformStackDepth][sMask] = shaderProgram;
  return shaderProgram;
  //if
};

let rectVertexPositionBuffer;
let rectVertexColorBuffer;

let pathVertexPositionBuffer;
let pathVertexColorBuffer;

//2D Vertices and Texture UV coords
let rectVerts = new Float32Array([0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0]);

WebGL2D.prototype.initBuffers = function initBuffers() {
  let gl = this.gl;

  rectVertexPositionBuffer = gl.createBuffer();
  rectVertexColorBuffer = gl.createBuffer();

  pathVertexPositionBuffer = gl.createBuffer();
  pathVertexColorBuffer = gl.createBuffer();

  gl.bindBuffer(gl.ARRAY_BUFFER, rectVertexPositionBuffer);
  gl.bufferData(gl.ARRAY_BUFFER, rectVerts, gl.STATIC_DRAW);
};

//Maintains an array of all WebGL2D instances
WebGL2D.instances = [];

WebGL2D.prototype.postInit = function() {
  WebGL2D.instances.push(this);
};

//Extends gl context with Canvas2D API
WebGL2D.prototype.initCanvas2DAPI = function initCanvas2DAPI() {
  let gl2d = this,
    gl = this.gl;

  //Rendering Canvas for text fonts
  let textCanvas = document.createElement('canvas');
  textCanvas.width = gl2d.canvas.width;
  textCanvas.height = gl2d.canvas.height;
  let textCtx = textCanvas.getContext('2d');

  let reRGBAColor = /^rgb(a)?\(\s*(-?[\d]+)(%)?\s*,\s*(-?[\d]+)(%)?\s*,\s*(-?[\d]+)(%)?\s*,?\s*(-?[\d\.]+)?\s*\)$/;
  let reHSLAColor = /^hsl(a)?\(\s*(-?[\d\.]+)\s*,\s*(-?[\d\.]+)%\s*,\s*(-?[\d\.]+)%\s*,?\s*(-?[\d\.]+)?\s*\)$/;
  let reHex6Color = /^#([0-9A-Fa-f]{6})$/;
  let reHex3Color = /^#([0-9A-Fa-f])([0-9A-Fa-f])([0-9A-Fa-f])$/;

  function HSLAToRGBA(h, s, l, a) {
    let r, g, b, m1, m2;

    //Clamp and Normalize values
    h = (((h % 360) + 360) % 360) / 360;
    s = s > 100 ? 1 : s / 100;
    s = s < 0 ? 0 : s;
    l = l > 100 ? 1 : l / 100;
    l = l < 0 ? 0 : l;

    m2 = l <= 0.5 ? l * (s + 1) : l + s - l * s;
    m1 = l * 2 - m2;

    function getHue(value) {
      let hue;

      if(value * 6 < 1) {
        hue = m1 + (m2 - m1) * value * 6;
      } else if(value * 2 < 1) {
        hue = m2;
      } else if(value * 3 < 2) {
        hue = m1 + (m2 - m1) * (2 / 3 - value) * 6;
      } else {
        hue = m1;
      }

      return hue;
    }

    r = getHue(h + 1 / 3);
    g = getHue(h);
    b = getHue(h - 1 / 3);

    return [r, g, b, a];
  }

  //Converts rgb(a) color string to gl color vector
  function colorStringToVec4(value) {
    let result = [],
      match,
      channel,
      isPercent,
      hasAlpha,
      alphaChannel,
      sameType;

    if((match = reRGBAColor.exec(value))) {
      (hasAlpha = match[1]), (alphaChannel = parseFloat(match[8]));

      if((hasAlpha && isNaN(alphaChannel)) || (!hasAlpha && !isNaN(alphaChannel))) {
        return false;
      }

      sameType = match[3];

      for(let i = 2; i < 8; i += 2) {
        (channel = match[i]), (isPercent = match[i + 1]);

        if(isPercent !== sameType) {
          return false;
        }

        //Clamp and normalize values
        if(isPercent) {
          channel = channel > 100 ? 1 : channel / 100;
          channel = channel < 0 ? 0 : channel;
        } else {
          channel = channel > 255 ? 1 : channel / 255;
          channel = channel < 0 ? 0 : channel;
        }

        result.push(channel);
      }

      result.push(hasAlpha ? alphaChannel : 1.0);
    } else if((match = reHSLAColor.exec(value))) {
      (hasAlpha = match[1]), (alphaChannel = parseFloat(match[5]));
      result = HSLAToRGBA(match[2], match[3], match[4], parseFloat(hasAlpha && alphaChannel ? alphaChannel : 1.0));
    } else if((match = reHex6Color.exec(value))) {
      let colorInt = parseInt(match[1], 16);
      result = [((colorInt & 0xff0000) >> 16) / 255, ((colorInt & 0x00ff00) >> 8) / 255, (colorInt & 0x0000ff) / 255, 1.0];
    } else if((match = reHex3Color.exec(value))) {
      let hexString = '#' + [match[1], match[1], match[2], match[2], match[3], match[3]].join('');
      result = colorStringToVec4(hexString);
    } else if(value.toLowerCase() in colorKeywords) {
      result = colorStringToVec4(colorKeywords[value.toLowerCase()]);
    } else if(value.toLowerCase() === 'transparent') {
      result = [0, 0, 0, 0];
    } else {
      //Color keywords not yet implemented, ie "orange", return hot pink
      return false;
    }

    return result;
  }

  function colorVecToString(vec4) {
    return 'rgba(' + vec4[0] * 255 + ', ' + vec4[1] * 255 + ', ' + vec4[2] * 255 + ', ' + parseFloat(vec4[3]) + ')';
  }

  var colorKeywords = {
    aliceblue: '#f0f8ff',
    antiquewhite: '#faebd7',
    aqua: '#00ffff',
    aquamarine: '#7fffd4',
    azure: '#f0ffff',
    beige: '#f5f5dc',
    bisque: '#ffe4c4',
    black: '#000000',
    blanchedalmond: '#ffebcd',
    blue: '#0000ff',
    blueviolet: '#8a2be2',
    brown: '#a52a2a',
    burlywood: '#deb887',
    cadetblue: '#5f9ea0',
    chartreuse: '#7fff00',
    chocolate: '#d2691e',
    coral: '#ff7f50',
    cornflowerblue: '#6495ed',
    cornsilk: '#fff8dc',
    crimson: '#dc143c',
    cyan: '#00ffff',
    darkblue: '#00008b',
    darkcyan: '#008b8b',
    darkgoldenrod: '#b8860b',
    darkgray: '#a9a9a9',
    darkgreen: '#006400',
    darkkhaki: '#bdb76b',
    darkmagenta: '#8b008b',
    darkolivegreen: '#556b2f',
    darkorange: '#ff8c00',
    darkorchid: '#9932cc',
    darkred: '#8b0000',
    darksalmon: '#e9967a',
    darkseagreen: '#8fbc8f',
    darkslateblue: '#483d8b',
    darkslategray: '#2f4f4f',
    darkturquoise: '#00ced1',
    darkviolet: '#9400d3',
    deeppink: '#ff1493',
    deepskyblue: '#00bfff',
    dimgray: '#696969',
    dodgerblue: '#1e90ff',
    firebrick: '#b22222',
    floralwhite: '#fffaf0',
    forestgreen: '#228b22',
    fuchsia: '#ff00ff',
    gainsboro: '#dcdcdc',
    ghostwhite: '#f8f8ff',
    gold: '#ffd700',
    goldenrod: '#daa520',
    gray: '#808080',
    green: '#008000',
    greenyellow: '#adff2f',
    grey: '#808080',
    honeydew: '#f0fff0',
    hotpink: '#ff69b4',
    indianred: '#cd5c5c',
    indigo: '#4b0082',
    ivory: '#fffff0',
    khaki: '#f0e68c',
    lavender: '#e6e6fa',
    lavenderblush: '#fff0f5',
    lawngreen: '#7cfc00',
    lemonchiffon: '#fffacd',
    lightblue: '#add8e6',
    lightcoral: '#f08080',
    lightcyan: '#e0ffff',
    lightgoldenrodyellow: '#fafad2',
    lightgrey: '#d3d3d3',
    lightgreen: '#90ee90',
    lightpink: '#ffb6c1',
    lightsalmon: '#ffa07a',
    lightseagreen: '#20b2aa',
    lightskyblue: '#87cefa',
    lightslategray: '#778899',
    lightsteelblue: '#b0c4de',
    lightyellow: '#ffffe0',
    lime: '#00ff00',
    limegreen: '#32cd32',
    linen: '#faf0e6',
    magenta: '#ff00ff',
    maroon: '#800000',
    mediumaquamarine: '#66cdaa',
    mediumblue: '#0000cd',
    mediumorchid: '#ba55d3',
    mediumpurple: '#9370d8',
    mediumseagreen: '#3cb371',
    mediumslateblue: '#7b68ee',
    mediumspringgreen: '#00fa9a',
    mediumturquoise: '#48d1cc',
    mediumvioletred: '#c71585',
    midnightblue: '#191970',
    mintcream: '#f5fffa',
    mistyrose: '#ffe4e1',
    moccasin: '#ffe4b5',
    navajowhite: '#ffdead',
    navy: '#000080',
    oldlace: '#fdf5e6',
    olive: '#808000',
    olivedrab: '#6b8e23',
    orange: '#ffa500',
    orangered: '#ff4500',
    orchid: '#da70d6',
    palegoldenrod: '#eee8aa',
    palegreen: '#98fb98',
    paleturquoise: '#afeeee',
    palevioletred: '#d87093',
    papayawhip: '#ffefd5',
    peachpuff: '#ffdab9',
    peru: '#cd853f',
    pink: '#ffc0cb',
    plum: '#dda0dd',
    powderblue: '#b0e0e6',
    purple: '#800080',
    red: '#ff0000',
    rosybrown: '#bc8f8f',
    royalblue: '#4169e1',
    saddlebrown: '#8b4513',
    salmon: '#fa8072',
    sandybrown: '#f4a460',
    seagreen: '#2e8b57',
    seashell: '#fff5ee',
    sienna: '#a0522d',
    silver: '#c0c0c0',
    skyblue: '#87ceeb',
    slateblue: '#6a5acd',
    slategray: '#708090',
    snow: '#fffafa',
    springgreen: '#00ff7f',
    steelblue: '#4682b4',
    tan: '#d2b48c',
    teal: '#008080',
    thistle: '#d8bfd8',
    tomato: '#ff6347',
    turquoise: '#40e0d0',
    violet: '#ee82ee',
    wheat: '#f5deb3',
    white: '#ffffff',
    whitesmoke: '#f5f5f5',
    yellow: '#ffff00',
    yellowgreen: '#9acd32'
  };

  //Maintain drawing state params during gl.save and gl.restore. see saveDrawState() and restoreDrawState()
  let drawState = {},
    drawStateStack = [];

  //A fast simple shallow clone
  function cloneObject(obj) {
    let target = {};
    for(let i in obj) {
      if(obj.hasOwnProperty(i)) {
        target[i] = obj[i];
      }
    }
    return target;
  }

  function saveDrawState() {
    let bakedDrawState = {
      fillStyle: [drawState.fillStyle[0], drawState.fillStyle[1], drawState.fillStyle[2], drawState.fillStyle[3]],
      strokeStyle: [drawState.strokeStyle[0], drawState.strokeStyle[1], drawState.strokeStyle[2], drawState.strokeStyle[3]],
      globalAlpha: drawState.globalAlpha,
      globalCompositeOperation: drawState.globalCompositeOperation,
      lineCap: drawState.lineCap,
      lineJoin: drawState.lineJoin,
      lineWidth: drawState.lineWidth,
      miterLimit: drawState.miterLimit,
      shadowColor: drawState.shadowColor,
      shadowBlur: drawState.shadowBlur,
      shadowOffsetX: drawState.shadowOffsetX,
      shadowOffsetY: drawState.shadowOffsetY,
      textAlign: drawState.textAlign,
      font: drawState.font,
      textBaseline: drawState.textBaseline
    };

    drawStateStack.push(bakedDrawState);
  }

  function restoreDrawState() {
    if(drawStateStack.length) {
      drawState = drawStateStack.pop();
    }
  }

  //WebGL requires colors as a vector while Canvas2D sets colors as an rgba string
  //These getters and setters store the original rgba string as well as convert to a vector
  drawState.fillStyle = [0, 0, 0, 1]; //default black

  Object.defineProperty(gl, 'fillStyle', {
    get() {
      return colorVecToString(drawState.fillStyle);
    },
    set(value) {
      drawState.fillStyle = colorStringToVec4(value) || drawState.fillStyle;
    }
  });

  drawState.strokeStyle = [0, 0, 0, 1]; //default black

  Object.defineProperty(gl, 'strokeStyle', {
    get() {
      return colorVecToString(drawState.strokeStyle);
    },
    set(value) {
      drawState.strokeStyle = colorStringToVec4(value) || drawStyle.strokeStyle;
    }
  });

  //WebGL already has a lineWidth() function but Canvas2D requires a lineWidth property
  //Store the original lineWidth() function for later use
  gl.$lineWidth = gl.lineWidth;
  drawState.lineWidth = 1.0;

  Object.defineProperty(gl, 'lineWidth', {
    get() {
      return drawState.lineWidth;
    },
    set(value) {
      gl.$lineWidth(value);
      drawState.lineWidth = value;
    }
  });

  //Currently unsupported attributes and their default values
  drawState.lineCap = 'butt';

  Object.defineProperty(gl, 'lineCap', {
    get() {
      return drawState.lineCap;
    },
    set(value) {
      drawState.lineCap = value;
    }
  });

  drawState.lineJoin = 'miter';

  Object.defineProperty(gl, 'lineJoin', {
    get() {
      return drawState.lineJoin;
    },
    set(value) {
      drawState.lineJoin = value;
    }
  });

  drawState.miterLimit = 10;

  Object.defineProperty(gl, 'miterLimit', {
    get() {
      return drawState.miterLimit;
    },
    set(value) {
      drawState.miterLimit = value;
    }
  });

  drawState.shadowOffsetX = 0;

  Object.defineProperty(gl, 'shadowOffsetX', {
    get() {
      return drawState.shadowOffsetX;
    },
    set(value) {
      drawState.shadowOffsetX = value;
    }
  });

  drawState.shadowOffsetY = 0;

  Object.defineProperty(gl, 'shadowOffsetY', {
    get() {
      return drawState.shadowOffsetY;
    },
    set(value) {
      drawState.shadowOffsetY = value;
    }
  });

  drawState.shadowBlur = 0;

  Object.defineProperty(gl, 'shadowBlur', {
    get() {
      return drawState.shadowBlur;
    },
    set(value) {
      drawState.shadowBlur = value;
    }
  });

  drawState.shadowColor = 'rgba(0, 0, 0, 0.0)';

  Object.defineProperty(gl, 'shadowColor', {
    get() {
      return drawState.shadowColor;
    },
    set(value) {
      drawState.shadowColor = value;
    }
  });

  drawState.font = '10px sans-serif';

  Object.defineProperty(gl, 'font', {
    get() {
      return drawState.font;
    },
    set(value) {
      textCtx.font = value;
      drawState.font = value;
    }
  });

  drawState.textAlign = 'start';

  Object.defineProperty(gl, 'textAlign', {
    get() {
      return drawState.textAlign;
    },
    set(value) {
      drawState.textAlign = value;
    }
  });

  drawState.textBaseline = 'alphabetic';

  Object.defineProperty(gl, 'textBaseline', {
    get() {
      return drawState.textBaseline;
    },
    set(value) {
      drawState.textBaseline = value;
    }
  });

  //This attribute will need to control global alpha of objects drawn.
  drawState.globalAlpha = 1.0;

  Object.defineProperty(gl, 'globalAlpha', {
    get() {
      return drawState.globalAlpha;
    },
    set(value) {
      drawState.globalAlpha = value;
    }
  });

  //This attribute will need to set the gl.blendFunc mode
  drawState.globalCompositeOperation = 'source-over';

  Object.defineProperty(gl, 'globalCompositeOperation', {
    get() {
      return drawState.globalCompositeOperation;
    },
    set(value) {
      drawState.globalCompositeOperation = value;
    }
  });

  //Need a solution for drawing text that isnt stupid slow
  gl.fillText = function fillText(text, x, y) {
    textCtx.clearRect(0, 0, gl2d.canvas.width, gl2d.canvas.height);
    textCtx.fillStyle = gl.fillStyle;
    textCtx.fillText(text, x, y);

    gl.drawImage(textCanvas, 0, 0);
  };

  gl.strokeText = function strokeText() {};

  gl.measureText = function measureText() {
    return 1;
  };

  let tempCanvas = document.createElement('canvas');
  let tempCtx = tempCanvas.getContext('2d');

  gl.save = function save() {
    gl2d.transform.pushMatrix();
    saveDrawState();
  };

  gl.restore = function restore() {
    gl2d.transform.popMatrix();
    restoreDrawState();
  };

  gl.translate = function translate(x, y) {
    gl2d.transform.translate(x, y);
  };

  gl.rotate = function rotate(a) {
    gl2d.transform.rotate(a);
  };

  gl.scale = function scale(x, y) {
    gl2d.transform.scale(x, y);
  };

  gl.createImageData = function createImageData(width, height) {
    return tempCtx.createImageData(width, height);
  };

  gl.getImageData = function getImageData(x, y, width, height) {
    let data = tempCtx.createImageData(width, height);
    let buffer = new Uint8Array(width * height * 4);
    gl.readPixels(x, y, width, height, gl.RGBA, gl.UNSIGNED_BYTE, buffer);
    let w = width * 4,
      h = height;
    for(let i = 0, maxI = h / 2; i < maxI; ++i) {
      for(let j = 0, maxJ = w; j < maxJ; ++j) {
        let index1 = i * w + j;
        let index2 = (h - i - 1) * w + j;
        data.data[index1] = buffer[index2];
        data.data[index2] = buffer[index1];
      } //for
    } //for

    return data;
  };

  gl.putImageData = function putImageData(imageData, x, y) {
    gl.drawImage(imageData, x, y);
  };

  gl.transform = function transform(m11, m12, m21, m22, dx, dy) {
    let m = gl2d.transform.m_stack[gl2d.transform.c_stack];

    m[0] *= m11;
    m[1] *= m21;
    m[2] *= dx;
    m[3] *= m12;
    m[4] *= m22;
    m[5] *= dy;
    m[6] = 0;
    m[7] = 0;
  };

  function sendTransformStack(sp) {
    let stack = gl2d.transform.m_stack;
    for(let i = 0, maxI = gl2d.transform.c_stack + 1; i < maxI; ++i) {
      gl.uniformMatrix3fv(sp.uTransforms[i], false, stack[maxI - 1 - i]);
    } //for
  }

  gl.setTransform = function setTransform(m11, m12, m21, m22, dx, dy) {
    gl2d.transform.setIdentity();
    gl.transform.apply(this, arguments);
  };

  gl.fillRect = function fillRect(x, y, width, height) {
    let transform = gl2d.transform;
    let shaderProgram = gl2d.initShaders(transform.c_stack + 2, 0);

    gl.bindBuffer(gl.ARRAY_BUFFER, rectVertexPositionBuffer);
    gl.vertexAttribPointer(shaderProgram.vertexPositionAttribute, 4, gl.FLOAT, false, 0, 0);

    transform.pushMatrix();

    transform.translate(x, y);
    transform.scale(width, height);

    sendTransformStack(shaderProgram);

    gl.uniform4f(shaderProgram.uColor, drawState.fillStyle[0], drawState.fillStyle[1], drawState.fillStyle[2], drawState.fillStyle[3]);

    gl.drawArrays(gl.TRIANGLE_FAN, 0, 4);

    transform.popMatrix();
  };

  gl.strokeRect = function strokeRect(x, y, width, height) {
    let transform = gl2d.transform;
    let shaderProgram = gl2d.initShaders(transform.c_stack + 2, 0);

    gl.bindBuffer(gl.ARRAY_BUFFER, rectVertexPositionBuffer);
    gl.vertexAttribPointer(shaderProgram.vertexPositionAttribute, 4, gl.FLOAT, false, 0, 0);

    transform.pushMatrix();

    transform.translate(x, y);
    transform.scale(width, height);

    sendTransformStack(shaderProgram);

    gl.uniform4f(shaderProgram.uColor, drawState.strokeStyle[0], drawState.strokeStyle[1], drawState.strokeStyle[2], drawState.strokeStyle[3]);

    gl.drawArrays(gl.LINE_LOOP, 0, 4);

    transform.popMatrix();
  };

  gl.clearRect = function clearRect(x, y, width, height) {};

  let subPaths = [];

  function SubPath(x, y) {
    this.closed = false;
    this.verts = [x, y, 0, 0];
  }

  //Empty the list of subpaths so that the context once again has zero subpaths
  gl.beginPath = function beginPath() {
    subPaths.length = 0;
  };

  //Mark last subpath as closed and create a new subpath with the same starting point as the previous subpath
  gl.closePath = function closePath() {
    if(subPaths.length) {
      //Mark last subpath closed.
      let prevPath = subPaths[subPaths.length - 1],
        startX = prevPath.verts[0],
        startY = prevPath.verts[1];
      prevPath.closed = true;

      //Create new subpath using the starting position of previous subpath
      let newPath = new SubPath(startX, startY);
      subPaths.push(newPath);
    }
  };

  //Create a new subpath with the specified point as its first (and only) point
  gl.moveTo = function moveTo(x, y) {
    subPaths.push(new SubPath(x, y));
  };

  gl.lineTo = function lineTo(x, y) {
    if(subPaths.length) {
      subPaths[subPaths.length - 1].verts.push(x, y, 0, 0);
    } else {
      //Create a new subpath if none currently exist
      gl.moveTo(x, y);
    }
  };

  gl.quadraticCurveTo = function quadraticCurveTo(cp1x, cp1y, x, y) {};

  gl.bezierCurveTo = function bezierCurveTo(cp1x, cp1y, cp2x, cp2y, x, y) {};

  gl.arcTo = function arcTo() {};

  //Adds a closed rect subpath and creates a new subpath
  gl.rect = function rect(x, y, w, h) {
    gl.moveTo(x, y);
    gl.lineTo(x + w, y);
    gl.lineTo(x + w, y + h);
    gl.lineTo(x, y + h);
    gl.closePath();
  };

  gl.arc = function arc(x, y, radius, startAngle, endAngle, anticlockwise) {};

  function fillSubPath(index) {
    let transform = gl2d.transform;
    let shaderProgram = gl2d.initShaders(transform.c_stack + 2, 0);

    let subPath = subPaths[index];
    let verts = subPath.verts;

    gl.bindBuffer(gl.ARRAY_BUFFER, pathVertexPositionBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(verts), gl.STATIC_DRAW);

    gl.vertexAttribPointer(shaderProgram.vertexPositionAttribute, 4, gl.FLOAT, false, 0, 0);

    transform.pushMatrix();

    sendTransformStack(shaderProgram);

    gl.uniform4f(shaderProgram.uColor, drawState.fillStyle[0], drawState.fillStyle[1], drawState.fillStyle[2], drawState.fillStyle[3]);

    gl.drawArrays(gl.TRIANGLE_FAN, 0, verts.length / 4);

    transform.popMatrix();
  }

  gl.fill = function fill() {
    for(let i = 0; i < subPaths.length; i++) {
      fillSubPath(i);
    }
  };

  function strokeSubPath(index) {
    let transform = gl2d.transform;
    let shaderProgram = gl2d.initShaders(transform.c_stack + 2, 0);

    let subPath = subPaths[index];
    let verts = subPath.verts;

    gl.bindBuffer(gl.ARRAY_BUFFER, pathVertexPositionBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(verts), gl.STATIC_DRAW);

    gl.vertexAttribPointer(shaderProgram.vertexPositionAttribute, 4, gl.FLOAT, false, 0, 0);

    transform.pushMatrix();

    sendTransformStack(shaderProgram);

    gl.uniform4f(shaderProgram.uColor, drawState.strokeStyle[0], drawState.strokeStyle[1], drawState.strokeStyle[2], drawState.strokeStyle[3]);

    if(subPath.closed) {
      gl.drawArrays(gl.LINE_LOOP, 0, verts.length / 4);
    } else {
      gl.drawArrays(gl.LINE_STRIP, 0, verts.length / 4);
    }

    transform.popMatrix();
  }

  gl.stroke = function stroke() {
    for(let i = 0; i < subPaths.length; i++) {
      strokeSubPath(i);
    }
  };

  gl.clip = function clip() {};

  gl.isPointInPath = function isPointInPath() {};

  gl.drawFocusRing = function drawFocusRing() {};

  let imageCache = [],
    textureCache = [];

  function Texture(image) {
    this.obj = gl.createTexture();
    this.index = textureCache.push(this);

    imageCache.push(image);

    //we may wish to consider tiling large images like this instead of scaling and
    //adjust appropriately (flip to next texture source and tile offset) when drawing
    if(image.width > gl2d.maxTextureSize || image.height > gl2d.maxTextureSize) {
      let canvas = document.createElement('canvas');

      canvas.width = image.width > gl2d.maxTextureSize ? gl2d.maxTextureSize : image.width;
      canvas.height = image.height > gl2d.maxTextureSize ? gl2d.maxTextureSize : image.height;

      let ctx = canvas.getContext('2d');

      ctx.drawImage(image, 0, 0, image.width, image.height, 0, 0, canvas.width, canvas.height);

      image = canvas;
    }

    gl.bindTexture(gl.TEXTURE_2D, this.obj);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, image);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);

    //Enable Mip mapping on power-of-2 textures
    if(isPOT(image.width) && isPOT(image.height)) {
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR_MIPMAP_LINEAR);
      gl.generateMipmap(gl.TEXTURE_2D);
    } else {
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    }

    //Unbind texture
    gl.bindTexture(gl.TEXTURE_2D, null);
  }

  gl.drawImage = function drawImage(image, a, b, c, d, e, f, g, h) {
    let transform = gl2d.transform;

    transform.pushMatrix();

    let sMask = shaderMask.texture;
    let doCrop = false;

    //drawImage(image, dx, dy)
    if(arguments.length === 3) {
      transform.translate(a, b);
      transform.scale(image.width, image.height);
    }

    //drawImage(image, dx, dy, dw, dh)
    else if(arguments.length === 5) {
      transform.translate(a, b);
      transform.scale(c, d);
    }

    //drawImage(image, sx, sy, sw, sh, dx, dy, dw, dh)
    else if(arguments.length === 9) {
      transform.translate(e, f);
      transform.scale(g, h);
      sMask = sMask | shaderMask.crop;
      doCrop = true;
    }

    let shaderProgram = gl2d.initShaders(transform.c_stack, sMask);

    let texture,
      cacheIndex = imageCache.indexOf(image);

    if(cacheIndex !== -1) {
      texture = textureCache[cacheIndex];
    } else {
      texture = new Texture(image);
    }

    if(doCrop) {
      gl.uniform4f(shaderProgram.uCropSource, a / image.width, b / image.height, c / image.width, d / image.height);
    }

    gl.bindBuffer(gl.ARRAY_BUFFER, rectVertexPositionBuffer);
    gl.vertexAttribPointer(shaderProgram.vertexPositionAttribute, 4, gl.FLOAT, false, 0, 0);

    gl.bindTexture(gl.TEXTURE_2D, texture.obj);
    gl.activeTexture(gl.TEXTURE0);

    gl.uniform1i(shaderProgram.uSampler, 0);

    sendTransformStack(shaderProgram);
    gl.drawArrays(gl.TRIANGLE_FAN, 0, 4);

    transform.popMatrix();
  };
};

//Crosskit Rendering Engine
//Rendering Elements
let cakecanvas, cakepen, renderer, canvas, board, svg_board;
let biggest_x, biggest_y;

//Index Of Views Creation
let index = (biggest_x = biggest_y = 0),
  webgl_texts = 0,
  //Important Variables For Correction
  u,
  no_use = 'none',
  domvg_polygon_points = '',
  infinite = 'indefinite',
  //Getting Body Element
  //And Yes,There Must Be <body> Element To Use Crosskit
  body = document.body;

//If Shapes Are In SVG Or DOM Mode It Cannot Be Drawn Directly
//So It Will Be Stored In Array When Clearing Graphics
//As Shapes Are Objects To Be Drawn
let svg_shapes;
let dom_shapes = (svg_shapes = []);

//Lines And Polygons And Triangles Cannot Be Drawn In DOM Mode Using CSS Styles
//So We Will Use Some SVG Inside DOM With Storing These SVGs
let svg_anims, dom_svgs_shapes;
let dom_svgs = (dom_svgs_shapes = svg_anims = []);

//Texts In WebGL Stored Into Arrays With Their Canvas
let images;
let text_svg,
  texts = (images = []);

//Modes Of Rendering

const WEBGL = 'WEBGL',
  CANVAS = 'CANVAS',
  SVG = 'SVG',
  DOM = 'DOM'; //Simple

window.update = function() {
  return (
    window.requestAnimationFrame ||
    window.webkitRequestAnimationFrame ||
    window.mozRequestAnimationFrame ||
    window.msRequestAnimationFrame ||
    window.oRequestAnimationFrame ||
    function(callback, fps) {
      window.setTimeout(callback, 1000 / fps);
    }
  );
};

const crosskit = {
  compatible_width: window.innerWidth - 25,
  compatible_height: window.innerHeight - 25,
  version: '0.8.8',
  init(v) {
    renderer = v.renderer.toString();
    if(renderer == CANVAS) {
      canvas = document.createElement('canvas');
      canvas.width = v.w;
      canvas.height = v.h;
      /*canvas.style.position = 'relative';
      canvas.style.left = '8px';*/
      (v.parent || body.parentNode).appendChild(canvas);
      cakecanvas = document.getElementsByTagName('canvas')[index];
      cakepen = this.context = cakecanvas.getContext('2d', v);
    }

    if(renderer == SVG) {
      cakecanvas = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
      cakecanvas.setAttribute('width', v.w);
      cakecanvas.setAttribute('height', v.h);
      body.appendChild(cakecanvas);
    }

    if(renderer == WEBGL) {
      canvas = document.createElement('canvas');
      canvas.width = v.w;
      canvas.height = v.h;
      canvas.style.position = 'relative';
      canvas.style.left = '8px';
      body.parentNode.appendChild(canvas);
      cakecanvas = canvas;
      WebGL2D.enable(cakecanvas);
      cakepen = cakecanvas.getContext('webgl-2d');
    }

    if(renderer == DOM) {
      board = document.createElement('div');
      board.id = 'board';
      board.style.width = v.w;
      board.style.height = v.h;
      board.style.position = 'relative';
      board.style.bottom = '3px';
      body.appendChild(board);
      cakecanvas = board;
      svg_board = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
      svg_board.setAttribute('width', v.w);
      svg_board.setAttribute('height', v.h);
      svg_board.style.zIndex = 1;
      board.appendChild(svg_board);
    }
    index++; //Increase Index Of Elements Creation
    console.info('%cCROSSKIT ' + crosskit.version + '\nRendering Mode: ' + renderer, 'background-color: purple; color: white;');
  },
  line(v) {
    if(renderer == CANVAS || renderer == WEBGL) {
      cakepen.globalAlpha = v.a;
      cakepen.strokeStyle = v.stroke;
      cakepen.lineWidth = v.line_width;
      cakepen.rotate(v.angle / 50);
      cakepen.beginPath();
      cakepen.moveTo(v.pos1[0], v.pos1[1]);
      cakepen.lineTo(v.pos2[0], v.pos2[1]);
      cakepen.lineTo(v.pos1[0], v.pos1[1]);
      cakepen.closePath();
      cakepen.stroke();
      cakepen.globalAlpha = 1;
      cakepen.rotate(-v.angle);
    }
    if(renderer == DOM) {
      if(v.pos1[0] > biggest_x) biggest_x = v.pos1[0];
      if(v.pos1[1] > biggest_y) biggest_y = v.pos1[1];
      if(v.pos2[0] > biggest_x) biggest_x = v.pos2[0];
      if(v.pos2[1] > biggest_y) biggest_y = v.pos2[1];
      dom_svgs_shapes.push(document.createElementNS('http://www.w3.org/2000/svg', 'line'));
      dom_svgs_shapes[dom_svgs_shapes.length - 1].setAttribute('x1', v.pos1[0].toString());
      dom_svgs_shapes[dom_svgs_shapes.length - 1].setAttribute('y1', v.pos1[1].toString());
      dom_svgs_shapes[dom_svgs_shapes.length - 1].setAttribute('x2', v.pos2[0].toString());
      dom_svgs_shapes[dom_svgs_shapes.length - 1].setAttribute('y2', v.pos2[1].toString());
      dom_svgs_shapes[dom_svgs_shapes.length - 1].setAttribute('stroke', v.stroke);
      dom_svgs_shapes[dom_svgs_shapes.length - 1].style.strokeWidth = v.line_width;
      dom_svgs_shapes[dom_svgs_shapes.length - 1].style.transform = 'rotate(' + v.angle + 'deg)';
      dom_svgs_shapes[dom_svgs_shapes.length - 1].style.opacity = v.a;
      svg_board.appendChild(dom_svgs_shapes[dom_svgs_shapes.length - 1]);
    }
    //And Sorry,Drawing Lines And Anything Related-To Polygons
    //Including Triangles And Polygons Are Not Supported In DOM
    if(renderer == SVG) {
      svg_shapes.push(document.createElementNS('http://www.w3.org/2000/svg', 'line'));
      svg_shapes[svg_shapes.length - 1].setAttribute('x1', v.pos1[0].toString());
      svg_shapes[svg_shapes.length - 1].setAttribute('y1', v.pos1[1].toString());
      svg_shapes[svg_shapes.length - 1].setAttribute('x2', v.pos2[0].toString());
      svg_shapes[svg_shapes.length - 1].setAttribute('y2', v.pos2[1].toString());
      svg_shapes[svg_shapes.length - 1].setAttribute('stroke', v.stroke);
      svg_shapes[svg_shapes.length - 1].style.strokeWidth = v.line_width;
      svg_shapes[svg_shapes.length - 1].style.transform = 'rotate(' + v.angle + 'deg)';
      svg_shapes[svg_shapes.length - 1].style.opacity = v.a;
      cakecanvas.appendChild(svg_shapes[svg_shapes.length - 1]);
    }
  }, //And When Drawing Shapes In SVG Or DOM We Get The Last Array Element Which Is The Shape We Pushed To Draw
  rect(v) {
    if(renderer == CANVAS || renderer == WEBGL) {
      cakepen.globalAlpha = v.a;
      cakepen.fillStyle = v.fill;
      cakepen.strokeStyle = v.stroke;
      cakepen.rotate(v.angle / 50);
      if(v.r == undefined || v.r == null || v.r == 0) {
        cakepen.fillRect(v.x, v.y, v.w, v.h);
        cakepen.strokeRect(v.x, v.y, v.w, v.h);
      }
      if(v.r > 0) {
        cakepen.beginPath();
        cakepen.moveTo(v.x + v.r, v.y);
        cakepen.lineTo(v.x + v.w - v.r, v.y);
        cakepen.quadraticCurveTo(v.x + v.w, v.y, v.x + v.w, v.y + v.r);
        cakepen.lineTo(v.x + v.w, v.y + v.h - v.r);
        cakepen.quadraticCurveTo(v.x + v.w, v.y + v.h, v.x + v.w - v.r, v.y + v.h);
        cakepen.lineTo(v.x + v.r, v.y + v.h);
        cakepen.quadraticCurveTo(v.x, v.y + v.h, v.x, v.y + v.h - v.r);
        cakepen.lineTo(v.x, v.y + v.r);
        cakepen.quadraticCurveTo(v.x, v.y, v.x + v.r, v.y);
        cakepen.closePath();
        cakepen.fill();
        cakepen.stroke();
      }
      cakepen.rotate(-v.angle);
      cakepen.globalAlpha = 1;
    }

    if(renderer == DOM) {
      dom_shapes.push(document.createElement('div'));
      dom_shapes[dom_shapes.length - 1].style.backgroundColor = v.fill;
      dom_shapes[dom_shapes.length - 1].style.border = '2px ' + v.stroke + ' solid';
      dom_shapes[dom_shapes.length - 1].style.position = 'absolute';
      dom_shapes[dom_shapes.length - 1].style.width = v.w + 'px';
      dom_shapes[dom_shapes.length - 1].style.height = v.h + 'px';
      dom_shapes[dom_shapes.length - 1].style.top = v.y + 'px';
      dom_shapes[dom_shapes.length - 1].style.left = v.x + 'px';
      dom_shapes[dom_shapes.length - 1].style.borderRadius = v.r + 'px';
      dom_shapes[dom_shapes.length - 1].style.opacity = v.a;
      dom_shapes[dom_shapes.length - 1].style.transform = 'rotate(' + v.angle + 'deg)';
      cakecanvas.appendChild(dom_shapes[dom_shapes.length - 1]);
    }

    if(renderer == SVG) {
      svg_shapes.push(document.createElementNS('http://www.w3.org/2000/svg', 'rect'));
      svg_shapes[svg_shapes.length - 1].setAttribute('x', v.x);
      svg_shapes[svg_shapes.length - 1].setAttribute('y', v.y);
      svg_shapes[svg_shapes.length - 1].setAttribute('width', v.w);
      svg_shapes[svg_shapes.length - 1].setAttribute('height', v.h);
      svg_shapes[svg_shapes.length - 1].setAttribute('rx', v.r);
      svg_shapes[svg_shapes.length - 1].setAttribute('ry', v.r);
      svg_shapes[svg_shapes.length - 1].setAttribute('fill', v.fill);
      svg_shapes[svg_shapes.length - 1].setAttribute('stroke', v.stroke);
      svg_shapes[svg_shapes.length - 1].style.transform = 'rotate(' + v.angle + 'deg)';
      svg_shapes[svg_shapes.length - 1].style.opacity = v.a;
      cakecanvas.appendChild(svg_shapes[svg_shapes.length - 1]);
    }
  },
  square(v) {
    if(renderer == CANVAS || renderer == WEBGL) {
      cakepen.globalAlpha = v.a;
      cakepen.fillStyle = v.fill;
      cakepen.strokeStyle = v.stroke;
      cakepen.rotate(v.angle / 50);
      cakepen.fillRect(v.x, v.y, v.size, v.size);
      cakepen.rotate(-v.angle);
      cakepen.globalAlpha = 1;
    }

    if(renderer == DOM) {
      dom_shapes.push(document.createElement('div'));
      dom_shapes[dom_shapes.length - 1].style.backgroundColor = v.fill;
      dom_shapes[dom_shapes.length - 1].style.border = '2px ' + v.stroke + ' solid';
      dom_shapes[dom_shapes.length - 1].style.position = 'absolute';
      dom_shapes[dom_shapes.length - 1].style.width = v.size / 2 + 'px';
      dom_shapes[dom_shapes.length - 1].style.height = v.size / 2 + 'px';
      dom_shapes[dom_shapes.length - 1].style.top = v.y - v.size * 2 + 'px';
      dom_shapes[dom_shapes.length - 1].style.left = v.x - v.size * 2 + 'px';
      dom_shapes[dom_shapes.length - 1].style.transform = 'rotate(' + v.angle + 'deg)';
      dom_shapes[dom_shapes.length - 1].style.opacity = v.a;
      cakecanvas.appendChild(dom_shapes[dom_shapes.length - 1]);
    }

    if(renderer == SVG) {
      svg_shapes.push(document.createElementNS('http://www.w3.org/2000/svg', 'rect'));
      svg_shapes[svg_shapes.length - 1].setAttribute('x', v.x);
      svg_shapes[svg_shapes.length - 1].setAttribute('y', v.y);
      svg_shapes[svg_shapes.length - 1].setAttribute('width', v.size);
      svg_shapes[svg_shapes.length - 1].setAttribute('height', v.size);
      svg_shapes[svg_shapes.length - 1].setAttribute('fill', v.fill);
      svg_shapes[svg_shapes.length - 1].setAttribute('stroke', v.stroke);
      svg_shapes[svg_shapes.length - 1].style.transform = 'rotate(' + v.angle + 'deg)';
      svg_shapes[svg_shapes.length - 1].style.opacity = v.a;
      cakecanvas.appendChild(svg_shapes[svg_shapes.length - 1]);
    }
  },
  pixel(v) {
    if(renderer == CANVAS || renderer == WEBGL) {
      cakepen.globalAlpha = v.a;
      cakepen.fillStyle = v.color;
      cakepen.rotate(v.angle / 50);
      cakepen.fillRect(v.x, v.y, 1, 1);
      cakepen.rotate(-v.angle);
      cakepen.globalAlpha = 1;
    }

    if(renderer == DOM) {
      dom_shapes.push(document.createElement('div'));
      dom_shapes[dom_shapes.length - 1].style.backgroundColor = v.color;
      dom_shapes[dom_shapes.length - 1].style.position = 'absolute';
      dom_shapes[dom_shapes.length - 1].style.width = '1px';
      dom_shapes[dom_shapes.length - 1].style.height = '1px';
      dom_shapes[dom_shapes.length - 1].style.top = v.y + '1px';
      dom_shapes[dom_shapes.length - 1].style.left = v.x + '1px';
      dom_shapes[dom_shapes.length - 1].style.transform = 'rotate(' + v.angle + 'deg)';
      dom_shapes[dom_shapes.length - 1].style.opacity = v.a;
      cakecanvas.appendChild(dom_shapes[dom_shapes.length - 1]);
    }

    if(renderer == SVG) {
      svg_shapes.push(document.createElementNS('http://www.w3.org/2000/svg', 'rect'));
      svg_shapes[svg_shapes.length - 1].setAttribute('x', v.x);
      svg_shapes[svg_shapes.length - 1].setAttribute('y', v.y);
      svg_shapes[svg_shapes.length - 1].setAttribute('width', 1);
      svg_shapes[svg_shapes.length - 1].setAttribute('height', 1);
      svg_shapes[svg_shapes.length - 1].setAttribute('color', v.color);
      svg_shapes[svg_shapes.length - 1].style.transform = 'rotate(' + v.angle + 'deg)';
      svg_shapes[svg_shapes.length - 1].style.opacity = v.a;
      cakecanvas.appendChild(svg_shapes[svg_shapes.length - 1]);
    }
  },
  roundedrect(v) {
    if(renderer == CANVAS) {
      cakepen.globalAlpha = v.a;
      cakepen.fillStyle = v.fill;
      cakepen.strokeStyle = v.stroke;
      cakepen.rotate(v.angle / 50);
      cakepen.beginPath();
      cakepen.moveTo(v.x + v.r, v.y);
      cakepen.lineTo(v.x + v.w - v.r, v.y);
      cakepen.quadraticCurveTo(v.x + v.w, v.y, v.x + v.w, v.y + v.r);
      cakepen.lineTo(v.x + v.w, v.y + v.h - v.r);
      cakepen.quadraticCurveTo(v.x + v.w, v.y + v.h, v.x + v.w - v.r, v.y + v.h);
      cakepen.lineTo(v.x + v.r, v.y + v.h);
      cakepen.quadraticCurveTo(v.x, v.y + v.h, v.x, v.y + v.h - v.r);
      cakepen.lineTo(v.x, v.y + v.r);
      cakepen.quadraticCurveTo(v.x, v.y, v.x + v.r, v.y);
      cakepen.closePath();
      cakepen.fill();
      cakepen.stroke();
      cakepen.rotate(-v.angle);
      cakepen.globalAlpha = 1;
    }

    if(renderer == DOM) {
      dom_shapes.push(document.createElement('div'));
      dom_shapes[dom_shapes.length - 1].style.backgroundColor = v.fill;
      dom_shapes[dom_shapes.length - 1].style.border = '2px ' + v.stroke + ' solid';
      dom_shapes[dom_shapes.length - 1].style.position = 'absolute';
      dom_shapes[dom_shapes.length - 1].style.width = v.w / 2 + 'px';
      dom_shapes[dom_shapes.length - 1].style.height = v.h / 2 + 'px';
      dom_shapes[dom_shapes.length - 1].style.borderRadius = v.r + 'px';
      dom_shapes[dom_shapes.length - 1].style.top = v.y - v.h * 2 + 'px';
      dom_shapes[dom_shapes.length - 1].style.left = v.x - v.w * 2 + 'px';
      dom_shapes[dom_shapes.length - 1].style.transform = 'rotate(' + v.angle + 'deg)';
      dom_shapes[dom_shapes.length - 1].style.opacity = v.a;
      cakecanvas.appendChild(dom_shapes[dom_shapes.length - 1]);
    }

    if(renderer == SVG) {
      svg_shapes.push(document.createElementNS('http://www.w3.org/2000/svg', 'rect'));
      svg_shapes[svg_shapes.length - 1].setAttribute('x', v.x);
      svg_shapes[svg_shapes.length - 1].setAttribute('y', v.y);
      svg_shapes[svg_shapes.length - 1].setAttribute('width', v.w);
      svg_shapes[svg_shapes.length - 1].setAttribute('height', v.h);
      svg_shapes[svg_shapes.length - 1].setAttribute('rx', v.r);
      svg_shapes[svg_shapes.length - 1].setAttribute('ry', v.r);
      svg_shapes[svg_shapes.length - 1].setAttribute('fill', v.fill);
      svg_shapes[svg_shapes.length - 1].setAttribute('stroke', v.stroke);
      svg_shapes[svg_shapes.length - 1].style.transform = 'rotate(' + v.angle + 'deg)';
      svg_shapes[svg_shapes.length - 1].style.opacity = v.a;
      cakecanvas.appendChild(svg_shapes[svg_shapes.length - 1]);
    }

    if(renderer == WEBGL) {
      if(v.angle == undefined) v.angle = 0;
      cakepen.globalAlpha = v.a;
      cakepen.fillStyle = v.fill;
      cakepen.strokeStyle = v.fill;
      cakepen.rotate(v.angle / 50);
      let i, angle, x1, y1;
      for(i = 0; i < 360; i += 0.1) {
        angle = i;
        x1 = v.r * Math.cos((angle * Math.PI) / 180);
        y1 = v.r * Math.sin((angle * Math.PI) / 180);
        cakepen.fillRect(v.x + x1 + v.r, v.y + y1 + v.r, v.r * 1.5, v.r * 1.5);
        cakepen.strokeRect(v.x + x1 + v.r, v.y + y1 + v.r, v.r * 1.5, v.r * 1.5);
      }
      cakepen.rotate(-v.angle);
      cakepen.globalAlpha = 1;
    }
  },
  circle(v) {
    if(renderer == CANVAS) {
      cakepen.globalAlpha = v.a;
      cakepen.fillStyle = v.fill;
      cakepen.strokeStyle = v.stroke;
      cakepen.rotate(v.angle / 50);
      cakepen.beginPath();
      cakepen.arc(v.x, v.y, v.r, 90, 180 * Math.PI);
      cakepen.closePath();
      cakepen.fill();
      cakepen.stroke();
      cakepen.rotate(-v.angle);
      cakepen.globalAlpha = 1;
    }

    if(renderer == DOM) {
      dom_shapes.push(document.createElement('div'));
      dom_shapes[dom_shapes.length - 1].style.backgroundColor = v.fill;
      dom_shapes[dom_shapes.length - 1].style.border = '2px ' + v.stroke + ' solid';
      dom_shapes[dom_shapes.length - 1].style.position = 'absolute';
      dom_shapes[dom_shapes.length - 1].style.width = v.r * 1.85 + 'px';
      dom_shapes[dom_shapes.length - 1].style.height = v.r * 1.85 + 'px';
      dom_shapes[dom_shapes.length - 1].style.borderRadius = '50%';
      dom_shapes[dom_shapes.length - 1].style.top = v.y - v.r + 'px';
      dom_shapes[dom_shapes.length - 1].style.left = v.x - v.r + 'px';
      dom_shapes[dom_shapes.length - 1].style.transform = 'rotate(' + v.angle + 'deg)';
      dom_shapes[dom_shapes.length - 1].style.opacity = v.a;
      cakecanvas.appendChild(dom_shapes[dom_shapes.length - 1]);
    }

    if(renderer == SVG) {
      svg_shapes.push(document.createElementNS('http://www.w3.org/2000/svg', 'circle'));
      svg_shapes[svg_shapes.length - 1].setAttribute('cx', v.x);
      svg_shapes[svg_shapes.length - 1].setAttribute('cy', v.y);
      svg_shapes[svg_shapes.length - 1].setAttribute('r', v.r);
      svg_shapes[svg_shapes.length - 1].setAttribute('fill', v.fill);
      svg_shapes[svg_shapes.length - 1].setAttribute('stroke', v.stroke);
      svg_shapes[svg_shapes.length - 1].style.transform = 'rotate(' + v.angle + 'deg)';
      svg_shapes[svg_shapes.length - 1].style.opacity = v.a;
      cakecanvas.appendChild(svg_shapes[svg_shapes.length - 1]);
    }

    if(renderer == WEBGL) {
      cakepen.globalAlpha = v.a;
      cakepen.fillStyle = v.stroke;
      cakepen.strokeStyle = v.fill;
      if(v.angle == undefined) v.angle = 0;
      cakepen.rotate(v.angle / 50);
      var i, angle, x1, y1;
      cakepen.beginPath();
      for(i = 0; i < 360; i += 0.1) {
        angle = i;
        x1 = v.r * Math.cos((angle * Math.PI) / 180);
        y1 = v.r * Math.sin((angle * Math.PI) / 180);
        cakepen.moveTo(v.x, v.y);
        cakepen.lineTo(x1 + v.x, y1 + v.y);
        cakepen.lineTo(x1 + v.x, y1 + v.y);
      }
      cakepen.closePath();
      cakepen.fill();
      cakepen.stroke();
      var i, angle, x1, y1;
      for(i = 0; i < 360; i += 0.1) {
        angle = i;
        x1 = v.r * Math.cos((angle * Math.PI) / 180);
        y1 = v.r * Math.sin((angle * Math.PI) / 180);
        cakepen.fillRect(v.x + x1, v.y + y1, 2, 2);
      }
      cakepen.rotate(-v.angle);
      cakepen.globalAlpha = 1;
    }
  },
  img(v) {
    if(renderer == CANVAS || renderer == WEBGL) {
      cakepen.globalAlpha = v.a;
      cakepen.rotate(v.angle / 50);
      images.push(new Image(v.w, v.h));
      images[images.length - 1].src = v.img;
      images[images.length - 1].onload = () => cakepen.drawImage(images[images.length - 1], v.x, v.y, v.w, v.h);
      cakepen.rotate(-v.angle);
      cakepen.globalAlpha = 1;
    }
    if(renderer == DOM) {
      dom_shapes.push(new Image());
      dom_shapes[dom_shapes.length - 1].src = v.img;
      dom_shapes[dom_shapes.length - 1].style.position = 'absolute';
      dom_shapes[dom_shapes.length - 1].style.width = v.w + 'px';
      dom_shapes[dom_shapes.length - 1].style.height = v.h + 'px';
      dom_shapes[dom_shapes.length - 1].style.borderRadius = v.r + 'px';
      dom_shapes[dom_shapes.length - 1].style.top = v.y - v.h / 60 + 'px';
      dom_shapes[dom_shapes.length - 1].style.left = v.x - v.w / 60 + 'px';
      dom_shapes[dom_shapes.length - 1].style.transform = 'rotate(' + v.angle + 'deg)';
      dom_shapes[dom_shapes.length - 1].style.opacity = v.a;
      cakecanvas.appendChild(dom_shapes[dom_shapes.length - 1]);
    }
    if(renderer == SVG) {
      svg_shapes.push(document.createElementNS('http://www.w3.org/2000/svg', 'image'));
      svg_shapes[svg_shapes.length - 1].setAttribute('href', v.img);
      svg_shapes[svg_shapes.length - 1].setAttribute('x', v.x);
      svg_shapes[svg_shapes.length - 1].setAttribute('y', v.y);
      svg_shapes[svg_shapes.length - 1].setAttribute('rx', v.r);
      svg_shapes[svg_shapes.length - 1].setAttribute('ry', v.r);
      svg_shapes[svg_shapes.length - 1].setAttribute('width', v.w);
      svg_shapes[svg_shapes.length - 1].setAttribute('height', v.h);
      svg_shapes[svg_shapes.length - 1].style.transform = 'rotate(' + v.angle + 'deg)';
      svg_shapes[svg_shapes.length - 1].style.opacity = v.a;
      cakecanvas.appendChild(svg_shapes[svg_shapes.length - 1]);
    }
  }, //NOTES: If Parameter To Use With DOM Or SVG,Set It To 0 Or "none" In Case Of That
  //NOTES: v.size Parameter Only For SVG And DOM,Font Size Setted In CANVAS Mode With font
  text(v) {
    if(renderer == CANVAS || renderer == WEBGL) {
      cakepen.globalAlpha = v.a;
      cakepen.font = v.size + 'px ' + v.font;
      cakepen.fillStyle = v.fill;
      cakepen.strokeStyle = v.stroke;
      cakepen.rotate(v.angle / 50);
      cakepen.fillText(v.txt, v.x, v.y);
      cakepen.strokeText(v.txt, v.x, v.y);
      cakepen.rotate(-v.angle);
      cakepen.globalAlpha = 1;
    }

    if(renderer == DOM) {
      dom_shapes.push(document.createElement('strong'));
      dom_shapes[dom_shapes.length - 1].innerHTML = v.txt;
      dom_shapes[dom_shapes.length - 1].style.fontFamily = v.font;
      dom_shapes[dom_shapes.length - 1].style.fontSize = v.size + 'px';
      dom_shapes[dom_shapes.length - 1].style.position = 'absolute';
      dom_shapes[dom_shapes.length - 1].style.color = v.fill;
      dom_shapes[dom_shapes.length - 1].style.top = v.y - v.size / 2 + 'px';
      dom_shapes[dom_shapes.length - 1].style.left = v.x - v.size / 2 + 'px';
      dom_shapes[dom_shapes.length - 1].style.transform = 'rotate(' + v.angle + 'deg)';
      dom_shapes[dom_shapes.length - 1].style.opacity = v.a;
      cakecanvas.appendChild(dom_shapes[dom_shapes.length - 1]);
    }

    if(renderer == SVG) {
      svg_shapes.push(document.createElementNS('http://www.w3.org/2000/svg', 'text'));
      svg_shapes[svg_shapes.length - 1].setAttribute('x', v.x);
      svg_shapes[svg_shapes.length - 1].setAttribute('y', v.y);
      svg_shapes[svg_shapes.length - 1].innerHTML = v.txt;
      svg_shapes[svg_shapes.length - 1].setAttribute('fill', v.fill);
      svg_shapes[svg_shapes.length - 1].setAttribute('stroke', v.stroke);
      svg_shapes[svg_shapes.length - 1].style.fontFamily = v.font;
      svg_shapes[svg_shapes.length - 1].style.transform = 'rotate(' + v.angle + 'deg)';
      svg_shapes[svg_shapes.length - 1].style.opacity = v.a;
      svg_shapes[svg_shapes.length - 1].style.fontSize = v.size + 'px';
      cakecanvas.appendChild(svg_shapes[svg_shapes.length - 1]);
    }
  },
  triangle(v) {
    if(renderer == CANVAS || renderer == WEBGL) {
      cakepen.globalAlpha = v.a;
      cakepen.strokeStyle = v.stroke;
      cakepen.fillStyle = v.fill;
      cakepen.lineWidth = v.line_width;
      cakepen.rotate(v.angle / 50);
      cakepen.beginPath();
      cakepen.moveTo(v.pos1[0], v.pos1[1]);
      cakepen.lineTo(v.pos2[0], v.pos2[1]);
      cakepen.lineTo(v.pos3[0], v.pos3[1]);
      cakepen.lineTo(v.pos1[0], v.pos1[1]);
      cakepen.closePath();
      cakepen.fill();
      cakepen.stroke();
      cakepen.rotate(-v.angle);
      cakepen.globalAlpha = 1;
    }
    if(renderer == DOM) {
      if(v.pos1[0] > biggest_x) biggest_x = v.pos1[0];
      if(v.pos1[1] > biggest_y) biggest_y = v.pos1[1];
      if(v.pos2[0] > biggest_x) biggest_x = v.pos2[0];
      if(v.pos2[1] > biggest_y) biggest_y = v.pos2[1];
      if(v.pos3[0] > biggest_x) biggest_x = v.pos3[0];
      if(v.pos3[1] > biggest_y) biggest_y = v.pos3[1];
      dom_svgs_shapes.push(document.createElementNS('http://www.w3.org/2000/svg', 'polygon'));
      dom_svgs_shapes[dom_svgs_shapes.length - 1].setAttribute(
        'points',
        (v.pos1[0] + ',' + v.pos1[1] + ' ' + v.pos2[0] + ',' + v.pos2[1] + ' ' + v.pos3[0] + ',' + v.pos3[1] + ' ' + v.pos1[0] + ',' + v.pos1[1]).toString()
      );
      dom_svgs_shapes[dom_svgs_shapes.length - 1].setAttribute('fill', v.fill);
      dom_svgs_shapes[dom_svgs_shapes.length - 1].setAttribute('stroke', v.stroke);
      dom_svgs_shapes[dom_svgs_shapes.length - 1].style.strokeWidth = v.line_width;
      dom_svgs_shapes[dom_svgs_shapes.length - 1].style.transform = 'rotate(' + v.angle + 'deg)';
      dom_svgs_shapes[dom_svgs_shapes.length - 1].style.opacity = v.a;
      svg_board.appendChild(dom_svgs_shapes[dom_svgs_shapes.length - 1]);
    }
    if(renderer == SVG) {
      svg_shapes.push(document.createElementNS('http://www.w3.org/2000/svg', 'polygon'));
      svg_shapes[svg_shapes.length - 1].setAttribute(
        'points',
        (v.pos1[0] + ',' + v.pos1[1] + ' ' + v.pos2[0] + ',' + v.pos2[1] + ' ' + v.pos3[0] + ',' + v.pos3[1] + ' ' + v.pos1[0] + ',' + v.pos1[1]).toString()
      );
      svg_shapes[svg_shapes.length - 1].setAttribute('fill', v.fill);
      svg_shapes[svg_shapes.length - 1].setAttribute('stroke', v.stroke);
      svg_shapes[svg_shapes.length - 1].style.strokeWidth = v.line_width;
      svg_shapes[svg_shapes.length - 1].style.transform = 'rotate(' + v.angle + 'deg)';
      svg_shapes[svg_shapes.length - 1].style.opacity = v.a;
      cakecanvas.appendChild(svg_shapes[svg_shapes.length - 1]);
    }
  },
  polygon(v) {
    if(renderer == CANVAS || renderer == WEBGL) {
      cakepen.globalAlpha = v.a;
      cakepen.fillStyle = v.fill;
      cakepen.strokeStyle = v.stroke;
      cakepen.rotate(v.angle / 50);
      cakepen.beginPath();
      cakepen.moveTo(v.points[0][0], v.points[0][1]);
      for(var i = 0; i < v.points.length; i++) cakepen.lineTo(v.points[i][0], v.points[i][1]);
      cakepen.closePath();
      cakepen.fill();
      cakepen.stroke();
      cakepen.rotate(-v.angle);
      cakepen.globalAlpha = 1;
    }
    if(renderer == DOM) {
      domvg_polygon_points = '';
      domvg_polygon_points += v.points[0][0] + ',' + v.points[0][1] + ' ';
      for(var i = 0; i < v.points.length; i++) {
        if(v.points[i][0] > biggest_x) biggest_x = v.points[i][0];
        if(v.points[i][1] > biggest_y) biggest_y = v.points[i][1];
        domvg_polygon_points += v.points[i][0] + ',' + v.points[i][1] + ' ';
      }
      dom_svgs_shapes.push(document.createElementNS('http://www.w3.org/2000/svg', 'polygon'));
      dom_svgs_shapes[dom_svgs_shapes.length - 1].setAttribute('points', domvg_polygon_points.toString());
      dom_svgs_shapes[dom_svgs_shapes.length - 1].setAttribute('fill', v.fill);
      dom_svgs_shapes[dom_svgs_shapes.length - 1].setAttribute('stroke', v.stroke);
      dom_svgs_shapes[dom_svgs_shapes.length - 1].style.transform = 'rotate(' + v.angle + 'deg)';
      dom_svgs_shapes[dom_svgs_shapes.length - 1].style.opacity = v.a;
      svg_board.appendChild(dom_svgs_shapes[dom_svgs_shapes.length - 1]);
    }
    if(renderer == SVG) {
      svg_shapes.push(document.createElementNS('http://www.w3.org/2000/svg', 'polygon'));
      domvg_polygon_points = '';
      domvg_polygon_points += v.points[0][0] + ',' + v.points[0][1] + ' ';
      for(var i = 0; i < v.points.length; i++) domvg_polygon_points += v.points[i][0] + ',' + v.points[i][1] + ' ';
      svg_shapes[svg_shapes.length - 1].setAttribute('points', domvg_polygon_points.toString());
      svg_shapes[svg_shapes.length - 1].setAttribute('fill', v.fill);
      svg_shapes[svg_shapes.length - 1].setAttribute('stroke', v.stroke);
      svg_shapes[svg_shapes.length - 1].style.transform = 'rotate(' + v.angle + 'deg)';
      svg_shapes[svg_shapes.length - 1].style.opacity = v.a;
      cakecanvas.appendChild(svg_shapes[svg_shapes.length - 1]);
    }
  },
  clear() {
    if(renderer == CANVAS || renderer == WEBGL) {
      cakepen.fillStyle = 'transparent';
      cakepen.fillRect(0, 0, cakepen.canvas.width, cakepen.canvas.height);
      cakepen.clearRect(0, 0, cakepen.canvas.width, cakepen.canvas.height);
    }
    //The Technology Used Here Is Somehow Weird
    //It Removes Every Shape Drawn From document
    //Then Remove All Elements From Array Using [].slice(0,array_length)
    if(renderer == DOM) {
      for(doms = 0; doms < dom_shapes.length; doms++) {
        dom_shapes.slice(0, dom_shapes.length);
        cakecanvas.removeChild(dom_shapes[doms]);
      }
      for(svid = 0; svid < dom_svgs_shapes.length; svid++) {
        dom_svgs_shapes.slice(0, dom_svgs_shapes.length);
        svg_board.removeChild(dom_svgs_shapes[svid]);
      }
    }
    if(renderer == SVG) {
      for(svgos = 0; svgos < dom_shapes.length; svgos++) {
        svg_shapes.slice(0, svg_shapes.length);
        cakecanvas.removeChild(svg_shapes[svgos]);
      }
    }
  },
  bgcolor(c) {
    if(renderer == CANVAS || renderer == WEBGL || renderer == SVG || renderer == DOM) cakecanvas.style.backgroundColor = c;
    if(renderer == DOM) svg_board.style.backgroundColor = c;
  },
  bgimg(v) {
    cakecanvas.style.backgroundImage = 'url(' + v.src + ')';
    cakecanvas.style.opacity = v.a;
  },
  animate(v) {
    if(renderer == CANVAS || renderer == WEBGL || renderer == DOM) window.requestAnimationFrame(v.frame);
    if(renderer == SVG) {
      svg_anims.push(document.createElementNS('http://www.w3.org/2000/svg', 'animate'));
      svg_anims[svg_anims.length - 1].setAttribute('attributeType', 'XML');
      svg_anims[svg_anims.length - 1].setAttribute('attributeName', v.attr);
      svg_anims[svg_anims.length - 1].setAttribute('dur', v.dur + 's');
      svg_anims[svg_anims.length - 1].setAttribute('to', v.to);
      svg_anims[svg_anims.length - 1].setAttribute('from', v.from);
      svg_anims[svg_anims.length - 1].setAttribute('repeatCount', v.loop_num);
      svg_anims[svg_anims.length - 1].setAttribute('repeatDur', v.loop_dur);
      svg_anims[svg_anims.length - 1].anim_id = 'animation-' + svg_anims[svg_anims.length - 1];
      svg_anims[svg_anims.length - 1].setAttribute('id', svg_anims[svg_anims.length - 1].anim_id);
      let svg_obj = svg_shapes[v.index];
      let prev_anim = document.getElementById(svg_anims[svg_anims.length - 1].anim_id);
      if(prev_anim != null) svg_obj.removeChild(prev_anim);
      svg_obj.appendChild(svg_anims[svg_anims.length - 1]);
    }
  },
  interval(f, t) {
    return setInterval(f, t);
  },
  timer(f, t) {
    return setTimeout(f, t);
  },
  update(f, t) {
    return window.update(f, t);
  },
  pause(v) {
    if(v.interval == undefined && (renderer == DOM || renderer == CANVAS || renderer == WEBGL)) window.cancelAnimationFrame(v.frame);
    if(!(v.interval == undefined) && (renderer == DOM || renderer == CANVAS || renderer == WEBGL)) window.clearInterval(v.interval);
  }
};
let rgb = function(v) {
  return 'rgb(' + v.r + ',' + v.g + ',' + v.b + ')';
};
let rgba = function(v) {
  return 'rgba(' + v.r + ',' + v.g + ',' + v.b + ',' + v.a + ')';
};
let hsl = function(v) {
  return 'hsl(' + v.h + ',' + v.s + ',' + v.l + ')';
};
let hsla = function(v) {
  return 'hsla(' + v.h + ',' + v.s + ',' + v.l + ',' + v.a + ')';
};
window.addEventListener('keypress', e => {
  if(e.key == 'f' && !window.fullscreen) document.documentElement.requestFullscreen();
  if(e.key == 'f' && window.fullscreen) document.documentElement.exitFullscreen();
});

/* ----------------------- end of './lib/crosskit.js' ----------------------- */

/* ------------------------ start of './lib/util.js' ------------------------ */

// ==UserScript==

// @name         util.js
// @namespace    util
// @version      0.2
// @description  geom.js, align.js, bbox.js, util.js, graph.js, intersection.js, point.js, line.js, lineList.js, element.js, node.js, trbl.js, rect.js, size.js, iterator.js, pointList.js, matrix.js, circle.js, polygonFinder.js, polygon.js, sweepLine.js, transformation.js, vector.js, simplify.js
// @author       You
// @match        *://*/*
// @exclude      *://127.0.0.1*/*
// @updateURL    http://127.0.0.1:9000/lib/util.js
// @grant        none
// @run-at       document-end
// ==/UserScript==

/* jshint esversion: 6 */
/* jshint ignore:start */

/**
 * Class for utility.
 *
 * @class      Util (name)
 */

const inspectSymbol = Symbol.for('nodejs.util.inspect.custom');

function Util(g) {
  const globalObject = g || Util.getGlobalObject();
  globalObject.Util = Util;
  return Util;
}

Util.toString = undefined;
//export const Util = {};

const lineSplit = new RegExp('\\n', 'g');

Util.inspectSymbol = inspectSymbol;

Util.formatAnnotatedObject = function(subject, o) {
  const { indent = '  ', spacing = ' ', separator = ',', newline = '\n', maxlen = 30, depth = 1, level = 0 } = o;
  const i = indent.repeat(o.level || 0);
  let nl = newline != '' ? newline + i : spacing;
  const opts = {
    ...o,
    newline: depth >= 0 ? newline : '',
    depth: depth - 1,
    level: level + 1
  };
  if(subject && subject.toSource !== undefined) return subject.toSource();
  if(subject instanceof Date) return 'new Date(' + new Date().toISOString() + ')';
  if(typeof subject == 'string') return "'" + subject + "'";
  if(typeof subject == 'number') return subject;
  if(subject != null && subject.y2 !== undefined) return `rect[${spacing}${subject.x}${separator}${subject.y} | ${subject.x2}${separator}${subject.y2} (${subject.w}x${subject.h}) ]`;
  if(Util.isObject(subject) && 'map' in subject && typeof subject.map == 'function') return `[${nl}${subject.map(i => Util.formatAnnotatedObject(i, opts)).join(separator + nl)}]`;
  if(typeof subject === 'string' || subject instanceof String) return `'${subject}'`;
  let longest = '';
  let r = [];
  for(let k in subject) {
    const v = subject[k];
    if(k.length > longest.length) longest = k;
    let s = '';
    if(typeof v === 'symbol') {
      s = 'Symbol';
    } else if(typeof v === 'string' || v instanceof String) {
      s = `'${v}'`;
    } else if(typeof v === 'function') {
      s = (v + '').replace(lineSplit, '\n' + i);
      s = (Util.fnName(s) || 'function') + '()';
    } else if(typeof v === 'number' || typeof v === 'boolean') {
      s = `${v}`;
    } else if(v === null) {
      s = 'null';
    } else if(v && v.length !== undefined) {
      try {
        s = depth <= 0 ? `Array(${v.length})` : `[ ${v.map(item => Util.formatAnnotatedObject(item, opts)).join(', ')} ]`;
      } catch(err) {
        s = `[${v}]`;
      }
    } else if(v && v.toSource !== undefined) {
      s = v.toSource();
    } else if(opts.depth >= 0) {
      s = s.length > maxlen ? `[Object ${Util.objName(v)}]` : Util.formatAnnotatedObject(v, opts);
    } else {
      let c = Util.className(v);
      let t = Util.ucfirst(typeof v);

      s = `[${t}${c !== t ? ' ' : ''}${c !== t ? c : ''}]`;
    }
    if(s == '') s = typeof v;
    r.push([k, s]);
  }
  let padding = x => indent + (opts.newline != '' ? Util.pad(x, longest.length, spacing) : spacing);
  let j = separator + spacing;
  if(r.length > 6) {
    nl = opts.newline + i;
    j = separator + (opts.newline != '' ? nl : spacing);
  }
  let ret = '{' + opts.newline + r.map(arr => padding(arr[0]) + arr[0] + ':' + spacing + arr[1]).join(j) + opts.newline + i + '}';
  return ret;
};
Util.curry = (fn, arity) => {
  if(arity == null) arity = fn.length;
  let ret = function curried(...args) {
    let thisObj = this;
    if(args.length >= arity) return fn.apply(this, args);

    let n = arity - args.length;
    let a = Array.from({ length: n }, (v, i) => String.fromCharCode(65 + i));
    let Curried = function(...a) {
      return curried.apply(thisObj, a);
    }; //;
    return [
      function() {
        return Curried(...args);
      },
      function(a) {
        return Curried(...args, a);
      },
      function(a, b) {
        return Curried(...args, a, b);
      },
      function(a, b, c) {
        return r(...args, a, b, c);
      },
      function(a, b, c, d) {
        return Curried(...args, a, b, c, d);
      }
    ][n];
    return new Function(...a, `const { curried,thisObj,args} = this; return curried.apply(thisObj, args.concat([${a.join(',')}]))`).bind({ args, thisObj, curried });
  };
  Object.defineProperties(ret, {
    length: {
      value: arity,
      configurable: true,
      writable: true,
      enumerable: false
    },
    orig: {
      get() {
        return fn;
      }
    }
  });
  return ret;
};
Util.arityN = (fn, n) => {
  const arityFn = [
    function(fn) {
      return function() {
        return fn();
      };
    },
    function(fn) {
      return function(a) {
        return fn(a);
      };
    },
    function(fn) {
      return function(a, b) {
        return fn(a, b);
      };
    },
    function(fn) {
      return function(a, b, c) {
        return fn(a, b, c);
      };
    },
    function(fn) {
      return function(a, b, c, d) {
        return fn(a, b, c, d);
      };
    },
    function(fn) {
      return function(a, b, c, d, e) {
        return fn(a, b, c, d, e);
        H;
      };
    }
  ];
  if(n && n <= 5) return arityFn[n](fn);
  return fn;
};

Util.getter = target => {
  let self;
  if(typeof target.get == 'function') self = target.get;
  else
    self = function(key) {
      if(!target) {
        if(this !== self && this) target = this;
        self.target = target;
      }
      let obj = target;
      if(!self.fn) {
        if(typeof obj == 'object' && obj !== null) {
          if(typeof obj.get == 'function') self.fn = key => obj.get(key);
        }
        if(!self.fn) self.fn = key => obj[key];
      }
      return self.fn(key);
    };
  if(target !== undefined) self.target = target;
  return self;
};
Util.setter = target => {
  if(typeof target.set == 'function') return target.set;
  let set;
  set = function(key, value) {
    if(!target) {
      if(this !== set && this) target = this;
      set.target = target;
    }
    let obj = target;
    if(!set.fn) {
      if(typeof obj == 'object' && obj !== null) {
        if(typeof obj.set == 'function') set.fn = (key, value) => obj.set(key, value);
      }
    }
    if(!set.fn) set.fn = (key, value) => ((obj[key] = value), obj);
    return set.fn(key, value);
  };
  if(target !== undefined) set.target = target;
  return set;
};
Util.remover = target => (typeof target == 'object' && target !== null ? (typeof target.delete == 'function' ? key => target.delete(key) : key => delete target[key]) : null);
Util.hasFn = target => (typeof target == 'object' && target !== null ? (typeof target.has == 'function' ? key => target.has(key) : key => key in target) : null);
Util.adder = target => {
  let self;

  if(target instanceof Set) return arg => target.add(arg);
  if(target instanceof Array) return arg => (target.push(arg), target);

  self = function(obj, arg = 1) {
    if(!target) if (obj) target = obj;

    if(!self.fn) ChooseFn(arg, obj);
    //console.debug('adder', self.fn + '');

    // if(!self.fn) console.log('adder', { target, thisObj: this, fn: self.fn + '', arg });
    return self.fn(obj, arg);
  };
  if(target && !self.fn) {
    ChooseFn(',', target);
    target = null;
  }

  return self;

  function ChooseFn(a, o) {
    if(!self.fn) {
      if(typeof target == 'object' && target !== null) {
        if(typeof target.add == 'function') self.fn = (obj, arg) => (obj.add(arg), undefined);
        else if(typeof target.push == 'function') self.fn = (obj, arg) => (obj.push(arg), undefined);
      }
    }
    let isNum = Util.isNumeric(a);
    //console.debug('ChooseFn', { a, o, f: self.fn });
    if(!self.fn) {
      if(typeof o == 'string') self.fn = (obj, arg) => (obj == '' ? '' : obj + ', ') + arg;
      else if(typeof o == 'number') self.fn = (num, arg) => (typeof num == 'number' ? num : 0) + +arg;
      else if(a) self.fn = (obj, arg) => ((obj || (isNum || typeof arg == 'number' ? 0 : '')) + isNum ? +arg : ',' + arg);
    }
  }
};
Util.updater = (target, get, set, fn) => {
  let value;

  /* prettier-ignore */ get = get || Util.getter(target);
  /* prettier-ignore */ set = set || Util.setter(target);

  return (k, f, i) => doUpdate(k, f || fn, i);
  function doUpdate(key, func, i) {
    value = get.call(target, key);
    let tmp = func(value, i, key);

    if(tmp !== undefined && tmp != value) {
      set.call(target, key, tmp);

      value = get.call(target, key);
    }
    return value;
  }
};
Util.getOrCreate = (target, create = () => ({}), set) => {
  const get = Util.getter(target),
    has = Util.hasFn(target);
  /* prettier-ignore */ set = set || Util.setter(target);
  let value;
  return key => (value = has.call(target, key) ? get.call(target, key) : ((value = create(key, target)), set.call(target, key, value), value));
};
Util.accumulate = (entries, dest = new Map()) => {
  let get = Util.getOrCreate(dest, () => []);
  for(let [key, value] of entries) Util.adder(get(key))(value);
  return dest;
};
Util.memoize = (fn, storage = new Map()) => {
  let self;
  const getter = typeof storage.get == 'function' ? storage.get : typeof storage == 'function' ? storage : Util.getter(storage);
  const setter = typeof storage.set == 'function' ? storage.set : typeof storage == 'function' ? storage : Util.setter(storage);
  self = function(...args) {
    // let n = args[0]; // just taking one argument here
    let cached;
    let key = args[0];

    if((cached = getter.call(storage, key))) {
      //console.log('Fetching from cache');
      return cached;
    }
    let result = fn.call(this, ...args);
    setter.call(storage, key, result);
    return result;
  };
  self.cache = storage;
  return Object.freeze(self);
};
Util.once = (fn, thisArg, memoFn) => {
  let ran = false;
  let ret;

  return function(...args) {
    if(!ran) {
      ran = true;
      ret = fn.call(thisArg || this, ...args);
    } else if(typeof memoFn == 'function') {
      ret = memoFn(ret);
    }
    return ret;
  };
};
Util.delay = (func, wait, thisObj) => {
  if(typeof func != 'function') throw new TypeError(FUNC_ERROR_TEXT);
  return function(...args) {
    setTimeout(function () {
      func.apply(thisObj || this, args);
    }, wait);
  };
};
Util.throttle = (f, t, thisObj) => {
  let lastCall;
  return function(...args) {
    let previousCall = lastCall;
    lastCall = Date.now();
    if(
      previousCall === undefined || // function is being called for the first time
      lastCall - previousCall > t
    )
      // throttle time has elapsed
      f.apply(thisObj || this, args);
  };
};
Util.debounce = (func, wait, options = {}) => {
  if(!Number.isFinite(wait)) throw new TypeError('Expected `wait` to be a finite number');
  let id, args, ctx, timestamp, r;
  const { leading, thisObj } = options;
  if(null == wait) wait = 100;
  function later() {
    let last = Date.now() - timestamp;
    if(last < wait && last >= 0) {
      id = setTimeout(later, wait - last);
    } else {
      id = null;
      if(!leading) {
        r = func.apply(ctx, args);
        ctx = args = null;
      }
    }
  }
  function debounced(...a) {
    ctx = thisObj || this;
    args = a;
    timestamp = Date.now();
    let callNow = leading && !id;
    if(!id) id = setTimeout(later, wait);
    if(callNow) {
      r = func.apply(ctx, args);
      ctx = args = null;
    }
    return r;
  }
  debounced.clear = function() {
    if(id) {
      clearTimeout(id);
      id = null;
    }
  };
  debounced.flush = function() {
    if(id) {
      r = func.apply(ctx, args);
      ctx = args = null;
      clearTimeout(id);
      id = null;
    }
  };
  return debounced;
};

Util.debounceAsync = (fn, wait, options = {}) => {
  if(!Number.isFinite(wait)) throw new TypeError('Expected `wait` to be a finite number');
  let r,
    id,
    resolveList = [];
  const { thisObj, leading } = options;
  return function(...a) {
    return new Promise(resolve => {
      const callNow = leading && !id;
      clearTimeout(id);
      id = setTimeout(() => {
        id = null;
        const result = leading ? r : fn.apply(thisObj || this, a);
        for(resolve of resolveList) resolve(result);
        resolveList = [];
      }, wait);
      if(callNow) {
        r = fn.apply(thisObj || this, a);
        resolve(r);
      } else {
        resolveList.push(resolve);
      }
    });
  };
};

/*Util.debounce = (f, t, thisObj) => {
  let lastCall, lastCallTimer;
  return function(...args) {
    let previousCall = lastCall;
    lastCall = Date.now();
    if(previousCall && lastCall - previousCall <= t) clearTimeout(lastCallTimer);

    return new Promise((resolve, reject) => {
      lastCallTimer = setTimeout(() => resolve(f.apply(thisObj || this, args)), t);
    });
  };
};*/
Util.getGlobalObject = Util.memoize(arg => {
  const retfn = typeof arg == 'function' ? arg : typeof arg == 'string' ? g => g[arg] : g => g;

  return Util.tryCatch(
    () => globalThis,
    retfn,
    err =>
      Util.tryCatch(
        () => globalThis,
        retfn,
        err =>
          Util.tryCatch(
            () => window,
            retfn,
            err => console.log('Util.getGlobalObject:', err)
          )
      )
  );
});

Util.isDebug = Util.memoize(() => {
  if(process !== undefined && process.env.NODE_ENV === 'production') return false;
  return true;
});

/*Util.log = Util.curry(function(n, base) {
  return Math.log(n) / (base ? Math.log(base) : 1);
});*/
Util.log = (...args) => {
  let location;
  if(args[0] instanceof Util.location) location = args.shift();
  else {
    let stack = Util.getStackFrames(2);
    if(/\/util\.js$/.test(stack[0].fileName)) stack = stack.slice(1);
    location = stack[0].getLocation();
  }
  let locationStr = location.toString(true);
  let c = [(locationStr[inspectSymbol] || locationStr.toString).call(locationStr)];
  c.push(' ');
  let filters = Util.log.filters;
  let results = filters.map(f => f.test(locationStr));
  if(filters.every(f => !f.test(locationStr))) return;
  console.log('log', { args, c });
  Util.putStack();
  args = args.reduce((a, p, i) => {
    if(Util.isObject(p) && p[Util.log.methodName]) p = p[Util.log.methodName]();
    else if(Util.isObject(p) && p[inspectSymbol]) p = p[inspectSymbol]();
    else if(typeof p != 'string') {
      if(Util.isObject(p) && typeof p.toString == 'function' && !Util.isNativeFunction(p.toString)) p = p.toString();
      else p = Util.inspect(p, { multiline: false });
    }

    //  if(i > 0) a.push(',');
    a.push(p);
    //    a.append([p]);
    return a;
  }, c);
  if(args.toConsole) args.toConsole();
  else console.log(...args);
};

Object.defineProperty(Util.log, 'methodName', {
  get: () => (Util.isBrowser() ? 'toConsole' : 'toAnsi256')
});

Util.log.filters = [/.*/];
Util.log.setFilters = function(args) {
  this.filters = [...args].map(arg => (arg instanceof RegExp ? arg : new RegExp(arg)));
};
Util.log.getFilters = function() {
  return this.filters;
};

Util.msg = (strings, ...substitutions) => {
  let i,
    o = [];
  for(i = 0; i < Math.max(strings.length, substitutions.length); i++) {
    if(strings[i] !== undefined) o.push(strings[i].trim());
    if(substitutions[i] !== undefined) o.push(substitutions[i]);
  }
  console.log(...o);
};

Util.logBase = Util.curry((base, n) => Math.log(n) / Math.log(base));

Util.generalLog = function(n, x) {
  return Math.log(x) / Math.log(n);
};
Util.toSource = function(arg, opts = {}) {
  const { quote = "'", colors = false, multiline = false, json = false } = opts;
  const { c = Util.coloring(colors) } = opts;
  let o = [];
  const { print = (...args) => (o = c.concat(o, c.text(...args))) } = opts;
  if(Util.isArray(arg)) {
    print('[', 1, 36);
    for(let item of arg) {
      if(o.length > 0) print(', ');
      Util.toSource(item, { ...opts, c, print });
    }
    print(']', 1, 36);
  } else if(typeof arg == 'number' || arg === undefined || arg === null) print(arg, 1, 35);
  else if(typeof arg == 'string') print(`${quote}${arg}${quote}`, 1, 36);
  else if(arg && arg.x !== undefined && arg.y !== undefined) {
    print('[', 1, 36);
    print(arg.x, 1, 32);
    print(',', 1, 36);
    print(arg.y, 1, 32);
    print(']', 1, 36);
  } else if(typeof arg == 'object') {
    let i = 0;
    let m = arg instanceof Map;
    if(m) {
      print('new ', 1, 31);
      print('Map', 1, 33);
    }
    print((m ? '([[' : '{') + (multiline ? '\n  ' : ' '), 1, 36);
    for(const [prop, value] of Util.entries(arg)) {
      if(i > 0) {
        let s = multiline ? ',\n  ' : ', ';
        if(m) s = ' ]' + s + '[ ';
        print(s, 1, 36);
      }
      if(!m) print(json ? `"${prop}"` : prop, 1, 33);
      else Util.toSource(prop, { ...opts, c, print });
      print(m ? ', ' : ': ', 1, 36);
      Util.toSource(value, { ...opts, c, print });
      i++;
    }
    print(multiline ? '\n' : ' ' + (m ? ']])' : '}'), 1, 36);
  }
  return o;
};
Util.debug = function(message) {
  const args = [...arguments];
  let cache = [];
  const removeCircular = function(key, value) {
    if(typeof value === 'object' && value !== null) {
      if(cache.indexOf(value) !== -1) return;
      cache.push(value);
    }
    return value;
  };
  const str = args
    .map(arg => (typeof arg === 'object' ? JSON.toString(arg, removeCircular) : arg))
    .join(' ')
    .replace(lineSplit, '');
  //console.log("STR: "+str);
  //console.log.call(console, str);
  //Util.log.apply(Util, args)
};
Util.type = function({ type }) {
  return (type && String(type).split(new RegExp('[ ()]', 'g'))[1]) || '';
};
Util.functionName = function(fn) {
  if(typeof fn == 'function' && typeof fn.name == 'string') return fn.name;
  try {
    const matches = /function\s*([^(]*)\(.*/g.exec(fn + '');
    if(matches && matches[1]) return matches[1];
  } catch {}
  return null;
};
Util.className = function(obj) {
  let proto;
  //console.log("class:", obj);
  try {
    proto = Object.getPrototypeOf(obj);
  } catch(err) {
    try {
      proto = obj.prototype;
    } catch(err) {}
  }
  if(Util.isObject(proto) && 'constructor' in proto) return Util.functionName(proto.constructor);
};
Util.unwrapComponent = function(c) {
  for(;;) {
    if(c.wrappedComponent) c = c.wrappedComponent;
    else if(c.WrappedComponent) c = c.WrappedComponent;
    else break;
  }
  return c;
};
Util.componentName = function(c) {
  for(;;) {
    if(c.displayName || c.name) {
      return (c.displayName || c.name).replace(/.*\(([A-Za-z0-9_]+).*/, '$1');
    } else if(c.wrappedComponent) c = c.wrappedComponent;
    else if(c.WrappedComponent) c = c.WrappedComponent;
    else break;
  }
  return Util.fnName(c);
};
Util.count = function(s, ch) {
  return (String(s).match(new RegExp(ch, 'g')) || Util.array()).length;
};
Util.parseNum = function(str) {
  let num = parseFloat(str);
  if(isNaN(num)) num = 0;
  return num;
};
Util.minmax = function(num, min, max) {
  return Math.min(Math.max(num, min), max);
};
Util.getExponential = function(num) {
  let str = typeof num == 'string' ? num : num.toExponential();
  const matches = /e\+?(.*)$/.exec(str);
  //console.log("matches: ", matches);
  return parseInt(matches[1]);
};
Util.getNumberParts = function(num) {
  let str = typeof num == 'string' ? num : num.toExponential();
  const matches = /^(-?)(.*)e\+?(.*)$/.exec(str);
  //console.log("matches: ", matches);
  const negative = matches[1] == '-';
  return {
    negative,
    mantissa: parseFloat(matches[2]),
    exponent: parseInt(matches[3])
  };
};
Util.pow2 = function(n) {
  return Math.pow(2, n);
};
Util.pow10 = function(n) {
  return n >= 0 ? Math.pow(10, n) : 1 / Math.pow(10, -n);
};
Util.bitValue = function(n) {
  return Util.pow2(n - 1);
};
Util.bitMask = function(bits, start = 0) {
  let r = 0;
  let b = 1 << start;

  for(let i = 0; i < bits; i++) {
    r |= b;
    b <<= 1;
  }
  return r;
};

Util.bitGroups = function(num, bpp, minLen) {
  let m = Util.bitMask(bpp, 0);
  let n = Math.floor(64 / bpp);
  let r = [];
  for(let i = 0; i < n; i++) {
    r.push(num & m);
    num /= m + 1;
  }
  while(r.length > 0 && r[r.length - 1] == 0 /* && Util.mod(r.length *bpp, 8) > 0*/) r.pop();
  while(r.length < minLen) r.push(0);
  return r;
};

Util.bitStuff = (arr, bpp) => {
  const m = Util.bitMask(bpp, 0);
  return arr.reduce(([b, f], n) => [b + (n & m) * f, f * (m + 1)], [0, 1])[0];
};

Util.toBinary = function(num) {
  return parseInt(num).toString(2);
};
Util.toBits = function(num) {
  let a = Util.toBinary(num).split('').reverse();
  return Array.from(Object.assign({}, a, { length: 50 }), bit => (bit ? 1 : 0));
};
Util.getBit = function(v, n) {
  let s = v.toString(2);
  return n < s.length ? parseInt(s[s.length - n - 1]) : 0;
};
Util.isSet = function(v, n) {
  return Util.getBit(v, n) == 1;
};
Util.bitCount = function(n) {
  return Util.count(Util.toBinary(n), '1');
};
Util.bitNo = function(n) {
  for(let i = 0; n; i++) {
    if(n & 1) return i;
    n >>= 1;
  }
};

Util.toggleBit = function(num, bit) {
  const n = Number(num);
  return Util.isSet(n, bit) ? n - Util.pow2(bit) : n + Util.pow2(bit);
};
Util.setBit = function(num, bit) {
  const n = Number(num);
  return Util.isSet(n, bit) ? n : n + Util.pow2(bit);
};
Util.clearBit = function(num, bit) {
  const n = Number(num);
  return Util.isSet(n, bit) ? n - Util.pow2(bit) : n;
};
Util.range = function(...args) {
  let [start, end, step = 1] = args;
  let ret;
  start /= step;
  end /= step;
  if(start > end) {
    ret = [];
    while(start >= end) ret.push(start--);
  } else {
    ret = Array.from({ length: end - start + 1 }, (v, k) => k + start);
  }
  if(step != 1) {
    ret = ret.map(n => n * step);
  }
  //console.log("Util.range ", r);
  return ret;
};
Util.set = function(obj, prop, value) {
  const set = obj instanceof Map ? (prop, value) => obj.set(prop, value) : (prop, value) => (obj[prop] = value);
  if(arguments.length == 1)
    return (prop, value) => {
      set(prop, value);
      return set;
    };
  if(arguments.length == 2) return value => set(prop, value);
  return set(prop, value);
};

Util.get = Util.curry((obj, prop) => (obj instanceof Map ? obj.get(prop) : obj[prop]));
Util.symbols = (() => {
  const { asyncIterator, hasInstance, isConcatSpreadable, iterator, match, matchAll, replace, search, species, split, toPrimitive, toStringTag, unscopables } = Symbol;
  return {
    inspect: inspectSymbol,
    asyncIterator,
    hasInstance,
    isConcatSpreadable,
    iterator,
    match,
    matchAll,
    replace,
    search,
    species,
    split,
    toPrimitive,
    toStringTag,
    unscopables
  };
})();

/*
  const { indent = '  ', newline = '\n', depth = 2, spacing = ' ' } = typeof opts == 'object' ? opts : { indent: '', newline: '', depth: typeof opts == 'number' ? opts : 10, spacing: ' ' };

  return Util.formatAnnotatedObject(obj, { indent, newline, depth, spacing });
};*/
Util.bitArrayToNumbers = function(arr) {
  let numbers = [];
  for(let i = 0; i < arr.length; i++) {
    const number = i + 1;
    if(arr[i]) numbers.push(number);
  }
  return numbers;
};
Util.bitsToNumbers = function(bits) {
  let a = Util.toBinary(bits).split('');
  let r = [];
  //return a;
  a.forEach((val, key, arr) => val == '1' && r.unshift(a.length - key));
  return r;
};
Util.shuffle = function(arr, rnd = Util.rng) {
  arr.sort((a, b) => 0.5 - rnd());
  return arr;
};
Util.sortNum = function(arr) {
  arr.sort((a, b) => a - b);
  //console.log("Util.sortNum ", { arr });
  return arr;
};
Util.draw = (arr, n = 1, rnd = Util.rng) => {
  let pos = Util.randInt(0, arr.length - n - 1, rnd);
  const r = arr.splice(pos, n);
  return n == 1 ? r[0] : r;
};
Util.is = function(what, ...pred) {
  let fnlist = pred.map(type => (Util.isConstructor(type) ? what instanceof type : this.is[type]));
  //console.debug('fnlist:', fnlist);
  return fnlist.every(fn => fn(what));
};

Util.instanceOf = (value, ctor) => Util.isObject(value) && Util.isConstructor(ctor) && value instanceof ctor;

Util.onoff = function(val) {
  if(Util.is.on(val)) return true;
  if(Util.is.off(val)) return false;
  return undefined;
};
Util.numbersToBits = function(arr) {
  return arr.reduce((bits, num) => bits + Util.bitValue(num), 0);
};
Util.randomNumbers = function([start, end], draws) {
  const r = Util.draw(Util.range(start, end), draws);
  //console.log("Util.randomNumbers ", { start, end, draws, r });
  return r;
};
Util.randomBits = function(r = [1, 50], n = 5) {
  return Util.numbersToBits(Util.randomNumbers(r, n));
};
Util.padFn = function(len, char = ' ', fn = (str, pad) => pad) {
  return (s, n = len) => {
    let m = Util.stripAnsi(s).length;
    s = s ? s.toString() : '' + s;
    return fn(s, m < n ? char.repeat(n - m) : '');
  };
};
Util.pad = function(s, n, char = ' ') {
  return Util.padFn(n, char)(s);
};
Util.abbreviate = function(str, max = 40, suffix = '...') {
  max = +max;
  if(isNaN(max)) max = Infinity;
  if(Util.isArray(str)) {
    return Array.prototype.slice.call(str, 0, Math.min(str.length, max)).concat([suffix]);
  }
  if(typeof str != 'string' || !Number.isFinite(max) || max < 0) return str;
  str = '' + str;
  if(str.length > max) {
    return str.substring(0, max - suffix.length) + suffix;
  }
  return str;
};
Util.trim = function(str, charset) {
  const r1 = RegExp('^[' + charset + ']*');
  const r2 = RegExp('[' + charset + ']*$');
  return str.replace(r1, '').replace(r2, '');
};
Util.trimRight = function(str, charset) {
  const r2 = RegExp('[' + charset + ']*$');
  return str.replace(r2, '');
};
Util.indent = (text, space = '  ') => {
  text = text.trim();
  if(!/\n/.test(text)) return text;
  return text.replace(/(\n)/g, '\n' + space) + '\n';
};
Util.define = (obj, ...args) => {
  if(typeof args[0] == 'object') {
    const [arg, overwrite = true] = args;
    let adecl = Object.getOwnPropertyDescriptors(arg);
    let odecl = {};
    for(let prop in adecl) {
      if(prop in obj) {
        if(!overwrite) continue;
        else delete obj[prop];
      }
      if(Object.getOwnPropertyDescriptor(obj, prop)) delete odecl[prop];
      else
        odecl[prop] = {
          ...adecl[prop],
          enumerable: false,
          configurable: true,
          writeable: true
        };
    }
    Object.defineProperties(obj, odecl);
    return obj;
  }
  const [key, value, enumerable = false] = args;
  Object.defineProperty(obj, key, {
    enumerable,
    configurable: true,
    writable: true,
    value
  });
  return obj;
};
Util.memoizedProperties = (obj, methods) => {
  let decls = {};
  for(let method in methods) {
    const memoize = Util.memoize(methods[method]);
    decls[method] = {
      get() {
        return memoize.call(this);
      },
      enumerable: true,
      configurable: true
    };
  }
  return Object.defineProperties(obj, decls);
};
Util.copyWhole = (dst, ...args) => {
  let chain = [];
  for(let src of args) chain = chain.concat(Util.getPrototypeChain(src).reverse());
  //console.debug('chain:', ...chain);
  for(let obj of chain) Util.define(dst, obj);
  return dst;
};
Util.copyEntries = (obj, entries) => {
  for(let [k, v] of entries) obj[k] = v;
  return obj;
};

Util.extend = (...args) => {
  let deep = false;
  if(typeof args[0] == 'boolean') deep = args.shift();

  let result = args[0];
  if(Util.isUnextendable(result)) throw new Error('extendee must be an object');
  let extenders = args.slice(1);
  let len = extenders.length;
  for(let i = 0; i < len; i++) {
    let extender = extenders[i];
    for(let key in extender) {
      if(true || extender.hasOwnProperty(key)) {
        let value = extender[key];
        if(deep && Util.isCloneable(value)) {
          let base = Array.isArray(value) ? [] : {};
          result[key] = Util.extend(true, result.hasOwnProperty(key) && !Util.isUnextendable(result[key]) ? result[key] : base, value);
        } else {
          result[key] = value;
        }
      }
    }
  }
  return result;
};

Util.isCloneable = obj => Array.isArray(obj) || {}.toString.call(obj) == '[object Object]';

Util.isUnextendable = val => !val || (typeof val != 'object' && typeof val != 'function');

/*
Util.extend = (obj, ...args) => {
  for(let other of args) {
    for(let key of Util.iterateMembers(other, (k, value) => obj[k] === undefined && [k, value])) {
      const value = other[key];
      try {
        Object.defineProperty(obj, key, {
          value,
          enumerable: false,
          configurable: false,
          writable: false
        });
      } catch(err) {
        console.log('extend:' + err + '\n', { obj, key, value });
      }
    }
  }
  return obj;
};*/

Util.static = (obj, functions, thisObj, pred = (k, v, f) => true) => {
  for(let [name, fn] of Util.iterateMembers(
    functions,

    Util.tryPredicate((key, depth) => obj[key] === undefined && typeof functions[key] == 'function' && pred(key, depth, functions) && [key, value])
  )) {
    const value = function(...args) {
      return fn.call(thisObj || obj, this, ...args);
    };
    try {
      obj[name] = value;

      /*        Object.defineProperty(obj, name, { value, enumerable: false, configurable: false, writable: false });*/
    } catch(err) {
      console.log('static:', err);
    }
  }
  return obj;
};
Util.defineGetter = (obj, key, fn, enumerable = false) =>
  obj[key] === undefined &&
  Object.defineProperty(obj, key, {
    enumerable,
    configurable: true,
    get: fn
  });
Util.defineGetterSetter = (obj, key, g, s, enumerable = false) =>
  obj[key] === undefined &&
  Object.defineProperty(obj, key, {
    get: g,
    set: s,
    enumerable
  });
Util.defineGettersSetters = (obj, gettersSetters) => {
  for(let name in gettersSetters) Util.defineGetterSetter(obj, name, gettersSetters[name], gettersSetters[name]);
};

Util.extendArray = function(arr = Array.prototype) {
  /*  Util.define(arr, 'tail', function() {
    return this[this.length - 1];
  });*/
  Util.define(arr, 'match', function(pred) {
    return Util.match(this, pred);
  });
  Util.define(arr, 'clear', function() {
    this.splice(0, this, length);
    return this;
  });
  Util.define(arr, 'unique', function() {
    return this.filter((item, i, a) => a.indexOf(item) == i);
  });
  Util.defineGetterSetter(
    arr,
    'tail',
    function() {
      return Util.tail(this);
    },
    function(value) {
      if(this.length == 0) this.push(value);
      else this[this.length - 1] = value;
    }
  );

  /*Util.define(arr, 'inspect', function(opts = {}) {
    return Util.inspect(this, { depth: 100, ...opts });
  });*/
};
Util.adapter = function(obj, getLength = obj => obj.length, getKey = (obj, index) => obj.key(index), getItem = (obj, key) => obj[key], setItem = (obj, index, value) => (obj[index] = value)) {
  const adapter = obj && {
    /* prettier-ignore */ get length() {
      return getLength(obj);
    },
    /* prettier-ignore */ get instance() {
      return obj;
    },
    key(i) {
      return getKey(obj, i);
    },
    get(key) {
      return getItem(obj, key);
    },
    has(key) {
      return this.get(key) !== undefined;
    },
    set(key, value) {
      return setItem(obj, key, value);
    },
    *keys() {
      const length = getLength(obj);
      for(let i = 0; i < length; i++) yield getKey(obj, i);
    },
    *entries() {
      for(let key of this.keys()) yield [key, getItem(obj, key)];
    },
    [Symbol.iterator]() {
      return this.entries();
    },
    toObject() {
      return Object.fromEntries(this.entries());
    },
    toMap() {
      return new Map(this.entries());
    }
  };
  return adapter;
};
Util.adapter.localStorage = function(s) {
  s = Util.tryCatch(
    () => !s && globalThis.window,
    w => w.localStorage,
    () => s
  );

  return Util.adapter(
    s,
    l => l.length,
    (l, i) => l.key(i),
    (l, key) => JSON.parse(l.getItem(key)),
    (l, key, v) => l.setItem(key, JSON.toString(v))
  );
};
let doExtendArray = Util.extendArray;
Util.array = function(a) {
  if(!(a instanceof Array)) {
    if(Util.isObject(a) && 'length' in a) a = Array.from(a);
  }
  if(doExtendArray)
    try {
      /*  if(Array.prototype.match === undefined) doExtendArray(Array.prototype);*/
      if(a.match === undefined) {
        doExtendArray(Array.prototype);
        if(a.match) doExtendArray = null;
      }
      if(a.match === undefined) doExtendArray(a);
    } catch(err) {}
  return a;
};
Util.arrayFromEntries = entries =>
  Array.from(
    entries.map(([k, v]) => k),
    key => entries.find(([k, v]) => k === key)[1]
  );

Util.toMap = function(hash = {}, fn) {
  let m, gen;
  if(hash instanceof Array && typeof fn == 'function') hash = hash.map(fn);

  if(hash[Symbol.iterator] !== undefined) gen = hash[Symbol.iterator]();
  else if(Util.isGenerator(hash)) gen = hash;
  else gen = Object.entries(hash);

  m = new Map(gen);

  /*
  if(m instanceof Array) m[Symbol.iterator] = m.entries;*/
  try {
    //if(m.toObject === undefined) Util.extendMap();
    if(Map.prototype.toObject === undefined) Util.extendMap(Map.prototype);
  } catch(err) {}
  return m;
};
Util.extendMap = function(map) {
  if(map.entries === undefined) {
    map.entries = function* iterator() {
      for(let entry of map) {
        yield entry.name !== undefined && entry.value !== undefined ? [entry.name, entry.value] : entry[0] !== undefined && entry[1] !== undefined ? entry : [entry, map[entry]];
      }
    };
  }
  map.toObject = function() {
    return Object.fromEntries(this.entries());
  };
  map.match = function(...args) {
    return Util.match.apply(this, args);
  };
};
Util.fromEntries = Object.fromEntries
  ? Object.fromEntries
  : entries => {
      let ret = {};
      for(let [k, v] of entries) {
        ret[k] = v;
      }
      return ret;
    };

Util.objectFrom = function(any) {
  if('toJS' in any) any = any.toJS();
  else if(Util.isArray(any)) return Util.fromEntries(any);
  else if('entries' in any) return Util.fromEntries(any.entries());
  return Object.assign({}, any);
};
Util.tail = function(arr) {
  return arr && arr.length > 0 ? arr[arr.length - 1] : null;
};
Util.splice = function(str, index, delcount, insert) {
  const chars = str.split('');
  Array.prototype.splice.apply(chars, arguments);
  return chars.join('');
};
Util.identity = arg => arg;
Util.reverse = arr => arr.reverse();

Util.keyOf = function(obj, prop) {
  const keys = Object.keys(obj);
  for(let k in keys) {
    if(obj[k] === prop) return k;
  }
  return undefined;
};
Util.rotateRight = function(arr, n) {
  arr.unshift(...arr.splice(n, arr.length));
  return arr;
};
Util.repeater = function(n, what) {
  if(typeof what == 'function')
    return (function* () {
      for(let i = 0; i < n; i++) yield what();
    })();
  return (function* () {
    for(let i = 0; i < n; i++) yield what;
  })();
};
Util.repeat = function(n, what) {
  return [...Util.repeater(n, what)];
};
Util.arrayDim = function(dimensions, init) {
  let args = [...dimensions];
  args.reverse();
  let ret = init;
  while(args.length > 0) {
    const n = args.shift();
    ret = Util.repeat(n, ret);
  }
  return ret;
};
Util.flatten = function(arr) {
  let ret = [];
  for(let i = 0; i < arr.length; i++) {
    ret = [...ret, ...arr[i]];
  }
  return ret;
};
Util.chunkArray = (a, size) =>
  a.reduce((acc, item, i) => {
    const idx = i % size;
    if(idx == 0) acc.push([]);

    acc[acc.length - 1].push(item);
    return acc;
  }, []);

Util.partition = function* (a, size) {
  for(let i = 0; i < a.length; i += size) yield a.slice(i, i + size);
};

Util.difference = (a, b, incicludes) => {
  //console.log('Util.difference', { a, b, includes });
  if(typeof includes != 'function') return [a.filter(x => !b.includes(x)), b.filter(x => !a.includes(x))];

  return [a.filter(x => !includes(b, x)), b.filter(x => !includes(a, x))];
};
Util.intersect = (a, b) => a.filter(Set.prototype.has, new Set(b));
Util.symmetricDifference = (a, b) => [].concat(...Util.difference(a, b));
Util.union = (a, b, equality) => {
  if(equality === undefined) return [...new Set([...a, ...b])];

  return Util.unique([...a, ...b], equality);
};

Util.chances = function(numbers, matches) {
  const f = Util.factorial;
  return f(numbers) / (f(matches) * f(numbers - matches));
};
Util.sum = arr => arr.reduce((acc, n) => acc + n, 0);

Util.expr = fn => {
  let nargs = fn.length;
  let ret = Util.curry(fn);

  return ret;
  return expr;
  function expr(...args) {
    let nums = [];

    function addArgs(args) {
      while(args.length > 0) {
        const arg = args.shift();

        if(typeof arg == 'function') args.unshift(arg(...args.splice(0, arg.length)));
        else if(typeof arg == 'number') nums.push(arg);
      }
    }
    addArgs(args);
    //console.debug('nargs:', nargs);
    //console.debug('nums.length:', nums.length);
    if(nums.length >= nargs) return fn(...nums);

    //let args = ['a','b','c','d'].slice(0,nargs - nums.length);
    let ret = function returnFn(...args) {
      addArgs(args.slice(0, nargs - nums.length));

      //console.log('nums.length:', nums.length);
      if(nums.length >= nargs) return fn(...nums);
      return returnFn;
    };
    ret.nums = nums;

    return ret;
  }
};

Util.add = Util.curry((a, b) => a + b);
Util.sub = Util.curry((a, b) => a - b);
Util.mul = Util.curry((a, b) => a * b);
Util.div = Util.curry((a, b) => a / b);
Util.xor = Util.curry((a, b) => a ^ b);
Util.or = Util.curry((a, b) => a | b);
Util.and = Util.curry((a, b) => a & b);
Util.mod = (a, b) => (typeof b == 'number' ? ((a % b) + b) % b : n => ((n % a) + a) % a);
Util.pow = Util.curry((a, b) => Math.pow(a, b));

/*Util.define(String.prototype,
  'splice',
  function(index, delcount, insert) {
    return Util.splice.apply(this, [this, ...arguments]);
  }
);*/
Util.fnName = function(f, parent) {
  if(typeof f == 'function') {
    if(f.name !== undefined) return f.name;
    const s = typeof f.toSource == 'function' ? f.toSource() : f + '';
    const matches = /([A-Za-z_][0-9A-Za-z_]*)\w*[(\]]/.exec(s);
    if(matches) return matches[1];
    if(parent !== undefined) {
      for(let key in parent) {
        if(parent[key] === f) return key;
      }
    }
  }
};

Util.objName = function(o) {
  if(o === undefined || o == null) return `${o}`;
  if(typeof o === 'function' || o instanceof Function) return Util.fnName(o);
  if(o.constructor) return Util.fnName(o.constructor);
  const s = `${o.type}`;
  return s;
};
Util.findKey = function(obj, pred, thisVal) {
  let fn = typeof pred == 'function' ? value : v => v === pred;
  for(let k in obj) if(fn.call(thisVal, obj[k], k)) return k;
};
Util.find = function(arr, value, prop = 'id') {
  let pred;
  if(typeof value == 'function') {
    pred = value;
  } else if(prop && prop.length !== undefined) {
    pred = function(obj) {
      if(obj[prop] == value) return true;
      return false;
    };
  } else {
    pred = typeof prop == 'function' ? obj => prop(value, obj) : obj => obj[prop] == value;
  }
  if(typeof arr.find == 'function') return arr.find(pred);
  if(!arr[Symbol.iterator] && typeof arr.entries == 'function') {
    let entryPred = pred;
    pred = ([key, value], arr) => entryPred(value, key, arr);
    arr = arr.entries();
  }
  for(let v of arr) {
    if(pred(v)) return v;
  }
  return null;
};

Util.findIndex = function(obj, pred, thisArg) {
  if(typeof obj.findIndex == 'function') return obj.findIndex(pred, thisArg);
  return Util.findKey(obj, pred, thisArg);
};

Util.match = function(arg, pred) {
  let match = pred;
  if(pred instanceof RegExp) {
    const re = pred;
    match = (val, key) => (val && val.tagName !== undefined && re.test(val.tagName)) || (typeof key === 'string' && re.test(key)) || (typeof val === 'string' && re.test(val));
  }
  if(Util.isArray(arg)) {
    if(!(arg instanceof Array)) arg = [...arg];
    return arg.reduce((acc, val, key) => {
      if(match(val, key, arg)) acc.push(val);
      return acc;
    }, []);
  } else if(Util.isMap(arg)) {
    //console.log('Util.match ', { arg });
    return [...arg.keys()].reduce((acc, key) => (match(arg.get(key), key, arg) ? acc.set(key, arg.get(key)) : acc), new Map());
  }
  return Util.filter(arg, match);
};
Util.toHash = function(map, keyTransform = k => Util.camelize('' + k)) {
  let ret = {};
  Util.foreach(map, (v, k) => (ret[keyTransform(k)] = v));
  return ret;
};
Util.indexOf = function(obj, prop) {
  for(let key in obj) {
    if(obj[key] === prop) return key;
  }
  return undefined;
};

/*
Util.injectProps = function(options) {
  return function(InitialComponent) {
    return function DndStateInjector() {
      return <InitialComponent {...options} />;
    }
  }
}*/

Util.greatestCommonDenominator = (a, b) => (b ? Util.greatestCommonDenominator(b, a % b) : a);

Util.leastCommonMultiple = (n1, n2) => {
  //Find the gcd first
  let gcd = Util.greatestCommonDenominator(n1, n2);

  //then calculate the lcm
  return (n1 * n2) / gcd;
};
Util.matchAll = Util.curry(function* (re, str) {
  let match;
  re = re instanceof RegExp ? re : new RegExp(Util.isArray(re) ? '(' + re.join('|') + ')' : re, 'g');
  do {
    if((match = re.exec(str))) yield match;
  } while(match != null);
});

Util.inspect = function(obj, opts = {}) {
  const {
    quote = '"',
    multiline = true,
    toString = Symbol.toStringTag || 'toString' /*Util.symbols.toStringTag*/,
    stringFn = str => str,
    indent = '',
    colors = false,
    stringColor = [1, 36],
    spacing = '',
    newline = '\n',
    padding = ' ',
    separator = ',',
    colon = ': ',
    depth = 10,
    json = false
  } = {
    ...Util.inspect.defaultOpts,
    toString: Util.symbols.inspect,
    colors: true,
    multiline: true,
    newline: '\n',
    ...opts
  };

  try {
    if(Util == obj) return Util;
  } catch(e) {}
  //console.log("Util.inspect", {quote,colors,multiline,json})

  let out;
  const { c = Util.coloring(colors) } = opts;
  const { print = (...args) => (out = c.concat(out, c.text(...args))) } = opts;
  const sep = multiline && depth > 0 ? (space = false) => newline + indent + (space ? '  ' : '') : (space = false) => (space ? spacing : '');
  if(typeof obj == 'number') {
    print(obj + '', 1, 36);
  } else if(typeof obj == 'undefined' || obj === null) {
    print(obj + '', 1, 35);
  } else if(typeof obj == 'function' /*|| obj instanceof Function || Util.className(obj) == 'Function'*/) {
    obj = '' + obj;
    //  if(!multiline)
    obj = obj.split(lineSplit)[0].replace(/{\s*$/, '{}');
    print(obj);
  } else if(typeof obj == 'string') {
    print(`${quote}${stringFn(obj)}${quote}`, 1, 36);
  } else if(obj instanceof Date) {
    print(`new `, 1, 31);

    print(`Date`, 1, 33);
    print(`(`, 1, 36);
    print(obj.getTime() + obj.getMilliseconds() / 1000, 1, 36);
    print(`)`, 1, 36);
  } else if(Object.getPrototypeOf(obj) == Array.prototype) {
    let i;
    print(`[`, 1, 36);
    for(i = 0; i < obj.length; i++) {
      if(i > 0) print(separator, 1, 36);
      else print(padding);
      print(sep(i > 0));
      Util.inspect(obj[i], {
        ...opts,
        c,
        print,
        newline: newline + '  ',
        depth: depth - 1
      });
    }
    print((padding || '') + `]`, 1, 36);
  } else if(Util.isObject(obj)) {
    const inspect = toString ? obj[toString] : null;
    if(typeof inspect == 'function' && !Util.isNativeFunction(inspect) && !/Util.inspect/.test(inspect + '')) {
      let s = inspect.call(obj, depth, { ...opts });
      //console.debug('s:', s);
      //console.debug('inspect:', inspect + '');

      out += s;
    } else {
      let isMap = obj instanceof Map;
      let keys = isMap ? obj.keys() : Object.getOwnPropertyNames(obj);
      //console.debug("keys:", keys);

      if(Object.getPrototypeOf(obj) !== Object.prototype) print(Util.className(obj) + ' ', 1, 31);
      isMap ? print(`(${obj.size}) {${sep(true)}`, 1, 36) : print('{' + (sep(true) || padding), 1, 36);
      let i = 0;
      let getFn = isMap ? key => obj.get(key) : key => obj[key];
      let propSep = isMap ? [' => ', 0] : [': ', 1, 36];
      for(let key of keys) {
        const value = getFn(key);
        if(i > 0) print(separator + sep(true), 36);
        if(typeof key == 'symbol') print(key.toString(), 1, 32);
        else if(Util.isObject(key) && typeof key[toString] == 'function') print(isMap ? `'${key[toString]()}'` : json ? `"${key.toString()}"` : key[toString](), 1, isMap ? 36 : 33);
        else if(typeof key == 'string' || (!isMap && Util.isObject(key) && typeof key.toString == 'function'))
          print(json ? `"${key.toString()}"` : isMap || /(-)/.test(key) ? `'${key}'` : key, 1, isMap ? 36 : 33);
        else
          Util.inspect(key, {
            ...opts,
            c,
            print,
            newline: newline + '  ',
            newline: '',
            multiline: false,
            toString: 'toString',
            depth: depth - 1
          });
        print(...propSep);
        if(typeof value == 'number') print(`${value}`, 1, 36);
        else if(typeof value == 'string' || value instanceof String) print(`${quote}${value}${quote}`, 1, 36);
        else if(typeof value == 'object')
          Util.inspect(value, {
            ...opts,
            print,
            multiline: isMap && !(value instanceof Map) ? false : multiline,
            newline: newline + '  ',
            depth: depth - 1
          });
        else print((value + '').replace(lineSplit, sep(true)));
        i++;
      }
      print(`${multiline ? newline : padding}}`, 1, 36);
    }
  }
  return out;
};

Util.inspect.defaultOpts = {
  spacing: ' ',
  padding: ' '
};

Util.dump = function(name, props) {
  const args = [name];
  for(let key in props) {
    f;
    args.push(`\n\t${key}: `);
    args.push(props[key]);
  }
  const w = Util.tryCatch(
    () => globalThis.window,
    w => w,
    () => null
  );

  if(w) {
    //if(window.alert !== undefined)
    //alert(args);
    if(w.console !== undefined) w.console.log(...args);
  }
};
Util.ucfirst = function(str) {
  if(typeof str != 'string') str = str + '';
  return str.substring(0, 1).toUpperCase() + str.substring(1);
};
Util.lcfirst = function(str) {
  if(typeof str != 'string') str = str + '';
  return str.substring(0, 1).toLowerCase() + str.substring(1);
};
Util.typeOf = v => {
  let type = typeof v;
  if(type == 'object' && v != null && Object.getPrototypeOf(v) != Object.prototype) type = Util.className(v);
  else type = Util.ucfirst(type);
  return type;
};

/**
 * Camelize a string, cutting the string by multiple separators like
 * hyphens, underscores and spaces.
 *
 * @param {text} string Text to camelize
 * @return string Camelized text
 */
Util.camelize = (text, sep = '') =>
  text.replace(/^([A-Z])|[\s-_]+(\w)/g, (match, p1, p2, offset) => {
    if(p2) return sep + p2.toUpperCase();
    return p1.toLowerCase();
  });

Util.decamelize = function(str, separator = '-') {
  return /.[A-Z]/.test(str)
    ? str
        .replace(/([a-z\d])([A-Z])/g, '$1' + separator + '$2')
        .replace(/([A-Z]+)([A-Z][a-z\d]+)/g, '$1' + separator + '$2')
        .toLowerCase()
    : str;
};
Util.ifThenElse = function(pred = value => !!value, _then = () => {}, _else = () => {}) {
  return function(value) {
    let result = pred(value);
    let ret = result ? _then(value) : _else(value);
    return ret;
  };
};
Util.if = (value, _then, _else, pred) => Util.ifThenElse(pred || (v => !!v), _then || (() => value), _else || (() => value))(value);

Util.ifElse = (value, _else, pred) => Util.ifThenElse(pred || (v => !!v), () => value, _else ? () => _else : () => value)(value);
Util.ifThen = (value, _then, pred) => Util.ifThenElse(pred || (v => !!v), _then ? () => _then : () => value, () => value)(value);

Util.switch = ({ default: defaultCase, ...cases }) =>
  function(value) {
    if(value in cases) return cases[value];
    return defaultCase;
  };

Util.transform = Util.curry(function* (fn, arr) {
  for(let item of arr) yield fn(item);
});

Util.colorDump = (iterable, textFn) => {
  textFn = textFn || ((color, n) => ('   ' + (n + 1)).slice(-3) + ` ${color}`);

  let j = 0;
  const filters = 'font-weight: bold; text-shadow: 0px 0px 1px rgba(0,0,0,0.8); filter: drop-shadow(30px 10px 4px #4444dd)';

  if(!Util.isArray(iterable)) iterable = [...iterable];
  for(let j = 0; j < iterable.length; j++) {
    const [i, color] = iterable[j].length == 2 ? iterable[j] : [j, iterable[j]];
    console.log(
      `  %c    %c ${color} %c ${textFn(color, i)}`,
      `background: ${color}; font-size: 18px; ${filters};`,
      `background: none; color: ${color}; min-width: 120px; ${filters}; `,
      `color: black; font-size: 12px;`
    );
  }
};

Util.bucketInserter = (map, ...extraArgs) => {
  let inserter;
  inserter =
    typeof map.has == 'function'
      ? function(...args) {
          //console.log("bucketInsert:",map,args);
          for(let [k, v] of args) {
            let a;
            map.has(k) ? (a = map.get(k)) : map.set(k, (a = []));
            a.push(v);
          }
          return inserter;
        }
      : function(...args) {
          for(let arg of args) {
            for(let k in arg) {
              const v = arg[k];
              let a = map[k] || [];
              if(typeof a.push == 'function') a.push(v);

              map[k] = a;
            }
          }
        };
  inserter(...extraArgs);
  inserter.map = map;
  return inserter;
};
Util.fifo = function fifo() {
  let resolve = () => {};
  const queue = [];

  //(there's no arrow function syntax for this)
  async function* generator() {
    for(;;) {
      if(!queue.length) {
        //there's nothing in the queue, wait until push()
        await new Promise(r => (resolve = r));
      }
      yield queue.shift();
    }
  }

  return {
    push(...args) {
      for(let event of args) {
        queue.push(event);
        if(queue.length === 1) resolve(); //allow the generator to resume
      }
      return this;
    },
    loop: generator(),

    process: async function run() {
      for await(const event of this.loop) {
        console.info('event:', event);
      }
    }
  };
};
Util.isEmail = function(v) {
  return /^[\-\w]+(\.[\-\w]+)*@[\-\w]+(\.[\-\w]+)+$/.test(v);
};
Util.isString = function(v) {
  return Object.prototype.toString.call(v) == '[object String]';
};

/**
 * Determines whether the specified v is numeric.
 *
 * @param      {<type>}   v       { parameter_description }
 * @return     {boolean}  True if the specified v is numeric, False otherwise.
 */
Util.isNumeric = v => /^[-+]?(0x|0b|0o|)[0-9]*\.?[0-9]+(|[Ee][-+]?[0-9]+)$/.test(v + '');

Util.isUndefined = arg => arg === undefined;
Util.isObject = obj => !(obj === null) && { object: obj, function: obj }[typeof obj];
Util.isPrimitive = obj => !(obj === null) && obj !== false && obj !== true && { number: obj, string: obj, boolean: obj, undefined: obj }[typeof obj];
Util.isFunction = arg => {
  if(arg !== undefined) return typeof arg == 'function' || !!(arg && arg.constructor && arg.call && arg.apply);

  /*
  let fn = arg => Util.isFunction(arg);
  fn.inverse = arg => !Util.isFunction(arg);
  return fn;*/
};
Util.not = fn =>
  function(...args) {
    return !fn(...args);
  };
Util.isAsync = fn => typeof fn == 'function' && /^[\n]*async/.test(fn + '') /*|| fn() instanceof Promise*/;

Util.isArrowFunction = fn => (Util.isFunction(fn) && !('prototype' in fn)) || /\ =>\ /.test(('' + fn).replace(/\n.*/g, ''));

Util.isEmptyString = v => Util.isString(v) && (v == '' || v.length == 0);

Util.isEmpty = (...args) => {
  function empty(v) {
    if(typeof v == 'object' && !!v && v.constructor == Object && Object.keys(v).length == 0) return true;
    if(!v || v === null) return true;
    if(typeof v == 'object' && v.length !== undefined && v.length === 0) return true;
    return false;
  }
  return args.length ? empty(args[0]) : empty;
};
Util.isNonEmpty = (...args) => {
  const empty = Util.isEmpty();
  const nonEmpty = v => !empty(v);
  return args.length ? nonEmpty(args[0]) : nonEmpty;
};
Util.isIpAddress = v => {
  const n = (v + '').split('.').map(i => +i);
  return n.length == 4 && n.every(i => !isNaN(i) && i >= 0 && i <= 255);
};
Util.isPortNumber = v => {
  const n = +v;
  return !isNaN(n) && n >= 0 && n <= 65535;
};

Util.hasProps = function(obj, props) {
  const keys = Object.keys(obj);
  return props ? props.every(prop => 'prop' in obj) : keys.length > 0;
};
Util.validatePassword = function(value) {
  return value.length > 7 && new RegExp('^(?![d]+$)(?![a-zA-Z]+$)(?![!#$%^&*]+$)[da-zA-Z!#$ %^&*]').test(value) && !/\s/.test(value);
};
Util.clone = function(obj, proto) {
  if(Util.isArray(obj)) return obj.slice();
  try {
    let ret = new obj.constructor(obj);
    return ret;
  } catch(err) {}
  if(typeof obj == 'object') return Object.create(proto || obj.constructor.prototype || Object.getPrototypeOf(obj), Object.getOwnPropertyDescriptors(obj));
};
//deep copy
Util.deepClone = function(data) {
  return JSON.parse(JSON.toString(data));
};
//Function
Util.findVal = function(object, propName, maxDepth = 10) {
  if(maxDepth <= 0) return null;
  for(let key in object) {
    if(key === propName) {
      //console.log(propName);
      //console.log(object[key]);
      return object[key];
    }
    let value = Util.findVal(object[key], propName, maxDepth - 1);
    if(value !== undefined) return value;
  }
};
//Deep copy for ObservableArray/Object == There is a problem
Util.deepCloneObservable = function(data) {
  let o;
  const t = typeof data;
  if(t === 'object') return data;

  if(t === 'object') {
    if(data.length) {
      for(const value of data) {
        o.push(this.deepCloneObservable(value));
      }
      return o;
    }
    for(const i in data) {
      o[i] = this.deepCloneObservable(data[i]);
    }
    return o;
  }
};
//Convert ObservableArray to Array
Util.toArray = function(observableArray) {
  return observableArray.slice();
};

/**
 * Convert the original array to tree
 * @param data original array
 * @param id id field
 * @param pId parent id field
 * @param appId the parent id value of the level one array
 */
Util.arryToTree = function(data, id, pId, appId) {
  const arr = [];
  data.map((e, i) => {
    e[pId] === appId && arr.push(e);
  });
  const res = this.to3wei(arr, data, id, pId);
  return res;
};

/**
 * Convert a first-level branch array to a tree
 * @param a level one branch array
 * @param old original array
 * @param id id field
 * @param pId parent id field
 */
Util.to3wei = function(a, old, id, pId) {
  a.map((e, i) => {
    a[i].children = [];
    old.map((se, si) => {
      if(se[pId] === a[i][id]) {
        a[i].children = [...a[i].children, se];
        this.to3wei(a[i].children, old, id, pId);
      }
    });
    if(!a[i].children.length) {
      delete a[i].children;
    }
  });
  return a;
};

/**
 * Exchange 2 element positions in the array
 * @param arr original array
 * @param i First element Starting from 0
 * @param j The second element starts at 0
 */
Util.arrExchangePos = function(arr, i, j) {
  arr[i] = arr.splice(j, 1, arr[i])[0];
};
Util.arrRemove = function(arr, i) {
  const index = arr.indexOf(i);
  if(index > -1) arr.splice(index, 1);
};
Util.move = function(src, dst = []) {
  let items = src.splice(0, src.length);
  dst.splice(dst.length, 0, ...items);
  return dst;
};
Util.moveIf = function(src, pred, dst = []) {
  let items = src.splice(0, src.length);
  let i = 0;
  for(let item of items) (pred(item, i++) ? src : dst).push(item);

  return dst;
};
//Remove the storage when logging out
Util.logOutClearStorage = function() {
  localStorage.removeItem('userToken');
  localStorage.removeItem('userLoginPermission');
  localStorage.removeItem('ssoToken');
  localStorage.removeItem('userId');
  localStorage.removeItem('userInfo');
  localStorage.removeItem('userGroupList');
  localStorage.removeItem('gameAuthList');
};
//Take the cookies
Util.getCookie = function(cookie, name) {
  let arr = cookie.match(new RegExp('(^| )' + name + '=([^;]*)(;|$)'));
  if(arr != null) return unescape(arr[2]);
  return null;
};
Util.parseCookie = function(c = document.cookie) {
  if(!(typeof c == 'string' && c && c.length > 0)) return {};
  let key = '';
  let value = '';
  const ws = ' \r\n\t';
  let i = 0;
  let ret = {};
  const skip = (pred = char => ws.indexOf(char) != -1) => {
    let start = i;
    while(i < c.length && pred(c[i])) i++;
    let r = c.substring(start, i);
    return r;
  };
  do {
    let str = skip(char => char != '=' && char != ';');
    if(c[i] == '=' && str != 'path') {
      i++;
      key = str;
      value = skip(char => char != ';');
    } else {
      i++;
      skip();
    }
    if(key != '') ret[key] = value;
    skip();
  } while(i < c.length);
  return ret;
};

/*
    matches.shift();
    return matches.reduce((acc, part) => {
      const a = part.trim().split('=');
      return { ...acc, [a[0]]: decodeURIComponent(a[1]) };
    }, {});
  };*/
Util.encodeCookie = c =>
  Object.entries(c)
    .map(([key, value]) => `${key}=${encodeURIComponent(value)}`)
    .join('; ');
Util.setCookies = c =>
  Object.entries(c).forEach(([key, value]) => {
    document.cookie = `${key}=${value}`;
    //console.log(`Setting cookie[${key}] = ${value}`);
  });
Util.clearCookies = function(c) {
  return Util.setCookies(
    Object.keys(Util.parseCookie(c)).reduce(
      (acc, name) =>
        Object.assign(acc, {
          [name]: `; max-age=0; expires=${new Date().toUTCString()}`
        }),
      {}
    )
  );
};
Util.deleteCookie = function(name) {
  const w = Util.tryCatch(
    () => globalThis.window,
    w => w,
    () => null
  );

  if(w) document.cookie = `${name}=; expires=Thu, 01 Jan 1970 00:00:01 GMT;`;
};
Util.accAdd = function(arg1, arg2) {
  let r1, r2, m;
  try {
    r1 = arg1.toString().split('.')[1].length;
  } catch(e) {
    r1 = 0;
  }
  try {
    r2 = arg2.toString().split('.')[1].length;
  } catch(e) {
    r2 = 0;
  }
  m = Math.pow(10, Math.max(r1, r2));
  return (arg1 * m + arg2 * m) / m;
};
//js subtraction calculation
//
Util.Subtr = function(arg1, arg2) {
  let r1, r2, m, n;
  try {
    r1 = arg1.toString().split('.')[1].length;
  } catch(e) {
    r1 = 0;
  }
  try {
    r2 = arg2.toString().split('.')[1].length;
  } catch(e) {
    r2 = 0;
  }
  m = Math.pow(10, Math.max(r1, r2));
  //last modify by deeka
  //动态控制精度长度
  n = r1 >= r2 ? r1 : r2;
  return (arg1 * m - arg2 * m) / m;
};
//js division function
//
Util.accDiv = function(arg1, arg2) {
  let t1 = 0;
  let t2 = 0;
  let r1;
  let r2;
  try {
    t1 = arg1.toString().split('.')[1].length;
  } catch(e) {}
  try {
    t2 = arg2.toString().split('.')[1].length;
  } catch(e) {}
  r1 = Number(arg1.toString().replace('.', ''));
  r2 = Number(arg2.toString().replace('.', ''));
  return (r1 / r2) * Math.pow(10, t2 - t1);
};
//js multiplication function
//
Util.accMul = function(arg1, arg2) {
  let m = 0;
  const s1 = arg1.toString();
  const s2 = arg2.toString();
  try {
    m += s1.split('.')[1].length;
  } catch(e) {}
  try {
    m += s2.split('.')[1].length;
  } catch(e) {}
  return (Number(s1.replace('.', '')) * Number(s2.replace('.', ''))) / Math.pow(10, m);
};
Util.dateFormatter = function(date, formate) {
  const year = date.getFullYear();
  let month = date.getMonth() + 1;
  month = month > 9 ? month : `0${month}`;
  let day = date.getDate();
  day = day > 9 ? day : `0${day}`;
  let hour = date.getHours();
  hour = hour > 9 ? hour : `0${hour}`;
  let minute = date.getMinutes();
  minute = minute > 9 ? minute : `0${minute}`;
  let second = date.getSeconds();
  second = second > 9 ? second : `0${second}`;
  return formate
    .replace(/Y+/, `${year}`.slice(-formate.match(/Y/g).length))
    .replace(/M+/, month)
    .replace(/D+/, day)
    .replace(/h+/, hour)
    .replace(/m+/, minute)
    .replace(/s+/, second);
};
Util.numberFormatter = function(numStr) {
  let numSplit = numStr.split('.');
  return numSplit[0].replace(/\B(?=(\d{3})+(?!\d))/g, ',').concat(`.${numSplit[1]}`);
};
Util.searchObject = function(object, matchCallback, currentPath, result, searched) {
  currentPath = currentPath || '';
  result = result || [];
  searched = searched || [];
  if(searched.indexOf(object) !== -1 && object === Object(object)) {
    return;
  }
  searched.push(object);
  if(matchCallback(object)) {
    result.push({ path: currentPath, value: object });
  }
  try {
    if(object === Object(object)) {
      for(const property in object) {
        const desc = Object.getOwnPropertyDescriptor(object, property);
        //console.log('x ', {property, desc})
        if(property.indexOf('$') !== 0 && typeof object[property] !== 'function' && !desc.get && !desc.set) {
          if(typeof object[property] === 'object') {
            try {
              JSON.toString(object[property]);
            } catch(err) {
              continue;
            }
          }
          //if (Object.prototype.hasOwnProperty.call(object, property)) {
          Util.searchObject(object[property], matchCallback, `${currentPath}.${property}`, result, searched);
          //}
        }
      }
    }
  } catch(e) {
    //console.log(object);
    //throw e;
  }
  return result;
};
Util.getURL = Util.memoize((req = {}) =>
  Util.tryCatch(
    () => process.argv[1],
    () => 'file://' + Util.scriptDir(),

    Util.tryCatch(
      () => window.location.href,

      url => url,

      () => {
        let proto = Util.tryCatch(() => (process.env.NODE_ENV === 'production' ? 'https' : null)) || 'http';
        let port = Util.tryCatch(() => (process.env.PORT ? parseInt(process.env.PORT) : process.env.NODE_ENV === 'production' ? 443 : null)) || 3000;
        let host = Util.tryCatch(() => globalThis.ip) || Util.tryCatch(() => globalThis.host) || Util.tryCatch(() => window.location.host.replace(/:.*/g, '')) || 'localhost';
        if(req && req.headers && req.headers.host !== undefined) host = req.headers.host.replace(/:.*/, '');
        else Util.tryCatch(() => process.env.HOST !== undefined && (host = process.env.HOST));
        if(req.url !== undefined) return req.url;
        const url = `${proto}://${host}:${port}`;
        return url;
      }
    )
  )
);
Util.parseQuery = function(url = Util.getURL()) {
  let startIndex;
  let query = {};
  try {
    if((startIndex = url.indexOf('?')) != -1) url = url.substring(startIndex);
    const args = [...url.matchAll(/[?&]([^=&#]+)=?([^&#]*)/g)];
    if(args) {
      for(let i = 0; i < args.length; i++) {
        const k = args[i][1];
        query[k] = decodeURIComponent(args[i][2]);
      }
    }
    return query;
  } catch(err) {
    return undefined;
  }
};
Util.encodeQuery = function(data) {
  const ret = [];
  for(let d in data) if(data[d] !== undefined) ret.push(`${encodeURIComponent(d)}=${encodeURIComponent(data[d])}`);
  return ret.join('&');
};
Util.parseURL = function(href = this.getURL()) {
  //console.debug('href:', href);
  const matches = new RegExp('^([^:]+://)?([^/:]*)(:[0-9]*)?(/?[^#]*)?(#.*)?', 'g').exec(href);
  const [all, proto, host, port, location = '', fragment] = matches;
  //console.debug('matches:', matches);
  if(!matches) return null;
  const argstr = location.indexOf('?') != -1 ? location.replace(/^[^?]*\?/, '') : ''; /* + "&test=1"*/
  const pmatches =
    typeof argstr === 'string'
      ? argstr
          .split(/&/g)
          .map(part => {
            let a = part.split(/=/);
            let b = a.shift();
            return [b, a.join('=')];
          })
          .filter(([k, v]) => !(k.length == 0 && v.length == 0))
      : [];
  const params = [...pmatches].reduce((acc, m) => {
    acc[m[0]] = m[1];
    return acc;
  }, {});
  //console.log("PARAMS: ", { argstr, pmatches, params });
  const ret = {
    protocol: proto ? proto.replace('://', '') : 'http',
    host,
    location: location.replace(/\?.*/, ''),
    query: params
  };
  Object.assign(ret, {
    href(override) {
      if(typeof override === 'object') Object.assign(this, override);
      const qstr = Util.encodeQuery(this.query);
      return (this.protocol ? `${this.protocol}://` : '') + (this.host ? this.host : '') + (this.port ? `:${this.port}` : '') + `${this.location}` + (qstr != '' ? `?${qstr}` : '');
    }
  });
  if(typeof port === 'string') ret.port = parseInt(port.substring(1));
  else if(ret.protocol == 'https') ret.port = 443;
  else if(ret.protocol == 'http') ret.port = 80;
  if(fragment) ret.fragment = fragment;
  return ret;
};
Util.makeURL = function(...args) {
  let href = typeof args[0] == 'string' ? args.shift() : Util.getURL();
  let url = Util.parseURL(href);
  let obj = typeof args[0] == 'object' ? args.shift() : {};
  if('host' in obj /*|| 'protocol' in obj*/) url = Util.filterOutKeys(url, [/*'protocol',*/ 'host', 'port']);
  Object.assign(url, obj);
  return url.href();

  /*
  let href = typeof args[0] === "string" ? args.shift() : this.getURL();
  let urlObj = null;
  urlObj = this.parseURL(href);
  return urlObj ? urlObj.href(args[0]) : null;*/
};
Util.numberFromURL = function(url, fn) {
  const obj = typeof url === 'object' ? url : this.parseURL(url);
  const nr_match = RegExp('.*[^0-9]([0-9]+)$').exec(url.location);
  const nr_arg = nr_match ? nr_match[1] : undefined;
  const nr = nr_arg && parseInt(nr_arg);
  if(!isNaN(nr) && typeof fn === 'function') fn(nr);
  return nr;
};
Util.tryPromise = fn => new Promise((resolve, reject) => Util.tryCatch(fn, resolve, reject));

Util.tryFunction = (fn, resolve = a => a, reject = () => null) => {
  if(typeof resolve != 'function') {
    let rval = resolve;
    resolve = () => rval;
  }
  if(typeof reject != 'function') {
    let cval = reject;
    reject = () => cval;
  }
  return Util.isAsync(fn)
    ? async function(...args) {
        let ret;
        try {
          ret = await fn(...args);
        } catch(err) {
          return reject(err, ...args);
        }
        return resolve(ret, ...args);
      }
    : function(...args) {
        let ret;
        try {
          ret = fn(...args);
        } catch(err) {
          return reject(err, ...args);
        }
        return resolve(ret, ...args);
      };
};
Util.tryCatch = (fn, resolve = a => a, reject = () => null, ...args) => {
  if(Util.isAsync(fn))
    return fn(...args)
      .then(resolve)
      .catch(reject);

  return Util.tryFunction(fn, resolve, reject)(...args);
};
Util.putError = err => {
  let e = Util.isObject(err) && err instanceof Error ? err : Util.exception(err);
  (console.info || console.log)('Util.putError ', e);
  let s = err.stack ? Util.stack(err.stack) : null;

  (console.error || console.log)('ERROR:\n' + err.message + (s ? '\nstack:\n' + s.toString() : s));
};
Util.putStack = (stack = new Util.stack().slice(3)) => {
  stack = stack instanceof Util.stack ? stack : Util.stack(stack);
  console.log('Util.putStack', Util.className(stack));

  console.log('STACK TRACE:\n' + stack.toString());
};

Util.trap = (() => {
  Error.stackTraceLimit = 100;
  return fn => /* prettier-ignore */ Util.tryFunction(fn, ret => ret, Util.putError);
})();

Util.tryPredicate = (fn, defaultRet) =>
  Util.tryFunction(
    fn,
    ret => ret,
    () => defaultRet
  );

Util.isBrowser = function() {
  let ret = false;

  Util.tryCatch(
    () => window,
    w => (Util.isObject(w) ? (ret = true) : undefined),
    () => {}
  );
  Util.tryCatch(
    () => document,
    d => (d == window.document && Util.isObject(d) ? (ret = true) : undefined),
    () => {}
  );
  return ret;
  //return !!(globalThis.window && globalThis.window.document);
};

Util.waitFor = async function waitFor(msecs) {
  if(!globalThis.setTimeout) {
    await import('os').then(({ setTimeout, clearTimeout, setInterval, clearInterval }) => {
      //console.log('', { setTimeout, clearTimeout, setInterval, clearInterval });
      Object.assign(globalThis, {
        setTimeout,
        clearTimeout,
        setInterval,
        clearInterval
      });
    });
  }
  if(msecs <= 0) return;

  let promise, clear, timerId;
  promise = new Promise(async (resolve, reject) => {
    timerId = setTimeout(() => resolve(), msecs);
    clear = () => {
      clearTimeout(timerId);
      reject();
    };
  });
  promise.clear = clear;
  return promise;
};

Util.timeout = async (msecs, promises, promiseClass = Promise) => await promiseClass.race([Util.waitFor(msecs)].concat(Util.isArray(promises) ? promises : [promises]));
Util.isServer = function() {
  return !Util.isBrowser();
};
Util.isMobile = function() {
  return true;
};
Util.uniquePred = (cmp = null) => (typeof cmp == 'function' ? (el, i, arr) => arr.findIndex(item => cmp(el, item)) == i : (el, i, arr) => arr.indexOf(el) == i);

Util.unique = (arr, cmp) => arr.filter(Util.uniquePred(cmp));
Util.allEqual = (arr, cmp = (a, b) => a == b) => arr.every((e, i, a) => cmp(e, a[0]));

Util.zip = a => a.reduce((a, b) => (a.length > b.length ? a : b), []).map((_, i) => a.map(arr => arr[i]));

Util.histogram = (...args) => {
  let arr = args.shift();
  const t = typeof args[0] == 'function' ? args.shift() : (k, v) => k;
  let [out = false ? {} : new Map(), initVal = () => 0 /* new Set()*/, setVal = v => v] = args;

  const set = /*Util.isObject(out) && typeof out.set == 'function' ? (k, v) => out.set(k, v) :*/ Util.setter(out);
  const get = Util.getOrCreate(out, initVal, set);
  let ctor = Object.getPrototypeOf(out) !== Object.prototype ? out.constructor : null;
  let tmp;

  if(Util.isObject(arr) && !Array.isArray(arr) && typeof arr.entries == 'function') arr = arr.entries();
  arr = [...arr];
  let entries = arr.map((it, i) => [i, it]);
  let x = {};
  let iv = initVal();
  const add = Util.adder(iv);
  const upd = Util.updater(out, get, set);

  let r = arr.map((item, i) => {
    let arg;
    let key;
    tmp = t(item, i);
    if(tmp) {
      key = tmp;
      if(Util.isArray(tmp) && tmp.length >= 2) [key, arg] = tmp.slice(-2);
      else arg = tmp;
    }
    [key, arg] = [key].concat(setVal(arg, i)).slice(-2);
    return [
      key,
      upd(key, (entry, idx, key) => {
        return add(entry, typeof entry == 'number' ? 1 : item);
      })
    ];
  });
  return out;
  //console.debug('r:', r);
  if(ctor) {
    let entries = r;
    let keys = r.map(([k, v]) => k);
    entries = [...entries].sort((a, b) => b[1] - a[1]);
    let tmp = new ctor(entries);
    r = tmp;
  }
  return r;
};
Util.concat = function* (...args) {
  for(let arg of args) {
    if(Util.isGenerator(arg)) {
      console.error('isGenerator:', arg);
      yield* arg;
    } else {
      /* if(Util.isArray(arg))*/
      for(let item of arg) yield item;
    }

    /*   else  else {
      throw new Error("No such arg type:"+typeof(arg));
    }*/
  }
};
Util.distinct = function(arr) {
  return Array.prototype.filter.call(arr, (value, index, me) => me.indexOf(value) === index);
};
Util.rangeMinMax = function(arr, field) {
  const numbers = [...arr].map(obj => obj[field]);
  return [Math.min(...numbers), Math.max(...numbers)];
};

Util.remap = (...args) => {
  const getR = () => (Util.isArray(args[0]) ? args.shift() : args.splice(0, 2));
  const _from = getR(),
    to = getR();

  const f = [to[1] - to[0], _from[1] - _from[0]];
  const factor = f[0] / f[1];

  const r = val => (val - _from[0]) * factor + to[0];

  return r;
};
Util.mergeLists = function(arr1, arr2, key = 'id') {
  let hash = {};

  for(let obj of arr1) hash[obj[key]] = obj;
  for(let obj of arr2) hash[obj[key]] = obj;
  return Object.values(hash);

  /* let hash = arr1.reduce((acc, it) => Object.assign({ [it[key]]: it }, acc), {});
  hash = arr2.reduce((acc, it) => Object.assign({ [it[key]]: it }, acc), {});
  let ret = [];
  for(let k in hash) {
    if(hash[k][key]) ret.push(hash[k]);
  }
  return ret;*/
};

Util.foreach = function(o, fn) {
  for(let [k, v] of Util.entries(o)) {
    if(fn(v, k, o) === false) break;
  }
};
Util.all = function(obj, pred) {
  for(let k in obj) if(!pred(obj[k])) return false;
  return true;
};
Util.isGenerator = function(fn) {
  return (typeof fn == 'function' && /^[^(]*\*/.test(fn.toString())) || (['function', 'object'].indexOf(typeof fn) != -1 && fn.next !== undefined);
};
Util.isIterator = obj => Util.isObject(obj) && typeof obj.next == 'function';

Util.isIterable = obj => {
  try {
    for(let item of obj) return true;
  } catch(err) {}
  return false;
};
Util.isNativeFunction = Util.tryFunction(x => typeof x == 'function' && /^[^\n]*\[(native\ code|[a-z\ ]*)\]/.test(x + ''));

Util.isConstructor = x => {
  if(x !== undefined) {
    let ret,
      members = [];
    const handler = {
      construct(target, args) {
        return Object.create(target.prototype);
      }
    };
    try {
      ret = new new Proxy(x, handler)();
    } catch(e) {
      ret = false;
    }
    let proto = (x && x.prototype) || Object.getPrototypeOf(ret);
    members = Util.getMemberNames(proto).filter(m => m !== 'constructor');
    //console.log('members:', !!ret, members, Util.fnName(x));
    return !!ret && members.length > 0;
  }
};

Util.filter = function(a, pred) {
  if(typeof pred != 'function') pred = Util.predicate(pred);
  if(Util.isArray(a)) return a.filter(pred);
  /*return (function* () {
      for(let [k, v] of a.entries()) if(pred(v, k, a)) yield v;
    })();*/

  if(Util.isGenerator(a))
    return (function* () {
      for(let item of a) if(pred(item)) yield item;
    })();
  let isa = Util.isArray(a);
  let ret = {};
  let fn = (k, v) => (ret[k] = v);
  for(let [k, v] of Util.entries(a)) if(pred(v, k, a)) fn(k, v);
  return Object.setPrototypeOf(ret, Object.getPrototypeOf(a));
};
Util.reduce = (obj, fn, accu) => {
  if(Util.isGenerator(obj)) {
    let i = 0;
    for(let item of obj) accu = fn(accu, item, i++, obj);
    return accu;
  }
  for(let key in obj) accu = fn(accu, obj[key], key, obj);
  return accu;
};
Util.mapFunctional = fn =>
  function* (arg) {
    for(let item of arg) yield fn(item);
  };
Util.map = (...args) => {
  let [obj, fn] = args;
  let ret = a => a;

  if(Util.isIterator(obj)) {
    return ret(function* () {
      let i = 0;
      for(let item of obj) yield fn(item, i++, obj);
    })();
  }
  if(typeof obj == 'function') return Util.mapFunctional(...args);

  if(typeof obj.map == 'function') return obj.map(fn);

  if(typeof obj.entries == 'function') {
    const ctor = obj.constructor;
    obj = obj.entries();
    ret = a => new ctor([...a]);
    //    ret = a => new ctor(a);
  }

  /*console.log("obj",(obj));
console.log("isGenerator",Util.isGenerator(obj));*/

  if(Util.isGenerator(obj))
    return ret(
      (function* () {
        let i = 0;
        for(let item of obj) yield fn(item, i++, obj);
      })()
    );
  //  if(typeof fn != 'function') return Util.toMap(...arguments);

  ret = {};
  for(let key in obj) {
    if(obj.hasOwnProperty(key)) {
      let item = fn(key, obj[key], obj);
      if(item) ret[item[0]] = item[1];
    }
  }
  return ret; //Object.setPrototypeOf(ret,Object.getPrototypeOf(obj));
};

/*Util.indexedMap = (arr, fn = arg => arg.name) => {
  return new Proxy(arr, {
    get(target, prop, receiver) {
      let idx = arr.findIndex(item => fn(item) == 'prop');
      if(idx != -1)
        prop = idx;

      return Reflect.get(arr, idx, receiver);
    }
  });
};*/

Util.entriesToObj = function(arr) {
  return [...arr].reduce((acc, item) => {
    const k = item[0];
    const v = item[1];
    acc[k] = v;
    return acc;
  }, {});
};
Util.isDate = function(d) {
  return d instanceof Date || (typeof d == 'string' && /[0-9][0-9][0-9][0-9]-[0-9][0-9]-[0-9][0-9]T[0-9][0-9]:[0-9][0-9]:[0-9][0-9]/.test(d));
};
Util.parseDate = function(d) {
  if(Util.isDate(d)) {
    d = new Date(d);
  }
  return d;
  //return /^[0-9]+$/.test(d) ? Util.fromUnixTime(d) : new Date(d);
};
Util.isoDate = function(date) {
  try {
    if(typeof date == 'number') date = new Date(date);
    const minOffset = date.getTimezoneOffset();
    const milliseconds = date.valueOf() - minOffset * 60 * 1000;
    date = new Date(milliseconds);
    return date.toISOString().replace(/T.*/, '');
  } catch(err) {}
  return null;
};
Util.toUnixTime = function(dateObj, utc = false) {
  if(!(dateObj instanceof Date)) dateObj = new Date(dateObj);
  let epoch = Math.floor(dateObj.getTime() / 1000);
  if(utc) epoch += dateObj.getTimezoneOffset() * 60;
  return epoch;
};
Util.unixTime = function(utc = false) {
  return Util.toUnixTime(new Date(), utc);
};
Util.fromUnixTime = function(epoch, utc = false) {
  let t = parseInt(epoch);
  let d = new Date(0);
  utc ? d.setUTCSeconds(t) : d.setSeconds(t);
  return d;
};
Util.formatTime = function(date = new Date(), format = 'HH:MM:SS') {
  let n;
  let out = '';
  if(typeof date == 'number') date = new Date(date);
  for(let i = 0; i < format.length; i += n) {
    n = 1;
    while(format[i] == format[i + n]) n++;
    const fmt = format.substring(i, i + n);
    let num = fmt;
    if(fmt.startsWith('H')) num = `0${date.getHours()}`.substring(0, n);
    else if(fmt.startsWith('M')) num = `0${date.getMinutes()}`.substring(0, n);
    else if(fmt.startsWith('S')) num = `0${date.getSeconds()}`.substring(0, n);
    out += num;
  }
  return out;
};
Util.leapYear = function(year) {
  if(year % 400 == 0) return true;
  if(year % 100 == 0) return false;
  if(year % 4 == 0) return true;
  return false;
};
Util.timeSpan = function(s) {
  const seconds = s % 60;
  s = Math.floor(s / 60);
  const minutes = s % 60;
  s = Math.floor(s / 60);
  const hours = s % 24;
  s = Math.floor(s / 24);
  const days = s % 7;
  s = Math.floor(s / 7);
  const weeks = s;
  let ret = '';
  ret = `${('0' + hours).substring(0, 2)}:${('0' + minutes).substring(0, 2)}:${('0' + seconds).substring(0, 2)}`;
  if(days) ret = `${days} days ${ret}`;
  if(weeks) ret = `${weeks} weeks ${ret}`;
  return ret;
};
Util.rng = Math.random;
Util.randFloat = function(min, max, rnd = Util.rng) {
  return rnd() * (max - min) + min;
};
Util.randInt = (...args) => {
  let range = args.splice(0, 2);
  let rnd = args.shift() || Util.rng;
  if(range.length < 2) range.unshift(0);
  return Math.round(Util.randFloat(...range, rnd));
};
Util.randStr = (len, charset, rnd = Util.rng) => {
  let o = '';
  if(!charset) charset = '_0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz';

  while(--len >= 0) {
    o += charset[Math.round(rnd() * (charset.length - 1))];
  }
  return o;
};

Util.hex = function(num, numDigits) {
  let v = typeof num == 'number' ? num : parseInt(num);
  let s = v.toString(16);
  numDigits = numDigits || Math.floor((s.length + 1) / 2) * 2;
  return ('0'.repeat(numDigits) + s).slice(-numDigits);
};
Util.numberParts = (num, base) => {
  let exp = 0;
  let sgn = 0;
  if(num === 0) return { sign: 0, mantissa: 0, exponent: 0 };
  if(num < 0) (sgn = 1), (num = -num);
  while(num > base) (num /= base), exp++;
  while(num < 1) (num *= base), exp--;
  return { sign: sgn, mantissa: num, exponent: exp };
};
Util.roundDigits = precision => {
  precision = precision + '';
  let index = precision.indexOf('.');
  let frac = index == -1 ? '' : precision.slice(index + 1);
  return frac.length;

  return -Util.clamp(-Infinity, 0, Math.floor(Math.log10(precision - Number.EPSILON)));
};

Util.roundFunction = (prec, digits, type) => {
  digits = digits || Util.roundDigits(prec);
  type = type || 'round';

  const fn = Math[type];
  if(prec == 1) return fn;

  return function(value) {
    let ret = fn(value / prec) * prec;
    if(typeof digits == 'number' && digits >= 1 && digits <= 100) ret = +ret.toFixed(digits);
    return ret;
  };
};
Util.roundTo = function(value, prec, digits, type) {
  if(!isFinite(value)) return value;
  digits = digits || Util.roundDigits(prec);
  type = type || 'round';
  const fn = Math[type];
  if(prec == 1) return fn(value);
  let ret = prec > Number.EPSILON ? fn(value / prec) * prec : value;

  if(typeof digits == 'number' && digits >= 1 && digits <= 100) ret = +ret.toFixed(digits);
  else ret = Math[type](ret);
  return ret;
};
Util.base64 = (() => {
  const g = Util.getGlobalObject();

  return {
    encode: Util.tryFunction(
      utf8 => g.btoa(g.unescape(g.encodeURIComponent(utf8))),
      v => v,
      utf8 => Buffer.from(utf8).toString('base64')
    ),
    decode: Util.tryFunction(
      base64 => g.decodeURIComponent(g.escape(g.atob(base64))),
      v => v,
      string => Buffer.from(string, 'base64').toString('utf-8')
    )
  };
})();

Util.formatRecord = function(obj) {
  let ret = {};
  for(let key in obj) {
    let val = obj[key];
    if(val instanceof Array) val = val.map(item => Util.formatRecord(item));
    else if(/^-?[0-9]+$/.test(val)) val = parseInt(val);
    else if(/^-?[.0-9]+$/.test(val)) val = parseFloat(val);
    else if(val == 'true' || val == 'false') val = Boolean(val);
    ret[key] = val;
  }
  return ret;
};
Util.isArray =
  Array.isArray ||
  function(obj) {
    if(obj.constructor === Array) return true;
    return (
      (obj && !Util.isGetter(obj, 'length') && Util.isObject(obj) && 'length' in obj && !(obj instanceof String) && !(obj instanceof Function) && typeof obj == 'function') || obj instanceof Array
    );
  };
Util.isArrayLike = obj => typeof obj == 'object' && obj !== null && 'length' in obj;

Util.equals = function(a, b) {
  if(Util.isArray(a) && Util.isArray(b)) {
    return a.length == b.length && a.every((e, i) => b[i] === e);
  } else if(Util.isObject(a) && Util.isObject(b)) {
    const size_a = Util.size(a);

    if(size_a != Util.size(b)) return false;

    for(let k in a) if(!Util.equals(a[k], b[k])) return false;

    return true;
  }
  return a == b;
};
/*#define _GNU_SOURCE
#include <ctype.h>
#include <string.h>

int
strverscmp(const char* a0, const char* b0) {
  const unsigned char* a = (const void*)a0;
  const unsigned char* b = (const void*)b0;
  size_t i, dp, j;
  int z = 1;
  for(dp = i = 0; a[i] == b[i]; i++) {
    int c = a[i];
    if(!c)
      return 0;
    if(!isdigit(c))
      dp = i + 1, z = 1;
    else if(c != '0')
      z = 0;
  }
  if(a[dp] != '0' && b[dp] != '0') {
    for(j = i; isdigit(a[j]); j++)
      if(!isdigit(b[j]))
        return 1;
    if(isdigit(b[j]))
      return -1;
  } else if(z && dp < i && (isdigit(a[i]) || isdigit(b[i]))) {
    return (unsigned char)(a[i] - '0') - (unsigned char)(b[i] - '0');
  }
  return a[i] - b[i];
}*/
Util.versionCompare = (a, b) => {
  // console.log("Util.versionCompare",{a,b});
  if(typeof a != 'string') a = a + '';
  if(typeof b != 'string') b = b + '';

  let i,
    dp,
    j,
    z = 1;
  const isdigit = c => /^[0-9]$/.test(c);

  for(dp = i = 0; a[i] == b[i]; i++) {
    let c;
    if(!(c = a[i])) return 0;
    if(!isdigit(c)) (dp = i + 1), (z = 1);
    else if(c != '0') z = 0;
  }
  if(a[dp] != '0' && b[dp] != '0') {
    for(j = i; isdigit(a[j]); j++) if(!isdigit(b[j])) return 1;
    if(isdigit(b[j])) return -1;
  } else if(z && dp < i && (isdigit(a[i]) || isdigit(b[i]))) {
    return a.codePointAt(i) - 0x30 - (b.codePointAt(i) - 0x30);
  }

  return a.codePointAt(i) - b.codePointAt(i);
};

/*
Util.isObject = function(obj) {
  const type = typeof obj;
  return type === 'function' || (type === 'object' && !!obj);
};*/

Util.isGetter = (obj, propName) => {
  while(obj) {
    let desc = Object.getOwnPropertyDescriptor(obj, propName);
    if(desc && 'get' in desc) return true;
    obj = Object.getPrototypeOf(obj);
  }
  return false;
};
Util.isBool = value => value === true || value === false;
Util.size = (...args) => {
  function size(obj) {
    if(Util.isObject(obj)) {
      if(obj instanceof Map) return obj.size;
      else if('length' in obj) return obj.length;
      else return Object.keys(obj).length;
    }
  }
  if(args.length == 0) return size;
  return size(args[0]);
};
Util.isMap = function(obj) {
  return (obj && obj.get !== undefined && obj.keys !== undefined) || obj instanceof Map;
};
Util.effectiveDeviceWidth = function() {
  let deviceWidth = window.orientation == 0 ? window.screen.width : window.screen.height;
  //iOS returns available pixels, Android returns pixels / pixel ratio
  //http://www.quirksmode.org/blog/archives/2012/07/more_about_devi.html
  if(navigator.userAgent.indexOf('Android') >= 0 && window.devicePixelRatio) {
    deviceWidth = deviceWidth / window.devicePixelRatio;
  }
  return deviceWidth;
};
Util.getFormFields = function(initialState) {
  return Util.mergeObjects([
    initialState,
    [...document.forms].reduce(
      (acc, { elements }) => [...elements].reduce((acc2, { name, value }) => (name == '' || value == undefined || value == 'undefined' ? acc2 : Object.assign(acc2, { [name]: value })), acc),
      {}
    )
  ]);
};
Util.mergeObjects = function(objArr, predicate = (dst, src, key) => (src[key] == '' ? undefined : src[key])) {
  let args = objArr;
  let obj = {};
  for(let i = 0; i < args.length; i++) {
    for(let key in args[i]) {
      const newVal = predicate(obj, args[i], key);
      if(newVal != undefined) obj[key] = newVal;
    }
  }
  return obj;
};
Util.getUserAgent = function(headers = req.headers) {
  const agent = useragent.parse(headers['user-agent']);
  return agent;
};
Util.factor = function(start, end) {
  let f = 1;
  for(let i = start; i <= end; i++) {
    f = f * i;
  }
  return f;
};
Util.factorial = function(n) {
  return Util.factor(1, n);
};
Util.increment = function(obj, key) {
  if(obj[key] >= 1) obj[key] == 0;
  obj[key]++;
  return obj[key];
};
Util.counter = function() {
  let i = 0;
  let self = function() {
    return i++;
  };
  return self;
};
Util.filterKeys = function(obj, pred = k => true) {
  let ret = {};
  if(pred instanceof RegExp) {
    let re = pred;
    pred = str => re.test(str);
  } else if(Util.isArray(pred)) {
    let a = pred;
    pred = str => a.indexOf(str) != -1;
  }
  for(let key in obj) if(pred(key, obj[key], obj)) ret[key] = obj[key];
  //Object.setPrototypeOf(ret, Object.getPrototypeOf(obj));
  return ret;
};
Util.filterMembers = function(obj, fn) {
  const pred = (k, v, o) => fn(v, k, o);
  return Util.filterKeys(obj, pred);
};
Util.filterOutMembers = function(obj, fn) {
  const pred = (v, k, o) => !fn(v, k, o);
  return Util.filterMembers(obj, pred);
};
Util.dumpMembers = obj => Util.filterOutMembers(obj, Util.isFunction);

Util.filterOutKeys = function(obj, arr) {
  if(typeof obj != 'object') return obj;
  const pred =
    typeof arr == 'function' ? (v, k, o) => arr(k, v, o) : arr instanceof RegExp ? (k, v) => arr.test(k) /*|| arr.test(v)*/ : Array.isArray(arr) ? key => arr.indexOf(key) != -1 : () => ({});
  return Util.filterOutMembers(obj, (v, k, o) => pred(k, v, o));
};
Util.removeKeys = function(obj, arr) {
  if(typeof obj != 'object') return obj;
  const pred = typeof arr == 'function' ? (v, k, o) => arr(k, v, o) : arr instanceof RegExp ? (k, v) => arr.test(k) /*|| arr.test(v)*/ : key => arr.indexOf(key) != -1;
  for(let key in obj) {
    if(pred(key, obj[key], obj)) delete obj[key];
  }
};
Util.getKeys = function(obj, arr) {
  let ret = {};
  for(let key of arr) ret[key] = obj[key];

  return ret;
};
Util.numbersConvert = function(str) {
  return str
    .split('')
    .map((ch, i) => (new RegExp('[ :,./]').test(ch) ? ch : String.fromCharCode((str.charCodeAt(i) & 0x0f) + 0x30)))
    .join('');
};
Util.entries = function(arg) {
  if(Util.isArray(arg) || Util.isObject(arg)) {
    if(typeof arg.entries == 'function') return arg.entries();
    else if(Util.isIterable(arg))
      return (function* () {
        for(let key in arg) yield [key, arg[key]];
      })();
    return Object.entries(arg);
  }
};
Util.keys = function(arg) {
  let ret;
  if(Util.isObject(arg)) {
    ret =
      typeof arg.keys == 'function'
        ? arg.keys
        : function* () {
            for(let key in this) yield key;
          };
  }
  if(ret) return ret.call(arg);
};
Util.values = function(arg) {
  let ret;
  if(Util.isObject(arg)) {
    ret =
      typeof arg.values == 'function'
        ? arg.values
        : function* () {
            for(let key in arg) yield arg[key];
          };
  }
  if(ret) return ret.call(arg);
};
Util.removeEqual = function(a, b) {
  let c = {};
  for(let key of Util.keys(a)) {
    if(b[key] === a[key]) continue;
    //console.log(`removeEqual '${a[key]}' === '${b[key]}'`);
    c[key] = a[key];
  }
  //console.log(`removeEqual`,c);

  return c;
};
Util.clear = obj => (typeof obj.splice == 'function' ? obj.splice(0, obj.length) : obj.clear());

Util.remove = (arr, item) => Util.removeIf(arr, (other, i, arr) => item === other);
Util.removeIf = function(arr, pred) {
  let count = 0;
  if(Util.isObject(arr) && typeof arr.splice == 'function') {
    let idx;
    for(count = 0; (idx = arr.findIndex(pred)) != -1; count++) arr.splice(idx, idx + 1);
  } else {
    for(let [key, value] of arr) {
      if(pred(value, key, arr)) {
        arr.delete(key);
        count++;
      }
    }
  }
  return count;
};
Util.traverse = function(o, fn) {
  if(typeof fn == 'function')
    return Util.foreach(o, (v, k, a) => {
      fn(v, k, a);
      if(typeof v === 'object') Util.traverse(v, fn);
    });
  function* walker(o, depth = 0) {
    for(let [k, v] of Util.entries(o)) {
      yield [v, k, o, depth];
      if(typeof v == 'object' && v !== null) yield* walker(v, depth + 1);
    }
  }
  return walker(o);
};
Util.traverseWithPath = function(o, rootPath = []) {
  for(let key of rootPath) o = o[key];

  function* walker(o, path) {
    for(let [k, v] of Util.entries(o)) {
      let p = [...path, k];
      yield [v, k, o, p];
      if(typeof v == 'object' && v !== null) yield* walker(v, p);
    }
  }

  return walker(o, []);
};
Util.indexByPath = function(o, p) {
  for(let key of p) o = o[key];
  return o;
};

Util.pushUnique = (arr, ...args) => args.reduce((acc, item) => (arr.indexOf(item) == -1 ? (arr.push(item), acc + 1) : acc), 0);

Util.insertSorted = function(arr, item, cmp = (a, b) => b - a) {
  let i = 0,
    len = arr.length;
  while(i < len) {
    if(cmp(item, arr[i]) >= 0) break;
    i++;
  }
  i < len ? arr.splice(i, 0, item) : arr.push(item);
  return i;
};
Util.inserter = (dest, next = (k, v) => {}) => {
  // if(typeof dest == 'function' && dest.map !== undefined) dest = dest.map;

  const insert =
    /*dest instanceof Map ||
    dest instanceof WeakMap ||*/
    typeof dest.set == 'function' && dest.set.length >= 2 ? (k, v) => dest.set(k, v) : Util.isArray(dest) ? (k, v) => dest.push([k, v]) : (k, v) => (dest[k] = v);
  let fn;
  fn = function(key, value) {
    insert(key, value);
    next(key, value);
    return fn;
  };
  fn.dest = dest;
  fn.insert = insert;
  return fn;
};

Util.keyIterator = obj => {
  let it;
  if(typeof obj.keys == 'function' && Util.isIterator((it = obj.keys()))) {
    return it;
  } else if(Util.isArray(obj)) {
    return Array.prototype.keys.call(obj);
  } else if('length' in obj) {
    return Array.prototype[Symbol.iterator].call(obj);
  }
};

Util.entryIterator = obj => {
  let it;
  if(typeof obj.entries == 'function' && Util.isIterator((it = obj.entries()))) {
    return it;
  } else if(Util.isArray(obj)) {
    return Array.prototype.entries.call(obj);
  } else if('length' in obj) {
    return (function* () {
      for(let key of Array.prototype[Symbol.iterator].call(obj)) yield [key, obj[key]];
    })();
  }
};

/*Util.bitIterator = function BitIterator(source, inBits, outBits) {
  const iterator = this instanceof BitIterator ? this : Object.create(BitIterator.prototype);

  iterator.bits = [0];
  iterator.size = 0;
  iterator.next = function(bitsWanted = outBits) {
    let output = { bits: [0], size: 0 };
    for(;;) {
      if(iterator.size == 0) fillBits(iterator);
      //console.log("iterator.bits =",iterator.bits, " iterator.size =",iterator.size);
      moveBits(iterator, output, bitsWanted);
      if(output.size == bitsWanted) break;
    }
    return output.bits;
  };
  function readBits(buffer, n) {
    n = Math.min(buffer.size, n);
    let size = 0,
      bits = [];
    while(n >= 16) {
      bits.push(buffer.bits.shift());
      buffer.size -= 16; n -= 16;
      size += 16;
    }

    if(n > 0) {
      const mask = (1 << n) - 1;
      bits.push(buffer.bits & mask);
      size += n;
      buffer.bits >>= n;
      buffer.size -= n;
    }
    return [bits, size];
  }
  const pad = '00000000000000000000000000000000';
  function writeBits(buffer, value, size) {
    buffer.bits = [...Util.partition((pad + value.toString(2)).slice(-32), 16)].map(n => parseInt(n, 2)).reverse();

    buffer.size = size;
    console.log("buffer.bits:",buffer.bits,"buffer.size:",buffer.size);
  }
  function moveBits(input, output, len) {
    let [bits, size] = readBits(input, len);
    writeBits(output, bits, size);
  }
  function fillBits(buffer) {
    const value = source();
    writeBits(buffer, value, inBits);
  }
  return iterator;
};
*/
Util.mapAdapter = getSetFunction => {
  let r = {
    get(key) {
      return getSetFunction(key);
    },
    set(key, value) {
      getSetFunction(key, value);
      return this;
    }
  };
  let tmp = getSetFunction();
  if(Util.isIterable(tmp) || Util.isPromise(tmp)) r.keys = () => getSetFunction();

  if(getSetFunction[Symbol.iterator]) r.entries = getSetFunction[Symbol.iterator];
  else {
    let g = getSetFunction();
    if(Util.isIterable(g) || Util.isGenerator(g)) r.entries = () => getSetFunction();
  }

  return Util.mapFunction(r);
};

/**
 * @param Array   forward
 * @param Array   backward
 *
 * component2path,  path2eagle  => component2eagle
 *  eagle2path, path2component =>
 */
Util.mapFunction = map => {
  let fn;
  fn = function(...args) {
    const [key, value] = args;
    switch (args.length) {
      case 0:
        return fn.keys();
      case 1:
        return fn.get(key);
      case 2:
        return fn.set(key, value);
    }
  };

  fn.map = (m => {
    while(Util.isFunction(m) && m.map !== undefined) m = m.map;
    return m;
  })(map);

  if(map instanceof Map || (Util.isObject(map) && typeof map.get == 'function' && typeof map.set == 'function')) {
    fn.set = (key, value) => (map.set(key, value), (k, v) => fn(k, v));
    fn.get = key => map.get(key);
  } else if(map instanceof Cache || (Util.isObject(map) && typeof map.match == 'function' && typeof map.put == 'function')) {
    fn.set = (key, value) => (map.put(key, value), (k, v) => fn(k, v));
    fn.get = key => map.match(key);
  } else if(Util.isObject(map) && typeof map.getItem == 'function' && typeof map.setItem == 'function') {
    fn.set = (key, value) => (map.setItem(key, value), (k, v) => fn(k, v));
    fn.get = key => map.getItem(key);
  } else {
    fn.set = (key, value) => ((map[key] = value), (k, v) => fn(k, v));
    fn.get = key => map[key];
  }

  fn.update = function(key, fn = (k, v) => v) {
    let oldValue = this.get(key);
    let newValue = fn(oldValue, key);
    if(oldValue != newValue) {
      if(newValue === undefined && typeof map.delete == 'function') map.delete(key);
      else this.set(key, newValue);
    }
    return newValue;
  };

  if(typeof map.entries == 'function') {
    fn.entries = function* () {
      for(let [key, value] of map.entries()) yield [key, value];
    };
    fn.values = function* () {
      for(let [key, value] of map.entries()) yield value;
    };
    fn.keys = function* () {
      for(let [key, value] of map.entries()) yield key;
    };
    fn[Symbol.iterator] = fn.entries;
    fn[inspectSymbol] = function() {
      return new Map(this.map(([key, value]) => [Util.isArray(key) ? key.join('.') : key, value]));
    };
  } else if(typeof map.keys == 'function') {
    if(Util.isAsync(map.keys) || Util.isPromise(map.keys())) {
      fn.keys = async () => [...(await map.keys())];

      fn.entries = async () => {
        let r = [];
        for(let key of await fn.keys()) r.push([key, await fn.get(key)]);
        return r;
      };
      fn.values = async () => {
        let r = [];
        for(let key of await fn.keys()) r.push(await fn.get(key));
        return r;
      };
    } else {
      fn.keys = function* () {
        for(let key of map.keys()) yield key;
      };

      fn.entries = function* () {
        for(let key of fn.keys()) yield [key, fn(key)];
      };
      fn.values = function* () {
        for(let key of fn.keys()) yield fn(key);
      };
    }
  }

  if(typeof fn.entries == 'function') {
    fn.filter = function(pred) {
      return Util.mapFunction(
        new Map(
          (function* () {
            let i = 0;
            for(let [key, value] of fn.entries()) if(pred([key, value], i++)) yield [key, value];
          })()
        )
      );
    };
    fn.map = function(t) {
      return Util.mapFunction(
        new Map(
          (function* () {
            let i = 0;

            for(let [key, value] of fn.entries()) yield t([key, value], i++);
          })()
        )
      );
    };
    fn.forEach = function(fn) {
      let i = 0;

      for(let [key, value] of this.entries()) fn([key, value], i++);
    };
  }
  if(typeof map.delete == 'function') fn.delete = key => map.delete(key);

  if(typeof map.has == 'function') fn.has = key => map.has(key);
  return fn;
};

Util.mapWrapper = (map, toKey = key => key, fromKey = key => key) => {
  let fn = Util.mapFunction(map);
  fn.set = (key, value) => (map.set(toKey(key), value), (k, v) => fn(k, v));
  fn.get = key => map.get(toKey(key));
  if(typeof map.keys == 'function') fn.keys = () => [...map.keys()].map(fromKey);
  if(typeof map.entries == 'function')
    fn.entries = function* () {
      for(let [key, value] of map.entries()) yield [fromKey(key), value];
    };
  if(typeof map.values == 'function')
    fn.values = function* () {
      for(let value of map.values()) yield value;
    };
  if(typeof map.has == 'function') fn.has = key => map.has(toKey(key));
  if(typeof map.delete == 'function') fn.delete = key => map.delete(toKey(key));

  fn.map = (m => {
    while(Util.isFunction(m) && m.map !== undefined) m = m.map;
    return m;
  })(map);

  return fn;
};

/**
 * @param Array   forward
 * @param Array   backward
 *
 * component2path,  path2eagle  => component2eagle
 *  eagle2path, path2component =>
 */
Util.mapCombinator = (forward, backward) => {
  let fn;
  fn = function(key, value) {
    if(value === undefined) return fn.get(key);
    return fn.set(key, value);
  };

  /* prettier-ignore */
  fn.get=  forward.reduceRight((a,m) => makeGetter(m, key => a(key)), a => a);
  return fn;
  function makeGetter(map, next = a => a) {
    return key => (false && console.log('getter', { map, key }), next(map.get(key)));
  }
};

Util.predicate = (fn_or_regex, pred) => {
  let fn = fn_or_regex;
  if(typeof fn_or_regex == 'string') fn_or_regex = new RegExp('^' + fn_or_regex + '$');
  if(fn_or_regex instanceof RegExp) {
    fn = arg => fn_or_regex.test(arg + '');
    fn.valueOf = function() {
      return fn_or_regex;
    };
  }
  if(typeof pred == 'function') return arg => pred(arg, fn);
  return fn;
};
Util.some = predicates => {
  predicates = predicates.map(Util.predicate);
  return value => predicates.some(pred => pred(value));
};
Util.every = predicates => {
  predicates = predicates.map(Util.predicate);
  return value => predicates.every(pred => pred(value));
};

Util.iterateMembers = function* (obj, predicate = (name, depth, obj, proto) => true, depth = 0) {
  let names = [];
  let pred = Util.predicate(predicate);
  const proto = Object.getPrototypeOf(obj);

  /* for(let name in obj) if(pred(name, depth, obj)) yield name;
   */
  let descriptors = Object.getOwnPropertyDescriptors(obj);
  for(let name in descriptors) {
    const { value, get, set, enumerable, configurable, writable } = descriptors[name];

    if(typeof get == 'function') continue;

    if(pred(name, depth, obj)) yield name;
  }
  //for(let symbol of Object.getOwnPropertySymbols(obj)) if(pred(symbol, depth, obj)) yield symbol;
  if(proto) yield* Util.iterateMembers(proto, predicate, depth + 1);
};

Util.and =
  (...predicates) =>
  (...args) =>
    predicates.every(pred => pred(...args));
Util.or =
  (...predicates) =>
  (...args) =>
    predicates.some(pred => pred(...args));

Util.members = Util.curry((pred, obj) => Util.unique([...Util.iterateMembers(obj, Util.tryPredicate(pred))]));

Util.memberNameFilter = (depth = 1, start = 0) =>
  Util.and(
    (m, l, o) => start <= l && l < depth + start,
    (m, l, o) => typeof m != 'string' || ['caller', 'callee', 'constructor', 'arguments'].indexOf(m) == -1,
    (name, depth, obj, proto) => obj != Object.prototype
  );

Util.getMemberNames = (obj, ...args) => {
  let filters = [];
  let depth = 1,
    start = 0;
  while(args.length > 0) {
    if(args.length >= 2 && typeof args[0] == 'number') {
      const n = args.splice(0, 2);
      depth = n[0];
      start = n[1];
      continue;
    }
    filters.push(args.shift());
  }
  filters.unshift(Util.memberNameFilter(depth, start));
  return Util.members(Util.and(...filters))(obj);
};
Util.getMemberEntries = (obj, ...args) => Util.getMemberNames(obj, ...args).map(name => [name, obj[name]]);

Util.objectReducer =
  (filterFn, accFn = (a, m, o) => ({ ...a, [m]: o[m] }), accu = {}) =>
  (obj, ...args) =>
    Util.members(filterFn(...args), obj).reduce(
      Util.tryFunction(
        (a, m) => accFn(a, m, obj),
        (r, a, m) => r,
        (r, a) => a
      ),
      accu
    );
Util.incrementer = (incFn = (c, n, self) => (self.count = c + n)) => {
  let self, incr;
  if(typeof incFn == 'number') {
    incr = incFn;
    incFn = (c, n, self) => (self.count = +c + +n * incr);
  }
  const inc = (i, n = 1) => self.incFn.call(self, i || 0, n, self);
  self = function Count(n = 1) {
    self.count = inc(self.count, n, self);
    return self;
  };
  self.incFn = incFn;
  self.valueOf = function() {
    return this.count;
  };
  return self;
};

Util.mapReducer = (setFn, filterFn = (key, value) => true, mapObj = new Map()) => {
  setFn = setFn || Util.setter(mapObj);
  let fn;
  let next = Util.tryFunction(((acc, mem, idx) => (filterFn(mem, idx) ? (setFn(idx, mem), acc) : null), r => r, () => mapObj));
  fn = function ReduceIntoMap(arg, acc = mapObj) {
    if(Util.isObject(arg) && typeof arg.reduce == 'function') return arg.reduce((acc, arg) => (Util.isArray(arg) ? arg : Util.members(arg)).reduce(reducer, acc), self.map);
    let c = Util.counter();
    for(let mem of arg) acc = next(acc, mem, c());
    return acc;
  };
  return Object.assign(fn, { setFn, filterFn, mapObj, next });
};

Util.getMembers = Util.objectReducer(Util.memberNameFilter);

Util.getMemberDescriptors = Util.objectReducer(Util.memberNameFilter, (a, m, o) => ({
  ...a,
  [m]: Object.getOwnPropertyDescriptor(o, m)
}));

Util.methodNameFilter = (depth = 1, start = 0) =>
  Util.and(
    (m, l, o) =>
      Util.tryCatch(
        () => typeof o[m] == 'function',
        b => b,
        () => false
      ),
    Util.memberNameFilter(depth, start)
  );

Util.getMethodNames = (obj, depth = 1, start = 0) => Util.members(Util.methodNameFilter(depth, start))(obj);

Util.getMethods = Util.objectReducer(Util.methodNameFilter);

Util.getMethodDescriptors = Util.objectReducer(Util.methodNameFilter, (a, m, o) => ({
  ...a,
  [m]: Object.getOwnPropertyDescriptor(o, m)
}));

Util.inherit = (dst, src, depth = 1) => {
  for(let k of Util.getMethodNames(src, depth)) dst[k] = src[k];
  return dst;
};
Util.inherits =
  typeof Object.create === 'function'
    ? function inherits(ctor, superCtor) {
        if(superCtor) {
          ctor.super_ = superCtor;
          ctor.prototype = Object.create(superCtor.prototype, {
            constructor: {
              value: ctor,
              enumerable: false,
              writable: true,
              configurable: true
            }
          });
        }
      } // old school shim for old browsers
    : function inherits(ctor, superCtor) {
        if(superCtor) {
          ctor.super_ = superCtor;
          let TempCtor = function() {};
          TempCtor.prototype = superCtor.prototype;
          ctor.prototype = new TempCtor();
          ctor.prototype.constructor = ctor;
        }
      };
//Util.bindMethods = (obj, methods, dest = {}) => Util.bindMethodsTo(obj, methods ?? obj, dest);
Util.bindMethods = (obj, methods, dest) => {
  dest ??= obj;
  if(Util.isArray(methods)) {
    for(let name of methods) if(typeof obj[name] == 'function') dest[name] = obj[name].bind(obj);
    return dest;
  }
  let names = Util.getMethodNames(methods);
  for(let name of names) if(typeof methods[name] == 'function') dest[name] = methods[name].bind(obj);
  return dest;
};
Util.getConstructor = obj => obj.constructor || Object.getPrototypeOf(obj).constructor;
Util.getPrototypeChain = function(obj, fn = p => p) {
  let ret = [];
  let proto;
  do {
    proto = obj.__proto__ || Object.getPrototypeOf(obj);
    ret.push(fn(proto, obj));
    if(proto === Object.prototype || proto.constructor === Object) break;
    obj = proto;
  } while(obj);

  return ret;
};
Util.getObjectChain = (obj, fn = p => p) => [fn(obj)].concat(Util.getPrototypeChain(obj, fn));

Util.getPropertyDescriptors = function(obj) {
  return Util.getObjectChain(obj, p => Object.getOwnPropertyDescriptors(p));
};

Util.getConstructorChain = (ctor, fn = (c, p) => c) => Util.getPrototypeChain(ctor, (p, o) => fn(o, p));

Util.weakAssign = function(...args) {
  let obj = args.shift();
  args.forEach(other => {
    for(let key in other) {
      if(obj[key] === undefined && other[key] !== undefined) obj[key] = other[key];
    }
  });
  return obj;
};

/*Util.getErrorStack = function(position = 2) {
  let stack=[];
  let error;
    Error.stackTraceLimit = 100;
     const oldPrepareStackTrace = Error.prepareStackTrace;
  Error.prepareStackTrace = (_, stack) => stack;
 try {

  throw new Error('my error');

 } catch(e) {
  console.log("e.stack",[...e.stack]);
  stack = e.stack;
 }
 Error.prepareStackTrace = oldPrepareStackTrace;

 return stack;
}*/
Util.exception = function Exception(...args) {
  let e, stack;
  let proto = Util.exception.prototype;

  if(args[0] instanceof Error) {
    let exc = args.shift();
    const { message, stack: callerStack } = exc;
    e = { message };
    //   e.proto = Object.getPrototypeOf(exc);

    if(callerStack) stack = callerStack;
  } else {
    const [message, callerStack] = args;
    e = { message };
    if(callerStack) stack = callerStack;
  }
  if(stack) e.stack = Util.stack(stack);

  return Object.setPrototypeOf(e, proto);
};

Util.define(
  Util.exception.prototype,
  {
    toString(color = false) {
      const { message, stack, proto } = this;
      return `${Util.fnName((proto && proto.constructor) || this.constructor)}: ${message}
Stack:${Util.stack.prototype.toString.call(stack, color, stack.columnWidths)}`;
    },
    [Symbol.toStringTag]() {
      return this.toString(false);
    },
    [inspectSymbol]() {
      return Util.exception.prototype.toString.call(this, true);
    }
  },
  true
);
Util.location = function Location(...args) {
  console.log('Util.location(', ...args, ')');
  let ret = this instanceof Util.location ? this : Object.setPrototypeOf({}, Util.location.prototype);
  if(args.length == 3) {
    const [fileName, lineNumber, columnNumber, functionName] = args;
    Object.assign(ret, { fileName, lineNumber, columnNumber, functionName });
  } else if(args.length == 1 && args[0].fileName !== undefined) {
    const { fileName, lineNumber, columnNumber, functionName } = args.shift();
    Object.assign(ret, { fileName, lineNumber, columnNumber, functionName });
  }
  if(Util.colorCtor) ret.colorCtor = Util.colorCtor;

  return ret;
};

// prettier-ignore
Util.location.palettes = [[[128, 128, 0], [255, 0, 255], [0, 255, 255] ], [[9, 119, 18], [139, 0, 255], [0, 165, 255]]];

Util.define(Util.location.prototype, {
  toString(color = false) {
    let { fileName, lineNumber, columnNumber, functionName } = this;
    console.log('this:', this, {
      fileName,
      lineNumber,
      columnNumber,
      functionName
    });
    fileName = fileName.replace(Util.makeURL({ location: '' }), '');
    let text = /*color ? new this.colorCtor() : */ '';
    const c = /*color ? (t, color) => text.write(t, color) :*/ t => (text += t);
    const palette = Util.location.palettes[Util.isBrowser() ? 1 : 0];
    if(functionName) c(functionName.replace(/\s*\[.*/g, '').replace(/^Function\./, '') + ' ', palette[1]);

    c(fileName, palette[0]);
    c(':', palette[1]);
    c(lineNumber, palette[2]);
    c(':', palette[1]);
    c(columnNumber, palette[2]);
    return text;
  },
  [Symbol.toStringTag]() {
    return Util.location.prototype.toString.call(this, false);
  },
  [inspectSymbol]() {
    return Util.location.prototype.toString.call(this, !Util.isBrowser());
  },
  getFileName() {
    return this.fileName;
  },
  getLineNumber() {
    return this.lineNumber;
  },
  getColumnNumber() {
    return this.columnNumber;
  }
});

Util.stackFrame = function StackFrame(frame) {
  //   console.debug('Util.stackFrame', frame, frame.getFunctionName, frame.getFileName);
  ['methodName', 'functionName', 'fileName', 'lineNumber', 'columnNumber', 'typeName', 'thisObj'].forEach(prop => {
    let fn = prop == 'thisObj' ? 'getThis' : 'get' + Util.ucfirst(prop);
    if(frame[prop] === undefined && typeof frame[fn] == 'function') frame[prop] = frame[fn]();
  });
  if(Util.colorCtor) frame.colorCtor = Util.colorCtor;

  return Object.setPrototypeOf(frame, Util.stackFrame.prototype);
};

Util.define(Util.stackFrame, {
  methodNames: [
    'getThis',
    'getTypeName',
    'getFunction',
    'getFunctionName',
    'getMethodName',
    'getFileName',
    'getLineNumber',
    'getColumnNumber',
    'getEvalOrigin',
    'isToplevel',
    'isEval',
    'isNative',
    'isConstructor',
    'isAsync',
    'isPromiseAll',
    'getPromiseIndex'
  ]
});
Util.memoizedProperties(Util.stackFrame, {
  propertyMap() {
    return this.methodNames.map(method => [method, Util.lcfirst(method.replace(/^get/, ''))]).map(([method, func]) => [method, func == 'this' ? 'thisObj' : func]);
  }
});

Util.define(
  Util.stackFrame.prototype,
  {
    getFunction() {
      if(this.isConstructor) return this.functionName + '.constructor';

      return this.typeName ? `${this.typeName}.${this.methodName}` : this.functionName;
    },
    getMethodName() {
      return this.methodName;
    },
    getFunctionName() {
      return this.functionName;
    },
    getTypeName() {
      return this.typeName;
    },
    getFileName() {
      return this.fileName;
    },
    getLineNumber() {
      return this.lineNumber;
    },
    getColumnNumber() {
      return this.columnNumber;
    }
  },
  true
);

Util.define(
  Util.stackFrame.prototype,
  {
    colorCtor: null,
    get() {
      const { fileName, columnNumber, lineNumber } = this;
      return fileName ? `${fileName}:${lineNumber}:${columnNumber}` : null;
    },
    toString(color, opts = {}) {
      const { columnWidths = [0, 0, 0, 0], stripUrl } = opts;

      let text = color && this.colorCtor ? new this.colorCtor() : '';
      const c = color && this.colorCtor ? (t, color) => text.write(t, color) : t => (text += t);
      let fields = ['functionName', 'fileName', 'lineNumber', 'columnNumber'];
      const colors = [
        [0, 255, 0],
        [255, 255, 0],
        [0, 255, 255],
        [0, 255, 255]
      ];
      let { functionName, methodName, typeName, fileName, lineNumber, columnNumber } = this;
      //  console.log('toString:', { functionName, methodName, typeName, fileName, lineNumber, columnNumber });
      if(stripUrl && typeof fileName == 'string') fileName = fileName.replace(typeof stripUrl == 'string' ? stripUrl : /.*:\/\/[^\/]*\//, '');
      let colonList = [fileName, lineNumber, columnNumber]
        .map(p => ('' + p == 'undefined' ? undefined : p))
        .filter(p => p !== undefined && p != 'undefined' && ['number', 'string'].indexOf(typeof p) != -1)
        .join(':');
      let columns = [typeof this.getFunction == 'function' ? this.getFunction() : this.function, colonList];
      columns = columns.map((f, i) => (f + '')[i >= 2 ? 'padStart' : 'padEnd'](columnWidths[i] || 0, ' '));
      return columns.join(' ') + c('', 0);
    },
    getLocation() {
      return new Util.location(this);
    },
    /* prettier-ignore */ get location() {
      return this.getLocation();
    },
    [Symbol.toStringTag]() {
      return this.toString(false);
    },
    [inspectSymbol](...args) {
      return Util.stackFrame.prototype.toString.call(this, true, this.columnWidths);
    }
  },
  true
);
Util.scriptName = () =>
  Util.tryCatch(
    () => Util.getArgs(),
    args => args[0],
    () => Util.getURL()
  );
Util.getFunctionName = () => {
  const frame = Util.getCallerStack(2)[0];
  return frame.getFunctionName() || frame.getMethodName();
};
Util.getFunctionArguments = fn => {
  let head = (fn + '').replace(/(=>|{\n).*/g, '').replace(/^function\s*/, '');
  let args = head.replace(/^\((.*)\)\s*$/g, '$1').split(/,\s*/g);
  return args;
};

Util.scriptDir = () =>
  Util.tryCatch(
    () => Util.scriptName(),
    script => (script + '').replace(new RegExp('\\/[^/]*$', 'g'), ''),
    () => Util.getURL()
  );
Util.stack = function Stack(stack, offset) {
  //console.log('Util.stack (1)', stack);

  if(typeof stack == 'number') return Object.setPrototypeOf(new Array(stack), Util.stack.prototype);

  if(Util.platform == 'quickjs') {
    if(!stack) stack = getStack();
    if(!(typeof stack == 'string')) stack = stack + '';
  } else if(!stack) {
    if(offset === undefined) offset = 1;
    stack = getStack();
    const { propertyMap } = Util.stackFrame;
    //console.log('stack', stack + '');
    stack = [...stack].map(frame =>
      propertyMap
        .filter(([m, p]) => typeof frame[m] == 'function' && frame[m]() !== undefined)
        .reduce(
          (acc, [method, property]) => ({
            ...acc,
            /* prettier-ignore */ get [property]() {
              return frame[method]();
            }
          }),
          {}
        )
    );

    //console.debug('stack ctor:', [...stack]);
    //console.debug('stack frame[0]:', [...stack][0]);
  } else if(!(typeof stack == 'string')) stack = stackToString(stack, 0);
  function getStack() {
    let stack;
    const oldPrepareStackTrace = Error.prepareStackTrace;
    Error.prepareStackTrace = (_, stack) => stack;
    Error.stackTraceLimit = Infinity;

    stack = new Error().stack;
    Error.prepareStackTrace = oldPrepareStackTrace;
    return stack;
  }

  function stackToString(st, start = 0) {
    if(Util.isArray(st)) {
      st = [
        ...(function* () {
          for(let i = start; i < st.length; i++) yield st[i];
        })()
      ].join('\n');
    }
    return st;
  }

  //console.log('stack String:', offset, typeof stack, stack);

  if(typeof stack == 'number') {
    throw new Error();
  }
  //console.debug('stack:', typeof stack, stack);

  if(typeof stack == 'string') {
    stack = stack.trim().split(lineSplit);
    const re = new RegExp('.* at ([^ ][^ ]*) \\(([^)]*)\\)');
    stack = stack.map(frame =>
      typeof frame == 'string'
        ? frame
            .replace(/^\s*at\s+/, '')
            .split(/[()]+/g)
            .map(part => part.trim())
        : frame
    );
    stack = stack.map(frame => (Util.isArray(frame) ? (frame.length < 2 ? ['', ...frame] : frame).slice(0, 2) : frame));
    stack = stack.map(([func, file]) => [
      func,
      file
        .split(/:/g)
        .reverse()
        .map(n => (!isNaN(+n) ? +n : n))
    ]);
    stack = stack.map(([func, file]) => [func, file.length >= 3 ? file : file.length >= 2 ? ['', ...file] : ['', '', ...file]]);
    stack = stack.map(([func, [columnNumber, lineNumber, ...file]]) => ({
      functionName: func.replace(/Function\.Util/, 'Util'),
      methodName: func.replace(/.*\./, ''),
      fileName: file.reverse().join(':'),
      lineNumber,
      columnNumber
    }));
    //    console.log('Util.stack (2)', Util.inspect(stack[0]  ));

    stack = stack.map(({ methodName, functionName: func, fileName: file, columnNumber: column, lineNumber: line }) => ({
      functionName: func,
      methodName,
      fileName: file.replace(/.*:\/\/[^\/]*/g, ''),
      lineNumber: Util.ifThenElse(
        s => s != '',
        s => +s,
        () => undefined
      )(line + file.replace(/.*[^0-9]([0-9]*)$/g, '$1')),
      columnNumber: Util.ifThenElse(
        s => s != '',
        s => +s,
        () => undefined
      )(column)
    }));
  } else {
    //console.log('stack:', stack[0]);
    stack = stack.map(frame => new Util.stackFrame(frame)); //Util.getCallers(1, Number.MAX_SAFE_INTEGER, () => true, stack);
  }
  //  stack = stack.map(frame => Object.setPrototypeOf(frame, Util.stackFrame.prototype));
  stack = stack.map(frame => new Util.stackFrame(frame));

  if(offset > 0) stack = stack.slice(offset);
  stack = Object.setPrototypeOf(stack, Util.stack.prototype);
  //stack.forEach(frame => console.log("stack frame:",frame));
  //
  return stack;
};

Util.stack.prototype = Object.assign(Util.stack.prototype, Util.getMethods(new Array(), 1, 1));
Object.defineProperty(Util.stack, Symbol.species, { get: () => Util.stack });
Object.defineProperty(Util.stack.prototype, Symbol.species, {
  get: () => Util.stack
});
Object.defineProperty(Util.stack.prototype, Symbol.iterator, {
  *value() {
    for(let i = 0; i < this.length; i++) yield this[i];
  }
});

Util.stack.prototype = Object.assign(Util.stack.prototype, {
  toString(opts = {}) {
    const { colors = false, stripUrl = Util.makeURL({ location: '/' }) } = opts;
    const { columnWidths } = this;
    let a = [];

    for(let i = 0; i < this.length; i++)
      a.push(
        Util.stackFrame.prototype.toString.call(this[i], colors, {
          columnWidths,
          stripUrl
        })
      );
    let s = a.join('\n');
    return s + '\n';
  },
  [Symbol.toStringTag]() {
    return Util.stack.prototype.toString.call(this);
  },
  [inspectSymbol](...args) {
    const { columnWidths } = this;
    return '\n' + this.map(f => f.toString(!Util.isBrowser(), { columnWidths })).join('\n');
  },
  getFunctionName() {
    return this.functionName;
  },
  getMethodName() {
    return this.methodName;
  },
  getFileName() {
    return this.fileName;
  },
  getLineNumber() {
    return this.lineNumber;
  }
});

Object.defineProperties(Util.stack.prototype, {
  columnWidths: {
    get() {
      // console.log('this:', [...this]);
      return this.reduce((a, f) => ['getFunction'].map((fn, i) => Math.max(a[i], ((typeof f[fn] == 'function' ? f[fn]() : '') + '').length)), [0, 0, 0, 0]);
    }
  }
});

Util.getCallerStack = function(position = 2, limit = 1000, stack) {
  Error.stackTraceLimit = position + limit;
  if(position >= Error.stackTraceLimit) {
    throw new TypeError(`getCallerFile(position) requires position be less then Error.stackTraceLimit but position was: '${position}' and Error.stackTraceLimit was: '${Error.stackTraceLimit}'`);
  }
  const oldPrepareStackTrace = Error.prepareStackTrace;
  Error.prepareStackTrace = (_, stack) => stack;

  stack = Util.stack(stack, position);

  return stack.slice(0, limit);
};
Util.getCallerFile = function(position = 2) {
  let stack = Util.getCallerStack();
  if(stack !== null && typeof stack === 'object') {
    const frame = stack[position];
    return frame ? `${frame.getFileName()}:${frame.getLineNumber()}` : undefined;
  }
};
Util.getCallerFunction = function(position = 2) {
  let stack = Util.getCallerStack(position + 1);
  if(stack !== null && typeof stack === 'object') {
    const frame = stack[0];
    return frame ? frame.getFunction() : undefined;
  }
};
Util.getCallerFunctionName = function(position = 2) {
  let stack = Util.getCallerStack(position + 1);
  if(stack !== null && typeof stack === 'object') {
    const frame = stack[0];
    return frame ? frame.getMethodName() || frame.getFunctionName() : undefined;
  }
};
Util.getCallerFunctionNames = function(position = 2) {
  let stack = Util.getCallerStack(position + 1);
  if(stack !== null && typeof stack === 'object') {
    let ret = [];
    for(let i = 0; stack[i]; i++) {
      const frame = stack[i];
      const method = frame.getMethodName();
      ret.push(method ? frame.getFunction() + '.' + method : frame.getFunctionName());
    }
    return ret;
  }
};
Util.getCaller = function(index = 1, stack) {
  const methods = [
    'getThis',
    'getTypeName',
    'getFunction',
    'getFunctionName',
    'getMethodName',
    'getFileName',
    'getLineNumber',
    'getColumnNumber',
    'getEvalOrigin',
    'isToplevel',
    'isEval',
    'isNative',
    'isConstructor'
  ];
  stack = stack || Util.getCallerStack(2, 1 + index, stack);
  let thisIndex = stack.findIndex(f => f.functionName.endsWith('getCaller'));
  index += thisIndex + 1;
  const frame = stack[index];
  return frame;
};
Util.getCallers = function(index = 1, num = Number.MAX_SAFE_INTEGER, stack) {
  const methods = [
    'getThis',
    'getTypeName',
    'getFunction',
    'getFunctionName',
    'getMethodName',
    'getFileName',
    'getLineNumber',
    'getColumnNumber',
    'getEvalOrigin',
    'isToplevel',
    'isEval',
    'isNative',
    'isConstructor'
  ];
  stack = stack || Util.getCallerStack(2, num + index, stack);
  let thisIndex = stack.findIndex(f => ((f.functionName || f.methodName) + '').endsWith('getCaller'));
  index += thisIndex + 1;
  return stack.slice(index);
};

/*Object.defineProperty(Util, 'stackFrame', {
  get: function() {
  return this.getCallerStack(2);
  }
});*/
Util.getStackFrames = function(offset = 2) {
  let frames = Util.getCallerStack(0);
  frames = frames.map(frame => {
    if(Object.getPrototypeOf(frame) !== Util.stackFrame.prototype) frame = Util.stackFrame(frame);
    return frame;
  });

  return frames.slice(offset);
};
Util.getStackFrame = function(offset = 2) {
  return Util.getStackFrames(offset)[0];
};
Util.rotateLeft = function(x, n) {
  n = n & 0x1f;
  return (x << n) | ((x >> (32 - n)) & ~((-1 >> n) << n));
};
Util.rotateRight = function(x, n) {
  n = n & 0x1f;
  return Util.rotateLeft(x, 32 - n);
};
Util.hashString = function(string, bits = 32, mask = 0xffffffff) {
  let ret = 0;
  let bitc = 0;
  for(let i = 0; i < string.length; i++) {
    const code = string.charCodeAt(i);
    ret *= 186;
    ret ^= code;
    bitc += 8;
    ret = Util.rotateLeft(ret, 7) & mask;
  }
  return ret & 0x7fffffff;
};
Util.flatTree = function(tree, addOutput) {
  const ret = [];
  if(!addOutput) addOutput = arg => ret.push(arg);
  addOutput(Util.filterKeys(tree, key => key !== 'children'));
  if(typeof tree.children == 'object' && tree.children !== null && tree.children.length) for(let child of tree.children) Util.flatTree(child, addOutput);
  return ret;
};
Util.traverseTree = function(tree, fn, depth = 0, parent = null) {
  fn(tree, depth, parent);
  if(Util.isObject(tree.children) && tree.children.length > 0) for(let child of tree.children) Util.traverseTree(child, fn, depth + 1, tree);
};

Util.walkTree = function(node, pred, t, depth = 0, parent = null) {
  return (function* () {
    if(!pred) pred = i => true;
    if(!t)
      t = function(i) {
        i.depth = depth;
        return i;
      };
    if(pred(node, depth, parent)) {
      yield t(node);
      if(typeof node == 'object' && node !== null && typeof node.children == 'object' && node.children.length) {
        for(let child of [...node.children]) {
          yield* Util.walkTree(child, pred, t, depth + 1, node.parent_id);
        }
      }
    }
  })();
};

Util.isPromise = function(obj) {
  return (Boolean(obj) && typeof obj.then === 'function') || obj instanceof Promise;
};

/* eslint-disable no-use-before-define */
if(typeof setImmediate !== 'function') var setImmediate = fn => setTimeout(fn, 0);
Util.next = function(iter, observer, prev = undefined) {
  let item;
  try {
    item = iter.next(prev);
  } catch(err) {
    return observer.error(err);
  }
  const value = item.value;
  if(item.done) return observer.complete();
  if(isPromise(value)) {
    value
      .then(val => {
        observer.next(val);
        setImmediate(() => Util.next(iter, observer, val));
      })
      .catch(err => observer.error(err));
  } else {
    observer.next(value);
    setImmediate(() => Util.next(iter, observer, value));
  }
};
Util.getImageAverageColor = function(imageElement, options) {
  if(!imageElement) {
    return false;
  }
  options = options || {};
  const settings = {
    tooDark: (options.tooDark || 0.03) * 255 * 3 /* How dark is too dark for a pixel */,
    tooLight: (options.tooLight || 0.97) * 255 * 3 /*How light is too light for a pixel */,
    tooAlpha: (options.tooAlpha || 0.1) * 255 /*How transparent is too transparent for a pixel */
  };
  const w = imageElement.width;
  let h = imageElement.height;
  //Setup canvas and draw image onto it
  const context = document.createElement('canvas').getContext('2d');
  context.drawImage(imageElement, 0, 0, w, h);
  //Extract the rgba data for the image from the canvas
  const subpixels = context.getImageData(0, 0, w, h).data;
  const pixels = {
    r: 0,
    g: 0,
    b: 0,
    a: 0
  };
  let processedPixels = 0;
  const pixel = {
    r: 0,
    g: 0,
    b: 0,
    a: 0
  };
  let luma = 0; //Having luma in the pixel object caused ~10% performance penalty for some reason
  //Loop through the rgba data
  for(let i = 0, l = w * h * 4; i < l; i += 4) {
    pixel.r = subpixels[i];
    pixel.g = subpixels[i + 1];
    pixel.b = subpixels[i + 2];
    pixel.a = subpixels[i + 4];
    //Only consider pixels that aren't black, white, or too transparent
    if(
      pixel.a > settings.tooAlpha &&
      (luma = pixel.r + pixel.g + pixel.b) > settings.tooDark && //Luma is assigned inside the conditional to avoid re-calculation when alpha is not met
      luma < settings.tooLight
    ) {
      pixels.r += pixel.r;
      pixels.g += pixel.g;
      pixels.b += pixel.b;
      pixels.a += pixel.a;
      processedPixels++;
    }
  }
  //Values of the channels that make up the average color
  let channels = {
    r: null,
    g: null,
    b: null,
    a: null
  };
  if(processedPixels > 0) {
    channels = {
      r: Math.round(pixels.r / processedPixels),
      g: Math.round(pixels.g / processedPixels),
      b: Math.round(pixels.b / processedPixels),
      a: Math.round(pixels.a / processedPixels)
    };
  }
  const o = Object.assign({}, channels, {
    toStringRgb() {
      //Returns a CSS compatible RGB string (e.g. '255, 255, 255')
      const { r, g, b } = this;
      return [r, g, b].join(', ');
    },
    toStringRgba() {
      //Returns a CSS compatible RGBA string (e.g. '255, 255, 255, 1.0')
      const { r, g, b, a } = this;
      return [r, g, b, a].join(', ');
    },
    toStringHex() {
      //Returns a CSS compatible HEX coloor string (e.g. 'FFA900')
      const toHex = function(d) {
        h = Math.round(d).toString(16);
        if(h.length < 2) {
          h = `0${h}`;
        }
        return h;
      };
      const { r, g, b } = this;
      return [toHex(r), toHex(g), toHex(b)].join('');
    }
  });
  return o;
};
Util.jsonToObject = function(jsonStr) {
  let ret = null;
  try {
    ret = JSON.parse(jsonStr);
  } catch(error) {
    let pos = +('' + error)
      .split('\n')
      .reverse()[0]
      .replace(/.*position\ ([0-9]+).*/, '$1');
    console.error('Unexpected token: ', jsonStr);
    console.error('Unexpected token at:', jsonStr.substring(pos));
    ret = null;
  }
  return ret;
};
Util.splitLines = function(str, max_linelen = Number.MAX_SAFE_INTEGER) {
  const tokens = str.split(/\s/g);
  let lines = [];
  let line = tokens.shift();
  for(; tokens.length; ) {
    if((line.length ? line.length + 1 : 0) + tokens[0].length > max_linelen) {
      lines.push(line);
      line = '';
    }
    if(line != '') line += ' ';
    line += tokens.shift();
  }
  if(line != '') lines.push(line);
  return lines;
};
Util.splitAt = function* (str, ...indexes) {
  let prev = 0;
  for(let index of indexes.sort((a, b) => a - b).concat([str.length])) {
    if(index >= prev) {
      yield str.substring(prev, index);
      if(index >= str.length) break;
      prev = index;
    }
  }
};
Util.decodeEscapes = function(text) {
  let matches = [...Util.matchAll(/([^\\]*)(\\u[0-9a-f]{4}|\\)/gi, text)];
  if(matches.length) {
    matches = matches.map(m => [...m].slice(1)).map(([s, t]) => s + String.fromCodePoint(parseInt(t.substring(2), 16)));
    text = matches.join('');
  }
  return text;
};

Util.stripXML = text =>
  text
    .replace(/<br(|\ *\/)>/gi, '\n')
    .replace(/<[^>]*>/g, '')
    .replace(/[\t\ ]+/g, ' ')
    .replace(/(\n[\t\ ]*)+\n/g, '\n');

Util.stripHTML = html =>
  html
    .replace(/\s*\n\s*/g, ' ')
    .replace(/<[^>]*>/g, '\n')
    .split(lineSplit)
    .map(p => p.trim())
    .filter(p => p != '');

Util.stripNonPrintable = text => text.replace(/[^\x20-\x7f\x0a\x0d\x09]/g, '');
Util.decodeHTMLEntities = function(text) {
  let entities = {
    amp: '&',
    apos: "'",
    '#x27': "'",
    '#x2F': '/',
    '#39': "'",
    '#47': '/',
    lt: '<',
    gt: '>',
    nbsp: ' ',
    quot: '"'
  };
  return text.replace(new RegExp('&([^;]+);', 'gm'), (match, entity) => entities[entity] || match);
};
Util.encodeHTMLEntities = (str, charset = '\u00A0-\u9999<>&') => str.replace(new RegExp(`[${charset}](?!#)`, 'gim'), i => '&#' + i.charCodeAt(0) + ';');

Util.stripAnsi = function(str) {
  return (str + '').replace(new RegExp('\x1b[[(?);]{0,2}(;?[0-9])*.', 'g'), '');
};
Util.proxy = (obj = {}, handler) =>
  new Proxy(obj, {
    get(target, key, receiver) {
      //console.log(`Util.proxy getting ${key}!`);
      return Reflect.get(target, key, receiver);
    },
    set(target, key, value, receiver) {
      //console.log(`Util.proxy setting ${key}!`);
      return Reflect.set(target, key, value, receiver);
    },
    ...handler
  });

Util.propertyLookup = (obj = {}, handler = key => null) =>
  Util.proxy(obj, {
    get(target, key, receiver) {
      return handler(key);
    }
  });

Util.traceProxy = (obj, handler) => {
  let proxy;
  handler = /*handler || */ function(name, args) {
    console.log(`Calling method '${name}':`, ...args);
  };
  //console.log('handler', { handler }, handler + '');
  proxy = new Proxy(obj, {
    get(target, key, receiver) {
      let member = Reflect.get(obj, key, receiver);
      if(0 && typeof member == 'function') {
        let method = member; // member.bind(obj);
        member = function() {
          //          handler.call(receiver, key, arguments);
          return method.apply(obj, arguments);
        };
        member = method.bind(obj);
        console.log('Util.traceProxy', key, (member + '').replace(/\n\s+/g, ' ').split(lineSplit)[0]);
      }
      return member;
    }
  });
  return proxy;
};

Util.proxyTree = function proxyTree(...callbacks) {
  const [setCallback, applyCallback = () => {}] = callbacks;
  const handler = {
    get(target, key) {
      return node([...this.path, key]);
    },
    set(target, key, value) {
      return setCallback(this.path, key, value);
    },
    apply(target, thisArg, args) {
      return applyCallback(this.path, ...args);
    }
  };
  function node(path) {
    return new Proxy(() => {}, { path, ...handler });
  }

  return node([]);
};

/*
 * Calls a constructor with an arbitrary number of arguments.
 *
 * This idea was borrowed from a StackOverflow answer:
 * http://stackoverflow.com/questions/1606797/use-of-apply-with-new-operator-is-this-possible/1608546#1608546
 *
 * And from this MDN doc:
 * https://developer.mozilla.org/en/JavaScript/Reference/Global_Objects/function/apply
 *
 * @param constructor- Constructor to call
 * @param arguments- any number of arguments
 * @return A 'new' instance of the constructor with the arguments passed
 */
Util.construct = constructor => {
  function F(args) {
    return constructor.apply(this, args);
  }

  F.prototype = constructor.prototype;

  // since arguments isn't a first-class array, we'll use a shim
  // Big thanks to Felix Geisendörfer for the idea:
  // http://debuggable.com/posts/turning-javascript-s-arguments-object-into-an-array:4ac50ef8-3bd0-4a2d-8c2e-535ccbdd56cb
  return new F(Array.prototype.slice.call(arguments, 1));
};

/*
 * Calls construct() with a constructor and an array of arguments.
 *
 * @param constructor- Constructor to call
 * @param array- an array of arguments to apply
 * @return A 'new' instance of the constructor with the arguments passed
 */
Util.constructApply = (constructor, array) => {
  let args = [].slice.call(array);
  return construct.apply(null, [constructor].concat(args));
};

Util.immutable = args => {
  const argsType = typeof args === 'object' && Util.isArray(args) ? 'array' : 'object';
  const errorText = argsType === 'array' ? "Error! You can't change elements of this array" : "Error! You can't change properties of this object";
  const handler = {
    set: () => {
      throw new Error(errorText);
    },
    deleteProperty: () => {
      throw new Error(errorText);
    },
    defineProperty: () => {
      throw new Error(errorText);
    }
  };
  return new Proxy(args, handler);
};

Util.immutableClass = (orig, ...proto) => {
  let name = Util.fnName(orig).replace(/Mutable/g, '');
  let imName = 'Immutable' + name;
  proto = proto || [];
  let initialProto = proto.map(p =>
    Util.isArrowFunction(p)
      ? p
      : ctor => {
          for(let n in p) ctor.prototype[n] = p[n];
        }
  );
  let body = `class ${imName} extends ${name} {\n  constructor(...args) {\n    super(...args);\n    if(new.target === ${imName})\n      return Object.freeze(this);\n  }\n};\n\n${imName}.prototype.constructor = ${imName};\n\nreturn ${imName};`;
  for(let p of initialProto) p(orig);
  let ctor; // = new Function(name, body)(orig);

  let imm = base => {
    let cls;
    cls = class extends base {
      constructor(...args) {
        super(...args);
        if(new.target === cls) return Object.freeze(this);
      }
    };
    return cls;
  };
  ctor = imm(orig);

  //console.log('immutableClass', { initialProto, body }, orig);
  let species = ctor;

  /* prettier-ignore */ //Object.assign(ctor, { [Symbol.species]: ctor });

  return ctor;
};

Util.partial = function partial(fn /*, arg1, arg2 etc */) {
  let partialArgs = [].slice.call(arguments, 1);
  if(!partialArgs.length) {
    return fn;
  }
  return function() {
    let args = [].slice.call(arguments);
    let derivedArgs = [];
    for(let i = 0; i < partialArgs.length; i++) {
      let thisPartialArg = partialArgs[i];
      derivedArgs[i] = thisPartialArg === undefined ? args.shift() : thisPartialArg;
    }
    return fn.apply(this, derivedArgs.concat(args));
  };
};

Util.clamp = Util.curry((min, max, value) => Math.max(min, Math.min(max, value)));

Util.coloring = (useColor = true) =>
  !useColor
    ? {
        code(...args) {
          return '';
        },
        text(text) {
          return text;
        },
        concat(...args) {
          let out = args.shift() || [''];
          if(typeof out == 'string') out = [out];
          for(let arg of args) {
            if(Util.isArray(arg)) {
              for(let subarg of arg) out[0] += subarg;
            } else out[0] += arg;
          }
          return out;
        }
      }
    : Util.isBrowser()
    ? {
        palette: [
          'rgb(0,0,0)',
          'rgb(80,0,0)',
          'rgb(0,80,0)',
          'rgb(80,80,0)',
          'rgb(0,0,80)',
          'rgb(80,0,80)',
          'rgb(0,80,80)',
          'rgb(80,80,80)',
          'rgb(0,0,0)',
          'rgb(160,0,0)',
          'rgb(0,160,0)',
          'rgb(160,160,0)',
          'rgb(0,0,160)',
          'rgb(160,0,160)',
          'rgb(0,160,160)',
          'rgb(160,160,160)'
        ],
        /*Util.range(0, 15).map(i =>
            `rgb(${Util.range(0, 2)
              .map(bitno => Util.getBit(i, bitno) * (i & 0x08 ? 160 : 80))
              .join(',')})`
        )*/ code(...args) {
          let css = '';
          let bold = 0;
          for(let arg of args) {
            let c = (arg % 10) + bold;
            let rgb = this.palette[c];
            //console.realLog("code:", {arg, c, rgb});
            if(arg >= 40) css += `background-color:${rgb};`;
            else if(arg >= 30) css += `color:${rgb};`;
            else if(arg == 1) bold = 8;
            else if(arg == 0) bold = 0;
            else throw new Error('No such color code:' + arg);
          }
          css += 'padding: 2px 0 2px 0;';
          return css;
        },
        text(text, ...color) {
          return [`%c${text}`, this.code(...color)];
        },
        concat(...args) {
          let out = args.shift() || [''];
          for(let arg of args) {
            if(Util.isArray(arg) && typeof arg[0] == 'string') out[0] += arg.shift();
            else if(Util.isObject(arg)) {
              out.push(arg);
              continue;
            }

            out = out.concat(arg);
          }
          return out;
        }
      }
    : {
        code(...args) {
          return `\x1b[${[...args].join(';')}m`;
        },
        text(text, ...color) {
          return this.code(...color) + text + this.code(0);
        },
        concat(...args) {
          return args.join('');
        }
      };

let color;
Util.colorText = (...args) => {
  if(!color) color = Util.coloring();
  return color.text(...args);
};
Util.decodeAnsi = (str, index) => {
  let ret = [];
  const len = str.length;
  if(index === undefined) index = str.lastIndexOf('\x1b');
  const isDigit = c => '0123456789'.indexOf(c) != -1;
  const notDigit = c => !isDigit(c);
  const findIndex = (pred, start) => {
    let i;
    for(i = start; i < len; i++) if(pred(str[i])) break;
    return i;
  };
  if(str[++index] == '[') {
    let newIndex;
    for(++index; index < len; index = newIndex) {
      let isNum = isDigit(str[index]);
      newIndex = isNum ? findIndex(notDigit, index) : index + 1;
      if(isNum) {
        let num = parseInt(str.substring(index, newIndex));
        ret.push(num);
      } else {
        ret.push(str[index]);
        break;
      }
      if(str[newIndex] == ';') newIndex++;
    }
  }
  return ret;
};
Util.stripAnsi = str => {
  let o = '';
  for(let i = 0; i < str.length; i++) {
    if(str[i] == '\x1b' && str[i + 1] == '[') {
      while(!/[A-Za-z]/.test(str[i])) i++;
      continue;
    }
    o += str[i];
  }
  return o;
};

Util.ansiCode = (...args) => {
  if(!color) color = Util.coloring();
  return color.code(...args);
};
Util.ansi = Util.coloring(true);
Util.wordWrap = (str, width, delimiter) => {
  // use this on single lines of text only
  if(str.length > width) {
    let p = width;
    for(; p > 0 && str[p] != ' '; p--) {}
    if(p > 0) {
      let left = str.substring(0, p);
      let right = str.substring(p + 1);
      return left + delimiter + Util.wordWrap(right, width, delimiter);
    }
  }
  return str;
};
Util.multiParagraphWordWrap = (str, width, delimiter) => {
  // use this on multi-paragraph lines of xcltext
  let arr = str.split(delimiter);
  for(let i = 0; i < arr.length; i++) if(arr[i].length > width) arr[i] = Util.wordWrap(arr[i], width, delimiter);
  return arr.join(delimiter);
};
Util.defineInspect = (proto, ...props) => {
  if(!Util.isBrowser()) {
    const c = Util.coloring();
    proto[inspectSymbol] = function() {
      const obj = this;
      return (
        c.text(Util.fnName(proto.constructor) + ' ', 1, 31) +
        Util.inspect(
          props.reduce((acc, key) => {
            acc[key] = obj[key];
            return acc;
          }, {}),
          {
            multiline: false,
            colors: true,
            colon: ':',
            spacing: '',
            separator: ', ',
            padding: ' '
          }
        )
      );
    };
  }
};

Util.inRange = Util.curry((a, b, value) => value >= a && value <= b);

Util.bindProperties = (proxy, target, props, gen) => {
  if(props instanceof Array) props = Object.fromEntries(props.map(name => [name, name]));
  const [propMap, propNames] = Util.isArray(props) ? [props.reduce((acc, name) => ({ ...acc, [name]: name }), {}), props] : [props, Object.keys(props)];

  gen ??= p => v => v === undefined ? target[propMap[p]] : (target[propMap[p]] = v);
  const propGetSet = propNames
    .map(k => [k, propMap[k]])

    .reduce(
      (a, [k, v]) => ({
        ...a,
        [k]: Util.isFunction(v) ? (...args) => v.call(target, k, ...args) : (gen && gen(k)) || ((...args) => (args.length > 0 ? (target[k] = args[0]) : target[k]))
      }),
      {}
    );

  /*  console.log(`Util.bindProperties`, { proxy, target, props, gen });*/
  //console.log(`Util.bindProperties`, { propMap, propNames, propGetSet });
  Object.defineProperties(
    proxy,
    propNames.reduce(
      (a, k) => {
        const prop = props[k];
        const get_set = propGetSet[k]; //typeof prop == 'function' ? prop : gen(prop);
        return {
          ...a,
          [k]: {
            get: get_set,
            set: get_set,
            enumerable: true
          }
        };
      },
      {
        __getter_setter__: { get: () => gen, enumerable: false },
        __bound_target__: { get: () => target, enumerable: false }
      }
    )
  );
  return proxy;
};

Util.weakKey = (function () {
  const map = new WeakMap();
  let index = 0;
  return obj => {
    let key = map.get(obj);
    if(!key) {
      key = 'weak-key-' + index++;
      map.set(obj, key);
    }
    return key;
  };
})();

Object.assign(Util.is, {
  array: Util.isArray,
  bool: Util.isBool,
  constructor: Util.isConstructor,
  date: Util.isDate,
  email: Util.isEmail,
  empty: Util.isEmpty,
  nonEmpty: Util.isNonEmpty,
  emptyString: Util.isEmptyString,
  generator: Util.isGenerator,
  iterable: Util.isIterable,
  map: Util.isMap,
  nativeFunction: Util.isNativeFunction,
  object: Util.isObject,
  promise: Util.isPromise,
  function: Util.isFunction,
  string: Util.isString,
  on: val => val == 'on' || val == 'yes' || val === 'true' || val === true,
  off: val => val == 'off' || val == 'no' || val === 'false' || val === false,
  true: val => val === 'true' || val === true,
  false: val => val === 'false' || val === false
});

class AssertionFailed extends Error {
  constructor(message, stack) {
    super(/*'@ ' + location + ': ' +*/ message);
    //this.location = location;
    this.type = 'Assertion failed';

    stack = stack || this.stack;

    this.stack = stack;
  }
}

Util.assert = function assert(val, message) {
  if(typeof val == 'function') {
    message = message || val + '';
    val = val();
  }
  if(!val) throw new AssertionFailed(message || `val == ${val}`);
};
Util.assertEqual = function assertEqual(val1, val2, message) {
  if(val1 != val2) throw new AssertionFailed(message || `${val1} != ${val2}`);
};

Util.assignGlobal = () => Util.weakAssign(Util.getGlobalObject(), Util);

Util.weakMapper = function(createFn, map = new WeakMap(), hitFn) {
  let self = function(obj, ...args) {
    let ret;
    if(map.has(obj)) {
      ret = map.get(obj);
      if(typeof hitFn == 'function') hitFn(obj, ret);
    } else {
      ret = createFn(obj, ...args);
      //if(ret !== undefined)
      map.set(obj, ret);
    }
    return ret;
  };
  self.set = (k, v) => map.set(k, v);
  self.get = k => map.get(k);
  self.map = map;
  return self;
};

Util.merge = function(...args) {
  let ret;
  let isMap = args[0] instanceof Map;
  let t = isMap ? a => new Map(Object.entries(a)) : a => a;

  if(isMap) {
    /*  if(!args.every(arg => Util.isObject(arg) && arg instanceof Map))
    args =args.map(arg => new Map(Util.entries(arg)));
*/
    ret = new Map();

    for(let arg of args) for (let [key, value] of Util.entries(arg)) ret.set(key, value);
  } else {
    ret = args.reduce((acc, arg) => ({ ...acc, ...arg }), {});
  }

  return ret;
};

Util.transformer = (a, ...l) =>
  (l || []).reduce(
    (c, f) =>
      function(...v) {
        return f.apply(this, [c.apply(this, v), ...v]);
      },
    a
  );

/* XXX */ Util.copyTextToClipboard = (i, t) => {
  if(!Util.isBrowser()) {
    return import('./childProcess.js').then(async module => {
      let fs, std;
      let childProcess = await module.PortableChildProcess((a, b, c) => {
        fs = b;
        std = c;
      });
      console.log('childProcess', { childProcess, fs, std });
      let proc = childProcess('xclip', ['-in'], {
        block: false,
        stdio: ['pipe'],
        env: { DISPLAY: Util.getEnv('DISPLAY') }
      });
      console.log('proc.stdin', proc.stdin);

      console.log('write =', await fs.write(proc.stdin, i));
      await fs.close(proc.stdin);
      return await proc.wait();
    });
  }
  let doc = Util.tryCatch(() => document);
  if(!doc) return;
  if(!t) t = doc.body;
  const e = doc.createElement('textarea');
  const prev = doc.activeElement;
  e.value = i + '';
  e.setAttribute('readonly', '');
  e.style.contain = 'strict';
  e.style.position = 'absolute';
  e.style.left = '-9999px';
  e.style.fontSize = '12pt';
  const s = doc.getSelection();
  let orig = false;
  if(s.rangeCount > 0) {
    orig = s.getRangeAt(0);
  }
  t.append(e);
  e.select();
  e.selectionStart = 0;
  e.selectionEnd = i.length;
  let isSuccess = false;
  try {
    isSuccess = doc.execCommand('copy');
  } catch(_) {}
  e.remove();
  if(orig) {
    s.removeAllRanges();
    s.addRange(orig);
  }
  if(prev) {
    prev.focus();
  }
  return isSuccess;
};

Util.toPlainObject = obj => Util.toPlainObjectT(obj, v => (Util.isObject(v) ? Util.toPlainObject(v) : v));

Util.toBuiltinObject = obj => (Array.isArray(obj) ? obj.map(Util.toBuiltinObject) : Util.toPlainObjectT(obj, v => (Util.isObject(v) ? Util.toBuiltinObject(v) : v)));

Util.toPlainObjectT = (obj, t = (v, n) => v) => [...Object.getOwnPropertyNames(obj)].reduce((acc, k) => ({ ...acc, [k]: t(obj[k], k) }), {});

Util.timer = msecs => {
  let ret, id, rej, createdTime, startTime, stopTime, endTime, res, delay, n, timer;
  createdTime = new Date();
  const remaining = () => {
    let r = startTime + msecs - (typeof stopTime == 'number' ? stopTime : new Date());
    return r >= 0 ? r : 0;
  };
  const finish = callback => {
    stopTime = new Date();
    if(stopTime.valueOf() > endTime.valueOf()) stopTime = endTime;
    if(typeof callback == 'function') callback(stopTime);
    res((n = remaining()));
  };
  const log = (method, ...args) =>
    console.log(`${Date.now() - createdTime.valueOf()} timer#${id}.${method}`, ...args.map(obj => Util.toPlainObject(obj || {}, v => v || (v instanceof Date ? `+${v.valueOf() - createdTime}` : v))));
  const timeout = (msecs, tmr = timer) => {
    let now = Date.now();
    if(!startTime) startTime = new Date(now);
    endTime = new Date(now + msecs);
    stopTime = undefined;
    id = setTimeout(() => {
      finish(typeof tmr.callback == 'function' ? (...args) => tmr.callback(...args) : () => {});
      log(`finish`, tmr);
    }, msecs);
    log('start', tmr);
  };
  const add = (arr, ...items) => [...(arr ? arr : []), ...items];

  timer = {
    subscribers: [],
    /* prettier-ignore */ get delay() {
      return delay;
    },
    /* prettier-ignore */ get created() {
      return createdTime;
    },
    /* prettier-ignore */ get start() {
      return startTime || new Date(endTime.valueOf() - delay);
    },
    /* prettier-ignore */ get stop() {
      return stopTime instanceof Date ? stopTime : undefined;
    },
    /* prettier-ignore */ get elapsed() {
      return delay + (stopTime || new Date()).valueOf() - endTime.valueOf();
    },
    /* prettier-ignore */ get end() {
      return endTime;
    },
    /* prettier-ignore */ get remain() {
      return endTime.valueOf() - (stopTime || new Date()).valueOf();
    },
    cancel() {
      log('cancel', this);
      clearTimeout(id);
      finish();
      return this;
    },
    pause() {
      let { remain, pause } = this;
      stopTime = new Date();
      clearTimeout(id);
      this.resume = function() {
        timeout(remain, this);
        this.pause = pause;
        delete this.resume;
        delete this.restart;
        log('resume', this);
        return this;
      };
      this.restart = function() {
        timeout(delay, this);
        this.pause = pause;
        delete this.resume;
        delete this.restart;
        log('restart', this);
        return this;
      };
      delete this.pause;
      log('pause', this);
      return this;
    },
    callback(...args) {
      log('callback', this);
      const { subscribers } = this;
      for(let f of subscribers) f.call(this, ...args);
      return this;
    },
    subscribe(f) {
      const { subscribers } = this;
      if(subscribers.indexOf(f) == -1) subscribers.push(f);
      return this;
    },
    unsubscribe(f) {
      const { subscribers } = this;
      let idx = subscribers.indexOf(f);
      if(idx != -1) subscribers.splice(idx, idx + 1);
      return this;
    }
  };
  const start = () =>
    new Promise((resolve, reject) => {
      res = resolve;
      rej = reject;
      timeout((delay = msecs));
    });
  ret = start();
  return Util.define(ret, timer);
};
/**
 * ???????????''
 * new Promise(Util.thenableReject('ERROR').then)
 *
 * @param      {<type>}  error   The error
 */
Util.thenableReject = error => ({
  then: (resolve, reject) => reject(error)
});
Util.wrapGenerator = fn =>
  Util.isGenerator(fn)
    ? function(...args) {
        return [...fn.call(this, ...args)];
      }
    : fn;

Util.wrapGeneratorMethods = obj => {
  for(let name of Util.getMethodNames(obj, 1, 0)) obj[name] = Util.wrapGenerator(obj[name]);
  return obj;
};

Util.decorateIterable = (proto, generators = false) => {
  const methods = {
    forEach(fn, thisArg) {
      for(let [i, item] of this.entries()) fn.call(thisArg, item, i, this);
    },
    *map(fn, thisArg) {
      for(let [i, item] of this.entries()) yield fn.call(thisArg, item, i, this);
    },
    *filter(pred, thisArg) {
      for(let [i, item] of this.entries()) if(pred.call(thisArg, item, i, this)) yield item;
    },
    findIndex(pred, thisArg) {
      for(let [i, item] of this.entries()) if(pred(item, i, this)) return i;
      return -1;
    },
    indexOf(item, startIndex = -1) {
      return this.findIndex((e, i) => i >= startIndex && e == item);
    },
    find(pred, thisArg) {
      let idx = this.findIndex(pred, thisArg);
      if(idx != -1) return typeof this.item == 'function' ? this.item(idx) : this[idx];
    },
    every(pred, thisArg) {
      for(let [i, item] of this.entries()) if(!pred(item, i++, this)) return false;
      return true;
    },
    some(pred, thisArg) {
      for(let [i, item] of this.entries()) if(pred(item, i, this)) return true;
      return false;
    },
    reduce(fn, accu) {
      for(let [i, item] of this.entries()) accu = fn(accu, item, i, this);
      return accu;
    },
    *entries() {
      let i = 0;
      for(let item of this) yield [i++, item];
    },
    *keys() {
      for(let [i, item] of this.entries()) yield i;
    },
    *values() {
      for(let [i, item] of this.entries()) yield item;
    }
  };
  Util.define(proto, methods, false);
  if(!generators) {
    for(let name in methods) {
      if(typeof name == 'symbol') continue;
      if(name == 'entries') continue;
      let gen = proto[name];
      proto[name] = Util.wrapGenerator(gen);
    }
  }

  return proto;
};

Util.swap = (a, b) => [b, a];
Util.swapArray = ([a, b]) => [b, a];

Util.cacheAdapter = (st, defaultOpts = {}) => {
  if(typeof st == 'string')
    st = Util.tryCatch(
      () => window.caches,
      async c => c.open(st),
      () => null
    );
  return {
    async getItem(request, opts = {}) {
      if(typeof request == 'number') request = await this.key(request);
      return await (await st).match(request, { ...defaultOpts, ...opts });
    },
    async setItem(request, response) {
      return await (await st).put(request, response);
    },
    async addItem(request) {
      await (await st).add(request);
      let response = await this.getItem(request);
      if(response) response = response.clone();
      return response;
    },
    async removeItem(request, opts = {}) {
      if(typeof request == 'number') request = await this.key(request);
      return await (await st).delete(request, { ...defaultOpts, ...opts });
    },
    async key(index) {
      return (await (await st).keys())[index];
    },
    async keys(urls = false, t = a => a) {
      let keys = await (await st).keys();
      if(urls) keys = keys.map(response => response.url);
      if(typeof t == 'function') keys = keys.map(r => t(r));

      return keys;
    },
    async clear() {
      let keys = await (await st).keys();
      for(let key of keys) await this.removeItem(key);
    }
  };
};
Util.cachedFetch = (allOpts = {}) => {
  let { cache = 'fetch', fetch = Util.getGlobalObject('fetch'), debug, print, ...opts } = allOpts;
  const storage = Util.cacheAdapter(cache);
  const baseURL = Util.memoize(() => Util.makeURL({ location: '' }));

  let self = async function CachedFetch(request, opts = {}) {
    let response;
    try {
      if(typeof request == 'string') request = new Request(request, { ...self.defaultOpts, ...opts });

      if(!request.url.startsWith(baseURL())) {
        request = new Request(request.url, { ...request, mode: 'no-cors' });
      }
      response = await storage.getItem(request, {
        ...self.defaultOpts,
        ...opts
      });

      if(response == undefined) {
        response = await /*self.*/ fetch(request, {
          ...self.defaultOpts,
          ...opts
        });

        if(response) {
          let item = response.clone();
          item.time = new Date();
          storage.setItem(request, item);
        }
      } else {
        response.cached = true;
      }
    } catch(err) {
      throw new Error(`CachedFetch: ` + (request.url || request) + ' ' + err.message);
    }
    return response;
  };
  if(debug)
    self = Util.printReturnValue(self, {
      print: print || ((returnValue, fn, ...args) => console.debug(`cachedFetch[${cache}] (`, ...args, `) =`, returnValue))
    });

  Util.define(self, { fetch, cache, storage, opts });
  return self;
};

Util.proxyObject = (root, handler) => {
  const ptr = path => path.reduce((a, i) => a[i], root);
  const nodes = Util.weakMapper(
    (value, path) =>
      new Proxy(value, {
        get(target, key) {
          let prop = value[key];
          if(Util.isObject(prop) || Util.isArray(prop)) return new node([...path, key]);
          return handler && handler.get ? handler.get(prop, key) : prop;
        }
      })
  );
  function node(path) {
    let value = ptr(path);
    //console.log("node:",{path,value});
    return nodes(value, path);
  }
  return node([]);
};
Util.parseXML = function(xmlStr) {
  return Util.tryCatch(
    () => new DOM(),
    parser => parser.parseFromString(xmlStr, 'application/xml')
  );
};

Util.weakAssoc = (fn = (value, ...args) => Object.assign(value, ...args)) => {
  let mapper = Util.tryCatch(
    () => new WeakMap(),
    map => Util.weakMapper((obj, ...args) => Util.merge(...args), map),
    () =>
      (obj, ...args) =>
        Util.define(obj, ...args)
  );
  let self = (obj, ...args) => {
    let value = mapper(obj, ...args);
    return fn(value, ...args);
  };
  self.mapper = mapper;

  return self;
};
Util.getArgv = Util.memoize(() =>
  Util.tryCatch(
    () => {
      let a = process.argv;
      if(!Util.isArray(a)) throw new Error();
      return a;
    },
    a => a,
    () =>
      Util.tryCatch(
        () => thisFilename(),
        fn => [fn],
        () =>
          Util.tryCatch(
            () => scriptArgs,
            a => ['qjs', ...a]
          )
      )
  )
);
Util.getArgs = Util.memoize(() =>
  Util.tryCatch(
    () => {
      let a = process.argv;
      if(!Util.isArray(a)) throw new Error();
      return a;
    },
    a => a.slice(1),
    () => Util.tryCatch(() => scriptArgs)
  )
);
/*  options Object/Map

    option Array [has_arg,callback,val]

*/
Util.getOpt = (options = {}, args) => {
  let short, long;
  let result = {};
  let positional = (result['@'] = []);
  if(!(options instanceof Array)) options = Object.entries(options);
  const findOpt = arg => options.find(([optname, option]) => (Array.isArray(option) ? option.indexOf(arg) != -1 : false) || arg == optname);
  let [, params] = options.find(opt => opt[0] == '@') || [];
  if(typeof params == 'string') params = params.split(',');
  // console.log('Util.getOpt options', options);
  // console.log('Util.getOpt params', params);
  for(let i = 0; i < args.length; i++) {
    const arg = args[i];
    let opt;
    if(arg[0] == '-') {
      let name, value, start, end;
      if(arg[1] == '-') long = true;
      else short = true;
      //console.log('Util.getOpt', { arg, short, long });
      start = short ? 1 : 2;
      if(short) end = 2;
      else if((end = arg.indexOf('=')) == -1) end = arg.length;
      name = arg.substring(start, end);
      //console.log('Util.getOpt', { start, end, name });
      if((opt = findOpt(name))) {
        //console.log('Util.getOpt', { opt });
        const [has_arg, handler] = opt[1];
        if(has_arg) {
          if(arg.length > end) value = arg.substring(end + (arg[end] == '='));
          else value = args[++i];
        } else {
          value = true;
        }
        //console.log('Util.getOpt #1', { name, handler });
        Util.tryCatch(
          () => handler(value, result[opt[0]], options, result),
          v => (value = v),
          () => null
        );
        //console.log('Util.getOpt #2', { name, value, fn: typeof opt[1] + ' ' + opt[1] + '' });
        result[opt[0]] = value;
        continue;
      }
    }
    if(params.length) {
      const param = params.shift();
      // console.log('Util.getOpt', { positional, param });
      if((opt = findOpt(param))) {
        const [, [, handler]] = opt;
        let value = arg;
        //console.log('Util.getOpt #3', { param, handler });
        if(typeof handler == 'function')
          value = Util.tryCatch(
            () => handler(value, result[opt[0]], options, result),
            v => v
          );
        const name = opt[0];
        //console.log('Util.getOpt #4', { name, value });
        result[opt[0]] = value;
        continue;
      }
    }
    result['@'] = [...(result['@'] ?? []), arg];
  }
  //console.log('Util.getOpt', { result });
  return result;
};
Util.getEnv = async varName =>
  Util.tryCatch(
    () => process.env,
    async e => e[varName],
    () => false /* XXX (globalThis.std ? std.getenv(varName) : Util.tryCatch(async () => await import('std').then(std => std.getenv(varName)))) */
  );
Util.getEnvVars = async () =>
  Util.tryCatch(
    () => process.env,
    async e => e,
    () => false
    // XXX     Util.tryCatch(
    //        async () =>
    //          await import('./childProcess.js').then(async ({ PortableChildProcess }) => {
    //            let childProcess = await PortableChildProcess();
    //            (await import('./filesystem.js')).default(fs => (Util.globalThis().filesystem = fs));
    //            let proc = childProcess('env', [], {
    //              block: false,
    //              stdio: [null, 'pipe']
    //            });
    //            let data = '\n';
    //            for await(let output of await filesystem.asyncReader(proc.stdout)) data += filesystem.bufferToString(output);
    //            let matches = [...Util.matchAll(/(^|\n)[A-Za-z_][A-Za-z0-9_]*=.*/gm, data)];
    //            let indexes = matches.map(match => match.index);
    //            let ranges = indexes.reduce((acc, idx, i, a) => [...acc, [idx + 1, a[i + 1]]], []);
    //            let vars = ranges
    //              .map(r => data.substring(...r))
    //              .map(line => {
    //                let eqPos = line.indexOf('=');
    //                return [line.substring(0, eqPos), line.substring(eqPos + 1)];
    //              });
    //            return Object.fromEntries(vars);
    //          })
    //      )
  );

Util.safeFunction = (fn, trapExceptions, thisObj) => {
  const isAsync = Util.isAsync(fn);
  let exec = isAsync
    ? async function(...args) {
        return await fn.call(this || thisObj, ...args);
      }
    : function(...args) {
        return fn.call(this || thisObj, ...args);
      };
  if(trapExceptions) {
    const handleException = typeof trapExceptions == 'function' ? trapExceptions : Util.putError;
    Error.stackTraceLimit = Infinity;
    exec = Util.tryFunction(
      exec, //async (...args) => { Error.stackTraceLimit=Infinity;  return await exec(...args); },
      a => a,
      error => {
        if(Util.isObject(error)) {
          if(error.stack !== undefined) error.stack = new Util.stack(error.stack);
          handleException(error);
        }
      }
    );
  }
  return exec;
};
Util.safeCall = (fn, ...args) => Util.safeApply(fn, args);
Util.safeApply = (fn, args = []) => Util.safeFunction(fn, true)(...args);

Util.exit = exitCode => {
  const { callExitHandlers } = Util;
  //console.log('Util.exit', { exitCode, callExitHandlers });
  if(callExitHandlers) callExitHandlers(exitCode);
  const stdExit = std => {
    std.gc();
    std.exit(exitCode);
  };
  if(globalThis.std) return stdExit(globalThis.std);
  return;
  /* XXX import('std')
    .then(stdExit)
    .catch(() =>*/ Util.tryCatch(
    () => [process, process.exit],
    ([obj, exit]) => exit.call(obj, exitCode),
    () => false
  );
};
Util.atexit = handler => {
  const { handlers } = Util.callMain;
  Util.pushUnique(handlers, handler);
  if(typeof Util.trapExit == 'function') Util.trapExit();
};
Util.callMain = async (fn, trapExceptions) =>
  await Util.safeFunction(
    async (...args) => {
      Util.callMain.handlers = [];
      const { handlers } = Util.callMain;
      const callExitHandlers = (Util.callExitHandlers = Util.once(async ret => {
        if(handlers) for(const handler of handlers) await handler(ret);
        // Util.exit(ret);
      }));
      Util.trapExit = Util.once(() => Util.signal(15, callExitHandlers));
      /* XXX if(Util.getPlatform() == 'quickjs') await import('std').then(module => module.gc()); */
      let ret = await fn(...args);
      await callExitHandlers(ret);
    },
    trapExceptions &&
      (typeof trapExceptions == 'function'
        ? trapExceptions
        : err => {
            let { message, stack } = err;
            stack = new Util.stack(err.stack);
            const scriptDir = Util.tryCatch(
              () => process.argv[1],
              argv1 => argv1.replace(/\/[^\/]*$/g, '')
            );
            console.log('Exception:', message, '\nStack:' + (stack.toString({ colors: true, stripUrl: `file://${scriptDir}/` }) + '').replace(/(^|\n)/g, '\n  '));
            Util.exit(1);
          })
  )(...Util.getArgs().slice(1));

Util.printReturnValue = (fn, opts = {}) => {
  const {
    print = (returnValue, fn, ...args) => {
      let stack = Util.getCallerStack();

      (console.debug || console.log)('RETURN VAL:', /*Util.inspect(*/ returnValue /*, { colors: false })*/, {
        /*fn,
         */ args /*,
        stack*/
      });
    }
  } = opts;
  let self;
  self = (...args) => {
    let returnValue = fn(...args);

    print.call(self, returnValue, fn, ...args);
    return returnValue;
    /*fn = Util.tryFunction(fn, (returnValue, ...args) => {
      print.call(self, returnValue, fn, ...args);
      return returnValue;
    });

    return fn(...args);*/
  };
  Util.define(self, { fn, opts });
  return self;
};
Util.callMain.handlers = [];

Util.replaceAll = (needles, haystack) => {
  return Util.entries(needles)
    .map(([re, str]) => [typeof re == 'string' ? new RegExp(re, 'g') : re, str])
    .reduce((acc, [match, replacement]) => acc.replace(match, replacement), haystack);
};

Util.quote = (str, q = '"') => {
  return q + str.replace(new RegExp(q, 'g'), '\\' + q) + q;
};

Util.escape = (str, pred = codePoint => codePoint < 32 || codePoint > 0xff) => {
  let s = '';
  for(let i = 0; i < str.length; i++) {
    let code = str.codePointAt(i);
    if(!pred(code)) {
      s += str[i];
      continue;
    }

    if(code == 0) s += `\\0`;
    else if(code == 10) s += `\\n`;
    else if(code == 13) s += `\\r`;
    else if(code == 9) s += `\\t`;
    else if(code <= 0xff) s += `\\x${('0' + code.toString(16)).slice(-2)}`;
    else s += `\\u${('0000' + code.toString(16)).slice(-4)}`;
  }
  return s;
};
Util.escapeRegex = string => string.replace(/[-\/\\^$*+?.()|[\]{}]/g, '\\$&');

Util.consolePrinter = function ConsolePrinter(log = console.log) {
  let self;

  self = function(...args) {
    self.add(...args);
    self.print();
    self.clear();
  };

  delete self.length;

  Object.setPrototypeOf(self, Util.extend(Util.consolePrinter.prototype, Util.getMethods(Object.getPrototypeOf(self), 1, 0)));
  self.splice(0, self.length, '');
  self.log = (...args) => log(...args);

  return self;
};
Object.assign(Util.consolePrinter.prototype, Util.getMethods(Array.prototype));

Util.consoleJoin = function(...args) {
  let out = 'push' in this ? this : [];
  if(out.length == 0) out.push('');
  let match = Util.matchAll(/%(?:o|O|d|i|s|f|s|d|c)/g);
  for(let [fmt, ...styles] of args) {
    console.log('Util.consoleJoin', { fmt, styles, out });
    let substs = [...match(fmt)];
    if(substs.length != styles.length) {
      const code = [substs.length, styles.length];
      //console.log("substs:",substs);
      throw new Error(`${code.join(' != ')} ${code.join(', ')}`);
    }
    if(out[0]) out[0] += ' ';
    out[0] += fmt;

    for(let style of styles) Array.prototype.push.call(out, style);
    // Array.prototype.splice.call(out, out.length, 0, ...styles);
    //console.log('Util.consoleJoin', [...out]);
  }
  return out;
};

Util.consoleConcat = function(...args) {
  let self;
  self = function ConsoleConcat(...args) {
    if(args.length == 1 && Array.isArray(args[0])) args = args[0];
    return self.add(...args);
  };
  self.add = Util.consoleJoin;
  /*  function concat(out, args) {
 console.log('concat', { out: [...out], args: [...args] });
   while(args.length) {
      let arg = args.shift();
      if(typeof arg == 'string') {
        let matches = [...Util.matchAll(/%[cos]/g, arg)];
        if(matches.length > 0 && args.length >= matches.length) {
          out[0] += arg;
          out.splice(out.length, 0, ...args.splice(0, matches.length));
        } else {
          out[0] += arg.replace(/%/g, '%%');
        }
      } else if(Util.isArray(arg) && typeof arg[0] == 'string' && /%[cos]/.test(arg[0])) {
        concat(out, arg);
      } else {
        out[0] += ' %o';
        out.push(arg);
      }
    }
    return out;
  }
*/ delete self.length;
  Object.setPrototypeOf(self, Util.extend(Util.consoleConcat.prototype, Object.getPrototypeOf(self)));
  //self.push('');
  if(args.length) self.add(...args);
  return self;
};

Util.consoleConcat.prototype = Object.assign(Util.consoleConcat.prototype, Util.getMethods(Array.prototype, 1, 0), {
  [inspectSymbol]() {
    return [this, [...this]];
  },
  [Symbol.iterator]() {
    return Array.prototype[Symbol.iterator].call(this);
  },
  clear() {
    return this.splice(0, this.length);
  },
  print(log = (...args) => console.info(...args)) {
    log(...this);
  }
});
Util.consolePrinter.prototype.length = 1;
Util.consolePrinter.prototype[0] = '';
Object.assign(Util.consolePrinter.prototype, Util.consoleConcat.prototype, {
  print() {
    const a = [...this];
    const i = a.map(i => Util.inspect(i));
    console.debug('a: ' + i.shift(), ...i);

    Util.consoleConcat.prototype.print.call(this, this.log);
  },
  output() {
    const a = [...this];
    this.clear();
    return a;
  },
  add(...args) {
    let { i = 0 } = this;

    for(; args.length > 0; i++) {
      let arg = args.shift();
      //  console.debug('arg:', i, typeof(arg) == 'string'  ? Util.abbreviate(arg) : arg);

      if(Util.isArray(arg) && /%c/.test(arg[0])) {
        this.i = i;
        this.add(...arg);
        continue;
      }
      if(i > 0) this[0] += ' ';
      if(typeof arg != 'string') {
        this[0] += '%o';
        this.push(arg);
      } else {
        this[0] += arg;
        if(/color:/.test(this[0])) {
          throw new Error(`this[0] is CSS: i=${i}\nthis[0] = "${this[0]}"\narg= ${typeof arg} "${(arg + '').replace(lineSplit, '\\n')}"`);
        }

        const matches = [...Util.matchAll(['%c', '%o'], arg)];
        console.debug('matches.length:', matches.length, ' args.length:', args.length);

        if(matches.length > 0) {
          const styles = args.splice(0, matches.length);
          this.splice(this.length, 0, ...styles);
        }
      }
    }
  }
});

Util.booleanAdapter = (getSetFn, trueValue = 1, falseValue = 0) =>
  function(value) {
    if(value !== undefined) {
      getSetFn(value ? trueValue : falseValue);
    } else {
      let ret = getSetFn();
      if(ret === trueValue) return true;
      if(ret === falseValue) return false;
    }
  };

Util.getSet = (get, set = () => {}, thisObj) =>
  function(...args) {
    if(args.length > 0) return set.call(thisObj || this, ...args);
    return get.call(thisObj || this);
  };

Util.deriveGetSet = (fn, get = v => v, set = v => v, thisObj) =>
  Util.getSet(
    () => get(fn()),
    v => fn(set(v)),
    thisObj
  );
Util.extendFunction = (handler = () => {}) =>
  class ExFunc extends Function {
    constructor() {
      super('...args', 'return this.__self__.__call__(...args)');
      var self = this.bind(this);
      this.__self__ = self;
      return self;
    }

    // Example `__call__` method.
    __call__(...args) {
      return handler(...args);
    }
  };
Util.isatty = async fd => {
  let ret;
  for(let module of ['os', 'tty']) {
    try {
      ret = await import(module).then(mod => mod.isatty(fd));
    } catch(err) {
      ret = undefined;
    }
    if(ret !== undefined) break;
  }
  return ret;
};
Util.ttyGetWinSize = (fd = 1) => {
  let ret;
  if(Util.getPlatform() == 'quickjs') return import('os').then(m => m.ttyGetWinSize(fd));
  const stream = process[['stdin', 'stdout', 'stderr'][fd] || 'stdout'];
  return new Promise(stream.cols ? (resolve, reject) => resolve([stream.cols, stream.rows]) : (resolve, reject) => resolve(stream?.getWindowSize?.()));
};
Util.ttySetRaw = globalThis.os
  ? os.ttySetRaw
  : (fd = 0, mode = true) => {
      let ret;
      const stream = typeof fd == 'number' ? process[['stdin', 'stdout', 'stderr'][fd] || 'stdin'] : fd;
      return stream?.setRawMode?.(mode);
    };
Util.stdio = (fd, mode = true) => {
  if(Util.getPlatform() == 'quickjs') return std[['in', 'out', 'err'][fd]];

  let ret;
  const stream = typeof fd == 'number' ? process[['stdin', 'stdout', 'stderr'][fd] || 'stdin'] : fd;
  return stream?.setRawMode?.(mode);
};

Util.signal = (num, act) => {
  //console.log('Util.signal', { num, act });
  let ret;
  return import('os')
    .then(m => {
      if(typeof num == 'string' && num in m) num = m[num];

      m.signal(num, act);
    })
    .catch(() => process.on(num, act));
};

/**
 * Measure the average execution time of a function
 * @param {Function} fn A function for performance measurement
 * @param {Array} args Function arguments
 * @param {Object} options
 * @returns {Number} Result in milliseconds
 */
Util.timeit = (fn, args = [], options = {}) => {
  const valid = fn && typeof fn === 'function';
  if(!valid) throw new Error('No function provided.');

  const NS_PER_SEC = 1e9;
  const { e, r, l, d } = { e: 1000, r: 1, l: true, d: 6, ...options };
  const { hrtime } = Util;

  let results = [];
  for(let i = 0; i < r; i++) {
    const start = hrtime();
    for(let i = 1; i < e; i++) {
      fn(args);
    }
    const diff = hrtime(start);
    const elapsed = (diff[0] * NS_PER_SEC + diff[1]) * 0.000001;
    const result = elapsed / e;
    results.push(+(Math.round(result + `e+${6}`) + `e-${6}`));
  }
  const ms = results.reduce((p, c) => p + c, 0) / results.length;

  if(l) {
    console.log(`Function   : ${fn.name}()`);
    console.log(`Average    : ${ms.toFixed(d)}ms`);
    console.log(`Repetitions: ${r}`);
    console.log(`Executions : ${e}`);
  }

  return ms;
};

Util.lazyProperty = (obj, name, getter, opts = {}) => {
  const replaceProperty = value => {
    delete obj[name];
    Object.defineProperty(obj, name, { value, ...opts });
    return value;
  };
  const isAsync = Util.isAsync(getter);
  //console.log(`Util.lazyProperty name=${name} isAsync=${isAsync} getter=${getter}`);

  return Object.defineProperty(obj, name, {
    get: isAsync
      ? async function() {
          return replaceProperty(await getter.call(obj, name));
        }
      : function() {
          const value = getter.call(obj, name);
          let isPromise = Util.isObject(value) && value instanceof Promise;
          //console.log(`Util.lazyProperty`, name, value, isPromise);
          if(isPromise) {
            value.then(v => {
              replaceProperty(v);
              //console.log(`Util.lazyProperty resolved `, obj[name]);
              return v;
            });
            return value;
          }
          return replaceProperty(value);
        },
    configurable: true,
    ...opts
  });
};

Util.lazyProperties = (obj, gettersObj, opts = {}) => {
  opts = { enumerable: false, ...opts };
  for(let prop in gettersObj) {
    // console.log('Util.lazyProperties', { prop });
    Util.lazyProperty(obj, prop, gettersObj[prop], opts);
  }
  return obj;
};

Util.calcHRTime = (f = (a, b) => a + b) =>
  function(a, b) {
    const ms = f(a[1], b[1]);
    const div = Math.floor(ms / 1e9);
    const rem = ms % 1e9;

    return [f(a[0], b[0]) + div, rem];
  };
Util.addHRTime = Util.calcHRTime((a, b) => a + b);
Util.subHRTime = Util.calcHRTime((a, b) => a - b);

Util.getHRTime = Util.memoize(() => {
  const { now } = Util;

  class HighResolutionTime extends Array {
    constructor(secs = 0, nano = 0) {
      super(2);
      this[0] = secs;
      this[1] = nano;
      return Object.freeze(this);
    }
    static create(s, n) {
      const sign = Math.sign(s * 1e3 + n * 1e-6);
      s *= sign;
      n *= sign;
      if(n < 0) {
        s--;
        n += 1e9;
      }
      if(n >= 1e9) {
        s++;
        n -= 1e9;
      }
      return new HighResolutionTime(s * sign, n * sign);
    }
    /* prettier-ignore */ get seconds() {
      const [s, n] = this;
      return s + n * 1e-9;
    }
    /* prettier-ignore */ get milliseconds() {
      const [s, n] = this;
      return s * 1e3 + n * 1e-6;
    }
    /* prettier-ignore */ get nanoseconds() {
      const [s, n] = this;
      return s * 1e9 + n;
    }
    [Symbol.toPrimitive]() {
      return this.milliseconds;
    }
    diff(o) {
      let s = o[0] - this[0];
      let n = o[1] - this[1];
      return HighResolutionTime.create(s, n);
    }
    sum(o) {
      /*     let s = o[0] + this[0];
      let n = o[1] + this[1];*/
      return HighResolutionTime.create(...Util.addHRTime(o, this));
    }
    since(o) {
      let s = this[0] - o[0];
      let n = this[1] - o[1];
      return HighResolutionTime.create(s, n);
    }
    toString() {
      let secs = this.seconds;
      let msecs = (secs % 1) * 1e3;
      let nsecs = (msecs % 1) * 1e6;
      let ret = secs >= 1 ? `${Math.floor(secs)}s ` : '';
      return ret + `${Util.roundTo(msecs, 0.001)}ms`;
    }
    inspect() {
      return [this.seconds, this.nanoseconds];
    }
    [inspectSymbol]() {
      return [this.seconds, this.nanoseconds];
      let secs = this.seconds;
      let msecs = (secs % 1) * 1e3;
      let nsecs = (msecs % 1) * 1e6;
      return `${Math.floor(secs)}s ${Util.roundTo(msecs, 0.001)}ms`;
      return `${Math.floor(secs)}s ${Math.floor(msecs)}ms ${Math.floor(nsecs)}ns`;
    }
  }
  Util.getGlobalObject().HighResolutionTime = HighResolutionTime;

  return Util.isAsync(now)
    ? async function hrtime(previousTimestamp) {
        var clocktime = await now();
        var secs = Math.floor(Number(clocktime / 1000));
        var nano = Math.floor(Number(clocktime % 1000) * 1e6);
        let ts = new HighResolutionTime(secs, nano);
        if(previousTimestamp) ts = ts.since(previousTimestamp);
        return ts;
      }
    : function hrtime(previousTimestamp) {
        var clocktime = now();
        var secs = Math.floor(clocktime / 1000);
        var nano = Math.floor((clocktime % 1000) * 1e6);
        let ts = new HighResolutionTime(secs, nano);
        if(previousTimestamp) ts = ts.since(previousTimestamp);
        return ts;
      };
});

Util.lazyProperty(Util, 'animationFrame', () => {
  const { now } = Util;

  return (minDelay = 0) => {
    if(minDelay <= 0) return new Promise(resolve => requestAnimationFrame(resolve));
    const start = now();

    return new Promise(resolve => {
      requestAnimationFrame(animationFrame);

      function animationFrame(t) {
        if(t - start >= minDelay) resolve(t);
        requestAnimationFrame(animationFrame);
      }
    });
  };
});

Util.lazyProperty(Util, 'hrtime', Util.getHRTime);
//Util.startTime = Util.hrtime();

Util.lazyProperty(
  Util,
  'now',
  (Util.getNow = () => {
    const g = Util.getGlobalObject();
    // polyfil for window.performance.now
    var performance = g.performance || {};
    var performanceNow = performance.now || performance.mozNow || performance.msNow || performance.oNow || performance.webkitNow;

    if(performanceNow) {
      //console.log('performanceNow', performanceNow);
      performanceNow = performanceNow.bind(performance); //Util.bind(performanceNow, performance);
    }

    if(!performanceNow && g.cv?.getTickCount) {
      const freq = g.cv.getTickFrequency() / 1000;
      const mul = 1 / freq;
      const getTicks = g.cv.getTickCount;
      performanceNow = () => getTicks() * mul;
    }
    if(!performanceNow && Util.getPlatform() == 'quickjs') {
      let gettime;
      const CLOCK_REALTIME = 0;
      const CLOCK_MONOTONIC = 1;
      const CLOCK_MONOTONIC_RAW = 4;
      const CLOCK_BOOTTIME = 7;

      console.log('STACK:', Util.getCallerStack());

      performanceNow = async function(clock = CLOCK_MONOTONIC_RAW) {
        /* XXX
        if(!gettime) {
          const { dlsym, RTLD_DEFAULT, define, call } = await import('ffi.so');
          const clock_gettime = dlsym(RTLD_DEFAULT, 'clock_gettime');
          define('clock_gettime', clock_gettime, null, 'int', 'int', 'void *');
          gettime = (clk_id, tp) => call('clock_gettime', clk_id, tp);
        }*/
        let data = new ArrayBuffer(16);

        gettime(clock, data);
        let [secs, nsecs] = new BigUint64Array(data, 0, 2);

        let t = /*BigFloat*/ secs * 1e3 + nsecs * 1e-6;
        return t;
      };
    }

    if(!performanceNow) {
      const getTime = Date.now;
      performanceNow = getTime;
    }

    return performanceNow;
  })
);

Util.formatColumns = a => {
  let maxWidth = a.reduce((acc, row, i) => row.map((col, j) => Math.max(acc[j] || 0, (col + '').length)));

  // console.debug(maxWidth);

  return a.map(row => row.map((col, j) => (col + '').padEnd(maxWidth[j])).join(' ')).join('\n');
};

Util.getPlatform = () =>
  Util.tryCatch(
    () => process.versions.node,
    () => 'node',
    Util.tryCatch(
      () => globalThis.scriptArgs[0],
      () => 'quickjs',
      Util.tryCatch(
        () => window.navigator,
        () => 'browser',
        () => undefined
      )
    )
  );

Util.defineGetter(Util, 'platform', Util.memoize(Util.getPlatform));
Util.defineGetter(
  Util,
  'env',
  Util.memoize(async () => {
    let env = await Util.getEnvVars();
    Util.define(Util, 'env', env);
    return env;
  })
);

Util.colIndexes = line => [...line].reduce(([prev, cols], char, i) => [char, [...cols, ...(/\s/.test(prev) && /[^\s]/.test(char) ? [i] : [])]], [' ', []])[1];

Util.colSplit = (line, indexes) => {
  indexes = indexes || Util.colIndexes(line);
  let ret = [];
  for(let i = 0; i < indexes.length; i++) {
    let col = indexes[i];
    let next = indexes[i + 1] || line.length;

    ret.push(line.substring(col, next));
  }
  return ret;
};

Util.bitsToNames = (flags, map = (name, flag) => name) => {
  const entries = [...Util.entries(flags)];

  return function* (value) {
    for(let [name, flag] of entries) if(value & flag && (value & flag) == flag) yield map(name, flag);
  };
};

// time a given function
Util.instrument = (
  fn,
  log = (duration, name, args, ret) => console.log(`function '${name}'` + (ret !== undefined ? ` {= ${Util.abbreviate(Util.escape(ret + ''))}}` : '') + ` timing: ${duration.toFixed(3)}ms`),
  logInterval = 0 //1000
) => {
  const { now, hrtime, functionName } = Util;
  let last = now();
  let duration = 0,
    times = 0;
  const name = functionName(fn) || '<anonymous>';
  const isAsync = Util.isAsync(fn) || Util.isAsync(now);
  const doLog = isAsync
    ? async (args, ret) => {
        let t = await now();
        if(t - (await last) >= logInterval) {
          log(duration / times, name, args, ret);
          duration = times = 0;
          last = t;
        }
      }
    : (args, ret) => {
        let t = now();
        //console.log('doLog', { passed: t - last, logInterval });
        if(t - last >= logInterval) {
          log(duration / times, name, args, ret);
          duration = times = 0;
          last = t;
        }
      };

  return isAsync
    ? async function(...args) {
        const start = await now();
        let ret = await fn.apply(this, args);
        duration += (await now()) - start;
        times++;
        await doLog(args, ret);
        return ret;
      }
    : function(...args) {
        const start = hrtime();
        let ret = fn.apply(this, args);
        duration += now() - start;
        times++;
        doLog(args, ret);
        return ret;
      };
};

Util.trace = (fn, enter, leave, both = () => {}) => {
  enter = enter || ((name, args) => console.log(`function '${name}' (${args.map(arg => inspect(arg)).join(', ')}`));

  leave = leave || ((name, ret) => console.log(`function '${name}'` + (ret !== undefined ? ` {= ${Util.abbreviate(Util.escape(ret + ''))}}` : '')));

  let orig = fn;

  return function(...args) {
    let ret;
    both('enter', fn.name, args);
    enter(fn.name, args);

    ret = orig.call(this, ...args);
    both('leave', fn.name, ret);
    leave(fn.name, ret);
    return ret;
  };
};

Util.bind = function(f, ...args) {
  let ret,
    boundThis = args[0];

  if(args.length < 2)
    ret = function() {
      if(new.target /*this instanceof ret*/) {
        let ret_ = f.apply(this, arguments);
        return Object(ret_) === ret_ ? ret_ : this;
      } else return f.apply(boundThis, arguments);
    };
  else {
    let boundArgs = new Array(args.length - 1);
    for(let i = 1; i < args.length; i++) boundArgs[i - 1] = args[i];

    ret = function() {
      let boundLen = boundArgs.length,
        args = new Array(boundLen + arguments.length),
        i;
      for(i = 0; i < boundLen; i++) args[i] = boundArgs[i];
      for(i = 0; i < arguments.length; i++) args[boundLen + i] = arguments[i];

      if(new.target /*this instanceof ret*/) {
        let ret_ = f.apply(this, args);
        return Object(ret_) === ret_ ? ret_ : this;
      } else return f.apply(boundThis, args);
    };
  }

  ret.prototype = f.prototype;
  return ret;
};

Util.bytesToUTF8 = function* (bytes) {
  if(bytes instanceof ArrayBuffer) bytes = new Uint8Array(bytes);
  let state = 0,
    val = 0;
  for(const c of bytes) {
    if(state !== 0 && c >= 0x80 && c < 0xc0) {
      val = (val << 6) | (c & 0x3f);
      state--;
      if(state === 0) yield val;
    } else if(c >= 0xc0 && c < 0xf8) {
      state = 1 + (c >= 0xe0) + (c >= 0xf0);
      val = c & ((1 << (6 - state)) - 1);
    } else {
      state = 0;
      yield c;
    }
  }
};
Util.codePointsToString = codePoints => {
  let s = '';
  for(let c of codePoints) s += String.fromCodePoint(c);
  return s;
};
Util.bufferToString = b => Util.codePointsToString(Util.bytesToUTF8(b));

Util.levenshteinDistance = function levenshteinDistance(a, b) {
  if(!a || !b) return (a || b).length;
  var m = [];
  for(var i = 0; i <= b.length; i++) {
    m[i] = [i];
    if(i === 0) continue;
    for(var j = 0; j <= a.length; j++) {
      m[0][j] = j;
      if(j === 0) continue;
      m[i][j] = b.charAt(i - 1) == a.charAt(j - 1) ? m[i - 1][j - 1] : Math.min(m[i - 1][j - 1] + 1, m[i][j - 1] + 1, m[i - 1][j] + 1);
    }
  }
  return m[b.length][a.length];
};

Util.padTrunc = (...args) => {
  let [len, s] = args;
  const end = len >= 0;
  len = Math.abs(len);
  if(args.length < 2) {
    return (s, pad = ' ') => {
      s = s + '';
      len ??= s.length;
      return s.length > len ? s.slice(0, len) : s['pad' + (end ? 'End' : 'Start')](len, pad);
    };
  } else {
    s = s + '';
    len ??= s.length;
    return s.length > len ? s.slice(0, len) : s['pad' + (end ? 'End' : 'Start')](len, ' ');
  }
};

Util.setReadHandler = (fd, handler) => (Util.getPlatform() == 'quickjs' ? import('os').then(os => os.setReadHandler(fd, handler)) : fd.on('data', handler));

/* ------------------------- end of './lib/util.js' ------------------------- */

/* --------------------- start of './lib/color/rgba.js' --------------------- */

/**
 * @brief [brief description]
 * @param r  red value 0-255
 * @param g  green value 0-255
 * @param b  blue value 0-255
 * @param a  alpha value 0-1
 *
 * @return [description]
 */

function RGBA(...args) {
  let ret = this instanceof RGBA ? this : {};
  let c = [];

  if(args.length == 1 && Util.isArray(args[0]) && args[0].length >= 3) args = args[0];
  //console.log('RGBA(', args, ')');

  if(args.length >= 3) {
    const [r = 0, g = 0, b = 0, a = 255] = args;
    ret.r = r;
    ret.g = g;
    ret.b = b;
    if(!isNaN(+a) /* && +a !== 255*/) ret.a = a;
  } else if(args.length <= 2) {
    const arg = args[0];
    if(typeof arg === 'number') {
      Object.assign(ret, RGBA.decode[args[1] !== undefined ? args[1] : RGBA.order.ABGR](arg));
    } else if(typeof arg === 'string') {
      if(arg.startsWith('#')) {
        c = arg.length >= 7 ? /^#?([0-9a-fA-F]{2})([0-9a-fA-F]{2})([0-9a-fA-F]{2})([0-9a-fA-F]{2})?$/i.exec(arg) : /^#?([0-9a-fA-F])([0-9a-fA-F])([0-9a-fA-F])([0-9a-fA-F])?$/i.exec(arg);

        let mul = arg.length >= 7 ? 1 : 17;

        //console.log('RGBA match:', c, ' mul:', mul);

        ret.r = parseInt(c[1], 16) * mul;
        ret.g = parseInt(c[2], 16) * mul;
        ret.b = parseInt(c[3], 16) * mul;
        if(c.length > 3) {
          let a = parseInt(c[4], 16) * mul;
          /*if(a !== 255)*/ ret.a = a;
        }
      } else if(arg.toLowerCase().startsWith('rgb')) {
        c = arg.match(/[\d.%]+/g).map(x => (x.endsWith('%') ? parseFloat(x.slice(0, -1)) * 2.55 : +x));

        c = [...c].slice();

        ret.r = Math.round(c[0]);
        ret.g = Math.round(c[1]);
        ret.b = Math.round(c[2]);
        if(c.length > 3) ret.a = Math.round(c[3] * 255);
      }
    } else if(typeof arg === 'object' && arg.r !== undefined) {
      ret.r = arg.r;
      ret.g = arg.g;
      ret.b = arg.b;
      if(arg.a !== undefined) ret.a = arg.a;
    } else {
      ret.r = 0;
      ret.g = 0;
      ret.b = 0;
      ret.a = 0;
    }
  }

  if(ret.a !== undefined && isNaN(+ret.a)) ret.a = 255;
  if(isNaN(ret.a)) ret.a = 255;

  //console.log('RGBA ', ret);
  if(!(ret instanceof RGBA)) return ret; //Object.setPrototypeOf(ret, RGBA.prototype);
}

RGBA.properties = ['r', 'g', 'b', 'a'];

const isRGBA = obj => RGBA.properties.every(prop => obj.hasOwnProperty(prop));

RGBA.fromString = str => {
  let c = Util.tryCatch(
    () => new HSLA(str),
    c => c.toRGBA(),
    () => undefined
  );
  if(!c)
    c = Util.tryCatch(
      () => new RGBA(str),
      c => c,
      () => undefined
    );
  return c;
};
RGBA.order = {
  RGBA: 0,
  BGRA: 1,
  ARGB: 2,
  ABGR: 3
};

RGBA.decode = [
  /*RGBA:*/ n => ({
    r: (n >> 24) & 0xff,
    g: (n >> 16) & 0xff,
    b: (n >> 8) & 0xff,
    a: n & 0xff
  }),
  /*BGRA:*/ n => ({
    b: (n >> 24) & 0xff,
    g: (n >> 16) & 0xff,
    r: (n >> 8) & 0xff,
    a: n & 0xff
  }),
  /*ARGB:*/ n => ({
    a: (n >> 24) & 0xff,
    r: (n >> 16) & 0xff,
    g: (n >> 8) & 0xff,
    b: n & 0xff
  }),
  /*ABGR:*/ n => ({
    a: (n >> 24) & 0xff,
    b: (n >> 16) & 0xff,
    g: (n >> 8) & 0xff,
    r: n & 0xff
  })
];
RGBA.encode = [
  /*RGBA:*/ ({ r, g, b, a }) => [r, g, b, a].map(n => ('00' + (n & 0xff).toString(16)).slice(-2)).join(''),
  /*BGRA:*/ ({ r, g, b, a }) => [b, g, r, a].map(n => ('00' + (n & 0xff).toString(16)).slice(-2)).join(''),
  /*ARGB:*/ ({ r, g, b, a }) => [a, r, g, b].map(n => ('00' + (n & 0xff).toString(16)).slice(-2)).join(''),
  /*ABGR:*/ ({ r, g, b, a }) => [a, b, g, r].map(n => ('00' + (n & 0xff).toString(16)).slice(-2)).join('')
];
RGBA.fmt = [({ r, g, b, a }) => [r, g, b, a], ({ b, g, r, a }) => [b, g, r, a], ({ a, r, g, b }) => [a, r, g, b], ({ a, b, g, r }) => [a, b, g, r]];

RGBA.calculators = [
  ({ r, g, b, a }) => ((r * 256 + g) * 256 + b) * 256 + a,
  ({ b, g, r, a }) => ((b * 256 + g) * 256 + r) * 256 + a,
  ({ a, r, g, b }) => ((a * 256 + r) * 256 + g) * 256 + b,
  ({ a, b, g, r }) => ((a * 256 + b) * 256 + g) * 256 + r
];

RGBA.prototype.clone = function() {
  const ctor = this.constructor[Symbol.species];
  const { r, g, b, a } = this;
  return new ctor(r, g, b, a);
};
RGBA.prototype.binaryValue = function(order = 0) {
  const { r, g, b, a } = this;
  return RGBA.calculators[order](RGBA.clamp(this));
};
RGBA.prototype.valid = function() {
  const { r, g, b, a } = this;
  return [r, g, b, a].every(n => {
    n = +n;
    return !isNaN(n) && n >= 0 && n <= 255;
  });
};

RGBA.prototype.compareTo = function(other) {
  let d = RGBA.prototype.binaryValue.call(other) - RGBA.prototype.binaryValue.call(this);
  return d < 0 ? -1 : d > 0 ? 1 : 0;
};
RGBA.fromHex = (hex, alpha = 255) => {
  if(hex[0] != '#') hex = ('ffffffff' + hex).slice(-8);

  const matches =
    hex && (hex.length >= 7 ? /^#?([0-9a-fA-F]{2})([0-9a-fA-F]{2})([0-9a-fA-F]{2})([0-9a-fA-F]{2})?$/i.exec(hex) : /^#?([0-9a-fA-F])([0-9a-fA-F])([0-9a-fA-F])([0-9a-fA-F])?$/i.exec(hex));
  if(matches === null) return null;
  let mul = hex.length >= 7 ? 1 : 17;

  const [r, g, b, a] = [...matches].slice(1).map(x => parseInt(x, 16) * mul);
  //console.log('RGBA.fromHex', { hex, matches, r, g, b, a });
  return new RGBA(r, g, b, matches.length > 3 && !isNaN(a) ? a : alpha);
};

RGBA.prototype.hex = function(opts = {}) {
  const { bits, prefix = '#', order = RGBA.order.RGBA } = opts;
  const { r, g, b, a } = RGBA.clamp(RGBA.round(this));
  const n = RGBA.encode[order]({ r, g, b, a });
  return prefix + ('0000000000' + n.toString(16)).slice(-8).slice(0, a === 255 || a === undefined ? 6 : 8);
};

/*RGBA.prototype.valueOf = function() {
  const hex = RGBA.prototype.hex.call(this);
  return parseInt('0x' + hex.slice(1));
};*/

RGBA.prototype.toRGB = function() {
  const { r, g, b } = this;
  return new RGBA(r, g, b, 255);
};

RGBA.toHex = rgba => RGBA.prototype.hex.call(rgba);

RGBA.clamp = rgba => RGBA(Math.min(Math.max(rgba.r, 0), 255), Math.min(Math.max(rgba.g, 0), 255), Math.min(Math.max(rgba.b, 0), 255), Math.min(Math.max(rgba.a, 0), 255));
RGBA.round = rgba => RGBA.prototype.round.call(rgba);

RGBA.prototype.round = function() {
  const { r, g, b, a } = this;
  let x = [r, g, b, a].map(n => Math.round(n));
  if(Object.isFrozen(this)) return new RGBA(...x);
  this.r = x[0];
  this.g = x[1];
  this.b = x[2];
  this.a = x[3];
  return this;
};
RGBA.prototype.setOpacity = function(a) {
  this.a = Util.clamp(0, 255, a * 255);
  return this;
};
RGBA.normalize = function(rgba, src = 255, dst = 1.0) {
  return {
    r: (rgba.r * dst) / src,
    g: (rgba.g * dst) / src,
    b: (rgba.b * dst) / src,
    a: (rgba.a * dst) / src
  };
};
RGBA.prototype.css = () => prop => (prop ? prop + ':' : '') + 'rgba(' + this.r + ', ' + this.g + ', ' + this.b + ', ' + (this.a / 255).toFixed(3) + ')';

RGBA.prototype.toCSS = function(fmt = num => +num.toFixed(3)) {
  const { r, g, b, a } = this;
  const sep = ',';
  if(a === undefined || a == 255) return 'rgb(' + fmt(r) + sep + fmt(g) + sep + fmt(b) + ')';
  return 'rgba(' + fmt(r) + sep + fmt(g) + sep + fmt(b) + sep + (a * 100) / 255 + '%)';
};
RGBA.prototype.toString = function(opts) {
  return RGBA.prototype.hex.call(this, opts);
};
Util.defineGetter(RGBA.prototype, Symbol.toStringTag, function() {
  const { r, g, b, a } = this;
  return `[object RGBA ${r},${g},${b},${a}]`;
});
RGBA.prototype[Symbol.toPrimitive] = function(hint) {
  //console.log("RGBA.toPrimitive", {hint});
  if(hint == 'number') return +RGBA.prototype.hex.call(this, { prefix: '0x' });
  if(hint == 'string') return RGBA.prototype.toString.call(this);
  if(hint == 'default') return RGBA.prototype.hex.call(this, { prefix: '#' });
};
function toHex(n) {
  return '0x' + ('00' + (+n).toString(16)).slice(-2);
}
RGBA.prototype.toSource = function(sep = ',') {
  let a = this.a;
  if(a === undefined) return 'new RGBA(' + this.r + sep + this.g + sep + this.b + ')';
  let s = 'new RGBA(' + toHex(this.r) + sep + toHex(this.g) + sep + toHex(this.b);

  if(a != 255) s += sep + toHex(a);

  s += ')';
  return s;
};

RGBA.prototype.normalize = function(src = 255, dst = 1.0) {
  return new RGBA((this.r * dst) / src, (this.g * dst) / src, (this.b * dst) / src, (this.a * dst) / src);
};

RGBA.blend = (a, b, o = 0.5) => {
  a = new RGBA(a);
  b = new RGBA(b);
  return new RGBA(Math.round(a.r * (1 - o) + b.r * o), Math.round(a.g * (1 - o) + b.g * o), Math.round(a.b * (1 - o) + b.b * o), Math.round(a.a * (1 - o) + b.a * o));
};

RGBA.prototype.toAlpha = Util.curry(function (other) {
  let color = RGBA.normalize(this);
  let src = new RGBA(other).normalize();
  let alpha = {};

  alpha.a = src.a;
  if(color.r < 0.0001) alpha.r = src.r;
  else if(src.r > color.r) alpha.r = (src.r - color.r) / (1.0 - color.r);
  else if(src.r < color.r) alpha.r = (color.r - src.r) / color.r;
  else alpha.r = 0.0;
  if(color.g < 0.0001) alpha.g = src.g;
  else if(src.g > color.g) alpha.g = (src.g - color.g) / (1.0 - color.g);
  else if(src.g < color.g) alpha.g = (color.g - src.g) / color.g;
  else alpha.g = 0.0;
  if(color.b < 0.0001) alpha.b = src.b;
  else if(src.b > color.b) alpha.b = (src.b - color.b) / (1.0 - color.b);
  else if(src.b < color.b) alpha.b = (color.b - src.b) / color.b;
  else alpha.b = 0.0;
  if(alpha.r > alpha.g) {
    if(alpha.r > alpha.b) {
      src.a = alpha.r;
    } else {
      src.a = alpha.b;
    }
  } else if(alpha.g > alpha.b) {
    src.a = alpha.g;
  } else {
    src.a = alpha.b;
  }
  if(src.a >= 0.0001) {
    src.r = (src.r - color.r) / src.a + color.r;
    src.g = (src.g - color.g) / src.a + color.g;
    src.b = (src.b - color.b) / src.a + color.b;
    src.a *= alpha.a;
  }

  let dst = new RGBA(src.r * 255, src.g * 255, src.b * 255, src.a * 255);

  return dst.round();
});
RGBA.prototype.toRGB = function() {
  const ctor = this.constructor[Symbol.species];
  const { r, g, b } = this;
  return new ctor(r, g, b, 255);
};
RGBA.prototype.toBGRA = function() {
  const { r, g, b, a } = this;
  return [b, g, r, a];
};
RGBA.prototype.toHSLA = function() {
  let { r, g, b, a } = this;
  r /= 255;
  g /= 255;
  b /= 255;
  a /= 255;
  let max = Math.max(r, g, b);
  let min = Math.min(r, g, b);
  let h;
  let s;
  let l = (max + min) / 2;
  if(max == min) {
    h = s = 0; //achromatic
  } else {
    let d = max - min;
    s = l > 0.5 ? d / (2 - max - min) : d / (max + min);
    switch (max) {
      case r:
        h = (g - b) / d + (g < b ? 6 : 0);
        break;
      case g:
        h = (b - r) / d + 2;
        break;
      case b:
        h = (r - g) / d + 4;
        break;
    }
    h /= 6;
  }

  h *= 360;
  s *= 100;
  l *= 100;

  //console.log("RGBA.toHSLA ", { h, s, l, a });

  return new (Object.isFrozen(this) ? ImmutableHSLA : HSLA)(Math.round(h), Util.roundTo(s, 100 / 255), Util.roundTo(l, 100 / 255), Util.roundTo(a, 1 / 255));
};

RGBA.prototype.toCMYK = function() {
  let res = {};
  let { r, g, b } = RGBA.prototype.normalize.call(this, 255);

  res.k = Math.min(1 - r, 1 - g, 1 - b);
  res.c = (1 - r - res.k) / (1 - res.k);
  res.m = (1 - g - res.k) / (1 - res.k);
  res.y = (1 - b - res.k) / (1 - res.k);

  return {
    c: Math.round(res.c * 100),
    m: Math.round(res.m * 100),
    y: Math.round(res.y * 100),
    k: Math.round(res.k * 100),
    a: this.a
  };
};

RGBA.prototype.toLAB = function() {
  let { r, g, b } = RGBA.prototype.normalize.call(this, 255);
  let x, y, z;

  r = r > 0.04045 ? Math.pow((r + 0.055) / 1.055, 2.4) : r / 12.92;
  g = g > 0.04045 ? Math.pow((g + 0.055) / 1.055, 2.4) : g / 12.92;
  b = b > 0.04045 ? Math.pow((b + 0.055) / 1.055, 2.4) : b / 12.92;

  x = (r * 0.4124 + g * 0.3576 + b * 0.1805) / 0.95047;
  y = (r * 0.2126 + g * 0.7152 + b * 0.0722) / 1.0;
  z = (r * 0.0193 + g * 0.1192 + b * 0.9505) / 1.08883;

  x = x > 0.008856 ? Math.pow(x, 1 / 3) : 7.787 * x + 16 / 116;
  y = y > 0.008856 ? Math.pow(y, 1 / 3) : 7.787 * y + 16 / 116;
  z = z > 0.008856 ? Math.pow(z, 1 / 3) : 7.787 * z + 16 / 116;

  return { l: 116 * y - 16, A: 500 * (x - y), B: 200 * (y - z), a: this.a };
};
RGBA.fromLAB = function(lab) {
  let y = (lab.l + 16) / 116,
    x = lab.A / 500 + y,
    z = y - lab.B / 200,
    r,
    g,
    b;

  x = 0.95047 * (x * x * x > 0.008856 ? x * x * x : (x - 16 / 116) / 7.787);
  y = 1.0 * (y * y * y > 0.008856 ? y * y * y : (y - 16 / 116) / 7.787);
  z = 1.08883 * (z * z * z > 0.008856 ? z * z * z : (z - 16 / 116) / 7.787);

  r = x * 3.2406 + y * -1.5372 + z * -0.4986;
  g = x * -0.9689 + y * 1.8758 + z * 0.0415;
  b = x * 0.0557 + y * -0.204 + z * 1.057;

  r = r > 0.0031308 ? 1.055 * Math.pow(r, 1 / 2.4) - 0.055 : 12.92 * r;
  g = g > 0.0031308 ? 1.055 * Math.pow(g, 1 / 2.4) - 0.055 : 12.92 * g;
  b = b > 0.0031308 ? 1.055 * Math.pow(b, 1 / 2.4) - 0.055 : 12.92 * b;

  return new this(Math.round(Math.max(0, Math.min(1, r)) * 255), Math.round(Math.max(0, Math.min(1, g)) * 255), Math.round(Math.max(0, Math.min(1, b)) * 255), lab.a || 255);
  return this;
};

RGBA.prototype.linear = function() {
  let { r, g, b } = RGBA.prototype.normalize.call(this, 255);

  //apply gamma
  let gamma = 2.2;
  r = Math.pow(r, gamma); //linearize red
  g = Math.pow(g, gamma); //linearize green
  b = Math.pow(b, gamma); //linearize blue
  return { r, g, b };
};

RGBA.prototype.luminance = function() {
  let lin = RGBA.prototype.linear.call(this);
  let Y = 0.2126 * lin.r; //red channel
  Y = Y + 0.7152 * lin.g; //green channel
  Y = Y + 0.0722 * lin.b; //blue channel
  return Y;
};
RGBA.prototype.invert = function() {
  const { r, g, b, a } = RGBA.clamp(this);
  return new RGBA(255 - r, 255 - g, 255 - b, a);
};
RGBA.prototype.blackwhite = function(a = this.a) {
  return this.luminanace() >= 0.558 ? new RGBA(255, 255, 255, a) : new RGBA(0, 0, 0, a);
};
RGBA.prototype.distance = function(other) {
  return Math.sqrt(Math.pow(other.r - this.r, 2) + Math.pow(other.g - this.g, 2) + Math.pow(other.b - this.b, 2)) / 441.67295593006370984949;
};
RGBA.prototype.luminanace = function() {
  const { r, g, b } = RGBA.prototype.normalize.call(this, 255);
  let a = [r, g, b].map(v => (v <= 0.03928 ? v / 12.92 : Math.pow((v + 0.055) / 1.055, 2.4)));
  return a[0] * 0.2126 + a[1] * 0.7152 + a[2] * 0.0722;
};
RGBA.prototype.contrast = function contrast(other) {
  let lum1 = this.luminanace();
  let lum2 = RGBA.prototype.luminanace.call(other);
  let brightest = Math.max(lum1, lum2);
  let darkest = Math.min(lum1, lum2);
  return (brightest + 0.05) / (darkest + 0.05);
};
RGBA.prototype.toConsole = function(fn = 'toString', css = []) {
  const textColor = this.invert().blackwhite();
  const bgColor = textColor.invert();
  if(typeof fn == 'string' && fn in this) {
    let method = fn;
    fn = () => this[method]();
  } else if(typeof fn == 'function') {
    let method = fn;
    fn = () => method.call(this, this);
  }
  let text = fn();
  let style = [`text-shadow: 1px 1px 1px ${bgColor.hex()}`, `border: 1px solid black`, `padding: 2px`, `background-color: ${this.hex()}`, `color: ${textColor}`, `background-color: none`, ...css];
  return [`%c${text}%c`, style.join(';'), `color: inherit; background-color: inherit;`];
};
RGBA.prototype.dump = function(...args) {
  console.log(...this.toConsole(...args));
};

RGBA.prototype.equals = function(other) {
  const { r, g, b, a } = this;
  return r == other.r && g == other.g && b == other.b && a == other.a;
};
RGBA.prototype.toObject = function() {
  const { r, g, b, a } = RGBA.clamp(this);
  return { r, g, b, a };
};
RGBA.prototype.toArray = function() {
  return Uint8Array.from(this);
};
RGBA.prototype.toAnsi = function(background = false) {
  const { r, g, b } = this;

  return `\u001b[${background ? 48 : 38};2;${[r, g, b].join(';')}m  `;
};
RGBA.fromAnsi256 = function(n) {
  let r, g, b;
  let c;
  if(n < 16) {
    r = n & 1;
    n >>= 1;
    g = n & 1;
    n >>= 1;
    b = n & 1;
    n >>= 1;
    c = [r, g, b].map(v => [n & 1 ? 85 : 0, n & 1 ? 255 : 170][v]);
  } else if(n >= 16 && n < 232) {
    n -= 16;
    b = n % 6;
    n /= 6;
    g = n % 6;
    n /= 6;
    r = n % 6;
    c = [r, g, b].map(n => (n * 255) / 5);
  } else if(n >= 232) {
    n -= 231;
    r = g = b = (n * 255) / (255 - 231);
    c = [r, g, b];
  }
  if(c) {
    c = c.map(Math.round);
    c = c.map(n => Math.min(255, n));
    return new RGBA(...c);
  }
};
RGBA.nearestColor = (color, palette, distFn = (a, b) => Math.sqrt(Math.pow(a.r - b.r, 2) + Math.pow(a.g - b.g, 2) + Math.pow(a.b - b.b, 2))) => {
  if(!(color instanceof RGBA)) color = new RGBA(color);
  //console.log("RGBA.nearestColor", color.hex(),Util.className(color),Util.className(palette));
  if(!color) return null;
  let dist,
    minDist = Infinity,
    rgb,
    v,
    vi;
  palette || (palette = RGBA.palette16);
  for(let i = 0; i < palette.length; ++i) {
    rgb = palette[i];
    dist = distFn(color, rgb);
    if(dist < minDist) {
      minDist = dist;
      v = palette[i];
      vi = i;
    }
  }
  if(v) {
    return {
      value: v,
      index: vi,
      distance: minDist
    };
  }
  return v;
};
RGBA.prototype.toAnsi256 = function(background = false) {
  const { r, g, b } = this;
  const { index, distance } = RGBA.nearestColor(this, RGBA.palette16);
  if(distance == 0) {
    let bold = index & 0x08 ? 1 : 0;
    let num = (index & 0x07) + (background ? 40 : 30);

    return `\x1b[${bold};${num}m`;
  }
  const fromRGB = (r, g, b) => {
    if(r === g && g === b) {
      if(r < 8) return 16;

      if(r > 248) return 231;
      return Math.round(((r - 8) / 247) * 24) + 232;
    }
    return 16 + 36 * Math.round((r / 255) * 5) + 6 * Math.round((g / 255) * 5) + Math.round((b / 255) * 5);
  };
  let value = fromRGB(r, g, b);
  const toString = (background = false) => `\x1b[${background ? 48 : 38};5;${value}m`;
  let ret = toString(background);
  //ret.value = value;
  return ret;
};

RGBA.prototype[Symbol.iterator] = function* () {
  const { r, g, b, a } = this;
  yield* [r, g, b, a][Symbol.iterator]();
};

RGBA.prototype[Symbol.for('nodejs.util.inspect.custom')] = function() {
  const { r, g, b, a } = this;
  let arr = a !== undefined && a != 255 ? [r, g, b, a] : [r, g, b];
  let ret = arr
    .map(toHex) /*.map(n => (n + '').padStart(3, ' '))*/
    .join(',');
  const color = this.toAnsi(/*256*/ true);
  const l = this.toHSLA().l;
  let s = '';

  s += arr.map(n => `\x1b[0;33m${toHex(n)}\x1b[0m`).join('');
  s = color + s;

  return `\x1b[1;31mRGBA\x1b[1;36m` + `(${ret})`.padEnd(18, ' ') + ` ${color}    \x1b[0m`;
};

Util.define(RGBA, {
  get palette16() {
    //const clamp = Util.clamp(0, 255);
    // /* prettier-ignore */ const a = [[0, 0, 0], [2, 0, 0], [0, 2, 0], [2, 2, 0], [0, 0, 2], [2, 0, 2], [0, 2, 2], [3, 3, 3], [2, 2, 2], [4, 0, 0], [0, 4, 0], [4, 4, 0], [0, 0, 4], [4, 0, 4], [0, 4, 4], [4, 4, 4] ];
    // /* prettier-ignore */ const b = [[1, 1, 1], [4, 0, 0], [2, 3, 1], [4, 3, 0], [1, 2, 3], [2, 2, 2], [1, 3, 3], [4, 4, 4], [2, 2, 2], [4, 1, 1], [3, 4, 1], [4, 4, 2], [2, 3, 4], [3, 2, 3], [1, 4, 4], [4, 4, 4] ];
    //return b.map(c => new RGBA(c.map(n => Math.round(clamp(n * 64)))));
    return this.palette256.slice(0, 16);
  },
  get palette256() {
    /* prettier-ignore */ return [0x000000, 0xaa0000, 0x00aa00, 0xaaaa00, 0x0000aa, 0xaa00aa, 0x00aaaa, 0xaaaaaa, 0x555555, 0xff5555, 0x55ff55, 0xffff55, 0x5555ff, 0xff55ff, 0x55ffff, 0xffffff, 0x000000, 0x010933, 0x031166, 0x041a99, 0x0622cc, 0x072bff, 0x093300, 0x0a3c33, 0x0b4466, 0x0d4d99, 0x0e55cc, 0x105eff, 0x116600, 0x126f33, 0x147766, 0x158099, 0x1788cc, 0x1891ff, 0x1a9900, 0x1ba233, 0x1caa66, 0x1eb399, 0x1fbbcc, 0x21c4ff, 0x22cc00, 0x23d533, 0x25dd66, 0x26e699, 0x28eecc, 0x29f7ff, 0x2bff00, 0x2cff33, 0x2dff66, 0x2fff99, 0x30ffcc, 0x32ffff, 0x330000, 0x340933, 0x361166, 0x371a99, 0x3922cc, 0x3a2aff, 0x3c3300, 0x3d3c33, 0x3e4466, 0x404d99, 0x4155cc, 0x435dff, 0x446600, 0x456e33, 0x477766, 0x488099, 0x4a88cc, 0x4b91ff, 0x4d9900, 0x4ea133, 0x4faa66, 0x51b399, 0x52bbcc, 0x54c4ff, 0x55cc00, 0x56d433, 0x58dd66, 0x59e699, 0x5beecc, 0x5cf7ff, 0x5eff00, 0x5fff33, 0x60ff66, 0x62ff99, 0x63ffcc, 0x65ffff, 0x660000, 0x670833, 0x691166, 0x6a1a99, 0x6c22cc, 0x6d2bff, 0x6f3300, 0x703b33, 0x714466, 0x734d99, 0x7455cc, 0x765eff, 0x776600, 0x786e33, 0x7a7766, 0x7b8099, 0x7d88cc, 0x7e91ff, 0x809900, 0x81a133, 0x82aa66, 0x84b399, 0x85bbcc, 0x87c4ff, 0x88cc00, 0x89d533, 0x8bdd66, 0x8ce699, 0x8eeecc, 0x8ff6ff, 0x91ff00, 0x92ff33, 0x93ff66, 0x95ff99, 0x96ffcc, 0x98ffff, 0x990000, 0x9a0933, 0x9c1166, 0x9d1a99, 0x9f22cc, 0xa02aff, 0xa23300, 0xa33c33, 0xa44466, 0xa64d99, 0xa755cc, 0xa95dff, 0xaa6600, 0xab6f33, 0xad7766, 0xae8099, 0xb088cc, 0xb190ff, 0xb39900, 0xb4a233, 0xb5aa66, 0xb7b399, 0xb8bbcc, 0xbac3ff, 0xbbcc00, 0xbcd533, 0xbedd66, 0xbfe699, 0xc1eecc, 0xc2f6ff, 0xc4ff00, 0xc5ff33, 0xc6ff66, 0xc8ff99, 0xc9ffcc, 0xcbffff, 0xcc0000, 0xcd0933, 0xcf1166, 0xd01a99, 0xd222cc, 0xd32aff, 0xd53300, 0xd63c33, 0xd74466, 0xd94d99, 0xda55cc, 0xdc5dff, 0xdd6600, 0xde6f33, 0xe07766, 0xe18099, 0xe388cc, 0xe490ff, 0xe69900, 0xe7a233, 0xe8aa66, 0xeab399, 0xebbbcc, 0xedc3ff, 0xeecc00, 0xefd533, 0xf1dd66, 0xf2e699, 0xf4eecc, 0xf5f6ff, 0xf7ff00, 0xf8ff33, 0xf9ff66, 0xfbff99, 0xfcffcc, 0xfeffff, 0xff0000, 0xff0933, 0xff1166, 0xff1a99, 0xff22cc, 0xff2aff, 0xff3300, 0xff3c33, 0xff4466, 0xff4d99, 0xff55cc, 0xff5dff, 0xff6600, 0xff6e33, 0xff7766, 0xff8099, 0xff88cc, 0xff91ff, 0xff9900, 0xffa133, 0xffaa66, 0xffb399, 0xffbbcc, 0xffc4ff, 0xffcc00, 0xffd433, 0xffdd66, 0xffe699, 0xffeecc, 0xfff7ff, 0xffff00, 0xffff33, 0xffff66, 0xffff99, 0xffffcc, 0xffffff, 0x0b0b0b, 0x151515, 0x202020, 0x2b2b2b, 0x353535, 0x404040, 0x4a4a4a, 0x555555, 0x606060, 0x6a6a6a, 0x757575, 0x808080, 0x8a8a8a, 0x959595, 0x9f9f9f, 0xaaaaaa, 0xb5b5b5, 0xbfbfbf, 0xcacaca, 0xd5d5d5, 0xdfdfdf, 0xeaeaea, 0xf4f4f4, 0xffffff].map(n => new RGBA(n|0xff000000, RGBA.order.ARGB));
  }
});

RGBA.random = function(r = [0, 255], g = [0, 255], b = [0, 255], a = [255, 255], rng = Math.random) {
  return new RGBA(Util.randInt(...r, rng), Util.randInt(...g, rng), Util.randInt(...b, rng), Util.randInt(...a, rng));
};

for(let name of ['hex', 'toRGB', 'round', 'toHSLA', 'toCMYK', 'toLAB', 'linear', 'luminance', 'distance']) {
  RGBA[name] = (...args) => RGBA.prototype[name].call(...args);
}
/*
for(let name of ['fromLAB']) {
  RGBA[name] = arg => {
    let ret = new RGBA();
    return RGBA.prototype[name].call(ret, arg);
  };
}*/

Util.defineGetter(RGBA, Symbol.species, function() {
  return this;
});

const ImmutableRGBA = Util.immutableClass(RGBA);
Util.defineGetter(ImmutableRGBA, Symbol.species, () => ImmutableRGBA);

/* ---------------------- end of './lib/color/rgba.js' ---------------------- */

/* --------------------- start of './lib/color/hsla.js' --------------------- */

/**
 * @brief [brief description]
 * @param h  hue value 0-360
 * @param s  saturation 0-100%
 * @param l  luminance 0-100%
 * @param a  alpha 0-1.0
 *
 * @return [description]
 */

function HSLA(h = 0, s = 0, l = 0, a = 1.0) {
  const args = [...arguments];
  let c = [];
  let ret = this instanceof HSLA ? this : {};

  /*  if(!this) return Object.assign({}, HSLA.prototype, { h, s, l, a });*/

  if(typeof args[0] == 'object' && 'h' in args[0] && 's' in args[0] && 'l' in args[0]) {
    ret.h = args[0].h;
    ret.s = args[0].s;
    ret.l = args[0].l;
    ret.a = args[0].a || 1.0;
  } else if(args.length >= 3) {
    ret.h = Math.round(+h);
    ret.s = s;
    ret.l = l;
    ret.a = a;
  } else if(typeof args[0] == 'string') {
    const arg = args[0];
    if(typeof arg === 'string') {
      let matches = /hsla\(\s*([0-9.]+)\s*,\s*([0-9.]+%?)\s*,\s*([0-9.]+%?),\s*([0-9.]+)\s*\)/g.exec(arg) || /hsl\(\s*([0-9.]+)\s*,\s*([0-9.]+%?)\s*,\s*([0-9.]+%?)\s*\)/g.exec(arg);

      if(matches != null) c = [...matches].slice(1);
    }

    if(c.length < 3) throw new Error('Invalid HSLA color:' + args);
    ret.h = c[0];
    ret.s = c[1];
    ret.l = c[2];
    ret.a = c[3] !== undefined ? c[3] : 1.0;

    ['h', 's', 'l', 'a'].forEach(channel => {
      if(String(ret[channel]).endsWith('%')) ret[channel] = parseFloat(ret[channel].slice(0, -1));
      else ret[channel] = parseFloat(ret[channel]) * (channel == 'a' || channel == 'h' ? 1 : 100);
    });
  } else {
    ret.h = 0;
    ret.s = 0;
    ret.l = 0;
    ret.a = 0;
  }

  //console.log('HSLA ', { c, ret, args });
  if(!(ret instanceof HSLA)) return ret;
}

HSLA.prototype.properties = ['h', 's', 'l', 'a'];

HSLA.prototype.clone = function() {
  const ctor = this.constructor[Symbol.species];
  const { h, s, l, a } = this;
  return new ctor(h, s, l, a);
};
//export const isHSLA = obj => HSLA.properties.every(prop => obj.hasOwnProperty(prop));

HSLA.prototype.css = function() {
  const hsla = HSLA.clamp(HSLA.round(this));
  return HSLA.setcss(hsla)();
};
HSLA.prototype.toHSL = function() {
  const { h, s, l } = this;
  return new HSLA(h, s, l, 1.0);
};

HSLA.prototype.clamp = function() {
  this.h = (this.h % 360) + (this.h < 0 ? 360 : 0);
  this.s = Math.min(Math.max(this.s, 0), 100);
  this.l = Math.min(Math.max(this.l, 0), 100);
  this.a = Math.min(Math.max(this.a, 0), 1);
  return this;
};
HSLA.prototype.round = function(prec = 1 / 255, digits = 3) {
  const { h, s, l, a } = this;
  const precs = [360, 100, 100, 1];
  let x = [h, s, l, a].map((n, i) => Util.roundTo(n, precs[i] * prec, digits));
  if(Object.isFrozen(this)) return new HSLA(...x);
  this.h = x[0];
  this.s = x[1];
  this.l = x[2];
  this.a = x[3];
  return this;
};
HSLA.prototype.add = function(h = 0, s = 0, l = 0, a = 0) {
  this.h += h;
  this.s += s;
  this.l += l;
  this.a += a;
  return this.clamp();
};
HSLA.prototype.sub = function(h = 0, s = 0, l = 0, a = 0) {
  this.h -= h;
  this.s -= s;
  this.l -= l;
  this.a -= a;
  return this.clamp();
};

HSLA.prototype.sum = function(...args) {
  let r = new HSLA(...args);
  r.add(this.h, this.s, this.l, this.a);
  return r;
};

HSLA.prototype.diff = function(...args) {
  let r = new HSLA(...args);
  r.sub(this.h, this.s, this.l, this.a);
  return r;
};

HSLA.prototype.mul = function(h = 1, s = 1, l = 1, a = 1) {
  this.h *= h;
  this.s *= s;
  this.l *= l;
  this.a *= a;
  return this.clamp();
};

HSLA.prototype.prod = function(...args) {
  let r = new HSLA(...args);
  r.mul(this.h, this.s, this.l, this.a);
  return r;
};

HSLA.prototype.hex = function() {
  return RGBA.prototype.hex.call(HSLA.prototype.toRGBA.call(this));
};

HSLA.prototype.valueOf = function() {
  const hex = HSLA.prototype.hex.call(this);
  return parseInt('0x' + hex.slice(1));
};
HSLA.prototype[Symbol.toStringTag] = function() {
  return HSLA.prototype.toString.call(this);
};
HSLA.prototype[Symbol.toPrimitive] = function(hint) {
  if(hint == 'default') return HSLA.prototype.hex.call(this);
  return HSLA.prototype.toString.call(this);
};
HSLA.prototype.toBGRA = function() {
  let { r, g, b, a } = this.toRGBA();
  return [b, g, r, a];
};
HSLA.prototype.toRGBA = function() {
  let { h, s, l, a } = this;

  let r, g, b, m, c, x;

  if(!isFinite(h)) h = 0;
  if(!isFinite(s)) s = 0;
  if(!isFinite(l)) l = 0;

  h /= 60;
  if(h < 0) h = 6 - (-h % 6);
  h %= 6;

  s = Math.max(0, Math.min(1, s / 100));
  l = Math.max(0, Math.min(1, l / 100));

  c = (1 - Math.abs(2 * l - 1)) * s;
  x = c * (1 - Math.abs((h % 2) - 1));

  if(h < 1) {
    r = c;
    g = x;
    b = 0;
  } else if(h < 2) {
    r = x;
    g = c;
    b = 0;
  } else if(h < 3) {
    r = 0;
    g = c;
    b = x;
  } else if(h < 4) {
    r = 0;
    g = x;
    b = c;
  } else if(h < 5) {
    r = x;
    g = 0;
    b = c;
  } else {
    r = c;
    g = 0;
    b = x;
  }

  m = l - c / 2;
  r = Math.round((r + m) * 255);
  g = Math.round((g + m) * 255);
  b = Math.round((b + m) * 255);
  a = Math.round(a * 255);

  return new (Object.isFrozen(this) ? ImmutableRGBA : RGBA)(r, g, b, a);
};

HSLA.prototype.toString = function(prec = 1 / 255) {
  const h = Util.roundTo(this.h, 360 * prec, 3);
  const s = Util.roundTo(this.s, 100 * prec, 2);
  const l = Util.roundTo(this.l, 100 * prec, 2);
  const a = Util.roundTo(this.a, 1 * prec, 4);

  if(this.a == 1) return `hsl(${(h + '').padStart(3, ' ')},${(s + '%').padStart(4, ' ')},${(l + '%').padEnd(6, ' ')})`;
  return `hsla(${h},${s}%,${l}%,${a})`;
};
HSLA.prototype.toSource = function(prec = 1 / 255) {
  const h = Util.roundTo(this.h, 360 * prec, 0);
  const s = Util.roundTo(this.s, 100 * prec, 2);
  const l = Util.roundTo(this.l, 100 * prec, 2);
  const a = Util.roundTo(this.a, 1 * prec, 4);

  return `new HSLA(${(this.a == 1 ? [h, s, l] : [h, s, l, a]).join(', ')})`;
};

HSLA.fromString = str => {
  let c = Util.tryCatch(
    () => new RGBA(str),
    c => (c.valid() ? c : null),
    () => undefined
  );
  if(!c)
    c = Util.tryCatch(
      () => new HSLA(str),
      c => (c.valid() ? c : null),
      () => undefined
    );
  return c;
};

HSLA.prototype.valid = function() {
  const { h, s, l, a } = this;
  return [h, s, l, a].every(n => !isNaN(n) && typeof n == 'number');
};
HSLA.random = function(h = [0, 360], s = [0, 100], l = [0, 100], a = [0, 1], rng = Util.rng) {
  return new HSLA(Util.randInt(...[...h, 360].slice(0, 2), rng), Util.randInt(...[...s, 100].slice(0, 2), rng), Util.randInt(...[...l, 50].slice(0, 2), rng), Util.randFloat(...a, rng));
};
HSLA.prototype.dump = function() {
  //console.log(`[%c    %c]`, `background: ${this.toString()};`, `background: none`, this);
  return this;
};
HSLA.prototype.binaryValue = function() {
  const { h, s, l, a } = this;
  const byte = (() => {
    const clamp = Util.clamp(0, 255);
    return (val, range = 255) => clamp((val * 255) / range) % 256;
  })();

  return ((byte(h, 360) * 256 + byte(s)) * 256 + byte(l)) * 256 + byte(a, 1);
  //console.log(`[%c    %c]`, `background: ${this.toString()};`, `background: none`, this);
  return this;
};
HSLA.prototype.toObject = function() {
  const [h, s, l, a] = HSLA.prototype.toArray.call(this);
  return { h, s, l, a };
};
HSLA.prototype.toArray = function() {
  return Array.from(HSLA.prototype.round.call(HSLA.prototype.clamp.call(this)));
};

HSLA.prototype.equals = function(other) {
  const { h, s, l, a } = this;
  return h == other.h && s == other.s && l == other.l && a == other.a;
};
HSLA.prototype.compareTo = function(other) {
  let d = HSLA.prototype.binaryValue.call(other) - HSLA.prototype.binaryValue.call(this);
  return d < 0 ? -1 : d > 0 ? 1 : 0;
};
HSLA.prototype.toAnsi256 = function() {
  const rgba = HSLA.prototype.toRGBA.call(this);
  return RGBA.prototype.toAnsi256.call(rgba);
};
HSLA.prototype.toConsole = function(fn = 'toString') {
  const textColor = this.toRGBA().invert().blackwhite();
  const bgColor = this;
  return [
    `%c${this[fn]()}%c`,
    `text-shadow: 1px 1px 1px ${bgColor.toString()}; border: 1px solid black; padding: 2px; background-color: ${this.toString()}; color: ${textColor};`,
    `background-color: none;`
  ];
};
HSLA.prototype[Symbol.iterator] = function() {
  const { h, s, l, a } = this;
  return [h, s, l, a][Symbol.iterator]();
};

HSLA.prototype[Symbol.for('nodejs.util.inspect.custom')] = HSLA.prototype.inspect = function(options = {}) {
  const { colors = true } = options;
  const { h, s, l, a } = this;
  const haveAlpha = !isNaN(a) && a !== 1;
  let arr = haveAlpha ? [h, s, l, a] : [h, s, l];
  let ret = arr.map((n, i) => (Util.roundTo(n, i == 3 ? 1 / 255 : i == 0 ? 1 : 100 / 255, 2) + '').padStart(i < 3 ? 3 : 2, ' ')).join(', ');
  const color = this.toRGBA().toAnsi(/*256*/ true);
  let o = '';
  let c = colors ? (str, ...a) => `\x1b[${a.join(';')}m${str}\x1b[0m` : str => str;

  o += arr.map(n => c(n, 0, 33)).join('');
  o = color + o;

  return c('HSLA', 1, 31) + c(`(${ret})`.padEnd(24, ' '), 1, 36) + ` ${color}    \x1b[0m`;
};

HSLA.blend = (a, b, o = 0.5) => {
  a = new HSLA(a);
  b = new HSLA(b);
  return new HSLA(Math.round(a.h * (1 - o) + b.h * o), Math.round(a.s * (1 - o) + b.s * o), Math.round(a.l * (1 - o) + b.l * o), Math.round(a.a * (1 - o) + b.a * o));
};

for(let name of ['css', 'toHSL', 'clamp', 'round', 'hex', 'toRGBA', 'toString']) {
  HSLA[name] = hsla => HSLA.prototype[name].call(hsla || new HSLA());
}

const isHSLA = obj => HSLA.properties.every(prop => obj.hasOwnProperty(prop));

Util.defineGetter(HSLA, Symbol.species, function() {
  return this;
});

const ImmutableHSLA = Util.immutableClass(HSLA);
Util.defineGetter(ImmutableHSLA, Symbol.species, () => ImmutableHSLA);

/* ---------------------- end of './lib/color/hsla.js' ---------------------- */

/* ----------------- start of './lib/color/coloredText.js' ------------------ */

const FOREGROUND = Symbol.for('foreground');
const BACKGROUND = Symbol.for('background');
const NO_COLOR = Symbol.for('no-color');
const INSPECT = Symbol.for('nodejs.util.inspect.custom');
const TO_STRING = Symbol.toStringTag;

class ColoredText extends Array {
  static FG = FOREGROUND;
  static BG = BACKGROUND;
  static NC = NO_COLOR;

  static get [Symbol.species]() {
    return ColoredText;
  }

  constructor(...args) {
    super();
    Util.define(this, { current: { [FOREGROUND]: null, [BACKGROUND]: null } });

    const { FG, BG, NC } = ColoredText;

    for(let arg of args) {
      if(arg === NC) {
        this.current[FG] = null;
        this.current[BG] = null;
      } else if(Util.isObject(arg)) {
        if(arg[FG] !== undefined) this.current[FG] = arg[FG] || null;
        if(arg[BG] !== undefined) this.current[BG] = arg[BG] || null;
      }
      this.push(arg);
    }
  }

  write(text, fg, bg) {
    if(fg) this.setForeground(fg);
    if(bg) this.setBackground(bg);
    this.push(text);
  }

  append(text, fg, bg) {
    return this.write(text, fg, bg);
  }

  setForeground(color) {
    if(color instanceof Array) color = new RGBA(...color);
    const last = this[this.length - 1];
    if(Util.isObject(last)) last[FOREGROUND] = color;
    else this.push({ [FOREGROUND]: color });
  }

  setBackground(color) {
    if(color instanceof Array) color = new RGBA(...color);
    const last = this[this.length - 1];
    if(Util.isObject(last)) last[BACKGROUND] = color;
    else this.push({ [BACKGROUND]: color });
  }

  getForeground() {
    const { current } = this;
    return current[FOREGROUND];
  }

  getBackground() {
    const { current } = this;
    return current[BACKGROUND];
  }

  unshift(...args) {
    Array.prototype.splice.call(this, 0, 0, ...args);
    return this;
  }

  push(...args) {
    for(let arg of args) Array.prototype.push.call(this, arg);
    return this;
  }

  pop(n = 1) {
    let r = Array.prototype.splice.call(this, this.length - n, n);
    return n > 1 ? r : r[0];
  }

  shift(n = 1) {
    let r = Array.prototype.splice.call(this, 0, n);
    return n > 1 ? r : r[0];
  }

  setColors(fg, bg) {
    const { current } = this;
    let o = {};
    const { FG, BG } = ColoredText;

    if(this.getForeground() !== fg) current[FG] = o[FG] = fg;
    else fg = null;

    if(this.getBackground() !== bg) current[BG] = o[BG] = bg;
    else bg = null;

    if(fg || bg) this.push(o);
  }

  clearColors() {
    this.push(NO_COLOR);
  }

  stripColors() {
    return this.filter(p => !Util.isObject(p) && p !== NO_COLOR);
  }

  output() {
    const a = this[Symbol.for('nodejs.util.inspect.custom')]();
    console.log(...a);
  }

  toArray() {
    let a = Util.isBrowser() ? this.toConsole() : this.toAnsi256();

    return a;
  }

  toString(color = true) {
    let a = this;

    if(!color || Util.isBrowser()) a = a.stripColors();

    a = Util.isBrowser() ? a : a.toAnsi256();
    return a.join('');
  }

  valueOf(color = true) {
    return this.toString(color);
  }

  [Symbol.for('nodejs.util.inspect.custom')]() {
    return Util.isBrowser() ? this.toConsole() : this.toAnsi256();
  }

  toConsole() {
    const { FG, BG, NC } = ColoredText;
    let a = [];
    let t = '';
    let state = { [FG]: null, [BG]: null };
    for(let p of this) {
      if(p === NC) {
        t += '%c';
        a.push('color:none; background-color: none;');
        state[FG] = null;
        state[BG] = null;
      } else if(Util.isObject(p)) {
        const fg = p[FG];
        const bg = p[BG];
        let css = [];
        if(fg || bg) {
          t += '%c';
          if(fg) {
            css.push(`color:${fg.toString()};`);
            state[FG] = fg;
          }
          if(bg) {
            css.push(`background-color:${bg.toString()};`);
            state[BG] = bg;
          }
        }
        if(css.length) a.push(css.join(' '));
      } else {
        t += p;
      }
    }
    a.unshift(t);
    if(state[FG] !== null || state[BG] !== null) {
      this.push(NC);
      return this.toConsole();
    }
    Object.assign(a, {
      append(...args) {
        let s = '',
          v = [],
          i = 0;
        for(let a of args) {
          if(i == 0 || /%c/.test(a)) s += a;
          else v.push(a);
          i++;
        }
        this[0] += ' ' + s;
        if(v.length) Array.prototype.splice.call(this, this.length, 0, ...v);
        return this;
      },
      prepend(...args) {
        let s = '',
          v = [],
          i = 0;
        for(let a of args) {
          if(i == 0 || /%c/.test(a)) s += a;
          else v.push(a);
          i++;
        }
        this[0] = s + this[0];
        if(v.length) Array.prototype.splice.call(this, 1, 0, ...v);
        return this;
      },
      [INSPECT]() {
        return [...this]; //.reduce((a,p) => a ? [...a, ' ', p] : [p], []);
      },
      toConsole(c = console) {
        //throw new Error('coloredText.toConsole');
        c.log(...this[INSPECT]());
      }
    });
    return a;
  }

  toAnsi256() {
    const { FG, BG, NC } = ColoredText;
    let a = [];
    let state = { [FG]: null, [BG]: null };

    for(let p of this) {
      if(p === NC) {
        state[FG] = null;
        state[BG] = null;
      } else if(Util.isObject(p)) {
        if(p[FG] !== undefined) state[FG] = p[FG];
        if(p[BG] !== undefined) state[BG] = p[BG];
      }
      a.push(partToStr(p));
    }

    if(state[FG] !== null || state[BG] !== null) {
      this.push(NC);
      //  console.log("this:",[...this].map(partToStr).join("").split("\x1b"));
      return this.toAnsi256();
    }
    Object.assign(a, {
      append(...args) {
        for(let other of args) {
          if(Util.isArray(other)) {
            let i = 0;
            for(let arg of other) {
              this.push(arg);
              ++i;
            }
          } else {
            this.push(other);
          }
        }
        return this;
      },
      [INSPECT]() {
        return [this[TO_STRING]()];
      },
      [TO_STRING]() {
        let s = '';
        for(let p of [...this]) s += partToStr(p);
        return s + `\x1b[0m`;
      },
      toConsole(c = console) {
        c.log(...this[INSPECT]());
      }
    });

    function partToStr(p) {
      let s = '';
      if(typeof p == 'symbol' || p === NC) {
        s += `\x1b[0m`;
      } else if(Util.isObject(p)) {
        if(Util.isObject(p[FG]) && p[FG].toAnsi256) s += p[FG].toAnsi256(false);
        if(Util.isObject(p[BG]) && p[BG].toAnsi256) s += p[BG].toAnsi256(true);
      } else {
        s += p + '';
      }
      return s;
    }

    return a;
  }
}

/* ------------------ end of './lib/color/coloredText.js' ------------------- */

/* ------------------- start of './lib/color/convert.js' -------------------- */

/*
 * concatenated ../../../../../mnt/oldroot/home/roman/Sources/js-color/convert-colors/src/util.js
 */

/**
 * @private
 * @func rgb2hue
 * @desc Return a hue angle from an RGB color
 * @param {Number} r - Red (0 - 100)
 * @param {Number} g - Red (0 - 100)
 * @param {Number} b - Red (0 - 100)
 * @param {Number} f - Hue Fallback (0 - 360)
 * @return {Number} Hue Angle (0 - 360)
 * @example
 * rgb2hue(100, 0, 0)
 * @example
 * rgb2hue(100, 0, 0, 0)
 */

function rgb2hue(rgbR, rgbG, rgbB, fallbackhue = 0) {
  const value = rgb2value(rgbR, rgbG, rgbB);
  const whiteness = rgb2whiteness(rgbR, rgbG, rgbB);
  const delta = value - whiteness;

  if(delta) {
    // calculate segment
    const segment = value === rgbR ? (rgbG - rgbB) / delta : value === rgbG ? (rgbB - rgbR) / delta : (rgbR - rgbG) / delta;

    // calculate shift
    const shift = value === rgbR ? (segment < 0 ? 360 / 60 : 0 / 60) : value === rgbG ? 120 / 60 : 240 / 60;

    // calculate hue
    const hue = (segment + shift) * 60;

    return hue;
  } else {
    // otherwise return the Hue Fallback
    return fallbackhue;
  }
}
/**
 * @private
 * @func hue2rgb
 * @desc Return an RGB channel from a hue angle
 * @param {Number} t1
 * @param {Number} t2
 * @param {Number} h - Hue Angle (0 - 360)
 * @return {Number} RGB channel (0 - 100)
 * @example
 * hue2rgb(0, 0, 0)
 */

function hue2rgb(t1, t2, hue) {
  // calculate the ranged hue
  const rhue = hue < 0 ? hue + 360 : hue > 360 ? hue - 360 : hue;

  // calculate the rgb value
  const rgb = rhue * 6 < 360 ? t1 + ((t2 - t1) * rhue) / 60 : rhue * 2 < 360 ? t2 : rhue * 3 < 720 ? t1 + ((t2 - t1) * (240 - rhue)) / 60 : t1;

  return rgb;
}
/**
 * @private
 * @func luminance2contrast
 * @desc Return the contrast ratio between 2 luminance.
 * @param {Number} l1 - Relative luminance of one color
 * @param {Number} l2 - Relative luminance of another color
 * @return {Number} Contrast ratio between the 2 luminance
 * @example
 * luminance2contrast(0.2126, 0) // => 5.252
 * @link https://www.w3.org/TR/WCAG21/#dfn-contrast-ratio
 */

function luminance2contrast(relativeLuminance1, relativeLuminance2) {
  // l1 is the relative luminance of the lighter of the colors
  const l1 = max(relativeLuminance1, relativeLuminance2);

  // l1 is the relative luminance of the darker of the colors
  const l2 = min(relativeLuminance1, relativeLuminance2);

  return (l1 * precision + 0.05 * precision) / (l2 * precision + 0.05 * precision);
}
/* RGB tooling
/* ========================================================================== */

function rgb2value(rgbR, rgbG, rgbB) {
  const value = max(rgbR, rgbG, rgbB);
  return value;
}

function rgb2whiteness(rgbR, rgbG, rgbB) {
  const whiteness = min(rgbR, rgbG, rgbB);
  return whiteness;
}
/* Math matrix
/* ========================================================================== */

function matrix(params, mats) {
  return mats.map(mat =>
    mat.reduce(
      // (acc, value, index) => acc + params[index] * value,
      (acc, value, index) => acc + (params[index] * precision * (value * precision)) / precision / precision,
      0
    )
  );
}
/* Precision
/* ========================================================================== */

const precision = 100000000; /* D50 reference white
/* ========================================================================== */

const [wd50X, wd50Y, wd50Z] = [96.42, 100, 82.49]; /* Math Expressions
/* ========================================================================== */

const atan2d = (y, x) => rad2deg(atan2(y, x)); // arc-tangent in degrees

const cosd = x => cos(deg2rad(x)); // cosine of the specified angle in degrees

const deg2rad = x => (x * PI) / 180;

const rad2deg = x => (x * 180) / PI;

const sind = x => sin(deg2rad(x)); // sine in degrees

/* Math Constants
/* ========================================================================== */

const abs = Math.abs;

const atan2 = Math.atan2;

const cbrt = Math.cbrt;

const cos = Math.cos;

const exp = Math.exp;

const floor = Math.floor;

const max = Math.max;

const min = Math.min;

const PI = Math.PI;

const pow = Math.pow;

const sin = Math.sin;

const sqrt = Math.sqrt;

const epsilon = pow(6, 3) / pow(29, 3);

const kappa = pow(29, 3) / pow(3, 3);

/*
 * concatenated ../../../../../mnt/oldroot/home/roman/Sources/js-color/convert-colors/src/rgb-hsl.js
 */

/**
 * @func rgb2hsl
 * @desc Return a HSL color from an RGB color
 * @param {Number} r - red (0 - 100)
 * @param {Number} g - green (0 - 100)
 * @param {Number} b - blue (0 - 100)
 * @param {Number=} f - Hue Fallback (0 - 360)
 * @return {ArrayHSL}
 * @example
 * rgb2hsl(0, 100, 100) // => [0, 100, 50]
 * @link https://www.w3.org/TR/css-color-3/#hsl-color
 * @link https://www.w3.org/TR/css-color-4/#hsl-to-rgb
 * @link https://www.rapidtables.com/convert/color/rgb-to-hsl.html
 * @link https://www.rapidtables.com/convert/color/hsl-to-rgb.html
 */

function rgb2hsl(rgbR, rgbG, rgbB, fallbackhue) {
  const hslH = rgb2hue(rgbR, rgbG, rgbB, fallbackhue);
  const hslV = rgb2value(rgbR, rgbG, rgbB);
  const hslW = rgb2whiteness(rgbR, rgbG, rgbB);

  // calculate value/whiteness delta
  const hslD = hslV - hslW;

  // calculate lightness
  const hslL = (hslV + hslW) / 2;

  // calculate saturation
  const hslS = hslD === 0 ? 0 : (hslD / (100 - abs(2 * hslL - 100))) * 100;

  return [hslH, hslS, hslL];
}
/**
 * @func hsl2rgb
 * @desc Return an RGB color from an HSL color
 * @param {Number} h - Hue Angle (0 - 360)
 * @param {Number} s - Saturation (0 - 100)
 * @param {Number} l - Lightness (0 - 100)
 * @return {ArrayRGB}
 * @example
 * hsl2rgb(0, 100, 50) // => [0, 100, 100]
 * @link https://www.w3.org/TR/css-color-3/#hsl-color
 * @link https://www.w3.org/TR/css-color-4/#hsl-to-rgb
 * @link https://www.rapidtables.com/convert/color/rgb-to-hsl.html
 * @link https://www.rapidtables.com/convert/color/hsl-to-rgb.html
 */

function hsl2rgb(hslH, hslS, hslL) {
  // calcuate t2
  const t2 = hslL <= 50 ? (hslL * (hslS + 100)) / 100 : hslL + hslS - (hslL * hslS) / 100;

  // calcuate t1
  const t1 = hslL * 2 - t2;

  // calculate rgb
  const [rgbR, rgbG, rgbB] = [hue2rgb(t1, t2, hslH + 120), hue2rgb(t1, t2, hslH), hue2rgb(t1, t2, hslH - 120)];

  return [rgbR, rgbG, rgbB];
}

/*
 * concatenated ../../../../../mnt/oldroot/home/roman/Sources/js-color/convert-colors/src/rgb-hwb.js
 */

/**
 * @func rgb2hwb
 * @desc Return an HWB color from an RGB color
 * @param {Number} r - Red (0 - 100)
 * @param {Number} g - Green (0 - 100)
 * @param {Number} b - Blue (0 - 100)
 * @param {Number} f - Hue Fallback (0 - 360)
 * @return {ArrayHWB}
 * @example
 * rgb2hwb(100, 0, 0) // => [0, 0, 0]
 * @link https://www.w3.org/TR/css-color-4/#hwb-to-rgb
 * @link http://alvyray.com/Papers/CG/hwb2rgb.htm
 */

function rgb2hwb(rgbR, rgbG, rgbB, fallbackhue) {
  const hwbH = rgb2hue(rgbR, rgbG, rgbB, fallbackhue);
  const hwbW = rgb2whiteness(rgbR, rgbG, rgbB);
  const hwbV = rgb2value(rgbR, rgbG, rgbB);
  const hwbB = 100 - hwbV;
  return [hwbH, hwbW, hwbB];
}
/**
 * @func hwb2rgb
 * @desc Return an RGB color from an HWB color
 * @param {Number} h - Hue Angle (0 - 360)
 * @param {Number} w - Whiteness (0 - 100)
 * @param {Number} b - Blackness (0 - 100)
 * @return {ArrayRGB}
 * @example
 * hwb2rgb(0, 0, 0) // => [100, 0, 0]
 * @link https://www.w3.org/TR/css-color-4/#hwb-to-rgb
 * @link http://alvyray.com/Papers/CG/hwb2rgb.htm
 */

function hwb2rgb(hwbH, hwbW, hwbB, fallbackhue) {
  const [rgbR, rgbG, rgbB] = hsl2rgb(hwbH, 100, 50, fallbackhue).map(v => (v * (100 - hwbW - hwbB)) / 100 + hwbW);
  return [rgbR, rgbG, rgbB];
}

/*
 * concatenated ../../../../../mnt/oldroot/home/roman/Sources/js-color/convert-colors/src/rgb-hsv.js
 */

/**
 * @func rgb2hsv
 * @desc Return an HSV color from an RGB color
 * @param {Number} h - Hue Angle (0 - 360)
 * @param {Number} s - Saturation (0 - 100)
 * @param {Number} v - Value (0 - 100)
 * @param {Number=} f - Hue Fallback (0 - 360)
 * @return {ArrayHSV}
 * @example
 * rgb2hsv(100, 0, 0) // => [0, 100, 100]
 * @link http://alvyray.com/Papers/CG/hsv2rgb.htm
 */

function rgb2hsv(rgbR, rgbG, rgbB, fallbackhue) {
  const hsvV = rgb2value(rgbR, rgbG, rgbB);
  const hsvW = rgb2whiteness(rgbR, rgbG, rgbB);
  const hsvH = rgb2hue(rgbR, rgbG, rgbB, fallbackhue);

  // calculate saturation
  const hsvS = hsvV === hsvW ? 0 : ((hsvV - hsvW) / hsvV) * 100;

  return [hsvH, hsvS, hsvV];
}
/**
 * @func hsv2rgb
 * @desc Return an RGB color from an HSV color
 * @param {Number} h - Hue Angle (0 - 360)
 * @param {Number} s - Saturation (0 - 100)
 * @param {Number} v - Value (0 - 100)
 * @return {ArrayRGB}
 * @example
 * hsv2rgb(100, 0, 0) // => [100, 0, 0]
 * @link http://alvyray.com/Papers/CG/hsv2rgb.htm
 */

function hsv2rgb(hsvH, hsvS, hsvV) {
  const rgbI = floor(hsvH / 60);

  // calculate rgb parts
  const rgbF = (hsvH / 60 - rgbI) & 1 ? hsvH / 60 - rgbI : 1 - hsvH / 60 - rgbI;

  const rgbM = (hsvV * (100 - hsvS)) / 100;
  const rgbN = (hsvV * (100 - hsvS * rgbF)) / 100;
  const rgbT = (hsvV * (100 - ((100 - rgbF) * hsvS) / 100)) / 100;
  const [rgbR, rgbG, rgbB] =
    rgbI === 5 ? [hsvV, rgbM, rgbN] : rgbI === 4 ? [rgbT, rgbM, hsvV] : rgbI === 3 ? [rgbM, rgbN, hsvV] : rgbI === 2 ? [rgbM, hsvV, rgbT] : rgbI === 1 ? [rgbN, hsvV, rgbM] : [hsvV, rgbT, rgbM];
  return [rgbR, rgbG, rgbB];
}

/*
 * concatenated ../../../../../mnt/oldroot/home/roman/Sources/js-color/convert-colors/src/rgb-xyz.js
 */

/**
 * @func rgb2xyz
 * @desc Return an XYZ color from an RGB color
 * @param {Number} r - Red (0 - 100)
 * @param {Number} g - Green (0 - 100)
 * @param {Number} b - Blue (0 - 100)
 * @return {ArrayXYZ}
 * @example
 * rgb2xyz(100, 0, 0) // => [41.25, 21.27, 1.93]
 * @link https://www.w3.org/TR/css-color-4/#rgb-to-lab
 * @link https://www.w3.org/TR/css-color-4/#color-conversion-code
 */

function rgb2xyz(rgbR, rgbG, rgbB) {
  const [lrgbR, lrgbB, lrgbG] = [rgbR, rgbG, rgbB].map(v => (v > 4.045 ? pow((v + 5.5) / 105.5, 2.4) * 100 : v / 12.92));
  const [xyzX, xyzY, xyzZ] = matrix(
    [lrgbR, lrgbB, lrgbG],
    [
      [0.4124564, 0.3575761, 0.1804375],
      [0.2126729, 0.7151522, 0.072175],
      [0.0193339, 0.119192, 0.9503041]
    ]
  );
  return [xyzX, xyzY, xyzZ];
}
/**
 * @func xyz2rgb
 * @desc Return an XYZ color from an RGB color
 * @param {Number} x - Chromaticity of X
 * @param {Number} y - Chromaticity of Y
 * @param {Number} z - Chromaticity of Z
 * @return {ArrayRGB}
 * @example
 * xyz2rgb(41.25, 21.27, 1.93) // => [100, 0, 0]
 * @link https://www.w3.org/TR/css-color-4/#rgb-to-lab
 * @link https://www.w3.org/TR/css-color-4/#color-conversion-code
 */

function xyz2rgb(xyzX, xyzY, xyzZ) {
  const [lrgbR, lrgbB, lrgbG] = matrix(
    [xyzX, xyzY, xyzZ],
    [
      [3.2404542, -1.5371385, -0.4985314],
      [-0.969266, 1.8760108, 0.041556],
      [0.0556434, -0.2040259, 1.0572252]
    ]
  );
  const [rgbR, rgbG, rgbB] = [lrgbR, lrgbB, lrgbG].map(v => (v > 0.31308 ? 1.055 * pow(v / 100, 1 / 2.4) * 100 - 5.5 : 12.92 * v));
  return [rgbR, rgbG, rgbB];
}

/*
 * concatenated ../../../../../mnt/oldroot/home/roman/Sources/js-color/convert-colors/src/hsl-hsv.js
 */

/**
 * @func hsl2hsv
 * @desc Return an HSV color from an HSL color
 * @param {Number} h - Hue Angle (0 - 360)
 * @param {Number} s - Saturation (0 - 100)
 * @param {Number} l - Lightness (0 - 100)
 * @return {ArrayHSV}
 * @example
 * hsl2hsv(0, 100, 50)
 * @link https://gist.github.com/defims/0ca2ef8832833186ed396a2f8a204117
 */

function hsl2hsv(hslH, hslS, hslL) {
  const hsv1 = (hslS * (hslL < 50 ? hslL : 100 - hslL)) / 100;
  const hsvS = hsv1 === 0 ? 0 : ((2 * hsv1) / (hslL + hsv1)) * 100;
  const hsvV = hslL + hsv1;
  return [hslH, hsvS, hsvV];
}
/**
 * @func hsv2hsl
 * @desc Return an HSL color from an HSV color
 * @param {Number} h - Hue Angle (0 - 360)
 * @param {Number} s - Saturation (0 - 100)
 * @param {Number} v - Value (0 - 100)
 * @return {ArrayHSL}
 * @example
 * hsv2hsl(0, 0, 0) // => [0, 100, 50]
 * @link https://gist.github.com/defims/0ca2ef8832833186ed396a2f8a204117
 */

function hsv2hsl(hsvH, hsvS, hsvV) {
  const hslL = ((200 - hsvS) * hsvV) / 100;
  const [hslS, hslV] = [hslL === 0 || hslL === 200 ? 0 : ((hsvS * hsvV) / 100 / (hslL <= 100 ? hslL : 200 - hslL)) * 100, (hslL * 5) / 10];
  return [hsvH, hslS, hslV];
}

/*
 * concatenated ../../../../../mnt/oldroot/home/roman/Sources/js-color/convert-colors/src/hwb-hsv.js
 */

/**
 * @func hwb2hsv
 * @desc Return an HSV color from an HWB color
 * @param {Number} h - Hue Angle (0 - 360)
 * @param {Number} w - Whiteness (0 - 100)
 * @param {Number} b - Blackness (0 - 100)
 * @return {ArrayHSV}
 * @example
 * hwb2hsv(0, 0, 0) // => [0, 100, 100]
 * @link https://en.wikipedia.org/wiki/HWB_color_model#Converting_to_and_from_HSV
 */

function hwb2hsv(hwbH, hwbW, hwbB) {
  const [hsvH, hsvS, hsvV] = [hwbH, hwbB === 100 ? 0 : 100 - (hwbW / (100 - hwbB)) * 100, 100 - hwbB];
  return [hsvH, hsvS, hsvV];
}
/**
 * @func hsv2hwb
 * @desc Return an HWB color from an HSV color
 * @param {Number} h - Hue Angle (0 - 360)
 * @param {Number} s - Saturation (0 - 100)
 * @param {Number} v - Value (0 - 100)
 * @return {ArrayHWB}
 * @example
 * hsv2hwb(0, 100, 100) // => [0, 0, 0]
 * @link https://en.wikipedia.org/wiki/HWB_color_model#Converting_to_and_from_HSV
 */

function hsv2hwb(hsvH, hsvS, hsvV) {
  const [hwbH, hwbW, hwbB] = [hsvH, ((100 - hsvS) * hsvV) / 100, 100 - hsvV];
  return [hwbH, hwbW, hwbB];
}

/*
 * concatenated ../../../../../mnt/oldroot/home/roman/Sources/js-color/convert-colors/src/lab-xyz.js
 */

/**
 * @func lab2xyz
 * @desc Return an XYZ color from a LAB color
 * @param {Number} l - CIE Lightness
 * @param {Number} a - Red/Green Coordinate
 * @param {Number} b - Yellow/Blue Coordinate
 * @return {ArrayXYZ}
 * @example
 * lab2xyz(54.29, 80.82, 69.88) // => 41.25, 21.27, 1.93
 * @link https://www.w3.org/TR/css-color-4/#rgb-to-lab
 * @link https://www.w3.org/TR/css-color-4/#color-conversion-code
 * @link https://www.easyrgb.com/en/math.php
 */

function lab2xyz(labL, labA, labB) {
  // compute f, starting with the luminance-related term
  const f2 = (labL + 16) / 116;

  const f1 = labA / 500 + f2;
  const f3 = f2 - labB / 200;

  // compute pre-scaled XYZ
  const [initX, initY, initZ] = [
    pow(f1, 3) > epsilon ? pow(f1, 3) : (116 * f1 - 16) / kappa,
    labL > kappa * epsilon ? pow((labL + 16) / 116, 3) : labL / kappa,
    pow(f3, 3) > epsilon ? pow(f3, 3) : (116 * f3 - 16) / kappa
  ];

  const [xyzX, xyzY, xyzZ] = matrix(
    // compute XYZ by scaling pre-scaled XYZ by reference white
    [initX * wd50X, initY * wd50Y, initZ * wd50Z], // calculate D65 XYZ from D50 XYZ
    [
      [0.9555766, -0.0230393, 0.0631636],
      [-0.0282895, 1.0099416, 0.0210077],
      [0.0122982, -0.020483, 1.3299098]
    ]
  );

  return [xyzX, xyzY, xyzZ];
}
/**
 * @func xyz2lab
 * @desc Return an LAB color from a XYZ color
 * @param {Number} x - Chromaticity of X
 * @param {Number} y - Chromaticity of Y
 * @param {Number} z - Chromaticity of Z
 * @return {ArrayLAB}
 * @example
 * xyz2lab(41.25, 21.27, 1.93) // => [54.29, 80.82, 69.88]
 * @link https://www.w3.org/TR/css-color-4/#rgb-to-lab
 * @link https://www.w3.org/TR/css-color-4/#color-conversion-code
 * @link https://www.easyrgb.com/en/math.php
 */

function xyz2lab(xyzX, xyzY, xyzZ) {
  // calculate D50 XYZ from D65 XYZ
  const [d50X, d50Y, d50Z] = matrix(
    [xyzX, xyzY, xyzZ],
    [
      [1.0478112, 0.0228866, -0.050127],
      [0.0295424, 0.9904844, -0.0170491],
      [-0.0092345, 0.0150436, 0.7521316]
    ]
  );

  // calculate f
  const [f1, f2, f3] = [d50X / wd50X, d50Y / wd50Y, d50Z / wd50Z].map(value => (value > epsilon ? cbrt(value) : (kappa * value + 16) / 116));

  const [labL, labA, labB] = [116 * f2 - 16, 500 * (f1 - f2), 200 * (f2 - f3)];
  return [labL, labA, labB];
}

/*
 * concatenated ../../../../../mnt/oldroot/home/roman/Sources/js-color/convert-colors/src/lab-lch.js
 */

/**
 * @func lab2lch
 * @desc Return an LCH color from a LAB color
 * @param {Number} l - CIE Lightness
 * @param {Number} a - Red/Green Coordinate
 * @param {Number} b - Yellow/Blue Coordinate
 * @return {ArrayLAB}
 * @example
 * lab2lch(54.29, 80.82, 69.88) // => [54.29, 106.84, 40.85]
 * @link https://www.w3.org/TR/css-color-4/#color-conversion-code
 * @link https://www.w3.org/TR/css-color-4/#lch-to-lab
 */

function lab2lch(labL, labA, labB) {
  const [lchC, lchH] = [
    sqrt(pow(labA, 2) + pow(labB, 2)), // convert to chroma
    rad2deg(atan2(labB, labA))
  ];

  // convert to hue, in degrees
  return [labL, lchC, lchH];
}
/**
 * @func lch2lab
 * @desc Return a LAB color from an LCH color
 * @param {Number} l - CIE Lightness
 * @param {Number} c - CIE Chroma
 * @param {Number} h - CIE Hue Angle
 * @return {ArrayLCH}
 * @example
 * lch2lab(54.29, 106.84, 40.85) // => [54.29, 80.82, 69.88]
 * @link https://www.w3.org/TR/css-color-4/#color-conversion-code
 * @link https://www.w3.org/TR/css-color-4/#lch-to-lab
 */

function lch2lab(lchL, lchC, lchH) {
  // convert to Lab a and b from the polar form
  const [labA, labB] = [lchC * cosd(lchH), lchC * sind(lchH)];

  return [lchL, labA, labB];
}

/*
 * concatenated ../../../../../mnt/oldroot/home/roman/Sources/js-color/convert-colors/src/rgb-contrast.js
 */

function rgb2contrast(rgb1, rgb2) {
  const luminance1 = rgb2luminance(...rgb1);
  const luminance2 = rgb2luminance(...rgb2);
  return luminance2contrast(luminance1, luminance2);
}
/**
 * @private
 * @func rgb2luminance
 * @desc Return the relative brightness of RGB
 * @param {Number} r - Red (0 - 100)
 * @param {Number} g - Green (0 - 100)
 * @param {Number} b - Blue (0 - 100)
 * @return {Number} Relative luminance of the color
 * @example
 * rgb2luminance(100, 0, 0) // => 0.2126
 * @link https://www.w3.org/TR/WCAG21/#dfn-relative-luminance
 */

function rgb2luminance(rgbR, rgbG, rgbB) {
  return (adjustChannel(rgbR) * coefficientR + adjustChannel(rgbG) * coefficientG + adjustChannel(rgbB) * coefficientB) / precision;
}
// low-gamma adjust coefficients
const adjustChannel = x => (x <= 3.928 ? x / lowc : adjustGamma(x));
const adjustGamma = x => pow((x + 5.5) / 105.5, 2.4);
const lowc = 1292; // red/green/blue coefficients
const coefficientR = 0.2126 * precision;
const coefficientG = 0.7152 * precision;
const coefficientB = 0.0722 * precision;

/*
 * concatenated ../../../../../mnt/oldroot/home/roman/Sources/js-color/convert-colors/src/hex-rgb.js
 */

/**
 * @func hex2rgb
 * @desc Return an RGBA color from a Hex color.
 * @param {StringHex} hex
 * @return {ArrayRGBA}
 * @example
 * hex2rgb("#f00") // => [100, 0, 0, 100]
 * hex2rgb("#f00f") // => [100, 0, 0, 100]
 * @example
 * hex2rgb("#ff0000") // => [100, 0, 0, 100]
 * hex2rgb("#ff0000ff") // => [100, 0, 0, 100]
 */

function hex2rgb(hex) {
  // #<hex-color>{3,4,6,8}
  const [, r, g, b, a, rr, gg, bb, aa] = hex.match(hexColorMatch) || [];

  if(rr !== undefined || r !== undefined) {
    const red = rr !== undefined ? parseInt(rr, 16) : parseInt(r + r, 16);
    const green = gg !== undefined ? parseInt(gg, 16) : parseInt(g + g, 16);
    const blue = bb !== undefined ? parseInt(bb, 16) : parseInt(b + b, 16);
    const alpha = aa !== undefined ? parseInt(aa, 16) : a !== undefined ? parseInt(a + a, 16) : 255;
    return [red, green, blue, alpha].map(c => (c * 100) / 255);
  }

  return undefined;
}
/**
 * @func rgb2hex
 * @desc Return a HEX color from an RGB color
 * @param {Number} r - Red (0 - 100)
 * @param {Number} g - Green (0 - 100)
 * @param {Number} b - Blue (0 - 100)
 * @return {StringHex}
 * @example
 * rgb2hex(100, 0, 0) // => "#ff0000"
 */

function rgb2hex(rgbR, rgbG, rgbB) {
  return `#${((1 << 24) + (Math.round((rgbR * 255) / 100) << 16) + (Math.round((rgbG * 255) / 100) << 8) + Math.round((rgbB * 255) / 100)).toString(16).slice(1)}`;
}

const hexColorMatch = /^#?(?:([a-f0-9])([a-f0-9])([a-f0-9])([a-f0-9])?|([a-f0-9]{2})([a-f0-9]{2})([a-f0-9]{2})([a-f0-9]{2})?)$/i;

/*
 * concatenated ../../../../../mnt/oldroot/home/roman/Sources/js-color/convert-colors/src/keyword-rgb.js
 */

/**
 * @func keyword2rgb
 * @desc Return an RGB color from a CSS keyword color
 * @param {StringKeyword} keyword
 * @return {ArrayRGB}
 * @example
 * keyword2rgb('red') // => [100, 0, 0]
 */

function keyword2rgb(keyword) {
  const rgb255 = keywords[String(keyword).toLowerCase()];
  return rgb255 ? rgb255.map(x => (x * 100) / 255) : null;
}

const keywords = {
  aliceblue: [240, 248, 255],
  antiquewhite: [250, 235, 215],
  aqua: [0, 255, 255],
  aquamarine: [127, 255, 212],
  azure: [240, 255, 255],
  beige: [245, 245, 220],
  bisque: [255, 228, 196],
  black: [0, 0, 0],
  blanchedalmond: [255, 235, 205],
  blue: [0, 0, 255],
  blueviolet: [138, 43, 226],
  brown: [165, 42, 42],
  burlywood: [222, 184, 135],
  cadetblue: [95, 158, 160],
  chartreuse: [127, 255, 0],
  chocolate: [210, 105, 30],
  coral: [255, 127, 80],
  cornflowerblue: [100, 149, 237],
  cornsilk: [255, 248, 220],
  crimson: [220, 20, 60],
  cyan: [0, 255, 255],
  darkblue: [0, 0, 139],
  darkcyan: [0, 139, 139],
  darkgoldenrod: [184, 134, 11],
  darkgray: [169, 169, 169],
  darkgreen: [0, 100, 0],
  darkgrey: [169, 169, 169],
  darkkhaki: [189, 183, 107],
  darkmagenta: [139, 0, 139],
  darkolivegreen: [85, 107, 47],
  darkorange: [255, 140, 0],
  darkorchid: [153, 50, 204],
  darkred: [139, 0, 0],
  darksalmon: [233, 150, 122],
  darkseagreen: [143, 188, 143],
  darkslateblue: [72, 61, 139],
  darkslategray: [47, 79, 79],
  darkslategrey: [47, 79, 79],
  darkturquoise: [0, 206, 209],
  darkviolet: [148, 0, 211],
  deeppink: [255, 20, 147],
  deepskyblue: [0, 191, 255],
  dimgray: [105, 105, 105],
  dimgrey: [105, 105, 105],
  dodgerblue: [30, 144, 255],
  firebrick: [178, 34, 34],
  floralwhite: [255, 250, 240],
  forestgreen: [34, 139, 34],
  fuchsia: [255, 0, 255],
  gainsboro: [220, 220, 220],
  ghostwhite: [248, 248, 255],
  gold: [255, 215, 0],
  goldenrod: [218, 165, 32],
  gray: [128, 128, 128],
  green: [0, 128, 0],
  greenyellow: [173, 255, 47],
  grey: [128, 128, 128],
  honeydew: [240, 255, 240],
  hotpink: [255, 105, 180],
  indianred: [205, 92, 92],
  indigo: [75, 0, 130],
  ivory: [255, 255, 240],
  khaki: [240, 230, 140],
  lavender: [230, 230, 250],
  lavenderblush: [255, 240, 245],
  lawngreen: [124, 252, 0],
  lemonchiffon: [255, 250, 205],
  lightblue: [173, 216, 230],
  lightcoral: [240, 128, 128],
  lightcyan: [224, 255, 255],
  lightgoldenrodyellow: [250, 250, 210],
  lightgray: [211, 211, 211],
  lightgreen: [144, 238, 144],
  lightgrey: [211, 211, 211],
  lightpink: [255, 182, 193],
  lightsalmon: [255, 160, 122],
  lightseagreen: [32, 178, 170],
  lightskyblue: [135, 206, 250],
  lightslategray: [119, 136, 153],
  lightslategrey: [119, 136, 153],
  lightsteelblue: [176, 196, 222],
  lightyellow: [255, 255, 224],
  lime: [0, 255, 0],
  limegreen: [50, 205, 50],
  linen: [250, 240, 230],
  magenta: [255, 0, 255],
  maroon: [128, 0, 0],
  mediumaquamarine: [102, 205, 170],
  mediumblue: [0, 0, 205],
  mediumorchid: [186, 85, 211],
  mediumpurple: [147, 112, 219],
  mediumseagreen: [60, 179, 113],
  mediumslateblue: [123, 104, 238],
  mediumspringgreen: [0, 250, 154],
  mediumturquoise: [72, 209, 204],
  mediumvioletred: [199, 21, 133],
  midnightblue: [25, 25, 112],
  mintcream: [245, 255, 250],
  mistyrose: [255, 228, 225],
  moccasin: [255, 228, 181],
  navajowhite: [255, 222, 173],
  navy: [0, 0, 128],
  oldlace: [253, 245, 230],
  olive: [128, 128, 0],
  olivedrab: [107, 142, 35],
  orange: [255, 165, 0],
  orangered: [255, 69, 0],
  orchid: [218, 112, 214],
  palegoldenrod: [238, 232, 170],
  palegreen: [152, 251, 152],
  paleturquoise: [175, 238, 238],
  palevioletred: [219, 112, 147],
  papayawhip: [255, 239, 213],
  peachpuff: [255, 218, 185],
  peru: [205, 133, 63],
  pink: [255, 192, 203],
  plum: [221, 160, 221],
  powderblue: [176, 224, 230],
  purple: [128, 0, 128],
  rebeccapurple: [102, 51, 153],
  red: [255, 0, 0],
  rosybrown: [188, 143, 143],
  royalblue: [65, 105, 225],
  saddlebrown: [139, 69, 19],
  salmon: [250, 128, 114],
  sandybrown: [244, 164, 96],
  seagreen: [46, 139, 87],
  seashell: [255, 245, 238],
  sienna: [160, 82, 45],
  silver: [192, 192, 192],
  skyblue: [135, 206, 235],
  slateblue: [106, 90, 205],
  slategray: [112, 128, 144],
  slategrey: [112, 128, 144],
  snow: [255, 250, 250],
  springgreen: [0, 255, 127],
  steelblue: [70, 130, 180],
  tan: [210, 180, 140],
  teal: [0, 128, 128],
  thistle: [216, 191, 216],
  tomato: [255, 99, 71],
  transparent: [0, 0, 0],
  turquoise: [64, 224, 208],
  violet: [238, 130, 238],
  wheat: [245, 222, 179],
  white: [255, 255, 255],
  whitesmoke: [245, 245, 245],
  yellow: [255, 255, 0],
  yellowgreen: [154, 205, 50]
};

/*
 * concatenated ../../../../../mnt/oldroot/home/roman/Sources/js-color/convert-colors/src/lab-ciede.js
 */

function lab2ciede([L1, a1, b1], [L2, a2, b2]) {
  const c1 = sqrt(pow(a1, 2) + pow(b1, 2));
  const c2 = sqrt(pow(a2, 2) + pow(b2, 2));
  const deltaLPrime = L2 - L1;
  const lBar = (L1 + L2) / 2;
  const cBar = (c1 + c2) / 2;
  const cBarPow7 = pow(cBar, 7);
  const cCoeff = sqrt(cBarPow7 / (cBarPow7 + pow(25, 7)));
  const a1Prime = a1 + (a1 / 2) * (1 - cCoeff);
  const a2Prime = a2 + (a2 / 2) * (1 - cCoeff);
  const c1Prime = sqrt(a1Prime * a1Prime + b1 * b1);
  const c2Prime = sqrt(a2Prime * a2Prime + b2 * b2);
  const cBarPrime = (c1Prime + c2Prime) / 2;
  const deltaCPrime = c2Prime - c1Prime;
  const h1Prime = a1Prime === 0 && b1 === 0 ? 0 : atan2d(b1, a1Prime) % 360;
  const h2Prime = a2Prime === 0 && b2 === 0 ? 0 : atan2d(b2, a2Prime) % 360;
  let deltaSmallHPrime;
  let deltaBigHPrime;
  let hBarPrime;

  if(c1Prime === 0 || c2Prime === 0) {
    deltaSmallHPrime = 0;
    deltaBigHPrime = 0;
    hBarPrime = h1Prime + h2Prime;
  } else {
    deltaSmallHPrime = abs(h1Prime - h2Prime) <= 180 ? h2Prime - h1Prime : h2Prime <= h1Prime ? h2Prime - h1Prime + 360 : h2Prime - h1Prime - 360;
    deltaBigHPrime = 2 * sqrt(c1Prime * c2Prime) * sind(deltaSmallHPrime / 2);
    hBarPrime = abs(h1Prime - h2Prime) <= 180 ? (h1Prime + h2Prime) / 2 : h1Prime + h2Prime < 360 ? (h1Prime + h2Prime + 360) / 2 : (h1Prime + h2Prime - 360) / 2;
  }

  const T =
    1 -
    0.17 * precision * cosd(hBarPrime - 30) +
    0.24 * precision * cosd(2 * hBarPrime) +
    0.32 * precision * cosd(3 * hBarPrime + 6) -
    (0.2 * precision * cosd(4 * hBarPrime - 63)) / precision / precision;
  const slCoeff = (lBar - 50) * (lBar - 50);
  const sl = 1 + (0.015 * precision * slCoeff) / sqrt(20 + slCoeff) / precision;
  const sc = 1 + (0.045 * precision * cBarPrime) / precision;
  const sh = 1 + (0.015 * precision * cBarPrime * T) / precision;
  const RtCoeff = 60 * exp(-((hBarPrime - 275) / 25) * ((hBarPrime - 275) / 25));
  const Rt = -2 * cCoeff * sind(RtCoeff);
  const term1 = deltaLPrime / (kl * sl);
  const term2 = deltaCPrime / (kc * sc);
  const term3 = deltaBigHPrime / (kh * sh);
  const term4 = Rt * term2 * term3;
  return sqrt(term1 * term1 + term2 * term2 + term3 * term3 + term4);
}
// weight factors
const kl = 1;
const kc = 1;
const kh = 1;

/*
 * concatenated ../../../../../mnt/oldroot/home/roman/Sources/js-color/convert-colors/src/index.js
 */

/* Convert between RGB and Lab
/* ========================================================================== */

/**
 * @func rgb2lab
 * @desc Return a CIE LAB color from an RGB color
 * @param {Number} r - Red (0 - 100)
 * @param {Number} g - Green (0 - 100)
 * @param {Number} b - Blue (0 - 100)
 * @return {ArrayLAB}
 * @example
 * rgb2lab(100, 0, 0) // => [54.29, 80.82, 69.88]
 */

function rgb2lab(rgbR, rgbG, rgbB) {
  const [xyzX, xyzY, xyzZ] = rgb2xyz(rgbR, rgbG, rgbB);
  const [labL, labA, labB] = xyz2lab(xyzX, xyzY, xyzZ);
  return [labL, labA, labB];
}
/**
 * @func lab2rgb
 * @desc Return an RGB color from a CIE LAB color
 * @param {Number} l - CIE Lightness
 * @param {Number} a - Red/Green Coordinate
 * @param {Number} b - Yellow/Blue Coordinate
 * @return {ArrayRGBA}
 * @example
 * lab2rgb(54.29, 80.82, 69.88) // => [100, 0, 0]
 */

function lab2rgb(labL, labA, labB) {
  const [xyzX, xyzY, xyzZ] = lab2xyz(labL, labA, labB);
  const [rgbR, rgbG, rgbB] = xyz2rgb(xyzX, xyzY, xyzZ);
  return [rgbR, rgbG, rgbB];
}
/* Convert between RGB and LCH
/* ========================================================================== */

/**
 * @func rgb2lch
 * @desc Return a CIE LAB color from an RGB color
 * @param {Number} r - Red (0 - 100)
 * @param {Number} g - Green (0 - 100)
 * @param {Number} b - Blue (0 - 100)
 * @return {ArrayLCH}
 * @example
 * rgb2lch(100, 0, 0) // => [54.29, 106.84, 40.85]
 */

function rgb2lch(rgbR, rgbG, rgbB) {
  const [xyzX, xyzY, xyzZ] = rgb2xyz(rgbR, rgbG, rgbB);
  const [labL, labA, labB] = xyz2lab(xyzX, xyzY, xyzZ);
  const [lchL, lchC, lchH] = lab2lch(labL, labA, labB);
  return [lchL, lchC, lchH];
}
/**
 * @func lch2rgb
 * @desc Return an RGB color from a CIE LCH color
 * @param {Number} l - CIE Lightness
 * @param {Number} c - CIE Chroma
 * @param {Number} h - CIE Hue
 * @return {ArrayRGBA}
 * @example
 * lch2rgb(54.29, 106.84, 40.85) // => [100, 0, 0]
 */

function lch2rgb(lchL, lchC, lchH) {
  const [labL, labA, labB] = lch2lab(lchL, lchC, lchH);
  const [xyzX, xyzY, xyzZ] = lab2xyz(labL, labA, labB);
  const [rgbR, rgbG, rgbB] = xyz2rgb(xyzX, xyzY, xyzZ);
  return [rgbR, rgbG, rgbB];
}
/* Convert between HSL and HWB
/* ========================================================================== */

/**
 * @func hwb2hsl
 * @desc Return an HSV color from an HWB color
 * @param {Number} h - Hue Angle (0 - 360)
 * @param {Number} w - Whiteness (0 - 100)
 * @param {Number} b - Blackness (0 - 100)
 * @return {ArrayHSL}
 * @example
 * hwb2hsl(0, 0, 0) // => [0, 0, 100]
 */

function hwb2hsl(hwbH, hwbW, hwbB) {
  const [hsvH, hsvS, hsvV] = hwb2hsv(hwbH, hwbW, hwbB);
  const [hslH, hslS, hslL] = hsv2hsl(hsvH, hsvS, hsvV);
  return [hslH, hslS, hslL];
}
/**
 * @func hsl2hwb
 * @desc Return an HWB color from an HSL color
 * @param {Number} h - Hue Angle (0 - 360)
 * @param {Number} s - Saturation (0 - 100)
 * @param {Number} l - Lightness (0 - 100)
 * @return {ArrayHWB}
 * @example
 * hsl2hwb(0, 0, 100) // => [0, 0, 0]
 */

function hsl2hwb(hslH, hslS, hslL) {
  const [, hsvS, hsvV] = hsl2hsv(hslH, hslS, hslL);
  const [, hwbW, hwbB] = hsv2hwb(hslH, hsvS, hsvV);
  return [hslH, hwbW, hwbB];
}
/* Convert between HSL and Lab
/* ========================================================================== */

/**
 * @func hsl2lab
 * @desc Return a CIE LAB color from an HSL color
 * @param {Number} h - Hue Angle (0 - 360)
 * @param {Number} s - Saturation (0 - 100)
 * @param {Number} l - Lightness (0 - 100)
 * @return {ArrayLAB}
 * @example
 * hsl2lab(0, 100, 50) // => [54.29, 80.82, 69.88]
 */

function hsl2lab(hslH, hslS, hslL) {
  const [rgbR, rgbG, rgbB] = hsl2rgb(hslH, hslS, hslL);
  const [xyzX, xyzY, xyzZ] = rgb2xyz(rgbR, rgbG, rgbB);
  const [labL, labA, labB] = xyz2lab(xyzX, xyzY, xyzZ);
  return [labL, labA, labB];
}
/**
 * @func lab2hsl
 * @desc Return a HSL color from a CIE LAB color
 * @param {Number} l - CIE Lightness
 * @param {Number} a - Red/Green Coordinate
 * @param {Number} b - Yellow/Blue Coordinate
 * @param {Number=} f - Hue Fallback (0 - 360)
 * @return {ArrayHSL}
 * @example
 * lab2hsl(54.29, 80.82, 69.88) // => [0, 100, 50]
 */

function lab2hsl(labL, labA, labB, fallbackhue) {
  const [xyzX, xyzY, xyzZ] = lab2xyz(labL, labA, labB);
  const [rgbR, rgbG, rgbB] = xyz2rgb(xyzX, xyzY, xyzZ);
  const [hslH, hslS, hslL] = rgb2hsl(rgbR, rgbG, rgbB, fallbackhue);
  return [hslH, hslS, hslL];
}
/* Convert between HSL and LCH
/* ========================================================================== */

/**
 * @func hsl2lch
 * @desc Return a CIE LCH color from an HSL color
 * @param {Number} h - Hue Angle (0 - 360)
 * @param {Number} s - Saturation (0 - 100)
 * @param {Number} l - Lightness (0 - 100)
 * @return {ArrayLCH}
 * @example
 * hsl2lch(0, 100, 50) // => [54.29, 106.84, 40.85]
 */

function hsl2lch(hslH, hslS, hslL) {
  const [rgbR, rgbG, rgbB] = hsl2rgb(hslH, hslS, hslL);
  const [xyzX, xyzY, xyzZ] = rgb2xyz(rgbR, rgbG, rgbB);
  const [labL, labA, labB] = xyz2lab(xyzX, xyzY, xyzZ);
  const [lchL, lchC, lchH] = lab2lch(labL, labA, labB);
  return [lchL, lchC, lchH];
}
/**
 * @func lch2hsl
 * @desc Return an HSL from a CIE LCH color
 * @param {Number} l - CIE Lightness
 * @param {Number} c - CIE Chroma
 * @param {Number} h - CIE Hue Angle
 * @return {ArrayLCH}
 * @example
 * lch2hsl(54.29, 106.84, 40.85) // => [0, 100, 50]
 */

function lch2hsl(lchL, lchC, lchH, fallbackhue) {
  const [labL, labA, labB] = lch2lab(lchL, lchC, lchH);
  const [xyzX, xyzY, xyzZ] = lab2xyz(labL, labA, labB);
  const [rgbR, rgbG, rgbB] = xyz2rgb(xyzX, xyzY, xyzZ);
  const [hslH, hslS, hslL] = rgb2hsl(rgbR, rgbG, rgbB, fallbackhue);
  return [hslH, hslS, hslL];
}
/* Convert between HSL and XYZ
/* ========================================================================== */

/**
 * @func hsl2xyz
 * @desc Return an XYZ color from an HSL color
 * @param {Number} h - Hue Angle (0 - 360)
 * @param {Number} s - Saturation (0 - 100)
 * @param {Number} l - Lightness (0 - 100)
 * @return {ArrayXYZ}
 * @example
 * hsl2xyz(0, 100, 50) // => [41.25, 21.27, 1.93]
 */

function hsl2xyz(hslH, hslS, hslL) {
  const [rgbR, rgbG, rgbB] = hsl2rgb(hslH, hslS, hslL);
  const [xyzX, xyzY, xyzZ] = rgb2xyz(rgbR, rgbG, rgbB);
  return [xyzX, xyzY, xyzZ];
}
/**
 * @func xyz2hsl
 * @desc Return an HSL color from an XYZ color
 * @param {Number} x - Chromaticity of X
 * @param {Number} y - Chromaticity of Y
 * @param {Number} z - Chromaticity of Z
 * @return {ArrayHSL}
 * @example
 * xyz2hsl(0, 100, 50) // => [41.25, 21.27, 1.93]
 */

function xyz2hsl(xyzX, xyzY, xyzZ, fallbackhue) {
  const [rgbR, rgbG, rgbB] = xyz2rgb(xyzX, xyzY, xyzZ);
  const [hslH, hslS, hslL] = rgb2hsl(rgbR, rgbG, rgbB, fallbackhue);
  return [hslH, hslS, hslL];
}
/* Convert between HWB and Lab
/* ========================================================================== */

/**
 * @func hwb2lab
 * @desc Return a CIE LAB color from an HWB color
 * @param {Number} h - Hue Angle (0 - 360)
 * @param {Number} w - Whiteness (0 - 100)
 * @param {Number} b - Blackness (0 - 100)
 * @return {ArrayLAB}
 * @example
 * hwb2lab(0, 0, 0) // => [54.29, 80.82, 69.88]
 */

function hwb2lab(hwbH, hwbW, hwbB) {
  const [rgbR, rgbG, rgbB] = hwb2rgb(hwbH, hwbW, hwbB);
  const [xyzX, xyzY, xyzZ] = rgb2xyz(rgbR, rgbG, rgbB);
  const [labL, labA, labB] = xyz2lab(xyzX, xyzY, xyzZ);
  return [labL, labA, labB];
}
/**
 * @func lab2hwb
 * @desc Return an HWB color from a CIE LAB color
 * @param {Number} l - CIE Lightness
 * @param {Number} a - Red/Green Coordinate
 * @param {Number} b - Yellow/Blue Coordinate
 * @return {ArrayHWB}
 * @example
 * lab2hwb(54.29, 80.82, 69.88) // => [0, 0, 0]
 */

function lab2hwb(labL, labA, labB, fallbackhue) {
  const [xyzX, xyzY, xyzZ] = lab2xyz(labL, labA, labB);
  const [rgbR, rgbG, rgbB] = xyz2rgb(xyzX, xyzY, xyzZ);
  const [hwbH, hwbW, hwbB] = rgb2hwb(rgbR, rgbG, rgbB, fallbackhue);
  return [hwbH, hwbW, hwbB];
}
/* Convert between HWB and LCH
/* ========================================================================== */

/**
 * @func hwb2lch
 * @desc Return a CIE LCH color from an HWB color
 * @param {Number} h - Hue Angle (0 - 360)
 * @param {Number} w - Whiteness (0 - 100)
 * @param {Number} b - Blackness (0 - 100)
 * @return {ArrayLCH}
 * @example
 * hwb2lch(0, 0, 0) // => [54.29, 106.84, 40.85]
 */

function hwb2lch(hwbH, hwbW, hwbB) {
  const [rgbR, rgbG, rgbB] = hwb2rgb(hwbH, hwbW, hwbB);
  const [xyzX, xyzY, xyzZ] = rgb2xyz(rgbR, rgbG, rgbB);
  const [labL, labA, labB] = xyz2lab(xyzX, xyzY, xyzZ);
  const [lchL, lchC, lchH] = lab2lch(labL, labA, labB);
  return [lchL, lchC, lchH];
}
/**
 * @func lch2hwb
 * @desc Return an HWB color from a CIE LCH color
 * @param {Number} l - CIE Lightness
 * @param {Number} c - CIE Chroma
 * @param {Number} h - CIE Hue Angle
 * @return {ArrayLCH}
 * @example
 * lch2hwb(54.29, 106.84, 40.85) // => [0, 0, 0]
 */

function lch2hwb(lchL, lchC, lchH, fallbackhue) {
  const [labL, labA, labB] = lch2lab(lchL, lchC, lchH);
  const [xyzX, xyzY, xyzZ] = lab2xyz(labL, labA, labB);
  const [rgbR, rgbG, rgbB] = xyz2rgb(xyzX, xyzY, xyzZ);
  const [hwbH, hwbW, hwbB] = rgb2hwb(rgbR, rgbG, rgbB, fallbackhue);
  return [hwbH, hwbW, hwbB];
}
/* Convert between HWB and XYZ
/* ========================================================================== */

/**
 * @func hwb2xyz
 * @desc Return an XYZ color from an HWB color
 * @param {Number} h - Hue Angle (0 - 360)
 * @param {Number} w - Whiteness (0 - 100)
 * @param {Number} b - Blackness (0 - 100)
 * @return {ArrayXYZ}
 * @example
 * hwb2xyz(0, 0, 0) // => [41.25, 21.27, 1.93]
 */

function hwb2xyz(hwbH, hwbW, hwbB) {
  const [rgbR, rgbG, rgbB] = hwb2rgb(hwbH, hwbW, hwbB);
  const [xyzX, xyzY, xyzZ] = rgb2xyz(rgbR, rgbG, rgbB);
  return [xyzX, xyzY, xyzZ];
}
/**
 * @func xyz2hwb
 * @desc Return an HWB color from an XYZ color
 * @param {Number} x - Chromaticity of X
 * @param {Number} y - Chromaticity of Y
 * @param {Number} z - Chromaticity of Z
 * @return {ArrayXYZ}
 * @example
 * xyz2hwb(0, 0, 0) // => [41.25, 21.27, 1.93]
 */

function xyz2hwb(xyzX, xyzY, xyzZ, fallbackhue) {
  const [rgbR, rgbG, rgbB] = xyz2rgb(xyzX, xyzY, xyzZ);
  const [hwbH, hwbW, hwbB] = rgb2hwb(rgbR, rgbG, rgbB, fallbackhue);
  return [hwbH, hwbW, hwbB];
}
/* Convert between HSV and Lab
/* ========================================================================== */

/**
 * @func hsv2lab
 * @desc Return a CIE LAB color from an HSV color
 * @param {Number} h - Hue Angle (0 - 360)
 * @param {Number} s - Saturation (0 - 100)
 * @param {Number} v - Value (0 - 100)
 * @return {ArrayLAB}
 * @example
 * hsv2lab(0, 100, 100) // => [54.29, 80.82, 69.88]
 */

function hsv2lab(hsvH, hsvS, hsvV) {
  const [rgbR, rgbG, rgbB] = hsv2rgb(hsvH, hsvS, hsvV);
  const [xyzX, xyzY, xyzZ] = rgb2xyz(rgbR, rgbG, rgbB);
  const [labL, labA, labB] = xyz2lab(xyzX, xyzY, xyzZ);
  return [labL, labA, labB];
}
/**
 * @func lab2hsv
 * @desc Return an HSV color from a CIE LAB color
 * @param {Number} l - CIE Lightness
 * @param {Number} a - Red/Green Coordinate
 * @param {Number} b - Yellow/Blue Coordinate
 * @return {ArrayHSV}
 * @example
 * lab2hsv(54.29, 80.82, 69.88) // => [0, 100, 100]
 */

function lab2hsv(labL, labA, labB, fallbackhue) {
  const [xyzX, xyzY, xyzZ] = lab2xyz(labL, labA, labB);
  const [rgbR, rgbG, rgbB] = xyz2rgb(xyzX, xyzY, xyzZ);
  const [hsvH, hsvS, hsvV] = rgb2hsv(rgbR, rgbG, rgbB, fallbackhue);
  return [hsvH, hsvS, hsvV];
}
/* Convert between HSV and LCH
/* ========================================================================== */

/**
 * @func hsv2lch
 * @desc Return a CIE LCH color from an HSV color
 * @param {Number} h - Hue Angle (0 - 360)
 * @param {Number} s - Saturation (0 - 100)
 * @param {Number} v - Value (0 - 100)
 * @return {ArrayLCH}
 * @example
 * hsv2lch(0, 100, 100) // => [54.29, 106.84, 40.85]
 */

function hsv2lch(hsvH, hsvS, hsvV) {
  const [rgbR, rgbG, rgbB] = hsv2rgb(hsvH, hsvS, hsvV);
  const [xyzX, xyzY, xyzZ] = rgb2xyz(rgbR, rgbG, rgbB);
  const [labL, labA, labB] = xyz2lab(xyzX, xyzY, xyzZ);
  const [lchL, lchC, lchH] = lab2lch(labL, labA, labB);
  return [lchL, lchC, lchH];
}
/**
 * @func lch2hsv
 * @desc Return an HSV color from a CIE LCH color
 * @param {Number} l - CIE Lightness
 * @param {Number} c - CIE Chroma
 * @param {Number} h - CIE Hue Angle
 * @return {ArrayHSV}
 * @example
 * lch2hsv(54.29, 106.84, 40.85) // => [0, 100, 100]
 */

function lch2hsv(lchL, lchC, lchH, fallbackhue) {
  const [labL, labA, labB] = lch2lab(lchL, lchC, lchH);
  const [xyzX, xyzY, xyzZ] = lab2xyz(labL, labA, labB);
  const [rgbR, rgbG, rgbB] = xyz2rgb(xyzX, xyzY, xyzZ);
  const [hsvH, hsvS, hsvV] = rgb2hsv(rgbR, rgbG, rgbB, fallbackhue);
  return [hsvH, hsvS, hsvV];
}
/* Convert between HSV and XYZ
/* ========================================================================== */

/**
 * @func hsv2xyz
 * @desc Return an XYZ color from an HSV color
 * @param {Number} h - Hue Angle (0 - 360)
 * @param {Number} s - Saturation (0 - 100)
 * @param {Number} v - Value (0 - 100)
 * @return {ArrayXYZ}
 * @example
 * hsv2xyz(0, 100, 100) // => [41.25, 21.27, 1.93]
 */

function hsv2xyz(hsvH, hsvS, hsvV) {
  const [rgbR, rgbG, rgbB] = hsv2rgb(hsvH, hsvS, hsvV);
  const [xyzX, xyzY, xyzZ] = rgb2xyz(rgbR, rgbG, rgbB);
  return [xyzX, xyzY, xyzZ];
}
/**
 * @func xyz2hsv
 * @desc Return an XYZ color from an HSV color
 * @param {Number} x - Chromaticity of X
 * @param {Number} y - Chromaticity of Y
 * @param {Number} z - Chromaticity of Z
 * @return {ArrayHSV}
 * @example
 * xyz2hsv(41.25, 21.27, 1.93) // => [0, 100, 100]
 */

function xyz2hsv(xyzX, xyzY, xyzZ, fallbackhue) {
  const [rgbR, rgbG, rgbB] = xyz2rgb(xyzX, xyzY, xyzZ);
  const [hsvH, hsvS, hsvV] = rgb2hsv(rgbR, rgbG, rgbB, fallbackhue);
  return [hsvH, hsvS, hsvV];
}
/* Convert between XYZ and LCH
/* ========================================================================== */

/**
 * @func xyz2lch
 * @desc Return a CIE LCH color from an XYZ color
 * @param {Number} x - Chromaticity of X
 * @param {Number} y - Chromaticity of Y
 * @param {Number} z - Chromaticity of Z
 * @return {ArrayLCH}
 * @example
 * xyz2lch(41.25, 21.27, 1.93) // => [54.29, 106.84, 40.85]
 */

function xyz2lch(xyzX, xyzY, xyzZ) {
  const [labL, labA, labB] = xyz2lab(xyzX, xyzY, xyzZ);
  const [lchL, lchC, lchH] = lab2lch(labL, labA, labB);
  return [lchL, lchC, lchH];
}
/**
 * @func lch2xyz
 * @desc Return an XYZ color from a CIE LCH color
 * @param {Number} l - CIE Lightness
 * @param {Number} c - CIE Chroma
 * @param {Number} h - CIE Hue Angle
 * @return {ArrayXYZ}
 * @example
 * lch2xyz(54.29, 106.84, 40.85) // => [41.25, 21.27, 1.93]
 */

function lch2xyz(lchL, lchC, lchH) {
  const [labL, labA, labB] = lch2lab(lchL, lchC, lchH);
  const [xyzX, xyzY, xyzZ] = lab2xyz(labL, labA, labB);
  return [xyzX, xyzY, xyzZ];
}
/* Hex input conversions
/* ========================================================================== */

/**
 * @func hex2hsl
 * @desc Return an HSL color from a Hex color
 * @param {StringHex} hex
 * @return {ArrayHSL}
 * @example
 * hex2hsl("#f00") // => [0, 100, 50]
 */

function hex2hsl(hex) {
  return rgb2hsl(...hex2rgb(hex));
}
/**
 * @func hex2hsv
 * @desc Return an HSL color from a Hex color
 * @param {StringHex} hex
 * @return {ArrayHSV}
 * @example
 * hex2hsv("#f00") // => [0, 100, 100]
 */

function hex2hsv(hex) {
  return rgb2hsv(...hex2rgb(hex));
}
/**
 * @func hex2hwb
 * @desc Return an HWB color from a Hex color
 * @param {StringHex} hex
 * @return {ArrayHWB}
 * @example
 * hex2hwb("#f00") // => [0, 0, 0]
 */

function hex2hwb(hex) {
  return rgb2hwb(...hex2rgb(hex));
}
/**
 * @func hex2lab
 * @desc Return a CIE LAB color from a Hex color
 * @param {StringHex} hex
 * @return {ArrayLAB}
 * @example
 * hex2lab("#f00") // => [54.29, 80.82, 69.88]
 */

function hex2lab(hex) {
  return rgb2lab(...hex2rgb(hex));
}
/**
 * @func hex2lch
 * @desc Return a CIE LCH color from a Hex color
 * @param {StringHex} hex
 * @return {ArrayLCH}
 * @example
 * hex2lch("#f00") // => [54.29, 106.84, 40.85]
 */

function hex2lch(hex) {
  return rgb2lch(...hex2rgb(hex));
}
/**
 * @func hex2xyz
 * @desc Return an XYZ color from a Hex color
 * @param {StringHex} hex
 * @return {ArrayXYZ}
 * @example
 * hex2xyz("#f00") // => [41.25, 21.27, 1.93]
 */

function hex2xyz(hex) {
  return rgb2xyz(...hex2rgb(hex));
}
/* Hex output conversions
/* ========================================================================== */

/**
 * @func hsl2hex
 * @desc Return a Hex color from an HSL color
 * @param {Number} h - Hue Angle (0 - 360)
 * @param {Number} s - Saturation (0 - 100)
 * @param {Number} l - Lightness (0 - 100)
 * @return {StringHex}
 * @example
 * hsl2hex(0, 100, 50) // => "#f00"
 */

function hsl2hex(hslH, hslS, hslL) {
  return rgb2hex(...hsl2rgb(hslH, hslS, hslL));
}
/**
 * @func hsv2hex
 * @desc Return a Hex color from an HSV color
 * @param {Number} h - Hue Angle (0 - 360)
 * @param {Number} s - Saturation (0 - 100)
 * @param {Number} v - Value (0 - 100)
 * @return {StringHex}
 * @example
 * hsv2hex(0, 100, 100) // => "#f00"
 */

function hsv2hex(hsvH, hsvS, hsvV) {
  return rgb2hex(...hsl2rgb(hsvH, hsvS, hsvV));
}
/**
 * @func hwb2hex
 * @desc Return a Hex color from an HWB color
 * @param {Number} h - Hue Angle (0 - 360)
 * @param {Number} w - Whiteness (0 - 100)
 * @param {Number} b - Blackness (0 - 100)
 * @return {StringHex}
 * @example
 * hwb2hex(0, 0, 0) // => "#f00"
 */

function hwb2hex(hwbH, hwbW, hwbB) {
  return rgb2hex(...hwb2rgb(hwbH, hwbW, hwbB));
}
/**
 * @func lch2hex
 * @desc Return a Hex color from a CIE LAB color
 * @param {Number} l - CIE Lightness
 * @param {Number} a - Red/Green Coordinate
 * @param {Number} b - Yellow/Blue Coordinate
 * @return {StringHex}
 * @example
 * lch2hex(54.29, 80.82, 69.88) // => "#f00"
 */

function lab2hex(labL, labA, labB) {
  return rgb2hex(...lab2rgb(labL, labA, labB));
}
/**
 * @func lch2hex
 * @desc Return a Hex color from a CIE LCH color
 * @param {Number} l - CIE Lightness
 * @param {Number} c - CIE Chroma
 * @param {Number} h - CIE Hue Angle
 * @return {StringHex}
 * @example
 * lch2hex(54.29, 106.84, 40.85) // => "#f00"
 */

function lch2hex(lchL, lchC, lchH) {
  return rgb2hex(...lch2rgb(lchL, lchC, lchH));
}
/**
 * @func xyz2hex
 * @desc Return a Hex color from an XYZ color
 * @param {Number} x - Chromaticity of X
 * @param {Number} y - Chromaticity of Y
 * @param {Number} z - Chromaticity of Z
 * @return {StringHex}
 * @example
 * xyz2hex(41.25, 21.27, 1.93) // => "#f00"
 */

function xyz2hex(xyzX, xyzY, xyzZ) {
  return rgb2hex(...xyz2rgb(xyzX, xyzY, xyzZ));
}
/* CIEDE conversions
/* ========================================================================== */

/**
 * @func hex2ciede
 * @desc Return the CIEDE2000 difference between 2 HEX colors
 * @param {StringHex} hex1
 * @param {StringHex} hex2
 * @return {NumberCIEDE}
 * @example
 * hex2ciede('#fff', '#000') // => 100
 */

function hex2ciede(hex1, hex2) {
  return lab2ciede(hex2lab(hex1), hex2lab(hex2));
}
/**
 * @func hsl2ciede
 * @desc Return the CIEDE2000 difference between 2 HSL colors
 * @param {ArrayHSL} hsl1
 * @param {ArrayHSL} hsl2
 * @return {NumberCIEDE}
 * @example
 * hsl2ciede([0, 0, 100], [0, 0, 0]) // => 100
 */

function hsl2ciede(hsl1, hsl2) {
  return lab2ciede(hsl2lab(...hsl1), hsl2lab(...hsl2));
}
/**
 * @func hsv2ciede
 * @desc Return the CIEDE2000 difference between 2 HSV colors
 * @param {ArrayHSV} hsl1
 * @param {ArrayHSV} hsl2
 * @return {NumberCIEDE}.
 * @example
 * hsv2ciede([0, 0, 40], [0, 0, 0]) // => 100
 */

function hsv2ciede(hsv1, hsv2) {
  return lab2ciede(hsv2lab(...hsv1), hsv2lab(...hsv2));
}
/**
 * @func hwb2ciede
 * @desc Return the CIEDE2000 difference between 2 HWB colors
 * @param {ArrayHWB} hwb1
 * @param {ArrayHWB} hwb2
 * @return {NumberCIEDE}.
 * @example
 * hwb2ciede([0, 0, 40], [0, 0, 0]) // => 100
 */

function hwb2ciede(hwb1, hwb2) {
  return lab2ciede(hwb2lab(...hwb1), hwb2lab(...hwb2));
}
/**
 * @func keyword2ciede
 * @desc Return the CIEDE2000 difference between 2 keyword colors
 * @param {StringKeyword} keyword1
 * @param {StringKeyword} keyword2
 * @return {NumberCIEDE}.
 * @example
 * keyword2ciede('white', 'black') // => 100
 */

function keyword2ciede(keyword1, keyword2) {
  return lab2ciede(keyword2lab(keyword1), keyword2lab(keyword2));
}
/**
 * @func lch2ciede
 * @desc Return the CIEDE2000 difference between 2 LCH colors
 * @param {ArrayLCH} lch1
 * @param {ArrayLCH} lch2
 * @return {NumberCIEDE}.
 * @example
 * lch2ciede([100, 0.03, -82.2], [0, 0, 0]) // => 100
 */

function lch2ciede(lch1, lch2) {
  return lab2ciede(lch2lab(...lch1), lch2lab(...lch2));
}
/**
 * @func rgb2ciede
 * @desc Return the CIEDE2000 difference between 2 RGB colors
 * @param {ArrayRGB} rgb1
 * @param {ArrayRGB} rgb2
 * @return {NumberCIEDE}.
 * @example
 * rgb2ciede([100, 100, 100], [0, 0, 0]) // => 100
 */

function rgb2ciede(rgb1, rgb2) {
  return lab2ciede(rgb2lab(...rgb1), rgb2lab(...rgb2));
}
/**
 * @func xyz2ciede
 * @desc Return the CIEDE2000 difference between 2 XYZ colors
 * @param {ArrayXYZ} xyz1
 * @param {ArrayXYZ} xyz2
 * @return {NumberCIEDE}.
 * @example
 * xyz2ciede([95.05, 100, 108.88], [0, 0, 0]) // => 100
 */

function xyz2ciede(xyz1, xyz2) {
  return lab2ciede(xyz2lab(...xyz1), xyz2lab(...xyz2));
}
/* Contrast conversions
/* ========================================================================== */

/**
 * @func hex2contrast
 * @desc Return the contrast ratio of 2 HEX colors
 * @param {StringHex} hex1
 * @param {StringHex} hex2
 * @return {NumberContrast}
 * @example
 * rgb2contrast("#fff", '#000') // => 21
 */

function hex2contrast(hex1, hex2) {
  return rgb2contrast(hex2rgb(hex1), hex2rgb(hex2));
}
/**
 * @func hsl2contrast
 * @desc Return the contrast ratio of 2 HSL colors
 * @param {ArrayHSL} hsl1
 * @param {ArrayHSL} hsl2
 * @return {NumberContrast}
 * @example
 * hsl2contrast([0, 0, 100], [0, 0, 0]) // => 21
 */

function hsl2contrast(hsl1, hsl2) {
  return rgb2contrast(hsl2rgb(...hsl1), hsl2rgb(...hsl2));
}
/**
 * @func hsv2contrast
 * @desc Return the contrast ratio of 2 HSV colors
 * @param {ArrayHSV} hsv1
 * @param {ArrayHSV} hsv2
 * @return {NumberContrast}
 * @example
 * hsv2contrast([0, 0, 100], [0, 0, 0]) // => 21
 */

function hsv2contrast(hsv1, hsv2) {
  return rgb2contrast(hsv2rgb(...hsv1), hsv2rgb(...hsv2));
}
/**
 * @func hwb2contrast
 * @desc Return the contrast ratio of 2 HWB colors
 * @param {ArrayHWB} hwb1
 * @param {ArrayHWB} hwb2
 * @return {NumberContrast}
 * @example
 * hwb2contrast([0, 100, 0], [0, 0, 100]) // => 21
 */

function hwb2contrast(hwb1, hwb2) {
  return rgb2contrast(hwb2rgb(...hwb1), hwb2rgb(...hwb2));
}
/**
 * @func keyword2contrast
 * @desc Return the contrast ratio of 2 keyword colors
 * @param {StringKeyword} keyword1
 * @param {StringKeyword} keyword2
 * @return {NumberContrast}
 * @example
 * keyword2contrast('white', 'black') // => 21
 */

function keyword2contrast(keyword1, keyword2) {
  return rgb2contrast(keyword2rgb(keyword1), keyword2rgb(keyword2));
}
/**
 * @func lab2contrast
 * @desc Return the contrast ratio of 2 LAB colors
 * @param {ArrayLAB} lab1
 * @param {ArrayLAB} lab2
 * @return {NumberContrast}
 * @example
 * lab2contrast([100, 0.003, -0.025], [0, 0, 0]) // => 21
 */

function lab2contrast(lab1, lab2) {
  return rgb2contrast(lab2rgb(...lab1), lab2rgb(...lab2));
}
/**
 * @func lch2contrast
 * @desc Return the contrast ratio of 2 LCH colors
 * @param {ArrayLCH} lch1
 * @param {ArrayLCH} lch2
 * @return {NumberContrast}
 * @example
 * lch2contrast([100, 0.025, -82.2], [0, 0, 0]) // => 21
 */

function lch2contrast(lch1, lch2) {
  return rgb2contrast(lch2rgb(...lch1), lch2rgb(...lch2));
}
/**
 * @func xyz2contrast
 * @desc Return the contrast ratio of 2 XYZ colors
 * @param {ArrayXYZ} xyz1
 * @param {ArrayXYZ} xyz2
 * @return {NumberContrast}
 * @example
 * xyz2contrast([95.05, 100, 108.88], [0, 0, 0]) // => 21
 */

function xyz2contrast(xyz1, xyz2) {
  return rgb2contrast(xyz2rgb(...xyz1), xyz2rgb(...xyz2));
}
/* Keyword Conversions
/* ========================================================================== */

/**
 * @func keyword2hex
 * @desc Return an RGB color from a keyword color
 * @param {StringKeyword} keyword - CSS Color Keyword
 * @return {String}
 * @example
 * keyword2hex('white') // => "#ffffff"
 */

function keyword2hex(keyword) {
  return rgb2hex(...keyword2rgb(keyword));
}
/**
 * @func keyword2hsl
 * @desc Return an HSL color from a keyword color
 * @param {StringKeyword}
 * @return {ArrayHSL}
 * @example
 * keyword2hsl('white') // => [0, 0, 100]
 */

function keyword2hsl(keyword) {
  return rgb2hsl(...keyword2rgb(keyword));
}
/**
 * @func keyword2hsv
 * @desc Return an HSV color from a keyword color
 * @param {StringKeyword}
 * @return {ArrayHSV}
 * @example
 * keyword2hsv('white') // => [0, 0, 100]
 */

function keyword2hsv(keyword) {
  return rgb2hsv(...keyword2rgb(keyword));
}
/**
 * @func keyword2hwb
 * @desc Return an HWB color from a keyword color
 * @param {StringKeyword}
 * @return {ArrayHWB}
 * @example
 * keyword2hwb('red') // => [0, 0, 0]
 */

function keyword2hwb(keyword) {
  return rgb2hwb(...keyword2rgb(keyword));
}
/**
 * @func keyword2lab
 * @desc Return a CIE LAB color from a keyword color
 * @param {StringKeyword}
 * @return {ArrayLAB}
 * @example
 * keyword2lab('red') // => [54.29, 80.82, 69.88]
 */

function keyword2lab(keyword) {
  return rgb2lab(...keyword2rgb(keyword));
}
/**
 * @func keyword2lch
 * @desc Return a CIE LCH color from a keyword color
 * @param {StringKeyword}
 * @return {ArrayLCH}
 * @example
 * keyword2lch('red') // => [54.29, 106.84, 40.85]
 */

function keyword2lch(keyword) {
  return rgb2lch(...keyword2rgb(keyword));
}
/**
 * @func keyword2lch
 * @desc Return an XYZ color from a keyword color
 * @param {StringKeyword}
 * @return {ArrayXYZ}
 * @example
 * keyword2lch('red') // => [41.25, 21.27, 1.93]
 */

function keyword2xyz(keyword) {
  return rgb2xyz(...keyword2rgb(keyword));
}
/* All Conversions
/* ========================================================================== */

/* -------------------- end of './lib/color/convert.js' --------------------- */

/* -------------------- start of './lib/geom/matrix.js' --------------------- */

function matrixMultiply(a, b) {
  var result = [];
  for(let i = 0; i < a.length; i++) {
    result[i] = [];
    for(let j = 0; j < b[0].length; j++) {
      var sum = 0;
      for(let k = 0; k < a[0].length; k++) {
        sum += a[i][k] * b[k][j];
      }
      result[i][j] = sum;
    }
  }
  return result;
}

function Matrix(...args) {
  let arg = args[0];
  let ret = this instanceof Matrix || new.target === Matrix ? this : [undefined, 0, 0, undefined, 0, 0, undefined, 0, 0];

  const isObj = Util.isObject(arg);

  if(isObj && arg.xx !== undefined && arg.yx !== undefined && arg.xy !== undefined && arg.yy !== undefined && arg.x0 !== undefined && arg.y0 !== undefined) {
    ret[0] = arg.xx;
    ret[1] = arg.xy;
    ret[2] = arg.x0;
    ret[3] = arg.yx;
    ret[4] = arg.yy;
    ret[5] = arg.y0;
  } else if(isObj && arg.a !== undefined && arg.b !== undefined && arg.c !== undefined && arg.d !== undefined && arg.e !== undefined && arg.f !== undefined) {
    ret[0] = arg.a; //xx
    ret[1] = arg.c; //xy
    ret[2] = arg.e; //x0
    ret[3] = arg.b; //yx
    ret[4] = arg.d; //yy
    ret[5] = arg.f; //y0
  } else if(args.length >= 6) {
    Matrix.prototype.init.call(ret, ...args);
    /*} else if(typeof arg === 'number') {
    Matrix.prototype.init.call(ret, ...args);*/
  } else if(typeof arg === 'string' && /matrix\([^)]*\)/.test(arg)) {
    let [xx, xy, x0, yy, yx, y0] = [...arg.matchAll(/[-.0-9]+/g)].map(m => parseFloat(m[0]));
    ret[0] = xx;
    ret[1] = xy;
    ret[2] = x0;
    ret[3] = yx;
    ret[4] = yy;
    ret[5] = y0;
  } else {
    Array.prototype.splice.call(ret, 0, ret.length, 1, 0, 0, 0, 1, 0, 0, 0, 1);
  }
  for(let i = 0; i < 9; i++) if(ret[i] === undefined) ret[i] = [1, 0, 0, 0, 1, 0, 0, 0, 1][i];

  if(!(this instanceof Matrix)) return ret;
}

Matrix.DEG2RAD = Math.PI / 180;
Matrix.RAD2DEG = 180 / Math.PI;

/*
Object.assign(Matrix.prototype, Array.prototype);

Matrix.prototype.splice = Array.prototype.splice;
Matrix.prototype.slice = Array.prototype.slice;
*/

Object.assign(Matrix.prototype, new Array() || Util.getMethods(Array.prototype));

Matrix.prototype.constructor = Matrix;

Object.defineProperty(Matrix, Symbol.species, {
  get() {
    return Matrix;
  }
});

Util.defineGetter(Matrix.prototype, Symbol.toStringTag, function() {
  return `[object ${Matrix.prototype.toString.apply(this, arguments).replace(/^[^(]*/g, '')}]`;
});
Matrix.prototype[Symbol.isConcatSpreadable] = false;

Object.defineProperty(Matrix.prototype, 'length', {
  value: 6,
  enumerable: false,
  writable: true,
  configurable: false
});

Matrix.prototype.keys = ['xx', 'xy', 'x0', 'yx', 'yy', 'y0'];
Matrix.prototype.keySeq = ['xx', 'yx', 'xy', 'yy', 'x0', 'y0'];

const keyIndexes = {
  xx: 0,
  a: 0,
  xy: 1,
  c: 1,
  x0: 2,
  tx: 2,
  e: 2,
  yx: 3,
  b: 3,
  yy: 4,
  d: 4,
  y0: 5,
  ty: 5,
  f: 5
};
Matrix.keyIndexes = keyIndexes;
Matrix.valueNames = ['a', 'c', 'e', 'b', 'd', 'f'];

Matrix.prototype.at = function(col, row = 0) {
  return this[row * 3 + col];
};
Matrix.prototype.get = function(field) {
  if(typeof field == 'number' && field < this.length) return this[field];

  if((field = keyIndexes[field])) return this[field];
};

const MatrixProps = (obj = {}) =>
  Object.entries(keyIndexes).reduce(
    (acc, [k, i]) => ({
      ...acc,
      [k]: {
        get() {
          return this[i];
        },
        set(v) {
          this[i] = v;
        },
        enumerable: true
      }
    }),
    obj
  );

Object.defineProperties(Matrix.prototype, MatrixProps());

//prettier-ignore
/*Object.defineProperties(Matrix.prototype, {
  xx: {get: function() { return this[0]; }, set: function(v) {this[0] = v; }, enumerable: true },
  xy: {get: function() { return this[1]; }, set: function(v) {this[1] = v; }, enumerable: true },
  x0: {get: function() { return this[2]; }, set: function(v) {this[2] = v; }, enumerable: true },
  yx: {get: function() { return this[3]; }, set: function(v) {this[3] = v; }, enumerable: true },
  yy: {get: function() { return this[4]; }, set: function(v) {this[4] = v; }, enumerable: true },
  y0: {get: function() { return this[5]; }, set: function(v) {this[5] = v; }, enumerable: true }
});*/

//Object.defineProperties(Matrix.prototype,['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i'].reduce((acc,prop,i) => ({ ...acc, [prop]: { get: function() { return this[i]; }, set: function(v) { this[i] = v; } } }), {}));

Matrix.propDescriptors =MatrixProps;

Matrix.prototype.init = function(...args) {
  if(args.length == 1) args = args[0];

  if(Array.isArray(args[0])) {
    const [row0 = [0, 0, 0], row1 = [0, 0, 0], row2 = [0, 0, 0]] = args;

    Array.prototype.splice.call(this, 0, row0.length, ...row0);
    Array.prototype.splice.call(this, 3, row1.length, ...row1);
    Array.prototype.splice.call(this, 6, row2.length, ...row2);
  } else {
    Array.prototype.splice.call(this, 0, Math.min(this.length, args.length), ...args);
  }
  return this;
};

Matrix.prototype.setRow = function(...args) {
  const start = args.shift() * 3;
  const end = Math.max(3, args.length);
  for(let i = 0; i < end; i++) this[start + i] = args[i];
  return this;
};

Matrix.prototype.multiply = function(other) {
  return Object.setPrototypeOf(matrixMultiply(this.rows(), other.rows()).flat(), Matrix.prototype);
};

Matrix.prototype.multiplySelf = function(other) {
  Array.prototype.splice.call(this, 0, 6, ...matrixMultiply(this.rows(), other.rows()).flat());
  return this;
};
Matrix.prototype.toObject = function() {
  const { xx, xy, x0, yx, yy, y0 } = this;
  return { xx, xy, x0, yx, yy, y0 };
};
Matrix.prototype.entries = function() {
  return Object.entries(Matrix.prototype.toObject.call(this));
};
Matrix.prototype.clone = function() {
  return new this.constructor[Symbol.species](this);
};
Matrix.prototype.round = function(prec = 1e-12, digits = 12) {
  let m = new Matrix();
  m.init(...[...this].slice(0, 9).map(n => Util.roundTo(n, prec, digits)));
  return m;
};
Matrix.prototype.roundSelf = function(prec = 1e-12, digits = 12) {
  Matrix.prototype.init.call(this, ...[...this].slice(0, 9).map(n => Util.roundTo(n, prec, digits)));
  return this;
};

Matrix.prototype.row = function(row) {
  let i = row * 3;
  return Array.prototype.slice.call(this, i, i + 3);
};
Matrix.prototype.column = function(col) {
  let column = [];
  for(let row of Matrix.prototype.rows.call(this)) column.push(row[col]);
  return column;
};
Matrix.prototype.columns = function() {
  let ret = [];
  for(let i = 0; i < 3; i++) ret.push(Matrix.prototype.column.call(this, i));
  return ret;
};
Matrix.prototype.rows = function() {
  let ret = [];
  for(let i = 0; i < 9; i += 3) ret.push([this[i + 0], this[i + 1], this[i + 2]]);
  return ret;
};

Matrix.prototype.toArray = function() {
  return Array.from(this);
};
Matrix.prototype.isIdentity = function() {
  return this.equals(Matrix.IDENTITY);
};

Matrix.prototype.determinant = function() {
  return this[0] * (this[4] * this[8] - this[5] * this[7]) + this[1] * (this[5] * this[6] - this[3] * this[8]) + this[2] * (this[3] * this[7] - this[4] * this[6]);
};

Matrix.prototype.invert = function() {
  /* const det = Matrix.prototype.determinant.call(this);
  return new Matrix([
    (this[4] * this[8] - this[5] * this[7]) / det,
    (this[2] * this[7] - this[1] * this[8]) / det,
    (this[1] * this[5] - this[2] * this[4]) / det,
    (this[5] * this[6] - this[3] * this[8]) / det,
    (this[0] * this[8] - this[2] * this[6]) / det,
    (this[2] * this[3] - this[0] * this[5]) / det,
    (this[3] * this[7] - this[4] * this[6]) / det,
    (this[6] * this[1] - this[0] * this[7]) / det,
    (this[0] * this[4] - this[1] * this[3]) / det
  ]);*/
  let { a, b, c, d, e, f } = this;
  let den = a * d - b * c;

  return new Matrix({
    a: d / den,
    b: b / -den,
    c: c / -den,
    d: a / den,
    e: (d * e - c * f) / -den,
    f: (b * e - a * f) / den
  });
};

Matrix.prototype.scalarProduct = function(f) {
  return new Matrix({
    xx: this[0] * f,
    xy: this[1] * f,
    x0: this[2] * f,
    yx: this[3] * f,
    yy: this[4] * f,
    y0: this[5] * f
  });
};

Matrix.prototype.toSource = function(construct = false, multiline = true) {
  const nl = multiline ? '\n' : '';
  const rows = Matrix.prototype.rows.call(this);
  const src = `${rows.map(row => row.join(',')).join(multiline ? ',\n ' : ',')}`;
  return construct ? `new Matrix([${nl}${src}${nl}])` : `[${src}]`;
};

Matrix.prototype.toString = function(separator = ' ') {
  let rows = Matrix.prototype.rows.call(this);
  let name = rows[0].length == 3 ? 'matrix' : 'matrix3d';

  if(rows[0].length == 3) {
    rows = [['a', 'b', 'c', 'd', 'e', 'f'].map(k => this[keyIndexes[k]])];
  }

  return `${name}(` + rows.map(row => row.join(',' + separator)).join(',' + separator) + ')';
};
const inspectSym = 'inspect' || Symbol.for('nodejs.util.inspect.custom');
Matrix.prototype[inspectSym] = function() {
  let columns = Matrix.prototype.columns.call(this);
  let numRows = Math.max(...columns.map(col => col.length));
  let numCols = columns.length;
  columns = columns.map(column => column.map(n => (typeof n == 'number' ? Util.roundTo(n, 1e-12, 12) : undefined)));
  let pad = columns.map(column => Math.max(...column.map(n => (n + '').length)));
  let s = '│ ';
  for(let row = 0; row < numRows; row++) {
    if(row > 0) s += ` │\n│ `;
    for(let col = 0; col < numCols; col++) {
      if(col > 0) s += ', ';
      s += (columns[col][row] + '').padStart(pad[col]);
    }
  }
  s += ' │';
  let l = s.indexOf('\n');
  s = '\u250c' + ' '.repeat(l - 2) + '\u2510\n' + s;
  s = s + '\n\u2514' + ' '.repeat(l - 2) + '\u2518\n';

  return '\n' + s;
};

Matrix.prototype.toSVG = function() {
  return 'matrix(' + ['a', 'b', 'c', 'd', 'e', 'f'].map(k => this[keyIndexes[k]]).join(',') + ')';
};

Matrix.prototype.toDOM = function(ctor = DOMMatrix) {
  const rows = Matrix.prototype.rows.call(this);
  const [a, c, e] = rows[0];
  const [b, d, f] = rows[1];
  return new ctor([a, b, c, d, e, f]);
};
Matrix.prototype.toJSON = function() {
  const rows = Matrix.prototype.rows.call(this);
  const [a, c, e] = rows[0];
  const [b, d, f] = rows[1];
  return { a, b, c, d, e, f };
};
Matrix.fromJSON = obj => new Matrix(obj);
Matrix.fromDOM = matrix => {
  const { a, b, c, d, e, f } = matrix;
  return new Matrix(a, c, e, b, d, f);
};

Matrix.prototype.equals = function(other) {
  return this.length <= other.length && Array.prototype.every.call(this, (n, i) => other[i] == n);
};

Matrix.prototype.transformDistance = function(d) {
  const k = 'x' in d && 'y' in d ? ['x', 'y'] : 'width' in d && 'height' in d ? ['width', 'height'] : [0, 1];
  const x = this[0] * d[k[0]] + this[2] * d[k[1]];
  const y = this[1] * d[k[0]] + this[3] * d[k[1]];
  d[k[0]] = x;
  d[k[1]] = y;
  return d;
};

Matrix.prototype.transformXY = function(x, y) {
  const { a, b, c, d, e, f } = this;
  return [a * x + c * y + e, b * x + d * y + f];
};

Matrix.prototype.transformPoint = function(p) {
  const { a, b, c, d, e, f } = this;
  const m0 = [a, c, e],
    m1 = [b, d, f];
  const k = 'x' in p && 'y' in p ? ['x', 'y'] : [0, 1];
  const x = m0[0] * p[k[0]] + m0[1] * p[k[1]] + m0[2];
  const y = m1[0] * p[k[0]] + m1[1] * p[k[1]] + m1[2];
  p[k[0]] = x;
  p[k[1]] = y;
  return p;
};

Matrix.prototype.transformGenerator = function(what = 'point') {
  const matrix = Object.freeze(this.clone());
  return function* (list) {
    const method = Matrix.prototype['transform_' + what] || (typeof what == 'function' && what) || Matrix.prototype.transformXY;

    for(let item of list) yield item instanceof Array ? method.apply(matrix, [...item]) : method.call(matrix, { ...item });
  };
};

Matrix.prototype.transformPoints = function* (list) {
  for(let i = 0; i < list.length; i++) yield Matrix.prototype.transformPoint.call(this, { ...list[i] });
};

Matrix.prototype.transformWH = function(width, height) {
  const w = this[0] * width + this[1] * height;
  const h = this[3] * width + this[4] * height;
  return [w, h];
};

Matrix.prototype.transformSize = function(s) {
  const [a, c, e, b, d, f] = this;

  const w = a * s.width + c * s.height;
  const h = b * s.width + d * s.height;
  s.width = w;
  s.height = h;
  return s;
};

Matrix.prototype.transformXYWH = function(x, y, width, height) {
  return [...Matrix.prototype.transformXY.call(this, x, y), ...Matrix.prototype.transformWH.call(this, width, height)];
};

Matrix.prototype.transformRect = function(rect) {
  let { x1, y1, x2, y2 } = rect;
  [x1, y1] = Matrix.prototype.transformXY.call(this, x1, y1);
  [x2, y2] = Matrix.prototype.transformXY.call(this, x2, y2);
  let xrange = [x1, x2];
  let yrange = [y1, y2];

  [x1, x2] = [Math.min, Math.max].map(fn => fn(...xrange));
  [y1, y2] = [Math.min, Math.max].map(fn => fn(...yrange));
  Object.assign(rect, { x1, x2, y1, y2 });
  return rect;
};

Matrix.prototype.pointTransformer = function() {
  const matrix = this;
  return point => matrix.transformPoint(point);
};

Matrix.prototype.transformer = function() {
  const matrix = this;
  return {
    point: point => matrix.transformPoint(point),
    xy: (x, y) => matrix.transformXY(x, y),
    size: s => matrix.transformSize(s),
    wh: (w, h) => matrix.transformWH(w, h),
    rect: rect => matrix.transformRect(rect),
    points: list => matrix.transformPoints(list),
    distance: d => matrix.transformDistance(d)
  };
};

Matrix.prototype.scaleSign = function() {
  return this[0] * this[4] < 0 || this[1] * this[3] > 0 ? -1 : 1;
};
Matrix.prototype.affineTransform = function(a, b) {
  let xx, yx, xy, yy, tx, ty;
  if(typeof a == 'object' && a.toPoints !== undefined) a = a.toPoints();
  if(typeof b == 'object' && b.toPoints !== undefined) b = b.toPoints();
  xx =
    (b[0].x * a[1].y + b[1].x * a[2].y + b[2].x * a[0].y - b[0].x * a[2].y - b[1].x * a[0].y - b[2].x * a[1].y) /
    (a[0].x * a[1].y + a[1].x * a[2].y + a[2].x * a[0].y - a[0].x * a[2].y - a[1].x * a[0].y - a[2].x * a[1].y);
  yx =
    (b[0].y * a[1].y + b[1].y * a[2].y + b[2].y * a[0].y - b[0].y * a[2].y - b[1].y * a[0].y - b[2].y * a[1].y) /
    (a[0].x * a[1].y + a[1].x * a[2].y + a[2].x * a[0].y - a[0].x * a[2].y - a[1].x * a[0].y - a[2].x * a[1].y);
  xy =
    (a[0].x * b[1].x + a[1].x * b[2].x + a[2].x * b[0].x - a[0].x * b[2].x - a[1].x * b[0].x - a[2].x * b[1].x) /
    (a[0].x * a[1].y + a[1].x * a[2].y + a[2].x * a[0].y - a[0].x * a[2].y - a[1].x * a[0].y - a[2].x * a[1].y);
  yy =
    (a[0].x * b[1].y + a[1].x * b[2].y + a[2].x * b[0].y - a[0].x * b[2].y - a[1].x * b[0].y - a[2].x * b[1].y) /
    (a[0].x * a[1].y + a[1].x * a[2].y + a[2].x * a[0].y - a[0].x * a[2].y - a[1].x * a[0].y - a[2].x * a[1].y);
  tx =
    (a[0].x * a[1].y * b[2].x + a[1].x * a[2].y * b[0].x + a[2].x * a[0].y * b[1].x - a[0].x * a[2].y * b[1].x - a[1].x * a[0].y * b[2].x - a[2].x * a[1].y * b[0].x) /
    (a[0].x * a[1].y + a[1].x * a[2].y + a[2].x * a[0].y - a[0].x * a[2].y - a[1].x * a[0].y - a[2].x * a[1].y);
  ty =
    (a[0].x * a[1].y * b[2].y + a[1].x * a[2].y * b[0].y + a[2].x * a[0].y * b[1].y - a[0].x * a[2].y * b[1].y - a[1].x * a[0].y * b[2].y - a[2].x * a[1].y * b[0].y) /
    (a[0].x * a[1].y + a[1].x * a[2].y + a[2].x * a[0].y - a[0].x * a[2].y - a[1].x * a[0].y - a[2].x * a[1].y);
  this.setRow.call(this, 0, xx, xy, tx);
  this.setRow.call(this, 1, yx, yy, ty);
  this.setRow.call(this, 2, 0, 0, 1);
  return this;
};

Matrix.getAffineTransform = (a, b) => {
  let matrix = new Matrix();
  matrix.affineTransform(a, b);
  return matrix;
};

Matrix.prototype.decompose = function(degrees = false, useLU = true) {
  let { a, b, c, d, e, f } = this;

  let translate = { x: e, y: f },
    rotation = 0,
    scale = { x: 1, y: 1 },
    skew = { x: 0, y: 0 };

  let determ = a * d - b * c,
    r,
    s;

  const calcFromValues = (r1, m1, r2, m2) => {
    if(!isFinite(r1)) return r2;
    else if(!isFinite(r2)) return r1;
    (m1 = Math.abs(m1)), (m2 = Math.abs(m2));
    return Util.roundTo((m1 * r1 + m2 * r2) / (m1 + m2), 0.0001);
  };

  //if(useLU) {
  let sign, cos, sin;
  sign = Matrix.prototype.scaleSign.call(this);
  rotation = (Math.atan2(this[3], this[4]) + Math.atan2(-sign * this[1], sign * this[0])) / 2;
  cos = Math.cos(rotation);
  sin = Math.sin(rotation);
  scale = {
    x: calcFromValues(this[0] / cos, cos, -this[1] / sin, sin),
    y: calcFromValues(this[4] / cos, cos, this[3] / sin, sin)
  };

  /*if(c || d) {
    if(a) {
      skew = { x: Math.atan(c / a), y: Math.atan(b / a) };
      scale = { x: a, y: determ / a };
    } else {
      scale = { x: c, y: d };
      skew.x = Math.PI * 0.25;
    }
  } else {
    if(a || b) {
      r = Math.sqrt(a * a + b * b);
      rotation = b > 0 ? Math.acos(a / r) : -Math.acos(a / r);
      scale = { x: r, y: determ / r };
      skew.x = Math.atan((a * c + b * d) / (r * r));
    } else if(c || d) {
      s = Math.sqrt(c * c + d * d);
      rotation = Math.PI * 0.5 - (d > 0 ? Math.acos(-c / s) : -Math.acos(c / s));
      scale = { x: determ / s, y: s };
      skew.y = Math.atan((a * c + b * d) / (s * s));
    } else {
      scale = { x: 0, y: 0 };
    }
  }*/

  return {
    translate,
    rotate: degrees === true ? Util.roundTo((rotation * 180) / Math.PI, 0.1) : rotation,
    scale,
    skew:
      degrees == true
        ? {
            x: Util.roundTo((skew.x * 180) / Math.PI, 0.1),
            y: Util.roundTo((skew.y * 180) / Math.PI, 0.1)
          }
        : skew
  };
};

Matrix.prototype.initIdentity = function() {
  return Matrix.prototype.init.call(this, [
    [1, 0, 0],
    [0, 1, 0]
  ]);
};
Matrix.prototype.isIdentity = function() {
  return Matrix.prototype.equals.call(
    this,
    [
      [1, 0, 0],
      [0, 1, 0],
      [0, 0, 1]
    ].flat()
  );
};
Matrix.prototype.initTranslate = function(tx, ty) {
  return Matrix.prototype.init.call(this, 1, 0, tx, 0, 1, ty);
};

Matrix.prototype.initScale = function(sx, sy) {
  if(sy === undefined) sy = sx;
  return Matrix.prototype.init.call(this, sx, 0, 0, sy, 0, 0);
};

Object.defineProperties(Matrix.prototype, {
  scaling: {
    get() {
      const x = Math.sign(this.a) * Util.roundTo(Math.sqrt(this.a ** 2 + this.c ** 2), null, 13);
      const y = Math.sign(this.b) * Util.roundTo(Math.sqrt(this.b ** 2 + this.d ** 2), null, 13);
      return { x, y };
    },
    configurable: true
  },
  rotation: {
    get() {
      const { c, a, scaling } = this;
      return Math.atan2(-c / scaling.y, a / scaling.x);
    },
    configurable: true
  },
  translation: {
    get() {
      return { x: this.e, y: this.f };
    },
    configurable: true
  }
});

Matrix.prototype.initRotate = function(angle, deg = false) {
  const rad = deg ? DEG2RAD * angle : angle;
  const s = Math.sin(rad);
  const c = Math.cos(rad);

  /*  Matrix.prototype.setRow.call(this, 0, c, s, 0);
  Matrix.prototype.setRow.call(this, 1, -s, c, 0);
  Matrix.prototype.setRow.call(this, 2, 0,0,1);*/

  Matrix.prototype.setRow.call(this, 0, c, -s, 0);
  Matrix.prototype.setRow.call(this, 1, s, c, 0);
  Matrix.prototype.setRow.call(this, 2, 0, 0, 1);

  return this;
};
Matrix.prototype.initSkew = function(x, y, deg = false) {
  const ax = Math.tan(deg ? DEG2RAD * x : x);
  const ay = Math.tan(deg ? DEG2RAD * y : y);
  return Matrix.prototype.init.call(this, 1, ay, ax, 1, 0, 0);
};
Matrix.prototype[Symbol.iterator] = function* () {
  for(let i = 0; i < 27; i++) {
    if(this[i] === undefined) break;
    yield this[i];
  }
};

Matrix.identity = () => new Matrix();

Matrix.IDENTITY = Object.freeze(Matrix.identity());

for(let name of [
  'toObject',
  'init',
  'toArray',
  'isIdentity',
  'determinant',
  'invert',
  'multiply',
  'scalarProduct',
  'toSource',
  // 'toString',
  'toSVG',
  'equals',
  'initIdentity',
  'isIdentity',
  'initTranslate',
  'initScale',
  'initRotate',
  'scaleSign',
  'decompose',
  'transformer'
]) {
  Matrix[name] = (matrix, ...args) => Matrix.prototype[name].call(matrix || new Matrix(matrix), ...args);
}

for(let name of ['translate', 'scale', 'rotate', 'skew']) {
  Matrix[name] = (...args) => Matrix.prototype['init_' + name].call(new Matrix(), ...args);
}

for(let name of ['translate', 'scale', 'rotate', 'skew']) {
  Matrix.prototype[name] = function(...args) {
    return Matrix.prototype.multiply.call(this, new Matrix()['init_' + name](...args));
  };
  Matrix.prototype[name + 'Self'] = Matrix.prototype[name + '_self'] = function(...args) {
    return Matrix.prototype.multiplySelf.call(this, new Matrix()['init_' + name](...args));
  };
}

for(let name of ['transformDistance', 'transformXY', 'transformPoint', 'transformPoints', 'transformWH', 'transformSize', 'transformRect', 'affineTransform']) {
  const method = Matrix.prototype[name];

  if(method.length == 2) {
    Matrix[name] = Util.curry((m, a, b) => Matrix.prototype[name].call(m || new Matrix(m), a, b));
  } else if(method.length == 1) {
    Matrix[name] = Util.curry((m, a) => Matrix.prototype[name].call(m || new Matrix(m), a));
  }
}

Util.defineGetter(Matrix, Symbol.species, function() {
  return this;
});

const isMatrix = m => Util.isObject(m) && (m instanceof Matrix || (m.length !== undefined && (m.length == 6 || m.length == 9) && m.every(el => typeof el == 'number')));

const ImmutableMatrix = Util.immutableClass(Matrix);
Util.defineGetter(ImmutableMatrix, Symbol.species, () => ImmutableMatrix);

/* --------------------- end of './lib/geom/matrix.js' ---------------------- */

//import { Element } from './lib/dom/element.js';

/* --------------------- start of './lib/geom/point.js' --------------------- */

const SymSpecies = Util.tryCatch(
  () => Symbol,
  sym => sym.species
);

const CTOR = obj => {
  if(obj[SymSpecies]) return obj[SymSpecies];
  let p = Object.getPrototypeOf(obj);
  if(p[SymSpecies]) return p[SymSpecies];
  return p.constructor;
};

function Point(...args) {
  let isNew = this instanceof Point;
  args = args[0] instanceof Array ? args.shift() : args;
  let p = isNew ? this : new Point(...args);
  let arg = args.shift();

  if(!new.target) if (arg instanceof Point) return arg;

  if(typeof arg === 'undefined') {
    p.x = arg;
    p.y = args.shift();
  } else if(typeof arg === 'number') {
    p.x = parseFloat(arg);
    p.y = parseFloat(args.shift());
  } else if(typeof arg === 'string') {
    const matches = [...arg.matchAll(/([-+]?d*.?d+)(?:[eE]([-+]?d+))?/g)];

    p.x = parseFloat(matches[0]);
    p.y = parseFloat(matches[1]);
  } else if(typeof arg == 'object' && arg !== null && (arg.x !== undefined || arg.y !== undefined)) {
    p.x = arg.x;
    p.y = arg.y;
  } else if(typeof arg == 'object' && arg !== null && arg.length > 0 && x !== undefined && y !== undefined) {
    p.x = parseFloat(arg.shift());
    p.y = parseFloat(arg.shift());
  } else if(typeof args[0] === 'number' && typeof args[1] === 'number') {
    p.x = args[0];
    p.y = args[1];
    args.shift(2);
  } else {
    p.x = 0;
    p.y = 0;
  }
  if(p.x === undefined) p.x = 0;
  if(p.y === undefined) p.y = 0;
  if(isNaN(p.x)) p.x = undefined;
  if(isNaN(p.y)) p.y = undefined;

  if(!isNew) {
    /* if(p.prototype == Object) p.prototype = Point.prototype;
    else Object.assign(p, Point.prototype);*/
    return p;
  }
}

Point.getOther = args => (console.debug('getOther', ...args), typeof args[0] == 'number' ? [{ x: args[0], y: args[1] }] : args);

Object.defineProperties(Point.prototype, {
  X: {
    get() {
      return this.x;
    }
  },
  Y: {
    get() {
      return this.y;
    }
  }
});

Point.prototype.move = function(x, y) {
  this.x += x;
  this.y += y;
  return this;
};
Point.prototype.moveTo = function(x, y) {
  this.x = x;
  this.y = y;
  return this;
};
Point.prototype.clear = function(x, y) {
  this.x = 0;
  this.y = 0;
  return this;
};
Point.prototype.set = function(fn) {
  if(typeof fn != 'function') {
    Point.apply(this, [...arguments]);
    return this;
  }
  return fn(this.x, this.y);
};
Point.prototype.clone = function() {
  const ctor = this[Symbol.species] || this.constructor[Symbol.species];

  return new ctor({ x: this.x, y: this.y });
};
Point.prototype.sum = function(...args) {
  const p = new Point(...args);
  let r = new this.constructor(this.x, this.y);
  r.x += p.x;
  r.y += p.y;
  return r;
};
Point.prototype.add = function(...args) {
  const other = new Point(...args);
  this.x += other.x;
  this.y += other.y;
  return this;
};
Point.prototype.diff = function(arg) {
  let { x, y } = this;
  let fn = function(other) {
    let r = new Point(x, y);
    return r.sub(other);
  };
  if(arg) return fn(arg);
  return fn;
};
Point.prototype.sub = function(...args) {
  const other = new Point(...args);
  this.x -= other.x;
  this.y -= other.y;
  return this;
};
Point.prototype.prod = function(f) {
  const o = isPoint(f) ? f : { x: f, y: f };
  return new Point(this.x * o.x, this.y * o.y);
};
Point.prototype.mul = function(f) {
  const o = isPoint(f) ? f : { x: f, y: f };
  this.x *= o.x;
  this.y *= o.y;
  return this;
};
Point.prototype.quot = function(other) {
  other = isPoint(other) ? other : { x: other, y: other };
  return new Point(this.x / other.x, this.y / other.y);
};
Point.prototype.div = function(other) {
  other = isPoint(other) ? other : { x: other, y: other };
  this.x /= other.x;
  this.y /= other.y;
  return this;
};
Point.prototype.comp = function() {
  return new Point({ x: -this.x, y: -this.y });
};
Point.prototype.neg = function() {
  this.x *= -1;
  this.y *= -1;
  return this;
};
Point.prototype.distanceSquared = function(other = { x: 0, y: 0 }) {
  return (other.y - this.y) * (other.y - this.y) + (other.x - this.x) * (other.x - this.x);
};
Point.prototype.distance = function(other = { x: 0, y: 0 }) {
  return Math.sqrt(Point.prototype.distanceSquared.call(this, Point(other)));
};
Point.prototype.equals = function(other) {
  let { x, y } = this;
  return +x == +other.x && +y == +other.y;
};
Point.prototype.round = function(precision = 0.001, digits, type) {
  let { x, y } = this;
  digits = digits || Util.roundDigits(precision);
  type = type || 'round';
  this.x = Util.roundTo(x, precision, digits, type);
  this.y = Util.roundTo(y, precision, digits, type);
  return this;
};
Point.prototype.ceil = function() {
  let { x, y } = this;
  this.x = Math.ceil(x);
  this.y = Math.ceil(y);
  return this;
};
Point.prototype.floor = function() {
  let { x, y } = this;
  this.x = Math.floor(x);
  this.y = Math.floor(y);
  return this;
};

Point.prototype.dot = function(other) {
  return this.x * other.x + this.y * other.y;
};

Point.prototype.values = function() {
  return [this.x, this.y];
};
Point.prototype.fromAngle = function(angle, dist = 1.0) {
  this.x = Math.cos(angle) * dist;
  this.y = Math.sin(angle) * dist;
  return this;
};
Point.prototype.toAngle = function(deg = false) {
  return Math.atan2(this.x, this.y) * (deg ? 180 / Math.PI : 1);
};
Point.prototype.angle = function(other, deg = false) {
  other = other || { x: 0, y: 0 };
  return Point.prototype.diff.call(this, other).toAngle(deg);
};
Point.prototype.rotate = function(angle, origin = { x: 0, y: 0 }) {
  this.x -= origin.x;
  this.y -= origin.y;
  let c = Math.cos(angle),
    s = Math.sin(angle);
  let xnew = this.x * c - this.y * s;
  let ynew = this.x * s + this.y * c;
  this.x = xnew;
  this.y = ynew;
  return this;
};
Util.defineGetter(Point.prototype, Symbol.iterator, function() {
  const { x, y } = this;
  let a = [x, y];
  return a[Symbol.iterator].bind(a);
});

Point.prototype.valueOf = function(shl = 16) {
  const { x, y } = this;

  if(shl < 0) return x * (1 << Math.abs(shl)) + y;

  return x + y * (1 << shl);
};
Point.prototype.toString = function(opts = {}) {
  const { precision = 0.001, unit = '', separator = ',', left = '', right = '', pad = 0 } = opts;
  let x = Util.roundTo(this.x, precision);
  let y = Util.roundTo(this.y, precision);
  if(pad > 0) {
    x = x + '';
    y = y + '';
    if(y[0] != '-') y = ' ' + y;
    if(x[0] != '-') x = ' ' + x;
  }
  //console.debug("toString", {x,y}, {pad});
  return `${left}${(x + '').padStart(pad, ' ')}${unit}${separator}${(y + '').padEnd(pad, ' ')}${unit}${right}`;
};
Point.prototype[Symbol.toStringTag] = 'Point';
Point.prototype.toSource = function(opts = {}) {
  const { asArray = false, plainObj = false, pad = a => a /*a.padStart(4, ' ')*/, showNew = true } = opts;
  let x = pad(this.x + '');
  let y = pad(this.y + '');
  let c = t => t;
  if(typeof this != 'object' || this === null) return '';
  if(asArray) return `[${x},${y}]`;
  if(plainObj) return `{x:${x},y:${y}}`;

  return `${c(showNew ? 'new ' : '', 1, 31)}${c('Point', 1, 33)}${c('(', 1, 36)}${c(x, 1, 32)}${c(',', 1, 36)}${c(y, 1, 32)}${c(')', 1, 36)}`;
};

/*Point.prototype.toSource = function() {
  return '{x:' + this.x + ',y:' + this.y + '}';
};*/
Point.prototype.toObject = function(proto = Point.prototype) {
  const { x, y } = this;
  const obj = { x, y };
  Object.setPrototypeOf(obj, proto);
  return obj;
};
Point.prototype.toCSS = function(precision = 0.001, edges = ['left', 'top']) {
  return {
    [edges[0]]: Util.roundTo(this.x, precision) + 'px',
    [edges[1]]: Util.roundTo(this.y, precision) + 'px'
  };
};
Point.prototype.toFixed = function(digits) {
  return new Point(+this.x.toFixed(digits), +this.y.toFixed(digits));
};
Point.prototype.isNull = function() {
  return this.x == 0 && this.y == 0;
};
Point.prototype.inside = function(rect) {
  return this.x >= rect.x && this.x < rect.x + rect.width && this.y >= rect.y && this.y < rect.y + rect.height;
};
Point.prototype.transform = function(m, round = true) {
  if(Util.isObject(m) && typeof m.toMatrix == 'function') m = m.toMatrix();
  //if(Util.isObject(m) && typeof m.transformPoint == 'function') return m.transformPoint(this);

  const x = m[0] * this.x + m[1] * this.y + m[2];
  const y = m[3] * this.x + m[4] * this.y + m[5];

  this.x = x;
  this.y = y;
  if(round) Point.prototype.round.call(this, 1e-13, 13);

  return this;
};
Point.prototype.scaleTo = function(minmax) {
  return new Point({
    x: (this.x - minmax.x1) / (minmax.x2 - minmax.x1),
    y: (this.y - minmax.y1) / (minmax.y2 - minmax.y1)
  });
};
Point.prototype.normalize = function() {
  let d = Point.prototype.distance.call(this);
  return Point.prototype.div.call(this, { x: d, y: d });
};
Point.prototype.normal = function() {
  let d = Point.prototype.distance.call(this);
  return new Point({ x: this.x / d, y: this.y / d });
};

Point.fromString = str => new Point(...str.split(/[^-.0-9]+/g).map(n => +n));
Point.move = (point, x, y) => Point.prototype.move.call(point, x, y);
Point.angle = (point, other, deg = false) => Point.prototype.angle.call(point, other, deg);
Point.inside = (point, rect) => Point.prototype.inside.call(point, rect);
Point.sub = (point, other) => Point.prototype.sub.call(point, other);
Point.prod = (a, b) => Point.prototype.prod.call(a, b);
Point.quot = (a, b) => Point.prototype.quot.call(a, b);
Point.equals = (a, b) => Point.prototype.equals.call(a, b);
Point.round = (point, prec, digits, type) => Point.prototype.round.call(point, prec, digits, type);
Point.fromAngle = (angle, f) => new Point().fromAngle(angle, f);

for(let name of [
  'clone',
  'comp',
  'neg',
  'sides',
  'dimension',
  'toString',
  //'toSource',
  'toCSS',
  'sub',
  'diff',
  'add',
  'sum',
  'distance'
]) {
  Point[name] = (point, ...args) => Point.prototype[name].call(Point(point), ...args);
}
Point.interpolate = (p1, p2, a) => {
  a = Util.clamp(0, 1, a);
  return new Point(p1.x * (1.0 - a) + p2.x * a, p1.y * (1.0 - a) + p2.y * a);
};

Point.toSource = (point, { space = ' ', padding = ' ', separator = ',' }) => `{${padding}x:${space}${point.x}${separator}y:${space}${point.y}${padding}}`;

const isPoint = o =>
  o &&
  ((o.x !== undefined && o.y !== undefined) ||
    ((o.left !== undefined || o.right !== undefined) && (o.top !== undefined || o.bottom !== undefined)) ||
    o instanceof Point ||
    Object.getPrototypeOf(o).constructor === Point);

Point.isPoint = isPoint;

Point.prototype[Util.inspectSymbol] = function(depth, options) {
  const { x, y } = this;
  return /*Object.setPrototypeOf*/ { x, y } /*, Point.prototype*/;
};

Point.bind = (o, keys, g) => {
  keys ??= ['x', 'y'];
  o ??= new Point();
  g ??= k => value => value !== undefined ? (o[k] = value) : o[k];

  const { x, y } = Array.isArray(keys) ? keys.reduce((acc, name, i) => ({ ...acc, [keys[i]]: name }), {}) : keys;
  return Object.setPrototypeOf(Util.bindProperties({}, o, { x, y }), Point.prototype);
};

Util.defineGetter(Point, Symbol.species, function() {
  return this;
});

const ImmutablePoint = Util.immutableClass(Point);
Util.defineGetter(ImmutablePoint, Symbol.species, () => ImmutablePoint);

/* ---------------------- end of './lib/geom/point.js' ---------------------- */

/* ---------------- start of './lib/geom/transformation.js' ----------------- */

const RAD2DEG = 180 / Math.PI;
const DEG2RAD = Math.PI / 180;

class Transformation {
  //typeName = null;

  constructor(transformation) {
    if(transformation instanceof Transformation) return transformation;
    if(transformation instanceof TransformationList) return transformation;

    if(typeof transformation == 'string') return Transformation.fromString(transformation);
    //Util.define(this, { typeName });
    //this.type = type;
    //
    throw new TypeError('Transformation');

    return this;
  }

  get [Symbol.toStringTag]() {
    return Util.functionName(this.constructor);
  }

  get type() {
    let type =
      this.typeName ||
      Util.className(this)
        .toLowerCase()
        .replace(/transform(ation)?/, '')
        .replace(/(ion|ing)$/, 'e');
    return type;
  }

  get [Symbol.isConcatSpreadable]() {
    return this.constructor === TransformationList || Object.getPrototypeOf(this) == TransformationList.prototype || Object.getPrototypeOf(this).constructor == TransformationList;
  }
  get axes() {
    return this.axis !== undefined ? [this.axis] : ['x', 'y', 'z'].filter(axis => axis in this);
  }
  get props() {
    return this.axes.concat(['axis', 'angle'].filter(key => key in this));
  }

  has(axis) {
    if(this.axis !== undefined) return axis === this.axis;
    return axis in this;
  }

  get is3D() {
    return this.has('z');
  }

  entries() {
    return this.props.map(prop => [prop, this[prop]]);
  }

  toJSON() {
    return Object.fromEntries(this.entries());
  }

  vector(unit) {
    if(unit === undefined) unit = this.unit;
    return (this.is3D ? ['x', 'y', 'z'] : ['x', 'y']).map(unit ? axis => this[axis] + unit : axis => this[axis]);
  }

  toString(tUnit) {
    return `${this.type}${this.is3D ? '3d' : ''}(${this.vector(tUnit).join(', ')})`;
  }

  /*  toSource(unit) {
    return Util.colorText('new ',1,31)+Util.colorText(Util.className(this), 1,33) +Util.colorText('(' +this.vector(unit).join(', ') + ')', 1 ,36);
  }*/

  clone() {
    let desc = Object.getOwnPropertyDescriptors(this);
    let props = this.props.reduce((acc, prop) => ({ ...acc, [prop]: desc[prop] }), {});
    return Object.create(Object.getPrototypeOf(this), props);
  }

  static fromString(arg) {
    let cmdLen = arg.indexOf('(');
    let argStr = arg.slice(cmdLen + 1, arg.indexOf(')'));
    let args = argStr.split(/[,\s\ ]+/g);
    let cmd = arg.substring(0, cmdLen);
    let t;
    let unit;

    args = args
      .filter(arg => /^[-+0-9.]+[a-z]*$/.test(arg))
      .map(arg => {
        if(/[a-z]$/.test(arg)) {
          unit = arg.replace(/[-+0-9.]*/g, '');
          arg = arg.replace(/[a-z]*$/g, '');
        }

        return +arg;
      });
    //console.log('fromString', { cmd, args });

    const is3D = cmd.toLowerCase().endsWith('3d');
    if(is3D) cmd = cmd.slice(0, -2);

    if(cmd.startsWith('rotat')) {
      const axis = is3D ? '' : cmd.slice(6);
      args = axis != '' ? [args[0], axis] : args;
      t = new Rotation(...args);
    } else if(cmd.startsWith('translat')) {
      const axis = is3D ? '' : cmd.slice(9);
      args = axis != '' ? [args[0], axis] : args;
      t = new Translation(...args);
    } else if(cmd.startsWith('scal')) {
      const axis = is3D ? '' : cmd.slice(5);
      args = axis != '' ? [args[0], axis] : args;
      t = new Scaling(...args);
    } else if(cmd.startsWith('matrix')) {
      const [a, b, c, d, e, f] = args;
      t = new MatrixTransformation(a, c, e, b, d, f);
    }
    if(unit) t.unit = unit;
    return t;
  }

  /*[Symbol.toStringTag]() {
    return this.toString();
  }*/

  [Symbol.toPrimitive](hint) {
    // console.log("hint:",hint);
    if(hint == 'string' || hint == 'default') return this.toString();

    return this.toString() != '';
  }

  /* [Symbol.for('nodejs.util.inspect.custom')]() {
      return this;
    }*/

  static get rotation() {
    return Rotation;
  }
  static get translation() {
    return Translation;
  }
  static get scaling() {
    return Scaling;
  }
  static get matrix() {
    return MatrixTransformation;
  }
}

//Transformation.prototype[Symbol.toStringTag]='Transformation';

Object.defineProperty(Transformation, Symbol.hasInstance, {
  value(inst) {
    return [Transformation, MatrixTransformation, Rotation, Translation, Scaling, TransformationList].some(ctor => Object.getPrototypeOf(inst) == ctor.prototype);
  }
});

const ImmutableTransformation = Util.immutableClass(Transformation);

class Rotation extends Transformation {
  angle = 0;
  //axis = undefined;

  constructor(angle, x, y) {
    super('rotate');

    if(typeof x == 'string' && ['x', 'y', 'z'].indexOf(x.toLowerCase()) != -1) {
      this.axis = x.toLowerCase();
    } else if(!isNaN(+x) && !isNaN(+y)) {
      this.center = [+x, +y];
    }
    //else this.axis = 'z';
    this.angle = angle;
  }

  invert() {
    return new Rotation(-this.angle, this.axis);
  }

  get values() {
    return { [this.axis || 'z']: this.angle };
  }

  get is3D() {
    return this.axis == 'z';
  }

  isZero() {
    return this.angle == 0;
  }

  toString(rUnit) {
    rUnit = rUnit || this.unit || '';
    const axis = this.axis !== undefined ? this.axis.toUpperCase() : '';
    const angle = this.constructor.convertAngle(this.angle, rUnit);
    return `rotate${this.is3D ? axis : ''}(${angle}${rUnit}${this.center ? this.center.map(coord => `, ${coord}`).join('') : ''})`;
  }

  toSource() {
    let o = Util.colorText('new ', 1, 31) + Util.colorText(Util.className(this), 1, 33) + Util.colorText('(' + this.angle + ')', 1, 36);

    return o;
  }

  toMatrix(matrix = Matrix.identity()) {
    const { center, angle } = this;
    if(center) matrix.translateSelf(...[...center].map(coord => -coord));
    matrix.rotateSelf(DEG2RAD * angle);
    if(center) matrix.translateSelf(...center);
    return matrix.roundSelf();
  }

  accumulate(other) {
    if(this.type !== other.type && this.axis !== other.axis) throw new Error(Util.className(this) + ': accumulate mismatch');
    return new Rotation(this.angle + other.angle, this.axis);
  }

  static convertAngle(angle, unit) {
    switch (unit) {
      case 'deg':
        return angle;
      case 'rad':
        return DEG2RAD * angle;
      case 'turn':
        return angle / 360;
      default:
        return angle;
    }
  }
}

Object.defineProperty(Rotation.prototype, Symbol.toStringTag, { value: 'Rotation', enumerable: false });

const ImmutableRotation = Util.immutableClass(Rotation);

class Translation extends Transformation {
  x = 0;
  y = 0;
  //z = undefined;

  constructor(...args) {
    super('translate');

    if(typeof args[1] == 'string' && ['x', 'y', 'z'].indexOf(args[1].toLowerCase()) != -1) {
      const n = args.shift();
      const axis = args.shift().toLowerCase();
      this[axis] = n;
    } else {
      let numDim = [...args, '.'].findIndex(a => isNaN(+a));
      const [x = 0, y = 0, z] = args.splice(0, numDim);
      this.x = +x;
      this.y = +y;
      if(z !== undefined) this.z = +z;
    }
    if(args.length > 0 && typeof args[0] == 'string') this.unit = args.shift();
  }

  get values() {
    const { x, y, z } = this;
    return 'z' in this ? { x, y, z } : { x, y };
  }

  isZero() {
    const { x, y, z } = this;
    return 'z' in this ? x == 0 && y == 0 && z == 0 : x == 0 && y == 0;
  }

  toMatrix(matrix = Matrix.identity()) {
    const { x, y } = this;
    return matrix.translateSelf(x, y);
  }

  /*clone() {
    const { x, y, z } = this;
    return z !== undefined ? new Translation(x, y, z) : new Translation(x, y);
  }*/

  invert() {
    const { x, y, z } = this;
    return z !== undefined ? new Translation(-x, -y, -z) : new Translation(Math.abs(x) == 0 ? 0 : -x, Math.abs(y) == 0 ? 0 : -y);
  }

  accumulate(other) {
    if(this.type !== other.type) throw new Error(Util.className(this) + ': accumulate mismatch');

    if(this.is3D) return new Translation(this.x + other.x, this.y + other.y, this.z + other.z);
    return new Translation(this.x + other.x, this.y + other.y);
  }
}
Object.defineProperty(Translation.prototype, Symbol.toStringTag, { value: 'Translation', enumerable: false });

const ImmutableTranslation = Util.immutableClass(Translation);

class Scaling extends Transformation {
  x = 1;
  y = 1;
  //z = undefined;

  constructor(...args) {
    super('scale');

    if(typeof args[1] == 'string' && ['x', 'y', 'z'].indexOf(args[1].toLowerCase()) != -1) {
      const n = args.shift();
      const axis = args.shift().toLowerCase();
      this[axis] = n;
    } else {
      const [x = 1, y, z] = args.splice(0, 3);
      this.x = +x;
      this.y = y === undefined ? this.x : +y;
      if(z !== undefined) this.z = +z;
    }
  }

  get values() {
    const { x, y, z } = this;
    return 'z' in this ? { x, y, z } : { x, y };
  }

  toMatrix(matrix = Matrix.identity()) {
    const { x, y } = this;
    return matrix.scaleSelf(x, y);
  }

  isZero() {
    const { x, y, z } = this;
    return 'z' in this ? x == 1 && y == 1 && z == 1 : x == 1 && y == 1;
  }

  toString() {
    const vector = this.vector('');
    const coords = /*Util.allEqual(vector) ? vector[0] : */ vector.join(', ');

    return `${this.type}${this.is3D ? '3d' : ''}(${coords})`;
  }

  /*clone() {
    const { x, y, z } = this;
    return z !== undefined ? new Scaling(x, y, z) : new Scaling(x, y);
  }*/

  invert() {
    const { x, y, z } = this;
    return z !== undefined ? new Scaling(1 / x, 1 / y, 1 / z) : new Scaling(1 / x, 1 / y);
  }

  accumulate(other) {
    if(this.type !== other.type) throw new Error(Util.className(this) + ': accumulate mismatch');

    if(this.is3D) return new Scaling(this.x * other.x, this.y * other.y, this.z * other.z);
    return new Scaling(this.x * other.x, this.y * other.y);
  }
}
Object.defineProperty(Scaling.prototype, Symbol.toStringTag, { value: 'Scaling', enumerable: false });

const ImmutableScaling = Util.immutableClass(Scaling);

class MatrixTransformation extends Transformation {
  matrix = Matrix.IDENTITY;

  constructor(init) {
    super('matrix');

    if(init instanceof Matrix) this.matrix = init;
    else if(isMatrix(init)) this.matrix = new Matrix(init);
    else this.matrix = new Matrix(...arguments);
  }

  get values() {
    return this.matrix.values();
  }

  toMatrix(matrix = Matrix.identity()) {
    return matrix.multiplySelf(this.matrix);
  }

  toString() {
    return this.matrix.toString('');
  }

  invert() {
    return new MatrixTransformation(this.matrix.invert());
  }

  isZero() {
    return this.matrix.isIdentity();
  }

  accumulate(other) {
    if(this.type !== other.type) throw new Error(Util.className(this) + ': accumulate mismatch');

    return new MatrixTransformation(this.matrix.multiply(other.matrix));
  }
}
Object.defineProperty(MatrixTransformation.prototype, Symbol.toStringTag, {
  value: 'MatrixTransformation',
  enumerable: false
});

const ImmutableMatrixTransformation = Util.immutableClass(MatrixTransformation);

class TransformationList extends Array {
  constructor(init, tUnit, rUnit) {
    super();
    if(Util.isObject(init)) {
      if(tUnit === undefined) tUnit = init.translationUnit || init.tUnit;
      if(rUnit == undefined) rUnit = init.rotationUnit || init.rUnit;
    }
    //   if(typeof init != 'number' && typeof init != 'undefined' && !(Util.isArray(init) && init.length == 0)) console.debug(`TransformationList.constructor(`, typeof init == 'string' ? Util.abbreviate(init) : init, tUnit, rUnit, `)`);
    if(init) {
      this.initialize(init);
      // if(!(typeof init == 'number' || (Util.isArray(init) && init.length == 0))) console.debug(`TransformationList   initialized to:`, this);
    }
    if(typeof tUnit == 'string') this.translationUnit = tUnit;
    if(typeof rUnit == 'string') this.rotationUnit = rUnit;

    return this;
  }

  initialize(init) {
    if(typeof init == 'number') while(this.length < init) this.push(undefined);
    else if(typeof init == 'string') TransformationList.prototype.fromString.call(this, init);
    else if(init instanceof Array) TransformationList.prototype.fromArray.call(this, init);
    else throw new Error('No such initialization: ' + init);
    return this;
  }

  get [Symbol.isConcatSpreadable]() {
    return true;
  }

  /*
  [Symbol.toStringTag]() {
    return this.toSource();
  }*/

  static get [Symbol.species]() {
    return TransformationList;
  }

  get [Symbol.species]() {
    return TransformationList;
  }

  fromString(str) {
    let n,
      a = [];

    for(let i = 0; i < str.length; i += n) {
      let s = str.slice(i);
      n = s.indexOf(')') + 1;
      if(n == 0) n = str.length;
      s = s.slice(0, n).trim();
      if(s != '') a.push(s);
    }
    return this.fromArray(a);
  }

  fromArray(arr) {
    for(let i = 0; i < arr.length; i++) {
      const arg = arr[i];

      if(arg instanceof Transformation) this.push(arg);
      else if(typeof arg == 'string') this.push(Transformation.fromString(arg));
      else throw new Error('No such transformation: ' + arg);
    }

    return this;
  }

  get translationUnit() {
    return (Util.isObject(this.translation) && this.translation.unit) || this.tUnit;
  }
  set translationUnit(value) {
    if(Util.isObject(this.translation)) this.translation.unit = value;
    else this.tUnit = value;
  }

  get rotationUnit() {
    return (Util.isObject(this.rotation) && this.rotation.unit) || this.rUnit;
  }
  set rotationUnit(value) {
    if(Util.isObject(this.rotation)) this.rotation.unit = value;
    else this.rUnit = value;
  }
  static fromString(str) {
    return new TransformationList().fromString(str);
  }

  static fromArray(arr) {
    return new TransformationList().fromArray(arr);
  }

  static fromMatrix(matrix) {
    matrix = matrix instanceof Matrix ? matrix : new Matrix(matrix);

    const transformations = Matrix.decompose(matrix, true);

    Util.extend(transformations.scale, {
      toArray() {
        return [this.x, this.y];
      }
    });
    Util.extend(transformations.translate, {
      toArray() {
        return [this.x, this.y];
      }
    });

    let ret = new TransformationList();

    ret.translate(...transformations.translate.toArray());
    ret.rotate(transformations.rotate);
    ret.scale(...transformations.scale.toArray());

    return ret;
  }

  push(...args) {
    for(let arg of args) {
      if(typeof arg == 'string') arg = Transformation.fromString(arg);
      else if(isMatrix(arg)) arg = new MatrixTransformation(arg);

      Array.prototype.push.call(this, arg);
    }
    return this;
  }

  clone() {
    return this.map(t => t.clone()); // this.slice();
  }

  map(fn) {
    return this.baseCall(Array.prototype.map)(fn);
  }

  slice(...args) {
    return this.baseCall(Array.prototype.slice)(...args);
  }

  splice(...args) {
    return this.baseCall(Array.prototype.splice)(...args);
  }

  concat(...args) {
    return this.baseCall(Array.prototype.concat)(...args);
  }

  filter(pred) {
    return this.baseCall(Array.prototype.filter)(pred);
  }

  baseCall(c = Array.prototype.map) {
    return (...args) => {
      const { tUnit, rUnit } = this;
      let r = c.call(this, ...args);
      if(tUnit) r.tUnit = tUnit;
      if(rUnit) r.rUnit = rUnit;
      return r;
    };
  }

  unshift(...args) {
    for(let arg of args.reverse()) {
      if(typeof arg == 'string') arg = Transformation.fromString(arg);
      Array.prototype.unshift.call(this, arg);
    }
    return this;
  }

  rotate(...args) {
    let rotation = new Rotation(...args);
    if(!rotation.isZero()) Array.prototype.push.call(this, rotation);
    return this;
  }

  translate(x, y) {
    let trans = this.filter(t => !t.type.startsWith('translat'));
    let vec = new Point(x, y);

    //trans.toMatrix().transformPoint(vec);

    vec = vec.round(0.00001, 5);
    //console.log("from:", new Point(x,y), " to:", vec);
    let translation = new Translation(vec.x, vec.y);

    if(!translation.isZero()) /*    if(Math.abs(vec.x) != 0 || Math.abs(vec.y) != 0) */ Array.prototype.push.call(this, translation);

    return this;
  }

  scale(...args) {
    let scaling = new Scaling(...args);
    if(!scaling.isZero()) Array.prototype.push.call(this, scaling);
    return this;
  }

  matrix(...args) {
    let matrixTransformation = new MatrixTransformation(...args);
    if(!matrixTransformation.isZero()) Array.prototype.push.call(this, matrixTransformation);
    return this;
  }

  toString(tUnit, rUnit) {
    if(this.length > 0) {
      tUnit = tUnit || this.translationUnit;
      rUnit = rUnit || this.rotationUnit;
      let r = this.map(t => t && t.type && t.toString(t.type.startsWith('scal') ? '' : t.type.startsWith('rotat') ? rUnit : tUnit)).join(' ');
      return r;
    }
    return '';
  }

  toDOM() {
    return this.toString('px', 'deg');
  }

  /*  [Symbol.toStringTag]() {
    return this.toString();
  }*/

  toSource() {
    let s = Util.colorText('new ', 1, 31) + Util.colorText(Util.className(this), 1, 33) + Util.colorText('([', 1, 36);

    s += this.map(t => t.toSource()).join(', ');
    return s + Util.colorText('])', 1, 36);
  }

  toMatrices() {
    return Array.prototype.map.call([...this], t => t.toMatrix());
  }

  toMatrix(matrix = Matrix.identity()) {
    for(let other of this.toMatrices()) matrix.multiplySelf(other);

    return matrix.roundSelf();
  }

  undo() {
    let ret = new TransformationList();

    for(let i = this.length - 1; i >= 0; i--) Array.prototype.push.call(ret, this[i].invert());

    return ret;
  }

  merge(...args) {
    for(let arg of args) {
      if(typeof arg == 'string') arg = TransformationList.fromString(arg);

      TransformationList.prototype.push.apply(this, arg);
    }
    return this;
  }

  decompose(degrees = true, transformationList = true) {
    let matrix = this.toMatrix();
    const { translate, rotate, scale } = matrix.decompose(degrees);
    let decomposed = { translate, rotate, scale };

    if(transformationList) {
      let ret = new TransformationList();
      ret.translate(translate.x, translate.y, translate.z);
      ret.rotate(rotate);
      ret.scale(scale.x, scale.y, scale.z);
      return ret;
    }

    decomposed.scale.toArray = decomposed.translate.toArray = function toArray() {
      return [this.x, this.y];
    };
    return decomposed;
  }

  findLastIndex(predicate) {
    for(let i = this.length - 1; i >= 0; --i) {
      const x = this[i];
      if(predicate(x)) return i;
    }
    return null;
  }

  findLast(predicate) {
    let index = this.findLastIndex(predicate);
    return this[index];
  }

  get rotation() {
    return this.findLast(item => item && item.type && item.type.startsWith('rotat'));
  }
  set rotation(value) {
    let index = this.findLastIndex(item => item.type.startsWith('rotat'));
    value = value instanceof Rotation ? value : new Rotation(value);
    Array.prototype.splice.call(this, index, 1, value);
  }

  get scaling() {
    return this.findLast(item => item && item.type && item.type.startsWith('scal'));
  }
  set scaling(value) {
    let index = this.findLastIndex(item => item.type.startsWith('scal'));
    value = value instanceof Scaling ? value : new Scaling(value);
    Array.prototype.splice.call(this, index, 1, value);
  }

  get translation() {
    return this.findLast(item => item && item.type && typeof item.type == 'string' && item.type.startsWith('translat'));
  }

  set translation(value) {
    let index = this.findLastIndex(item => item && item.type && item.type.startsWith('transl'));
    value = value instanceof Translation ? value : new Translation(value);
    Array.prototype.splice.call(this, index, 1, value);
  }
  /*  map(...args) {
    return Array.prototype.map.apply(Array.from(this), args);
  }*/

  get last() {
    return this.at(-1);
  }
  get first() {
    return this.at(0);
  }

  at(pos) {
    if(pos < 0) pos += this.length;
    return this[pos];
  }

  collapse() {
    let ret = new TransformationList();

    for(let i = 0; i < this.length; i++) {
      let item = this[i];
      if(i + 1 < this.length && this[i + 1].type == this[i].type) {
        item = item.accumulate(this[i + 1]);
        i++;
      } else {
        item = item.clone();
      }
      Array.prototype.push.call(ret, item);
    }
    return ret;
  }

  collapseAll() {
    return TransformationList.fromMatrix(this.toMatrix());
  }

  get angle() {
    let matrix = this.toMatrix();
    let t = matrix.decompose();
    let { rotate } = t;
    //console.log('ROTATION:', rotate);
    return rotate;
  }

  invert() {
    //return this.reduce((acc, t) => [t.invert(), ...acc], []);
    return new TransformationList(this.reduceRight((acc, t) => [...acc, t.invert()], []));
  }

  join(sep = ' ') {
    return Array.prototype.join.call(this, sep);
  }

  clear() {
    Array.prototype.splice.call(this, 0, this.length);
    return this;
  }

  apply(obj, round = true) {
    if(typeof obj.transform == 'function') {
      const matrix = this.toMatrix();
      return obj.transform(matrix, round);
    }
  }
}
Object.defineProperty(TransformationList.prototype, Symbol.toStringTag, {
  value: 'TransformationList',
  enumerable: false
});

const { concat, copyWithin, find, findIndex, lastIndexOf, pop, push, shift, unshift, slice, splice, includes, indexOf, entries, filter, map, every, some, reduce, reduceRight } = Array.prototype;

Util.inherit(
  TransformationList.prototype,
  {
    // concat,
    copyWithin,
    find,
    findIndex,
    lastIndexOf,
    pop,
    shift,
    //   slice,
    //splice,
    includes,
    indexOf,
    entries,
    //  filter,
    //  map,
    every,
    some,
    reduce,
    reduceRight
  },
  {
    [Symbol.iterator]() {
      return Array.prototype[Symbol.iterator];
    },
    [Symbol.isConcatSpreadable]() {
      return true;
    }
  }
);

//Object.setPrototypeOf(TransformationList.prototype, Transformation.prototype);

const ImmutableTransformationList = Util.immutableClass(TransformationList);

ImmutableTransformationList.prototype.rotate = function(...args) {
  return this.concat([new ImmutableRotation(...args)]);
};

ImmutableTransformationList.prototype.translate = function(...args) {
  return this.concat([new ImmutableTranslation(...args)]);
};

ImmutableTransformationList.prototype.scale = function(...args) {
  return this.concat([new ImmutableScaling(...args)]);
};

Util.defineGetter(ImmutableTransformationList, Symbol.species, () => ImmutableTransformationList);

/* ----------------- end of './lib/geom/transformation.js' ------------------ */

/* -------------------- start of './lib/async/events.js' -------------------- */

// Generate a Promise that listens only once for an event

function once(emitter, ...events) {
  return new Promise(resolve => {
    events.forEach(type => emitter.addEventListener(type, handler, { passive: true }));
    function handler(event) {
      events.forEach(type => emitter.removeEventListener(type, handler, { passive: true }));
      resolve(event);
    }
  });
}

// Turn any event emitter into a stream

async function* streamify(event, element, cond = last => true) {
  let events = Array.isArray(event) ? event : [event];
  let last;
  do {
    yield (last = await once(element, ...events));
  } while(cond(last));
}

// Only pass along event if some time has passed since the last one

async function* throttle(stream, delay) {
  let lastTime;
  let thisTime;
  for await(let event of stream) {
    thisTime = new Date().getTime();
    if(!lastTime || thisTime - lastTime > delay) {
      lastTime = thisTime;
      yield event;
    }
  }
}

let identity = e => e;

// Only pass along events that differ from the last one

async function* distinct(stream, extract = identity) {
  let previous;
  let current;
  for await(let event of stream) {
    current = extract(event);
    if(current !== previous) {
      previous = current;
      yield event;
    }
  }
}

// Invoke a callback every time an event arrives

async function subscribe(stream, callback) {
  for await(let event of stream) callback(event);
}

// run();

/* --------------------- end of './lib/async/events.js' --------------------- */

function main() {
  Object.assign(globalThis, { crosskit, RGBA, HSLA, Util, Matrix, TransformationList });

  const w = 320;
  const h = 200;
  const parent = document.body;

  crosskit.init({
    renderer: CANVAS,
    parent,
    w,
    h,
    alpha: false
  });

  crosskit.clear();
  crosskit.rect({
    x: 0,
    y: 0,
    w,
    h,
    fill: 'black',
    stroke: 'black',
    angle: 0
  });

  const buffer = new ArrayBuffer(w * (h + 2));
  const palette = CreatePalette();
  const paletteHSL = CreatePaletteHSL();
  /*const paletteX = palette.map(color => color.hex());
  const palette32 = Uint32Array.from(palette, c => +c);*/
  const pixels = Array.from({ length: h + 2 }).map((v, i) => new Uint8ClampedArray(buffer, i * w, w));
  const { context } = crosskit;
  const image = context.createImageData(w, h);

  const { now, waitFor, animationFrame } = Util;
  const fps = 50;
  const matrix = new Matrix().translate(160, 100).scale(0.5);

  Object.assign(globalThis, {
    buffer,
    palette,
    paletteHSL,
    pixels,
    context,
    image,
    fps,
    matrix
  });

  async function Loop() {
    const delay = 1000 / fps;
    const log = (t, name) => globalThis.doLog && console.log(`${name} timing: ${t.toFixed(3)}ms`);
    const fire = (...args) => Fire(...args); //Util.instrument(Fire, log);
    const redraw = (...args) => Redraw(...args); //Util.instrument(Redraw, log);

    await once(window, 'load');

    Init();

    for(;;) {
      fire();
      redraw();
      await animationFrame(delay);
    }
  }

  function Fire() {
    for(let x = 0; x < w; x++) {
      pixels[h][x] = 255 - (RandomByte() % 128);
      pixels[h + 1][x] = 255 - (RandomByte() % 128);
    }

    for(let y = 0; y < h; y++) {
      for(let x = 0; x < w; x++) {
        const sum = [pixels[y + 1][Modulo(x - 1, w)], pixels[y + 1][x], pixels[y + 1][Modulo(x + 1, w)], pixels[y + 2][x]].reduce((a, p) => a + (p | 0), 0);

        pixels[y][x] = (sum * 15) >>> 6;
      }
    }
  }

  async function Redraw() {
    const { data } = image;

    let i = 0;
    let t = [...matrix];

    /* context.transform(...t);
    console.log("t:", context.currentTransform);
  */
    for(let y = 0; y < h; y++) {
      for(let x = 0; x < w; x++) {
        const c = palette[pixels[y][x]];
        data[i++] = c.r;
        data[i++] = c.g;
        data[i++] = c.b;
        data[i++] = c.a;
      }
    }

    context.putImageData(image, 0, 0);
  }

  function CreatePalette() {
    const colors = new Array(256);

    for(let i = 0; i < 64; i++) {
      const value = i * 4;

      colors[i] = new RGBA(value, 0, 0); // black to red
      colors[i + 64] = new RGBA(255, value, 0); // red to yellow
      colors[i + 128] = new RGBA(255, 255, value); // yellow to white
      colors[i + 192] = new RGBA(255, 255, 255); // all white
    }
    return colors;
  }

  function CreatePaletteHSL() {
    const colors = new Array(256);

    const hues = [new HSLA(0, 100, 0), new HSLA(0, 100, 50), new HSLA(30, 100, 50), new HSLA(60, 100, 50), new HSLA(60, 100, 100), new HSLA(60, 100, 100)]; /*.map(hsla => hsla.toRGBA())*/

    const breakpoints = [0, 51, 80, 154, 205, 256];
    console.log('breakpoints:', breakpoints);

    for(let i = 0; i < 256; i++) {
      const hue = (v => (v == -1 ? () => hues.length - 2 : v => v))(breakpoints.findIndex(b => i < b));
      const range = breakpoints[hue] - 1 - breakpoints[hue - 1];
      //console.log("hue:", {i,hue, range});

      colors[i] = HSLA.blend(hues[hue - 1], hues[hue], (i - breakpoints[hue - 1]) / range).toRGBA();
    }
    return colors;
  }

  // For random numbers, use "x = 181 * x + 359" from
  // Tom Dickens "Random Number Generator for Microcontrollers"
  // https://web.archive.org/web/20170323204917/http://home.earthlink.net/~tdickens/68hc11/random/68hc11random.html
  let scratch = 0;

  function RandomByte() {
    const value = 181 * scratch + 359;
    scratch = value >>> 0;
    return (value >>> 8) & 0xff;
  }

  function Modulo(n, m) {
    return ((n % m) + m) % m;
  }

  let element, rect, rc;

  function Blaze(x, y) {
    for(let ty = y - 1; ty < y + 1; ty++) {
      for(let tx = x - 1; tx < x + 1; tx++) {
        pixels[ty][tx] = rc;
      }
    }

    pixels[y + 1][x] = rc;
  }

  function MouseHandler(e) {
    let { target, buttons, type } = e;

    if('touches' in e) {
      //console.log(`${e.type}`, ...e.touches);
      for(let touch of [...e.touches]) {
        const { clientX, clientY } = touch;
        MouseHandler({
          type,
          target,
          buttons,
          offsetX: Math.trunc(clientX) - rect.x,
          offsetY: Math.trunc(clientY) - rect.y
        });
      }
      return;
    }

    globalThis.pointerEvent = e;

    const x = Math.round((e.offsetX * w) / rect.width);
    const y = Math.round((e.offsetY * h) / rect.height);

    //console.log(`${e.type} @ ${x},${y}`);

    try {
      if(/(down|start)$/.test(type)) rc = pixels[y][x] > 0x30 ? 0 : RandomByte() | 0x80;

      Blaze(x, y);
    } catch(e) {}
  }

  async function* MouseIterator() {
    for(;;) {
      yield await once(element, 'mousedown', 'touchstart');

      for await(let event of streamify(['mouseup', 'mousemove', 'touchend', 'touchmove'], element)) {
        yield event;
        if(/(up|end)$/.test(event.type)) break;
      }
    }
  }

  function ResizeHandler(e) {
    rect = element.getBoundingClientRect();
    console.log('rect', rect);
  }

  Object.assign(globalThis, { RandomByte });

  function Init() {
    window.canvas = element = document.querySelector('canvas');

    rect = element.getBoundingClientRect();

    window.addEventListener('resize', ResizeHandler, true);

    const handler = MouseHandler; /*|| Util.instrument(MouseHandler, (duration, name, args, ret) => console.log(`handler time: ${duration}`))*/

    subscribe(MouseIterator(), handler);
  }

  Loop();
}

main();
