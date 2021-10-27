define('glAccum', dlsym(libGL, 'glAccum'), null, 'void', 'unsigned int', 'float');
export function glAccum(op, value) {
  return call('glAccum', op, value);
}

define('glAlphaFunc', dlsym(libGL, 'glAlphaFunc'), null, 'void', 'unsigned int', 'float');
export function glAlphaFunc(func, ref) {
  return call('glAlphaFunc', func, ref);
}

define('glArrayElement', dlsym(libGL, 'glArrayElement'), null, 'void', 'int');
export function glArrayElement(i) {
  return call('glArrayElement', i);
}

define('glBegin', dlsym(libGL, 'glBegin'), null, 'void', 'unsigned int');
export function glBegin(mode) {
  return call('glBegin', mode);
}

define('glBindTexture', dlsym(libGL, 'glBindTexture'), null, 'void', 'unsigned int', 'unsigned int');
export function glBindTexture(target, texture) {
  return call('glBindTexture', target, texture);
}

define('glBitmap', dlsym(libGL, 'glBitmap'), null, 'void', 'int', 'int', 'float', 'float', 'float', 'float', 'void *');
export function glBitmap(width, height, xorig, yorig, xmove, ymove, bitmap) {
  return call('glBitmap', width, height, xorig, yorig, xmove, ymove, bitmap);
}

define('glBlendFunc', dlsym(libGL, 'glBlendFunc'), null, 'void', 'unsigned int', 'unsigned int');
export function glBlendFunc(sfactor, dfactor) {
  return call('glBlendFunc', sfactor, dfactor);
}

define('glCallList', dlsym(libGL, 'glCallList'), null, 'void', 'unsigned int');
export function glCallList(list) {
  return call('glCallList', list);
}

define('glCallLists', dlsym(libGL, 'glCallLists'), null, 'void', 'int', 'unsigned int', 'void *');
export function glCallLists(n, type, lists) {
  return call('glCallLists', n, type, lists);
}

define('glClear', dlsym(libGL, 'glClear'), null, 'void', 'unsigned int');
export function glClear(mask) {
  return call('glClear', mask);
}

define('glClearAccum', dlsym(libGL, 'glClearAccum'), null, 'void', 'float', 'float', 'float', 'float');
export function glClearAccum(red, green, blue, alpha) {
  return call('glClearAccum', red, green, blue, alpha);
}

define('glClearColor', dlsym(libGL, 'glClearColor'), null, 'void', 'float', 'float', 'float', 'float');
export function glClearColor(red, green, blue, alpha) {
  return call('glClearColor', red, green, blue, alpha);
}

define('glClearDepth', dlsym(libGL, 'glClearDepth'), null, 'void', 'double');
export function glClearDepth(depth) {
  return call('glClearDepth', depth);
}

define('glClearIndex', dlsym(libGL, 'glClearIndex'), null, 'void', 'float');
export function glClearIndex(c) {
  return call('glClearIndex', c);
}

define('glClearStencil', dlsym(libGL, 'glClearStencil'), null, 'void', 'int');
export function glClearStencil(s) {
  return call('glClearStencil', s);
}

define('glClipPlane', dlsym(libGL, 'glClipPlane'), null, 'void', 'unsigned int', 'void *');
export function glClipPlane(plane, equation) {
  return call('glClipPlane', plane, equation);
}

define('glColor3b', dlsym(libGL, 'glColor3b'), null, 'void', 'sint8', 'sint8', 'sint8');
export function glColor3b(red, green, blue) {
  return call('glColor3b', red, green, blue);
}

define('glColor3bv', dlsym(libGL, 'glColor3bv'), null, 'void', 'void *');
export function glColor3bv(v) {
  return call('glColor3bv', v);
}

define('glColor3d', dlsym(libGL, 'glColor3d'), null, 'void', 'double', 'double', 'double');
export function glColor3d(red, green, blue) {
  return call('glColor3d', red, green, blue);
}

define('glColor3dv', dlsym(libGL, 'glColor3dv'), null, 'void', 'void *');
export function glColor3dv(v) {
  return call('glColor3dv', v);
}

define('glColor3f', dlsym(libGL, 'glColor3f'), null, 'void', 'float', 'float', 'float');
export function glColor3f(red, green, blue) {
  return call('glColor3f', red, green, blue);
}

define('glColor3fv', dlsym(libGL, 'glColor3fv'), null, 'void', 'void *');
export function glColor3fv(v) {
  return call('glColor3fv', v);
}

define('glColor3i', dlsym(libGL, 'glColor3i'), null, 'void', 'int', 'int', 'int');
export function glColor3i(red, green, blue) {
  return call('glColor3i', red, green, blue);
}

define('glColor3iv', dlsym(libGL, 'glColor3iv'), null, 'void', 'void *');
export function glColor3iv(v) {
  return call('glColor3iv', v);
}

define('glColor3s', dlsym(libGL, 'glColor3s'), null, 'void', 'short', 'short', 'short');
export function glColor3s(red, green, blue) {
  return call('glColor3s', red, green, blue);
}

define('glColor3sv', dlsym(libGL, 'glColor3sv'), null, 'void', 'void *');
export function glColor3sv(v) {
  return call('glColor3sv', v);
}

define('glColor3ub', dlsym(libGL, 'glColor3ub'), null, 'void', 'unsigned char', 'unsigned char', 'unsigned char');
export function glColor3ub(red, green, blue) {
  return call('glColor3ub', red, green, blue);
}

define('glColor3ubv', dlsym(libGL, 'glColor3ubv'), null, 'void', 'void *');
export function glColor3ubv(v) {
  return call('glColor3ubv', v);
}

define('glColor3ui', dlsym(libGL, 'glColor3ui'), null, 'void', 'unsigned int', 'unsigned int', 'unsigned int');
export function glColor3ui(red, green, blue) {
  return call('glColor3ui', red, green, blue);
}

define('glColor3uiv', dlsym(libGL, 'glColor3uiv'), null, 'void', 'void *');
export function glColor3uiv(v) {
  return call('glColor3uiv', v);
}

define('glColor3us', dlsym(libGL, 'glColor3us'), null, 'void', 'uint16', 'uint16', 'uint16');
export function glColor3us(red, green, blue) {
  return call('glColor3us', red, green, blue);
}

define('glColor3usv', dlsym(libGL, 'glColor3usv'), null, 'void', 'void *');
export function glColor3usv(v) {
  return call('glColor3usv', v);
}

define('glColor4b', dlsym(libGL, 'glColor4b'), null, 'void', 'sint8', 'sint8', 'sint8', 'sint8');
export function glColor4b(red, green, blue, alpha) {
  return call('glColor4b', red, green, blue, alpha);
}

define('glColor4bv', dlsym(libGL, 'glColor4bv'), null, 'void', 'void *');
export function glColor4bv(v) {
  return call('glColor4bv', v);
}

define('glColor4d', dlsym(libGL, 'glColor4d'), null, 'void', 'double', 'double', 'double', 'double');
export function glColor4d(red, green, blue, alpha) {
  return call('glColor4d', red, green, blue, alpha);
}

define('glColor4dv', dlsym(libGL, 'glColor4dv'), null, 'void', 'void *');
export function glColor4dv(v) {
  return call('glColor4dv', v);
}

define('glColor4f', dlsym(libGL, 'glColor4f'), null, 'void', 'float', 'float', 'float', 'float');
export function glColor4f(red, green, blue, alpha) {
  return call('glColor4f', red, green, blue, alpha);
}

define('glColor4fv', dlsym(libGL, 'glColor4fv'), null, 'void', 'void *');
export function glColor4fv(v) {
  return call('glColor4fv', v);
}

define('glColor4i', dlsym(libGL, 'glColor4i'), null, 'void', 'int', 'int', 'int', 'int');
export function glColor4i(red, green, blue, alpha) {
  return call('glColor4i', red, green, blue, alpha);
}

define('glColor4iv', dlsym(libGL, 'glColor4iv'), null, 'void', 'void *');
export function glColor4iv(v) {
  return call('glColor4iv', v);
}

define('glColor4s', dlsym(libGL, 'glColor4s'), null, 'void', 'short', 'short', 'short', 'short');
export function glColor4s(red, green, blue, alpha) {
  return call('glColor4s', red, green, blue, alpha);
}

define('glColor4sv', dlsym(libGL, 'glColor4sv'), null, 'void', 'void *');
export function glColor4sv(v) {
  return call('glColor4sv', v);
}

define(
  'glColor4ub',
  dlsym(libGL, 'glColor4ub'),
  null,
  'void',
  'unsigned char',
  'unsigned char',
  'unsigned char',
  'unsigned char'
);
export function glColor4ub(red, green, blue, alpha) {
  return call('glColor4ub', red, green, blue, alpha);
}

define('glColor4ubv', dlsym(libGL, 'glColor4ubv'), null, 'void', 'void *');
export function glColor4ubv(v) {
  return call('glColor4ubv', v);
}

define(
  'glColor4ui',
  dlsym(libGL, 'glColor4ui'),
  null,
  'void',
  'unsigned int',
  'unsigned int',
  'unsigned int',
  'unsigned int'
);
export function glColor4ui(red, green, blue, alpha) {
  return call('glColor4ui', red, green, blue, alpha);
}

define('glColor4uiv', dlsym(libGL, 'glColor4uiv'), null, 'void', 'void *');
export function glColor4uiv(v) {
  return call('glColor4uiv', v);
}

define('glColor4us', dlsym(libGL, 'glColor4us'), null, 'void', 'uint16', 'uint16', 'uint16', 'uint16');
export function glColor4us(red, green, blue, alpha) {
  return call('glColor4us', red, green, blue, alpha);
}

define('glColor4usv', dlsym(libGL, 'glColor4usv'), null, 'void', 'void *');
export function glColor4usv(v) {
  return call('glColor4usv', v);
}

define(
  'glColorMask',
  dlsym(libGL, 'glColorMask'),
  null,
  'void',
  'unsigned char',
  'unsigned char',
  'unsigned char',
  'unsigned char'
);
export function glColorMask(red, green, blue, alpha) {
  return call('glColorMask', red, green, blue, alpha);
}

define('glColorMaterial', dlsym(libGL, 'glColorMaterial'), null, 'void', 'unsigned int', 'unsigned int');
export function glColorMaterial(face, mode) {
  return call('glColorMaterial', face, mode);
}

define('glColorPointer', dlsym(libGL, 'glColorPointer'), null, 'void', 'int', 'unsigned int', 'int', 'void *');
export function glColorPointer(size, type, stride, pointer) {
  return call('glColorPointer', size, type, stride, pointer);
}

define('glCopyPixels', dlsym(libGL, 'glCopyPixels'), null, 'void', 'int', 'int', 'int', 'int', 'unsigned int');
export function glCopyPixels(x, y, width, height, type) {
  return call('glCopyPixels', x, y, width, height, type);
}

define(
  'glCopyTexImage1D',
  dlsym(libGL, 'glCopyTexImage1D'),
  null,
  'void',
  'unsigned int',
  'int',
  'unsigned int',
  'int',
  'int',
  'int',
  'int'
);
export function glCopyTexImage1D(target, level, internalFormat, x, y, width, border) {
  return call('glCopyTexImage1D', target, level, internalFormat, x, y, width, border);
}

define(
  'glCopyTexImage2D',
  dlsym(libGL, 'glCopyTexImage2D'),
  null,
  'void',
  'unsigned int',
  'int',
  'unsigned int',
  'int',
  'int',
  'int',
  'int',
  'int'
);
export function glCopyTexImage2D(target, level, internalFormat, x, y, width, height, border) {
  return call('glCopyTexImage2D', target, level, internalFormat, x, y, width, height, border);
}

define(
  'glCopyTexSubImage1D',
  dlsym(libGL, 'glCopyTexSubImage1D'),
  null,
  'void',
  'unsigned int',
  'int',
  'int',
  'int',
  'int',
  'int'
);
export function glCopyTexSubImage1D(target, level, xoffset, x, y, width) {
  return call('glCopyTexSubImage1D', target, level, xoffset, x, y, width);
}

define(
  'glCopyTexSubImage2D',
  dlsym(libGL, 'glCopyTexSubImage2D'),
  null,
  'void',
  'unsigned int',
  'int',
  'int',
  'int',
  'int',
  'int',
  'int',
  'int'
);
export function glCopyTexSubImage2D(target, level, xoffset, yoffset, x, y, width, height) {
  return call('glCopyTexSubImage2D', target, level, xoffset, yoffset, x, y, width, height);
}

define('glCullFace', dlsym(libGL, 'glCullFace'), null, 'void', 'unsigned int');
export function glCullFace(mode) {
  return call('glCullFace', mode);
}

define('glDeleteLists', dlsym(libGL, 'glDeleteLists'), null, 'void', 'unsigned int', 'int');
export function glDeleteLists(list, range) {
  return call('glDeleteLists', list, range);
}

define('glDeleteTextures', dlsym(libGL, 'glDeleteTextures'), null, 'void', 'int', 'void *');
export function glDeleteTextures(n, textures) {
  return call('glDeleteTextures', n, textures);
}

define('glDepthFunc', dlsym(libGL, 'glDepthFunc'), null, 'void', 'unsigned int');
export function glDepthFunc(func) {
  return call('glDepthFunc', func);
}

define('glDepthMask', dlsym(libGL, 'glDepthMask'), null, 'void', 'unsigned char');
export function glDepthMask(flag) {
  return call('glDepthMask', flag);
}

define('glDepthRange', dlsym(libGL, 'glDepthRange'), null, 'void', 'double', 'double');
export function glDepthRange(zNear, zFar) {
  return call('glDepthRange', zNear, zFar);
}

define('glDisable', dlsym(libGL, 'glDisable'), null, 'void', 'unsigned int');
export function glDisable(cap) {
  return call('glDisable', cap);
}

define('glDisableClientState', dlsym(libGL, 'glDisableClientState'), null, 'void', 'unsigned int');
export function glDisableClientState(array) {
  return call('glDisableClientState', array);
}

define('glDrawArrays', dlsym(libGL, 'glDrawArrays'), null, 'void', 'unsigned int', 'int', 'int');
export function glDrawArrays(mode, first, count) {
  return call('glDrawArrays', mode, first, count);
}

define('glDrawBuffer', dlsym(libGL, 'glDrawBuffer'), null, 'void', 'unsigned int');
export function glDrawBuffer(mode) {
  return call('glDrawBuffer', mode);
}

define('glDrawElements', dlsym(libGL, 'glDrawElements'), null, 'void', 'unsigned int', 'int', 'unsigned int', 'void *');
export function glDrawElements(mode, count, type, indices) {
  return call('glDrawElements', mode, count, type, indices);
}

define(
  'glDrawPixels',
  dlsym(libGL, 'glDrawPixels'),
  null,
  'void',
  'int',
  'int',
  'unsigned int',
  'unsigned int',
  'void *'
);
export function glDrawPixels(width, height, format, type, pixels) {
  return call('glDrawPixels', width, height, format, type, pixels);
}

define('glEdgeFlag', dlsym(libGL, 'glEdgeFlag'), null, 'void', 'unsigned char');
export function glEdgeFlag(flag) {
  return call('glEdgeFlag', flag);
}

define('glEdgeFlagPointer', dlsym(libGL, 'glEdgeFlagPointer'), null, 'void', 'int', 'void *');
export function glEdgeFlagPointer(stride, pointer) {
  return call('glEdgeFlagPointer', stride, pointer);
}

define('glEdgeFlagv', dlsym(libGL, 'glEdgeFlagv'), null, 'void', 'void *');
export function glEdgeFlagv(flag) {
  return call('glEdgeFlagv', flag);
}

define('glEnable', dlsym(libGL, 'glEnable'), null, 'void', 'unsigned int');
export function glEnable(cap) {
  return call('glEnable', cap);
}

define('glEnableClientState', dlsym(libGL, 'glEnableClientState'), null, 'void', 'unsigned int');
export function glEnableClientState(array) {
  return call('glEnableClientState', array);
}

define('glEvalCoord1d', dlsym(libGL, 'glEvalCoord1d'), null, 'void', 'double');
export function glEvalCoord1d(u) {
  return call('glEvalCoord1d', u);
}

define('glEvalCoord1dv', dlsym(libGL, 'glEvalCoord1dv'), null, 'void', 'void *');
export function glEvalCoord1dv(u) {
  return call('glEvalCoord1dv', u);
}

define('glEvalCoord1f', dlsym(libGL, 'glEvalCoord1f'), null, 'void', 'float');
export function glEvalCoord1f(u) {
  return call('glEvalCoord1f', u);
}

define('glEvalCoord1fv', dlsym(libGL, 'glEvalCoord1fv'), null, 'void', 'void *');
export function glEvalCoord1fv(u) {
  return call('glEvalCoord1fv', u);
}

define('glEvalCoord2d', dlsym(libGL, 'glEvalCoord2d'), null, 'void', 'double', 'double');
export function glEvalCoord2d(u, v) {
  return call('glEvalCoord2d', u, v);
}

define('glEvalCoord2dv', dlsym(libGL, 'glEvalCoord2dv'), null, 'void', 'void *');
export function glEvalCoord2dv(u) {
  return call('glEvalCoord2dv', u);
}

define('glEvalCoord2f', dlsym(libGL, 'glEvalCoord2f'), null, 'void', 'float', 'float');
export function glEvalCoord2f(u, v) {
  return call('glEvalCoord2f', u, v);
}

define('glEvalCoord2fv', dlsym(libGL, 'glEvalCoord2fv'), null, 'void', 'void *');
export function glEvalCoord2fv(u) {
  return call('glEvalCoord2fv', u);
}

define('glEvalMesh1', dlsym(libGL, 'glEvalMesh1'), null, 'void', 'unsigned int', 'int', 'int');
export function glEvalMesh1(mode, i1, i2) {
  return call('glEvalMesh1', mode, i1, i2);
}

define('glEvalMesh2', dlsym(libGL, 'glEvalMesh2'), null, 'void', 'unsigned int', 'int', 'int', 'int', 'int');
export function glEvalMesh2(mode, i1, i2, j1, j2) {
  return call('glEvalMesh2', mode, i1, i2, j1, j2);
}

define('glEvalPoint1', dlsym(libGL, 'glEvalPoint1'), null, 'void', 'int');
export function glEvalPoint1(i) {
  return call('glEvalPoint1', i);
}

define('glEvalPoint2', dlsym(libGL, 'glEvalPoint2'), null, 'void', 'int', 'int');
export function glEvalPoint2(i, j) {
  return call('glEvalPoint2', i, j);
}

define('glFeedbackBuffer', dlsym(libGL, 'glFeedbackBuffer'), null, 'void', 'int', 'unsigned int', 'void *');
export function glFeedbackBuffer(size, type, buffer) {
  return call('glFeedbackBuffer', size, type, buffer);
}

define('glFogf', dlsym(libGL, 'glFogf'), null, 'void', 'unsigned int', 'float');
export function glFogf(pname, param) {
  return call('glFogf', pname, param);
}

define('glFogfv', dlsym(libGL, 'glFogfv'), null, 'void', 'unsigned int', 'void *');
export function glFogfv(pname, params) {
  return call('glFogfv', pname, params);
}

define('glFogi', dlsym(libGL, 'glFogi'), null, 'void', 'unsigned int', 'int');
export function glFogi(pname, param) {
  return call('glFogi', pname, param);
}

define('glFogiv', dlsym(libGL, 'glFogiv'), null, 'void', 'unsigned int', 'void *');
export function glFogiv(pname, params) {
  return call('glFogiv', pname, params);
}

define('glFrontFace', dlsym(libGL, 'glFrontFace'), null, 'void', 'unsigned int');
export function glFrontFace(mode) {
  return call('glFrontFace', mode);
}

define(
  'glFrustum',
  dlsym(libGL, 'glFrustum'),
  null,
  'void',
  'double',
  'double',
  'double',
  'double',
  'double',
  'double'
);
export function glFrustum(left, right, bottom, top, zNear, zFar) {
  return call('glFrustum', left, right, bottom, top, zNear, zFar);
}

define('glGenLists', dlsym(libGL, 'glGenLists'), null, 'unsigned int', 'int');
export function glGenLists(range) {
  return call('glGenLists', range);
}

define('glGenTextures', dlsym(libGL, 'glGenTextures'), null, 'void', 'int', 'void *');
export function glGenTextures(n, textures) {
  return call('glGenTextures', n, textures);
}

define('glGetBooleanv', dlsym(libGL, 'glGetBooleanv'), null, 'void', 'unsigned int', 'void *');
export function glGetBooleanv(pname, params) {
  return call('glGetBooleanv', pname, params);
}

define('glGetClipPlane', dlsym(libGL, 'glGetClipPlane'), null, 'void', 'unsigned int', 'void *');
export function glGetClipPlane(plane, equation) {
  return call('glGetClipPlane', plane, equation);
}

define('glGetDoublev', dlsym(libGL, 'glGetDoublev'), null, 'void', 'unsigned int', 'void *');
export function glGetDoublev(pname, params) {
  return call('glGetDoublev', pname, params);
}

define('glGetFloatv', dlsym(libGL, 'glGetFloatv'), null, 'void', 'unsigned int', 'void *');
export function glGetFloatv(pname, params) {
  return call('glGetFloatv', pname, params);
}

define('glGetIntegerv', dlsym(libGL, 'glGetIntegerv'), null, 'void', 'unsigned int', 'void *');
export function glGetIntegerv(pname, params) {
  return call('glGetIntegerv', pname, params);
}

define('glGetLightfv', dlsym(libGL, 'glGetLightfv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glGetLightfv(light, pname, params) {
  return call('glGetLightfv', light, pname, params);
}

define('glGetLightiv', dlsym(libGL, 'glGetLightiv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glGetLightiv(light, pname, params) {
  return call('glGetLightiv', light, pname, params);
}

define('glGetMapdv', dlsym(libGL, 'glGetMapdv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glGetMapdv(target, query, v) {
  return call('glGetMapdv', target, query, v);
}

define('glGetMapfv', dlsym(libGL, 'glGetMapfv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glGetMapfv(target, query, v) {
  return call('glGetMapfv', target, query, v);
}

define('glGetMapiv', dlsym(libGL, 'glGetMapiv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glGetMapiv(target, query, v) {
  return call('glGetMapiv', target, query, v);
}

define('glGetMaterialfv', dlsym(libGL, 'glGetMaterialfv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glGetMaterialfv(face, pname, params) {
  return call('glGetMaterialfv', face, pname, params);
}

define('glGetMaterialiv', dlsym(libGL, 'glGetMaterialiv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glGetMaterialiv(face, pname, params) {
  return call('glGetMaterialiv', face, pname, params);
}

define('glGetPixelMapfv', dlsym(libGL, 'glGetPixelMapfv'), null, 'void', 'unsigned int', 'void *');
export function glGetPixelMapfv(map, values) {
  return call('glGetPixelMapfv', map, values);
}

define('glGetPixelMapuiv', dlsym(libGL, 'glGetPixelMapuiv'), null, 'void', 'unsigned int', 'void *');
export function glGetPixelMapuiv(map, values) {
  return call('glGetPixelMapuiv', map, values);
}

define('glGetPixelMapusv', dlsym(libGL, 'glGetPixelMapusv'), null, 'void', 'unsigned int', 'void *');
export function glGetPixelMapusv(map, values) {
  return call('glGetPixelMapusv', map, values);
}

define('glGetPointerv', dlsym(libGL, 'glGetPointerv'), null, 'void', 'unsigned int', 'void *');
export function glGetPointerv(pname, params) {
  return call('glGetPointerv', pname, params);
}

define('glGetPolygonStipple', dlsym(libGL, 'glGetPolygonStipple'), null, 'void', 'void *');
export function glGetPolygonStipple(mask) {
  return call('glGetPolygonStipple', mask);
}

define('glGetString', dlsym(libGL, 'glGetString'), null, 'void *', 'unsigned int');
export function glGetString(name) {
  return call('glGetString', name);
}

define('glGetTexEnvfv', dlsym(libGL, 'glGetTexEnvfv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glGetTexEnvfv(target, pname, params) {
  return call('glGetTexEnvfv', target, pname, params);
}

define('glGetTexEnviv', dlsym(libGL, 'glGetTexEnviv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glGetTexEnviv(target, pname, params) {
  return call('glGetTexEnviv', target, pname, params);
}

define('glGetTexGendv', dlsym(libGL, 'glGetTexGendv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glGetTexGendv(coord, pname, params) {
  return call('glGetTexGendv', coord, pname, params);
}

define('glGetTexGenfv', dlsym(libGL, 'glGetTexGenfv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glGetTexGenfv(coord, pname, params) {
  return call('glGetTexGenfv', coord, pname, params);
}

define('glGetTexGeniv', dlsym(libGL, 'glGetTexGeniv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glGetTexGeniv(coord, pname, params) {
  return call('glGetTexGeniv', coord, pname, params);
}

define(
  'glGetTexImage',
  dlsym(libGL, 'glGetTexImage'),
  null,
  'void',
  'unsigned int',
  'int',
  'unsigned int',
  'unsigned int',
  'void *'
);
export function glGetTexImage(target, level, format, type, pixels) {
  return call('glGetTexImage', target, level, format, type, pixels);
}

define(
  'glGetTexLevelParameterfv',
  dlsym(libGL, 'glGetTexLevelParameterfv'),
  null,
  'void',
  'unsigned int',
  'int',
  'unsigned int',
  'void *'
);
export function glGetTexLevelParameterfv(target, level, pname, params) {
  return call('glGetTexLevelParameterfv', target, level, pname, params);
}

define(
  'glGetTexLevelParameteriv',
  dlsym(libGL, 'glGetTexLevelParameteriv'),
  null,
  'void',
  'unsigned int',
  'int',
  'unsigned int',
  'void *'
);
export function glGetTexLevelParameteriv(target, level, pname, params) {
  return call('glGetTexLevelParameteriv', target, level, pname, params);
}

define(
  'glGetTexParameterfv',
  dlsym(libGL, 'glGetTexParameterfv'),
  null,
  'void',
  'unsigned int',
  'unsigned int',
  'void *'
);
export function glGetTexParameterfv(target, pname, params) {
  return call('glGetTexParameterfv', target, pname, params);
}

define(
  'glGetTexParameteriv',
  dlsym(libGL, 'glGetTexParameteriv'),
  null,
  'void',
  'unsigned int',
  'unsigned int',
  'void *'
);
export function glGetTexParameteriv(target, pname, params) {
  return call('glGetTexParameteriv', target, pname, params);
}

define('glHint', dlsym(libGL, 'glHint'), null, 'void', 'unsigned int', 'unsigned int');
export function glHint(target, mode) {
  return call('glHint', target, mode);
}

define('glIndexd', dlsym(libGL, 'glIndexd'), null, 'void', 'double');
export function glIndexd(c) {
  return call('glIndexd', c);
}

define('glIndexdv', dlsym(libGL, 'glIndexdv'), null, 'void', 'void *');
export function glIndexdv(c) {
  return call('glIndexdv', c);
}

define('glIndexf', dlsym(libGL, 'glIndexf'), null, 'void', 'float');
export function glIndexf(c) {
  return call('glIndexf', c);
}

define('glIndexfv', dlsym(libGL, 'glIndexfv'), null, 'void', 'void *');
export function glIndexfv(c) {
  return call('glIndexfv', c);
}

define('glIndexi', dlsym(libGL, 'glIndexi'), null, 'void', 'int');
export function glIndexi(c) {
  return call('glIndexi', c);
}

define('glIndexiv', dlsym(libGL, 'glIndexiv'), null, 'void', 'void *');
export function glIndexiv(c) {
  return call('glIndexiv', c);
}

define('glIndexMask', dlsym(libGL, 'glIndexMask'), null, 'void', 'unsigned int');
export function glIndexMask(mask) {
  return call('glIndexMask', mask);
}

define('glIndexPointer', dlsym(libGL, 'glIndexPointer'), null, 'void', 'unsigned int', 'int', 'void *');
export function glIndexPointer(type, stride, pointer) {
  return call('glIndexPointer', type, stride, pointer);
}

define('glIndexs', dlsym(libGL, 'glIndexs'), null, 'void', 'short');
export function glIndexs(c) {
  return call('glIndexs', c);
}

define('glIndexsv', dlsym(libGL, 'glIndexsv'), null, 'void', 'void *');
export function glIndexsv(c) {
  return call('glIndexsv', c);
}

define('glIndexub', dlsym(libGL, 'glIndexub'), null, 'void', 'unsigned char');
export function glIndexub(c) {
  return call('glIndexub', c);
}

define('glIndexubv', dlsym(libGL, 'glIndexubv'), null, 'void', 'void *');
export function glIndexubv(c) {
  return call('glIndexubv', c);
}

define('glInterleavedArrays', dlsym(libGL, 'glInterleavedArrays'), null, 'void', 'unsigned int', 'int', 'void *');
export function glInterleavedArrays(format, stride, pointer) {
  return call('glInterleavedArrays', format, stride, pointer);
}

define('glIsEnabled', dlsym(libGL, 'glIsEnabled'), null, 'unsigned char', 'unsigned int');
export function glIsEnabled(cap) {
  return call('glIsEnabled', cap);
}

define('glIsList', dlsym(libGL, 'glIsList'), null, 'unsigned char', 'unsigned int');
export function glIsList(list) {
  return call('glIsList', list);
}

define('glIsTexture', dlsym(libGL, 'glIsTexture'), null, 'unsigned char', 'unsigned int');
export function glIsTexture(texture) {
  return call('glIsTexture', texture);
}

define('glLightf', dlsym(libGL, 'glLightf'), null, 'void', 'unsigned int', 'unsigned int', 'float');
export function glLightf(light, pname, param) {
  return call('glLightf', light, pname, param);
}

define('glLightfv', dlsym(libGL, 'glLightfv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glLightfv(light, pname, params) {
  return call('glLightfv', light, pname, params);
}

define('glLighti', dlsym(libGL, 'glLighti'), null, 'void', 'unsigned int', 'unsigned int', 'int');
export function glLighti(light, pname, param) {
  return call('glLighti', light, pname, param);
}

define('glLightiv', dlsym(libGL, 'glLightiv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glLightiv(light, pname, params) {
  return call('glLightiv', light, pname, params);
}

define('glLightModelf', dlsym(libGL, 'glLightModelf'), null, 'void', 'unsigned int', 'float');
export function glLightModelf(pname, param) {
  return call('glLightModelf', pname, param);
}

define('glLightModelfv', dlsym(libGL, 'glLightModelfv'), null, 'void', 'unsigned int', 'void *');
export function glLightModelfv(pname, params) {
  return call('glLightModelfv', pname, params);
}

define('glLightModeli', dlsym(libGL, 'glLightModeli'), null, 'void', 'unsigned int', 'int');
export function glLightModeli(pname, param) {
  return call('glLightModeli', pname, param);
}

define('glLightModeliv', dlsym(libGL, 'glLightModeliv'), null, 'void', 'unsigned int', 'void *');
export function glLightModeliv(pname, params) {
  return call('glLightModeliv', pname, params);
}

define('glLineStipple', dlsym(libGL, 'glLineStipple'), null, 'void', 'int', 'uint16');
export function glLineStipple(factor, pattern) {
  return call('glLineStipple', factor, pattern);
}

define('glLineWidth', dlsym(libGL, 'glLineWidth'), null, 'void', 'float');
export function glLineWidth(width) {
  return call('glLineWidth', width);
}

define('glListBase', dlsym(libGL, 'glListBase'), null, 'void', 'unsigned int');
export function glListBase(base) {
  return call('glListBase', base);
}

define('glLoadMatrixd', dlsym(libGL, 'glLoadMatrixd'), null, 'void', 'void *');
export function glLoadMatrixd(m) {
  return call('glLoadMatrixd', m);
}

define('glLoadMatrixf', dlsym(libGL, 'glLoadMatrixf'), null, 'void', 'void *');
export function glLoadMatrixf(m) {
  return call('glLoadMatrixf', m);
}

define('glLoadName', dlsym(libGL, 'glLoadName'), null, 'void', 'unsigned int');
export function glLoadName(name) {
  return call('glLoadName', name);
}

define('glLogicOp', dlsym(libGL, 'glLogicOp'), null, 'void', 'unsigned int');
export function glLogicOp(opcode) {
  return call('glLogicOp', opcode);
}

define('glMap1d', dlsym(libGL, 'glMap1d'), null, 'void', 'unsigned int', 'double', 'double', 'int', 'int', 'void *');
export function glMap1d(target, u1, u2, stride, order, points) {
  return call('glMap1d', target, u1, u2, stride, order, points);
}

define('glMap1f', dlsym(libGL, 'glMap1f'), null, 'void', 'unsigned int', 'float', 'float', 'int', 'int', 'void *');
export function glMap1f(target, u1, u2, stride, order, points) {
  return call('glMap1f', target, u1, u2, stride, order, points);
}

define(
  'glMap2d',
  dlsym(libGL, 'glMap2d'),
  null,
  'void',
  'unsigned int',
  'double',
  'double',
  'int',
  'int',
  'double',
  'double',
  'int',
  'int',
  'void *'
);
export function glMap2d(target, u1, u2, ustride, uorder, v1, v2, vstride, vorder, points) {
  return call('glMap2d', target, u1, u2, ustride, uorder, v1, v2, vstride, vorder, points);
}

define(
  'glMap2f',
  dlsym(libGL, 'glMap2f'),
  null,
  'void',
  'unsigned int',
  'float',
  'float',
  'int',
  'int',
  'float',
  'float',
  'int',
  'int',
  'void *'
);
export function glMap2f(target, u1, u2, ustride, uorder, v1, v2, vstride, vorder, points) {
  return call('glMap2f', target, u1, u2, ustride, uorder, v1, v2, vstride, vorder, points);
}

define('glMapGrid1d', dlsym(libGL, 'glMapGrid1d'), null, 'void', 'int', 'double', 'double');
export function glMapGrid1d(un, u1, u2) {
  return call('glMapGrid1d', un, u1, u2);
}

define('glMapGrid1f', dlsym(libGL, 'glMapGrid1f'), null, 'void', 'int', 'float', 'float');
export function glMapGrid1f(un, u1, u2) {
  return call('glMapGrid1f', un, u1, u2);
}

define('glMapGrid2d', dlsym(libGL, 'glMapGrid2d'), null, 'void', 'int', 'double', 'double', 'int', 'double', 'double');
export function glMapGrid2d(un, u1, u2, vn, v1, v2) {
  return call('glMapGrid2d', un, u1, u2, vn, v1, v2);
}

define('glMapGrid2f', dlsym(libGL, 'glMapGrid2f'), null, 'void', 'int', 'float', 'float', 'int', 'float', 'float');
export function glMapGrid2f(un, u1, u2, vn, v1, v2) {
  return call('glMapGrid2f', un, u1, u2, vn, v1, v2);
}

define('glMaterialf', dlsym(libGL, 'glMaterialf'), null, 'void', 'unsigned int', 'unsigned int', 'float');
export function glMaterialf(face, pname, param) {
  return call('glMaterialf', face, pname, param);
}

define('glMaterialfv', dlsym(libGL, 'glMaterialfv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glMaterialfv(face, pname, params) {
  return call('glMaterialfv', face, pname, params);
}

define('glMateriali', dlsym(libGL, 'glMateriali'), null, 'void', 'unsigned int', 'unsigned int', 'int');
export function glMateriali(face, pname, param) {
  return call('glMateriali', face, pname, param);
}

define('glMaterialiv', dlsym(libGL, 'glMaterialiv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glMaterialiv(face, pname, params) {
  return call('glMaterialiv', face, pname, params);
}

define('glMatrixMode', dlsym(libGL, 'glMatrixMode'), null, 'void', 'unsigned int');
export function glMatrixMode(mode) {
  return call('glMatrixMode', mode);
}

define('glMultMatrixd', dlsym(libGL, 'glMultMatrixd'), null, 'void', 'void *');
export function glMultMatrixd(m) {
  return call('glMultMatrixd', m);
}

define('glMultMatrixf', dlsym(libGL, 'glMultMatrixf'), null, 'void', 'void *');
export function glMultMatrixf(m) {
  return call('glMultMatrixf', m);
}

define('glNewList', dlsym(libGL, 'glNewList'), null, 'void', 'unsigned int', 'unsigned int');
export function glNewList(list, mode) {
  return call('glNewList', list, mode);
}

define('glNormal3b', dlsym(libGL, 'glNormal3b'), null, 'void', 'sint8', 'sint8', 'sint8');
export function glNormal3b(nx, ny, nz) {
  return call('glNormal3b', nx, ny, nz);
}

define('glNormal3bv', dlsym(libGL, 'glNormal3bv'), null, 'void', 'void *');
export function glNormal3bv(v) {
  return call('glNormal3bv', v);
}

define('glNormal3d', dlsym(libGL, 'glNormal3d'), null, 'void', 'double', 'double', 'double');
export function glNormal3d(nx, ny, nz) {
  return call('glNormal3d', nx, ny, nz);
}

define('glNormal3dv', dlsym(libGL, 'glNormal3dv'), null, 'void', 'void *');
export function glNormal3dv(v) {
  return call('glNormal3dv', v);
}

define('glNormal3f', dlsym(libGL, 'glNormal3f'), null, 'void', 'float', 'float', 'float');
export function glNormal3f(nx, ny, nz) {
  return call('glNormal3f', nx, ny, nz);
}

define('glNormal3fv', dlsym(libGL, 'glNormal3fv'), null, 'void', 'void *');
export function glNormal3fv(v) {
  return call('glNormal3fv', v);
}

define('glNormal3i', dlsym(libGL, 'glNormal3i'), null, 'void', 'int', 'int', 'int');
export function glNormal3i(nx, ny, nz) {
  return call('glNormal3i', nx, ny, nz);
}

define('glNormal3iv', dlsym(libGL, 'glNormal3iv'), null, 'void', 'void *');
export function glNormal3iv(v) {
  return call('glNormal3iv', v);
}

define('glNormal3s', dlsym(libGL, 'glNormal3s'), null, 'void', 'short', 'short', 'short');
export function glNormal3s(nx, ny, nz) {
  return call('glNormal3s', nx, ny, nz);
}

define('glNormal3sv', dlsym(libGL, 'glNormal3sv'), null, 'void', 'void *');
export function glNormal3sv(v) {
  return call('glNormal3sv', v);
}

define('glNormalPointer', dlsym(libGL, 'glNormalPointer'), null, 'void', 'unsigned int', 'int', 'void *');
export function glNormalPointer(type, stride, pointer) {
  return call('glNormalPointer', type, stride, pointer);
}

define('glOrtho', dlsym(libGL, 'glOrtho'), null, 'void', 'double', 'double', 'double', 'double', 'double', 'double');
export function glOrtho(left, right, bottom, top, zNear, zFar) {
  return call('glOrtho', left, right, bottom, top, zNear, zFar);
}

define('glPassThrough', dlsym(libGL, 'glPassThrough'), null, 'void', 'float');
export function glPassThrough(token) {
  return call('glPassThrough', token);
}

define('glPixelMapfv', dlsym(libGL, 'glPixelMapfv'), null, 'void', 'unsigned int', 'int', 'void *');
export function glPixelMapfv(map, mapsize, values) {
  return call('glPixelMapfv', map, mapsize, values);
}

define('glPixelMapuiv', dlsym(libGL, 'glPixelMapuiv'), null, 'void', 'unsigned int', 'int', 'void *');
export function glPixelMapuiv(map, mapsize, values) {
  return call('glPixelMapuiv', map, mapsize, values);
}

define('glPixelMapusv', dlsym(libGL, 'glPixelMapusv'), null, 'void', 'unsigned int', 'int', 'void *');
export function glPixelMapusv(map, mapsize, values) {
  return call('glPixelMapusv', map, mapsize, values);
}

define('glPixelStoref', dlsym(libGL, 'glPixelStoref'), null, 'void', 'unsigned int', 'float');
export function glPixelStoref(pname, param) {
  return call('glPixelStoref', pname, param);
}

define('glPixelStorei', dlsym(libGL, 'glPixelStorei'), null, 'void', 'unsigned int', 'int');
export function glPixelStorei(pname, param) {
  return call('glPixelStorei', pname, param);
}

define('glPixelTransferf', dlsym(libGL, 'glPixelTransferf'), null, 'void', 'unsigned int', 'float');
export function glPixelTransferf(pname, param) {
  return call('glPixelTransferf', pname, param);
}

define('glPixelTransferi', dlsym(libGL, 'glPixelTransferi'), null, 'void', 'unsigned int', 'int');
export function glPixelTransferi(pname, param) {
  return call('glPixelTransferi', pname, param);
}

define('glPixelZoom', dlsym(libGL, 'glPixelZoom'), null, 'void', 'float', 'float');
export function glPixelZoom(xfactor, yfactor) {
  return call('glPixelZoom', xfactor, yfactor);
}

define('glPointSize', dlsym(libGL, 'glPointSize'), null, 'void', 'float');
export function glPointSize(size) {
  return call('glPointSize', size);
}

define('glPolygonMode', dlsym(libGL, 'glPolygonMode'), null, 'void', 'unsigned int', 'unsigned int');
export function glPolygonMode(face, mode) {
  return call('glPolygonMode', face, mode);
}

define('glPolygonOffset', dlsym(libGL, 'glPolygonOffset'), null, 'void', 'float', 'float');
export function glPolygonOffset(factor, units) {
  return call('glPolygonOffset', factor, units);
}

define('glPolygonStipple', dlsym(libGL, 'glPolygonStipple'), null, 'void', 'void *');
export function glPolygonStipple(mask) {
  return call('glPolygonStipple', mask);
}

define('glPrioritizeTextures', dlsym(libGL, 'glPrioritizeTextures'), null, 'void', 'int', 'void *', 'void *');
export function glPrioritizeTextures(n, textures, priorities) {
  return call('glPrioritizeTextures', n, textures, priorities);
}

define('glPushAttrib', dlsym(libGL, 'glPushAttrib'), null, 'void', 'unsigned int');
export function glPushAttrib(mask) {
  return call('glPushAttrib', mask);
}

define('glPushClientAttrib', dlsym(libGL, 'glPushClientAttrib'), null, 'void', 'unsigned int');
export function glPushClientAttrib(mask) {
  return call('glPushClientAttrib', mask);
}

define('glPushName', dlsym(libGL, 'glPushName'), null, 'void', 'unsigned int');
export function glPushName(name) {
  return call('glPushName', name);
}

define('glRasterPos2d', dlsym(libGL, 'glRasterPos2d'), null, 'void', 'double', 'double');
export function glRasterPos2d(x, y) {
  return call('glRasterPos2d', x, y);
}

define('glRasterPos2dv', dlsym(libGL, 'glRasterPos2dv'), null, 'void', 'void *');
export function glRasterPos2dv(v) {
  return call('glRasterPos2dv', v);
}

define('glRasterPos2f', dlsym(libGL, 'glRasterPos2f'), null, 'void', 'float', 'float');
export function glRasterPos2f(x, y) {
  return call('glRasterPos2f', x, y);
}

define('glRasterPos2fv', dlsym(libGL, 'glRasterPos2fv'), null, 'void', 'void *');
export function glRasterPos2fv(v) {
  return call('glRasterPos2fv', v);
}

define('glRasterPos2i', dlsym(libGL, 'glRasterPos2i'), null, 'void', 'int', 'int');
export function glRasterPos2i(x, y) {
  return call('glRasterPos2i', x, y);
}

define('glRasterPos2iv', dlsym(libGL, 'glRasterPos2iv'), null, 'void', 'void *');
export function glRasterPos2iv(v) {
  return call('glRasterPos2iv', v);
}

define('glRasterPos2s', dlsym(libGL, 'glRasterPos2s'), null, 'void', 'short', 'short');
export function glRasterPos2s(x, y) {
  return call('glRasterPos2s', x, y);
}

define('glRasterPos2sv', dlsym(libGL, 'glRasterPos2sv'), null, 'void', 'void *');
export function glRasterPos2sv(v) {
  return call('glRasterPos2sv', v);
}

define('glRasterPos3d', dlsym(libGL, 'glRasterPos3d'), null, 'void', 'double', 'double', 'double');
export function glRasterPos3d(x, y, z) {
  return call('glRasterPos3d', x, y, z);
}

define('glRasterPos3dv', dlsym(libGL, 'glRasterPos3dv'), null, 'void', 'void *');
export function glRasterPos3dv(v) {
  return call('glRasterPos3dv', v);
}

define('glRasterPos3f', dlsym(libGL, 'glRasterPos3f'), null, 'void', 'float', 'float', 'float');
export function glRasterPos3f(x, y, z) {
  return call('glRasterPos3f', x, y, z);
}

define('glRasterPos3fv', dlsym(libGL, 'glRasterPos3fv'), null, 'void', 'void *');
export function glRasterPos3fv(v) {
  return call('glRasterPos3fv', v);
}

define('glRasterPos3i', dlsym(libGL, 'glRasterPos3i'), null, 'void', 'int', 'int', 'int');
export function glRasterPos3i(x, y, z) {
  return call('glRasterPos3i', x, y, z);
}

define('glRasterPos3iv', dlsym(libGL, 'glRasterPos3iv'), null, 'void', 'void *');
export function glRasterPos3iv(v) {
  return call('glRasterPos3iv', v);
}

define('glRasterPos3s', dlsym(libGL, 'glRasterPos3s'), null, 'void', 'short', 'short', 'short');
export function glRasterPos3s(x, y, z) {
  return call('glRasterPos3s', x, y, z);
}

define('glRasterPos3sv', dlsym(libGL, 'glRasterPos3sv'), null, 'void', 'void *');
export function glRasterPos3sv(v) {
  return call('glRasterPos3sv', v);
}

define('glRasterPos4d', dlsym(libGL, 'glRasterPos4d'), null, 'void', 'double', 'double', 'double', 'double');
export function glRasterPos4d(x, y, z, w) {
  return call('glRasterPos4d', x, y, z, w);
}

define('glRasterPos4dv', dlsym(libGL, 'glRasterPos4dv'), null, 'void', 'void *');
export function glRasterPos4dv(v) {
  return call('glRasterPos4dv', v);
}

define('glRasterPos4f', dlsym(libGL, 'glRasterPos4f'), null, 'void', 'float', 'float', 'float', 'float');
export function glRasterPos4f(x, y, z, w) {
  return call('glRasterPos4f', x, y, z, w);
}

define('glRasterPos4fv', dlsym(libGL, 'glRasterPos4fv'), null, 'void', 'void *');
export function glRasterPos4fv(v) {
  return call('glRasterPos4fv', v);
}

define('glRasterPos4i', dlsym(libGL, 'glRasterPos4i'), null, 'void', 'int', 'int', 'int', 'int');
export function glRasterPos4i(x, y, z, w) {
  return call('glRasterPos4i', x, y, z, w);
}

define('glRasterPos4iv', dlsym(libGL, 'glRasterPos4iv'), null, 'void', 'void *');
export function glRasterPos4iv(v) {
  return call('glRasterPos4iv', v);
}

define('glRasterPos4s', dlsym(libGL, 'glRasterPos4s'), null, 'void', 'short', 'short', 'short', 'short');
export function glRasterPos4s(x, y, z, w) {
  return call('glRasterPos4s', x, y, z, w);
}

define('glRasterPos4sv', dlsym(libGL, 'glRasterPos4sv'), null, 'void', 'void *');
export function glRasterPos4sv(v) {
  return call('glRasterPos4sv', v);
}

define('glReadBuffer', dlsym(libGL, 'glReadBuffer'), null, 'void', 'unsigned int');
export function glReadBuffer(mode) {
  return call('glReadBuffer', mode);
}

define(
  'glReadPixels',
  dlsym(libGL, 'glReadPixels'),
  null,
  'void',
  'int',
  'int',
  'int',
  'int',
  'unsigned int',
  'unsigned int',
  'void *'
);
export function glReadPixels(x, y, width, height, format, type, pixels) {
  return call('glReadPixels', x, y, width, height, format, type, pixels);
}

define('glRectd', dlsym(libGL, 'glRectd'), null, 'void', 'double', 'double', 'double', 'double');
export function glRectd(x1, y1, x2, y2) {
  return call('glRectd', x1, y1, x2, y2);
}

define('glRectdv', dlsym(libGL, 'glRectdv'), null, 'void', 'void *', 'void *');
export function glRectdv(v1, v2) {
  return call('glRectdv', v1, v2);
}

define('glRectf', dlsym(libGL, 'glRectf'), null, 'void', 'float', 'float', 'float', 'float');
export function glRectf(x1, y1, x2, y2) {
  return call('glRectf', x1, y1, x2, y2);
}

define('glRectfv', dlsym(libGL, 'glRectfv'), null, 'void', 'void *', 'void *');
export function glRectfv(v1, v2) {
  return call('glRectfv', v1, v2);
}

define('glRecti', dlsym(libGL, 'glRecti'), null, 'void', 'int', 'int', 'int', 'int');
export function glRecti(x1, y1, x2, y2) {
  return call('glRecti', x1, y1, x2, y2);
}

define('glRectiv', dlsym(libGL, 'glRectiv'), null, 'void', 'void *', 'void *');
export function glRectiv(v1, v2) {
  return call('glRectiv', v1, v2);
}

define('glRects', dlsym(libGL, 'glRects'), null, 'void', 'short', 'short', 'short', 'short');
export function glRects(x1, y1, x2, y2) {
  return call('glRects', x1, y1, x2, y2);
}

define('glRectsv', dlsym(libGL, 'glRectsv'), null, 'void', 'void *', 'void *');
export function glRectsv(v1, v2) {
  return call('glRectsv', v1, v2);
}

define('glRenderMode', dlsym(libGL, 'glRenderMode'), null, 'int', 'unsigned int');
export function glRenderMode(mode) {
  return call('glRenderMode', mode);
}

define('glRotated', dlsym(libGL, 'glRotated'), null, 'void', 'double', 'double', 'double', 'double');
export function glRotated(angle, x, y, z) {
  return call('glRotated', angle, x, y, z);
}

define('glRotatef', dlsym(libGL, 'glRotatef'), null, 'void', 'float', 'float', 'float', 'float');
export function glRotatef(angle, x, y, z) {
  return call('glRotatef', angle, x, y, z);
}

define('glScaled', dlsym(libGL, 'glScaled'), null, 'void', 'double', 'double', 'double');
export function glScaled(x, y, z) {
  return call('glScaled', x, y, z);
}

define('glScalef', dlsym(libGL, 'glScalef'), null, 'void', 'float', 'float', 'float');
export function glScalef(x, y, z) {
  return call('glScalef', x, y, z);
}

define('glScissor', dlsym(libGL, 'glScissor'), null, 'void', 'int', 'int', 'int', 'int');
export function glScissor(x, y, width, height) {
  return call('glScissor', x, y, width, height);
}

define('glSelectBuffer', dlsym(libGL, 'glSelectBuffer'), null, 'void', 'int', 'void *');
export function glSelectBuffer(size, buffer) {
  return call('glSelectBuffer', size, buffer);
}

define('glShadeModel', dlsym(libGL, 'glShadeModel'), null, 'void', 'unsigned int');
export function glShadeModel(mode) {
  return call('glShadeModel', mode);
}

define('glStencilFunc', dlsym(libGL, 'glStencilFunc'), null, 'void', 'unsigned int', 'int', 'unsigned int');
export function glStencilFunc(func, ref, mask) {
  return call('glStencilFunc', func, ref, mask);
}

define('glStencilMask', dlsym(libGL, 'glStencilMask'), null, 'void', 'unsigned int');
export function glStencilMask(mask) {
  return call('glStencilMask', mask);
}

define('glStencilOp', dlsym(libGL, 'glStencilOp'), null, 'void', 'unsigned int', 'unsigned int', 'unsigned int');
export function glStencilOp(fail, zfail, zpass) {
  return call('glStencilOp', fail, zfail, zpass);
}

define('glTexCoord1d', dlsym(libGL, 'glTexCoord1d'), null, 'void', 'double');
export function glTexCoord1d(s) {
  return call('glTexCoord1d', s);
}

define('glTexCoord1dv', dlsym(libGL, 'glTexCoord1dv'), null, 'void', 'void *');
export function glTexCoord1dv(v) {
  return call('glTexCoord1dv', v);
}

define('glTexCoord1f', dlsym(libGL, 'glTexCoord1f'), null, 'void', 'float');
export function glTexCoord1f(s) {
  return call('glTexCoord1f', s);
}

define('glTexCoord1fv', dlsym(libGL, 'glTexCoord1fv'), null, 'void', 'void *');
export function glTexCoord1fv(v) {
  return call('glTexCoord1fv', v);
}

define('glTexCoord1i', dlsym(libGL, 'glTexCoord1i'), null, 'void', 'int');
export function glTexCoord1i(s) {
  return call('glTexCoord1i', s);
}

define('glTexCoord1iv', dlsym(libGL, 'glTexCoord1iv'), null, 'void', 'void *');
export function glTexCoord1iv(v) {
  return call('glTexCoord1iv', v);
}

define('glTexCoord1s', dlsym(libGL, 'glTexCoord1s'), null, 'void', 'short');
export function glTexCoord1s(s) {
  return call('glTexCoord1s', s);
}

define('glTexCoord1sv', dlsym(libGL, 'glTexCoord1sv'), null, 'void', 'void *');
export function glTexCoord1sv(v) {
  return call('glTexCoord1sv', v);
}

define('glTexCoord2d', dlsym(libGL, 'glTexCoord2d'), null, 'void', 'double', 'double');
export function glTexCoord2d(s, t) {
  return call('glTexCoord2d', s, t);
}

define('glTexCoord2dv', dlsym(libGL, 'glTexCoord2dv'), null, 'void', 'void *');
export function glTexCoord2dv(v) {
  return call('glTexCoord2dv', v);
}

define('glTexCoord2f', dlsym(libGL, 'glTexCoord2f'), null, 'void', 'float', 'float');
export function glTexCoord2f(s, t) {
  return call('glTexCoord2f', s, t);
}

define('glTexCoord2fv', dlsym(libGL, 'glTexCoord2fv'), null, 'void', 'void *');
export function glTexCoord2fv(v) {
  return call('glTexCoord2fv', v);
}

define('glTexCoord2i', dlsym(libGL, 'glTexCoord2i'), null, 'void', 'int', 'int');
export function glTexCoord2i(s, t) {
  return call('glTexCoord2i', s, t);
}

define('glTexCoord2iv', dlsym(libGL, 'glTexCoord2iv'), null, 'void', 'void *');
export function glTexCoord2iv(v) {
  return call('glTexCoord2iv', v);
}

define('glTexCoord2s', dlsym(libGL, 'glTexCoord2s'), null, 'void', 'short', 'short');
export function glTexCoord2s(s, t) {
  return call('glTexCoord2s', s, t);
}

define('glTexCoord2sv', dlsym(libGL, 'glTexCoord2sv'), null, 'void', 'void *');
export function glTexCoord2sv(v) {
  return call('glTexCoord2sv', v);
}

define('glTexCoord3d', dlsym(libGL, 'glTexCoord3d'), null, 'void', 'double', 'double', 'double');
export function glTexCoord3d(s, t, r) {
  return call('glTexCoord3d', s, t, r);
}

define('glTexCoord3dv', dlsym(libGL, 'glTexCoord3dv'), null, 'void', 'void *');
export function glTexCoord3dv(v) {
  return call('glTexCoord3dv', v);
}

define('glTexCoord3f', dlsym(libGL, 'glTexCoord3f'), null, 'void', 'float', 'float', 'float');
export function glTexCoord3f(s, t, r) {
  return call('glTexCoord3f', s, t, r);
}

define('glTexCoord3fv', dlsym(libGL, 'glTexCoord3fv'), null, 'void', 'void *');
export function glTexCoord3fv(v) {
  return call('glTexCoord3fv', v);
}

define('glTexCoord3i', dlsym(libGL, 'glTexCoord3i'), null, 'void', 'int', 'int', 'int');
export function glTexCoord3i(s, t, r) {
  return call('glTexCoord3i', s, t, r);
}

define('glTexCoord3iv', dlsym(libGL, 'glTexCoord3iv'), null, 'void', 'void *');
export function glTexCoord3iv(v) {
  return call('glTexCoord3iv', v);
}

define('glTexCoord3s', dlsym(libGL, 'glTexCoord3s'), null, 'void', 'short', 'short', 'short');
export function glTexCoord3s(s, t, r) {
  return call('glTexCoord3s', s, t, r);
}

define('glTexCoord3sv', dlsym(libGL, 'glTexCoord3sv'), null, 'void', 'void *');
export function glTexCoord3sv(v) {
  return call('glTexCoord3sv', v);
}

define('glTexCoord4d', dlsym(libGL, 'glTexCoord4d'), null, 'void', 'double', 'double', 'double', 'double');
export function glTexCoord4d(s, t, r, q) {
  return call('glTexCoord4d', s, t, r, q);
}

define('glTexCoord4dv', dlsym(libGL, 'glTexCoord4dv'), null, 'void', 'void *');
export function glTexCoord4dv(v) {
  return call('glTexCoord4dv', v);
}

define('glTexCoord4f', dlsym(libGL, 'glTexCoord4f'), null, 'void', 'float', 'float', 'float', 'float');
export function glTexCoord4f(s, t, r, q) {
  return call('glTexCoord4f', s, t, r, q);
}

define('glTexCoord4fv', dlsym(libGL, 'glTexCoord4fv'), null, 'void', 'void *');
export function glTexCoord4fv(v) {
  return call('glTexCoord4fv', v);
}

define('glTexCoord4i', dlsym(libGL, 'glTexCoord4i'), null, 'void', 'int', 'int', 'int', 'int');
export function glTexCoord4i(s, t, r, q) {
  return call('glTexCoord4i', s, t, r, q);
}

define('glTexCoord4iv', dlsym(libGL, 'glTexCoord4iv'), null, 'void', 'void *');
export function glTexCoord4iv(v) {
  return call('glTexCoord4iv', v);
}

define('glTexCoord4s', dlsym(libGL, 'glTexCoord4s'), null, 'void', 'short', 'short', 'short', 'short');
export function glTexCoord4s(s, t, r, q) {
  return call('glTexCoord4s', s, t, r, q);
}

define('glTexCoord4sv', dlsym(libGL, 'glTexCoord4sv'), null, 'void', 'void *');
export function glTexCoord4sv(v) {
  return call('glTexCoord4sv', v);
}

define('glTexCoordPointer', dlsym(libGL, 'glTexCoordPointer'), null, 'void', 'int', 'unsigned int', 'int', 'void *');
export function glTexCoordPointer(size, type, stride, pointer) {
  return call('glTexCoordPointer', size, type, stride, pointer);
}

define('glTexEnvf', dlsym(libGL, 'glTexEnvf'), null, 'void', 'unsigned int', 'unsigned int', 'float');
export function glTexEnvf(target, pname, param) {
  return call('glTexEnvf', target, pname, param);
}

define('glTexEnvfv', dlsym(libGL, 'glTexEnvfv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glTexEnvfv(target, pname, params) {
  return call('glTexEnvfv', target, pname, params);
}

define('glTexEnvi', dlsym(libGL, 'glTexEnvi'), null, 'void', 'unsigned int', 'unsigned int', 'int');
export function glTexEnvi(target, pname, param) {
  return call('glTexEnvi', target, pname, param);
}

define('glTexEnviv', dlsym(libGL, 'glTexEnviv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glTexEnviv(target, pname, params) {
  return call('glTexEnviv', target, pname, params);
}

define('glTexGend', dlsym(libGL, 'glTexGend'), null, 'void', 'unsigned int', 'unsigned int', 'double');
export function glTexGend(coord, pname, param) {
  return call('glTexGend', coord, pname, param);
}

define('glTexGendv', dlsym(libGL, 'glTexGendv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glTexGendv(coord, pname, params) {
  return call('glTexGendv', coord, pname, params);
}

define('glTexGenf', dlsym(libGL, 'glTexGenf'), null, 'void', 'unsigned int', 'unsigned int', 'float');
export function glTexGenf(coord, pname, param) {
  return call('glTexGenf', coord, pname, param);
}

define('glTexGenfv', dlsym(libGL, 'glTexGenfv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glTexGenfv(coord, pname, params) {
  return call('glTexGenfv', coord, pname, params);
}

define('glTexGeni', dlsym(libGL, 'glTexGeni'), null, 'void', 'unsigned int', 'unsigned int', 'int');
export function glTexGeni(coord, pname, param) {
  return call('glTexGeni', coord, pname, param);
}

define('glTexGeniv', dlsym(libGL, 'glTexGeniv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glTexGeniv(coord, pname, params) {
  return call('glTexGeniv', coord, pname, params);
}

define(
  'glTexImage1D',
  dlsym(libGL, 'glTexImage1D'),
  null,
  'void',
  'unsigned int',
  'int',
  'int',
  'int',
  'int',
  'unsigned int',
  'unsigned int',
  'void *'
);
export function glTexImage1D(target, level, internalformat, width, border, format, type, pixels) {
  return call('glTexImage1D', target, level, internalformat, width, border, format, type, pixels);
}

define(
  'glTexImage2D',
  dlsym(libGL, 'glTexImage2D'),
  null,
  'void',
  'unsigned int',
  'int',
  'int',
  'int',
  'int',
  'int',
  'unsigned int',
  'unsigned int',
  'void *'
);
export function glTexImage2D(target, level, internalformat, width, height, border, format, type, pixels) {
  return call('glTexImage2D', target, level, internalformat, width, height, border, format, type, pixels);
}

define('glTexParameterf', dlsym(libGL, 'glTexParameterf'), null, 'void', 'unsigned int', 'unsigned int', 'float');
export function glTexParameterf(target, pname, param) {
  return call('glTexParameterf', target, pname, param);
}

define('glTexParameterfv', dlsym(libGL, 'glTexParameterfv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glTexParameterfv(target, pname, params) {
  return call('glTexParameterfv', target, pname, params);
}

define('glTexParameteri', dlsym(libGL, 'glTexParameteri'), null, 'void', 'unsigned int', 'unsigned int', 'int');
export function glTexParameteri(target, pname, param) {
  return call('glTexParameteri', target, pname, param);
}

define('glTexParameteriv', dlsym(libGL, 'glTexParameteriv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glTexParameteriv(target, pname, params) {
  return call('glTexParameteriv', target, pname, params);
}

define(
  'glTexSubImage1D',
  dlsym(libGL, 'glTexSubImage1D'),
  null,
  'void',
  'unsigned int',
  'int',
  'int',
  'int',
  'unsigned int',
  'unsigned int',
  'void *'
);
export function glTexSubImage1D(target, level, xoffset, width, format, type, pixels) {
  return call('glTexSubImage1D', target, level, xoffset, width, format, type, pixels);
}

define(
  'glTexSubImage2D',
  dlsym(libGL, 'glTexSubImage2D'),
  null,
  'void',
  'unsigned int',
  'int',
  'int',
  'int',
  'int',
  'int',
  'unsigned int',
  'unsigned int',
  'void *'
);
export function glTexSubImage2D(target, level, xoffset, yoffset, width, height, format, type, pixels) {
  return call('glTexSubImage2D', target, level, xoffset, yoffset, width, height, format, type, pixels);
}

define('glTranslated', dlsym(libGL, 'glTranslated'), null, 'void', 'double', 'double', 'double');
export function glTranslated(x, y, z) {
  return call('glTranslated', x, y, z);
}

define('glTranslatef', dlsym(libGL, 'glTranslatef'), null, 'void', 'float', 'float', 'float');
export function glTranslatef(x, y, z) {
  return call('glTranslatef', x, y, z);
}

define('glVertex2d', dlsym(libGL, 'glVertex2d'), null, 'void', 'double', 'double');
export function glVertex2d(x, y) {
  return call('glVertex2d', x, y);
}

define('glVertex2dv', dlsym(libGL, 'glVertex2dv'), null, 'void', 'void *');
export function glVertex2dv(v) {
  return call('glVertex2dv', v);
}

define('glVertex2f', dlsym(libGL, 'glVertex2f'), null, 'void', 'float', 'float');
export function glVertex2f(x, y) {
  return call('glVertex2f', x, y);
}

define('glVertex2fv', dlsym(libGL, 'glVertex2fv'), null, 'void', 'void *');
export function glVertex2fv(v) {
  return call('glVertex2fv', v);
}

define('glVertex2i', dlsym(libGL, 'glVertex2i'), null, 'void', 'int', 'int');
export function glVertex2i(x, y) {
  return call('glVertex2i', x, y);
}

define('glVertex2iv', dlsym(libGL, 'glVertex2iv'), null, 'void', 'void *');
export function glVertex2iv(v) {
  return call('glVertex2iv', v);
}

define('glVertex2s', dlsym(libGL, 'glVertex2s'), null, 'void', 'short', 'short');
export function glVertex2s(x, y) {
  return call('glVertex2s', x, y);
}

define('glVertex2sv', dlsym(libGL, 'glVertex2sv'), null, 'void', 'void *');
export function glVertex2sv(v) {
  return call('glVertex2sv', v);
}

define('glVertex3d', dlsym(libGL, 'glVertex3d'), null, 'void', 'double', 'double', 'double');
export function glVertex3d(x, y, z) {
  return call('glVertex3d', x, y, z);
}

define('glVertex3dv', dlsym(libGL, 'glVertex3dv'), null, 'void', 'void *');
export function glVertex3dv(v) {
  return call('glVertex3dv', v);
}

define('glVertex3f', dlsym(libGL, 'glVertex3f'), null, 'void', 'float', 'float', 'float');
export function glVertex3f(x, y, z) {
  return call('glVertex3f', x, y, z);
}

define('glVertex3fv', dlsym(libGL, 'glVertex3fv'), null, 'void', 'void *');
export function glVertex3fv(v) {
  return call('glVertex3fv', v);
}

define('glVertex3i', dlsym(libGL, 'glVertex3i'), null, 'void', 'int', 'int', 'int');
export function glVertex3i(x, y, z) {
  return call('glVertex3i', x, y, z);
}

define('glVertex3iv', dlsym(libGL, 'glVertex3iv'), null, 'void', 'void *');
export function glVertex3iv(v) {
  return call('glVertex3iv', v);
}

define('glVertex3s', dlsym(libGL, 'glVertex3s'), null, 'void', 'short', 'short', 'short');
export function glVertex3s(x, y, z) {
  return call('glVertex3s', x, y, z);
}

define('glVertex3sv', dlsym(libGL, 'glVertex3sv'), null, 'void', 'void *');
export function glVertex3sv(v) {
  return call('glVertex3sv', v);
}

define('glVertex4d', dlsym(libGL, 'glVertex4d'), null, 'void', 'double', 'double', 'double', 'double');
export function glVertex4d(x, y, z, w) {
  return call('glVertex4d', x, y, z, w);
}

define('glVertex4dv', dlsym(libGL, 'glVertex4dv'), null, 'void', 'void *');
export function glVertex4dv(v) {
  return call('glVertex4dv', v);
}

define('glVertex4f', dlsym(libGL, 'glVertex4f'), null, 'void', 'float', 'float', 'float', 'float');
export function glVertex4f(x, y, z, w) {
  return call('glVertex4f', x, y, z, w);
}

define('glVertex4fv', dlsym(libGL, 'glVertex4fv'), null, 'void', 'void *');
export function glVertex4fv(v) {
  return call('glVertex4fv', v);
}

define('glVertex4i', dlsym(libGL, 'glVertex4i'), null, 'void', 'int', 'int', 'int', 'int');
export function glVertex4i(x, y, z, w) {
  return call('glVertex4i', x, y, z, w);
}

define('glVertex4iv', dlsym(libGL, 'glVertex4iv'), null, 'void', 'void *');
export function glVertex4iv(v) {
  return call('glVertex4iv', v);
}

define('glVertex4s', dlsym(libGL, 'glVertex4s'), null, 'void', 'short', 'short', 'short', 'short');
export function glVertex4s(x, y, z, w) {
  return call('glVertex4s', x, y, z, w);
}

define('glVertex4sv', dlsym(libGL, 'glVertex4sv'), null, 'void', 'void *');
export function glVertex4sv(v) {
  return call('glVertex4sv', v);
}

define('glVertexPointer', dlsym(libGL, 'glVertexPointer'), null, 'void', 'int', 'unsigned int', 'int', 'void *');
export function glVertexPointer(size, type, stride, pointer) {
  return call('glVertexPointer', size, type, stride, pointer);
}

define('glViewport', dlsym(libGL, 'glViewport'), null, 'void', 'int', 'int', 'int', 'int');
export function glViewport(x, y, width, height) {
  return call('glViewport', x, y, width, height);
}
