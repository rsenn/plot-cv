define('glAccum', dlsym(RTLD_DEFAULT, 'glAccum'), null, 'void', 'int', 'int');
function glAccum(op, value) {
  return call('glAccum', op, value);
}
define('glActiveTexture', dlsym(RTLD_DEFAULT, 'glActiveTexture'), null, 'void', 'int');
function glActiveTexture(texture) {
  return call('glActiveTexture', texture);
}
define('glActiveTextureARB', dlsym(RTLD_DEFAULT, 'glActiveTextureARB'), null, 'void', 'int');
function glActiveTextureARB(texture) {
  return call('glActiveTextureARB', texture);
}
define('glAlphaFunc', dlsym(RTLD_DEFAULT, 'glAlphaFunc'), null, 'void', 'int', 'int');
function glAlphaFunc(func, ref) {
  return call('glAlphaFunc', func, ref);
}
define('glAreTexturesResident', dlsym(RTLD_DEFAULT, 'glAreTexturesResident'), null, 'undefined', 'int', 'void *', 'void *');
function glAreTexturesResident(n, textures, residences) {
  return call('glAreTexturesResident', n, textures, residences);
}
define('glArrayElement', dlsym(RTLD_DEFAULT, 'glArrayElement'), null, 'void', 'int');
function glArrayElement(i) {
  return call('glArrayElement', i);
}
define('glBegin', dlsym(RTLD_DEFAULT, 'glBegin'), null, 'void', 'int');
function glBegin(mode) {
  return call('glBegin', mode);
}
define('glBindTexture', dlsym(RTLD_DEFAULT, 'glBindTexture'), null, 'void', 'int', 'uint32');
function glBindTexture(target, texture) {
  return call('glBindTexture', target, texture);
}
define('glBitmap', dlsym(RTLD_DEFAULT, 'glBitmap'), null, 'void', 'int', 'int', 'int', 'int', 'int', 'int', 'void *');
function glBitmap(width, height, xorig, yorig, xmove, ymove, bitmap) {
  return call('glBitmap', width, height, xorig, yorig, xmove, ymove, bitmap);
}
define('glBlendColor', dlsym(RTLD_DEFAULT, 'glBlendColor'), null, 'void', 'int', 'int', 'int', 'int');
function glBlendColor(red, green, blue, alpha) {
  return call('glBlendColor', red, green, blue, alpha);
}
define('glBlendEquation', dlsym(RTLD_DEFAULT, 'glBlendEquation'), null, 'void', 'int');
function glBlendEquation(mode) {
  return call('glBlendEquation', mode);
}
define('glBlendEquationSeparateATI', dlsym(RTLD_DEFAULT, 'glBlendEquationSeparateATI'), null, 'void', 'int', 'int');
function glBlendEquationSeparateATI(modeRGB, modeA) {
  return call('glBlendEquationSeparateATI', modeRGB, modeA);
}
define('glBlendFunc', dlsym(RTLD_DEFAULT, 'glBlendFunc'), null, 'void', 'int', 'int');
function glBlendFunc(sfactor, dfactor) {
  return call('glBlendFunc', sfactor, dfactor);
}
define('glCallList', dlsym(RTLD_DEFAULT, 'glCallList'), null, 'void', 'uint32');
function glCallList(list) {
  return call('glCallList', list);
}
define('glCallLists', dlsym(RTLD_DEFAULT, 'glCallLists'), null, 'void', 'int', 'int', 'void *');
function glCallLists(n, type, lists) {
  return call('glCallLists', n, type, lists);
}
define('glClear', dlsym(RTLD_DEFAULT, 'glClear'), null, 'void', 'int');
function glClear(mask) {
  return call('glClear', mask);
}
define('glClearAccum', dlsym(RTLD_DEFAULT, 'glClearAccum'), null, 'void', 'int', 'int', 'int', 'int');
function glClearAccum(red, green, blue, alpha) {
  return call('glClearAccum', red, green, blue, alpha);
}
define('glClearColor', dlsym(RTLD_DEFAULT, 'glClearColor'), null, 'void', 'int', 'int', 'int', 'int');
function glClearColor(red, green, blue, alpha) {
  return call('glClearColor', red, green, blue, alpha);
}
define('glClearDepth', dlsym(RTLD_DEFAULT, 'glClearDepth'), null, 'void', 'long');
function glClearDepth(depth) {
  return call('glClearDepth', depth);
}
define('glClearIndex', dlsym(RTLD_DEFAULT, 'glClearIndex'), null, 'void', 'int');
function glClearIndex(c) {
  return call('glClearIndex', c);
}
define('glClearStencil', dlsym(RTLD_DEFAULT, 'glClearStencil'), null, 'void', 'int');
function glClearStencil(s) {
  return call('glClearStencil', s);
}
define('glClientActiveTexture', dlsym(RTLD_DEFAULT, 'glClientActiveTexture'), null, 'void', 'int');
function glClientActiveTexture(texture) {
  return call('glClientActiveTexture', texture);
}
define('glClientActiveTextureARB', dlsym(RTLD_DEFAULT, 'glClientActiveTextureARB'), null, 'void', 'int');
function glClientActiveTextureARB(texture) {
  return call('glClientActiveTextureARB', texture);
}
define('glClipPlane', dlsym(RTLD_DEFAULT, 'glClipPlane'), null, 'void', 'int', 'void *');
function glClipPlane(plane, equation) {
  return call('glClipPlane', plane, equation);
}
define('glColor3b', dlsym(RTLD_DEFAULT, 'glColor3b'), null, 'void', 'int8', 'int8', 'int8');
function glColor3b(red, green, blue) {
  return call('glColor3b', red, green, blue);
}
define('glColor3bv', dlsym(RTLD_DEFAULT, 'glColor3bv'), null, 'void', 'void *');
function glColor3bv(v) {
  return call('glColor3bv', v);
}
define('glColor3d', dlsym(RTLD_DEFAULT, 'glColor3d'), null, 'void', 'long', 'long', 'long');
function glColor3d(red, green, blue) {
  return call('glColor3d', red, green, blue);
}
define('glColor3dv', dlsym(RTLD_DEFAULT, 'glColor3dv'), null, 'void', 'void *');
function glColor3dv(v) {
  return call('glColor3dv', v);
}
define('glColor3f', dlsym(RTLD_DEFAULT, 'glColor3f'), null, 'void', 'int', 'int', 'int');
function glColor3f(red, green, blue) {
  return call('glColor3f', red, green, blue);
}
define('glColor3fv', dlsym(RTLD_DEFAULT, 'glColor3fv'), null, 'void', 'void *');
function glColor3fv(v) {
  return call('glColor3fv', v);
}
define('glColor3i', dlsym(RTLD_DEFAULT, 'glColor3i'), null, 'void', 'int', 'int', 'int');
function glColor3i(red, green, blue) {
  return call('glColor3i', red, green, blue);
}
define('glColor3iv', dlsym(RTLD_DEFAULT, 'glColor3iv'), null, 'void', 'void *');
function glColor3iv(v) {
  return call('glColor3iv', v);
}
define('glColor3s', dlsym(RTLD_DEFAULT, 'glColor3s'), null, 'void', 'int16', 'int16', 'int16');
function glColor3s(red, green, blue) {
  return call('glColor3s', red, green, blue);
}
define('glColor3sv', dlsym(RTLD_DEFAULT, 'glColor3sv'), null, 'void', 'void *');
function glColor3sv(v) {
  return call('glColor3sv', v);
}
define('glColor3ub', dlsym(RTLD_DEFAULT, 'glColor3ub'), null, 'void', 'int8', 'int8', 'int8');
function glColor3ub(red, green, blue) {
  return call('glColor3ub', red, green, blue);
}
define('glColor3ubv', dlsym(RTLD_DEFAULT, 'glColor3ubv'), null, 'void', 'void *');
function glColor3ubv(v) {
  return call('glColor3ubv', v);
}
define('glColor3ui', dlsym(RTLD_DEFAULT, 'glColor3ui'), null, 'void', 'uint32', 'uint32', 'uint32');
function glColor3ui(red, green, blue) {
  return call('glColor3ui', red, green, blue);
}
define('glColor3uiv', dlsym(RTLD_DEFAULT, 'glColor3uiv'), null, 'void', 'void *');
function glColor3uiv(v) {
  return call('glColor3uiv', v);
}
define('glColor3us', dlsym(RTLD_DEFAULT, 'glColor3us'), null, 'void', 'uint16', 'uint16', 'uint16');
function glColor3us(red, green, blue) {
  return call('glColor3us', red, green, blue);
}
define('glColor3usv', dlsym(RTLD_DEFAULT, 'glColor3usv'), null, 'void', 'void *');
function glColor3usv(v) {
  return call('glColor3usv', v);
}
define('glColor4b', dlsym(RTLD_DEFAULT, 'glColor4b'), null, 'void', 'int8', 'int8', 'int8', 'int8');
function glColor4b(red, green, blue, alpha) {
  return call('glColor4b', red, green, blue, alpha);
}
define('glColor4bv', dlsym(RTLD_DEFAULT, 'glColor4bv'), null, 'void', 'void *');
function glColor4bv(v) {
  return call('glColor4bv', v);
}
define('glColor4d', dlsym(RTLD_DEFAULT, 'glColor4d'), null, 'void', 'long', 'long', 'long', 'long');
function glColor4d(red, green, blue, alpha) {
  return call('glColor4d', red, green, blue, alpha);
}
define('glColor4dv', dlsym(RTLD_DEFAULT, 'glColor4dv'), null, 'void', 'void *');
function glColor4dv(v) {
  return call('glColor4dv', v);
}
define('glColor4f', dlsym(RTLD_DEFAULT, 'glColor4f'), null, 'void', 'int', 'int', 'int', 'int');
function glColor4f(red, green, blue, alpha) {
  return call('glColor4f', red, green, blue, alpha);
}
define('glColor4fv', dlsym(RTLD_DEFAULT, 'glColor4fv'), null, 'void', 'void *');
function glColor4fv(v) {
  return call('glColor4fv', v);
}
define('glColor4i', dlsym(RTLD_DEFAULT, 'glColor4i'), null, 'void', 'int', 'int', 'int', 'int');
function glColor4i(red, green, blue, alpha) {
  return call('glColor4i', red, green, blue, alpha);
}
define('glColor4iv', dlsym(RTLD_DEFAULT, 'glColor4iv'), null, 'void', 'void *');
function glColor4iv(v) {
  return call('glColor4iv', v);
}
define('glColor4s', dlsym(RTLD_DEFAULT, 'glColor4s'), null, 'void', 'int16', 'int16', 'int16', 'int16');
function glColor4s(red, green, blue, alpha) {
  return call('glColor4s', red, green, blue, alpha);
}
define('glColor4sv', dlsym(RTLD_DEFAULT, 'glColor4sv'), null, 'void', 'void *');
function glColor4sv(v) {
  return call('glColor4sv', v);
}
define('glColor4ub', dlsym(RTLD_DEFAULT, 'glColor4ub'), null, 'void', 'int8', 'int8', 'int8', 'int8');
function glColor4ub(red, green, blue, alpha) {
  return call('glColor4ub', red, green, blue, alpha);
}
define('glColor4ubv', dlsym(RTLD_DEFAULT, 'glColor4ubv'), null, 'void', 'void *');
function glColor4ubv(v) {
  return call('glColor4ubv', v);
}
define('glColor4ui', dlsym(RTLD_DEFAULT, 'glColor4ui'), null, 'void', 'uint32', 'uint32', 'uint32', 'uint32');
function glColor4ui(red, green, blue, alpha) {
  return call('glColor4ui', red, green, blue, alpha);
}
define('glColor4uiv', dlsym(RTLD_DEFAULT, 'glColor4uiv'), null, 'void', 'void *');
function glColor4uiv(v) {
  return call('glColor4uiv', v);
}
define('glColor4us', dlsym(RTLD_DEFAULT, 'glColor4us'), null, 'void', 'uint16', 'uint16', 'uint16', 'uint16');
function glColor4us(red, green, blue, alpha) {
  return call('glColor4us', red, green, blue, alpha);
}
define('glColor4usv', dlsym(RTLD_DEFAULT, 'glColor4usv'), null, 'void', 'void *');
function glColor4usv(v) {
  return call('glColor4usv', v);
}
define('glColorMask', dlsym(RTLD_DEFAULT, 'glColorMask'), null, 'void', 'int8', 'int8', 'int8', 'int8');
function glColorMask(red, green, blue, alpha) {
  return call('glColorMask', red, green, blue, alpha);
}
define('glColorMaterial', dlsym(RTLD_DEFAULT, 'glColorMaterial'), null, 'void', 'int', 'int');
function glColorMaterial(face, mode) {
  return call('glColorMaterial', face, mode);
}
define('glColorPointer', dlsym(RTLD_DEFAULT, 'glColorPointer'), null, 'void', 'int', 'int', 'int', 'void *');
function glColorPointer(size, type, stride, ptr) {
  return call('glColorPointer', size, type, stride, ptr);
}
define('glColorSubTable', dlsym(RTLD_DEFAULT, 'glColorSubTable'), null, 'void', 'int', 'int', 'int', 'int', 'int', 'void *');
function glColorSubTable(target, start, count, format, type, data) {
  return call('glColorSubTable', target, start, count, format, type, data);
}
define('glColorTable', dlsym(RTLD_DEFAULT, 'glColorTable'), null, 'void', 'int', 'int', 'int', 'int', 'int', 'void *');
function glColorTable(target, internalformat, width, format, type, table) {
  return call('glColorTable', target, internalformat, width, format, type, table);
}
define('glColorTableParameterfv', dlsym(RTLD_DEFAULT, 'glColorTableParameterfv'), null, 'void', 'int', 'int', 'void *');
function glColorTableParameterfv(target, pname, params) {
  return call('glColorTableParameterfv', target, pname, params);
}
define('glColorTableParameteriv', dlsym(RTLD_DEFAULT, 'glColorTableParameteriv'), null, 'void', 'int', 'int', 'void *');
function glColorTableParameteriv(target, pname, params) {
  return call('glColorTableParameteriv', target, pname, params);
}
define('glCompressedTexImage1D', dlsym(RTLD_DEFAULT, 'glCompressedTexImage1D'), null, 'void', 'int', 'int', 'int', 'int', 'int', 'int', 'void *');
function glCompressedTexImage1D(target, level, internalformat, width, border, imageSize, data) {
  return call('glCompressedTexImage1D', target, level, internalformat, width, border, imageSize, data);
}
define('glCompressedTexImage2D', dlsym(RTLD_DEFAULT, 'glCompressedTexImage2D'), null, 'void', 'int', 'int', 'int', 'int', 'int', 'int', 'int', 'void *');
function glCompressedTexImage2D(target, level, internalformat, width, height, border, imageSize, data) {
  return call('glCompressedTexImage2D', target, level, internalformat, width, height, border, imageSize, data);
}
define('glCompressedTexImage3D', dlsym(RTLD_DEFAULT, 'glCompressedTexImage3D'), null, 'void', 'int', 'int', 'int', 'int', 'int', 'int', 'int', 'int', 'void *');
function glCompressedTexImage3D(target, level, internalformat, width, height, depth, border, imageSize, data) {
  return call('glCompressedTexImage3D', target, level, internalformat, width, height, depth, border, imageSize, data);
}
define('glCompressedTexSubImage1D', dlsym(RTLD_DEFAULT, 'glCompressedTexSubImage1D'), null, 'void', 'int', 'int', 'int', 'int', 'int', 'int', 'void *');
function glCompressedTexSubImage1D(target, level, xoffset, width, format, imageSize, data) {
  return call('glCompressedTexSubImage1D', target, level, xoffset, width, format, imageSize, data);
}
define('glCompressedTexSubImage2D', dlsym(RTLD_DEFAULT, 'glCompressedTexSubImage2D'), null, 'void', 'int', 'int', 'int', 'int', 'int', 'int', 'int', 'int', 'void *');
function glCompressedTexSubImage2D(target, level, xoffset, yoffset, width, height, format, imageSize, data) {
  return call('glCompressedTexSubImage2D', target, level, xoffset, yoffset, width, height, format, imageSize, data);
}
define('glCompressedTexSubImage3D', dlsym(RTLD_DEFAULT, 'glCompressedTexSubImage3D'), null, 'void', 'int', 'int', 'int', 'int', 'int', 'int', 'int', 'int', 'int', 'int', 'void *');
function glCompressedTexSubImage3D(target, level, xoffset, yoffset, zoffset, width, height, depth, format, imageSize, data) {
  return call('glCompressedTexSubImage3D', target, level, xoffset, yoffset, zoffset, width, height, depth, format, imageSize, data);
}
define('glConvolutionFilter1D', dlsym(RTLD_DEFAULT, 'glConvolutionFilter1D'), null, 'void', 'int', 'int', 'int', 'int', 'int', 'void *');
function glConvolutionFilter1D(target, internalformat, width, format, type, image) {
  return call('glConvolutionFilter1D', target, internalformat, width, format, type, image);
}
define('glConvolutionFilter2D', dlsym(RTLD_DEFAULT, 'glConvolutionFilter2D'), null, 'void', 'int', 'int', 'int', 'int', 'int', 'int', 'void *');
function glConvolutionFilter2D(target, internalformat, width, height, format, type, image) {
  return call('glConvolutionFilter2D', target, internalformat, width, height, format, type, image);
}
define('glConvolutionParameterf', dlsym(RTLD_DEFAULT, 'glConvolutionParameterf'), null, 'void', 'int', 'int', 'int');
function glConvolutionParameterf(target, pname, params) {
  return call('glConvolutionParameterf', target, pname, params);
}
define('glConvolutionParameterfv', dlsym(RTLD_DEFAULT, 'glConvolutionParameterfv'), null, 'void', 'int', 'int', 'void *');
function glConvolutionParameterfv(target, pname, params) {
  return call('glConvolutionParameterfv', target, pname, params);
}
define('glConvolutionParameteri', dlsym(RTLD_DEFAULT, 'glConvolutionParameteri'), null, 'void', 'int', 'int', 'int');
function glConvolutionParameteri(target, pname, params) {
  return call('glConvolutionParameteri', target, pname, params);
}
define('glConvolutionParameteriv', dlsym(RTLD_DEFAULT, 'glConvolutionParameteriv'), null, 'void', 'int', 'int', 'void *');
function glConvolutionParameteriv(target, pname, params) {
  return call('glConvolutionParameteriv', target, pname, params);
}
define('glCopyColorSubTable', dlsym(RTLD_DEFAULT, 'glCopyColorSubTable'), null, 'void', 'int', 'int', 'int', 'int', 'int');
function glCopyColorSubTable(target, start, x, y, width) {
  return call('glCopyColorSubTable', target, start, x, y, width);
}
define('glCopyColorTable', dlsym(RTLD_DEFAULT, 'glCopyColorTable'), null, 'void', 'int', 'int', 'int', 'int', 'int');
function glCopyColorTable(target, internalformat, x, y, width) {
  return call('glCopyColorTable', target, internalformat, x, y, width);
}
define('glCopyConvolutionFilter1D', dlsym(RTLD_DEFAULT, 'glCopyConvolutionFilter1D'), null, 'void', 'int', 'int', 'int', 'int', 'int');
function glCopyConvolutionFilter1D(target, internalformat, x, y, width) {
  return call('glCopyConvolutionFilter1D', target, internalformat, x, y, width);
}
define('glCopyConvolutionFilter2D', dlsym(RTLD_DEFAULT, 'glCopyConvolutionFilter2D'), null, 'void', 'int', 'int', 'int', 'int', 'int', 'int');
function glCopyConvolutionFilter2D(target, internalformat, x, y, width, height) {
  return call('glCopyConvolutionFilter2D', target, internalformat, x, y, width, height);
}
define('glCopyPixels', dlsym(RTLD_DEFAULT, 'glCopyPixels'), null, 'void', 'int', 'int', 'int', 'int', 'int');
function glCopyPixels(x, y, width, height, type) {
  return call('glCopyPixels', x, y, width, height, type);
}
define('glCopyTexImage1D', dlsym(RTLD_DEFAULT, 'glCopyTexImage1D'), null, 'void', 'int', 'int', 'int', 'int', 'int', 'int', 'int');
function glCopyTexImage1D(target, level, internalformat, x, y, width, border) {
  return call('glCopyTexImage1D', target, level, internalformat, x, y, width, border);
}
define('glCopyTexImage2D', dlsym(RTLD_DEFAULT, 'glCopyTexImage2D'), null, 'void', 'int', 'int', 'int', 'int', 'int', 'int', 'int', 'int');
function glCopyTexImage2D(target, level, internalformat, x, y, width, height, border) {
  return call('glCopyTexImage2D', target, level, internalformat, x, y, width, height, border);
}
define('glCopyTexSubImage1D', dlsym(RTLD_DEFAULT, 'glCopyTexSubImage1D'), null, 'void', 'int', 'int', 'int', 'int', 'int', 'int');
function glCopyTexSubImage1D(target, level, xoffset, x, y, width) {
  return call('glCopyTexSubImage1D', target, level, xoffset, x, y, width);
}
define('glCopyTexSubImage2D', dlsym(RTLD_DEFAULT, 'glCopyTexSubImage2D'), null, 'void', 'int', 'int', 'int', 'int', 'int', 'int', 'int', 'int');
function glCopyTexSubImage2D(target, level, xoffset, yoffset, x, y, width, height) {
  return call('glCopyTexSubImage2D', target, level, xoffset, yoffset, x, y, width, height);
}
define('glCopyTexSubImage3D', dlsym(RTLD_DEFAULT, 'glCopyTexSubImage3D'), null, 'void', 'int', 'int', 'int', 'int', 'int', 'int', 'int', 'int', 'int');
function glCopyTexSubImage3D(target, level, xoffset, yoffset, zoffset, x, y, width, height) {
  return call('glCopyTexSubImage3D', target, level, xoffset, yoffset, zoffset, x, y, width, height);
}
define('glCullFace', dlsym(RTLD_DEFAULT, 'glCullFace'), null, 'void', 'int');
function glCullFace(mode) {
  return call('glCullFace', mode);
}
define('glDeleteLists', dlsym(RTLD_DEFAULT, 'glDeleteLists'), null, 'void', 'uint32', 'int');
function glDeleteLists(list, range) {
  return call('glDeleteLists', list, range);
}
define('glDeleteTextures', dlsym(RTLD_DEFAULT, 'glDeleteTextures'), null, 'void', 'int', 'void *');
function glDeleteTextures(n, textures) {
  return call('glDeleteTextures', n, textures);
}
define('glDepthFunc', dlsym(RTLD_DEFAULT, 'glDepthFunc'), null, 'void', 'int');
function glDepthFunc(func) {
  return call('glDepthFunc', func);
}
define('glDepthMask', dlsym(RTLD_DEFAULT, 'glDepthMask'), null, 'void', 'int8');
function glDepthMask(flag) {
  return call('glDepthMask', flag);
}
define('glDepthRange', dlsym(RTLD_DEFAULT, 'glDepthRange'), null, 'void', 'long', 'long');
function glDepthRange(near_val, far_val) {
  return call('glDepthRange', near_val, far_val);
}
define('glDisable', dlsym(RTLD_DEFAULT, 'glDisable'), null, 'void', 'int');
function glDisable(cap) {
  return call('glDisable', cap);
}
define('glDisableClientState', dlsym(RTLD_DEFAULT, 'glDisableClientState'), null, 'void', 'int');
function glDisableClientState(cap) {
  return call('glDisableClientState', cap);
}
define('glDrawArrays', dlsym(RTLD_DEFAULT, 'glDrawArrays'), null, 'void', 'int', 'int', 'int');
function glDrawArrays(mode, first, count) {
  return call('glDrawArrays', mode, first, count);
}
define('glDrawBuffer', dlsym(RTLD_DEFAULT, 'glDrawBuffer'), null, 'void', 'int');
function glDrawBuffer(mode) {
  return call('glDrawBuffer', mode);
}
define('glDrawElements', dlsym(RTLD_DEFAULT, 'glDrawElements'), null, 'void', 'int', 'int', 'int', 'void *');
function glDrawElements(mode, count, type, indices) {
  return call('glDrawElements', mode, count, type, indices);
}
define('glDrawPixels', dlsym(RTLD_DEFAULT, 'glDrawPixels'), null, 'void', 'int', 'int', 'int', 'int', 'void *');
function glDrawPixels(width, height, format, type, pixels) {
  return call('glDrawPixels', width, height, format, type, pixels);
}
define('glDrawRangeElements', dlsym(RTLD_DEFAULT, 'glDrawRangeElements'), null, 'void', 'int', 'uint32', 'uint32', 'int', 'int', 'void *');
function glDrawRangeElements(mode, start, end, count, type, indices) {
  return call('glDrawRangeElements', mode, start, end, count, type, indices);
}
define('glEdgeFlag', dlsym(RTLD_DEFAULT, 'glEdgeFlag'), null, 'void', 'int8');
function glEdgeFlag(flag) {
  return call('glEdgeFlag', flag);
}
define('glEdgeFlagPointer', dlsym(RTLD_DEFAULT, 'glEdgeFlagPointer'), null, 'void', 'int', 'void *');
function glEdgeFlagPointer(stride, ptr) {
  return call('glEdgeFlagPointer', stride, ptr);
}
define('glEdgeFlagv', dlsym(RTLD_DEFAULT, 'glEdgeFlagv'), null, 'void', 'void *');
function glEdgeFlagv(flag) {
  return call('glEdgeFlagv', flag);
}
define('glEnable', dlsym(RTLD_DEFAULT, 'glEnable'), null, 'void', 'int');
function glEnable(cap) {
  return call('glEnable', cap);
}
define('glEnableClientState', dlsym(RTLD_DEFAULT, 'glEnableClientState'), null, 'void', 'int');
function glEnableClientState(cap) {
  return call('glEnableClientState', cap);
}
define('glEnd', dlsym(RTLD_DEFAULT, 'glEnd'), null, 'void');
function glEnd() {
  return call('glEnd', );
}
define('glEndList', dlsym(RTLD_DEFAULT, 'glEndList'), null, 'void');
function glEndList() {
  return call('glEndList', );
}
define('glEvalCoord1d', dlsym(RTLD_DEFAULT, 'glEvalCoord1d'), null, 'void', 'long');
function glEvalCoord1d(u) {
  return call('glEvalCoord1d', u);
}
define('glEvalCoord1dv', dlsym(RTLD_DEFAULT, 'glEvalCoord1dv'), null, 'void', 'void *');
function glEvalCoord1dv(u) {
  return call('glEvalCoord1dv', u);
}
define('glEvalCoord1f', dlsym(RTLD_DEFAULT, 'glEvalCoord1f'), null, 'void', 'int');
function glEvalCoord1f(u) {
  return call('glEvalCoord1f', u);
}
define('glEvalCoord1fv', dlsym(RTLD_DEFAULT, 'glEvalCoord1fv'), null, 'void', 'void *');
function glEvalCoord1fv(u) {
  return call('glEvalCoord1fv', u);
}
define('glEvalCoord2d', dlsym(RTLD_DEFAULT, 'glEvalCoord2d'), null, 'void', 'long', 'long');
function glEvalCoord2d(u, v) {
  return call('glEvalCoord2d', u, v);
}
define('glEvalCoord2dv', dlsym(RTLD_DEFAULT, 'glEvalCoord2dv'), null, 'void', 'void *');
function glEvalCoord2dv(u) {
  return call('glEvalCoord2dv', u);
}
define('glEvalCoord2f', dlsym(RTLD_DEFAULT, 'glEvalCoord2f'), null, 'void', 'int', 'int');
function glEvalCoord2f(u, v) {
  return call('glEvalCoord2f', u, v);
}
define('glEvalCoord2fv', dlsym(RTLD_DEFAULT, 'glEvalCoord2fv'), null, 'void', 'void *');
function glEvalCoord2fv(u) {
  return call('glEvalCoord2fv', u);
}
define('glEvalMesh1', dlsym(RTLD_DEFAULT, 'glEvalMesh1'), null, 'void', 'int', 'int', 'int');
function glEvalMesh1(mode, i1, i2) {
  return call('glEvalMesh1', mode, i1, i2);
}
define('glEvalMesh2', dlsym(RTLD_DEFAULT, 'glEvalMesh2'), null, 'void', 'int', 'int', 'int', 'int', 'int');
function glEvalMesh2(mode, i1, i2, j1, j2) {
  return call('glEvalMesh2', mode, i1, i2, j1, j2);
}
define('glEvalPoint1', dlsym(RTLD_DEFAULT, 'glEvalPoint1'), null, 'void', 'int');
function glEvalPoint1(i) {
  return call('glEvalPoint1', i);
}
define('glEvalPoint2', dlsym(RTLD_DEFAULT, 'glEvalPoint2'), null, 'void', 'int', 'int');
function glEvalPoint2(i, j) {
  return call('glEvalPoint2', i, j);
}
define('glFeedbackBuffer', dlsym(RTLD_DEFAULT, 'glFeedbackBuffer'), null, 'void', 'int', 'int', 'void *');
function glFeedbackBuffer(size, type, buffer) {
  return call('glFeedbackBuffer', size, type, buffer);
}
define('glFinish', dlsym(RTLD_DEFAULT, 'glFinish'), null, 'void');
function glFinish() {
  return call('glFinish', );
}
define('glFlush', dlsym(RTLD_DEFAULT, 'glFlush'), null, 'void');
function glFlush() {
  return call('glFlush', );
}
define('glFogf', dlsym(RTLD_DEFAULT, 'glFogf'), null, 'void', 'int', 'int');
function glFogf(pname, param) {
  return call('glFogf', pname, param);
}
define('glFogfv', dlsym(RTLD_DEFAULT, 'glFogfv'), null, 'void', 'int', 'void *');
function glFogfv(pname, params) {
  return call('glFogfv', pname, params);
}
define('glFogi', dlsym(RTLD_DEFAULT, 'glFogi'), null, 'void', 'int', 'int');
function glFogi(pname, param) {
  return call('glFogi', pname, param);
}
define('glFogiv', dlsym(RTLD_DEFAULT, 'glFogiv'), null, 'void', 'int', 'void *');
function glFogiv(pname, params) {
  return call('glFogiv', pname, params);
}
define('glFrontFace', dlsym(RTLD_DEFAULT, 'glFrontFace'), null, 'void', 'int');
function glFrontFace(mode) {
  return call('glFrontFace', mode);
}
define('glFrustum', dlsym(RTLD_DEFAULT, 'glFrustum'), null, 'void', 'long', 'long', 'long', 'long', 'long', 'long');
function glFrustum(left, right, bottom, top, near_val, far_val) {
  return call('glFrustum', left, right, bottom, top, near_val, far_val);
}
define('glGenLists', dlsym(RTLD_DEFAULT, 'glGenLists'), null, 'uint32', 'int');
function glGenLists(range) {
  return call('glGenLists', range);
}
define('glGenTextures', dlsym(RTLD_DEFAULT, 'glGenTextures'), null, 'void', 'int', 'void *');
function glGenTextures(n, textures) {
  return call('glGenTextures', n, textures);
}
define('glGetBooleanv', dlsym(RTLD_DEFAULT, 'glGetBooleanv'), null, 'void', 'int', 'void *');
function glGetBooleanv(pname, params) {
  return call('glGetBooleanv', pname, params);
}
define('glGetClipPlane', dlsym(RTLD_DEFAULT, 'glGetClipPlane'), null, 'void', 'int', 'void *');
function glGetClipPlane(plane, equation) {
  return call('glGetClipPlane', plane, equation);
}
define('glGetColorTable', dlsym(RTLD_DEFAULT, 'glGetColorTable'), null, 'void', 'int', 'int', 'int', 'void *');
function glGetColorTable(target, format, type, table) {
  return call('glGetColorTable', target, format, type, table);
}
define('glGetColorTableParameterfv', dlsym(RTLD_DEFAULT, 'glGetColorTableParameterfv'), null, 'void', 'int', 'int', 'void *');
function glGetColorTableParameterfv(target, pname, params) {
  return call('glGetColorTableParameterfv', target, pname, params);
}
define('glGetColorTableParameteriv', dlsym(RTLD_DEFAULT, 'glGetColorTableParameteriv'), null, 'void', 'int', 'int', 'void *');
function glGetColorTableParameteriv(target, pname, params) {
  return call('glGetColorTableParameteriv', target, pname, params);
}
define('glGetCompressedTexImage', dlsym(RTLD_DEFAULT, 'glGetCompressedTexImage'), null, 'void', 'int', 'int', 'void *');
function glGetCompressedTexImage(target, lod, img) {
  return call('glGetCompressedTexImage', target, lod, img);
}
define('glGetConvolutionFilter', dlsym(RTLD_DEFAULT, 'glGetConvolutionFilter'), null, 'void', 'int', 'int', 'int', 'void *');
function glGetConvolutionFilter(target, format, type, image) {
  return call('glGetConvolutionFilter', target, format, type, image);
}
define('glGetConvolutionParameterfv', dlsym(RTLD_DEFAULT, 'glGetConvolutionParameterfv'), null, 'void', 'int', 'int', 'void *');
function glGetConvolutionParameterfv(target, pname, params) {
  return call('glGetConvolutionParameterfv', target, pname, params);
}
define('glGetConvolutionParameteriv', dlsym(RTLD_DEFAULT, 'glGetConvolutionParameteriv'), null, 'void', 'int', 'int', 'void *');
function glGetConvolutionParameteriv(target, pname, params) {
  return call('glGetConvolutionParameteriv', target, pname, params);
}
define('glGetDoublev', dlsym(RTLD_DEFAULT, 'glGetDoublev'), null, 'void', 'int', 'void *');
function glGetDoublev(pname, params) {
  return call('glGetDoublev', pname, params);
}
define('glGetError', dlsym(RTLD_DEFAULT, 'glGetError'), null, 'int');
function glGetError() {
  return call('glGetError', );
}
define('glGetFloatv', dlsym(RTLD_DEFAULT, 'glGetFloatv'), null, 'void', 'int', 'void *');
function glGetFloatv(pname, params) {
  return call('glGetFloatv', pname, params);
}
define('glGetHistogram', dlsym(RTLD_DEFAULT, 'glGetHistogram'), null, 'void', 'int', 'int8', 'int', 'int', 'void *');
function glGetHistogram(target, reset, format, type, values) {
  return call('glGetHistogram', target, reset, format, type, values);
}
define('glGetHistogramParameterfv', dlsym(RTLD_DEFAULT, 'glGetHistogramParameterfv'), null, 'void', 'int', 'int', 'void *');
function glGetHistogramParameterfv(target, pname, params) {
  return call('glGetHistogramParameterfv', target, pname, params);
}
define('glGetHistogramParameteriv', dlsym(RTLD_DEFAULT, 'glGetHistogramParameteriv'), null, 'void', 'int', 'int', 'void *');
function glGetHistogramParameteriv(target, pname, params) {
  return call('glGetHistogramParameteriv', target, pname, params);
}
define('glGetIntegerv', dlsym(RTLD_DEFAULT, 'glGetIntegerv'), null, 'void', 'int', 'void *');
function glGetIntegerv(pname, params) {
  return call('glGetIntegerv', pname, params);
}
define('glGetLightfv', dlsym(RTLD_DEFAULT, 'glGetLightfv'), null, 'void', 'int', 'int', 'void *');
function glGetLightfv(light, pname, params) {
  return call('glGetLightfv', light, pname, params);
}
define('glGetLightiv', dlsym(RTLD_DEFAULT, 'glGetLightiv'), null, 'void', 'int', 'int', 'void *');
function glGetLightiv(light, pname, params) {
  return call('glGetLightiv', light, pname, params);
}
define('glGetMapdv', dlsym(RTLD_DEFAULT, 'glGetMapdv'), null, 'void', 'int', 'int', 'void *');
function glGetMapdv(target, query, v) {
  return call('glGetMapdv', target, query, v);
}
define('glGetMapfv', dlsym(RTLD_DEFAULT, 'glGetMapfv'), null, 'void', 'int', 'int', 'void *');
function glGetMapfv(target, query, v) {
  return call('glGetMapfv', target, query, v);
}
define('glGetMapiv', dlsym(RTLD_DEFAULT, 'glGetMapiv'), null, 'void', 'int', 'int', 'void *');
function glGetMapiv(target, query, v) {
  return call('glGetMapiv', target, query, v);
}
define('glGetMaterialfv', dlsym(RTLD_DEFAULT, 'glGetMaterialfv'), null, 'void', 'int', 'int', 'void *');
function glGetMaterialfv(face, pname, params) {
  return call('glGetMaterialfv', face, pname, params);
}
define('glGetMaterialiv', dlsym(RTLD_DEFAULT, 'glGetMaterialiv'), null, 'void', 'int', 'int', 'void *');
function glGetMaterialiv(face, pname, params) {
  return call('glGetMaterialiv', face, pname, params);
}
define('glGetMinmax', dlsym(RTLD_DEFAULT, 'glGetMinmax'), null, 'void', 'int', 'int8', 'int', 'int', 'void *');
function glGetMinmax(target, reset, format, types, values) {
  return call('glGetMinmax', target, reset, format, types, values);
}
define('glGetMinmaxParameterfv', dlsym(RTLD_DEFAULT, 'glGetMinmaxParameterfv'), null, 'void', 'int', 'int', 'void *');
function glGetMinmaxParameterfv(target, pname, params) {
  return call('glGetMinmaxParameterfv', target, pname, params);
}
define('glGetMinmaxParameteriv', dlsym(RTLD_DEFAULT, 'glGetMinmaxParameteriv'), null, 'void', 'int', 'int', 'void *');
function glGetMinmaxParameteriv(target, pname, params) {
  return call('glGetMinmaxParameteriv', target, pname, params);
}
define('glGetPixelMapfv', dlsym(RTLD_DEFAULT, 'glGetPixelMapfv'), null, 'void', 'int', 'void *');
function glGetPixelMapfv(map, values) {
  return call('glGetPixelMapfv', map, values);
}
define('glGetPixelMapuiv', dlsym(RTLD_DEFAULT, 'glGetPixelMapuiv'), null, 'void', 'int', 'void *');
function glGetPixelMapuiv(map, values) {
  return call('glGetPixelMapuiv', map, values);
}
define('glGetPixelMapusv', dlsym(RTLD_DEFAULT, 'glGetPixelMapusv'), null, 'void', 'int', 'void *');
function glGetPixelMapusv(map, values) {
  return call('glGetPixelMapusv', map, values);
}
define('glGetPointerv', dlsym(RTLD_DEFAULT, 'glGetPointerv'), null, 'void', 'int', 'void *');
function glGetPointerv(pname, params) {
  return call('glGetPointerv', pname, params);
}
define('glGetPolygonStipple', dlsym(RTLD_DEFAULT, 'glGetPolygonStipple'), null, 'void', 'void *');
function glGetPolygonStipple(mask) {
  return call('glGetPolygonStipple', mask);
}
define('glGetSeparableFilter', dlsym(RTLD_DEFAULT, 'glGetSeparableFilter'), null, 'void', 'int', 'int', 'int', 'void *', 'void *', 'void *');
function glGetSeparableFilter(target, format, type, row, column, span) {
  return call('glGetSeparableFilter', target, format, type, row, column, span);
}
define('glGetString', dlsym(RTLD_DEFAULT, 'glGetString'), null, 'undefined', 'int');
function glGetString(name) {
  return call('glGetString', name);
}
define('glGetTexEnvfv', dlsym(RTLD_DEFAULT, 'glGetTexEnvfv'), null, 'void', 'int', 'int', 'void *');
function glGetTexEnvfv(target, pname, params) {
  return call('glGetTexEnvfv', target, pname, params);
}
define('glGetTexEnviv', dlsym(RTLD_DEFAULT, 'glGetTexEnviv'), null, 'void', 'int', 'int', 'void *');
function glGetTexEnviv(target, pname, params) {
  return call('glGetTexEnviv', target, pname, params);
}
define('glGetTexGendv', dlsym(RTLD_DEFAULT, 'glGetTexGendv'), null, 'void', 'int', 'int', 'void *');
function glGetTexGendv(coord, pname, params) {
  return call('glGetTexGendv', coord, pname, params);
}
define('glGetTexGenfv', dlsym(RTLD_DEFAULT, 'glGetTexGenfv'), null, 'void', 'int', 'int', 'void *');
function glGetTexGenfv(coord, pname, params) {
  return call('glGetTexGenfv', coord, pname, params);
}
define('glGetTexGeniv', dlsym(RTLD_DEFAULT, 'glGetTexGeniv'), null, 'void', 'int', 'int', 'void *');
function glGetTexGeniv(coord, pname, params) {
  return call('glGetTexGeniv', coord, pname, params);
}
define('glGetTexImage', dlsym(RTLD_DEFAULT, 'glGetTexImage'), null, 'void', 'int', 'int', 'int', 'int', 'void *');
function glGetTexImage(target, level, format, type, pixels) {
  return call('glGetTexImage', target, level, format, type, pixels);
}
define('glGetTexLevelParameterfv', dlsym(RTLD_DEFAULT, 'glGetTexLevelParameterfv'), null, 'void', 'int', 'int', 'int', 'void *');
function glGetTexLevelParameterfv(target, level, pname, params) {
  return call('glGetTexLevelParameterfv', target, level, pname, params);
}
define('glGetTexLevelParameteriv', dlsym(RTLD_DEFAULT, 'glGetTexLevelParameteriv'), null, 'void', 'int', 'int', 'int', 'void *');
function glGetTexLevelParameteriv(target, level, pname, params) {
  return call('glGetTexLevelParameteriv', target, level, pname, params);
}
define('glGetTexParameterfv', dlsym(RTLD_DEFAULT, 'glGetTexParameterfv'), null, 'void', 'int', 'int', 'void *');
function glGetTexParameterfv(target, pname, params) {
  return call('glGetTexParameterfv', target, pname, params);
}
define('glGetTexParameteriv', dlsym(RTLD_DEFAULT, 'glGetTexParameteriv'), null, 'void', 'int', 'int', 'void *');
function glGetTexParameteriv(target, pname, params) {
  return call('glGetTexParameteriv', target, pname, params);
}
define('glHint', dlsym(RTLD_DEFAULT, 'glHint'), null, 'void', 'int', 'int');
function glHint(target, mode) {
  return call('glHint', target, mode);
}
define('glHistogram', dlsym(RTLD_DEFAULT, 'glHistogram'), null, 'void', 'int', 'int', 'int', 'int8');
function glHistogram(target, width, internalformat, sink) {
  return call('glHistogram', target, width, internalformat, sink);
}
define('glIndexMask', dlsym(RTLD_DEFAULT, 'glIndexMask'), null, 'void', 'uint32');
function glIndexMask(mask) {
  return call('glIndexMask', mask);
}
define('glIndexPointer', dlsym(RTLD_DEFAULT, 'glIndexPointer'), null, 'void', 'int', 'int', 'void *');
function glIndexPointer(type, stride, ptr) {
  return call('glIndexPointer', type, stride, ptr);
}
define('glIndexd', dlsym(RTLD_DEFAULT, 'glIndexd'), null, 'void', 'long');
function glIndexd(c) {
  return call('glIndexd', c);
}
define('glIndexdv', dlsym(RTLD_DEFAULT, 'glIndexdv'), null, 'void', 'void *');
function glIndexdv(c) {
  return call('glIndexdv', c);
}
define('glIndexf', dlsym(RTLD_DEFAULT, 'glIndexf'), null, 'void', 'int');
function glIndexf(c) {
  return call('glIndexf', c);
}
define('glIndexfv', dlsym(RTLD_DEFAULT, 'glIndexfv'), null, 'void', 'void *');
function glIndexfv(c) {
  return call('glIndexfv', c);
}
define('glIndexi', dlsym(RTLD_DEFAULT, 'glIndexi'), null, 'void', 'int');
function glIndexi(c) {
  return call('glIndexi', c);
}
define('glIndexiv', dlsym(RTLD_DEFAULT, 'glIndexiv'), null, 'void', 'void *');
function glIndexiv(c) {
  return call('glIndexiv', c);
}
define('glIndexs', dlsym(RTLD_DEFAULT, 'glIndexs'), null, 'void', 'int16');
function glIndexs(c) {
  return call('glIndexs', c);
}
define('glIndexsv', dlsym(RTLD_DEFAULT, 'glIndexsv'), null, 'void', 'void *');
function glIndexsv(c) {
  return call('glIndexsv', c);
}
define('glIndexub', dlsym(RTLD_DEFAULT, 'glIndexub'), null, 'void', 'int8');
function glIndexub(c) {
  return call('glIndexub', c);
}
define('glIndexubv', dlsym(RTLD_DEFAULT, 'glIndexubv'), null, 'void', 'void *');
function glIndexubv(c) {
  return call('glIndexubv', c);
}
define('glInitNames', dlsym(RTLD_DEFAULT, 'glInitNames'), null, 'void');
function glInitNames() {
  return call('glInitNames', );
}
define('glInterleavedArrays', dlsym(RTLD_DEFAULT, 'glInterleavedArrays'), null, 'void', 'int', 'int', 'void *');
function glInterleavedArrays(format, stride, pointer) {
  return call('glInterleavedArrays', format, stride, pointer);
}
define('glIsEnabled', dlsym(RTLD_DEFAULT, 'glIsEnabled'), null, 'undefined', 'int');
function glIsEnabled(cap) {
  return call('glIsEnabled', cap);
}
define('glIsList', dlsym(RTLD_DEFAULT, 'glIsList'), null, 'undefined', 'uint32');
function glIsList(list) {
  return call('glIsList', list);
}
define('glIsTexture', dlsym(RTLD_DEFAULT, 'glIsTexture'), null, 'undefined', 'uint32');
function glIsTexture(texture) {
  return call('glIsTexture', texture);
}
define('glLightModelf', dlsym(RTLD_DEFAULT, 'glLightModelf'), null, 'void', 'int', 'int');
function glLightModelf(pname, param) {
  return call('glLightModelf', pname, param);
}
define('glLightModelfv', dlsym(RTLD_DEFAULT, 'glLightModelfv'), null, 'void', 'int', 'void *');
function glLightModelfv(pname, params) {
  return call('glLightModelfv', pname, params);
}
define('glLightModeli', dlsym(RTLD_DEFAULT, 'glLightModeli'), null, 'void', 'int', 'int');
function glLightModeli(pname, param) {
  return call('glLightModeli', pname, param);
}
define('glLightModeliv', dlsym(RTLD_DEFAULT, 'glLightModeliv'), null, 'void', 'int', 'void *');
function glLightModeliv(pname, params) {
  return call('glLightModeliv', pname, params);
}
define('glLightf', dlsym(RTLD_DEFAULT, 'glLightf'), null, 'void', 'int', 'int', 'int');
function glLightf(light, pname, param) {
  return call('glLightf', light, pname, param);
}
define('glLightfv', dlsym(RTLD_DEFAULT, 'glLightfv'), null, 'void', 'int', 'int', 'void *');
function glLightfv(light, pname, params) {
  return call('glLightfv', light, pname, params);
}
define('glLighti', dlsym(RTLD_DEFAULT, 'glLighti'), null, 'void', 'int', 'int', 'int');
function glLighti(light, pname, param) {
  return call('glLighti', light, pname, param);
}
define('glLightiv', dlsym(RTLD_DEFAULT, 'glLightiv'), null, 'void', 'int', 'int', 'void *');
function glLightiv(light, pname, params) {
  return call('glLightiv', light, pname, params);
}
define('glLineStipple', dlsym(RTLD_DEFAULT, 'glLineStipple'), null, 'void', 'int', 'uint16');
function glLineStipple(factor, pattern) {
  return call('glLineStipple', factor, pattern);
}
define('glLineWidth', dlsym(RTLD_DEFAULT, 'glLineWidth'), null, 'void', 'int');
function glLineWidth(width) {
  return call('glLineWidth', width);
}
define('glListBase', dlsym(RTLD_DEFAULT, 'glListBase'), null, 'void', 'uint32');
function glListBase(base) {
  return call('glListBase', base);
}
define('glLoadIdentity', dlsym(RTLD_DEFAULT, 'glLoadIdentity'), null, 'void');
function glLoadIdentity() {
  return call('glLoadIdentity', );
}
define('glLoadMatrixd', dlsym(RTLD_DEFAULT, 'glLoadMatrixd'), null, 'void', 'void *');
function glLoadMatrixd(m) {
  return call('glLoadMatrixd', m);
}
define('glLoadMatrixf', dlsym(RTLD_DEFAULT, 'glLoadMatrixf'), null, 'void', 'void *');
function glLoadMatrixf(m) {
  return call('glLoadMatrixf', m);
}
define('glLoadName', dlsym(RTLD_DEFAULT, 'glLoadName'), null, 'void', 'uint32');
function glLoadName(name) {
  return call('glLoadName', name);
}
define('glLoadTransposeMatrixd', dlsym(RTLD_DEFAULT, 'glLoadTransposeMatrixd'), null, 'void', 'void *');
function glLoadTransposeMatrixd(m) {
  return call('glLoadTransposeMatrixd', m);
}
define('glLoadTransposeMatrixf', dlsym(RTLD_DEFAULT, 'glLoadTransposeMatrixf'), null, 'void', 'void *');
function glLoadTransposeMatrixf(m) {
  return call('glLoadTransposeMatrixf', m);
}
define('glLogicOp', dlsym(RTLD_DEFAULT, 'glLogicOp'), null, 'void', 'int');
function glLogicOp(opcode) {
  return call('glLogicOp', opcode);
}
define('glMap1d', dlsym(RTLD_DEFAULT, 'glMap1d'), null, 'void', 'int', 'long', 'long', 'int', 'int', 'void *');
function glMap1d(target, u1, u2, stride, order, points) {
  return call('glMap1d', target, u1, u2, stride, order, points);
}
define('glMap1f', dlsym(RTLD_DEFAULT, 'glMap1f'), null, 'void', 'int', 'int', 'int', 'int', 'int', 'void *');
function glMap1f(target, u1, u2, stride, order, points) {
  return call('glMap1f', target, u1, u2, stride, order, points);
}
define('glMap2d', dlsym(RTLD_DEFAULT, 'glMap2d'), null, 'void', 'int', 'long', 'long', 'int', 'int', 'long', 'long', 'int', 'int', 'void *');
function glMap2d(target, u1, u2, ustride, uorder, v1, v2, vstride, vorder, points) {
  return call('glMap2d', target, u1, u2, ustride, uorder, v1, v2, vstride, vorder, points);
}
define('glMap2f', dlsym(RTLD_DEFAULT, 'glMap2f'), null, 'void', 'int', 'int', 'int', 'int', 'int', 'int', 'int', 'int', 'int', 'void *');
function glMap2f(target, u1, u2, ustride, uorder, v1, v2, vstride, vorder, points) {
  return call('glMap2f', target, u1, u2, ustride, uorder, v1, v2, vstride, vorder, points);
}
define('glMapGrid1d', dlsym(RTLD_DEFAULT, 'glMapGrid1d'), null, 'void', 'int', 'long', 'long');
function glMapGrid1d(un, u1, u2) {
  return call('glMapGrid1d', un, u1, u2);
}
define('glMapGrid1f', dlsym(RTLD_DEFAULT, 'glMapGrid1f'), null, 'void', 'int', 'int', 'int');
function glMapGrid1f(un, u1, u2) {
  return call('glMapGrid1f', un, u1, u2);
}
define('glMapGrid2d', dlsym(RTLD_DEFAULT, 'glMapGrid2d'), null, 'void', 'int', 'long', 'long', 'int', 'long', 'long');
function glMapGrid2d(un, u1, u2, vn, v1, v2) {
  return call('glMapGrid2d', un, u1, u2, vn, v1, v2);
}
define('glMapGrid2f', dlsym(RTLD_DEFAULT, 'glMapGrid2f'), null, 'void', 'int', 'int', 'int', 'int', 'int', 'int');
function glMapGrid2f(un, u1, u2, vn, v1, v2) {
  return call('glMapGrid2f', un, u1, u2, vn, v1, v2);
}
define('glMaterialf', dlsym(RTLD_DEFAULT, 'glMaterialf'), null, 'void', 'int', 'int', 'int');
function glMaterialf(face, pname, param) {
  return call('glMaterialf', face, pname, param);
}
define('glMaterialfv', dlsym(RTLD_DEFAULT, 'glMaterialfv'), null, 'void', 'int', 'int', 'void *');
function glMaterialfv(face, pname, params) {
  return call('glMaterialfv', face, pname, params);
}
define('glMateriali', dlsym(RTLD_DEFAULT, 'glMateriali'), null, 'void', 'int', 'int', 'int');
function glMateriali(face, pname, param) {
  return call('glMateriali', face, pname, param);
}
define('glMaterialiv', dlsym(RTLD_DEFAULT, 'glMaterialiv'), null, 'void', 'int', 'int', 'void *');
function glMaterialiv(face, pname, params) {
  return call('glMaterialiv', face, pname, params);
}
define('glMatrixMode', dlsym(RTLD_DEFAULT, 'glMatrixMode'), null, 'void', 'int');
function glMatrixMode(mode) {
  return call('glMatrixMode', mode);
}
define('glMinmax', dlsym(RTLD_DEFAULT, 'glMinmax'), null, 'void', 'int', 'int', 'int8');
function glMinmax(target, internalformat, sink) {
  return call('glMinmax', target, internalformat, sink);
}
define('glMultMatrixd', dlsym(RTLD_DEFAULT, 'glMultMatrixd'), null, 'void', 'void *');
function glMultMatrixd(m) {
  return call('glMultMatrixd', m);
}
define('glMultMatrixf', dlsym(RTLD_DEFAULT, 'glMultMatrixf'), null, 'void', 'void *');
function glMultMatrixf(m) {
  return call('glMultMatrixf', m);
}
define('glMultTransposeMatrixd', dlsym(RTLD_DEFAULT, 'glMultTransposeMatrixd'), null, 'void', 'void *');
function glMultTransposeMatrixd(m) {
  return call('glMultTransposeMatrixd', m);
}
define('glMultTransposeMatrixf', dlsym(RTLD_DEFAULT, 'glMultTransposeMatrixf'), null, 'void', 'void *');
function glMultTransposeMatrixf(m) {
  return call('glMultTransposeMatrixf', m);
}
define('glMultiTexCoord1d', dlsym(RTLD_DEFAULT, 'glMultiTexCoord1d'), null, 'void', 'int', 'long');
function glMultiTexCoord1d(target, s) {
  return call('glMultiTexCoord1d', target, s);
}
define('glMultiTexCoord1dARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord1dARB'), null, 'void', 'int', 'long');
function glMultiTexCoord1dARB(target, s) {
  return call('glMultiTexCoord1dARB', target, s);
}
define('glMultiTexCoord1dv', dlsym(RTLD_DEFAULT, 'glMultiTexCoord1dv'), null, 'void', 'int', 'void *');
function glMultiTexCoord1dv(target, v) {
  return call('glMultiTexCoord1dv', target, v);
}
define('glMultiTexCoord1dvARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord1dvARB'), null, 'void', 'int', 'void *');
function glMultiTexCoord1dvARB(target, v) {
  return call('glMultiTexCoord1dvARB', target, v);
}
define('glMultiTexCoord1f', dlsym(RTLD_DEFAULT, 'glMultiTexCoord1f'), null, 'void', 'int', 'int');
function glMultiTexCoord1f(target, s) {
  return call('glMultiTexCoord1f', target, s);
}
define('glMultiTexCoord1fARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord1fARB'), null, 'void', 'int', 'int');
function glMultiTexCoord1fARB(target, s) {
  return call('glMultiTexCoord1fARB', target, s);
}
define('glMultiTexCoord1fv', dlsym(RTLD_DEFAULT, 'glMultiTexCoord1fv'), null, 'void', 'int', 'void *');
function glMultiTexCoord1fv(target, v) {
  return call('glMultiTexCoord1fv', target, v);
}
define('glMultiTexCoord1fvARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord1fvARB'), null, 'void', 'int', 'void *');
function glMultiTexCoord1fvARB(target, v) {
  return call('glMultiTexCoord1fvARB', target, v);
}
define('glMultiTexCoord1i', dlsym(RTLD_DEFAULT, 'glMultiTexCoord1i'), null, 'void', 'int', 'int');
function glMultiTexCoord1i(target, s) {
  return call('glMultiTexCoord1i', target, s);
}
define('glMultiTexCoord1iARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord1iARB'), null, 'void', 'int', 'int');
function glMultiTexCoord1iARB(target, s) {
  return call('glMultiTexCoord1iARB', target, s);
}
define('glMultiTexCoord1iv', dlsym(RTLD_DEFAULT, 'glMultiTexCoord1iv'), null, 'void', 'int', 'void *');
function glMultiTexCoord1iv(target, v) {
  return call('glMultiTexCoord1iv', target, v);
}
define('glMultiTexCoord1ivARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord1ivARB'), null, 'void', 'int', 'void *');
function glMultiTexCoord1ivARB(target, v) {
  return call('glMultiTexCoord1ivARB', target, v);
}
define('glMultiTexCoord1s', dlsym(RTLD_DEFAULT, 'glMultiTexCoord1s'), null, 'void', 'int', 'int16');
function glMultiTexCoord1s(target, s) {
  return call('glMultiTexCoord1s', target, s);
}
define('glMultiTexCoord1sARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord1sARB'), null, 'void', 'int', 'int16');
function glMultiTexCoord1sARB(target, s) {
  return call('glMultiTexCoord1sARB', target, s);
}
define('glMultiTexCoord1sv', dlsym(RTLD_DEFAULT, 'glMultiTexCoord1sv'), null, 'void', 'int', 'void *');
function glMultiTexCoord1sv(target, v) {
  return call('glMultiTexCoord1sv', target, v);
}
define('glMultiTexCoord1svARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord1svARB'), null, 'void', 'int', 'void *');
function glMultiTexCoord1svARB(target, v) {
  return call('glMultiTexCoord1svARB', target, v);
}
define('glMultiTexCoord2d', dlsym(RTLD_DEFAULT, 'glMultiTexCoord2d'), null, 'void', 'int', 'long', 'long');
function glMultiTexCoord2d(target, s, t) {
  return call('glMultiTexCoord2d', target, s, t);
}
define('glMultiTexCoord2dARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord2dARB'), null, 'void', 'int', 'long', 'long');
function glMultiTexCoord2dARB(target, s, t) {
  return call('glMultiTexCoord2dARB', target, s, t);
}
define('glMultiTexCoord2dv', dlsym(RTLD_DEFAULT, 'glMultiTexCoord2dv'), null, 'void', 'int', 'void *');
function glMultiTexCoord2dv(target, v) {
  return call('glMultiTexCoord2dv', target, v);
}
define('glMultiTexCoord2dvARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord2dvARB'), null, 'void', 'int', 'void *');
function glMultiTexCoord2dvARB(target, v) {
  return call('glMultiTexCoord2dvARB', target, v);
}
define('glMultiTexCoord2f', dlsym(RTLD_DEFAULT, 'glMultiTexCoord2f'), null, 'void', 'int', 'int', 'int');
function glMultiTexCoord2f(target, s, t) {
  return call('glMultiTexCoord2f', target, s, t);
}
define('glMultiTexCoord2fARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord2fARB'), null, 'void', 'int', 'int', 'int');
function glMultiTexCoord2fARB(target, s, t) {
  return call('glMultiTexCoord2fARB', target, s, t);
}
define('glMultiTexCoord2fv', dlsym(RTLD_DEFAULT, 'glMultiTexCoord2fv'), null, 'void', 'int', 'void *');
function glMultiTexCoord2fv(target, v) {
  return call('glMultiTexCoord2fv', target, v);
}
define('glMultiTexCoord2fvARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord2fvARB'), null, 'void', 'int', 'void *');
function glMultiTexCoord2fvARB(target, v) {
  return call('glMultiTexCoord2fvARB', target, v);
}
define('glMultiTexCoord2i', dlsym(RTLD_DEFAULT, 'glMultiTexCoord2i'), null, 'void', 'int', 'int', 'int');
function glMultiTexCoord2i(target, s, t) {
  return call('glMultiTexCoord2i', target, s, t);
}
define('glMultiTexCoord2iARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord2iARB'), null, 'void', 'int', 'int', 'int');
function glMultiTexCoord2iARB(target, s, t) {
  return call('glMultiTexCoord2iARB', target, s, t);
}
define('glMultiTexCoord2iv', dlsym(RTLD_DEFAULT, 'glMultiTexCoord2iv'), null, 'void', 'int', 'void *');
function glMultiTexCoord2iv(target, v) {
  return call('glMultiTexCoord2iv', target, v);
}
define('glMultiTexCoord2ivARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord2ivARB'), null, 'void', 'int', 'void *');
function glMultiTexCoord2ivARB(target, v) {
  return call('glMultiTexCoord2ivARB', target, v);
}
define('glMultiTexCoord2s', dlsym(RTLD_DEFAULT, 'glMultiTexCoord2s'), null, 'void', 'int', 'int16', 'int16');
function glMultiTexCoord2s(target, s, t) {
  return call('glMultiTexCoord2s', target, s, t);
}
define('glMultiTexCoord2sARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord2sARB'), null, 'void', 'int', 'int16', 'int16');
function glMultiTexCoord2sARB(target, s, t) {
  return call('glMultiTexCoord2sARB', target, s, t);
}
define('glMultiTexCoord2sv', dlsym(RTLD_DEFAULT, 'glMultiTexCoord2sv'), null, 'void', 'int', 'void *');
function glMultiTexCoord2sv(target, v) {
  return call('glMultiTexCoord2sv', target, v);
}
define('glMultiTexCoord2svARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord2svARB'), null, 'void', 'int', 'void *');
function glMultiTexCoord2svARB(target, v) {
  return call('glMultiTexCoord2svARB', target, v);
}
define('glMultiTexCoord3d', dlsym(RTLD_DEFAULT, 'glMultiTexCoord3d'), null, 'void', 'int', 'long', 'long', 'long');
function glMultiTexCoord3d(target, s, t, r) {
  return call('glMultiTexCoord3d', target, s, t, r);
}
define('glMultiTexCoord3dARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord3dARB'), null, 'void', 'int', 'long', 'long', 'long');
function glMultiTexCoord3dARB(target, s, t, r) {
  return call('glMultiTexCoord3dARB', target, s, t, r);
}
define('glMultiTexCoord3dv', dlsym(RTLD_DEFAULT, 'glMultiTexCoord3dv'), null, 'void', 'int', 'void *');
function glMultiTexCoord3dv(target, v) {
  return call('glMultiTexCoord3dv', target, v);
}
define('glMultiTexCoord3dvARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord3dvARB'), null, 'void', 'int', 'void *');
function glMultiTexCoord3dvARB(target, v) {
  return call('glMultiTexCoord3dvARB', target, v);
}
define('glMultiTexCoord3f', dlsym(RTLD_DEFAULT, 'glMultiTexCoord3f'), null, 'void', 'int', 'int', 'int', 'int');
function glMultiTexCoord3f(target, s, t, r) {
  return call('glMultiTexCoord3f', target, s, t, r);
}
define('glMultiTexCoord3fARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord3fARB'), null, 'void', 'int', 'int', 'int', 'int');
function glMultiTexCoord3fARB(target, s, t, r) {
  return call('glMultiTexCoord3fARB', target, s, t, r);
}
define('glMultiTexCoord3fv', dlsym(RTLD_DEFAULT, 'glMultiTexCoord3fv'), null, 'void', 'int', 'void *');
function glMultiTexCoord3fv(target, v) {
  return call('glMultiTexCoord3fv', target, v);
}
define('glMultiTexCoord3fvARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord3fvARB'), null, 'void', 'int', 'void *');
function glMultiTexCoord3fvARB(target, v) {
  return call('glMultiTexCoord3fvARB', target, v);
}
define('glMultiTexCoord3i', dlsym(RTLD_DEFAULT, 'glMultiTexCoord3i'), null, 'void', 'int', 'int', 'int', 'int');
function glMultiTexCoord3i(target, s, t, r) {
  return call('glMultiTexCoord3i', target, s, t, r);
}
define('glMultiTexCoord3iARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord3iARB'), null, 'void', 'int', 'int', 'int', 'int');
function glMultiTexCoord3iARB(target, s, t, r) {
  return call('glMultiTexCoord3iARB', target, s, t, r);
}
define('glMultiTexCoord3iv', dlsym(RTLD_DEFAULT, 'glMultiTexCoord3iv'), null, 'void', 'int', 'void *');
function glMultiTexCoord3iv(target, v) {
  return call('glMultiTexCoord3iv', target, v);
}
define('glMultiTexCoord3ivARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord3ivARB'), null, 'void', 'int', 'void *');
function glMultiTexCoord3ivARB(target, v) {
  return call('glMultiTexCoord3ivARB', target, v);
}
define('glMultiTexCoord3s', dlsym(RTLD_DEFAULT, 'glMultiTexCoord3s'), null, 'void', 'int', 'int16', 'int16', 'int16');
function glMultiTexCoord3s(target, s, t, r) {
  return call('glMultiTexCoord3s', target, s, t, r);
}
define('glMultiTexCoord3sARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord3sARB'), null, 'void', 'int', 'int16', 'int16', 'int16');
function glMultiTexCoord3sARB(target, s, t, r) {
  return call('glMultiTexCoord3sARB', target, s, t, r);
}
define('glMultiTexCoord3sv', dlsym(RTLD_DEFAULT, 'glMultiTexCoord3sv'), null, 'void', 'int', 'void *');
function glMultiTexCoord3sv(target, v) {
  return call('glMultiTexCoord3sv', target, v);
}
define('glMultiTexCoord3svARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord3svARB'), null, 'void', 'int', 'void *');
function glMultiTexCoord3svARB(target, v) {
  return call('glMultiTexCoord3svARB', target, v);
}
define('glMultiTexCoord4d', dlsym(RTLD_DEFAULT, 'glMultiTexCoord4d'), null, 'void', 'int', 'long', 'long', 'long', 'long');
function glMultiTexCoord4d(target, s, t, r, q) {
  return call('glMultiTexCoord4d', target, s, t, r, q);
}
define('glMultiTexCoord4dARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord4dARB'), null, 'void', 'int', 'long', 'long', 'long', 'long');
function glMultiTexCoord4dARB(target, s, t, r, q) {
  return call('glMultiTexCoord4dARB', target, s, t, r, q);
}
define('glMultiTexCoord4dv', dlsym(RTLD_DEFAULT, 'glMultiTexCoord4dv'), null, 'void', 'int', 'void *');
function glMultiTexCoord4dv(target, v) {
  return call('glMultiTexCoord4dv', target, v);
}
define('glMultiTexCoord4dvARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord4dvARB'), null, 'void', 'int', 'void *');
function glMultiTexCoord4dvARB(target, v) {
  return call('glMultiTexCoord4dvARB', target, v);
}
define('glMultiTexCoord4f', dlsym(RTLD_DEFAULT, 'glMultiTexCoord4f'), null, 'void', 'int', 'int', 'int', 'int', 'int');
function glMultiTexCoord4f(target, s, t, r, q) {
  return call('glMultiTexCoord4f', target, s, t, r, q);
}
define('glMultiTexCoord4fARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord4fARB'), null, 'void', 'int', 'int', 'int', 'int', 'int');
function glMultiTexCoord4fARB(target, s, t, r, q) {
  return call('glMultiTexCoord4fARB', target, s, t, r, q);
}
define('glMultiTexCoord4fv', dlsym(RTLD_DEFAULT, 'glMultiTexCoord4fv'), null, 'void', 'int', 'void *');
function glMultiTexCoord4fv(target, v) {
  return call('glMultiTexCoord4fv', target, v);
}
define('glMultiTexCoord4fvARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord4fvARB'), null, 'void', 'int', 'void *');
function glMultiTexCoord4fvARB(target, v) {
  return call('glMultiTexCoord4fvARB', target, v);
}
define('glMultiTexCoord4i', dlsym(RTLD_DEFAULT, 'glMultiTexCoord4i'), null, 'void', 'int', 'int', 'int', 'int', 'int');
function glMultiTexCoord4i(target, s, t, r, q) {
  return call('glMultiTexCoord4i', target, s, t, r, q);
}
define('glMultiTexCoord4iARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord4iARB'), null, 'void', 'int', 'int', 'int', 'int', 'int');
function glMultiTexCoord4iARB(target, s, t, r, q) {
  return call('glMultiTexCoord4iARB', target, s, t, r, q);
}
define('glMultiTexCoord4iv', dlsym(RTLD_DEFAULT, 'glMultiTexCoord4iv'), null, 'void', 'int', 'void *');
function glMultiTexCoord4iv(target, v) {
  return call('glMultiTexCoord4iv', target, v);
}
define('glMultiTexCoord4ivARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord4ivARB'), null, 'void', 'int', 'void *');
function glMultiTexCoord4ivARB(target, v) {
  return call('glMultiTexCoord4ivARB', target, v);
}
define('glMultiTexCoord4s', dlsym(RTLD_DEFAULT, 'glMultiTexCoord4s'), null, 'void', 'int', 'int16', 'int16', 'int16', 'int16');
function glMultiTexCoord4s(target, s, t, r, q) {
  return call('glMultiTexCoord4s', target, s, t, r, q);
}
define('glMultiTexCoord4sARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord4sARB'), null, 'void', 'int', 'int16', 'int16', 'int16', 'int16');
function glMultiTexCoord4sARB(target, s, t, r, q) {
  return call('glMultiTexCoord4sARB', target, s, t, r, q);
}
define('glMultiTexCoord4sv', dlsym(RTLD_DEFAULT, 'glMultiTexCoord4sv'), null, 'void', 'int', 'void *');
function glMultiTexCoord4sv(target, v) {
  return call('glMultiTexCoord4sv', target, v);
}
define('glMultiTexCoord4svARB', dlsym(RTLD_DEFAULT, 'glMultiTexCoord4svARB'), null, 'void', 'int', 'void *');
function glMultiTexCoord4svARB(target, v) {
  return call('glMultiTexCoord4svARB', target, v);
}
define('glNewList', dlsym(RTLD_DEFAULT, 'glNewList'), null, 'void', 'uint32', 'int');
function glNewList(list, mode) {
  return call('glNewList', list, mode);
}
define('glNormal3b', dlsym(RTLD_DEFAULT, 'glNormal3b'), null, 'void', 'int8', 'int8', 'int8');
function glNormal3b(nx, ny, nz) {
  return call('glNormal3b', nx, ny, nz);
}
define('glNormal3bv', dlsym(RTLD_DEFAULT, 'glNormal3bv'), null, 'void', 'void *');
function glNormal3bv(v) {
  return call('glNormal3bv', v);
}
define('glNormal3d', dlsym(RTLD_DEFAULT, 'glNormal3d'), null, 'void', 'long', 'long', 'long');
function glNormal3d(nx, ny, nz) {
  return call('glNormal3d', nx, ny, nz);
}
define('glNormal3dv', dlsym(RTLD_DEFAULT, 'glNormal3dv'), null, 'void', 'void *');
function glNormal3dv(v) {
  return call('glNormal3dv', v);
}
define('glNormal3f', dlsym(RTLD_DEFAULT, 'glNormal3f'), null, 'void', 'int', 'int', 'int');
function glNormal3f(nx, ny, nz) {
  return call('glNormal3f', nx, ny, nz);
}
define('glNormal3fv', dlsym(RTLD_DEFAULT, 'glNormal3fv'), null, 'void', 'void *');
function glNormal3fv(v) {
  return call('glNormal3fv', v);
}
define('glNormal3i', dlsym(RTLD_DEFAULT, 'glNormal3i'), null, 'void', 'int', 'int', 'int');
function glNormal3i(nx, ny, nz) {
  return call('glNormal3i', nx, ny, nz);
}
define('glNormal3iv', dlsym(RTLD_DEFAULT, 'glNormal3iv'), null, 'void', 'void *');
function glNormal3iv(v) {
  return call('glNormal3iv', v);
}
define('glNormal3s', dlsym(RTLD_DEFAULT, 'glNormal3s'), null, 'void', 'int16', 'int16', 'int16');
function glNormal3s(nx, ny, nz) {
  return call('glNormal3s', nx, ny, nz);
}
define('glNormal3sv', dlsym(RTLD_DEFAULT, 'glNormal3sv'), null, 'void', 'void *');
function glNormal3sv(v) {
  return call('glNormal3sv', v);
}
define('glNormalPointer', dlsym(RTLD_DEFAULT, 'glNormalPointer'), null, 'void', 'int', 'int', 'void *');
function glNormalPointer(type, stride, ptr) {
  return call('glNormalPointer', type, stride, ptr);
}
define('glOrtho', dlsym(RTLD_DEFAULT, 'glOrtho'), null, 'void', 'long', 'long', 'long', 'long', 'long', 'long');
function glOrtho(left, right, bottom, top, near_val, far_val) {
  return call('glOrtho', left, right, bottom, top, near_val, far_val);
}
define('glPassThrough', dlsym(RTLD_DEFAULT, 'glPassThrough'), null, 'void', 'int');
function glPassThrough(token) {
  return call('glPassThrough', token);
}
define('glPixelMapfv', dlsym(RTLD_DEFAULT, 'glPixelMapfv'), null, 'void', 'int', 'int', 'void *');
function glPixelMapfv(map, mapsize, values) {
  return call('glPixelMapfv', map, mapsize, values);
}
define('glPixelMapuiv', dlsym(RTLD_DEFAULT, 'glPixelMapuiv'), null, 'void', 'int', 'int', 'void *');
function glPixelMapuiv(map, mapsize, values) {
  return call('glPixelMapuiv', map, mapsize, values);
}
define('glPixelMapusv', dlsym(RTLD_DEFAULT, 'glPixelMapusv'), null, 'void', 'int', 'int', 'void *');
function glPixelMapusv(map, mapsize, values) {
  return call('glPixelMapusv', map, mapsize, values);
}
define('glPixelStoref', dlsym(RTLD_DEFAULT, 'glPixelStoref'), null, 'void', 'int', 'int');
function glPixelStoref(pname, param) {
  return call('glPixelStoref', pname, param);
}
define('glPixelStorei', dlsym(RTLD_DEFAULT, 'glPixelStorei'), null, 'void', 'int', 'int');
function glPixelStorei(pname, param) {
  return call('glPixelStorei', pname, param);
}
define('glPixelTransferf', dlsym(RTLD_DEFAULT, 'glPixelTransferf'), null, 'void', 'int', 'int');
function glPixelTransferf(pname, param) {
  return call('glPixelTransferf', pname, param);
}
define('glPixelTransferi', dlsym(RTLD_DEFAULT, 'glPixelTransferi'), null, 'void', 'int', 'int');
function glPixelTransferi(pname, param) {
  return call('glPixelTransferi', pname, param);
}
define('glPixelZoom', dlsym(RTLD_DEFAULT, 'glPixelZoom'), null, 'void', 'int', 'int');
function glPixelZoom(xfactor, yfactor) {
  return call('glPixelZoom', xfactor, yfactor);
}
define('glPointSize', dlsym(RTLD_DEFAULT, 'glPointSize'), null, 'void', 'int');
function glPointSize(size) {
  return call('glPointSize', size);
}
define('glPolygonMode', dlsym(RTLD_DEFAULT, 'glPolygonMode'), null, 'void', 'int', 'int');
function glPolygonMode(face, mode) {
  return call('glPolygonMode', face, mode);
}
define('glPolygonOffset', dlsym(RTLD_DEFAULT, 'glPolygonOffset'), null, 'void', 'int', 'int');
function glPolygonOffset(factor, units) {
  return call('glPolygonOffset', factor, units);
}
define('glPolygonStipple', dlsym(RTLD_DEFAULT, 'glPolygonStipple'), null, 'void', 'void *');
function glPolygonStipple(mask) {
  return call('glPolygonStipple', mask);
}
define('glPopAttrib', dlsym(RTLD_DEFAULT, 'glPopAttrib'), null, 'void');
function glPopAttrib() {
  return call('glPopAttrib', );
}
define('glPopClientAttrib', dlsym(RTLD_DEFAULT, 'glPopClientAttrib'), null, 'void');
function glPopClientAttrib() {
  return call('glPopClientAttrib', );
}
define('glPopMatrix', dlsym(RTLD_DEFAULT, 'glPopMatrix'), null, 'void');
function glPopMatrix() {
  return call('glPopMatrix', );
}
define('glPopName', dlsym(RTLD_DEFAULT, 'glPopName'), null, 'void');
function glPopName() {
  return call('glPopName', );
}
define('glPrioritizeTextures', dlsym(RTLD_DEFAULT, 'glPrioritizeTextures'), null, 'void', 'int', 'void *', 'void *');
function glPrioritizeTextures(n, textures, priorities) {
  return call('glPrioritizeTextures', n, textures, priorities);
}
define('glPushAttrib', dlsym(RTLD_DEFAULT, 'glPushAttrib'), null, 'void', 'int');
function glPushAttrib(mask) {
  return call('glPushAttrib', mask);
}
define('glPushClientAttrib', dlsym(RTLD_DEFAULT, 'glPushClientAttrib'), null, 'void', 'int');
function glPushClientAttrib(mask) {
  return call('glPushClientAttrib', mask);
}
define('glPushMatrix', dlsym(RTLD_DEFAULT, 'glPushMatrix'), null, 'void');
function glPushMatrix() {
  return call('glPushMatrix', );
}
define('glPushName', dlsym(RTLD_DEFAULT, 'glPushName'), null, 'void', 'uint32');
function glPushName(name) {
  return call('glPushName', name);
}
define('glRasterPos2d', dlsym(RTLD_DEFAULT, 'glRasterPos2d'), null, 'void', 'long', 'long');
function glRasterPos2d(x, y) {
  return call('glRasterPos2d', x, y);
}
define('glRasterPos2dv', dlsym(RTLD_DEFAULT, 'glRasterPos2dv'), null, 'void', 'void *');
function glRasterPos2dv(v) {
  return call('glRasterPos2dv', v);
}
define('glRasterPos2f', dlsym(RTLD_DEFAULT, 'glRasterPos2f'), null, 'void', 'int', 'int');
function glRasterPos2f(x, y) {
  return call('glRasterPos2f', x, y);
}
define('glRasterPos2fv', dlsym(RTLD_DEFAULT, 'glRasterPos2fv'), null, 'void', 'void *');
function glRasterPos2fv(v) {
  return call('glRasterPos2fv', v);
}
define('glRasterPos2i', dlsym(RTLD_DEFAULT, 'glRasterPos2i'), null, 'void', 'int', 'int');
function glRasterPos2i(x, y) {
  return call('glRasterPos2i', x, y);
}
define('glRasterPos2iv', dlsym(RTLD_DEFAULT, 'glRasterPos2iv'), null, 'void', 'void *');
function glRasterPos2iv(v) {
  return call('glRasterPos2iv', v);
}
define('glRasterPos2s', dlsym(RTLD_DEFAULT, 'glRasterPos2s'), null, 'void', 'int16', 'int16');
function glRasterPos2s(x, y) {
  return call('glRasterPos2s', x, y);
}
define('glRasterPos2sv', dlsym(RTLD_DEFAULT, 'glRasterPos2sv'), null, 'void', 'void *');
function glRasterPos2sv(v) {
  return call('glRasterPos2sv', v);
}
define('glRasterPos3d', dlsym(RTLD_DEFAULT, 'glRasterPos3d'), null, 'void', 'long', 'long', 'long');
function glRasterPos3d(x, y, z) {
  return call('glRasterPos3d', x, y, z);
}
define('glRasterPos3dv', dlsym(RTLD_DEFAULT, 'glRasterPos3dv'), null, 'void', 'void *');
function glRasterPos3dv(v) {
  return call('glRasterPos3dv', v);
}
define('glRasterPos3f', dlsym(RTLD_DEFAULT, 'glRasterPos3f'), null, 'void', 'int', 'int', 'int');
function glRasterPos3f(x, y, z) {
  return call('glRasterPos3f', x, y, z);
}
define('glRasterPos3fv', dlsym(RTLD_DEFAULT, 'glRasterPos3fv'), null, 'void', 'void *');
function glRasterPos3fv(v) {
  return call('glRasterPos3fv', v);
}
define('glRasterPos3i', dlsym(RTLD_DEFAULT, 'glRasterPos3i'), null, 'void', 'int', 'int', 'int');
function glRasterPos3i(x, y, z) {
  return call('glRasterPos3i', x, y, z);
}
define('glRasterPos3iv', dlsym(RTLD_DEFAULT, 'glRasterPos3iv'), null, 'void', 'void *');
function glRasterPos3iv(v) {
  return call('glRasterPos3iv', v);
}
define('glRasterPos3s', dlsym(RTLD_DEFAULT, 'glRasterPos3s'), null, 'void', 'int16', 'int16', 'int16');
function glRasterPos3s(x, y, z) {
  return call('glRasterPos3s', x, y, z);
}
define('glRasterPos3sv', dlsym(RTLD_DEFAULT, 'glRasterPos3sv'), null, 'void', 'void *');
function glRasterPos3sv(v) {
  return call('glRasterPos3sv', v);
}
define('glRasterPos4d', dlsym(RTLD_DEFAULT, 'glRasterPos4d'), null, 'void', 'long', 'long', 'long', 'long');
function glRasterPos4d(x, y, z, w) {
  return call('glRasterPos4d', x, y, z, w);
}
define('glRasterPos4dv', dlsym(RTLD_DEFAULT, 'glRasterPos4dv'), null, 'void', 'void *');
function glRasterPos4dv(v) {
  return call('glRasterPos4dv', v);
}
define('glRasterPos4f', dlsym(RTLD_DEFAULT, 'glRasterPos4f'), null, 'void', 'int', 'int', 'int', 'int');
function glRasterPos4f(x, y, z, w) {
  return call('glRasterPos4f', x, y, z, w);
}
define('glRasterPos4fv', dlsym(RTLD_DEFAULT, 'glRasterPos4fv'), null, 'void', 'void *');
function glRasterPos4fv(v) {
  return call('glRasterPos4fv', v);
}
define('glRasterPos4i', dlsym(RTLD_DEFAULT, 'glRasterPos4i'), null, 'void', 'int', 'int', 'int', 'int');
function glRasterPos4i(x, y, z, w) {
  return call('glRasterPos4i', x, y, z, w);
}
define('glRasterPos4iv', dlsym(RTLD_DEFAULT, 'glRasterPos4iv'), null, 'void', 'void *');
function glRasterPos4iv(v) {
  return call('glRasterPos4iv', v);
}
define('glRasterPos4s', dlsym(RTLD_DEFAULT, 'glRasterPos4s'), null, 'void', 'int16', 'int16', 'int16', 'int16');
function glRasterPos4s(x, y, z, w) {
  return call('glRasterPos4s', x, y, z, w);
}
define('glRasterPos4sv', dlsym(RTLD_DEFAULT, 'glRasterPos4sv'), null, 'void', 'void *');
function glRasterPos4sv(v) {
  return call('glRasterPos4sv', v);
}
define('glReadBuffer', dlsym(RTLD_DEFAULT, 'glReadBuffer'), null, 'void', 'int');
function glReadBuffer(mode) {
  return call('glReadBuffer', mode);
}
define('glReadPixels', dlsym(RTLD_DEFAULT, 'glReadPixels'), null, 'void', 'int', 'int', 'int', 'int', 'int', 'int', 'void *');
function glReadPixels(x, y, width, height, format, type, pixels) {
  return call('glReadPixels', x, y, width, height, format, type, pixels);
}
define('glRectd', dlsym(RTLD_DEFAULT, 'glRectd'), null, 'void', 'long', 'long', 'long', 'long');
function glRectd(x1, y1, x2, y2) {
  return call('glRectd', x1, y1, x2, y2);
}
define('glRectdv', dlsym(RTLD_DEFAULT, 'glRectdv'), null, 'void', 'void *', 'void *');
function glRectdv(v1, v2) {
  return call('glRectdv', v1, v2);
}
define('glRectf', dlsym(RTLD_DEFAULT, 'glRectf'), null, 'void', 'int', 'int', 'int', 'int');
function glRectf(x1, y1, x2, y2) {
  return call('glRectf', x1, y1, x2, y2);
}
define('glRectfv', dlsym(RTLD_DEFAULT, 'glRectfv'), null, 'void', 'void *', 'void *');
function glRectfv(v1, v2) {
  return call('glRectfv', v1, v2);
}
define('glRecti', dlsym(RTLD_DEFAULT, 'glRecti'), null, 'void', 'int', 'int', 'int', 'int');
function glRecti(x1, y1, x2, y2) {
  return call('glRecti', x1, y1, x2, y2);
}
define('glRectiv', dlsym(RTLD_DEFAULT, 'glRectiv'), null, 'void', 'void *', 'void *');
function glRectiv(v1, v2) {
  return call('glRectiv', v1, v2);
}
define('glRects', dlsym(RTLD_DEFAULT, 'glRects'), null, 'void', 'int16', 'int16', 'int16', 'int16');
function glRects(x1, y1, x2, y2) {
  return call('glRects', x1, y1, x2, y2);
}
define('glRectsv', dlsym(RTLD_DEFAULT, 'glRectsv'), null, 'void', 'void *', 'void *');
function glRectsv(v1, v2) {
  return call('glRectsv', v1, v2);
}
define('glRenderMode', dlsym(RTLD_DEFAULT, 'glRenderMode'), null, 'int', 'int');
function glRenderMode(mode) {
  return call('glRenderMode', mode);
}
define('glResetHistogram', dlsym(RTLD_DEFAULT, 'glResetHistogram'), null, 'void', 'int');
function glResetHistogram(target) {
  return call('glResetHistogram', target);
}
define('glResetMinmax', dlsym(RTLD_DEFAULT, 'glResetMinmax'), null, 'void', 'int');
function glResetMinmax(target) {
  return call('glResetMinmax', target);
}
define('glRotated', dlsym(RTLD_DEFAULT, 'glRotated'), null, 'void', 'long', 'long', 'long', 'long');
function glRotated(angle, x, y, z) {
  return call('glRotated', angle, x, y, z);
}
define('glRotatef', dlsym(RTLD_DEFAULT, 'glRotatef'), null, 'void', 'int', 'int', 'int', 'int');
function glRotatef(angle, x, y, z) {
  return call('glRotatef', angle, x, y, z);
}
define('glSampleCoverage', dlsym(RTLD_DEFAULT, 'glSampleCoverage'), null, 'void', 'int', 'int8');
function glSampleCoverage(value, invert) {
  return call('glSampleCoverage', value, invert);
}
define('glScaled', dlsym(RTLD_DEFAULT, 'glScaled'), null, 'void', 'long', 'long', 'long');
function glScaled(x, y, z) {
  return call('glScaled', x, y, z);
}
define('glScalef', dlsym(RTLD_DEFAULT, 'glScalef'), null, 'void', 'int', 'int', 'int');
function glScalef(x, y, z) {
  return call('glScalef', x, y, z);
}
define('glScissor', dlsym(RTLD_DEFAULT, 'glScissor'), null, 'void', 'int', 'int', 'int', 'int');
function glScissor(x, y, width, height) {
  return call('glScissor', x, y, width, height);
}
define('glSelectBuffer', dlsym(RTLD_DEFAULT, 'glSelectBuffer'), null, 'void', 'int', 'void *');
function glSelectBuffer(size, buffer) {
  return call('glSelectBuffer', size, buffer);
}
define('glSeparableFilter2D', dlsym(RTLD_DEFAULT, 'glSeparableFilter2D'), null, 'void', 'int', 'int', 'int', 'int', 'int', 'int', 'void *', 'void *');
function glSeparableFilter2D(target, internalformat, width, height, format, type, row, column) {
  return call('glSeparableFilter2D', target, internalformat, width, height, format, type, row, column);
}
define('glShadeModel', dlsym(RTLD_DEFAULT, 'glShadeModel'), null, 'void', 'int');
function glShadeModel(mode) {
  return call('glShadeModel', mode);
}
define('glStencilFunc', dlsym(RTLD_DEFAULT, 'glStencilFunc'), null, 'void', 'int', 'int', 'uint32');
function glStencilFunc(func, ref, mask) {
  return call('glStencilFunc', func, ref, mask);
}
define('glStencilMask', dlsym(RTLD_DEFAULT, 'glStencilMask'), null, 'void', 'uint32');
function glStencilMask(mask) {
  return call('glStencilMask', mask);
}
define('glStencilOp', dlsym(RTLD_DEFAULT, 'glStencilOp'), null, 'void', 'int', 'int', 'int');
function glStencilOp(fail, zfail, zpass) {
  return call('glStencilOp', fail, zfail, zpass);
}
define('glTexCoord1d', dlsym(RTLD_DEFAULT, 'glTexCoord1d'), null, 'void', 'long');
function glTexCoord1d(s) {
  return call('glTexCoord1d', s);
}
define('glTexCoord1dv', dlsym(RTLD_DEFAULT, 'glTexCoord1dv'), null, 'void', 'void *');
function glTexCoord1dv(v) {
  return call('glTexCoord1dv', v);
}
define('glTexCoord1f', dlsym(RTLD_DEFAULT, 'glTexCoord1f'), null, 'void', 'int');
function glTexCoord1f(s) {
  return call('glTexCoord1f', s);
}
define('glTexCoord1fv', dlsym(RTLD_DEFAULT, 'glTexCoord1fv'), null, 'void', 'void *');
function glTexCoord1fv(v) {
  return call('glTexCoord1fv', v);
}
define('glTexCoord1i', dlsym(RTLD_DEFAULT, 'glTexCoord1i'), null, 'void', 'int');
function glTexCoord1i(s) {
  return call('glTexCoord1i', s);
}
define('glTexCoord1iv', dlsym(RTLD_DEFAULT, 'glTexCoord1iv'), null, 'void', 'void *');
function glTexCoord1iv(v) {
  return call('glTexCoord1iv', v);
}
define('glTexCoord1s', dlsym(RTLD_DEFAULT, 'glTexCoord1s'), null, 'void', 'int16');
function glTexCoord1s(s) {
  return call('glTexCoord1s', s);
}
define('glTexCoord1sv', dlsym(RTLD_DEFAULT, 'glTexCoord1sv'), null, 'void', 'void *');
function glTexCoord1sv(v) {
  return call('glTexCoord1sv', v);
}
define('glTexCoord2d', dlsym(RTLD_DEFAULT, 'glTexCoord2d'), null, 'void', 'long', 'long');
function glTexCoord2d(s, t) {
  return call('glTexCoord2d', s, t);
}
define('glTexCoord2dv', dlsym(RTLD_DEFAULT, 'glTexCoord2dv'), null, 'void', 'void *');
function glTexCoord2dv(v) {
  return call('glTexCoord2dv', v);
}
define('glTexCoord2f', dlsym(RTLD_DEFAULT, 'glTexCoord2f'), null, 'void', 'int', 'int');
function glTexCoord2f(s, t) {
  return call('glTexCoord2f', s, t);
}
define('glTexCoord2fv', dlsym(RTLD_DEFAULT, 'glTexCoord2fv'), null, 'void', 'void *');
function glTexCoord2fv(v) {
  return call('glTexCoord2fv', v);
}
define('glTexCoord2i', dlsym(RTLD_DEFAULT, 'glTexCoord2i'), null, 'void', 'int', 'int');
function glTexCoord2i(s, t) {
  return call('glTexCoord2i', s, t);
}
define('glTexCoord2iv', dlsym(RTLD_DEFAULT, 'glTexCoord2iv'), null, 'void', 'void *');
function glTexCoord2iv(v) {
  return call('glTexCoord2iv', v);
}
define('glTexCoord2s', dlsym(RTLD_DEFAULT, 'glTexCoord2s'), null, 'void', 'int16', 'int16');
function glTexCoord2s(s, t) {
  return call('glTexCoord2s', s, t);
}
define('glTexCoord2sv', dlsym(RTLD_DEFAULT, 'glTexCoord2sv'), null, 'void', 'void *');
function glTexCoord2sv(v) {
  return call('glTexCoord2sv', v);
}
define('glTexCoord3d', dlsym(RTLD_DEFAULT, 'glTexCoord3d'), null, 'void', 'long', 'long', 'long');
function glTexCoord3d(s, t, r) {
  return call('glTexCoord3d', s, t, r);
}
define('glTexCoord3dv', dlsym(RTLD_DEFAULT, 'glTexCoord3dv'), null, 'void', 'void *');
function glTexCoord3dv(v) {
  return call('glTexCoord3dv', v);
}
define('glTexCoord3f', dlsym(RTLD_DEFAULT, 'glTexCoord3f'), null, 'void', 'int', 'int', 'int');
function glTexCoord3f(s, t, r) {
  return call('glTexCoord3f', s, t, r);
}
define('glTexCoord3fv', dlsym(RTLD_DEFAULT, 'glTexCoord3fv'), null, 'void', 'void *');
function glTexCoord3fv(v) {
  return call('glTexCoord3fv', v);
}
define('glTexCoord3i', dlsym(RTLD_DEFAULT, 'glTexCoord3i'), null, 'void', 'int', 'int', 'int');
function glTexCoord3i(s, t, r) {
  return call('glTexCoord3i', s, t, r);
}
define('glTexCoord3iv', dlsym(RTLD_DEFAULT, 'glTexCoord3iv'), null, 'void', 'void *');
function glTexCoord3iv(v) {
  return call('glTexCoord3iv', v);
}
define('glTexCoord3s', dlsym(RTLD_DEFAULT, 'glTexCoord3s'), null, 'void', 'int16', 'int16', 'int16');
function glTexCoord3s(s, t, r) {
  return call('glTexCoord3s', s, t, r);
}
define('glTexCoord3sv', dlsym(RTLD_DEFAULT, 'glTexCoord3sv'), null, 'void', 'void *');
function glTexCoord3sv(v) {
  return call('glTexCoord3sv', v);
}
define('glTexCoord4d', dlsym(RTLD_DEFAULT, 'glTexCoord4d'), null, 'void', 'long', 'long', 'long', 'long');
function glTexCoord4d(s, t, r, q) {
  return call('glTexCoord4d', s, t, r, q);
}
define('glTexCoord4dv', dlsym(RTLD_DEFAULT, 'glTexCoord4dv'), null, 'void', 'void *');
function glTexCoord4dv(v) {
  return call('glTexCoord4dv', v);
}
define('glTexCoord4f', dlsym(RTLD_DEFAULT, 'glTexCoord4f'), null, 'void', 'int', 'int', 'int', 'int');
function glTexCoord4f(s, t, r, q) {
  return call('glTexCoord4f', s, t, r, q);
}
define('glTexCoord4fv', dlsym(RTLD_DEFAULT, 'glTexCoord4fv'), null, 'void', 'void *');
function glTexCoord4fv(v) {
  return call('glTexCoord4fv', v);
}
define('glTexCoord4i', dlsym(RTLD_DEFAULT, 'glTexCoord4i'), null, 'void', 'int', 'int', 'int', 'int');
function glTexCoord4i(s, t, r, q) {
  return call('glTexCoord4i', s, t, r, q);
}
define('glTexCoord4iv', dlsym(RTLD_DEFAULT, 'glTexCoord4iv'), null, 'void', 'void *');
function glTexCoord4iv(v) {
  return call('glTexCoord4iv', v);
}
define('glTexCoord4s', dlsym(RTLD_DEFAULT, 'glTexCoord4s'), null, 'void', 'int16', 'int16', 'int16', 'int16');
function glTexCoord4s(s, t, r, q) {
  return call('glTexCoord4s', s, t, r, q);
}
define('glTexCoord4sv', dlsym(RTLD_DEFAULT, 'glTexCoord4sv'), null, 'void', 'void *');
function glTexCoord4sv(v) {
  return call('glTexCoord4sv', v);
}
define('glTexCoordPointer', dlsym(RTLD_DEFAULT, 'glTexCoordPointer'), null, 'void', 'int', 'int', 'int', 'void *');
function glTexCoordPointer(size, type, stride, ptr) {
  return call('glTexCoordPointer', size, type, stride, ptr);
}
define('glTexEnvf', dlsym(RTLD_DEFAULT, 'glTexEnvf'), null, 'void', 'int', 'int', 'int');
function glTexEnvf(target, pname, param) {
  return call('glTexEnvf', target, pname, param);
}
define('glTexEnvfv', dlsym(RTLD_DEFAULT, 'glTexEnvfv'), null, 'void', 'int', 'int', 'void *');
function glTexEnvfv(target, pname, params) {
  return call('glTexEnvfv', target, pname, params);
}
define('glTexEnvi', dlsym(RTLD_DEFAULT, 'glTexEnvi'), null, 'void', 'int', 'int', 'int');
function glTexEnvi(target, pname, param) {
  return call('glTexEnvi', target, pname, param);
}
define('glTexEnviv', dlsym(RTLD_DEFAULT, 'glTexEnviv'), null, 'void', 'int', 'int', 'void *');
function glTexEnviv(target, pname, params) {
  return call('glTexEnviv', target, pname, params);
}
define('glTexGend', dlsym(RTLD_DEFAULT, 'glTexGend'), null, 'void', 'int', 'int', 'long');
function glTexGend(coord, pname, param) {
  return call('glTexGend', coord, pname, param);
}
define('glTexGendv', dlsym(RTLD_DEFAULT, 'glTexGendv'), null, 'void', 'int', 'int', 'void *');
function glTexGendv(coord, pname, params) {
  return call('glTexGendv', coord, pname, params);
}
define('glTexGenf', dlsym(RTLD_DEFAULT, 'glTexGenf'), null, 'void', 'int', 'int', 'int');
function glTexGenf(coord, pname, param) {
  return call('glTexGenf', coord, pname, param);
}
define('glTexGenfv', dlsym(RTLD_DEFAULT, 'glTexGenfv'), null, 'void', 'int', 'int', 'void *');
function glTexGenfv(coord, pname, params) {
  return call('glTexGenfv', coord, pname, params);
}
define('glTexGeni', dlsym(RTLD_DEFAULT, 'glTexGeni'), null, 'void', 'int', 'int', 'int');
function glTexGeni(coord, pname, param) {
  return call('glTexGeni', coord, pname, param);
}
define('glTexGeniv', dlsym(RTLD_DEFAULT, 'glTexGeniv'), null, 'void', 'int', 'int', 'void *');
function glTexGeniv(coord, pname, params) {
  return call('glTexGeniv', coord, pname, params);
}
define('glTexImage1D', dlsym(RTLD_DEFAULT, 'glTexImage1D'), null, 'void', 'int', 'int', 'int', 'int', 'int', 'int', 'int', 'void *');
function glTexImage1D(target, level, internalFormat, width, border, format, type, pixels) {
  return call('glTexImage1D', target, level, internalFormat, width, border, format, type, pixels);
}
define('glTexImage2D', dlsym(RTLD_DEFAULT, 'glTexImage2D'), null, 'void', 'int', 'int', 'int', 'int', 'int', 'int', 'int', 'int', 'void *');
function glTexImage2D(target, level, internalFormat, width, height, border, format, type, pixels) {
  return call('glTexImage2D', target, level, internalFormat, width, height, border, format, type, pixels);
}
define('glTexImage3D', dlsym(RTLD_DEFAULT, 'glTexImage3D'), null, 'void', 'int', 'int', 'int', 'int', 'int', 'int', 'int', 'int', 'int', 'void *');
function glTexImage3D(target, level, internalFormat, width, height, depth, border, format, type, pixels) {
  return call('glTexImage3D', target, level, internalFormat, width, height, depth, border, format, type, pixels);
}
define('glTexParameterf', dlsym(RTLD_DEFAULT, 'glTexParameterf'), null, 'void', 'int', 'int', 'int');
function glTexParameterf(target, pname, param) {
  return call('glTexParameterf', target, pname, param);
}
define('glTexParameterfv', dlsym(RTLD_DEFAULT, 'glTexParameterfv'), null, 'void', 'int', 'int', 'void *');
function glTexParameterfv(target, pname, params) {
  return call('glTexParameterfv', target, pname, params);
}
define('glTexParameteri', dlsym(RTLD_DEFAULT, 'glTexParameteri'), null, 'void', 'int', 'int', 'int');
function glTexParameteri(target, pname, param) {
  return call('glTexParameteri', target, pname, param);
}
define('glTexParameteriv', dlsym(RTLD_DEFAULT, 'glTexParameteriv'), null, 'void', 'int', 'int', 'void *');
function glTexParameteriv(target, pname, params) {
  return call('glTexParameteriv', target, pname, params);
}
define('glTexSubImage1D', dlsym(RTLD_DEFAULT, 'glTexSubImage1D'), null, 'void', 'int', 'int', 'int', 'int', 'int', 'int', 'void *');
function glTexSubImage1D(target, level, xoffset, width, format, type, pixels) {
  return call('glTexSubImage1D', target, level, xoffset, width, format, type, pixels);
}
define('glTexSubImage2D', dlsym(RTLD_DEFAULT, 'glTexSubImage2D'), null, 'void', 'int', 'int', 'int', 'int', 'int', 'int', 'int', 'int', 'void *');
function glTexSubImage2D(target, level, xoffset, yoffset, width, height, format, type, pixels) {
  return call('glTexSubImage2D', target, level, xoffset, yoffset, width, height, format, type, pixels);
}
define('glTexSubImage3D', dlsym(RTLD_DEFAULT, 'glTexSubImage3D'), null, 'void', 'int', 'int', 'int', 'int', 'int', 'int', 'int', 'int', 'int', 'int', 'void *');
function glTexSubImage3D(target, level, xoffset, yoffset, zoffset, width, height, depth, format, type, pixels) {
  return call('glTexSubImage3D', target, level, xoffset, yoffset, zoffset, width, height, depth, format, type, pixels);
}
define('glTranslated', dlsym(RTLD_DEFAULT, 'glTranslated'), null, 'void', 'long', 'long', 'long');
function glTranslated(x, y, z) {
  return call('glTranslated', x, y, z);
}
define('glTranslatef', dlsym(RTLD_DEFAULT, 'glTranslatef'), null, 'void', 'int', 'int', 'int');
function glTranslatef(x, y, z) {
  return call('glTranslatef', x, y, z);
}
define('glVertex2d', dlsym(RTLD_DEFAULT, 'glVertex2d'), null, 'void', 'long', 'long');
function glVertex2d(x, y) {
  return call('glVertex2d', x, y);
}
define('glVertex2dv', dlsym(RTLD_DEFAULT, 'glVertex2dv'), null, 'void', 'void *');
function glVertex2dv(v) {
  return call('glVertex2dv', v);
}
define('glVertex2f', dlsym(RTLD_DEFAULT, 'glVertex2f'), null, 'void', 'int', 'int');
function glVertex2f(x, y) {
  return call('glVertex2f', x, y);
}
define('glVertex2fv', dlsym(RTLD_DEFAULT, 'glVertex2fv'), null, 'void', 'void *');
function glVertex2fv(v) {
  return call('glVertex2fv', v);
}
define('glVertex2i', dlsym(RTLD_DEFAULT, 'glVertex2i'), null, 'void', 'int', 'int');
function glVertex2i(x, y) {
  return call('glVertex2i', x, y);
}
define('glVertex2iv', dlsym(RTLD_DEFAULT, 'glVertex2iv'), null, 'void', 'void *');
function glVertex2iv(v) {
  return call('glVertex2iv', v);
}
define('glVertex2s', dlsym(RTLD_DEFAULT, 'glVertex2s'), null, 'void', 'int16', 'int16');
function glVertex2s(x, y) {
  return call('glVertex2s', x, y);
}
define('glVertex2sv', dlsym(RTLD_DEFAULT, 'glVertex2sv'), null, 'void', 'void *');
function glVertex2sv(v) {
  return call('glVertex2sv', v);
}
define('glVertex3d', dlsym(RTLD_DEFAULT, 'glVertex3d'), null, 'void', 'long', 'long', 'long');
function glVertex3d(x, y, z) {
  return call('glVertex3d', x, y, z);
}
define('glVertex3dv', dlsym(RTLD_DEFAULT, 'glVertex3dv'), null, 'void', 'void *');
function glVertex3dv(v) {
  return call('glVertex3dv', v);
}
define('glVertex3f', dlsym(RTLD_DEFAULT, 'glVertex3f'), null, 'void', 'int', 'int', 'int');
function glVertex3f(x, y, z) {
  return call('glVertex3f', x, y, z);
}
define('glVertex3fv', dlsym(RTLD_DEFAULT, 'glVertex3fv'), null, 'void', 'void *');
function glVertex3fv(v) {
  return call('glVertex3fv', v);
}
define('glVertex3i', dlsym(RTLD_DEFAULT, 'glVertex3i'), null, 'void', 'int', 'int', 'int');
function glVertex3i(x, y, z) {
  return call('glVertex3i', x, y, z);
}
define('glVertex3iv', dlsym(RTLD_DEFAULT, 'glVertex3iv'), null, 'void', 'void *');
function glVertex3iv(v) {
  return call('glVertex3iv', v);
}
define('glVertex3s', dlsym(RTLD_DEFAULT, 'glVertex3s'), null, 'void', 'int16', 'int16', 'int16');
function glVertex3s(x, y, z) {
  return call('glVertex3s', x, y, z);
}
define('glVertex3sv', dlsym(RTLD_DEFAULT, 'glVertex3sv'), null, 'void', 'void *');
function glVertex3sv(v) {
  return call('glVertex3sv', v);
}
define('glVertex4d', dlsym(RTLD_DEFAULT, 'glVertex4d'), null, 'void', 'long', 'long', 'long', 'long');
function glVertex4d(x, y, z, w) {
  return call('glVertex4d', x, y, z, w);
}
define('glVertex4dv', dlsym(RTLD_DEFAULT, 'glVertex4dv'), null, 'void', 'void *');
function glVertex4dv(v) {
  return call('glVertex4dv', v);
}
define('glVertex4f', dlsym(RTLD_DEFAULT, 'glVertex4f'), null, 'void', 'int', 'int', 'int', 'int');
function glVertex4f(x, y, z, w) {
  return call('glVertex4f', x, y, z, w);
}
define('glVertex4fv', dlsym(RTLD_DEFAULT, 'glVertex4fv'), null, 'void', 'void *');
function glVertex4fv(v) {
  return call('glVertex4fv', v);
}
define('glVertex4i', dlsym(RTLD_DEFAULT, 'glVertex4i'), null, 'void', 'int', 'int', 'int', 'int');
function glVertex4i(x, y, z, w) {
  return call('glVertex4i', x, y, z, w);
}
define('glVertex4iv', dlsym(RTLD_DEFAULT, 'glVertex4iv'), null, 'void', 'void *');
function glVertex4iv(v) {
  return call('glVertex4iv', v);
}
define('glVertex4s', dlsym(RTLD_DEFAULT, 'glVertex4s'), null, 'void', 'int16', 'int16', 'int16', 'int16');
function glVertex4s(x, y, z, w) {
  return call('glVertex4s', x, y, z, w);
}
define('glVertex4sv', dlsym(RTLD_DEFAULT, 'glVertex4sv'), null, 'void', 'void *');
function glVertex4sv(v) {
  return call('glVertex4sv', v);
}
define('glVertexPointer', dlsym(RTLD_DEFAULT, 'glVertexPointer'), null, 'void', 'int', 'int', 'int', 'void *');
function glVertexPointer(size, type, stride, ptr) {
  return call('glVertexPointer', size, type, stride, ptr);
}
define('glViewport', dlsym(RTLD_DEFAULT, 'glViewport'), null, 'void', 'int', 'int', 'int', 'int');
function glViewport(x, y, width, height) {
  return call('glViewport', x, y, width, height);
}