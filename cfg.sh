cfg() {
  if type gcc 2>/dev/null >/dev/null && type g++ 2>/dev/null >/dev/null; then
    : ${CC:=gcc} ${CXX:=g++}
  elif type clang 2>/dev/null >/dev/null && type clang++ 2>/dev/null >/dev/null; then
    : ${CC:=clang} ${CXX:=clang++}
  fi

  : ${build:=`$CC -dumpmachine | sed 's|-pc-|-|g'`}

  if [ -z "$host" -a -z "$builddir" ]; then
    host=$build
    case "$host" in
      x86_64-w64-mingw32) host="$host"; : ${builddir=build/$host}; : ${prefix=/mingw64} ;;
      i686-w64-mingw32) host="$host"; : ${builddir=build/$host}; : ${prefix=/mingw32} ;;
      x86_64-pc-*) host="$host"; : ${builddir=build/$host}; : ${prefix=/usr} ;;
      i686-pc-*) host="$host"; : ${builddir=build/$host}; : ${prefix=/usr} ;;
    esac
  fi

  : ${prefix:=/usr/local}
  : ${libdir:=$prefix/lib}
  [ -d "$libdir/$host" ] && libdir=$libdir/$host

  if [ -e "$TOOLCHAIN" ]; then
    cmakebuild=$(basename "$TOOLCHAIN" .cmake)
    cmakebuild=${cmakebuild%.toolchain}
    cmakebuild=${cmakebuild#toolchain-}
    : ${builddir=build/$cmakebuild}
  else
   : ${builddir=build/$host}
  fi
  case "$host" in
    *msys*) ;;
    *) test -n "$builddir" && builddir=`echo $builddir | sed 's|-pc-|-|g'` ;;
  esac
  
  case $(uname -o) in
   # MSys|MSYS|Msys) SYSTEM="MSYS" ;;
    *) SYSTEM="Unix" ;;
  esac

  case "$STATIC:$TYPE" in
    YES:*|yes:*|y:*|1:*|ON:*|on:* | *:*[Ss]tatic*) set -- "$@" \
      -DBUILD_SHARED_LIBS=OFF \
      -DENABLE_PIC=OFF ;;
  esac

  [ -n "$PKG_CONFIG_PATH" ] && echo "PKG_CONFIG_PATH=$PKG_CONFIG_PATH" 1>&2
  [ -n "$PKG_CONFIG" ] && case "$PKG_CONFIG" in
     */*) ;;
     *) PKG_CONFIG=$(which "$PKG_CONFIG") ;; 
  esac
  : ${generator:="CodeLite - Unix Makefiles"}

 (mkdir -p $builddir
  : ${relsrcdir=`realpath --relative-to "$builddir" .`}
  : set -x
  cd "${builddir:-.}"
  IFS="$IFS "
 set -- -Wno-dev \
    -G "$generator" \
    ${prefix:+-DCMAKE_INSTALL_PREFIX="$prefix"} \
    ${VERBOSE:+-DCMAKE_VERBOSE_MAKEFILE=${VERBOSE:-OFF}} \
    -DCMAKE_BUILD_TYPE="${TYPE:-Debug}" \
    -DBUILD_SHARED_LIBS=ON \
    ${CC:+-DCMAKE_C_COMPILER="$CC"} \
    ${CXX:+-DCMAKE_CXX_COMPILER="$CXX"} \
    ${PKG_CONFIG:+-DPKG_CONFIG_EXECUTABLE="$PKG_CONFIG"} \
    ${TOOLCHAIN:+-DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN"} \
    ${CC:+-DCMAKE_C_COMPILER="$CC"} \
    ${CXX:+-DCMAKE_CXX_COMPILER="$CXX"} \
    ${MAKE:+-DCMAKE_MAKE_PROGRAM="$MAKE"} \
    "$@" \
    $relsrcdir 
  eval "${CMAKE:-cmake} \"\$@\""
 ) 2>&1 |tee "${builddir##*/}.log"
}


cfg-android() {
  (
    : ${builddir=build/android}
    cfg \
      -DCMAKE_INSTALL_PREFIX=/opt/arm-linux-androideabi/sysroot/usr \
      -DCMAKE_VERBOSE_MAKEFILE=TRUE \
      -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN:-/opt/android-cmake/android.cmake} \
      -DANDROID_NATIVE_API_LEVEL=21 \
      -DPKG_CONFIG_EXECUTABLE=arm-linux-androideabi-pkg-config \
      -DCMAKE_PREFIX_PATH=/opt/arm-linux-androideabi/sysroot/usr \
      -DCMAKE_MAKE_PROGRAM=/usr/bin/make \
      -DCMAKE_MODULE_PATH="/opt/OpenCV-3.4.1-android-sdk/sdk/native/jni/abi-armeabi-v7a" \
      -DOpenCV_DIR="/opt/OpenCV-3.4.1-android-sdk/sdk/native/jni/abi-armeabi-v7a" \
      "$@"
  )
}

cfg-diet() {
  (
    : ${build=$(${CC:-gcc} -dumpmachine | sed 's|-pc-|-|g')}
    : ${host=${build/-gnu/-diet}}
    : ${prefix=/opt/diet}
    : ${libdir=/opt/diet/lib-${host%%-*}}
    : ${bindir=/opt/diet/bin-${host%%-*}}

    : ${CC="diet-gcc"}
    export CC

    if type pkgconf >/dev/null; then
      export PKG_CONFIG=pkgconf
    fi

    : ${PKG_CONFIG_PATH="$libdir/pkgconfig"}
    export PKG_CONFIG_PATH

    builddir=build/${host%-*}-diet \
      cfg \
      -DCMAKE_INSTALL_PREFIX="$prefix" \
      -DSHARED_LIBS=OFF \
      -DCMAKE_VERBOSE_MAKEFILE=ON \
      -DCMAKE_FIND_ROOT_PATH="$prefix" \
      -DCMAKE_SYSTEM_LIBRARY_PATH="$prefix/lib-${host%%-*}" \
      -D{CMAKE_INSTALL_LIBDIR=,INSTALL_LIB_DIR=$prefix/}"lib-${host%%-*}" \
      ${launcher:+-DCMAKE_C_COMPILER_LAUNCHER="$launcher"} \
      "$@"
  )
}

cfg-diet64() {
  (
    build=$(gcc -dumpmachine | sed 's|-pc-|-|g')
    host=${build%%-*}-linux-diet
    host=x86_64-${host#*-}

    export prefix=/opt/diet

    builddir=build/$host \
      CC="diet-gcc" \
      cfg-diet \
      "$@"
  )
}

cfg-diet32() {
  (
    build=$(gcc -dumpmachine | sed 's|-pc-|-|g')
    host=${build%%-*}-linux-diet
    host=i686-${host#*-}

    if type diet32-clang 2>/dev/null >/dev/null; then
      CC="diet32-clang"
      export CC
    elif type diet32-gcc 2>/dev/null >/dev/null; then
      CC="diet32-gcc"
      export CC
    else
      CC="gcc"
      launcher="/opt/diet/bin-i386/diet"
      CFLAGS="-m32"
      export CC launcher CFLAGS
    fi

    builddir=build/$host \
      cfg-diet "$@"
  )
}

cfg-mingw() {
  (
    build=$(gcc -dumpmachine | sed 's|-pc-|-|g')
    : ${host=${build%%-*}-w64-mingw32}
    : ${prefix=/usr/$host/sys-root/mingw}

    case "$host" in
    x86_64-*) : ${TOOLCHAIN=/opt/cmake-toolchains/mingw64.cmake} ;;
    *) : ${TOOLCHAIN=/opt/cmake-toolchains/mingw32.cmake} ;;
    esac

    : ${PKG_CONFIG_PATH=/usr/${host}/sys-root/mingw/lib/pkgconfig}

    export TOOLCHAIN PKG_CONFIG_PATH

    VERBOSE=TRUE \
      builddir=build/$host \
      bindir=$prefix/bin \
      libdir=$prefix/lib \
      cfg \
      "$@"
  )
}

cfg-emscripten() {
  (
    build=$(cc -dumpmachine | sed 's|-pc-|-|g')
    host=$(emcc -dumpmachine)
    builddir=build/${host%-*}-emscripten

    : ${prefix=$EMSCRIPTEN/system}
    : ${libdir=$prefix/lib}
    : ${bindir=$prefix/bin}

    CC="emcc" \
      CXX="em++" \
      PKG_CONFIG_PATH="$EMSCRIPTEN/system/lib/pkgconfig" \
      cfg \
      -DENABLE_PIC=FALSE \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_TOOLCHAIN_FILE="$EMSCRIPTEN/cmake/Modules/Platform/Emscripten.cmake" \
      -DCPU_BASELINE='' \
      -DCPU_DISPATCH='' \
      -DCV_TRACE=OFF \
      -DBUILD_SHARED_LIBS=OFF \
      -DWITH_1394=OFF \
      -DWITH_ADE=OFF \
      -DWITH_VTK=OFF \
      -DWITH_EIGEN=OFF \
      -DWITH_FFMPEG=OFF \
      -DWITH_GSTREAMER=OFF \
      -DWITH_GTK=OFF \
      -DWITH_GTK_2_X=OFF \
      -DWITH_IPP=OFF \
      -DWITH_JASPER=OFF \
      -DWITH_JPEG=OFF \
      -DWITH_WEBP=OFF \
      -DWITH_OPENEXR=OFF \
      -DWITH_OPENGL=OFF \
      -DWITH_OPENVX=OFF \
      -DWITH_OPENNI=OFF \
      -DWITH_OPENNI2=OFF \
      -DWITH_PNG=OFF \
      -DWITH_TBB=OFF \
      -DWITH_TIFF=OFF \
      -DWITH_V4L=OFF \
      -DWITH_OPENCL=OFF \
      -DWITH_OPENCL_SVM=OFF \
      -DWITH_OPENCLAMDFFT=OFF \
      -DWITH_OPENCLAMDBLAS=OFF \
      -DWITH_GPHOTO2=OFF \
      -DWITH_LAPACK=OFF \
      -DWITH_ITT=OFF \
      -DWITH_QUIRC=OFF \
      -DBUILD_ZLIB=ON \
      -DBUILD_opencv_apps=OFF \
      -DBUILD_opencv_calib3d=ON \
      -DBUILD_opencv_dnn=ON \
      -DBUILD_opencv_features2d=ON \
      -DBUILD_opencv_flann=ON \
      -DBUILD_opencv_gapi=OFF \
      -DBUILD_opencv_ml=OFF \
      -DBUILD_opencv_photo=ON \
      -DBUILD_opencv_imgcodecs=OFF \
      -DBUILD_opencv_shape=OFF \
      -DBUILD_opencv_videoio=OFF \
      -DBUILD_opencv_videostab=OFF \
      -DBUILD_opencv_highgui=OFF \
      -DBUILD_opencv_superres=OFF \
      -DBUILD_opencv_stitching=OFF \
      -DBUILD_opencv_java=OFF \
      -DBUILD_opencv_java_bindings_generator=OFF \
      -DBUILD_opencv_js=ON \
      -DBUILD_opencv_python2=OFF \
      -DBUILD_opencv_python3=OFF \
      -DBUILD_opencv_python_bindings_generator=OFF \
      -DBUILD_EXAMPLES=OFF \
      -DBUILD_PACKAGE=OFF \
      -DBUILD_TESTS=OFF \
      -DBUILD_PERF_TESTS=OFF \
      -DBUILD_DOCS=OFF \
      -DWITH_PTHREADS_PF=OFF \
      -DCV_ENABLE_INTRINSICS=OFF \
      -DBUILD_WASM_INTRIN_TESTS=OFF \
      "-DCMAKE_C_FLAGS='-s WASM=1 -s USE_PTHREADS=0 -s LLD_REPORT_UNDEFINED'" \
      "-DCMAKE_CXX_FLAGS='-s WASM=1 -s USE_PTHREADS=0 -s LLD_REPORT_UNDEFINED'" \
      "$@"
  )
}

cfg-tcc() {
  (
    build=$(cc -dumpmachine | sed 's|-pc-|-|g')
    host=${build/-gnu/-tcc}
    builddir=build/$host
    prefix=/usr
    includedir=/usr/lib/$build/tcc/include
    libdir=/usr/lib/$build/tcc/
    bindir=/usr/bin

    CC=${TCC:-tcc} \
      cfg \
      -DCMAKE_VERBOSE_MAKEFILE=ON \
      "$@"
  )
}

cfg-musl() {
  (
    : ${build=$(${CC:-gcc} -dumpmachine | sed 's|-pc-|-|g')}
    : ${host=${build%-*}-musl}

    : ${prefix=/usr}
    : ${includedir=/usr/include/$host}
    : ${libdir=/usr/lib/$host}
    : ${bindir=/usr/bin/$host}

    : ${builddir=build/$host}

    CC=musl-gcc \
      PKG_CONFIG=musl-pkg-config \
      cfg \
      -DSHARED_LIBS=OFF \
      -DCMAKE_VERBOSE_MAKEFILE=ON \
      "$@"
  )
}

cfg-musl64() {
  (
    build=$(gcc -dumpmachine | sed 's|-pc-|-|g')
    host=${build%%-*}-linux-musl
    host=x86_64-${host#*-}

    builddir=build/$host \
      CFLAGS="-m64" \
      cfg-musl \
      -DCMAKE_C_COMPILER="musl-gcc" \
      "$@"
  )
}

cfg-musl32() {
  (
    build=$(gcc -dumpmachine | sed 's|-pc-|-|g')
    host=$(echo "$build" | sed "s|x86_64|i686| ; s|-gnu|-musl|")

    builddir=build/$host \
      CFLAGS="-m32" \
      cfg-musl \
      -DCMAKE_C_COMPILER="musl-gcc" \
      "$@"
  )
}

cfg-msys() {
  (
    build=$(gcc -dumpmachine | sed 's|-pc-|-|g')
    : ${host=${build%%-*}-pc-msys}
    : ${prefix=/usr/$host/sysroot/usr}

    builddir=build/$host \
      bindir=$prefix/bin \
      libdir=$prefix/lib \
      CC="$host-gcc" \
      cfg \
      -DCMAKE_CROSSCOMPILING=TRUE \
      "$@"
  )
}

cfg-msys32() {
  (
    build=$(gcc -dumpmachine | sed 's|-pc-|-|g')
    host=${build%%-*}-pc-msys
    host=i686-${host#*-}
    cfg-msys "$@"
  )
}

cfg-termux() {
  (
    builddir=build/aarch64
    cfg \
      -DCMAKE_INSTALL_PREFIX=/data/data/com.termux/files/usr \
      -DCMAKE_VERBOSE_MAKEFILE=TRUE \
      -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN:-/opt/android-cmake/android.cmake} \
      -DANDROID_NATIVE_API_LEVEL=21 \
      -DPKG_CONFIG_EXECUTABLE=arm-linux-androideabi-pkg-config \
      -DCMAKE_PREFIX_PATH=/data/data/com.termux/files/usr \
      -DCMAKE_MAKE_PROGRAM=/usr/bin/make \
      -DCMAKE_MODULE_PATH="/data/data/com.termux/files/usr/lib/cmake" \
      "$@"
  )
}
cfg-wasm() {
  export VERBOSE
  (
    EMCC=$(which emcc)
    EMSCRIPTEN=$(dirname "$EMCC")
    EMSCRIPTEN=${EMSCRIPTEN%%/bin*}
    test -f /opt/cmake-toolchains/generic/Emscripten-wasm.cmake && TOOLCHAIN=/opt/cmake-toolchains/generic/Emscripten-wasm.cmake
    test '!' -f "$TOOLCHAIN" && TOOLCHAIN=$(find "$EMSCRIPTEN" -iname emscripten.cmake)
    test -f "$TOOLCHAIN" || unset TOOLCHAIN
    : ${prefix:="$EMSCRIPTEN"}
    builddir=build/emscripten-wasm \
      CC="$EMCC" \
      cfg \
      -DEMSCRIPTEN_PREFIX="$EMSCRIPTEN" \
      ${TOOLCHAIN:+-DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN"} \
      -DCMAKE_EXE_LINKER_FLAGS="-s WASM=1" \
      -DCMAKE_EXECUTABLE_SUFFIX=".html" \
      -DCMAKE_EXECUTABLE_SUFFIX_INIT=".html" \
      -DUSE_{ZLIB,BZIP,LZMA,SSL}=OFF \
      "$@"
  )
}

cfg-tcc() {
  (
    build=$(cc -dumpmachine | sed 's|-pc-|-|g')
    host=${build/-gnu/-tcc}
    builddir=build/$host
    prefix=/usr
    includedir=/usr/lib/$build/tcc/include
    libdir=/usr/lib/$build/tcc/
    bindir=/usr/bin

    CC=${TCC:-tcc} \
      cfg \
      -DCMAKE_VERBOSE_MAKEFILE=ON \
      "$@"
  )
}

cfg-emscripten() {
  (
    build=$(cc -dumpmachine | sed 's|-pc-|-|g')
    host=$(emcc -dumpmachine)
    : ${builddir=build/${host%-*}-emscripten}
    : ${prefix=$EMSCRIPTEN/system}
    : ${libdir=$prefix/lib}
    : ${bindir=$prefix/bin}
    : ${EMSCRIPTEN=$EMSDK/upstream/emscripten}
    export TOOLCHAIN="${EMSCRIPTEN}/cmake/Modules/Platform/Emscripten.cmake"
    PKG_CONFIG_PATH="$(
      set -- /opt/*-wasm/lib/pkgconfig
      IFS=":"
      echo "$*"
    )${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
    PKG_CONFIG_PATH="${PKG_CONFIG_PATH:+$PKG_CONFIG_PATH:}${EMSCRIPTEN}/system/lib/pkgconfig"
    export PKG_CONFIG_PATH
    echo PKG_CONFIG_PATH="${PKG_CONFIG_PATH}"
    CC="emcc" CXX="em++" TYPE="Release" VERBOSE="TRUE" \
      CFLAGS="'-sWASM=1 -sUSE_PTHREADS=0 -sLLD_REPORT_UNDEFINED'" \
      CXXFLAGS="'-sWASM=1 -sUSE_PTHREADS=0 -sLLD_REPORT_UNDEFINED'" \
      CMAKE_WRAPPER="emcmake" \
      prefix=/opt/${PWD##*/}-wasm \
      cfg \
      -DENABLE_PIC=FALSE \
      "$@"
  )
}

cfg-aarch64() {
 (: ${build=$(cc -dumpmachine | sed 's|-pc-|-|g')}
  : ${host=aarch64-${build#*-}}
  : ${builddir=build/$host}

  : ${prefix=/usr/aarch64-linux-gnu/sysroot/usr}

  : ${TOOLCHAIN=/opt/cmake-toolchains/aarch64-linux-gnu.toolchain.cmake}
  export prefix TOOLCHAIN

  PKG_CONFIG=$(which ${host}-pkg-config) \
  cfg "$@")
}