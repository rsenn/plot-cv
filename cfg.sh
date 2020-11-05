 
cfg() {
  : ${build:=`gcc -dumpmachine`}
  : ${VERBOSE:=OFF}
  : ${PKG_CONFIG_PATH="/opt/opencv4/lib/pkgconfig:/usr/lib/pkgconfig:/usr/lib/x86_64-linux-gnu/pkgconfig:/usr/share/pkgconfig"}

  export PKG_CONFIG_PATH

  if [ -n "$TOOLCHAIN" ]; then
    toolchain=`basename "$TOOLCHAIN" .cmake`
    compilers=$(grep -i compiler $TOOLCHAIN |sed 's,.*compiler\s*(\([^ ]*\)[^)]*).*,\1,p' -n)
    sysroot=$(sed -n '/SYSROOT/ { \|^\s*/| {   s,^\s*,,; p; q } }' $TOOLCHAIN; sed -n '\|^\s*/| {   s,^\s*,,; p; q }' $TOOLCHAIN)
    if [ -n "$sysroot" -a -d "$sysroot" ]; then
      SYSROOT="$sysroot"
      : ${prefix:="$sysroot"}
      echo "SYSROOT $SYSROOT" 1>&2
    fi
    for c in $compilers; do 
      h=$("$c" -dumpmachine )
      if [ -z "$host" ]; then
        [ -n "$h" ] && host=$h
        [ -n "$h" ] && CC="$c"
     fi
    done
    if [ -e "$TOOLCHAIN" ]; then
      cmakebuild=$(basename "$TOOLCHAIN" .cmake)
      cmakebuild=${cmakebuild%.toolchain}
      cmakebuild=${cmakebuild#toolchain-}
    fi
  fi

 (if [ -z "$host" ]; then
    host=$build
    case "$host" in
      *musl*) host="$host" prefix=/opt/musl ;;
      *diet*) host="$host" prefix=/opt/diet ;;
      x86_64-w64-mingw32) host="$host" prefix=/mingw64 ;;
      i686-w64-mingw32) host="$host" prefix=/mingw32 ;;
      x86_64-pc-*) host="$host" prefix=/usr ;;
      i686-pc-*) host="$host" prefix=/usr ;;
    esac
  fi
  : ${prefix:=/usr/local}
  : ${libdir:=$prefix/lib}
  [ -d "$libdir/$host" ] && libdir=$libdir/$host

  case $(uname -o) in
   # MSys|MSYS|Msys) SYSTEM="MSYS" ;;
    *) SYSTEM="Unix" ;;
  esac

  case "$STATIC:$TYPE" in
    YES:*|yes:*|y:*|1:*|ON:*|on:* | *:*[Ss]tatic*) set -- "$@" \
      -DENABLE_PIC=OFF ;;
  esac
  if [ -z "$generator" ]; then
#    if type ninja 2>/dev/null; then
#      builddir=$builddir-ninja
#      generator="Ninja"
#    else
      generator="Unix Makefiles"
#    fi
  fi
  case "$generator" in
    *" - "*) break ;;
    *)
  if type codelite 2>/dev/null; then
    generator="Sublime Text 2 - $generator"
  fi
  ;;
  esac

  if [ -n "$host" ]; then
    case "$host" in
      x86_64*) machine=x64 ;;
      i[3-6]86*) machine=x86 ;;
      aarch64*) machine=arm64 ;;
      arm-*) machine=arm ;;
      *) machine=${host%%-*} ;;
    esac
    case "$host" in
      *-linux-*) os=linux ;;
      *-mingw32*) os=mingw32 ;;
      *-msys*) os=msys ;;
      *) os=${host#*-}; os=${os%%-*} ;;
    esac
  fi
    case "$SHARED" in
      TRUE|ON|YES|1) link=shared ;;
      *) link=static SHARED=OFF ;;
    esac
    : ${TYPE:=RelWithDebInfo}
    case "$TYPE" in
      *Debug*) type=debug suffix=debug symbols=debug ;;
      *Deb*) type=relwithdebinfo suffix=relwithdeb symbols=debug ;;
      *MinSize*) type=minsizerel suffix=minsize symbols= ;;
      *) type=release suffix=release symbols= ;;
    esac
    case "$suffix" in
      relwithdeb*) unset suffix ;;
    esac
    if [ -z "$CC" ]; then
      for compiler in gcc clang cc; do
        if type $compiler >/dev/null 2>/dev/null; then
          CC="$compiler"
          break
        fi
      done
    fi
    : ${builddir:=build/$os-$machine-$link${suffix:+-$suffix}}
 (mkdir -p $builddir
  : ${relsrcdir=`realpath --relative-to "$builddir" .`}
  set -x
  cd $builddir
  ${CMAKE_WRAPPER} ${CMAKE:-cmake} -Wno-dev \
    -G "$generator" \
    ${VERBOSE:+-DCMAKE_VERBOSE_MAKEFILE=$VERBOSE} \
    -DCMAKE_BUILD_TYPE="${TYPE}" \
    ${CC:+-DCMAKE_C_COMPILER="$CC"} \
    ${CXX:+-DCMAKE_CXX_COMPILER="$CXX"} \
    ${PKG_CONFIG:+-DPKG_CONFIG_EXECUTABLE="$PKG_CONFIG"} \
    ${TOOLCHAIN:+-DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN"} \
    ${CC:+-DCMAKE_C_COMPILER="$CC"} \
    ${CXX:+-DCMAKE_CXX_COMPILER="$CXX"} \
    ${MAKE:+-DCMAKE_MAKE_PROGRAM="$MAKE"} \
    ${prefix:+-DCMAKE_INSTALL_PREFIX="$prefix"} \
    "$@" \
    $relsrcdir 2>&1 ) |tee "${builddir##*/}.log")
}

cfg-android ()
{
  (: ${builddir=build/android}
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
 (: ${build=$(${CC:-gcc} -dumpmachine | sed 's|-pc-|-|g')}
  : ${host=${build/-gnu/-diet}}
  : ${prefix=/opt/diet}
  : ${libdir=/opt/diet/lib-${host%%-*}}
  : ${bindir=/opt/diet/bin-${host%%-*}}

  : ${CC="diet-gcc"}
  export CC

  if type pkgconf >/dev/null; then
    export PKG_CONFIG=pkgconf
  fi

  : ${PKG_CONFIG_PATH="$libdir/pkgconfig"}; export PKG_CONFIG_PATH
  
  builddir=build/${host%-*}-diet \
  cfg \
    -DCMAKE_INSTALL_PREFIX="$prefix" \
    -DSHARED_LIBS=OFF \
    -DCMAKE_VERBOSE_MAKEFILE=ON \
    -DCMAKE_FIND_ROOT_PATH="$prefix" \
    -DCMAKE_SYSTEM_LIBRARY_PATH="$prefix/lib-${host%%-*}" \
    -D{CMAKE_INSTALL_LIBDIR=,INSTALL_LIB_DIR=$prefix/}"lib-${host%%-*}" \
      ${launcher:+-DCMAKE_C_COMPILER_LAUNCHER="$launcher"} \
    "$@")
}

cfg-diet64() {
 (build=$(gcc -dumpmachine | sed 's|-pc-|-|g')
  host=${build%%-*}-linux-diet
  host=x86_64-${host#*-}

  export prefix=/opt/diet

  builddir=build/$host \
  CC="diet-gcc" \
  cfg-diet \
  "$@")
}

cfg-diet32() {
 (build=$(gcc -dumpmachine | sed 's|-pc-|-|g')
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
  cfg-diet  "$@")
}

cfg-mingw() {
 (build=$(gcc -dumpmachine | sed 's|-pc-|-|g')
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
    "$@")
}

cfg-emscripten() {
 (build=$(cc -dumpmachine | sed 's|-pc-|-|g')
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
    "$@")
}

cfg-tcc() {
 (build=$(cc -dumpmachine | sed 's|-pc-|-|g')
  host=${build/-gnu/-tcc}
  builddir=build/$host
  prefix=/usr
  includedir=/usr/lib/$build/tcc/include
  libdir=/usr/lib/$build/tcc/
  bindir=/usr/bin

  CC=${TCC:-tcc} \
  cfg \
    -DCMAKE_VERBOSE_MAKEFILE=ON \
    "$@")
}

cfg-musl() {
 (: ${build=$(${CC:-gcc} -dumpmachine | sed 's|-pc-|-|g')}
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
    "$@")
}


cfg-musl64() {
 (build=$(gcc -dumpmachine | sed 's|-pc-|-|g')
  host=${build%%-*}-linux-musl
  host=x86_64-${host#*-}

  builddir=build/$host \
  CFLAGS="-m64" \
  cfg-musl \
  -DCMAKE_C_COMPILER="musl-gcc" \
  "$@")
}

cfg-musl32() {
 (build=$(gcc -dumpmachine | sed 's|-pc-|-|g')
  host=$(echo "$build" | sed "s|x86_64|i686| ; s|-gnu|-musl|")

  builddir=build/$host \
  CFLAGS="-m32" \
  cfg-musl \
  -DCMAKE_C_COMPILER="musl-gcc" \
  "$@")
}

cfg-msys() {
 (build=$(gcc -dumpmachine | sed 's|-pc-|-|g')
  : ${host=${build%%-*}-pc-msys}
  : ${prefix=/usr/$host/sysroot/usr}

  builddir=build/$host \
  bindir=$prefix/bin \
  libdir=$prefix/lib \
  CC="$host-gcc" \
  cfg \
    -DCMAKE_CROSSCOMPILING=TRUE \
    "$@")
}

cfg-msys32() {
 (build=$(gcc -dumpmachine | sed 's|-pc-|-|g')
  host=${build%%-*}-pc-msys
  host=i686-${host#*-}
  cfg-msys "$@")
}

cfg-rpi4()
{
  (builddir=build/rpi4
  : ${host=aarch64-linux-gnu}
  : ${build=aarch64-linux-gnu}
  : ${CC=aarch64-linux-gnu-gcc}
  : ${CXX=aarch64-linux-gnu-g++-8}
  
  prefix=/usr/aarch64-linux-gnu/sysroot/usr
  CC=aarch64-linux-gnu-gcc-8 CXX=aarch64-linux-gnu-g++-8 \
    cfg \
  -DCMAKE_INSTALL_PREFIX=$prefix \
  -DCMAKE_VERBOSE_MAKEFILE=TRUE \
  -DANDROID_NATIVE_API_LEVEL=21 \
  -DPKG_CONFIG_EXECUTABLE=/usr/bin/aarch64-linux-gnu-pkg-config \
  -DCMAKE_PREFIX_PATH=$prefix \
  -DCMAKE_SYSROOT=${prefix%/usr} \
  -DCMAKE_MAKE_PROGRAM=/usr/bin/make \
   -DCMAKE_MODULE_PATH="$prefix/lib/cmake" \
    -DCMAKE_CROSSCOMPILING=True \
  -DLLVM_TARGET_ARCH=AArch64 \
  -DLLVM_DEFAULT_TARGET_TRIPLE=aarch64-linux-gnueabihf \
  -DCMAKE_CXX_FLAGS='-march=armv8-a -mtune=cortex-a72' \
   "$@"
    )
}
cfg-termux()
{
  (builddir=build/rpi4
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
 (EMCC=$(which emcc)
  EMSCRIPTEN=$(dirname "$EMCC");
  EMSCRIPTEN=${EMSCRIPTEN%%/bin*};
  test -f /opt/cmake-toolchains/generic/Emscripten-wasm.cmake && TOOLCHAIN=/opt/cmake-toolchains/generic/Emscripten-wasm.cmake
  test '!' -f "$TOOLCHAIN" && TOOLCHAIN=$(find "$EMSCRIPTEN" -iname emscripten.cmake);
  test -f "$TOOLCHAIN" || unset TOOLCHAIN;
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
  "$@")
}
 
cfg-tcc() {
 (build=$(cc -dumpmachine | sed 's|-pc-|-|g')
  host=${build/-gnu/-tcc}
  builddir=build/$host
  prefix=/usr
  includedir=/usr/lib/$build/tcc/include
  libdir=/usr/lib/$build/tcc/
  bindir=/usr/bin

  CC=${TCC:-tcc} \
  cfg \
    -DCMAKE_VERBOSE_MAKEFILE=ON \
    "$@")
}

cfg-emscripten() {
  (build=$(cc -dumpmachine | sed 's|-pc-|-|g')
  host=$(emcc -dumpmachine)
  : ${builddir=build/${host%-*}-emscripten}
  : ${prefix=$EMSCRIPTEN/system}
  : ${libdir=$prefix/lib}
  : ${bindir=$prefix/bin}
  : ${EMSCRIPTEN=$EMSDK/upstream/emscripten}
  export TOOLCHAIN="${EMSCRIPTEN}/cmake/Modules/Platform/Emscripten.cmake"
  PKG_CONFIG_PATH="$(set -- /opt/*-wasm/lib/pkgconfig; IFS=":"; echo "$*")${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
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
    "$@")
}
