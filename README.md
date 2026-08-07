# EAGLE viewer (name pending — see below)

A browser-based viewer/editor for EAGLE CAD `.brd`/`.sch`/`.lbr` XML files: parses
the EAGLE XML DOM, renders schematics and boards to SVG, and drives a CAM pipeline
(Gerber export, G-code conversion) for board files. Includes a project/file chooser
for browsing available EAGLE projects on the backend.

Currently named "EAGLE renderer" (`index.html` `<title>`) — that name undersells the
CAM/editing features and needs to change. Candidates:

- **Eyrie** / **Aerie** — a bird of prey's nest; keeps the EAGLE association as a real
  English word instead of the acronym.
- **TalonView**
- **Wingspan**
- **BoardScope**
- **SchemaHawk**
- **CircuitEye**
- **GerberEye**

## Architecture

- **`index.html` + `main.js`** — the browser frontend. Preact-based UI (custom bundled
  `lib/preact.js`, not the npm package) that renders the SVG document, project/file
  chooser, layer controls, and CAM tooling. Talks to the backend over `fetch`/XHR
  (`commands.js`) and WebSockets (serial bridge, live update pushes).
- **`server.js`** — the current backend: a Node/Express server exposing the project's
  file listing, EAGLE file load/save, Gerber/G-code conversion, serial-port bridging
  (websocket), and static asset serving that `main.js` depends on.
- **`upload-server.js`** — an experimental, partial replacement backend built directly
  on qjs-net's low-level `createServer()` (QuickJS), meant to eventually take over from
  `server.js`. Not yet endpoint-compatible with `main.js`/`commands.js` — see `TODO.md`.

## OpenCV/ImGui Viewer

Simple OpenCV viewer using Dear ImGui interface.

Only testd on MacOSX.

# Dependency

- OpenCV>3.0
- SDL2
- Dear ImGui

# How to build

```
git clone https://github.com/ocornut/imgui.git
mkdir build
cd build
export OpenCV_DIR=OPENCV_INSTALL_PATH
cmake ..
make
```

# Root Script Index

Every `*.js` file directly in the repo root that has its own `main(...)` entry point
(`grep -lE '\bmain\s*\(' *.js`), sorted by line count as a rough proxy for
importance/complexity — substantial standalone tools/servers/apps at the top,
small one-off `test-*.js` smoke-test scripts at the bottom.

| Script | Lines | Description |
|---|---:|---|
| `tradeview.js` | 2166 | GLFW/nanovg GUI viewer for a crypto trading bot's SQLite database — renders candlestick charts, signals, trade brackets, and equity curve |
| `resolve-imports2.js` | 1730 | recursively resolves and concatenates a JS module's ES6 import graph into a single bundled `.es` output file |
| `eagle-shell.js` | 1657 | interactive REPL shell for loading EAGLE CAD schematic/board XML projects and manipulating parts, layers, and renderers, with gerber/gcode/SVG tools |
| `ast-shell.js` | 1655 | interactive REPL shell for compiling C/C++ sources to clang AST-dump JSON and exploring types/functions/structs, with FFI class generation |
| `resolve-imports.js` | 1397 | CLI ES-module tool that lexes JS files' import/export statements and can strip/rewrite imports, print dependency trees, or write reports |
| `upload-server.js` | 1328 | qjs-net HTTPS/WebSocket file-upload server — accepts multipart uploads, lists/serves them with directory browsing and thumbnails, plus a REPL |
| `fire.js` | 1019 | browser demo — renders a classic "Doom fire" pixel-propagation effect on canvas, seeded by mouse/touch input via a requestAnimationFrame loop |
| `debugger-server.js` | 1014 | qjs-lws WebSocket server that spawns and manages a QuickJS debugger session, relaying debugger protocol messages to connected clients |
| `server.js` | 947 | Express.js backend for the EAGLE/gerber CAD viewer web app — routes for gerber/gcode conversion, VFS/file access, serial ports, project save/config |
| `test-video.js` | 786 | interactive OpenCV.js video-processing test harness — reads a video/camera source through a Canny/contour/Hough pipeline with MIDI input |
| `test-opencv.js` | 723 | interactive OpenCV.js/GLFW smoke-test app — builds a live image-processing pipeline (threshold, Hough, skeleton) with mouse/keyboard tuning |
| `svg-crop-pad.js` | 682 | CLI that loads SVG files, computes/prints their bounding box, and rewrites the `viewBox`/width/height to crop or pad the drawing |
| `webakeit-telegram.js` | 633 | site-specific variant of the Telegram-export-to-blog converter, tailored to build `blog.html` for the webakeit bakery site |
| `dump-structs.js` | 592 | runs C/C++ source through `clang -ast-dump`, analyzes typedef'd structs/function prototypes, and generates JS ArrayBuffer struct/FFI bindings |
| `telegram-to-blog.js` | 569 | converts a Telegram Desktop HTML chat export into a self-contained static HTML blog page, copying/rotating referenced photos |
| `dump-ast.js` | 496 | runs `clang -ast-dump=json` on a C source file, then walks the AST to locate structs, printf-family calls, and format-string argument ranges |
| `test-xml.js` | 472 | reads a Sublime Text `.tmTheme` XML color scheme, extracts its color palette, applies a chosen transform, and writes back a modified theme |
| `list-exports.js` | 458 | lexes JS/BNF-family source files to find `import`/`export` statements and prints an `import { ... } from 'file'` summary line per file |
| `object_detection.js` | 451 | OpenCV.js DNN sample — loads a Caffe/TensorFlow/Darknet/ONNX model, runs object detection on video/camera/image with NMS |
| `create-tamper.js` | 437 | parses a set of JS module files, inlines/resolves their imports/exports recursively, and emits a bundled Tampermonkey userscript |
| `adsb-server.js` | 430 | qjs-lws WebSocket/TCP server that watches ADS-B state-dump files (inotify) and serves live aircraft-state updates to connected clients |
| `google-contacts.js` | 415 | QuickJS CLI that OAuth2-authenticates with Google People API, fetches all contacts, and prints/dumps them sorted by last-modified time |
| `photo_categorizer.js` | 412 | runs YOLOv8 (ONNX via OpenCV dnn) on all images in a folder, sorts each into a category subfolder based on detected objects |
| `test-rpc.js` | 410 | RPC/WebSocket test harness — starts an RPC client or server (file-serving, upload, proxy handlers) and drops into an interactive REPL |
| `test-image.js` | 404 | interactive OpenCV image viewer/pipeline tool — opens a window with mouse callback, runs images through an `ImagePipeline`, exports SVG/JSON |
| `eagle-server.js` | 376 | WS/HTTP backend server (custom `net` module) — serves static files plus `/config` and `/files` routes over TLS/WS, with a live-control REPL |
| `test-imgui.js` | 345 | interactive GLFW+nanovg+ImGui demo window exercising most ImGui widget bindings alongside a simple nanovg-drawn diamond shape |
| `ffi.js` | 327 | low-level libc FFI test/demo — dlopen/dlsym libc, calls malloc/strlen/strdup, then opens a raw TCP socket via syscalls and sends a manual HTTP GET |
| `test.js` | 314 | exercises OpenCV geometry/contour bindings plus edge/contour detection (Canny, findContours) on an input image, walking the contour hierarchy |
| `test-ffi.js` | 308 | exercises the custom `ffi` QuickJS module — dlopen/dlsym/call on libc functions, a bundled test.so, and inline x86-64 machine code |
| `test-parsetmpl.js` | 308 | tests `ECMAScriptParser`'s handling of tagged template literals, extracting tagged-template call nodes and running a JSX-string tokenizer |
| `list-proxies.js` | 303 | Node.js CLI that scrapes free proxy lists from multiple sources for given country codes, pings each proxy, and writes sorted results |
| `test-quickjs.js` | 299 | smoke test/exercise of the `opencv` module bindings (CV_* constants, Contour push/index/fitLine/fitEllipse), largely mirroring test-cv.js |
| `svg-box.js` | 292 | generates a laser-cutter SVG template for a box (lid/bottom/front/back/sides) with measurement annotations from CLI dimension args |
| `yolo.js` | 287 | real YOLOv3 object-detection CLI using OpenCV `dnn` — runs inference on image/video/webcam, draws bounding boxes+labels, saves JSON detections |
| `test-ecmascript2.js` | 284 | near-duplicate of test-ecmascript.js — parses a JS file, writes AST JSON and reprinted source, reporting parse success/failure |
| `test-eagle.js` | 278 | exercises the `lib/eagle.js` EAGLE CAD project API — loads a board/schematic project, normalizes pads, aligns elements to grid, saves documents |
| `test-mmap.js` | 275 | binary-patching tool — mmaps a target executable, scans it for byte patterns (license-check signatures), and writes out a patched copy |
| `blackbody.js` | 263 | looks up an RGB gamma correction triple for a given color temperature in a blackbody color table and prints an mpv/ffmpeg filter string |
| `doc-scanner.js` | 256 | OpenCV document-scanner script — finds the largest 4-point contour in an image, warps/crops it to a fixed page size, displays/writes results |
| `list-exports2.js` | 256 | CLI that parses a JS file, extracts its exported/imported identifiers, and prints/writes a synthesized `import {...} from './file'` statement |
| `test-nanovg.js` | 251 | interactive GLFW+nanovg+ImGui+OpenCV demo — draws a background image, animates transformed shapes with nanovg, renders an ImGui panel |
| `clang-list.js` | 249 | CLI that runs/loads a clang AST dump for given source files (caching), flattens it, and prints a table of named declarations |
| `test-ecmascript.js` | 248 | CLI that parses a JS file (or stdin) with the custom `ECMAScriptParser`, dumps the AST to JSON, and reports imports/requires/templates |
| `test-kolorwheel.js` | 238 | generates/prints color palettes/gradients using `KolorWheel`/`HSLA`/`RGBA`, mapping Eagle CAD layer names to a randomized palette |
| `test-cv.js` | 230 | smoke test of the `opencv` module bindings — Rect/Point/Size/Line/Mat/Contour constructors, Mat pixel get/set, iterators |
| `get-msys-packages.js` | 227 | downloads MSYS2/MinGW package DB files, resolves package/dependency args against them, and generates a script to curl+extract matches |
| `svgtransform.js` | 222 | CLI that reads an SVG/XML file, applies accumulated `transform` attributes as matrices to child elements, and rewrites their coordinates |
| `test-tmtheme.js` | 221 | converts TextMate `.tmLanguage`/`.tmTheme` plist XML files to/from a JS object model, extracts scope names, and rewrites the plist file |
| `cv-shell.js` | 213 | interactive OpenCV/CV REPL shell wiring up image/video helpers, history, and globals for live experimentation with `qjs-opencv` |
| `svg-scale.js` | 213 | CLI that reads SVG file(s), inlines CSS classes into attributes, reformats path data, and resizes the SVG to a given size, rewriting in place |
| `gerber2gcode.js` | 208 | CLI that groups Gerber/drill files by board, parses them, builds a `pcb2gcode` command line, runs it, and post-processes the resulting SVG |
| `scan.js` | 197 | OpenCV-based A4 document scanner CLI: detects the page quadrilateral, perspective-warps/thresholds it, and writes contour JSON plus PNG outputs |
| `check-discogs.js` | 196 | fetches Discogs seller order pages via spawned `curl` for given order IDs and prints/collects order info (mostly a stub) |
| `esparse.js` | 195 | CLI that parses a JS source file with the custom `ECMAScriptParser`, adds comments to the AST, and writes it as JSON to an output file |
| `compile_commands.js` | 194 | loads a `compile_commands.json`, builds `CompileCommand` objects and dependency maps, and parses CMake `link.txt` files |
| `test-ecmascript3.js` | 193 | CLI driver for the project's custom ECMAScript parser: parses input file(s), pretty-prints the AST back to `.es`/`.ast.json`, scans imports |
| `test-bjson.js` | 177 | standalone test of a custom `bjson.so` binary-JSON module — round-trips a BJSON file to JSON, with zlib/XML FFI helpers defined but unused in main() |
| `test-ini.js` | 175 | parses a `.desktop`/INI-style file with a custom INI grammar and converts it into an icon entry or reshuffles sections, rewriting the file |
| `test-umat.js` | 175 | OpenCV demo dumping Mat type constants, then running Canny/Hough/contour analysis on a schematic image and displaying results |
| `xml2js.js` | 172 | CLI that reads an XML file (e.g. EAGLE board file), converts it to a `Tree`, strips empty/wrapper nodes, and writes cleaned JSON/BJSON/XML |
| `test-filesystem.js` | 164 | TinyTest-based unit test suite exercising the `filesystem` module's buffer/file/dir API against a temp directory |
| `test-debuggerprotocol.js` | 159 | CLI that opens a raw TCP socket to a QuickJS debugger endpoint (client or server mode) and drives the `DebuggerProtocol` class |
| `adsb-client.js` | 157 | polls the OpenSky Network REST API for ADS-B aircraft state vectors every 10s and appends the JSON responses to rotating log files |
| `test-imread.js` | 156 | OpenCV demo that loads a bitmap-font image, segments it into a character grid, renders each glyph as ASCII art in a GUI window |
| `test-membrane.js` | 153 | parses an EAGLE `.brd` XML file, exercising the `deep`/`ImmutablePath`/`XMLIterator`/`ImmutableXPath` membrane-style path-mapping APIs |
| `sky_detect.js` | 149 | OpenCV CLI tool that loads an image, builds an HSV-based sky mask, finds the largest contour, and writes mask/contour PNG outputs |
| `test-midi.js` | 149 | standalone smoke test connecting to a TCP MIDI stream, decoding incoming bytes into MIDI events and logging them |
| `search-engine.js` | 147 | async scraper that queries several search-engine URL templates, parses HTML with the custom DOM parser, and extracts `src`/`href` links |
| `test-cparse.js` | 144 | exercises the custom `cpp`/`cparse` C preprocessor and parser libraries by preprocessing/parsing a hardcoded quickjs C source file |
| `test-minnet.js` | 143 | smoke test/demo of the `net` (minnet) module's `createServer`/`client`/`fetch` APIs, selectable via CLI arg |
| `convert-imports.js` | 142 | CLI that parses a JS file, finds `require()`/`import` nodes via `deep.select`, and converts between `import` and `require` styles |
| `test-glfw.js` | 138 | opens a GLFW/OpenGL window, loads image(s) via `opencv.imread`, uploads them as GL textures, and animates a color-cycling textured quad |
| `midi-tcp.js` | 130 | connects to a TCP MIDI stream, parses incoming MIDI events with `MIDIStream`/`MIDIEvent`, and reads raw stdin bytes (Ctrl-D to quit) |
| `eagle2gerber.js` | 129 | CLI tool that invokes the Eagle CAD binary to export Gerber/Excellon layers from `.brd` files, then zips the generated output folder |
| `test-features2d.js` | 119 | iterates OpenCV's Feature2D detector/descriptor classes (ORB, AKAZE, SURF, MSER, etc.), computes keypoints, and displays them |
| `test-fft.js` | 109 | loads a WAV file via `libsndfile` (raw FFI), runs an FFT (`lib/dsp/fft.js`) on a windowed segment, and prints the dominant frequency bin(s) |
| `yt-playlist.js` | 98 | runs `yt-dlp` as a subprocess to fetch JSON metadata for a YouTube playlist URL, then writes the collected results to `out.json` |
| `test-attributes.js` | 97 | reads XML files, collects the set of attribute names used per tag across all elements, and prints which attribute names hold numeric values |
| `bs.to-serie.js` | 96 | scrapes a "Burning Series" streaming site — reads show URLs from a text file and extracts metadata (title, actors, genres, years) into an array |
| `test-net.js` | 93 | smoke-test of the `net.so` module's `fetch()` — performs a GET request to google.com with a custom User-Agent |
| `xml2react.js` | 93 | reads XML (from a file or stdin) and converts it into `h(...)`-style React/hyperscript-like JS source code, printed to stdout |
| `read-deps.js` | 90 | runs `g++ -MM` to compute header dependencies for C/C++ sources, writes `deps.mk`, and cross-checks against a manual `#include` scan |
| `test-proxy.js` | 89 | reads an Eagle `.brd` XML file, wraps it in a lazy `Proxy`-based node/nodelist tree, and walks it with `deep.select` to find named parts |
| `extract-structs.js` | 87 | lexes a C source file with `CLexer` and prints out top-level `struct`/`typedef` declarations found at column 1 |
| `make-bitmap.js` | 87 | interactive OpenCV pipeline tool that loads an image, applies grayscale + adjustable threshold via a trackbar, and displays the result |
| `test-coverage.js` | 86 | reads a V8 coverage JSON report, matches script URLs against a regex, and computes/extracts used vs. unused source code ranges |
| `count-neighbours.js` | 85 | loads an image, thresholds/skeletonizes it with OpenCV, computes pixel-neighborhood/skeleton tracing, and drops into an interactive REPL |
| `test-inspect.js` | 82 | smoke-tests the QuickJS `inspect`/`Console` utilities by building an object with exotic values and printing it with various inspect options |
| `test-jsjs.js` | 82 | parses JS source file(s) with the custom `ECMAScriptParser`, re-prints the AST via `Printer` to a `.es` output file |
| `video-image-similarity.js` | 82 | opens a video file in an OpenCV `Window`, computes frame-to-frame MSE similarity in grayscale, and lets the user save frames as PNG |
| `ftrace.js` | 81 | parses an strace log file, reconstructs resumed syscalls, and writes lists of referenced filenames and functions that touched files |
| `test-childprocess.js` | 81 | spawns `ls -la` via `child_process.spawn`, sends SIGSTOP/SIGCONT to it, and streams its stdout via a custom `Repeater`-based reader |
| `eagle-query.js` | 80 | loads Eagle `.brd` board files, extracts R/C/L component values, builds value histograms, and prints a formatted report with color-code bands |
| `mailcow-db.js` | 79 | connects to a mailcow MySQL database (hardcoded host/credentials) and runs a test `SELECT * FROM mailbox` query |
| `test-cpp.js` | 76 | runs the local `lib/cpp.js` C preprocessor against a source file (invoking `cc1 -E` for reference), printing the expanded output |
| `splitdiff.js` | 75 | CLI tool that parses unified diff files and splits each hunk/chunk into its own `.diff` file, optionally skipping whitespace-only chunks |
| `test-geda-netlist.js` | 74 | parses gEDA netlist files, extracts components/nets, and writes each as a formatted JSON file |
| `test-diagram.js` | 73 | uses the local `diagram.js` axis/plotting helpers with `opencv` to draw a chart onto a Mat, saving `diagram.png` |
| `test-geom.js` | 73 | smoke-tests the `lib/geom.js` Point/PointList/Matrix API — rotate, centroid, bbox, area, transform, etc. |
| `test-websocket.js` | 73 | connects to a local `ws://127.0.0.1:3000/ws` server, sends a PING message, and handles HELLO/PONG/USERS/INFO/QUIT protocol messages |
| `test-renderer.js` | 69 | renders Eagle CAD schematic/board files for given basenames via `EagleDocument`/`Renderer`/Preact, writing SVGs to `tmp/*.svg` |
| `httpd.js` | 68 | spawns an `os.Worker` running `./ws-worker.js`, forwards `--host`/`--port` options to it, and relays messages (simple HTTP/WebSocket server driver) |
| `images2video.js` | 68 | CLI (opencv) that resizes/pads a list of images to a fixed canvas and encodes them into an MP4 via `VideoWriter` |
| `test-meriyah.js` | 68 | CLI that parses JS files via the npm `meriyah` package's `parseScript` and writes each file's AST as `<name>.ast.json` |
| `test-parse.js` | 68 | loads a grammar file into the custom `Grammar` parser, generates a JS parser module, and runs its `selector` against a sample string |
| `test-esprima.js` | 67 | CLI that parses JS files via the local `esprima` module's `parseModule` and writes each file's AST as `<name>.ast.json` |
| `test-console.js` | 66 | exercises the `console`/`inspect` implementation's formatting of functions, bound functions, Maps, WeakMaps, and null-prototype objects |
| `test-jslexer.js` | 66 | lexes a given file (default `./lib/misc.js`) using `lib/jslexer.js`, printing each token's id/lexeme/location |
| `test-utf8.js` | 65 | reads a file's raw bytes and decodes UTF-8 byte sequences into code points/strings using both a generator and a reduce-based decoder |
| `eagle2svg.js` | 64 | CLI that opens Eagle CAD files (schematic/board/library), renders them via `EagleDocument`/`Renderer` to SVG, and writes `<name>-<type>.svg` files |
| `bjson.js` | 63 | CLI that converts files between JSON and the binary `bjson.so` format based on `getOpt` args |
| `test-debug.js` | 63 | connects a `SocketDebugClient` (Debug Adapter Protocol) to a debug adapter on localhost:6666, sets breakpoints, and listens for stop events |
| `test-shparse.js` | 63 | spawns/reads a shell-parser AST via external `shparse2ast` (or a `.json` file), then parses and logs it |
| `meriyah.js` | 59 | CLI that parses a JS file with the local `meriyah` parser and writes the resulting AST as JSON to stdout or a file |
| `test-tree.js` | 59 | exercises the `Tree` wrapper class over a nested sample object, testing entries/indexOf/keyOf/pathOf/push/shift/flat/remove APIs |
| `test-lexer.js` | 57 | lexes a given file (default `./lib/ecmascript/parser.js`) with the custom ECMAScript `Lexer`, printing each token colorized by type |
| `test-eval.js` | 53 | parses a hardcoded JS snippet with the custom `ECMAScriptParser`, evaluates its AST in a sandboxed `Environment`, and writes reprinted source |
| `test-util.js` | 50 | exercises the local `Util` helper module (trace, getPlatform, waitFor, now, isAsync) and basic `filesystem` file writing |
| `test-octagon.js` | 46 | generates octagon/10-gon polygons via `MakePolygon`, builds SVG paths with `SvgPath`, and writes the composed SVG to `output.svg` |
| `test-cli.js` | 45 | smoke-tests the `opencv` binding's geometry/draw classes (Point, Size, Rect, Contour, Line, Draw) by constructing and iterating them |
| `test-lex.js` | 43 | runs the custom ECMAScript `Lexer` over input file(s) (or stdin), lexing all tokens and recording per-file errors |
| `test-ngql.js` | 41 | builds a GraphQL query with `nanographql`, POSTs it to a hardcoded Heroku GraphQL endpoint, and logs the JSON response |
| `test-pointer.js` | 39 | exercises the `pointer`/`deep` native modules — builds a `Pointer`, uses `deep.set` to write into an object via the pointer path |
| `test-gpio.js` | 38 | smoke-tests the native `gpio` module by initializing output/input pins, setting/reading a pin, and dumping the GPIO mmap buffer |
| `test-description.js` | 37 | reads Eagle CAD XML files given as args and extracts/prints the `<description>` text found in the board/schematic/library element |
| `test-sockets.js` | 36 | opens an `AsyncSocket` (AF_INET/TCP), connects to a hardcoded host on port 22, asynchronously receives data, and logs the escaped response |
| `test-circuit.js` | 34 | parses an electronics `.circuit` file with `CircuitFileParser`, logs its elements, then re-serializes it with `CircuitFileWriter` |
| `fontsample.js` | 33 | builds a byte array covering all 256 char codes as a font sample, logs it, and writes the string form to `output.txt` |
| `test-archive.js` | 31 | opens a tar.xz archive via the `archive` module, iterates/extracts its entries, and logs progress and file count |
| `test-readline.js` | 31 | reads a single line of input via the local `readline.js` module's `readline()` at a prompt |
| `takephoto.js` | 30 | uses the `opencv` binding's `RaspiCam` to capture a photo at 2592x1944 and saves it to `photo.png` |
| `test-repl.js` | 29 | starts an interactive `REPL` (from `./repl.js`) with mouse tracking enabled via the `Terminal` module, adding a custom compile directive |
| `test-io.js` | 28 | implements an async `FileReader` using `filesystem` + `Repeater` and streams its own source in 64-byte chunks to the console |
| `test-call.js` | 27 | defines a `Callable` class extending `Function` and tests constructing it both from a JS function and from `(args, body)` strings |
| `test-css3.js` | 27 | runs the `grammar-css3.js` `selector` parser against a hard-coded complex CSS selector string, reporting SUCCESS/FAIL |
| `test-path.js` | 26 | exercises the local `lib/path.js` module's API (basename, relative, sep, delimiter, etc.) against the current script path |
| `test-clexer.js` | 25 | reads a C source file (default `pa_devs.c`) and tokenizes it with the `tokenize` module, logging each token |
| `test-stack.js` | 24 | logs a custom-inspected object and exercises `Util.getStackFrame`/`Util.getCallerStack` to print current call-stack info |
| `fetch-post-prices.js` | 22 | spawns `curl` via `os.exec`/pipes to fetch the Swiss Post pricing info page and prints the raw HTML/JSON response |
| `test-readdir.js` | 22 | recursively lists `.c`/`.h` files under `.` using `ReadDirRecursive`/a filter generator, then sends itself `SIGUSR1` |
| `test-containers.js` | 21 | exercises `MultiKeyMap`, `MultiBiMap`, and `HashMultimap` container classes by inserting sample entries and logging their contents/keys/values |
| `test-sourcemap.js` | 21 | builds `SourceMap` objects from `//# sourceMappingURL=` comments for htm/preact map files and logs their base64 and comment forms |
| `test-editline.js` | 20 | uses `ffi` to `dlsym` the `rl_meta_chars` symbol from the loaded `editline` library handle and dumps it as a hex address and ArrayBuffer |
| `test-bash-parser.js` | 19 | reads `../cfg.sh` and parses it with the `bash-parser` module, logging the resulting AST |
| `test-import.js` | 17 | imports the `bjson` module and dynamically imports `ffi`, logging both to check they load correctly |
| `test-parsexml.js` | 17 | reads an XML file, parses it with `lib/xml/parse.js`, and re-serializes the result back to XML via `lib/json.js`'s `toXML` |
| `test-linenoise.js` | 15 | loads/saves history and runs an interactive `linenoise` prompt loop, echoing each entered line until `quit` |
| `test-psql.js` | 14 | uses the local `psql.js` FFI wrapper to open a libpq connection to a local Postgres DB, run `SELECT * FROM test`, and log the result/error |
| `test-exception.js` | 13 | logs its args then unconditionally throws an `Error('This is an error')` to exercise error/exception handling |
| `test-sqlite3.js` | 10 | calls `sqlite3_open` from the local sqlite3 FFI binding to open a Chrome `History` sqlite file and logs the return code |
| `test-stream.js` | 9 | opens `tmp/7seg-2.54.brd` via the `filesystem` module, reads it async, and opens `tmp/test.txt` for writing (result unused) |
