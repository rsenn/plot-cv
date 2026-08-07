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
