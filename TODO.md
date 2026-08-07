# TODO / Work Summary

## EAGLE viewer (index.html, main.js, server.js, upload-server.js) — 2026-08-07

See `README.md` for a description of this subproject (name still pending — "EAGLE
renderer" is a placeholder).

1. ~~File chooser doesn't render~~ — **fixed 2026-08-07**, two independent bugs:
   - `lib/preact.js`'s `render()` (in the `lib` submodule) called `diff()` with a
     comma-operator-collapsed expression instead of separate arguments, so every
     `diff()` call actually received `newVNode = isHydrating` (a boolean) and bailed
     out immediately — nothing in the app ever rendered at all, not just the chooser.
   - Separately, `main.js:124`'s `const Timer = { ...Timers, once: dom.Timer };`
     pointed `.once` at the raw `Timer` constructor instead of its `.promise` static,
     so `Timer.once(1000).then(...)` (used by the file list's search-box mount at
     `main.js:1952-1957`) crashed with an uncaught exception during the file list's
     first render — which aborted attaching that whole (already-built) subtree to the
     document before Preact ever got to insert it, so the chooser panel never
     appeared even once the first bug above was fixed. Changed to
     `once: dom.Timer.promise`.
   - Verified end-to-end in a browser: file chooser now renders its full project
     list, and clicking an entry loads and renders the actual board/schematic.
   - Fixing this surfaced three previously-masked, unrelated errors on every
     document load (`EagleMaps`, layer-color population, `SaveSVG`/XMLSerializer) —
     logged separately in `BUGS`, not fixed here.

2. **Migrate `server.js`'s endpoints to `upload-server.js`** (qjs-net-based). Gaps
   found: `upload-server.js` is missing `/gerber`, `/gcode`, `/list-serial` + the
   `/serial` websocket, `/contours`, a proper `/save`, `/favicon.ico`, the
   `data|tmp|vfs` static route, `/!urls`, `/github`, and the CSS catch-all route, and
   has no timestamped `/index.html` handler. Its `file`/`files`/`files2` routes are
   present but structurally different (different route names/response shapes) from
   `server.js`'s `/file` and `POST /files` (the one `commands.js`'s `ListProjects`
   actually calls) — not wire-compatible with `main.js` as-is.

3. **Isolate + document `main.js`'s actual backend endpoints, then build a shared
   `eagle-server.js`.** First write a markdown file enumerating exactly which
   endpoints `main.js`/`commands.js` call on `server.js`/`upload-server.js` (not
   written yet). Then implement `eagle-server.js` using `serve()`, so the same
   network code runs on both Bun and QuickJS+qjs-lws via:
   ```js
   const { serve } = typeof Bun == 'undefined' ? await import('serve.js') : Bun;
   ```
   **Not implemented yet** — qjs-lws's `serve()` (`quickjs/qjs-lws/lib/serve.js`) is
   already deliberately Bun-`Bun.serve()`-shaped (`fetch`, `routes`, `websocket`,
   async-iterator form; `Request.formData()`/`.blob()`/`.arrayBuffer()` already work).
   Concrete gaps that need fixing there before `eagle-server.js` can be truly shared
   code:
   - No imperative `server.upgrade(req, {data})` — `serve()` only supports one fixed
     `options.websocket` mountpoint set up front, but `server.js` needs two distinct
     WS endpoints on the same port (the generic `/ws`+`/.websocket`+`/ws/.websocket`
     hub, and `/serial`).
   - No WS pub/sub (`ws.subscribe(topic)`/`server.publish(topic, msg)`) — `server.js`'s
     `/contours` route broadcasts to all sockets manually; Bun-style code would use
     publish/subscribe instead.
   - Response streaming without an explicit `content-length` fully buffers the body
     (`serve.js:202-220`'s `flush()`) instead of chunked-encoding it like Bun —
     matters for large Gerber/G-code/file responses.

4. **Migrate the EAGLE renderer's generic-DOM layer onto the qjs-module `'dom'`,
   and remove the dead legacy `lib/eagle/element.js` stack.** Investigated both
   renderer code paths (`eagle2svg.js` CLI and `eagle-renderer.js`/browser) before
   writing this — the actual state differs from a first-glance assumption, noted
   below so the next person doesn't have to re-derive it:
   - `eagle2svg.js` (CLI, `#!/usr/bin/env qjsm`) and root `eagle.js` **already don't
     use `lib/eagle/element.js`** — that's a separate, unrelated legacy stack
     (`lib/eagle/element.js`, `node.js`, `nodeList.js`, `nodeMap.js`, `ref.js`,
     `document.js`, `project.js`, `svgRenderer.js`, `renderer.js`, `boardRenderer.js`,
     `schematicRenderer.js`, `libraryRenderer.js`, plus unrelated dead
     `classes.js`/`elementProxy.js`) with **zero live consumers anywhere in the
     repo** today (checked every external importer, direct and transitive — none).
     Root `eagle.js:2` already builds on a generic DOM (`EagleElement extends
     Element`) — just not the qjs-module one.
   - There are **two near-identical ~2100-line copies** of the generic DOM: the
     native qjs-module `'dom'` (`quickjs/qjs-modules/lib/dom.js`, resolvable as a
     bare specifier under `qjsm` — already used elsewhere, e.g. `eagle-shell.js:7`),
     and a portable duplicate at root `lib/dom.js` (imports local reimplementations
     `./deep.js`, `./misc.js`, `./pointer.js`, `./tree_walker.js`, `./xml.js`
     instead of qjs-native `'deep'`/`'xml'`/etc.). Root `eagle.js:2` and
     `lib/eagle-dom.js` both import the **portable copy**, not the native module.
     Confirmed both really are backed by a plain `{tagName, attributes, children}`
     object/array tree (via the native `'xml'` parser + `'deep'` path-based tree
     ops under the hood) — i.e. genuinely "a virtual generic DOM on top of an
     object/array-tree", matching the description this item is named after.
   - **Actual migration**: since `eagle2svg.js` only ever runs under `qjsm`
     already (shebang, `scriptArgs`, native `'fs'`/`'console'`/`'util'`), it can
     safely switch to importing the real bare-specifier `'dom'` module instead of
     `./lib/dom.js`, collapsing to one source of truth for that path. The browser
     path (`lib/eagle-dom.js`, used by `main.js`) must keep using the portable
     `lib/dom.js` copy, since browsers/bundlers can't resolve the native `'dom'`
     specifier — so this is a CLI-side-only migration, not a full replacement of
     `lib/dom.js`.
   - **Separately**: delete/archive the dead `lib/eagle/element.js` legacy subtree
     listed above (no live consumers found) — this is cleanup, not migration, since
     there's nothing left importing it to move off of.

5. **Make `eagle-renderer.js` actually work in the browser.** Traced the full
   dependency tree of `eagle-renderer.js` (`lib/preact.js`, `lib/geom.js`,
   `lib/color/rgba.js`, `lib/eagle/common.js`, `lib/eagle/components.js` and all of
   `lib/eagle/components/*.js`) — no Node/QuickJS-only APIs anywhere (`os`, `std`,
   `scriptArgs`, native bare-specifier modules), and it's already wired into
   `index.html` (`main.js:22` imports `Renderer`/`SchematicRenderer`/etc. from
   `lib/eagle.js`, which is a barrel re-exporting straight from root
   `eagle-renderer.js` — same implementation the CLI path uses). So the renderer
   module itself needs no portability changes.
   The actual blocker is a **stale call signature at the call site**,
   `RenderProject()` in `main.js` (~`main.js:1173-1190`):
   - `main.js:1176`: `new Renderer(doc, ReactComponent.append, config.debugFlag())`
     — but the current `Renderer` factory (`eagle-renderer.js:671-685`) only takes
     `doc`; the extra two args are leftover from the old `lib/eagle/renderer.js`
     API and are silently ignored (harmless).
   - `main.js:1190`: `project.renderer.render(doc, null, {})` — but
     `SchematicRenderer.render(sheetNo = 0)` (`eagle-renderer.js:492`) expects a
     **numeric sheet index** as its only argument. Passing `doc` here makes
     `sheetNo` a non-numeric value, so `filterChildren(findChild(schematic,
     'sheets'), 'sheet')[sheetNo]` (`eagle-renderer.js:495-497`) indexes an array
     with a bad key, `sheet` ends up `undefined`, and every `if(sheet)` branch is
     skipped — the renderer doesn't throw, it just silently produces a
     near-empty/degenerate SVG. (`BoardRenderer.render()`/`LibraryRenderer.render()`
     take no args, so they're unaffected by the extra args, but would still need
     checking once the schematic path is fixed.)
   - **Fix**: update the call site to `new Renderer(doc)` and
     `project.renderer.render(sheetNo)` (some real sheet index, defaulting to `0`),
     matching the current `eagle-renderer.js` API. Not yet implemented — flagged
     here as the concrete next step, not done as part of this investigation.
   - Same `RenderProject()` function also reads `project.renderer.rect`,
     `.bounds`, `.size`, and reads+writes `.grid` (`main.js:1181-1184`), but none
     of `rect`/`bounds`/`size`/`grid` exist anywhere on `EagleSVGRenderer`/
     `SchematicRenderer`/`BoardRenderer`/`LibraryRenderer` in `eagle-renderer.js`
     — only local `bounds` variables scoped inside `render()`, never exposed as
     instance properties. Same root cause (stale contract from the old renderer),
     same fix location. Either add `size`/`bounds`/`grid` getters (and a `grid`
     setter) to `EagleSVGRenderer`, computed from the last `render()` call's bbox
     plus the drawing's `<grid>` element, or change `main.js` to pull bounds from
     the rendered SVG's `viewBox` instead. Not yet implemented.

6. **`eagle.js`/`lib/eagle-dom.js` model gained `getColor()`/`getLayer()`/
   `handlers` — fixed 2026-08-07.** Audited every place `main.js`,
   `lib/eagle/components.js`, and `lib/eagle/components/*.js` call a method on an
   EAGLE document/element/layer object, to find the full set of gaps behind the
   `layer.getColor is not a function` crash (previously logged in `BUGS`, now
   removed as fixed):
   - `main.js`'s layer-panel code was written against the old, dead
     `lib/eagle/element.js` API (per-element `getColor()`/`getLayer()`, a
     trkl-reactive `handlers` map) — main.js never got updated when the app moved
     onto the new generic-DOM `eagle.js`/`lib/eagle-dom.js` model, which had none
     of that.
   - `eagle-renderer.js`'s `EagleAdapter` (used by `lib/eagle/components/*.js`,
     `eagle-renderer.js:73-249`) already reimplements an equivalent surface
     correctly — `getLayer()`/`getColor()`/`handlers.visible` on wrapped elements,
     resolving through `Palette` (`lib/eagle/common.js:25`) — but `main.js`'s
     layer-panel code (`main.js:1192-1216`) builds `usedLayers` straight from raw
     `doc.layers` instead of going through the adapter, so it never got this.
   - **Fix implemented**: added `getLayer()`/`getColor()` directly to the base
     `EagleElement` class, and `getLayer()`/`getColor()`/a real trkl-backed
     `handlers.visible` signal (plus a working `visible` setter — it was
     getter-only before) to `LayerElement`, in both `eagle.js` (CLI/`eagle2svg.js`
     path) and `lib/eagle-dom.js` (the copy `main.js` actually imports at
     runtime, `main.js:23`) identically. `EagleDocument` gained a memoized
     `.palette` (board/schematic `Palette` resolved to `RGBA`, matching
     `EagleAdapter`'s logic) and `.getLayer(ref)`. Both `element.visible = v` and
     `element.handlers.visible(v)` now go through the same per-element trkl
     signal (cached in a `WeakMap`), so they can't desync — `main.js:2275-2276`
     switches between the two paths depending on context, and previously only one
     of them would have been reactive.
   - **Live-verified 2026-08-07**: loaded a real schematic via the file chooser,
     opened the layer selector dropdown (`main.js:2465-2494`) — populated with all
     131 used layers, real color swatches, and toggling a layer's visibility icon
     updates the row live (crossed-out eye ↔ open eye), confirming the trkl signal
     is genuinely reactive end-to-end, not just statically correct.

7. **Layer selector dropdown was actually broken by a separate, deeper bug in
   `lib/dom.js` itself — fixed 2026-08-07.** The `getColor()`/`handlers` fix in
   item 6 above was necessary but not sufficient: reproduced outside the browser
   (`node` + `EagleParser().parseFromString(...)`, same as `main.js`'s
   `LoadFile()`) and found `[...doc.layers]` was yielding 256 entries for a file
   with only 150 real `<layer>` elements, with the *first* entry being a bogus
   `[Function: undefined]` instead of a `LayerElement` — which is exactly why
   `usedLayers.map(layer => ({..., color: layer.getColor(), ...}))`
   (`main.js:1198-1201`) kept throwing even after item 6's fix.
   - Root cause: `NamedMap`'s `[Symbol.iterator]` (`lib/dom.js:928-931`, used by
     the `<layers>` element's number/name-indexed proxy) did
     `for(let i = 0; this[i]; i++) yield this[i]`, relying on falsiness to detect
     the end of a *sparse* number→name array (EAGLE layer numbers aren't
     contiguous). For a hole (no layer at that number), the proxy's numeric-index
     path reassigns `prop` to `undefined`, which falls through to
     `Reflect.get(target, undefined, receiver)` — and that resolves to a truthy
     `Function`, not `undefined`, because of the separate `Symbol.inspect` bug
     logged in `BUGS` (`[Symbol.inspect]` isn't a real symbol, so it silently
     defines methods under the literal string key `"undefined"` on
     `Element.prototype`/`Node.prototype`). The loop therefore couldn't detect
     holes and kept running for the array's full 256-slot length, yielding a mix
     of real `LayerElement`s and this stray function for every gap.
   - **Fix implemented**: rewrote the iterator to use
     `for(const key of keys().filter(isPropertyKey)) yield this[key];` —
     `keys().filter(isPropertyKey)` was already used correctly by this same
     proxy's `length`/`ownKeys` traps two lines away (`lib/dom.js:926,953`), so
     this makes the iterator consistent with them and immune to sparse holes
     regardless of the `Symbol.inspect` bug's status.
   - Verified both outside the browser (Node script mirroring `main.js`'s exact
     `usedLayers.map(...)` call: 150/150 real `LayerElement`s, no throw, resolved
     hex colors, working trkl visibility toggle including the `element.visible =
     v` direct-assignment path `main.js:2275-2276` also uses) and live in the
     browser (see item 6's live-verification note above — same session).
   - The underlying `Symbol.inspect` typo itself was fixed separately right
     after (see item 8 below) at your explicit request.

8. **`Symbol.inspect` typo fixed — 2026-08-07** (both `lib/dom.js` and its
   native-qjs-module twin `quickjs/qjs-modules/lib/dom.js`, at your request).
   Turned out to be a plain rename, not a new symbol to invent:
   `lib/dom.js:52` already had an unused `const inspectSymbol =
   Symbol.for('quickjs.inspect.custom');` sitting right there — every
   `[Symbol.inspect]` computed-key method definition (9 sites) and both call
   sites (`lib/dom.js:996,1342`) were simply supposed to reference that local
   constant instead of the nonexistent `Symbol.inspect` global. Fixed with a
   straight `Symbol.inspect` → `inspectSymbol` replacement (11 occurrences),
   confirmed present and identical (including the same unused constant) in
   `quickjs/qjs-modules/lib/dom.js`, fixed there too (12 occurrences including
   the declaration).
   - Verified: both files pass `node --check`. Re-ran the Node reproduction
     script from item 7 — `layersEl['undefined']` now correctly returns
     `undefined` (previously the stray inspect function), `[...doc.layers]`
     still yields all 150 real `LayerElement`s (unaffected, since item 7's
     iterator fix no longer exercises this path either way), and the custom
     inspect method itself is now genuinely callable and produces real
     formatted output instead of being permanently dead code.

## Active this week (2026-07-07 → 2026-07-09)

### 1. EAGLE CAD toolkit (eagle.js, eagle-renderer.js, eagle-shell.js, eagle2svg.js)
DOM-style parser + SVG renderer for EAGLE `.brd`/`.sch`/`.lbr` XML files.
- Reworked `eagle.js` element accessors: replaced the old `Collection(this)` constructor-return trick with plain getters (`ChildrenByTag`, `NamedChildByAttr`) — cleaner, more predictable API (commit `9f09c1f0`).
- Rewrote `eagle-renderer.js` from scratch as an adapter layer (`EagleAdapter`) that presents DOM elements with the numeric-attribute / layer-object / palette API expected by `lib/eagle/components.js`, replacing the old hand-rolled palette + transform-string code (commit `ba721d47`, ~870 lines touched).
- Recovered `dir-helpers.js`, which had been accidentally clobbered in commit `8200d99c` (2026-07-07 11:51) with a raw ImageMagick PostScript dump (502k+ lines, likely a misdirected shell redirect) instead of its JS source — rewritten back to a working `DirIterator`/`RecursiveDirIterator` module an hour later in commit `9f09c1f0`.
- Checked (full history search, `git log --all -S`): no other file was hit by the same mistake — `eagle.js`, `eagle2svg.js`, `tradeview.js` in the same commit are legitimate edits, confirmed by diff.
- Followed by a repo-wide Prettier-style reflow (long lines wrapped) across `main.js`, `eagle-shell.js`, `upload-server.js`, `tradeview.js`, `google-contacts.js`, `http-client.js`, `common.user.js`, `dir-helpers.js`, `eagle2svg.js`, `yt-playlist.js` (commit `2983d9a0`) — formatting only, no logic changes.

### 2. YouTube playlist metadata fetcher (yt-playlist.js) — most actively iterated file
New script (added 2026-07-08), rewritten 5 times same day/next day:
- Started as `spawnSync` calling `yt-dlp -J` (blocking, buggy — referenced `result` before assignment).
- Converted to async `spawn` + manual line-reading loop over `waitRead`/`gets` on the child's stdout pipe, parsing NDJSON output.
- Tuned logging (`abbreviate(line, 120)` cap), fixed output to preserve newlines between JSON records, dumps final result to `out.json`.
- **TODO:** the old `spawnSync` version is left commented out at the top of the file — decide whether to delete it or keep as fallback reference.
- **TODO:** cookies path is hardcoded to `/home/roman/cookies.txt` — fine for personal use, but worth a comment/env var if this is meant to be portable.

### 3. Google Contacts sync (google-contacts.js, http-client.js)
OAuth2 flow (browser redirect + local callback server) against the Google People API, using a custom `HttpClient` with pluggable curl/lws/native backends. No functional changes this week — only caught up in the formatting pass.

### 4. Trading view (tradeview.js)
GLFW + nanovg live candle chart viewer, backed by SQLite (`candles`/`predictions`/`actions` tables), with ATR/ADX/regime-filter indicators and position sizing. No functional changes this week — formatting pass only.

### 5. Userscript helpers (common.user.js)
Added `define`/`nonenumerable`/`declare` helpers and a `quote`/`extractTable` pair for scraping HTML tables into aligned plain-text — useful for pulling tabular data off pages via the userscript. Loosened the `@match` from `*://*/*` to `*`.

## Dormant sub-projects (last touched mid/late May, unchanged since)

- **Computer vision / photo pipeline** — `yolo.js`, `object_detection.js`, `sky_detect.js`, `detect_sky.js`, `detect_lines.js`, `photo_categorizer.js`: OpenCV-based (via `qjs-opencv`) object/sky detection feeding a photo categorizer.
- **Telegram/blog tooling** — `telegram-to-blog.js`, `webakeit-telegram.js`, `scan.js`: scraping/relaying Telegram content into blog posts.
- **Circuit conversion** — `circuit2circuitjs.js`: converts circuit descriptions to the `circuitjs` simulator format.

## Submodule note
`quickjs`, `qjs-opencv`, and `eslint-plugin-unused-imports` all show as modified (submodule pointer bumps) with their own dirty working trees (build artifacts, nested submodule changes, and — in `eslint-plugin-unused-imports` — real source edits under `src/`). Not covered by this summary; revisit separately if that work needs tracking.
