# TODO / Work Summary

Generated from `./*.js` file modification times + git history. Reflects state as of 2026-07-10.

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
