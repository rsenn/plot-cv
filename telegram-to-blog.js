// Generic Telegram (Desktop HTML export) → single-file Web blog converter.
//
// Reads a Telegram chat-export `messages.html` (or follow-up `messages2.html`,
// passed via argv) and emits a self-contained blog HTML file. CSS and the
// photo-viewer JS are inlined into the output; photos referenced from the
// export are copied (or rotated) into PHOTOS_DIR alongside the output.
//
// Environment:
//   BLOG_TITLE       page <title> (default: "Blog")
//   BLOG_OUTPUT      output html path (default: "blog.html")
//   BLOG_PHOTOS_DIR  where photos are copied to (default: "photos")
//   DEBUG            verbose copy/convert logging
//
// User-tweakable: populate EXCLUDE_IDS / PHOTO_PROPERTIES below for per-export
// fixups. Both are empty by default and have no effect when empty.

import { Parser, Serializer } from 'dom';
import { writeFileSync, existsSync, constants, copyFileSync, unlinkSync, statSync } from 'fs';
import { Console } from 'console';
import { dirname, join, basename, relative } from 'path';
import { glob, GLOB_TILDE, mapWrapper } from 'util';
import htmlentities from './lib/htmlentities.js';
import { inspect } from 'inspect';
import { spawn } from 'child_process';
import { popen } from 'std';
import extendGenerator from 'extendGenerator';

extendGenerator();

const { DEBUG, BLOG_TITLE, BLOG_OUTPUT, BLOG_PHOTOS_DIR } = process.env;

const TITLE = BLOG_TITLE || 'Blog';
const OUTPUT_FILE = BLOG_OUTPUT || 'blog.html';
const PHOTOS_DIR = BLOG_PHOTOS_DIR || 'photos';

// Per-export user tweaks. Leave empty for a clean run.
const EXCLUDE_IDS = []; // e.g. [955, 1022] — Telegram message IDs to skip
const PHOTO_PROPERTIES = {}; // e.g. { 808: { rotate: -90 } } — photo-id → ImageMagick tweaks

const Message2Element = mapWrapper(new WeakMap());

// Inlined into <head><style> of the generated blog.
const DEFAULT_CSS = `
body { margin: 0; background: #fafafa; color: #222; font-family: system-ui, sans-serif; }
div.blog { padding: 1em; max-width: 50em; margin: 0 auto; }
#header { display: none; }
#content { }
h4 { margin: 1.5em 0 0.5em; }
p { line-height: 1.5; }
img.bywidth { max-width: 15em; cursor: pointer; }
img.landscape { max-height: 15em; cursor: pointer; }
img.close { position: absolute; top: 1em; right: 1em; cursor: pointer; }
.overlay {
  background: rgba(0,0,0,0.85);
  position: fixed; inset: 0;
  display: flex; align-items: center; justify-content: center;
  z-index: 1000;
}
.overlay img { max-width: 100vw; max-height: 100vh; }
.overlay .close-btn {
  position: absolute; top: 0.5em; right: 0.75em;
  color: white; font-size: 3em; line-height: 1; cursor: pointer;
  user-select: none;
}
`;

// Inlined into <body><script> of the generated blog.
const DEFAULT_JS = `
function ShowPhoto(el) {
  var src = el.getAttribute('data-fullsize') || el.getAttribute('src');
  var w = el.naturalWidth || el.width, h = el.naturalHeight || el.height;
  var overlay = document.createElement('div');
  overlay.className = 'overlay';
  var img = document.createElement('img');
  img.src = src;
  if(w && h) img.style[w > h ? 'width' : 'height'] = '100%';
  overlay.appendChild(img);
  var close = document.createElement('span');
  close.className = 'close-btn';
  close.textContent = '\\u00D7';
  overlay.appendChild(close);
  var dismiss = function() { document.body.removeChild(overlay); };
  close.addEventListener('click', dismiss);
  overlay.addEventListener('click', function(e) { if(e.target === overlay) dismiss(); });
  document.body.appendChild(overlay);
}
`;

export function toXML(e) {
  return new Serializer().serializeToString(e);
}

export function InputFileName() {
  return process.argv[2] || glob('~/Downloads/Telegram Desktop/ChatExport*/*.html', GLOB_TILDE).reverse()[0];
}

export function InputDir(f = InputFileName()) {
  return dirname(f);
}

export function ExecAndGetOutput(prog, ...params) {
  const f = popen(prog + params.map(p => ` '${p}'`).join(''), 'r');
  const s = f.readAsString();
  f.close();
  return s;
}

export function Identify(f) {
  const s = ExecAndGetOutput('identify', f);

  if(typeof s != 'string') throw new Error(`Identify(): s is not a string ${inspect(s)}`);

  const [width, height] = s
    .match(/\d+x\d+/g)[0]
    .split('x')
    .map(n => +n);

  return {
    width,
    height,
    format: width > height ? 'landscape' : width < height ? 'portrait' : 'square',
  };
}

export function Copy(src, dest) {
  try {
    unlinkSync(dest);
  } catch(e) {}

  if(DEBUG) console.log(`Copying '${relative(src)}' to '${dest}'.`);

  return copyFileSync(src, dest, constants.COPYFILE_FICLONE);
}

export function Convert(src, dest, options = []) {
  try {
    unlinkSync(dest);
  } catch(e) {}

  if(DEBUG) console.log(`Converting '${relative(src)}' to '${dest}'.`);

  const cmd = ['convert-im6', src, ...options, dest];
  const child = spawn(cmd);

  child.wait();

  if(!(child.exited && child.exitcode == 0)) console.error(`Command '${cmd.join(' ')}' failed.`);
}

export function TextTransform(s) {
  return s
    .split(/\s+/g)
    .map(w => htmlentities.decode(w))
    .map(w => htmlentities.encode(w))
    .join(' ');
}

export function* Iterate(a, prog) {
  const { length } = a;

  if(prog) prog(0, length);

  for(let i = 0; i < length; i++) {
    yield a[i];

    if(prog) prog(i + 1, length);
  }
}

export function GetClasses(e) {
  return (e.getAttribute('class') || '').split(/\s+/g);
}

export function ProcessText(e, date) {
  let child,
    text = '';
  const r = [],
    it = Iterate(e.children);

  while((child = it.next().value)) {
    if(isListItem()) {
      Yield();

      do {
        if(child.tagName == 'br') {
          text += '\n';
          child = it.next().value;

          if(!isListItem()) break;
        }

        AddText();
      } while((child = it.next().value));

      Yield();

      continue;
    }

    AddText();
  }

  Yield();

  function isListItem(s = child.textContent) {
    return /•/.test(s);
  }

  function AddText(s = child.textContent) {
    text += (text != '' ? ' ' : '') + s;
  }

  function Yield() {
    if(text != '') {
      r.push({ type: 'text', text, date });
      text = '';
    }
  }

  return r;
}

export function* GetMessage(msg) {
  let body = msg.querySelector('.body');

  const tmp = body.querySelector('.forwarded.body');

  if(tmp) body = tmp;

  if(body.querySelector('.reply_to')) return;

  if(body) {
    let date;

    if(GetClasses(body).indexOf('details') != -1) {
      const text = body.innerText;

      if(!/^Channel/.test(text)) yield { type: 'day', text: text.trim() };
    }

    if(body.children) {
      for(const e of [...Iterate(body.children)]) {
        if(e.nodeType != e.ELEMENT_NODE) continue;

        const cl = GetClasses(e);

        if(cl.indexOf('date') != -1) {
          const str = e.getAttribute('title');
          const [d, m, y, hr, min, sec] = [...str.matchAll(/\d+/g)].map(([a]) => a);

          date = new Date(`${y}-${m}-${d}T${hr}:${min}:${sec}`);
          continue;
        }

        if(cl.indexOf('text') != -1) {
          const r = ProcessText(e, date);

          if(r.length > 1) {
            console.log('e', e.parentNode.parentNode);
            console.log('r', r);
          }

          yield* r;
        } else if(cl.indexOf('media_wrap') != -1 || cl.indexOf('userpic_wrap') != -1) {
          for(const f of e.querySelectorAll('a')) {
            const img = f.children[0];
            const thumb = img.getAttribute('src');
            const link = f.getAttribute('href');

            yield { type: 'photo', link, thumb, date };
          }
        }
      }
    } else if(body.innerText) {
      yield { type: 'text', text: body.innerText };
    }
  }
}

export function* LogGenerator(gen, fn = arg => console.log(arg)) {
  let i = 0;

  for(const elem of gen) {
    fn(elem, i++);
    yield elem;
  }
}

export function GetId(e) {
  const [m] = e.getAttribute('id')?.match(/\d+/g) ?? [];

  if(m) return +m;
}

export function* GetMessages(hist) {
  if(!hist) throw new Error('GetMessages: history element is required');

  const { err } = std;

  function Progress(i, n) {
    err.puts(`\rEntry ${i}/${n} (${((i * 100) / n).toFixed(1)}%)\x1b[K` + (i == n ? '\n' : ''));
    err.flush();
  }

  for(const e of Iterate(hist, Progress)) {
    const id = GetId(e);

    if(EXCLUDE_IDS.includes(id)) continue;

    try {
      yield* GetMessage(e);
    } catch(error) {
      console.log('GetMessages', e);
      throw error;
    }
  }
}

export function AccumulateDays(messages) {
  let days = {},
    day;

  for(const msg of messages) {
    if(msg.type == 'day') {
      day = msg.text;
      continue;
    }

    days[day] ??= [];
    days[day].push(msg);
  }

  return Object.entries(days);
}

export function* GenerateDays(days) {
  for(const [day, messages] of days) {
    yield { type: 'day', text: day };
    yield* messages;
  }
}

function FormatDate(d) {
  return d
    ?.toJSON()
    ?.replaceAll(/([TZ]|\.\d\d\d\s*)/g, ' ')
    ?.trimEnd();
}

// Stable anchor slug from a day's text, e.g. "23 March 2024" → "23-March-2024".
function DayAnchor(text) {
  return text
    .trim()
    .replace(/\s+/g, '-')
    .replace(/[^A-Za-z0-9\-]/g, '');
}

export function* GenerateBlog(messages) {
  try {
    let prevMsg,
      i = 0;

    for(const msg of messages) {
      const { type, text, link, thumb, date } = msg;

      if(type == 'day' && prevMsg && prevMsg.type == 'day') continue;
      prevMsg = msg;

      const o = { type };

      if(date) o.date = FormatDate(date);
      if(text) o.text = text;
      if(link) o.link = basename(link);

      if(DEBUG) console.log(`Generate blog message #${i}`, inspect(o, { colors: true, compact: true, breakLength: Infinity }));

      switch (type) {
        case 'day':
          yield { tagName: 'a', attributes: { name: DayAnchor(text) }, children: [''] };
          yield { tagName: 'h4', children: [TextTransform(text)] };
          break;

        case 'text':
          const lines = text.split(/[\n\r]+/g).map(l => TextTransform(l));

          const children = lines.reduce((a, l) => (Array.isArray(a) ? [...a, { tagName: 'br' }, l] : [l]), undefined);

          yield { tagName: 'p', children };
          break;

        case 'photo':
          const id = +link.replace(/.*photo_(\d+)@.*/g, '$1');
          const input = [link, thumb].map(p => join(InputDir(), p));
          const output = [link, thumb].map(p => join(PHOTOS_DIR, basename(p)));

          const isNew = output.every(p => !existsSync(p));

          if(isNew) console.log(`New photo '${output[0]}'`);

          if(id in PHOTO_PROPERTIES && PHOTO_PROPERTIES[id].rotate) {
            const { rotate } = PHOTO_PROPERTIES[id];
            const options = ['-rotate', rotate + '', '-quality', '99'];

            Convert(input[0], output[0], options);
            Convert(input[1], output[1], options);
          } else {
            Copy(input[0], output[0]);
            Copy(input[1], output[1]);
          }

          const { width, height, format } = Identify(output[1]);

          yield {
            tagName: 'img',
            attributes: {
              src: output[1],
              'data-fullsize': output[0],
              class: format == 'landscape' ? 'landscape' : 'bywidth',
              onclick: 'ShowPhoto(this);',
              width,
              height,
            },
          };

          break;
      }

      ++i;
    }
  } catch(e) {
    console.log('GenerateBlog', e);
    throw e;
  }
}

export function OutputBlog(iterator) {
  return {
    tagName: '?xml',
    attributes: { version: '1.0', encoding: 'utf-8' },
    children: [
      { tagName: '!DOCTYPE html' },
      {
        tagName: 'html',
        attributes: {},
        children: [
          {
            tagName: 'head',
            attributes: {},
            children: [
              { tagName: 'meta', attributes: { charset: 'utf-8' } },
              { tagName: 'meta', attributes: { name: 'viewport', content: 'width=device-width, initial-scale=1' } },
              { tagName: 'title', attributes: {}, children: [TITLE] },
              { tagName: 'style', attributes: { type: 'text/css' }, children: [DEFAULT_CSS] },
              { tagName: 'script', attributes: {}, children: [DEFAULT_JS] },
            ],
          },
          {
            tagName: 'body',
            attributes: {},
            children: [
              {
                tagName: 'div',
                attributes: { 'data-name': 'blog', class: 'blog' },
                children: [
                  { tagName: 'div', attributes: { id: 'header' }, children: [''] },
                  {
                    tagName: 'div',
                    attributes: { id: 'content' },
                    children: [
                      {
                        tagName: 'div',
                        attributes: { id: 'text' },
                        children: [...iterator].reduce((a, e) => {
                          if(a.length && e.tagName != 'a') {
                            a[a.length - 1].children.push(e);
                          } else {
                            a.push({ tagName: 'div', children: [e] });
                          }
                          return a;
                        }, []),
                      },
                    ],
                  },
                ],
              },
            ],
          },
        ],
      },
    ],
  };
}

main(...scriptArgs.slice(1));

function main(...args) {
  const { stdout, stderr } = process;

  globalThis.console = new Console({ stdout, stderr, inspectOptions: { depth: 4, compact: false } });

  const file = InputFileName();

  if(!file) throw new Error('No input file. Pass a Telegram-export HTML path as argv, or place one in ~/Downloads/Telegram Desktop/ChatExport*/');

  const document = new Parser().parseFromFile(file);

  const historyElement = document.querySelector('.history');

  Object.assign(globalThis, {
    file,
    document,
    historyElement,
    toXML,
    InputDir,
    GetId,
    GetMessages,
    GetMessage,
    GenerateBlog,
    OutputBlog,
    Copy,
    TextTransform,
    GetClasses,
    AccumulateDays,
    GenerateDays,
    Message2Element,
  });

  try {
    const messages = [...GetMessages(historyElement.children)];
    const days = AccumulateDays(messages).reverse();

    console.log(`DIAG messages=${messages.length} days=${days.length} historyChildren=${historyElement.children.length}`);
    if(messages.length) console.log(`DIAG first message:`, inspect(messages[0], { compact: true, breakLength: Infinity }));
    if(days.length) console.log(`DIAG first day:`, inspect(days[0][0], { compact: true }), `with ${days[0][1].length} entries`);

    Object.assign(globalThis, { messages, days });

    if(existsSync(OUTPUT_FILE)) unlinkSync(OUTPUT_FILE);

    const iterator = GenerateBlog(GenerateDays(days));
    const items = [...iterator];
    console.log(`DIAG iterator items=${items.length}`);

    const tree = OutputBlog(items);
    const serialized = toXML(tree);
    console.log(`DIAG serialized length=${serialized.length}`);
    console.log(`DIAG serialized head=${serialized.slice(0, 200)}`);

    const output = serialized.replaceAll(/([^>"])\b(\w+:\/\/[^\s<"]+)\b([^<"])/g, (match, p1, p2, p3) => {
      let text = p2;

      if(/\.(pdf|docx?)$/i.test(p2))
        if(p2.length > 30) {
          text = text.replace(/.*\//g, '');
          p1 += '<br />';
          p3 = '<br />' + p3;
        }

      return `${p1}<a href="${p2}">${text}</a>${p3}`;
    });

    console.log(`DIAG output length=${output.length}, writing to ${OUTPUT_FILE}`);
    writeFileSync(OUTPUT_FILE, output);
    console.log(`DIAG wrote OK; on-disk size=${statSync(OUTPUT_FILE).size}`);
  } catch(error) {
    console.log('exception', error);
    throw error;
  }
}
