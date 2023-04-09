/*
 * QuickJS Read Eval Print Loop
 67 * Copyright (c) 2017-2020 Fabrice Bellard
 * Copyright (c) 2017-2020 Charlie Gordon
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
import * as Terminal from './terminal.js';
import filesystem from './lib/filesystem.js';
import { isatty } from 'tty';
import { extendArray } from './lib/misc.js';
import Util from './lib/util.js';

('use strip');

extendArray(Array.prototype);

export default function REPL(title = 'QuickJS') {
  /* add 'os' and 'std' bindings */
  /*globalThis.os = os;
  globalThis.std = std;*/
  const { std, os } = globalThis;
  var input, output;

  let fs = globalThis.fs ? globalThis.fs : filesystem && filesystem.openSync ? filesystem : null;
  if(!fs)
    filesystem.PortableFileSystem(filesystem => {
      fs = filesystem;
      console.log('process', process);
      input = globalThis.process && process.stdin ? process.stdin : std.in;
      output = globalThis.process && process.stderr ? process.stderr : std.out;
      //console.log('', { input, output });
    });
  else {
    input = stdio(0);
    output = stdio(1) ?? process.stdout;
  }

  var tty_set_raw = once(fd => os.ttySetRaw(fd));
  var put_banner = once(() => std.puts('QuickJS - Type "\\h" for this.help\n'));

  /*ttySetRaw(input);
  ttySetRaw(output);
*/
  var thisObj = this;

  const puts = str => std.out.puts(str);
  const flush = () => {
    //   console.log("flush", {flushSync:fs.flushSync+'',output});
    std.out.flush(); // fs.flushSync(output);
  };

  /* XXX: use preprocessor ? */
  var config_numcalc = false; //typeof os.open === 'undefined';
  var has_jscalc = typeof Fraction === 'function';
  var has_bignum = typeof BigFloat === 'function';

  var colors = {
    none: '\x1b[0m',
    black: '\x1b[30m',
    red: '\x1b[31m',
    green: '\x1b[32m',
    yellow: '\x1b[33m',
    blue: '\x1b[34m',
    magenta: '\x1b[35m',
    cyan: '\x1b[36m',
    white: '\x1b[37m',
    gray: '\x1b[30;1m',
    grey: '\x1b[30;1m',
    bright_red: '\x1b[31;1m',
    bright_green: '\x1b[32;1m',
    bright_yellow: '\x1b[33;1m',
    bright_blue: '\x1b[34;1m',
    bright_magenta: '\x1b[35;1m',
    bright_cyan: '\x1b[36;1m',
    bright_white: '\x1b[37;1m'
  };

  var styles;
  if(config_numcalc) {
    styles = {
      default: 'black',
      comment: 'white',
      string: 'green',
      regex: 'cyan',
      number: 'green',
      keyword: 'blue',
      function: 'gray',
      type: 'bright_magenta',
      identifier: 'yellow',
      error: 'bright_red',
      result: 'black',
      error_msg: 'bright_red'
    };
  } else {
    styles = {
      default: 'bright_cyan',
      comment: 'bright_green',
      string: 'bright_cyan',
      regex: 'bright_magenta',
      number: 'bright_cyan',
      keyword: 'bright_red',
      function: 'bright_yellow',
      type: 'bright_red',
      identifier: 'bright_yellow',
      error: 'red',
      result: 'white',
      error_msg: 'bright_red'
    };
  }
  var repl = this instanceof REPL ? this : {};

  repl.puts = puts;
  repl.flush = flush;

  var running = true;
  var history = [];
  var clip_board = '';
  var prec;
  var expBits;
  var log2_10;

  var pstate = '';
  var prompt = '';
  var plen = 0;
  var ps1;
  if(config_numcalc) ps1 = '> ';
  else ps1 = `${title.replace(/-.*/g, '')} > `;
  var ps2 = '  ... ';
  var utf8 = true;
  var showTime = false;
  var showColors = true;
  var evalTime = 0;

  var mexpr = '';
  var level = 0;
  //var cmd = '';
  var cursorPos = 0;
  var lastCmd = '';
  var lastCursorPos = 0;
  var historyIndex;
  var thisFun, lastFun;
  var quoteFlag = false;
  var search = 0;
  var search_pattern = '';
  var search_index,
    search_matches = [];

  var utf8_state = 0;
  var utf8_val = 0;

  var term_read_buf;
  var termWidth;
  /* current X position of the cursor in the terminal */
  var term_cursor_x = 0;

  var readline_keys;
  repl.readline_state = 0;
  var readline_cb;

  var hex_mode = false;
  var eval_mode = 'std';

  var commands = {
    /* command table */ '\x01': beginningOfLine /* ^A - bol */,
    '\x02': backwardChar /* ^B - backward-char */,
    '\x03': controlC /* ^C - abort */,
    '\x04': controlD /* ^D - delete-char or exit */,
    '\x05': endOfLine /* ^E - eol */,
    '\x06': forwardChar /* ^F - forward-char */,
    '\x07': abort /* ^G - bell */,
    '\x08': backwardDeleteChar /* ^H - backspace */,
    '\x09': completion /* ^I - history-search-backward */,
    '\x0a': acceptLine /* ^J - newline */,
    '\x0b': killLine /* ^K - delete to end of line */,
    '\x0d': acceptLine /* ^M - enter */,
    '\x0e': historyNext /* ^N - down */,
    '\x10': historyPrevious /* ^P - up */,
    '\x11': quotedInsert /* ^Q - quoted-insert */,
    '\x12': reverseSearch /* ^R - reverse-search */,
    '\x13': forwardSearch /* ^S - search */,
    '\x14': transposeChars /* ^T - transpose */,
    '\x18': reset /* ^X - cancel */,
    '\x19': yank /* ^Y - yank */,
    '\x1bOA': historyPrevious /* ^[OA - up */,
    '\x1bOB': historyNext /* ^[OB - down */,
    '\x1bOC': forwardChar /* ^[OC - right */,
    '\x1bOD': backwardChar /* ^[OD - left */,
    '\x1bOF': forwardWord /* ^[OF - ctrl-right */,
    '\x1bOH': backwardWord /* ^[OH - ctrl-left */,
    '\x1b[1;5C': forwardWord /* ^[[1;5C - ctrl-right */,
    '\x1b[1;5D': backwardWord /* ^[[1;5D - ctrl-left */,
    '\x1b[1~': beginningOfLine /* ^[[1~ - bol */,
    '\x1b[3~': deleteChar /* ^[[3~ - delete */,
    '\x1b[4~': endOfLine /* ^[[4~ - eol */,
    '\x1b[5~': historySearchBackward /* ^[[5~ - page up */,
    '\x1b[6~': historySearchForward /* ^[[5~ - page down */,
    '\x1b[A': historyPrevious /* ^[[A - up */,
    '\x1b[B': historyNext /* ^[[B - down */,
    '\x1b[C': forwardChar /* ^[[C - right */,
    '\x1b[D': backwardChar /* ^[[D - left */,
    '\x1b[F': endOfLine /* ^[[F - end */,
    '\x1b[H': beginningOfLine /* ^[[H - home */,
    '\x1b\x7f': backwardKillWord /* M-C-? - backwardKillWord */,
    '\x1bb': backwardWord /* M-b - backwardWord */,
    '\x1bd': killWord /* M-d - killWord */,
    '\x1bf': forwardWord /* M-f - backwardWord */,
    '\x1bk': backwardKillLine /* M-k - backwardKillLine */,
    '\x1bl': downcaseWord /* M-l - downcaseWord */,
    '\x1bt': transposeWords /* M-t - transposeWords */,
    '\x1bu': upcaseWord /* M-u - upcaseWord */,
    '\x7f': backwardDeleteChar /* ^? - delete */
  };

  let currentCommand = '';

  Object.defineProperties(repl, {
    cmd: {
      get() {
        return currentCommand;
      },
      set(value) {
        currentCommand = value;
      },
      enumerable: false
    },
    history: { value: history, enumerable: false }
  });

  async function termInit() {
    //   repl.debug("termInit");
    var tab;
    this.term_fd = input && input.fileno ? input.fileno() : 1;
    /* get the terminal size */
    termWidth = 80;
    if(isatty(this.term_fd)) {
      if(ttyGetWinSize) {
        await ttyGetWinSize(1).then(tab => {
          repl.debug('termInit', { tab });
          termWidth = tab[0];
        });
      }
      tty_set_raw(this.term_fd);
      repl.debug('TTY setup done');
    }

    /* install a Ctrl-C signal handler */
    signal('SIGINT', sigintHandler);

    /* install a handler to read stdin */
    term_read_buf = new Uint8Array(1);

    setReadHandler(input ?? this.term_fd, () => repl.termReadHandler());

    repl.debug('termInit');
    repl.debug('this.term_fd', this.term_fd);
  }

  function sigintHandler() {
    /* send Ctrl-C to readline */
    repl.handleByte(3);
  }

  function termReadHandler() {
    var l, i;
    repl.debug('term_read_buf', term_read_buf);
    const { buffer } = term_read_buf;
    l = fs.readSync(input, buffer, 0, 1);
    repl.debug('termReadHandler', { l, buffer });

    for(i = 0; i < l; i++) {
      repl.debug('termReadHandler', {
        code: term_read_buf[i],
        char: String.fromCharCode(term_read_buf[i])
      });
      repl.handleByte(term_read_buf[i]);

      if(!running) break;
    }
  }

  function handleByte(c) {
    repl.debug('handleByte', { c, utf8 });
    if(!utf8) {
      repl.handleChar(c);
    } else if(utf8_state !== 0 && c >= 0x80 && c < 0xc0) {
      utf8_val = (utf8_val << 6) | (c & 0x3f);
      utf8_state--;
      if(utf8_state === 0) {
        repl.handleChar(utf8_val);
      }
    } else if(c >= 0xc0 && c < 0xf8) {
      utf8_state = 1 + (c >= 0xe0) + (c >= 0xf0);
      utf8_val = c & ((1 << (6 - utf8_state)) - 1);
    } else {
      utf8_state = 0;
      repl.handleChar(c);
    }
  }

  function isAlpha(c) {
    return typeof c === 'string' && ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'));
  }

  function isDigit(c) {
    return typeof c === 'string' && c >= '0' && c <= '9';
  }

  function isWord(c) {
    return typeof c === 'string' && (repl.isAlpha(c) || repl.isDigit(c) || c == '_' || c == '$');
  }

  function ucsLength(str) {
    var len,
      c,
      i,
      str_len = str.length;
    len = 0;
    /* we never count the trailing surrogate to have the
         following property: repl.ucsLength(str) =
         repl.ucsLength(str.substring(0, a)) + repl.ucsLength(str.substring(a,
         str.length)) for 0 <= a <= str.length */
    for(i = 0; i < str_len; i++) {
      c = str.charCodeAt(i);
      if(c < 0xdc00 || c >= 0xe000) len++;
    }
    return len;
  }

  function isTrailingSurrogate(c) {
    var d;
    if(typeof c !== 'string') return false;
    d = c.codePointAt(0); /* can be NaN if empty string */
    return d >= 0xdc00 && d < 0xe000;
  }

  function isBalanced(a, b) {
    switch (a + b) {
      case '()':
      case '[]':
      case '{}':
        return true;
    }
    return false;
  }

  function printColorText(str, start, style_names) {
    var i, j;
    for(j = start; j < str.length; ) {
      var style = style_names[(i = j)];
      while(++j < str.length && style_names[j] == style) continue;
      puts(colors[styles[style] || 'default']);
      puts(str.substring(i, j));
      puts(colors['none']);
    }
  }

  function printCsi(n, code) {
    puts('\x1b[' + (n != 1 ? n : '') + code);
  }

  /* XXX: handle double-width characters */
  function moveCursor(delta) {
    //if(isNaN(delta)) return;

    var i, l;
    if(delta > 0) {
      while(delta != 0) {
        if(term_cursor_x == termWidth - 1) {
          puts('\n'); /* translated to CRLF */
          term_cursor_x = 0;
          delta--;
        } else {
          l = Math.min(termWidth - 1 - term_cursor_x, delta);
          repl.printCsi(l, 'C'); /* right */
          delta -= l;
          term_cursor_x += l;
        }
      }
    } else {
      delta = -delta;
      while(delta != 0) {
        if(term_cursor_x == 0) {
          repl.printCsi(1, 'A'); /* up */
          repl.printCsi(termWidth - 1, 'C'); /* right */
          delta--;
          term_cursor_x = termWidth - 1;
        } else {
          // repl.debug("moveCursor", {delta,term_cursor_x, termWidth});
          l = Math.min(delta, term_cursor_x);

          if(isNaN(l)) throw new Error(`moveCursor l=${l}`);
          repl.printCsi(l, 'D'); /* left */
          delta -= l;
          term_cursor_x -= l;
        }
      }
    }
  }

  function update() {
    var i,
      cmd_len,
      cmd_line = repl.cmd,
      colorize = showColors;

    if(search) {
      const re = new RegExp((search_pattern = cmd_line.replace(/([\(\)\?\+\*])/g, '.' /*'\\$1'*/)), 'i');
      const num = search > 0 ? search - 1 : search;
      //search_index = history.findLastIndex(c => re.test(c) && --num == 0);
      let historySearch = [...history.entries()].rotateLeft(historyIndex);
      search_matches.splice(0, search_matches.length, ...historySearch.filter(([i, c]) => re.test(c)));
      //num = search > 0 ? search - 1 : search;
      const match = search_matches.at(num);
      const [histidx = -1, histcmd = ''] = match || [];
      const histdir = search > 0 ? 'forward' : 'reverse';
      const histpos = search < 0 ? historySearch.indexOf(match) - historySearch.length : historySearch.indexOf(match);
      search_index = histidx;
      let line_start = `(${histdir}-search[${histpos}])\``;
      cmd_line = `${line_start}${repl.cmd}': ${histcmd}`;
      colorize = false;
      const start = cmd_line.length - histcmd.length - 3 - repl.cmd.length + cursorPos;
      let r = cmd_line.substring(start);

      puts(`\x1b[1G`);
      puts(cmd_line);
      puts('\x1b[J');
      lastCmd = cmd_line;
      term_cursor_x = start;
      lastCursorPos = cursorPos;
      let nback = repl.ucsLength(r);
      std.out.flush();
      if(isNaN(nback)) throw new Error(`update nback=${nback}`);
      puts(`\x1b[${nback}D`);
      //repl.moveCursor(-repl.ucsLength(r));
      std.out.flush();
      return;
    } else if(cmd_line != lastCmd) {
      /* cursorPos is the position in 16 bit characters inside the
           UTF-16 string 'cmd_line' */
      if(!colorize && lastCmd.substring(0, lastCursorPos) == cmd_line.substring(0, lastCursorPos)) {
        /* optimize common case */
        puts(cmd_line.substring(lastCursorPos));
      } else {
        /* goto the start of the line */
        // repl.debug("lastCmd",lastCmd, lastCursorPos);
        const leading = lastCmd.substring(0, lastCursorPos);
        //repl.debug("leading",leading);
        const move_x = -repl.ucsLength(leading);
        //repl.debug("move_x",move_x);
        repl.moveCursor(move_x);

        if(colorize) {
          var str = mexpr ? mexpr + '\n' + cmd_line : cmd_line;
          var start = str.length - cmd_line.length;
          var colorstate = repl.colorizeJs(str);
          repl.printColorText(str, start, colorstate[2]);
        } else {
          puts(cmd_line);
        }
      }
      term_cursor_x = (term_cursor_x + repl.ucsLength(cmd_line)) % termWidth;
      if(term_cursor_x == 0) {
        /* show the cursor on the next line */
        puts(' \x08');
      }
      /* remove the trailing characters */
      puts('\x1b[J');
      lastCmd = cmd_line;
      lastCursorPos = cmd_line.length;
    }
    if(cursorPos > lastCursorPos) {
      repl.moveCursor(repl.ucsLength(cmd_line.substring(lastCursorPos, cursorPos)));
    } else if(cursorPos < lastCursorPos) {
      repl.moveCursor(-repl.ucsLength(cmd_line.substring(cursorPos, lastCursorPos)));
    }
    lastCursorPos = cursorPos;
    //console.log('\nrepl', repl.flush + '');
    //console.log('fs', fs.flushSync, { output });
    repl.flush();
    //    fs.flushSync(output);
  }

  /* editing commands */
  function insert(str) {
    if(str) {
      repl.cmd = repl.cmd.substring(0, cursorPos) + str + repl.cmd.substring(cursorPos);
      cursorPos += str.length;
    }
  }

  function quotedInsert() {
    quoteFlag = true;
  }

  function abort() {
    repl.cmd = '';
    cursorPos = 0;
    return -2;
  }

  function alert() {}

  function reverseSearch() {
    if(search == 0) repl.cmd = '';
    search--;
    readline_cb = searchCb;
    repl.debug('reverseSearch', { search, cursorPos, term_cursor_x });
    repl.update();
    return -2;
  }
  function forwardSearch() {
    if(search == 0) repl.cmd = '';
    search++;
    readline_cb = searchCb;
    //repl.debug('forwardSearch', { search, cursorPos, term_cursor_x });
    repl.update();
    return -2;
  }

  function searchCb(pattern) {
    if(pattern !== null) {
      const histcmd = history[search_index];
      //repl.debug('searchCb', { pattern, histcmd,cmd }, history.slice(-2));
      search = 0;
      readline_cb = readlineHandleCmd;
      repl.cmd = '';
      repl.readlineHandleCmd(histcmd ?? '');

      repl.update();
    }
  }

  function beginningOfLine() {
    cursorPos = 0;
  }

  function endOfLine() {
    cursorPos = repl.cmd.length;
  }

  function forwardChar() {
    if(cursorPos < repl.cmd.length) {
      cursorPos++;
      while(repl.isTrailingSurrogate(repl.cmd.charAt(cursorPos))) cursorPos++;
    }
  }

  function backwardChar() {
    if(cursorPos > 0) {
      cursorPos--;
      while(repl.isTrailingSurrogate(repl.cmd.charAt(cursorPos))) cursorPos--;
    }
  }

  function skipWordForward(pos) {
    while(pos < repl.cmd.length && !repl.isWord(repl.cmd.charAt(pos))) pos++;
    while(pos < repl.cmd.length && repl.isWord(repl.cmd.charAt(pos))) pos++;
    return pos;
  }

  function skipWordBackward(pos) {
    while(pos > 0 && !repl.isWord(repl.cmd.charAt(pos - 1))) pos--;
    while(pos > 0 && repl.isWord(repl.cmd.charAt(pos - 1))) pos--;
    return pos;
  }

  function forwardWord() {
    cursorPos = repl.skipWordForward(cursorPos);
  }

  function backwardWord() {
    cursorPos = repl.skipWordBackward(cursorPos);
  }

  function acceptLine() {
    puts('\n');
    repl.historyAdd(search ? history[search_index] : repl.cmd);
    repl.debug('acceptLine', { cmd: repl.cmd, historyIndex, search, history_length: history.length }, [...history.entries()].slice(historyIndex - 3, historyIndex + 2));
    return -1;
  }

  function historySet(entries) {
    if(entries) history.splice(0, history.length, ...entries);
  }

  /* function historyGet() {
    return history;
  }*/
  function historyPos() {
    return historyIndex;
  }

  function historyAdd(str) {
    //repl.debug('historyAdd', { str });

    if(str) {
      str = str.trim();
      history.push(str);
    }
    historyIndex = history.length;
  }

  function numLines(str) {
    return str.split(/\n/g).length;
  }
  function lastLine(str) {
    let lines = str.split(/\n/g);
    return lines[lines.length - 1];
  }

  function historyPrevious() {
    let numLines = repl.cmd.split(/\n/g).length;

    if(numLines > 1) repl.readlineClear();
    search = 0;

    if(historyIndex > 0) {
      if(historyIndex == history.length) {
        history.push(repl.cmd);
      }
      historyIndex--;
      repl.cmd = history[historyIndex];
      cursorPos = repl.cmd.length;
    }
  }

  function historyNext() {
    let numLines = repl.cmd.split(/\n/g).length;

    if(numLines > 1) repl.readlineClear();
    search = 0;

    if(historyIndex < history.length - 1) {
      historyIndex++;
      repl.cmd = history[historyIndex];
      cursorPos = repl.cmd.length;
    }
  }

  function historySearch(dir) {
    var pos = cursorPos;
    for(var i = 1; i <= history.length; i++) {
      var index = (history.length + i * dir + historyIndex) % history.length;
      if(history[index].substring(0, pos) == repl.cmd.substring(0, pos)) {
        historyIndex = index;
        repl.cmd = history[index];
        return;
      }
    }
  }

  function historySearchBackward() {
    return repl.historySearch(-1);
  }

  function historySearchForward() {
    return repl.historySearch(1);
  }

  function deleteCharDir(dir) {
    var start, end;

    start = cursorPos;
    if(dir < 0) {
      start--;
      while(repl.isTrailingSurrogate(repl.cmd.charAt(start))) start--;
    }
    end = start + 1;
    while(repl.isTrailingSurrogate(repl.cmd.charAt(end))) end++;

    if(start >= 0 && start < repl.cmd.length) {
      if(lastFun === killRegion) {
        repl.killRegion(start, end, dir);
      } else {
        repl.cmd = repl.cmd.substring(0, start) + repl.cmd.substring(end);
        cursorPos = start;
      }
    }
  }

  function deleteChar() {
    repl.deleteCharDir(1);
  }

  function controlD() {
    if(repl.cmd.length == 0) {
      puts('\n');

      (repl.cleanup ?? std.exit)(0);

      return -3; /* exit read eval print loop */
    } else {
      repl.deleteCharDir(1);
    }
  }

  function backwardDeleteChar() {
    repl.deleteCharDir(-1);
  }

  function transposeChars() {
    var pos = cursorPos;
    if(repl.cmd.length > 1 && pos > 0) {
      if(pos == repl.cmd.length) pos--;
      repl.cmd = repl.cmd.substring(0, pos - 1) + repl.cmd.substring(pos, pos + 1) + repl.cmd.substring(pos - 1, pos) + repl.cmd.substring(pos + 1);
      cursorPos = pos + 1;
    }
  }

  function transposeWords() {
    var p1 = repl.skipWordBackward(cursorPos);
    var p2 = repl.skipWordForward(p1);
    var p4 = repl.skipWordForward(cursorPos);
    var p3 = repl.skipWordBackward(p4);

    if(p1 < p2 && p2 <= cursorPos && cursorPos <= p3 && p3 < p4) {
      repl.cmd = repl.cmd.substring(0, p1) + repl.cmd.substring(p3, p4) + repl.cmd.substring(p2, p3) + repl.cmd.substring(p1, p2);
      cursorPos = p4;
    }
  }

  function upcaseWord() {
    var end = repl.skipWordForward(cursorPos);
    repl.cmd = repl.cmd.substring(0, cursorPos) + repl.cmd.substring(cursorPos, end).toUpperCase() + repl.cmd.substring(end);
  }

  function downcaseWord() {
    var end = repl.skipWordForward(cursorPos);
    repl.cmd = repl.cmd.substring(0, cursorPos) + repl.cmd.substring(cursorPos, end).toLowerCase() + repl.cmd.substring(end);
  }

  function killRegion(start, end, dir) {
    var s = repl.cmd.substring(start, end);
    if(lastFun !== killRegion) clip_board = s;
    else if(dir < 0) clip_board = s + clip_board;
    else clip_board = clip_board + s;

    repl.cmd = repl.cmd.substring(0, start) + repl.cmd.substring(end);
    if(cursorPos > end) cursorPos -= end - start;
    else if(cursorPos > start) cursorPos = start;
    thisFun = killRegion;
  }

  function killLine() {
    repl.killRegion(cursorPos, repl.cmd.length, 1);
  }

  function backwardKillLine() {
    repl.killRegion(0, cursorPos, -1);
  }

  function killWord() {
    repl.killRegion(cursorPos, repl.skipWordForward(cursorPos), 1);
  }

  function backwardKillWord() {
    repl.killRegion(repl.skipWordBackward(cursorPos), cursorPos, -1);
  }

  function yank() {
    repl.insert(clip_board);
  }

  function controlC() {
    if(lastFun === controlC) {
      puts('\n');

      running = false;
      (repl.cleanup ?? std.exit)(0);
    } else {
      puts('\n(Press Ctrl-C again to quit)\n');
      repl.cmd = '';
      repl.readlinePrintPrompt();
    }
  }

  function reset() {
    repl.cmd = '';
    cursorPos = 0;
  }

  function getContextWord(line, pos) {
    var s = '';
    while(pos > 0 && repl.isWord(line[pos - 1])) {
      pos--;
      s = line[pos] + s;
    }
    return s;
  }
  function getContextObject(line, pos) {
    var obj, base, c;
    if(pos <= 0 || ' ~!%^&*(-+={[|:;,<>?/'.indexOf(line[pos - 1]) >= 0) return globalThis;
    if(pos >= 2 && line[pos - 1] === '.') {
      pos--;
      obj = {};
      switch ((c = line[pos - 1])) {
        case "'":
        case '"':
          return 'a';
        case ']':
          return [];
        case '}':
          return {};
        case '/':
          return /\ /;
        default:
          if(repl.isWord(c)) {
            base = repl.getContextWord(line, pos);
            if(['true', 'false', 'null', 'this'].includes(base) || !isNaN(+base)) return eval(base);
            obj = repl.getContextObject(line, pos - base.length);
            if(obj === null || obj === void 0) return obj;
            if(obj === globalThis && obj[base] === void 0) {
              let ret;
              try {
                ret = eval(base);
              } catch(e) {}
              return ret;
            } else return obj[base];
          }
          return {};
      }
    }
    return void 0;
  }

  function getDirectoryEntries(pathStr = '.', mask = '*') {
    let dir, base;
    let stat = fs.stat(pathStr);
    if(stat && stat?.isDirectory() && pathStr.endsWith('/')) {
      dir = pathStr;
      base = '';
    } else {
      dir = path.dirname(pathStr);
      base = path.basename(pathStr);
    }
    let expr = mask.replace(/\./g, '\\.').replace(/\*/g, '.*');
    expr = (mask.startsWith('*') ? '' : '^') + expr + (mask.endsWith('*') ? '' : '$');
    let re = new RegExp(expr);
    let entries = fs
      .readdir(dir)
      .map(entry => {
        let st = fs.stat(path.join(dir, entry));
        return entry + (st && st.isDirectory() ? '/' : '');
      })
      .sort((a, b) => b.endsWith('/') - a.endsWith('/') || a.localeCompare(b));
    entries = entries.filter(entry => re.test(entry) || entry.endsWith('/'));
    if(base != '') entries = entries.filter(entry => entry.startsWith(base));
    return entries.map(entry => path.join(dir, entry));
  }

  function getFilenameCompletions(line, pos) {
    let s = line.slice(0, pos).replace(/^\\[^ ]?\s*/, '');

    //  let s = repl.getContextWord(line, pos);
    //repl.debug('getFilenameCompletions', { line, pos, s });

    let mask = '*';

    if(line.startsWith('\\i')) mask = '*.(js|so)';
    let tab = repl.getDirectoryEntries(s, mask);
    return { tab, pos: s.length, ctx: {} };
  }

  function getCompletions(line, pos) {
    var s, obj, ctx_obj, r, i, j, paren;

    if(/\\[il]/.test(repl.cmd)) return repl.getFilenameCompletions(line, pos);

    s = repl.getContextWord(line, pos);
    //    repl.printStatus('getCompletions', { line, pos, repl.cmd, word: s });

    ctx_obj = repl.getContextObject(line, pos - s.length);

    r = [];
    /* enumerate properties from object and its prototype chain,
           add non-numeric regular properties with s as e prefix
         */
    for(i = 0, obj = ctx_obj; i < 10 && obj !== null && obj !== void 0; i++) {
      var props = Object.getOwnPropertyNames(obj);
      /* add non-numeric regular properties */
      for(j = 0; j < props.length; j++) {
        var prop = props[j];
        if(typeof prop == 'string' && '' + +prop != prop && prop.startsWith(s)) r.push(prop);
      }
      obj = Object.getPrototypeOf(obj);
    }
    if(r.length > 1) {
      /* sort list with internal names last and remove duplicates */
      function symcmp(a, b) {
        if(a[0] != b[0]) {
          if(a[0] == '_') return 1;
          if(b[0] == '_') return -1;
        }
        if(a < b) return -1;
        if(a > b) return +1;
        return 0;
      }
      r.sort(symcmp);
      for(i = j = 1; i < r.length; i++) {
        if(r[i] != r[i - 1]) r[j++] = r[i];
      }
      r.length = j;
    }
    /* 'tab' = list of completions, 'pos' = cursor position inside
           the completions */
    return { tab: r, pos: s.length, ctx: ctx_obj };
  }

  function completion() {
    var tab, res, s, i, j, len, t, max_width, col, n_cols, row, n_rows;
    res = repl.getCompletions(repl.cmd, cursorPos);
    tab = res.tab;
    if(tab.length === 0) return;
    s = tab[0];
    len = s.length;
    /* add the chars which are identical in all the completions */
    for(i = 1; i < tab.length; i++) {
      t = tab[i];
      for(j = 0; j < len; j++) {
        if(t[j] !== s[j]) {
          len = j;
          break;
        }
      }
    }
    for(i = res.pos; i < len; i++) {
      repl.insert(s[i]);
    }
    if(lastFun === completion && tab.length == 1) {
      /* append parentheses to function names */
      var m = res.ctx[tab[0]];
      if(typeof m == 'function') {
        repl.insert('(');
        if(m.length == 0) repl.insert(')');
      } else if(typeof m == 'object') {
        repl.insert('.');
      }
    }
    /* show the possible completions */
    if(lastFun === completion && tab.length >= 2) {
      max_width = 0;
      for(i = 0; i < tab.length; i++) max_width = Math.max(max_width, tab[i].length);
      max_width += 2;
      n_cols = Math.max(1, Math.floor((termWidth + 1) / max_width));
      n_rows = Math.ceil(tab.length / n_cols);
      puts('\n');
      /* display the sorted list column-wise */
      for(row = 0; row < n_rows; row++) {
        for(col = 0; col < n_cols; col++) {
          i = col * n_rows + row;
          if(i >= tab.length) break;
          s = tab[i];
          if(col != n_cols - 1) s = s.padEnd(max_width);
          puts(s);
        }
        puts('\n');
      }
      /* show a new prompt */
      repl.readlinePrintPrompt();
    }
  }

  function dupstr(str, count) {
    var res = '';
    while(count-- > 0) res += str;
    return res;
  }

  function readlineClear() {
    const numLines = (repl.cmd ?? '').split(/\n/g).length;

    if(numLines > 1) Terminal.cursorUp(numLines - 1);

    Terminal.cursorHome();
    Terminal.eraseInDisplay();
    repl.readlinePrintPrompt();
  }

  function readlinePrintPrompt() {
    repl.puts('\r\x1b[K');
    repl.puts(prompt);
    term_cursor_x = repl.ucsLength(prompt) % termWidth;
    lastCmd = '';
    lastCursorPos = 0;
  }

  function readlineStart(defstr, cb) {
    repl.debug('readlineStart', { defstr, cb });
    let a = (defstr || '').split(/\n/g);
    mexpr = a.slice(0, -1).join('\n');
    repl.cmd = a[a.length - 1];

    cursorPos = repl.cmd.length;
    historyIndex = history.length;
    readline_cb = cb;

    prompt = pstate;

    if(mexpr) {
      prompt += repl.dupstr(' ', plen - prompt.length);
      prompt += ps2;
    } else {
      if(showTime) {
        var t = Math.round(evalTime) + ' ';
        evalTime = 0;
        t = repl.dupstr('0', 5 - t.length) + t;
        prompt += t.substring(0, t.length - 4) + '.' + t.substring(t.length - 4);
      }
      plen = prompt.length;
      prompt += ps1;
    }
    repl.readlinePrintPrompt();
    repl.update();
    repl.readline_state = 0;
  }

  function handleChar(c1) {
    var c;
    repl.debug('handleChar', {
      c1,
      state: repl.readline_state,
      readline_keys
    });
    c = String.fromCodePoint(c1);

    for(;;) {
      switch (repl.readline_state) {
        case 0:
          if(c == '\x1b') {
            /* '^[' - ESC */
            readline_keys = c;
            repl.readline_state = 1;
          } else {
            repl.handleKey(c);
          }
          break;
        case 1 /* '^[ */:
          readline_keys += c;
          if(c == '[') {
            repl.readline_state = 2;
          } else if(c == 'O') {
            repl.readline_state = 3;
          } else {
            repl.handleKey(readline_keys);
            repl.readline_state = 0;
          }
          break;
        case 2 /* '^[[' - CSI */:
          readline_keys += c;

          if(c == '<') {
            repl.readline_state = 4;
          } else if(!(c == ';' || (c >= '0' && c <= '9'))) {
            repl.handleKey(readline_keys);
            repl.readline_state = 0;
          }
          break;

        case 3 /* '^[O' - ESC2 */:
          readline_keys += c;
          repl.handleKey(readline_keys);
          repl.readline_state = 0;
          break;

        case 4:
          if(!(c == ';' || (c >= '0' && c <= '9') || c == 'M' || c == 'm')) {
            repl.handleMouse(readline_keys);

            repl.readline_state = 0;
            readline_keys = '';
            continue;
          } else {
            readline_keys += c;
          }

          break;
      }
      break;
    }
  }

  function handleMouse(keys) {
    const [button, x, y, cmd] = [...Util.matchAll(/([0-9]+|[A-Za-z]+)/g, keys)].map(p => p[1]).map(p => (!isNaN(+p) ? +p : p));
    let press = cmd == 'm';

    repl.debug('handleMouse', { button, x, y, press });
  }

  function handleKey(keys) {
    var fun;
    repl.debug('handleKey:', { keys });

    if(quoteFlag) {
      if(repl.ucsLength(keys) === 1) repl.insert(keys);
      quoteFlag = false;
    } else if((fun = commands[keys])) {
      const ret = fun(keys);
      repl.debug('handleKey', {
        keys,
        fun,
        ret,
        cmd: repl.cmd,
        historyIndex
      });
      thisFun = fun;
      switch (ret) {
        case -1:
          readline_cb(repl.cmd);
          return;
        case -2:
          readline_cb(null);
          return;
        case -3:
          /* uninstall a Ctrl-C signal handler */
          signal('SIGINT', null);
          /* uninstall the stdin read handler */
          fs.setReadHandler(this.term_fd, null);
          return;
        default:
          if(
            search &&
            [
              acceptLine,
              backwardChar,
              backwardDeleteChar,
              backwardKillLine,
              backwardKillWord,
              backwardWord,
              beginningOfLine,
              deleteChar,
              //endOfLine,
              forwardChar,
              forwardWord,
              killLine,
              killWord
            ].indexOf(fun) == -1
          ) {
            const histcmd = history[search_index];

            //readline_cb = readlineHandleCmd;
            search = 0;
            //repl.cmd = histcmd;
            puts(`\x1b[1G`);
            puts(`\x1b[J`);
            cursorPos = histcmd.length;
            repl.readlineStart(histcmd, readlineHandleCmd);
            repl.update();
            return;
          }
          break;
      }
      lastFun = thisFun;
    } else if(repl.ucsLength(keys) === 1 && keys >= ' ') {
      repl.insert(keys);
      lastFun = insert;
    } else {
      repl.alert(); /* beep! */
    }

    cursorPos = cursorPos < 0 ? 0 : cursorPos > repl.cmd.length ? repl.cmd.length : cursorPos;
    repl.update();
  }

  function numberToString(a, radix) {
    var s;
    if(!isFinite(a)) {
      /* NaN, Infinite */
      return a.toString();
    } else {
      if(a == 0) {
        if(1 / a < 0) s = '-0';
        else s = '0';
      } else {
        if(radix == 16 && a === Math.floor(a)) {
          var s;
          if(a < 0) {
            a = -a;
            s = '-';
          } else {
            s = '';
          }
          s += '0x' + a.toString(16);
        } else {
          s = a.toString();
        }
      }
      return s;
    }
  }

  function bigfloatToString(a, radix) {
    var s;
    if(!BigFloat.isFinite(a)) {
      /* NaN, Infinite */
      if(eval_mode !== 'math') {
        return 'BigFloat(' + a.toString() + ')';
      } else {
        return a.toString();
      }
    } else {
      if(a == 0) {
        if(1 / a < 0) s = '-0';
        else s = '0';
      } else {
        if(radix == 16) {
          var s;
          if(a < 0) {
            a = -a;
            s = '-';
          } else {
            s = '';
          }
          s += '0x' + a.toString(16);
        } else {
          s = a.toString();
        }
      }
      if(typeof a === 'bigfloat' && eval_mode !== 'math') {
        s += 'l';
      } else if(eval_mode !== 'std' && s.indexOf('.') < 0 && ((radix == 16 && s.indexOf('p') < 0) || (radix == 10 && s.indexOf('e') < 0))) {
        /* add a decimal point so that the floating point type
                   is visible */
        s += '.0';
      }
      return s;
    }
  }

  function bigintToString(a, radix) {
    var s;
    if(radix == 16) {
      var s;
      if(a < 0) {
        a = -a;
        s = '-';
      } else {
        s = '';
      }
      s += '0x' + a.toString(16);
    } else {
      s = a.toString();
    }
    if(eval_mode === 'std') s += 'n';
    return s;
  }

  function print(a) {
    var stack = [];

    function printRec(a) {
      var n, i, keys, key, type, s;

      type = typeof a;
      if(type === 'object') {
        if(a === null) {
          puts(a);
        } else if(stack.indexOf(a) >= 0) {
          puts('[circular]');
        } else if(
          has_jscalc &&
          (a instanceof Fraction || a instanceof Complex || a instanceof Mod || a instanceof Polynomial || a instanceof PolyMod || a instanceof RationalFunction || a instanceof Series)
        ) {
          puts(a.toString());
        } else {
          stack.push(a);
          if(Array.isArray(a)) {
            n = a.length;
            puts('[ ');
            for(i = 0; i < n; i++) {
              if(i !== 0) puts(', ');
              if(i in a) {
                printRec(a[i]);
              } else {
                puts('<empty>');
              }
              if(i > 20) {
                puts('...');
                break;
              }
            }
            puts(' ]');
          } else if(Object.__getClass(a) === 'RegExp') {
            puts(a.toString());
          } else {
            keys = Object.keys(a);
            n = keys.length;
            puts('{ ');
            for(i = 0; i < n; i++) {
              if(i !== 0) puts(', ');
              key = keys[i];
              puts(key, ': ');
              printRec(a[key]);
            }
            puts(' }');
          }
          stack.pop(a);
        }
      } else if(type === 'string') {
        s = a.__quote();
        if(s.length > 79) s = s.substring(0, 75) + '..."';
        puts(s);
      } else if(type === 'number') {
        puts(repl.numberToString(a, hex_mode ? 16 : 10));
      } else if(type === 'bigint') {
        puts(repl.bigintToString(a, hex_mode ? 16 : 10));
      } else if(type === 'bigfloat') {
        puts(repl.bigfloatToString(a, hex_mode ? 16 : 10));
      } else if(type === 'bigdecimal') {
        puts(a.toString() + 'm');
      } else if(type === 'symbol') {
        puts(String(a));
      } else if(type === 'function') {
        puts('function ' + a.name + '()');
      } else {
        puts(a);
      }
    }
    printRec(a);
  }

  function extractDirective(a) {
    var pos;
    if(a[0] !== '\\') return '';
    for(pos = 1; pos < a.length; pos++) {
      if(!repl.isAlpha(a[pos])) break;
    }
    return a.substring(1, pos);
  }

  /* return true if the string after cmd can be evaluted as JS */
  function handleDirective(cmd, expr) {
    var param, prec1, expBits1;
    const [, ...args] = expr.split(/\s+/g);

    if(cmd === 'h' || cmd === '?' || cmd == 'help') {
      repl.help();
    } else if(cmd === 'load') {
      var filename = expr.substring(cmd.length + 1).trim();
      if(filename.lastIndexOf('.') <= filename.lastIndexOf('/')) filename += '.js';
      std.loadScript(filename);
      return false;
    } else if(cmd === 'x') {
      hex_mode = true;
    } else if(cmd === 'd') {
      hex_mode = false;
    } else if(cmd === 't') {
      showTime = !showTime;
    } else if(has_bignum && cmd === 'p') {
      param = expr
        .substring(cmd.length + 1)
        .trim()
        .split(' ');
      if(param.length === 1 && param[0] === '') {
        puts('BigFloat precision=' + prec + ' bits (~' + Math.floor(prec / log2_10) + ' digits), exponent size=' + expBits + ' bits\n');
      } else if(param[0] === 'f16') {
        prec = 11;
        expBits = 5;
      } else if(param[0] === 'f32') {
        prec = 24;
        expBits = 8;
      } else if(param[0] === 'f64') {
        prec = 53;
        expBits = 11;
      } else if(param[0] === 'f128') {
        prec = 113;
        expBits = 15;
      } else {
        prec1 = parseInt(param[0]);
        if(param.length >= 2) expBits1 = parseInt(param[1]);
        else expBits1 = BigFloatEnv.expBitsMax;
        if(Number.isNaN(prec1) || prec1 < BigFloatEnv.precMin || prec1 > BigFloatEnv.precMax) {
          puts('Invalid precision\n');
          return false;
        }
        if(Number.isNaN(expBits1) || expBits1 < BigFloatEnv.expBitsMin || expBits1 > BigFloatEnv.expBitsMax) {
          puts('Invalid exponent bits\n');
          return false;
        }
        prec = prec1;
        expBits = expBits1;
      }
      return false;
    } else if(has_bignum && cmd === 'digits') {
      param = expr.substring(cmd.length + 1).trim();
      prec1 = Math.ceil(parseFloat(param) * log2_10);
      if(prec1 < BigFloatEnv.precMin || prec1 > BigFloatEnv.precMax) {
        puts('Invalid precision\n');
        return false;
      }
      prec = prec1;
      expBits = BigFloatEnv.expBitsMax;
      return false;
    } else if(has_bignum && cmd === 'mode') {
      param = expr.substring(cmd.length + 1).trim();
      if(param === '') {
        puts('Running mode=' + eval_mode + '\n');
      } else if(param === 'std' || param === 'math') {
        eval_mode = param;
      } else {
        puts('Invalid mode\n');
      }
      return false;
    } else if(cmd === 'clear') {
      puts('\x1b[H\x1b[J');
    } else if(cmd === 'q') {
      running = false;
      //(thisObj.exit ?? std.exit)(0);
      return false;
    } else if(cmd === 'i') {
      thisObj
        .importModule(...args)
        .catch(e => {
          repl.printStatus(`ERROR importing:`, e);
          done = true;
        })
        .then(({ moduleName, modulePath, module }) => {
          repl.printStatus(`imported '${moduleName}' from '${modulePath}':`, module);
          done = true;
        });
      /*while(!done) std.sleep(50);*/

      //    repl.debug("handleDirective", {cmd,module,exports});
      return false;
    } else if(has_jscalc && cmd === 'a') {
      algebraicMode = true;
    } else if(has_jscalc && cmd === 'n') {
      algebraicMode = false;
    } else if(repl.directives[cmd]) {
      const handler = repl.directives[cmd];
      return repl.handler(...args) === false ? false : true;
    } else {
      puts('Unknown directive: ' + cmd + '\n');
      return false;
    }
    return true;
  }

  if(config_numcalc) {
    /* called by the GUI */
    globalThis.execCmd = function(cmd) {
      switch (cmd) {
        case 'dec':
          hex_mode = false;
          break;
        case 'hex':
          hex_mode = true;
          break;
        case 'num':
          algebraicMode = false;
          break;
        case 'alg':
          algebraicMode = true;
          break;
      }
    };
  }

  function help() {
    function sel(n) {
      return n ? '*' : ' ';
    }
    puts(
      '\\h          this help\n' +
        '\\x             ' +
        sel(hex_mode) +
        'hexadecimal number display\n' +
        '\\d             ' +
        sel(!hex_mode) +
        'decimal number display\n' +
        '\\t             ' +
        sel(showTime) +
        'toggle timing display\n' +
        '\\clear              clear the terminal\n'
    );

    if(has_jscalc) {
      puts('\\a             ' + sel(algebraicMode) + 'algebraic mode\n' + '\\n             ' + sel(!algebraicMode) + 'numeric mode\n');
    }
    if(has_bignum) {
      puts("\\p [m [e]]       set the BigFloat precision to 'm' bits\n" + "\\digits n   set the BigFloat precision to 'ceil(n*log2(10))' bits\n");
      if(!has_jscalc) {
        puts('\\mode [std|math] change the running mode (current = ' + eval_mode + ')\n');
      }
    }
    puts('\\i [module] import module\n');
    if(!config_numcalc) {
      puts('\\q          exit\n');
    }
  }

  function printStatus(...args) {
    /*puts('\x1b[1S');
    puts('\x1b[1F');*/
    //    puts('\x1b[1G');
    puts('\x1b[1K\r');
    for(let arg of args) repl.show(arg);
    puts('\n');
    //    puts('\x1b[1K\r');
    repl.readlineRemovePrompt();
    repl.readlinePrintPrompt();
    this.out.flush();
  }

  function evalAndPrint(expr) {
    var result;

    try {
      if(eval_mode === 'math') expr = '"use math"; void 0;' + expr;
      var now = new Date().getTime();
      /* eval as a script */

      result = (std?.evalScript ?? eval)(expr, { backtrace_barrier: true });
      evalTime = new Date().getTime() - now;

      repl.printStatus(colors[styles.result], result, '\n', colors.none);
      repl.update();
    } catch(error) {
      let output = `${error.constructor.name || 'EXCEPTION'}: ` + colors[styles.error_msg] + error?.message + '\n';

      //      puts(error.stack+'');

      if(error instanceof Error || typeof error?.message == 'string') {
        repl.debug((error?.type ?? className(error)) + ': ' + error?.message);
        if(error?.stack) output += error?.stack;
      } else {
        output += 'Throw: ' + error;
      }
      output += colors.none;

      puts(output, '\n\r');
    }

    /* set the last result */
    globalThis._ = repl.result = result;
    if(isPromise(result)) {
      result.then(value => {
        result.resolved = true;
        repl.printStatus(`Promise resolved to:`, typeOf(value), console.config({ depth: 1, multiline: true }), value);
        globalThis.$ = value;
      });
    }
  }

  function cmdStart(title) {
    if(repl.help) puts(`${title} - Type "\\h" for help\n`);
    if(has_bignum) {
      log2_10 = Math.log(10) / Math.log(2);
      prec = 113;
      expBits = 15;
      if(has_jscalc) {
        eval_mode = 'math';
        /* XXX: numeric mode should always be the default ? */
        globalThis.algebraicMode = config_numcalc;
      }
    }

    repl.cmdReadlineStart();
  }

  function cmdReadlineStart() {
    repl.readlineStart(repl.dupstr('    ', level), readlineHandleCmd);
  }

  function readlineHandleCmd(expr) {
    let ret = repl.handleCmd(expr);
    //repl.debug('readlineHandleCmd', { expr, mexpr, ret });
    repl.cmdReadlineStart();
  }

  function handleCmd(expr) {
    var colorstate, command;
    if(expr === null || expr === '') {
      expr = '';
      return -1;
    }
    repl.debug('handleCmd', { expr, cmd: repl.cmd });
    if(expr === '?') {
      repl.help();
      return -2;
    }
    command = repl.extractDirective(expr);
    if(command.length > 0) {
      if(!repl.handleDirective(command, expr)) return -3;
      expr = expr.substring(command.length + 1);
    }
    if(expr === '') return -4;

    if(mexpr) expr = mexpr + '\n' + expr;
    colorstate = repl.colorizeJs(expr);
    pstate = colorstate[0];
    level = colorstate[1];
    if(pstate) {
      mexpr = expr;
      return -5;
    }
    mexpr = '';

    if(has_bignum) {
      BigFloatEnv.setPrec(evalAndPrint.bind(null, expr), prec, expBits);
    } else {
      repl.evalAndPrint(expr);
    }
    level = 0;
    let histidx = history?.findLastIndex(entry => expr?.startsWith(entry));

    repl.historyAdd(history.splice(histidx, historyIndex - histidx).join('\n'));
    //repl.debug('handleCmd', {histidx}, history.slice(histidx));
    /* run the garbage collector after each command */
    if(getPlatform() == 'quickjs') std.gc();
  }

  function colorizeJs(str) {
    var i,
      c,
      start,
      n = str.length;
    var style,
      state = '',
      level = 0;
    var primary,
      can_regex = 1;
    var r = [];

    function pushState(c) {
      state += c;
    }
    function lastState(c) {
      return state.substring(state.length - 1);
    }
    function popState(c) {
      var c = lastState();
      state = state.substring(0, state.length - 1);
      return c;
    }

    function parseBlockComment() {
      style = 'comment';
      pushState('/');
      for(i++; i < n - 1; i++) {
        if(str[i] == '*' && str[i + 1] == '/') {
          i += 2;
          popState('/');
          break;
        }
      }
    }

    function parseLineComment() {
      style = 'comment';
      for(i++; i < n; i++) {
        if(str[i] == '\n') {
          break;
        }
      }
    }

    function parseString(delim) {
      style = 'string';
      pushState(delim);
      while(i < n) {
        c = str[i++];
        if(c == '\n') {
          style = 'error';
          continue;
        }
        if(c == '\\') {
          if(i >= n) break;
          i++;
        } else if(c == delim) {
          popState();
          break;
        }
      }
    }

    function parseRegex() {
      style = 'regex';
      pushState('/');
      while(i < n) {
        c = str[i++];
        if(c == '\n') {
          style = 'error';
          continue;
        }
        if(c == '\\') {
          if(i < n) {
            i++;
          }
          continue;
        }
        if(lastState() == '[') {
          if(c == ']') {
            popState();
          }
          //ECMA 5: ignore '/' inside char classes
          continue;
        }
        if(c == '[') {
          pushState('[');
          if(str[i] == '[' || str[i] == ']') i++;
          continue;
        }
        if(c == '/') {
          popState();
          while(i < n && repl.isWord(str[i])) i++;
          break;
        }
      }
    }

    function parseNumber() {
      style = 'number';
      while(i < n && (repl.isWord(str[i]) || (str[i] == '.' && (i == n - 1 || str[i + 1] != '.')))) {
        i++;
      }
    }

    var js_keywords =
      '|' +
      'break|case|catch|continue|debugger|default|delete|do|' +
      'else|finally|for|function|if|in|instanceof|new|' +
      'return|switch|this|throw|try|typeof|while|with|' +
      'class|const|enum|import|export|extends|super|' +
      'implements|interface|let|package|private|protected|' +
      'public|static|yield|' +
      'undefined|null|true|false|Infinity|NaN|' +
      'eval|arguments|' +
      'await|';

    var js_no_regex = '|this|super|undefined|null|true|false|Infinity|NaN|arguments|';
    var js_types = '|void|var|';

    function parseIdentifier() {
      can_regex = 1;

      while(i < n && repl.isWord(str[i])) i++;

      var w = '|' + str.substring(start, i) + '|';

      if(js_keywords.indexOf(w) >= 0) {
        style = 'keyword';
        if(js_no_regex.indexOf(w) >= 0) can_regex = 0;
        return;
      }

      var i1 = i;
      while(i1 < n && str[i1] == ' ') i1++;

      if(i1 < n && str[i1] == '(') {
        style = 'function';
        return;
      }

      if(js_types.indexOf(w) >= 0) {
        style = 'type';
        return;
      }

      style = 'identifier';
      can_regex = 0;
    }

    function setStyle(_from, to) {
      while(r.length < _from) r.push('default');
      while(r.length < to) r.push(style);
    }

    for(i = 0; i < n; ) {
      style = null;
      start = i;
      switch ((c = str[i++])) {
        case ' ':
        case '\t':
        case '\r':
        case '\n':
          continue;
        case '+':
        case '-':
          if(i < n && str[i] == c) {
            i++;
            continue;
          }
          can_regex = 1;
          continue;
        case '/':
          if(i < n && str[i] == '*') {
            //block comment
            parseBlockComment();
            break;
          }
          if(i < n && str[i] == '/') {
            //line comment
            parseLineComment();
            break;
          }
          if(can_regex) {
            parseRegex();
            can_regex = 0;
            break;
          }
          can_regex = 1;
          continue;
        case "'":
        case '"':
        case '`':
          parseString(c);
          can_regex = 0;
          break;
        case '(':
        case '[':
        case '{':
          can_regex = 1;
          level++;
          pushState(c);
          continue;
        case ')':
        case ']':
        case '}':
          can_regex = 0;
          if(level > 0 && repl.isBalanced(lastState(), c)) {
            level--;
            popState();
            continue;
          }
          style = 'error';
          break;
        default:
          if(repl.isDigit(c)) {
            parseNumber();
            can_regex = 0;
            break;
          }
          if(repl.isWord(c) || c == '$') {
            parseIdentifier();
            break;
          }
          can_regex = 1;
          continue;
      }
      if(style) setStyle(start, i);
    }
    setStyle(n, n);
    return [state, level, r];
  }
  Object.assign(repl, { debug() {}, printStatus });
  Object.defineProperties(
    repl,
    Object.fromEntries([
      [
        'termWidth',
        {
          get() {
            return termWidth;
          }
        }
      ],
      [
        'history',
        {
          get() {
            return history;
          }
        }
      ],
      ...Object.entries({
        run,
        runSync,
        termInit,
        sigintHandler,
        termReadHandler,
        handleByte,
        isAlpha,
        isDigit,
        isWord,
        ucsLength,
        isTrailingSurrogate,
        isBalanced,
        printColorText,
        printCsi,
        moveCursor,
        update,
        insert,
        quotedInsert,
        abort,
        alert,
        beginningOfLine,
        endOfLine,
        forwardChar,
        backwardChar,
        skipWordForward,
        skipWordBackward,
        forwardWord,
        backwardWord,
        acceptLine,
        historyGet,
        historyPos,
        historySet,
        historyAdd,
        historyPrevious,
        historyNext,
        historySearch,
        historySearchBackward,
        historySearchForward,
        deleteCharDir,
        deleteChar,
        controlD,
        backwardDeleteChar,
        transposeChars,
        transposeWords,
        upcaseWord,
        downcaseWord,
        killRegion,
        killLine,
        backwardKillLine,
        killWord,
        backwardKillWord,
        yank,
        controlC,
        reset,
        getContextWord,
        getContextObject,
        getCompletions,
        completion,
        dupstr,
        readlinePrintPrompt,
        readlineStart,
        readlineClear,
        handleChar,
        handleKey,
        numberToString,
        bigfloatToString,
        bigintToString,
        print,
        extractDirective,
        handleDirective,
        // help,
        evalAndPrint,
        cmdStart,
        cmdReadlineStart,
        readlineHandleCmd,
        handleCmd,
        colorizeJs,
        getDirectoryEntries,
        getFilenameCompletions,
        wrapPrintFunction
      }).map(([name, value]) => [name, { value, enumerable: false, writable: false }])
    ])
  );

  return this;

  function waitRead(fd) {
    return new Promise((resolve, reject) => {
      fs.setReadHandler(fd, () => {
        fs.setReadHandler(fd, null);
        resolve();
      });
    });
  }

  async function run() {
    repl.debug('run');

    if(!console.options) console.options = {};

    /* console.options.depth = 2;
    console.options.compact = 2;
    console.options.maxArrayLength = Infinity;*/

    await repl.termInit();
    repl.cmdStart(title);

    do {
      // console.log("run", {fs});
      await fs.waitRead(input);

      await repl.termReadHandler();
    } while(running);
  }
  function runSync() {
    repl.debug('run');

    if(!console.options) console.options = {};

    /* console.options.depth = 2;
    console.options.compact = 2;
    console.options.maxArrayLength = Infinity;*/

    repl.termInit();
    repl.cmdStart(title);

    setReadHandler(input, () => repl.termReadHandler());
  }

  function wrapPrintFunction(fn, thisObj) {
    return (...args) => {
      let ret;
      puts('\r\x1b[K');
      ret = fn.call(thisObj, ...args);
      //repl.termInit();
      //repl.cmdReadlineStart(title);
      // puts('\r\x1b[J');
      repl.readlinePrintPrompt();
      repl.update();
    };
  }
}
