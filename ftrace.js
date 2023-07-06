import fs from 'fs';
import * as path from 'path';
import { Console } from 'console';
import { unescape } from 'misc';
import { IfDebug, LogIfDebug, ReadFd, ReadFile, LoadHistory, ReadJSON, ReadXML, WriteFile, WriteJSON, WriteXML, ReadBJSON, WriteBJSON, Filter, FilterImages, SortFiles, StatFiles, FdReader, CopyToClipboard, LogCall } from './io-helpers.js';
import { Spawn } from './os-helpers.js';

const StrDecode = s => ((s = s.slice(1, -1)), unescape(s));
const StrFmt = s => (s[0] == '"' && s[s.length - 1] == '"' ? StrDecode(s) : s == 'NULL' ? null : !isNaN(+s) ? +s : s);

function main(...args) {
  globalThis.console = new Console({
    inspectOptions: {
      compact: 2,
      customInspect: true,
      maxArrayLength: 200
    }
  });

  if(!args.length) args = ['strace.log'];

  for(let logfile of args) {
    const allFiles = new Set();
    const fileFuncs = new Set();
    const processes = {};
    let fd = fs.fopen(logfile, 'r');
    let line;
    console.log('fd', fd);

    while((line = fd.getline())) {
      let pidLen = line.indexOf(' ');
      let pidStr = line.substring(0, pidLen);
      let pidNum = +pidStr;

      let argIndex = line.indexOf('(');
      let fnIndex = line.substring(0, argIndex).indexOf(' ');
      let fnName = line.substring(fnIndex + 1, argIndex);

      //  if(/ /.test(fnName))
      if(/ /.test(fnName)) {
        if(/resumed>/.test(line) && processes[pidNum]) {
          line = processes[pidNum].replace(/<unfini.*/g, '') + line.replace(/.*resumed>/g, '');
          delete processes[pidNum];
          argIndex = line.indexOf('(');
          fnIndex = line.substring(0, argIndex).indexOf(' ');
          fnName = line.substring(fnIndex + 1, argIndex);
        } else {
          throw new Error(`Parsing line: '${line}'\nPrevious: '${processes[pidNum]}'`);
        }
      }
      if(argIndex == -1) continue;
      let resultIndex = line.indexOf(' = ');

      let argLastIndex = line.substring(0, resultIndex != -1 ? resultIndex : line.length).indexOf(')');

      let args = line
        .substring(argIndex + 1, argLastIndex)
        .split(/, /g)
        .map(StrFmt);
      let result = resultIndex != -1 ? line.substring(resultIndex + 3) : null;

      const re = /([^\(]*)\(([^,]+(\)|,\s*)*|\s+=\s+)/g;

      const strings = args.filter(arg => typeof arg == 'string');
      const files = strings.filter(str => !allFiles.has(str) && fs.existsSync(str));
      //console.log(fnName, args);
      //console.log('result', result);
      if(files) {
        //console.log('files', files);
        if(files.length) fileFuncs.add(fnName);
        for(let file of files) allFiles.add(file);
      }
      processes[pidNum] = line;
    }

    const sorted = [...allFiles].sort();

    // console.log('fileFuncs', [...fileFuncs].join('\n'));
    WriteFile(logfile + '.files', sorted.join('\n'));
    WriteFile(logfile + '.funcs', [...fileFuncs].join('\n'));
  }
}

main(...scriptArgs.slice(1));
