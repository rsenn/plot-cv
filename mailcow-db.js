import { abbreviate, ansiStyles } from 'util';
import { className } from 'util';
import { randStr } from 'util';
import { startInteractive } from 'util';
import { Console } from 'console';
import { MySQL } from 'mysql';
import { MySQLResult } from 'mysql';
import { exit } from 'std';
('use strict');
('use math');

let i,
  q,
  resultNum = 0;

const result = r => {
  let prop = 'result' + ++resultNum;
  globalThis[prop] = r;
  console.log(/*'globalThis.' +*/ prop + ' =', r);
  return r;
};

async function main(...args) {
  globalThis.console = new Console({
    inspectOptions: {
      compact: false,
      customInspect: true,
      showHidden: false,
      hideKeys: ['query']
    }
  });

  Object.assign(globalThis, { MySQL, MySQLResult });

  let my = (globalThis.my = new MySQL());

  my.resultType |= MySQL.RESULT_OBJECT;

  my.setOption(MySQL.OPT_NONBLOCK, true);

  console.log('2: my.getOption(OPT_NONBLOCK) =', my.getOption(MySQL.OPT_NONBLOCK));

  console.log('my.connect() =', await my.connect('127.0.0.1', 'mailcow', 'PASFApnWBm1JuFJ6dkQfG2cbj3WW', 'mailcow', 13306));

  q = globalThis.q = async s => (
    console.log(`q('\x1b[0;32m${abbreviate(s, 1000)}'\x1b[0m)`),
    result(
      await my.query(s).then(
        val => result(val),
        err => {
          const { redBright, reset } = ansiStyles;
          console.log(`${redBright.open + className(err) + reset.close}:`, err.message);
          return null;
        }
      )
    )
  );

  i = 0;
  let res = (globalThis.res = await q(`SELECT * FROM mailbox LIMIT 0,10;`));

  console.log(`res =`, res);
  let rows = (globalThis.rows = []);
  for await(let row of res) {
    rows.push(row);
    console.log(`row[${i++}] =`, row);
  }

  //startInteractive();
  // os.kill(process.pid, os.SIGUSR1);
}

try {
  main(...scriptArgs.slice(1)).catch(err => console.log(`FAIL: ${err.message}\n${err.stack}`));
} catch(error) {
  console.log(`FAIL: ${error.message}\n${error.stack}`);
  exit(1);
} finally {
  console.log('SUCCESS');
}