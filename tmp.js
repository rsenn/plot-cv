switch (key & 0xfff) {
  case 0xf08 /* backspace */:
  case 0x08 /* backspace */:
    if(frameShow > 0) {
      frameShow--;
      pipeline.step(-1);
    }
    break;
  case 0xf52 /* up */:
  case 0x3c /* < */:
    paramNav.prev();
    if(paramIndexes[0] != -1 && paramNav.index < paramIndexes[0])
      paramNav.index = paramIndexes[1];

    console.log(`Param #${paramNav.index} '${
        paramNav.name
      }' selected (${+paramNav.param})`
    );
    RedrawStatus();
    RedrawWindow();
    break;
  case 0xf54 /*down  */:
  case 0x3e /* > */:
    paramNav.next();
    if(paramIndexes[1] != -1 && paramNav.index > paramIndexes[1])
      paramNav.index = paramIndexes[0];

    console.log(`Param #${paramNav.index} '${
        paramNav.name
      }' selected (${+paramNav.param})`
    );
    RedrawStatus();
    RedrawWindow();
    break;

  case 0xf53 /* right */:
  case 0x2b /* + */:
    paramNav.param.increment();
    console.log(`Param ${paramNav.name}: ${+paramNav.param}`);
    pipeline.recalc(frameShow);
    break;

  case 0xfff /* DELETE */:
  case 0x9f /* numpad DEL */:
  case 0xf9f /* numpad DEL */:
    paramNav.param.reset();
    console.log(`Param ${paramNav.name}: ${inspect(paramNav.param)}`);
    pipeline.recalc(frameShow);
    break;

  case 0xf51 /* left */:
  case 0x2d /* - */:
  case 0xad /* numpad - */:
  case 0xfad /* numpad - */:
  case 0x2fad /* numpad - */:
    paramNav.param.decrement();
    console.log(`Param ${paramNav.name}: ${+paramNav.param}`);
    pipeline.recalc(frameShow);
    break;

  case 0x31: /* 1 */
  case 0x32: /* 2 */
  case 0x33: /* 3 */
  case 0x34: /* 4 */
  case 0x35: /* 5 */
  case 0x36: /* 6 */
  case 0x37: /* 7 */
  case 0x38: /* 8 */
  case 0x39: /* 9 */
  case 0x30 /* 0 */:
    let v = key & 0xf || 10;
    paramNav.param.alpha = v / 10;
    console.log(`Param ${paramNav.name}: ${+paramNav.param}`);
    pipeline.recalc(frameShow);
    break;
  case 0xa7 /* § */:
    paramNav.param.alpha = 0;
    console.log(`Param ${paramNav.name}: ${+paramNav.param}`);
    pipeline.recalc(frameShow);
    break;

  case 0x20:
    frameShow = Util.mod(frameShow + 1, pipeline.size);
    pipeline.step();
    break;

  default: {
    if(key !== -1) console.log('key:', ToHex(key));
    break;
  }
}
let text = `  ${
  idx + paramIndexes[0] == paramNav.index ? '\x1b[1;31m' : ''
}   ${name.padEnd(13)}\x1b[0m   \x1b[1;36m${+paramNav.get(name)}\x1b[0m\n`;
new NumericParam(config.rho || 1, 1, 30, 0.25);
console.log(`Saved config to '${basename + '.config.json'}'`,
  inspect(configObj, { compact: false })
);
