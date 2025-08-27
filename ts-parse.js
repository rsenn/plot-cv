import * as acorn from 'acorn';
import tsPlugin from 'acorn-typescript';
import { readFileSync } from 'fs';

/*
 *
 * */
const node = acorn.Parser.extend(tsPlugin()).parse(readFileSync(process.argv[2], 'utf-8'), {
  sourceType: 'module',
  ecmaVersion: 'latest',
  locations: true,
});
