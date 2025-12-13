import * as path from 'path';
import { IfDebug, ReadFile, WriteJSON } from './io-helpers.js';
import { ECMAScriptParser, Identifier, ImportDeclaration, ImportSpecifier, Literal, Printer } from './lib/ecmascript.js';
import { Console } from 'console';
import deep from 'deep';
#!/usr/bin/env qjsm
Object.assign(ReadImport.prototype, { [Symbol.toStringTag]: 'Import' });

function main(...args) {
  globalThis.console = new Console({
    stdout: process.stderr,
    inspectOptions: { depth: 100, customInspect: true, compact: 2 }
  });

  let params = Util.getOpt(
    {
      help: [
        false,
        (v, r, o) => {
          console.log(`Usage: ${scriptArgs[0]} [OPTIONS]\n`);
          console.log(o.map(([name, [arg, fn, ch]]) => `  --${(name + ', -' + ch).padEnd(20)}`).join('\n'));
          process.exit(0);
        },
        'h'
      ],
      'output-ast': [true, null, 'a'],
      output: [true, null, 'o'],
      require: [false, null, 'r'],
      import: [false, null, 'i'],
      debug: [false, (v, r, o, result) => (result.debug | 0) + 1, 'x'],
      '@': 'input'
    },
    args
  );
  console.log('params', params);

  let target = params['require'] ? 'require' : 'import';

  if(params['@'].length == 0) {
    console.log(`Usage: ${process.argv[0]} [OPTIONS] <file...>`);
    return 1;
  }

  //ECMAScriptParser.instrumentate();

  for(let file of params['@']) {
    let data = ReadFile(file);

    let parser = new ECMAScriptParser(data, file, IfDebug('parser'));
    let ast = parser.parseProgram();

    //parser.addCommentsToNodes(ast);

    //console.log('ast', ast);

    WriteJSON(params['output-ast'] ?? path.basename(file) + '.ast.json', ast);

    const isRequire = node => node.type == 'CallExpression' && node.callee.name == 'require';
    const isImport = node => node.type == 'ImportDeclaration';

    let imports = deep.select(ast, node => isRequire(node) || isImport(node));

    imports = imports.map(([n, p]) => new ReadImport(n, p, ast));

    //    console.log('imports', console.config({ depth: 10 }), imports);

    for(let imp of imports) {
      const { path, module, names, type } = imp;

      if(type == target) continue;

      switch (target) {
        case 'import': {
          let node = new ImportDeclaration(names.map(([local, imported]) => new ImportSpecifier(new Identifier(imported), new Identifier(local))));
          let x = deep.get(ast, path);
          let y = deep.get(ast, path.slice(0, -1));
          let z = deep.get(ast, path.slice(0, -2));
          console.log('path', path);
          console.log('y', path.slice(0, -1), y);
          console.log('z', path.slice(0, -2), z);
          /*console.log('x.type', x.type);
console.log('ESNode.assoc(x)', ESNode.assoc(x));*/
          ///  deep.set(ast, path, node);
          break;
        }
        case 'require': {
          break;
        }
      }
      console.log('import', console.config({ depth: 10 }), imp);
    }

    if(params['output']) {
      let printer = new Printer({ colors: false, indent: 2 });
      let code = printer.print(ast);
      console.log('code', code);
      //  WriteFile(params['output'], printer.print(ast));
    }

    /* let flat = deep.flatten(ast, new Map());
    console.log('flat',console.config({ depth: 1 }), flat);*/
  }
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error && error.message}\n${error && error.stack}`);
} finally {
  console.log('SUCCESS');
}

function ReadImport(node, path, ast) {
  this.module = null;
  this.names = [];
  switch (node.type) {
    case 'ImportDeclaration': {
      this.type = 'import';
      this.module = Literal.string(node.source);
      for(let spec of node.specifiers) {
        this.names.push([Identifier.string(spec.local), spec.imported ?? 'default']);
      }
      break;
    }
    case 'CallExpression': {
      this.type = 'require';
      this.module = Literal.string(node.arguments[0]);
      let a = [];
      let n = node;
      do {
        path = path.slice(0, -1);
        n = deep.get(ast, path);
        a = [n, ...a];
      } while(n.type != 'VariableDeclarator');
      let member = a.filter(n => n.type == 'MemberExpression').reverse();
      if(member.length) member = member.map(n => n.property.raw).join('.');
      else member = 'default';
      this.names.push([Identifier.string(n.id), member]);
      break;
    }
  }
  Util.define(this, { path });
}