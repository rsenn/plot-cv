import { Archive } from 'archive';
function main(...args) {
  console.log('Archive.version', Archive.version);

  let ar = Archive.read(args[0] ?? 'quickjs-2021-03-27.tar.xz');

  console.log('ar', ar);
  let buf = new ArrayBuffer(1024);

  for(let ent of ar) {
    if(ent.pathname.endsWith('/')) continue;
    //if(!/\.xml$/.test(ent.pathname)) continue;
    console.log('ent', ent);

    ar.extract(ent, 0, (archive, entry) => {
      console.log('extract progress', archive.filterBytes(0), archive.filterBytes(-1));
    });
    /*  let r = ar.read()
   console.log('r', r && r.byteLength, r && ent.size - r.byteLength);*/
  }
  console.log('ar.fileCount', ar.fileCount);
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(error);
  console.log(`FAIL: ${error.message}\n${error.stack}`);
  std.exit(1);
} finally {
  console.log('SUCCESS');
}