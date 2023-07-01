s = new RPCSocket('ws://127.0.0.1:9000', RPCClient);
s.connect().then(() =>
  setTimeout(async () => {
    cli = RPCClient.last;
    api = cli.api;
    id = (await api.new({ class: 'Socket' })).id;
    methods = await api.methods({ id });
    s2 = await rpc.connect('ws://127.0.0.1:9000/ws', RPCServer);
    fac = new RPCFactory(api);
    console.log('fac', fac);
    ro1 = await fac({ class: 'Socket' });
    console.log('ro1', ro1);
    for(let name of ['Socket', 'Worker']) {
      let { id, ...o } = await api.new({ class: name });
      console.log(`${name}#${id}`, o);
    }
  }, 100)
);
