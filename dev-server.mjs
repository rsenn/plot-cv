import  "module-alias/register.js";

import { createServer } from "http";
import { parse } from "url";
import next from "next";

const app = next({ dev: "development" });
const handle = app.getRequestHandler();
const port = process.env.PORT || 3000;
app.prepare().then(() => {
  createServer((req, res) => {
    handle(req, res, parse(req.url, true));
  }).listen(port, err => {
    if(err) throw err;
    console.log(`> Ready on http://localhost:${port}`);
  });
});
