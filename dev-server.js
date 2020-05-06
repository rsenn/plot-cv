require("module-alias/register");

const { createServer } = require("http");
const { parse } = require("url");
const next = require("next");

const app = next({ dev: "development" });
const handle = app.getRequestHandler();
const port = process.env.PORT || 3001;

app.prepare().then(() => {
  createServer((req, res) => {
    handle(req, res, parse(req.url, true));
  }).listen(port, err => {
    if(err) throw err;
    console.log(`> Ready on http://localhost:${port}`);
  });
});
