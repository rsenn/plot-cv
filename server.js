import express from "express";
import path from "path";
import fs from "fs";

var app = express();
const p = path.join(path.dirname(process.argv[1]), ".");

console.log("Serving from", p);

app.use(express.text({ type: "application/xml" }));

app.use((req, res, next) => {
  if(!/lib\//.test(req.url)) console.log("Request:", req.url);
  next();
});

app.use("/static", express.static(p));
app.use("/modules", express.static(path.join(p, "node_modules")));
app.use("/components", express.static(path.join(p, "components")));
app.use("/lib", express.static(path.join(p, "lib")));

app.get("/files.html", async (req, res) => {
  let files = [...(await fs.promises.readdir("."))].filter(entry => /\.(brd|sch)$/.test(entry));

  res.type("json");
  res.json({ files });
});
app.get("/autoplacer.html", (req, res) => {
  res.sendFile(path.join(p, "autoplacer.html"));
});

app.post("/save", async (req, res) => {
  const { body } = req;
  console.log("req.headers:", req.headers);
  console.log("save body:", body.substring(0, 100), "...");
  const filename = req.headers["content-disposition"].replace(/.*"([^"]*)".*/, "$1") || "output.svg";
  let result = await fs.promises.writeFile(filename, body, { mode: 0o600, flag: "w" });
  res.json({ result });
});

app.get("/", (req, res) => {
  res.redirect(302, "/autoplacer.html");
});
const port = process.env.PORT || 3000;

app.listen(port, () => {
  console.log(`Ready at http://127.0.0.1:${port}`);
});
