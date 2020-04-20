import express from "express";
import path from "path";
import fs from "fs";

var app = express();
const p = path.join(path.dirname(process.argv[1]), ".");

console.log("Serving from", p);

app.use((req, res, next) => {
  console.log("Request:", req.url);
  next();
});

app.use("/static", express.static(p));
app.use("/lib", express.static(path.join(p, "lib")));

app.get("/autoplacer.html", (req, res) => {
  res.sendFile(path.join(p, "autoplacer.html"));
});

app.get("/", (req, res) => {
  res.redirect(302, "/autoplacer.html");
});

app.listen(3000);
