var express = require('express');
var path = require('path');
var fs = require('fs');
var app = express();
const p = path.join(__dirname, '.');

console.log("Serving from", p);

app.use((req,res, next) => {
console.log("Request:", req.url);
next();
});

app.use('/static', express.static(p));
app.use('/lib', express.static(path.join(p, "lib")));


app.get('/autoplacer.html', (req,res) => { res.sendFile(path.join(p, 'autoplacer.html')); });

app.get('/', (req,res) => {
  res.redirect(302, '/autoplacer.html');
});

app.listen(3000);