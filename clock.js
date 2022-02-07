
function clock()
{
  // 数字が 1ケタのとき、「0」を加えて 2ケタにする
  var twoDigit =function(num){
    　     var digit
         if( num < 10 )
          { digit = "0" + num; }
         else { digit = num; }
         return digit;
   }
  // 曜日を表す各文字列の配列
  var weeks = new Array("Sun","Mon","Thu","Wed","Thr","Fri","Sat");

 // 現在日時を表すインスタンスを取得
  var now = new Date();

    var year = now.getFullYear();
    var month = twoDigit(now.getMonth() + 1)
    var day = twoDigit(now.getDate());
    var week = weeks[now.getDay()];
    var hour = twoDigit(now.getHours());
    var minute = twoDigit(now.getMinutes());
    var second = twoDigit(now.getSeconds());
   var second2 = now.getSeconds();

 //　HTML: <div id="clock_date">(ココの日付文字列を書き換え)</div>
document.getElementById("clock_date").textContent =  year + "/" + month + "/" + day + " (" + week + ")";

//　HTML: <div id="clock_time">(ココの時刻文字列を書き換え)</div>
document.getElementById("clock_time").textContent = hour + ":" + minute + ":" + second;

//var img = document.getElementById("clock_picone");
//img.src = "./repredator/pre8.png"; // not working...



//document.getElementById("clock_picone").textContent = "<img src = ""./repredator/pre" + second2 + """.png>";

//------------commentout 5.1-------------

var picnum = document.getElementById("clock_picone").textContent = second2 % 10 ;
//picnum = document.getElementById("clock_pictwo").textContent = "<img src = " + '"' + "./repredator/pre" + second2 % 10 + ".png" + '"' + ">";

//------------commentout 5.1-------------


}


var INTERVAL = 2;
var x = 0;
var animating = false;
var timer;

function move(){
   aCar = document.getElementById("clock_picone");
   x = x + 5;
   aCar.style.left = x;
   if ( x > 500){
     x = 0;
   }
   //timer = setInterval("move()",INTERVAL);
   alert("in move");

}

function move3(){
  aCar = document.getElementById("clock_picthree");
  x = x + 5;
  aCar.style.left = x + "px";
  if ( x > 500){
    x = 0;
  }

  //alert("in move");
  //timer = setInterval("move3()",INTERVAL);   don't work well...
  //console.log("in move3 func");
  //console.error("in move3 err msg"); can't use this??
  alert("in move3 Apr30");

  //timer = setInterval("move3()",INTERVAL); //  don't work well...


}

var num = 0;
function changeImg(){
  document.images[0].src = "repredator/pre" + num + ".png";

  document.getElementById("human_number").textContent = num ;

  num++;
  if(num == 10) num = 0;


}

var numw = 0;
function changeImgWithN(numw){
  document.images[0].src = "repredator/pre" + numw + ".png";

  document.getElementById("watM2").textContent = numw ;

  //num++;
  //if(num == 10) num = 0;

}


// 上記のclock関数を1000ミリ秒ごと(毎秒)に実行
setInterval(clock, 1000);

