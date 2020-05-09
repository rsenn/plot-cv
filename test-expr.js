function isPunctuator(word) {
  switch (word.length) {
    case 1:
      return "=.-%}>,*[<!/]~&(;?|):+^{@".indexOf(word) >= 0;

    case 2:
      return ["!=", "*=", "&&", "<<", "/=", "||", ">>", "&=", "==", "++", "|=", "<=", "--", "+=", "^=", ">=", "-=", "%=", "=>"].indexOf(word) >= 0;

    case 3:
      return ["!==", "===", ">>=", "-->>", "<<=", "..."].indexOf(word) >= 0;

    case 4: {
      return word === "-->>=";
    }

    default:
      return false;
  }
}

//console.log(testFn(">="));

console.log(isPunctuator(process.argv[2] || ""));
