// Copyright 2018-2024 the Deno authors. All rights reserved. MIT license.
// This module is browser compatible.
/** Options for {@linkcode globToRegExp}. */ const regExpEscapeChars = [
  "!",
  "$",
  "(",
  ")",
  "*",
  "+",
  ".",
  "=",
  "?",
  "[",
  "\\",
  "^",
  "{",
  "|"
];
const rangeEscapeChars = [
  "-",
  "\\",
  "]"
];
export function _globToRegExp(c, glob, { extended = true, globstar: globstarOption = true, // os = osType,
caseInsensitive = false } = {}) {
  if (glob === "") {
    return /(?!)/;
  }
  // Remove trailing separators.
  let newLength = glob.length;
  for(; newLength > 1 && c.seps.includes(glob[newLength - 1]); newLength--);
  glob = glob.slice(0, newLength);
  let regExpString = "";
  // Terminates correctly. Trust that `j` is incremented every iteration.
  for(let j = 0; j < glob.length;){
    let segment = "";
    const groupStack = [];
    let inRange = false;
    let inEscape = false;
    let endsWithSep = false;
    let i = j;
    // Terminates with `i` at the non-inclusive end of the current segment.
    for(; i < glob.length && !c.seps.includes(glob[i]); i++){
      if (inEscape) {
        inEscape = false;
        const escapeChars = inRange ? rangeEscapeChars : regExpEscapeChars;
        segment += escapeChars.includes(glob[i]) ? `\\${glob[i]}` : glob[i];
        continue;
      }
      if (glob[i] === c.escapePrefix) {
        inEscape = true;
        continue;
      }
      if (glob[i] === "[") {
        if (!inRange) {
          inRange = true;
          segment += "[";
          if (glob[i + 1] === "!") {
            i++;
            segment += "^";
          } else if (glob[i + 1] === "^") {
            i++;
            segment += "\\^";
          }
          continue;
        } else if (glob[i + 1] === ":") {
          let k = i + 1;
          let value = "";
          while(glob[k + 1] !== undefined && glob[k + 1] !== ":"){
            value += glob[k + 1];
            k++;
          }
          if (glob[k + 1] === ":" && glob[k + 2] === "]") {
            i = k + 2;
            if (value === "alnum") segment += "\\dA-Za-z";
            else if (value === "alpha") segment += "A-Za-z";
            else if (value === "ascii") segment += "\x00-\x7F";
            else if (value === "blank") segment += "\t ";
            else if (value === "cntrl") segment += "\x00-\x1F\x7F";
            else if (value === "digit") segment += "\\d";
            else if (value === "graph") segment += "\x21-\x7E";
            else if (value === "lower") segment += "a-z";
            else if (value === "print") segment += "\x20-\x7E";
            else if (value === "punct") {
              segment += "!\"#$%&'()*+,\\-./:;<=>?@[\\\\\\]^_‘{|}~";
            } else if (value === "space") segment += "\\s\v";
            else if (value === "upper") segment += "A-Z";
            else if (value === "word") segment += "\\w";
            else if (value === "xdigit") segment += "\\dA-Fa-f";
            continue;
          }
        }
      }
      if (glob[i] === "]" && inRange) {
        inRange = false;
        segment += "]";
        continue;
      }
      if (inRange) {
        if (glob[i] === "\\") {
          segment += `\\\\`;
        } else {
          segment += glob[i];
        }
        continue;
      }
      if (glob[i] === ")" && groupStack.length > 0 && groupStack[groupStack.length - 1] !== "BRACE") {
        segment += ")";
        const type = groupStack.pop();
        if (type === "!") {
          segment += c.wildcard;
        } else if (type !== "@") {
          segment += type;
        }
        continue;
      }
      if (glob[i] === "|" && groupStack.length > 0 && groupStack[groupStack.length - 1] !== "BRACE") {
        segment += "|";
        continue;
      }
      if (glob[i] === "+" && extended && glob[i + 1] === "(") {
        i++;
        groupStack.push("+");
        segment += "(?:";
        continue;
      }
      if (glob[i] === "@" && extended && glob[i + 1] === "(") {
        i++;
        groupStack.push("@");
        segment += "(?:";
        continue;
      }
      if (glob[i] === "?") {
        if (extended && glob[i + 1] === "(") {
          i++;
          groupStack.push("?");
          segment += "(?:";
        } else {
          segment += ".";
        }
        continue;
      }
      if (glob[i] === "!" && extended && glob[i + 1] === "(") {
        i++;
        groupStack.push("!");
        segment += "(?!";
        continue;
      }
      if (glob[i] === "{") {
        groupStack.push("BRACE");
        segment += "(?:";
        continue;
      }
      if (glob[i] === "}" && groupStack[groupStack.length - 1] === "BRACE") {
        groupStack.pop();
        segment += ")";
        continue;
      }
      if (glob[i] === "," && groupStack[groupStack.length - 1] === "BRACE") {
        segment += "|";
        continue;
      }
      if (glob[i] === "*") {
        if (extended && glob[i + 1] === "(") {
          i++;
          groupStack.push("*");
          segment += "(?:";
        } else {
          const prevChar = glob[i - 1];
          let numStars = 1;
          while(glob[i + 1] === "*"){
            i++;
            numStars++;
          }
          const nextChar = glob[i + 1];
          if (globstarOption && numStars === 2 && [
            ...c.seps,
            undefined
          ].includes(prevChar) && [
            ...c.seps,
            undefined
          ].includes(nextChar)) {
            segment += c.globstar;
            endsWithSep = true;
          } else {
            segment += c.wildcard;
          }
        }
        continue;
      }
      segment += regExpEscapeChars.includes(glob[i]) ? `\\${glob[i]}` : glob[i];
    }
    // Check for unclosed groups or a dangling backslash.
    if (groupStack.length > 0 || inRange || inEscape) {
      // Parse failure. Take all characters from this segment literally.
      segment = "";
      for (const c of glob.slice(j, i)){
        segment += regExpEscapeChars.includes(c) ? `\\${c}` : c;
        endsWithSep = false;
      }
    }
    regExpString += segment;
    if (!endsWithSep) {
      regExpString += i < glob.length ? c.sep : c.sepMaybe;
      endsWithSep = true;
    }
    // Terminates with `i` at the start of the next segment.
    while(c.seps.includes(glob[i]))i++;
    // Check that the next value of `j` is indeed higher than the current value.
    if (!(i > j)) {
      throw new Error("Assertion failure: i > j (potential infinite loop)");
    }
    j = i;
  }
  regExpString = `^${regExpString}$`;
  return new RegExp(regExpString, caseInsensitive ? "i" : "");
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbImh0dHBzOi8vZGVuby5sYW5kL3N0ZEAwLjIxNC4wL3BhdGgvX2NvbW1vbi9nbG9iX3RvX3JlZ19leHAudHMiXSwic291cmNlc0NvbnRlbnQiOlsiLy8gQ29weXJpZ2h0IDIwMTgtMjAyNCB0aGUgRGVubyBhdXRob3JzLiBBbGwgcmlnaHRzIHJlc2VydmVkLiBNSVQgbGljZW5zZS5cbi8vIFRoaXMgbW9kdWxlIGlzIGJyb3dzZXIgY29tcGF0aWJsZS5cblxuLyoqIE9wdGlvbnMgZm9yIHtAbGlua2NvZGUgZ2xvYlRvUmVnRXhwfS4gKi9cbmV4cG9ydCBpbnRlcmZhY2UgR2xvYk9wdGlvbnMge1xuICAvKiogRXh0ZW5kZWQgZ2xvYiBzeW50YXguXG4gICAqIFNlZSBodHRwczovL3d3dy5saW51eGpvdXJuYWwuY29tL2NvbnRlbnQvYmFzaC1leHRlbmRlZC1nbG9iYmluZy5cbiAgICpcbiAgICogQGRlZmF1bHQge3RydWV9XG4gICAqL1xuICBleHRlbmRlZD86IGJvb2xlYW47XG4gIC8qKiBHbG9ic3RhciBzeW50YXguXG4gICAqIFNlZSBodHRwczovL3d3dy5saW51eGpvdXJuYWwuY29tL2NvbnRlbnQvZ2xvYnN0YXItbmV3LWJhc2gtZ2xvYmJpbmctb3B0aW9uLlxuICAgKiBJZiBmYWxzZSwgYCoqYCBpcyB0cmVhdGVkIGxpa2UgYCpgLlxuICAgKlxuICAgKiBAZGVmYXVsdCB7dHJ1ZX1cbiAgICovXG4gIGdsb2JzdGFyPzogYm9vbGVhbjtcbiAgLyoqIFdoZXRoZXIgZ2xvYnN0YXIgc2hvdWxkIGJlIGNhc2UtaW5zZW5zaXRpdmUuICovXG4gIGNhc2VJbnNlbnNpdGl2ZT86IGJvb2xlYW47XG59XG5cbmV4cG9ydCB0eXBlIEdsb2JUb1JlZ0V4cE9wdGlvbnMgPSBHbG9iT3B0aW9ucztcblxuY29uc3QgcmVnRXhwRXNjYXBlQ2hhcnMgPSBbXG4gIFwiIVwiLFxuICBcIiRcIixcbiAgXCIoXCIsXG4gIFwiKVwiLFxuICBcIipcIixcbiAgXCIrXCIsXG4gIFwiLlwiLFxuICBcIj1cIixcbiAgXCI/XCIsXG4gIFwiW1wiLFxuICBcIlxcXFxcIixcbiAgXCJeXCIsXG4gIFwie1wiLFxuICBcInxcIixcbl07XG5jb25zdCByYW5nZUVzY2FwZUNoYXJzID0gW1wiLVwiLCBcIlxcXFxcIiwgXCJdXCJdO1xuXG5leHBvcnQgaW50ZXJmYWNlIEdsb2JDb25zdGFudHMge1xuICBzZXA6IHN0cmluZztcbiAgc2VwTWF5YmU6IHN0cmluZztcbiAgc2Vwczogc3RyaW5nW107XG4gIGdsb2JzdGFyOiBzdHJpbmc7XG4gIHdpbGRjYXJkOiBzdHJpbmc7XG4gIGVzY2FwZVByZWZpeDogc3RyaW5nO1xufVxuXG5leHBvcnQgZnVuY3Rpb24gX2dsb2JUb1JlZ0V4cChcbiAgYzogR2xvYkNvbnN0YW50cyxcbiAgZ2xvYjogc3RyaW5nLFxuICB7XG4gICAgZXh0ZW5kZWQgPSB0cnVlLFxuICAgIGdsb2JzdGFyOiBnbG9ic3Rhck9wdGlvbiA9IHRydWUsXG4gICAgLy8gb3MgPSBvc1R5cGUsXG4gICAgY2FzZUluc2Vuc2l0aXZlID0gZmFsc2UsXG4gIH06IEdsb2JUb1JlZ0V4cE9wdGlvbnMgPSB7fSxcbik6IFJlZ0V4cCB7XG4gIGlmIChnbG9iID09PSBcIlwiKSB7XG4gICAgcmV0dXJuIC8oPyEpLztcbiAgfVxuXG4gIC8vIFJlbW92ZSB0cmFpbGluZyBzZXBhcmF0b3JzLlxuICBsZXQgbmV3TGVuZ3RoID0gZ2xvYi5sZW5ndGg7XG4gIGZvciAoOyBuZXdMZW5ndGggPiAxICYmIGMuc2Vwcy5pbmNsdWRlcyhnbG9iW25ld0xlbmd0aCAtIDFdKTsgbmV3TGVuZ3RoLS0pO1xuICBnbG9iID0gZ2xvYi5zbGljZSgwLCBuZXdMZW5ndGgpO1xuXG4gIGxldCByZWdFeHBTdHJpbmcgPSBcIlwiO1xuXG4gIC8vIFRlcm1pbmF0ZXMgY29ycmVjdGx5LiBUcnVzdCB0aGF0IGBqYCBpcyBpbmNyZW1lbnRlZCBldmVyeSBpdGVyYXRpb24uXG4gIGZvciAobGV0IGogPSAwOyBqIDwgZ2xvYi5sZW5ndGg7KSB7XG4gICAgbGV0IHNlZ21lbnQgPSBcIlwiO1xuICAgIGNvbnN0IGdyb3VwU3RhY2s6IHN0cmluZ1tdID0gW107XG4gICAgbGV0IGluUmFuZ2UgPSBmYWxzZTtcbiAgICBsZXQgaW5Fc2NhcGUgPSBmYWxzZTtcbiAgICBsZXQgZW5kc1dpdGhTZXAgPSBmYWxzZTtcbiAgICBsZXQgaSA9IGo7XG5cbiAgICAvLyBUZXJtaW5hdGVzIHdpdGggYGlgIGF0IHRoZSBub24taW5jbHVzaXZlIGVuZCBvZiB0aGUgY3VycmVudCBzZWdtZW50LlxuICAgIGZvciAoOyBpIDwgZ2xvYi5sZW5ndGggJiYgIWMuc2Vwcy5pbmNsdWRlcyhnbG9iW2ldKTsgaSsrKSB7XG4gICAgICBpZiAoaW5Fc2NhcGUpIHtcbiAgICAgICAgaW5Fc2NhcGUgPSBmYWxzZTtcbiAgICAgICAgY29uc3QgZXNjYXBlQ2hhcnMgPSBpblJhbmdlID8gcmFuZ2VFc2NhcGVDaGFycyA6IHJlZ0V4cEVzY2FwZUNoYXJzO1xuICAgICAgICBzZWdtZW50ICs9IGVzY2FwZUNoYXJzLmluY2x1ZGVzKGdsb2JbaV0pID8gYFxcXFwke2dsb2JbaV19YCA6IGdsb2JbaV07XG4gICAgICAgIGNvbnRpbnVlO1xuICAgICAgfVxuXG4gICAgICBpZiAoZ2xvYltpXSA9PT0gYy5lc2NhcGVQcmVmaXgpIHtcbiAgICAgICAgaW5Fc2NhcGUgPSB0cnVlO1xuICAgICAgICBjb250aW51ZTtcbiAgICAgIH1cblxuICAgICAgaWYgKGdsb2JbaV0gPT09IFwiW1wiKSB7XG4gICAgICAgIGlmICghaW5SYW5nZSkge1xuICAgICAgICAgIGluUmFuZ2UgPSB0cnVlO1xuICAgICAgICAgIHNlZ21lbnQgKz0gXCJbXCI7XG4gICAgICAgICAgaWYgKGdsb2JbaSArIDFdID09PSBcIiFcIikge1xuICAgICAgICAgICAgaSsrO1xuICAgICAgICAgICAgc2VnbWVudCArPSBcIl5cIjtcbiAgICAgICAgICB9IGVsc2UgaWYgKGdsb2JbaSArIDFdID09PSBcIl5cIikge1xuICAgICAgICAgICAgaSsrO1xuICAgICAgICAgICAgc2VnbWVudCArPSBcIlxcXFxeXCI7XG4gICAgICAgICAgfVxuICAgICAgICAgIGNvbnRpbnVlO1xuICAgICAgICB9IGVsc2UgaWYgKGdsb2JbaSArIDFdID09PSBcIjpcIikge1xuICAgICAgICAgIGxldCBrID0gaSArIDE7XG4gICAgICAgICAgbGV0IHZhbHVlID0gXCJcIjtcbiAgICAgICAgICB3aGlsZSAoZ2xvYltrICsgMV0gIT09IHVuZGVmaW5lZCAmJiBnbG9iW2sgKyAxXSAhPT0gXCI6XCIpIHtcbiAgICAgICAgICAgIHZhbHVlICs9IGdsb2JbayArIDFdO1xuICAgICAgICAgICAgaysrO1xuICAgICAgICAgIH1cbiAgICAgICAgICBpZiAoZ2xvYltrICsgMV0gPT09IFwiOlwiICYmIGdsb2JbayArIDJdID09PSBcIl1cIikge1xuICAgICAgICAgICAgaSA9IGsgKyAyO1xuICAgICAgICAgICAgaWYgKHZhbHVlID09PSBcImFsbnVtXCIpIHNlZ21lbnQgKz0gXCJcXFxcZEEtWmEtelwiO1xuICAgICAgICAgICAgZWxzZSBpZiAodmFsdWUgPT09IFwiYWxwaGFcIikgc2VnbWVudCArPSBcIkEtWmEtelwiO1xuICAgICAgICAgICAgZWxzZSBpZiAodmFsdWUgPT09IFwiYXNjaWlcIikgc2VnbWVudCArPSBcIlxceDAwLVxceDdGXCI7XG4gICAgICAgICAgICBlbHNlIGlmICh2YWx1ZSA9PT0gXCJibGFua1wiKSBzZWdtZW50ICs9IFwiXFx0IFwiO1xuICAgICAgICAgICAgZWxzZSBpZiAodmFsdWUgPT09IFwiY250cmxcIikgc2VnbWVudCArPSBcIlxceDAwLVxceDFGXFx4N0ZcIjtcbiAgICAgICAgICAgIGVsc2UgaWYgKHZhbHVlID09PSBcImRpZ2l0XCIpIHNlZ21lbnQgKz0gXCJcXFxcZFwiO1xuICAgICAgICAgICAgZWxzZSBpZiAodmFsdWUgPT09IFwiZ3JhcGhcIikgc2VnbWVudCArPSBcIlxceDIxLVxceDdFXCI7XG4gICAgICAgICAgICBlbHNlIGlmICh2YWx1ZSA9PT0gXCJsb3dlclwiKSBzZWdtZW50ICs9IFwiYS16XCI7XG4gICAgICAgICAgICBlbHNlIGlmICh2YWx1ZSA9PT0gXCJwcmludFwiKSBzZWdtZW50ICs9IFwiXFx4MjAtXFx4N0VcIjtcbiAgICAgICAgICAgIGVsc2UgaWYgKHZhbHVlID09PSBcInB1bmN0XCIpIHtcbiAgICAgICAgICAgICAgc2VnbWVudCArPSBcIiFcXFwiIyQlJicoKSorLFxcXFwtLi86Ozw9Pj9AW1xcXFxcXFxcXFxcXF1eX+KAmHt8fX5cIjtcbiAgICAgICAgICAgIH0gZWxzZSBpZiAodmFsdWUgPT09IFwic3BhY2VcIikgc2VnbWVudCArPSBcIlxcXFxzXFx2XCI7XG4gICAgICAgICAgICBlbHNlIGlmICh2YWx1ZSA9PT0gXCJ1cHBlclwiKSBzZWdtZW50ICs9IFwiQS1aXCI7XG4gICAgICAgICAgICBlbHNlIGlmICh2YWx1ZSA9PT0gXCJ3b3JkXCIpIHNlZ21lbnQgKz0gXCJcXFxcd1wiO1xuICAgICAgICAgICAgZWxzZSBpZiAodmFsdWUgPT09IFwieGRpZ2l0XCIpIHNlZ21lbnQgKz0gXCJcXFxcZEEtRmEtZlwiO1xuICAgICAgICAgICAgY29udGludWU7XG4gICAgICAgICAgfVxuICAgICAgICB9XG4gICAgICB9XG5cbiAgICAgIGlmIChnbG9iW2ldID09PSBcIl1cIiAmJiBpblJhbmdlKSB7XG4gICAgICAgIGluUmFuZ2UgPSBmYWxzZTtcbiAgICAgICAgc2VnbWVudCArPSBcIl1cIjtcbiAgICAgICAgY29udGludWU7XG4gICAgICB9XG5cbiAgICAgIGlmIChpblJhbmdlKSB7XG4gICAgICAgIGlmIChnbG9iW2ldID09PSBcIlxcXFxcIikge1xuICAgICAgICAgIHNlZ21lbnQgKz0gYFxcXFxcXFxcYDtcbiAgICAgICAgfSBlbHNlIHtcbiAgICAgICAgICBzZWdtZW50ICs9IGdsb2JbaV07XG4gICAgICAgIH1cbiAgICAgICAgY29udGludWU7XG4gICAgICB9XG5cbiAgICAgIGlmIChcbiAgICAgICAgZ2xvYltpXSA9PT0gXCIpXCIgJiYgZ3JvdXBTdGFjay5sZW5ndGggPiAwICYmXG4gICAgICAgIGdyb3VwU3RhY2tbZ3JvdXBTdGFjay5sZW5ndGggLSAxXSAhPT0gXCJCUkFDRVwiXG4gICAgICApIHtcbiAgICAgICAgc2VnbWVudCArPSBcIilcIjtcbiAgICAgICAgY29uc3QgdHlwZSA9IGdyb3VwU3RhY2sucG9wKCkhO1xuICAgICAgICBpZiAodHlwZSA9PT0gXCIhXCIpIHtcbiAgICAgICAgICBzZWdtZW50ICs9IGMud2lsZGNhcmQ7XG4gICAgICAgIH0gZWxzZSBpZiAodHlwZSAhPT0gXCJAXCIpIHtcbiAgICAgICAgICBzZWdtZW50ICs9IHR5cGU7XG4gICAgICAgIH1cbiAgICAgICAgY29udGludWU7XG4gICAgICB9XG5cbiAgICAgIGlmIChcbiAgICAgICAgZ2xvYltpXSA9PT0gXCJ8XCIgJiYgZ3JvdXBTdGFjay5sZW5ndGggPiAwICYmXG4gICAgICAgIGdyb3VwU3RhY2tbZ3JvdXBTdGFjay5sZW5ndGggLSAxXSAhPT0gXCJCUkFDRVwiXG4gICAgICApIHtcbiAgICAgICAgc2VnbWVudCArPSBcInxcIjtcbiAgICAgICAgY29udGludWU7XG4gICAgICB9XG5cbiAgICAgIGlmIChnbG9iW2ldID09PSBcIitcIiAmJiBleHRlbmRlZCAmJiBnbG9iW2kgKyAxXSA9PT0gXCIoXCIpIHtcbiAgICAgICAgaSsrO1xuICAgICAgICBncm91cFN0YWNrLnB1c2goXCIrXCIpO1xuICAgICAgICBzZWdtZW50ICs9IFwiKD86XCI7XG4gICAgICAgIGNvbnRpbnVlO1xuICAgICAgfVxuXG4gICAgICBpZiAoZ2xvYltpXSA9PT0gXCJAXCIgJiYgZXh0ZW5kZWQgJiYgZ2xvYltpICsgMV0gPT09IFwiKFwiKSB7XG4gICAgICAgIGkrKztcbiAgICAgICAgZ3JvdXBTdGFjay5wdXNoKFwiQFwiKTtcbiAgICAgICAgc2VnbWVudCArPSBcIig/OlwiO1xuICAgICAgICBjb250aW51ZTtcbiAgICAgIH1cblxuICAgICAgaWYgKGdsb2JbaV0gPT09IFwiP1wiKSB7XG4gICAgICAgIGlmIChleHRlbmRlZCAmJiBnbG9iW2kgKyAxXSA9PT0gXCIoXCIpIHtcbiAgICAgICAgICBpKys7XG4gICAgICAgICAgZ3JvdXBTdGFjay5wdXNoKFwiP1wiKTtcbiAgICAgICAgICBzZWdtZW50ICs9IFwiKD86XCI7XG4gICAgICAgIH0gZWxzZSB7XG4gICAgICAgICAgc2VnbWVudCArPSBcIi5cIjtcbiAgICAgICAgfVxuICAgICAgICBjb250aW51ZTtcbiAgICAgIH1cblxuICAgICAgaWYgKGdsb2JbaV0gPT09IFwiIVwiICYmIGV4dGVuZGVkICYmIGdsb2JbaSArIDFdID09PSBcIihcIikge1xuICAgICAgICBpKys7XG4gICAgICAgIGdyb3VwU3RhY2sucHVzaChcIiFcIik7XG4gICAgICAgIHNlZ21lbnQgKz0gXCIoPyFcIjtcbiAgICAgICAgY29udGludWU7XG4gICAgICB9XG5cbiAgICAgIGlmIChnbG9iW2ldID09PSBcIntcIikge1xuICAgICAgICBncm91cFN0YWNrLnB1c2goXCJCUkFDRVwiKTtcbiAgICAgICAgc2VnbWVudCArPSBcIig/OlwiO1xuICAgICAgICBjb250aW51ZTtcbiAgICAgIH1cblxuICAgICAgaWYgKGdsb2JbaV0gPT09IFwifVwiICYmIGdyb3VwU3RhY2tbZ3JvdXBTdGFjay5sZW5ndGggLSAxXSA9PT0gXCJCUkFDRVwiKSB7XG4gICAgICAgIGdyb3VwU3RhY2sucG9wKCk7XG4gICAgICAgIHNlZ21lbnQgKz0gXCIpXCI7XG4gICAgICAgIGNvbnRpbnVlO1xuICAgICAgfVxuXG4gICAgICBpZiAoZ2xvYltpXSA9PT0gXCIsXCIgJiYgZ3JvdXBTdGFja1tncm91cFN0YWNrLmxlbmd0aCAtIDFdID09PSBcIkJSQUNFXCIpIHtcbiAgICAgICAgc2VnbWVudCArPSBcInxcIjtcbiAgICAgICAgY29udGludWU7XG4gICAgICB9XG5cbiAgICAgIGlmIChnbG9iW2ldID09PSBcIipcIikge1xuICAgICAgICBpZiAoZXh0ZW5kZWQgJiYgZ2xvYltpICsgMV0gPT09IFwiKFwiKSB7XG4gICAgICAgICAgaSsrO1xuICAgICAgICAgIGdyb3VwU3RhY2sucHVzaChcIipcIik7XG4gICAgICAgICAgc2VnbWVudCArPSBcIig/OlwiO1xuICAgICAgICB9IGVsc2Uge1xuICAgICAgICAgIGNvbnN0IHByZXZDaGFyID0gZ2xvYltpIC0gMV07XG4gICAgICAgICAgbGV0IG51bVN0YXJzID0gMTtcbiAgICAgICAgICB3aGlsZSAoZ2xvYltpICsgMV0gPT09IFwiKlwiKSB7XG4gICAgICAgICAgICBpKys7XG4gICAgICAgICAgICBudW1TdGFycysrO1xuICAgICAgICAgIH1cbiAgICAgICAgICBjb25zdCBuZXh0Q2hhciA9IGdsb2JbaSArIDFdO1xuICAgICAgICAgIGlmIChcbiAgICAgICAgICAgIGdsb2JzdGFyT3B0aW9uICYmIG51bVN0YXJzID09PSAyICYmXG4gICAgICAgICAgICBbLi4uYy5zZXBzLCB1bmRlZmluZWRdLmluY2x1ZGVzKHByZXZDaGFyKSAmJlxuICAgICAgICAgICAgWy4uLmMuc2VwcywgdW5kZWZpbmVkXS5pbmNsdWRlcyhuZXh0Q2hhcilcbiAgICAgICAgICApIHtcbiAgICAgICAgICAgIHNlZ21lbnQgKz0gYy5nbG9ic3RhcjtcbiAgICAgICAgICAgIGVuZHNXaXRoU2VwID0gdHJ1ZTtcbiAgICAgICAgICB9IGVsc2Uge1xuICAgICAgICAgICAgc2VnbWVudCArPSBjLndpbGRjYXJkO1xuICAgICAgICAgIH1cbiAgICAgICAgfVxuICAgICAgICBjb250aW51ZTtcbiAgICAgIH1cblxuICAgICAgc2VnbWVudCArPSByZWdFeHBFc2NhcGVDaGFycy5pbmNsdWRlcyhnbG9iW2ldKSA/IGBcXFxcJHtnbG9iW2ldfWAgOiBnbG9iW2ldO1xuICAgIH1cblxuICAgIC8vIENoZWNrIGZvciB1bmNsb3NlZCBncm91cHMgb3IgYSBkYW5nbGluZyBiYWNrc2xhc2guXG4gICAgaWYgKGdyb3VwU3RhY2subGVuZ3RoID4gMCB8fCBpblJhbmdlIHx8IGluRXNjYXBlKSB7XG4gICAgICAvLyBQYXJzZSBmYWlsdXJlLiBUYWtlIGFsbCBjaGFyYWN0ZXJzIGZyb20gdGhpcyBzZWdtZW50IGxpdGVyYWxseS5cbiAgICAgIHNlZ21lbnQgPSBcIlwiO1xuICAgICAgZm9yIChjb25zdCBjIG9mIGdsb2Iuc2xpY2UoaiwgaSkpIHtcbiAgICAgICAgc2VnbWVudCArPSByZWdFeHBFc2NhcGVDaGFycy5pbmNsdWRlcyhjKSA/IGBcXFxcJHtjfWAgOiBjO1xuICAgICAgICBlbmRzV2l0aFNlcCA9IGZhbHNlO1xuICAgICAgfVxuICAgIH1cblxuICAgIHJlZ0V4cFN0cmluZyArPSBzZWdtZW50O1xuICAgIGlmICghZW5kc1dpdGhTZXApIHtcbiAgICAgIHJlZ0V4cFN0cmluZyArPSBpIDwgZ2xvYi5sZW5ndGggPyBjLnNlcCA6IGMuc2VwTWF5YmU7XG4gICAgICBlbmRzV2l0aFNlcCA9IHRydWU7XG4gICAgfVxuXG4gICAgLy8gVGVybWluYXRlcyB3aXRoIGBpYCBhdCB0aGUgc3RhcnQgb2YgdGhlIG5leHQgc2VnbWVudC5cbiAgICB3aGlsZSAoYy5zZXBzLmluY2x1ZGVzKGdsb2JbaV0pKSBpKys7XG5cbiAgICAvLyBDaGVjayB0aGF0IHRoZSBuZXh0IHZhbHVlIG9mIGBqYCBpcyBpbmRlZWQgaGlnaGVyIHRoYW4gdGhlIGN1cnJlbnQgdmFsdWUuXG4gICAgaWYgKCEoaSA+IGopKSB7XG4gICAgICB0aHJvdyBuZXcgRXJyb3IoXCJBc3NlcnRpb24gZmFpbHVyZTogaSA+IGogKHBvdGVudGlhbCBpbmZpbml0ZSBsb29wKVwiKTtcbiAgICB9XG4gICAgaiA9IGk7XG4gIH1cblxuICByZWdFeHBTdHJpbmcgPSBgXiR7cmVnRXhwU3RyaW5nfSRgO1xuICByZXR1cm4gbmV3IFJlZ0V4cChyZWdFeHBTdHJpbmcsIGNhc2VJbnNlbnNpdGl2ZSA/IFwiaVwiIDogXCJcIik7XG59XG4iXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6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
// denoCacheMetadata=2038903535745696946,13597886773016452265