// Ported from js-yaml v3.13.1:
// https://github.com/nodeca/js-yaml/commit/665aadda42349dcae869f12040d9b10ef18d12da
// Copyright 2011-2015 by Vitaly Puzrin. All rights reserved. MIT license.
// Copyright 2018-2022 the Deno authors. All rights reserved. MIT license.
import { YAMLError } from "../error.ts";
import { Mark } from "../mark.ts";
import * as common from "../utils.ts";
import { LoaderState } from "./loader_state.ts";
const { hasOwn } = Object;
const CONTEXT_FLOW_IN = 1;
const CONTEXT_FLOW_OUT = 2;
const CONTEXT_BLOCK_IN = 3;
const CONTEXT_BLOCK_OUT = 4;
const CHOMPING_CLIP = 1;
const CHOMPING_STRIP = 2;
const CHOMPING_KEEP = 3;
const PATTERN_NON_PRINTABLE = // deno-lint-ignore no-control-regex
/[\x00-\x08\x0B\x0C\x0E-\x1F\x7F-\x84\x86-\x9F\uFFFE\uFFFF]|[\uD800-\uDBFF](?![\uDC00-\uDFFF])|(?:[^\uD800-\uDBFF]|^)[\uDC00-\uDFFF]/;
const PATTERN_NON_ASCII_LINE_BREAKS = /[\x85\u2028\u2029]/;
const PATTERN_FLOW_INDICATORS = /[,\[\]\{\}]/;
const PATTERN_TAG_HANDLE = /^(?:!|!!|![a-z\-]+!)$/i;
const PATTERN_TAG_URI = /^(?:!|[^,\[\]\{\}])(?:%[0-9a-f]{2}|[0-9a-z\-#;\/\?:@&=\+\$,_\.!~\*'\(\)\[\]])*$/i;
function _class(obj) {
  return Object.prototype.toString.call(obj);
}
function isEOL(c) {
  return c === 0x0a || /* LF */ c === 0x0d /* CR */ ;
}
function isWhiteSpace(c) {
  return c === 0x09 || /* Tab */ c === 0x20 /* Space */ ;
}
function isWsOrEol(c) {
  return c === 0x09 /* Tab */  || c === 0x20 /* Space */  || c === 0x0a /* LF */  || c === 0x0d /* CR */ ;
}
function isFlowIndicator(c) {
  return c === 0x2c /* , */  || c === 0x5b /* [ */  || c === 0x5d /* ] */  || c === 0x7b /* { */  || c === 0x7d /* } */ ;
}
function fromHexCode(c) {
  if (0x30 <= /* 0 */ c && c <= 0x39 /* 9 */ ) {
    return c - 0x30;
  }
  const lc = c | 0x20;
  if (0x61 <= /* a */ lc && lc <= 0x66 /* f */ ) {
    return lc - 0x61 + 10;
  }
  return -1;
}
function escapedHexLen(c) {
  if (c === 0x78 /* x */ ) {
    return 2;
  }
  if (c === 0x75 /* u */ ) {
    return 4;
  }
  if (c === 0x55 /* U */ ) {
    return 8;
  }
  return 0;
}
function fromDecimalCode(c) {
  if (0x30 <= /* 0 */ c && c <= 0x39 /* 9 */ ) {
    return c - 0x30;
  }
  return -1;
}
function simpleEscapeSequence(c) {
  return c === 0x30 /* 0 */  ? "\x00" : c === 0x61 /* a */  ? "\x07" : c === 0x62 /* b */  ? "\x08" : c === 0x74 /* t */  ? "\x09" : c === 0x09 /* Tab */  ? "\x09" : c === 0x6e /* n */  ? "\x0A" : c === 0x76 /* v */  ? "\x0B" : c === 0x66 /* f */  ? "\x0C" : c === 0x72 /* r */  ? "\x0D" : c === 0x65 /* e */  ? "\x1B" : c === 0x20 /* Space */  ? " " : c === 0x22 /* " */  ? "\x22" : c === 0x2f /* / */  ? "/" : c === 0x5c /* \ */  ? "\x5C" : c === 0x4e /* N */  ? "\x85" : c === 0x5f /* _ */  ? "\xA0" : c === 0x4c /* L */  ? "\u2028" : c === 0x50 /* P */  ? "\u2029" : "";
}
function charFromCodepoint(c) {
  if (c <= 0xffff) {
    return String.fromCharCode(c);
  }
  // Encode UTF-16 surrogate pair
  // https://en.wikipedia.org/wiki/UTF-16#Code_points_U.2B010000_to_U.2B10FFFF
  return String.fromCharCode((c - 0x010000 >> 10) + 0xd800, (c - 0x010000 & 0x03ff) + 0xdc00);
}
const simpleEscapeCheck = Array.from({
  length: 256
}); // integer, for fast access
const simpleEscapeMap = Array.from({
  length: 256
});
for(let i = 0; i < 256; i++){
  simpleEscapeCheck[i] = simpleEscapeSequence(i) ? 1 : 0;
  simpleEscapeMap[i] = simpleEscapeSequence(i);
}
function generateError(state, message) {
  return new YAMLError(message, new Mark(state.filename, state.input, state.position, state.line, state.position - state.lineStart));
}
function throwError(state, message) {
  throw generateError(state, message);
}
function throwWarning(state, message) {
  if (state.onWarning) {
    state.onWarning.call(null, generateError(state, message));
  }
}
const directiveHandlers = {
  YAML (state, _name, ...args) {
    if (state.version !== null) {
      return throwError(state, "duplication of %YAML directive");
    }
    if (args.length !== 1) {
      return throwError(state, "YAML directive accepts exactly one argument");
    }
    const match = /^([0-9]+)\.([0-9]+)$/.exec(args[0]);
    if (match === null) {
      return throwError(state, "ill-formed argument of the YAML directive");
    }
    const major = parseInt(match[1], 10);
    const minor = parseInt(match[2], 10);
    if (major !== 1) {
      return throwError(state, "unacceptable YAML version of the document");
    }
    state.version = args[0];
    state.checkLineBreaks = minor < 2;
    if (minor !== 1 && minor !== 2) {
      return throwWarning(state, "unsupported YAML version of the document");
    }
  },
  TAG (state, _name, ...args) {
    if (args.length !== 2) {
      return throwError(state, "TAG directive accepts exactly two arguments");
    }
    const handle = args[0];
    const prefix = args[1];
    if (!PATTERN_TAG_HANDLE.test(handle)) {
      return throwError(state, "ill-formed tag handle (first argument) of the TAG directive");
    }
    if (state.tagMap && hasOwn(state.tagMap, handle)) {
      return throwError(state, `there is a previously declared suffix for "${handle}" tag handle`);
    }
    if (!PATTERN_TAG_URI.test(prefix)) {
      return throwError(state, "ill-formed tag prefix (second argument) of the TAG directive");
    }
    if (typeof state.tagMap === "undefined") {
      state.tagMap = {};
    }
    state.tagMap[handle] = prefix;
  }
};
function captureSegment(state, start, end, checkJson) {
  let result;
  if (start < end) {
    result = state.input.slice(start, end);
    if (checkJson) {
      for(let position = 0, length = result.length; position < length; position++){
        const character = result.charCodeAt(position);
        if (!(character === 0x09 || 0x20 <= character && character <= 0x10ffff)) {
          return throwError(state, "expected valid JSON character");
        }
      }
    } else if (PATTERN_NON_PRINTABLE.test(result)) {
      return throwError(state, "the stream contains non-printable characters");
    }
    state.result += result;
  }
}
function mergeMappings(state, destination, source, overridableKeys) {
  if (!common.isObject(source)) {
    return throwError(state, "cannot merge mappings; the provided source object is unacceptable");
  }
  const keys = Object.keys(source);
  for(let i = 0, len = keys.length; i < len; i++){
    const key = keys[i];
    if (!hasOwn(destination, key)) {
      destination[key] = source[key];
      overridableKeys[key] = true;
    }
  }
}
function storeMappingPair(state, result, overridableKeys, keyTag, keyNode, valueNode, startLine, startPos) {
  // The output is a plain object here, so keys can only be strings.
  // We need to convert keyNode to a string, but doing so can hang the process
  // (deeply nested arrays that explode exponentially using aliases).
  if (Array.isArray(keyNode)) {
    keyNode = Array.prototype.slice.call(keyNode);
    for(let index = 0, quantity = keyNode.length; index < quantity; index++){
      if (Array.isArray(keyNode[index])) {
        return throwError(state, "nested arrays are not supported inside keys");
      }
      if (typeof keyNode === "object" && _class(keyNode[index]) === "[object Object]") {
        keyNode[index] = "[object Object]";
      }
    }
  }
  // Avoid code execution in load() via toString property
  // (still use its own toString for arrays, timestamps,
  // and whatever user schema extensions happen to have @@toStringTag)
  if (typeof keyNode === "object" && _class(keyNode) === "[object Object]") {
    keyNode = "[object Object]";
  }
  keyNode = String(keyNode);
  if (result === null) {
    result = {};
  }
  if (keyTag === "tag:yaml.org,2002:merge") {
    if (Array.isArray(valueNode)) {
      for(let index = 0, quantity = valueNode.length; index < quantity; index++){
        mergeMappings(state, result, valueNode[index], overridableKeys);
      }
    } else {
      mergeMappings(state, result, valueNode, overridableKeys);
    }
  } else {
    if (!state.json && !hasOwn(overridableKeys, keyNode) && hasOwn(result, keyNode)) {
      state.line = startLine || state.line;
      state.position = startPos || state.position;
      return throwError(state, "duplicated mapping key");
    }
    result[keyNode] = valueNode;
    delete overridableKeys[keyNode];
  }
  return result;
}
function readLineBreak(state) {
  const ch = state.input.charCodeAt(state.position);
  if (ch === 0x0a /* LF */ ) {
    state.position++;
  } else if (ch === 0x0d /* CR */ ) {
    state.position++;
    if (state.input.charCodeAt(state.position) === 0x0a /* LF */ ) {
      state.position++;
    }
  } else {
    return throwError(state, "a line break is expected");
  }
  state.line += 1;
  state.lineStart = state.position;
}
function skipSeparationSpace(state, allowComments, checkIndent) {
  let lineBreaks = 0, ch = state.input.charCodeAt(state.position);
  while(ch !== 0){
    while(isWhiteSpace(ch)){
      ch = state.input.charCodeAt(++state.position);
    }
    if (allowComments && ch === 0x23 /* # */ ) {
      do {
        ch = state.input.charCodeAt(++state.position);
      }while (ch !== 0x0a && /* LF */ ch !== 0x0d && /* CR */ ch !== 0)
    }
    if (isEOL(ch)) {
      readLineBreak(state);
      ch = state.input.charCodeAt(state.position);
      lineBreaks++;
      state.lineIndent = 0;
      while(ch === 0x20 /* Space */ ){
        state.lineIndent++;
        ch = state.input.charCodeAt(++state.position);
      }
    } else {
      break;
    }
  }
  if (checkIndent !== -1 && lineBreaks !== 0 && state.lineIndent < checkIndent) {
    throwWarning(state, "deficient indentation");
  }
  return lineBreaks;
}
function testDocumentSeparator(state) {
  let _position = state.position;
  let ch = state.input.charCodeAt(_position);
  // Condition state.position === state.lineStart is tested
  // in parent on each call, for efficiency. No needs to test here again.
  if ((ch === 0x2d || /* - */ ch === 0x2e) && ch === state.input.charCodeAt(_position + 1) && ch === state.input.charCodeAt(_position + 2)) {
    _position += 3;
    ch = state.input.charCodeAt(_position);
    if (ch === 0 || isWsOrEol(ch)) {
      return true;
    }
  }
  return false;
}
function writeFoldedLines(state, count) {
  if (count === 1) {
    state.result += " ";
  } else if (count > 1) {
    state.result += common.repeat("\n", count - 1);
  }
}
function readPlainScalar(state, nodeIndent, withinFlowCollection) {
  const kind = state.kind;
  const result = state.result;
  let ch = state.input.charCodeAt(state.position);
  if (isWsOrEol(ch) || isFlowIndicator(ch) || ch === 0x23 /* # */  || ch === 0x26 /* & */  || ch === 0x2a /* * */  || ch === 0x21 /* ! */  || ch === 0x7c /* | */  || ch === 0x3e /* > */  || ch === 0x27 /* ' */  || ch === 0x22 /* " */  || ch === 0x25 /* % */  || ch === 0x40 /* @ */  || ch === 0x60 /* ` */ ) {
    return false;
  }
  let following;
  if (ch === 0x3f || /* ? */ ch === 0x2d /* - */ ) {
    following = state.input.charCodeAt(state.position + 1);
    if (isWsOrEol(following) || withinFlowCollection && isFlowIndicator(following)) {
      return false;
    }
  }
  state.kind = "scalar";
  state.result = "";
  let captureEnd, captureStart = captureEnd = state.position;
  let hasPendingContent = false;
  let line = 0;
  while(ch !== 0){
    if (ch === 0x3a /* : */ ) {
      following = state.input.charCodeAt(state.position + 1);
      if (isWsOrEol(following) || withinFlowCollection && isFlowIndicator(following)) {
        break;
      }
    } else if (ch === 0x23 /* # */ ) {
      const preceding = state.input.charCodeAt(state.position - 1);
      if (isWsOrEol(preceding)) {
        break;
      }
    } else if (state.position === state.lineStart && testDocumentSeparator(state) || withinFlowCollection && isFlowIndicator(ch)) {
      break;
    } else if (isEOL(ch)) {
      line = state.line;
      const lineStart = state.lineStart;
      const lineIndent = state.lineIndent;
      skipSeparationSpace(state, false, -1);
      if (state.lineIndent >= nodeIndent) {
        hasPendingContent = true;
        ch = state.input.charCodeAt(state.position);
        continue;
      } else {
        state.position = captureEnd;
        state.line = line;
        state.lineStart = lineStart;
        state.lineIndent = lineIndent;
        break;
      }
    }
    if (hasPendingContent) {
      captureSegment(state, captureStart, captureEnd, false);
      writeFoldedLines(state, state.line - line);
      captureStart = captureEnd = state.position;
      hasPendingContent = false;
    }
    if (!isWhiteSpace(ch)) {
      captureEnd = state.position + 1;
    }
    ch = state.input.charCodeAt(++state.position);
  }
  captureSegment(state, captureStart, captureEnd, false);
  if (state.result) {
    return true;
  }
  state.kind = kind;
  state.result = result;
  return false;
}
function readSingleQuotedScalar(state, nodeIndent) {
  let ch, captureStart, captureEnd;
  ch = state.input.charCodeAt(state.position);
  if (ch !== 0x27 /* ' */ ) {
    return false;
  }
  state.kind = "scalar";
  state.result = "";
  state.position++;
  captureStart = captureEnd = state.position;
  while((ch = state.input.charCodeAt(state.position)) !== 0){
    if (ch === 0x27 /* ' */ ) {
      captureSegment(state, captureStart, state.position, true);
      ch = state.input.charCodeAt(++state.position);
      if (ch === 0x27 /* ' */ ) {
        captureStart = state.position;
        state.position++;
        captureEnd = state.position;
      } else {
        return true;
      }
    } else if (isEOL(ch)) {
      captureSegment(state, captureStart, captureEnd, true);
      writeFoldedLines(state, skipSeparationSpace(state, false, nodeIndent));
      captureStart = captureEnd = state.position;
    } else if (state.position === state.lineStart && testDocumentSeparator(state)) {
      return throwError(state, "unexpected end of the document within a single quoted scalar");
    } else {
      state.position++;
      captureEnd = state.position;
    }
  }
  return throwError(state, "unexpected end of the stream within a single quoted scalar");
}
function readDoubleQuotedScalar(state, nodeIndent) {
  let ch = state.input.charCodeAt(state.position);
  if (ch !== 0x22 /* " */ ) {
    return false;
  }
  state.kind = "scalar";
  state.result = "";
  state.position++;
  let captureEnd, captureStart = captureEnd = state.position;
  let tmp;
  while((ch = state.input.charCodeAt(state.position)) !== 0){
    if (ch === 0x22 /* " */ ) {
      captureSegment(state, captureStart, state.position, true);
      state.position++;
      return true;
    }
    if (ch === 0x5c /* \ */ ) {
      captureSegment(state, captureStart, state.position, true);
      ch = state.input.charCodeAt(++state.position);
      if (isEOL(ch)) {
        skipSeparationSpace(state, false, nodeIndent);
      // TODO(bartlomieju): rework to inline fn with no type cast?
      } else if (ch < 256 && simpleEscapeCheck[ch]) {
        state.result += simpleEscapeMap[ch];
        state.position++;
      } else if ((tmp = escapedHexLen(ch)) > 0) {
        let hexLength = tmp;
        let hexResult = 0;
        for(; hexLength > 0; hexLength--){
          ch = state.input.charCodeAt(++state.position);
          if ((tmp = fromHexCode(ch)) >= 0) {
            hexResult = (hexResult << 4) + tmp;
          } else {
            return throwError(state, "expected hexadecimal character");
          }
        }
        state.result += charFromCodepoint(hexResult);
        state.position++;
      } else {
        return throwError(state, "unknown escape sequence");
      }
      captureStart = captureEnd = state.position;
    } else if (isEOL(ch)) {
      captureSegment(state, captureStart, captureEnd, true);
      writeFoldedLines(state, skipSeparationSpace(state, false, nodeIndent));
      captureStart = captureEnd = state.position;
    } else if (state.position === state.lineStart && testDocumentSeparator(state)) {
      return throwError(state, "unexpected end of the document within a double quoted scalar");
    } else {
      state.position++;
      captureEnd = state.position;
    }
  }
  return throwError(state, "unexpected end of the stream within a double quoted scalar");
}
function readFlowCollection(state, nodeIndent) {
  let ch = state.input.charCodeAt(state.position);
  let terminator;
  let isMapping = true;
  let result = {};
  if (ch === 0x5b /* [ */ ) {
    terminator = 0x5d; /* ] */ 
    isMapping = false;
    result = [];
  } else if (ch === 0x7b /* { */ ) {
    terminator = 0x7d; /* } */ 
  } else {
    return false;
  }
  if (state.anchor !== null && typeof state.anchor != "undefined" && typeof state.anchorMap != "undefined") {
    state.anchorMap[state.anchor] = result;
  }
  ch = state.input.charCodeAt(++state.position);
  const tag = state.tag, anchor = state.anchor;
  let readNext = true;
  let valueNode, keyNode, keyTag = keyNode = valueNode = null, isExplicitPair, isPair = isExplicitPair = false;
  let following = 0, line = 0;
  const overridableKeys = {};
  while(ch !== 0){
    skipSeparationSpace(state, true, nodeIndent);
    ch = state.input.charCodeAt(state.position);
    if (ch === terminator) {
      state.position++;
      state.tag = tag;
      state.anchor = anchor;
      state.kind = isMapping ? "mapping" : "sequence";
      state.result = result;
      return true;
    }
    if (!readNext) {
      return throwError(state, "missed comma between flow collection entries");
    }
    keyTag = keyNode = valueNode = null;
    isPair = isExplicitPair = false;
    if (ch === 0x3f /* ? */ ) {
      following = state.input.charCodeAt(state.position + 1);
      if (isWsOrEol(following)) {
        isPair = isExplicitPair = true;
        state.position++;
        skipSeparationSpace(state, true, nodeIndent);
      }
    }
    line = state.line;
    composeNode(state, nodeIndent, CONTEXT_FLOW_IN, false, true);
    keyTag = state.tag || null;
    keyNode = state.result;
    skipSeparationSpace(state, true, nodeIndent);
    ch = state.input.charCodeAt(state.position);
    if ((isExplicitPair || state.line === line) && ch === 0x3a /* : */ ) {
      isPair = true;
      ch = state.input.charCodeAt(++state.position);
      skipSeparationSpace(state, true, nodeIndent);
      composeNode(state, nodeIndent, CONTEXT_FLOW_IN, false, true);
      valueNode = state.result;
    }
    if (isMapping) {
      storeMappingPair(state, result, overridableKeys, keyTag, keyNode, valueNode);
    } else if (isPair) {
      result.push(storeMappingPair(state, null, overridableKeys, keyTag, keyNode, valueNode));
    } else {
      result.push(keyNode);
    }
    skipSeparationSpace(state, true, nodeIndent);
    ch = state.input.charCodeAt(state.position);
    if (ch === 0x2c /* , */ ) {
      readNext = true;
      ch = state.input.charCodeAt(++state.position);
    } else {
      readNext = false;
    }
  }
  return throwError(state, "unexpected end of the stream within a flow collection");
}
function readBlockScalar(state, nodeIndent) {
  let chomping = CHOMPING_CLIP, didReadContent = false, detectedIndent = false, textIndent = nodeIndent, emptyLines = 0, atMoreIndented = false;
  let ch = state.input.charCodeAt(state.position);
  let folding = false;
  if (ch === 0x7c /* | */ ) {
    folding = false;
  } else if (ch === 0x3e /* > */ ) {
    folding = true;
  } else {
    return false;
  }
  state.kind = "scalar";
  state.result = "";
  let tmp = 0;
  while(ch !== 0){
    ch = state.input.charCodeAt(++state.position);
    if (ch === 0x2b || /* + */ ch === 0x2d /* - */ ) {
      if (CHOMPING_CLIP === chomping) {
        chomping = ch === 0x2b /* + */  ? CHOMPING_KEEP : CHOMPING_STRIP;
      } else {
        return throwError(state, "repeat of a chomping mode identifier");
      }
    } else if ((tmp = fromDecimalCode(ch)) >= 0) {
      if (tmp === 0) {
        return throwError(state, "bad explicit indentation width of a block scalar; it cannot be less than one");
      } else if (!detectedIndent) {
        textIndent = nodeIndent + tmp - 1;
        detectedIndent = true;
      } else {
        return throwError(state, "repeat of an indentation width identifier");
      }
    } else {
      break;
    }
  }
  if (isWhiteSpace(ch)) {
    do {
      ch = state.input.charCodeAt(++state.position);
    }while (isWhiteSpace(ch))
    if (ch === 0x23 /* # */ ) {
      do {
        ch = state.input.charCodeAt(++state.position);
      }while (!isEOL(ch) && ch !== 0)
    }
  }
  while(ch !== 0){
    readLineBreak(state);
    state.lineIndent = 0;
    ch = state.input.charCodeAt(state.position);
    while((!detectedIndent || state.lineIndent < textIndent) && ch === 0x20 /* Space */ ){
      state.lineIndent++;
      ch = state.input.charCodeAt(++state.position);
    }
    if (!detectedIndent && state.lineIndent > textIndent) {
      textIndent = state.lineIndent;
    }
    if (isEOL(ch)) {
      emptyLines++;
      continue;
    }
    // End of the scalar.
    if (state.lineIndent < textIndent) {
      // Perform the chomping.
      if (chomping === CHOMPING_KEEP) {
        state.result += common.repeat("\n", didReadContent ? 1 + emptyLines : emptyLines);
      } else if (chomping === CHOMPING_CLIP) {
        if (didReadContent) {
          // i.e. only if the scalar is not empty.
          state.result += "\n";
        }
      }
      break;
    }
    // Folded style: use fancy rules to handle line breaks.
    if (folding) {
      // Lines starting with white space characters (more-indented lines) are not folded.
      if (isWhiteSpace(ch)) {
        atMoreIndented = true;
        // except for the first content line (cf. Example 8.1)
        state.result += common.repeat("\n", didReadContent ? 1 + emptyLines : emptyLines);
      // End of more-indented block.
      } else if (atMoreIndented) {
        atMoreIndented = false;
        state.result += common.repeat("\n", emptyLines + 1);
      // Just one line break - perceive as the same line.
      } else if (emptyLines === 0) {
        if (didReadContent) {
          // i.e. only if we have already read some scalar content.
          state.result += " ";
        }
      // Several line breaks - perceive as different lines.
      } else {
        state.result += common.repeat("\n", emptyLines);
      }
    // Literal style: just add exact number of line breaks between content lines.
    } else {
      // Keep all line breaks except the header line break.
      state.result += common.repeat("\n", didReadContent ? 1 + emptyLines : emptyLines);
    }
    didReadContent = true;
    detectedIndent = true;
    emptyLines = 0;
    const captureStart = state.position;
    while(!isEOL(ch) && ch !== 0){
      ch = state.input.charCodeAt(++state.position);
    }
    captureSegment(state, captureStart, state.position, false);
  }
  return true;
}
function readBlockSequence(state, nodeIndent) {
  let line, following, detected = false, ch;
  const tag = state.tag, anchor = state.anchor, result = [];
  if (state.anchor !== null && typeof state.anchor !== "undefined" && typeof state.anchorMap !== "undefined") {
    state.anchorMap[state.anchor] = result;
  }
  ch = state.input.charCodeAt(state.position);
  while(ch !== 0){
    if (ch !== 0x2d /* - */ ) {
      break;
    }
    following = state.input.charCodeAt(state.position + 1);
    if (!isWsOrEol(following)) {
      break;
    }
    detected = true;
    state.position++;
    if (skipSeparationSpace(state, true, -1)) {
      if (state.lineIndent <= nodeIndent) {
        result.push(null);
        ch = state.input.charCodeAt(state.position);
        continue;
      }
    }
    line = state.line;
    composeNode(state, nodeIndent, CONTEXT_BLOCK_IN, false, true);
    result.push(state.result);
    skipSeparationSpace(state, true, -1);
    ch = state.input.charCodeAt(state.position);
    if ((state.line === line || state.lineIndent > nodeIndent) && ch !== 0) {
      return throwError(state, "bad indentation of a sequence entry");
    } else if (state.lineIndent < nodeIndent) {
      break;
    }
  }
  if (detected) {
    state.tag = tag;
    state.anchor = anchor;
    state.kind = "sequence";
    state.result = result;
    return true;
  }
  return false;
}
function readBlockMapping(state, nodeIndent, flowIndent) {
  const tag = state.tag, anchor = state.anchor, result = {}, overridableKeys = {};
  let following, allowCompact = false, line, pos, keyTag = null, keyNode = null, valueNode = null, atExplicitKey = false, detected = false, ch;
  if (state.anchor !== null && typeof state.anchor !== "undefined" && typeof state.anchorMap !== "undefined") {
    state.anchorMap[state.anchor] = result;
  }
  ch = state.input.charCodeAt(state.position);
  while(ch !== 0){
    following = state.input.charCodeAt(state.position + 1);
    line = state.line; // Save the current line.
    pos = state.position;
    //
    // Explicit notation case. There are two separate blocks:
    // first for the key (denoted by "?") and second for the value (denoted by ":")
    //
    if ((ch === 0x3f || /* ? */ ch === 0x3a) && /* : */ isWsOrEol(following)) {
      if (ch === 0x3f /* ? */ ) {
        if (atExplicitKey) {
          storeMappingPair(state, result, overridableKeys, keyTag, keyNode, null);
          keyTag = keyNode = valueNode = null;
        }
        detected = true;
        atExplicitKey = true;
        allowCompact = true;
      } else if (atExplicitKey) {
        // i.e. 0x3A/* : */ === character after the explicit key.
        atExplicitKey = false;
        allowCompact = true;
      } else {
        return throwError(state, "incomplete explicit mapping pair; a key node is missed; or followed by a non-tabulated empty line");
      }
      state.position += 1;
      ch = following;
    //
    // Implicit notation case. Flow-style node as the key first, then ":", and the value.
    //
    } else if (composeNode(state, flowIndent, CONTEXT_FLOW_OUT, false, true)) {
      if (state.line === line) {
        ch = state.input.charCodeAt(state.position);
        while(isWhiteSpace(ch)){
          ch = state.input.charCodeAt(++state.position);
        }
        if (ch === 0x3a /* : */ ) {
          ch = state.input.charCodeAt(++state.position);
          if (!isWsOrEol(ch)) {
            return throwError(state, "a whitespace character is expected after the key-value separator within a block mapping");
          }
          if (atExplicitKey) {
            storeMappingPair(state, result, overridableKeys, keyTag, keyNode, null);
            keyTag = keyNode = valueNode = null;
          }
          detected = true;
          atExplicitKey = false;
          allowCompact = false;
          keyTag = state.tag;
          keyNode = state.result;
        } else if (detected) {
          return throwError(state, "can not read an implicit mapping pair; a colon is missed");
        } else {
          state.tag = tag;
          state.anchor = anchor;
          return true; // Keep the result of `composeNode`.
        }
      } else if (detected) {
        return throwError(state, "can not read a block mapping entry; a multiline key may not be an implicit key");
      } else {
        state.tag = tag;
        state.anchor = anchor;
        return true; // Keep the result of `composeNode`.
      }
    } else {
      break; // Reading is done. Go to the epilogue.
    }
    //
    // Common reading code for both explicit and implicit notations.
    //
    if (state.line === line || state.lineIndent > nodeIndent) {
      if (composeNode(state, nodeIndent, CONTEXT_BLOCK_OUT, true, allowCompact)) {
        if (atExplicitKey) {
          keyNode = state.result;
        } else {
          valueNode = state.result;
        }
      }
      if (!atExplicitKey) {
        storeMappingPair(state, result, overridableKeys, keyTag, keyNode, valueNode, line, pos);
        keyTag = keyNode = valueNode = null;
      }
      skipSeparationSpace(state, true, -1);
      ch = state.input.charCodeAt(state.position);
    }
    if (state.lineIndent > nodeIndent && ch !== 0) {
      return throwError(state, "bad indentation of a mapping entry");
    } else if (state.lineIndent < nodeIndent) {
      break;
    }
  }
  //
  // Epilogue.
  //
  // Special case: last mapping's node contains only the key in explicit notation.
  if (atExplicitKey) {
    storeMappingPair(state, result, overridableKeys, keyTag, keyNode, null);
  }
  // Expose the resulting mapping.
  if (detected) {
    state.tag = tag;
    state.anchor = anchor;
    state.kind = "mapping";
    state.result = result;
  }
  return detected;
}
function readTagProperty(state) {
  let position, isVerbatim = false, isNamed = false, tagHandle = "", tagName, ch;
  ch = state.input.charCodeAt(state.position);
  if (ch !== 0x21 /* ! */ ) return false;
  if (state.tag !== null) {
    return throwError(state, "duplication of a tag property");
  }
  ch = state.input.charCodeAt(++state.position);
  if (ch === 0x3c /* < */ ) {
    isVerbatim = true;
    ch = state.input.charCodeAt(++state.position);
  } else if (ch === 0x21 /* ! */ ) {
    isNamed = true;
    tagHandle = "!!";
    ch = state.input.charCodeAt(++state.position);
  } else {
    tagHandle = "!";
  }
  position = state.position;
  if (isVerbatim) {
    do {
      ch = state.input.charCodeAt(++state.position);
    }while (ch !== 0 && ch !== 0x3e /* > */ )
    if (state.position < state.length) {
      tagName = state.input.slice(position, state.position);
      ch = state.input.charCodeAt(++state.position);
    } else {
      return throwError(state, "unexpected end of the stream within a verbatim tag");
    }
  } else {
    while(ch !== 0 && !isWsOrEol(ch)){
      if (ch === 0x21 /* ! */ ) {
        if (!isNamed) {
          tagHandle = state.input.slice(position - 1, state.position + 1);
          if (!PATTERN_TAG_HANDLE.test(tagHandle)) {
            return throwError(state, "named tag handle cannot contain such characters");
          }
          isNamed = true;
          position = state.position + 1;
        } else {
          return throwError(state, "tag suffix cannot contain exclamation marks");
        }
      }
      ch = state.input.charCodeAt(++state.position);
    }
    tagName = state.input.slice(position, state.position);
    if (PATTERN_FLOW_INDICATORS.test(tagName)) {
      return throwError(state, "tag suffix cannot contain flow indicator characters");
    }
  }
  if (tagName && !PATTERN_TAG_URI.test(tagName)) {
    return throwError(state, `tag name cannot contain such characters: ${tagName}`);
  }
  if (isVerbatim) {
    state.tag = tagName;
  } else if (typeof state.tagMap !== "undefined" && hasOwn(state.tagMap, tagHandle)) {
    state.tag = state.tagMap[tagHandle] + tagName;
  } else if (tagHandle === "!") {
    state.tag = `!${tagName}`;
  } else if (tagHandle === "!!") {
    state.tag = `tag:yaml.org,2002:${tagName}`;
  } else {
    return throwError(state, `undeclared tag handle "${tagHandle}"`);
  }
  return true;
}
function readAnchorProperty(state) {
  let ch = state.input.charCodeAt(state.position);
  if (ch !== 0x26 /* & */ ) return false;
  if (state.anchor !== null) {
    return throwError(state, "duplication of an anchor property");
  }
  ch = state.input.charCodeAt(++state.position);
  const position = state.position;
  while(ch !== 0 && !isWsOrEol(ch) && !isFlowIndicator(ch)){
    ch = state.input.charCodeAt(++state.position);
  }
  if (state.position === position) {
    return throwError(state, "name of an anchor node must contain at least one character");
  }
  state.anchor = state.input.slice(position, state.position);
  return true;
}
function readAlias(state) {
  let ch = state.input.charCodeAt(state.position);
  if (ch !== 0x2a /* * */ ) return false;
  ch = state.input.charCodeAt(++state.position);
  const _position = state.position;
  while(ch !== 0 && !isWsOrEol(ch) && !isFlowIndicator(ch)){
    ch = state.input.charCodeAt(++state.position);
  }
  if (state.position === _position) {
    return throwError(state, "name of an alias node must contain at least one character");
  }
  const alias = state.input.slice(_position, state.position);
  if (typeof state.anchorMap !== "undefined" && !hasOwn(state.anchorMap, alias)) {
    return throwError(state, `unidentified alias "${alias}"`);
  }
  if (typeof state.anchorMap !== "undefined") {
    state.result = state.anchorMap[alias];
  }
  skipSeparationSpace(state, true, -1);
  return true;
}
function composeNode(state, parentIndent, nodeContext, allowToSeek, allowCompact) {
  let allowBlockScalars, allowBlockCollections, indentStatus = 1, atNewLine = false, hasContent = false, type, flowIndent, blockIndent;
  if (state.listener && state.listener !== null) {
    state.listener("open", state);
  }
  state.tag = null;
  state.anchor = null;
  state.kind = null;
  state.result = null;
  const allowBlockStyles = allowBlockScalars = allowBlockCollections = CONTEXT_BLOCK_OUT === nodeContext || CONTEXT_BLOCK_IN === nodeContext;
  if (allowToSeek) {
    if (skipSeparationSpace(state, true, -1)) {
      atNewLine = true;
      if (state.lineIndent > parentIndent) {
        indentStatus = 1;
      } else if (state.lineIndent === parentIndent) {
        indentStatus = 0;
      } else if (state.lineIndent < parentIndent) {
        indentStatus = -1;
      }
    }
  }
  if (indentStatus === 1) {
    while(readTagProperty(state) || readAnchorProperty(state)){
      if (skipSeparationSpace(state, true, -1)) {
        atNewLine = true;
        allowBlockCollections = allowBlockStyles;
        if (state.lineIndent > parentIndent) {
          indentStatus = 1;
        } else if (state.lineIndent === parentIndent) {
          indentStatus = 0;
        } else if (state.lineIndent < parentIndent) {
          indentStatus = -1;
        }
      } else {
        allowBlockCollections = false;
      }
    }
  }
  if (allowBlockCollections) {
    allowBlockCollections = atNewLine || allowCompact;
  }
  if (indentStatus === 1 || CONTEXT_BLOCK_OUT === nodeContext) {
    const cond = CONTEXT_FLOW_IN === nodeContext || CONTEXT_FLOW_OUT === nodeContext;
    flowIndent = cond ? parentIndent : parentIndent + 1;
    blockIndent = state.position - state.lineStart;
    if (indentStatus === 1) {
      if (allowBlockCollections && (readBlockSequence(state, blockIndent) || readBlockMapping(state, blockIndent, flowIndent)) || readFlowCollection(state, flowIndent)) {
        hasContent = true;
      } else {
        if (allowBlockScalars && readBlockScalar(state, flowIndent) || readSingleQuotedScalar(state, flowIndent) || readDoubleQuotedScalar(state, flowIndent)) {
          hasContent = true;
        } else if (readAlias(state)) {
          hasContent = true;
          if (state.tag !== null || state.anchor !== null) {
            return throwError(state, "alias node should not have Any properties");
          }
        } else if (readPlainScalar(state, flowIndent, CONTEXT_FLOW_IN === nodeContext)) {
          hasContent = true;
          if (state.tag === null) {
            state.tag = "?";
          }
        }
        if (state.anchor !== null && typeof state.anchorMap !== "undefined") {
          state.anchorMap[state.anchor] = state.result;
        }
      }
    } else if (indentStatus === 0) {
      // Special case: block sequences are allowed to have same indentation level as the parent.
      // http://www.yaml.org/spec/1.2/spec.html#id2799784
      hasContent = allowBlockCollections && readBlockSequence(state, blockIndent);
    }
  }
  if (state.tag !== null && state.tag !== "!") {
    if (state.tag === "?") {
      for(let typeIndex = 0, typeQuantity = state.implicitTypes.length; typeIndex < typeQuantity; typeIndex++){
        type = state.implicitTypes[typeIndex];
        // Implicit resolving is not allowed for non-scalar types, and '?'
        // non-specific tag is only assigned to plain scalars. So, it isn't
        // needed to check for 'kind' conformity.
        if (type.resolve(state.result)) {
          // `state.result` updated in resolver if matched
          state.result = type.construct(state.result);
          state.tag = type.tag;
          if (state.anchor !== null && typeof state.anchorMap !== "undefined") {
            state.anchorMap[state.anchor] = state.result;
          }
          break;
        }
      }
    } else if (hasOwn(state.typeMap[state.kind || "fallback"], state.tag)) {
      type = state.typeMap[state.kind || "fallback"][state.tag];
      if (state.result !== null && type.kind !== state.kind) {
        return throwError(state, `unacceptable node kind for !<${state.tag}> tag; it should be "${type.kind}", not "${state.kind}"`);
      }
      if (!type.resolve(state.result)) {
        // `state.result` updated in resolver if matched
        return throwError(state, `cannot resolve a node with !<${state.tag}> explicit tag`);
      } else {
        state.result = type.construct(state.result);
        if (state.anchor !== null && typeof state.anchorMap !== "undefined") {
          state.anchorMap[state.anchor] = state.result;
        }
      }
    } else {
      return throwError(state, `unknown tag !<${state.tag}>`);
    }
  }
  if (state.listener && state.listener !== null) {
    state.listener("close", state);
  }
  return state.tag !== null || state.anchor !== null || hasContent;
}
function readDocument(state) {
  const documentStart = state.position;
  let position, directiveName, directiveArgs, hasDirectives = false, ch;
  state.version = null;
  state.checkLineBreaks = state.legacy;
  state.tagMap = {};
  state.anchorMap = {};
  while((ch = state.input.charCodeAt(state.position)) !== 0){
    skipSeparationSpace(state, true, -1);
    ch = state.input.charCodeAt(state.position);
    if (state.lineIndent > 0 || ch !== 0x25 /* % */ ) {
      break;
    }
    hasDirectives = true;
    ch = state.input.charCodeAt(++state.position);
    position = state.position;
    while(ch !== 0 && !isWsOrEol(ch)){
      ch = state.input.charCodeAt(++state.position);
    }
    directiveName = state.input.slice(position, state.position);
    directiveArgs = [];
    if (directiveName.length < 1) {
      return throwError(state, "directive name must not be less than one character in length");
    }
    while(ch !== 0){
      while(isWhiteSpace(ch)){
        ch = state.input.charCodeAt(++state.position);
      }
      if (ch === 0x23 /* # */ ) {
        do {
          ch = state.input.charCodeAt(++state.position);
        }while (ch !== 0 && !isEOL(ch))
        break;
      }
      if (isEOL(ch)) break;
      position = state.position;
      while(ch !== 0 && !isWsOrEol(ch)){
        ch = state.input.charCodeAt(++state.position);
      }
      directiveArgs.push(state.input.slice(position, state.position));
    }
    if (ch !== 0) readLineBreak(state);
    if (hasOwn(directiveHandlers, directiveName)) {
      directiveHandlers[directiveName](state, directiveName, ...directiveArgs);
    } else {
      throwWarning(state, `unknown document directive "${directiveName}"`);
    }
  }
  skipSeparationSpace(state, true, -1);
  if (state.lineIndent === 0 && state.input.charCodeAt(state.position) === 0x2d /* - */  && state.input.charCodeAt(state.position + 1) === 0x2d /* - */  && state.input.charCodeAt(state.position + 2) === 0x2d /* - */ ) {
    state.position += 3;
    skipSeparationSpace(state, true, -1);
  } else if (hasDirectives) {
    return throwError(state, "directives end mark is expected");
  }
  composeNode(state, state.lineIndent - 1, CONTEXT_BLOCK_OUT, false, true);
  skipSeparationSpace(state, true, -1);
  if (state.checkLineBreaks && PATTERN_NON_ASCII_LINE_BREAKS.test(state.input.slice(documentStart, state.position))) {
    throwWarning(state, "non-ASCII line breaks are interpreted as content");
  }
  state.documents.push(state.result);
  if (state.position === state.lineStart && testDocumentSeparator(state)) {
    if (state.input.charCodeAt(state.position) === 0x2e /* . */ ) {
      state.position += 3;
      skipSeparationSpace(state, true, -1);
    }
    return;
  }
  if (state.position < state.length - 1) {
    return throwError(state, "end of the stream or a document separator is expected");
  } else {
    return;
  }
}
function loadDocuments(input, options) {
  input = String(input);
  options = options || {};
  if (input.length !== 0) {
    // Add tailing `\n` if not exists
    if (input.charCodeAt(input.length - 1) !== 0x0a /* LF */  && input.charCodeAt(input.length - 1) !== 0x0d /* CR */ ) {
      input += "\n";
    }
    // Strip BOM
    if (input.charCodeAt(0) === 0xfeff) {
      input = input.slice(1);
    }
  }
  const state = new LoaderState(input, options);
  // Use 0 as string terminator. That significantly simplifies bounds check.
  state.input += "\0";
  while(state.input.charCodeAt(state.position) === 0x20 /* Space */ ){
    state.lineIndent += 1;
    state.position += 1;
  }
  while(state.position < state.length - 1){
    readDocument(state);
  }
  return state.documents;
}
function isCbFunction(fn) {
  return typeof fn === "function";
}
export function loadAll(input, iteratorOrOption, options) {
  if (!isCbFunction(iteratorOrOption)) {
    return loadDocuments(input, iteratorOrOption);
  }
  const documents = loadDocuments(input, options);
  const iterator = iteratorOrOption;
  for(let index = 0, length = documents.length; index < length; index++){
    iterator(documents[index]);
  }
  return void 0;
}
export function load(input, options) {
  const documents = loadDocuments(input, options);
  if (documents.length === 0) {
    return;
  }
  if (documents.length === 1) {
    return documents[0];
  }
  throw new YAMLError("expected a single document in the stream, but found more");
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbImh0dHBzOi8vZGVuby5sYW5kL3N0ZEAwLjE2OC4wL2VuY29kaW5nL195YW1sL2xvYWRlci9sb2FkZXIudHMiXSwic291cmNlc0NvbnRlbnQiOlsiLy8gUG9ydGVkIGZyb20ganMteWFtbCB2My4xMy4xOlxuLy8gaHR0cHM6Ly9naXRodWIuY29tL25vZGVjYS9qcy15YW1sL2NvbW1pdC82NjVhYWRkYTQyMzQ5ZGNhZTg2OWYxMjA0MGQ5YjEwZWYxOGQxMmRhXG4vLyBDb3B5cmlnaHQgMjAxMS0yMDE1IGJ5IFZpdGFseSBQdXpyaW4uIEFsbCByaWdodHMgcmVzZXJ2ZWQuIE1JVCBsaWNlbnNlLlxuLy8gQ29weXJpZ2h0IDIwMTgtMjAyMiB0aGUgRGVubyBhdXRob3JzLiBBbGwgcmlnaHRzIHJlc2VydmVkLiBNSVQgbGljZW5zZS5cblxuaW1wb3J0IHsgWUFNTEVycm9yIH0gZnJvbSBcIi4uL2Vycm9yLnRzXCI7XG5pbXBvcnQgeyBNYXJrIH0gZnJvbSBcIi4uL21hcmsudHNcIjtcbmltcG9ydCB0eXBlIHsgVHlwZSB9IGZyb20gXCIuLi90eXBlLnRzXCI7XG5pbXBvcnQgKiBhcyBjb21tb24gZnJvbSBcIi4uL3V0aWxzLnRzXCI7XG5pbXBvcnQgeyBMb2FkZXJTdGF0ZSwgTG9hZGVyU3RhdGVPcHRpb25zLCBSZXN1bHRUeXBlIH0gZnJvbSBcIi4vbG9hZGVyX3N0YXRlLnRzXCI7XG5cbnR5cGUgQW55ID0gY29tbW9uLkFueTtcbnR5cGUgQXJyYXlPYmplY3Q8VCA9IEFueT4gPSBjb21tb24uQXJyYXlPYmplY3Q8VD47XG5cbmNvbnN0IHsgaGFzT3duIH0gPSBPYmplY3Q7XG5cbmNvbnN0IENPTlRFWFRfRkxPV19JTiA9IDE7XG5jb25zdCBDT05URVhUX0ZMT1dfT1VUID0gMjtcbmNvbnN0IENPTlRFWFRfQkxPQ0tfSU4gPSAzO1xuY29uc3QgQ09OVEVYVF9CTE9DS19PVVQgPSA0O1xuXG5jb25zdCBDSE9NUElOR19DTElQID0gMTtcbmNvbnN0IENIT01QSU5HX1NUUklQID0gMjtcbmNvbnN0IENIT01QSU5HX0tFRVAgPSAzO1xuXG5jb25zdCBQQVRURVJOX05PTl9QUklOVEFCTEUgPVxuICAvLyBkZW5vLWxpbnQtaWdub3JlIG5vLWNvbnRyb2wtcmVnZXhcbiAgL1tcXHgwMC1cXHgwOFxceDBCXFx4MENcXHgwRS1cXHgxRlxceDdGLVxceDg0XFx4ODYtXFx4OUZcXHVGRkZFXFx1RkZGRl18W1xcdUQ4MDAtXFx1REJGRl0oPyFbXFx1REMwMC1cXHVERkZGXSl8KD86W15cXHVEODAwLVxcdURCRkZdfF4pW1xcdURDMDAtXFx1REZGRl0vO1xuY29uc3QgUEFUVEVSTl9OT05fQVNDSUlfTElORV9CUkVBS1MgPSAvW1xceDg1XFx1MjAyOFxcdTIwMjldLztcbmNvbnN0IFBBVFRFUk5fRkxPV19JTkRJQ0FUT1JTID0gL1ssXFxbXFxdXFx7XFx9XS87XG5jb25zdCBQQVRURVJOX1RBR19IQU5ETEUgPSAvXig/OiF8ISF8IVthLXpcXC1dKyEpJC9pO1xuY29uc3QgUEFUVEVSTl9UQUdfVVJJID1cbiAgL14oPzohfFteLFxcW1xcXVxce1xcfV0pKD86JVswLTlhLWZdezJ9fFswLTlhLXpcXC0jO1xcL1xcPzpAJj1cXCtcXCQsX1xcLiF+XFwqJ1xcKFxcKVxcW1xcXV0pKiQvaTtcblxuZnVuY3Rpb24gX2NsYXNzKG9iajogdW5rbm93bik6IHN0cmluZyB7XG4gIHJldHVybiBPYmplY3QucHJvdG90eXBlLnRvU3RyaW5nLmNhbGwob2JqKTtcbn1cblxuZnVuY3Rpb24gaXNFT0woYzogbnVtYmVyKTogYm9vbGVhbiB7XG4gIHJldHVybiBjID09PSAweDBhIHx8IC8qIExGICovIGMgPT09IDB4MGQgLyogQ1IgKi87XG59XG5cbmZ1bmN0aW9uIGlzV2hpdGVTcGFjZShjOiBudW1iZXIpOiBib29sZWFuIHtcbiAgcmV0dXJuIGMgPT09IDB4MDkgfHwgLyogVGFiICovIGMgPT09IDB4MjAgLyogU3BhY2UgKi87XG59XG5cbmZ1bmN0aW9uIGlzV3NPckVvbChjOiBudW1iZXIpOiBib29sZWFuIHtcbiAgcmV0dXJuIChcbiAgICBjID09PSAweDA5IC8qIFRhYiAqLyB8fFxuICAgIGMgPT09IDB4MjAgLyogU3BhY2UgKi8gfHxcbiAgICBjID09PSAweDBhIC8qIExGICovIHx8XG4gICAgYyA9PT0gMHgwZCAvKiBDUiAqL1xuICApO1xufVxuXG5mdW5jdGlvbiBpc0Zsb3dJbmRpY2F0b3IoYzogbnVtYmVyKTogYm9vbGVhbiB7XG4gIHJldHVybiAoXG4gICAgYyA9PT0gMHgyYyAvKiAsICovIHx8XG4gICAgYyA9PT0gMHg1YiAvKiBbICovIHx8XG4gICAgYyA9PT0gMHg1ZCAvKiBdICovIHx8XG4gICAgYyA9PT0gMHg3YiAvKiB7ICovIHx8XG4gICAgYyA9PT0gMHg3ZCAvKiB9ICovXG4gICk7XG59XG5cbmZ1bmN0aW9uIGZyb21IZXhDb2RlKGM6IG51bWJlcik6IG51bWJlciB7XG4gIGlmICgweDMwIDw9IC8qIDAgKi8gYyAmJiBjIDw9IDB4MzkgLyogOSAqLykge1xuICAgIHJldHVybiBjIC0gMHgzMDtcbiAgfVxuXG4gIGNvbnN0IGxjID0gYyB8IDB4MjA7XG5cbiAgaWYgKDB4NjEgPD0gLyogYSAqLyBsYyAmJiBsYyA8PSAweDY2IC8qIGYgKi8pIHtcbiAgICByZXR1cm4gbGMgLSAweDYxICsgMTA7XG4gIH1cblxuICByZXR1cm4gLTE7XG59XG5cbmZ1bmN0aW9uIGVzY2FwZWRIZXhMZW4oYzogbnVtYmVyKTogbnVtYmVyIHtcbiAgaWYgKGMgPT09IDB4NzggLyogeCAqLykge1xuICAgIHJldHVybiAyO1xuICB9XG4gIGlmIChjID09PSAweDc1IC8qIHUgKi8pIHtcbiAgICByZXR1cm4gNDtcbiAgfVxuICBpZiAoYyA9PT0gMHg1NSAvKiBVICovKSB7XG4gICAgcmV0dXJuIDg7XG4gIH1cbiAgcmV0dXJuIDA7XG59XG5cbmZ1bmN0aW9uIGZyb21EZWNpbWFsQ29kZShjOiBudW1iZXIpOiBudW1iZXIge1xuICBpZiAoMHgzMCA8PSAvKiAwICovIGMgJiYgYyA8PSAweDM5IC8qIDkgKi8pIHtcbiAgICByZXR1cm4gYyAtIDB4MzA7XG4gIH1cblxuICByZXR1cm4gLTE7XG59XG5cbmZ1bmN0aW9uIHNpbXBsZUVzY2FwZVNlcXVlbmNlKGM6IG51bWJlcik6IHN0cmluZyB7XG4gIHJldHVybiBjID09PSAweDMwIC8qIDAgKi9cbiAgICA/IFwiXFx4MDBcIlxuICAgIDogYyA9PT0gMHg2MSAvKiBhICovXG4gICAgPyBcIlxceDA3XCJcbiAgICA6IGMgPT09IDB4NjIgLyogYiAqL1xuICAgID8gXCJcXHgwOFwiXG4gICAgOiBjID09PSAweDc0IC8qIHQgKi9cbiAgICA/IFwiXFx4MDlcIlxuICAgIDogYyA9PT0gMHgwOSAvKiBUYWIgKi9cbiAgICA/IFwiXFx4MDlcIlxuICAgIDogYyA9PT0gMHg2ZSAvKiBuICovXG4gICAgPyBcIlxceDBBXCJcbiAgICA6IGMgPT09IDB4NzYgLyogdiAqL1xuICAgID8gXCJcXHgwQlwiXG4gICAgOiBjID09PSAweDY2IC8qIGYgKi9cbiAgICA/IFwiXFx4MENcIlxuICAgIDogYyA9PT0gMHg3MiAvKiByICovXG4gICAgPyBcIlxceDBEXCJcbiAgICA6IGMgPT09IDB4NjUgLyogZSAqL1xuICAgID8gXCJcXHgxQlwiXG4gICAgOiBjID09PSAweDIwIC8qIFNwYWNlICovXG4gICAgPyBcIiBcIlxuICAgIDogYyA9PT0gMHgyMiAvKiBcIiAqL1xuICAgID8gXCJcXHgyMlwiXG4gICAgOiBjID09PSAweDJmIC8qIC8gKi9cbiAgICA/IFwiL1wiXG4gICAgOiBjID09PSAweDVjIC8qIFxcICovXG4gICAgPyBcIlxceDVDXCJcbiAgICA6IGMgPT09IDB4NGUgLyogTiAqL1xuICAgID8gXCJcXHg4NVwiXG4gICAgOiBjID09PSAweDVmIC8qIF8gKi9cbiAgICA/IFwiXFx4QTBcIlxuICAgIDogYyA9PT0gMHg0YyAvKiBMICovXG4gICAgPyBcIlxcdTIwMjhcIlxuICAgIDogYyA9PT0gMHg1MCAvKiBQICovXG4gICAgPyBcIlxcdTIwMjlcIlxuICAgIDogXCJcIjtcbn1cblxuZnVuY3Rpb24gY2hhckZyb21Db2RlcG9pbnQoYzogbnVtYmVyKTogc3RyaW5nIHtcbiAgaWYgKGMgPD0gMHhmZmZmKSB7XG4gICAgcmV0dXJuIFN0cmluZy5mcm9tQ2hhckNvZGUoYyk7XG4gIH1cbiAgLy8gRW5jb2RlIFVURi0xNiBzdXJyb2dhdGUgcGFpclxuICAvLyBodHRwczovL2VuLndpa2lwZWRpYS5vcmcvd2lraS9VVEYtMTYjQ29kZV9wb2ludHNfVS4yQjAxMDAwMF90b19VLjJCMTBGRkZGXG4gIHJldHVybiBTdHJpbmcuZnJvbUNoYXJDb2RlKFxuICAgICgoYyAtIDB4MDEwMDAwKSA+PiAxMCkgKyAweGQ4MDAsXG4gICAgKChjIC0gMHgwMTAwMDApICYgMHgwM2ZmKSArIDB4ZGMwMCxcbiAgKTtcbn1cblxuY29uc3Qgc2ltcGxlRXNjYXBlQ2hlY2sgPSBBcnJheS5mcm9tPG51bWJlcj4oeyBsZW5ndGg6IDI1NiB9KTsgLy8gaW50ZWdlciwgZm9yIGZhc3QgYWNjZXNzXG5jb25zdCBzaW1wbGVFc2NhcGVNYXAgPSBBcnJheS5mcm9tPHN0cmluZz4oeyBsZW5ndGg6IDI1NiB9KTtcbmZvciAobGV0IGkgPSAwOyBpIDwgMjU2OyBpKyspIHtcbiAgc2ltcGxlRXNjYXBlQ2hlY2tbaV0gPSBzaW1wbGVFc2NhcGVTZXF1ZW5jZShpKSA/IDEgOiAwO1xuICBzaW1wbGVFc2NhcGVNYXBbaV0gPSBzaW1wbGVFc2NhcGVTZXF1ZW5jZShpKTtcbn1cblxuZnVuY3Rpb24gZ2VuZXJhdGVFcnJvcihzdGF0ZTogTG9hZGVyU3RhdGUsIG1lc3NhZ2U6IHN0cmluZyk6IFlBTUxFcnJvciB7XG4gIHJldHVybiBuZXcgWUFNTEVycm9yKFxuICAgIG1lc3NhZ2UsXG4gICAgbmV3IE1hcmsoXG4gICAgICBzdGF0ZS5maWxlbmFtZSBhcyBzdHJpbmcsXG4gICAgICBzdGF0ZS5pbnB1dCxcbiAgICAgIHN0YXRlLnBvc2l0aW9uLFxuICAgICAgc3RhdGUubGluZSxcbiAgICAgIHN0YXRlLnBvc2l0aW9uIC0gc3RhdGUubGluZVN0YXJ0LFxuICAgICksXG4gICk7XG59XG5cbmZ1bmN0aW9uIHRocm93RXJyb3Ioc3RhdGU6IExvYWRlclN0YXRlLCBtZXNzYWdlOiBzdHJpbmcpOiBuZXZlciB7XG4gIHRocm93IGdlbmVyYXRlRXJyb3Ioc3RhdGUsIG1lc3NhZ2UpO1xufVxuXG5mdW5jdGlvbiB0aHJvd1dhcm5pbmcoc3RhdGU6IExvYWRlclN0YXRlLCBtZXNzYWdlOiBzdHJpbmcpIHtcbiAgaWYgKHN0YXRlLm9uV2FybmluZykge1xuICAgIHN0YXRlLm9uV2FybmluZy5jYWxsKG51bGwsIGdlbmVyYXRlRXJyb3Ioc3RhdGUsIG1lc3NhZ2UpKTtcbiAgfVxufVxuXG5pbnRlcmZhY2UgRGlyZWN0aXZlSGFuZGxlcnMge1xuICBbZGlyZWN0aXZlOiBzdHJpbmddOiAoXG4gICAgc3RhdGU6IExvYWRlclN0YXRlLFxuICAgIG5hbWU6IHN0cmluZyxcbiAgICAuLi5hcmdzOiBzdHJpbmdbXVxuICApID0+IHZvaWQ7XG59XG5cbmNvbnN0IGRpcmVjdGl2ZUhhbmRsZXJzOiBEaXJlY3RpdmVIYW5kbGVycyA9IHtcbiAgWUFNTChzdGF0ZSwgX25hbWUsIC4uLmFyZ3M6IHN0cmluZ1tdKSB7XG4gICAgaWYgKHN0YXRlLnZlcnNpb24gIT09IG51bGwpIHtcbiAgICAgIHJldHVybiB0aHJvd0Vycm9yKHN0YXRlLCBcImR1cGxpY2F0aW9uIG9mICVZQU1MIGRpcmVjdGl2ZVwiKTtcbiAgICB9XG5cbiAgICBpZiAoYXJncy5sZW5ndGggIT09IDEpIHtcbiAgICAgIHJldHVybiB0aHJvd0Vycm9yKHN0YXRlLCBcIllBTUwgZGlyZWN0aXZlIGFjY2VwdHMgZXhhY3RseSBvbmUgYXJndW1lbnRcIik7XG4gICAgfVxuXG4gICAgY29uc3QgbWF0Y2ggPSAvXihbMC05XSspXFwuKFswLTldKykkLy5leGVjKGFyZ3NbMF0pO1xuICAgIGlmIChtYXRjaCA9PT0gbnVsbCkge1xuICAgICAgcmV0dXJuIHRocm93RXJyb3Ioc3RhdGUsIFwiaWxsLWZvcm1lZCBhcmd1bWVudCBvZiB0aGUgWUFNTCBkaXJlY3RpdmVcIik7XG4gICAgfVxuXG4gICAgY29uc3QgbWFqb3IgPSBwYXJzZUludChtYXRjaFsxXSwgMTApO1xuICAgIGNvbnN0IG1pbm9yID0gcGFyc2VJbnQobWF0Y2hbMl0sIDEwKTtcbiAgICBpZiAobWFqb3IgIT09IDEpIHtcbiAgICAgIHJldHVybiB0aHJvd0Vycm9yKHN0YXRlLCBcInVuYWNjZXB0YWJsZSBZQU1MIHZlcnNpb24gb2YgdGhlIGRvY3VtZW50XCIpO1xuICAgIH1cblxuICAgIHN0YXRlLnZlcnNpb24gPSBhcmdzWzBdO1xuICAgIHN0YXRlLmNoZWNrTGluZUJyZWFrcyA9IG1pbm9yIDwgMjtcbiAgICBpZiAobWlub3IgIT09IDEgJiYgbWlub3IgIT09IDIpIHtcbiAgICAgIHJldHVybiB0aHJvd1dhcm5pbmcoc3RhdGUsIFwidW5zdXBwb3J0ZWQgWUFNTCB2ZXJzaW9uIG9mIHRoZSBkb2N1bWVudFwiKTtcbiAgICB9XG4gIH0sXG5cbiAgVEFHKHN0YXRlLCBfbmFtZSwgLi4uYXJnczogc3RyaW5nW10pIHtcbiAgICBpZiAoYXJncy5sZW5ndGggIT09IDIpIHtcbiAgICAgIHJldHVybiB0aHJvd0Vycm9yKHN0YXRlLCBcIlRBRyBkaXJlY3RpdmUgYWNjZXB0cyBleGFjdGx5IHR3byBhcmd1bWVudHNcIik7XG4gICAgfVxuXG4gICAgY29uc3QgaGFuZGxlID0gYXJnc1swXTtcbiAgICBjb25zdCBwcmVmaXggPSBhcmdzWzFdO1xuXG4gICAgaWYgKCFQQVRURVJOX1RBR19IQU5ETEUudGVzdChoYW5kbGUpKSB7XG4gICAgICByZXR1cm4gdGhyb3dFcnJvcihcbiAgICAgICAgc3RhdGUsXG4gICAgICAgIFwiaWxsLWZvcm1lZCB0YWcgaGFuZGxlIChmaXJzdCBhcmd1bWVudCkgb2YgdGhlIFRBRyBkaXJlY3RpdmVcIixcbiAgICAgICk7XG4gICAgfVxuXG4gICAgaWYgKHN0YXRlLnRhZ01hcCAmJiBoYXNPd24oc3RhdGUudGFnTWFwLCBoYW5kbGUpKSB7XG4gICAgICByZXR1cm4gdGhyb3dFcnJvcihcbiAgICAgICAgc3RhdGUsXG4gICAgICAgIGB0aGVyZSBpcyBhIHByZXZpb3VzbHkgZGVjbGFyZWQgc3VmZml4IGZvciBcIiR7aGFuZGxlfVwiIHRhZyBoYW5kbGVgLFxuICAgICAgKTtcbiAgICB9XG5cbiAgICBpZiAoIVBBVFRFUk5fVEFHX1VSSS50ZXN0KHByZWZpeCkpIHtcbiAgICAgIHJldHVybiB0aHJvd0Vycm9yKFxuICAgICAgICBzdGF0ZSxcbiAgICAgICAgXCJpbGwtZm9ybWVkIHRhZyBwcmVmaXggKHNlY29uZCBhcmd1bWVudCkgb2YgdGhlIFRBRyBkaXJlY3RpdmVcIixcbiAgICAgICk7XG4gICAgfVxuXG4gICAgaWYgKHR5cGVvZiBzdGF0ZS50YWdNYXAgPT09IFwidW5kZWZpbmVkXCIpIHtcbiAgICAgIHN0YXRlLnRhZ01hcCA9IHt9O1xuICAgIH1cbiAgICBzdGF0ZS50YWdNYXBbaGFuZGxlXSA9IHByZWZpeDtcbiAgfSxcbn07XG5cbmZ1bmN0aW9uIGNhcHR1cmVTZWdtZW50KFxuICBzdGF0ZTogTG9hZGVyU3RhdGUsXG4gIHN0YXJ0OiBudW1iZXIsXG4gIGVuZDogbnVtYmVyLFxuICBjaGVja0pzb246IGJvb2xlYW4sXG4pIHtcbiAgbGV0IHJlc3VsdDogc3RyaW5nO1xuICBpZiAoc3RhcnQgPCBlbmQpIHtcbiAgICByZXN1bHQgPSBzdGF0ZS5pbnB1dC5zbGljZShzdGFydCwgZW5kKTtcblxuICAgIGlmIChjaGVja0pzb24pIHtcbiAgICAgIGZvciAoXG4gICAgICAgIGxldCBwb3NpdGlvbiA9IDAsIGxlbmd0aCA9IHJlc3VsdC5sZW5ndGg7XG4gICAgICAgIHBvc2l0aW9uIDwgbGVuZ3RoO1xuICAgICAgICBwb3NpdGlvbisrXG4gICAgICApIHtcbiAgICAgICAgY29uc3QgY2hhcmFjdGVyID0gcmVzdWx0LmNoYXJDb2RlQXQocG9zaXRpb24pO1xuICAgICAgICBpZiAoXG4gICAgICAgICAgIShjaGFyYWN0ZXIgPT09IDB4MDkgfHwgKDB4MjAgPD0gY2hhcmFjdGVyICYmIGNoYXJhY3RlciA8PSAweDEwZmZmZikpXG4gICAgICAgICkge1xuICAgICAgICAgIHJldHVybiB0aHJvd0Vycm9yKHN0YXRlLCBcImV4cGVjdGVkIHZhbGlkIEpTT04gY2hhcmFjdGVyXCIpO1xuICAgICAgICB9XG4gICAgICB9XG4gICAgfSBlbHNlIGlmIChQQVRURVJOX05PTl9QUklOVEFCTEUudGVzdChyZXN1bHQpKSB7XG4gICAgICByZXR1cm4gdGhyb3dFcnJvcihzdGF0ZSwgXCJ0aGUgc3RyZWFtIGNvbnRhaW5zIG5vbi1wcmludGFibGUgY2hhcmFjdGVyc1wiKTtcbiAgICB9XG5cbiAgICBzdGF0ZS5yZXN1bHQgKz0gcmVzdWx0O1xuICB9XG59XG5cbmZ1bmN0aW9uIG1lcmdlTWFwcGluZ3MoXG4gIHN0YXRlOiBMb2FkZXJTdGF0ZSxcbiAgZGVzdGluYXRpb246IEFycmF5T2JqZWN0LFxuICBzb3VyY2U6IEFycmF5T2JqZWN0LFxuICBvdmVycmlkYWJsZUtleXM6IEFycmF5T2JqZWN0PGJvb2xlYW4+LFxuKSB7XG4gIGlmICghY29tbW9uLmlzT2JqZWN0KHNvdXJjZSkpIHtcbiAgICByZXR1cm4gdGhyb3dFcnJvcihcbiAgICAgIHN0YXRlLFxuICAgICAgXCJjYW5ub3QgbWVyZ2UgbWFwcGluZ3M7IHRoZSBwcm92aWRlZCBzb3VyY2Ugb2JqZWN0IGlzIHVuYWNjZXB0YWJsZVwiLFxuICAgICk7XG4gIH1cblxuICBjb25zdCBrZXlzID0gT2JqZWN0LmtleXMoc291cmNlKTtcbiAgZm9yIChsZXQgaSA9IDAsIGxlbiA9IGtleXMubGVuZ3RoOyBpIDwgbGVuOyBpKyspIHtcbiAgICBjb25zdCBrZXkgPSBrZXlzW2ldO1xuICAgIGlmICghaGFzT3duKGRlc3RpbmF0aW9uLCBrZXkpKSB7XG4gICAgICBkZXN0aW5hdGlvbltrZXldID0gKHNvdXJjZSBhcyBBcnJheU9iamVjdClba2V5XTtcbiAgICAgIG92ZXJyaWRhYmxlS2V5c1trZXldID0gdHJ1ZTtcbiAgICB9XG4gIH1cbn1cblxuZnVuY3Rpb24gc3RvcmVNYXBwaW5nUGFpcihcbiAgc3RhdGU6IExvYWRlclN0YXRlLFxuICByZXN1bHQ6IEFycmF5T2JqZWN0IHwgbnVsbCxcbiAgb3ZlcnJpZGFibGVLZXlzOiBBcnJheU9iamVjdDxib29sZWFuPixcbiAga2V5VGFnOiBzdHJpbmcgfCBudWxsLFxuICBrZXlOb2RlOiBBbnksXG4gIHZhbHVlTm9kZTogdW5rbm93bixcbiAgc3RhcnRMaW5lPzogbnVtYmVyLFxuICBzdGFydFBvcz86IG51bWJlcixcbik6IEFycmF5T2JqZWN0IHtcbiAgLy8gVGhlIG91dHB1dCBpcyBhIHBsYWluIG9iamVjdCBoZXJlLCBzbyBrZXlzIGNhbiBvbmx5IGJlIHN0cmluZ3MuXG4gIC8vIFdlIG5lZWQgdG8gY29udmVydCBrZXlOb2RlIHRvIGEgc3RyaW5nLCBidXQgZG9pbmcgc28gY2FuIGhhbmcgdGhlIHByb2Nlc3NcbiAgLy8gKGRlZXBseSBuZXN0ZWQgYXJyYXlzIHRoYXQgZXhwbG9kZSBleHBvbmVudGlhbGx5IHVzaW5nIGFsaWFzZXMpLlxuICBpZiAoQXJyYXkuaXNBcnJheShrZXlOb2RlKSkge1xuICAgIGtleU5vZGUgPSBBcnJheS5wcm90b3R5cGUuc2xpY2UuY2FsbChrZXlOb2RlKTtcblxuICAgIGZvciAobGV0IGluZGV4ID0gMCwgcXVhbnRpdHkgPSBrZXlOb2RlLmxlbmd0aDsgaW5kZXggPCBxdWFudGl0eTsgaW5kZXgrKykge1xuICAgICAgaWYgKEFycmF5LmlzQXJyYXkoa2V5Tm9kZVtpbmRleF0pKSB7XG4gICAgICAgIHJldHVybiB0aHJvd0Vycm9yKHN0YXRlLCBcIm5lc3RlZCBhcnJheXMgYXJlIG5vdCBzdXBwb3J0ZWQgaW5zaWRlIGtleXNcIik7XG4gICAgICB9XG5cbiAgICAgIGlmIChcbiAgICAgICAgdHlwZW9mIGtleU5vZGUgPT09IFwib2JqZWN0XCIgJiZcbiAgICAgICAgX2NsYXNzKGtleU5vZGVbaW5kZXhdKSA9PT0gXCJbb2JqZWN0IE9iamVjdF1cIlxuICAgICAgKSB7XG4gICAgICAgIGtleU5vZGVbaW5kZXhdID0gXCJbb2JqZWN0IE9iamVjdF1cIjtcbiAgICAgIH1cbiAgICB9XG4gIH1cblxuICAvLyBBdm9pZCBjb2RlIGV4ZWN1dGlvbiBpbiBsb2FkKCkgdmlhIHRvU3RyaW5nIHByb3BlcnR5XG4gIC8vIChzdGlsbCB1c2UgaXRzIG93biB0b1N0cmluZyBmb3IgYXJyYXlzLCB0aW1lc3RhbXBzLFxuICAvLyBhbmQgd2hhdGV2ZXIgdXNlciBzY2hlbWEgZXh0ZW5zaW9ucyBoYXBwZW4gdG8gaGF2ZSBAQHRvU3RyaW5nVGFnKVxuICBpZiAodHlwZW9mIGtleU5vZGUgPT09IFwib2JqZWN0XCIgJiYgX2NsYXNzKGtleU5vZGUpID09PSBcIltvYmplY3QgT2JqZWN0XVwiKSB7XG4gICAga2V5Tm9kZSA9IFwiW29iamVjdCBPYmplY3RdXCI7XG4gIH1cblxuICBrZXlOb2RlID0gU3RyaW5nKGtleU5vZGUpO1xuXG4gIGlmIChyZXN1bHQgPT09IG51bGwpIHtcbiAgICByZXN1bHQgPSB7fTtcbiAgfVxuXG4gIGlmIChrZXlUYWcgPT09IFwidGFnOnlhbWwub3JnLDIwMDI6bWVyZ2VcIikge1xuICAgIGlmIChBcnJheS5pc0FycmF5KHZhbHVlTm9kZSkpIHtcbiAgICAgIGZvciAoXG4gICAgICAgIGxldCBpbmRleCA9IDAsIHF1YW50aXR5ID0gdmFsdWVOb2RlLmxlbmd0aDtcbiAgICAgICAgaW5kZXggPCBxdWFudGl0eTtcbiAgICAgICAgaW5kZXgrK1xuICAgICAgKSB7XG4gICAgICAgIG1lcmdlTWFwcGluZ3Moc3RhdGUsIHJlc3VsdCwgdmFsdWVOb2RlW2luZGV4XSwgb3ZlcnJpZGFibGVLZXlzKTtcbiAgICAgIH1cbiAgICB9IGVsc2Uge1xuICAgICAgbWVyZ2VNYXBwaW5ncyhzdGF0ZSwgcmVzdWx0LCB2YWx1ZU5vZGUgYXMgQXJyYXlPYmplY3QsIG92ZXJyaWRhYmxlS2V5cyk7XG4gICAgfVxuICB9IGVsc2Uge1xuICAgIGlmIChcbiAgICAgICFzdGF0ZS5qc29uICYmXG4gICAgICAhaGFzT3duKG92ZXJyaWRhYmxlS2V5cywga2V5Tm9kZSkgJiZcbiAgICAgIGhhc093bihyZXN1bHQsIGtleU5vZGUpXG4gICAgKSB7XG4gICAgICBzdGF0ZS5saW5lID0gc3RhcnRMaW5lIHx8IHN0YXRlLmxpbmU7XG4gICAgICBzdGF0ZS5wb3NpdGlvbiA9IHN0YXJ0UG9zIHx8IHN0YXRlLnBvc2l0aW9uO1xuICAgICAgcmV0dXJuIHRocm93RXJyb3Ioc3RhdGUsIFwiZHVwbGljYXRlZCBtYXBwaW5nIGtleVwiKTtcbiAgICB9XG4gICAgcmVzdWx0W2tleU5vZGVdID0gdmFsdWVOb2RlO1xuICAgIGRlbGV0ZSBvdmVycmlkYWJsZUtleXNba2V5Tm9kZV07XG4gIH1cblxuICByZXR1cm4gcmVzdWx0O1xufVxuXG5mdW5jdGlvbiByZWFkTGluZUJyZWFrKHN0YXRlOiBMb2FkZXJTdGF0ZSkge1xuICBjb25zdCBjaCA9IHN0YXRlLmlucHV0LmNoYXJDb2RlQXQoc3RhdGUucG9zaXRpb24pO1xuXG4gIGlmIChjaCA9PT0gMHgwYSAvKiBMRiAqLykge1xuICAgIHN0YXRlLnBvc2l0aW9uKys7XG4gIH0gZWxzZSBpZiAoY2ggPT09IDB4MGQgLyogQ1IgKi8pIHtcbiAgICBzdGF0ZS5wb3NpdGlvbisrO1xuICAgIGlmIChzdGF0ZS5pbnB1dC5jaGFyQ29kZUF0KHN0YXRlLnBvc2l0aW9uKSA9PT0gMHgwYSAvKiBMRiAqLykge1xuICAgICAgc3RhdGUucG9zaXRpb24rKztcbiAgICB9XG4gIH0gZWxzZSB7XG4gICAgcmV0dXJuIHRocm93RXJyb3Ioc3RhdGUsIFwiYSBsaW5lIGJyZWFrIGlzIGV4cGVjdGVkXCIpO1xuICB9XG5cbiAgc3RhdGUubGluZSArPSAxO1xuICBzdGF0ZS5saW5lU3RhcnQgPSBzdGF0ZS5wb3NpdGlvbjtcbn1cblxuZnVuY3Rpb24gc2tpcFNlcGFyYXRpb25TcGFjZShcbiAgc3RhdGU6IExvYWRlclN0YXRlLFxuICBhbGxvd0NvbW1lbnRzOiBib29sZWFuLFxuICBjaGVja0luZGVudDogbnVtYmVyLFxuKTogbnVtYmVyIHtcbiAgbGV0IGxpbmVCcmVha3MgPSAwLFxuICAgIGNoID0gc3RhdGUuaW5wdXQuY2hhckNvZGVBdChzdGF0ZS5wb3NpdGlvbik7XG5cbiAgd2hpbGUgKGNoICE9PSAwKSB7XG4gICAgd2hpbGUgKGlzV2hpdGVTcGFjZShjaCkpIHtcbiAgICAgIGNoID0gc3RhdGUuaW5wdXQuY2hhckNvZGVBdCgrK3N0YXRlLnBvc2l0aW9uKTtcbiAgICB9XG5cbiAgICBpZiAoYWxsb3dDb21tZW50cyAmJiBjaCA9PT0gMHgyMyAvKiAjICovKSB7XG4gICAgICBkbyB7XG4gICAgICAgIGNoID0gc3RhdGUuaW5wdXQuY2hhckNvZGVBdCgrK3N0YXRlLnBvc2l0aW9uKTtcbiAgICAgIH0gd2hpbGUgKGNoICE9PSAweDBhICYmIC8qIExGICovIGNoICE9PSAweDBkICYmIC8qIENSICovIGNoICE9PSAwKTtcbiAgICB9XG5cbiAgICBpZiAoaXNFT0woY2gpKSB7XG4gICAgICByZWFkTGluZUJyZWFrKHN0YXRlKTtcblxuICAgICAgY2ggPSBzdGF0ZS5pbnB1dC5jaGFyQ29kZUF0KHN0YXRlLnBvc2l0aW9uKTtcbiAgICAgIGxpbmVCcmVha3MrKztcbiAgICAgIHN0YXRlLmxpbmVJbmRlbnQgPSAwO1xuXG4gICAgICB3aGlsZSAoY2ggPT09IDB4MjAgLyogU3BhY2UgKi8pIHtcbiAgICAgICAgc3RhdGUubGluZUluZGVudCsrO1xuICAgICAgICBjaCA9IHN0YXRlLmlucHV0LmNoYXJDb2RlQXQoKytzdGF0ZS5wb3NpdGlvbik7XG4gICAgICB9XG4gICAgfSBlbHNlIHtcbiAgICAgIGJyZWFrO1xuICAgIH1cbiAgfVxuXG4gIGlmIChcbiAgICBjaGVja0luZGVudCAhPT0gLTEgJiZcbiAgICBsaW5lQnJlYWtzICE9PSAwICYmXG4gICAgc3RhdGUubGluZUluZGVudCA8IGNoZWNrSW5kZW50XG4gICkge1xuICAgIHRocm93V2FybmluZyhzdGF0ZSwgXCJkZWZpY2llbnQgaW5kZW50YXRpb25cIik7XG4gIH1cblxuICByZXR1cm4gbGluZUJyZWFrcztcbn1cblxuZnVuY3Rpb24gdGVzdERvY3VtZW50U2VwYXJhdG9yKHN0YXRlOiBMb2FkZXJTdGF0ZSk6IGJvb2xlYW4ge1xuICBsZXQgX3Bvc2l0aW9uID0gc3RhdGUucG9zaXRpb247XG4gIGxldCBjaCA9IHN0YXRlLmlucHV0LmNoYXJDb2RlQXQoX3Bvc2l0aW9uKTtcblxuICAvLyBDb25kaXRpb24gc3RhdGUucG9zaXRpb24gPT09IHN0YXRlLmxpbmVTdGFydCBpcyB0ZXN0ZWRcbiAgLy8gaW4gcGFyZW50IG9uIGVhY2ggY2FsbCwgZm9yIGVmZmljaWVuY3kuIE5vIG5lZWRzIHRvIHRlc3QgaGVyZSBhZ2Fpbi5cbiAgaWYgKFxuICAgIChjaCA9PT0gMHgyZCB8fCAvKiAtICovIGNoID09PSAweDJlKSAvKiAuICovICYmXG4gICAgY2ggPT09IHN0YXRlLmlucHV0LmNoYXJDb2RlQXQoX3Bvc2l0aW9uICsgMSkgJiZcbiAgICBjaCA9PT0gc3RhdGUuaW5wdXQuY2hhckNvZGVBdChfcG9zaXRpb24gKyAyKVxuICApIHtcbiAgICBfcG9zaXRpb24gKz0gMztcblxuICAgIGNoID0gc3RhdGUuaW5wdXQuY2hhckNvZGVBdChfcG9zaXRpb24pO1xuXG4gICAgaWYgKGNoID09PSAwIHx8IGlzV3NPckVvbChjaCkpIHtcbiAgICAgIHJldHVybiB0cnVlO1xuICAgIH1cbiAgfVxuXG4gIHJldHVybiBmYWxzZTtcbn1cblxuZnVuY3Rpb24gd3JpdGVGb2xkZWRMaW5lcyhzdGF0ZTogTG9hZGVyU3RhdGUsIGNvdW50OiBudW1iZXIpIHtcbiAgaWYgKGNvdW50ID09PSAxKSB7XG4gICAgc3RhdGUucmVzdWx0ICs9IFwiIFwiO1xuICB9IGVsc2UgaWYgKGNvdW50ID4gMSkge1xuICAgIHN0YXRlLnJlc3VsdCArPSBjb21tb24ucmVwZWF0KFwiXFxuXCIsIGNvdW50IC0gMSk7XG4gIH1cbn1cblxuZnVuY3Rpb24gcmVhZFBsYWluU2NhbGFyKFxuICBzdGF0ZTogTG9hZGVyU3RhdGUsXG4gIG5vZGVJbmRlbnQ6IG51bWJlcixcbiAgd2l0aGluRmxvd0NvbGxlY3Rpb246IGJvb2xlYW4sXG4pOiBib29sZWFuIHtcbiAgY29uc3Qga2luZCA9IHN0YXRlLmtpbmQ7XG4gIGNvbnN0IHJlc3VsdCA9IHN0YXRlLnJlc3VsdDtcbiAgbGV0IGNoID0gc3RhdGUuaW5wdXQuY2hhckNvZGVBdChzdGF0ZS5wb3NpdGlvbik7XG5cbiAgaWYgKFxuICAgIGlzV3NPckVvbChjaCkgfHxcbiAgICBpc0Zsb3dJbmRpY2F0b3IoY2gpIHx8XG4gICAgY2ggPT09IDB4MjMgLyogIyAqLyB8fFxuICAgIGNoID09PSAweDI2IC8qICYgKi8gfHxcbiAgICBjaCA9PT0gMHgyYSAvKiAqICovIHx8XG4gICAgY2ggPT09IDB4MjEgLyogISAqLyB8fFxuICAgIGNoID09PSAweDdjIC8qIHwgKi8gfHxcbiAgICBjaCA9PT0gMHgzZSAvKiA+ICovIHx8XG4gICAgY2ggPT09IDB4MjcgLyogJyAqLyB8fFxuICAgIGNoID09PSAweDIyIC8qIFwiICovIHx8XG4gICAgY2ggPT09IDB4MjUgLyogJSAqLyB8fFxuICAgIGNoID09PSAweDQwIC8qIEAgKi8gfHxcbiAgICBjaCA9PT0gMHg2MCAvKiBgICovXG4gICkge1xuICAgIHJldHVybiBmYWxzZTtcbiAgfVxuXG4gIGxldCBmb2xsb3dpbmc6IG51bWJlcjtcbiAgaWYgKGNoID09PSAweDNmIHx8IC8qID8gKi8gY2ggPT09IDB4MmQgLyogLSAqLykge1xuICAgIGZvbGxvd2luZyA9IHN0YXRlLmlucHV0LmNoYXJDb2RlQXQoc3RhdGUucG9zaXRpb24gKyAxKTtcblxuICAgIGlmIChcbiAgICAgIGlzV3NPckVvbChmb2xsb3dpbmcpIHx8XG4gICAgICAod2l0aGluRmxvd0NvbGxlY3Rpb24gJiYgaXNGbG93SW5kaWNhdG9yKGZvbGxvd2luZykpXG4gICAgKSB7XG4gICAgICByZXR1cm4gZmFsc2U7XG4gICAgfVxuICB9XG5cbiAgc3RhdGUua2luZCA9IFwic2NhbGFyXCI7XG4gIHN0YXRlLnJlc3VsdCA9IFwiXCI7XG4gIGxldCBjYXB0dXJlRW5kOiBudW1iZXIsXG4gICAgY2FwdHVyZVN0YXJ0ID0gKGNhcHR1cmVFbmQgPSBzdGF0ZS5wb3NpdGlvbik7XG4gIGxldCBoYXNQZW5kaW5nQ29udGVudCA9IGZhbHNlO1xuICBsZXQgbGluZSA9IDA7XG4gIHdoaWxlIChjaCAhPT0gMCkge1xuICAgIGlmIChjaCA9PT0gMHgzYSAvKiA6ICovKSB7XG4gICAgICBmb2xsb3dpbmcgPSBzdGF0ZS5pbnB1dC5jaGFyQ29kZUF0KHN0YXRlLnBvc2l0aW9uICsgMSk7XG5cbiAgICAgIGlmIChcbiAgICAgICAgaXNXc09yRW9sKGZvbGxvd2luZykgfHxcbiAgICAgICAgKHdpdGhpbkZsb3dDb2xsZWN0aW9uICYmIGlzRmxvd0luZGljYXRvcihmb2xsb3dpbmcpKVxuICAgICAgKSB7XG4gICAgICAgIGJyZWFrO1xuICAgICAgfVxuICAgIH0gZWxzZSBpZiAoY2ggPT09IDB4MjMgLyogIyAqLykge1xuICAgICAgY29uc3QgcHJlY2VkaW5nID0gc3RhdGUuaW5wdXQuY2hhckNvZGVBdChzdGF0ZS5wb3NpdGlvbiAtIDEpO1xuXG4gICAgICBpZiAoaXNXc09yRW9sKHByZWNlZGluZykpIHtcbiAgICAgICAgYnJlYWs7XG4gICAgICB9XG4gICAgfSBlbHNlIGlmIChcbiAgICAgIChzdGF0ZS5wb3NpdGlvbiA9PT0gc3RhdGUubGluZVN0YXJ0ICYmIHRlc3REb2N1bWVudFNlcGFyYXRvcihzdGF0ZSkpIHx8XG4gICAgICAod2l0aGluRmxvd0NvbGxlY3Rpb24gJiYgaXNGbG93SW5kaWNhdG9yKGNoKSlcbiAgICApIHtcbiAgICAgIGJyZWFrO1xuICAgIH0gZWxzZSBpZiAoaXNFT0woY2gpKSB7XG4gICAgICBsaW5lID0gc3RhdGUubGluZTtcbiAgICAgIGNvbnN0IGxpbmVTdGFydCA9IHN0YXRlLmxpbmVTdGFydDtcbiAgICAgIGNvbnN0IGxpbmVJbmRlbnQgPSBzdGF0ZS5saW5lSW5kZW50O1xuICAgICAgc2tpcFNlcGFyYXRpb25TcGFjZShzdGF0ZSwgZmFsc2UsIC0xKTtcblxuICAgICAgaWYgKHN0YXRlLmxpbmVJbmRlbnQgPj0gbm9kZUluZGVudCkge1xuICAgICAgICBoYXNQZW5kaW5nQ29udGVudCA9IHRydWU7XG4gICAgICAgIGNoID0gc3RhdGUuaW5wdXQuY2hhckNvZGVBdChzdGF0ZS5wb3NpdGlvbik7XG4gICAgICAgIGNvbnRpbnVlO1xuICAgICAgfSBlbHNlIHtcbiAgICAgICAgc3RhdGUucG9zaXRpb24gPSBjYXB0dXJlRW5kO1xuICAgICAgICBzdGF0ZS5saW5lID0gbGluZTtcbiAgICAgICAgc3RhdGUubGluZVN0YXJ0ID0gbGluZVN0YXJ0O1xuICAgICAgICBzdGF0ZS5saW5lSW5kZW50ID0gbGluZUluZGVudDtcbiAgICAgICAgYnJlYWs7XG4gICAgICB9XG4gICAgfVxuXG4gICAgaWYgKGhhc1BlbmRpbmdDb250ZW50KSB7XG4gICAgICBjYXB0dXJlU2VnbWVudChzdGF0ZSwgY2FwdHVyZVN0YXJ0LCBjYXB0dXJlRW5kLCBmYWxzZSk7XG4gICAgICB3cml0ZUZvbGRlZExpbmVzKHN0YXRlLCBzdGF0ZS5saW5lIC0gbGluZSk7XG4gICAgICBjYXB0dXJlU3RhcnQgPSBjYXB0dXJlRW5kID0gc3RhdGUucG9zaXRpb247XG4gICAgICBoYXNQZW5kaW5nQ29udGVudCA9IGZhbHNlO1xuICAgIH1cblxuICAgIGlmICghaXNXaGl0ZVNwYWNlKGNoKSkge1xuICAgICAgY2FwdHVyZUVuZCA9IHN0YXRlLnBvc2l0aW9uICsgMTtcbiAgICB9XG5cbiAgICBjaCA9IHN0YXRlLmlucHV0LmNoYXJDb2RlQXQoKytzdGF0ZS5wb3NpdGlvbik7XG4gIH1cblxuICBjYXB0dXJlU2VnbWVudChzdGF0ZSwgY2FwdHVyZVN0YXJ0LCBjYXB0dXJlRW5kLCBmYWxzZSk7XG5cbiAgaWYgKHN0YXRlLnJlc3VsdCkge1xuICAgIHJldHVybiB0cnVlO1xuICB9XG5cbiAgc3RhdGUua2luZCA9IGtpbmQ7XG4gIHN0YXRlLnJlc3VsdCA9IHJlc3VsdDtcbiAgcmV0dXJuIGZhbHNlO1xufVxuXG5mdW5jdGlvbiByZWFkU2luZ2xlUXVvdGVkU2NhbGFyKFxuICBzdGF0ZTogTG9hZGVyU3RhdGUsXG4gIG5vZGVJbmRlbnQ6IG51bWJlcixcbik6IGJvb2xlYW4ge1xuICBsZXQgY2gsIGNhcHR1cmVTdGFydCwgY2FwdHVyZUVuZDtcblxuICBjaCA9IHN0YXRlLmlucHV0LmNoYXJDb2RlQXQoc3RhdGUucG9zaXRpb24pO1xuXG4gIGlmIChjaCAhPT0gMHgyNyAvKiAnICovKSB7XG4gICAgcmV0dXJuIGZhbHNlO1xuICB9XG5cbiAgc3RhdGUua2luZCA9IFwic2NhbGFyXCI7XG4gIHN0YXRlLnJlc3VsdCA9IFwiXCI7XG4gIHN0YXRlLnBvc2l0aW9uKys7XG4gIGNhcHR1cmVTdGFydCA9IGNhcHR1cmVFbmQgPSBzdGF0ZS5wb3NpdGlvbjtcblxuICB3aGlsZSAoKGNoID0gc3RhdGUuaW5wdXQuY2hhckNvZGVBdChzdGF0ZS5wb3NpdGlvbikpICE9PSAwKSB7XG4gICAgaWYgKGNoID09PSAweDI3IC8qICcgKi8pIHtcbiAgICAgIGNhcHR1cmVTZWdtZW50KHN0YXRlLCBjYXB0dXJlU3RhcnQsIHN0YXRlLnBvc2l0aW9uLCB0cnVlKTtcbiAgICAgIGNoID0gc3RhdGUuaW5wdXQuY2hhckNvZGVBdCgrK3N0YXRlLnBvc2l0aW9uKTtcblxuICAgICAgaWYgKGNoID09PSAweDI3IC8qICcgKi8pIHtcbiAgICAgICAgY2FwdHVyZVN0YXJ0ID0gc3RhdGUucG9zaXRpb247XG4gICAgICAgIHN0YXRlLnBvc2l0aW9uKys7XG4gICAgICAgIGNhcHR1cmVFbmQgPSBzdGF0ZS5wb3NpdGlvbjtcbiAgICAgIH0gZWxzZSB7XG4gICAgICAgIHJldHVybiB0cnVlO1xuICAgICAgfVxuICAgIH0gZWxzZSBpZiAoaXNFT0woY2gpKSB7XG4gICAgICBjYXB0dXJlU2VnbWVudChzdGF0ZSwgY2FwdHVyZVN0YXJ0LCBjYXB0dXJlRW5kLCB0cnVlKTtcbiAgICAgIHdyaXRlRm9sZGVkTGluZXMoc3RhdGUsIHNraXBTZXBhcmF0aW9uU3BhY2Uoc3RhdGUsIGZhbHNlLCBub2RlSW5kZW50KSk7XG4gICAgICBjYXB0dXJlU3RhcnQgPSBjYXB0dXJlRW5kID0gc3RhdGUucG9zaXRpb247XG4gICAgfSBlbHNlIGlmIChcbiAgICAgIHN0YXRlLnBvc2l0aW9uID09PSBzdGF0ZS5saW5lU3RhcnQgJiZcbiAgICAgIHRlc3REb2N1bWVudFNlcGFyYXRvcihzdGF0ZSlcbiAgICApIHtcbiAgICAgIHJldHVybiB0aHJvd0Vycm9yKFxuICAgICAgICBzdGF0ZSxcbiAgICAgICAgXCJ1bmV4cGVjdGVkIGVuZCBvZiB0aGUgZG9jdW1lbnQgd2l0aGluIGEgc2luZ2xlIHF1b3RlZCBzY2FsYXJcIixcbiAgICAgICk7XG4gICAgfSBlbHNlIHtcbiAgICAgIHN0YXRlLnBvc2l0aW9uKys7XG4gICAgICBjYXB0dXJlRW5kID0gc3RhdGUucG9zaXRpb247XG4gICAgfVxuICB9XG5cbiAgcmV0dXJuIHRocm93RXJyb3IoXG4gICAgc3RhdGUsXG4gICAgXCJ1bmV4cGVjdGVkIGVuZCBvZiB0aGUgc3RyZWFtIHdpdGhpbiBhIHNpbmdsZSBxdW90ZWQgc2NhbGFyXCIsXG4gICk7XG59XG5cbmZ1bmN0aW9uIHJlYWREb3VibGVRdW90ZWRTY2FsYXIoXG4gIHN0YXRlOiBMb2FkZXJTdGF0ZSxcbiAgbm9kZUluZGVudDogbnVtYmVyLFxuKTogYm9vbGVhbiB7XG4gIGxldCBjaCA9IHN0YXRlLmlucHV0LmNoYXJDb2RlQXQoc3RhdGUucG9zaXRpb24pO1xuXG4gIGlmIChjaCAhPT0gMHgyMiAvKiBcIiAqLykge1xuICAgIHJldHVybiBmYWxzZTtcbiAgfVxuXG4gIHN0YXRlLmtpbmQgPSBcInNjYWxhclwiO1xuICBzdGF0ZS5yZXN1bHQgPSBcIlwiO1xuICBzdGF0ZS5wb3NpdGlvbisrO1xuICBsZXQgY2FwdHVyZUVuZDogbnVtYmVyLFxuICAgIGNhcHR1cmVTdGFydCA9IChjYXB0dXJlRW5kID0gc3RhdGUucG9zaXRpb24pO1xuICBsZXQgdG1wOiBudW1iZXI7XG4gIHdoaWxlICgoY2ggPSBzdGF0ZS5pbnB1dC5jaGFyQ29kZUF0KHN0YXRlLnBvc2l0aW9uKSkgIT09IDApIHtcbiAgICBpZiAoY2ggPT09IDB4MjIgLyogXCIgKi8pIHtcbiAgICAgIGNhcHR1cmVTZWdtZW50KHN0YXRlLCBjYXB0dXJlU3RhcnQsIHN0YXRlLnBvc2l0aW9uLCB0cnVlKTtcbiAgICAgIHN0YXRlLnBvc2l0aW9uKys7XG4gICAgICByZXR1cm4gdHJ1ZTtcbiAgICB9XG4gICAgaWYgKGNoID09PSAweDVjIC8qIFxcICovKSB7XG4gICAgICBjYXB0dXJlU2VnbWVudChzdGF0ZSwgY2FwdHVyZVN0YXJ0LCBzdGF0ZS5wb3NpdGlvbiwgdHJ1ZSk7XG4gICAgICBjaCA9IHN0YXRlLmlucHV0LmNoYXJDb2RlQXQoKytzdGF0ZS5wb3NpdGlvbik7XG5cbiAgICAgIGlmIChpc0VPTChjaCkpIHtcbiAgICAgICAgc2tpcFNlcGFyYXRpb25TcGFjZShzdGF0ZSwgZmFsc2UsIG5vZGVJbmRlbnQpO1xuXG4gICAgICAgIC8vIFRPRE8oYmFydGxvbWllanUpOiByZXdvcmsgdG8gaW5saW5lIGZuIHdpdGggbm8gdHlwZSBjYXN0P1xuICAgICAgfSBlbHNlIGlmIChjaCA8IDI1NiAmJiBzaW1wbGVFc2NhcGVDaGVja1tjaF0pIHtcbiAgICAgICAgc3RhdGUucmVzdWx0ICs9IHNpbXBsZUVzY2FwZU1hcFtjaF07XG4gICAgICAgIHN0YXRlLnBvc2l0aW9uKys7XG4gICAgICB9IGVsc2UgaWYgKCh0bXAgPSBlc2NhcGVkSGV4TGVuKGNoKSkgPiAwKSB7XG4gICAgICAgIGxldCBoZXhMZW5ndGggPSB0bXA7XG4gICAgICAgIGxldCBoZXhSZXN1bHQgPSAwO1xuXG4gICAgICAgIGZvciAoOyBoZXhMZW5ndGggPiAwOyBoZXhMZW5ndGgtLSkge1xuICAgICAgICAgIGNoID0gc3RhdGUuaW5wdXQuY2hhckNvZGVBdCgrK3N0YXRlLnBvc2l0aW9uKTtcblxuICAgICAgICAgIGlmICgodG1wID0gZnJvbUhleENvZGUoY2gpKSA+PSAwKSB7XG4gICAgICAgICAgICBoZXhSZXN1bHQgPSAoaGV4UmVzdWx0IDw8IDQpICsgdG1wO1xuICAgICAgICAgIH0gZWxzZSB7XG4gICAgICAgICAgICByZXR1cm4gdGhyb3dFcnJvcihzdGF0ZSwgXCJleHBlY3RlZCBoZXhhZGVjaW1hbCBjaGFyYWN0ZXJcIik7XG4gICAgICAgICAgfVxuICAgICAgICB9XG5cbiAgICAgICAgc3RhdGUucmVzdWx0ICs9IGNoYXJGcm9tQ29kZXBvaW50KGhleFJlc3VsdCk7XG5cbiAgICAgICAgc3RhdGUucG9zaXRpb24rKztcbiAgICAgIH0gZWxzZSB7XG4gICAgICAgIHJldHVybiB0aHJvd0Vycm9yKHN0YXRlLCBcInVua25vd24gZXNjYXBlIHNlcXVlbmNlXCIpO1xuICAgICAgfVxuXG4gICAgICBjYXB0dXJlU3RhcnQgPSBjYXB0dXJlRW5kID0gc3RhdGUucG9zaXRpb247XG4gICAgfSBlbHNlIGlmIChpc0VPTChjaCkpIHtcbiAgICAgIGNhcHR1cmVTZWdtZW50KHN0YXRlLCBjYXB0dXJlU3RhcnQsIGNhcHR1cmVFbmQsIHRydWUpO1xuICAgICAgd3JpdGVGb2xkZWRMaW5lcyhzdGF0ZSwgc2tpcFNlcGFyYXRpb25TcGFjZShzdGF0ZSwgZmFsc2UsIG5vZGVJbmRlbnQpKTtcbiAgICAgIGNhcHR1cmVTdGFydCA9IGNhcHR1cmVFbmQgPSBzdGF0ZS5wb3NpdGlvbjtcbiAgICB9IGVsc2UgaWYgKFxuICAgICAgc3RhdGUucG9zaXRpb24gPT09IHN0YXRlLmxpbmVTdGFydCAmJlxuICAgICAgdGVzdERvY3VtZW50U2VwYXJhdG9yKHN0YXRlKVxuICAgICkge1xuICAgICAgcmV0dXJuIHRocm93RXJyb3IoXG4gICAgICAgIHN0YXRlLFxuICAgICAgICBcInVuZXhwZWN0ZWQgZW5kIG9mIHRoZSBkb2N1bWVudCB3aXRoaW4gYSBkb3VibGUgcXVvdGVkIHNjYWxhclwiLFxuICAgICAgKTtcbiAgICB9IGVsc2Uge1xuICAgICAgc3RhdGUucG9zaXRpb24rKztcbiAgICAgIGNhcHR1cmVFbmQgPSBzdGF0ZS5wb3NpdGlvbjtcbiAgICB9XG4gIH1cblxuICByZXR1cm4gdGhyb3dFcnJvcihcbiAgICBzdGF0ZSxcbiAgICBcInVuZXhwZWN0ZWQgZW5kIG9mIHRoZSBzdHJlYW0gd2l0aGluIGEgZG91YmxlIHF1b3RlZCBzY2FsYXJcIixcbiAgKTtcbn1cblxuZnVuY3Rpb24gcmVhZEZsb3dDb2xsZWN0aW9uKHN0YXRlOiBMb2FkZXJTdGF0ZSwgbm9kZUluZGVudDogbnVtYmVyKTogYm9vbGVhbiB7XG4gIGxldCBjaCA9IHN0YXRlLmlucHV0LmNoYXJDb2RlQXQoc3RhdGUucG9zaXRpb24pO1xuICBsZXQgdGVybWluYXRvcjogbnVtYmVyO1xuICBsZXQgaXNNYXBwaW5nID0gdHJ1ZTtcbiAgbGV0IHJlc3VsdDogUmVzdWx0VHlwZSA9IHt9O1xuICBpZiAoY2ggPT09IDB4NWIgLyogWyAqLykge1xuICAgIHRlcm1pbmF0b3IgPSAweDVkOyAvKiBdICovXG4gICAgaXNNYXBwaW5nID0gZmFsc2U7XG4gICAgcmVzdWx0ID0gW107XG4gIH0gZWxzZSBpZiAoY2ggPT09IDB4N2IgLyogeyAqLykge1xuICAgIHRlcm1pbmF0b3IgPSAweDdkOyAvKiB9ICovXG4gIH0gZWxzZSB7XG4gICAgcmV0dXJuIGZhbHNlO1xuICB9XG5cbiAgaWYgKFxuICAgIHN0YXRlLmFuY2hvciAhPT0gbnVsbCAmJlxuICAgIHR5cGVvZiBzdGF0ZS5hbmNob3IgIT0gXCJ1bmRlZmluZWRcIiAmJlxuICAgIHR5cGVvZiBzdGF0ZS5hbmNob3JNYXAgIT0gXCJ1bmRlZmluZWRcIlxuICApIHtcbiAgICBzdGF0ZS5hbmNob3JNYXBbc3RhdGUuYW5jaG9yXSA9IHJlc3VsdDtcbiAgfVxuXG4gIGNoID0gc3RhdGUuaW5wdXQuY2hhckNvZGVBdCgrK3N0YXRlLnBvc2l0aW9uKTtcblxuICBjb25zdCB0YWcgPSBzdGF0ZS50YWcsXG4gICAgYW5jaG9yID0gc3RhdGUuYW5jaG9yO1xuICBsZXQgcmVhZE5leHQgPSB0cnVlO1xuICBsZXQgdmFsdWVOb2RlLFxuICAgIGtleU5vZGUsXG4gICAga2V5VGFnOiBzdHJpbmcgfCBudWxsID0gKGtleU5vZGUgPSB2YWx1ZU5vZGUgPSBudWxsKSxcbiAgICBpc0V4cGxpY2l0UGFpcjogYm9vbGVhbixcbiAgICBpc1BhaXIgPSAoaXNFeHBsaWNpdFBhaXIgPSBmYWxzZSk7XG4gIGxldCBmb2xsb3dpbmcgPSAwLFxuICAgIGxpbmUgPSAwO1xuICBjb25zdCBvdmVycmlkYWJsZUtleXM6IEFycmF5T2JqZWN0PGJvb2xlYW4+ID0ge307XG4gIHdoaWxlIChjaCAhPT0gMCkge1xuICAgIHNraXBTZXBhcmF0aW9uU3BhY2Uoc3RhdGUsIHRydWUsIG5vZGVJbmRlbnQpO1xuXG4gICAgY2ggPSBzdGF0ZS5pbnB1dC5jaGFyQ29kZUF0KHN0YXRlLnBvc2l0aW9uKTtcblxuICAgIGlmIChjaCA9PT0gdGVybWluYXRvcikge1xuICAgICAgc3RhdGUucG9zaXRpb24rKztcbiAgICAgIHN0YXRlLnRhZyA9IHRhZztcbiAgICAgIHN0YXRlLmFuY2hvciA9IGFuY2hvcjtcbiAgICAgIHN0YXRlLmtpbmQgPSBpc01hcHBpbmcgPyBcIm1hcHBpbmdcIiA6IFwic2VxdWVuY2VcIjtcbiAgICAgIHN0YXRlLnJlc3VsdCA9IHJlc3VsdDtcbiAgICAgIHJldHVybiB0cnVlO1xuICAgIH1cbiAgICBpZiAoIXJlYWROZXh0KSB7XG4gICAgICByZXR1cm4gdGhyb3dFcnJvcihzdGF0ZSwgXCJtaXNzZWQgY29tbWEgYmV0d2VlbiBmbG93IGNvbGxlY3Rpb24gZW50cmllc1wiKTtcbiAgICB9XG5cbiAgICBrZXlUYWcgPSBrZXlOb2RlID0gdmFsdWVOb2RlID0gbnVsbDtcbiAgICBpc1BhaXIgPSBpc0V4cGxpY2l0UGFpciA9IGZhbHNlO1xuXG4gICAgaWYgKGNoID09PSAweDNmIC8qID8gKi8pIHtcbiAgICAgIGZvbGxvd2luZyA9IHN0YXRlLmlucHV0LmNoYXJDb2RlQXQoc3RhdGUucG9zaXRpb24gKyAxKTtcblxuICAgICAgaWYgKGlzV3NPckVvbChmb2xsb3dpbmcpKSB7XG4gICAgICAgIGlzUGFpciA9IGlzRXhwbGljaXRQYWlyID0gdHJ1ZTtcbiAgICAgICAgc3RhdGUucG9zaXRpb24rKztcbiAgICAgICAgc2tpcFNlcGFyYXRpb25TcGFjZShzdGF0ZSwgdHJ1ZSwgbm9kZUluZGVudCk7XG4gICAgICB9XG4gICAgfVxuXG4gICAgbGluZSA9IHN0YXRlLmxpbmU7XG4gICAgY29tcG9zZU5vZGUoc3RhdGUsIG5vZGVJbmRlbnQsIENPTlRFWFRfRkxPV19JTiwgZmFsc2UsIHRydWUpO1xuICAgIGtleVRhZyA9IHN0YXRlLnRhZyB8fCBudWxsO1xuICAgIGtleU5vZGUgPSBzdGF0ZS5yZXN1bHQ7XG4gICAgc2tpcFNlcGFyYXRpb25TcGFjZShzdGF0ZSwgdHJ1ZSwgbm9kZUluZGVudCk7XG5cbiAgICBjaCA9IHN0YXRlLmlucHV0LmNoYXJDb2RlQXQoc3RhdGUucG9zaXRpb24pO1xuXG4gICAgaWYgKChpc0V4cGxpY2l0UGFpciB8fCBzdGF0ZS5saW5lID09PSBsaW5lKSAmJiBjaCA9PT0gMHgzYSAvKiA6ICovKSB7XG4gICAgICBpc1BhaXIgPSB0cnVlO1xuICAgICAgY2ggPSBzdGF0ZS5pbnB1dC5jaGFyQ29kZUF0KCsrc3RhdGUucG9zaXRpb24pO1xuICAgICAgc2tpcFNlcGFyYXRpb25TcGFjZShzdGF0ZSwgdHJ1ZSwgbm9kZUluZGVudCk7XG4gICAgICBjb21wb3NlTm9kZShzdGF0ZSwgbm9kZUluZGVudCwgQ09OVEVYVF9GTE9XX0lOLCBmYWxzZSwgdHJ1ZSk7XG4gICAgICB2YWx1ZU5vZGUgPSBzdGF0ZS5yZXN1bHQ7XG4gICAgfVxuXG4gICAgaWYgKGlzTWFwcGluZykge1xuICAgICAgc3RvcmVNYXBwaW5nUGFpcihcbiAgICAgICAgc3RhdGUsXG4gICAgICAgIHJlc3VsdCxcbiAgICAgICAgb3ZlcnJpZGFibGVLZXlzLFxuICAgICAgICBrZXlUYWcsXG4gICAgICAgIGtleU5vZGUsXG4gICAgICAgIHZhbHVlTm9kZSxcbiAgICAgICk7XG4gICAgfSBlbHNlIGlmIChpc1BhaXIpIHtcbiAgICAgIChyZXN1bHQgYXMgQXJyYXlPYmplY3RbXSkucHVzaChcbiAgICAgICAgc3RvcmVNYXBwaW5nUGFpcihcbiAgICAgICAgICBzdGF0ZSxcbiAgICAgICAgICBudWxsLFxuICAgICAgICAgIG92ZXJyaWRhYmxlS2V5cyxcbiAgICAgICAgICBrZXlUYWcsXG4gICAgICAgICAga2V5Tm9kZSxcbiAgICAgICAgICB2YWx1ZU5vZGUsXG4gICAgICAgICksXG4gICAgICApO1xuICAgIH0gZWxzZSB7XG4gICAgICAocmVzdWx0IGFzIFJlc3VsdFR5cGVbXSkucHVzaChrZXlOb2RlIGFzIFJlc3VsdFR5cGUpO1xuICAgIH1cblxuICAgIHNraXBTZXBhcmF0aW9uU3BhY2Uoc3RhdGUsIHRydWUsIG5vZGVJbmRlbnQpO1xuXG4gICAgY2ggPSBzdGF0ZS5pbnB1dC5jaGFyQ29kZUF0KHN0YXRlLnBvc2l0aW9uKTtcblxuICAgIGlmIChjaCA9PT0gMHgyYyAvKiAsICovKSB7XG4gICAgICByZWFkTmV4dCA9IHRydWU7XG4gICAgICBjaCA9IHN0YXRlLmlucHV0LmNoYXJDb2RlQXQoKytzdGF0ZS5wb3NpdGlvbik7XG4gICAgfSBlbHNlIHtcbiAgICAgIHJlYWROZXh0ID0gZmFsc2U7XG4gICAgfVxuICB9XG5cbiAgcmV0dXJuIHRocm93RXJyb3IoXG4gICAgc3RhdGUsXG4gICAgXCJ1bmV4cGVjdGVkIGVuZCBvZiB0aGUgc3RyZWFtIHdpdGhpbiBhIGZsb3cgY29sbGVjdGlvblwiLFxuICApO1xufVxuXG5mdW5jdGlvbiByZWFkQmxvY2tTY2FsYXIoc3RhdGU6IExvYWRlclN0YXRlLCBub2RlSW5kZW50OiBudW1iZXIpOiBib29sZWFuIHtcbiAgbGV0IGNob21waW5nID0gQ0hPTVBJTkdfQ0xJUCxcbiAgICBkaWRSZWFkQ29udGVudCA9IGZhbHNlLFxuICAgIGRldGVjdGVkSW5kZW50ID0gZmFsc2UsXG4gICAgdGV4dEluZGVudCA9IG5vZGVJbmRlbnQsXG4gICAgZW1wdHlMaW5lcyA9IDAsXG4gICAgYXRNb3JlSW5kZW50ZWQgPSBmYWxzZTtcblxuICBsZXQgY2ggPSBzdGF0ZS5pbnB1dC5jaGFyQ29kZUF0KHN0YXRlLnBvc2l0aW9uKTtcblxuICBsZXQgZm9sZGluZyA9IGZhbHNlO1xuICBpZiAoY2ggPT09IDB4N2MgLyogfCAqLykge1xuICAgIGZvbGRpbmcgPSBmYWxzZTtcbiAgfSBlbHNlIGlmIChjaCA9PT0gMHgzZSAvKiA+ICovKSB7XG4gICAgZm9sZGluZyA9IHRydWU7XG4gIH0gZWxzZSB7XG4gICAgcmV0dXJuIGZhbHNlO1xuICB9XG5cbiAgc3RhdGUua2luZCA9IFwic2NhbGFyXCI7XG4gIHN0YXRlLnJlc3VsdCA9IFwiXCI7XG5cbiAgbGV0IHRtcCA9IDA7XG4gIHdoaWxlIChjaCAhPT0gMCkge1xuICAgIGNoID0gc3RhdGUuaW5wdXQuY2hhckNvZGVBdCgrK3N0YXRlLnBvc2l0aW9uKTtcblxuICAgIGlmIChjaCA9PT0gMHgyYiB8fCAvKiArICovIGNoID09PSAweDJkIC8qIC0gKi8pIHtcbiAgICAgIGlmIChDSE9NUElOR19DTElQID09PSBjaG9tcGluZykge1xuICAgICAgICBjaG9tcGluZyA9IGNoID09PSAweDJiIC8qICsgKi8gPyBDSE9NUElOR19LRUVQIDogQ0hPTVBJTkdfU1RSSVA7XG4gICAgICB9IGVsc2Uge1xuICAgICAgICByZXR1cm4gdGhyb3dFcnJvcihzdGF0ZSwgXCJyZXBlYXQgb2YgYSBjaG9tcGluZyBtb2RlIGlkZW50aWZpZXJcIik7XG4gICAgICB9XG4gICAgfSBlbHNlIGlmICgodG1wID0gZnJvbURlY2ltYWxDb2RlKGNoKSkgPj0gMCkge1xuICAgICAgaWYgKHRtcCA9PT0gMCkge1xuICAgICAgICByZXR1cm4gdGhyb3dFcnJvcihcbiAgICAgICAgICBzdGF0ZSxcbiAgICAgICAgICBcImJhZCBleHBsaWNpdCBpbmRlbnRhdGlvbiB3aWR0aCBvZiBhIGJsb2NrIHNjYWxhcjsgaXQgY2Fubm90IGJlIGxlc3MgdGhhbiBvbmVcIixcbiAgICAgICAgKTtcbiAgICAgIH0gZWxzZSBpZiAoIWRldGVjdGVkSW5kZW50KSB7XG4gICAgICAgIHRleHRJbmRlbnQgPSBub2RlSW5kZW50ICsgdG1wIC0gMTtcbiAgICAgICAgZGV0ZWN0ZWRJbmRlbnQgPSB0cnVlO1xuICAgICAgfSBlbHNlIHtcbiAgICAgICAgcmV0dXJuIHRocm93RXJyb3Ioc3RhdGUsIFwicmVwZWF0IG9mIGFuIGluZGVudGF0aW9uIHdpZHRoIGlkZW50aWZpZXJcIik7XG4gICAgICB9XG4gICAgfSBlbHNlIHtcbiAgICAgIGJyZWFrO1xuICAgIH1cbiAgfVxuXG4gIGlmIChpc1doaXRlU3BhY2UoY2gpKSB7XG4gICAgZG8ge1xuICAgICAgY2ggPSBzdGF0ZS5pbnB1dC5jaGFyQ29kZUF0KCsrc3RhdGUucG9zaXRpb24pO1xuICAgIH0gd2hpbGUgKGlzV2hpdGVTcGFjZShjaCkpO1xuXG4gICAgaWYgKGNoID09PSAweDIzIC8qICMgKi8pIHtcbiAgICAgIGRvIHtcbiAgICAgICAgY2ggPSBzdGF0ZS5pbnB1dC5jaGFyQ29kZUF0KCsrc3RhdGUucG9zaXRpb24pO1xuICAgICAgfSB3aGlsZSAoIWlzRU9MKGNoKSAmJiBjaCAhPT0gMCk7XG4gICAgfVxuICB9XG5cbiAgd2hpbGUgKGNoICE9PSAwKSB7XG4gICAgcmVhZExpbmVCcmVhayhzdGF0ZSk7XG4gICAgc3RhdGUubGluZUluZGVudCA9IDA7XG5cbiAgICBjaCA9IHN0YXRlLmlucHV0LmNoYXJDb2RlQXQoc3RhdGUucG9zaXRpb24pO1xuXG4gICAgd2hpbGUgKFxuICAgICAgKCFkZXRlY3RlZEluZGVudCB8fCBzdGF0ZS5saW5lSW5kZW50IDwgdGV4dEluZGVudCkgJiZcbiAgICAgIGNoID09PSAweDIwIC8qIFNwYWNlICovXG4gICAgKSB7XG4gICAgICBzdGF0ZS5saW5lSW5kZW50Kys7XG4gICAgICBjaCA9IHN0YXRlLmlucHV0LmNoYXJDb2RlQXQoKytzdGF0ZS5wb3NpdGlvbik7XG4gICAgfVxuXG4gICAgaWYgKCFkZXRlY3RlZEluZGVudCAmJiBzdGF0ZS5saW5lSW5kZW50ID4gdGV4dEluZGVudCkge1xuICAgICAgdGV4dEluZGVudCA9IHN0YXRlLmxpbmVJbmRlbnQ7XG4gICAgfVxuXG4gICAgaWYgKGlzRU9MKGNoKSkge1xuICAgICAgZW1wdHlMaW5lcysrO1xuICAgICAgY29udGludWU7XG4gICAgfVxuXG4gICAgLy8gRW5kIG9mIHRoZSBzY2FsYXIuXG4gICAgaWYgKHN0YXRlLmxpbmVJbmRlbnQgPCB0ZXh0SW5kZW50KSB7XG4gICAgICAvLyBQZXJmb3JtIHRoZSBjaG9tcGluZy5cbiAgICAgIGlmIChjaG9tcGluZyA9PT0gQ0hPTVBJTkdfS0VFUCkge1xuICAgICAgICBzdGF0ZS5yZXN1bHQgKz0gY29tbW9uLnJlcGVhdChcbiAgICAgICAgICBcIlxcblwiLFxuICAgICAgICAgIGRpZFJlYWRDb250ZW50ID8gMSArIGVtcHR5TGluZXMgOiBlbXB0eUxpbmVzLFxuICAgICAgICApO1xuICAgICAgfSBlbHNlIGlmIChjaG9tcGluZyA9PT0gQ0hPTVBJTkdfQ0xJUCkge1xuICAgICAgICBpZiAoZGlkUmVhZENvbnRlbnQpIHtcbiAgICAgICAgICAvLyBpLmUuIG9ubHkgaWYgdGhlIHNjYWxhciBpcyBub3QgZW1wdHkuXG4gICAgICAgICAgc3RhdGUucmVzdWx0ICs9IFwiXFxuXCI7XG4gICAgICAgIH1cbiAgICAgIH1cblxuICAgICAgLy8gQnJlYWsgdGhpcyBgd2hpbGVgIGN5Y2xlIGFuZCBnbyB0byB0aGUgZnVuY3Rpb24ncyBlcGlsb2d1ZS5cbiAgICAgIGJyZWFrO1xuICAgIH1cblxuICAgIC8vIEZvbGRlZCBzdHlsZTogdXNlIGZhbmN5IHJ1bGVzIHRvIGhhbmRsZSBsaW5lIGJyZWFrcy5cbiAgICBpZiAoZm9sZGluZykge1xuICAgICAgLy8gTGluZXMgc3RhcnRpbmcgd2l0aCB3aGl0ZSBzcGFjZSBjaGFyYWN0ZXJzIChtb3JlLWluZGVudGVkIGxpbmVzKSBhcmUgbm90IGZvbGRlZC5cbiAgICAgIGlmIChpc1doaXRlU3BhY2UoY2gpKSB7XG4gICAgICAgIGF0TW9yZUluZGVudGVkID0gdHJ1ZTtcbiAgICAgICAgLy8gZXhjZXB0IGZvciB0aGUgZmlyc3QgY29udGVudCBsaW5lIChjZi4gRXhhbXBsZSA4LjEpXG4gICAgICAgIHN0YXRlLnJlc3VsdCArPSBjb21tb24ucmVwZWF0KFxuICAgICAgICAgIFwiXFxuXCIsXG4gICAgICAgICAgZGlkUmVhZENvbnRlbnQgPyAxICsgZW1wdHlMaW5lcyA6IGVtcHR5TGluZXMsXG4gICAgICAgICk7XG5cbiAgICAgICAgLy8gRW5kIG9mIG1vcmUtaW5kZW50ZWQgYmxvY2suXG4gICAgICB9IGVsc2UgaWYgKGF0TW9yZUluZGVudGVkKSB7XG4gICAgICAgIGF0TW9yZUluZGVudGVkID0gZmFsc2U7XG4gICAgICAgIHN0YXRlLnJlc3VsdCArPSBjb21tb24ucmVwZWF0KFwiXFxuXCIsIGVtcHR5TGluZXMgKyAxKTtcblxuICAgICAgICAvLyBKdXN0IG9uZSBsaW5lIGJyZWFrIC0gcGVyY2VpdmUgYXMgdGhlIHNhbWUgbGluZS5cbiAgICAgIH0gZWxzZSBpZiAoZW1wdHlMaW5lcyA9PT0gMCkge1xuICAgICAgICBpZiAoZGlkUmVhZENvbnRlbnQpIHtcbiAgICAgICAgICAvLyBpLmUuIG9ubHkgaWYgd2UgaGF2ZSBhbHJlYWR5IHJlYWQgc29tZSBzY2FsYXIgY29udGVudC5cbiAgICAgICAgICBzdGF0ZS5yZXN1bHQgKz0gXCIgXCI7XG4gICAgICAgIH1cblxuICAgICAgICAvLyBTZXZlcmFsIGxpbmUgYnJlYWtzIC0gcGVyY2VpdmUgYXMgZGlmZmVyZW50IGxpbmVzLlxuICAgICAgfSBlbHNlIHtcbiAgICAgICAgc3RhdGUucmVzdWx0ICs9IGNvbW1vbi5yZXBlYXQoXCJcXG5cIiwgZW1wdHlMaW5lcyk7XG4gICAgICB9XG5cbiAgICAgIC8vIExpdGVyYWwgc3R5bGU6IGp1c3QgYWRkIGV4YWN0IG51bWJlciBvZiBsaW5lIGJyZWFrcyBiZXR3ZWVuIGNvbnRlbnQgbGluZXMuXG4gICAgfSBlbHNlIHtcbiAgICAgIC8vIEtlZXAgYWxsIGxpbmUgYnJlYWtzIGV4Y2VwdCB0aGUgaGVhZGVyIGxpbmUgYnJlYWsuXG4gICAgICBzdGF0ZS5yZXN1bHQgKz0gY29tbW9uLnJlcGVhdChcbiAgICAgICAgXCJcXG5cIixcbiAgICAgICAgZGlkUmVhZENvbnRlbnQgPyAxICsgZW1wdHlMaW5lcyA6IGVtcHR5TGluZXMsXG4gICAgICApO1xuICAgIH1cblxuICAgIGRpZFJlYWRDb250ZW50ID0gdHJ1ZTtcbiAgICBkZXRlY3RlZEluZGVudCA9IHRydWU7XG4gICAgZW1wdHlMaW5lcyA9IDA7XG4gICAgY29uc3QgY2FwdHVyZVN0YXJ0ID0gc3RhdGUucG9zaXRpb247XG5cbiAgICB3aGlsZSAoIWlzRU9MKGNoKSAmJiBjaCAhPT0gMCkge1xuICAgICAgY2ggPSBzdGF0ZS5pbnB1dC5jaGFyQ29kZUF0KCsrc3RhdGUucG9zaXRpb24pO1xuICAgIH1cblxuICAgIGNhcHR1cmVTZWdtZW50KHN0YXRlLCBjYXB0dXJlU3RhcnQsIHN0YXRlLnBvc2l0aW9uLCBmYWxzZSk7XG4gIH1cblxuICByZXR1cm4gdHJ1ZTtcbn1cblxuZnVuY3Rpb24gcmVhZEJsb2NrU2VxdWVuY2Uoc3RhdGU6IExvYWRlclN0YXRlLCBub2RlSW5kZW50OiBudW1iZXIpOiBib29sZWFuIHtcbiAgbGV0IGxpbmU6IG51bWJlcixcbiAgICBmb2xsb3dpbmc6IG51bWJlcixcbiAgICBkZXRlY3RlZCA9IGZhbHNlLFxuICAgIGNoOiBudW1iZXI7XG4gIGNvbnN0IHRhZyA9IHN0YXRlLnRhZyxcbiAgICBhbmNob3IgPSBzdGF0ZS5hbmNob3IsXG4gICAgcmVzdWx0OiB1bmtub3duW10gPSBbXTtcblxuICBpZiAoXG4gICAgc3RhdGUuYW5jaG9yICE9PSBudWxsICYmXG4gICAgdHlwZW9mIHN0YXRlLmFuY2hvciAhPT0gXCJ1bmRlZmluZWRcIiAmJlxuICAgIHR5cGVvZiBzdGF0ZS5hbmNob3JNYXAgIT09IFwidW5kZWZpbmVkXCJcbiAgKSB7XG4gICAgc3RhdGUuYW5jaG9yTWFwW3N0YXRlLmFuY2hvcl0gPSByZXN1bHQ7XG4gIH1cblxuICBjaCA9IHN0YXRlLmlucHV0LmNoYXJDb2RlQXQoc3RhdGUucG9zaXRpb24pO1xuXG4gIHdoaWxlIChjaCAhPT0gMCkge1xuICAgIGlmIChjaCAhPT0gMHgyZCAvKiAtICovKSB7XG4gICAgICBicmVhaztcbiAgICB9XG5cbiAgICBmb2xsb3dpbmcgPSBzdGF0ZS5pbnB1dC5jaGFyQ29kZUF0KHN0YXRlLnBvc2l0aW9uICsgMSk7XG5cbiAgICBpZiAoIWlzV3NPckVvbChmb2xsb3dpbmcpKSB7XG4gICAgICBicmVhaztcbiAgICB9XG5cbiAgICBkZXRlY3RlZCA9IHRydWU7XG4gICAgc3RhdGUucG9zaXRpb24rKztcblxuICAgIGlmIChza2lwU2VwYXJhdGlvblNwYWNlKHN0YXRlLCB0cnVlLCAtMSkpIHtcbiAgICAgIGlmIChzdGF0ZS5saW5lSW5kZW50IDw9IG5vZGVJbmRlbnQpIHtcbiAgICAgICAgcmVzdWx0LnB1c2gobnVsbCk7XG4gICAgICAgIGNoID0gc3RhdGUuaW5wdXQuY2hhckNvZGVBdChzdGF0ZS5wb3NpdGlvbik7XG4gICAgICAgIGNvbnRpbnVlO1xuICAgICAgfVxuICAgIH1cblxuICAgIGxpbmUgPSBzdGF0ZS5saW5lO1xuICAgIGNvbXBvc2VOb2RlKHN0YXRlLCBub2RlSW5kZW50LCBDT05URVhUX0JMT0NLX0lOLCBmYWxzZSwgdHJ1ZSk7XG4gICAgcmVzdWx0LnB1c2goc3RhdGUucmVzdWx0KTtcbiAgICBza2lwU2VwYXJhdGlvblNwYWNlKHN0YXRlLCB0cnVlLCAtMSk7XG5cbiAgICBjaCA9IHN0YXRlLmlucHV0LmNoYXJDb2RlQXQoc3RhdGUucG9zaXRpb24pO1xuXG4gICAgaWYgKChzdGF0ZS5saW5lID09PSBsaW5lIHx8IHN0YXRlLmxpbmVJbmRlbnQgPiBub2RlSW5kZW50KSAmJiBjaCAhPT0gMCkge1xuICAgICAgcmV0dXJuIHRocm93RXJyb3Ioc3RhdGUsIFwiYmFkIGluZGVudGF0aW9uIG9mIGEgc2VxdWVuY2UgZW50cnlcIik7XG4gICAgfSBlbHNlIGlmIChzdGF0ZS5saW5lSW5kZW50IDwgbm9kZUluZGVudCkge1xuICAgICAgYnJlYWs7XG4gICAgfVxuICB9XG5cbiAgaWYgKGRldGVjdGVkKSB7XG4gICAgc3RhdGUudGFnID0gdGFnO1xuICAgIHN0YXRlLmFuY2hvciA9IGFuY2hvcjtcbiAgICBzdGF0ZS5raW5kID0gXCJzZXF1ZW5jZVwiO1xuICAgIHN0YXRlLnJlc3VsdCA9IHJlc3VsdDtcbiAgICByZXR1cm4gdHJ1ZTtcbiAgfVxuICByZXR1cm4gZmFsc2U7XG59XG5cbmZ1bmN0aW9uIHJlYWRCbG9ja01hcHBpbmcoXG4gIHN0YXRlOiBMb2FkZXJTdGF0ZSxcbiAgbm9kZUluZGVudDogbnVtYmVyLFxuICBmbG93SW5kZW50OiBudW1iZXIsXG4pOiBib29sZWFuIHtcbiAgY29uc3QgdGFnID0gc3RhdGUudGFnLFxuICAgIGFuY2hvciA9IHN0YXRlLmFuY2hvcixcbiAgICByZXN1bHQgPSB7fSxcbiAgICBvdmVycmlkYWJsZUtleXMgPSB7fTtcbiAgbGV0IGZvbGxvd2luZzogbnVtYmVyLFxuICAgIGFsbG93Q29tcGFjdCA9IGZhbHNlLFxuICAgIGxpbmU6IG51bWJlcixcbiAgICBwb3M6IG51bWJlcixcbiAgICBrZXlUYWcgPSBudWxsLFxuICAgIGtleU5vZGUgPSBudWxsLFxuICAgIHZhbHVlTm9kZSA9IG51bGwsXG4gICAgYXRFeHBsaWNpdEtleSA9IGZhbHNlLFxuICAgIGRldGVjdGVkID0gZmFsc2UsXG4gICAgY2g6IG51bWJlcjtcblxuICBpZiAoXG4gICAgc3RhdGUuYW5jaG9yICE9PSBudWxsICYmXG4gICAgdHlwZW9mIHN0YXRlLmFuY2hvciAhPT0gXCJ1bmRlZmluZWRcIiAmJlxuICAgIHR5cGVvZiBzdGF0ZS5hbmNob3JNYXAgIT09IFwidW5kZWZpbmVkXCJcbiAgKSB7XG4gICAgc3RhdGUuYW5jaG9yTWFwW3N0YXRlLmFuY2hvcl0gPSByZXN1bHQ7XG4gIH1cblxuICBjaCA9IHN0YXRlLmlucHV0LmNoYXJDb2RlQXQoc3RhdGUucG9zaXRpb24pO1xuXG4gIHdoaWxlIChjaCAhPT0gMCkge1xuICAgIGZvbGxvd2luZyA9IHN0YXRlLmlucHV0LmNoYXJDb2RlQXQoc3RhdGUucG9zaXRpb24gKyAxKTtcbiAgICBsaW5lID0gc3RhdGUubGluZTsgLy8gU2F2ZSB0aGUgY3VycmVudCBsaW5lLlxuICAgIHBvcyA9IHN0YXRlLnBvc2l0aW9uO1xuXG4gICAgLy9cbiAgICAvLyBFeHBsaWNpdCBub3RhdGlvbiBjYXNlLiBUaGVyZSBhcmUgdHdvIHNlcGFyYXRlIGJsb2NrczpcbiAgICAvLyBmaXJzdCBmb3IgdGhlIGtleSAoZGVub3RlZCBieSBcIj9cIikgYW5kIHNlY29uZCBmb3IgdGhlIHZhbHVlIChkZW5vdGVkIGJ5IFwiOlwiKVxuICAgIC8vXG4gICAgaWYgKChjaCA9PT0gMHgzZiB8fCAvKiA/ICovIGNoID09PSAweDNhKSAmJiAvKiA6ICovIGlzV3NPckVvbChmb2xsb3dpbmcpKSB7XG4gICAgICBpZiAoY2ggPT09IDB4M2YgLyogPyAqLykge1xuICAgICAgICBpZiAoYXRFeHBsaWNpdEtleSkge1xuICAgICAgICAgIHN0b3JlTWFwcGluZ1BhaXIoXG4gICAgICAgICAgICBzdGF0ZSxcbiAgICAgICAgICAgIHJlc3VsdCxcbiAgICAgICAgICAgIG92ZXJyaWRhYmxlS2V5cyxcbiAgICAgICAgICAgIGtleVRhZyBhcyBzdHJpbmcsXG4gICAgICAgICAgICBrZXlOb2RlLFxuICAgICAgICAgICAgbnVsbCxcbiAgICAgICAgICApO1xuICAgICAgICAgIGtleVRhZyA9IGtleU5vZGUgPSB2YWx1ZU5vZGUgPSBudWxsO1xuICAgICAgICB9XG5cbiAgICAgICAgZGV0ZWN0ZWQgPSB0cnVlO1xuICAgICAgICBhdEV4cGxpY2l0S2V5ID0gdHJ1ZTtcbiAgICAgICAgYWxsb3dDb21wYWN0ID0gdHJ1ZTtcbiAgICAgIH0gZWxzZSBpZiAoYXRFeHBsaWNpdEtleSkge1xuICAgICAgICAvLyBpLmUuIDB4M0EvKiA6ICovID09PSBjaGFyYWN0ZXIgYWZ0ZXIgdGhlIGV4cGxpY2l0IGtleS5cbiAgICAgICAgYXRFeHBsaWNpdEtleSA9IGZhbHNlO1xuICAgICAgICBhbGxvd0NvbXBhY3QgPSB0cnVlO1xuICAgICAgfSBlbHNlIHtcbiAgICAgICAgcmV0dXJuIHRocm93RXJyb3IoXG4gICAgICAgICAgc3RhdGUsXG4gICAgICAgICAgXCJpbmNvbXBsZXRlIGV4cGxpY2l0IG1hcHBpbmcgcGFpcjsgYSBrZXkgbm9kZSBpcyBtaXNzZWQ7IG9yIGZvbGxvd2VkIGJ5IGEgbm9uLXRhYnVsYXRlZCBlbXB0eSBsaW5lXCIsXG4gICAgICAgICk7XG4gICAgICB9XG5cbiAgICAgIHN0YXRlLnBvc2l0aW9uICs9IDE7XG4gICAgICBjaCA9IGZvbGxvd2luZztcblxuICAgICAgLy9cbiAgICAgIC8vIEltcGxpY2l0IG5vdGF0aW9uIGNhc2UuIEZsb3ctc3R5bGUgbm9kZSBhcyB0aGUga2V5IGZpcnN0LCB0aGVuIFwiOlwiLCBhbmQgdGhlIHZhbHVlLlxuICAgICAgLy9cbiAgICB9IGVsc2UgaWYgKGNvbXBvc2VOb2RlKHN0YXRlLCBmbG93SW5kZW50LCBDT05URVhUX0ZMT1dfT1VULCBmYWxzZSwgdHJ1ZSkpIHtcbiAgICAgIGlmIChzdGF0ZS5saW5lID09PSBsaW5lKSB7XG4gICAgICAgIGNoID0gc3RhdGUuaW5wdXQuY2hhckNvZGVBdChzdGF0ZS5wb3NpdGlvbik7XG5cbiAgICAgICAgd2hpbGUgKGlzV2hpdGVTcGFjZShjaCkpIHtcbiAgICAgICAgICBjaCA9IHN0YXRlLmlucHV0LmNoYXJDb2RlQXQoKytzdGF0ZS5wb3NpdGlvbik7XG4gICAgICAgIH1cblxuICAgICAgICBpZiAoY2ggPT09IDB4M2EgLyogOiAqLykge1xuICAgICAgICAgIGNoID0gc3RhdGUuaW5wdXQuY2hhckNvZGVBdCgrK3N0YXRlLnBvc2l0aW9uKTtcblxuICAgICAgICAgIGlmICghaXNXc09yRW9sKGNoKSkge1xuICAgICAgICAgICAgcmV0dXJuIHRocm93RXJyb3IoXG4gICAgICAgICAgICAgIHN0YXRlLFxuICAgICAgICAgICAgICBcImEgd2hpdGVzcGFjZSBjaGFyYWN0ZXIgaXMgZXhwZWN0ZWQgYWZ0ZXIgdGhlIGtleS12YWx1ZSBzZXBhcmF0b3Igd2l0aGluIGEgYmxvY2sgbWFwcGluZ1wiLFxuICAgICAgICAgICAgKTtcbiAgICAgICAgICB9XG5cbiAgICAgICAgICBpZiAoYXRFeHBsaWNpdEtleSkge1xuICAgICAgICAgICAgc3RvcmVNYXBwaW5nUGFpcihcbiAgICAgICAgICAgICAgc3RhdGUsXG4gICAgICAgICAgICAgIHJlc3VsdCxcbiAgICAgICAgICAgICAgb3ZlcnJpZGFibGVLZXlzLFxuICAgICAgICAgICAgICBrZXlUYWcgYXMgc3RyaW5nLFxuICAgICAgICAgICAgICBrZXlOb2RlLFxuICAgICAgICAgICAgICBudWxsLFxuICAgICAgICAgICAgKTtcbiAgICAgICAgICAgIGtleVRhZyA9IGtleU5vZGUgPSB2YWx1ZU5vZGUgPSBudWxsO1xuICAgICAgICAgIH1cblxuICAgICAgICAgIGRldGVjdGVkID0gdHJ1ZTtcbiAgICAgICAgICBhdEV4cGxpY2l0S2V5ID0gZmFsc2U7XG4gICAgICAgICAgYWxsb3dDb21wYWN0ID0gZmFsc2U7XG4gICAgICAgICAga2V5VGFnID0gc3RhdGUudGFnO1xuICAgICAgICAgIGtleU5vZGUgPSBzdGF0ZS5yZXN1bHQ7XG4gICAgICAgIH0gZWxzZSBpZiAoZGV0ZWN0ZWQpIHtcbiAgICAgICAgICByZXR1cm4gdGhyb3dFcnJvcihcbiAgICAgICAgICAgIHN0YXRlLFxuICAgICAgICAgICAgXCJjYW4gbm90IHJlYWQgYW4gaW1wbGljaXQgbWFwcGluZyBwYWlyOyBhIGNvbG9uIGlzIG1pc3NlZFwiLFxuICAgICAgICAgICk7XG4gICAgICAgIH0gZWxzZSB7XG4gICAgICAgICAgc3RhdGUudGFnID0gdGFnO1xuICAgICAgICAgIHN0YXRlLmFuY2hvciA9IGFuY2hvcjtcbiAgICAgICAgICByZXR1cm4gdHJ1ZTsgLy8gS2VlcCB0aGUgcmVzdWx0IG9mIGBjb21wb3NlTm9kZWAuXG4gICAgICAgIH1cbiAgICAgIH0gZWxzZSBpZiAoZGV0ZWN0ZWQpIHtcbiAgICAgICAgcmV0dXJuIHRocm93RXJyb3IoXG4gICAgICAgICAgc3RhdGUsXG4gICAgICAgICAgXCJjYW4gbm90IHJlYWQgYSBibG9jayBtYXBwaW5nIGVudHJ5OyBhIG11bHRpbGluZSBrZXkgbWF5IG5vdCBiZSBhbiBpbXBsaWNpdCBrZXlcIixcbiAgICAgICAgKTtcbiAgICAgIH0gZWxzZSB7XG4gICAgICAgIHN0YXRlLnRhZyA9IHRhZztcbiAgICAgICAgc3RhdGUuYW5jaG9yID0gYW5jaG9yO1xuICAgICAgICByZXR1cm4gdHJ1ZTsgLy8gS2VlcCB0aGUgcmVzdWx0IG9mIGBjb21wb3NlTm9kZWAuXG4gICAgICB9XG4gICAgfSBlbHNlIHtcbiAgICAgIGJyZWFrOyAvLyBSZWFkaW5nIGlzIGRvbmUuIEdvIHRvIHRoZSBlcGlsb2d1ZS5cbiAgICB9XG5cbiAgICAvL1xuICAgIC8vIENvbW1vbiByZWFkaW5nIGNvZGUgZm9yIGJvdGggZXhwbGljaXQgYW5kIGltcGxpY2l0IG5vdGF0aW9ucy5cbiAgICAvL1xuICAgIGlmIChzdGF0ZS5saW5lID09PSBsaW5lIHx8IHN0YXRlLmxpbmVJbmRlbnQgPiBub2RlSW5kZW50KSB7XG4gICAgICBpZiAoXG4gICAgICAgIGNvbXBvc2VOb2RlKHN0YXRlLCBub2RlSW5kZW50LCBDT05URVhUX0JMT0NLX09VVCwgdHJ1ZSwgYWxsb3dDb21wYWN0KVxuICAgICAgKSB7XG4gICAgICAgIGlmIChhdEV4cGxpY2l0S2V5KSB7XG4gICAgICAgICAga2V5Tm9kZSA9IHN0YXRlLnJlc3VsdDtcbiAgICAgICAgfSBlbHNlIHtcbiAgICAgICAgICB2YWx1ZU5vZGUgPSBzdGF0ZS5yZXN1bHQ7XG4gICAgICAgIH1cbiAgICAgIH1cblxuICAgICAgaWYgKCFhdEV4cGxpY2l0S2V5KSB7XG4gICAgICAgIHN0b3JlTWFwcGluZ1BhaXIoXG4gICAgICAgICAgc3RhdGUsXG4gICAgICAgICAgcmVzdWx0LFxuICAgICAgICAgIG92ZXJyaWRhYmxlS2V5cyxcbiAgICAgICAgICBrZXlUYWcgYXMgc3RyaW5nLFxuICAgICAgICAgIGtleU5vZGUsXG4gICAgICAgICAgdmFsdWVOb2RlLFxuICAgICAgICAgIGxpbmUsXG4gICAgICAgICAgcG9zLFxuICAgICAgICApO1xuICAgICAgICBrZXlUYWcgPSBrZXlOb2RlID0gdmFsdWVOb2RlID0gbnVsbDtcbiAgICAgIH1cblxuICAgICAgc2tpcFNlcGFyYXRpb25TcGFjZShzdGF0ZSwgdHJ1ZSwgLTEpO1xuICAgICAgY2ggPSBzdGF0ZS5pbnB1dC5jaGFyQ29kZUF0KHN0YXRlLnBvc2l0aW9uKTtcbiAgICB9XG5cbiAgICBpZiAoc3RhdGUubGluZUluZGVudCA+IG5vZGVJbmRlbnQgJiYgY2ggIT09IDApIHtcbiAgICAgIHJldHVybiB0aHJvd0Vycm9yKHN0YXRlLCBcImJhZCBpbmRlbnRhdGlvbiBvZiBhIG1hcHBpbmcgZW50cnlcIik7XG4gICAgfSBlbHNlIGlmIChzdGF0ZS5saW5lSW5kZW50IDwgbm9kZUluZGVudCkge1xuICAgICAgYnJlYWs7XG4gICAgfVxuICB9XG5cbiAgLy9cbiAgLy8gRXBpbG9ndWUuXG4gIC8vXG5cbiAgLy8gU3BlY2lhbCBjYXNlOiBsYXN0IG1hcHBpbmcncyBub2RlIGNvbnRhaW5zIG9ubHkgdGhlIGtleSBpbiBleHBsaWNpdCBub3RhdGlvbi5cbiAgaWYgKGF0RXhwbGljaXRLZXkpIHtcbiAgICBzdG9yZU1hcHBpbmdQYWlyKFxuICAgICAgc3RhdGUsXG4gICAgICByZXN1bHQsXG4gICAgICBvdmVycmlkYWJsZUtleXMsXG4gICAgICBrZXlUYWcgYXMgc3RyaW5nLFxuICAgICAga2V5Tm9kZSxcbiAgICAgIG51bGwsXG4gICAgKTtcbiAgfVxuXG4gIC8vIEV4cG9zZSB0aGUgcmVzdWx0aW5nIG1hcHBpbmcuXG4gIGlmIChkZXRlY3RlZCkge1xuICAgIHN0YXRlLnRhZyA9IHRhZztcbiAgICBzdGF0ZS5hbmNob3IgPSBhbmNob3I7XG4gICAgc3RhdGUua2luZCA9IFwibWFwcGluZ1wiO1xuICAgIHN0YXRlLnJlc3VsdCA9IHJlc3VsdDtcbiAgfVxuXG4gIHJldHVybiBkZXRlY3RlZDtcbn1cblxuZnVuY3Rpb24gcmVhZFRhZ1Byb3BlcnR5KHN0YXRlOiBMb2FkZXJTdGF0ZSk6IGJvb2xlYW4ge1xuICBsZXQgcG9zaXRpb246IG51bWJlcixcbiAgICBpc1ZlcmJhdGltID0gZmFsc2UsXG4gICAgaXNOYW1lZCA9IGZhbHNlLFxuICAgIHRhZ0hhbmRsZSA9IFwiXCIsXG4gICAgdGFnTmFtZTogc3RyaW5nLFxuICAgIGNoOiBudW1iZXI7XG5cbiAgY2ggPSBzdGF0ZS5pbnB1dC5jaGFyQ29kZUF0KHN0YXRlLnBvc2l0aW9uKTtcblxuICBpZiAoY2ggIT09IDB4MjEgLyogISAqLykgcmV0dXJuIGZhbHNlO1xuXG4gIGlmIChzdGF0ZS50YWcgIT09IG51bGwpIHtcbiAgICByZXR1cm4gdGhyb3dFcnJvcihzdGF0ZSwgXCJkdXBsaWNhdGlvbiBvZiBhIHRhZyBwcm9wZXJ0eVwiKTtcbiAgfVxuXG4gIGNoID0gc3RhdGUuaW5wdXQuY2hhckNvZGVBdCgrK3N0YXRlLnBvc2l0aW9uKTtcblxuICBpZiAoY2ggPT09IDB4M2MgLyogPCAqLykge1xuICAgIGlzVmVyYmF0aW0gPSB0cnVlO1xuICAgIGNoID0gc3RhdGUuaW5wdXQuY2hhckNvZGVBdCgrK3N0YXRlLnBvc2l0aW9uKTtcbiAgfSBlbHNlIGlmIChjaCA9PT0gMHgyMSAvKiAhICovKSB7XG4gICAgaXNOYW1lZCA9IHRydWU7XG4gICAgdGFnSGFuZGxlID0gXCIhIVwiO1xuICAgIGNoID0gc3RhdGUuaW5wdXQuY2hhckNvZGVBdCgrK3N0YXRlLnBvc2l0aW9uKTtcbiAgfSBlbHNlIHtcbiAgICB0YWdIYW5kbGUgPSBcIiFcIjtcbiAgfVxuXG4gIHBvc2l0aW9uID0gc3RhdGUucG9zaXRpb247XG5cbiAgaWYgKGlzVmVyYmF0aW0pIHtcbiAgICBkbyB7XG4gICAgICBjaCA9IHN0YXRlLmlucHV0LmNoYXJDb2RlQXQoKytzdGF0ZS5wb3NpdGlvbik7XG4gICAgfSB3aGlsZSAoY2ggIT09IDAgJiYgY2ggIT09IDB4M2UgLyogPiAqLyk7XG5cbiAgICBpZiAoc3RhdGUucG9zaXRpb24gPCBzdGF0ZS5sZW5ndGgpIHtcbiAgICAgIHRhZ05hbWUgPSBzdGF0ZS5pbnB1dC5zbGljZShwb3NpdGlvbiwgc3RhdGUucG9zaXRpb24pO1xuICAgICAgY2ggPSBzdGF0ZS5pbnB1dC5jaGFyQ29kZUF0KCsrc3RhdGUucG9zaXRpb24pO1xuICAgIH0gZWxzZSB7XG4gICAgICByZXR1cm4gdGhyb3dFcnJvcihcbiAgICAgICAgc3RhdGUsXG4gICAgICAgIFwidW5leHBlY3RlZCBlbmQgb2YgdGhlIHN0cmVhbSB3aXRoaW4gYSB2ZXJiYXRpbSB0YWdcIixcbiAgICAgICk7XG4gICAgfVxuICB9IGVsc2Uge1xuICAgIHdoaWxlIChjaCAhPT0gMCAmJiAhaXNXc09yRW9sKGNoKSkge1xuICAgICAgaWYgKGNoID09PSAweDIxIC8qICEgKi8pIHtcbiAgICAgICAgaWYgKCFpc05hbWVkKSB7XG4gICAgICAgICAgdGFnSGFuZGxlID0gc3RhdGUuaW5wdXQuc2xpY2UocG9zaXRpb24gLSAxLCBzdGF0ZS5wb3NpdGlvbiArIDEpO1xuXG4gICAgICAgICAgaWYgKCFQQVRURVJOX1RBR19IQU5ETEUudGVzdCh0YWdIYW5kbGUpKSB7XG4gICAgICAgICAgICByZXR1cm4gdGhyb3dFcnJvcihcbiAgICAgICAgICAgICAgc3RhdGUsXG4gICAgICAgICAgICAgIFwibmFtZWQgdGFnIGhhbmRsZSBjYW5ub3QgY29udGFpbiBzdWNoIGNoYXJhY3RlcnNcIixcbiAgICAgICAgICAgICk7XG4gICAgICAgICAgfVxuXG4gICAgICAgICAgaXNOYW1lZCA9IHRydWU7XG4gICAgICAgICAgcG9zaXRpb24gPSBzdGF0ZS5wb3NpdGlvbiArIDE7XG4gICAgICAgIH0gZWxzZSB7XG4gICAgICAgICAgcmV0dXJuIHRocm93RXJyb3IoXG4gICAgICAgICAgICBzdGF0ZSxcbiAgICAgICAgICAgIFwidGFnIHN1ZmZpeCBjYW5ub3QgY29udGFpbiBleGNsYW1hdGlvbiBtYXJrc1wiLFxuICAgICAgICAgICk7XG4gICAgICAgIH1cbiAgICAgIH1cblxuICAgICAgY2ggPSBzdGF0ZS5pbnB1dC5jaGFyQ29kZUF0KCsrc3RhdGUucG9zaXRpb24pO1xuICAgIH1cblxuICAgIHRhZ05hbWUgPSBzdGF0ZS5pbnB1dC5zbGljZShwb3NpdGlvbiwgc3RhdGUucG9zaXRpb24pO1xuXG4gICAgaWYgKFBBVFRFUk5fRkxPV19JTkRJQ0FUT1JTLnRlc3QodGFnTmFtZSkpIHtcbiAgICAgIHJldHVybiB0aHJvd0Vycm9yKFxuICAgICAgICBzdGF0ZSxcbiAgICAgICAgXCJ0YWcgc3VmZml4IGNhbm5vdCBjb250YWluIGZsb3cgaW5kaWNhdG9yIGNoYXJhY3RlcnNcIixcbiAgICAgICk7XG4gICAgfVxuICB9XG5cbiAgaWYgKHRhZ05hbWUgJiYgIVBBVFRFUk5fVEFHX1VSSS50ZXN0KHRhZ05hbWUpKSB7XG4gICAgcmV0dXJuIHRocm93RXJyb3IoXG4gICAgICBzdGF0ZSxcbiAgICAgIGB0YWcgbmFtZSBjYW5ub3QgY29udGFpbiBzdWNoIGNoYXJhY3RlcnM6ICR7dGFnTmFtZX1gLFxuICAgICk7XG4gIH1cblxuICBpZiAoaXNWZXJiYXRpbSkge1xuICAgIHN0YXRlLnRhZyA9IHRhZ05hbWU7XG4gIH0gZWxzZSBpZiAoXG4gICAgdHlwZW9mIHN0YXRlLnRhZ01hcCAhPT0gXCJ1bmRlZmluZWRcIiAmJlxuICAgIGhhc093bihzdGF0ZS50YWdNYXAsIHRhZ0hhbmRsZSlcbiAgKSB7XG4gICAgc3RhdGUudGFnID0gc3RhdGUudGFnTWFwW3RhZ0hhbmRsZV0gKyB0YWdOYW1lO1xuICB9IGVsc2UgaWYgKHRhZ0hhbmRsZSA9PT0gXCIhXCIpIHtcbiAgICBzdGF0ZS50YWcgPSBgISR7dGFnTmFtZX1gO1xuICB9IGVsc2UgaWYgKHRhZ0hhbmRsZSA9PT0gXCIhIVwiKSB7XG4gICAgc3RhdGUudGFnID0gYHRhZzp5YW1sLm9yZywyMDAyOiR7dGFnTmFtZX1gO1xuICB9IGVsc2Uge1xuICAgIHJldHVybiB0aHJvd0Vycm9yKHN0YXRlLCBgdW5kZWNsYXJlZCB0YWcgaGFuZGxlIFwiJHt0YWdIYW5kbGV9XCJgKTtcbiAgfVxuXG4gIHJldHVybiB0cnVlO1xufVxuXG5mdW5jdGlvbiByZWFkQW5jaG9yUHJvcGVydHkoc3RhdGU6IExvYWRlclN0YXRlKTogYm9vbGVhbiB7XG4gIGxldCBjaCA9IHN0YXRlLmlucHV0LmNoYXJDb2RlQXQoc3RhdGUucG9zaXRpb24pO1xuICBpZiAoY2ggIT09IDB4MjYgLyogJiAqLykgcmV0dXJuIGZhbHNlO1xuXG4gIGlmIChzdGF0ZS5hbmNob3IgIT09IG51bGwpIHtcbiAgICByZXR1cm4gdGhyb3dFcnJvcihzdGF0ZSwgXCJkdXBsaWNhdGlvbiBvZiBhbiBhbmNob3IgcHJvcGVydHlcIik7XG4gIH1cbiAgY2ggPSBzdGF0ZS5pbnB1dC5jaGFyQ29kZUF0KCsrc3RhdGUucG9zaXRpb24pO1xuXG4gIGNvbnN0IHBvc2l0aW9uID0gc3RhdGUucG9zaXRpb247XG4gIHdoaWxlIChjaCAhPT0gMCAmJiAhaXNXc09yRW9sKGNoKSAmJiAhaXNGbG93SW5kaWNhdG9yKGNoKSkge1xuICAgIGNoID0gc3RhdGUuaW5wdXQuY2hhckNvZGVBdCgrK3N0YXRlLnBvc2l0aW9uKTtcbiAgfVxuXG4gIGlmIChzdGF0ZS5wb3NpdGlvbiA9PT0gcG9zaXRpb24pIHtcbiAgICByZXR1cm4gdGhyb3dFcnJvcihcbiAgICAgIHN0YXRlLFxuICAgICAgXCJuYW1lIG9mIGFuIGFuY2hvciBub2RlIG11c3QgY29udGFpbiBhdCBsZWFzdCBvbmUgY2hhcmFjdGVyXCIsXG4gICAgKTtcbiAgfVxuXG4gIHN0YXRlLmFuY2hvciA9IHN0YXRlLmlucHV0LnNsaWNlKHBvc2l0aW9uLCBzdGF0ZS5wb3NpdGlvbik7XG4gIHJldHVybiB0cnVlO1xufVxuXG5mdW5jdGlvbiByZWFkQWxpYXMoc3RhdGU6IExvYWRlclN0YXRlKTogYm9vbGVhbiB7XG4gIGxldCBjaCA9IHN0YXRlLmlucHV0LmNoYXJDb2RlQXQoc3RhdGUucG9zaXRpb24pO1xuXG4gIGlmIChjaCAhPT0gMHgyYSAvKiAqICovKSByZXR1cm4gZmFsc2U7XG5cbiAgY2ggPSBzdGF0ZS5pbnB1dC5jaGFyQ29kZUF0KCsrc3RhdGUucG9zaXRpb24pO1xuICBjb25zdCBfcG9zaXRpb24gPSBzdGF0ZS5wb3NpdGlvbjtcblxuICB3aGlsZSAoY2ggIT09IDAgJiYgIWlzV3NPckVvbChjaCkgJiYgIWlzRmxvd0luZGljYXRvcihjaCkpIHtcbiAgICBjaCA9IHN0YXRlLmlucHV0LmNoYXJDb2RlQXQoKytzdGF0ZS5wb3NpdGlvbik7XG4gIH1cblxuICBpZiAoc3RhdGUucG9zaXRpb24gPT09IF9wb3NpdGlvbikge1xuICAgIHJldHVybiB0aHJvd0Vycm9yKFxuICAgICAgc3RhdGUsXG4gICAgICBcIm5hbWUgb2YgYW4gYWxpYXMgbm9kZSBtdXN0IGNvbnRhaW4gYXQgbGVhc3Qgb25lIGNoYXJhY3RlclwiLFxuICAgICk7XG4gIH1cblxuICBjb25zdCBhbGlhcyA9IHN0YXRlLmlucHV0LnNsaWNlKF9wb3NpdGlvbiwgc3RhdGUucG9zaXRpb24pO1xuICBpZiAoXG4gICAgdHlwZW9mIHN0YXRlLmFuY2hvck1hcCAhPT0gXCJ1bmRlZmluZWRcIiAmJlxuICAgICFoYXNPd24oc3RhdGUuYW5jaG9yTWFwLCBhbGlhcylcbiAgKSB7XG4gICAgcmV0dXJuIHRocm93RXJyb3Ioc3RhdGUsIGB1bmlkZW50aWZpZWQgYWxpYXMgXCIke2FsaWFzfVwiYCk7XG4gIH1cblxuICBpZiAodHlwZW9mIHN0YXRlLmFuY2hvck1hcCAhPT0gXCJ1bmRlZmluZWRcIikge1xuICAgIHN0YXRlLnJlc3VsdCA9IHN0YXRlLmFuY2hvck1hcFthbGlhc107XG4gIH1cbiAgc2tpcFNlcGFyYXRpb25TcGFjZShzdGF0ZSwgdHJ1ZSwgLTEpO1xuICByZXR1cm4gdHJ1ZTtcbn1cblxuZnVuY3Rpb24gY29tcG9zZU5vZGUoXG4gIHN0YXRlOiBMb2FkZXJTdGF0ZSxcbiAgcGFyZW50SW5kZW50OiBudW1iZXIsXG4gIG5vZGVDb250ZXh0OiBudW1iZXIsXG4gIGFsbG93VG9TZWVrOiBib29sZWFuLFxuICBhbGxvd0NvbXBhY3Q6IGJvb2xlYW4sXG4pOiBib29sZWFuIHtcbiAgbGV0IGFsbG93QmxvY2tTY2FsYXJzOiBib29sZWFuLFxuICAgIGFsbG93QmxvY2tDb2xsZWN0aW9uczogYm9vbGVhbixcbiAgICBpbmRlbnRTdGF0dXMgPSAxLCAvLyAxOiB0aGlzPnBhcmVudCwgMDogdGhpcz1wYXJlbnQsIC0xOiB0aGlzPHBhcmVudFxuICAgIGF0TmV3TGluZSA9IGZhbHNlLFxuICAgIGhhc0NvbnRlbnQgPSBmYWxzZSxcbiAgICB0eXBlOiBUeXBlLFxuICAgIGZsb3dJbmRlbnQ6IG51bWJlcixcbiAgICBibG9ja0luZGVudDogbnVtYmVyO1xuXG4gIGlmIChzdGF0ZS5saXN0ZW5lciAmJiBzdGF0ZS5saXN0ZW5lciAhPT0gbnVsbCkge1xuICAgIHN0YXRlLmxpc3RlbmVyKFwib3BlblwiLCBzdGF0ZSk7XG4gIH1cblxuICBzdGF0ZS50YWcgPSBudWxsO1xuICBzdGF0ZS5hbmNob3IgPSBudWxsO1xuICBzdGF0ZS5raW5kID0gbnVsbDtcbiAgc3RhdGUucmVzdWx0ID0gbnVsbDtcblxuICBjb25zdCBhbGxvd0Jsb2NrU3R5bGVzID0gKGFsbG93QmxvY2tTY2FsYXJzID1cbiAgICBhbGxvd0Jsb2NrQ29sbGVjdGlvbnMgPVxuICAgICAgQ09OVEVYVF9CTE9DS19PVVQgPT09IG5vZGVDb250ZXh0IHx8IENPTlRFWFRfQkxPQ0tfSU4gPT09IG5vZGVDb250ZXh0KTtcblxuICBpZiAoYWxsb3dUb1NlZWspIHtcbiAgICBpZiAoc2tpcFNlcGFyYXRpb25TcGFjZShzdGF0ZSwgdHJ1ZSwgLTEpKSB7XG4gICAgICBhdE5ld0xpbmUgPSB0cnVlO1xuXG4gICAgICBpZiAoc3RhdGUubGluZUluZGVudCA+IHBhcmVudEluZGVudCkge1xuICAgICAgICBpbmRlbnRTdGF0dXMgPSAxO1xuICAgICAgfSBlbHNlIGlmIChzdGF0ZS5saW5lSW5kZW50ID09PSBwYXJlbnRJbmRlbnQpIHtcbiAgICAgICAgaW5kZW50U3RhdHVzID0gMDtcbiAgICAgIH0gZWxzZSBpZiAoc3RhdGUubGluZUluZGVudCA8IHBhcmVudEluZGVudCkge1xuICAgICAgICBpbmRlbnRTdGF0dXMgPSAtMTtcbiAgICAgIH1cbiAgICB9XG4gIH1cblxuICBpZiAoaW5kZW50U3RhdHVzID09PSAxKSB7XG4gICAgd2hpbGUgKHJlYWRUYWdQcm9wZXJ0eShzdGF0ZSkgfHwgcmVhZEFuY2hvclByb3BlcnR5KHN0YXRlKSkge1xuICAgICAgaWYgKHNraXBTZXBhcmF0aW9uU3BhY2Uoc3RhdGUsIHRydWUsIC0xKSkge1xuICAgICAgICBhdE5ld0xpbmUgPSB0cnVlO1xuICAgICAgICBhbGxvd0Jsb2NrQ29sbGVjdGlvbnMgPSBhbGxvd0Jsb2NrU3R5bGVzO1xuXG4gICAgICAgIGlmIChzdGF0ZS5saW5lSW5kZW50ID4gcGFyZW50SW5kZW50KSB7XG4gICAgICAgICAgaW5kZW50U3RhdHVzID0gMTtcbiAgICAgICAgfSBlbHNlIGlmIChzdGF0ZS5saW5lSW5kZW50ID09PSBwYXJlbnRJbmRlbnQpIHtcbiAgICAgICAgICBpbmRlbnRTdGF0dXMgPSAwO1xuICAgICAgICB9IGVsc2UgaWYgKHN0YXRlLmxpbmVJbmRlbnQgPCBwYXJlbnRJbmRlbnQpIHtcbiAgICAgICAgICBpbmRlbnRTdGF0dXMgPSAtMTtcbiAgICAgICAgfVxuICAgICAgfSBlbHNlIHtcbiAgICAgICAgYWxsb3dCbG9ja0NvbGxlY3Rpb25zID0gZmFsc2U7XG4gICAgICB9XG4gICAgfVxuICB9XG5cbiAgaWYgKGFsbG93QmxvY2tDb2xsZWN0aW9ucykge1xuICAgIGFsbG93QmxvY2tDb2xsZWN0aW9ucyA9IGF0TmV3TGluZSB8fCBhbGxvd0NvbXBhY3Q7XG4gIH1cblxuICBpZiAoaW5kZW50U3RhdHVzID09PSAxIHx8IENPTlRFWFRfQkxPQ0tfT1VUID09PSBub2RlQ29udGV4dCkge1xuICAgIGNvbnN0IGNvbmQgPSBDT05URVhUX0ZMT1dfSU4gPT09IG5vZGVDb250ZXh0IHx8XG4gICAgICBDT05URVhUX0ZMT1dfT1VUID09PSBub2RlQ29udGV4dDtcbiAgICBmbG93SW5kZW50ID0gY29uZCA/IHBhcmVudEluZGVudCA6IHBhcmVudEluZGVudCArIDE7XG5cbiAgICBibG9ja0luZGVudCA9IHN0YXRlLnBvc2l0aW9uIC0gc3RhdGUubGluZVN0YXJ0O1xuXG4gICAgaWYgKGluZGVudFN0YXR1cyA9PT0gMSkge1xuICAgICAgaWYgKFxuICAgICAgICAoYWxsb3dCbG9ja0NvbGxlY3Rpb25zICYmXG4gICAgICAgICAgKHJlYWRCbG9ja1NlcXVlbmNlKHN0YXRlLCBibG9ja0luZGVudCkgfHxcbiAgICAgICAgICAgIHJlYWRCbG9ja01hcHBpbmcoc3RhdGUsIGJsb2NrSW5kZW50LCBmbG93SW5kZW50KSkpIHx8XG4gICAgICAgIHJlYWRGbG93Q29sbGVjdGlvbihzdGF0ZSwgZmxvd0luZGVudClcbiAgICAgICkge1xuICAgICAgICBoYXNDb250ZW50ID0gdHJ1ZTtcbiAgICAgIH0gZWxzZSB7XG4gICAgICAgIGlmIChcbiAgICAgICAgICAoYWxsb3dCbG9ja1NjYWxhcnMgJiYgcmVhZEJsb2NrU2NhbGFyKHN0YXRlLCBmbG93SW5kZW50KSkgfHxcbiAgICAgICAgICByZWFkU2luZ2xlUXVvdGVkU2NhbGFyKHN0YXRlLCBmbG93SW5kZW50KSB8fFxuICAgICAgICAgIHJlYWREb3VibGVRdW90ZWRTY2FsYXIoc3RhdGUsIGZsb3dJbmRlbnQpXG4gICAgICAgICkge1xuICAgICAgICAgIGhhc0NvbnRlbnQgPSB0cnVlO1xuICAgICAgICB9IGVsc2UgaWYgKHJlYWRBbGlhcyhzdGF0ZSkpIHtcbiAgICAgICAgICBoYXNDb250ZW50ID0gdHJ1ZTtcblxuICAgICAgICAgIGlmIChzdGF0ZS50YWcgIT09IG51bGwgfHwgc3RhdGUuYW5jaG9yICE9PSBudWxsKSB7XG4gICAgICAgICAgICByZXR1cm4gdGhyb3dFcnJvcihcbiAgICAgICAgICAgICAgc3RhdGUsXG4gICAgICAgICAgICAgIFwiYWxpYXMgbm9kZSBzaG91bGQgbm90IGhhdmUgQW55IHByb3BlcnRpZXNcIixcbiAgICAgICAgICAgICk7XG4gICAgICAgICAgfVxuICAgICAgICB9IGVsc2UgaWYgKFxuICAgICAgICAgIHJlYWRQbGFpblNjYWxhcihzdGF0ZSwgZmxvd0luZGVudCwgQ09OVEVYVF9GTE9XX0lOID09PSBub2RlQ29udGV4dClcbiAgICAgICAgKSB7XG4gICAgICAgICAgaGFzQ29udGVudCA9IHRydWU7XG5cbiAgICAgICAgICBpZiAoc3RhdGUudGFnID09PSBudWxsKSB7XG4gICAgICAgICAgICBzdGF0ZS50YWcgPSBcIj9cIjtcbiAgICAgICAgICB9XG4gICAgICAgIH1cblxuICAgICAgICBpZiAoc3RhdGUuYW5jaG9yICE9PSBudWxsICYmIHR5cGVvZiBzdGF0ZS5hbmNob3JNYXAgIT09IFwidW5kZWZpbmVkXCIpIHtcbiAgICAgICAgICBzdGF0ZS5hbmNob3JNYXBbc3RhdGUuYW5jaG9yXSA9IHN0YXRlLnJlc3VsdDtcbiAgICAgICAgfVxuICAgICAgfVxuICAgIH0gZWxzZSBpZiAoaW5kZW50U3RhdHVzID09PSAwKSB7XG4gICAgICAvLyBTcGVjaWFsIGNhc2U6IGJsb2NrIHNlcXVlbmNlcyBhcmUgYWxsb3dlZCB0byBoYXZlIHNhbWUgaW5kZW50YXRpb24gbGV2ZWwgYXMgdGhlIHBhcmVudC5cbiAgICAgIC8vIGh0dHA6Ly93d3cueWFtbC5vcmcvc3BlYy8xLjIvc3BlYy5odG1sI2lkMjc5OTc4NFxuICAgICAgaGFzQ29udGVudCA9IGFsbG93QmxvY2tDb2xsZWN0aW9ucyAmJlxuICAgICAgICByZWFkQmxvY2tTZXF1ZW5jZShzdGF0ZSwgYmxvY2tJbmRlbnQpO1xuICAgIH1cbiAgfVxuXG4gIGlmIChzdGF0ZS50YWcgIT09IG51bGwgJiYgc3RhdGUudGFnICE9PSBcIiFcIikge1xuICAgIGlmIChzdGF0ZS50YWcgPT09IFwiP1wiKSB7XG4gICAgICBmb3IgKFxuICAgICAgICBsZXQgdHlwZUluZGV4ID0gMCwgdHlwZVF1YW50aXR5ID0gc3RhdGUuaW1wbGljaXRUeXBlcy5sZW5ndGg7XG4gICAgICAgIHR5cGVJbmRleCA8IHR5cGVRdWFudGl0eTtcbiAgICAgICAgdHlwZUluZGV4KytcbiAgICAgICkge1xuICAgICAgICB0eXBlID0gc3RhdGUuaW1wbGljaXRUeXBlc1t0eXBlSW5kZXhdO1xuXG4gICAgICAgIC8vIEltcGxpY2l0IHJlc29sdmluZyBpcyBub3QgYWxsb3dlZCBmb3Igbm9uLXNjYWxhciB0eXBlcywgYW5kICc/J1xuICAgICAgICAvLyBub24tc3BlY2lmaWMgdGFnIGlzIG9ubHkgYXNzaWduZWQgdG8gcGxhaW4gc2NhbGFycy4gU28sIGl0IGlzbid0XG4gICAgICAgIC8vIG5lZWRlZCB0byBjaGVjayBmb3IgJ2tpbmQnIGNvbmZvcm1pdHkuXG5cbiAgICAgICAgaWYgKHR5cGUucmVzb2x2ZShzdGF0ZS5yZXN1bHQpKSB7XG4gICAgICAgICAgLy8gYHN0YXRlLnJlc3VsdGAgdXBkYXRlZCBpbiByZXNvbHZlciBpZiBtYXRjaGVkXG4gICAgICAgICAgc3RhdGUucmVzdWx0ID0gdHlwZS5jb25zdHJ1Y3Qoc3RhdGUucmVzdWx0KTtcbiAgICAgICAgICBzdGF0ZS50YWcgPSB0eXBlLnRhZztcbiAgICAgICAgICBpZiAoc3RhdGUuYW5jaG9yICE9PSBudWxsICYmIHR5cGVvZiBzdGF0ZS5hbmNob3JNYXAgIT09IFwidW5kZWZpbmVkXCIpIHtcbiAgICAgICAgICAgIHN0YXRlLmFuY2hvck1hcFtzdGF0ZS5hbmNob3JdID0gc3RhdGUucmVzdWx0O1xuICAgICAgICAgIH1cbiAgICAgICAgICBicmVhaztcbiAgICAgICAgfVxuICAgICAgfVxuICAgIH0gZWxzZSBpZiAoXG4gICAgICBoYXNPd24oc3RhdGUudHlwZU1hcFtzdGF0ZS5raW5kIHx8IFwiZmFsbGJhY2tcIl0sIHN0YXRlLnRhZylcbiAgICApIHtcbiAgICAgIHR5cGUgPSBzdGF0ZS50eXBlTWFwW3N0YXRlLmtpbmQgfHwgXCJmYWxsYmFja1wiXVtzdGF0ZS50YWddO1xuXG4gICAgICBpZiAoc3RhdGUucmVzdWx0ICE9PSBudWxsICYmIHR5cGUua2luZCAhPT0gc3RhdGUua2luZCkge1xuICAgICAgICByZXR1cm4gdGhyb3dFcnJvcihcbiAgICAgICAgICBzdGF0ZSxcbiAgICAgICAgICBgdW5hY2NlcHRhYmxlIG5vZGUga2luZCBmb3IgITwke3N0YXRlLnRhZ30+IHRhZzsgaXQgc2hvdWxkIGJlIFwiJHt0eXBlLmtpbmR9XCIsIG5vdCBcIiR7c3RhdGUua2luZH1cImAsXG4gICAgICAgICk7XG4gICAgICB9XG5cbiAgICAgIGlmICghdHlwZS5yZXNvbHZlKHN0YXRlLnJlc3VsdCkpIHtcbiAgICAgICAgLy8gYHN0YXRlLnJlc3VsdGAgdXBkYXRlZCBpbiByZXNvbHZlciBpZiBtYXRjaGVkXG4gICAgICAgIHJldHVybiB0aHJvd0Vycm9yKFxuICAgICAgICAgIHN0YXRlLFxuICAgICAgICAgIGBjYW5ub3QgcmVzb2x2ZSBhIG5vZGUgd2l0aCAhPCR7c3RhdGUudGFnfT4gZXhwbGljaXQgdGFnYCxcbiAgICAgICAgKTtcbiAgICAgIH0gZWxzZSB7XG4gICAgICAgIHN0YXRlLnJlc3VsdCA9IHR5cGUuY29uc3RydWN0KHN0YXRlLnJlc3VsdCk7XG4gICAgICAgIGlmIChzdGF0ZS5hbmNob3IgIT09IG51bGwgJiYgdHlwZW9mIHN0YXRlLmFuY2hvck1hcCAhPT0gXCJ1bmRlZmluZWRcIikge1xuICAgICAgICAgIHN0YXRlLmFuY2hvck1hcFtzdGF0ZS5hbmNob3JdID0gc3RhdGUucmVzdWx0O1xuICAgICAgICB9XG4gICAgICB9XG4gICAgfSBlbHNlIHtcbiAgICAgIHJldHVybiB0aHJvd0Vycm9yKHN0YXRlLCBgdW5rbm93biB0YWcgITwke3N0YXRlLnRhZ30+YCk7XG4gICAgfVxuICB9XG5cbiAgaWYgKHN0YXRlLmxpc3RlbmVyICYmIHN0YXRlLmxpc3RlbmVyICE9PSBudWxsKSB7XG4gICAgc3RhdGUubGlzdGVuZXIoXCJjbG9zZVwiLCBzdGF0ZSk7XG4gIH1cbiAgcmV0dXJuIHN0YXRlLnRhZyAhPT0gbnVsbCB8fCBzdGF0ZS5hbmNob3IgIT09IG51bGwgfHwgaGFzQ29udGVudDtcbn1cblxuZnVuY3Rpb24gcmVhZERvY3VtZW50KHN0YXRlOiBMb2FkZXJTdGF0ZSkge1xuICBjb25zdCBkb2N1bWVudFN0YXJ0ID0gc3RhdGUucG9zaXRpb247XG4gIGxldCBwb3NpdGlvbjogbnVtYmVyLFxuICAgIGRpcmVjdGl2ZU5hbWU6IHN0cmluZyxcbiAgICBkaXJlY3RpdmVBcmdzOiBzdHJpbmdbXSxcbiAgICBoYXNEaXJlY3RpdmVzID0gZmFsc2UsXG4gICAgY2g6IG51bWJlcjtcblxuICBzdGF0ZS52ZXJzaW9uID0gbnVsbDtcbiAgc3RhdGUuY2hlY2tMaW5lQnJlYWtzID0gc3RhdGUubGVnYWN5O1xuICBzdGF0ZS50YWdNYXAgPSB7fTtcbiAgc3RhdGUuYW5jaG9yTWFwID0ge307XG5cbiAgd2hpbGUgKChjaCA9IHN0YXRlLmlucHV0LmNoYXJDb2RlQXQoc3RhdGUucG9zaXRpb24pKSAhPT0gMCkge1xuICAgIHNraXBTZXBhcmF0aW9uU3BhY2Uoc3RhdGUsIHRydWUsIC0xKTtcblxuICAgIGNoID0gc3RhdGUuaW5wdXQuY2hhckNvZGVBdChzdGF0ZS5wb3NpdGlvbik7XG5cbiAgICBpZiAoc3RhdGUubGluZUluZGVudCA+IDAgfHwgY2ggIT09IDB4MjUgLyogJSAqLykge1xuICAgICAgYnJlYWs7XG4gICAgfVxuXG4gICAgaGFzRGlyZWN0aXZlcyA9IHRydWU7XG4gICAgY2ggPSBzdGF0ZS5pbnB1dC5jaGFyQ29kZUF0KCsrc3RhdGUucG9zaXRpb24pO1xuICAgIHBvc2l0aW9uID0gc3RhdGUucG9zaXRpb247XG5cbiAgICB3aGlsZSAoY2ggIT09IDAgJiYgIWlzV3NPckVvbChjaCkpIHtcbiAgICAgIGNoID0gc3RhdGUuaW5wdXQuY2hhckNvZGVBdCgrK3N0YXRlLnBvc2l0aW9uKTtcbiAgICB9XG5cbiAgICBkaXJlY3RpdmVOYW1lID0gc3RhdGUuaW5wdXQuc2xpY2UocG9zaXRpb24sIHN0YXRlLnBvc2l0aW9uKTtcbiAgICBkaXJlY3RpdmVBcmdzID0gW107XG5cbiAgICBpZiAoZGlyZWN0aXZlTmFtZS5sZW5ndGggPCAxKSB7XG4gICAgICByZXR1cm4gdGhyb3dFcnJvcihcbiAgICAgICAgc3RhdGUsXG4gICAgICAgIFwiZGlyZWN0aXZlIG5hbWUgbXVzdCBub3QgYmUgbGVzcyB0aGFuIG9uZSBjaGFyYWN0ZXIgaW4gbGVuZ3RoXCIsXG4gICAgICApO1xuICAgIH1cblxuICAgIHdoaWxlIChjaCAhPT0gMCkge1xuICAgICAgd2hpbGUgKGlzV2hpdGVTcGFjZShjaCkpIHtcbiAgICAgICAgY2ggPSBzdGF0ZS5pbnB1dC5jaGFyQ29kZUF0KCsrc3RhdGUucG9zaXRpb24pO1xuICAgICAgfVxuXG4gICAgICBpZiAoY2ggPT09IDB4MjMgLyogIyAqLykge1xuICAgICAgICBkbyB7XG4gICAgICAgICAgY2ggPSBzdGF0ZS5pbnB1dC5jaGFyQ29kZUF0KCsrc3RhdGUucG9zaXRpb24pO1xuICAgICAgICB9IHdoaWxlIChjaCAhPT0gMCAmJiAhaXNFT0woY2gpKTtcbiAgICAgICAgYnJlYWs7XG4gICAgICB9XG5cbiAgICAgIGlmIChpc0VPTChjaCkpIGJyZWFrO1xuXG4gICAgICBwb3NpdGlvbiA9IHN0YXRlLnBvc2l0aW9uO1xuXG4gICAgICB3aGlsZSAoY2ggIT09IDAgJiYgIWlzV3NPckVvbChjaCkpIHtcbiAgICAgICAgY2ggPSBzdGF0ZS5pbnB1dC5jaGFyQ29kZUF0KCsrc3RhdGUucG9zaXRpb24pO1xuICAgICAgfVxuXG4gICAgICBkaXJlY3RpdmVBcmdzLnB1c2goc3RhdGUuaW5wdXQuc2xpY2UocG9zaXRpb24sIHN0YXRlLnBvc2l0aW9uKSk7XG4gICAgfVxuXG4gICAgaWYgKGNoICE9PSAwKSByZWFkTGluZUJyZWFrKHN0YXRlKTtcblxuICAgIGlmIChoYXNPd24oZGlyZWN0aXZlSGFuZGxlcnMsIGRpcmVjdGl2ZU5hbWUpKSB7XG4gICAgICBkaXJlY3RpdmVIYW5kbGVyc1tkaXJlY3RpdmVOYW1lXShzdGF0ZSwgZGlyZWN0aXZlTmFtZSwgLi4uZGlyZWN0aXZlQXJncyk7XG4gICAgfSBlbHNlIHtcbiAgICAgIHRocm93V2FybmluZyhzdGF0ZSwgYHVua25vd24gZG9jdW1lbnQgZGlyZWN0aXZlIFwiJHtkaXJlY3RpdmVOYW1lfVwiYCk7XG4gICAgfVxuICB9XG5cbiAgc2tpcFNlcGFyYXRpb25TcGFjZShzdGF0ZSwgdHJ1ZSwgLTEpO1xuXG4gIGlmIChcbiAgICBzdGF0ZS5saW5lSW5kZW50ID09PSAwICYmXG4gICAgc3RhdGUuaW5wdXQuY2hhckNvZGVBdChzdGF0ZS5wb3NpdGlvbikgPT09IDB4MmQgLyogLSAqLyAmJlxuICAgIHN0YXRlLmlucHV0LmNoYXJDb2RlQXQoc3RhdGUucG9zaXRpb24gKyAxKSA9PT0gMHgyZCAvKiAtICovICYmXG4gICAgc3RhdGUuaW5wdXQuY2hhckNvZGVBdChzdGF0ZS5wb3NpdGlvbiArIDIpID09PSAweDJkIC8qIC0gKi9cbiAgKSB7XG4gICAgc3RhdGUucG9zaXRpb24gKz0gMztcbiAgICBza2lwU2VwYXJhdGlvblNwYWNlKHN0YXRlLCB0cnVlLCAtMSk7XG4gIH0gZWxzZSBpZiAoaGFzRGlyZWN0aXZlcykge1xuICAgIHJldHVybiB0aHJvd0Vycm9yKHN0YXRlLCBcImRpcmVjdGl2ZXMgZW5kIG1hcmsgaXMgZXhwZWN0ZWRcIik7XG4gIH1cblxuICBjb21wb3NlTm9kZShzdGF0ZSwgc3RhdGUubGluZUluZGVudCAtIDEsIENPTlRFWFRfQkxPQ0tfT1VULCBmYWxzZSwgdHJ1ZSk7XG4gIHNraXBTZXBhcmF0aW9uU3BhY2Uoc3RhdGUsIHRydWUsIC0xKTtcblxuICBpZiAoXG4gICAgc3RhdGUuY2hlY2tMaW5lQnJlYWtzICYmXG4gICAgUEFUVEVSTl9OT05fQVNDSUlfTElORV9CUkVBS1MudGVzdChcbiAgICAgIHN0YXRlLmlucHV0LnNsaWNlKGRvY3VtZW50U3RhcnQsIHN0YXRlLnBvc2l0aW9uKSxcbiAgICApXG4gICkge1xuICAgIHRocm93V2FybmluZyhzdGF0ZSwgXCJub24tQVNDSUkgbGluZSBicmVha3MgYXJlIGludGVycHJldGVkIGFzIGNvbnRlbnRcIik7XG4gIH1cblxuICBzdGF0ZS5kb2N1bWVudHMucHVzaChzdGF0ZS5yZXN1bHQpO1xuXG4gIGlmIChzdGF0ZS5wb3NpdGlvbiA9PT0gc3RhdGUubGluZVN0YXJ0ICYmIHRlc3REb2N1bWVudFNlcGFyYXRvcihzdGF0ZSkpIHtcbiAgICBpZiAoc3RhdGUuaW5wdXQuY2hhckNvZGVBdChzdGF0ZS5wb3NpdGlvbikgPT09IDB4MmUgLyogLiAqLykge1xuICAgICAgc3RhdGUucG9zaXRpb24gKz0gMztcbiAgICAgIHNraXBTZXBhcmF0aW9uU3BhY2Uoc3RhdGUsIHRydWUsIC0xKTtcbiAgICB9XG4gICAgcmV0dXJuO1xuICB9XG5cbiAgaWYgKHN0YXRlLnBvc2l0aW9uIDwgc3RhdGUubGVuZ3RoIC0gMSkge1xuICAgIHJldHVybiB0aHJvd0Vycm9yKFxuICAgICAgc3RhdGUsXG4gICAgICBcImVuZCBvZiB0aGUgc3RyZWFtIG9yIGEgZG9jdW1lbnQgc2VwYXJhdG9yIGlzIGV4cGVjdGVkXCIsXG4gICAgKTtcbiAgfSBlbHNlIHtcbiAgICByZXR1cm47XG4gIH1cbn1cblxuZnVuY3Rpb24gbG9hZERvY3VtZW50cyhpbnB1dDogc3RyaW5nLCBvcHRpb25zPzogTG9hZGVyU3RhdGVPcHRpb25zKTogdW5rbm93bltdIHtcbiAgaW5wdXQgPSBTdHJpbmcoaW5wdXQpO1xuICBvcHRpb25zID0gb3B0aW9ucyB8fCB7fTtcblxuICBpZiAoaW5wdXQubGVuZ3RoICE9PSAwKSB7XG4gICAgLy8gQWRkIHRhaWxpbmcgYFxcbmAgaWYgbm90IGV4aXN0c1xuICAgIGlmIChcbiAgICAgIGlucHV0LmNoYXJDb2RlQXQoaW5wdXQubGVuZ3RoIC0gMSkgIT09IDB4MGEgLyogTEYgKi8gJiZcbiAgICAgIGlucHV0LmNoYXJDb2RlQXQoaW5wdXQubGVuZ3RoIC0gMSkgIT09IDB4MGQgLyogQ1IgKi9cbiAgICApIHtcbiAgICAgIGlucHV0ICs9IFwiXFxuXCI7XG4gICAgfVxuXG4gICAgLy8gU3RyaXAgQk9NXG4gICAgaWYgKGlucHV0LmNoYXJDb2RlQXQoMCkgPT09IDB4ZmVmZikge1xuICAgICAgaW5wdXQgPSBpbnB1dC5zbGljZSgxKTtcbiAgICB9XG4gIH1cblxuICBjb25zdCBzdGF0ZSA9IG5ldyBMb2FkZXJTdGF0ZShpbnB1dCwgb3B0aW9ucyk7XG5cbiAgLy8gVXNlIDAgYXMgc3RyaW5nIHRlcm1pbmF0b3IuIFRoYXQgc2lnbmlmaWNhbnRseSBzaW1wbGlmaWVzIGJvdW5kcyBjaGVjay5cbiAgc3RhdGUuaW5wdXQgKz0gXCJcXDBcIjtcblxuICB3aGlsZSAoc3RhdGUuaW5wdXQuY2hhckNvZGVBdChzdGF0ZS5wb3NpdGlvbikgPT09IDB4MjAgLyogU3BhY2UgKi8pIHtcbiAgICBzdGF0ZS5saW5lSW5kZW50ICs9IDE7XG4gICAgc3RhdGUucG9zaXRpb24gKz0gMTtcbiAgfVxuXG4gIHdoaWxlIChzdGF0ZS5wb3NpdGlvbiA8IHN0YXRlLmxlbmd0aCAtIDEpIHtcbiAgICByZWFkRG9jdW1lbnQoc3RhdGUpO1xuICB9XG5cbiAgcmV0dXJuIHN0YXRlLmRvY3VtZW50cztcbn1cblxuZXhwb3J0IHR5cGUgQ2JGdW5jdGlvbiA9IChkb2M6IHVua25vd24pID0+IHZvaWQ7XG5mdW5jdGlvbiBpc0NiRnVuY3Rpb24oZm46IHVua25vd24pOiBmbiBpcyBDYkZ1bmN0aW9uIHtcbiAgcmV0dXJuIHR5cGVvZiBmbiA9PT0gXCJmdW5jdGlvblwiO1xufVxuXG5leHBvcnQgZnVuY3Rpb24gbG9hZEFsbDxUIGV4dGVuZHMgQ2JGdW5jdGlvbiB8IExvYWRlclN0YXRlT3B0aW9ucz4oXG4gIGlucHV0OiBzdHJpbmcsXG4gIGl0ZXJhdG9yT3JPcHRpb24/OiBULFxuICBvcHRpb25zPzogTG9hZGVyU3RhdGVPcHRpb25zLFxuKTogVCBleHRlbmRzIENiRnVuY3Rpb24gPyB2b2lkIDogdW5rbm93bltdIHtcbiAgaWYgKCFpc0NiRnVuY3Rpb24oaXRlcmF0b3JPck9wdGlvbikpIHtcbiAgICByZXR1cm4gbG9hZERvY3VtZW50cyhpbnB1dCwgaXRlcmF0b3JPck9wdGlvbiBhcyBMb2FkZXJTdGF0ZU9wdGlvbnMpIGFzIEFueTtcbiAgfVxuXG4gIGNvbnN0IGRvY3VtZW50cyA9IGxvYWREb2N1bWVudHMoaW5wdXQsIG9wdGlvbnMpO1xuICBjb25zdCBpdGVyYXRvciA9IGl0ZXJhdG9yT3JPcHRpb247XG4gIGZvciAobGV0IGluZGV4ID0gMCwgbGVuZ3RoID0gZG9jdW1lbnRzLmxlbmd0aDsgaW5kZXggPCBsZW5ndGg7IGluZGV4KyspIHtcbiAgICBpdGVyYXRvcihkb2N1bWVudHNbaW5kZXhdKTtcbiAgfVxuXG4gIHJldHVybiB2b2lkIDAgYXMgQW55O1xufVxuXG5leHBvcnQgZnVuY3Rpb24gbG9hZChpbnB1dDogc3RyaW5nLCBvcHRpb25zPzogTG9hZGVyU3RhdGVPcHRpb25zKTogdW5rbm93biB7XG4gIGNvbnN0IGRvY3VtZW50cyA9IGxvYWREb2N1bWVudHMoaW5wdXQsIG9wdGlvbnMpO1xuXG4gIGlmIChkb2N1bWVudHMubGVuZ3RoID09PSAwKSB7XG4gICAgcmV0dXJuO1xuICB9XG4gIGlmIChkb2N1bWVudHMubGVuZ3RoID09PSAxKSB7XG4gICAgcmV0dXJuIGRvY3VtZW50c1swXTtcbiAgfVxuICB0aHJvdyBuZXcgWUFNTEVycm9yKFxuICAgIFwiZXhwZWN0ZWQgYSBzaW5nbGUgZG9jdW1lbnQgaW4gdGhlIHN0cmVhbSwgYnV0IGZvdW5kIG1vcmVcIixcbiAgKTtcbn1cbiJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQSwrQkFBK0I7QUFDL0Isb0ZBQW9GO0FBQ3BGLDBFQUEwRTtBQUMxRSwwRUFBMEU7QUFFMUUsU0FBUyxTQUFTLFFBQVEsY0FBYztBQUN4QyxTQUFTLElBQUksUUFBUSxhQUFhO0FBRWxDLFlBQVksWUFBWSxjQUFjO0FBQ3RDLFNBQVMsV0FBVyxRQUF3QyxvQkFBb0I7QUFLaEYsTUFBTSxFQUFFLE1BQU0sRUFBRSxHQUFHO0FBRW5CLE1BQU0sa0JBQWtCO0FBQ3hCLE1BQU0sbUJBQW1CO0FBQ3pCLE1BQU0sbUJBQW1CO0FBQ3pCLE1BQU0sb0JBQW9CO0FBRTFCLE1BQU0sZ0JBQWdCO0FBQ3RCLE1BQU0saUJBQWlCO0FBQ3ZCLE1BQU0sZ0JBQWdCO0FBRXRCLE1BQU0sd0JBQ0osb0NBQW9DO0FBQ3BDO0FBQ0YsTUFBTSxnQ0FBZ0M7QUFDdEMsTUFBTSwwQkFBMEI7QUFDaEMsTUFBTSxxQkFBcUI7QUFDM0IsTUFBTSxrQkFDSjtBQUVGLFNBQVMsT0FBTyxHQUFZO0VBQzFCLE9BQU8sT0FBTyxTQUFTLENBQUMsUUFBUSxDQUFDLElBQUksQ0FBQztBQUN4QztBQUVBLFNBQVMsTUFBTSxDQUFTO0VBQ3RCLE9BQU8sTUFBTSxRQUFRLE1BQU0sR0FBRyxNQUFNLEtBQUssTUFBTTtBQUNqRDtBQUVBLFNBQVMsYUFBYSxDQUFTO0VBQzdCLE9BQU8sTUFBTSxRQUFRLE9BQU8sR0FBRyxNQUFNLEtBQUssU0FBUztBQUNyRDtBQUVBLFNBQVMsVUFBVSxDQUFTO0VBQzFCLE9BQ0UsTUFBTSxLQUFLLE9BQU8sT0FDbEIsTUFBTSxLQUFLLFNBQVMsT0FDcEIsTUFBTSxLQUFLLE1BQU0s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
// denoCacheMetadata=4127863751734188155,197609887046283124