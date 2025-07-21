// Copyright 2018-2022 the Deno authors. All rights reserved. MIT license.
// This module is browser compatible.
/** EndOfLine character enum */ export var EOL = /*#__PURE__*/ function(EOL) {
  EOL["LF"] = "\n";
  EOL["CRLF"] = "\r\n";
  return EOL;
}({});
const regDetect = /(?:\r?\n)/g;
/**
 * Detect the EOL character for string input.
 * returns null if no newline
 */ export function detect(content) {
  const d = content.match(regDetect);
  if (!d || d.length === 0) {
    return null;
  }
  const hasCRLF = d.some((x)=>x === EOL.CRLF);
  return hasCRLF ? EOL.CRLF : EOL.LF;
}
/** Format the file to the targeted EOL */ export function format(content, eol) {
  return content.replace(regDetect, eol);
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbImh0dHBzOi8vZGVuby5sYW5kL3N0ZEAwLjEzMy4wL2ZzL2VvbC50cyJdLCJzb3VyY2VzQ29udGVudCI6WyIvLyBDb3B5cmlnaHQgMjAxOC0yMDIyIHRoZSBEZW5vIGF1dGhvcnMuIEFsbCByaWdodHMgcmVzZXJ2ZWQuIE1JVCBsaWNlbnNlLlxuLy8gVGhpcyBtb2R1bGUgaXMgYnJvd3NlciBjb21wYXRpYmxlLlxuXG4vKiogRW5kT2ZMaW5lIGNoYXJhY3RlciBlbnVtICovXG5leHBvcnQgZW51bSBFT0wge1xuICBMRiA9IFwiXFxuXCIsXG4gIENSTEYgPSBcIlxcclxcblwiLFxufVxuXG5jb25zdCByZWdEZXRlY3QgPSAvKD86XFxyP1xcbikvZztcblxuLyoqXG4gKiBEZXRlY3QgdGhlIEVPTCBjaGFyYWN0ZXIgZm9yIHN0cmluZyBpbnB1dC5cbiAqIHJldHVybnMgbnVsbCBpZiBubyBuZXdsaW5lXG4gKi9cbmV4cG9ydCBmdW5jdGlvbiBkZXRlY3QoY29udGVudDogc3RyaW5nKTogRU9MIHwgbnVsbCB7XG4gIGNvbnN0IGQgPSBjb250ZW50Lm1hdGNoKHJlZ0RldGVjdCk7XG4gIGlmICghZCB8fCBkLmxlbmd0aCA9PT0gMCkge1xuICAgIHJldHVybiBudWxsO1xuICB9XG4gIGNvbnN0IGhhc0NSTEYgPSBkLnNvbWUoKHg6IHN0cmluZyk6IGJvb2xlYW4gPT4geCA9PT0gRU9MLkNSTEYpO1xuXG4gIHJldHVybiBoYXNDUkxGID8gRU9MLkNSTEYgOiBFT0wuTEY7XG59XG5cbi8qKiBGb3JtYXQgdGhlIGZpbGUgdG8gdGhlIHRhcmdldGVkIEVPTCAqL1xuZXhwb3J0IGZ1bmN0aW9uIGZvcm1hdChjb250ZW50OiBzdHJpbmcsIGVvbDogRU9MKTogc3RyaW5nIHtcbiAgcmV0dXJuIGNvbnRlbnQucmVwbGFjZShyZWdEZXRlY3QsIGVvbCk7XG59XG4iXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IkFBQUEsMEVBQTBFO0FBQzFFLHFDQUFxQztBQUVyQyw2QkFBNkIsR0FDN0IsT0FBTyxJQUFBLEFBQUssNkJBQUE7OztTQUFBO01BR1g7QUFFRCxNQUFNLFlBQVk7QUFFbEI7OztDQUdDLEdBQ0QsT0FBTyxTQUFTLE9BQU8sT0FBZTtFQUNwQyxNQUFNLElBQUksUUFBUSxLQUFLLENBQUM7RUFDeEIsSUFBSSxDQUFDLEtBQUssRUFBRSxNQUFNLEtBQUssR0FBRztJQUN4QixPQUFPO0VBQ1Q7RUFDQSxNQUFNLFVBQVUsRUFBRSxJQUFJLENBQUMsQ0FBQyxJQUF1QixNQUFNLElBQUksSUFBSTtFQUU3RCxPQUFPLFVBQVUsSUFBSSxJQUFJLEdBQUcsSUFBSSxFQUFFO0FBQ3BDO0FBRUEsd0NBQXdDLEdBQ3hDLE9BQU8sU0FBUyxPQUFPLE9BQWUsRUFBRSxHQUFRO0VBQzlDLE9BQU8sUUFBUSxPQUFPLENBQUMsV0FBVztBQUNwQyJ9
// denoCacheMetadata=13380034026384327041,945628620740946963