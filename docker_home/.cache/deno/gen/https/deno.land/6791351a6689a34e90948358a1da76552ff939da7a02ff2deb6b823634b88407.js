// Copyright 2018-2024 the Deno authors. All rights reserved. MIT license.
// This module is browser compatible.
/** Test whether the given string is a glob */ export function isGlob(str) {
  const chars = {
    "{": "}",
    "(": ")",
    "[": "]"
  };
  const regex = /\\(.)|(^!|\*|\?|[\].+)]\?|\[[^\\\]]+\]|\{[^\\}]+\}|\(\?[:!=][^\\)]+\)|\([^|]+\|[^\\)]+\))/;
  if (str === "") {
    return false;
  }
  let match;
  while(match = regex.exec(str)){
    if (match[2]) return true;
    let idx = match.index + match[0].length;
    // if an open bracket/brace/paren is escaped,
    // set the index to the next closing character
    const open = match[1];
    const close = open ? chars[open] : null;
    if (open && close) {
      const n = str.indexOf(close, idx);
      if (n !== -1) {
        idx = n + 1;
      }
    }
    str = str.slice(idx);
  }
  return false;
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbImh0dHBzOi8vZGVuby5sYW5kL3N0ZEAwLjIxNC4wL3BhdGgvaXNfZ2xvYi50cyJdLCJzb3VyY2VzQ29udGVudCI6WyIvLyBDb3B5cmlnaHQgMjAxOC0yMDI0IHRoZSBEZW5vIGF1dGhvcnMuIEFsbCByaWdodHMgcmVzZXJ2ZWQuIE1JVCBsaWNlbnNlLlxuLy8gVGhpcyBtb2R1bGUgaXMgYnJvd3NlciBjb21wYXRpYmxlLlxuXG4vKiogVGVzdCB3aGV0aGVyIHRoZSBnaXZlbiBzdHJpbmcgaXMgYSBnbG9iICovXG5leHBvcnQgZnVuY3Rpb24gaXNHbG9iKHN0cjogc3RyaW5nKTogYm9vbGVhbiB7XG4gIGNvbnN0IGNoYXJzOiBSZWNvcmQ8c3RyaW5nLCBzdHJpbmc+ID0geyBcIntcIjogXCJ9XCIsIFwiKFwiOiBcIilcIiwgXCJbXCI6IFwiXVwiIH07XG4gIGNvbnN0IHJlZ2V4ID1cbiAgICAvXFxcXCguKXwoXiF8XFwqfFxcP3xbXFxdLispXVxcP3xcXFtbXlxcXFxcXF1dK1xcXXxcXHtbXlxcXFx9XStcXH18XFwoXFw/WzohPV1bXlxcXFwpXStcXCl8XFwoW158XStcXHxbXlxcXFwpXStcXCkpLztcblxuICBpZiAoc3RyID09PSBcIlwiKSB7XG4gICAgcmV0dXJuIGZhbHNlO1xuICB9XG5cbiAgbGV0IG1hdGNoOiBSZWdFeHBFeGVjQXJyYXkgfCBudWxsO1xuXG4gIHdoaWxlICgobWF0Y2ggPSByZWdleC5leGVjKHN0cikpKSB7XG4gICAgaWYgKG1hdGNoWzJdKSByZXR1cm4gdHJ1ZTtcbiAgICBsZXQgaWR4ID0gbWF0Y2guaW5kZXggKyBtYXRjaFswXS5sZW5ndGg7XG5cbiAgICAvLyBpZiBhbiBvcGVuIGJyYWNrZXQvYnJhY2UvcGFyZW4gaXMgZXNjYXBlZCxcbiAgICAvLyBzZXQgdGhlIGluZGV4IHRvIHRoZSBuZXh0IGNsb3NpbmcgY2hhcmFjdGVyXG4gICAgY29uc3Qgb3BlbiA9IG1hdGNoWzFdO1xuICAgIGNvbnN0IGNsb3NlID0gb3BlbiA/IGNoYXJzW29wZW5dIDogbnVsbDtcbiAgICBpZiAob3BlbiAmJiBjbG9zZSkge1xuICAgICAgY29uc3QgbiA9IHN0ci5pbmRleE9mKGNsb3NlLCBpZHgpO1xuICAgICAgaWYgKG4gIT09IC0xKSB7XG4gICAgICAgIGlkeCA9IG4gKyAxO1xuICAgICAgfVxuICAgIH1cblxuICAgIHN0ciA9IHN0ci5zbGljZShpZHgpO1xuICB9XG5cbiAgcmV0dXJuIGZhbHNlO1xufVxuIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiJBQUFBLDBFQUEwRTtBQUMxRSxxQ0FBcUM7QUFFckMsNENBQTRDLEdBQzVDLE9BQU8sU0FBUyxPQUFPLEdBQVc7RUFDaEMsTUFBTSxRQUFnQztJQUFFLEtBQUs7SUFBSyxLQUFLO0lBQUssS0FBSztFQUFJO0VBQ3JFLE1BQU0sUUFDSjtFQUVGLElBQUksUUFBUSxJQUFJO0lBQ2QsT0FBTztFQUNUO0VBRUEsSUFBSTtFQUVKLE1BQVEsUUFBUSxNQUFNLElBQUksQ0FBQyxLQUFPO0lBQ2hDLElBQUksS0FBSyxDQUFDLEVBQUUsRUFBRSxPQUFPO0lBQ3JCLElBQUksTUFBTSxNQUFNLEtBQUssR0FBRyxLQUFLLENBQUMsRUFBRSxDQUFDLE1BQU07SUFFdkMsNkNBQTZDO0lBQzdDLDhDQUE4QztJQUM5QyxNQUFNLE9BQU8sS0FBSyxDQUFDLEVBQUU7SUFDckIsTUFBTSxRQUFRLE9BQU8sS0FBSyxDQUFDLEtBQUssR0FBRztJQUNuQyxJQUFJLFFBQVEsT0FBTztNQUNqQixNQUFNLElBQUksSUFBSSxPQUFPLENBQUMsT0FBTztNQUM3QixJQUFJLE1BQU0sQ0FBQyxHQUFHO1FBQ1osTUFBTSxJQUFJO01BQ1o7SUFDRjtJQUVBLE1BQU0sSUFBSSxLQUFLLENBQUM7RUFDbEI7RUFFQSxPQUFPO0FBQ1QifQ==
// denoCacheMetadata=8291940015278131542,5007292195208714400