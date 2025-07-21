// Ported and adapted from js-yaml-js-types v1.0.0:
// https://github.com/nodeca/js-yaml-js-types/tree/ac537e7bbdd3c2cbbd9882ca3919c520c2dc022b
// Copyright 2011-2015 by Vitaly Puzrin. All rights reserved. MIT license.
// Copyright 2018-2022 the Deno authors. All rights reserved. MIT license.
import { Type } from "../type.ts";
const REGEXP = /^\/(?<regexp>[\s\S]+)\/(?<modifiers>[gismuy]*)$/;
export const regexp = new Type("tag:yaml.org,2002:js/regexp", {
  kind: "scalar",
  resolve (data) {
    if (data === null || !data.length) {
      return false;
    }
    const regexp = `${data}`;
    if (regexp.charAt(0) === "/") {
      // Ensure regex is properly terminated
      if (!REGEXP.test(data)) {
        return false;
      }
      // Check no duplicate modifiers
      const modifiers = [
        ...regexp.match(REGEXP)?.groups?.modifiers ?? ""
      ];
      if (new Set(modifiers).size < modifiers.length) {
        return false;
      }
    }
    return true;
  },
  construct (data) {
    const { regexp = `${data}`, modifiers = "" } = `${data}`.match(REGEXP)?.groups ?? {};
    return new RegExp(regexp, modifiers);
  },
  predicate (object) {
    return object instanceof RegExp;
  },
  represent (object) {
    return object.toString();
  }
});
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbImh0dHBzOi8vZGVuby5sYW5kL3N0ZEAwLjE2OC4wL2VuY29kaW5nL195YW1sL3R5cGUvcmVnZXhwLnRzIl0sInNvdXJjZXNDb250ZW50IjpbIi8vIFBvcnRlZCBhbmQgYWRhcHRlZCBmcm9tIGpzLXlhbWwtanMtdHlwZXMgdjEuMC4wOlxuLy8gaHR0cHM6Ly9naXRodWIuY29tL25vZGVjYS9qcy15YW1sLWpzLXR5cGVzL3RyZWUvYWM1MzdlN2JiZGQzYzJjYmJkOTg4MmNhMzkxOWM1MjBjMmRjMDIyYlxuLy8gQ29weXJpZ2h0IDIwMTEtMjAxNSBieSBWaXRhbHkgUHV6cmluLiBBbGwgcmlnaHRzIHJlc2VydmVkLiBNSVQgbGljZW5zZS5cbi8vIENvcHlyaWdodCAyMDE4LTIwMjIgdGhlIERlbm8gYXV0aG9ycy4gQWxsIHJpZ2h0cyByZXNlcnZlZC4gTUlUIGxpY2Vuc2UuXG5cbmltcG9ydCB7IFR5cGUgfSBmcm9tIFwiLi4vdHlwZS50c1wiO1xuaW1wb3J0IHR5cGUgeyBBbnkgfSBmcm9tIFwiLi4vdXRpbHMudHNcIjtcblxuY29uc3QgUkVHRVhQID0gL15cXC8oPzxyZWdleHA+W1xcc1xcU10rKVxcLyg/PG1vZGlmaWVycz5bZ2lzbXV5XSopJC87XG5cbmV4cG9ydCBjb25zdCByZWdleHAgPSBuZXcgVHlwZShcInRhZzp5YW1sLm9yZywyMDAyOmpzL3JlZ2V4cFwiLCB7XG4gIGtpbmQ6IFwic2NhbGFyXCIsXG4gIHJlc29sdmUoZGF0YTogQW55KSB7XG4gICAgaWYgKChkYXRhID09PSBudWxsKSB8fCAoIWRhdGEubGVuZ3RoKSkge1xuICAgICAgcmV0dXJuIGZhbHNlO1xuICAgIH1cblxuICAgIGNvbnN0IHJlZ2V4cCA9IGAke2RhdGF9YDtcbiAgICBpZiAocmVnZXhwLmNoYXJBdCgwKSA9PT0gXCIvXCIpIHtcbiAgICAgIC8vIEVuc3VyZSByZWdleCBpcyBwcm9wZXJseSB0ZXJtaW5hdGVkXG4gICAgICBpZiAoIVJFR0VYUC50ZXN0KGRhdGEpKSB7XG4gICAgICAgIHJldHVybiBmYWxzZTtcbiAgICAgIH1cbiAgICAgIC8vIENoZWNrIG5vIGR1cGxpY2F0ZSBtb2RpZmllcnNcbiAgICAgIGNvbnN0IG1vZGlmaWVycyA9IFsuLi4ocmVnZXhwLm1hdGNoKFJFR0VYUCk/Lmdyb3Vwcz8ubW9kaWZpZXJzID8/IFwiXCIpXTtcbiAgICAgIGlmIChuZXcgU2V0KG1vZGlmaWVycykuc2l6ZSA8IG1vZGlmaWVycy5sZW5ndGgpIHtcbiAgICAgICAgcmV0dXJuIGZhbHNlO1xuICAgICAgfVxuICAgIH1cblxuICAgIHJldHVybiB0cnVlO1xuICB9LFxuICBjb25zdHJ1Y3QoZGF0YTogc3RyaW5nKSB7XG4gICAgY29uc3QgeyByZWdleHAgPSBgJHtkYXRhfWAsIG1vZGlmaWVycyA9IFwiXCIgfSA9XG4gICAgICBgJHtkYXRhfWAubWF0Y2goUkVHRVhQKT8uZ3JvdXBzID8/IHt9O1xuICAgIHJldHVybiBuZXcgUmVnRXhwKHJlZ2V4cCwgbW9kaWZpZXJzKTtcbiAgfSxcbiAgcHJlZGljYXRlKG9iamVjdDogdW5rbm93bikge1xuICAgIHJldHVybiBvYmplY3QgaW5zdGFuY2VvZiBSZWdFeHA7XG4gIH0sXG4gIHJlcHJlc2VudChvYmplY3Q6IFJlZ0V4cCkge1xuICAgIHJldHVybiBvYmplY3QudG9TdHJpbmcoKTtcbiAgfSxcbn0pO1xuIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiJBQUFBLG1EQUFtRDtBQUNuRCwyRkFBMkY7QUFDM0YsMEVBQTBFO0FBQzFFLDBFQUEwRTtBQUUxRSxTQUFTLElBQUksUUFBUSxhQUFhO0FBR2xDLE1BQU0sU0FBUztBQUVmLE9BQU8sTUFBTSxTQUFTLElBQUksS0FBSywrQkFBK0I7RUFDNUQsTUFBTTtFQUNOLFNBQVEsSUFBUztJQUNmLElBQUksQUFBQyxTQUFTLFFBQVUsQ0FBQyxLQUFLLE1BQU0sRUFBRztNQUNyQyxPQUFPO0lBQ1Q7SUFFQSxNQUFNLFNBQVMsR0FBRyxNQUFNO0lBQ3hCLElBQUksT0FBTyxNQUFNLENBQUMsT0FBTyxLQUFLO01BQzVCLHNDQUFzQztNQUN0QyxJQUFJLENBQUMsT0FBTyxJQUFJLENBQUMsT0FBTztRQUN0QixPQUFPO01BQ1Q7TUFDQSwrQkFBK0I7TUFDL0IsTUFBTSxZQUFZO1dBQUssT0FBTyxLQUFLLENBQUMsU0FBUyxRQUFRLGFBQWE7T0FBSTtNQUN0RSxJQUFJLElBQUksSUFBSSxXQUFXLElBQUksR0FBRyxVQUFVLE1BQU0sRUFBRTtRQUM5QyxPQUFPO01BQ1Q7SUFDRjtJQUVBLE9BQU87RUFDVDtFQUNBLFdBQVUsSUFBWTtJQUNwQixNQUFNLEVBQUUsU0FBUyxHQUFHLE1BQU0sRUFBRSxZQUFZLEVBQUUsRUFBRSxHQUMxQyxHQUFHLE1BQU0sQ0FBQyxLQUFLLENBQUMsU0FBUyxVQUFVLENBQUM7SUFDdEMsT0FBTyxJQUFJLE9BQU8sUUFBUTtFQUM1QjtFQUNBLFdBQVUsTUFBZTtJQUN2QixPQUFPLGtCQUFrQjtFQUMzQjtFQUNBLFdBQVUsTUFBYztJQUN0QixPQUFPLE9BQU8sUUFBUTtFQUN4QjtBQUNGLEdBQUcifQ==
// denoCacheMetadata=7845526956393790702,10805635684898525330