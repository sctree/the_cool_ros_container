// Ported and adapted from js-yaml-js-types v1.0.0:
// https://github.com/nodeca/js-yaml-js-types/tree/ac537e7bbdd3c2cbbd9882ca3919c520c2dc022b
// Copyright 2011-2015 by Vitaly Puzrin. All rights reserved. MIT license.
// Copyright 2018-2022 the Deno authors. All rights reserved. MIT license.
import { Type } from "../type.ts";
export const undefinedType = new Type("tag:yaml.org,2002:js/undefined", {
  kind: "scalar",
  resolve () {
    return true;
  },
  construct () {
    return undefined;
  },
  predicate (object) {
    return typeof object === "undefined";
  },
  represent () {
    return "";
  }
});
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbImh0dHBzOi8vZGVuby5sYW5kL3N0ZEAwLjE2OC4wL2VuY29kaW5nL195YW1sL3R5cGUvdW5kZWZpbmVkLnRzIl0sInNvdXJjZXNDb250ZW50IjpbIi8vIFBvcnRlZCBhbmQgYWRhcHRlZCBmcm9tIGpzLXlhbWwtanMtdHlwZXMgdjEuMC4wOlxuLy8gaHR0cHM6Ly9naXRodWIuY29tL25vZGVjYS9qcy15YW1sLWpzLXR5cGVzL3RyZWUvYWM1MzdlN2JiZGQzYzJjYmJkOTg4MmNhMzkxOWM1MjBjMmRjMDIyYlxuLy8gQ29weXJpZ2h0IDIwMTEtMjAxNSBieSBWaXRhbHkgUHV6cmluLiBBbGwgcmlnaHRzIHJlc2VydmVkLiBNSVQgbGljZW5zZS5cbi8vIENvcHlyaWdodCAyMDE4LTIwMjIgdGhlIERlbm8gYXV0aG9ycy4gQWxsIHJpZ2h0cyByZXNlcnZlZC4gTUlUIGxpY2Vuc2UuXG5cbmltcG9ydCB7IFR5cGUgfSBmcm9tIFwiLi4vdHlwZS50c1wiO1xuXG5leHBvcnQgY29uc3QgdW5kZWZpbmVkVHlwZSA9IG5ldyBUeXBlKFwidGFnOnlhbWwub3JnLDIwMDI6anMvdW5kZWZpbmVkXCIsIHtcbiAga2luZDogXCJzY2FsYXJcIixcbiAgcmVzb2x2ZSgpIHtcbiAgICByZXR1cm4gdHJ1ZTtcbiAgfSxcbiAgY29uc3RydWN0KCkge1xuICAgIHJldHVybiB1bmRlZmluZWQ7XG4gIH0sXG4gIHByZWRpY2F0ZShvYmplY3QpIHtcbiAgICByZXR1cm4gdHlwZW9mIG9iamVjdCA9PT0gXCJ1bmRlZmluZWRcIjtcbiAgfSxcbiAgcmVwcmVzZW50KCkge1xuICAgIHJldHVybiBcIlwiO1xuICB9LFxufSk7XG4iXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IkFBQUEsbURBQW1EO0FBQ25ELDJGQUEyRjtBQUMzRiwwRUFBMEU7QUFDMUUsMEVBQTBFO0FBRTFFLFNBQVMsSUFBSSxRQUFRLGFBQWE7QUFFbEMsT0FBTyxNQUFNLGdCQUFnQixJQUFJLEtBQUssa0NBQWtDO0VBQ3RFLE1BQU07RUFDTjtJQUNFLE9BQU87RUFDVDtFQUNBO0lBQ0UsT0FBTztFQUNUO0VBQ0EsV0FBVSxNQUFNO0lBQ2QsT0FBTyxPQUFPLFdBQVc7RUFDM0I7RUFDQTtJQUNFLE9BQU87RUFDVDtBQUNGLEdBQUcifQ==
// denoCacheMetadata=16045020580520174384,9464252660123938981