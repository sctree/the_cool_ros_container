// Ported from js-yaml v3.13.1:
// https://github.com/nodeca/js-yaml/commit/665aadda42349dcae869f12040d9b10ef18d12da
// Copyright 2011-2015 by Vitaly Puzrin. All rights reserved. MIT license.
// Copyright 2018-2022 the Deno authors. All rights reserved. MIT license.
import { Type } from "../type.ts";
function resolveYamlNull(data) {
  const max = data.length;
  return max === 1 && data === "~" || max === 4 && (data === "null" || data === "Null" || data === "NULL");
}
function constructYamlNull() {
  return null;
}
function isNull(object) {
  return object === null;
}
export const nil = new Type("tag:yaml.org,2002:null", {
  construct: constructYamlNull,
  defaultStyle: "lowercase",
  kind: "scalar",
  predicate: isNull,
  represent: {
    canonical () {
      return "~";
    },
    lowercase () {
      return "null";
    },
    uppercase () {
      return "NULL";
    },
    camelcase () {
      return "Null";
    }
  },
  resolve: resolveYamlNull
});
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbImh0dHBzOi8vZGVuby5sYW5kL3N0ZEAwLjE2OC4wL2VuY29kaW5nL195YW1sL3R5cGUvbmlsLnRzIl0sInNvdXJjZXNDb250ZW50IjpbIi8vIFBvcnRlZCBmcm9tIGpzLXlhbWwgdjMuMTMuMTpcbi8vIGh0dHBzOi8vZ2l0aHViLmNvbS9ub2RlY2EvanMteWFtbC9jb21taXQvNjY1YWFkZGE0MjM0OWRjYWU4NjlmMTIwNDBkOWIxMGVmMThkMTJkYVxuLy8gQ29weXJpZ2h0IDIwMTEtMjAxNSBieSBWaXRhbHkgUHV6cmluLiBBbGwgcmlnaHRzIHJlc2VydmVkLiBNSVQgbGljZW5zZS5cbi8vIENvcHlyaWdodCAyMDE4LTIwMjIgdGhlIERlbm8gYXV0aG9ycy4gQWxsIHJpZ2h0cyByZXNlcnZlZC4gTUlUIGxpY2Vuc2UuXG5cbmltcG9ydCB7IFR5cGUgfSBmcm9tIFwiLi4vdHlwZS50c1wiO1xuXG5mdW5jdGlvbiByZXNvbHZlWWFtbE51bGwoZGF0YTogc3RyaW5nKTogYm9vbGVhbiB7XG4gIGNvbnN0IG1heCA9IGRhdGEubGVuZ3RoO1xuXG4gIHJldHVybiAoXG4gICAgKG1heCA9PT0gMSAmJiBkYXRhID09PSBcIn5cIikgfHxcbiAgICAobWF4ID09PSA0ICYmIChkYXRhID09PSBcIm51bGxcIiB8fCBkYXRhID09PSBcIk51bGxcIiB8fCBkYXRhID09PSBcIk5VTExcIikpXG4gICk7XG59XG5cbmZ1bmN0aW9uIGNvbnN0cnVjdFlhbWxOdWxsKCk6IG51bGwge1xuICByZXR1cm4gbnVsbDtcbn1cblxuZnVuY3Rpb24gaXNOdWxsKG9iamVjdDogdW5rbm93bik6IG9iamVjdCBpcyBudWxsIHtcbiAgcmV0dXJuIG9iamVjdCA9PT0gbnVsbDtcbn1cblxuZXhwb3J0IGNvbnN0IG5pbCA9IG5ldyBUeXBlKFwidGFnOnlhbWwub3JnLDIwMDI6bnVsbFwiLCB7XG4gIGNvbnN0cnVjdDogY29uc3RydWN0WWFtbE51bGwsXG4gIGRlZmF1bHRTdHlsZTogXCJsb3dlcmNhc2VcIixcbiAga2luZDogXCJzY2FsYXJcIixcbiAgcHJlZGljYXRlOiBpc051bGwsXG4gIHJlcHJlc2VudDoge1xuICAgIGNhbm9uaWNhbCgpOiBzdHJpbmcge1xuICAgICAgcmV0dXJuIFwiflwiO1xuICAgIH0sXG4gICAgbG93ZXJjYXNlKCk6IHN0cmluZyB7XG4gICAgICByZXR1cm4gXCJudWxsXCI7XG4gICAgfSxcbiAgICB1cHBlcmNhc2UoKTogc3RyaW5nIHtcbiAgICAgIHJldHVybiBcIk5VTExcIjtcbiAgICB9LFxuICAgIGNhbWVsY2FzZSgpOiBzdHJpbmcge1xuICAgICAgcmV0dXJuIFwiTnVsbFwiO1xuICAgIH0sXG4gIH0sXG4gIHJlc29sdmU6IHJlc29sdmVZYW1sTnVsbCxcbn0pO1xuIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiJBQUFBLCtCQUErQjtBQUMvQixvRkFBb0Y7QUFDcEYsMEVBQTBFO0FBQzFFLDBFQUEwRTtBQUUxRSxTQUFTLElBQUksUUFBUSxhQUFhO0FBRWxDLFNBQVMsZ0JBQWdCLElBQVk7RUFDbkMsTUFBTSxNQUFNLEtBQUssTUFBTTtFQUV2QixPQUNFLEFBQUMsUUFBUSxLQUFLLFNBQVMsT0FDdEIsUUFBUSxLQUFLLENBQUMsU0FBUyxVQUFVLFNBQVMsVUFBVSxTQUFTLE1BQU07QUFFeEU7QUFFQSxTQUFTO0VBQ1AsT0FBTztBQUNUO0FBRUEsU0FBUyxPQUFPLE1BQWU7RUFDN0IsT0FBTyxXQUFXO0FBQ3BCO0FBRUEsT0FBTyxNQUFNLE1BQU0sSUFBSSxLQUFLLDBCQUEwQjtFQUNwRCxXQUFXO0VBQ1gsY0FBYztFQUNkLE1BQU07RUFDTixXQUFXO0VBQ1gsV0FBVztJQUNUO01BQ0UsT0FBTztJQUNUO0lBQ0E7TUFDRSxPQUFPO0lBQ1Q7SUFDQTtNQUNFLE9BQU87SUFDVDtJQUNBO01BQ0UsT0FBTztJQUNUO0VBQ0Y7RUFDQSxTQUFTO0FBQ1gsR0FBRyJ9
// denoCacheMetadata=5565669590732590740,15795403715074989090