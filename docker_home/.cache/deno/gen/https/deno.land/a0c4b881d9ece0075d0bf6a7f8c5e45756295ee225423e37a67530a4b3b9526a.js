// Ported from js-yaml v3.13.1:
// https://github.com/nodeca/js-yaml/commit/665aadda42349dcae869f12040d9b10ef18d12da
// Copyright 2011-2015 by Vitaly Puzrin. All rights reserved. MIT license.
// Copyright 2018-2022 the Deno authors. All rights reserved. MIT license.
import { Type } from "../type.ts";
import { isBoolean } from "../utils.ts";
function resolveYamlBoolean(data) {
  const max = data.length;
  return max === 4 && (data === "true" || data === "True" || data === "TRUE") || max === 5 && (data === "false" || data === "False" || data === "FALSE");
}
function constructYamlBoolean(data) {
  return data === "true" || data === "True" || data === "TRUE";
}
export const bool = new Type("tag:yaml.org,2002:bool", {
  construct: constructYamlBoolean,
  defaultStyle: "lowercase",
  kind: "scalar",
  predicate: isBoolean,
  represent: {
    lowercase (object) {
      return object ? "true" : "false";
    },
    uppercase (object) {
      return object ? "TRUE" : "FALSE";
    },
    camelcase (object) {
      return object ? "True" : "False";
    }
  },
  resolve: resolveYamlBoolean
});
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbImh0dHBzOi8vZGVuby5sYW5kL3N0ZEAwLjE2OC4wL2VuY29kaW5nL195YW1sL3R5cGUvYm9vbC50cyJdLCJzb3VyY2VzQ29udGVudCI6WyIvLyBQb3J0ZWQgZnJvbSBqcy15YW1sIHYzLjEzLjE6XG4vLyBodHRwczovL2dpdGh1Yi5jb20vbm9kZWNhL2pzLXlhbWwvY29tbWl0LzY2NWFhZGRhNDIzNDlkY2FlODY5ZjEyMDQwZDliMTBlZjE4ZDEyZGFcbi8vIENvcHlyaWdodCAyMDExLTIwMTUgYnkgVml0YWx5IFB1enJpbi4gQWxsIHJpZ2h0cyByZXNlcnZlZC4gTUlUIGxpY2Vuc2UuXG4vLyBDb3B5cmlnaHQgMjAxOC0yMDIyIHRoZSBEZW5vIGF1dGhvcnMuIEFsbCByaWdodHMgcmVzZXJ2ZWQuIE1JVCBsaWNlbnNlLlxuXG5pbXBvcnQgeyBUeXBlIH0gZnJvbSBcIi4uL3R5cGUudHNcIjtcbmltcG9ydCB7IGlzQm9vbGVhbiB9IGZyb20gXCIuLi91dGlscy50c1wiO1xuXG5mdW5jdGlvbiByZXNvbHZlWWFtbEJvb2xlYW4oZGF0YTogc3RyaW5nKTogYm9vbGVhbiB7XG4gIGNvbnN0IG1heCA9IGRhdGEubGVuZ3RoO1xuXG4gIHJldHVybiAoXG4gICAgKG1heCA9PT0gNCAmJiAoZGF0YSA9PT0gXCJ0cnVlXCIgfHwgZGF0YSA9PT0gXCJUcnVlXCIgfHwgZGF0YSA9PT0gXCJUUlVFXCIpKSB8fFxuICAgIChtYXggPT09IDUgJiYgKGRhdGEgPT09IFwiZmFsc2VcIiB8fCBkYXRhID09PSBcIkZhbHNlXCIgfHwgZGF0YSA9PT0gXCJGQUxTRVwiKSlcbiAgKTtcbn1cblxuZnVuY3Rpb24gY29uc3RydWN0WWFtbEJvb2xlYW4oZGF0YTogc3RyaW5nKTogYm9vbGVhbiB7XG4gIHJldHVybiBkYXRhID09PSBcInRydWVcIiB8fCBkYXRhID09PSBcIlRydWVcIiB8fCBkYXRhID09PSBcIlRSVUVcIjtcbn1cblxuZXhwb3J0IGNvbnN0IGJvb2wgPSBuZXcgVHlwZShcInRhZzp5YW1sLm9yZywyMDAyOmJvb2xcIiwge1xuICBjb25zdHJ1Y3Q6IGNvbnN0cnVjdFlhbWxCb29sZWFuLFxuICBkZWZhdWx0U3R5bGU6IFwibG93ZXJjYXNlXCIsXG4gIGtpbmQ6IFwic2NhbGFyXCIsXG4gIHByZWRpY2F0ZTogaXNCb29sZWFuLFxuICByZXByZXNlbnQ6IHtcbiAgICBsb3dlcmNhc2Uob2JqZWN0OiBib29sZWFuKTogc3RyaW5nIHtcbiAgICAgIHJldHVybiBvYmplY3QgPyBcInRydWVcIiA6IFwiZmFsc2VcIjtcbiAgICB9LFxuICAgIHVwcGVyY2FzZShvYmplY3Q6IGJvb2xlYW4pOiBzdHJpbmcge1xuICAgICAgcmV0dXJuIG9iamVjdCA/IFwiVFJVRVwiIDogXCJGQUxTRVwiO1xuICAgIH0sXG4gICAgY2FtZWxjYXNlKG9iamVjdDogYm9vbGVhbik6IHN0cmluZyB7XG4gICAgICByZXR1cm4gb2JqZWN0ID8gXCJUcnVlXCIgOiBcIkZhbHNlXCI7XG4gICAgfSxcbiAgfSxcbiAgcmVzb2x2ZTogcmVzb2x2ZVlhbWxCb29sZWFuLFxufSk7XG4iXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IkFBQUEsK0JBQStCO0FBQy9CLG9GQUFvRjtBQUNwRiwwRUFBMEU7QUFDMUUsMEVBQTBFO0FBRTFFLFNBQVMsSUFBSSxRQUFRLGFBQWE7QUFDbEMsU0FBUyxTQUFTLFFBQVEsY0FBYztBQUV4QyxTQUFTLG1CQUFtQixJQUFZO0VBQ3RDLE1BQU0sTUFBTSxLQUFLLE1BQU07RUFFdkIsT0FDRSxBQUFDLFFBQVEsS0FBSyxDQUFDLFNBQVMsVUFBVSxTQUFTLFVBQVUsU0FBUyxNQUFNLEtBQ25FLFFBQVEsS0FBSyxDQUFDLFNBQVMsV0FBVyxTQUFTLFdBQVcsU0FBUyxPQUFPO0FBRTNFO0FBRUEsU0FBUyxxQkFBcUIsSUFBWTtFQUN4QyxPQUFPLFNBQVMsVUFBVSxTQUFTLFVBQVUsU0FBUztBQUN4RDtBQUVBLE9BQU8sTUFBTSxPQUFPLElBQUksS0FBSywwQkFBMEI7RUFDckQsV0FBVztFQUNYLGNBQWM7RUFDZCxNQUFNO0VBQ04sV0FBVztFQUNYLFdBQVc7SUFDVCxXQUFVLE1BQWU7TUFDdkIsT0FBTyxTQUFTLFNBQVM7SUFDM0I7SUFDQSxXQUFVLE1BQWU7TUFDdkIsT0FBTyxTQUFTLFNBQVM7SUFDM0I7SUFDQSxXQUFVLE1BQWU7TUFDdkIsT0FBTyxTQUFTLFNBQVM7SUFDM0I7RUFDRjtFQUNBLFNBQVM7QUFDWCxHQUFHIn0=
// denoCacheMetadata=7534841978346038712,9714985533201669930