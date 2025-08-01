// Ported from js-yaml v3.13.1:
// https://github.com/nodeca/js-yaml/commit/665aadda42349dcae869f12040d9b10ef18d12da
// Copyright 2011-2015 by Vitaly Puzrin. All rights reserved. MIT license.
// Copyright 2018-2022 the Deno authors. All rights reserved. MIT license.
import { Type } from "../type.ts";
export const map = new Type("tag:yaml.org,2002:map", {
  construct (data) {
    return data !== null ? data : {};
  },
  kind: "mapping"
});
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbImh0dHBzOi8vZGVuby5sYW5kL3N0ZEAwLjE2OC4wL2VuY29kaW5nL195YW1sL3R5cGUvbWFwLnRzIl0sInNvdXJjZXNDb250ZW50IjpbIi8vIFBvcnRlZCBmcm9tIGpzLXlhbWwgdjMuMTMuMTpcbi8vIGh0dHBzOi8vZ2l0aHViLmNvbS9ub2RlY2EvanMteWFtbC9jb21taXQvNjY1YWFkZGE0MjM0OWRjYWU4NjlmMTIwNDBkOWIxMGVmMThkMTJkYVxuLy8gQ29weXJpZ2h0IDIwMTEtMjAxNSBieSBWaXRhbHkgUHV6cmluLiBBbGwgcmlnaHRzIHJlc2VydmVkLiBNSVQgbGljZW5zZS5cbi8vIENvcHlyaWdodCAyMDE4LTIwMjIgdGhlIERlbm8gYXV0aG9ycy4gQWxsIHJpZ2h0cyByZXNlcnZlZC4gTUlUIGxpY2Vuc2UuXG5cbmltcG9ydCB7IFR5cGUgfSBmcm9tIFwiLi4vdHlwZS50c1wiO1xuaW1wb3J0IHR5cGUgeyBBbnkgfSBmcm9tIFwiLi4vdXRpbHMudHNcIjtcblxuZXhwb3J0IGNvbnN0IG1hcCA9IG5ldyBUeXBlKFwidGFnOnlhbWwub3JnLDIwMDI6bWFwXCIsIHtcbiAgY29uc3RydWN0KGRhdGEpOiBBbnkge1xuICAgIHJldHVybiBkYXRhICE9PSBudWxsID8gZGF0YSA6IHt9O1xuICB9LFxuICBraW5kOiBcIm1hcHBpbmdcIixcbn0pO1xuIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiJBQUFBLCtCQUErQjtBQUMvQixvRkFBb0Y7QUFDcEYsMEVBQTBFO0FBQzFFLDBFQUEwRTtBQUUxRSxTQUFTLElBQUksUUFBUSxhQUFhO0FBR2xDLE9BQU8sTUFBTSxNQUFNLElBQUksS0FBSyx5QkFBeUI7RUFDbkQsV0FBVSxJQUFJO0lBQ1osT0FBTyxTQUFTLE9BQU8sT0FBTyxDQUFDO0VBQ2pDO0VBQ0EsTUFBTTtBQUNSLEdBQUcifQ==
// denoCacheMetadata=7682663749182890105,6997981277034324713