// Ported from js-yaml v3.13.1:
// https://github.com/nodeca/js-yaml/commit/665aadda42349dcae869f12040d9b10ef18d12da
// Copyright 2011-2015 by Vitaly Puzrin. All rights reserved. MIT license.
// Copyright 2018-2022 the Deno authors. All rights reserved. MIT license.
import { Schema } from "../schema.ts";
import { json } from "./json.ts";
// Standard YAML's Core schema.
// http://www.yaml.org/spec/1.2/spec.html#id2804923
export const core = new Schema({
  include: [
    json
  ]
});
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbImh0dHBzOi8vZGVuby5sYW5kL3N0ZEAwLjE2OC4wL2VuY29kaW5nL195YW1sL3NjaGVtYS9jb3JlLnRzIl0sInNvdXJjZXNDb250ZW50IjpbIi8vIFBvcnRlZCBmcm9tIGpzLXlhbWwgdjMuMTMuMTpcbi8vIGh0dHBzOi8vZ2l0aHViLmNvbS9ub2RlY2EvanMteWFtbC9jb21taXQvNjY1YWFkZGE0MjM0OWRjYWU4NjlmMTIwNDBkOWIxMGVmMThkMTJkYVxuLy8gQ29weXJpZ2h0IDIwMTEtMjAxNSBieSBWaXRhbHkgUHV6cmluLiBBbGwgcmlnaHRzIHJlc2VydmVkLiBNSVQgbGljZW5zZS5cbi8vIENvcHlyaWdodCAyMDE4LTIwMjIgdGhlIERlbm8gYXV0aG9ycy4gQWxsIHJpZ2h0cyByZXNlcnZlZC4gTUlUIGxpY2Vuc2UuXG5cbmltcG9ydCB7IFNjaGVtYSB9IGZyb20gXCIuLi9zY2hlbWEudHNcIjtcbmltcG9ydCB7IGpzb24gfSBmcm9tIFwiLi9qc29uLnRzXCI7XG5cbi8vIFN0YW5kYXJkIFlBTUwncyBDb3JlIHNjaGVtYS5cbi8vIGh0dHA6Ly93d3cueWFtbC5vcmcvc3BlYy8xLjIvc3BlYy5odG1sI2lkMjgwNDkyM1xuZXhwb3J0IGNvbnN0IGNvcmUgPSBuZXcgU2NoZW1hKHtcbiAgaW5jbHVkZTogW2pzb25dLFxufSk7XG4iXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IkFBQUEsK0JBQStCO0FBQy9CLG9GQUFvRjtBQUNwRiwwRUFBMEU7QUFDMUUsMEVBQTBFO0FBRTFFLFNBQVMsTUFBTSxRQUFRLGVBQWU7QUFDdEMsU0FBUyxJQUFJLFFBQVEsWUFBWTtBQUVqQywrQkFBK0I7QUFDL0IsbURBQW1EO0FBQ25ELE9BQU8sTUFBTSxPQUFPLElBQUksT0FBTztFQUM3QixTQUFTO0lBQUM7R0FBSztBQUNqQixHQUFHIn0=
// denoCacheMetadata=8292128165459808667,16721076501939776692