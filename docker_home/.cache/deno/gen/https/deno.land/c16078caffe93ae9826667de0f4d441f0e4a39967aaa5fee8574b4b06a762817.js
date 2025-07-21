// Ported from js-yaml v3.13.1:
// https://github.com/nodeca/js-yaml/commit/665aadda42349dcae869f12040d9b10ef18d12da
// Copyright 2011-2015 by Vitaly Puzrin. All rights reserved. MIT license.
// Copyright 2018-2022 the Deno authors. All rights reserved. MIT license.
import { YAMLError } from "./error.ts";
function compileList(schema, name, result) {
  const exclude = [];
  for (const includedSchema of schema.include){
    result = compileList(includedSchema, name, result);
  }
  for (const currentType of schema[name]){
    for(let previousIndex = 0; previousIndex < result.length; previousIndex++){
      const previousType = result[previousIndex];
      if (previousType.tag === currentType.tag && previousType.kind === currentType.kind) {
        exclude.push(previousIndex);
      }
    }
    result.push(currentType);
  }
  return result.filter((_type, index)=>!exclude.includes(index));
}
function compileMap(...typesList) {
  const result = {
    fallback: {},
    mapping: {},
    scalar: {},
    sequence: {}
  };
  for (const types of typesList){
    for (const type of types){
      if (type.kind !== null) {
        result[type.kind][type.tag] = result["fallback"][type.tag] = type;
      }
    }
  }
  return result;
}
export class Schema {
  static SCHEMA_DEFAULT;
  implicit;
  explicit;
  include;
  compiledImplicit;
  compiledExplicit;
  compiledTypeMap;
  constructor(definition){
    this.explicit = definition.explicit || [];
    this.implicit = definition.implicit || [];
    this.include = definition.include || [];
    for (const type of this.implicit){
      if (type.loadKind && type.loadKind !== "scalar") {
        throw new YAMLError("There is a non-scalar type in the implicit list of a schema. Implicit resolving of such types is not supported.");
      }
    }
    this.compiledImplicit = compileList(this, "implicit", []);
    this.compiledExplicit = compileList(this, "explicit", []);
    this.compiledTypeMap = compileMap(this.compiledImplicit, this.compiledExplicit);
  }
  /* Returns a new extended schema from current schema */ extend(definition) {
    return new Schema({
      implicit: [
        ...new Set([
          ...this.implicit,
          ...definition?.implicit ?? []
        ])
      ],
      explicit: [
        ...new Set([
          ...this.explicit,
          ...definition?.explicit ?? []
        ])
      ],
      include: [
        ...new Set([
          ...this.include,
          ...definition?.include ?? []
        ])
      ]
    });
  }
  static create() {}
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbImh0dHBzOi8vZGVuby5sYW5kL3N0ZEAwLjE2OC4wL2VuY29kaW5nL195YW1sL3NjaGVtYS50cyJdLCJzb3VyY2VzQ29udGVudCI6WyIvLyBQb3J0ZWQgZnJvbSBqcy15YW1sIHYzLjEzLjE6XG4vLyBodHRwczovL2dpdGh1Yi5jb20vbm9kZWNhL2pzLXlhbWwvY29tbWl0LzY2NWFhZGRhNDIzNDlkY2FlODY5ZjEyMDQwZDliMTBlZjE4ZDEyZGFcbi8vIENvcHlyaWdodCAyMDExLTIwMTUgYnkgVml0YWx5IFB1enJpbi4gQWxsIHJpZ2h0cyByZXNlcnZlZC4gTUlUIGxpY2Vuc2UuXG4vLyBDb3B5cmlnaHQgMjAxOC0yMDIyIHRoZSBEZW5vIGF1dGhvcnMuIEFsbCByaWdodHMgcmVzZXJ2ZWQuIE1JVCBsaWNlbnNlLlxuXG5pbXBvcnQgeyBZQU1MRXJyb3IgfSBmcm9tIFwiLi9lcnJvci50c1wiO1xuaW1wb3J0IHR5cGUgeyBLaW5kVHlwZSwgVHlwZSB9IGZyb20gXCIuL3R5cGUudHNcIjtcbmltcG9ydCB0eXBlIHsgQW55LCBBcnJheU9iamVjdCB9IGZyb20gXCIuL3V0aWxzLnRzXCI7XG5cbmZ1bmN0aW9uIGNvbXBpbGVMaXN0KFxuICBzY2hlbWE6IFNjaGVtYSxcbiAgbmFtZTogXCJpbXBsaWNpdFwiIHwgXCJleHBsaWNpdFwiLFxuICByZXN1bHQ6IFR5cGVbXSxcbik6IFR5cGVbXSB7XG4gIGNvbnN0IGV4Y2x1ZGU6IG51bWJlcltdID0gW107XG5cbiAgZm9yIChjb25zdCBpbmNsdWRlZFNjaGVtYSBvZiBzY2hlbWEuaW5jbHVkZSkge1xuICAgIHJlc3VsdCA9IGNvbXBpbGVMaXN0KGluY2x1ZGVkU2NoZW1hLCBuYW1lLCByZXN1bHQpO1xuICB9XG5cbiAgZm9yIChjb25zdCBjdXJyZW50VHlwZSBvZiBzY2hlbWFbbmFtZV0pIHtcbiAgICBmb3IgKFxuICAgICAgbGV0IHByZXZpb3VzSW5kZXggPSAwO1xuICAgICAgcHJldmlvdXNJbmRleCA8IHJlc3VsdC5sZW5ndGg7XG4gICAgICBwcmV2aW91c0luZGV4KytcbiAgICApIHtcbiAgICAgIGNvbnN0IHByZXZpb3VzVHlwZSA9IHJlc3VsdFtwcmV2aW91c0luZGV4XTtcbiAgICAgIGlmIChcbiAgICAgICAgcHJldmlvdXNUeXBlLnRhZyA9PT0gY3VycmVudFR5cGUudGFnICYmXG4gICAgICAgIHByZXZpb3VzVHlwZS5raW5kID09PSBjdXJyZW50VHlwZS5raW5kXG4gICAgICApIHtcbiAgICAgICAgZXhjbHVkZS5wdXNoKHByZXZpb3VzSW5kZXgpO1xuICAgICAgfVxuICAgIH1cblxuICAgIHJlc3VsdC5wdXNoKGN1cnJlbnRUeXBlKTtcbiAgfVxuXG4gIHJldHVybiByZXN1bHQuZmlsdGVyKChfdHlwZSwgaW5kZXgpOiB1bmtub3duID0+ICFleGNsdWRlLmluY2x1ZGVzKGluZGV4KSk7XG59XG5cbmV4cG9ydCB0eXBlIFR5cGVNYXAgPSB7IFtrIGluIEtpbmRUeXBlIHwgXCJmYWxsYmFja1wiXTogQXJyYXlPYmplY3Q8VHlwZT4gfTtcbmZ1bmN0aW9uIGNvbXBpbGVNYXAoLi4udHlwZXNMaXN0OiBUeXBlW11bXSk6IFR5cGVNYXAge1xuICBjb25zdCByZXN1bHQ6IFR5cGVNYXAgPSB7XG4gICAgZmFsbGJhY2s6IHt9LFxuICAgIG1hcHBpbmc6IHt9LFxuICAgIHNjYWxhcjoge30sXG4gICAgc2VxdWVuY2U6IHt9LFxuICB9O1xuXG4gIGZvciAoY29uc3QgdHlwZXMgb2YgdHlwZXNMaXN0KSB7XG4gICAgZm9yIChjb25zdCB0eXBlIG9mIHR5cGVzKSB7XG4gICAgICBpZiAodHlwZS5raW5kICE9PSBudWxsKSB7XG4gICAgICAgIHJlc3VsdFt0eXBlLmtpbmRdW3R5cGUudGFnXSA9IHJlc3VsdFtcImZhbGxiYWNrXCJdW3R5cGUudGFnXSA9IHR5cGU7XG4gICAgICB9XG4gICAgfVxuICB9XG4gIHJldHVybiByZXN1bHQ7XG59XG5cbmV4cG9ydCBjbGFzcyBTY2hlbWEgaW1wbGVtZW50cyBTY2hlbWFEZWZpbml0aW9uIHtcbiAgcHVibGljIHN0YXRpYyBTQ0hFTUFfREVGQVVMVD86IFNjaGVtYTtcblxuICBwdWJsaWMgaW1wbGljaXQ6IFR5cGVbXTtcbiAgcHVibGljIGV4cGxpY2l0OiBUeXBlW107XG4gIHB1YmxpYyBpbmNsdWRlOiBTY2hlbWFbXTtcblxuICBwdWJsaWMgY29tcGlsZWRJbXBsaWNpdDogVHlwZVtdO1xuICBwdWJsaWMgY29tcGlsZWRFeHBsaWNpdDogVHlwZVtdO1xuICBwdWJsaWMgY29tcGlsZWRUeXBlTWFwOiBUeXBlTWFwO1xuXG4gIGNvbnN0cnVjdG9yKGRlZmluaXRpb246IFNjaGVtYURlZmluaXRpb24pIHtcbiAgICB0aGlzLmV4cGxpY2l0ID0gZGVmaW5pdGlvbi5leHBsaWNpdCB8fCBbXTtcbiAgICB0aGlzLmltcGxpY2l0ID0gZGVmaW5pdGlvbi5pbXBsaWNpdCB8fCBbXTtcbiAgICB0aGlzLmluY2x1ZGUgPSBkZWZpbml0aW9uLmluY2x1ZGUgfHwgW107XG5cbiAgICBmb3IgKGNvbnN0IHR5cGUgb2YgdGhpcy5pbXBsaWNpdCkge1xuICAgICAgaWYgKHR5cGUubG9hZEtpbmQgJiYgdHlwZS5sb2FkS2luZCAhPT0gXCJzY2FsYXJcIikge1xuICAgICAgICB0aHJvdyBuZXcgWUFNTEVycm9yKFxuICAgICAgICAgIFwiVGhlcmUgaXMgYSBub24tc2NhbGFyIHR5cGUgaW4gdGhlIGltcGxpY2l0IGxpc3Qgb2YgYSBzY2hlbWEuIEltcGxpY2l0IHJlc29sdmluZyBvZiBzdWNoIHR5cGVzIGlzIG5vdCBzdXBwb3J0ZWQuXCIsXG4gICAgICAgICk7XG4gICAgICB9XG4gICAgfVxuXG4gICAgdGhpcy5jb21waWxlZEltcGxpY2l0ID0gY29tcGlsZUxpc3QodGhpcywgXCJpbXBsaWNpdFwiLCBbXSk7XG4gICAgdGhpcy5jb21waWxlZEV4cGxpY2l0ID0gY29tcGlsZUxpc3QodGhpcywgXCJleHBsaWNpdFwiLCBbXSk7XG4gICAgdGhpcy5jb21waWxlZFR5cGVNYXAgPSBjb21waWxlTWFwKFxuICAgICAgdGhpcy5jb21waWxlZEltcGxpY2l0LFxuICAgICAgdGhpcy5jb21waWxlZEV4cGxpY2l0LFxuICAgICk7XG4gIH1cblxuICAvKiBSZXR1cm5zIGEgbmV3IGV4dGVuZGVkIHNjaGVtYSBmcm9tIGN1cnJlbnQgc2NoZW1hICovXG4gIHB1YmxpYyBleHRlbmQoZGVmaW5pdGlvbjogU2NoZW1hRGVmaW5pdGlvbikge1xuICAgIHJldHVybiBuZXcgU2NoZW1hKHtcbiAgICAgIGltcGxpY2l0OiBbXG4gICAgICAgIC4uLm5ldyBTZXQoWy4uLnRoaXMuaW1wbGljaXQsIC4uLihkZWZpbml0aW9uPy5pbXBsaWNpdCA/PyBbXSldKSxcbiAgICAgIF0sXG4gICAgICBleHBsaWNpdDogW1xuICAgICAgICAuLi5uZXcgU2V0KFsuLi50aGlzLmV4cGxpY2l0LCAuLi4oZGVmaW5pdGlvbj8uZXhwbGljaXQgPz8gW10pXSksXG4gICAgICBdLFxuICAgICAgaW5jbHVkZTogWy4uLm5ldyBTZXQoWy4uLnRoaXMuaW5jbHVkZSwgLi4uKGRlZmluaXRpb24/LmluY2x1ZGUgPz8gW10pXSldLFxuICAgIH0pO1xuICB9XG5cbiAgcHVibGljIHN0YXRpYyBjcmVhdGUoKSB7fVxufVxuXG5leHBvcnQgaW50ZXJmYWNlIFNjaGVtYURlZmluaXRpb24ge1xuICBpbXBsaWNpdD86IEFueVtdO1xuICBleHBsaWNpdD86IFR5cGVbXTtcbiAgaW5jbHVkZT86IFNjaGVtYVtdO1xufVxuIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiJBQUFBLCtCQUErQjtBQUMvQixvRkFBb0Y7QUFDcEYsMEVBQTBFO0FBQzFFLDBFQUEwRTtBQUUxRSxTQUFTLFNBQVMsUUFBUSxhQUFhO0FBSXZDLFNBQVMsWUFDUCxNQUFjLEVBQ2QsSUFBNkIsRUFDN0IsTUFBYztFQUVkLE1BQU0sVUFBb0IsRUFBRTtFQUU1QixLQUFLLE1BQU0sa0JBQWtCLE9BQU8sT0FBTyxDQUFFO0lBQzNDLFNBQVMsWUFBWSxnQkFBZ0IsTUFBTTtFQUM3QztFQUVBLEtBQUssTUFBTSxlQUFlLE1BQU0sQ0FBQyxLQUFLLENBQUU7SUFDdEMsSUFDRSxJQUFJLGdCQUFnQixHQUNwQixnQkFBZ0IsT0FBTyxNQUFNLEVBQzdCLGdCQUNBO01BQ0EsTUFBTSxlQUFlLE1BQU0sQ0FBQyxjQUFjO01BQzFDLElBQ0UsYUFBYSxHQUFHLEtBQUssWUFBWSxHQUFHLElBQ3BDLGFBQWEsSUFBSSxLQUFLLFlBQVksSUFBSSxFQUN0QztRQUNBLFFBQVEsSUFBSSxDQUFDO01BQ2Y7SUFDRjtJQUVBLE9BQU8sSUFBSSxDQUFDO0VBQ2Q7RUFFQSxPQUFPLE9BQU8sTUFBTSxDQUFDLENBQUMsT0FBTyxRQUFtQixDQUFDLFFBQVEsUUFBUSxDQUFDO0FBQ3BFO0FBR0EsU0FBUyxXQUFXLEdBQUcsU0FBbUI7RUFDeEMsTUFBTSxTQUFrQjtJQUN0QixVQUFVLENBQUM7SUFDWCxTQUFTLENBQUM7SUFDVixRQUFRLENBQUM7SUFDVCxVQUFVLENBQUM7RUFDYjtFQUVBLEtBQUssTUFBTSxTQUFTLFVBQVc7SUFDN0IsS0FBSyxNQUFNLFFBQVEsTUFBTztNQUN4QixJQUFJLEtBQUssSUFBSSxLQUFLLE1BQU07UUFDdEIsTUFBTSxDQUFDLEtBQUssSUFBSSxDQUFDLENBQUMsS0FBSyxHQUFHLENBQUMsR0FBRyxNQUFNLENBQUMsV0FBVyxDQUFDLEtBQUssR0FBRyxDQUFDLEdBQUc7TUFDL0Q7SUFDRjtFQUNGO0VBQ0EsT0FBTztBQUNUO0FBRUEsT0FBTyxNQUFNO0VBQ1gsT0FBYyxlQUF3QjtFQUUvQixTQUFpQjtFQUNqQixTQUFpQjtFQUNqQixRQUFrQjtFQUVsQixpQkFBeUI7RUFDekIsaUJBQXlCO0VBQ3pCLGdCQUF5QjtFQUVoQyxZQUFZLFVBQTRCLENBQUU7SUFDeEMsSUFBSSxDQUFDLFFBQVEsR0FBRyxXQUFXLFFBQVEsSUFBSSxFQUFFO0lBQ3pDLElBQUksQ0FBQyxRQUFRLEdBQUcsV0FBVyxRQUFRLElBQUksRUFBRTtJQUN6QyxJQUFJLENBQUMsT0FBTyxHQUFHLFdBQVcsT0FBTyxJQUFJLEVBQUU7SUFFdkMsS0FBSyxNQUFNLFFBQVEsSUFBSSxDQUFDLFFBQVEsQ0FBRTtNQUNoQyxJQUFJLEtBQUssUUFBUSxJQUFJLEtBQUssUUFBUSxLQUFLLFVBQVU7UUFDL0MsTUFBTSxJQUFJLFVBQ1I7TUFFSjtJQUNGO0lBRUEsSUFBSSxDQUFDLGdCQUFnQixHQUFHLFlBQVksSUFBSSxFQUFFLFlBQVksRUFBRTtJQUN4RCxJQUFJLENBQUMsZ0JBQWdCLEdBQUcsWUFBWSxJQUFJLEVBQUUsWUFBWSxFQUFFO0lBQ3hELElBQUksQ0FBQyxlQUFlLEdBQUcsV0FDckIsSUFBSSxDQUFDLGdCQUFnQixFQUNyQixJQUFJLENBQUMsZ0JBQWdCO0VBRXpCO0VBRUEscURBQXFELEdBQ3JELEFBQU8sT0FBTyxVQUE0QixFQUFFO0lBQzFDLE9BQU8sSUFBSSxPQUFPO01BQ2hCLFVBQVU7V0FDTCxJQUFJLElBQUk7YUFBSSxJQUFJLENBQUMsUUFBUTthQUFNLFlBQVksWUFBWSxFQUFFO1NBQUU7T0FDL0Q7TUFDRCxVQUFVO1dBQ0wsSUFBSSxJQUFJO2FBQUksSUFBSSxDQUFDLFFBQVE7YUFBTSxZQUFZLFlBQVksRUFBRTtTQUFFO09BQy9EO01BQ0QsU0FBUztXQUFJLElBQUksSUFBSTthQUFJLElBQUksQ0FBQyxPQUFPO2FBQU0sWUFBWSxXQUFXLEVBQUU7U0FBRTtPQUFFO0lBQzFFO0VBQ0Y7RUFFQSxPQUFjLFNBQVMsQ0FBQztBQUMxQiJ9
// denoCacheMetadata=17674201375333247772,17270394526319784945