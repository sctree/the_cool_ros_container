// Copyright 2018-2024 the Deno authors. All rights reserved. MIT license.
// This module is browser compatible.
import { join } from "./join.ts";
import { SEPARATOR } from "./constants.ts";
import { normalizeGlob } from "./normalize_glob.ts";
/** Like join(), but doesn't collapse "**\/.." when `globstar` is true. */ export function joinGlobs(globs, { extended = true, globstar = false } = {}) {
  if (!globstar || globs.length === 0) {
    return join(...globs);
  }
  if (globs.length === 0) return ".";
  let joined;
  for (const glob of globs){
    const path = glob;
    if (path.length > 0) {
      if (!joined) joined = path;
      else joined += `${SEPARATOR}${path}`;
    }
  }
  if (!joined) return ".";
  return normalizeGlob(joined, {
    extended,
    globstar
  });
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbImh0dHBzOi8vZGVuby5sYW5kL3N0ZEAwLjIxNC4wL3BhdGgvcG9zaXgvam9pbl9nbG9icy50cyJdLCJzb3VyY2VzQ29udGVudCI6WyIvLyBDb3B5cmlnaHQgMjAxOC0yMDI0IHRoZSBEZW5vIGF1dGhvcnMuIEFsbCByaWdodHMgcmVzZXJ2ZWQuIE1JVCBsaWNlbnNlLlxuLy8gVGhpcyBtb2R1bGUgaXMgYnJvd3NlciBjb21wYXRpYmxlLlxuXG5pbXBvcnQgeyBHbG9iT3B0aW9ucyB9IGZyb20gXCIuLi9fY29tbW9uL2dsb2JfdG9fcmVnX2V4cC50c1wiO1xuaW1wb3J0IHsgam9pbiB9IGZyb20gXCIuL2pvaW4udHNcIjtcbmltcG9ydCB7IFNFUEFSQVRPUiB9IGZyb20gXCIuL2NvbnN0YW50cy50c1wiO1xuaW1wb3J0IHsgbm9ybWFsaXplR2xvYiB9IGZyb20gXCIuL25vcm1hbGl6ZV9nbG9iLnRzXCI7XG5cbi8qKiBMaWtlIGpvaW4oKSwgYnV0IGRvZXNuJ3QgY29sbGFwc2UgXCIqKlxcLy4uXCIgd2hlbiBgZ2xvYnN0YXJgIGlzIHRydWUuICovXG5leHBvcnQgZnVuY3Rpb24gam9pbkdsb2JzKFxuICBnbG9iczogc3RyaW5nW10sXG4gIHsgZXh0ZW5kZWQgPSB0cnVlLCBnbG9ic3RhciA9IGZhbHNlIH06IEdsb2JPcHRpb25zID0ge30sXG4pOiBzdHJpbmcge1xuICBpZiAoIWdsb2JzdGFyIHx8IGdsb2JzLmxlbmd0aCA9PT0gMCkge1xuICAgIHJldHVybiBqb2luKC4uLmdsb2JzKTtcbiAgfVxuICBpZiAoZ2xvYnMubGVuZ3RoID09PSAwKSByZXR1cm4gXCIuXCI7XG4gIGxldCBqb2luZWQ6IHN0cmluZyB8IHVuZGVmaW5lZDtcbiAgZm9yIChjb25zdCBnbG9iIG9mIGdsb2JzKSB7XG4gICAgY29uc3QgcGF0aCA9IGdsb2I7XG4gICAgaWYgKHBhdGgubGVuZ3RoID4gMCkge1xuICAgICAgaWYgKCFqb2luZWQpIGpvaW5lZCA9IHBhdGg7XG4gICAgICBlbHNlIGpvaW5lZCArPSBgJHtTRVBBUkFUT1J9JHtwYXRofWA7XG4gICAgfVxuICB9XG4gIGlmICgham9pbmVkKSByZXR1cm4gXCIuXCI7XG4gIHJldHVybiBub3JtYWxpemVHbG9iKGpvaW5lZCwgeyBleHRlbmRlZCwgZ2xvYnN0YXIgfSk7XG59XG4iXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IkFBQUEsMEVBQTBFO0FBQzFFLHFDQUFxQztBQUdyQyxTQUFTLElBQUksUUFBUSxZQUFZO0FBQ2pDLFNBQVMsU0FBUyxRQUFRLGlCQUFpQjtBQUMzQyxTQUFTLGFBQWEsUUFBUSxzQkFBc0I7QUFFcEQsd0VBQXdFLEdBQ3hFLE9BQU8sU0FBUyxVQUNkLEtBQWUsRUFDZixFQUFFLFdBQVcsSUFBSSxFQUFFLFdBQVcsS0FBSyxFQUFlLEdBQUcsQ0FBQyxDQUFDO0VBRXZELElBQUksQ0FBQyxZQUFZLE1BQU0sTUFBTSxLQUFLLEdBQUc7SUFDbkMsT0FBTyxRQUFRO0VBQ2pCO0VBQ0EsSUFBSSxNQUFNLE1BQU0sS0FBSyxHQUFHLE9BQU87RUFDL0IsSUFBSTtFQUNKLEtBQUssTUFBTSxRQUFRLE1BQU87SUFDeEIsTUFBTSxPQUFPO0lBQ2IsSUFBSSxLQUFLLE1BQU0sR0FBRyxHQUFHO01BQ25CLElBQUksQ0FBQyxRQUFRLFNBQVM7V0FDakIsVUFBVSxHQUFHLFlBQVksTUFBTTtJQUN0QztFQUNGO0VBQ0EsSUFBSSxDQUFDLFFBQVEsT0FBTztFQUNwQixPQUFPLGNBQWMsUUFBUTtJQUFFO0lBQVU7RUFBUztBQUNwRCJ9
// denoCacheMetadata=4476044238494253673,4140865094322198514