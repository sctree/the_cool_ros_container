// Copyright 2018-2022 the Deno authors. All rights reserved. MIT license.
import { readDelim } from "./read_delim.ts";
/**
 * Read Reader chunk by chunk, splitting based on delimiter.
 *
 * @example
 * ```ts
 * import { readStringDelim } from "https://deno.land/std@$STD_VERSION/io/read_string_delim.ts";
 * import * as path from "https://deno.land/std@$STD_VERSION/path/mod.ts";
 *
 * const filename = path.join(Deno.cwd(), "std/io/README.md");
 * let fileReader = await Deno.open(filename);
 *
 * for await (let line of readStringDelim(fileReader, "\n")) {
 *   console.log(line);
 * }
 * ```
 */ export async function* readStringDelim(reader, delim, decoderOpts) {
  const encoder = new TextEncoder();
  const decoder = new TextDecoder(decoderOpts?.encoding, decoderOpts);
  for await (const chunk of readDelim(reader, encoder.encode(delim))){
    yield decoder.decode(chunk);
  }
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbImh0dHBzOi8vZGVuby5sYW5kL3N0ZEAwLjE2OC4wL2lvL3JlYWRfc3RyaW5nX2RlbGltLnRzIl0sInNvdXJjZXNDb250ZW50IjpbIi8vIENvcHlyaWdodCAyMDE4LTIwMjIgdGhlIERlbm8gYXV0aG9ycy4gQWxsIHJpZ2h0cyByZXNlcnZlZC4gTUlUIGxpY2Vuc2UuXG5cbmltcG9ydCB7IHR5cGUgUmVhZGVyIH0gZnJvbSBcIi4vdHlwZXMuZC50c1wiO1xuaW1wb3J0IHsgcmVhZERlbGltIH0gZnJvbSBcIi4vcmVhZF9kZWxpbS50c1wiO1xuXG4vKipcbiAqIFJlYWQgUmVhZGVyIGNodW5rIGJ5IGNodW5rLCBzcGxpdHRpbmcgYmFzZWQgb24gZGVsaW1pdGVyLlxuICpcbiAqIEBleGFtcGxlXG4gKiBgYGB0c1xuICogaW1wb3J0IHsgcmVhZFN0cmluZ0RlbGltIH0gZnJvbSBcImh0dHBzOi8vZGVuby5sYW5kL3N0ZEAkU1REX1ZFUlNJT04vaW8vcmVhZF9zdHJpbmdfZGVsaW0udHNcIjtcbiAqIGltcG9ydCAqIGFzIHBhdGggZnJvbSBcImh0dHBzOi8vZGVuby5sYW5kL3N0ZEAkU1REX1ZFUlNJT04vcGF0aC9tb2QudHNcIjtcbiAqXG4gKiBjb25zdCBmaWxlbmFtZSA9IHBhdGguam9pbihEZW5vLmN3ZCgpLCBcInN0ZC9pby9SRUFETUUubWRcIik7XG4gKiBsZXQgZmlsZVJlYWRlciA9IGF3YWl0IERlbm8ub3BlbihmaWxlbmFtZSk7XG4gKlxuICogZm9yIGF3YWl0IChsZXQgbGluZSBvZiByZWFkU3RyaW5nRGVsaW0oZmlsZVJlYWRlciwgXCJcXG5cIikpIHtcbiAqICAgY29uc29sZS5sb2cobGluZSk7XG4gKiB9XG4gKiBgYGBcbiAqL1xuZXhwb3J0IGFzeW5jIGZ1bmN0aW9uKiByZWFkU3RyaW5nRGVsaW0oXG4gIHJlYWRlcjogUmVhZGVyLFxuICBkZWxpbTogc3RyaW5nLFxuICBkZWNvZGVyT3B0cz86IHtcbiAgICBlbmNvZGluZz86IHN0cmluZztcbiAgICBmYXRhbD86IGJvb2xlYW47XG4gICAgaWdub3JlQk9NPzogYm9vbGVhbjtcbiAgfSxcbik6IEFzeW5jSXRlcmFibGVJdGVyYXRvcjxzdHJpbmc+IHtcbiAgY29uc3QgZW5jb2RlciA9IG5ldyBUZXh0RW5jb2RlcigpO1xuICBjb25zdCBkZWNvZGVyID0gbmV3IFRleHREZWNvZGVyKGRlY29kZXJPcHRzPy5lbmNvZGluZywgZGVjb2Rlck9wdHMpO1xuICBmb3IgYXdhaXQgKGNvbnN0IGNodW5rIG9mIHJlYWREZWxpbShyZWFkZXIsIGVuY29kZXIuZW5jb2RlKGRlbGltKSkpIHtcbiAgICB5aWVsZCBkZWNvZGVyLmRlY29kZShjaHVuayk7XG4gIH1cbn1cbiJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQSwwRUFBMEU7QUFHMUUsU0FBUyxTQUFTLFFBQVEsa0JBQWtCO0FBRTVDOzs7Ozs7Ozs7Ozs7Ozs7Q0FlQyxHQUNELE9BQU8sZ0JBQWdCLGdCQUNyQixNQUFjLEVBQ2QsS0FBYSxFQUNiLFdBSUM7RUFFRCxNQUFNLFVBQVUsSUFBSTtFQUNwQixNQUFNLFVBQVUsSUFBSSxZQUFZLGFBQWEsVUFBVTtFQUN2RCxXQUFXLE1BQU0sU0FBUyxVQUFVLFFBQVEsUUFBUSxNQUFNLENBQUMsUUFBUztJQUNsRSxNQUFNLFFBQVEsTUFBTSxDQUFDO0VBQ3ZCO0FBQ0YifQ==
// denoCacheMetadata=18170205023140767059,7284476868601766741