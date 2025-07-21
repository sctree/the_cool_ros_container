// Copyright 2018-2022 the Deno authors. All rights reserved. MIT license.
import { BufReader } from "./buf_reader.ts";
import { concat } from "../bytes/concat.ts";
/**
 * Read strings line-by-line from a Reader.
 *
 *  @example
 * ```ts
 * import { readLines } from "https://deno.land/std@$STD_VERSION/io/read_lines.ts";
 * import * as path from "https://deno.land/std@$STD_VERSION/path/mod.ts";
 *
 * const filename = path.join(Deno.cwd(), "std/io/README.md");
 * let fileReader = await Deno.open(filename);
 *
 * for await (let line of readLines(fileReader)) {
 *   console.log(line);
 * }
 * ```
 */ export async function* readLines(reader, decoderOpts) {
  const bufReader = new BufReader(reader);
  let chunks = [];
  const decoder = new TextDecoder(decoderOpts?.encoding, decoderOpts);
  while(true){
    const res = await bufReader.readLine();
    if (!res) {
      if (chunks.length > 0) {
        yield decoder.decode(concat(...chunks));
      }
      break;
    }
    chunks.push(res.line);
    if (!res.more) {
      yield decoder.decode(concat(...chunks));
      chunks = [];
    }
  }
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbImh0dHBzOi8vZGVuby5sYW5kL3N0ZEAwLjE2OC4wL2lvL3JlYWRfbGluZXMudHMiXSwic291cmNlc0NvbnRlbnQiOlsiLy8gQ29weXJpZ2h0IDIwMTgtMjAyMiB0aGUgRGVubyBhdXRob3JzLiBBbGwgcmlnaHRzIHJlc2VydmVkLiBNSVQgbGljZW5zZS5cblxuaW1wb3J0IHsgdHlwZSBSZWFkZXIgfSBmcm9tIFwiLi90eXBlcy5kLnRzXCI7XG5pbXBvcnQgeyBCdWZSZWFkZXIgfSBmcm9tIFwiLi9idWZfcmVhZGVyLnRzXCI7XG5pbXBvcnQgeyBjb25jYXQgfSBmcm9tIFwiLi4vYnl0ZXMvY29uY2F0LnRzXCI7XG5cbi8qKlxuICogUmVhZCBzdHJpbmdzIGxpbmUtYnktbGluZSBmcm9tIGEgUmVhZGVyLlxuICpcbiAqICBAZXhhbXBsZVxuICogYGBgdHNcbiAqIGltcG9ydCB7IHJlYWRMaW5lcyB9IGZyb20gXCJodHRwczovL2Rlbm8ubGFuZC9zdGRAJFNURF9WRVJTSU9OL2lvL3JlYWRfbGluZXMudHNcIjtcbiAqIGltcG9ydCAqIGFzIHBhdGggZnJvbSBcImh0dHBzOi8vZGVuby5sYW5kL3N0ZEAkU1REX1ZFUlNJT04vcGF0aC9tb2QudHNcIjtcbiAqXG4gKiBjb25zdCBmaWxlbmFtZSA9IHBhdGguam9pbihEZW5vLmN3ZCgpLCBcInN0ZC9pby9SRUFETUUubWRcIik7XG4gKiBsZXQgZmlsZVJlYWRlciA9IGF3YWl0IERlbm8ub3BlbihmaWxlbmFtZSk7XG4gKlxuICogZm9yIGF3YWl0IChsZXQgbGluZSBvZiByZWFkTGluZXMoZmlsZVJlYWRlcikpIHtcbiAqICAgY29uc29sZS5sb2cobGluZSk7XG4gKiB9XG4gKiBgYGBcbiAqL1xuZXhwb3J0IGFzeW5jIGZ1bmN0aW9uKiByZWFkTGluZXMoXG4gIHJlYWRlcjogUmVhZGVyLFxuICBkZWNvZGVyT3B0cz86IHtcbiAgICBlbmNvZGluZz86IHN0cmluZztcbiAgICBmYXRhbD86IGJvb2xlYW47XG4gICAgaWdub3JlQk9NPzogYm9vbGVhbjtcbiAgfSxcbik6IEFzeW5jSXRlcmFibGVJdGVyYXRvcjxzdHJpbmc+IHtcbiAgY29uc3QgYnVmUmVhZGVyID0gbmV3IEJ1ZlJlYWRlcihyZWFkZXIpO1xuICBsZXQgY2h1bmtzOiBVaW50OEFycmF5W10gPSBbXTtcbiAgY29uc3QgZGVjb2RlciA9IG5ldyBUZXh0RGVjb2RlcihkZWNvZGVyT3B0cz8uZW5jb2RpbmcsIGRlY29kZXJPcHRzKTtcbiAgd2hpbGUgKHRydWUpIHtcbiAgICBjb25zdCByZXMgPSBhd2FpdCBidWZSZWFkZXIucmVhZExpbmUoKTtcbiAgICBpZiAoIXJlcykge1xuICAgICAgaWYgKGNodW5rcy5sZW5ndGggPiAwKSB7XG4gICAgICAgIHlpZWxkIGRlY29kZXIuZGVjb2RlKGNvbmNhdCguLi5jaHVua3MpKTtcbiAgICAgIH1cbiAgICAgIGJyZWFrO1xuICAgIH1cbiAgICBjaHVua3MucHVzaChyZXMubGluZSk7XG4gICAgaWYgKCFyZXMubW9yZSkge1xuICAgICAgeWllbGQgZGVjb2Rlci5kZWNvZGUoY29uY2F0KC4uLmNodW5rcykpO1xuICAgICAgY2h1bmtzID0gW107XG4gICAgfVxuICB9XG59XG4iXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IkFBQUEsMEVBQTBFO0FBRzFFLFNBQVMsU0FBUyxRQUFRLGtCQUFrQjtBQUM1QyxTQUFTLE1BQU0sUUFBUSxxQkFBcUI7QUFFNUM7Ozs7Ozs7Ozs7Ozs7OztDQWVDLEdBQ0QsT0FBTyxnQkFBZ0IsVUFDckIsTUFBYyxFQUNkLFdBSUM7RUFFRCxNQUFNLFlBQVksSUFBSSxVQUFVO0VBQ2hDLElBQUksU0FBdUIsRUFBRTtFQUM3QixNQUFNLFVBQVUsSUFBSSxZQUFZLGFBQWEsVUFBVTtFQUN2RCxNQUFPLEtBQU07SUFDWCxNQUFNLE1BQU0sTUFBTSxVQUFVLFFBQVE7SUFDcEMsSUFBSSxDQUFDLEtBQUs7TUFDUixJQUFJLE9BQU8sTUFBTSxHQUFHLEdBQUc7UUFDckIsTUFBTSxRQUFRLE1BQU0sQ0FBQyxVQUFVO01BQ2pDO01BQ0E7SUFDRjtJQUNBLE9BQU8sSUFBSSxDQUFDLElBQUksSUFBSTtJQUNwQixJQUFJLENBQUMsSUFBSSxJQUFJLEVBQUU7TUFDYixNQUFNLFFBQVEsTUFBTSxDQUFDLFVBQVU7TUFDL0IsU0FBUyxFQUFFO0lBQ2I7RUFDRjtBQUNGIn0=
// denoCacheMetadata=14824451032214483892,8340396209732573440