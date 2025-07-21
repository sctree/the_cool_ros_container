// Copyright 2018-2023 the Deno authors. All rights reserved. MIT license.
// This module is browser compatible.
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
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbImh0dHBzOi8vZGVuby5sYW5kL3N0ZEAwLjE5MS4wL2lvL3JlYWRfbGluZXMudHMiXSwic291cmNlc0NvbnRlbnQiOlsiLy8gQ29weXJpZ2h0IDIwMTgtMjAyMyB0aGUgRGVubyBhdXRob3JzLiBBbGwgcmlnaHRzIHJlc2VydmVkLiBNSVQgbGljZW5zZS5cbi8vIFRoaXMgbW9kdWxlIGlzIGJyb3dzZXIgY29tcGF0aWJsZS5cblxuaW1wb3J0IHsgdHlwZSBSZWFkZXIgfSBmcm9tIFwiLi4vdHlwZXMuZC50c1wiO1xuaW1wb3J0IHsgQnVmUmVhZGVyIH0gZnJvbSBcIi4vYnVmX3JlYWRlci50c1wiO1xuaW1wb3J0IHsgY29uY2F0IH0gZnJvbSBcIi4uL2J5dGVzL2NvbmNhdC50c1wiO1xuXG4vKipcbiAqIFJlYWQgc3RyaW5ncyBsaW5lLWJ5LWxpbmUgZnJvbSBhIFJlYWRlci5cbiAqXG4gKiAgQGV4YW1wbGVcbiAqIGBgYHRzXG4gKiBpbXBvcnQgeyByZWFkTGluZXMgfSBmcm9tIFwiaHR0cHM6Ly9kZW5vLmxhbmQvc3RkQCRTVERfVkVSU0lPTi9pby9yZWFkX2xpbmVzLnRzXCI7XG4gKiBpbXBvcnQgKiBhcyBwYXRoIGZyb20gXCJodHRwczovL2Rlbm8ubGFuZC9zdGRAJFNURF9WRVJTSU9OL3BhdGgvbW9kLnRzXCI7XG4gKlxuICogY29uc3QgZmlsZW5hbWUgPSBwYXRoLmpvaW4oRGVuby5jd2QoKSwgXCJzdGQvaW8vUkVBRE1FLm1kXCIpO1xuICogbGV0IGZpbGVSZWFkZXIgPSBhd2FpdCBEZW5vLm9wZW4oZmlsZW5hbWUpO1xuICpcbiAqIGZvciBhd2FpdCAobGV0IGxpbmUgb2YgcmVhZExpbmVzKGZpbGVSZWFkZXIpKSB7XG4gKiAgIGNvbnNvbGUubG9nKGxpbmUpO1xuICogfVxuICogYGBgXG4gKi9cbmV4cG9ydCBhc3luYyBmdW5jdGlvbiogcmVhZExpbmVzKFxuICByZWFkZXI6IFJlYWRlcixcbiAgZGVjb2Rlck9wdHM/OiB7XG4gICAgZW5jb2Rpbmc/OiBzdHJpbmc7XG4gICAgZmF0YWw/OiBib29sZWFuO1xuICAgIGlnbm9yZUJPTT86IGJvb2xlYW47XG4gIH0sXG4pOiBBc3luY0l0ZXJhYmxlSXRlcmF0b3I8c3RyaW5nPiB7XG4gIGNvbnN0IGJ1ZlJlYWRlciA9IG5ldyBCdWZSZWFkZXIocmVhZGVyKTtcbiAgbGV0IGNodW5rczogVWludDhBcnJheVtdID0gW107XG4gIGNvbnN0IGRlY29kZXIgPSBuZXcgVGV4dERlY29kZXIoZGVjb2Rlck9wdHM/LmVuY29kaW5nLCBkZWNvZGVyT3B0cyk7XG4gIHdoaWxlICh0cnVlKSB7XG4gICAgY29uc3QgcmVzID0gYXdhaXQgYnVmUmVhZGVyLnJlYWRMaW5lKCk7XG4gICAgaWYgKCFyZXMpIHtcbiAgICAgIGlmIChjaHVua3MubGVuZ3RoID4gMCkge1xuICAgICAgICB5aWVsZCBkZWNvZGVyLmRlY29kZShjb25jYXQoLi4uY2h1bmtzKSk7XG4gICAgICB9XG4gICAgICBicmVhaztcbiAgICB9XG4gICAgY2h1bmtzLnB1c2gocmVzLmxpbmUpO1xuICAgIGlmICghcmVzLm1vcmUpIHtcbiAgICAgIHlpZWxkIGRlY29kZXIuZGVjb2RlKGNvbmNhdCguLi5jaHVua3MpKTtcbiAgICAgIGNodW5rcyA9IFtdO1xuICAgIH1cbiAgfVxufVxuIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiJBQUFBLDBFQUEwRTtBQUMxRSxxQ0FBcUM7QUFHckMsU0FBUyxTQUFTLFFBQVEsa0JBQWtCO0FBQzVDLFNBQVMsTUFBTSxRQUFRLHFCQUFxQjtBQUU1Qzs7Ozs7Ozs7Ozs7Ozs7O0NBZUMsR0FDRCxPQUFPLGdCQUFnQixVQUNyQixNQUFjLEVBQ2QsV0FJQztFQUVELE1BQU0sWUFBWSxJQUFJLFVBQVU7RUFDaEMsSUFBSSxTQUF1QixFQUFFO0VBQzdCLE1BQU0sVUFBVSxJQUFJLFlBQVksYUFBYSxVQUFVO0VBQ3ZELE1BQU8sS0FBTTtJQUNYLE1BQU0sTUFBTSxNQUFNLFVBQVUsUUFBUTtJQUNwQyxJQUFJLENBQUMsS0FBSztNQUNSLElBQUksT0FBTyxNQUFNLEdBQUcsR0FBRztRQUNyQixNQUFNLFFBQVEsTUFBTSxDQUFDLFVBQVU7TUFDakM7TUFDQTtJQUNGO0lBQ0EsT0FBTyxJQUFJLENBQUMsSUFBSSxJQUFJO0lBQ3BCLElBQUksQ0FBQyxJQUFJLElBQUksRUFBRTtNQUNiLE1BQU0sUUFBUSxNQUFNLENBQUMsVUFBVTtNQUMvQixTQUFTLEVBQUU7SUFDYjtFQUNGO0FBQ0YifQ==
// denoCacheMetadata=4842244447317198466,18428882363821901506