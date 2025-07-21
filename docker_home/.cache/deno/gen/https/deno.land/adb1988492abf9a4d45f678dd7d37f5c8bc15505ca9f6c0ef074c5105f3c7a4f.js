// Copyright 2018-2022 the Deno authors. All rights reserved. MIT license.
import { BytesList } from "../bytes/bytes_list.ts";
/** Generate longest proper prefix which is also suffix array. */ function createLPS(pat) {
  const lps = new Uint8Array(pat.length);
  lps[0] = 0;
  let prefixEnd = 0;
  let i = 1;
  while(i < lps.length){
    if (pat[i] == pat[prefixEnd]) {
      prefixEnd++;
      lps[i] = prefixEnd;
      i++;
    } else if (prefixEnd === 0) {
      lps[i] = 0;
      i++;
    } else {
      prefixEnd = lps[prefixEnd - 1];
    }
  }
  return lps;
}
/** Read delimited bytes from a Reader. */ export async function* readDelim(reader, delim) {
  // Avoid unicode problems
  const delimLen = delim.length;
  const delimLPS = createLPS(delim);
  const chunks = new BytesList();
  const bufSize = Math.max(1024, delimLen + 1);
  // Modified KMP
  let inspectIndex = 0;
  let matchIndex = 0;
  while(true){
    const inspectArr = new Uint8Array(bufSize);
    const result = await reader.read(inspectArr);
    if (result === null) {
      // Yield last chunk.
      yield chunks.concat();
      return;
    } else if (result < 0) {
      // Discard all remaining and silently fail.
      return;
    }
    chunks.add(inspectArr, 0, result);
    let localIndex = 0;
    while(inspectIndex < chunks.size()){
      if (inspectArr[localIndex] === delim[matchIndex]) {
        inspectIndex++;
        localIndex++;
        matchIndex++;
        if (matchIndex === delimLen) {
          // Full match
          const matchEnd = inspectIndex - delimLen;
          const readyBytes = chunks.slice(0, matchEnd);
          yield readyBytes;
          // Reset match, different from KMP.
          chunks.shift(inspectIndex);
          inspectIndex = 0;
          matchIndex = 0;
        }
      } else {
        if (matchIndex === 0) {
          inspectIndex++;
          localIndex++;
        } else {
          matchIndex = delimLPS[matchIndex - 1];
        }
      }
    }
  }
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbImh0dHBzOi8vZGVuby5sYW5kL3N0ZEAwLjE2OC4wL2lvL3JlYWRfZGVsaW0udHMiXSwic291cmNlc0NvbnRlbnQiOlsiLy8gQ29weXJpZ2h0IDIwMTgtMjAyMiB0aGUgRGVubyBhdXRob3JzLiBBbGwgcmlnaHRzIHJlc2VydmVkLiBNSVQgbGljZW5zZS5cblxuaW1wb3J0IHsgQnl0ZXNMaXN0IH0gZnJvbSBcIi4uL2J5dGVzL2J5dGVzX2xpc3QudHNcIjtcbmltcG9ydCB0eXBlIHsgUmVhZGVyIH0gZnJvbSBcIi4vdHlwZXMuZC50c1wiO1xuXG4vKiogR2VuZXJhdGUgbG9uZ2VzdCBwcm9wZXIgcHJlZml4IHdoaWNoIGlzIGFsc28gc3VmZml4IGFycmF5LiAqL1xuZnVuY3Rpb24gY3JlYXRlTFBTKHBhdDogVWludDhBcnJheSk6IFVpbnQ4QXJyYXkge1xuICBjb25zdCBscHMgPSBuZXcgVWludDhBcnJheShwYXQubGVuZ3RoKTtcbiAgbHBzWzBdID0gMDtcbiAgbGV0IHByZWZpeEVuZCA9IDA7XG4gIGxldCBpID0gMTtcbiAgd2hpbGUgKGkgPCBscHMubGVuZ3RoKSB7XG4gICAgaWYgKHBhdFtpXSA9PSBwYXRbcHJlZml4RW5kXSkge1xuICAgICAgcHJlZml4RW5kKys7XG4gICAgICBscHNbaV0gPSBwcmVmaXhFbmQ7XG4gICAgICBpKys7XG4gICAgfSBlbHNlIGlmIChwcmVmaXhFbmQgPT09IDApIHtcbiAgICAgIGxwc1tpXSA9IDA7XG4gICAgICBpKys7XG4gICAgfSBlbHNlIHtcbiAgICAgIHByZWZpeEVuZCA9IGxwc1twcmVmaXhFbmQgLSAxXTtcbiAgICB9XG4gIH1cbiAgcmV0dXJuIGxwcztcbn1cblxuLyoqIFJlYWQgZGVsaW1pdGVkIGJ5dGVzIGZyb20gYSBSZWFkZXIuICovXG5leHBvcnQgYXN5bmMgZnVuY3Rpb24qIHJlYWREZWxpbShcbiAgcmVhZGVyOiBSZWFkZXIsXG4gIGRlbGltOiBVaW50OEFycmF5LFxuKTogQXN5bmNJdGVyYWJsZUl0ZXJhdG9yPFVpbnQ4QXJyYXk+IHtcbiAgLy8gQXZvaWQgdW5pY29kZSBwcm9ibGVtc1xuICBjb25zdCBkZWxpbUxlbiA9IGRlbGltLmxlbmd0aDtcbiAgY29uc3QgZGVsaW1MUFMgPSBjcmVhdGVMUFMoZGVsaW0pO1xuICBjb25zdCBjaHVua3MgPSBuZXcgQnl0ZXNMaXN0KCk7XG4gIGNvbnN0IGJ1ZlNpemUgPSBNYXRoLm1heCgxMDI0LCBkZWxpbUxlbiArIDEpO1xuXG4gIC8vIE1vZGlmaWVkIEtNUFxuICBsZXQgaW5zcGVjdEluZGV4ID0gMDtcbiAgbGV0IG1hdGNoSW5kZXggPSAwO1xuICB3aGlsZSAodHJ1ZSkge1xuICAgIGNvbnN0IGluc3BlY3RBcnIgPSBuZXcgVWludDhBcnJheShidWZTaXplKTtcbiAgICBjb25zdCByZXN1bHQgPSBhd2FpdCByZWFkZXIucmVhZChpbnNwZWN0QXJyKTtcbiAgICBpZiAocmVzdWx0ID09PSBudWxsKSB7XG4gICAgICAvLyBZaWVsZCBsYXN0IGNodW5rLlxuICAgICAgeWllbGQgY2h1bmtzLmNvbmNhdCgpO1xuICAgICAgcmV0dXJuO1xuICAgIH0gZWxzZSBpZiAocmVzdWx0IDwgMCkge1xuICAgICAgLy8gRGlzY2FyZCBhbGwgcmVtYWluaW5nIGFuZCBzaWxlbnRseSBmYWlsLlxuICAgICAgcmV0dXJuO1xuICAgIH1cbiAgICBjaHVua3MuYWRkKGluc3BlY3RBcnIsIDAsIHJlc3VsdCk7XG4gICAgbGV0IGxvY2FsSW5kZXggPSAwO1xuICAgIHdoaWxlIChpbnNwZWN0SW5kZXggPCBjaHVua3Muc2l6ZSgpKSB7XG4gICAgICBpZiAoaW5zcGVjdEFycltsb2NhbEluZGV4XSA9PT0gZGVsaW1bbWF0Y2hJbmRleF0pIHtcbiAgICAgICAgaW5zcGVjdEluZGV4Kys7XG4gICAgICAgIGxvY2FsSW5kZXgrKztcbiAgICAgICAgbWF0Y2hJbmRleCsrO1xuICAgICAgICBpZiAobWF0Y2hJbmRleCA9PT0gZGVsaW1MZW4pIHtcbiAgICAgICAgICAvLyBGdWxsIG1hdGNoXG4gICAgICAgICAgY29uc3QgbWF0Y2hFbmQgPSBpbnNwZWN0SW5kZXggLSBkZWxpbUxlbjtcbiAgICAgICAgICBjb25zdCByZWFkeUJ5dGVzID0gY2h1bmtzLnNsaWNlKDAsIG1hdGNoRW5kKTtcbiAgICAgICAgICB5aWVsZCByZWFkeUJ5dGVzO1xuICAgICAgICAgIC8vIFJlc2V0IG1hdGNoLCBkaWZmZXJlbnQgZnJvbSBLTVAuXG4gICAgICAgICAgY2h1bmtzLnNoaWZ0KGluc3BlY3RJbmRleCk7XG4gICAgICAgICAgaW5zcGVjdEluZGV4ID0gMDtcbiAgICAgICAgICBtYXRjaEluZGV4ID0gMDtcbiAgICAgICAgfVxuICAgICAgfSBlbHNlIHtcbiAgICAgICAgaWYgKG1hdGNoSW5kZXggPT09IDApIHtcbiAgICAgICAgICBpbnNwZWN0SW5kZXgrKztcbiAgICAgICAgICBsb2NhbEluZGV4Kys7XG4gICAgICAgIH0gZWxzZSB7XG4gICAgICAgICAgbWF0Y2hJbmRleCA9IGRlbGltTFBTW21hdGNoSW5kZXggLSAxXTtcbiAgICAgICAgfVxuICAgICAgfVxuICAgIH1cbiAgfVxufVxuIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiJBQUFBLDBFQUEwRTtBQUUxRSxTQUFTLFNBQVMsUUFBUSx5QkFBeUI7QUFHbkQsK0RBQStELEdBQy9ELFNBQVMsVUFBVSxHQUFlO0VBQ2hDLE1BQU0sTUFBTSxJQUFJLFdBQVcsSUFBSSxNQUFNO0VBQ3JDLEdBQUcsQ0FBQyxFQUFFLEdBQUc7RUFDVCxJQUFJLFlBQVk7RUFDaEIsSUFBSSxJQUFJO0VBQ1IsTUFBTyxJQUFJLElBQUksTUFBTSxDQUFFO0lBQ3JCLElBQUksR0FBRyxDQUFDLEVBQUUsSUFBSSxHQUFHLENBQUMsVUFBVSxFQUFFO01BQzVCO01BQ0EsR0FBRyxDQUFDLEVBQUUsR0FBRztNQUNUO0lBQ0YsT0FBTyxJQUFJLGNBQWMsR0FBRztNQUMxQixHQUFHLENBQUMsRUFBRSxHQUFHO01BQ1Q7SUFDRixPQUFPO01BQ0wsWUFBWSxHQUFHLENBQUMsWUFBWSxFQUFFO0lBQ2hDO0VBQ0Y7RUFDQSxPQUFPO0FBQ1Q7QUFFQSx3Q0FBd0MsR0FDeEMsT0FBTyxnQkFBZ0IsVUFDckIsTUFBYyxFQUNkLEtBQWlCO0VBRWpCLHlCQUF5QjtFQUN6QixNQUFNLFdBQVcsTUFBTSxNQUFNO0VBQzdCLE1BQU0sV0FBVyxVQUFVO0VBQzNCLE1BQU0sU0FBUyxJQUFJO0VBQ25CLE1BQU0sVUFBVSxLQUFLLEdBQUcsQ0FBQyxNQUFNLFdBQVc7RUFFMUMsZUFBZTtFQUNmLElBQUksZUFBZTtFQUNuQixJQUFJLGFBQWE7RUFDakIsTUFBTyxLQUFNO0lBQ1gsTUFBTSxhQUFhLElBQUksV0FBVztJQUNsQyxNQUFNLFNBQVMsTUFBTSxPQUFPLElBQUksQ0FBQztJQUNqQyxJQUFJLFdBQVcsTUFBTTtNQUNuQixvQkFBb0I7TUFDcEIsTUFBTSxPQUFPLE1BQU07TUFDbkI7SUFDRixPQUFPLElBQUksU0FBUyxHQUFHO01BQ3JCLDJDQUEyQztNQUMzQztJQUNGO0lBQ0EsT0FBTyxHQUFHLENBQUMsWUFBWSxHQUFHO0lBQzFCLElBQUksYUFBYTtJQUNqQixNQUFPLGVBQWUsT0FBTyxJQUFJLEdBQUk7TUFDbkMsSUFBSSxVQUFVLENBQUMsV0FBVyxLQUFLLEtBQUssQ0FBQyxXQUFXLEVBQUU7UUFDaEQ7UUFDQTtRQUNBO1FBQ0EsSUFBSSxlQUFlLFVBQVU7VUFDM0IsYUFBYTtVQUNiLE1BQU0sV0FBVyxlQUFlO1VBQ2hDLE1BQU0sYUFBYSxPQUFPLEtBQUssQ0FBQyxHQUFHO1VBQ25DLE1BQU07VUFDTixtQ0FBbUM7VUFDbkMsT0FBTyxLQUFLLENBQUM7VUFDYixlQUFlO1VBQ2YsYUFBYTtRQUNmO01BQ0YsT0FBTztRQUNMLElBQUksZUFBZSxHQUFHO1VBQ3BCO1VBQ0E7UUFDRixPQUFPO1VBQ0wsYUFBYSxRQUFRLENBQUMsYUFBYSxFQUFFO1FBQ3ZDO01BQ0Y7SUFDRjtFQUNGO0FBQ0YifQ==
// denoCacheMetadata=3503253023903973849,2970496185526259938