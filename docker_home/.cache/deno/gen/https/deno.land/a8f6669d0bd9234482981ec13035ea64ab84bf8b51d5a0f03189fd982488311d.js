// Copyright 2018-2022 the Deno authors. All rights reserved. MIT license.
import { getFileInfoType } from "./_util.ts";
/**
 * Ensures that the directory exists.
 * If the directory structure does not exist, it is created. Like mkdir -p.
 * Requires the `--allow-read` and `--allow-write` flag.
 */ export async function ensureDir(dir) {
  try {
    const fileInfo = await Deno.lstat(dir);
    if (!fileInfo.isDirectory) {
      throw new Error(`Ensure path exists, expected 'dir', got '${getFileInfoType(fileInfo)}'`);
    }
  } catch (err) {
    if (err instanceof Deno.errors.NotFound) {
      // if dir not exists. then create it.
      await Deno.mkdir(dir, {
        recursive: true
      });
      return;
    }
    throw err;
  }
}
/**
 * Ensures that the directory exists.
 * If the directory structure does not exist, it is created. Like mkdir -p.
 * Requires the `--allow-read` and `--allow-write` flag.
 */ export function ensureDirSync(dir) {
  try {
    const fileInfo = Deno.lstatSync(dir);
    if (!fileInfo.isDirectory) {
      throw new Error(`Ensure path exists, expected 'dir', got '${getFileInfoType(fileInfo)}'`);
    }
  } catch (err) {
    if (err instanceof Deno.errors.NotFound) {
      // if dir not exists. then create it.
      Deno.mkdirSync(dir, {
        recursive: true
      });
      return;
    }
    throw err;
  }
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbImh0dHBzOi8vZGVuby5sYW5kL3N0ZEAwLjEzMy4wL2ZzL2Vuc3VyZV9kaXIudHMiXSwic291cmNlc0NvbnRlbnQiOlsiLy8gQ29weXJpZ2h0IDIwMTgtMjAyMiB0aGUgRGVubyBhdXRob3JzLiBBbGwgcmlnaHRzIHJlc2VydmVkLiBNSVQgbGljZW5zZS5cbmltcG9ydCB7IGdldEZpbGVJbmZvVHlwZSB9IGZyb20gXCIuL191dGlsLnRzXCI7XG5cbi8qKlxuICogRW5zdXJlcyB0aGF0IHRoZSBkaXJlY3RvcnkgZXhpc3RzLlxuICogSWYgdGhlIGRpcmVjdG9yeSBzdHJ1Y3R1cmUgZG9lcyBub3QgZXhpc3QsIGl0IGlzIGNyZWF0ZWQuIExpa2UgbWtkaXIgLXAuXG4gKiBSZXF1aXJlcyB0aGUgYC0tYWxsb3ctcmVhZGAgYW5kIGAtLWFsbG93LXdyaXRlYCBmbGFnLlxuICovXG5leHBvcnQgYXN5bmMgZnVuY3Rpb24gZW5zdXJlRGlyKGRpcjogc3RyaW5nKSB7XG4gIHRyeSB7XG4gICAgY29uc3QgZmlsZUluZm8gPSBhd2FpdCBEZW5vLmxzdGF0KGRpcik7XG4gICAgaWYgKCFmaWxlSW5mby5pc0RpcmVjdG9yeSkge1xuICAgICAgdGhyb3cgbmV3IEVycm9yKFxuICAgICAgICBgRW5zdXJlIHBhdGggZXhpc3RzLCBleHBlY3RlZCAnZGlyJywgZ290ICcke1xuICAgICAgICAgIGdldEZpbGVJbmZvVHlwZShmaWxlSW5mbylcbiAgICAgICAgfSdgLFxuICAgICAgKTtcbiAgICB9XG4gIH0gY2F0Y2ggKGVycikge1xuICAgIGlmIChlcnIgaW5zdGFuY2VvZiBEZW5vLmVycm9ycy5Ob3RGb3VuZCkge1xuICAgICAgLy8gaWYgZGlyIG5vdCBleGlzdHMuIHRoZW4gY3JlYXRlIGl0LlxuICAgICAgYXdhaXQgRGVuby5ta2RpcihkaXIsIHsgcmVjdXJzaXZlOiB0cnVlIH0pO1xuICAgICAgcmV0dXJuO1xuICAgIH1cbiAgICB0aHJvdyBlcnI7XG4gIH1cbn1cblxuLyoqXG4gKiBFbnN1cmVzIHRoYXQgdGhlIGRpcmVjdG9yeSBleGlzdHMuXG4gKiBJZiB0aGUgZGlyZWN0b3J5IHN0cnVjdHVyZSBkb2VzIG5vdCBleGlzdCwgaXQgaXMgY3JlYXRlZC4gTGlrZSBta2RpciAtcC5cbiAqIFJlcXVpcmVzIHRoZSBgLS1hbGxvdy1yZWFkYCBhbmQgYC0tYWxsb3ctd3JpdGVgIGZsYWcuXG4gKi9cbmV4cG9ydCBmdW5jdGlvbiBlbnN1cmVEaXJTeW5jKGRpcjogc3RyaW5nKTogdm9pZCB7XG4gIHRyeSB7XG4gICAgY29uc3QgZmlsZUluZm8gPSBEZW5vLmxzdGF0U3luYyhkaXIpO1xuICAgIGlmICghZmlsZUluZm8uaXNEaXJlY3RvcnkpIHtcbiAgICAgIHRocm93IG5ldyBFcnJvcihcbiAgICAgICAgYEVuc3VyZSBwYXRoIGV4aXN0cywgZXhwZWN0ZWQgJ2RpcicsIGdvdCAnJHtcbiAgICAgICAgICBnZXRGaWxlSW5mb1R5cGUoZmlsZUluZm8pXG4gICAgICAgIH0nYCxcbiAgICAgICk7XG4gICAgfVxuICB9IGNhdGNoIChlcnIpIHtcbiAgICBpZiAoZXJyIGluc3RhbmNlb2YgRGVuby5lcnJvcnMuTm90Rm91bmQpIHtcbiAgICAgIC8vIGlmIGRpciBub3QgZXhpc3RzLiB0aGVuIGNyZWF0ZSBpdC5cbiAgICAgIERlbm8ubWtkaXJTeW5jKGRpciwgeyByZWN1cnNpdmU6IHRydWUgfSk7XG4gICAgICByZXR1cm47XG4gICAgfVxuICAgIHRocm93IGVycjtcbiAgfVxufVxuIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiJBQUFBLDBFQUEwRTtBQUMxRSxTQUFTLGVBQWUsUUFBUSxhQUFhO0FBRTdDOzs7O0NBSUMsR0FDRCxPQUFPLGVBQWUsVUFBVSxHQUFXO0VBQ3pDLElBQUk7SUFDRixNQUFNLFdBQVcsTUFBTSxLQUFLLEtBQUssQ0FBQztJQUNsQyxJQUFJLENBQUMsU0FBUyxXQUFXLEVBQUU7TUFDekIsTUFBTSxJQUFJLE1BQ1IsQ0FBQyx5Q0FBeUMsRUFDeEMsZ0JBQWdCLFVBQ2pCLENBQUMsQ0FBQztJQUVQO0VBQ0YsRUFBRSxPQUFPLEtBQUs7SUFDWixJQUFJLGVBQWUsS0FBSyxNQUFNLENBQUMsUUFBUSxFQUFFO01BQ3ZDLHFDQUFxQztNQUNyQyxNQUFNLEtBQUssS0FBSyxDQUFDLEtBQUs7UUFBRSxXQUFXO01BQUs7TUFDeEM7SUFDRjtJQUNBLE1BQU07RUFDUjtBQUNGO0FBRUE7Ozs7Q0FJQyxHQUNELE9BQU8sU0FBUyxjQUFjLEdBQVc7RUFDdkMsSUFBSTtJQUNGLE1BQU0sV0FBVyxLQUFLLFNBQVMsQ0FBQztJQUNoQyxJQUFJLENBQUMsU0FBUyxXQUFXLEVBQUU7TUFDekIsTUFBTSxJQUFJLE1BQ1IsQ0FBQyx5Q0FBeUMsRUFDeEMsZ0JBQWdCLFVBQ2pCLENBQUMsQ0FBQztJQUVQO0VBQ0YsRUFBRSxPQUFPLEtBQUs7SUFDWixJQUFJLGVBQWUsS0FBSyxNQUFNLENBQUMsUUFBUSxFQUFFO01BQ3ZDLHFDQUFxQztNQUNyQyxLQUFLLFNBQVMsQ0FBQyxLQUFLO1FBQUUsV0FBVztNQUFLO01BQ3RDO0lBQ0Y7SUFDQSxNQUFNO0VBQ1I7QUFDRiJ9
// denoCacheMetadata=14206357602407234623,7613753408246105281