// Copyright 2018-2022 the Deno authors. All rights reserved. MIT license.
import { join } from "../path/mod.ts";
/**
 * Ensures that a directory is empty.
 * Deletes directory contents if the directory is not empty.
 * If the directory does not exist, it is created.
 * The directory itself is not deleted.
 * Requires the `--allow-read` and `--allow-write` flag.
 */ export async function emptyDir(dir) {
  try {
    const items = [];
    for await (const dirEntry of Deno.readDir(dir)){
      items.push(dirEntry);
    }
    while(items.length){
      const item = items.shift();
      if (item && item.name) {
        const filepath = join(dir, item.name);
        await Deno.remove(filepath, {
          recursive: true
        });
      }
    }
  } catch (err) {
    if (!(err instanceof Deno.errors.NotFound)) {
      throw err;
    }
    // if not exist. then create it
    await Deno.mkdir(dir, {
      recursive: true
    });
  }
}
/**
 * Ensures that a directory is empty.
 * Deletes directory contents if the directory is not empty.
 * If the directory does not exist, it is created.
 * The directory itself is not deleted.
 * Requires the `--allow-read` and `--allow-write` flag.
 */ export function emptyDirSync(dir) {
  try {
    const items = [
      ...Deno.readDirSync(dir)
    ];
    // If the directory exists, remove all entries inside it.
    while(items.length){
      const item = items.shift();
      if (item && item.name) {
        const filepath = join(dir, item.name);
        Deno.removeSync(filepath, {
          recursive: true
        });
      }
    }
  } catch (err) {
    if (!(err instanceof Deno.errors.NotFound)) {
      throw err;
    }
    // if not exist. then create it
    Deno.mkdirSync(dir, {
      recursive: true
    });
    return;
  }
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbImh0dHBzOi8vZGVuby5sYW5kL3N0ZEAwLjEzMy4wL2ZzL2VtcHR5X2Rpci50cyJdLCJzb3VyY2VzQ29udGVudCI6WyIvLyBDb3B5cmlnaHQgMjAxOC0yMDIyIHRoZSBEZW5vIGF1dGhvcnMuIEFsbCByaWdodHMgcmVzZXJ2ZWQuIE1JVCBsaWNlbnNlLlxuaW1wb3J0IHsgam9pbiB9IGZyb20gXCIuLi9wYXRoL21vZC50c1wiO1xuXG4vKipcbiAqIEVuc3VyZXMgdGhhdCBhIGRpcmVjdG9yeSBpcyBlbXB0eS5cbiAqIERlbGV0ZXMgZGlyZWN0b3J5IGNvbnRlbnRzIGlmIHRoZSBkaXJlY3RvcnkgaXMgbm90IGVtcHR5LlxuICogSWYgdGhlIGRpcmVjdG9yeSBkb2VzIG5vdCBleGlzdCwgaXQgaXMgY3JlYXRlZC5cbiAqIFRoZSBkaXJlY3RvcnkgaXRzZWxmIGlzIG5vdCBkZWxldGVkLlxuICogUmVxdWlyZXMgdGhlIGAtLWFsbG93LXJlYWRgIGFuZCBgLS1hbGxvdy13cml0ZWAgZmxhZy5cbiAqL1xuZXhwb3J0IGFzeW5jIGZ1bmN0aW9uIGVtcHR5RGlyKGRpcjogc3RyaW5nKSB7XG4gIHRyeSB7XG4gICAgY29uc3QgaXRlbXMgPSBbXTtcbiAgICBmb3IgYXdhaXQgKGNvbnN0IGRpckVudHJ5IG9mIERlbm8ucmVhZERpcihkaXIpKSB7XG4gICAgICBpdGVtcy5wdXNoKGRpckVudHJ5KTtcbiAgICB9XG5cbiAgICB3aGlsZSAoaXRlbXMubGVuZ3RoKSB7XG4gICAgICBjb25zdCBpdGVtID0gaXRlbXMuc2hpZnQoKTtcbiAgICAgIGlmIChpdGVtICYmIGl0ZW0ubmFtZSkge1xuICAgICAgICBjb25zdCBmaWxlcGF0aCA9IGpvaW4oZGlyLCBpdGVtLm5hbWUpO1xuICAgICAgICBhd2FpdCBEZW5vLnJlbW92ZShmaWxlcGF0aCwgeyByZWN1cnNpdmU6IHRydWUgfSk7XG4gICAgICB9XG4gICAgfVxuICB9IGNhdGNoIChlcnIpIHtcbiAgICBpZiAoIShlcnIgaW5zdGFuY2VvZiBEZW5vLmVycm9ycy5Ob3RGb3VuZCkpIHtcbiAgICAgIHRocm93IGVycjtcbiAgICB9XG5cbiAgICAvLyBpZiBub3QgZXhpc3QuIHRoZW4gY3JlYXRlIGl0XG4gICAgYXdhaXQgRGVuby5ta2RpcihkaXIsIHsgcmVjdXJzaXZlOiB0cnVlIH0pO1xuICB9XG59XG5cbi8qKlxuICogRW5zdXJlcyB0aGF0IGEgZGlyZWN0b3J5IGlzIGVtcHR5LlxuICogRGVsZXRlcyBkaXJlY3RvcnkgY29udGVudHMgaWYgdGhlIGRpcmVjdG9yeSBpcyBub3QgZW1wdHkuXG4gKiBJZiB0aGUgZGlyZWN0b3J5IGRvZXMgbm90IGV4aXN0LCBpdCBpcyBjcmVhdGVkLlxuICogVGhlIGRpcmVjdG9yeSBpdHNlbGYgaXMgbm90IGRlbGV0ZWQuXG4gKiBSZXF1aXJlcyB0aGUgYC0tYWxsb3ctcmVhZGAgYW5kIGAtLWFsbG93LXdyaXRlYCBmbGFnLlxuICovXG5leHBvcnQgZnVuY3Rpb24gZW1wdHlEaXJTeW5jKGRpcjogc3RyaW5nKTogdm9pZCB7XG4gIHRyeSB7XG4gICAgY29uc3QgaXRlbXMgPSBbLi4uRGVuby5yZWFkRGlyU3luYyhkaXIpXTtcblxuICAgIC8vIElmIHRoZSBkaXJlY3RvcnkgZXhpc3RzLCByZW1vdmUgYWxsIGVudHJpZXMgaW5zaWRlIGl0LlxuICAgIHdoaWxlIChpdGVtcy5sZW5ndGgpIHtcbiAgICAgIGNvbnN0IGl0ZW0gPSBpdGVtcy5zaGlmdCgpO1xuICAgICAgaWYgKGl0ZW0gJiYgaXRlbS5uYW1lKSB7XG4gICAgICAgIGNvbnN0IGZpbGVwYXRoID0gam9pbihkaXIsIGl0ZW0ubmFtZSk7XG4gICAgICAgIERlbm8ucmVtb3ZlU3luYyhmaWxlcGF0aCwgeyByZWN1cnNpdmU6IHRydWUgfSk7XG4gICAgICB9XG4gICAgfVxuICB9IGNhdGNoIChlcnIpIHtcbiAgICBpZiAoIShlcnIgaW5zdGFuY2VvZiBEZW5vLmVycm9ycy5Ob3RGb3VuZCkpIHtcbiAgICAgIHRocm93IGVycjtcbiAgICB9XG4gICAgLy8gaWYgbm90IGV4aXN0LiB0aGVuIGNyZWF0ZSBpdFxuICAgIERlbm8ubWtkaXJTeW5jKGRpciwgeyByZWN1cnNpdmU6IHRydWUgfSk7XG4gICAgcmV0dXJuO1xuICB9XG59XG4iXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IkFBQUEsMEVBQTBFO0FBQzFFLFNBQVMsSUFBSSxRQUFRLGlCQUFpQjtBQUV0Qzs7Ozs7O0NBTUMsR0FDRCxPQUFPLGVBQWUsU0FBUyxHQUFXO0VBQ3hDLElBQUk7SUFDRixNQUFNLFFBQVEsRUFBRTtJQUNoQixXQUFXLE1BQU0sWUFBWSxLQUFLLE9BQU8sQ0FBQyxLQUFNO01BQzlDLE1BQU0sSUFBSSxDQUFDO0lBQ2I7SUFFQSxNQUFPLE1BQU0sTUFBTSxDQUFFO01BQ25CLE1BQU0sT0FBTyxNQUFNLEtBQUs7TUFDeEIsSUFBSSxRQUFRLEtBQUssSUFBSSxFQUFFO1FBQ3JCLE1BQU0sV0FBVyxLQUFLLEtBQUssS0FBSyxJQUFJO1FBQ3BDLE1BQU0sS0FBSyxNQUFNLENBQUMsVUFBVTtVQUFFLFdBQVc7UUFBSztNQUNoRDtJQUNGO0VBQ0YsRUFBRSxPQUFPLEtBQUs7SUFDWixJQUFJLENBQUMsQ0FBQyxlQUFlLEtBQUssTUFBTSxDQUFDLFFBQVEsR0FBRztNQUMxQyxNQUFNO0lBQ1I7SUFFQSwrQkFBK0I7SUFDL0IsTUFBTSxLQUFLLEtBQUssQ0FBQyxLQUFLO01BQUUsV0FBVztJQUFLO0VBQzFDO0FBQ0Y7QUFFQTs7Ozs7O0NBTUMsR0FDRCxPQUFPLFNBQVMsYUFBYSxHQUFXO0VBQ3RDLElBQUk7SUFDRixNQUFNLFFBQVE7U0FBSSxLQUFLLFdBQVcsQ0FBQztLQUFLO0lBRXhDLHlEQUF5RDtJQUN6RCxNQUFPLE1BQU0sTUFBTSxDQUFFO01BQ25CLE1BQU0sT0FBTyxNQUFNLEtBQUs7TUFDeEIsSUFBSSxRQUFRLEtBQUssSUFBSSxFQUFFO1FBQ3JCLE1BQU0sV0FBVyxLQUFLLEtBQUssS0FBSyxJQUFJO1FBQ3BDLEtBQUssVUFBVSxDQUFDLFVBQVU7VUFBRSxXQUFXO1FBQUs7TUFDOUM7SUFDRjtFQUNGLEVBQUUsT0FBTyxLQUFLO0lBQ1osSUFBSSxDQUFDLENBQUMsZUFBZSxLQUFLLE1BQU0sQ0FBQyxRQUFRLEdBQUc7TUFDMUMsTUFBTTtJQUNSO0lBQ0EsK0JBQStCO0lBQy9CLEtBQUssU0FBUyxDQUFDLEtBQUs7TUFBRSxXQUFXO0lBQUs7SUFDdEM7RUFDRjtBQUNGIn0=
// denoCacheMetadata=8823204747793853789,3706445186530534910