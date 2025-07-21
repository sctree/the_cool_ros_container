import { compare, add as addTime } from "https://esm.sh/@foxglove/rostime";
import Heap from "https://esm.sh/heap";
import { BaseIterator } from "./BaseIterator.ts";
export class ForwardIterator extends BaseIterator {
  remainingChunkInfos;
  constructor(args){
    // Sort by smallest timestamp first
    super(args, (a, b)=>{
      return compare(a.time, b.time);
    });
    // These are all chunks that we can consider for iteration.
    // Only consider chunks with an endTime after or equal to our position.
    // Chunks before our position are not part of forward iteration.
    this.chunkInfos = this.chunkInfos.filter((info)=>{
      return compare(info.endTime, this.position) >= 0;
    });
    // The chunk info heap sorts chunk infos by increasing start time
    const chunkInfoHeap = new Heap((a, b)=>{
      return compare(a.startTime, b.startTime);
    });
    for (const info of this.chunkInfos){
      chunkInfoHeap.insert(info);
    }
    this.remainingChunkInfos = [];
    while(chunkInfoHeap.size() > 0){
      this.remainingChunkInfos.push(chunkInfoHeap.pop());
    }
  }
  async loadNext() {
    const stamp = this.position;
    const firstChunkInfo = this.remainingChunkInfos[0];
    if (!firstChunkInfo) {
      return false;
    }
    this.remainingChunkInfos[0] = undefined;
    let end = firstChunkInfo.endTime;
    const chunksToLoad = [
      firstChunkInfo
    ];
    for(let idx = 1; idx < this.remainingChunkInfos.length; ++idx){
      const nextChunkInfo = this.remainingChunkInfos[idx];
      if (!nextChunkInfo) {
        continue;
      }
      // The chunk starts after our selected end time, we end chunk selection
      if (compare(nextChunkInfo.startTime, end) > 0) {
        break;
      }
      // The chunk starts after our start, but before end so we will load it.
      chunksToLoad.push(nextChunkInfo);
      // If the chunk ends before or at the end time, we have fully consumed it.
      // Remove it from the remainingChunkInfos.
      const endCompare = compare(nextChunkInfo.endTime, end);
      if (endCompare <= 0) {
        this.remainingChunkInfos[idx] = undefined;
      }
    }
    // filter out undefined chunk infos
    this.remainingChunkInfos = this.remainingChunkInfos.filter(Boolean);
    // End of file or no more candidates
    if (chunksToLoad.length === 0) {
      return false;
    }
    // Add 1 nsec to make end 1 past the end for the next read
    this.position = end = addTime(end, {
      sec: 0,
      nsec: 1
    });
    const heap = this.heap;
    const newCache = new Map();
    for (const chunkInfo of chunksToLoad){
      let result = this.cachedChunkReadResults.get(chunkInfo.chunkPosition);
      if (!result) {
        result = await this.reader.readChunk(chunkInfo, this.decompress);
      }
      // Keep chunk read results for chunks where end is in the chunk
      // End is the next position we will read so we don't need to re-read the chunk
      if (compare(chunkInfo.startTime, end) <= 0 && compare(chunkInfo.endTime, end) >= 0) {
        newCache.set(chunkInfo.chunkPosition, result);
      }
      for (const indexData of result.indices){
        if (this.connectionIds && !this.connectionIds.has(indexData.conn)) {
          continue;
        }
        for (const indexEntry of indexData.indices ?? []){
          // ensure: stamp <= entry time < end
          if (compare(indexEntry.time, stamp) < 0 || compare(indexEntry.time, end) >= 0) {
            continue;
          }
          heap.push({
            time: indexEntry.time,
            offset: indexEntry.offset,
            chunkReadResult: result
          });
        }
      }
    }
    this.cachedChunkReadResults = newCache;
    return true;
  }
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbImh0dHBzOi8vcmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbS9qZWZmLWh5a2luL3JhcGlkX3Jvc19zZXJ2ZXIvZGV2L3N1YnJlcG9zL2ZveGdsb3ZlX3Jvc2JhZy9zcmMvRm9yd2FyZEl0ZXJhdG9yLnRzIl0sInNvdXJjZXNDb250ZW50IjpbImltcG9ydCB7IGNvbXBhcmUsIGFkZCBhcyBhZGRUaW1lIH0gZnJvbSBcImh0dHBzOi8vZXNtLnNoL0Bmb3hnbG92ZS9yb3N0aW1lXCI7XG5pbXBvcnQgSGVhcCBmcm9tIFwiaHR0cHM6Ly9lc20uc2gvaGVhcFwiIC8qIENIRUNLTUU6IGZpbGUocykgZGlkbid0IGV4aXN0LCBhc3N1bWluZyBucG0gKi87XG5cbmltcG9ydCB7IEJhc2VJdGVyYXRvciB9IGZyb20gXCIuL0Jhc2VJdGVyYXRvci50c1wiO1xuaW1wb3J0IHsgQ2h1bmtJbmZvIH0gZnJvbSBcIi4vcmVjb3JkLnRzXCI7XG5pbXBvcnQgeyBJdGVyYXRvckNvbnN0cnVjdG9yQXJncywgQ2h1bmtSZWFkUmVzdWx0IH0gZnJvbSBcIi4vdHlwZXMudHNcIjtcblxuZXhwb3J0IGNsYXNzIEZvcndhcmRJdGVyYXRvciBleHRlbmRzIEJhc2VJdGVyYXRvciB7XG4gIHByaXZhdGUgcmVtYWluaW5nQ2h1bmtJbmZvczogKENodW5rSW5mbyB8IHVuZGVmaW5lZClbXTtcblxuICBjb25zdHJ1Y3RvcihhcmdzOiBJdGVyYXRvckNvbnN0cnVjdG9yQXJncykge1xuICAgIC8vIFNvcnQgYnkgc21hbGxlc3QgdGltZXN0YW1wIGZpcnN0XG4gICAgc3VwZXIoYXJncywgKGEsIGIpID0+IHtcbiAgICAgIHJldHVybiBjb21wYXJlKGEudGltZSwgYi50aW1lKTtcbiAgICB9KTtcblxuICAgIC8vIFRoZXNlIGFyZSBhbGwgY2h1bmtzIHRoYXQgd2UgY2FuIGNvbnNpZGVyIGZvciBpdGVyYXRpb24uXG4gICAgLy8gT25seSBjb25zaWRlciBjaHVua3Mgd2l0aCBhbiBlbmRUaW1lIGFmdGVyIG9yIGVxdWFsIHRvIG91ciBwb3NpdGlvbi5cbiAgICAvLyBDaHVua3MgYmVmb3JlIG91ciBwb3NpdGlvbiBhcmUgbm90IHBhcnQgb2YgZm9yd2FyZCBpdGVyYXRpb24uXG4gICAgdGhpcy5jaHVua0luZm9zID0gdGhpcy5jaHVua0luZm9zLmZpbHRlcigoaW5mbykgPT4ge1xuICAgICAgcmV0dXJuIGNvbXBhcmUoaW5mby5lbmRUaW1lLCB0aGlzLnBvc2l0aW9uKSA+PSAwO1xuICAgIH0pO1xuXG4gICAgLy8gVGhlIGNodW5rIGluZm8gaGVhcCBzb3J0cyBjaHVuayBpbmZvcyBieSBpbmNyZWFzaW5nIHN0YXJ0IHRpbWVcbiAgICBjb25zdCBjaHVua0luZm9IZWFwID0gbmV3IEhlYXA8Q2h1bmtJbmZvPigoYSwgYikgPT4ge1xuICAgICAgcmV0dXJuIGNvbXBhcmUoYS5zdGFydFRpbWUsIGIuc3RhcnRUaW1lKTtcbiAgICB9KTtcblxuICAgIGZvciAoY29uc3QgaW5mbyBvZiB0aGlzLmNodW5rSW5mb3MpIHtcbiAgICAgIGNodW5rSW5mb0hlYXAuaW5zZXJ0KGluZm8pO1xuICAgIH1cblxuICAgIHRoaXMucmVtYWluaW5nQ2h1bmtJbmZvcyA9IFtdO1xuICAgIHdoaWxlIChjaHVua0luZm9IZWFwLnNpemUoKSA+IDApIHtcbiAgICAgIHRoaXMucmVtYWluaW5nQ2h1bmtJbmZvcy5wdXNoKGNodW5rSW5mb0hlYXAucG9wKCkpO1xuICAgIH1cbiAgfVxuXG4gIHByb3RlY3RlZCBvdmVycmlkZSBhc3luYyBsb2FkTmV4dCgpOiBQcm9taXNlPGJvb2xlYW4+IHtcbiAgICBjb25zdCBzdGFtcCA9IHRoaXMucG9zaXRpb247XG5cbiAgICBjb25zdCBmaXJzdENodW5rSW5mbyA9IHRoaXMucmVtYWluaW5nQ2h1bmtJbmZvc1swXTtcbiAgICBpZiAoIWZpcnN0Q2h1bmtJbmZvKSB7XG4gICAgICByZXR1cm4gZmFsc2U7XG4gICAgfVxuXG4gICAgdGhpcy5yZW1haW5pbmdDaHVua0luZm9zWzBdID0gdW5kZWZpbmVkO1xuXG4gICAgbGV0IGVuZCA9IGZpcnN0Q2h1bmtJbmZvLmVuZFRpbWU7XG4gICAgY29uc3QgY2h1bmtzVG9Mb2FkOiBDaHVua0luZm9bXSA9IFtmaXJzdENodW5rSW5mb107XG5cbiAgICBmb3IgKGxldCBpZHggPSAxOyBpZHggPCB0aGlzLnJlbWFpbmluZ0NodW5rSW5mb3MubGVuZ3RoOyArK2lkeCkge1xuICAgICAgY29uc3QgbmV4dENodW5rSW5mbyA9IHRoaXMucmVtYWluaW5nQ2h1bmtJbmZvc1tpZHhdO1xuICAgICAgaWYgKCFuZXh0Q2h1bmtJbmZvKSB7XG4gICAgICAgIGNvbnRpbnVlO1xuICAgICAgfVxuXG4gICAgICAvLyBUaGUgY2h1bmsgc3RhcnRzIGFmdGVyIG91ciBzZWxlY3RlZCBlbmQgdGltZSwgd2UgZW5kIGNodW5rIHNlbGVjdGlvblxuICAgICAgaWYgKGNvbXBhcmUobmV4dENodW5rSW5mby5zdGFydFRpbWUsIGVuZCkgPiAwKSB7XG4gICAgICAgIGJyZWFrO1xuICAgICAgfVxuXG4gICAgICAvLyBUaGUgY2h1bmsgc3RhcnRzIGFmdGVyIG91ciBzdGFydCwgYnV0IGJlZm9yZSBlbmQgc28gd2Ugd2lsbCBsb2FkIGl0LlxuICAgICAgY2h1bmtzVG9Mb2FkLnB1c2gobmV4dENodW5rSW5mbyk7XG5cbiAgICAgIC8vIElmIHRoZSBjaHVuayBlbmRzIGJlZm9yZSBvciBhdCB0aGUgZW5kIHRpbWUsIHdlIGhhdmUgZnVsbHkgY29uc3VtZWQgaXQuXG4gICAgICAvLyBSZW1vdmUgaXQgZnJvbSB0aGUgcmVtYWluaW5nQ2h1bmtJbmZvcy5cbiAgICAgIGNvbnN0IGVuZENvbXBhcmUgPSBjb21wYXJlKG5leHRDaHVua0luZm8uZW5kVGltZSwgZW5kKTtcbiAgICAgIGlmIChlbmRDb21wYXJlIDw9IDApIHtcbiAgICAgICAgdGhpcy5yZW1haW5pbmdDaHVua0luZm9zW2lkeF0gPSB1bmRlZmluZWQ7XG4gICAgICB9XG4gICAgfVxuXG4gICAgLy8gZmlsdGVyIG91dCB1bmRlZmluZWQgY2h1bmsgaW5mb3NcbiAgICB0aGlzLnJlbWFpbmluZ0NodW5rSW5mb3MgPSB0aGlzLnJlbWFpbmluZ0NodW5rSW5mb3MuZmlsdGVyKEJvb2xlYW4pO1xuXG4gICAgLy8gRW5kIG9mIGZpbGUgb3Igbm8gbW9yZSBjYW5kaWRhdGVzXG4gICAgaWYgKGNodW5rc1RvTG9hZC5sZW5ndGggPT09IDApIHtcbiAgICAgIHJldHVybiBmYWxzZTtcbiAgICB9XG5cbiAgICAvLyBBZGQgMSBuc2VjIHRvIG1ha2UgZW5kIDEgcGFzdCB0aGUgZW5kIGZvciB0aGUgbmV4dCByZWFkXG4gICAgdGhpcy5wb3NpdGlvbiA9IGVuZCA9IGFkZFRpbWUoZW5kLCB7IHNlYzogMCwgbnNlYzogMSB9KTtcblxuICAgIGNvbnN0IGhlYXAgPSB0aGlzLmhlYXA7XG4gICAgY29uc3QgbmV3Q2FjaGUgPSBuZXcgTWFwPG51bWJlciwgQ2h1bmtSZWFkUmVzdWx0PigpO1xuICAgIGZvciAoY29uc3QgY2h1bmtJbmZvIG9mIGNodW5rc1RvTG9hZCkge1xuICAgICAgbGV0IHJlc3VsdCA9IHRoaXMuY2FjaGVkQ2h1bmtSZWFkUmVzdWx0cy5nZXQoY2h1bmtJbmZvLmNodW5rUG9zaXRpb24pO1xuICAgICAgaWYgKCFyZXN1bHQpIHtcbiAgICAgICAgcmVzdWx0ID0gYXdhaXQgdGhpcy5yZWFkZXIucmVhZENodW5rKGNodW5rSW5mbywgdGhpcy5kZWNvbXByZXNzKTtcbiAgICAgIH1cblxuICAgICAgLy8gS2VlcCBjaHVuayByZWFkIHJlc3VsdHMgZm9yIGNodW5rcyB3aGVyZSBlbmQgaXMgaW4gdGhlIGNodW5rXG4gICAgICAvLyBFbmQgaXMgdGhlIG5leHQgcG9zaXRpb24gd2Ugd2lsbCByZWFkIHNvIHdlIGRvbid0IG5lZWQgdG8gcmUtcmVhZCB0aGUgY2h1bmtcbiAgICAgIGlmIChjb21wYXJlKGNodW5rSW5mby5zdGFydFRpbWUsIGVuZCkgPD0gMCAmJiBjb21wYXJlKGNodW5rSW5mby5lbmRUaW1lLCBlbmQpID49IDApIHtcbiAgICAgICAgbmV3Q2FjaGUuc2V0KGNodW5rSW5mby5jaHVua1Bvc2l0aW9uLCByZXN1bHQpO1xuICAgICAgfVxuXG4gICAgICBmb3IgKGNvbnN0IGluZGV4RGF0YSBvZiByZXN1bHQuaW5kaWNlcykge1xuICAgICAgICBpZiAodGhpcy5jb25uZWN0aW9uSWRzICYmICF0aGlzLmNvbm5lY3Rpb25JZHMuaGFzKGluZGV4RGF0YS5jb25uKSkge1xuICAgICAgICAgIGNvbnRpbnVlO1xuICAgICAgICB9XG4gICAgICAgIGZvciAoY29uc3QgaW5kZXhFbnRyeSBvZiBpbmRleERhdGEuaW5kaWNlcyA/PyBbXSkge1xuICAgICAgICAgIC8vIGVuc3VyZTogc3RhbXAgPD0gZW50cnkgdGltZSA8IGVuZFxuICAgICAgICAgIGlmIChjb21wYXJlKGluZGV4RW50cnkudGltZSwgc3RhbXApIDwgMCB8fCBjb21wYXJlKGluZGV4RW50cnkudGltZSwgZW5kKSA+PSAwKSB7XG4gICAgICAgICAgICBjb250aW51ZTtcbiAgICAgICAgICB9XG4gICAgICAgICAgaGVhcC5wdXNoKHsgdGltZTogaW5kZXhFbnRyeS50aW1lLCBvZmZzZXQ6IGluZGV4RW50cnkub2Zmc2V0LCBjaHVua1JlYWRSZXN1bHQ6IHJlc3VsdCB9KTtcbiAgICAgICAgfVxuICAgICAgfVxuICAgIH1cblxuICAgIHRoaXMuY2FjaGVkQ2h1bmtSZWFkUmVzdWx0cyA9IG5ld0NhY2hlO1xuICAgIHJldHVybiB0cnVlO1xuICB9XG59XG4iXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IkFBQUEsU0FBUyxPQUFPLEVBQUUsT0FBTyxPQUFPLFFBQVEsbUNBQW1DO0FBQzNFLE9BQU8sVUFBVSxzQkFBd0U7QUFFekYsU0FBUyxZQUFZLFFBQVEsb0JBQW9CO0FBSWpELE9BQU8sTUFBTSx3QkFBd0I7RUFDM0Isb0JBQStDO0VBRXZELFlBQVksSUFBNkIsQ0FBRTtJQUN6QyxtQ0FBbUM7SUFDbkMsS0FBSyxDQUFDLE1BQU0sQ0FBQyxHQUFHO01BQ2QsT0FBTyxRQUFR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
// denoCacheMetadata=11834315107054323666,814187404168086212