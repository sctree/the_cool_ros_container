import { compare, subtract as subTime } from "https://esm.sh/@foxglove/rostime";
import Heap from "https://esm.sh/heap";
import { BaseIterator } from "./BaseIterator.ts";
export class ReverseIterator extends BaseIterator {
  remainingChunkInfos;
  constructor(args){
    // Sort by largest timestamp first
    super(args, (a, b)=>{
      return compare(b.time, a.time);
    });
    // These are all chunks that we can consider for iteration.
    // Only consider chunks with a start before or equal to our position.
    // Chunks starting after our position are not part of reverse iteration
    this.chunkInfos = this.chunkInfos.filter((info)=>{
      return compare(info.startTime, this.position) <= 0;
    });
    // The chunk info heap sorts chunk infos by decreasing end time
    const chunkInfoHeap = new Heap((a, b)=>{
      return compare(b.endTime, a.endTime);
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
    let start = firstChunkInfo.startTime;
    const chunksToLoad = [
      firstChunkInfo
    ];
    for(let idx = 1; idx < this.remainingChunkInfos.length; ++idx){
      const nextChunkInfo = this.remainingChunkInfos[idx];
      if (!nextChunkInfo) {
        continue;
      }
      // The chunk ends before our selected start, we end chunk selection
      if (compare(nextChunkInfo.endTime, start) < 0) {
        break;
      }
      // The chunk ends after our start so we will load it
      chunksToLoad.push(nextChunkInfo);
      // If the chunk starts after or at the start time, we have fully consumed it
      const startCompare = compare(nextChunkInfo.startTime, start);
      if (startCompare >= 0) {
        this.remainingChunkInfos[idx] = undefined;
      }
    }
    // filter out undefined chunk infos
    this.remainingChunkInfos = this.remainingChunkInfos.filter(Boolean);
    // End of file or no more candidates
    if (chunksToLoad.length === 0) {
      return false;
    }
    // Subtract 1 nsec to make the next position 1 before
    this.position = start = subTime(start, {
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
      if (compare(chunkInfo.startTime, start) <= 0 && compare(chunkInfo.endTime, start) >= 0) {
        newCache.set(chunkInfo.chunkPosition, result);
      }
      for (const indexData of result.indices){
        if (this.connectionIds && !this.connectionIds.has(indexData.conn)) {
          continue;
        }
        for (const indexEntry of indexData.indices ?? []){
          // skip any time that is before our current timestamp or after end, we will never iterate to those
          if (compare(indexEntry.time, start) <= 0 || compare(indexEntry.time, stamp) > 0) {
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
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbImh0dHBzOi8vcmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbS9qZWZmLWh5a2luL3JhcGlkX3Jvc19zZXJ2ZXIvZGV2L3N1YnJlcG9zL2ZveGdsb3ZlX3Jvc2JhZy9zcmMvUmV2ZXJzZUl0ZXJhdG9yLnRzIl0sInNvdXJjZXNDb250ZW50IjpbImltcG9ydCB7IGNvbXBhcmUsIHN1YnRyYWN0IGFzIHN1YlRpbWUgfSBmcm9tIFwiaHR0cHM6Ly9lc20uc2gvQGZveGdsb3ZlL3Jvc3RpbWVcIjtcbmltcG9ydCBIZWFwIGZyb20gXCJodHRwczovL2VzbS5zaC9oZWFwXCIgLyogQ0hFQ0tNRTogZmlsZShzKSBkaWRuJ3QgZXhpc3QsIGFzc3VtaW5nIG5wbSAqLztcblxuaW1wb3J0IHsgQmFzZUl0ZXJhdG9yIH0gZnJvbSBcIi4vQmFzZUl0ZXJhdG9yLnRzXCI7XG5pbXBvcnQgeyBDaHVua0luZm8gfSBmcm9tIFwiLi9yZWNvcmQudHNcIjtcbmltcG9ydCB7IENodW5rUmVhZFJlc3VsdCwgSXRlcmF0b3JDb25zdHJ1Y3RvckFyZ3MgfSBmcm9tIFwiLi90eXBlcy50c1wiO1xuXG5leHBvcnQgY2xhc3MgUmV2ZXJzZUl0ZXJhdG9yIGV4dGVuZHMgQmFzZUl0ZXJhdG9yIHtcbiAgcHJpdmF0ZSByZW1haW5pbmdDaHVua0luZm9zOiAoQ2h1bmtJbmZvIHwgdW5kZWZpbmVkKVtdO1xuXG4gIGNvbnN0cnVjdG9yKGFyZ3M6IEl0ZXJhdG9yQ29uc3RydWN0b3JBcmdzKSB7XG4gICAgLy8gU29ydCBieSBsYXJnZXN0IHRpbWVzdGFtcCBmaXJzdFxuICAgIHN1cGVyKGFyZ3MsIChhLCBiKSA9PiB7XG4gICAgICByZXR1cm4gY29tcGFyZShiLnRpbWUsIGEudGltZSk7XG4gICAgfSk7XG5cbiAgICAvLyBUaGVzZSBhcmUgYWxsIGNodW5rcyB0aGF0IHdlIGNhbiBjb25zaWRlciBmb3IgaXRlcmF0aW9uLlxuICAgIC8vIE9ubHkgY29uc2lkZXIgY2h1bmtzIHdpdGggYSBzdGFydCBiZWZvcmUgb3IgZXF1YWwgdG8gb3VyIHBvc2l0aW9uLlxuICAgIC8vIENodW5rcyBzdGFydGluZyBhZnRlciBvdXIgcG9zaXRpb24gYXJlIG5vdCBwYXJ0IG9mIHJldmVyc2UgaXRlcmF0aW9uXG4gICAgdGhpcy5jaHVua0luZm9zID0gdGhpcy5jaHVua0luZm9zLmZpbHRlcigoaW5mbykgPT4ge1xuICAgICAgcmV0dXJuIGNvbXBhcmUoaW5mby5zdGFydFRpbWUsIHRoaXMucG9zaXRpb24pIDw9IDA7XG4gICAgfSk7XG5cbiAgICAvLyBUaGUgY2h1bmsgaW5mbyBoZWFwIHNvcnRzIGNodW5rIGluZm9zIGJ5IGRlY3JlYXNpbmcgZW5kIHRpbWVcbiAgICBjb25zdCBjaHVua0luZm9IZWFwID0gbmV3IEhlYXA8Q2h1bmtJbmZvPigoYSwgYikgPT4ge1xuICAgICAgcmV0dXJuIGNvbXBhcmUoYi5lbmRUaW1lLCBhLmVuZFRpbWUpO1xuICAgIH0pO1xuXG4gICAgZm9yIChjb25zdCBpbmZvIG9mIHRoaXMuY2h1bmtJbmZvcykge1xuICAgICAgY2h1bmtJbmZvSGVhcC5pbnNlcnQoaW5mbyk7XG4gICAgfVxuXG4gICAgdGhpcy5yZW1haW5pbmdDaHVua0luZm9zID0gW107XG4gICAgd2hpbGUgKGNodW5rSW5mb0hlYXAuc2l6ZSgpID4gMCkge1xuICAgICAgdGhpcy5yZW1haW5pbmdDaHVua0luZm9zLnB1c2goY2h1bmtJbmZvSGVhcC5wb3AoKSk7XG4gICAgfVxuICB9XG5cbiAgcHJvdGVjdGVkIG92ZXJyaWRlIGFzeW5jIGxvYWROZXh0KCk6IFByb21pc2U8Ym9vbGVhbj4ge1xuICAgIGNvbnN0IHN0YW1wID0gdGhpcy5wb3NpdGlvbjtcblxuICAgIGNvbnN0IGZpcnN0Q2h1bmtJbmZvID0gdGhpcy5yZW1haW5pbmdDaHVua0luZm9zWzBdO1xuICAgIGlmICghZmlyc3RDaHVua0luZm8pIHtcbiAgICAgIHJldHVybiBmYWxzZTtcbiAgICB9XG5cbiAgICB0aGlzLnJlbWFpbmluZ0NodW5rSW5mb3NbMF0gPSB1bmRlZmluZWQ7XG5cbiAgICBsZXQgc3RhcnQgPSBmaXJzdENodW5rSW5mby5zdGFydFRpbWU7XG4gICAgY29uc3QgY2h1bmtzVG9Mb2FkOiBDaHVua0luZm9bXSA9IFtmaXJzdENodW5rSW5mb107XG5cbiAgICBmb3IgKGxldCBpZHggPSAxOyBpZHggPCB0aGlzLnJlbWFpbmluZ0NodW5rSW5mb3MubGVuZ3RoOyArK2lkeCkge1xuICAgICAgY29uc3QgbmV4dENodW5rSW5mbyA9IHRoaXMucmVtYWluaW5nQ2h1bmtJbmZvc1tpZHhdO1xuICAgICAgaWYgKCFuZXh0Q2h1bmtJbmZvKSB7XG4gICAgICAgIGNvbnRpbnVlO1xuICAgICAgfVxuXG4gICAgICAvLyBUaGUgY2h1bmsgZW5kcyBiZWZvcmUgb3VyIHNlbGVjdGVkIHN0YXJ0LCB3ZSBlbmQgY2h1bmsgc2VsZWN0aW9uXG4gICAgICBpZiAoY29tcGFyZShuZXh0Q2h1bmtJbmZvLmVuZFRpbWUsIHN0YXJ0KSA8IDApIHtcbiAgICAgICAgYnJlYWs7XG4gICAgICB9XG5cbiAgICAgIC8vIFRoZSBjaHVuayBlbmRzIGFmdGVyIG91ciBzdGFydCBzbyB3ZSB3aWxsIGxvYWQgaXRcbiAgICAgIGNodW5rc1RvTG9hZC5wdXNoKG5leHRDaHVua0luZm8pO1xuXG4gICAgICAvLyBJZiB0aGUgY2h1bmsgc3RhcnRzIGFmdGVyIG9yIGF0IHRoZSBzdGFydCB0aW1lLCB3ZSBoYXZlIGZ1bGx5IGNvbnN1bWVkIGl0XG4gICAgICBjb25zdCBzdGFydENvbXBhcmUgPSBjb21wYXJlKG5leHRDaHVua0luZm8uc3RhcnRUaW1lLCBzdGFydCk7XG4gICAgICBpZiAoc3RhcnRDb21wYXJlID49IDApIHtcbiAgICAgICAgdGhpcy5yZW1haW5pbmdDaHVua0luZm9zW2lkeF0gPSB1bmRlZmluZWQ7XG4gICAgICB9XG4gICAgfVxuXG4gICAgLy8gZmlsdGVyIG91dCB1bmRlZmluZWQgY2h1bmsgaW5mb3NcbiAgICB0aGlzLnJlbWFpbmluZ0NodW5rSW5mb3MgPSB0aGlzLnJlbWFpbmluZ0NodW5rSW5mb3MuZmlsdGVyKEJvb2xlYW4pO1xuXG4gICAgLy8gRW5kIG9mIGZpbGUgb3Igbm8gbW9yZSBjYW5kaWRhdGVzXG4gICAgaWYgKGNodW5rc1RvTG9hZC5sZW5ndGggPT09IDApIHtcbiAgICAgIHJldHVybiBmYWxzZTtcbiAgICB9XG5cbiAgICAvLyBTdWJ0cmFjdCAxIG5zZWMgdG8gbWFrZSB0aGUgbmV4dCBwb3NpdGlvbiAxIGJlZm9yZVxuICAgIHRoaXMucG9zaXRpb24gPSBzdGFydCA9IHN1YlRpbWUoc3RhcnQsIHsgc2VjOiAwLCBuc2VjOiAxIH0pO1xuXG4gICAgY29uc3QgaGVhcCA9IHRoaXMuaGVhcDtcbiAgICBjb25zdCBuZXdDYWNoZSA9IG5ldyBNYXA8bnVtYmVyLCBDaHVua1JlYWRSZXN1bHQ+KCk7XG4gICAgZm9yIChjb25zdCBjaHVua0luZm8gb2YgY2h1bmtzVG9Mb2FkKSB7XG4gICAgICBsZXQgcmVzdWx0ID0gdGhpcy5jYWNoZWRDaHVua1JlYWRSZXN1bHRzLmdldChjaHVua0luZm8uY2h1bmtQb3NpdGlvbik7XG4gICAgICBpZiAoIXJlc3VsdCkge1xuICAgICAgICByZXN1bHQgPSBhd2FpdCB0aGlzLnJlYWRlci5yZWFkQ2h1bmsoY2h1bmtJbmZvLCB0aGlzLmRlY29tcHJlc3MpO1xuICAgICAgfVxuXG4gICAgICAvLyBLZWVwIGNodW5rIHJlYWQgcmVzdWx0cyBmb3IgY2h1bmtzIHdoZXJlIGVuZCBpcyBpbiB0aGUgY2h1bmtcbiAgICAgIC8vIEVuZCBpcyB0aGUgbmV4dCBwb3NpdGlvbiB3ZSB3aWxsIHJlYWQgc28gd2UgZG9uJ3QgbmVlZCB0byByZS1yZWFkIHRoZSBjaHVua1xuICAgICAgaWYgKGNvbXBhcmUoY2h1bmtJbmZvLnN0YXJ0VGltZSwgc3RhcnQpIDw9IDAgJiYgY29tcGFyZShjaHVua0luZm8uZW5kVGltZSwgc3RhcnQpID49IDApIHtcbiAgICAgICAgbmV3Q2FjaGUuc2V0KGNodW5rSW5mby5jaHVua1Bvc2l0aW9uLCByZXN1bHQpO1xuICAgICAgfVxuXG4gICAgICBmb3IgKGNvbnN0IGluZGV4RGF0YSBvZiByZXN1bHQuaW5kaWNlcykge1xuICAgICAgICBpZiAodGhpcy5jb25uZWN0aW9uSWRzICYmICF0aGlzLmNvbm5lY3Rpb25JZHMuaGFzKGluZGV4RGF0YS5jb25uKSkge1xuICAgICAgICAgIGNvbnRpbnVlO1xuICAgICAgICB9XG4gICAgICAgIGZvciAoY29uc3QgaW5kZXhFbnRyeSBvZiBpbmRleERhdGEuaW5kaWNlcyA/PyBbXSkge1xuICAgICAgICAgIC8vIHNraXAgYW55IHRpbWUgdGhhdCBpcyBiZWZvcmUgb3VyIGN1cnJlbnQgdGltZXN0YW1wIG9yIGFmdGVyIGVuZCwgd2Ugd2lsbCBuZXZlciBpdGVyYXRlIHRvIHRob3NlXG4gICAgICAgICAgaWYgKGNvbXBhcmUoaW5kZXhFbnRyeS50aW1lLCBzdGFydCkgPD0gMCB8fCBjb21wYXJlKGluZGV4RW50cnkudGltZSwgc3RhbXApID4gMCkge1xuICAgICAgICAgICAgY29udGludWU7XG4gICAgICAgICAgfVxuICAgICAgICAgIGhlYXAucHVzaCh7IHRpbWU6IGluZGV4RW50cnkudGltZSwgb2Zmc2V0OiBpbmRleEVudHJ5Lm9mZnNldCwgY2h1bmtSZWFkUmVzdWx0OiByZXN1bHQgfSk7XG4gICAgICAgIH1cbiAgICAgIH1cbiAgICB9XG5cbiAgICB0aGlzLmNhY2hlZENodW5rUmVhZFJlc3VsdHMgPSBuZXdDYWNoZTtcbiAgICByZXR1cm4gdHJ1ZTtcbiAgfVxufVxuIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiJBQUFBLFNBQVMsT0FBTyxFQUFFLFlBQVksT0FBTyxRQUFRLG1DQUFtQztBQUNoRixPQUFPLFVBQVUsc0JBQXdFO0FBRXpGLFNBQVMsWUFBWSxRQUFRLG9CQUFvQjtBQUlqRCxPQUFPLE1BQU0sd0JBQXdCO0VBQzNCLG9CQUErQztFQUV2RCxZQUFZLElBQTZCLENBQUU7SUFDekMsa0NBQWtDO0lBQ2xDLEtBQUssQ0FBQyxNQUFNLENBQUMsR0FBRztN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
// denoCacheMetadata=4996587369401957929,10745580859133010143