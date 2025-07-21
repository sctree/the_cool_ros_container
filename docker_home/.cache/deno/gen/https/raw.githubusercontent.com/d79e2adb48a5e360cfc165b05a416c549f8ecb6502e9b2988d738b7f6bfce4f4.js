// Copyright 2018-2020 Cruise LLC
// Copyright 2021 Foxglove Technologies Inc
//
// This source code is licensed under the Apache License, Version 2.0,
// found in the LICENSE file in the root directory of this source tree.
// You may not use this file except in compliance with the License.
import Heap from "https://esm.sh/heap";
function nmerge(key, ...iterables) {
  const heap = new Heap((a, b)=>{
    return key(a.value, b.value);
  });
  for(let i = 0; i < iterables.length; i++){
    const result = iterables[i].next();
    if (result.done !== true) {
      heap.push({
        i,
        value: result.value
      });
    }
  }
  return {
    next: ()=>{
      if (heap.empty()) {
        return {
          done: true,
          value: undefined
        };
      }
      const { i } = heap.front();
      const next = iterables[i].next();
      if (next.done === true) {
        return {
          value: heap.pop().value,
          done: false
        };
      }
      return {
        value: heap.replace({
          i,
          value: next.value
        }).value,
        done: false
      };
    }
  };
}
export default nmerge;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbImh0dHBzOi8vcmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbS9qZWZmLWh5a2luL3JhcGlkX3Jvc19zZXJ2ZXIvZGV2L3N1YnJlcG9zL2ZveGdsb3ZlX3Jvc2JhZy9zcmMvbm1lcmdlLnRzIl0sInNvdXJjZXNDb250ZW50IjpbIi8vIENvcHlyaWdodCAyMDE4LTIwMjAgQ3J1aXNlIExMQ1xuLy8gQ29weXJpZ2h0IDIwMjEgRm94Z2xvdmUgVGVjaG5vbG9naWVzIEluY1xuLy9cbi8vIFRoaXMgc291cmNlIGNvZGUgaXMgbGljZW5zZWQgdW5kZXIgdGhlIEFwYWNoZSBMaWNlbnNlLCBWZXJzaW9uIDIuMCxcbi8vIGZvdW5kIGluIHRoZSBMSUNFTlNFIGZpbGUgaW4gdGhlIHJvb3QgZGlyZWN0b3J5IG9mIHRoaXMgc291cmNlIHRyZWUuXG4vLyBZb3UgbWF5IG5vdCB1c2UgdGhpcyBmaWxlIGV4Y2VwdCBpbiBjb21wbGlhbmNlIHdpdGggdGhlIExpY2Vuc2UuXG5cbmltcG9ydCBIZWFwIGZyb20gXCJodHRwczovL2VzbS5zaC9oZWFwXCIgLyogQ0hFQ0tNRTogZmlsZShzKSBkaWRuJ3QgZXhpc3QsIGFzc3VtaW5nIG5wbSAqLztcblxuZnVuY3Rpb24gbm1lcmdlPFQ+KGtleTogKGE6IFQsIGI6IFQpID0+IG51bWJlciwgLi4uaXRlcmFibGVzOiBBcnJheTxJdGVyYXRvcjxUPj4pOiBJdGVyYXRvcjxUPiB7XG4gIGNvbnN0IGhlYXA6IEhlYXA8eyBpOiBudW1iZXI7IHZhbHVlOiBUIH0+ID0gbmV3IEhlYXAoKGEsIGIpID0+IHtcbiAgICByZXR1cm4ga2V5KGEudmFsdWUsIGIudmFsdWUpO1xuICB9KTtcbiAgZm9yIChsZXQgaSA9IDA7IGkgPCBpdGVyYWJsZXMubGVuZ3RoOyBpKyspIHtcbiAgICBjb25zdCByZXN1bHQgPSBpdGVyYWJsZXNbaV0hLm5leHQoKTtcbiAgICBpZiAocmVzdWx0LmRvbmUgIT09IHRydWUpIHtcbiAgICAgIGhlYXAucHVzaCh7IGksIHZhbHVlOiByZXN1bHQudmFsdWUgfSk7XG4gICAgfVxuICB9XG5cbiAgcmV0dXJuIHtcbiAgICBuZXh0OiAoKSA9PiB7XG4gICAgICBpZiAoaGVhcC5lbXB0eSgpKSB7XG4gICAgICAgIHJldHVybiB7IGRvbmU6IHRydWUsIHZhbHVlOiB1bmRlZmluZWQgfTtcbiAgICAgIH1cbiAgICAgIGNvbnN0IHsgaSB9ID0gaGVhcC5mcm9udCgpITtcbiAgICAgIGNvbnN0IG5leHQgPSBpdGVyYWJsZXNbaV0hLm5leHQoKTtcbiAgICAgIGlmIChuZXh0LmRvbmUgPT09IHRydWUpIHtcbiAgICAgICAgcmV0dXJuIHsgdmFsdWU6IGhlYXAucG9wKCkhLnZhbHVlLCBkb25lOiBmYWxzZSB9O1xuICAgICAgfVxuICAgICAgcmV0dXJuIHsgdmFsdWU6IGhlYXAucmVwbGFjZSh7IGksIHZhbHVlOiBuZXh0LnZhbHVlIH0pLnZhbHVlLCBkb25lOiBmYWxzZSB9O1xuICAgIH0sXG4gIH07XG59XG5cbmV4cG9ydCBkZWZhdWx0IG5tZXJnZTtcbiJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQSxpQ0FBaUM7QUFDakMsMkNBQTJDO0FBQzNDLEVBQUU7QUFDRixzRUFBc0U7QUFDdEUsdUVBQXVFO0FBQ3ZFLG1FQUFtRTtBQUVuRSxPQUFPLFVBQVUsc0JBQXdFO0FBRXpGLFNBQVMsT0FBVSxHQUEyQixFQUFFLEdBQUcsU0FBNkI7RUFDOUUsTUFBTSxPQUFzQyxJQUFJLEtBQUssQ0FBQyxHQUFHO0lBQ3ZELE9BQU8sSUFBSSxFQUFFLEtBQUssRUFBRSxFQUFFLEtBQUs7RUFDN0I7RUFDQSxJQUFLLElBQUksSUFBSSxHQUFHLElBQUksVUFBVSxNQUFNLEVBQUUsSUFBSztJQUN6QyxNQUFNLFNBQVMsU0FBUyxDQUFDLEVBQUUsQ0FBRSxJQUFJO0lBQ2pDLElBQUksT0FBTyxJQUFJLEtBQUssTUFBTTtNQUN4QixLQUFLLElBQUksQ0FBQztRQUFFO1FBQUcsT0FBTyxPQUFPLEtBQUs7TUFBQztJQUNyQztFQUNGO0VBRUEsT0FBTztJQUNMLE1BQU07TUFDSixJQUFJLEtBQUssS0FBSyxJQUFJO1FBQ2hCLE9BQU87VUFBRSxNQUFNO1VBQU0sT0FBTztRQUFVO01BQ3hDO01BQ0EsTUFBTSxFQUFFLENBQUMsRUFBRSxHQUFHLEtBQUssS0FBSztNQUN4QixNQUFNLE9BQU8sU0FBUyxDQUFDLEVBQUUsQ0FBRSxJQUFJO01BQy9CLElBQUksS0FBSyxJQUFJLEtBQUssTUFBTTtRQUN0QixPQUFPO1VBQUUsT0FBTyxLQUFLLEdBQUcsR0FBSSxLQUFLO1VBQUUsTUFBTTtRQUFNO01BQ2pEO01BQ0EsT0FBTztRQUFFLE9BQU8sS0FBSyxPQUFPLENBQUM7VUFBRTtVQUFHLE9BQU8sS0FBSyxLQUFLO1FBQUMsR0FBRyxLQUFLO1FBQUUsTUFBTTtNQUFNO0lBQzVFO0VBQ0Y7QUFDRjtBQUVBLGVBQWUsT0FBTyJ9
// denoCacheMetadata=699230162257983286,11319117182065962189