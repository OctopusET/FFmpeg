# Prompt for Next Session: Fix MVC idc=5 and Blu-ray 3D Support

Use this as the starting prompt for a new Claude Code session to implement
the missing features needed to decode real Blu-ray 3D content.

---

## Context

I'm implementing H.264 MVC (Multiview Video Coding) support in FFmpeg for
Blu-ray 3D playback. The base implementation is complete and works with our
FATE test sample, but a real Blu-ray 3D disc (5mintest.iso) fails because
it uses idc=5 (inter-view long-term reference modification) which is not
implemented.

### What's already done

- Full MVC decoder: NAL type 20 parsing, per-view POC, per-view DPB,
  inter-view refs, output FIFO, packet reordering, MPEG-TS merging
- idc=4 (inter-view short-term ref modification) works
- FATE test passes with existing sample
- See `doc/mvc_upstream_guide.md` for the full implementation reference
- See `doc/jm_vs_ffmpeg.md` for the feature gap analysis

### The problem

The Blu-ray 3D disc at `/mnt/BDMV/STREAM/00000.m2ts` uses **idc=5**
(inter-view long-term reference list modification) on EVERY dependent
view slice. Our code stubs it out with a warning at
`libavcodec/h264_refs.c:443-448`, causing all dependent view frames
to have missing references.

---

## Task 1: Implement idc=5 (CRITICAL)

**File:** `libavcodec/h264_refs.c`, function `ff_h264_build_ref_list()`

**Current code (lines 443-448):**
```c
case 5:
    /* MVC inter-view long-term ref modification (idc=5), not supported. */
    av_log(h->avctx, AV_LOG_WARNING,
           "MVC inter-view long-term ref modification not supported\n");
    i = -1;
    break;
```

**What to implement:** Same as idc=4 (lines 427-441) but for long-term refs.

**How idc=4 works (reference):**
```c
case 4: {
    /* MVC inter-view ref list modification (idc=4, Annex H).
     * Find picture with different view_id, same frame_num. */
    int cur_frame_num = h->cur_pic_ptr->frame_num;
    pic_structure = h->picture_structure;
    for (i = h->short_ref_count - 1; i >= 0; i--) {
        ref = h->short_ref[i];
        if (ref->view_id != h->cur_view_id &&
            ref->frame_num == cur_frame_num &&
            (ref->reference & pic_structure))
            break;
    }
    if (i >= 0)
        pic_id = ref->pic_id;
    break;
}
```

**How idc=5 should work:**

The inter-view reference picture was already appended to the ref list by
`h264_initialise_ref_list()` (lines 204-239 in `h264_refs.c`). idc=5 needs
to find it and move it to position `index` in the ref list.

Strategy: Search through the current ref list (or the DPB) for a picture
with different `view_id` and same `frame_num`. The difference from idc=4:
- idc=4 searches `short_ref[]`
- idc=5 should search `long_ref[]` first, then fall back to the DPB
  (since the inter-view ref may not be in long_ref but was appended
  to the ref list by init)

Alternatively, search the ref list itself for the inter-view ref that was
already appended. The key is finding the picture with `view_id != cur_view_id`
and `frame_num == cur_frame_num`.

**Reference pattern from the Blu-ray stream:**
```
Anchor P-slices:  [idc=5(val=0), idc=3]  -- base view is ONLY reference
Non-anchor slices: [idc=0(val=N), idc=5(val=0), idc=3]  -- temporal ref + base view
```

**Test:** After implementing, run:
```bash
./ffmpeg -hide_banner -ss 5 -t 2 -view_ids -1 \
  -i /mnt/BDMV/STREAM/00000.m2ts -map 0:v -f null -
# Should show ~96 frames (48 per view) with no errors

# Also verify FATE still passes:
make fate-h264-mvc SAMPLES=fate-suite/
```

---

## Task 2: Handle NAL type 24 (LOW priority)

NAL type 24 appears in the dependent view as a "dependent view access unit
delimiter." It's a Blu-ray convention. FFmpeg's default case already silently
ignores unknown NAL types, so this likely works as-is. Just verify no
warnings are logged for type 24.

If warnings appear, add it to the no-op list:
```c
case 24: /* dependent view AUD (Blu-ray) */
    break;
```

---

## Task 3: Update FATE test (if needed)

If the existing FATE test sample doesn't use idc=5, consider adding a
second test using a clip from the Blu-ray ISO to cover idc=5.

---

## Key files to read first

1. `libavcodec/h264_refs.c` -- ref list init + modification (idc=4/5)
2. `libavcodec/h264dec.c` -- NAL type handling, EXTEN_SLICE
3. `libavcodec/h264dec.h` -- MVC context fields
4. `doc/mvc_upstream_guide.md` -- full implementation reference
5. `doc/jm_vs_ffmpeg.md` -- feature gap analysis
6. `doc/mvc_design.md` -- architecture decisions

## Test commands

```bash
# Base view only (should always work):
./ffmpeg -hide_banner -t 5 -i /mnt/BDMV/STREAM/00000.m2ts -map 0:v -f null -

# Both views (currently fails, should work after fix):
./ffmpeg -hide_banner -t 5 -view_ids -1 -i /mnt/BDMV/STREAM/00000.m2ts -map 0:v -f null -

# Extract frames for visual inspection:
./ffmpeg -hide_banner -ss 10 -view_ids -1 -i /mnt/BDMV/STREAM/00000.m2ts \
  -map 0:v -vframes 10 -f image2 /tmp/frame_%03d.png

# FATE regression:
make fate-h264-mvc SAMPLES=fate-suite/
```
