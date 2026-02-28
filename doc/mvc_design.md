# H.264 MVC (Multiview Video Coding) Decoder -- Design Document

## Table of Contents

1. [MVC Spec Overview](#1-mvc-spec-overview)
2. [MPEG-TS Container Layer](#2-mpeg-ts-container-layer)
3. [NAL Unit Layer](#3-nal-unit-layer)
4. [Architecture: Single DPB with view_id Tagging](#4-architecture-single-dpb-with-view_id-tagging)
5. [Slice Decoding Flow](#5-slice-decoding-flow)
6. [Reference Picture Management](#6-reference-picture-management)
7. [Inter-View Prediction](#7-inter-view-prediction)
8. [POC (Picture Order Count) Handling](#8-poc-picture-order-count-handling)
9. [Output and Frame Threading](#9-output-and-frame-threading)
10. [Known Issues and Remaining Work](#10-known-issues-and-remaining-work)
11. [File Map](#11-file-map)


## 1. MVC Spec Overview

**Standard**: ITU-T H.264 Annex H (also ISO/IEC 14496-10 Annex H)
**Amendment**: Originally published as H.264 Amendment 1 (MVC extension, 2009)

MVC extends H.264/AVC to encode multiple camera views in a single bitstream.
The typical use case is stereoscopic 3D (2 views: left eye = base view,
right eye = dependent view), but the spec supports up to 1024 views.

### Key Concepts

```
Access Unit (AU)
+----------------------------------------------+
|  Base View (view_id=0)    | view_order_idx=0 |  <-- AVC-compatible
|  Dependent View (view_id=1) | view_order_idx=1 |  <-- MVC extension
+----------------------------------------------+
```

- **Base view**: Fully AVC-compatible. Standard H.264 decoders can decode
  just the base view and ignore MVC NALs. Uses standard NAL types 1-5.

- **Dependent view**: Encoded using MVC extension NAL types. Can reference
  both temporal references (same view, different time) AND inter-view
  references (different view, same time = same access unit).

- **Access Unit**: One complete coded picture from ALL views at a given
  time instant. In stereo, each AU contains one base frame + one dependent
  frame.

- **view_id**: 10-bit identifier in the NAL extension header. The base
  view typically has view_id=0. The dependent view has view_id>0 (often 1).

- **view_order_index (VOIdx)**: The decoding order of views within an AU.
  Base view is always decoded first (VOIdx=0). This is defined in the
  Subset SPS.

### MVC NAL Unit Types (H.264 Table 7-1)

| Type | Name             | Description                                    |
|------|------------------|------------------------------------------------|
| 14   | PREFIX            | Prefix NAL unit. Precedes base view slices in  |
|      |                  | MVC bitstreams. Contains MVC extension header.  |
|      |                  | Can be used for priority/temporal_id metadata.  |
| 15   | SUBSET_SPS       | Subset Sequence Parameter Set. Contains the    |
|      |                  | base SPS followed by MVC-specific extensions   |
|      |                  | (view dependency info, level values, etc).      |
| 20   | SLICE_EXTENSION  | Coded slice of an extension (MVC/SVC).         |
|      |                  | NAL header is 4 bytes instead of 1 byte.       |
|      |                  | The slice header after the extension header is  |
|      |                  | standard H.264.                                |

### NAL Header Format

Standard H.264 NAL header (1 byte):
```
+---+---+---+---+---+---+---+---+
| 0 | NRI   | Type (5 bits)     |
+---+---+---+---+---+---+---+---+
  ^forbidden    ^nal_ref_idc  ^nal_unit_type
```

MVC Extension NAL header (4 bytes total, for NAL types 14 and 20):
```
Byte 0: Standard NAL header (forbidden_zero_bit, nal_ref_idc, nal_unit_type)

Bytes 1-3 (24 bits): NAL unit header MVC extension
+---+--------+------------+-----+---+---+---+
| S | NI | P  | view_id    | T   | A | V | R |
+---+--------+------------+-----+---+---+---+
  1    1    6     10          3    1   1   1  = 24 bits

S  = svc_extension_flag   (1 bit)  -- 0 for MVC, 1 for SVC
NI = non_idr_flag         (1 bit)  -- 0 = IDR picture, 1 = non-IDR
P  = priority_id          (6 bits) -- priority for sub-bitstream extraction
view_id                   (10 bits) -- identifies the view
T  = temporal_id          (3 bits) -- temporal layer
A  = anchor_pic_flag      (1 bit)  -- 1 if this is an anchor (like IDR for views)
V  = inter_view_flag      (1 bit)  -- 1 if used for inter-view prediction
R  = reserved_one_bit     (1 bit)  -- must be 1
```

**Important**: After the 3-byte extension header, the bitstream continues
with a standard H.264 slice header (first_mb_in_slice, slice_type, pps_id,
frame_num, etc). The slice header and slice data are identical to regular
H.264 -- the only difference is the NAL header.


## 2. MPEG-TS Container Layer

### Stream Types

In MPEG-TS (ISO/IEC 13818-1), MVC uses two PIDs in the same program:

| Stream Type | Hex  | Description             | PID (typical) |
|------------|------|-------------------------|---------------|
| 0x1B       | 27   | H.264/AVC video         | 0x1011        |
| 0x20       | 32   | MVC dependent view      | 0x1012        |

The PMT (Program Map Table) lists both PIDs. The base view (0x1B) carries
standard H.264 NALs (SPS, PPS, IDR, SLICE) plus PREFIX NALs (type 14).
The dependent view (0x20) carries SUBSET_SPS (type 15) and SLICE_EXTENSION
NALs (type 20).

### Stream Merging Strategy

**Problem**: FFmpeg's demuxer creates separate AVStreams per PID. But the
decoder needs to see NALs from BOTH views to build reference lists correctly.
Without merging, the dependent view stream would lack the base view's
reference frames.

**Solution** (in `libavformat/mpegts.c` `pmt_cb()`):

1. During PMT parsing, detect `stream_type == 0x20` (MVC).
2. Create a PES filter for packet assembly on that PID, but do NOT create
   a new AVStream for it.
3. After parsing all PMT entries, find the base H.264 stream (type 0x1B)
   in the same program.
4. Point the MVC PES's `pes->st` to the base stream's AVStream.
5. Set `pes->merged_st = 1` so cleanup doesn't double-free.

**Result**: Packets from both PIDs appear on the SAME AVStream. The decoder
receives interleaved base and dependent view NALs. The parser's
`h264_find_frame_end()` handles the NAL type boundaries correctly because
it recognizes EXTEN_SLICE with `first_mb_in_slice == 0` as a new picture
boundary (when the view changes).

**Packet ordering**: In a typical MPEG-TS MVC stream:
```
PES packet 1 (PID 0x1011): [SPS][PPS][PREFIX][IDR]         -- base view
PES packet 2 (PID 0x1012): [SUBSET_SPS][EXTEN_SLICE(IDR)]  -- dep view
PES packet 3 (PID 0x1011): [PREFIX][SLICE(P)]               -- base view
PES packet 4 (PID 0x1012): [EXTEN_SLICE(P)]                 -- dep view
...
```

**Caveat**: PES packet ordering between PIDs is not guaranteed by MPEG-TS.
In practice, some streams deliver the dependent view PES BEFORE the base
view PES for the same time instant. This creates a chicken-and-egg problem:
the dependent view needs the base view as an inter-view reference, but the
base view hasn't been decoded yet. This is a known issue (see Section 10).


## 3. NAL Unit Layer

### Parser Changes (`h264_parser.c`)

The H.264 parser needs to handle MVC NAL types for frame boundary detection:

- **NAL type 20 (EXTEN_SLICE)**: The parser must skip the 3-byte MVC
  extension header before reading `first_mb_in_slice`. It uses
  `first_mb_in_slice == 0` as a frame boundary, just like regular slices.
  Additionally, it detects view_id changes as boundaries.

- **ref_pic_list_modification**: MVC defines idc values 4 and 5 for
  inter-view reference modifications. The parser must accept these
  (reading `abs_diff_view_idx_minus1`) instead of rejecting them as errors.

### Decoder NAL Dispatch (`h264dec.c` `decode_nal_units()`)

The decoder's NAL switch statement handles three new cases:

```
NAL type 15 (SUBSET_SPS):
    Parse as regular SPS. The MVC-specific extensions (view dependency
    info, num_views, etc) are currently ignored -- we rely on runtime
    detection of view_ids in the slice NALs.

NAL type 14 (PREFIX):
    No-op. The prefix NAL's MVC extension header contains metadata
    (priority_id, temporal_id, anchor_pic_flag) that we don't currently
    use. The prefix NAL precedes base view slices but doesn't carry
    slice data itself.

NAL type 20 (EXTEN_SLICE):
    1. Read the 24-bit MVC extension header (see format above).
    2. If svc_extension_flag == 1, skip (SVC not supported).
    3. Extract view_id and non_idr_flag.
    4. Set h->cur_view_id = view_id.
    5. Rewrite nal->type to IDR_SLICE or SLICE based on non_idr_flag.
       This allows the standard slice header parsing code to work
       unchanged -- the slice header after the extension header is
       identical to regular H.264.
    6. For IDR (non_idr_flag == 0): Do NOT call idr() for dependent
       views. The base view's IDR already cleared the DPB. Calling
       idr() again would remove the base view reference that the
       dependent view needs for inter-view prediction.
    7. Queue the slice for decoding via ff_h264_queue_decode_slice().
```

**Critical detail**: For regular SLICE/IDR_SLICE NALs (types 1/5),
`h->cur_view_id` is set to 0. This ensures that base view slices always
have view_id=0, even after processing a dependent view extension slice
that set cur_view_id to a non-zero value.


## 4. Architecture: Single DPB with view_id Tagging

### Why Not Per-Layer DPB?

The HEVC multiview decoder (in `libavcodec/hevc/`) uses a full per-layer
architecture: `HEVCLayerContext` with separate DPB, SPS, slice context,
and buffer pools per layer. This required refactoring ~15 files and
hundreds of call sites.

For H.264 MVC (specifically stereo 3D with 2 views), we use a simpler
approach:

### Single DPB Design

```
H264Context
  |
  +-- DPB[36]              <-- shared by all views
  |     |-- DPB[0]: view_id=0, frame_num=0, poc=0
  |     |-- DPB[1]: view_id=1, frame_num=0, poc=1
  |     |-- DPB[2]: view_id=0, frame_num=1, poc=2
  |     |-- DPB[3]: view_id=1, frame_num=1, poc=3
  |     ...
  |
  +-- short_ref[32]        <-- shared, filtered by view_id at use sites
  +-- long_ref[32]         <-- shared, filtered by view_id at use sites
  +-- cur_view_id          <-- current NAL's view_id
```

Each `H264Picture` carries a `view_id` field. All reference list
operations filter by `view_id` to create per-view "virtual partitions"
of the shared DPB.

### Why This Works

1. **Same resolution**: In stereo MVC, both views share the same SPS
   (resolution, bit depth, chroma format). The `mismatches_ref()` check
   catches any violations.

2. **DPB capacity**: H264_MAX_PICTURE_COUNT = 36. Typical MVC stereo
   uses ~8 refs per view = 16 total, well within 36.

3. **Minimal code changes**: Only reference management functions need
   view_id filtering. Slice decoding, MB decoding, deblocking, etc.
   work unchanged because they operate on the current picture's buffers,
   not on the DPB directly.

### Data Structures

```c
// In H264Picture (h264dec.h):
int view_id;  // MVC view ID. 0 = base view, >0 = dependent view.
              // Set in ff_h264_queue_decode_slice() when picture is allocated.
              // For non-MVC streams, always 0 (zero-initialized).

// In H264Context (h264dec.h):
int cur_view_id;  // view_id of the NAL currently being decoded.
                  // Set in decode_nal_units() before slice processing.
                  // Used by reference management to filter/tag pictures.
```


## 5. Slice Decoding Flow

### Normal H.264 (single view)

```
decode_nal_units()
  |
  +-- NAL_IDR_SLICE:
  |     idr() -> clear DPB
  |     cur_view_id = 0    (fall through to SLICE)
  |
  +-- NAL_SLICE:
  |     cur_view_id = 0
  |     ff_h264_queue_decode_slice()
  |       |-- allocate picture if first slice
  |       |     pic->view_id = h->cur_view_id  (= 0)
  |       |-- parse slice header
  |       |-- ff_h264_build_ref_list()
  |       |     h264_initialise_ref_list()  -- build default ref list
  |       |     ref_pic_list_reordering()   -- apply reordering commands
  |       |-- queue slice for decoding
  |
  +-- ff_h264_execute_decode_slices()  -- decode all queued slices
  +-- ff_h264_field_end()
        |-- ff_h264_execute_ref_pic_marking()  -- add to short/long ref
```

### MVC (two views per access unit)

```
decode_nal_units()
  |
  +-- NAL_IDR_SLICE (base view):
  |     idr() -> clear DPB
  |     cur_view_id = 0
  |     queue_decode_slice() -> allocate pic, view_id=0
  |
  +-- NAL_EXTEN_SLICE (dependent view IDR):
  |     parse MVC extension header -> cur_view_id = 1
  |     rewrite nal->type = IDR_SLICE
  |     do NOT call idr()  <-- critical: base view ref must survive
  |     queue_decode_slice() -> allocate NEW pic, view_id=1
  |
  +-- (execute slices, field_end for first AU)
  |
  +-- NAL_SLICE (base view P-frame):
  |     cur_view_id = 0
  |     queue_decode_slice() -> allocate pic, view_id=0
  |     ref list: only view_id=0 short refs
  |
  +-- NAL_EXTEN_SLICE (dependent view P-frame):
  |     cur_view_id = 1
  |     rewrite nal->type = SLICE
  |     queue_decode_slice() -> allocate pic, view_id=1
  |     ref list: view_id=1 short refs + inter-view ref from view_id=0
  |
  +-- (execute slices, field_end)
```

### field_end and Picture Completion

`ff_h264_field_end()` is called when a new picture starts (detected by
`first_mb_in_slice == 0` for a different picture than current). It:

1. Calls `ff_h264_execute_ref_pic_marking()` which adds the completed
   picture to `short_ref[]` or `long_ref[]`.
2. Updates POC state (`prev_poc_msb`, `prev_poc_lsb`, etc).
3. Reports progress for frame threading.

For MVC, field_end is called for EACH view's picture. The base view's
picture is added to short_ref first, then the dependent view's picture.
This ordering is important because the dependent view's reference list
building needs the base view to already be in short_ref.


## 6. Reference Picture Management

### Per-View Reference List Initialization (`h264_refs.c`)

H.264 spec 8.2.4: Reference picture list initialization.
MVC spec H.8.4: Modification for multiview.

The default reference picture lists (RefPicList0, RefPicList1) are built
from `short_ref[]` and `long_ref[]`. For MVC, each view should only see
its own temporal references:

```
h264_initialise_ref_list():
  1. Filter short_ref[] -> view_short_ref[] (same view_id only)
  2. Filter long_ref[]  -> view_long_ref[]  (same view_id only)
  3. Build default lists from filtered arrays (standard algorithm)
  4. For dependent view: append inter-view reference (see Section 7)
```

**Non-MVC streams**: All view_ids are 0 (zero-initialized). The filtering
is a no-op -- view_short_ref[] == short_ref[]. Zero overhead.

### Per-View Sliding Window (H.264 spec 8.2.5.3)

The sliding window mechanism removes the oldest short-term reference when
the DPB is full. For MVC, "full" is evaluated per-view:

```
generate_sliding_window_mmcos():
  - Count only same-view short-term refs (view_short_count)
  - Count only same-view long-term refs (view_long_count)
  - If view_short_count + view_long_count >= max_num_ref_frames:
      Remove oldest same-view short-term ref
  - Track oldest_view_idx: the index in short_ref[] of the oldest
    same-view picture (highest index = oldest, since newest is prepended)
```

### Per-View Short-Term Reference Insertion

When a decoded picture is added to `short_ref[]` (in
`ff_h264_execute_ref_pic_marking()`), it must not collide with the other
view's picture that has the same `frame_num`:

```
ff_h264_execute_ref_pic_marking(), !current_ref_assigned block:
  - Standard H.264: remove_short(frame_num) then insert
  - MVC: only remove if frame_num AND view_id both match
    (both views share the same frame_num in the same AU)
```

### Per-View DPB Overflow Check

After adding a reference, check if the per-view count exceeds
`max_num_ref_frames`. If so, remove the oldest same-view reference.
This prevents one view from consuming the other view's DPB space.

### Reference Picture List Modification (H.264 spec 8.2.4.3)

`ff_h264_build_ref_list()` applies `ref_pic_list_modification` commands
from the slice header. MVC adds two new `modification_of_pic_nums_idc`
values:

| idc | Meaning                              | Syntax element                |
|-----|--------------------------------------|-------------------------------|
| 0   | Short-term ref, subtract pic_num     | abs_diff_pic_num_minus1       |
| 1   | Short-term ref, add pic_num          | abs_diff_pic_num_minus1       |
| 2   | Long-term ref by index               | long_term_pic_num             |
| 3   | End of modifications                 | (none)                        |
| 4   | **MVC inter-view short-term ref**    | abs_diff_view_idx_minus1      |
| 5   | **MVC inter-view long-term ref**     | abs_diff_view_idx_minus1      |

For idc=4: Find a short-term reference from a DIFFERENT view in the same
access unit (same frame_num, different view_id). This places the inter-view
reference at a specific position in the reference list.

For idc=5: Same but for long-term inter-view refs. Currently treated as
a no-op (long-term inter-view refs are rare in stereo MVC).

The `ff_h264_decode_ref_pic_list_reordering()` function accepts idc values
0-5 (changed from 0-2). For idc 4/5, the parser reads
`abs_diff_view_idx_minus1` using the same `get_ue_golomb_long()`.


## 7. Inter-View Prediction

### Concept

A dependent view slice can reference the base view picture from the SAME
access unit. This is called inter-view prediction. It's analogous to
temporal prediction but across views instead of across time.

```
Time --->   t=0      t=1      t=2
View 0:     [I]  --> [P]  --> [P]
             |        |        |
             v        v        v     (inter-view prediction)
View 1:     [I]  --> [P]  --> [P]
             ^        ^        ^
             |        |        |     (temporal prediction)
```

### Finding the Inter-View Reference

Both views in the same access unit share the same `frame_num` (set in the
slice header). They do NOT share the same POC -- the dependent view's POC
is computed independently and may differ by 1 from the base view.

**Key insight**: Use `frame_num` (not POC) to identify the inter-view
reference. In `h264_initialise_ref_list()`:

```c
for (i = 0; i < h->short_ref_count; i++) {
    if (h->short_ref[i]->view_id == 0 &&           // base view
        h->short_ref[i]->frame_num == cur_frame_num) // same AU
        -> this is the inter-view reference
}
```

### Placement in Reference List

MVC spec H.8.4 says inter-view references are appended AFTER temporal
references in the initial reference picture list. So for a dependent view
P-slice:

```
RefPicList0 = [temporal_ref_1, temporal_ref_2, ..., inter_view_ref]
```

The inter-view ref is added at the first empty slot (where `parent == NULL`)
in the reference list.

### When Inter-View Ref is Not Found

If the base view picture for the current access unit isn't in `short_ref[]`
yet (e.g., due to packet ordering -- dependent view PES arrived before
base view PES), the inter-view reference is simply not added. This causes
"reference picture missing during reorder" errors but doesn't crash.

This is a known limitation. Possible solutions:
- Reorder packets at demuxer level (complex)
- Buffer access units and decode views in order (requires `receive_frame`)
- Accept the error for the first few frames (current approach)


## 8. POC (Picture Order Count) Handling

### Current State

Both views share a single `H264POCContext` (`h->poc`). This means the
dependent view's POC computation is affected by the base view's state:

```
Base IDR:    poc = 0      (poc_lsb=0, prev_poc_msb=0, prev_poc_lsb=0)
Dep IDR:     poc = 1      (poc_lsb=1, prev state from base IDR)
Base P fn=1: poc = 65536  (poc_lsb=0, prev state from dep IDR)
Dep P fn=1:  poc = 65537  (poc_lsb=1, prev state from base P)
```

The POC values differ between views (off by 1 in this example). This is
why inter-view references are looked up by `frame_num` instead of POC.

### Correct Approach (TODO)

Each view should have its own `H264POCContext` for independent POC tracking:

```c
H264POCContext view_poc[2];  // per-view POC state

// In slice processing:
int view_idx = (h->cur_view_id > 0) ? 1 : 0;
h264_init_poc(... &h->view_poc[view_idx] ...);
```

This would give each view correct POC values (e.g., both views at t=0
would have POC=0). Currently not implemented -- the shared POC works
because we use frame_num for inter-view lookup.


## 9. Output and Frame Threading

### Frame Threading Limitation

MVC decoding is incompatible with frame threading because:
- The parser splits each view's NALs into separate packets
- Frame threading requires one complete frame per decode call
- With MVC, two pictures (base + dependent) are produced per access unit

Currently, MVC extension slices are silently processed in all threading
modes. For correctness, frame threading should be disabled when MVC NALs
are detected.

### Output FIFO (Implemented)

The decoder uses `FF_CODEC_RECEIVE_FRAME_CB` with an `AVContainerFifo`
for buffering output frames. This enables multi-frame output from a single
decode call:

1. `h264_receive_frame()`: manages packet input and FIFO output
2. `h264_decode_packet()`: core decode logic, pushes frames to FIFO
3. `h264_flush_delayed_to_fifo()`: drains reorder buffer on EOF

For non-MVC streams, the FIFO contains at most one frame per decode call.
For MVC, both views' frames are pushed and drained one at a time.

Frame threading works via `ff_thread_receive_frame()` +
`ff_thread_get_packet()` (same as HEVC multiview decoder).

### AV_FRAME_DATA_VIEW_ID Side Data (Implemented)

Each output frame carries `AV_FRAME_DATA_VIEW_ID` side data (int = view_id)
when MVC is active. Additionally, `AVStereo3D` side data is attached with
`AV_STEREO3D_FRAMESEQUENCE` type: view_id 0 = left (base view),
view_id > 0 = right (dependent view).

### view_ids Option (Implemented)

Following the HEVC pattern, the `view_ids` AVOption (int array) controls
which views are decoded and output:
- Default (empty): base view only
- `-1`: decode and output all views
- Specific IDs: decode and output only listed views

The `view_ids_available` AVOption (export, readonly) reports which view IDs
were detected in the stream.


## 10. Known Issues and Remaining Work

### Known Issues (Resolved)

1. **Dependent view IDR not added to short_ref**: Fixed by introducing
   `idr_pic_flag` (H264Context) to decouple IdrPicFlag from nal_unit_type.
   MVC anchor pictures (non_idr_flag=0) now correctly read idr_pic_id and
   dec_ref_pic_marking IDR syntax. `picture_idr` is only set for base view
   to prevent DPB clearing on dependent view anchors.

2. **find_short view mismatch**: Fixed by adding view_id matching in
   `find_short()` -- two MVC views share the same frame_num, so matching
   frame_num alone returned the wrong picture.

3. **Thread context update**: Fixed by copying `picture_idr`, `cur_view_id`,
   `idr_pic_flag`, and `mvc_active` in `ff_h264_update_thread_context()`.

4. **Per-view POC context**: Split `h->poc` into base and dependent view
   contexts (`h->poc` + `h->dep_view_poc`). Each view's slice headers carry
   independent POC parameters; without separation the dep view overwrote
   the base view's prev_poc_msb/lsb tracking.

5. **NAL type 20 in get_last_needed_nal**: Added EXTEN_SLICE handling with
   4-byte header offset (1 NAL header + 3 MVC extension) to correctly reach
   first_mb_in_slice.

6. **False errors from dep view packets**: When MVC dep view PES packets
   merged into the base view stream contain only EXTEN_SLICE NALs (skipped
   by view_ids filtering), the decoder no longer reports "no frame!" errors.
   Fixed by checking `h->mvc_active`.

### Known Issues (Remaining)

1. **Packet ordering**: MPEG-TS PES packets from the dependent view PID
   may arrive before the base view PID for the same access unit. Causes
   1 "Missing reference" error for the first dep non-IDR frame.

2. **Inter-view prediction**: Dependent view P-slices reference the base
   view picture for inter-view prediction (same POC, different view_id).
   The ref list builder does not yet add inter-view refs, causing
   concealment errors (~10-20 per GOP) in the dep view.

3. **Non-monotonic DTS**: Both views output frames with the same timestamps,
   triggering "non monotonically increasing dts" muxer warnings. This is
   expected with frame-sequential stereo output; a proper stereo-aware
   muxer would handle it.

### Remaining Implementation

- [ ] Inter-view reference prediction (add base view pic to dep view ref list)
- [ ] Test with more MVC content (Blu-ray 3D, different cameras)
- [ ] Consider DTS adjustment for frame-sequential stereo output


## 11. File Map

### Modified Files

| File                          | Changes                                      |
|-------------------------------|----------------------------------------------|
| `libavformat/mpegts.c`        | MVC PID merging in `pmt_cb()`                |
| `libavcodec/h264dec.h`        | `H264Picture.view_id`, `H264Context.{cur_view_id,idr_pic_flag,mvc_active,dep_view_poc,view_ids,output_fifo}` |
| `libavcodec/h264dec.c`        | NAL type 14/15/20 handling, `receive_frame`, output FIFO, `view_ids` option, view_id side data, `get_last_needed_nal` NAL 20 |
| `libavcodec/h264_slice.c`     | `pic->view_id`, `idr_pic_flag` usage, thread context update, per-view POC in `h264_field_start` |
| `libavcodec/h264_picture.c`   | `view_id` propagation, per-view POC in `ff_h264_field_end` |
| `libavcodec/h264_refs.c`      | Per-view ref lists, inter-view refs, per-view sliding window, per-view DPB overflow, per-view MMCO_RESET |
| `libavcodec/h264_parser.c`    | MVC NAL type 20 frame boundary, idc 4/5 in reordering |

### Pre-existing Files (Phase 1-2, already committed)

| File                                  | Changes                            |
|---------------------------------------|------------------------------------|
| `libavcodec/cbs_h264.h`              | CBS MVC NAL unit structures        |
| `libavcodec/cbs_h264_syntax_template.c` | CBS parsing for MVC extension header |
| `libavcodec/h264_parse.c`            | Parser: MVC frame boundary + idc 4/5 |
| `libavcodec/extract_extradata_bsf.c` | Handle NAL types 14/15/20          |

### Relevant Spec Sections

| Spec Section | Topic                                          |
|--------------|------------------------------------------------|
| 7.3.1        | NAL unit syntax (Table 7-1 for NAL types)      |
| 7.4.1        | NAL unit semantics                             |
| G.7.3.1.1    | NAL unit header MVC extension syntax           |
| G.7.4.1.1    | NAL unit header MVC extension semantics        |
| H.7.3.2.1.4  | Subset SPS MVC extension syntax                |
| H.8.4        | Decoding process for MVC (reference lists)     |
| H.8.2.1      | Derivation of inter-view reference components  |
| 8.2.4        | Reference picture list initialization          |
| 8.2.4.3      | Reference picture list modification            |
| 8.2.5        | Decoded reference picture marking              |
| 8.2.5.3      | Sliding window reference picture marking       |
