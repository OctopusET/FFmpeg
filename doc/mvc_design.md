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
reference. Two lookup mechanisms:

1. **`mvc_base_pic` pointer** (preferred): In `h264_receive_frame()`, before
   draining dep view packets, the current base view picture is saved as
   `h->mvc_base_pic` with `MVC_IV_REF` flag protection to prevent premature
   unreferencing during dep view decoding. This is the fastest and most
   reliable lookup.

2. **DPB scan** (fallback): If `mvc_base_pic` is NULL or doesn't match,
   scan `h->DPB[]` for a picture with matching `frame_num` and different
   `view_id`. This covers edge cases where the pointer wasn't set.

In `h264_initialise_ref_list()` and ref list modification (idc=4/5):

```c
// Preferred: direct pointer
if (h->mvc_base_pic && h->mvc_base_pic->f->buf[0] &&
    h->mvc_base_pic->view_id != h->cur_view_id &&
    h->mvc_base_pic->frame_num == cur_frame_num) {
    ref = h->mvc_base_pic;
}
// Fallback: DPB scan
for (int j = 0; j < H264_MAX_PICTURE_COUNT; j++) {
    if (h->DPB[j].f->buf[0] &&
        h->DPB[j].view_id != h->cur_view_id &&
        h->DPB[j].frame_num == cur_frame_num) ...
}
```

### MVC_IV_REF Protection

The `MVC_IV_REF` flag (bit 3 in `H264Picture.reference`) prevents the base
view picture from being freed while the dependent view is decoding. Without
this, dep view MMCO or sliding window operations could remove the base view
picture that dep view slices need for inter-view prediction.

```c
// In h264_receive_frame(), before draining dep packets:
h->mvc_base_pic = h->cur_pic_ptr;
h->mvc_base_pic->reference |= MVC_IV_REF;

// After draining:
h->mvc_base_pic->reference &= ~MVC_IV_REF;
h->mvc_base_pic = NULL;
```

The `MVC_IV_REF` flag is also checked in `ff_h264_build_ref_list()` to
suppress false "Missing reference picture" errors:
```c
if (!FIELD_PICTURE(h) && !(ref & MVC_IV_REF) && (ref & 3) != 3)
    // error: missing reference
```

### Placement in Reference List

MVC spec H.8.4 says inter-view references are appended AFTER temporal
references in the initial reference picture list. So for a dependent view
P-slice:

```
RefPicList0 = [temporal_ref_1, temporal_ref_2, ..., inter_view_ref]
```

The inter-view ref is added at the first empty slot (where `parent == NULL`)
in the reference list. For field pictures, `pic_as_field()` adapts the
frame reference to the current field's parity. For frame pictures, the
reference bits are set to `PICT_FRAME`.

### Reference List Modification (idc=4 and idc=5)

The MVC bitstream uses idc=4 (inter-view short-term) and idc=5 (inter-view
long-term) in `ref_pic_list_modification()` to place the inter-view
reference at specific positions in the reference list:

- **idc=4**: Find a short-term reference from a DIFFERENT view with the
  same `frame_num`. Search `short_ref[]` for `view_id != cur_view_id`.

- **idc=5**: Same, but treated as a long-term inter-view reference. Uses
  `mvc_base_pic` first, then falls back to DPB scan. The `pic_structure`
  is initialized from `h->picture_structure` for correct field handling.

### MVC Packet Reordering (Accumulate-and-Drain)

MPEG-TS PES packet ordering between PIDs is not guaranteed. In practice,
dependent view PES packets often arrive BEFORE the base view PES for the
same access unit. This creates a chicken-and-egg problem: the dependent
view needs the base view as an inter-view reference, but the base view
hasn't been decoded yet.

**Solution** (in `h264_receive_frame()`):

1. When a dep-view-only packet arrives (detected by `h264_is_dep_view_packet()`
   which scans for EXTEN_SLICE NALs without any SLICE/IDR_SLICE NALs),
   accumulate it in `h->mvc_pending_pkts` (PacketList).

2. When a base view packet arrives, decode it first, then drain all pending
   dep view packets via `h264_drain_mvc_pending()`.

3. Before draining, set `h->mvc_base_pic` with `MVC_IV_REF` protection so
   the base view picture survives dep view decoding.

This ensures the base view picture is always in the DPB before any dep view
slice needs it. The only lost frame is the very first dep view extent in
SSIF files (lazy linking timing).


## 8. POC (Picture Order Count) Handling

### Per-View POC Contexts

Each view has its own `H264POCContext` for independent POC tracking:

```c
// In H264Context (h264dec.h):
H264POCContext poc;          // base view (view_id=0)
H264POCContext dep_view_poc; // dependent view (view_id>0)

// Selected throughout the code via:
H264POCContext *poc = h->cur_view_id ? &h->dep_view_poc : &h->poc;
```

Without separation, the dep view's slice header overwrites `prev_poc_msb`
and `prev_poc_lsb` in the shared POC context. When the next base view
slice arrives, it computes POC from the dep view's leftover state,
producing incorrect POC values and dropped frames.

Used in: `ff_h264_field_end()`, `h264_select_output_frame()`,
`ff_h264_queue_decode_slice()`, `idr()`, and MMCO_RESET.

### IDR and POC Reset

- Base view IDR: `idr()` resets `h->poc` from `decode_nal_units()`
- Dep view anchor: POC reset in `ff_h264_queue_decode_slice()`:
  ```c
  if (h->idr_pic_flag && h->cur_view_id && h->mvc_active) {
      poc->prev_frame_num = poc->prev_frame_num_offset = 0;
      poc->prev_poc_msb = 1<<16; poc->prev_poc_lsb = -1;
  }
  ```
- MMCO_RESET: resets only current view's POC and `last_pocs`/`last_pocs_dep`
- Flush/seek: temporarily disables `mvc_active` so `idr()` resets both views


## 9. Output and Frame Threading

### Frame Threading Limitation

MVC decoding is incompatible with frame threading because:
- The single DPB design requires both views decoded in the same thread
  context (the dep view needs the base view's picture in the same DPB)
- Frame threading gives each worker its own H264Context copy

**Auto-detection**: When MVC profiles (118=Multiview High, 128=Stereo High)
are detected in any SPS during `h264_decode_init()`, frame threading is
automatically disabled (`avctx->active_thread_type &= ~FF_THREAD_FRAME`).

**Runtime guard**: If frame threading is still active when an EXTEN_SLICE
NAL is encountered (e.g., profile wasn't detected in extradata), the dep
view slices are skipped with a log-once warning:
```
MVC: frame threading active, skipping dep view 1 (use -thread_type slice for both views)
```

Slice threading works fine with MVC -- all slices share the same DPB.

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

### Per-View Reorder Buffers

H.264 outputs decoded pictures in POC order using a reorder buffer
(`delayed_pic[]`). For MVC, the two views have interleaved but independent
POC sequences. If both views share a single reorder buffer, the
out-of-order detection logic (`out->poc < next_outputed_poc`) incorrectly
drops frames when view POCs interleave.

**Root cause example** (interlaced MVC with B-frames):
```
Base  view: poc=65536, 65538, 65540, ...
Dep   view: poc=65537, 65539, 65541, ...
In shared delayed_pic[]: 65536, 65537, 65538, 65539, ...

When dep poc=65540 is output before base poc=65538:
  next_outputed_poc = 65540
  base poc=65538 < 65540 -> dropped as "ooo" (out-of-order)
```

**Solution**: Separate reorder buffers per view:
```c
H264Picture *delayed_pic[H264_MAX_DPB_FRAMES + 2];      // base view
H264Picture *delayed_pic_dep[H264_MAX_DPB_FRAMES + 2];  // dep view
int last_pocs[H264_MAX_DPB_FRAMES];                      // base view
int last_pocs_dep[H264_MAX_DPB_FRAMES];                  // dep view
int next_outputed_poc;                                    // base view
int next_outputed_poc_dep;                                // dep view
```

In `h264_select_output_frame()`:
```c
int is_dep = cur->view_id && h->mvc_active;
H264Picture **delayed = is_dep ? h->delayed_pic_dep : h->delayed_pic;
int *last_pocs        = is_dep ? h->last_pocs_dep   : h->last_pocs;
int *next_poc         = is_dep ? &h->next_outputed_poc_dep : &h->next_outputed_poc;
```

All init/flush/close paths must handle both arrays. `unreference_pic()` must
check both `delayed_pic[]` AND `delayed_pic_dep[]` to protect pictures held
for delayed output.

**EOF drain**: `h264_flush_delayed_to_fifo()` drains base queue first, then
dep queue, via the shared `h264_flush_delayed_queue()` helper.

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
   dec_ref_pic_marking IDR syntax.

2. **find_short view mismatch**: Fixed by adding view_id matching in
   `find_short()` -- two MVC views share the same frame_num, so matching
   frame_num alone returned the wrong picture.

3. **Thread context update**: Fixed by copying `picture_idr`, `cur_view_id`,
   `idr_pic_flag`, `mvc_active`, and per-view reorder state in
   `ff_h264_update_thread_context()`.

4. **Per-view POC context**: Split `h->poc` into base and dependent view
   contexts (`h->poc` + `h->dep_view_poc`).

5. **NAL type 20 in get_last_needed_nal**: Added EXTEN_SLICE handling with
   4-byte header offset (1 NAL header + 3 MVC extension) to correctly reach
   first_mb_in_slice.

6. **False errors from dep view packets**: Fixed by checking `h->mvc_active`
   in the "no frame!" error path.

7. **Inter-view prediction**: Fully implemented. Base view picture is
   appended to dep view ref list, protected by MVC_IV_REF during decode.
   Both idc=4 (inter-view short-term) and idc=5 (inter-view long-term)
   ref list modifications are handled.

8. **Packet ordering**: Fixed with accumulate-and-drain pattern. Dep view
   PES packets are buffered in `mvc_pending_pkts` and decoded after the
   base view packet.

9. **Interlaced MVC**: Working. Per-view reorder buffers
   (`delayed_pic_dep[]`, `last_pocs_dep[]`, `next_outputed_poc_dep`)
   prevent cross-view POC drops. `pic_as_field()` adapts inter-view refs
   for field pictures.

10. **MMCO_RESET infinite loop**: Fixed by iterating `short_ref[]`
    backwards with `remove_short_at_index()` directly (view-filtered),
    instead of using `remove_short()` which calls `find_short()` and
    can't find cross-view entries.

11. **GOP boundary stale dep slices**: Fixed with `mvc_base_idr_decoded`
    flag. After base IDR, non-anchor dep slices from the old GOP are
    skipped.

12. **h264_register_view_id realloc dangling pointer**: Fixed by
    reassigning both `view_ids_available` and `view_pos_available`
    after each realloc.

### Known Issues (Remaining)

1. **Non-monotonic DTS**: Both views output frames with the same timestamps,
   triggering "non monotonically increasing dts" muxer warnings. This is
   expected with frame-sequential stereo output (same as HEVC multiview).

2. **Interlaced "Missing reference" warnings**: ~3-24 per interlaced stream
   at GOP boundaries where dep view MMCO operates on pictures already
   cleared by base view. Cosmetic -- fallback refs are used, output is
   correct.

3. **SSIF first dep frame lost**: In SSIF files, lazy linking happens after
   the base view is probed. The first 1-2 dep view packets arrive before
   linking and are dropped.

4. **View specifiers** (`-map 0:v:vpos:left`): Don't work for MPEG-TS MVC
   because Subset SPS arrives in a separate PES packet after `get_format()`
   already fired. Use `-view_ids -1` instead.

5. **1 dep frame lost at EOF**: For interlaced streams, one dep view frame
   may remain in the per-view reorder buffer at EOF.

6. **Frame threading + MVC**: Dep view is skipped (auto-detected and warned).
   Slice threading works fine.

7. **>2 views**: Only stereo (2 views) tested. Inter-view ref lookup uses
   `view_id != cur_view_id`, which works for 2 views but may need refinement
   for >2 views.

8. **Hardware acceleration**: MVC slices use software decode path only.

### All Implementation Complete

- [x] MPEG-TS PID merging + SSIF lazy linking
- [x] NAL type 14/15/20 handling in decoder
- [x] Per-view POC contexts
- [x] Inter-view reference prediction (idc=4 and idc=5)
- [x] MVC_IV_REF protection for inter-view refs
- [x] Per-view reorder buffers (interlaced MVC)
- [x] Per-view sliding window and DPB overflow check
- [x] Per-view MMCO_RESET (view-filtered iteration)
- [x] Accumulate-and-drain packet reordering
- [x] Output FIFO (receive_frame)
- [x] view_ids option + AV_FRAME_DATA_VIEW_ID side data
- [x] Stereo3D side data
- [x] Frame threading auto-disable
- [x] GOP boundary stale dep slice detection
- [x] FATE regression test
- [x] Full code audit (all MVC files reviewed)


## 11. File Map

### Modified Files

| File                          | Changes                                      |
|-------------------------------|----------------------------------------------|
| `libavformat/mpegts.c`        | MVC PID merging in `pmt_cb()`, SSIF lazy linking |
| `libavcodec/h264dec.h`        | `H264Picture.view_id`, `MVC_IV_REF`, per-view reorder buffers (`delayed_pic_dep[]`, `last_pocs_dep[]`, `next_outputed_poc_dep`), `H264Context.{cur_view_id,idr_pic_flag,mvc_active,mvc_base_pic,mvc_base_idr_decoded,dep_view_poc,mvc_pending_pkts,view_ids,output_fifo}` |
| `libavcodec/h264dec.c`        | NAL type 14/15/20 handling, `receive_frame`, output FIFO, accumulate-and-drain (`h264_is_dep_view_packet`, `h264_drain_mvc_pending`), `view_ids` option, `AV_FRAME_DATA_VIEW_ID` + `AVStereo3D` side data, `get_last_needed_nal` NAL 20, per-view reorder in `h264_select_output_frame`, `h264_flush_delayed_to_fifo`, frame threading auto-disable |
| `libavcodec/h264_slice.c`     | `pic->view_id`, `idr_pic_flag` decoupling, thread context update (incl. per-view reorder state), per-view POC selection, dep view anchor POC reset, `mvc_base_idr_decoded` stale-slice skip |
| `libavcodec/h264_picture.c`   | `view_id` propagation in `h264_copy_picture_params`, per-view POC in `ff_h264_field_end` |
| `libavcodec/h264_refs.c`      | Per-view ref lists (`find_short` view filter), inter-view refs (`mvc_base_pic` + DPB fallback), `MVC_IV_REF` check, `pic_as_field()` for interlaced, per-view sliding window, per-view DPB overflow, per-view MMCO_RESET (view-filtered iteration), `unreference_pic` delayed_pic_dep check, `ff_h264_remove_view_refs()`, idc=4/5 ref list modification |
| `libavcodec/h264_parser.c`    | MVC NAL type 20 frame boundary (4-byte header offset), Subset SPS parsing, idc 4/5 in reordering |

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

---

## Implementation Status

| Area | Status | Description |
|------|--------|-------------|
| CBS MVC parsing | Done | Bitstream parsing for Subset SPS, NAL type 14/20 |
| H.264 parser | Done | MVC NAL type recognition in the parser |
| extract_extradata BSF | Done | MVC NAL support for extradata extraction |
| MPEG-TS stream merging | Done | Merges dep view PID into base H.264 stream |
| MPEG-TS SSIF lazy linking | Done | Blu-ray 3D SSIF file support |
| MVC slice decoding | Done | NAL type 20 rewrite to SLICE, full decode |
| Per-view POC | Done | Independent POC tracking per view |
| Per-view reference mgmt | Done | DPB filtering by view_id, inter-view refs |
| Output FIFO (receive_frame) | Done | Multi-frame output for multiview |
| MVC packet reordering | Done | Fixes MPEG-TS PES ordering issues |
| view_ids option | Done | User selects which views to decode |
| AV_FRAME_DATA_VIEW_ID | Done | View ID side data on output frames |
| Stereo3D side data | Done | AV_STEREO3D_FRAMESEQUENCE with left/right |
| FATE test | Done | 52-frame regression test |

## Known Limitations

See Section 10 for the full list. Summary:

- **Frame threading + MVC**: dep view auto-skipped (single-DPB design)
- **>2 views**: only stereo (2 views) tested
- **MVC in MP4/MKV**: only MPEG-TS tested; may need container work
- **Hardware acceleration**: MVC slices use software decode path only
- **SSIF**: 1-2 frames lost at stream start (lazy linking timing)
- **View specifiers** (`-map 0:v:vpos:left`): don't work for MPEG-TS MVC;
  use `-view_ids -1` instead
- **Non-monotonic DTS**: expected with frame-sequential stereo output
- **Interlaced "Missing reference" warnings**: cosmetic, at GOP boundaries
- **1 dep frame lost at EOF**: interlaced streams, stays in reorder buffer

## Future Work

1. **Clean patch series**: reimplement from `doc/mvc_upstream_guide.md`
2. **MP4/MKV container support**: if needed based on sample testing
3. **Hardware acceleration**: extend hwaccel backends for MVC NAL types
4. **>2 views**: extend inter-view ref lookup for multiview (>2)

Note: other stereoscopic 3D formats (frame-packing, side-by-side, top-bottom)
are already supported by FFmpeg and do not require MVC decoding. MVC is
specifically for Blu-ray 3D and similar multi-PID multiview bitstreams.
