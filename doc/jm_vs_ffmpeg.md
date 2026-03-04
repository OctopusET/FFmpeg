# JM Reference Software vs FFmpeg H.264 Decoder: Feature Gap Analysis

Comprehensive comparison of what JM/JMVC has that FFmpeg's H.264 decoder lacks.
For use as a reference when implementing missing features.

---

## Summary

FFmpeg's H.264 decoder covers **all Main/High profile features** completely.
The gaps are in Extended profile tools and Annex G/H extensions:

| Feature | FFmpeg Status | Real-world Impact | Priority |
|---------|--------------|-------------------|----------|
| **MVC idc=5 (inter-view long-term ref)** | NOT supported | Blu-ray 3D (blocks decoding) | **CRITICAL** |
| **MVC interlaced (MBAFF+MVC)** | NOT supported | Some Blu-ray 3D content | HIGH |
| **MVC frame threading** | Disabled | Performance for MVC | MEDIUM |
| **MVC >2 views** | NOT supported | Research/niche | LOW |
| **MVC SEI messages (types 37-54)** | NOT supported | Metadata only | LOW |
| FMO (Flexible Macroblock Ordering) | NOT supported | Legacy mobile streams | LOW |
| ASO (Arbitrary Slice Ordering) | NOT supported | Depends on FMO | LOW |
| Data Partitioning (NAL 2/3/4) | NOT supported | Legacy error resilience | LOW |
| SP/SI slices | Parsed, not decoded correctly | Never adopted | VERY LOW |
| SVC (Scalable Video Coding) | NOT supported | Never widely adopted | LOW |
| Redundant slices | Parsed, not used | Never adopted | VERY LOW |
| Separate colour planes | NOT supported | Niche | VERY LOW |
| Advanced error concealment | Basic only | FFmpeg's EC is adequate | LOW |

---

## 1. MVC (Multiview Video Coding) -- Annex H

### What FFmpeg already supports (our patches)

| Feature | Status |
|---------|--------|
| NAL type 20 (EXTEN_SLICE) parsing & decode | Done |
| NAL type 14 (PREFIX) -- no-op | Done |
| NAL type 15 (Subset SPS) -- base SPS + view_id extraction | Done |
| Inter-view prediction via idc=4 (short-term ref) | Done |
| Per-view DPB management (view_id filtering) | Done |
| Per-view POC tracking (dep_view_poc) | Done |
| MPEG-TS PID merging (stream_type 0x20) | Done |
| MPEG-TS SSIF lazy linking | Done |
| MVC packet reordering (accumulate-and-drain) | Done |
| Output FIFO (receive_frame, multi-frame) | Done |
| view_ids option + AV_FRAME_DATA_VIEW_ID | Done |
| Stereo3D side data (AV_STEREO3D_FRAMESEQUENCE) | Done |
| FATE test | Done |

### What's missing

#### 1a. idc=5: Inter-view long-term ref list modification (CRITICAL)

**What it is:** H.264 Annex H, modification_of_pic_nums_idc=5. Inserts an
inter-view reference picture (from the base view, same access unit) into
the reference list as a long-term-style reference.

**Why it matters:** Real Blu-ray 3D content (our test ISO, encoded by
CyberLink) uses idc=5 on EVERY dependent view slice. Without it, no
dependent view frames decode correctly.

**Current code:** `libavcodec/h264_refs.c:443-448` -- logs warning, sets
`i = -1`, triggers "reference picture missing during reorder".

**How to fix:** Implement case 5 analogous to case 4 (lines 427-441):
- case 4: searches `short_ref[]` for different view_id, same frame_num
- case 5: should search the DPB or ref list for the inter-view picture
  (already appended by `h264_initialise_ref_list`), same matching criteria
  (different view_id, same frame_num), but following long-term ref semantics

**Reference pattern from real Blu-ray 3D stream:**
```
Anchor P-slices:  [idc=5(abs_diff_view_idx=0), idc=3(end)]
Non-anchor slices: [idc=0(abs_diff_pic_num), idc=5(abs_diff_view_idx=0), idc=3(end)]
```

Note: idc=4 is NEVER used in this sample. Some encoders prefer idc=5.

#### 1b. Interlaced MVC (HIGH priority)

Currently rejected at `h264dec.c:906-911`. Would need:
- MBAFF-aware inter-view ref handling
- Field-pair reference management across views
- Testing with interlaced MVC samples

#### 1c. Frame threading + MVC (MEDIUM priority)

Currently rejected at `h264dec.c:897-903`. Would need:
- Inter-view references available across thread contexts
- Or decode both views in same thread (simpler but less parallel)

#### 1d. Multi-view (>2 views) (LOW priority)

Inter-view ref lookup hardcodes `view_id == 0`. For >2 views:
- Use Subset SPS view dependency info to find correct anchor ref
- Extend h264_initialise_ref_list to add multiple inter-view refs

#### 1e. MVC SEI messages (LOW priority)

| SEI Type | Name | Use |
|----------|------|-----|
| 37 | MVC Scalable Nesting | Container for nested SEI |
| 38 | View Scalability Info | View dependency metadata |
| 39 | Multiview Scene Info | Scene geometry |
| 40 | Multiview Acquisition Info | Camera parameters |
| 44 | Base View Temporal HRD | Timing |
| 46 | Multiview View Position | View position metadata |

Type 46 (view position) could be useful for auto-detecting left/right views.

#### 1f. Full Subset SPS parsing (LOW priority)

Currently only view_ids are extracted. Missing:
- View dependency arrays (anchor_ref_l0/l1, non_anchor_ref_l0/l1)
- Level values per operation point
- Applicable operation point target views

---

## 2. SVC (Scalable Video Coding) -- Annex G

**Status:** NOT supported. `svc_extension_flag == 1` breaks out immediately.

**What it provides:**
- Spatial scalability (multiple resolutions in one stream)
- Temporal scalability (frame rate layers)
- Quality/SNR scalability (CGS, MGS)
- Inter-layer prediction

**Real-world adoption:** Very low. WebRTC moved to VP8/VP9/AV1 temporal
scalability. No consumer video uses H.264 SVC. Some surveillance cameras
and video conferencing systems used it briefly.

**Priority:** LOW. Not worth implementing unless specific demand appears.

---

## 3. FMO (Flexible Macroblock Ordering)

**Status:** NOT supported. Hardcoded `#define FMO 0` in `h264dec.h:55`.

**What it provides:** 7 macroblock ordering types for error resilience:
- Type 0: Interleaved slices
- Type 1: Dispersed
- Type 2: Foreground + leftover
- Type 3-5: Box-out, raster, wipe (changing patterns)
- Type 6: Explicit map

**Current handling:** `slice_group_count > 1` triggers
`avpriv_report_missing_feature(avctx, "FMO")` in PPS parsing.

**Real-world adoption:** Part of Baseline/Extended profile. Some old mobile
video conferencing encoders used it. Extremely rare in modern content.

**Priority:** LOW. Only matters for very old mobile H.264 streams.

---

## 4. ASO (Arbitrary Slice Ordering)

**Status:** NOT supported (depends on FMO).

Allows slices to arrive in non-raster-scan order. Since FMO is not
supported, ASO is implicitly unsupported.

**Priority:** LOW.

---

## 5. Data Partitioning (NAL types 2, 3, 4)

**Status:** NOT supported. Reports `avpriv_request_sample`.

**What it provides:** Splits slice data into three partitions:
- DPA (type 2): Slice header + motion vectors
- DPB (type 3): Intra residual coefficients
- DPC (type 4): Inter residual coefficients

Designed for unequal error protection on unreliable channels.

**Priority:** LOW. Never adopted in practice.

---

## 6. SP/SI Slices (Switching Pictures)

**Status:** Parsed but NOT correctly decoded.

Slice header fields are read (`sp_for_switch_flag`, `slice_qs_delta`), and
`slice_type_nos = slice_type & 3` maps SP->P and SI->I. But the secondary
quantization/transform process (the key feature of SP slices) is NOT
implemented. SP slices decode but produce slightly wrong output.

**Priority:** VERY LOW. No encoder outside JM generates SP/SI slices.

---

## 7. Redundant Slices

**Status:** Parsed but not used for error recovery.

`redundant_pic_cnt_present_flag` and `redundant_pic_cnt` are read from PPS
and slice header, but no logic uses redundant slices to replace missing
primary slices.

**Priority:** VERY LOW.

---

## 8. Fully Supported Features (no gaps)

These are complete in FFmpeg and match JM:

- CABAC / CAVLC entropy coding
- All transform sizes (4x4, 8x8) with custom scaling matrices
- Weighted prediction (all modes)
- MBAFF (macroblock-adaptive frame-field) for non-MVC
- Deblocking filter (all modes)
- VUI parameters (full parsing and use)
- All reference picture marking (MMCO, sliding window)
- B-slice direct prediction (spatial and temporal)
- Multiple reference frames
- Monochrome, 4:2:0, 4:2:2, 4:4:4 chroma formats
- All bit depths (8, 9, 10, 12, 14)
- PAFF (picture-adaptive frame-field)
- Long-term reference pictures
- Arbitrary number of slices per picture

---

## 9. SEI Messages Comparison

| SEI Type | Name | FFmpeg | JM |
|----------|------|--------|-----|
| 0 | Buffering Period | Yes | Yes |
| 1 | Picture Timing | Yes | Yes |
| 2 | Pan-Scan Rectangle | No | Yes |
| 3 | Filler Payload | No (harmless) | Yes |
| 4 | User Data Registered (T.35) | Yes | Yes |
| 5 | User Data Unregistered | Yes | Yes |
| 6 | Recovery Point | Yes | Yes |
| 7 | Dec Ref Pic Marking Repetition | No | Yes |
| 8 | Spare Pic | No | Yes |
| 9 | Scene Info | No | Yes |
| 10-12 | Sub-Seq Info | No | Yes |
| 13-14 | Frame Freeze | No | Yes |
| 19 | Film Grain Characteristics | Yes | Yes |
| 21 | Stereo Video Info | No | Yes |
| 23 | Tone Mapping Info | No | Yes |
| 37-54 | MVC-specific SEIs | No | Yes |
| 45 | Frame Packing Arrangement | Yes | Yes |
| 47 | Display Orientation | Yes | Yes |
| 137 | Mastering Display Colour Volume | Yes | N/A |
| 144 | Content Light Level | Yes | N/A |

---

## 10. Test Sample Analysis: 5mintest.iso (Blu-ray 3D)

### Stream structure

- 1 M2TS file, ~5m35s, 25.5 Mbps, 1920x1080p23.976
- PID 0x1011: Base view (H.264 High, profile 100, level 4.0)
- PID 0x1012: Dep view (MVC Stereo High, profile 128, level 4.0)
- PID 0x1100: PCM Blu-ray audio (48kHz stereo s16)

### Base view characteristics

- Standard H.264 High profile, no MVC-aware NALs at all
- NAL types: AUD, SPS, PPS, IDR, SLICE (no PREFIX, no SEI)
- GOP: 16 pictures, IDR + 15 ref/non-ref P/B slices

### Dependent view characteristics

- Profile 128 (Stereo High), 2 views (view_id 0 and 1)
- NAL types: type 24 (dep view AUD), Subset SPS, PPS, EXTEN_SLICE
- **No I-slices** -- anchor pictures are P-slices referencing base view only
- **B-slices present** (62.5% of slices)
- **6 slices per picture** (multi-slice)
- **idc=5 on EVERY slice** (idc=4 is never used)
- **No PREFIX NALs** in base view stream
- anchor_pic_flag used for IDR-equivalent signaling

### What blocks decoding

**Only idc=5** -- every dependent view slice uses it. The base view
decodes perfectly. The MPEG-TS merging works. The NAL type 20 handling
works. Everything works except the reference list modification step
where idc=5 is stubbed out.

### NAL type 24 (dependent view AUD)

This is a Blu-ray convention: NAL type 24 is used as an access unit
delimiter for the dependent view. It carries 1 byte of payload with
`primary_pic_type` like a standard AUD. Should be handled as a no-op
(silently ignore), which FFmpeg already does via the default case.

### Differences from our previous test sample

| Aspect | Previous (fate-suite) | This ISO |
|--------|----------------------|----------|
| Encoder | Unknown (x264?) | CyberLink PowerDirector |
| Profile | Multiview High (118) | Stereo High (128) |
| idc used | 4 (short-term) | 5 (long-term) |
| Dep view slices | I + P only | P + B (no I slices) |
| Slices per picture | 1 | 6 (multi-slice) |
| PREFIX NALs | Present | Absent |
| B-slices in dep view | No | Yes (62.5%) |
| Duration | ~1s (26 AUs) | 5m35s (816 AUs) |
| NAL type 24 | Absent | Present (dep view AUD) |
