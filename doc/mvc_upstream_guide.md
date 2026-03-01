# H.264 MVC Upstream Patch Guide

Reference implementation guide for rewriting the 21 development commits
into a clean 11-patch series for ffmpeg-devel submission.

Each section = one patch. Includes every code change needed.

---

## Patch 1/11: avcodec/cbs_h264: add MVC bitstream parsing

Already clean. Cherry-pick `3739c6354d`.

## Patch 2/11: tests: add H.264 MVC CBS API test

Already clean. Cherry-pick `326253d6e7`.

## Patch 3/11: avcodec/h264_parser: add MVC NAL type support

Already clean. Cherry-pick `496536d5fc`.

---

## Patch 4/11: avformat/mpegts: merge MVC dependent view PID

**File: `libavformat/mpegts.c`**

### 4a. MVC entry in stream type table (line ~815)

Already exists upstream:
```c
{ STREAM_TYPE_VIDEO_MVC,      AVMEDIA_TYPE_VIDEO, AV_CODEC_ID_H264       },
```

### 4b. `merged_st` flag in PESContext (line ~275)

Add a flag so mpegts_close_filter doesn't free a shared stream pointer:
```c
typedef struct PESContext {
    ...
    SLConfigDescr sl;
    int merged_st;       /* stream pointer shared with another PES */
} PESContext;
```

In `mpegts_close_filter()` (~line 574), use it:
```c
if (!pes->st || pes->merged_st) {
    av_freep(&filter->u.pes_filter.opaque);
}
```

### 4c. PMT callback: intercept MVC stream_type (in `pmt_cb`, ~line 2604)

When processing the descriptor loop, intercept `stream_type == 0x20` BEFORE
the normal stream creation. Create a bare PES filter (no AVStream) and skip
the descriptor loop:

```c
if (stream_type == STREAM_TYPE_VIDEO_MVC) {
    if (ts->pids[pid] && ts->pids[pid]->type == MPEGTS_PES) {
        pes = ts->pids[pid]->u.pes_filter.opaque;
    } else {
        mpegts_close_filter(ts, ts->pids[pid]);
        pes = add_pes_stream(ts, pid, pcr_pid);
        if (!pes)
            goto out;
    }
    pes->stream_type = stream_type;
    add_pid_to_program(prg, pid);
    /* Skip the descriptor loop for MVC -- we don't need codec
     * info since the base stream already has it. */
    desc_list_len = get16(&p, p_end);
    if (desc_list_len < 0)
        break;
    desc_list_len &= 0xfff;
    p += desc_list_len;
    continue;  /* skip normal stream creation */
}
```

### 4d. PMT callback: link MVC PES to base H.264 stream (after the per-stream loop)

Two-pass linking at the end of pmt_cb, after all streams are created:

```c
/*
 * Link MVC dependent view PES to the base H.264 stream.
 *
 * Two-pass approach: first find the base H.264 PES (stream_type 0x1B),
 * then link all MVC PES (stream_type 0x20) to share the base stream's
 * AVStream pointer.
 *
 * merged_st = 1 tells mpegts_close_filter() not to free the PES
 * context's stream pointer (since it's shared, not owned).
 *
 * After this, packets from the dependent view PID are delivered
 * on the same AVStream as the base view -- the decoder sees both.
 */
for (int i = 0; i < prg->nb_pids; i++) {
    int p_pid = prg->pids[i];
    PESContext *base_pes = NULL;

    if (p_pid >= NB_PID_MAX || !ts->pids[p_pid] ||
        ts->pids[p_pid]->type != MPEGTS_PES)
        continue;

    PESContext *cand = ts->pids[p_pid]->u.pes_filter.opaque;
    if (cand->stream_type == STREAM_TYPE_VIDEO_H264 && cand->st) {
        base_pes = cand;
        /* Pass 2: Link all MVC PES to the base stream */
        for (int j = 0; j < prg->nb_pids; j++) {
            int m_pid = prg->pids[j];
            if (m_pid < NB_PID_MAX && ts->pids[m_pid] &&
                ts->pids[m_pid]->type == MPEGTS_PES) {
                PESContext *m_pes = ts->pids[m_pid]->u.pes_filter.opaque;
                if (m_pes->stream_type == STREAM_TYPE_VIDEO_MVC && !m_pes->st) {
                    m_pes->st = base_pes->st;
                    m_pes->merged_st = 1;
                    av_log(ts->stream, AV_LOG_VERBOSE,
                           "MVC: merged dependent view PID 0x%x "
                           "with base H.264 PID 0x%x\n",
                           m_pes->pid, base_pes->pid);
                }
            }
        }
        break;
    }
}
```

### 4e. SSIF lazy linking (in `new_pes_packet`, ~line 1221)

When a PES has stream_type=0x20 but no `st` pointer yet (SSIF: base and dep
view appear in separate alternating PMTs), lazy-link on first data arrival:

```c
/* MVC dependent view (SSIF): link to base H.264
 * stream instead of creating a separate stream.
 * In Blu-ray 3D SSIF files, the base and dependent
 * view PIDs appear in separate alternating PMTs,
 * so the pmt_cb linking code cannot match them.
 * Lazy-link here when data arrives. */
if (pes->stream_type == STREAM_TYPE_VIDEO_MVC) {
    for (unsigned i = 0; i < ts->stream->nb_streams; i++) {
        AVStream *cand = ts->stream->streams[i];
        if (cand->codecpar->codec_id == AV_CODEC_ID_H264) {
            pes->st = cand;
            pes->merged_st = 1;
            av_log(ts->stream, AV_LOG_VERBOSE,
                   "MVC: linked dependent view PID 0x%x "
                   "to H.264 stream\n", pes->pid);
            break;
        }
    }
}
```

---

## Patch 5/11: avcodec/h264dec: handle MVC NAL types in decoder

**Files: `h264dec.h`, `h264dec.c`**

### 5a. Add MVC fields to H264Context (h264dec.h)

```c
/* After H264POCContext poc; */

/**
 * POC context for MVC dependent view (view_id > 0).
 *
 * In MVC, each view maintains independent POC state (poc_lsb,
 * poc_msb, delta_poc_bottom, prev_poc_msb/lsb, etc.) because
 * each view's slice headers carry their own POC parameters.
 * They share the same frame_num within an access unit, but
 * the prev_* tracking must be per-view to avoid the dependent
 * view overwriting the base view's POC state.
 *
 * h->poc is always the base view (view_id == 0).
 * h->dep_view_poc is for the first dependent view (view_id > 0).
 *
 * Code in h264_field_start, ff_h264_field_end, and MMCO_RESET
 * selects the correct context via:
 *   H264POCContext *poc = h->cur_view_id ? &h->dep_view_poc : &h->poc;
 *
 * For non-MVC streams, dep_view_poc is never used.
 */
H264POCContext dep_view_poc;
```

```c
/* After the threading fields */

/**
 * MVC: view_id of the NAL currently being decoded.
 *
 * Set in decode_nal_units() (h264dec.c):
 * - To 0 for NAL types SLICE/IDR_SLICE (base view).
 * - To the extension header's view_id for NAL type EXTEN_SLICE.
 *
 * Propagated to H264Picture.view_id when a new picture is allocated.
 * Used throughout h264_refs.c to filter references by view.
 *
 * For non-MVC streams, always 0.
 */
int cur_view_id;

/**
 * IdrPicFlag for MVC (H.264 Annex H, H.7.4.1.1).
 *
 * For NAL type 20 (EXTEN_SLICE), IdrPicFlag = !non_idr_flag.
 * When set, the slice header contains idr_pic_id and POC is
 * reset, even though nal_unit_type != 5 and the slice may be
 * a P-slice (inter-view predicted).
 *
 * For regular NAL types 1-5, this equals (nal_unit_type == 5).
 * Set in decode_nal_units() before calling queue_decode_slice().
 */
int idr_pic_flag;

/**
 * Set to 1 when MVC extension slices (NAL type 20) have been seen.
 * Used to decide whether to add AV_FRAME_DATA_VIEW_ID side data
 * and populate view_ids_available.
 */
int mvc_active;
```

### 5b. Add view_id to H264Picture (h264dec.h)

```c
/* In H264Picture, after the 'gray' field */

/**
 * MVC view identifier (H.264 Annex H).
 *
 * 0 = base view (standard AVC). >0 = dependent view (MVC extension).
 * Extracted from the 3-byte NAL extension header of NAL type 20
 * (EXTEN_SLICE). Set in ff_h264_queue_decode_slice() when the picture
 * is allocated (h264_slice.c).
 *
 * Used by reference management (h264_refs.c) to filter the shared DPB
 * into per-view reference lists. Both views in the same access unit
 * share the same frame_num but have different view_ids.
 *
 * For non-MVC streams, this is always 0 (zero-initialized by memset
 * in ff_h264_unref_picture).
 */
int view_id;
```

### 5c. Handle NAL types in decode_nal_units (h264dec.c)

In the NAL type switch inside `decode_nal_units()`:

```c
case H264_NAL_SUB_SPS: {
    /*
     * MVC Subset SPS (NAL type 15, H.264 Annex H, H.7.3.2.1.4).
     *
     * Contains a regular SPS followed by MVC-specific extensions
     * (num_views, view dependencies, applicable operation points).
     * We parse the regular SPS for resolution/profile info, then
     * read view_ids from the MVC extension to populate
     * view_ids_available early (needed for view specifiers in
     * fftools before the first EXTEN_SLICE arrives).
     */
    GetBitContext tmp_gb = nal->gb;
    int profile_idc = nal->data[1]; /* first SPS byte after NAL hdr */

    ff_h264_decode_seq_parameter_set(&tmp_gb, avctx, &h->ps, 0);

    /*
     * Parse the MVC extension to extract view_ids (H.7.3.2.1.4).
     * After the SPS data, for profiles 118/128:
     *   bit_equal_to_one (1), num_views_minus1 (ue),
     *   view_id[0..num_views_minus1] (ue each).
     */
    if ((profile_idc == 118 || profile_idc == 128) &&
        get_bits_left(&tmp_gb) > 1) {
        unsigned num_views;

        skip_bits(&tmp_gb, 1);  /* bit_equal_to_one */
        num_views = get_ue_golomb_long(&tmp_gb) + 1;
        if (num_views > 0 && num_views <= 1024 &&
            get_bits_left(&tmp_gb) >= 0) {
            if (!h->mvc_active)
                h->mvc_active = 1;
            for (unsigned i = 0; i < num_views; i++) {
                int vid = get_ue_golomb_long(&tmp_gb);
                if (vid >= 0 && vid <= 1023) {
                    ret = h264_register_view_id(h, vid);
                    if (ret < 0)
                        goto end;
                }
            }
        }
    }
    break;
}
case H264_NAL_PREFIX:
    /*
     * MVC Prefix NAL (NAL type 14).
     *
     * Precedes base view slices in MVC bitstreams. Contains the
     * 3-byte MVC extension header. We don't need it because the
     * base view is identified by NAL type (1 or 5), not by the
     * prefix.
     */
    break;
```

### 5d. Handle NAL type 20 in get_last_needed_nal (h264dec.c)

In the switch inside `get_last_needed_nal()`, add EXTEN_SLICE alongside
SLICE/IDR_SLICE:

```c
case H264_NAL_EXTEN_SLICE: {
    /*
     * NAL type 20 (EXTEN_SLICE) has a 3-byte MVC extension header
     * between the 1-byte NAL header and the slice header.
     * Skip it to reach first_mb_in_slice.
     */
    int hdr = nal->type == H264_NAL_EXTEN_SLICE ? 4 : 1;
    ret = init_get_bits8(&gb, nal->data + hdr, nal->size - hdr);
    ...
}
```

And for the SPS/PPS needed check:
```c
case H264_NAL_SUB_SPS:   /* fall through -- treat like SPS */
```

### 5e. Set cur_view_id and idr_pic_flag for base view slices

In the IDR_SLICE/SLICE handler:
```c
case H264_NAL_IDR_SLICE:
    ...
case H264_NAL_SLICE:
    h->cur_view_id = 0;
    h->idr_pic_flag = (nal->type == H264_NAL_IDR_SLICE);
    ...
```

### 5f. Copy MVC fields in ff_h264_update_thread_context (h264_slice.c, ~line 440)

```c
h->cur_view_id     = h1->cur_view_id;
h->idr_pic_flag    = h1->idr_pic_flag;
h->mvc_active      = h1->mvc_active;
```

And copy dep_view_poc (~line 423):
```c
memcpy(&h->dep_view_poc, &h1->dep_view_poc, sizeof(h->dep_view_poc));
```

---

## Patch 6/11: avcodec/h264: per-view POC and reference context

**Files: `h264_slice.c`, `h264_picture.c`, `h264_refs.c`**

### 6a. Per-view POC in h264_field_start (h264_slice.c, ~line 1410)

At the start of `h264_field_start()`, select POC context:
```c
H264POCContext *const poc = h->cur_view_id ? &h->dep_view_poc : &h->poc;
```

Then use `poc->` everywhere instead of `h->poc.` in this function:
- `poc->frame_num`, `poc->poc_lsb`, `poc->delta_poc_bottom`, etc.
- `poc->prev_frame_num`, `poc->frame_num_offset`
- The ff_h264_init_poc call: pass `poc` instead of `&h->poc`

### 6b. Per-view POC in ff_h264_field_end (h264_picture.c, ~line 199)

```c
H264POCContext *const poc = h->cur_view_id ? &h->dep_view_poc : &h->poc;
if (!h->droppable) {
    err = ff_h264_execute_ref_pic_marking(h);
    poc->prev_poc_msb = poc->poc_msb;
    poc->prev_poc_lsb = poc->poc_lsb;
}
poc->prev_frame_num_offset = poc->frame_num_offset;
poc->prev_frame_num        = poc->frame_num;
```

### 6c. Per-view POC in MMCO_RESET (h264_refs.c, ~line 931)

```c
case MMCO_RESET: {
    H264POCContext *const poc = h->cur_view_id ? &h->dep_view_poc : &h->poc;
    ...
```

### 6d. Per-view POC in multi-slice frame_num check (h264_slice.c, ~line 1779)

```c
const H264POCContext *poc = h->cur_view_id ? &h->dep_view_poc : &h->poc;
if (poc->frame_num != sl->frame_num) {
```

### 6e. Tag picture with view_id (h264_slice.c, after pic allocation ~line 542)

```c
h->cur_pic_ptr = pic;
/* MVC: Tag the newly allocated picture with the current view_id.
 * This is how we track which view each picture in the DPB belongs to.
 * For non-MVC streams, cur_view_id is always 0. */
pic->view_id = h->cur_view_id;
```

### 6f. Propagate view_id in h264_copy_picture_params (h264_picture.c, ~line 106)

```c
dst->view_id       = src->view_id;   /* MVC: propagate view_id for thread context updates */
```

### 6g. picture_idr logic for MVC (h264_slice.c, ~line 1671)

```c
/*
 * picture_idr controls DPB clearing in execute_ref_pic_marking().
 * For MVC dependent views, the base view IDR already cleared the DPB.
 * Setting picture_idr for a dep view anchor would incorrectly remove
 * the base view picture that the dep view needs for inter-view prediction.
 */
h->picture_idr = h->idr_pic_flag && !h->cur_view_id;
```

### 6h. Use idr_pic_flag everywhere instead of (nal_unit_type == 5)

Replace all `h->nal_unit_type == H264_NAL_IDR_SLICE` checks with
`h->idr_pic_flag` in h264_slice.c:

- idr_pic_id parsing (~line 1815): `if (h->idr_pic_flag)`
- ref_pic_marking call (~line 1879): pass `h->idr_pic_flag`
- KEY flag (~line 1688): `AV_FRAME_FLAG_KEY * !!h->idr_pic_flag`
- recovery (~line 1690): `if (h->idr_pic_flag)`
- skip_frame check (~line 2190): `!h->idr_pic_flag`
- debug logging (~line 2078): `h->idr_pic_flag ? " IDR" : ""`
- IDR/non-IDR mix check (~line 1950): `h->picture_idr && !h->idr_pic_flag`

---

## Patch 7/11: avcodec/h264: decode MVC extension slices

**Files: `h264dec.c`, `h264_slice.c`, `h264_refs.c`**

### 7a. EXTEN_SLICE handler in decode_nal_units (h264dec.c)

```c
case H264_NAL_EXTEN_SLICE: {
    /*
     * MVC Extension Slice (NAL type 20, H.264 Annex G/H).
     *
     * NAL header layout (3 bytes after standard 1-byte NAL header):
     *   svc_extension_flag  (1)  -- 0=MVC, 1=SVC
     *   non_idr_flag        (1)  -- 0=IDR, 1=non-IDR
     *   priority_id         (6)  -- sub-bitstream priority
     *   view_id             (10) -- identifies the view
     *   temporal_id         (3)  -- temporal scalability layer
     *   anchor_pic_flag     (1)  -- 1 if anchor (similar to IDR)
     *   inter_view_flag     (1)  -- 1 if used for inter-view prediction
     *   reserved_one_bit    (1)  -- must be 1
     *
     * Strategy: Parse the extension header, extract view_id, then
     * rewrite nal->type to IDR_SLICE or SLICE. This lets the
     * standard slice header parsing (ff_h264_queue_decode_slice)
     * handle it without changes.
     */
    int svc_ext, non_idr_flag;

    svc_ext = get_bits1(&nal->gb);
    if (svc_ext)
        break;  /* SVC not supported */

    non_idr_flag = get_bits1(&nal->gb);
    skip_bits(&nal->gb, 6);            // priority_id
    h->cur_view_id = get_bits(&nal->gb, 10);
    skip_bits(&nal->gb, 6);            // temporal_id(3), anchor(1), inter_view(1), reserved(1)

    if (!h->mvc_active) {
        h->mvc_active = 1;
        /* Register base view (view_id=0) when MVC is first detected */
        ret = h264_register_view_id(h, 0);
        if (ret < 0)
            goto end;
    }
    ret = h264_register_view_id(h, h->cur_view_id);
    if (ret < 0)
        goto end;

    /*
     * view_ids filtering: skip dependent views not requested.
     */
    if (h->nb_view_ids == 0) {
        /* Default: base view only */
        break;
    } else if (!(h->nb_view_ids == 1 && h->view_ids[0] == -1)) {
        int found = 0;
        for (unsigned j = 0; j < h->nb_view_ids; j++) {
            if (h->view_ids[j] == h->cur_view_id) {
                found = 1;
                break;
            }
        }
        if (!found)
            break;
    }

    /* Skip dep view under frame threading (needs base view in same thread) */
    if (avctx->active_thread_type & FF_THREAD_FRAME) {
        av_log(avctx, AV_LOG_DEBUG,
               "MVC: skipping dep view %d (frame threading active)\n",
               h->cur_view_id);
        break;
    }

    /* Skip dep view for interlaced content (PATCHWELCOME in core H.264) */
    if (h->ps.sps && FIELD_PICTURE(h)) {
        av_log(avctx, AV_LOG_DEBUG,
               "MVC: skipping dep view %d (interlaced not supported)\n",
               h->cur_view_id);
        break;
    }

    /*
     * Rewrite NAL type: the slice header after the extension header
     * is standard H.264. By rewriting to IDR_SLICE or SLICE, we
     * reuse all existing slice decoding code unchanged.
     */
    nal->type = non_idr_flag ? H264_NAL_SLICE : H264_NAL_IDR_SLICE;
    h->nal_unit_type = nal->type;

    h->idr_pic_flag = !non_idr_flag;

    if (!non_idr_flag) {
        if (!idr_cleared)
            idr(h);
        idr_cleared = 1;
        h->has_recovery_point = 1;
    }

    h->has_slice = 1;
    if ((err = ff_h264_queue_decode_slice(h, nal))) {
        H264SliceContext *sl = h->slice_ctx + h->nb_slice_ctx_queued;
        sl->ref_count[0] = sl->ref_count[1] = 0;
        break;
    }
    /* ... same post-slice logic as IDR/SLICE case ... */
    break;
}
```

### 7b. View transition in h264_field_start (h264_slice.c, ~line 2140)

When a new NAL's view_id differs from the current picture's view_id,
close the current picture so it goes into short_ref:

```c
} else if (h->cur_pic_ptr && !FIELD_PICTURE(h) && !h->first_field &&
           (h->idr_pic_flag ||
            h->cur_pic_ptr->view_id != h->cur_view_id)) {
    /*
     * Two cases reach here:
     * 1) Broken frame packetizing: an IDR slice starts a new
     *    picture while a non-IDR was in progress.
     * 2) MVC view transition: the current picture belongs to a
     *    different view than the incoming NAL. This is normal
     *    for MVC -- each access unit has back-to-back pictures
     *    from different views (e.g., dep IDR followed by base P).
     *    Without this, field_end would never be called for the
     *    current picture, so it would never be added to short_ref.
     */
    if (h->cur_pic_ptr->view_id == h->cur_view_id)
        av_log(h->avctx, AV_LOG_WARNING, "Broken frame packetizing\n");
    /*
     * With frame threading, the thread update function runs
     * execute_ref_pic_marking. Passing in_setup=1 here would
     * cause it to run twice, triggering "illegal short term
     * buffer state". Only run it here for non-frame-threaded.
     */
    ret = ff_h264_field_end(h, h->slice_ctx,
                            !(h->avctx->active_thread_type & FF_THREAD_FRAME));
    ff_thread_report_progress(&h->cur_pic_ptr->tf, INT_MAX, 0);
    ff_thread_report_progress(&h->cur_pic_ptr->tf, INT_MAX, 1);
    h->cur_pic_ptr = NULL;
    if (ret < 0)
        return ret;
}
```

### 7c. Per-view reference list filtering (h264_refs.c, h264_initialise_ref_list)

Filter the shared DPB into per-view arrays before building ref lists:

```c
static void h264_initialise_ref_list(H264Context *h, H264SliceContext *sl)
{
    int len;

    /*
     * MVC per-view DPB filtering (H.264 Annex H, H.8.4).
     *
     * The shared DPB may contain e.g.:
     *   short_ref[0]: view_id=0, fn=5  (base view)
     *   short_ref[1]: view_id=1, fn=5  (dependent view)
     *
     * If cur_view_id=1, view_short_ref[] will contain only [1].
     */
    H264Picture *view_short_ref[32];
    int view_short_count = 0;
    H264Picture *view_long_ref[32] = { NULL };

    for (int i = 0; i < h->short_ref_count; i++) {
        if (h->short_ref[i]->view_id == h->cur_view_id)
            view_short_ref[view_short_count++] = h->short_ref[i];
    }
    for (int i = 0; i < 16; i++) {
        if (h->long_ref[i] && h->long_ref[i]->view_id == h->cur_view_id)
            view_long_ref[i] = h->long_ref[i];
    }

    /* Use view_short_ref/view_short_count and view_long_ref
     * instead of h->short_ref/h->short_ref_count/h->long_ref
     * in the existing ref list building code below. */
    ...
```

### 7d. Inter-view reference (h264_refs.c, end of h264_initialise_ref_list)

After building normal ref lists, add the base view picture as an
additional reference for dependent view slices:

```c
/*
 * MVC inter-view reference picture (H.264 Annex H, H.8.4).
 *
 * For a dependent view slice, add the base view's picture
 * from the same access unit as an additional reference.
 */
if (h->cur_view_id) {
    H264Picture *iv_ref = NULL;
    int cur_frame_num = h->cur_pic_ptr->frame_num;

    /*
     * Search the full DPB (not just short_ref[]) for the base view
     * (view_id=0) picture with matching frame_num.
     * We search the full DPB because droppable base view frames
     * (nal_ref_idc==0) have reference=0 or DELAYED_PIC_REF and
     * are NOT in short_ref[], but they ARE still in the DPB.
     */
    for (int i = 0; i < H264_MAX_PICTURE_COUNT; i++) {
        if (h->DPB[i].f->buf[0] &&
            h->DPB[i].view_id == 0 &&
            h->DPB[i].frame_num == cur_frame_num) {
            iv_ref = &h->DPB[i];
            break;
        }
    }

    if (iv_ref) {
        for (int list = 0; list < sl->list_count; list++) {
            int pos = sl->ref_count[list];
            if (pos < FF_ARRAY_ELEMS(sl->ref_list[list])) {
                ref_from_h264pic(&sl->ref_list[list][pos], iv_ref);
                sl->ref_count[list]++;
                /*
                 * Droppable base view frame has reference=0 or
                 * DELAYED_PIC_REF which would fail validation.
                 * Override to PICT_FRAME so the inter-view ref
                 * passes validation.
                 */
                sl->ref_list[list][pos].reference = PICT_FRAME;
            }
        }
    }
}
```

### 7e. Per-view filtering in ref_pic_list_modification (h264_refs.c)

In `ff_h264_decode_ref_pic_list_reordering()`, add view_id checks:

Short-term ref reordering (~line 462):
```c
if (ref->frame_num == frame_num &&
    ref->view_id == h->cur_view_id &&
    (ref->reference & pic_structure))
```

Long-term ref reordering (~line 485):
```c
if (ref && (ref->reference & pic_structure) &&
    ref->view_id == h->cur_view_id) {
```

Add idc 4 (inter-view short-term ref modification, ~line 495):
```c
case 4: {
    /*
     * MVC inter-view reference picture list modification.
     * (H.264 Annex H, modification_of_pic_nums_idc == 4)
     *
     * For stereo MVC (2 views), abs_diff_view_idx_minus1 is
     * always 0 = "the first other view" = the base view.
     *
     * Find the inter-view reference by searching for a picture
     * with a DIFFERENT view_id but the SAME frame_num.
     */
    int cur_frame_num = h->cur_pic_ptr->frame_num;
    pic_structure = h->picture_structure;
    for (i = h->short_ref_count - 1; i >= 0; i--) {
        ref = h->short_ref[i];
        if (ref->view_id != h->cur_view_id &&
            ref->frame_num == cur_frame_num &&
            (ref->reference & pic_structure))
            break;
    }
    break;
}
case 5:
    /*
     * MVC inter-view long-term reference modification.
     * (modification_of_pic_nums_idc == 5)
     * Not commonly used in stereo MVC. Treat as no-op.
     */
    i = -1;
    break;
```

Update the idc validation to accept 4 and 5 (~line 620):
```c
} else if (op > 5) {
    /* Standard H.264 defines idc 0-3. MVC (Annex H) adds
     * idc 4 (inter-view short-term ref) and 5 (inter-view
     * long-term ref). Values > 5 are invalid. */
```

### 7f. Per-view find_short (h264_refs.c, ~line 681)

```c
if (pic->frame_num == frame_num &&
    pic->view_id == h->cur_view_id) {
```

### 7g. Per-view sliding window (h264_refs.c, ~line 785)

Count only same-view references:
```c
int view_short_count = 0;
int view_long_count = 0;
int oldest_view_idx = -1;

for (int i = 0; i < h->short_ref_count; i++) {
    if (h->short_ref[i]->view_id == h->cur_view_id) {
        view_short_count++;
        oldest_view_idx = i;  /* last match = highest index = oldest */
    }
}
for (int i = 0; i < 16; i++) {
    if (h->long_ref[i] && h->long_ref[i]->view_id == h->cur_view_id)
        view_long_count++;
}
```

Use `view_long_count + view_short_count` in the fullness check,
and `h->short_ref[oldest_view_idx]->frame_num` for the MMCO.

### 7h. Per-view duplicate detection in insert_into_short_ref (h264_refs.c, ~line 980)

```c
H264Picture *pic = NULL;
for (int j = 0; j < h->short_ref_count; j++) {
    if (h->short_ref[j]->frame_num == h->cur_pic_ptr->frame_num &&
        h->short_ref[j]->view_id == h->cur_view_id) {
        pic = h->short_ref[j];
        unreference_pic(h, pic, 0);
        remove_short_at_index(h, j);
        break;
    }
}
if (pic) {
    /*
     * MVC: the parser may split dep view NALs into a separate
     * packet AND also combine some into the base view packet.
     * The accumulate-and-drain mechanism then decodes both,
     * producing a duplicate short_ref entry. Not an error.
     */
    if (!h->mvc_active) {
        av_log(h->avctx, AV_LOG_ERROR,
               "illegal short term buffer state detected\n");
        err = AVERROR_INVALIDDATA;
    }
}
```

### 7i. Per-view DPB overflow check (h264_refs.c, ~line 1029)

```c
int view_short = 0, view_long = 0;
for (int i = 0; i < h->short_ref_count; i++)
    if (h->short_ref[i]->view_id == h->cur_view_id)
        view_short++;
for (int i = 0; i < 16; i++)
    if (h->long_ref[i] && h->long_ref[i]->view_id == h->cur_view_id)
        view_long++;

if (view_long + view_short > FFMAX(h->ps.sps->ref_frame_count, 1)) {
```

And in the overflow removal loop, find same-view refs:
```c
for (i = 0; i < 16; ++i)
    if (h->long_ref[i] &&
        h->long_ref[i]->view_id == h->cur_view_id)
        break;
```
```c
int oldest = -1;
for (int i = 0; i < h->short_ref_count; i++)
    if (h->short_ref[i]->view_id == h->cur_view_id)
        oldest = i;
```

---

## Patch 8/11: avcodec/h264dec: switch to receive_frame with output FIFO

**Files: `h264dec.h`, `h264dec.c`**

### 8a. Add output_fifo and mvc_pending_pkts to H264Context (h264dec.h)

```c
#include "libavutil/container_fifo.h"
#include "packet_internal.h"
```

```c
/**
 * Output FIFO for multi-frame output (MVC multiview).
 *
 * With MVC, a single access unit produces one frame per view.
 * The receive_frame API returns one frame at a time, so decoded
 * frames are pushed here and drained one by one.
 *
 * For non-MVC streams the FIFO contains exactly one frame per
 * decode call, so behavior is equivalent to the old decode_frame API.
 */
AVContainerFifo *output_fifo;

/**
 * Deferred dependent view packets for MVC packet reordering.
 *
 * In MPEG-TS with merged MVC PIDs, the dependent view PES often arrives
 * before the base view PES for the same access unit. h264_receive_frame()
 * accumulates dep-view-only packets here. When a base view packet arrives,
 * it is decoded first, then accumulated dep view packets are drained.
 */
PacketList mvc_pending_pkts;
```

### 8b. Initialize in h264_decode_init (h264dec.c)

```c
h->output_fifo = av_container_fifo_alloc_avframe(0);
if (!h->output_fifo)
    return AVERROR(ENOMEM);
```

### 8c. Free in h264_decode_end (h264dec.c)

```c
av_container_fifo_free(&h->output_fifo);
avpriv_packet_list_free(&h->mvc_pending_pkts);
```

### 8d. Drain in flush callback (h264dec.c)

```c
av_container_fifo_drain(h->output_fifo,
                        av_container_fifo_can_read(h->output_fifo));
avpriv_packet_list_free(&h->mvc_pending_pkts);
```

### 8e. finalize_frame pushes to FIFO instead of returning directly

In `finalize_frame()`, at the end where the frame is ready:
```c
ret = av_container_fifo_write(h->output_fifo, dst, 0);
```

### 8f. h264_decode_packet (refactored from h264_decode_frame)

Extract the core decode logic (parse NALs, decode slices, output) into
`h264_decode_packet(H264Context *h, const AVPacket *avpkt)`. This function
pushes frames to `h->output_fifo`. It does NOT return frames directly.

Important: when `mvc_active` is set and no slices were decoded (dep view NALs
all skipped by view_ids filtering), this is not an error:

```c
if (avctx->skip_frame >= AVDISCARD_NONREF ||
    buf_size >= 4 && !memcmp("Q264", buf, 4) ||
    h->mvc_active)
    /*
     * mvc_active: when MVC dep view packets are merged into the
     * base view stream, some packets contain only EXTEN_SLICE
     * NALs which are skipped by view_ids filtering. These are
     * not errors -- just empty from the base view's perspective.
     */
```

### 8g. h264_is_dep_view_packet helper

Scans raw packet data for start codes. Returns 1 if packet has EXTEN_SLICE
(type 20) NALs but no SLICE/IDR_SLICE (type 1/5) NALs:

```c
static int h264_is_dep_view_packet(const uint8_t *buf, int buf_size)
{
    int has_dep = 0;
    const uint8_t *end = buf + buf_size;

    while (buf < end - 4) {
        if (buf[0] == 0 && buf[1] == 0 && (buf[2] == 1 || (buf[2] == 0 && buf[3] == 1))) {
            int off = (buf[2] == 1) ? 3 : 4;
            if (buf + off < end) {
                int nal_type = buf[off] & 0x1f;
                switch (nal_type) {
                case H264_NAL_SLICE:
                case H264_NAL_IDR_SLICE:
                case H264_NAL_DPA:
                    return 0;  /* has base view slice */
                case H264_NAL_EXTEN_SLICE:
                    has_dep = 1;
                    break;
                }
            }
            buf += off + 1;
        } else {
            buf++;
        }
    }
    return has_dep;
}
```

### 8h. h264_drain_mvc_pending helper

```c
static int h264_drain_mvc_pending(H264Context *h)
{
    AVPacket *pkt = av_packet_alloc();
    int ret = 0;

    if (!pkt)
        return AVERROR(ENOMEM);

    while (avpriv_packet_list_get(&h->mvc_pending_pkts, pkt) >= 0) {
        ret = h264_decode_packet(h, pkt);
        av_packet_unref(pkt);
        if (ret < 0)
            break;
    }
    av_packet_free(&pkt);
    return ret;
}
```

### 8i. h264_receive_frame

```c
static int h264_receive_frame(AVCodecContext *avctx, AVFrame *frame)
{
    H264Context *h = avctx->priv_data;
    AVPacket *avpkt = av_packet_alloc();
    int ret;

    if (!avpkt)
        return AVERROR(ENOMEM);

    if (av_container_fifo_can_read(h->output_fifo))
        goto do_output;

get_packet:
    ret = ff_decode_get_packet(avctx, avpkt);
    if (ret == AVERROR_EOF) {
        ret = h264_drain_mvc_pending(h);
        if (ret < 0)
            goto end;
        /* Flush: decode with NULL packet */
        ret = h264_decode_packet(h, avpkt);
        goto do_output;
    }
    if (ret < 0)
        goto end;

    /* MVC packet reordering: accumulate dep-view-only packets */
    if (h->mvc_active && !h->is_avc &&
        h264_is_dep_view_packet(avpkt->data, avpkt->size)) {
        avpriv_packet_list_put(&h->mvc_pending_pkts, avpkt, NULL, 0);
        goto get_packet;
    }

    /* Decode base view packet */
    ret = h264_decode_packet(h, avpkt);
    av_packet_unref(avpkt);
    if (ret < 0)
        goto end;

    /* Drain accumulated dep-view packets */
    ret = h264_drain_mvc_pending(h);
    if (ret < 0)
        goto end;

do_output:
    if (av_container_fifo_read(h->output_fifo, frame, 0) >= 0)
        ret = 0;
    else
        ret = AVERROR(EAGAIN);

end:
    av_packet_free(&avpkt);
    return ret;
}
```

### 8j. Change codec registration

```c
FF_CODEC_RECEIVE_FRAME_CB(h264_receive_frame),
```

Remove the old `FF_CODEC_DECODE_CB(h264_decode_frame)`.

---

## Patch 9/11: avcodec/h264dec: add view_ids option and side data

**Files: `h264dec.h`, `h264dec.c`**

### 9a. Add view_ids and view_ids_available to H264Context (h264dec.h)

```c
int *view_ids;
unsigned nb_view_ids;

unsigned *view_ids_available;
unsigned nb_view_ids_available;

unsigned *view_pos_available;
unsigned nb_view_pos_available;
```

### 9b. h264_register_view_id helper (h264dec.c)

```c
static int h264_register_view_id(H264Context *h, int view_id)
{
    unsigned *tmp;

    for (unsigned i = 0; i < h->nb_view_ids_available; i++)
        if (h->view_ids_available[i] == (unsigned)view_id)
            return 0;

    tmp = av_realloc_array(h->view_ids_available,
                           h->nb_view_ids_available + 1, sizeof(*tmp));
    if (!tmp)
        return AVERROR(ENOMEM);
    h->view_ids_available = tmp;

    tmp = av_realloc_array(h->view_pos_available,
                           h->nb_view_pos_available + 1, sizeof(*tmp));
    if (!tmp)
        return AVERROR(ENOMEM);
    h->view_pos_available = tmp;

    h->view_ids_available[h->nb_view_ids_available++] = view_id;
    /* MVC convention: view_id 0 = left (base), view_id > 0 = right */
    h->view_pos_available[h->nb_view_pos_available++] =
        view_id == 0 ? AV_STEREO3D_VIEW_LEFT : AV_STEREO3D_VIEW_RIGHT;
    return 0;
}
```

### 9c. Free in h264_decode_end (h264dec.c)

```c
av_freep(&h->view_ids_available);
av_freep(&h->view_pos_available);
```

### 9d. AV_FRAME_DATA_VIEW_ID side data in output_frame (h264dec.c)

In `output_frame()`, after existing side data handling:

```c
if (h->mvc_active) {
    AVFrameSideData *sd = av_frame_side_data_new(
        &dst->side_data, &dst->nb_side_data,
        AV_FRAME_DATA_VIEW_ID, sizeof(int), 0);
    if (!sd) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }
    *(int *)sd->data = srcp->view_id;

    if (!av_frame_get_side_data(dst, AV_FRAME_DATA_STEREO3D)) {
        AVStereo3D *stereo = av_stereo3d_create_side_data(dst);
        if (!stereo) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
        stereo->type = AV_STEREO3D_FRAMESEQUENCE;
        stereo->view = srcp->view_id == 0 ? AV_STEREO3D_VIEW_LEFT
                                           : AV_STEREO3D_VIEW_RIGHT;
    }
}
```

### 9e. View output filtering in finalize_frame (h264dec.c)

```c
if (h->mvc_active && h->nb_view_ids > 0 &&
    !(h->nb_view_ids == 1 && h->view_ids[0] == -1)) {
    int found = 0;
    for (unsigned i = 0; i < h->nb_view_ids; i++) {
        if (h->view_ids[i] == out->view_id) {
            found = 1;
            break;
        }
    }
    if (!found)
        return 0;  /* skip this view */
}
```

### 9f. AVOptions (h264dec.c)

```c
{ "view_ids",
    "Array of view IDs that should be decoded and output; "
    "a single -1 to decode all views (MVC multiview)",
    .offset = OFFSET(view_ids), .type = AV_OPT_TYPE_INT | AV_OPT_TYPE_FLAG_ARRAY,
    .min = -1, .max = INT_MAX, .flags = VD },
{ "view_ids_available",
    "Array of available view IDs is exported here (MVC multiview)",
    .offset = OFFSET(view_ids_available),
    .type = AV_OPT_TYPE_UINT | AV_OPT_TYPE_FLAG_ARRAY,
    .flags = VDX | AV_OPT_FLAG_READONLY },
{ "view_pos_available",
    "Array of view positions for view_ids_available, as AVStereo3DView",
    .offset = OFFSET(view_pos_available),
    .type = AV_OPT_TYPE_UINT | AV_OPT_TYPE_FLAG_ARRAY,
    .flags = VDX | AV_OPT_FLAG_READONLY, .unit = "view_pos" },
    { "unspecified", .type = AV_OPT_TYPE_CONST,
        .default_val = { .i64 = AV_STEREO3D_VIEW_UNSPEC }, .unit = "view_pos" },
    { "left",  .type = AV_OPT_TYPE_CONST,
        .default_val = { .i64 = AV_STEREO3D_VIEW_LEFT },  .unit = "view_pos" },
    { "right", .type = AV_OPT_TYPE_CONST,
        .default_val = { .i64 = AV_STEREO3D_VIEW_RIGHT }, .unit = "view_pos" },
```

Needs `#include "libavutil/stereo3d.h"` and define:
```c
#define VDX (AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_DECODING_PARAM | AV_OPT_FLAG_EXPORT)
```

---

## Patch 10/11: avcodec/h264dec: skip dep view for unsupported configs

This is already integrated into the EXTEN_SLICE handler (Patch 7a):
- Frame threading check
- Interlaced check

If you want a separate patch, extract just those two `if` blocks from
the EXTEN_SLICE case. Otherwise fold them into Patch 7.

---

## Patch 11/11: fate: add H.264 MVC multiview decoding test

### 11a. FATE test definition (tests/fate/h264.mak)

```makefile
FATE_H264-$(call FRAMECRC, MPEGTS, H264, H264_PARSER) += fate-h264-mvc
```

```makefile
fate-h264-mvc:                                    CMD = framecrc -threads 1 -view_ids -1 -i $(TARGET_SAMPLES)/h264-mvc/mvc-progressive.ts -map 0:v -fps_mode passthrough
```

### 11b. Reference file (tests/ref/fate/h264-mvc)

52 frames (26 access units x 2 views), 1920x1080, 24fps.
See current file at `tests/ref/fate/h264-mvc` (57 lines).

### 11c. Sample file

`fate-suite/h264-mvc/mvc-progressive.ts` (3.8MB).
Must be uploaded to `samples.ffmpeg.org`.

---

## Key Design Decisions to Remember

1. **Single shared DPB**: Both views share DPB[36], tagged with view_id.
   Simpler than HEVC's per-layer approach (HEVCLayerContext).

2. **NAL type rewriting**: EXTEN_SLICE (20) -> SLICE (1) or IDR_SLICE (5).
   All existing slice code is reused unchanged.

3. **Accumulate-and-drain**: Dep view packets buffered in PacketList,
   drained after base decode. Fixes MPEG-TS PES ordering.

4. **Inter-view ref**: DPB search for base view picture with matching
   frame_num, added as extra long-term-like ref.

5. **PICT_FRAME override**: Droppable base view frames need
   `reference = PICT_FRAME` for inter-view ref validation.

6. **idr_pic_flag**: Replaces `nal_unit_type == 5` checks. MVC dep view
   anchors are IDR (non_idr_flag=0) but use NAL type 20.

7. **picture_idr**: `idr_pic_flag && !cur_view_id`. Prevents dep view
   anchor from clearing the DPB (base view picture still needed).

---

## Known Limitations (document in commit messages)

- Interlaced MVC: dep view skipped (pre-existing PATCHWELCOME)
- Frame threading: dep view skipped (single-DPB, needs base in same thread)
- SSIF: 1-2 frames lost at stream start (lazy linking timing)
- View specifiers (`-map 0:v:vpos:left`): don't work for MPEG-TS MVC
  because Subset SPS arrives too late; use `-view_ids -1` instead

---

## Testing Checklist

```bash
# Build
make -j$(nproc)

# No regressions
make fate-h264 -j$(nproc)

# CBS test
make fate-api-h264-mvc -j$(nproc)

# MVC decode test
make fate-h264-mvc -j$(nproc)

# Manual: both views
ffmpeg -view_ids -1 -i 00024.MTS -map 0:v -f null - 2>&1 | tail -1
# Expected: 294 frames (147 per view)

# Manual: base view only (default)
ffmpeg -i 00024.MTS -map 0:v -f null - 2>&1 | tail -1
# Expected: 147 frames

# Manual: view ID side data
ffprobe -show_frames -show_entries frame=view_id 00024.MTS 2>/dev/null | head -20

# Manual: slice threading
ffmpeg -threads 4 -thread_type slice -view_ids -1 -i 00024.MTS -map 0:v -f null -

# Manual: SSIF
ffmpeg -view_ids -1 -i small-00000.ssif -map 0:v -f null -
```
