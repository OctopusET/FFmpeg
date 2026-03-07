/*
 * H.26L/H.264/AVC/JVT/14496-10/... decoder
 * Copyright (c) 2003 Michael Niedermayer <michaelni@gmx.at>
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * H.264 / AVC / MPEG-4 part10 codec.
 * @author Michael Niedermayer <michaelni@gmx.at>
 */

#define UNCHECKED_BITSTREAM_READER 1

#include "config_components.h"

#include "libavutil/attributes.h"
#include "libavutil/avassert.h"
#include "libavutil/emms.h"
#include "libavutil/imgutils.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/stereo3d.h"
#include "libavutil/thread.h"
#include "libavutil/video_enc_params.h"

#include "codec_internal.h"
#include "decode.h"
#include "internal.h"
#include "error_resilience.h"
#include "avcodec.h"
#include "h264.h"
#include "h264dec.h"
#include "h2645_parse.h"
#include "h264data.h"
#include "h264_ps.h"
#include "golomb.h"
#include "hwaccel_internal.h"
#include "hwconfig.h"
#include "mpegutils.h"
#include "profiles.h"
#include "rectangle.h"
#include "libavutil/refstruct.h"
#include "thread.h"
#include "threadframe.h"

const uint16_t ff_h264_mb_sizes[4] = { 256, 384, 512, 768 };

int avpriv_h264_has_num_reorder_frames(AVCodecContext *avctx)
{
    H264Context *h = avctx->priv_data;
    return h && h->ps.sps ? h->ps.sps->num_reorder_frames : 0;
}

static void h264_er_decode_mb(void *opaque, int ref, int mv_dir, int mv_type,
                              int (*mv)[2][4][2],
                              int mb_x, int mb_y, int mb_intra, int mb_skipped)
{
    const H264Context *h = opaque;
    H264SliceContext *sl = &h->slice_ctx[0];

    sl->mb_x = mb_x;
    sl->mb_y = mb_y;
    sl->mb_xy = mb_x + mb_y * h->mb_stride;
    memset(sl->non_zero_count_cache, 0, sizeof(sl->non_zero_count_cache));
    av_assert1(ref >= 0);
    /* FIXME: It is possible albeit uncommon that slice references
     * differ between slices. We take the easy approach and ignore
     * it for now. If this turns out to have any relevance in
     * practice then correct remapping should be added. */
    if (ref >= sl->ref_count[0])
        ref = 0;
    if (!sl->ref_list[0][ref].data[0]) {
        av_log(h->avctx, AV_LOG_DEBUG, "Reference not available for error concealing\n");
        ref = 0;
    }
    if ((sl->ref_list[0][ref].reference&3) != 3) {
        av_log(h->avctx, AV_LOG_DEBUG, "Reference invalid\n");
        return;
    }
    fill_rectangle(&h->cur_pic.ref_index[0][4 * sl->mb_xy],
                   2, 2, 2, ref, 1);
    fill_rectangle(&sl->ref_cache[0][scan8[0]], 4, 4, 8, ref, 1);
    fill_rectangle(sl->mv_cache[0][scan8[0]], 4, 4, 8,
                   pack16to32((*mv)[0][0][0], (*mv)[0][0][1]), 4);
    sl->mb_mbaff =
    sl->mb_field_decoding_flag = 0;
    ff_h264_hl_decode_mb(h, &h->slice_ctx[0]);
}

void ff_h264_draw_horiz_band(const H264Context *h, H264SliceContext *sl,
                             int y, int height)
{
    AVCodecContext *avctx = h->avctx;
    const AVFrame   *src  = h->cur_pic.f;
    const AVPixFmtDescriptor *desc;
    int offset[AV_NUM_DATA_POINTERS];
    int vshift;
    const int field_pic = h->picture_structure != PICT_FRAME;

    if (!avctx->draw_horiz_band)
        return;

    if (field_pic && h->first_field && !(avctx->slice_flags & SLICE_FLAG_ALLOW_FIELD))
        return;

    if (field_pic) {
        height <<= 1;
        y      <<= 1;
    }

    height = FFMIN(height, avctx->height - y);

    desc   = av_pix_fmt_desc_get(avctx->pix_fmt);
    vshift = desc->log2_chroma_h;

    offset[0] = y * src->linesize[0];
    offset[1] =
    offset[2] = (y >> vshift) * src->linesize[1];
    for (int i = 3; i < AV_NUM_DATA_POINTERS; i++)
        offset[i] = 0;

    emms_c();

    avctx->draw_horiz_band(avctx, src, offset,
                           y, h->picture_structure, height);
}

void ff_h264_free_tables(H264Context *h)
{
    int i;

    av_freep(&h->intra4x4_pred_mode);
    av_freep(&h->chroma_pred_mode_table);
    av_freep(&h->cbp_table);
    av_freep(&h->mvd_table[0]);
    av_freep(&h->mvd_table[1]);
    av_freep(&h->direct_table);
    av_freep(&h->non_zero_count);
    av_freep(&h->slice_table_base);
    h->slice_table = NULL;
    av_freep(&h->list_counts);

    av_freep(&h->mb2b_xy);
    av_freep(&h->mb2br_xy);

    av_refstruct_pool_uninit(&h->qscale_table_pool);
    av_refstruct_pool_uninit(&h->mb_type_pool);
    av_refstruct_pool_uninit(&h->motion_val_pool);
    av_refstruct_pool_uninit(&h->ref_index_pool);

#if CONFIG_ERROR_RESILIENCE
    av_freep(&h->er.mb_index2xy);
    av_freep(&h->er.error_status_table);
    av_freep(&h->er.er_temp_buffer);
    av_freep(&h->dc_val_base);
#endif

    for (i = 0; i < h->nb_slice_ctx; i++) {
        H264SliceContext *sl = &h->slice_ctx[i];

        av_freep(&sl->bipred_scratchpad);
        av_freep(&sl->edge_emu_buffer);
        av_freep(&sl->top_borders[0]);
        av_freep(&sl->top_borders[1]);

        sl->bipred_scratchpad_allocated = 0;
        sl->edge_emu_buffer_allocated   = 0;
        sl->top_borders_allocated[0]    = 0;
        sl->top_borders_allocated[1]    = 0;
    }
}

int ff_h264_alloc_tables(H264Context *h)
{
    ERContext *const er = &h->er;
    const int big_mb_num = h->mb_stride * (h->mb_height + 1);
    const int row_mb_num = 2*h->mb_stride*FFMAX(h->nb_slice_ctx, 1);
    const int st_size = big_mb_num + h->mb_stride;
    int x, y;

    if (!FF_ALLOCZ_TYPED_ARRAY(h->intra4x4_pred_mode,     row_mb_num * 8)  ||
        !FF_ALLOCZ_TYPED_ARRAY(h->non_zero_count,         big_mb_num)      ||
        !FF_ALLOCZ_TYPED_ARRAY(h->slice_table_base,       st_size)         ||
        !FF_ALLOCZ_TYPED_ARRAY(h->cbp_table,              big_mb_num)      ||
        !FF_ALLOCZ_TYPED_ARRAY(h->chroma_pred_mode_table, big_mb_num)      ||
        !FF_ALLOCZ_TYPED_ARRAY(h->mvd_table[0],           row_mb_num * 8)  ||
        !FF_ALLOCZ_TYPED_ARRAY(h->mvd_table[1],           row_mb_num * 8)  ||
        !FF_ALLOCZ_TYPED_ARRAY(h->direct_table,           big_mb_num * 4)  ||
        !FF_ALLOCZ_TYPED_ARRAY(h->list_counts,            big_mb_num)      ||
        !FF_ALLOCZ_TYPED_ARRAY(h->mb2b_xy,                big_mb_num)      ||
        !FF_ALLOCZ_TYPED_ARRAY(h->mb2br_xy,               big_mb_num))
        return AVERROR(ENOMEM);
    h->slice_ctx[0].intra4x4_pred_mode = h->intra4x4_pred_mode;
    h->slice_ctx[0].mvd_table[0] = h->mvd_table[0];
    h->slice_ctx[0].mvd_table[1] = h->mvd_table[1];
    memset(h->slice_table_base, -1,
           st_size * sizeof(*h->slice_table_base));
    h->slice_table = h->slice_table_base + h->mb_stride * 2 + 1;
    for (y = 0; y < h->mb_height; y++)
        for (x = 0; x < h->mb_width; x++) {
            const int mb_xy = x + y * h->mb_stride;
            const int b_xy  = 4 * x + 4 * y * h->b_stride;

            h->mb2b_xy[mb_xy]  = b_xy;
            h->mb2br_xy[mb_xy] = 8 * (FMO ? mb_xy : (mb_xy % (2 * h->mb_stride)));
        }

    if (CONFIG_ERROR_RESILIENCE) {
        const int er_size = h->mb_height * h->mb_stride * (4*sizeof(int) + 1);
        int mb_array_size = h->mb_height * h->mb_stride;
        int y_size  = (2 * h->mb_width + 1) * (2 * h->mb_height + 1);
        int yc_size = y_size + 2 * big_mb_num;

        /* init ER */
        er->avctx          = h->avctx;
        er->decode_mb      = h264_er_decode_mb;
        er->opaque         = h;
        er->quarter_sample = 1;

        er->mb_num      = h->mb_num;
        er->mb_width    = h->mb_width;
        er->mb_height   = h->mb_height;
        er->mb_stride   = h->mb_stride;
        er->b8_stride   = h->mb_width * 2 + 1;

        // error resilience code looks cleaner with this
        if (!FF_ALLOCZ_TYPED_ARRAY(er->mb_index2xy,        h->mb_num + 1) ||
            !FF_ALLOCZ_TYPED_ARRAY(er->error_status_table, mb_array_size) ||
            !FF_ALLOCZ_TYPED_ARRAY(er->er_temp_buffer,     er_size)       ||
            !FF_ALLOCZ_TYPED_ARRAY(h->dc_val_base,         yc_size))
            return AVERROR(ENOMEM); // ff_h264_free_tables will clean up for us

        for (y = 0; y < h->mb_height; y++)
            for (x = 0; x < h->mb_width; x++)
                er->mb_index2xy[x + y * h->mb_width] = x + y * h->mb_stride;

        er->mb_index2xy[h->mb_height * h->mb_width] = (h->mb_height - 1) *
                                                      h->mb_stride + h->mb_width;
        er->dc_val[0] = h->dc_val_base + h->mb_width * 2 + 2;
        er->dc_val[1] = h->dc_val_base + y_size + h->mb_stride + 1;
        er->dc_val[2] = er->dc_val[1] + big_mb_num;
        for (int i = 0; i < yc_size; i++)
            h->dc_val_base[i] = 1024;
    }

    return 0;
}

/**
 * Init slice context
 */
void ff_h264_slice_context_init(H264Context *h, H264SliceContext *sl)
{
    sl->ref_cache[0][scan8[5]  + 1] =
    sl->ref_cache[0][scan8[7]  + 1] =
    sl->ref_cache[0][scan8[13] + 1] =
    sl->ref_cache[1][scan8[5]  + 1] =
    sl->ref_cache[1][scan8[7]  + 1] =
    sl->ref_cache[1][scan8[13] + 1] = PART_NOT_AVAILABLE;

    sl->er = &h->er;
}

static int h264_init_pic(H264Picture *pic)
{
    pic->f = av_frame_alloc();
    if (!pic->f)
        return AVERROR(ENOMEM);

    pic->f_grain = av_frame_alloc();
    if (!pic->f_grain)
        return AVERROR(ENOMEM);

    return 0;
}

static int h264_init_context(AVCodecContext *avctx, H264Context *h)
{
    int i, ret;

    h->avctx                 = avctx;
    h->cur_chroma_format_idc = -1;

    h->width_from_caller     = avctx->width;
    h->height_from_caller    = avctx->height;

    h->workaround_bugs       = avctx->workaround_bugs;
    h->flags                 = avctx->flags;
    h->poc.prev_poc_msb      = 1 << 16;
    h->poc.prev_frame_num    = -1;
    h->dep_view_poc.prev_poc_msb = 1 << 16;
    h->dep_view_poc.prev_frame_num = -1;
    h->recovery_frame        = -1;
    h->frame_recovered       = 0;
    h->sei.common.frame_packing.arrangement_cancel_flag = -1;
    h->sei.common.unregistered.x264_build = -1;

    h->next_outputed_poc = INT_MIN;
    h->next_outputed_poc_dep = INT_MIN;
    for (i = 0; i < FF_ARRAY_ELEMS(h->last_pocs); i++) {
        h->last_pocs[i] = INT_MIN;
        h->last_pocs_dep[i] = INT_MIN;
    }

    ff_h264_sei_uninit(&h->sei);

    if (avctx->active_thread_type & FF_THREAD_FRAME) {
        h->decode_error_flags_pool = av_refstruct_pool_alloc(sizeof(atomic_int), 0);
        if (!h->decode_error_flags_pool)
            return AVERROR(ENOMEM);
    }

    h->nb_slice_ctx = (avctx->active_thread_type & FF_THREAD_SLICE) ? avctx->thread_count : 1;
    h->slice_ctx = av_calloc(h->nb_slice_ctx, sizeof(*h->slice_ctx));
    if (!h->slice_ctx) {
        h->nb_slice_ctx = 0;
        return AVERROR(ENOMEM);
    }

    for (i = 0; i < H264_MAX_PICTURE_COUNT; i++) {
        if ((ret = h264_init_pic(&h->DPB[i])) < 0)
            return ret;
    }

    if ((ret = h264_init_pic(&h->cur_pic)) < 0)
        return ret;

    if ((ret = h264_init_pic(&h->last_pic_for_ec)) < 0)
        return ret;

    for (i = 0; i < h->nb_slice_ctx; i++)
        h->slice_ctx[i].h264 = h;

    return 0;
}

static void h264_free_pic(H264Context *h, H264Picture *pic)
{
    ff_h264_unref_picture(pic);
    av_frame_free(&pic->f);
    av_frame_free(&pic->f_grain);
}

static av_cold int h264_decode_end(AVCodecContext *avctx)
{
    H264Context *h = avctx->priv_data;
    int i;

    ff_h264_remove_all_refs(h);
    ff_h264_free_tables(h);

    for (i = 0; i < H264_MAX_PICTURE_COUNT; i++) {
        h264_free_pic(h, &h->DPB[i]);
    }
    memset(h->delayed_pic, 0, sizeof(h->delayed_pic));
    memset(h->delayed_pic_dep, 0, sizeof(h->delayed_pic_dep));

    h->cur_pic_ptr = NULL;

    av_refstruct_pool_uninit(&h->decode_error_flags_pool);
    av_container_fifo_free(&h->output_fifo);
    avpriv_packet_list_free(&h->mvc_pending_pkts);
    av_freep(&h->view_ids_available);
    av_freep(&h->view_pos_available);

    av_freep(&h->slice_ctx);
    h->nb_slice_ctx = 0;

    ff_h264_sei_uninit(&h->sei);
    ff_h264_ps_uninit(&h->ps);

    ff_h2645_packet_uninit(&h->pkt);

    h264_free_pic(h, &h->cur_pic);
    h264_free_pic(h, &h->last_pic_for_ec);

    return 0;
}

static AVOnce h264_vlc_init = AV_ONCE_INIT;

static av_cold int h264_decode_init(AVCodecContext *avctx)
{
    H264Context *h = avctx->priv_data;
    int ret;

    h->output_fifo = av_container_fifo_alloc_avframe(0);
    if (!h->output_fifo)
        return AVERROR(ENOMEM);

    ret = h264_init_context(avctx, h);
    if (ret < 0)
        return ret;

    ret = ff_thread_once(&h264_vlc_init, ff_h264_decode_init_vlc);
    if (ret != 0) {
        av_log(avctx, AV_LOG_ERROR, "pthread_once has failed.");
        return AVERROR_UNKNOWN;
    }

    if (!avctx->internal->is_copy) {
        if (avctx->extradata_size > 0 && avctx->extradata) {
            ret = ff_h264_decode_extradata(avctx->extradata, avctx->extradata_size,
                                           &h->ps, &h->is_avc, &h->nal_length_size,
                                           avctx->err_recognition, avctx);
           if (ret < 0) {
               int explode = avctx->err_recognition & AV_EF_EXPLODE;
               av_log(avctx, explode ? AV_LOG_ERROR: AV_LOG_WARNING,
                      "Error decoding the extradata\n");
               if (explode) {
                   return ret;
               }
               ret = 0;
           }
        }
    }

    if (h->ps.sps && h->ps.sps->bitstream_restriction_flag &&
        h->avctx->has_b_frames < h->ps.sps->num_reorder_frames) {
        h->avctx->has_b_frames = h->ps.sps->num_reorder_frames;
    }

    /* MVC multiview requires inter-view references within a single DPB,
     * which is incompatible with frame threading. Detect MVC from
     * extradata (profiles 118=Multiview High, 128=Stereo High) and
     * fall back to slice threading. */
    for (int i = 0; i < MAX_SPS_COUNT; i++) {
        if (h->ps.sps_list[i] &&
            (h->ps.sps_list[i]->profile_idc == 118 ||
             h->ps.sps_list[i]->profile_idc == 128)) {
            h->mvc_active = 1;
            if (avctx->active_thread_type & FF_THREAD_FRAME) {
                av_log(avctx, AV_LOG_INFO,
                       "MVC multiview detected, disabling frame threading\n");
                avctx->active_thread_type &= ~FF_THREAD_FRAME;
            }
            break;
        }
    }

    ff_h264_flush_change(h);

    if (h->enable_er < 0 && (avctx->active_thread_type & FF_THREAD_SLICE))
        h->enable_er = 0;

    if (h->enable_er && (avctx->active_thread_type & FF_THREAD_SLICE)) {
        av_log(avctx, AV_LOG_WARNING,
               "Error resilience with slice threads is enabled. It is unsafe and unsupported and may crash. "
               "Use it at your own risk\n");
    }

    return 0;
}

/**
 * instantaneous decoder refresh.
 */
static void idr(H264Context *h)
{
    int i;
    if (h->mvc_active) {
        H264POCContext *poc = h->cur_view_id ? &h->dep_view_poc : &h->poc;
        /* Mark stale dep view transition: base IDR after mid-stream
         * decoding (prev_frame_num > 0) means old-GOP dep slices may
         * still arrive and must be skipped.  At stream start (prev=-1
         * or 0), no stale deps exist. */
        if (!h->cur_view_id && poc->prev_frame_num > 0)
            h->mvc_base_idr_decoded = 1;
        ff_h264_remove_view_refs(h, h->cur_view_id);
        poc->prev_frame_num        =
        poc->prev_frame_num_offset = 0;
        poc->prev_poc_msb          = 1<<16;
        poc->prev_poc_lsb          = -1;
    } else {
        ff_h264_remove_all_refs(h);
        h->poc.prev_frame_num        =
        h->poc.prev_frame_num_offset = 0;
        h->poc.prev_poc_msb          = 1<<16;
        h->poc.prev_poc_lsb          = -1;
        h->dep_view_poc.prev_frame_num        =
        h->dep_view_poc.prev_frame_num_offset = 0;
        h->dep_view_poc.prev_poc_msb          = 1<<16;
        h->dep_view_poc.prev_poc_lsb          = -1;
    }
    if (h->mvc_active) {
        int *lp = h->cur_view_id ? h->last_pocs_dep : h->last_pocs;
        for (i = 0; i < H264_MAX_DPB_FRAMES; i++)
            lp[i] = INT_MIN;
    } else {
        for (i = 0; i < FF_ARRAY_ELEMS(h->last_pocs); i++) {
            h->last_pocs[i] = INT_MIN;
            h->last_pocs_dep[i] = INT_MIN;
        }
    }
}

/* forget old pics after a seek */
void ff_h264_flush_change(H264Context *h)
{
    int i, j;

    h->next_outputed_poc = INT_MIN;
    h->next_outputed_poc_dep = INT_MIN;
    h->prev_interlaced_frame = 1;
    /* Flush must remove ALL refs regardless of cur_view_id.
     * idr() in MVC mode only removes one view's refs, so
     * temporarily disable mvc_active for the flush. */
    { int save_mvc = h->mvc_active;
      h->mvc_active = 0;
      idr(h);
      h->mvc_active = save_mvc;
    }

    h->poc.prev_frame_num = -1;
    h->dep_view_poc.prev_frame_num = -1;
    if (h->cur_pic_ptr) {
        h->cur_pic_ptr->reference = 0;
        for (j=i=0; h->delayed_pic[i]; i++)
            if (h->delayed_pic[i] != h->cur_pic_ptr)
                h->delayed_pic[j++] = h->delayed_pic[i];
        h->delayed_pic[j] = NULL;
        for (j=i=0; h->delayed_pic_dep[i]; i++)
            if (h->delayed_pic_dep[i] != h->cur_pic_ptr)
                h->delayed_pic_dep[j++] = h->delayed_pic_dep[i];
        h->delayed_pic_dep[j] = NULL;
    }
    ff_h264_unref_picture(&h->last_pic_for_ec);

    h->first_field = 0;
    h->recovery_frame = -1;
    h->frame_recovered = 0;
    h->current_slice = 0;
    h->mmco_reset = 1;
    h->mvc_base_idr_decoded = 0;
    h->mvc_base_pic = NULL;
}

static av_cold void h264_decode_flush(AVCodecContext *avctx)
{
    H264Context *h = avctx->priv_data;
    int i;

    av_container_fifo_drain(h->output_fifo,
                            av_container_fifo_can_read(h->output_fifo));
    avpriv_packet_list_free(&h->mvc_pending_pkts);
    memset(h->delayed_pic, 0, sizeof(h->delayed_pic));
    memset(h->delayed_pic_dep, 0, sizeof(h->delayed_pic_dep));

    ff_h264_flush_change(h);
    ff_h264_sei_uninit(&h->sei);

    for (i = 0; i < H264_MAX_PICTURE_COUNT; i++)
        ff_h264_unref_picture(&h->DPB[i]);
    h->cur_pic_ptr = NULL;
    ff_h264_unref_picture(&h->cur_pic);

    h->mb_y = 0;
    h->non_gray = 0;

    ff_h264_free_tables(h);
    h->context_initialized = 0;

    if (FF_HW_HAS_CB(avctx, flush))
        FF_HW_SIMPLE_CALL(avctx, flush);
}

static int get_last_needed_nal(H264Context *h)
{
    int nals_needed = 0;
    int slice_type = 0;
    int picture_intra_only = 1;
    int first_slice = 0;
    int i, ret;

    for (i = 0; i < h->pkt.nb_nals; i++) {
        H2645NAL *nal = &h->pkt.nals[i];
        GetBitContext gb;

        /* packets can sometimes contain multiple PPS/SPS,
         * e.g. two PAFF field pictures in one packet, or a demuxer
         * which splits NALs strangely if so, when frame threading we
         * can't start the next thread until we've read all of them */
        switch (nal->type) {
        case H264_NAL_SPS:
        case H264_NAL_SUB_SPS:
        case H264_NAL_PPS:
            nals_needed = i;
            break;
        case H264_NAL_DPA:
        case H264_NAL_IDR_SLICE:
        case H264_NAL_SLICE:
        case H264_NAL_EXTEN_SLICE: {
            /*
             * NAL type 20 (EXTEN_SLICE) has a 3-byte MVC extension header
             * between the 1-byte NAL header and the slice header.
             * Skip it to reach first_mb_in_slice.
             */
            int hdr = nal->type == H264_NAL_EXTEN_SLICE ? 4 : 1;
            ret = init_get_bits8(&gb, nal->data + hdr, nal->size - hdr);
            if (ret < 0) {
                av_log(h->avctx, AV_LOG_ERROR, "Invalid zero-sized VCL NAL unit\n");
                if (h->avctx->err_recognition & AV_EF_EXPLODE)
                    return ret;

                break;
            }
            if (!get_ue_golomb_long(&gb) ||  // first_mb_in_slice
                !first_slice ||
                first_slice != nal->type)
                nals_needed = i;
            slice_type = get_ue_golomb_31(&gb);
            if (slice_type > 9)
                slice_type = 0;
            if (slice_type > 4)
                slice_type -= 5;

            slice_type = ff_h264_golomb_to_pict_type[slice_type];
            picture_intra_only &= (slice_type & 3) == AV_PICTURE_TYPE_I;
            if (!first_slice)
                first_slice = nal->type;
        }
        }
    }

    h->picture_intra_only = picture_intra_only;

    return nals_needed;
}

static void debug_green_metadata(const H264SEIGreenMetaData *gm, void *logctx)
{
    av_log(logctx, AV_LOG_DEBUG, "Green Metadata Info SEI message\n");
    av_log(logctx, AV_LOG_DEBUG, "  green_metadata_type: %d\n", gm->green_metadata_type);

    if (gm->green_metadata_type == 0) {
        av_log(logctx, AV_LOG_DEBUG, "  green_metadata_period_type: %d\n", gm->period_type);

        if (gm->period_type == 2)
            av_log(logctx, AV_LOG_DEBUG, "  green_metadata_num_seconds: %d\n", gm->num_seconds);
        else if (gm->period_type == 3)
            av_log(logctx, AV_LOG_DEBUG, "  green_metadata_num_pictures: %d\n", gm->num_pictures);

        av_log(logctx, AV_LOG_DEBUG, "  SEI GREEN Complexity Metrics: %f %f %f %f\n",
               (float)gm->percent_non_zero_macroblocks/255,
               (float)gm->percent_intra_coded_macroblocks/255,
               (float)gm->percent_six_tap_filtering/255,
               (float)gm->percent_alpha_point_deblocking_instance/255);

    } else if (gm->green_metadata_type == 1) {
        av_log(logctx, AV_LOG_DEBUG, "  xsd_metric_type: %d\n", gm->xsd_metric_type);

        if (gm->xsd_metric_type == 0)
            av_log(logctx, AV_LOG_DEBUG, "  xsd_metric_value: %f\n",
                   (float)gm->xsd_metric_value/100);
    }
}

/**
 * Check if view_id is in the user-requested view_ids list.
 * Returns 1 if the view should be decoded, 0 to skip.
 */
static int h264_view_requested(const H264Context *h, int view_id)
{
    if (h->nb_view_ids == 0)
        return view_id == 0;
    if (h->nb_view_ids == 1 && h->view_ids[0] == -1)
        return 1;
    for (unsigned i = 0; i < h->nb_view_ids; i++)
        if (h->view_ids[i] == view_id)
            return 1;
    return 0;
}

/**
 * Register a view_id in the view_ids_available export array.
 * Called when a new view_id is seen in an EXTEN_SLICE NAL.
 * No-op if the view_id is already registered.
 */
static int h264_register_view_id(H264Context *h, int view_id)
{
    unsigned *ids, *pos;
    unsigned n = h->nb_view_ids_available;

    for (unsigned i = 0; i < n; i++)
        if (h->view_ids_available[i] == (unsigned)view_id)
            return 0;

    ids = av_realloc_array(h->view_ids_available, n + 1, sizeof(*ids));
    if (!ids)
        return AVERROR(ENOMEM);

    pos = av_realloc_array(h->view_pos_available, n + 1, sizeof(*pos));
    if (!pos) {
        /* ids was already reallocated; assign it so it's not leaked.
         * The extra slot is unused but harmless (nb stays the same). */
        h->view_ids_available = ids;
        return AVERROR(ENOMEM);
    }
    h->view_ids_available = ids;
    h->view_pos_available = pos;

    h->view_ids_available[n] = view_id;
    /* MVC convention: view_id 0 = left (base), view_id > 0 = right */
    h->view_pos_available[n] =
        view_id == 0 ? AV_STEREO3D_VIEW_LEFT : AV_STEREO3D_VIEW_RIGHT;
    h->nb_view_ids_available = n + 1;
    h->nb_view_pos_available = n + 1;
    return 0;
}

static int decode_nal_units(H264Context *h, AVBufferRef *buf_ref,
                            const uint8_t *buf, int buf_size)
{
    AVCodecContext *const avctx = h->avctx;
    int nals_needed = 0; ///< number of NALs that need decoding before the next frame thread starts
    int idr_cleared=0;
    int i, ret = 0;

    h->has_slice = 0;
    h->nal_unit_type= 0;

    if (!(avctx->flags2 & AV_CODEC_FLAG2_CHUNKS)) {
        h->current_slice = 0;
        if (!h->first_field) {
            h->cur_pic_ptr = NULL;
            ff_h264_sei_uninit(&h->sei);
        }
    }

    if (h->nal_length_size == 4) {
        if (buf_size > 8 && AV_RB32(buf) == 1 && AV_RB32(buf+5) > (unsigned)buf_size) {
            h->is_avc = 0;
        }else if(buf_size > 3 && AV_RB32(buf) > 1 && AV_RB32(buf) <= (unsigned)buf_size)
            h->is_avc = 1;
    }

    ret = ff_h2645_packet_split(&h->pkt, buf, buf_size, avctx, h->nal_length_size,
                                avctx->codec_id, !!h->is_avc * H2645_FLAG_IS_NALFF);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR,
               "Error splitting the input into NAL units.\n");
        return ret;
    }

    if (avctx->active_thread_type & FF_THREAD_FRAME)
        nals_needed = get_last_needed_nal(h);
    if (nals_needed < 0)
        return nals_needed;

    for (i = 0; i < h->pkt.nb_nals; i++) {
        H2645NAL *nal = &h->pkt.nals[i];
        int max_slice_ctx, err;

        if (avctx->skip_frame >= AVDISCARD_NONREF &&
            nal->ref_idc == 0 && nal->type != H264_NAL_SEI)
            continue;

        // FIXME these should stop being context-global variables
        h->nal_ref_idc   = nal->ref_idc;
        h->nal_unit_type = nal->type;

        err = 0;
        switch (nal->type) {
        case H264_NAL_IDR_SLICE:
            h->cur_view_id = 0;
            if ((nal->data[1] & 0xFC) == 0x98) {
                av_log(h->avctx, AV_LOG_ERROR, "Invalid inter IDR frame\n");
                h->next_outputed_poc = INT_MIN;
                ret = -1;
                goto end;
            }
            if(!idr_cleared) {
                idr(h); // FIXME ensure we don't lose some frames if there is reordering
            }
            idr_cleared = 1;
            h->has_recovery_point = 1;
        case H264_NAL_SLICE:
            h->cur_view_id = 0;
            h->idr_pic_flag = (nal->type == H264_NAL_IDR_SLICE);
            h->has_slice = 1;

        slice_common:
            if ((err = ff_h264_queue_decode_slice(h, nal))) {
                H264SliceContext *sl = h->slice_ctx + h->nb_slice_ctx_queued;
                sl->ref_count[0] = sl->ref_count[1] = 0;
                break;
            }

            if (h->current_slice == 1) {
                if (avctx->active_thread_type & FF_THREAD_FRAME &&
                    i >= nals_needed && !h->setup_finished && h->cur_pic_ptr) {
                    ff_thread_finish_setup(avctx);
                    h->setup_finished = 1;
                }

                if (h->avctx->hwaccel &&
                    (ret = FF_HW_CALL(h->avctx, start_frame, buf_ref,
                                      buf, buf_size)) < 0)
                    goto end;
            }

            max_slice_ctx = avctx->hwaccel ? 1 : h->nb_slice_ctx;
            if (h->nb_slice_ctx_queued == max_slice_ctx) {
                if (h->avctx->hwaccel) {
                    ret = FF_HW_CALL(avctx, decode_slice, nal->raw_data, nal->raw_size);
                    h->nb_slice_ctx_queued = 0;
                } else
                    ret = ff_h264_execute_decode_slices(h);
                if (ret < 0 && (h->avctx->err_recognition & AV_EF_EXPLODE))
                    goto end;
            }
            break;
        case H264_NAL_DPA:
        case H264_NAL_DPB:
        case H264_NAL_DPC:
            avpriv_request_sample(avctx, "data partitioning");
            break;
        case H264_NAL_SEI:
            if (h->setup_finished) {
                avpriv_request_sample(avctx, "Late SEI");
                break;
            }
            ret = ff_h264_sei_decode(&h->sei, &nal->gb, &h->ps, avctx);
            h->has_recovery_point = h->has_recovery_point || h->sei.recovery_point.recovery_frame_cnt != -1;
            if (avctx->debug & FF_DEBUG_GREEN_MD)
                debug_green_metadata(&h->sei.green_metadata, h->avctx);
            if (ret < 0 && (h->avctx->err_recognition & AV_EF_EXPLODE))
                goto end;
            break;
        case H264_NAL_SPS: {
            GetBitContext tmp_gb = nal->gb;
            if (FF_HW_HAS_CB(avctx, decode_params)) {
                ret = FF_HW_CALL(avctx, decode_params,
                                 nal->type, nal->raw_data, nal->raw_size);
                if (ret < 0)
                    goto end;
            }
            if (ff_h264_decode_seq_parameter_set(&tmp_gb, avctx, &h->ps, 0) >= 0)
                break;
            av_log(h->avctx, AV_LOG_DEBUG,
                   "SPS decoding failure, trying again with the complete NAL\n");
            init_get_bits8(&tmp_gb, nal->raw_data + 1, nal->raw_size - 1);
            if (ff_h264_decode_seq_parameter_set(&tmp_gb, avctx, &h->ps, 0) >= 0)
                break;
            ff_h264_decode_seq_parameter_set(&nal->gb, avctx, &h->ps, 1);
            break;
        }
        case H264_NAL_PPS:
            if (FF_HW_HAS_CB(avctx, decode_params)) {
                ret = FF_HW_CALL(avctx, decode_params,
                                 nal->type, nal->raw_data, nal->raw_size);
                if (ret < 0)
                    goto end;
            }
            ret = ff_h264_decode_picture_parameter_set(&nal->gb, avctx, &h->ps,
                                                       nal->size_bits);
            if (ret < 0 && (h->avctx->err_recognition & AV_EF_EXPLODE))
                goto end;
            break;
        case H264_NAL_SUB_SPS: {
            /*
             * MVC Subset SPS (NAL type 15, H.264 Annex H, H.7.3.2.1.4).
             *
             * Contains a regular SPS followed by MVC extension fields
             * (num_views, view dependencies, applicable operation points).
             * We parse the regular SPS for resolution/profile info, then
             * read view_ids from the MVC extension to populate
             * view_ids_available early (needed for view specifiers in
             * fftools before the first EXTEN_SLICE arrives).
             *
             * The Subset SPS arrives on the dependent view's PID in
             * MPEG-TS. After stream merging, it appears on the same
             * AVStream as the base SPS.
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
                /* H.264 Annex H limits num_views_minus1.  CBS uses
                 * H264_MVC_MAX_VIEWS (2); allow a few more for
                 * future use but reject obviously corrupt values. */
                if (num_views > 0 && num_views <= 16 &&
                    get_bits_left(&tmp_gb) >= 0) {
                    if (!h->mvc_active)
                        h->mvc_active = 1;
                    for (unsigned i = 0; i < num_views && get_bits_left(&tmp_gb) > 0; i++) {
                        unsigned vid = get_ue_golomb_long(&tmp_gb);
                        if (vid <= 1023) {
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
            /* MVC prefix NAL (type 14): metadata for the following base
             * view slice. Not needed for decoding -- no-op. */
            break;
        case H264_NAL_EXTEN_SLICE: {
            /*
             * MVC extension slice (NAL type 20, H.264 Annex H).
             * 4-byte NAL header: 1 standard + 3 extension (svc_ext_flag,
             * non_idr_flag, priority_id, view_id, temporal_id, flags).
             * Parse the extension, extract view_id, then rewrite nal->type
             * to SLICE so standard slice parsing handles it unchanged.
             */
            int non_idr_flag;

            if (get_bits1(&nal->gb)) // svc_extension_flag == 1 means SVC
                break; // SVC (Scalable Video Coding) not supported
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

            /* view_ids filtering: skip views not requested by the user.
             * Default (empty): base view only. -1: all views. */
            if (!h264_view_requested(h, h->cur_view_id))
                break;

            /* MVC dep view needs base view in the same DPB. With frame
             * threading, each worker has a separate DPB so inter-view
             * prediction is impossible. Skip dep view slices when frame
             * threading is active. */
            if (avctx->active_thread_type & FF_THREAD_FRAME) {
                av_log_once(avctx, AV_LOG_WARNING, AV_LOG_DEBUG,
                            &h->mvc_frame_thread_warned,
                            "MVC: frame threading active, skipping dep view %d "
                            "(use -thread_type slice for both views)\n",
                            h->cur_view_id);
                break;
            }

            /* MVC: skip stale dep view slices from the previous GOP.
             * After a base IDR, dep PES packets from the old GOP may still
             * arrive (MPEG-TS PES ordering).  Non-anchor dep slices
             * (non_idr_flag=1) reference pictures that have been cleared
             * by the IDR and must be skipped.  The dep anchor
             * (non_idr_flag=0) starts the new dep-view GOP and clears
             * the flag. */
            if (h->mvc_base_idr_decoded && non_idr_flag)
                break;
            if (!non_idr_flag)
                h->mvc_base_idr_decoded = 0;

            /* Rewrite to SLICE (not IDR_SLICE): MVC dep view "IDR" uses
             * inter-view prediction (P-slice), which would be rejected
             * as "non-intra slice in IDR NAL". idr_pic_flag handles
             * the IDR semantics separately. */
            nal->type = H264_NAL_SLICE;
            h->nal_unit_type = nal->type;

            h->idr_pic_flag = !non_idr_flag;

            if (!non_idr_flag)
                h->has_recovery_point = 1;

            h->has_slice = 1;
            goto slice_common;
        }
        case H264_NAL_AUD:
        case H264_NAL_END_SEQUENCE:
        case H264_NAL_END_STREAM:
        case H264_NAL_FILLER_DATA:
        case H264_NAL_SPS_EXT:
        case H264_NAL_AUXILIARY_SLICE:
            break;
        default:
            av_log(avctx, AV_LOG_DEBUG, "Unknown NAL code: %d (%d bits)\n",
                   nal->type, nal->size_bits);
        }

        if (err < 0 && (h->avctx->err_recognition & AV_EF_EXPLODE)) {
            av_log(h->avctx, AV_LOG_ERROR, "decode_slice_header error\n");
            ret = err;
            goto end;
        }
    }

    ret = ff_h264_execute_decode_slices(h);
    if (ret < 0 && (h->avctx->err_recognition & AV_EF_EXPLODE))
        goto end;

    // set decode_error_flags to allow users to detect concealed decoding errors
    if ((ret < 0 || h->er.error_occurred) && h->cur_pic_ptr) {
        if (h->cur_pic_ptr->decode_error_flags) {
            /* Frame-threading in use */
            atomic_int *decode_error = h->cur_pic_ptr->decode_error_flags;
            /* Using atomics here is not supposed to provide synchronisation;
             * they are merely used to allow to set decode_error from both
             * decoding threads in case of coded slices. */
            atomic_fetch_or_explicit(decode_error, FF_DECODE_ERROR_DECODE_SLICES,
                                     memory_order_relaxed);
        } else
            h->cur_pic_ptr->f->decode_error_flags |= FF_DECODE_ERROR_DECODE_SLICES;
    }

    ret = 0;
end:

#if CONFIG_ERROR_RESILIENCE
    /*
     * FIXME: Error handling code does not seem to support interlaced
     * when slices span multiple rows
     * The ff_er_add_slice calls don't work right for bottom
     * fields; they cause massive erroneous error concealing
     * Error marking covers both fields (top and bottom).
     * This causes a mismatched s->error_count
     * and a bad error table. Further, the error count goes to
     * INT_MAX when called for bottom field, because mb_y is
     * past end by one (callers fault) and resync_mb_y != 0
     * causes problems for the first MB line, too.
     */
    if (!FIELD_PICTURE(h) && h->current_slice && h->enable_er) {

        H264SliceContext *sl = h->slice_ctx;
        int use_last_pic = h->last_pic_for_ec.f->buf[0] && !sl->ref_count[0];
        int decode_error_flags = 0;

        ff_h264_set_erpic(&h->er.cur_pic, h->cur_pic_ptr);

        if (use_last_pic) {
            ff_h264_set_erpic(&h->er.last_pic, &h->last_pic_for_ec);
            sl->ref_list[0][0].parent = &h->last_pic_for_ec;
            memcpy(sl->ref_list[0][0].data, h->last_pic_for_ec.f->data, sizeof(sl->ref_list[0][0].data));
            memcpy(sl->ref_list[0][0].linesize, h->last_pic_for_ec.f->linesize, sizeof(sl->ref_list[0][0].linesize));
            sl->ref_list[0][0].reference = h->last_pic_for_ec.reference;
        } else if (sl->ref_count[0]) {
            ff_h264_set_erpic(&h->er.last_pic, sl->ref_list[0][0].parent);
        } else
            ff_h264_set_erpic(&h->er.last_pic, NULL);

        if (sl->ref_count[1])
            ff_h264_set_erpic(&h->er.next_pic, sl->ref_list[1][0].parent);

        ff_er_frame_end(&h->er, &decode_error_flags);
        if (decode_error_flags) {
            if (h->cur_pic_ptr->decode_error_flags) {
                atomic_int *decode_error = h->cur_pic_ptr->decode_error_flags;
                atomic_fetch_or_explicit(decode_error, decode_error_flags,
                                         memory_order_relaxed);
            } else
                h->cur_pic_ptr->f->decode_error_flags |= decode_error_flags;
        }
        if (use_last_pic)
            memset(&sl->ref_list[0][0], 0, sizeof(sl->ref_list[0][0]));
    }
#endif /* CONFIG_ERROR_RESILIENCE */
    /* clean up */
    if (h->cur_pic_ptr && !h->droppable && h->has_slice) {
        ff_thread_report_progress(&h->cur_pic_ptr->tf, INT_MAX,
                                  h->picture_structure == PICT_BOTTOM_FIELD);
    }

    return (ret < 0) ? ret : buf_size;
}

static int h264_export_enc_params(AVFrame *f, const H264Picture *p)
{
    AVVideoEncParams *par;
    unsigned int nb_mb = p->mb_height * p->mb_width;
    unsigned int x, y;

    par = av_video_enc_params_create_side_data(f, AV_VIDEO_ENC_PARAMS_H264, nb_mb);
    if (!par)
        return AVERROR(ENOMEM);

    par->qp = p->pps->init_qp;

    par->delta_qp[1][0] = p->pps->chroma_qp_index_offset[0];
    par->delta_qp[1][1] = p->pps->chroma_qp_index_offset[0];
    par->delta_qp[2][0] = p->pps->chroma_qp_index_offset[1];
    par->delta_qp[2][1] = p->pps->chroma_qp_index_offset[1];

    for (y = 0; y < p->mb_height; y++)
        for (x = 0; x < p->mb_width; x++) {
            const unsigned int block_idx = y * p->mb_width + x;
            const unsigned int     mb_xy = y * p->mb_stride + x;
            AVVideoBlockParams *b = av_video_enc_params_block(par, block_idx);

            b->src_x = x * 16;
            b->src_y = y * 16;
            b->w     = 16;
            b->h     = 16;

            b->delta_qp = p->qscale_table[mb_xy] - par->qp;
        }

    return 0;
}

static int output_frame(H264Context *h, AVFrame *dst, H264Picture *srcp)
{
    int ret;

    ret = av_frame_ref(dst, srcp->needs_fg ? srcp->f_grain : srcp->f);
    if (ret < 0)
        return ret;

    if (srcp->needs_fg && (ret = av_frame_copy_props(dst, srcp->f)) < 0)
        return ret;

    if (srcp->decode_error_flags) {
        atomic_int *decode_error = srcp->decode_error_flags;
        /* The following is not supposed to provide synchronisation at all:
         * given that srcp has already finished decoding, decode_error
         * has already been set to its final value. */
        dst->decode_error_flags |= atomic_load_explicit(decode_error, memory_order_relaxed);
    }

    av_dict_set(&dst->metadata, "stereo_mode", ff_h264_sei_stereo_mode(&h->sei.common.frame_packing), 0);

    if (srcp->sei_recovery_frame_cnt == 0)
        dst->flags |= AV_FRAME_FLAG_KEY;

    if (h->avctx->export_side_data & AV_CODEC_EXPORT_DATA_VIDEO_ENC_PARAMS) {
        ret = h264_export_enc_params(dst, srcp);
        if (ret < 0)
            goto fail;
    }

    if (!(h->avctx->export_side_data & AV_CODEC_EXPORT_DATA_FILM_GRAIN))
        av_frame_remove_side_data(dst, AV_FRAME_DATA_FILM_GRAIN_PARAMS);

    /* MVC: attach view_id and stereo3d side data */
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
            /*
             * MVC convention: view_id 0 = left (base view),
             * view_id > 0 = right (dependent view).
             */
            stereo->view = srcp->view_id == 0 ? AV_STEREO3D_VIEW_LEFT
                                               : AV_STEREO3D_VIEW_RIGHT;
        }
    }

    return 0;
fail:
    av_frame_unref(dst);
    return ret;
}

static int is_avcc_extradata(const uint8_t *buf, int buf_size)
{
    int cnt= buf[5]&0x1f;
    const uint8_t *p= buf+6;
    if (!cnt)
        return 0;
    while(cnt--){
        int nalsize= AV_RB16(p) + 2;
        if(nalsize > buf_size - (p-buf) || (p[2] & 0x9F) != 7)
            return 0;
        p += nalsize;
    }
    cnt = *(p++);
    if(!cnt)
        return 0;
    while(cnt--){
        int nalsize= AV_RB16(p) + 2;
        if(nalsize > buf_size - (p-buf) || (p[2] & 0x9F) != 8)
            return 0;
        p += nalsize;
    }
    return 1;
}

/**
 * Finalize a decoded picture and push it into the output FIFO.
 *
 * Checks recovery/corruption flags, duplicates missing fields,
 * applies output_frame() post-processing (film grain, side data),
 * and writes the result into h->output_fifo.
 *
 * For non-MVC streams the FIFO will contain at most one frame per
 * decode call. For MVC, both the base and dependent view pictures
 * are pushed, so the caller can pop them one at a time.
 *
 * @return 0 on success or if the frame was skipped, negative on error
 */
static int finalize_frame(H264Context *h, H264Picture *out)
{
    AVFrame *dst;
    int ret;

    /*
     * MVC output filtering: skip frames whose view_id is not in the
     * user-requested view_ids list. The base view (view_id=0) is
     * always decoded for inter-view reference, but may not be output.
     */
    if (h->mvc_active && !h264_view_requested(h, out->view_id))
        return 0;

    if (!((h->avctx->flags & AV_CODEC_FLAG_OUTPUT_CORRUPT) ||
          (h->avctx->flags2 & AV_CODEC_FLAG2_SHOW_ALL) ||
          out->recovered))
        return 0;

    if (h->skip_gray > 0 &&
        h->non_gray && out->gray &&
        !(h->avctx->flags2 & AV_CODEC_FLAG2_SHOW_ALL))
        return 0;

    if (!h->avctx->hwaccel &&
        (out->field_poc[0] == INT_MAX ||
         out->field_poc[1] == INT_MAX)) {
        int p;
        AVFrame *f = out->f;
        int field = out->field_poc[0] == INT_MAX;
        uint8_t *dst_data[4];
        int linesizes[4];
        const uint8_t *src_data[4];

        av_log(h->avctx, AV_LOG_DEBUG, "Duplicating field %d to fill missing\n", field);

        for (p = 0; p < 4; p++) {
            dst_data[p] = f->data[p] + (field ^ 1) * f->linesize[p];
            src_data[p] = f->data[p] +  field      * f->linesize[p];
            linesizes[p] = 2 * f->linesize[p];
        }

        av_image_copy(dst_data, linesizes, src_data, linesizes,
                      f->format, f->width, f->height >> 1);
    }

    dst = av_frame_alloc();
    if (!dst)
        return AVERROR(ENOMEM);

    ret = output_frame(h, dst, out);
    if (ret < 0)
        goto fail;

    if (CONFIG_MPEGVIDEODEC) {
        ff_print_debug_info2(h->avctx, dst,
                             out->mb_type,
                             out->qscale_table,
                             out->motion_val,
                             out->mb_width, out->mb_height, out->mb_stride, 1);
    }

    ret = av_container_fifo_write(h->output_fifo, dst, 0);

fail:
    av_frame_free(&dst);
    return ret;
}

/**
 * Drain one delayed picture queue (POC order, lowest first).
 */
static int h264_flush_delayed_queue(H264Context *h, H264Picture **delayed)
{
    int ret, i, out_idx;
    H264Picture *out;

    while (delayed[0]) {
        out = delayed[0];
        out_idx = 0;
        for (i = 1;
             delayed[i] &&
             !(delayed[i]->f->flags & AV_FRAME_FLAG_KEY) &&
             !delayed[i]->mmco_reset;
             i++)
            if (delayed[i]->poc < out->poc) {
                out     = delayed[i];
                out_idx = i;
            }

        for (i = out_idx; delayed[i]; i++)
            delayed[i] = delayed[i + 1];

        if (out) {
            h->frame_recovered |= out->recovered;
            out->recovered |= h->frame_recovered & FRAME_RECOVERED_SEI;

            out->reference &= ~DELAYED_PIC_REF;
            ret = finalize_frame(h, out);
            if (ret < 0)
                return ret;
        }
    }

    return 0;
}

/**
 * Drain all remaining delayed pictures into the output FIFO.
 *
 * Called on EOF (zero-size packet) or NAL_END_SEQUENCE to flush the
 * reorder buffer. Pictures are output in POC order, lowest first.
 * For MVC, base and dep view queues are flushed separately.
 */
static int h264_flush_delayed_to_fifo(H264Context *h)
{
    int ret;

    h->cur_pic_ptr = NULL;
    h->first_field = 0;

    ret = h264_flush_delayed_queue(h, h->delayed_pic);
    if (ret < 0)
        return ret;

    return h264_flush_delayed_queue(h, h->delayed_pic_dep);
}

/**
 * Decode one packet and push output frames into the FIFO.
 *
 * This is the core decode function, adapted from the old h264_decode_frame().
 * Instead of returning a single frame directly, it pushes decoded frames
 * into h->output_fifo. The caller (h264_receive_frame) pops them one at
 * a time.
 *
 * @param avpkt the packet to decode (must be non-empty)
 * @return 0 on success, negative on error
 */
static int h264_decode_packet(H264Context *h, AVPacket *avpkt)
{
    AVCodecContext *avctx = h->avctx;
    const uint8_t *buf = avpkt->data;
    int buf_size       = avpkt->size;
    int buf_index;
    int ret;

    h->flags = avctx->flags;
    h->setup_finished = 0;
    h->nb_slice_ctx_queued = 0;

    ff_h264_unref_picture(&h->last_pic_for_ec);

    if (av_packet_get_side_data(avpkt, AV_PKT_DATA_NEW_EXTRADATA, NULL)) {
        size_t side_size;
        uint8_t *side = av_packet_get_side_data(avpkt, AV_PKT_DATA_NEW_EXTRADATA, &side_size);
        ff_h264_decode_extradata(side, side_size,
                                 &h->ps, &h->is_avc, &h->nal_length_size,
                                 avctx->err_recognition, avctx);
    }
    if (h->is_avc && buf_size >= 9 && buf[0]==1 && buf[2]==0 && (buf[4]&0xFC)==0xFC) {
        if (is_avcc_extradata(buf, buf_size))
            return ff_h264_decode_extradata(buf, buf_size,
                                            &h->ps, &h->is_avc, &h->nal_length_size,
                                            avctx->err_recognition, avctx);
    }

    buf_index = decode_nal_units(h, avpkt->buf, buf, buf_size);
    if (buf_index < 0)
        return AVERROR_INVALIDDATA;

    if (!h->cur_pic_ptr && h->nal_unit_type == H264_NAL_END_SEQUENCE) {
        av_assert0(buf_index <= buf_size);
        return h264_flush_delayed_to_fifo(h);
    }

    if (!(avctx->flags2 & AV_CODEC_FLAG2_CHUNKS) && (!h->cur_pic_ptr || !h->has_slice)) {
        if (avctx->skip_frame >= AVDISCARD_NONREF ||
            buf_size >= 4 && !memcmp("Q264", buf, 4) ||
            h->mvc_active)
            /*
             * mvc_active: when MVC dep view packets are merged into the
             * base view stream, some packets contain only EXTEN_SLICE
             * NALs which are skipped by view_ids filtering. These are
             * not errors -- just empty from the base view's perspective.
             */
            return 0;
        av_log(avctx, AV_LOG_ERROR, "no frame!\n");
        return AVERROR_INVALIDDATA;
    }

    if (!(avctx->flags2 & AV_CODEC_FLAG2_CHUNKS) ||
        (h->mb_y >= h->mb_height && h->mb_height)) {
        if ((ret = ff_h264_field_end(h, &h->slice_ctx[0], 0)) < 0)
            return ret;

        if (h->next_output_pic) {
            ret = finalize_frame(h, h->next_output_pic);
            if (ret < 0)
                return ret;
        }
    }

    ff_h264_unref_picture(&h->last_pic_for_ec);

    return 0;
}

/**
 * Classify whether an Annex B packet contains only MVC dependent view NALs.
 *
 * Scans for start codes (00 00 01 or 00 00 00 01) and checks NAL types.
 * Returns 1 if the packet has EXTEN_SLICE (type 20) NALs but no regular
 * SLICE/IDR_SLICE (type 1/5) NALs. Such packets are from the dependent
 * view and can be deferred to fix MPEG-TS PES ordering issues.
 */
static int h264_is_dep_view_packet(const uint8_t *buf, int buf_size)
{
    int has_dep = 0;

    for (int i = 0; i + 3 < buf_size; ) {
        int nal_start;

        if (buf[i] || buf[i + 1]) {
            i++;
            continue;
        }

        if (buf[i + 2] == 1)
            nal_start = i + 3;
        else if (buf[i + 2] == 0 && i + 3 < buf_size && buf[i + 3] == 1)
            nal_start = i + 4;
        else {
            i++;
            continue;
        }

        if (nal_start >= buf_size)
            break;

        switch (buf[nal_start] & 0x1F) {
        case H264_NAL_SLICE:
        case H264_NAL_IDR_SLICE:
        case H264_NAL_DPA:
            return 0;  /* has base view slice -- not dep-view-only */
        case H264_NAL_EXTEN_SLICE:
            has_dep = 1;
            break;
        }
        i = nal_start + 1;
    }
    return has_dep;
}

/**
 * Drain all accumulated dep-view packets from the pending list.
 *
 * In MPEG-TS with merged MVC PIDs, dep view PES packets often arrive
 * BEFORE the base view PES for the same access unit (different PIDs,
 * non-deterministic mux ordering).  h264_receive_frame() accumulates
 * these in mvc_pending_pkts.  After the base view packet is decoded
 * (so the inter-view reference picture is in the DPB), this function
 * drains the accumulated packets.
 */
static int h264_drain_mvc_pending(H264Context *h)
{
    AVPacket *pkt;
    int ret = 0;

    if (!h->mvc_pending_pkts.head)
        return 0;

    pkt = av_packet_alloc();
    if (!pkt)
        return AVERROR(ENOMEM);

    while (avpriv_packet_list_get(&h->mvc_pending_pkts, pkt) >= 0) {
        ret = h264_decode_packet(h, pkt);
        av_packet_unref(pkt);
        if (ret < 0) {
            avpriv_packet_list_free(&h->mvc_pending_pkts);
            break;
        }
    }
    av_packet_free(&pkt);
    return ret;
}

/**
 * receive_frame callback for the H.264 decoder.
 *
 * Uses an output FIFO to support multi-frame output from a single decode
 * call, which is needed for MVC multiview (each access unit produces one
 * frame per view). For non-MVC streams, the FIFO contains at most one
 * frame per decode call, so behavior is equivalent to the old API.
 *
 * MVC packet reordering:
 *   In MPEG-TS with merged PIDs, dependent view PES packets often arrive
 *   before the base view PES for the same access unit. The parser may also
 *   split a multi-slice dep view into multiple packets. All of these
 *   dep-view-only packets are accumulated in mvc_pending_pkts. When a
 *   base view packet arrives, it is decoded first, then all accumulated
 *   dep view packets are drained. This ensures the base view picture is
 *   in short_ref[] before any dep view slice needs it for inter-view
 *   prediction.
 *
 * Flow:
 *   1. If the FIFO has frames from a previous decode, return one.
 *   2. Get packets, accumulating dep-view-only packets in the pending list.
 *   3. When a base view packet arrives, decode it, then drain all pending.
 *   4. On EOF, drain pending and flush delayed pictures.
 *   5. Return one frame from the FIFO, or EAGAIN/EOF.
 */
static int h264_receive_frame(AVCodecContext *avctx, AVFrame *frame)
{
    H264Context *h = avctx->priv_data;
    AVCodecInternal *avci = avctx->internal;
    AVPacket *avpkt = avci->in_pkt;
    int ret;

    if (av_container_fifo_can_read(h->output_fifo))
        goto do_output;

get_packet:
    av_packet_unref(avpkt);
    ret = ff_decode_get_packet(avctx, avpkt);
    if (ret == AVERROR_EOF) {
        ret = h264_drain_mvc_pending(h);
        if (ret < 0)
            return ret;
        ret = h264_flush_delayed_to_fifo(h);
        if (ret < 0)
            return ret;
        goto do_output;
    } else if (ret < 0)
        return ret;

    /* MVC packet reordering: accumulate dep-view-only packets and decode
     * them after the base view packet.
     * !is_avc: AVC (MP4) format lacks start codes, so we can't scan raw
     * packet data to classify dep-view-only packets.
     * !is_frame_mt: frame threading gives each worker one packet, so the
     * accumulate-and-drain pattern doesn't work (dep view slices are
     * skipped by the EXTEN_SLICE handler instead).
     *
     * Note: do NOT gate on mvc_active here.  The first dep view PES in
     * MPEG-TS arrives before any EXTEN_SLICE has been decoded (which is
     * what sets mvc_active).  Detecting it early prevents "Missing
     * reference picture" errors and the loss of the first dep frame.
     * h264_is_dep_view_packet() returns 0 immediately for non-MVC
     * packets (finds SLICE at the first NAL), so the cost is negligible. */
    if (!h->is_avc && !avctx->internal->is_frame_mt &&
        h264_is_dep_view_packet(avpkt->data, avpkt->size)) {
        if (!h->mvc_active)
            h->mvc_active = 1;
        ret = avpriv_packet_list_put(&h->mvc_pending_pkts, avpkt, NULL, 0);
        if (ret < 0)
            return ret;
        goto get_packet;
    }

    /* MVC: drain accumulated dep-view packets BEFORE the next base view
     * packet. This ensures dep B-frames from the current GOP are decoded
     * while their base view references are still in the DPB.
     *
     * Save/restore field-tracking state: the base view may be mid-field
     * (first_field=1 with cur_pic_ptr pointing to the first field's
     * picture).  The dep drain calls h264_decode_packet which modifies
     * first_field and cur_pic_ptr as it processes dep view fields.
     * Without save/restore, the base view's second field would see
     * first_field=0 and be treated as a new frame instead of the
     * complementary field, losing one output frame.
     *
     * MVC_IV_REF: temporarily protect the base view picture from being
     * freed by dep view MMCO or sliding window operations.  Without this,
     * dep view decoding could remove the base picture that dep view slices
     * need for inter-view prediction.  Cleared after draining completes. */
    if (!avctx->internal->is_frame_mt && h->mvc_pending_pkts.head) {
        int save_first_field = h->first_field;
        H264Picture *save_cur_pic = h->cur_pic_ptr;
        int save_picture_structure = h->picture_structure;
        int save_droppable = h->droppable;

        if (h->cur_pic_ptr && !h->cur_pic_ptr->view_id) {
            h->mvc_base_pic = h->cur_pic_ptr;
            h->mvc_base_pic->reference |= MVC_IV_REF;
        }

        ret = h264_drain_mvc_pending(h);

        if (h->mvc_base_pic) {
            h->mvc_base_pic->reference &= ~MVC_IV_REF;
            h->mvc_base_pic = NULL;
        }

        h->first_field = save_first_field;
        h->cur_pic_ptr = save_cur_pic;
        h->picture_structure = save_picture_structure;
        h->droppable = save_droppable;

        if (ret < 0)
            return ret;
    }

    ret = h264_decode_packet(h, avpkt);
    if (ret < 0)
        return ret;

do_output:
    if (av_container_fifo_read(h->output_fifo, frame, 0) >= 0)
        return 0;

    return avci->draining ? AVERROR_EOF : AVERROR(EAGAIN);
}

#define OFFSET(x) offsetof(H264Context, x)
#define VD AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_DECODING_PARAM
#define VDX VD | AV_OPT_FLAG_EXPORT
static const AVOption h264_options[] = {
    { "is_avc", "is avc", OFFSET(is_avc), AV_OPT_TYPE_BOOL, {.i64 = 0}, 0, 1, VDX },
    { "nal_length_size", "nal_length_size", OFFSET(nal_length_size), AV_OPT_TYPE_INT, {.i64 = 0}, 0, 4, VDX },
    { "enable_er", "Enable error resilience on damaged frames (unsafe)", OFFSET(enable_er), AV_OPT_TYPE_BOOL, { .i64 = -1 }, -1, 1, VD },
    { "x264_build", "Assume this x264 version if no x264 version found in any SEI", OFFSET(x264_build), AV_OPT_TYPE_INT, {.i64 = -1}, -1, INT_MAX, VD },
    { "skip_gray", "Do not return gray gap frames", OFFSET(skip_gray), AV_OPT_TYPE_BOOL, {.i64 = 0}, 0, 1, VD },
    { "noref_gray", "Avoid using gray gap frames as references", OFFSET(noref_gray), AV_OPT_TYPE_BOOL, {.i64 = 1}, 0, 1, VD },
    { "view_ids",
        "Array of view IDs that should be decoded and output; "
        "a single -1 to decode all views (MVC multiview)",
        .offset = OFFSET(view_ids), .type = AV_OPT_TYPE_INT | AV_OPT_TYPE_FLAG_ARRAY,
        .min = -1, .max = 1023, .flags = VD },
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
    { NULL },
};

static const AVClass h264_class = {
    .class_name = "H264 Decoder",
    .item_name  = av_default_item_name,
    .option     = h264_options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFCodec ff_h264_decoder = {
    .p.name                = "h264",
    CODEC_LONG_NAME("H.264 / AVC / MPEG-4 AVC / MPEG-4 part 10"),
    .p.type                = AVMEDIA_TYPE_VIDEO,
    .p.id                  = AV_CODEC_ID_H264,
    .priv_data_size        = sizeof(H264Context),
    .init                  = h264_decode_init,
    .close                 = h264_decode_end,
    FF_CODEC_RECEIVE_FRAME_CB(h264_receive_frame),
    .p.capabilities        = AV_CODEC_CAP_DR1 |
                             AV_CODEC_CAP_DELAY | AV_CODEC_CAP_SLICE_THREADS |
                             AV_CODEC_CAP_FRAME_THREADS,
    .hw_configs            = (const AVCodecHWConfigInternal *const []) {
#if CONFIG_H264_DXVA2_HWACCEL
                               HWACCEL_DXVA2(h264),
#endif
#if CONFIG_H264_D3D11VA_HWACCEL
                               HWACCEL_D3D11VA(h264),
#endif
#if CONFIG_H264_D3D11VA2_HWACCEL
                               HWACCEL_D3D11VA2(h264),
#endif
#if CONFIG_H264_D3D12VA_HWACCEL
                               HWACCEL_D3D12VA(h264),
#endif
#if CONFIG_H264_NVDEC_HWACCEL
                               HWACCEL_NVDEC(h264),
#endif
#if CONFIG_H264_VAAPI_HWACCEL
                               HWACCEL_VAAPI(h264),
#endif
#if CONFIG_H264_VDPAU_HWACCEL
                               HWACCEL_VDPAU(h264),
#endif
#if CONFIG_H264_VIDEOTOOLBOX_HWACCEL
                               HWACCEL_VIDEOTOOLBOX(h264),
#endif
#if CONFIG_H264_VULKAN_HWACCEL
                               HWACCEL_VULKAN(h264),
#endif
                               NULL
                           },
    .caps_internal         = FF_CODEC_CAP_EXPORTS_CROPPING |
                             FF_CODEC_CAP_INIT_CLEANUP,
    .flush                 = h264_decode_flush,
    UPDATE_THREAD_CONTEXT(ff_h264_update_thread_context),
    UPDATE_THREAD_CONTEXT_FOR_USER(ff_h264_update_thread_context_for_user),
    .p.profiles            = NULL_IF_CONFIG_SMALL(ff_h264_profiles),
    .p.priv_class          = &h264_class,
};
