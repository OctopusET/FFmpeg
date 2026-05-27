/*
 * Copyright (c) 2026 Sungjoon Moon
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

/*
 * Test H.264 MVC CBS parsing: subset SPS (NAL type 15) and PPS
 * read-write round-trip.
 *
 * This verifies that the coded bitstream infrastructure can parse
 * and reconstruct MVC NAL units without data loss.
 */

#include <stdio.h>
#include <string.h>

#include "libavcodec/cbs.h"
#include "libavcodec/cbs_h264.h"
#include "libavcodec/codec_id.h"
#include "libavutil/buffer.h"
#include "libavutil/log.h"

/* Annex B subset SPS (NAL type 15, Stereo High profile, 2 views)
 * followed by a PPS (NAL type 8) and an MVC prefix NAL (type 14,
 * whose RBSP is empty per H.7.3.2.12). */
static const uint8_t mvc_nals[] = {
    0x00, 0x00, 0x00, 0x01, 0x2f, 0x80, 0x04, 0x28,
    0xac, 0xca, 0x82, 0x04, 0x45, 0x57, 0xca, 0x23,
    0x44, 0x00, 0x00, 0x00, 0x01, 0x28, 0xce, 0x38,
    0x80, 0x00, 0x00, 0x01, 0x6e, 0x00, 0x00, 0x05,
};

int main(void)
{
    CodedBitstreamContext *ctx = NULL;
    CodedBitstreamFragment frag = { 0 };
    CodedBitstreamFragment frag2 = { 0 };
    AVBufferRef *buf = NULL, *buf2 = NULL;
    int err, i, ret = 0;

    av_log_set_level(AV_LOG_ERROR);

    buf = av_buffer_alloc(sizeof(mvc_nals) + AV_INPUT_BUFFER_PADDING_SIZE);
    if (!buf)
        return 1;
    memcpy(buf->data, mvc_nals, sizeof(mvc_nals));
    memset(buf->data + sizeof(mvc_nals), 0, AV_INPUT_BUFFER_PADDING_SIZE);

    err = ff_cbs_init(&ctx, AV_CODEC_ID_H264, NULL);
    if (err < 0) {
        fprintf(stderr, "ff_cbs_init failed: %d\n", err);
        ret = 1;
        goto end;
    }

    /* Parse */
    err = ff_cbs_read(ctx, &frag, buf, buf->data, sizeof(mvc_nals));
    if (err < 0) {
        fprintf(stderr, "ff_cbs_read failed: %d\n", err);
        ret = 1;
        goto end;
    }

    if (frag.nb_units < 1) {
        fprintf(stderr, "No NAL units parsed\n");
        ret = 1;
        goto end;
    }

    /* Check that we got the expected NAL types */
    {
        int has_subset_sps = 0, has_pps = 0;
        for (i = 0; i < frag.nb_units; i++) {
            if (frag.units[i].type == H264_NAL_SUB_SPS)
                has_subset_sps = 1;
            if (frag.units[i].type == H264_NAL_PPS)
                has_pps = 1;
        }
        if (!has_subset_sps) {
            fprintf(stderr, "No subset SPS found\n");
            ret = 1;
            goto end;
        }
        if (!has_pps) {
            fprintf(stderr, "No PPS found\n");
            ret = 1;
            goto end;
        }
    }

    /* Verify subset SPS content */
    for (i = 0; i < frag.nb_units; i++) {
        if (frag.units[i].type == H264_NAL_SUB_SPS && frag.units[i].content) {
            H264RawSubsetSPS *ssps = frag.units[i].content;
            if (ssps->sps.profile_idc != 128 && ssps->sps.profile_idc != 118) {
                fprintf(stderr, "Unexpected profile_idc: %d\n",
                        ssps->sps.profile_idc);
                ret = 1;
                goto end;
            }
            if (ssps->mvc.num_views_minus1 + 1 != 2) {
                fprintf(stderr, "Expected 2 views, got %d\n",
                        ssps->mvc.num_views_minus1 + 1);
                ret = 1;
                goto end;
            }
        }
    }

    /* Write round-trip */
    err = ff_cbs_write_fragment_data(ctx, &frag);
    if (err < 0) {
        fprintf(stderr, "ff_cbs_write_fragment_data failed: %d\n", err);
        ret = 1;
        goto end;
    }

    /* Re-read */
    buf2 = av_buffer_alloc(frag.data_size + AV_INPUT_BUFFER_PADDING_SIZE);
    if (!buf2) {
        ret = 1;
        goto end;
    }
    memcpy(buf2->data, frag.data, frag.data_size);
    memset(buf2->data + frag.data_size, 0, AV_INPUT_BUFFER_PADDING_SIZE);

    err = ff_cbs_read(ctx, &frag2, buf2, buf2->data, frag.data_size);
    if (err < 0) {
        fprintf(stderr, "Round-trip re-read failed: %d\n", err);
        ret = 1;
        goto end;
    }

    if (frag2.nb_units != frag.nb_units) {
        fprintf(stderr, "Round-trip NAL count mismatch: %d vs %d\n",
                frag.nb_units, frag2.nb_units);
        ret = 1;
        goto end;
    }

    /* The rewrite of this input must be bit-exact. */
    if (frag.data_size != sizeof(mvc_nals) ||
        memcmp(frag.data, mvc_nals, sizeof(mvc_nals))) {
        fprintf(stderr, "Round-trip output is not bit-exact\n");
        ret = 1;
        goto end;
    }

end:
    ff_cbs_fragment_free(&frag2);
    ff_cbs_fragment_free(&frag);
    ff_cbs_close(&ctx);
    av_buffer_unref(&buf2);
    av_buffer_unref(&buf);
    return ret;
}
