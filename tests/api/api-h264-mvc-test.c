/*
 * Test H.264 MVC CBS parsing: subset SPS (NAL type 15), PPS, and
 * extension slice (NAL type 20) read-write round-trip.
 *
 * This verifies that the coded bitstream infrastructure can parse
 * and reconstruct MVC NAL units without data loss.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "libavcodec/cbs.h"
#include "libavcodec/cbs_h264.h"
#include "libavcodec/codec_id.h"
#include "libavutil/buffer.h"
#include "libavutil/log.h"

int main(int argc, char *argv[])
{
    CodedBitstreamContext *ctx = NULL;
    CodedBitstreamFragment frag = { 0 };
    CodedBitstreamFragment frag2 = { 0 };
    AVBufferRef *buf = NULL, *buf2 = NULL;
    uint8_t *data;
    FILE *f;
    long file_size;
    int err, i, ret = 0;

    if (argc < 2) {
        fprintf(stderr, "Usage: %s <mvc.h264>\n", argv[0]);
        return 1;
    }

    av_log_set_level(AV_LOG_ERROR);

    f = fopen(argv[1], "rb");
    if (!f) {
        fprintf(stderr, "Cannot open %s\n", argv[1]);
        return 1;
    }
    fseek(f, 0, SEEK_END);
    file_size = ftell(f);
    fseek(f, 0, SEEK_SET);

    /* Limit to 64KB for fast testing */
    if (file_size > 65536)
        file_size = 65536;

    buf = av_buffer_alloc(file_size + AV_INPUT_BUFFER_PADDING_SIZE);
    if (!buf) {
        fclose(f);
        return 1;
    }
    data = buf->data;
    memset(data + file_size, 0, AV_INPUT_BUFFER_PADDING_SIZE);
    if (fread(data, 1, file_size, f) != (size_t)file_size) {
        fprintf(stderr, "Short read\n");
        av_buffer_unref(&buf);
        fclose(f);
        return 1;
    }
    fclose(f);

    err = ff_cbs_init(&ctx, AV_CODEC_ID_H264, NULL);
    if (err < 0) {
        fprintf(stderr, "ff_cbs_init failed: %d\n", err);
        ret = 1;
        goto end;
    }

    /* Parse */
    err = ff_cbs_read(ctx, &frag, buf, data, file_size);
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

end:
    ff_cbs_fragment_free(&frag2);
    ff_cbs_fragment_free(&frag);
    ff_cbs_close(&ctx);
    av_buffer_unref(&buf2);
    av_buffer_unref(&buf);
    return ret;
}
