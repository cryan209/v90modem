/* SPDX-License-Identifier: GPL-2.0-or-later
 * Cisco legacy X-CCD <-> RFC 4040 section 3 CLEARMODE compatibility.
 * Both carry opaque 8 kHz DS0 octets. Only format metadata changes:
 * memcpy preserves every octet, including both G.711 zero codewords.
 * No audio conversion, resampling, PLC, gain, or sample insertion.
 * Build with the target Asterisk 22 configured source tree.
 */
/*** MODULEINFO
    <support_level>extended</support_level>
 ***/
#include "asterisk.h"
#include "asterisk/astobj2.h"
#include "asterisk/cli.h"
#include "asterisk/codec.h"
#include "asterisk/format.h"
#include "asterisk/format_cache.h"
#include "asterisk/frame.h"
#include "asterisk/logger.h"
#include "asterisk/module.h"
#include "asterisk/rtp_engine.h"
#include "asterisk/translate.h"

#define CAPACITY 8096
static struct ast_format *xccd_format;
static struct ast_format *clear_format;
static int octet_samples(struct ast_frame *f) { return f->datalen; }
static int octet_length(unsigned int n) { return n; }
static struct ast_codec xccd = {
    .name = "x-ccd", .description = "Cisco legacy opaque 64 kbit/s DS0",
    .type = AST_MEDIA_TYPE_AUDIO, .sample_rate = 8000,
    .minimum_ms = 10, .maximum_ms = 140, .default_ms = 20,
    .minimum_bytes = 80, .samples_count = octet_samples,
    .get_length = octet_length, .smooth = 1,
};
static int copy_octets(struct ast_trans_pvt *p, struct ast_frame *f)
{
    if (f->datalen < 0 || f->samples != f->datalen
        || f->datalen > CAPACITY - p->datalen) {
        ast_log(LOG_ERROR, "Invalid opaque DS0 frame: %d bytes, %d samples\n",
            f->datalen, f->samples);
        return -1;
    }
    memcpy(p->outbuf.uc + p->datalen, f->data.ptr, f->datalen);
    p->datalen += f->datalen;
    p->samples += f->samples;
    return 0;
}
static struct ast_frame *sample_frame(struct ast_format *format)
{
    static unsigned char bytes[160];
    static struct ast_frame f = {
        .frametype = AST_FRAME_VOICE, .samples = 160, .datalen = 160,
        .data.ptr = bytes,
    };
    f.subclass.format = format;
    return &f;
}
static struct ast_frame *xccd_sample(void) { return sample_frame(xccd_format); }
static struct ast_frame *clear_sample(void) { return sample_frame(clear_format); }
#define CODEC_ID(n) { .name = n, .type = AST_MEDIA_TYPE_AUDIO, .sample_rate = 8000 }
static struct ast_translator to_clear = {
    .name = "xccd_to_clearmode", .src_codec = CODEC_ID("x-ccd"),
    .dst_codec = CODEC_ID("clearmode"), .format = "clearmode",
    .framein = copy_octets, .sample = xccd_sample,
    .buffer_samples = CAPACITY, .buf_size = CAPACITY, .table_cost = 1,
};
static struct ast_translator to_xccd = {
    .name = "clearmode_to_xccd", .src_codec = CODEC_ID("clearmode"),
    .dst_codec = CODEC_ID("x-ccd"), .format = "x-ccd",
    .framein = copy_octets, .sample = clear_sample,
    .buffer_samples = CAPACITY, .buf_size = CAPACITY, .table_cost = 1,
};
/* Exercise real Asterisk translation paths, mixed packet sizes, all 256
 * codewords and repeated frames. No packets are emitted by this test. */
static int check_direction(struct ast_format *src, struct ast_format *dst)
{
    struct ast_trans_pvt *path = ast_translator_build_path(dst, src);
    const int lengths[] = {80, 160, 256, 1120};
    unsigned char bytes[1120];
    int i, j, bad = 0;
    if (!path) { return -1; }
    for (i = 0; i < 256 && !bad; ++i) {
        int n = lengths[i % 4];
        struct ast_frame in = { .frametype = AST_FRAME_VOICE,
            .subclass.format = src, .samples = n, .datalen = n,
            .data.ptr = bytes };
        struct ast_frame *out;
        for (j = 0; j < n; ++j) { bytes[j] = (unsigned char)(j + i); }
        out = ast_translate(path, &in, 0);
        bad = !out || out->samples != n || out->datalen != n
            || ast_format_cmp(out->subclass.format, dst) != AST_FORMAT_CMP_EQUAL
            || memcmp(bytes, out->data.ptr, n);
        if (out) { ast_frfree(out); }
    }
    ast_translator_free_path(path);
    return bad ? -1 : 0;
}
static char *cli_check(struct ast_cli_entry *e, int cmd, struct ast_cli_args *a)
{
    if (cmd == CLI_INIT) {
        e->command = "xccd test";
        e->usage = "Usage: xccd test\n  Verify byte-exact X-CCD/CLEARMODE paths in both directions.\n";
        return NULL;
    }
    if (cmd == CLI_GENERATE) { return NULL; }
    if (a->argc != 2) { return CLI_SHOWUSAGE; }
    if (check_direction(xccd_format, clear_format)
        || check_direction(clear_format, xccd_format)) {
        ast_cli(a->fd, "FAIL: opaque DS0 translation test\n");
        return CLI_FAILURE;
    }
    ast_cli(a->fd, "PASS: 512 frames, 206848 bytes, all 256 codewords; unchanged bytes and sample counts in both directions\n");
    return CLI_SUCCESS;
}
static struct ast_cli_entry cli[] = {
    AST_CLI_DEFINE(cli_check, "Verify opaque X-CCD/CLEARMODE bridge"),
};
static int load_module(void)
{
    struct ast_codec *codec;
    clear_format = ast_format_cache_get("clearmode");
    if (!clear_format) {
        ast_log(LOG_ERROR, "Load codec_clearmode.so before codec_xccd.so\n");
        return AST_MODULE_LOAD_DECLINE;
    }
    if (ast_codec_register(&xccd)) { return AST_MODULE_LOAD_DECLINE; }
    codec = ast_codec_get("x-ccd", AST_MEDIA_TYPE_AUDIO, 8000);
    if (!codec) { return AST_MODULE_LOAD_DECLINE; }
    xccd_format = ast_format_create_named("x-ccd", codec);
    ao2_cleanup(codec);
    if (!xccd_format || ast_format_cache_set(xccd_format)
        || ast_rtp_engine_load_format(xccd_format)) { return AST_MODULE_LOAD_DECLINE; }
    if (ast_register_translator(&to_clear)) { return AST_MODULE_LOAD_DECLINE; }
    if (ast_register_translator(&to_xccd)) {
        ast_unregister_translator(&to_clear);
        return AST_MODULE_LOAD_DECLINE;
    }
    ast_cli_register_multiple(cli, ARRAY_LEN(cli));
    return AST_MODULE_LOAD_SUCCESS;
}
static int unload_module(void)
{
    /* Codec registration pins the module until Asterisk shutdown. */
    return -1;
}
AST_MODULE_INFO(ASTERISK_GPL_KEY, AST_MODFLAG_LOAD_ORDER,
    "Byte-exact Cisco X-CCD / RFC 4040 CLEARMODE bridge",
    .support_level = AST_MODULE_SUPPORT_EXTENDED,
    .load = load_module, .unload = unload_module,
    .load_pri = AST_MODPRI_CHANNEL_DEPEND,
    .requires = "codec_clearmode",
);
