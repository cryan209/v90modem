/* SPDX-License-Identifier: GPL-2.0-or-later
 * RFC 4040 CLEARMODE: opaque 64 kbit/s DS0 octets, never audio translation.
 * Inspired by https://gist.github.com/7marcus9/2702b4fd299983b61c1d516aecd7800f
 * Uses public Asterisk 22 APIs; build with the target PBX's configured tree.
 */
/*** MODULEINFO
    <support_level>extended</support_level>
 ***/
#include "asterisk.h"
#include "asterisk/astobj2.h"
#include "asterisk/codec.h"
#include "asterisk/format.h"
#include "asterisk/format_cache.h"
#include "asterisk/frame.h"
#include "asterisk/logger.h"
#include "asterisk/module.h"
#include "asterisk/rtp_engine.h"

static int clearmode_samples(struct ast_frame *frame)
{
    return frame->datalen;
}

static int clearmode_length(unsigned int samples)
{
    return samples;
}

static struct ast_codec clearmode = {
    .name = "clearmode",
    .description = "RFC 4040 opaque 64 kbit/s DS0",
    .type = AST_MEDIA_TYPE_AUDIO,
    .sample_rate = 8000,
    .minimum_ms = 10,
    .maximum_ms = 140,
    .default_ms = 20,
    .minimum_bytes = 80,
    .samples_count = clearmode_samples,
    .get_length = clearmode_length,
    .smooth = 1,
};

static int load_module(void)
{
    struct ast_codec *codec;
    struct ast_format *format;
    int result;

    if (ast_codec_register(&clearmode)) {
        return AST_MODULE_LOAD_DECLINE;
    }
    codec = ast_codec_get("clearmode", AST_MEDIA_TYPE_AUDIO, 8000);
    if (!codec) {
        return AST_MODULE_LOAD_DECLINE;
    }
    format = ast_format_create_named("clearmode", codec);
    ao2_cleanup(codec);
    if (!format) {
        return AST_MODULE_LOAD_DECLINE;
    }
    result = ast_format_cache_set(format);
    if (!result) {
        result = ast_rtp_engine_load_format(format);
    }
    ao2_cleanup(format);
    return result ? AST_MODULE_LOAD_DECLINE : AST_MODULE_LOAD_SUCCESS;
}

static int unload_module(void)
{
    /* Asterisk codec registration pins its module until process shutdown:
     * main/codec.c __ast_codec_register_with_format(). No unregister API.
     */
    ast_log(LOG_WARNING, "CLEARMODE remains registered until Asterisk shuts down\n");
    return -1;
}

AST_MODULE_INFO(ASTERISK_GPL_KEY, AST_MODFLAG_LOAD_ORDER,
    "RFC 4040 CLEARMODE passthrough (no translation)",
    .support_level = AST_MODULE_SUPPORT_EXTENDED,
    .load = load_module,
    .unload = unload_module,
    .load_pri = AST_MODPRI_CHANNEL_DEPEND,
);
