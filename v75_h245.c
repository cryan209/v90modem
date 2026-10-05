/*
 * v75_h245.c -- V.75's typed H.245 messages <-> ASN.1 aligned PER.
 *
 * Builds and reads values of MultimediaSystemControlMessage (generated schema,
 * h245_schema.c) and lets per.c encode them.  Covers V.75 Tables 3-6 plus
 * Cor.1; a field outside the DSVD subset makes encode/decode fail rather than
 * silently drop it.
 */

#include "v75.h"
#include "per.h"

#include <string.h>

typedef struct {
    a_arena_t *a;
    int err;
    a_val_t *top;               /* the MultimediaSystemControlMessage being built */
} E;

static a_val_t *M(E *e, a_val_t *s, const char *n)
{
    a_val_t *c = a_member(e->a, s, n);

    if (!c)
        e->err = 1;
    return c;
}

static a_val_t *C(E *e, a_val_t *ch, const char *alt)
{
    a_val_t *c = a_choice(e->a, ch, alt);

    if (!c)
        e->err = 1;
    return c;
}

static void I(E *e, a_val_t *s, const char *n, int64_t x)
{
    a_val_t *c = M(e, s, n);

    if (c && a_set_int(c, x) < 0)
        e->err = 1;
}

static void B(E *e, a_val_t *s, const char *n, int x)
{
    a_val_t *c = M(e, s, n);

    if (c)
        a_set_bool(c, x);
}

static int64_t gi(const a_val_t *s, const char *n, int64_t dflt)
{
    const a_val_t *c = a_get(s, n);

    return c ? c->i : dflt;
}

/* ---- audio ---------------------------------------------------------------- */

static const struct { v75_audio_cap_t cap; const char *name; } AUD[] = {
    { V75_AUDIO_G711_ALAW_64K, "g711Alaw64k" }, { V75_AUDIO_G711_ALAW_56K, "g711Alaw56k" },
    { V75_AUDIO_G711_ULAW_64K, "g711Ulaw64k" }, { V75_AUDIO_G711_ULAW_56K, "g711Ulaw56k" },
    { V75_AUDIO_G722_64K, "g722-64k" }, { V75_AUDIO_G722_56K, "g722-56k" },
    { V75_AUDIO_G722_48K, "g722-48k" }, { V75_AUDIO_G723, "g7231" },
    { V75_AUDIO_G728, "g728" }, { V75_AUDIO_G729, "g729" },
    { V75_AUDIO_G729_ANNEX_A, "g729AnnexA" }, { V75_AUDIO_G729_W_ANNEX_B, "g729wAnnexB" },
    { V75_AUDIO_G729_ANNEX_A_W_ANNEX_B, "g729AnnexAwAnnexB" },
};
#define NAUD ((int)(sizeof(AUD) / sizeof(AUD[0])))

static const char *aud_name(v75_audio_cap_t c)
{
    int i;

    for (i = 0; i < NAUD; i++)
        if (AUD[i].cap == c)
            return AUD[i].name;
    return NULL;
}

static void put_audio(E *e, a_val_t *ac, const v75_audio_t *a)
{
    const char *n = aud_name(a->cap);
    a_val_t *c;

    if (!n) {
        e->err = 1;
        return;
    }
    c = C(e, ac, n);
    if (!c)
        return;
    if (a->cap == V75_AUDIO_G723) {
        I(e, c, "maxAl-sduAudioFrames", a->frames);
        B(e, c, "silenceSuppression", a->silence_suppression);
    } else {
        a_set_int(c, a->frames) < 0 ? (void)(e->err = 1) : (void)0;
    }
}

static int get_audio(const a_val_t *ac, v75_audio_t *a)
{
    const char *n = a_choice_name(ac);
    const a_val_t *v = a_choice_val(ac);
    int i;

    memset(a, 0, sizeof(*a));
    for (i = 0; n && i < NAUD; i++) {
        if (strcmp(AUD[i].name, n) == 0) {
            a->cap = AUD[i].cap;
            if (a->cap == V75_AUDIO_G723) {
                a->frames = (int)gi(v, "maxAl-sduAudioFrames", 1);
                a->silence_suppression = gi(v, "silenceSuppression", 0) != 0;
            } else {
                a->frames = (int)v->i;
            }
            return 0;
        }
    }
    return -1;
}

/* ---- data ----------------------------------------------------------------- */

static const struct { v75_dataproto_t p; const char *name; } DP[] = {
    { V75_DP_V14_BUFFERED, "v14buffered" }, { V75_DP_V42_LAPM, "v42lapm" },
    { V75_DP_HDLC_TUNNELLING, "hdlcFrameTunnelling" }, { V75_DP_TRANSPARENT, "transparent" },
    { V75_DP_SEGMENTATION_REASSEMBLY, "segmentationAndReassembly" },
    { V75_DP_HDLC_TUNNELLING_W_SAR, "hdlcFrameTunnelingwSAR" }, { V75_DP_V120, "v120" },
    { V75_DP_V76_W_COMPRESSION, "v76wCompression" },
};
#define NDP ((int)(sizeof(DP) / sizeof(DP[0])))

static void put_proto(E *e, a_val_t *dp, const v75_data_t *d)
{
    const char *n = NULL;
    a_val_t *c;
    int i;

    for (i = 0; i < NDP; i++)
        if (DP[i].p == d->protocol)
            n = DP[i].name;
    if (!n) {
        e->err = 1;
        return;
    }
    c = C(e, dp, n);
    if (c && d->protocol == V75_DP_V76_W_COMPRESSION) {
        static const char *const dir[4] = { 0, "transmitCompression", "receiveCompression",
                                            "transmitAndReceiveCompression" };
        a_val_t *ct, *v;

        if (d->compression < 1 || d->compression > 3) {
            e->err = 1;
            return;
        }
        ct = C(e, c, dir[d->compression]);
        v = C(e, ct, "v42bis");
        if (v) {
            I(e, v, "numberOfCodewords", d->v42bis_codewords);
            I(e, v, "maximumStringLength", d->v42bis_string);
        }
    }
}

static void get_proto(const a_val_t *dp, v75_data_t *d)
{
    const char *n = a_choice_name(dp);
    const a_val_t *v = a_choice_val(dp);
    int i;

    for (i = 0; n && i < NDP; i++)
        if (strcmp(DP[i].name, n) == 0)
            d->protocol = DP[i].p;
    if (d->protocol == V75_DP_V76_W_COMPRESSION) {
        const char *cn = a_choice_name(v);
        const a_val_t *ct = a_choice_val(v);
        const a_val_t *b = a_choice_val(ct);

        d->compression = cn && !strcmp(cn, "transmitCompression") ? 1 :
                         cn && !strcmp(cn, "receiveCompression") ? 2 : 3;
        d->v42bis_codewords = (int)gi(b, "numberOfCodewords", 0);
        d->v42bis_string = (int)gi(b, "maximumStringLength", 0);
    }
}

static void put_data(E *e, a_val_t *dac, const v75_data_t *d)
{
    a_val_t *app = M(e, dac, "application"), *c;
    const char *n;

    if (!app)
        return;
    if (d->app == V75_APP_DSVD_CONTROL) {
        C(e, app, "dsvdControl");
    } else {
        n = d->app == V75_APP_T120 ? "t120" : d->app == V75_APP_T434 ? "t434" :
            d->app == V75_APP_NONE ? "userData" : NULL;
        if (!n) {
            e->err = 1;                 /* t84 / nlpid / non-standard: not mapped */
            return;
        }
        c = C(e, app, n);
        if (c)
            put_proto(e, c, d);
    }
    I(e, dac, "maxBitRate", d->max_bit_rate);
}

static int get_data(const a_val_t *dac, v75_data_t *d)
{
    const a_val_t *app = a_get(dac, "application");
    const char *n = a_choice_name(app);

    memset(d, 0, sizeof(*d));
    if (!n)
        return -1;
    if (!strcmp(n, "dsvdControl"))
        d->app = V75_APP_DSVD_CONTROL;
    else {
        d->app = !strcmp(n, "t120") ? V75_APP_T120 : !strcmp(n, "t434") ? V75_APP_T434 : V75_APP_NONE;
        get_proto(a_choice_val(app), d);
    }
    d->max_bit_rate = (int)gi(dac, "maxBitRate", 0);
    return 0;
}

/* ---- V.76 logical channel parameters ------------------------------------- */

static void put_v76(E *e, a_val_t *lp, const v75_v76_params_t *m)
{
    a_val_t *h = M(e, lp, "hdlcParameters"), *x, *mode;

    if (!h)
        return;
    x = M(e, h, "crcLength");
    C(e, x, m->crc_len == 1 ? "crc8bit" : m->crc_len == 4 ? "crc32bit" : "crc16bit");
    I(e, h, "n401", m->n401);
    B(e, h, "loopbackTestProcedure", m->loopback_test);
    x = M(e, lp, "suspendResume");
    C(e, x, m->suspend_resume == V75_SR_WITH_ADDRESS ? "suspendResumewAddress" :
            m->suspend_resume == V75_SR_WITHOUT_ADDRESS ? "suspendResumewoAddress" :
            "noSuspendResume");
    B(e, lp, "uIH", m->uih);
    mode = M(e, lp, "mode");
    if (m->mode == V76_ERM) {
        a_val_t *erm = C(e, mode, "eRM");

        I(e, erm, "windowSize", m->window);
        x = M(e, erm, "recovery");
        C(e, x, m->recovery == V76_REC_SREJ ? "sREJ" : "rej");
    } else {
        C(e, mode, "uNERM");
    }
    x = M(e, lp, "v75Parameters");
    B(e, x, "audioHeaderPresent", m->audio_header);
}

static void get_v76(const a_val_t *lp, v75_v76_params_t *m)
{
    const a_val_t *h = a_get(lp, "hdlcParameters");
    const char *n;
    const a_val_t *mode = a_get(lp, "mode");

    memset(m, 0, sizeof(*m));
    n = a_choice_name(a_get(h, "crcLength"));
    m->crc_len = n && !strcmp(n, "crc8bit") ? 1 : n && !strcmp(n, "crc32bit") ? 4 : 2;
    m->n401 = (int)gi(h, "n401", 0);
    m->loopback_test = gi(h, "loopbackTestProcedure", 0) != 0;
    n = a_choice_name(a_get(lp, "suspendResume"));
    m->suspend_resume = n && !strcmp(n, "suspendResumewAddress") ? V75_SR_WITH_ADDRESS :
                        n && !strcmp(n, "suspendResumewoAddress") ? V75_SR_WITHOUT_ADDRESS :
                        V75_SR_NONE;
    m->uih = gi(lp, "uIH", 0) != 0;
    n = a_choice_name(mode);
    if (n && !strcmp(n, "eRM")) {
        const a_val_t *erm = a_choice_val(mode);
        const char *r = a_choice_name(a_get(erm, "recovery"));

        m->mode = V76_ERM;
        m->window = (int)gi(erm, "windowSize", 0);
        m->recovery = r && !strcmp(r, "sREJ") ? V76_REC_SREJ : V76_REC_REJ;
    } else {
        m->mode = V76_UNERM;
    }
    m->audio_header = gi(a_get(lp, "v75Parameters"), "audioHeaderPresent", 0) != 0;
}

static void put_datatype(E *e, a_val_t *dt, const v75_olc_dir_t *d)
{
    if (d->media == V75_MEDIA_AUDIO) {
        a_val_t *a = C(e, dt, "audioData");

        if (a)
            put_audio(e, a, &d->audio);
    } else {
        a_val_t *a = C(e, dt, "data");

        if (a)
            put_data(e, a, &d->data);
    }
}

static int get_datatype(const a_val_t *dt, v75_olc_dir_t *d)
{
    const char *n = a_choice_name(dt);

    if (!n)
        return -1;
    if (!strcmp(n, "audioData")) {
        d->media = V75_MEDIA_AUDIO;
        return get_audio(a_choice_val(dt), &d->audio);
    }
    if (!strcmp(n, "data")) {
        d->media = V75_MEDIA_DATA;
        return get_data(a_choice_val(dt), &d->data);
    }
    return -1;
}

/* ---- the messages ----------------------------------------------------------- */

static const a_type_t *top_type(void)
{
    return a_find_type("MultimediaSystemControlMessage");
}

static a_val_t *begin(E *e, const char *group, const char *msg)
{
    a_val_t *top = a_new(e->a, top_type()), *g, *m;

    if (!top) {
        e->err = 1;
        return NULL;
    }
    e->top = top;
    g = C(e, top, group);
    m = g ? C(e, g, msg) : NULL;
    return m;
}

static void put_tcs(E *e, const v75_tcs_t *t, a_val_t *m)
{
    static const uint32_t oid[6] = { 0, 0, 8, 245, 0, 1 };
    int i, j, k;

    I(e, m, "sequenceNumber", t->sequence_number);
    {
        a_val_t *p = M(e, m, "protocolIdentifier");

        if (p)
            a_set_oid(e->a, p, oid, 6);
    }
    if (t->has_mux) {
        a_val_t *mc = M(e, m, "multiplexCapability"), *v = C(e, mc, "v76Capability");

        if (v) {
            B(e, v, "suspendResumeCapabilitywAddress", t->mux.sr_with_address);
            B(e, v, "suspendResumeCapabilitywoAddress", t->mux.sr_without_address);
            B(e, v, "rejCapability", t->mux.rej);
            B(e, v, "sREJCapability", t->mux.srej);
            B(e, v, "mREJCapability", t->mux.msrej);
            B(e, v, "crc8bitCapability", t->mux.crc8);
            B(e, v, "crc16bitCapability", t->mux.crc16);
            B(e, v, "crc32bitCapability", t->mux.crc32);
            B(e, v, "uihCapability", t->mux.uih);
            I(e, v, "numOfDLCS", t->mux.num_dlcs);
            B(e, v, "twoOctetAddressFieldCapability", t->mux.two_octet_address);
            B(e, v, "loopBackTestCapability", t->mux.loopback_test);
            I(e, v, "n401Capability", t->mux.n401);
            I(e, v, "maxWindowSizeCapability", t->mux.max_window);
            B(e, M(e, v, "v75Capability"), "audioHeader", t->mux.audio_header);
        }
    }
    if (t->n_caps > 0) {
        a_val_t *tab = M(e, m, "capabilityTable");

        for (i = 0; i < t->n_caps; i++) {
            a_val_t *en = a_append(e->a, tab), *cap, *c;

            if (!en) {
                e->err = 1;
                return;
            }
            I(e, en, "capabilityTableEntryNumber", t->caps[i].number);
            cap = M(e, en, "capability");
            if (t->caps[i].is_audio) {
                c = C(e, cap, "receiveAndTransmitAudioCapability");
                if (c)
                    put_audio(e, c, &t->caps[i].audio);
            } else {
                c = C(e, cap, "receiveAndTransmitDataApplicationCapability");
                if (c)
                    put_data(e, c, &t->caps[i].data);
            }
        }
    }
    if (t->n_desc > 0) {
        a_val_t *ds = M(e, m, "capabilityDescriptors");

        for (i = 0; i < t->n_desc; i++) {
            a_val_t *d = a_append(e->a, ds), *sc;

            if (!d) {
                e->err = 1;
                return;
            }
            I(e, d, "capabilityDescriptorNumber", t->desc[i].number);
            if (t->desc[i].n_sets > 0) {
                sc = M(e, d, "simultaneousCapabilities");
                for (j = 0; j < t->desc[i].n_sets; j++) {
                    a_val_t *alt = a_append(e->a, sc);

                    if (!alt) {
                        e->err = 1;
                        return;
                    }
                    for (k = 0; k < t->desc[i].set[j].n_alts; k++) {
                        a_val_t *n = a_append(e->a, alt);

                        if (!n || a_set_int(n, t->desc[i].set[j].alt[k]) < 0)
                            e->err = 1;
                    }
                }
            }
        }
    }
}

static int get_tcs(const a_val_t *m, v75_tcs_t *t)
{
    const a_val_t *tab = a_get(m, "capabilityTable"), *ds = a_get(m, "capabilityDescriptors");
    const a_val_t *mc = a_get(m, "multiplexCapability");
    int i, j, k;

    memset(t, 0, sizeof(*t));
    t->sequence_number = (int)gi(m, "sequenceNumber", 0);
    if (mc) {
        const a_val_t *v = a_choice_val(mc);
        const a_val_t *a = a_get(v, "v75Capability");

        t->has_mux = true;
        t->mux.sr_with_address = gi(v, "suspendResumeCapabilitywAddress", 0);
        t->mux.sr_without_address = gi(v, "suspendResumeCapabilitywoAddress", 0);
        t->mux.rej = gi(v, "rejCapability", 0);
        t->mux.srej = gi(v, "sREJCapability", 0);
        t->mux.msrej = gi(v, "mREJCapability", 0);
        t->mux.crc8 = gi(v, "crc8bitCapability", 0);
        t->mux.crc16 = gi(v, "crc16bitCapability", 0);
        t->mux.crc32 = gi(v, "crc32bitCapability", 0);
        t->mux.uih = gi(v, "uihCapability", 0);
        t->mux.num_dlcs = (int)gi(v, "numOfDLCS", 0);
        t->mux.two_octet_address = gi(v, "twoOctetAddressFieldCapability", 0);
        t->mux.loopback_test = gi(v, "loopBackTestCapability", 0);
        t->mux.n401 = (int)gi(v, "n401Capability", 0);
        t->mux.max_window = (int)gi(v, "maxWindowSizeCapability", 0);
        t->mux.audio_header = gi(a, "audioHeader", 0) != 0;
    }
    if (tab) {
        for (i = 0; i < tab->n; i++) {
            const a_val_t *en = tab->kids[i], *cap = a_get(en, "capability");
            const char *n = a_choice_name(cap);
            v75_capentry_t *c;

            if (t->n_caps >= V75_MAX_CAPS)
                return -1;
            c = &t->caps[t->n_caps];
            memset(c, 0, sizeof(*c));
            c->number = (int)gi(en, "capabilityTableEntryNumber", 0);
            if (n && strstr(n, "AudioCapability")) {
                c->is_audio = true;
                if (get_audio(a_choice_val(cap), &c->audio) < 0)
                    return -1;
            } else if (n && strstr(n, "DataApplicationCapability")) {
                if (get_data(a_choice_val(cap), &c->data) < 0)
                    return -1;
            }
            t->n_caps++;
        }
    }
    if (ds) {
        for (i = 0; i < ds->n; i++) {
            const a_val_t *d = ds->kids[i], *sc = a_get(d, "simultaneousCapabilities");
            v75_capdesc_t *cd;

            if (t->n_desc >= V75_MAX_DESCRIPTORS)
                return -1;
            cd = &t->desc[t->n_desc++];
            cd->number = (int)gi(d, "capabilityDescriptorNumber", 0);
            for (j = 0; sc && j < sc->n; j++) {
                const a_val_t *alt = sc->kids[j];

                if (cd->n_sets >= V75_MAX_SIMUL || alt->n > V75_MAX_ALTS)
                    return -1;
                cd->set[cd->n_sets].n_alts = alt->n;
                for (k = 0; k < alt->n; k++)
                    cd->set[cd->n_sets].alt[k] = (int)alt->kids[k]->i;
                cd->n_sets++;
            }
        }
    }
    return 0;
}

static void put_olc_dir(E *e, a_val_t *fp, const v75_olc_dir_t *d, int forward)
{
    a_val_t *x;

    if (forward && d->has_port)
        I(e, fp, "portNumber", d->port);
    put_datatype(e, M(e, fp, "dataType"), d);
    x = M(e, fp, "multiplexParameters");
    x = C(e, x, "v76LogicalChannelParameters");
    if (x)
        put_v76(e, x, &d->mux);
}

static int get_olc_dir(const a_val_t *fp, v75_olc_dir_t *d, int forward)
{
    const a_val_t *mp = a_get(fp, "multiplexParameters");
    const char *n = a_choice_name(mp);

    memset(d, 0, sizeof(*d));
    if (forward && a_get(fp, "portNumber")) {
        d->has_port = true;
        d->port = (int)gi(fp, "portNumber", 0);
    }
    if (get_datatype(a_get(fp, "dataType"), d) < 0)
        return -1;
    if (n && !strcmp(n, "v76LogicalChannelParameters"))
        get_v76(a_choice_val(mp), &d->mux);
    return 0;
}

static void put_mode_audio(E *e, a_val_t *am, v75_audio_cap_t cap, int frames)
{
    const char *n = aud_name(cap);
    a_val_t *c;

    if (!n) {
        e->err = 1;
        return;
    }
    c = C(e, am, n);
    if (!c)
        return;
    if (cap == V75_AUDIO_G723)
        a_choice_at(e->a, c, 0);
    else if (cap == V75_AUDIO_G729_W_ANNEX_B || cap == V75_AUDIO_G729_ANNEX_A_W_ANNEX_B)
        a_set_int(c, frames) < 0 ? (void)(e->err = 1) : (void)0;
}

static int encode_msg_tree(const v75_msg_t *m, E *e, a_val_t **out)
{
    a_val_t *v = NULL;

    switch (m->type) {
    case V75_MSG_TCS:
        v = begin(e, "request", "terminalCapabilitySet");
        if (v)
            put_tcs(e, &m->u.tcs, v);
        break;
    case V75_MSG_TCS_ACK:
        v = begin(e, "response", "terminalCapabilitySetAck");
        if (v)
            I(e, v, "sequenceNumber", m->u.tcs_ack_sequence);
        break;
    case V75_MSG_TCS_REJECT:
        v = begin(e, "response", "terminalCapabilitySetReject");
        if (v) {
            a_val_t *c = M(e, v, "cause");

            I(e, v, "sequenceNumber", m->u.tcs_reject.sequence_number);
            if (m->u.tcs_reject.cause >= 3) {
                a_val_t *in = a_choice_at(e->a, c, 3);

                if (in)
                    a_choice_at(e->a, in, 1);       /* noneProcessed */
            } else if (!a_choice_at(e->a, c, m->u.tcs_reject.cause)) {
                e->err = 1;
            }
        }
        break;
    case V75_MSG_OLC:
        v = begin(e, "request", "openLogicalChannel");
        if (v) {
            const v75_olc_t *o = &m->u.olc;

            I(e, v, "forwardLogicalChannelNumber", o->fwd.channel);
            put_olc_dir(e, M(e, v, "forwardLogicalChannelParameters"), &o->fwd, 1);
            if (o->has_rev)
                put_olc_dir(e, M(e, v, "reverseLogicalChannelParameters"), &o->rev, 0);
        }
        break;
    case V75_MSG_OLC_ACK:
        v = begin(e, "response", "openLogicalChannelAck");
        if (v) {
            a_val_t *r = M(e, v, "reverseLogicalChannelParameters");

            I(e, v, "forwardLogicalChannelNumber", m->u.olc_ack.forward_channel);
            I(e, r, "reverseLogicalChannelNumber", m->u.olc_ack.reverse_channel);
            if (m->u.olc_ack.has_port)
                I(e, r, "portNumber", m->u.olc_ack.port);
        }
        break;
    case V75_MSG_OLC_REJECT:
        v = begin(e, "response", "openLogicalChannelReject");
        if (v) {
            I(e, v, "forwardLogicalChannelNumber", m->u.olc_reject.forward_channel);
            if (!a_choice_at(e->a, M(e, v, "cause"), m->u.olc_reject.cause))
                e->err = 1;
        }
        break;
    case V75_MSG_CLC:
        v = begin(e, "request", "closeLogicalChannel");
        if (v) {
            I(e, v, "forwardLogicalChannelNumber", m->u.clc.forward_channel);
            C(e, M(e, v, "source"), m->u.clc.source_lcse ? "lcse" : "user");
        }
        break;
    case V75_MSG_CLC_ACK:
        v = begin(e, "response", "closeLogicalChannelAck");
        if (v)
            I(e, v, "forwardLogicalChannelNumber", m->u.clc_ack_channel);
        break;
    case V75_MSG_REQUEST_MODE: {
        const v75_request_mode_t *r = &m->u.request_mode;

        v = begin(e, "request", "requestMode");
        if (v) {
            a_val_t *rm = M(e, v, "requestedModes"), *md, *me, *ty;

            I(e, v, "sequenceNumber", r->sequence_number);
            md = a_append(e->a, rm);
            me = md ? a_append(e->a, md) : NULL;
            if (!me) {
                e->err = 1;
                break;
            }
            ty = M(e, me, "type");
            if (r->media == V75_MEDIA_AUDIO) {
                put_mode_audio(e, C(e, ty, "audioMode"), r->audio, r->audio_frames);
            } else {
                a_val_t *dm = C(e, ty, "dataMode"), *app, *c;
                const char *n;

                if (dm) {
                    app = M(e, dm, "application");
                    if (r->data.app == V75_APP_DSVD_CONTROL) {
                        C(e, app, "dsvdControl");
                    } else {
                        n = r->data.app == V75_APP_T120 ? "t120" :
                            r->data.app == V75_APP_T434 ? "t434" : "userData";
                        c = C(e, app, n);
                        if (c)
                            put_proto(e, c, &r->data);
                    }
                    I(e, dm, "bitRate", r->data_bit_rate);
                }
            }
            if (r->v76_mode != V75_SR_NONE)
                C(e, M(e, me, "v76ModeParameters"),
                  r->v76_mode == V75_SR_WITH_ADDRESS ? "suspendResumewAddress"
                                                     : "suspendResumewoAddress");
            if (r->logical_channel > 0)
                I(e, me, "logicalChannelNumber", r->logical_channel);
        }
        break;
    }
    case V75_MSG_REQUEST_MODE_ACK:
        v = begin(e, "response", "requestModeAck");
        if (v) {
            I(e, v, "sequenceNumber", m->u.request_mode_ack.sequence_number);
            if (!a_choice_at(e->a, M(e, v, "response"), m->u.request_mode_ack.response))
                e->err = 1;
        }
        break;
    case V75_MSG_REQUEST_MODE_REJECT:
        v = begin(e, "response", "requestModeReject");
        if (v) {
            I(e, v, "sequenceNumber", m->u.request_mode_reject.sequence_number);
            if (!a_choice_at(e->a, M(e, v, "cause"), m->u.request_mode_reject.cause))
                e->err = 1;
        }
        break;
    default:
        e->err = 1;
    }
    *out = v;
    return e->err ? -1 : 0;
}

/* EndSessionCommand's value is itself a CHOICE; begin() has selected the
 * command, so the alternative inside needs its own step. */
static int encode_end_session(const v75_msg_t *m, E *e, a_val_t **top)
{
    static const char *const opt[] = { 0, "telephonyMode", "v8bis", "v34DSVD",
                                       "v34DuplexFAX", "v34H324" };
    a_val_t *es = begin(e, "command", "endSessionCommand");

    if (!es)
        return -1;
    if (m->u.end_session == V75_END_DISCONNECT) {
        C(e, es, "disconnect");
    } else {
        a_val_t *g = C(e, es, "gstnOptions");

        if (!g)
            return -1;
        C(e, g, opt[m->u.end_session]);
    }
    (void)top;
    return e->err ? -1 : 0;
}

static int h245_encode(const v75_msg_t *m, uint8_t *out, int max)
{
    static __thread uint8_t arena_buf[65536];
    a_arena_t ar;
    E e = { &ar, 0, NULL };
    a_val_t *v = NULL;

    a_arena_init(&ar, arena_buf, sizeof(arena_buf));
    if (m->type == V75_MSG_END_SESSION) {
        if (encode_end_session(m, &e, &v) < 0)
            return -1;
    } else if (encode_msg_tree(m, &e, &v) < 0) {
        return -1;
    }
    (void)v;
    return e.err || !e.top ? -1 : a_encode(e.top, out, max);
}

static int h245_decode(const uint8_t *in, int len, v75_msg_t *m)
{
    static __thread uint8_t arena_buf[65536];
    a_arena_t ar;
    a_val_t *top;
    const char *g, *n;
    const a_val_t *gv, *mv;

    a_arena_init(&ar, arena_buf, sizeof(arena_buf));
    memset(m, 0, sizeof(*m));
    top = a_decode(&ar, top_type(), in, len);
    if (!top)
        return -1;
    g = a_choice_name(top);
    gv = a_choice_val(top);
    n = a_choice_name(gv);
    mv = a_choice_val(gv);
    if (!g || !n)
        return -1;
    if (!strcmp(n, "terminalCapabilitySet")) {
        m->type = V75_MSG_TCS;
        return get_tcs(mv, &m->u.tcs);
    }
    if (!strcmp(n, "terminalCapabilitySetAck")) {
        m->type = V75_MSG_TCS_ACK;
        m->u.tcs_ack_sequence = (int)gi(mv, "sequenceNumber", 0);
        return 0;
    }
    if (!strcmp(n, "terminalCapabilitySetReject")) {
        m->type = V75_MSG_TCS_REJECT;
        m->u.tcs_reject.sequence_number = (int)gi(mv, "sequenceNumber", 0);
        m->u.tcs_reject.cause = a_choice_index(a_get(mv, "cause"));
        return 0;
    }
    if (!strcmp(n, "openLogicalChannel")) {
        const a_val_t *f = a_get(mv, "forwardLogicalChannelParameters");
        const a_val_t *r = a_get(mv, "reverseLogicalChannelParameters");
        v75_olc_t *o = &m->u.olc;

        m->type = V75_MSG_OLC;
        if (get_olc_dir(f, &o->fwd, 1) < 0)
            return -1;
        o->fwd.channel = (int)gi(mv, "forwardLogicalChannelNumber", 0);
        if (r) {
            o->has_rev = true;
            if (get_olc_dir(r, &o->rev, 0) < 0)
                return -1;
            o->rev.channel = o->fwd.channel;
        }
        return 0;
    }
    if (!strcmp(n, "openLogicalChannelAck")) {
        const a_val_t *r = a_get(mv, "reverseLogicalChannelParameters");

        m->type = V75_MSG_OLC_ACK;
        m->u.olc_ack.forward_channel = (int)gi(mv, "forwardLogicalChannelNumber", 0);
        m->u.olc_ack.reverse_channel = (int)gi(r, "reverseLogicalChannelNumber",
                                               m->u.olc_ack.forward_channel);
        if (r && a_get(r, "portNumber")) {
            m->u.olc_ack.has_port = true;
            m->u.olc_ack.port = (int)gi(r, "portNumber", 0);
        }
        return 0;
    }
    if (!strcmp(n, "openLogicalChannelReject")) {
        m->type = V75_MSG_OLC_REJECT;
        m->u.olc_reject.forward_channel = (int)gi(mv, "forwardLogicalChannelNumber", 0);
        m->u.olc_reject.cause = a_choice_index(a_get(mv, "cause"));
        return 0;
    }
    if (!strcmp(n, "closeLogicalChannel")) {
        const char *s = a_choice_name(a_get(mv, "source"));

        m->type = V75_MSG_CLC;
        m->u.clc.forward_channel = (int)gi(mv, "forwardLogicalChannelNumber", 0);
        m->u.clc.source_lcse = s && !strcmp(s, "lcse");
        return 0;
    }
    if (!strcmp(n, "closeLogicalChannelAck")) {
        m->type = V75_MSG_CLC_ACK;
        m->u.clc_ack_channel = (int)gi(mv, "forwardLogicalChannelNumber", 0);
        return 0;
    }
    if (!strcmp(n, "endSessionCommand")) {
        const a_val_t *es = mv;                     /* the EndSessionCommand value */
        const char *alt = a_choice_name(es);
        const char *o = a_choice_name(a_choice_val(es));

        m->type = V75_MSG_END_SESSION;
        if (alt && !strcmp(alt, "gstnOptions") && o)
            m->u.end_session = !strcmp(o, "telephonyMode") ? V75_END_GSTN_TELEPHONY :
                               !strcmp(o, "v8bis") ? V75_END_GSTN_V8BIS :
                               !strcmp(o, "v34DSVD") ? V75_END_GSTN_V34_DSVD :
                               !strcmp(o, "v34DuplexFAX") ? V75_END_GSTN_V34_DUPLEX_FAX :
                               V75_END_GSTN_V34_H324;
        else
            m->u.end_session = V75_END_DISCONNECT;
        return 0;
    }
    if (!strcmp(n, "requestMode")) {
        const a_val_t *rm = a_get(mv, "requestedModes");
        const a_val_t *md = rm && rm->n ? rm->kids[0] : NULL;
        const a_val_t *me = md && md->n ? md->kids[0] : NULL;
        const a_val_t *ty = a_get(me, "type");
        const char *tn = a_choice_name(ty);
        v75_request_mode_t *r = &m->u.request_mode;
        const char *s;

        m->type = V75_MSG_REQUEST_MODE;
        r->sequence_number = (int)gi(mv, "sequenceNumber", 0);
        if (!tn)
            return -1;
        if (!strcmp(tn, "audioMode")) {
            const a_val_t *am = a_choice_val(ty);
            const char *an = a_choice_name(am);
            int i;

            r->media = V75_MEDIA_AUDIO;
            for (i = 0; an && i < NAUD; i++)
                if (!strcmp(AUD[i].name, an))
                    r->audio = AUD[i].cap;
            if (r->audio == V75_AUDIO_G729_W_ANNEX_B || r->audio == V75_AUDIO_G729_ANNEX_A_W_ANNEX_B)
                r->audio_frames = (int)a_choice_val(am)->i;
        } else {
            const a_val_t *dm = a_choice_val(ty);

            r->media = V75_MEDIA_DATA;
            if (get_data(dm, &r->data) < 0)
                return -1;
            r->data_bit_rate = (int)gi(dm, "bitRate", 0);
        }
        s = a_choice_name(a_get(me, "v76ModeParameters"));
        r->v76_mode = s && !strcmp(s, "suspendResumewAddress") ? V75_SR_WITH_ADDRESS :
                      s ? V75_SR_WITHOUT_ADDRESS : V75_SR_NONE;
        r->logical_channel = (int)gi(me, "logicalChannelNumber", 0);
        return 0;
    }
    if (!strcmp(n, "requestModeAck")) {
        m->type = V75_MSG_REQUEST_MODE_ACK;
        m->u.request_mode_ack.sequence_number = (int)gi(mv, "sequenceNumber", 0);
        m->u.request_mode_ack.response = a_choice_index(a_get(mv, "response"));
        return 0;
    }
    if (!strcmp(n, "requestModeReject")) {
        m->type = V75_MSG_REQUEST_MODE_REJECT;
        m->u.request_mode_reject.sequence_number = (int)gi(mv, "sequenceNumber", 0);
        m->u.request_mode_reject.cause = a_choice_index(a_get(mv, "cause"));
        return 0;
    }
    return -1;
}

const v75_h245_codec_t v75_h245_codec = { h245_encode, h245_decode };
