#include <stdint.h>
#include <stdio.h>

/*
 * Read-only link-time diagnostic hook for SmartLink's analogue-side V.90
 * linear-mapping study.  Each record contains the two signed samples passed
 * to V90Demapper::linearMappingStudy().  The first call also snapshots the
 * demapper state so its learned Ucode tables and counters can be inspected.
 *
 * Add an alias for the original method before linking this object first:
 *
 *   objcopy --add-symbol \
 *     _ZN11V90Demapper18linearMappingStudyEss_original=.text:0x31410,global,function \
 *     dsplibs.o dsplibs_linmap.o
 */

extern void smartlink_linear_mapping_original(void *demapper,
                                              int16_t first,
                                              int16_t second)
    __asm__("_ZN11V90Demapper18linearMappingStudyEss_original");

void smartlink_linear_mapping_hook(void *demapper,
                                   int16_t first,
                                   int16_t second)
    __asm__("_ZN11V90Demapper18linearMappingStudyEss");

static FILE *record_file;
static unsigned record_count;

static void snapshot_demapper(const void *demapper)
{
    FILE *fp = fopen("/tmp/smartlink-demapper-before.bin", "wb");

    if (!fp)
        return;
    (void)fwrite(demapper, 1, 0x1ec0, fp);
    fclose(fp);
}

void
smartlink_linear_mapping_hook(void *demapper, int16_t first, int16_t second)
{
    int16_t record[2];

    if (!record_file) {
        record_file = fopen("/tmp/smartlink-linear-mapping.s16", "wb");
        snapshot_demapper(demapper);
    }
    if (record_file) {
        record[0] = first;
        record[1] = second;
        (void)fwrite(record, sizeof(record[0]), 2, record_file);
        record_count++;
        if ((record_count & 31U) == 0)
            (void)fflush(record_file);
    }
    smartlink_linear_mapping_original(demapper, first, second);
}
