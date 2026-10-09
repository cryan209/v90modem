/*
 * test_tmp.h - scratch file names for the test binaries.
 *
 * test_tmp("name") returns "$TMPDIR/name" (or "/tmp/name" when TMPDIR is unset),
 * the same string on every call for the same name.  tools/run_tests.py gives
 * each test its own TMPDIR so the suite can run in parallel; a test that wrote
 * to a fixed /tmp name would collide with another instance of itself.
 *
 * Header-only and static: each test binary gets its own small table.
 */
#ifndef V90MODEM_TEST_TMP_H
#define V90MODEM_TEST_TMP_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static inline const char *test_tmp(const char *name)
{
    static struct { const char *name; char path[512]; } table[64];
    static int used;
    const char *dir;
    size_t len;

    for (int i = 0; i < used; i++)
        if (strcmp(table[i].name, name) == 0)
            return table[i].path;
    if (used == (int) (sizeof(table)/sizeof(table[0])))
    {
        fprintf(stderr, "test_tmp: too many names\n");
        abort();
    }
    dir = getenv("TMPDIR");
    if (dir == NULL  ||  dir[0] == '\0')
        dir = "/tmp";
    len = strlen(dir);
    snprintf(table[used].path, sizeof(table[used].path), "%s%s%s",
             dir, (len > 0  &&  dir[len - 1] == '/') ? "" : "/", name);
    table[used].name = name;
    return table[used++].path;
}

#endif
