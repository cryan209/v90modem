/* Independent golden vectors execute Courier firmware c749..c788.
 * A passing same-implementation loopback would not establish this mapping. */
#include "vfc_mapper.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(int argc, char **argv)
{
    assert(argc <= 2);
    FILE *f = fopen(argc == 2 ? argv[1] : "reference-data/vfc/mapper.csv", "r");
    assert(f);
    char line[256];
    unsigned rows = 0;
    while (fgets(line, sizeof(line), f)) {
        if (line[0] == '#') continue;
        unsigned code, state, quadrant, index, dibit, next_state, next_quadrant;
        int i, q;
        assert(sscanf(line, "%u,%u,%u,%u,%u,%u,%u,%d,%d", &code, &state,
                      &quadrant, &index, &dibit, &next_state, &next_quadrant,
                      &i, &q) == 9);
        vfc_mapper_t s;
        vfc_point_t point;
        assert(vfc_mapper_init(&s, code));
        s.trellis = state;
        s.quadrant = quadrant;
        assert(vfc_mapper_put(&s, index, dibit, &point));
        if (s.trellis != next_state || s.quadrant != next_quadrant
            || point.i != i || point.q != q) {
            fprintf(stderr, "V.FC firmware mismatch at vector %u: %s", rows, line);
            return 1;
        }
        rows++;
    }
    assert(!ferror(f));
    fclose(f);
    assert(rows == 3584);
    vfc_mapper_t s = {32, 31, 3}, before = s;
    vfc_point_t point = {123, -456}, saved = point;
    assert(!vfc_mapper_init(&s, 16));
    assert(memcmp(&s, &before, sizeof(s)) == 0);
    assert(!vfc_mapper_put(&s, 160, 0, &point));
    assert(!vfc_mapper_put(&s, 0, 4, &point));
    assert(memcmp(&s, &before, sizeof(s)) == 0);
    assert(memcmp(&point, &saved, sizeof(point)) == 0);
    assert(vfc_mapper_init(&s, 8));
    assert(s.trellis == 0 && s.quadrant == 0 && s.code == 8);
    printf("PASS: V.FC mapper matches %u original firmware vectors\n", rows);
    return 0;
}
