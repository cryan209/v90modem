/* Process-isolation infrastructure tests, not modem/hardware qualification.
 * Include the supervisor to exercise its actual static environment/drain code.
 * No DSP rules are changed; ITU V.90 8.5.2 bearer accounting is untouched. */
#include <stdio.h>
#include <unistd.h>
static ssize_t controlled_read(int fd, void *buf, size_t size);
#define read controlled_read
#include "line_supervisor.c"
#undef read
#include "test_tmp.h"

static int mock_reads, mock_limit, mocking;
static ssize_t controlled_read(int fd, void *buf, size_t size)
{
    if (!mocking)
        return read(fd, buf, size);
    if (++mock_reads > mock_limit) {
        errno = EAGAIN;
        return -1;
    }
    memset(buf, 'x', size);
    return (ssize_t)size;
}

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL: line %d: %s\n", __LINE__, #condition); return 1; \
} } while (0)

static int environment_case(int which)
{
    char root[512], expected[1024], value[4096];
    struct stat st;

    unsetenv("ME_DUMP_DIR");
    unsetenv("VPCM_G711_TAP_DIR");
    unsetenv("ME_IO_SCHEDULE");
    unsetenv("DS_TX_BIT_DUMP");
    snprintf(root, sizeof(root), "%s", test_tmp("supervisor-env"));
    if (which == 0) {
        snprintf(value, sizeof(value), "%s/template.%%L", root);
        setenv("ME_DUMP_DIR", value, 1);
        CHECK(per_line_environment(3) == 0);
        snprintf(expected, sizeof(expected), "%s/template.3", root);
        CHECK(!strcmp(getenv("ME_DUMP_DIR"), expected));
        CHECK(stat(expected, &st) == 0 && S_ISDIR(st.st_mode));
    } else if (which == 1) {
        snprintf(value, sizeof(value), "%s/shared/nested", root);
        setenv("ME_DUMP_DIR", value, 1);
        setenv("VPCM_G711_TAP_DIR", value, 1);
        CHECK(per_line_environment(3) == 0);
        snprintf(expected, sizeof(expected), "%s/shared/nested/line3", root);
        CHECK(!strcmp(getenv("ME_DUMP_DIR"), expected));
        CHECK(!strcmp(getenv("VPCM_G711_TAP_DIR"), expected));
        CHECK(stat(expected, &st) == 0 && S_ISDIR(st.st_mode));
    } else if (which == 2) {
        CHECK(per_line_environment(3) == 0);
        CHECK(getenv("ME_DUMP_DIR") && *getenv("ME_DUMP_DIR"));
        CHECK(stat(getenv("ME_DUMP_DIR"), &st) == 0 && S_ISDIR(st.st_mode));
        CHECK(getenv("VPCM_G711_TAP_DIR") == NULL);
    } else if (which == 3) {
        setenv("ME_IO_SCHEDULE", "/not-created/schedule", 1);
        setenv("DS_TX_BIT_DUMP", "/not-created/tx.%L", 1);
        CHECK(per_line_environment(3) == 0);
        CHECK(!strcmp(getenv("ME_IO_SCHEDULE"), "/not-created/schedule.line3"));
        CHECK(!strcmp(getenv("DS_TX_BIT_DUMP"), "/not-created/tx.3"));
    } else if (which == 4) {
        memset(value, 'a', 2048);
        strcpy(value + 2048, "%L/end/%L");
        setenv("TEST_MULTILINE_VALUE", value, 1);
        CHECK(per_line_environment(3) == 0);
        CHECK(strlen(getenv("TEST_MULTILINE_VALUE")) == 2055);
        CHECK(!strcmp(getenv("TEST_MULTILINE_VALUE") + 2048, "3/end/3"));
    } else {
        FILE *file = fopen(test_tmp("supervisor-not-a-directory"), "w");

        CHECK(file != NULL);
        fclose(file);
        setenv("ME_DUMP_DIR", test_tmp("supervisor-not-a-directory"), 1);
        CHECK(per_line_environment(3) == -1);
    }
    return 0;
}

static int check_environment(int which)
{
    pid_t pid = fork();
    int status;

    CHECK(pid >= 0);
    if (!pid)
        _exit(environment_case(which));
    CHECK(waitpid(pid, &status, 0) == pid);
    CHECK(WIFEXITED(status) && WEXITSTATUS(status) == 0);
    printf("PASS: isolated environment case %d\n", which);
    return 0;
}

static int check_drain(void)
{
    worker_t wk = {0};
    FILE *out = fopen(test_tmp("supervisor-drain.log"), "w");
    int saved, calls;

    CHECK(out != NULL);
    fflush(stdout);
    saved = dup(STDOUT_FILENO);
    CHECK(saved >= 0);
    CHECK(dup2(fileno(out), STDOUT_FILENO) >= 0);
    mocking = 1;
    mock_reads = 0;
    mock_limit = 128;  /* Model a producer that never runs out of output. */
    wk.fd = 7;
    drain(&wk, 0, false);
    calls = mock_reads;
    mock_reads = 0;
    mock_limit = 2;
    drain(&wk, 0, true);
    mocking = 0;
    fflush(stdout);
    CHECK(dup2(saved, STDOUT_FILENO) >= 0);
    close(saved);
    fclose(out);
    CHECK(calls <= 32);
    CHECK(mock_reads == 3);
    printf("PASS: busy worker drain yields after %d reads; exit drain reaches EOF\n", calls);
    return 0;
}

static int check_pipe(void)
{
    int fds[2];

    CHECK(make_worker_pipe(fds) == 0);
    CHECK(fcntl(fds[0], F_GETFD) & FD_CLOEXEC);
    CHECK(fcntl(fds[1], F_GETFD) & FD_CLOEXEC);
    CHECK(fcntl(fds[0], F_GETFL) & O_NONBLOCK);
    CHECK(!(fcntl(fds[1], F_GETFL) & O_NONBLOCK));
    close(fds[0]);
    close(fds[1]);
    printf("PASS: worker pipe closes on exec and only supervisor reads are nonblocking\n");
    return 0;
}

static int check_ports(void)
{
    char *normal[] = {"modem"};
    char *large[] = {"modem", "--rtp-port", "10000"};
    char *overlap[] = {"modem", "--local-port", "4000", "--rtp-port", "4000"};
    char *overflow[] = {"modem", "--rtp-port", "65500"};
    char *bad[] = {"modem", "--local-port", "garbage"};
    char *missing[] = {"modem", "--rtp-port"};

    CHECK(valid_port_layout(1, normal, 8));
    CHECK(valid_port_layout(1, normal, 40));
    CHECK(!valid_port_layout(1, normal, 64));
    CHECK(valid_port_layout(3, large, 64));
    CHECK(!valid_port_layout(5, overlap, 2));
    CHECK(!valid_port_layout(3, overflow, 2));
    CHECK(!valid_port_layout(3, bad, 2));
    CHECK(!valid_port_layout(2, missing, 2));
    printf("PASS: reject overlapping, overflowing and malformed port allocations\n");
    return 0;
}

int main(int argc, char **argv)
{
    int failed = 0;

    /* A SIP-free executable fixture for restart/shutdown isolation tests. */
    if (argc > 1 && !strcmp(argv[1], "--fixture")) {
        const char *line = opt_value(argc, argv, "--line-index");
        bool recycle;

        if (line) {
            printf("fixture ready line=%s\n", line);
            fflush(stdout);
            for (;;)
                pause();
        }
        return line_supervisor_run(argc, argv,
                                   line_supervisor_parse(argc, argv, &recycle), false);
    }

    for (int i = 0; i < 6; i++)
        failed += check_environment(i);
    failed += check_drain();
    failed += check_pipe();
    failed += check_ports();
    return failed ? 1 : 0;
}
