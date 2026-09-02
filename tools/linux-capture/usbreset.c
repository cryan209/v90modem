#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#define USBDEVFS_RESET _IO('U', 20)
int main(int argc, char **argv) {
    int fd = open(argv[1], O_WRONLY);
    if (fd < 0) { perror("open"); return 1; }
    if (ioctl(fd, USBDEVFS_RESET, 0) < 0) { perror("ioctl"); return 1; }
    printf("reset ok\n");
    return 0;
}
