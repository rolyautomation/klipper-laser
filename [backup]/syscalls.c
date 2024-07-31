#include <sys/stat.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>
#include <sys/reent.h>

void _exit(int status) {
    while(1) {}
}

int *__errno(void) {
    static int error = 0;
    return &error;
}

int _kill(int pid, int sig) {
    *__errno() = EINVAL;
    return -1;
}

int _getpid(void) {
    return 1;
}

int _write(int file, char *ptr, int len) {
    return len;
}

int _kill_r(struct _reent *r, int pid, int sig) {
    r->_errno = EINVAL;
    return -1;
}

int _getpid_r(struct _reent *r) {
    return 1;
}

int _write_r(struct _reent *r, int file, char *ptr, int len) {
    return len;
}