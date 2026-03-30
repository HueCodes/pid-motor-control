#include <sys/stat.h>
#include <errno.h>

int _close(int fd)              { (void)fd; return -1; }
int _fstat(int fd, struct stat *st) { (void)fd; st->st_mode = S_IFCHR; return 0; }
int _isatty(int fd)             { (void)fd; return 1; }
int _lseek(int fd, int off, int w) { (void)fd; (void)off; (void)w; return 0; }
int _read(int fd, char *p, int len) { (void)fd; (void)p; (void)len; return 0; }
int _write(int fd, char *p, int len) { (void)fd; (void)p; (void)len; return len; }
void *_sbrk(int incr) {
    extern char end;
    static char *heap = 0;
    if (!heap) heap = &end;
    char *prev = heap;
    heap += incr;
    return prev;
}
void _exit(int status) { (void)status; while (1); }
int _kill(int pid, int sig) { (void)pid; (void)sig; errno = EINVAL; return -1; }
int _getpid(void) { return 1; }
