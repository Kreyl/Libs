#include <sys/stat.h>
//#include <stdlib.h>
#include <errno.h>
//#include <stdio.h>
//#include <signal.h>
//#include <time.h>
//#include <sys/time.h>
//#include <sys/times.h>


//extern int __io_putchar(int ch) __attribute__((weak));
//extern int __io_getchar(void) __attribute__((weak));
//char *__env[1] = { 0 };
//char **environ = __env;

//void initialise_monitor_handles() { }

caddr_t _sbrk(int incr) {
    extern uint8_t __heap_base__; // }
    extern uint8_t __heap_end__;  // } defined in linker script

    static uint8_t *current_end = &__heap_base__;
    uint8_t *current_block_address = current_end;

    /* Need to align heap to word boundary, else will get hard faults
    on Cortex-M0. So we assume that heap starts on word boundary,
    hence make sure we always add a multiple of 4 to it. */
    incr = (incr + 3) & (~3);
    if(current_end + incr > &__heap_end__) {
        errno = ENOMEM;
        return (caddr_t) -1;
    }
    current_end += incr;
    return (caddr_t)current_block_address;
}

int _getpid(void)
{
  return 1;
}

int _kill(int pid, int sig)
{
  (void)pid;
  (void)sig;
  errno = EINVAL;
  return -1;
}

void _exit (int status)
{
  _kill(status, -1);
  while (1) {}    /* Make sure we hang here */
}

__attribute__((weak)) int _read(int file, char *ptr, int len)
{
  (void)file;
  return 0;
}

__attribute__((weak)) int _write(int file, char *ptr, int len)
{
  (void)file;
  return len;
}

int _close(int file)
{
  (void)file;
  return -1;
}


int _fstat(int file, struct stat *st)
{
  (void)file;
  st->st_mode = S_IFCHR;
  return 0;
}

int _isatty(int file)
{
  (void)file;
  return 1;
}

int _lseek(int file, int ptr, int dir)
{
  (void)file;
  (void)ptr;
  (void)dir;
  return 0;
}

//int _open(char *path, int flags, ...)
//{
//  (void)path;
//  (void)flags;
//  /* Pretend like we always fail */
//  return -1;
//}
//
//int _wait(int *status)
//{
//  (void)status;
//  errno = ECHILD;
//  return -1;
//}

//int _unlink(char *name)
//{
//  (void)name;
//  errno = ENOENT;
//  return -1;
//}
//
//int _times(struct tms *buf)
//{
//  (void)buf;
//  return -1;
//}

//int _stat(char *file, struct stat *st)
//{
//  (void)file;
//  st->st_mode = S_IFCHR;
//  return 0;
//}
//
//int _link(char *old, char *new)
//{
//  (void)old;
//  (void)new;
//  errno = EMLINK;
//  return -1;
//}

//int _fork(void)
//{
//  errno = EAGAIN;
//  return -1;
//}

//int _execve(char *name, char **argv, char **env)
//{
//  (void)name;
//  (void)argv;
//  (void)env;
//  errno = ENOMEM;
//  return -1;
//}
