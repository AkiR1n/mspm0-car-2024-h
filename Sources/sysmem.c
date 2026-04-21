#include <errno.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "ti_msp_dl_config.h"

/* FreeRTOS 任务使用 pvPortMalloc（heap_4），不走 newlib 堆。
 * newlib 的 printf/malloc 若被非任务上下文调用会走 _sbrk，这里给个
 * 保守实现：指向链接脚本的 _end..__StackTop 之间的区域。 */

extern uint8_t _end;          /* 由 device_linker.lds 提供 */
extern uint8_t __StackTop;    /* 由 device_linker.lds 提供 */

static uint8_t *heap_end = NULL;

void *_sbrk(ptrdiff_t incr)
{
    if (heap_end == NULL) {
        heap_end = &_end;
    }

    uint8_t *prev = heap_end;
    if (heap_end + incr > &__StackTop) {
        errno = ENOMEM;
        return (void *)-1;
    }
    heap_end += incr;
    return prev;
}

int _close(int file)
{
    (void)file;
    return -1;
}

int _fstat(int file, struct stat *st)
{
    (void)file;
    if (st != NULL) {
        st->st_mode = S_IFCHR;
    }
    return 0;
}

int _isatty(int file)
{
    (void)file;
    return 1;
}

off_t _lseek(int file, off_t ptr, int dir)
{
    (void)file;
    (void)ptr;
    (void)dir;
    return 0;
}

ssize_t _read(int file, void *ptr, size_t len)
{
    (void)file;
    (void)ptr;
    (void)len;
    errno = ENOSYS;
    return -1;
}

ssize_t _write(int file, const void *ptr, size_t len)
{
    const uint8_t *buf = (const uint8_t *)ptr;

    (void)file;

    if (buf == NULL) {
        errno = EINVAL;
        return -1;
    }

    for (size_t i = 0; i < len; ++i) {
        DL_UART_transmitData(UART0_INST, buf[i]);
        while (!DL_UART_isTXFIFOEmpty(UART0_INST)) {
        }
    }

    return (ssize_t)len;
}
