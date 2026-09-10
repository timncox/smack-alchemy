#include "versio_alloc.h"
#include <string.h>

static unsigned char *g_pool;
static size_t         g_cap;
static size_t         g_used;
static int            g_failed;

/* Cortex-M7 wants 8-byte alignment for doubles; the engine stores int16 and
 * float, but align generously -- misaligned SDRAM access is slow at best. */
#define ALIGN 8

void versio_alloc_init(void *pool, size_t bytes)
{
    g_pool   = (unsigned char *)pool;
    g_cap    = bytes;
    g_used   = 0;
    g_failed = 0;
}

void *versio_calloc(size_t nmemb, size_t size)
{
    size_t want = nmemb * size;
    size_t base;

    if (nmemb != 0 && want / nmemb != size) { /* overflow */
        g_failed = 1;
        return 0;
    }

    base = (g_used + (ALIGN - 1)) & ~(size_t)(ALIGN - 1);
    if (g_pool == 0 || base + want > g_cap) {
        g_failed = 1;
        return 0;
    }

    g_used = base + want;
    memset(g_pool + base, 0, want); /* calloc contract: zeroed */
    return g_pool + base;
}

/* No-op by design -- see the header. The engine is created once and lives
 * until power-off, so reclaiming would only add a failure mode. */
void versio_free(void *ptr) { (void)ptr; }

size_t versio_alloc_used(void)     { return g_used; }
size_t versio_alloc_capacity(void) { return g_cap; }
int    versio_alloc_failed(void)   { return g_failed; }
