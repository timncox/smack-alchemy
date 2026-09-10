/*
 * Proves the engine runs on the SDRAM bump allocator instead of the heap,
 * natively, before any hardware is involved.
 *
 * This links smack_core_versio.c -- the same translation unit the firmware
 * builds -- so it exercises the real -Dcalloc redirect, not a simulation of
 * it. If this passes, the "Daisy has no heap" problem is solved with zero
 * engine edits.
 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../versio_alloc.h"
#include "../vendor/smack_core.h"

#define BLK 128
/* The firmware's pool, not a copy of it — versio_alloc.h owns the number so
 * a ring-size change cannot pass here and fail on hardware. */
#define POOL_BYTES VERSIO_POOL_BYTES

/*
 * versio_alloc.h cannot derive the pool from the ring itself (it would bind
 * the engine with C++ linkage in the firmware — see the note there), so the
 * relationship is checked here, at compile time, where both headers are C.
 * The ring is SMACK_RING_FRAMES * 2 channels * 2 bytes; the rest of the
 * engine is under 1 MB, so a megabyte of slack is a fair margin.
 */
typedef char pool_holds_the_ring[
    (VERSIO_POOL_BYTES >= (size_t)SMACK_RING_FRAMES * 4 + (1u << 20)) ? 1 : -1];

static float fake_bpm(void) { return 120.0f; }

static void make_host(host_api_v1_t *h)
{
    memset(h, 0, sizeof(*h));
    h->api_version      = 1;
    h->sample_rate      = SMACK_SR;
    h->frames_per_block = BLK;
    h->get_bpm          = fake_bpm;
}

static void test_engine_fits_the_pool(void)
{
    host_api_v1_t host;
    smack_t      *s;
    int16_t       in[BLK * 2], out[BLK * 2];
    void         *pool = malloc(POOL_BYTES);
    size_t        used;
    int           i;

    assert(pool);
    versio_alloc_init(pool, POOL_BYTES);
    make_host(&host);

    s = smack_create(&host);
    assert(s);
    assert(!versio_alloc_failed());

    used = versio_alloc_used();
    /* The ring alone is SMACK_RING_FRAMES * 2ch * 2 bytes. */
    assert(used > (size_t)SMACK_RING_FRAMES * 4);
    assert(used < POOL_BYTES);

    for (i = 0; i < BLK * 2; i++) in[i] = (int16_t)((i % 64) * 400 - 12800);
    for (i = 0; i < 50; i++) smack_process(s, in, out, BLK);

    printf("ok: engine live on bump allocator, %.2f MB of %u MB used\n",
           (double)used / (1024.0 * 1024.0), POOL_BYTES / (1024u * 1024u));

    smack_destroy(s); /* free() is a no-op; must not crash */
    free(pool);
}

static void test_too_small_a_pool_fails_loudly(void)
{
    host_api_v1_t host;
    smack_t      *s;
    void         *pool = malloc(64 * 1024);

    assert(pool);
    versio_alloc_init(pool, 64 * 1024); /* nowhere near enough for the ring */
    make_host(&host);

    s = smack_create(&host);
    /* The engine's own null-checks unwind and return NULL; what matters is
     * that we can SEE it, so the firmware can refuse to boot rather than run
     * half-initialised and silent. */
    assert(s == NULL);
    assert(versio_alloc_failed());

    printf("ok: undersized pool fails visibly (create=NULL, failed flag set)\n");
    free(pool);
}

int main(void)
{
    /* Order matters: the failure test poisons the global failed flag. */
    test_engine_fits_the_pool();
    test_too_small_a_pool_fails_loudly();
    printf("versio_alloc: all assertions passed\n");
    return 0;
}
