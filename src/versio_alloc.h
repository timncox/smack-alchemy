/*
 * versio_alloc — a bump allocator so the vendored engine needs no edits.
 *
 * smack_create() does four calloc()s (a 13.4 MB ring plus per-lane delay and
 * reverb lines) and smack_destroy() frees them. Daisy has no meaningful heap,
 * and rewriting smack_create() would fork the engine.
 *
 * Instead the ARM build compiles smack_core.c with
 *     -Dcalloc=versio_calloc -Dfree=versio_free
 * pointing those two calls at an SDRAM pool. Allocation happens once at boot
 * and is never returned, so free() is a no-op -- correct here, because the
 * module creates exactly one engine and keeps it until power-off.
 *
 * The pool itself is declared in smack_versio.cpp (it needs DSY_SDRAM_BSS
 * from libDaisy); this file stays plain C so it can be unit-tested natively.
 */
#ifndef VERSIO_ALLOC_H
#define VERSIO_ALLOC_H

#include <stddef.h>

/*
 * Pool size, defined here rather than in smack_versio.cpp so the firmware and
 * test_versio_alloc.c cannot disagree about it. They used to declare it
 * separately, and when the ring grew from 70 s to 150 s the firmware was
 * updated and the test was not -- it kept allocating 16 MB and failed with a
 * bare `assert(s)`.
 *
 * The ring dominates: SMACK_RING_FRAMES * 2 channels * 2 bytes, which is
 * 28.8 MB at 150 s. The lanes add ~160 KB and smack_t itself is small, so
 * rounding up to the next power of two covers everything with room to spare
 * and stays well inside the Versio's 64 MB of SDRAM.
 *
 * Deliberately NOT derived from SMACK_RING_FRAMES here: smack_core.h declares
 * no linkage of its own, and the firmware includes it inside an extern "C"
 * block. Pulling it in from this header -- which is included first -- would
 * bind the engine's symbols with C++ linkage and the include guard would then
 * skip the wrapped copy, so every smack_* call fails to link. The relationship
 * to the ring size is asserted in test_versio_alloc.c instead, where both
 * headers are C and can be compared safely.
 */
#define VERSIO_POOL_BYTES (32u * 1024u * 1024u)

#ifdef __cplusplus
extern "C" {
#endif

void   versio_alloc_init(void *pool, size_t bytes);
void  *versio_calloc(size_t nmemb, size_t size);
void   versio_free(void *ptr);

/* For the boot-time log: how much of the pool the engine actually took. */
size_t versio_alloc_used(void);
size_t versio_alloc_capacity(void);

/* Set when an allocation did not fit. If this is true after smack_create(),
 * the module is broken and must say so rather than run half-initialised. */
int    versio_alloc_failed(void);

#ifdef __cplusplus
}
#endif

#endif /* VERSIO_ALLOC_H */
