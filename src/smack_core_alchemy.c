/*
 * Compiles the vendored engine with its two allocation calls redirected to
 * the SDRAM bump allocator (versio_alloc.c, carried over from smack-versio
 * unchanged — the name is the file's provenance, not the platform).
 *
 * Doing it here rather than as a global -D keeps the redirect scoped to this
 * translation unit -- libDaisy, the Alchemy SDK and everything else keep the
 * real calloc/free. stdlib.h is included FIRST so the macros rewrite call
 * sites only, never the library's own declarations.
 *
 * Build this file; do NOT add vendor/smack_core.c to the source list too, or
 * you get duplicate symbols.
 */
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "versio_alloc.h"

#define calloc versio_calloc
#define free   versio_free

#include "vendor/smack_core.c"
