#ifndef CORE_PROFILE_CATALOG_H
#define CORE_PROFILE_CATALOG_H

#include <stdint.h>

#include "core/profile_engine.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PROFILE_CATALOG_MAX_PROFILES 8

typedef struct {
    reflow_profile_t profiles[PROFILE_CATALOG_MAX_PROFILES];
    uint32_t profile_count;
} reflow_profile_catalog_t;

/**
 * Parses a constrained YAML document containing `profiles:` with one or more
 * reflow profiles and their phases. This parser is intentionally limited to the
 * schema documented in `firmware/profiles/reflow-profiles.yaml`, so it can run
 * on-device without pulling in a full YAML library.
 *
 * Returns 0 on success, -1 on malformed input or validation failure.
 */
int profile_catalog_parse_yaml(const char *yaml, reflow_profile_catalog_t *out_catalog);

/** Returns the profile at index, or NULL if the index is out of range. */
const reflow_profile_t *profile_catalog_get(const reflow_profile_catalog_t *catalog, uint32_t index);

#ifdef __cplusplus
}
#endif

#endif /* CORE_PROFILE_CATALOG_H */
