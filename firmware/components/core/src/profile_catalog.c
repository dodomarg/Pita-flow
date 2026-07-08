#include "core/profile_catalog.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    int duration_s;
    int target_chamber_c;
    int ramp_rate_c_per_s;
    int bottom_heater_mode;
    int bottom_plate_limit_c;
    int top_heater_enabled;
} phase_field_mask_t;

static void trim_trailing(char *text)
{
    size_t len = strlen(text);
    while (len > 0) {
        char c = text[len - 1];
        if (c == '\r' || c == '\n' || isspace((unsigned char)c)) {
            text[--len] = '\0';
        } else {
            break;
        }
    }
}

static char *trim_leading(char *text)
{
    while (*text != '\0' && isspace((unsigned char)*text)) {
        text++;
    }
    return text;
}

static int leading_space_count(const char *text)
{
    int count = 0;
    while (text[count] == ' ') {
        count++;
    }
    return count;
}

static void strip_comment(char *text)
{
    for (size_t i = 0; text[i] != '\0'; ++i) {
        if (text[i] == '#') {
            text[i] = '\0';
            break;
        }
    }
}

static char *value_after_colon(char *text)
{
    char *colon = strchr(text, ':');
    if (colon == NULL) {
        return NULL;
    }
    *colon = '\0';
    return trim_leading(colon + 1);
}

static void strip_optional_quotes(char *text)
{
    size_t len = strlen(text);
    if (len >= 2 && ((text[0] == '"' && text[len - 1] == '"') ||
                     (text[0] == '\'' && text[len - 1] == '\''))) {
        memmove(text, text + 1, len - 2);
        text[len - 2] = '\0';
    }
}

static int parse_uint32_value(const char *text, uint32_t *out_value)
{
    char *end = NULL;
    unsigned long value = strtoul(text, &end, 10);
    if (text == end || end == NULL) {
        return -1;
    }
    while (*end != '\0') {
        if (!isspace((unsigned char)*end)) {
            return -1;
        }
        end++;
    }
    *out_value = (uint32_t)value;
    return 0;
}

static int parse_float_value(const char *text, float *out_value)
{
    char *end = NULL;
    float value = strtof(text, &end);
    if (text == end || end == NULL) {
        return -1;
    }
    while (*end != '\0') {
        if (!isspace((unsigned char)*end)) {
            return -1;
        }
        end++;
    }
    *out_value = value;
    return 0;
}

static int parse_bool_value(const char *text, int *out_value)
{
    if (strcmp(text, "true") == 0) {
        *out_value = 1;
        return 0;
    }
    if (strcmp(text, "false") == 0) {
        *out_value = 0;
        return 0;
    }
    return -1;
}

static int parse_bottom_heater_mode(const char *text, bottom_heater_mode_t *out_mode)
{
    if (strcmp(text, "off") == 0) {
        *out_mode = BOTTOM_HEATER_MODE_OFF;
        return 0;
    }
    if (strcmp(text, "limited") == 0) {
        *out_mode = BOTTOM_HEATER_MODE_LIMITED;
        return 0;
    }
    if (strcmp(text, "enabled") == 0) {
        *out_mode = BOTTOM_HEATER_MODE_ENABLED;
        return 0;
    }
    return -1;
}

static int phase_fields_complete(const phase_field_mask_t *mask)
{
    return mask->duration_s && mask->target_chamber_c && mask->ramp_rate_c_per_s &&
           mask->bottom_heater_mode && mask->bottom_plate_limit_c && mask->top_heater_enabled;
}

const reflow_profile_t *profile_catalog_get(const reflow_profile_catalog_t *catalog, uint32_t index)
{
    if (catalog == NULL || index >= catalog->profile_count) {
        return NULL;
    }
    return &catalog->profiles[index];
}

int profile_catalog_parse_yaml(const char *yaml, reflow_profile_catalog_t *out_catalog)
{
    if (yaml == NULL || out_catalog == NULL) {
        return -1;
    }

    memset(out_catalog, 0, sizeof(*out_catalog));

    size_t yaml_len = strlen(yaml);
    char *buffer = malloc(yaml_len + 1);
    if (buffer == NULL) {
        return -1;
    }
    memcpy(buffer, yaml, yaml_len + 1);

    int have_profiles_root = 0;
    reflow_profile_t *current_profile = NULL;
    profile_phase_t *current_phase = NULL;
    phase_field_mask_t phase_fields;
    memset(&phase_fields, 0, sizeof(phase_fields));

    for (char *line = strtok(buffer, "\n"); line != NULL; line = strtok(NULL, "\n")) {
        trim_trailing(line);
        strip_comment(line);
        trim_trailing(line);

        char *trimmed = trim_leading(line);
        if (*trimmed == '\0') {
            continue;
        }

        int indent = leading_space_count(line);

        if (indent == 0 && strcmp(trimmed, "profiles:") == 0) {
            have_profiles_root = 1;
            current_profile = NULL;
            current_phase = NULL;
            memset(&phase_fields, 0, sizeof(phase_fields));
            continue;
        }

        if (!have_profiles_root) {
            free(buffer);
            return -1;
        }

        if (indent == 2 && strncmp(trimmed, "- name:", 7) == 0) {
            if (current_phase != NULL && !phase_fields_complete(&phase_fields)) {
                free(buffer);
                return -1;
            }
            if (out_catalog->profile_count >= PROFILE_CATALOG_MAX_PROFILES) {
                free(buffer);
                return -1;
            }
            current_profile = &out_catalog->profiles[out_catalog->profile_count++];
            memset(current_profile, 0, sizeof(*current_profile));
            current_phase = NULL;
            memset(&phase_fields, 0, sizeof(phase_fields));

            char *value = trim_leading(trimmed + 7);
            strip_optional_quotes(value);
            strncpy(current_profile->name, value, sizeof(current_profile->name) - 1);
            continue;
        }

        if (indent == 4 && strcmp(trimmed, "phases:") == 0) {
            if (current_profile == NULL) {
                free(buffer);
                return -1;
            }
            continue;
        }

        if (indent == 6 && strncmp(trimmed, "- name:", 7) == 0) {
            if (current_profile == NULL || current_profile->phase_count >= PROFILE_ENGINE_MAX_PHASES) {
                free(buffer);
                return -1;
            }
            if (current_phase != NULL && !phase_fields_complete(&phase_fields)) {
                free(buffer);
                return -1;
            }

            current_phase = &current_profile->phases[current_profile->phase_count++];
            memset(current_phase, 0, sizeof(*current_phase));
            memset(&phase_fields, 0, sizeof(phase_fields));

            char *value = trim_leading(trimmed + 7);
            strip_optional_quotes(value);
            strncpy(current_phase->name, value, sizeof(current_phase->name) - 1);
            continue;
        }

        if (indent == 8) {
            if (current_phase == NULL) {
                free(buffer);
                return -1;
            }

            char *value = value_after_colon(trimmed);
            if (value == NULL) {
                free(buffer);
                return -1;
            }
            strip_optional_quotes(value);

            if (strcmp(trimmed, "duration_s") == 0) {
                if (parse_uint32_value(value, &current_phase->duration_s) != 0) {
                    free(buffer);
                    return -1;
                }
                phase_fields.duration_s = 1;
            } else if (strcmp(trimmed, "target_chamber_c") == 0) {
                if (parse_float_value(value, &current_phase->target_chamber_c) != 0) {
                    free(buffer);
                    return -1;
                }
                phase_fields.target_chamber_c = 1;
            } else if (strcmp(trimmed, "ramp_rate_c_per_s") == 0) {
                if (parse_float_value(value, &current_phase->ramp_rate_c_per_s) != 0) {
                    free(buffer);
                    return -1;
                }
                phase_fields.ramp_rate_c_per_s = 1;
            } else if (strcmp(trimmed, "bottom_heater_mode") == 0) {
                if (parse_bottom_heater_mode(value, &current_phase->bottom_heater_mode) != 0) {
                    free(buffer);
                    return -1;
                }
                phase_fields.bottom_heater_mode = 1;
            } else if (strcmp(trimmed, "bottom_plate_limit_c") == 0) {
                if (parse_float_value(value, &current_phase->bottom_plate_limit_c) != 0) {
                    free(buffer);
                    return -1;
                }
                phase_fields.bottom_plate_limit_c = 1;
            } else if (strcmp(trimmed, "top_heater_enabled") == 0) {
                if (parse_bool_value(value, &current_phase->top_heater_enabled) != 0) {
                    free(buffer);
                    return -1;
                }
                phase_fields.top_heater_enabled = 1;
            } else {
                free(buffer);
                return -1;
            }
            continue;
        }

        free(buffer);
        return -1;
    }

    free(buffer);

    if (current_phase != NULL && !phase_fields_complete(&phase_fields)) {
        return -1;
    }

    if (!have_profiles_root || out_catalog->profile_count == 0) {
        return -1;
    }

    for (uint32_t i = 0; i < out_catalog->profile_count; ++i) {
        if (!profile_engine_validate(&out_catalog->profiles[i])) {
            memset(out_catalog, 0, sizeof(*out_catalog));
            return -1;
        }
    }

    return 0;
}
