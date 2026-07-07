#include "web_api.h"

#include <string.h>

#include "cJSON.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "web_api";

static web_api_status_t s_status;
static SemaphoreHandle_t s_status_mutex;
static httpd_handle_t s_server = NULL;

static const char *profile_state_name(profile_engine_state_t state)
{
    switch (state) {
    case PROFILE_ENGINE_STATE_IDLE:
        return "idle";
    case PROFILE_ENGINE_STATE_RUNNING:
        return "running";
    case PROFILE_ENGINE_STATE_COMPLETE:
        return "complete";
    case PROFILE_ENGINE_STATE_FAULT:
        return "fault";
    default:
        return "unknown";
    }
}

void web_api_publish_status(const web_api_status_t *status)
{
    if (status == NULL || s_status_mutex == NULL) {
        return;
    }
    /* Bounded wait: the control task must never block indefinitely on the
     * web task, so a short timeout is used instead of portMAX_DELAY. */
    if (xSemaphoreTake(s_status_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        s_status = *status;
        xSemaphoreGive(s_status_mutex);
    }
}

static esp_err_t send_json_response(httpd_req_t *req, cJSON *root)
{
    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (json_str == NULL) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    esp_err_t err = httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);
    cJSON_free(json_str);
    return err;
}

static esp_err_t handle_get_status(httpd_req_t *req)
{
    web_api_status_t snapshot;
    if (xSemaphoreTake(s_status_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        snapshot = s_status;
        xSemaphoreGive(s_status_mutex);
    } else {
        memset(&snapshot, 0, sizeof(snapshot));
    }

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "chamber_temperature_c", snapshot.chamber_temperature_c);
    cJSON_AddNumberToObject(root, "plate_temperature_c", snapshot.plate_temperature_c);
    cJSON_AddNumberToObject(root, "top_heater_power_percent", snapshot.top_heater_power_percent);
    cJSON_AddNumberToObject(root, "bottom_heater_power_percent", snapshot.bottom_heater_power_percent);
    cJSON_AddStringToObject(root, "profile_state", profile_state_name(snapshot.profile_state));
    cJSON_AddNumberToObject(root, "current_phase_index", snapshot.current_phase_index);
    cJSON_AddNumberToObject(root, "safety_fault_flags", snapshot.safety_fault_flags);

    return send_json_response(req, root);
}

/* Endpoints below are placeholders for later milestones (profile storage,
 * run control, configuration persistence, telemetry history) per
 * docs/firmware-plan.md "Initial Firmware Milestones". They return 501 so
 * clients get a clear, well-formed response instead of a 404 while those
 * milestones are implemented incrementally. */
static esp_err_t handle_not_implemented(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "error", "not_implemented");
    httpd_resp_set_status(req, "501 Not Implemented");
    return send_json_response(req, root);
}

static const httpd_uri_t s_uri_status = {
    .uri = "/api/status",
    .method = HTTP_GET,
    .handler = handle_get_status,
};

static const httpd_uri_t s_uri_profile_get = {
    .uri = "/api/profile",
    .method = HTTP_GET,
    .handler = handle_not_implemented,
};

static const httpd_uri_t s_uri_profiles_get = {
    .uri = "/api/profiles",
    .method = HTTP_GET,
    .handler = handle_not_implemented,
};

static const httpd_uri_t s_uri_profile_select = {
    .uri = "/api/profile/select",
    .method = HTTP_POST,
    .handler = handle_not_implemented,
};

static const httpd_uri_t s_uri_run_start = {
    .uri = "/api/run/start",
    .method = HTTP_POST,
    .handler = handle_not_implemented,
};

static const httpd_uri_t s_uri_run_stop = {
    .uri = "/api/run/stop",
    .method = HTTP_POST,
    .handler = handle_not_implemented,
};

static const httpd_uri_t s_uri_config_get = {
    .uri = "/api/config",
    .method = HTTP_GET,
    .handler = handle_not_implemented,
};

static const httpd_uri_t s_uri_config_post = {
    .uri = "/api/config",
    .method = HTTP_POST,
    .handler = handle_not_implemented,
};

static const httpd_uri_t s_uri_log_recent = {
    .uri = "/api/log/recent",
    .method = HTTP_GET,
    .handler = handle_not_implemented,
};

esp_err_t web_api_start(void)
{
    if (s_status_mutex == NULL) {
        s_status_mutex = xSemaphoreCreateMutex();
        if (s_status_mutex == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    memset(&s_status, 0, sizeof(s_status));

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    /* Keep the web task at low priority relative to safety/control tasks,
     * per docs/firmware-plan.md FreeRTOS Task Model. */
    config.task_priority = tskIDLE_PRIORITY + 2;

    esp_err_t err = httpd_start(&s_server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(err));
        return err;
    }

    httpd_register_uri_handler(s_server, &s_uri_status);
    httpd_register_uri_handler(s_server, &s_uri_profile_get);
    httpd_register_uri_handler(s_server, &s_uri_profiles_get);
    httpd_register_uri_handler(s_server, &s_uri_profile_select);
    httpd_register_uri_handler(s_server, &s_uri_run_start);
    httpd_register_uri_handler(s_server, &s_uri_run_stop);
    httpd_register_uri_handler(s_server, &s_uri_config_get);
    httpd_register_uri_handler(s_server, &s_uri_config_post);
    httpd_register_uri_handler(s_server, &s_uri_log_recent);

    return ESP_OK;
}
