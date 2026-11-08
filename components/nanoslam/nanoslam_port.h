/*
 * Platform shims so the NanoSLAM sources build both in ESP-IDF and on a host
 * for unit tests. Only logging, yielding, large allocations, a mutex and a
 * microsecond clock differ between the two.
 */

#ifndef NANOSLAM_PORT_H
#define NANOSLAM_PORT_H

#include <stdint.h>
#include <stdlib.h>

#ifdef ESP_PLATFORM

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define NS_LOGI(tag, ...) ESP_LOGI(tag, __VA_ARGS__)
#define NS_LOGW(tag, ...) ESP_LOGW(tag, __VA_ARGS__)
#define NS_LOGD(tag, ...) ESP_LOGD(tag, __VA_ARGS__)
/* Long loops (ICP, raycasting, export) yield so lower-priority tasks and the
 * idle-task watchdog keep running. */
#define NS_YIELD() vTaskDelay(pdMS_TO_TICKS(1))
#define NS_NOW_US() ((uint64_t)esp_timer_get_time())

static inline void *ns_large_calloc(size_t n, size_t size) {
    void *p = heap_caps_calloc(n, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    return p != NULL ? p : heap_caps_calloc(n, size, MALLOC_CAP_8BIT);
}
static inline void ns_large_free(void *p) { heap_caps_free(p); }

typedef SemaphoreHandle_t ns_mutex_t;
static inline ns_mutex_t ns_mutex_create(void) { return xSemaphoreCreateMutex(); }
static inline void ns_mutex_lock(ns_mutex_t m) { if (m) xSemaphoreTake(m, portMAX_DELAY); }
static inline void ns_mutex_unlock(ns_mutex_t m) { if (m) xSemaphoreGive(m); }

#else /* host build */

#include <stdio.h>
#include <time.h>

#define NS_LOGI(tag, fmt, ...) printf("I %s: " fmt "\n", tag, ##__VA_ARGS__)
#define NS_LOGW(tag, fmt, ...) printf("W %s: " fmt "\n", tag, ##__VA_ARGS__)
#ifdef NANOSLAM_HOST_DEBUG
#define NS_LOGD(tag, fmt, ...) printf("D %s: " fmt "\n", tag, ##__VA_ARGS__)
#else
#define NS_LOGD(tag, fmt, ...) ((void)0)
#endif
#define NS_YIELD() ((void)0)

static inline uint64_t ns_host_now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}
#define NS_NOW_US() ns_host_now_us()

static inline void *ns_large_calloc(size_t n, size_t size) { return calloc(n, size); }
static inline void ns_large_free(void *p) { free(p); }

typedef int ns_mutex_t;
static inline ns_mutex_t ns_mutex_create(void) { return 1; }
static inline void ns_mutex_lock(ns_mutex_t m) { (void)m; }
static inline void ns_mutex_unlock(ns_mutex_t m) { (void)m; }

#endif

#endif /* NANOSLAM_PORT_H */
