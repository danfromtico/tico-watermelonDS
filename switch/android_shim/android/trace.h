// Switch stand-in for the NDK header: systrace sections are no-ops.
#pragma once

#include <stdbool.h>
#include <stdint.h>

static inline void ATrace_beginSection(const char* name) { (void)name; }
static inline void ATrace_endSection(void) {}
static inline bool ATrace_isEnabled(void) { return false; }
static inline void ATrace_setCounter(const char* name, int64_t value) { (void)name; (void)value; }
static inline void ATrace_beginAsyncSection(const char* name, int32_t cookie) { (void)name; (void)cookie; }
static inline void ATrace_endAsyncSection(const char* name, int32_t cookie) { (void)name; (void)cookie; }
