// Switch stand-in for the NDK header. An ANativeWindow* handed to the Vulkan
// presenter is really a libnx NWindow*; these helpers forward to it.
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ANativeWindow;
typedef struct ANativeWindow ANativeWindow;

void ANativeWindow_acquire(ANativeWindow* window);
void ANativeWindow_release(ANativeWindow* window);
int32_t ANativeWindow_getWidth(ANativeWindow* window);
int32_t ANativeWindow_getHeight(ANativeWindow* window);

#ifdef __cplusplus
}
#endif
