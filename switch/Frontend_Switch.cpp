// Switch replacements for the Android-only parts of the native frontend:
// the globals MelonDS.cpp owns, the NDK functions the shared code calls, and
// the features that are not available here (RetroAchievements, RetroArch
// shader presets, the OpenGL context).

#include <switch.h>

#include <EGL/egl.h>
#include <EGL/eglext.h>

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <filesystem>
#include <string>

#include <android/log.h>
#include <android/native_window.h>

#include "MelonDS.h"
#include "Platform.h"
#include "renderer/VulkanRetroArchFilterChain.h"
#include "retroachievements/RetroAchievementsManager.h"

#include "SwitchFrontend.h"

namespace MelonDSAndroid
{
OpenGLContext* openGlContext = nullptr;
AndroidFileHandler* fileHandler = nullptr;
AndroidCameraHandler* cameraHandler = nullptr;
std::string internalFilesDir;
std::shared_ptr<MelonEventMessenger> eventMessenger;

std::atomic<std::uint64_t> vulkanUltimaEsperaColaNs { 0 };
std::atomic<int> drsErrorLimitadorC { 0 };

// only Vulkan is used on the Switch
bool ensureOpenGlContext() { return false; }

bool isFastForwardActive() { return SwitchFrontend::FastForwardActive.load(); }

bool isVulkanPerfLoggingEnabled() { return false; }
bool isVulkanLatchPerfLoggingEnabled() { return false; }
bool isVulkanAsyncFrameTailEnabled() { return false; }
bool isVulkanSetupPerfLoggingEnabled() { return false; }
u32 getVulkanDiagnosticFlags() { return 0; }

bool areRenderer3DDebugControlsActive() { return false; }
bool isRenderer3DDebugFeatureEnabled(u32) { return true; }

VulkanRetroArchFilterChain::~VulkanRetroArchFilterChain() = default;
VulkanRetroArchFilterChain::VulkanRetroArchFilterChain(VulkanRetroArchFilterChain&&) noexcept {}
VulkanRetroArchFilterChain& VulkanRetroArchFilterChain::operator=(VulkanRetroArchFilterChain&&) noexcept { return *this; }
void VulkanRetroArchFilterChain::shutdown() {}

bool VulkanRetroArchFilterChain::configure(const std::string&, melonDS::u32, melonDS::u32, melonDS::u32, melonDS::u32,
    const std::vector<std::pair<std::string, float>>&)
{
    return false;
}

bool VulkanRetroArchFilterChain::recordFrame(VkCommandBuffer, VkImage, VkImage, melonDS::u64, bool, melonDS::u32)
{
    return false;
}

namespace RetroAchievements
{
RetroAchievementsManager::RetroAchievementsManager(melonDS::NDS* nds) :
    nds(nds),
    rcClientRuntime(nullptr),
    isRichPresenceEnabled(false),
    isRcClientRuntimeActive(false),
    runtimeMode {},
    rcClientSlowWindowCount(0),
    rcClientWindowFrameCount(0),
    rcClientWindowSlowFrameCount(0),
    rcClientWindowAccumulatedUs(0),
    rcClientWindowPeakUs(0),
    rcClientWindowCpuFrameCount(0),
    rcClientWindowCpuSlowFrameCount(0),
    rcClientWindowCpuAccumulatedUs(0),
    rcClientWindowCpuPeakUs(0)
{
}

RetroAchievementsManager::~RetroAchievementsManager() = default;
void RetroAchievementsManager::Close() {}
void RetroAchievementsManager::FrameUpdate() {}
bool RetroAchievementsManager::DoSavestate(melonDS::Savestate*) { return true; }
bool RetroAchievementsManager::AreSaveStatesAllowed() { return true; }
void RetroAchievementsManager::Reset() {}
}
}

namespace melonDS::Platform
{
FileHandle* OpenInternalFile(const std::string path, FileMode mode)
{
    std::filesystem::path fullFilePath = MelonDSAndroid::internalFilesDir;
    fullFilePath /= path;
    return OpenFile(fullFilePath.string(), mode);
}
}

extern "C"
{

int __android_log_vprint(int prio, const char* tag, const char* fmt, va_list args)
{
    if (prio < ANDROID_LOG_INFO)
        return 0;

    char line[1024];
    vsnprintf(line, sizeof(line), fmt, args);
    SwitchFrontend::Log("[%s] %s\n", tag ? tag : "", line);
    return 1;
}

int __android_log_print(int prio, const char* tag, const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int result = __android_log_vprint(prio, tag, fmt, args);
    va_end(args);
    return result;
}

int __android_log_write(int prio, const char* tag, const char* text)
{
    return __android_log_print(prio, tag, "%s", text);
}

// The presenter's ANativeWindow is the libnx NWindow, which the applet owns.
void ANativeWindow_acquire(ANativeWindow*) {}
void ANativeWindow_release(ANativeWindow*) {}

int32_t ANativeWindow_getWidth(ANativeWindow* window)
{
    u32 width = 1280, height = 720;
    nwindowGetDimensions(reinterpret_cast<NWindow*>(window), &width, &height);
    return width;
}

int32_t ANativeWindow_getHeight(ANativeWindow* window)
{
    u32 width = 1280, height = 720;
    nwindowGetDimensions(reinterpret_cast<NWindow*>(window), &width, &height);
    return height;
}

// EGL fence sync belongs to the OpenGL frame path, which is never taken here.
EGLSyncKHR eglCreateSyncKHR(EGLDisplay, EGLenum, const EGLint*) { return EGL_NO_SYNC_KHR; }
EGLBoolean eglDestroySyncKHR(EGLDisplay, EGLSyncKHR) { return EGL_TRUE; }
EGLint eglClientWaitSyncKHR(EGLDisplay, EGLSyncKHR, EGLint, EGLTimeKHR) { return EGL_CONDITION_SATISFIED_KHR; }
EGLint eglWaitSyncKHR(EGLDisplay, EGLSyncKHR, EGLint) { return EGL_TRUE; }

}
