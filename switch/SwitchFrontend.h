#pragma once

#include <atomic>
#include <cstdarg>
#include <string>

#include <switch.h>

namespace SwitchFrontend
{
// directory for emulator-owned files, with a trailing slash
extern std::string BaseDir;
extern bool StopRequested;

// read by the shared frontend and the audio thread
extern std::atomic<bool> FastForwardActive;
// while set, the DS microphone hears the blow noise
extern std::atomic<bool> MicBlowing;

namespace Cores
{
// Pins the calling thread to the core reserved for the emulated CPUs.
void PinEmulationThread();
// The core and affinity mask for the next worker / background thread.
void NextWorkCore(int& core, u32& mask);
void NextBackgroundCore(int& core, u32& mask);
void PinCurrentThreadToWorkCores();
}

void OpenLog(const std::string& path);
void CloseLog();
void Log(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void LogV(const char* fmt, va_list args);
}
