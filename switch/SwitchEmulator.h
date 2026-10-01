#pragma once

#include <switch.h>

#include <string>
#include <vector>

// The Switch runtime shared by every entrypoint: boots a cartridge through the
// Android app's native frontend (MelonInstance and its Vulkan pipeline), owns
// the presentation and audio threads, and paces emulation.
namespace SwitchEmulator
{
// How the two DS screens are arranged on the 1280x720 display.
enum class ScreenLayout
{
    SideBySide,
    Stacked,
    SingleScreen,
    LargeScreen,
    LargeScreenInverted,
};

// How each screen is sized inside the area its layout gives it.
enum class DisplaySize
{
    // largest size that keeps the DS aspect ratio
    Fill,
    // fills the whole area, ignoring the aspect ratio
    Stretch,
    // largest whole multiple of the DS resolution
    Original,
};

struct DisplayOptions
{
    ScreenLayout Layout = ScreenLayout::SideBySide;
    // puts the bottom screen where the top screen would go, and vice versa
    bool SwapScreens = false;
    DisplaySize Size = DisplaySize::Fill;
};

// Everything the user can configure, mirroring the Android app's video, audio
// and system preferences. Fields marked "boot" are read once by Boot; the rest
// can be changed while a game runs through ApplySettings.
struct Settings
{
    DisplayOptions Display;
    // presentation filter: 0 none, 1 linear, 2 xBR 2x, 3 HQ2x, 4 HQ4x, 5 Quilez, 6 LCD, 7 scanlines
    int Filter = 0;

    // boot: internal 3D resolution as a multiple of the DS's 256x192
    int RenderScale = 1;
    bool ThreadedRendering = true;
    bool BetterPolygons = false;
    // lowers the internal resolution while the game cannot hold full speed
    bool DynamicResolution = false;
    bool ConservativeCoverage = false;
    float ConservativeCoveragePx = 1.5f;
    float ConservativeCoverageDepthBias = 0.0f;
    bool ConservativeCoverageRepeat = true;
    bool ConservativeCoverageClamp = false;

    bool SoundEnabled = true;
    // 0 none, 1 linear, 2 cosine, 3 cubic
    int AudioInterpolation = 0;
    // 0 auto, 1 10-bit, 2 16-bit
    int AudioBitDepth = 0;
    // 0..256
    int Volume = 256;
    bool MuteOnFastForward = false;
    // 0 none, 1 blow noise while the microphone button is held
    int MicSource = 1;

    // boot
    bool UseJit = true;
    // boot: draws the 2D layers on another core while the emulated CPUs run.
    // Experimental: the fork's capture tracking syncs with the worker on most
    // DMA and VRAM accesses, which can make it slower than drawing in place.
    bool Threaded2D = false;
    // 0 off, 1 manual, 2 auto
    int FrameskipMode = 2;
    int FrameskipManualValue = 1;
    // speed while fast forwarding; values <= 0 mean unlimited
    float FastForwardSpeed = -1.0f;
    float FrameLimitSpeed = 1.0f;
    bool RewindEnabled = false;
    int RewindPeriodSeconds = 10;
    int RewindWindowSeconds = 60;

    // boot: written into the generated firmware's user settings
    std::string Nickname = "Player";
    std::string Message = "Hello!";
    // 0 Japanese, 1 English, 2 French, 3 German, 4 Italian, 5 Spanish
    int Language = 1;
    int FavouriteColour = 0;
    int BirthdayMonth = 1;
    int BirthdayDay = 1;
};

struct BootOptions
{
    std::string RomPath;
    std::string SavePath;
    // pipeline caches and other emulator-owned files; needs a trailing slash
    std::string DataDir;
    Settings Config;
};

struct Stats
{
    u64 Frames;
    u64 PresentedFrames;
    // time spent inside the emulator's runFrame, in system ticks
    u64 RunFrameTicks;
    // internal resolution multiple the renderer is currently using
    int RenderedScale;
    // per core, system ticks the core spent idle since boot
    u64 IdleTicks[3];
};

// Returns false with a message in error when the cartridge or Vulkan could not be set up.
bool Boot(const BootOptions& options, std::string& error);

// held is the pad's button state; touch comes from the touch screen.
void UpdateInput(u64 held);

// Runs one emulated frame, then sleeps to hold 60 fps.
void RunFrame();

// While paused the caller keeps the loop alive with IdleFrame instead of
// RunFrame; the last frame stays on screen and keeps being presented so
// anything drawn over it (the tico overlay) still animates.
void SetPaused(bool paused);
void IdleFrame();

// Applies the settings that can change while a game runs. Boot-only fields are
// remembered (GetSettings returns them) but take effect on the next launch.
void ApplySettings(const Settings& settings);
Settings GetSettings();

void SetDisplayOptions(const DisplayOptions& options);
DisplayOptions GetDisplayOptions();

void SetFastForward(bool enabled);
bool IsFastForward();

// Feeds the blow noise to the DS microphone while active, if the mic source allows it.
void SetMicActive(bool active);

void Reset();

// The rewind states currently held, newest first, as seconds before now.
std::vector<int> GetRewindPoints();
bool LoadRewindPoint(size_t index);

// Call between frames, from the thread that calls RunFrame.
bool SaveState(const std::string& path);
bool LoadState(const std::string& path);

// Replaces the active Action Replay codes; each entry is one cheat's code words.
void SetCheats(const std::vector<std::vector<u32>>& codes);

Stats GetStats();

// True once the core or the presenter asked to stop.
bool StopRequested();

void Shutdown();
}
