#include "SwitchEmulator.h"

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <list>
#include <optional>
#include <vector>

#include "Configuration.h"
#include "EmulatorArgsBuilder.h"
#include "MelonDS.h"
#include "MelonInstance.h"
#include "GPU.h"
#include "Savestate.h"
#include "net/Net.h"
#include "renderer/ScreenshotRenderer.h"

#include "SwitchFrontend.h"

using namespace melonDS;
using namespace MelonDSAndroid;

extern "C"
{
// NVK's command submission goes through nvservices transfer memory
u32 __nx_nv_transfermem_size = 16 * 1024 * 1024;
}

namespace
{
constexpr u32 kFbWidth = 1280;
constexpr u32 kFbHeight = 720;

constexpr u32 kDsWidth = 256;
constexpr u32 kDsHeight = 192;

struct Rect
{
    int X, Y, Width, Height;
};

// As on Android, emulation is paced at the display's 60 Hz rather than the
// DS's 59.83 Hz, and the audio output is told about the difference so it
// resamples to the right pitch instead of having to estimate the speed.
constexpr u64 kFrameNs = 1000000000ull / 60;
constexpr double kNdsFramesPerSecond = 59.8260982880808;
constexpr int kScanlinesPerFrame = 263;

constexpr u32 kAudioBufferCount = 3;
constexpr u32 kAudioBufferSize = 0x1000;
constexpr u32 kAudioBufferSamples = kAudioBufferSize / (2 * sizeof(s16));

std::shared_ptr<MelonInstance> Instance;
std::atomic<bool> Running { false };
int SurfaceId = 0;
bool AudioStarted = false;

u64 Frames = 0;
u64 RunFrameTicks = 0;
u64 NextFrameNs = 0;
u64 HeldBefore = 0;
bool Touching = false;

// Current holds what the user asked for; Booted holds the boot-only fields the
// running instance was actually started with.
SwitchEmulator::Settings Current;
SwitchEmulator::Settings Booted;
bool MicHeld = false;

// read by the audio thread
std::atomic<bool> AudioSoundEnabled { true };
std::atomic<bool> AudioMuteOnFastForward { false };
std::atomic<int> AudioVolume { 256 };

// frame limiter and frameskip state, following the Android app's emulation loop
double FrameLimitError = 0.0;
double LastTickMs = 0.0;
bool FrameskipRequested = false;
int FrameskipInhibitedFrames = 2;
int FrameskipManualCycle = 0;
bool DrsDebt = false;

// where the DS touch screen currently is on the display; empty when it is not shown
Rect TouchRect {};

// The 2D worker spins between scanlines, so it runs one priority step below
// the other worker threads: Horizon does not time-slice threads of equal
// priority, and a spinning worker would otherwise starve them.
void PlaceRender2DWorker()
{
    s32 priority = 0x2C;
    svcGetThreadPriority(&priority, CUR_THREAD_HANDLE);
    SwitchFrontend::Cores::PinCurrentThreadToWorkCores();
    svcSetThreadPriority(CUR_THREAD_HANDLE, priority + 1);
}

double NowMs()
{
    return armTicksToNs(armGetSystemTick()) / 1e6;
}

// Boot-only fields always come from the running instance, whatever was asked for since.
SwitchEmulator::Settings EffectiveSettings()
{
    SwitchEmulator::Settings settings = Current;
    settings.UseJit = Booted.UseJit;
    settings.Threaded2D = Booted.Threaded2D;
    settings.RenderScale = Booted.RenderScale;
    settings.Nickname = Booted.Nickname;
    settings.Message = Booted.Message;
    settings.Language = Booted.Language;
    settings.FavouriteColour = Booted.FavouriteColour;
    settings.BirthdayMonth = Booted.BirthdayMonth;
    settings.BirthdayDay = Booted.BirthdayDay;
    return settings;
}

std::shared_ptr<EmulatorConfiguration> MakeConfiguration(const SwitchEmulator::Settings& settings)
{
    auto config = std::make_shared<EmulatorConfiguration>();

    // FreeBIOS and generated firmware: nothing else is needed on the SD card.
    config->userInternalFirmwareAndBios = true;
    config->internalFilesDir = strdup(SwitchFrontend::BaseDir.c_str());
    config->fastForwardSpeedMultiplier = settings.FastForwardSpeed;
    config->frameLimitSpeedMultiplier = settings.FrameLimitSpeed;
    config->frameskipMode = settings.FrameskipMode;
    config->frameskipManualValue = settings.FrameskipManualValue;
    config->vulkanDrsEnabled = settings.DynamicResolution;
    config->useJit = settings.UseJit;
    config->consoleType = 0;

    config->audioSettings.soundEnabled = settings.SoundEnabled;
    config->audioSettings.muteOnFastForward = settings.MuteOnFastForward;
    config->audioSettings.volume = settings.Volume;
    config->audioSettings.audioInterpolation = settings.AudioInterpolation;
    config->audioSettings.audioBitrate = settings.AudioBitDepth;
    config->audioSettings.micSource = settings.MicSource;

    config->rewindEnabled = settings.RewindEnabled ? 1 : 0;
    config->rewindCaptureSpacingSeconds = settings.RewindPeriodSeconds;
    config->rewindLengthSeconds = settings.RewindWindowSeconds;

    FirmwareConfiguration& firmware = config->firmwareConfiguration;
    snprintf(firmware.username, sizeof(firmware.username), "%s", settings.Nickname.c_str());
    snprintf(firmware.message, sizeof(firmware.message), "%s", settings.Message.c_str());
    firmware.language = settings.Language;
    firmware.favouriteColour = settings.FavouriteColour;
    firmware.birthdayMonth = settings.BirthdayMonth;
    firmware.birthdayDay = settings.BirthdayDay;

    auto render = std::make_unique<VulkanRenderSettings>();
    render->threadedRendering = settings.ThreadedRendering;
    render->betterPolygons = settings.BetterPolygons;
    render->scale = std::clamp(settings.RenderScale, 1, 8);
    render->conservativeCoverageEnabled = settings.ConservativeCoverage;
    render->conservativeCoveragePx = settings.ConservativeCoveragePx;
    render->conservativeCoverageDepthBias = settings.ConservativeCoverageDepthBias;
    render->conservativeCoverageApplyRepeat = settings.ConservativeCoverageRepeat;
    render->conservativeCoverageApplyClamp = settings.ConservativeCoverageClamp;
    render->videoFiltering = static_cast<VulkanFilterMode>(std::clamp(settings.Filter, 0, 7));
    config->renderSettings = std::move(render);
    config->renderer = Renderer::Vulkan;

    return config;
}

// The speed emulation is currently limited to, as a multiple of real time; 0 means unlimited.
double TargetSpeed()
{
    if (SwitchFrontend::FastForwardActive.load())
        return Current.FastForwardSpeed > 0.0f ? Current.FastForwardSpeed : 0.0;
    return Current.FrameLimitSpeed > 0.0f ? Current.FrameLimitSpeed : 1.0;
}

void PublishRuntimeState()
{
    AudioSoundEnabled = Current.SoundEnabled;
    AudioMuteOnFastForward = Current.MuteOnFastForward;
    AudioVolume = std::clamp(Current.Volume, 0, 256);
    SwitchFrontend::MicBlowing = MicHeld && Current.MicSource == 1;
    if (Instance)
        Instance->setAudioOutputSpeedHint(TargetSpeed() * 60.0 / kNdsFramesPerSecond);
}

// Places a DS screen inside the cell its layout assigns to it.
Rect FitScreen(const Rect& cell, SwitchEmulator::DisplaySize size)
{
    if (size == SwitchEmulator::DisplaySize::Stretch)
        return cell;

    int width, height;
    if (size == SwitchEmulator::DisplaySize::Original)
    {
        int scale = std::min(cell.Width / (int)kDsWidth, cell.Height / (int)kDsHeight);
        if (scale < 1)
            scale = 1;
        width = kDsWidth * scale;
        height = kDsHeight * scale;
    }
    else
    {
        // DS screens are 4:3
        width = cell.Width;
        height = width * 3 / 4;
        if (height > cell.Height)
        {
            height = cell.Height;
            width = height * 4 / 3;
        }
    }
    return { cell.X + (cell.Width - width) / 2, cell.Y + (cell.Height - height) / 2, width, height };
}

VulkanSurfaceConfig MakeSurfaceConfig(const SwitchEmulator::Settings& settings, Rect& touchRect)
{
    const SwitchEmulator::DisplayOptions& options = settings.Display;
    using SwitchEmulator::ScreenLayout;

    const int w = kFbWidth;
    const int h = kFbHeight;

    // first is where the top screen goes unless the screens are swapped
    Rect first {};
    Rect second {};
    bool secondShown = true;
    switch (options.Layout)
    {
    case ScreenLayout::Stacked:
        first = { 0, 0, w, h / 2 };
        second = { 0, h / 2, w, h / 2 };
        break;
    case ScreenLayout::SingleScreen:
        first = { 0, 0, w, h };
        secondShown = false;
        break;
    case ScreenLayout::LargeScreen:
        first = { 0, 0, w * 3 / 4, h };
        second = { w * 3 / 4, h - h / 3, w / 4, h / 3 };
        break;
    case ScreenLayout::LargeScreenInverted:
        first = { w / 4, 0, w * 3 / 4, h };
        second = { 0, h - h / 3, w / 4, h / 3 };
        break;
    case ScreenLayout::SideBySide:
    default:
        first = { 0, 0, w / 2, h };
        second = { w / 2, 0, w / 2, h };
        break;
    }

    const Rect firstRect = FitScreen(first, options.Size);
    const Rect secondRect = FitScreen(second, options.Size);

    const Rect& topRect = options.SwapScreens ? secondRect : firstRect;
    const Rect& bottomRect = options.SwapScreens ? firstRect : secondRect;
    const bool topShown = options.SwapScreens ? secondShown : true;
    const bool bottomShown = options.SwapScreens ? true : secondShown;

    VulkanSurfaceConfig config {};
    config.topScreen = { topShown, topRect.X, topRect.Y, topRect.Width, topRect.Height };
    config.bottomScreen = { bottomShown, bottomRect.X, bottomRect.Y, bottomRect.Width, bottomRect.Height };
    config.filtering = static_cast<VulkanFilterMode>(std::clamp(settings.Filter, 0, 7));

    touchRect = bottomShown ? bottomRect : Rect {};
    return config;
}

// Presentation runs on its own thread, as on Android: it takes composed frames
// from the frame queue and presents them, blocking on the swapchain for vsync.
::Thread PresentWorker {};
std::atomic<u64> PresentedFrames { 0 };

void PresentThread(void*)
{
    while (Running)
    {
        u64 epoch = Instance->captureVulkanPresentationWaitEpoch();
        VulkanPresentationResult result = Instance->presentVulkanFrame(std::nullopt, std::nullopt, epoch);
        switch (result)
        {
        case VulkanPresentationResult::Presented:
            PresentedFrames++;
            break;
        case VulkanPresentationResult::NoProduct:
            Instance->waitForVulkanPresentationProduct(epoch, 50000000);
            break;
        case VulkanPresentationResult::FatalError:
            SwitchFrontend::Log("vulkan presentation failed\n");
            SwitchFrontend::StopRequested = true;
            return;
        default:
            svcSleepThread(1000000);
            break;
        }
    }
}

// Audio is pulled by its own thread through the core's adaptive output, which
// stretches the stream when emulation runs slightly off real time.
::Thread AudioWorker {};

void AudioThread(void*)
{
    if (R_FAILED(audoutInitialize()))
        return;
    if (R_FAILED(audoutStartAudioOut()))
    {
        audoutExit();
        return;
    }

    Instance->resetAudioOutputAdaptivo();

    AudioOutBuffer buffers[kAudioBufferCount] {};
    for (AudioOutBuffer& buffer : buffers)
    {
        buffer.buffer = aligned_alloc(0x1000, kAudioBufferSize);
        buffer.buffer_size = kAudioBufferSize;
        buffer.data_size = kAudioBufferSize;
        memset(buffer.buffer, 0, kAudioBufferSize);
        audoutAppendAudioOutBuffer(&buffer);
    }

    while (Running)
    {
        AudioOutBuffer* released = nullptr;
        u32 count = 0;
        if (R_FAILED(audoutWaitPlayFinish(&released, &count, 100000000)) || !released)
            continue;

        s16* samples = (s16*)released->buffer;
        int read = Instance->readAudioOutputAdaptivo(samples, kAudioBufferSamples);
        if (read < 0)
            read = 0;
        memset(samples + read * 2, 0, (kAudioBufferSamples - read) * 2 * sizeof(s16));

        const int volume = AudioVolume;
        if (!AudioSoundEnabled || (AudioMuteOnFastForward && SwitchFrontend::FastForwardActive.load()))
        {
            memset(samples, 0, kAudioBufferSize);
        }
        else if (volume < 256)
        {
            for (u32 i = 0; i < kAudioBufferSamples * 2; i++)
                samples[i] = ((s32)samples[i] * volume) >> 8;
        }
        released->data_size = kAudioBufferSize;
        audoutAppendAudioOutBuffer(released);
    }

    audoutStopAudioOut();
    audoutExit();
    for (AudioOutBuffer& buffer : buffers)
        free(buffer.buffer);
}

// priorityBoost raises the thread above the workers sharing its core. Horizon
// does not time-slice threads of equal priority, so without it a busy worker
// can hold the core until it blocks.
// Background threads go where SwitchFrontend::Cores puts them. The audio and
// presentation threads run above the worker threads, as in the Cemu and
// Vita3K ports: a busy worker would otherwise starve them under Horizon's
// strict priorities.
bool StartBackgroundThread(::Thread& thread, ThreadFunc entry, int priorityBoost)
{
    int core;
    u32 mask;
    SwitchFrontend::Cores::NextBackgroundCore(core, mask);

    s32 priority = 0x2C;
    svcGetThreadPriority(&priority, CUR_THREAD_HANDLE);
    if (R_FAILED(threadCreate(&thread, entry, nullptr, nullptr, 0x100000, priority - priorityBoost, core))
        && R_FAILED(threadCreate(&thread, entry, nullptr, nullptr, 0x100000, priority, core)))
    {
        return false;
    }
    svcSetThreadCoreMask(thread.handle, core, mask);
    return R_SUCCEEDED(threadStart(&thread));
}
}

namespace SwitchEmulator
{

bool Boot(const BootOptions& options, std::string& error)
{
    // the shared loader expects the save file to exist, as the Android app guarantees
    if (FILE* save = fopen(options.SavePath.c_str(), "ab"))
        fclose(save);

    SwitchFrontend::BaseDir = options.DataDir;
    MelonDSAndroid::internalFilesDir = options.DataDir;
    Current = options.Config;
    Booted = options.Config;
    SwitchFrontend::FastForwardActive = false;
    std::shared_ptr<EmulatorConfiguration> config = MakeConfiguration(Current);
    melonDS::GPU::DefaultThreaded2D = Current.Threaded2D;
    melonDS::GPU::Render2DWorkerInit = PlaceRender2DWorker;

    // the emulated CPUs get a core to themselves; see SwitchFrontend::Cores
    SwitchFrontend::Cores::PinEmulationThread();
    SwitchFrontend::Log("threaded 2D %s\n", Current.Threaded2D ? "on" : "off");

    auto args = BuildArgsFromConfiguration(*config, 0);
    if (!args.has_value())
    {
        error = "Could not build the emulator arguments.";
        return false;
    }

    static u32 screenshotBuffer[kDsWidth * kDsHeight * 2];
    Instance = std::make_shared<MelonInstance>(
        0,
        config,
        std::move(args.value()),
        std::make_shared<Net>(),
        std::make_unique<ScreenshotRenderer>(screenshotBuffer),
        config->consoleType);

    if (!Instance->loadRom(options.RomPath, options.SavePath))
    {
        error = "Could not load " + options.RomPath;
        Instance.reset();
        return false;
    }
    SwitchFrontend::Log("rom loaded\n");

    const VulkanSurfaceConfig surfaceConfig = MakeSurfaceConfig(Current, TouchRect);
    if (!Instance->precompileVulkanPipelines(surfaceConfig))
    {
        error = "Vulkan renderer setup failed, see the log.";
        Instance.reset();
        return false;
    }
    SwitchFrontend::Log("vulkan pipelines ready\n");

    PublishRuntimeState();
    Instance->start();

    SurfaceId = Instance->attachVulkanSurface((ANativeWindow*)nwindowGetDefault(), kFbWidth, kFbHeight);
    if (SurfaceId <= 0)
    {
        error = "Could not attach the Vulkan surface, see the log.";
        Instance->stop();
        Instance.reset();
        return false;
    }
    Instance->configureVulkanSurface(SurfaceId, surfaceConfig, VulkanBackgroundImage {});
    SwitchFrontend::Log("core started, surface %d attached\n", SurfaceId);

    hidInitializeTouchScreen();

    Running = true;
    if (!StartBackgroundThread(PresentWorker, PresentThread, 1))
    {
        error = "Could not start the presentation thread.";
        Running = false;
        Instance->detachVulkanSurface(SurfaceId);
        Instance->stop();
        Instance.reset();
        return false;
    }
    AudioStarted = StartBackgroundThread(AudioWorker, AudioThread, 2);

    NextFrameNs = armTicksToNs(armGetSystemTick());
    FrameLimitError = 0.0;
    LastTickMs = NowMs();
    FrameskipRequested = false;
    FrameskipInhibitedFrames = 2;
    FrameskipManualCycle = 0;
    DrsDebt = false;
    return true;
}

void UpdateInput(u64 held)
{
    // melonDS key indices
    static const HidNpadButton map[12] = {
        HidNpadButton_A,
        HidNpadButton_B,
        HidNpadButton_Minus,
        HidNpadButton_Plus,
        HidNpadButton_AnyRight,
        HidNpadButton_AnyLeft,
        HidNpadButton_AnyUp,
        HidNpadButton_AnyDown,
        HidNpadButton_R,
        HidNpadButton_L,
        HidNpadButton_X,
        HidNpadButton_Y,
    };

    for (u32 i = 0; i < 12; i++)
    {
        bool now = held & map[i];
        bool before = HeldBefore & map[i];
        if (now && !before)
            Instance->pressKey(i);
        else if (!now && before)
            Instance->releaseKey(i);
    }
    HeldBefore = held;

    HidTouchScreenState state {};
    if (hidGetTouchScreenStates(&state, 1) && state.count > 0)
    {
        int x = state.touches[0].x;
        int y = state.touches[0].y;
        if (TouchRect.Width > 0 && TouchRect.Height > 0
            && x >= TouchRect.X && x < TouchRect.X + TouchRect.Width
            && y >= TouchRect.Y && y < TouchRect.Y + TouchRect.Height)
        {
            Instance->touchScreen((x - TouchRect.X) * kDsWidth / TouchRect.Width, (y - TouchRect.Y) * kDsHeight / TouchRect.Height);
            Touching = true;
            return;
        }
    }
    if (Touching)
        Instance->releaseScreen();
    Touching = false;
}

// One emulated frame followed by the frame limiter. The limiter, the frameskip
// request and the dynamic-resolution debt signal follow the Android app's
// emulation loop, so the emulator's own frameskip and DRS logic sees the same
// inputs it was tuned for.
void RunFrame()
{
    const bool fastForward = SwitchFrontend::FastForwardActive.load();

    Instance->configurarFrameskip(Current.FrameskipMode, Current.FrameskipManualValue);
    Instance->configurarDrs(Current.DynamicResolution, DrsDebt);
    DrsDebt = false;

    const bool requestedThisFrame = FrameskipRequested;
    u64 frameStart = armGetSystemTick();
    const u32 lines = Instance->runFrame(FrameskipRequested);
    RunFrameTicks += armGetSystemTick() - frameStart;
    const bool grantedThisFrame = Instance->frameskipConcedidoUltimoFrame();
    FrameskipRequested = false;
    Frames++;

    double currentTick = NowMs();
    const double speed = TargetSpeed();
    if (speed <= 0.0)
    {
        // unlimited fast forward
        FrameLimitError = 0.0;
        LastTickMs = currentTick;
        return;
    }

    double delay = currentTick - LastTickMs;
    double frameTimeStep = (double)lines / (60.0 * speed * kScanlinesPerFrame) * 1000.0;
    if (frameTimeStep < 1.0)
        frameTimeStep = 1.0;

    if (!fastForward)
    {
        // time spent waiting on the frame queue is not time the frame took to emulate
        const double queueWait = MelonDSAndroid::vulkanUltimaEsperaColaNs.load(std::memory_order_acquire) / 1e6;
        if (queueWait > 0.0 && delay <= frameTimeStep * 1.05)
            delay -= std::min(queueWait, frameTimeStep);
    }

    FrameLimitError += frameTimeStep - delay;
    FrameLimitError = std::clamp(FrameLimitError, -frameTimeStep, frameTimeStep);

    if (FrameskipInhibitedFrames > 0 && lines > 0)
        FrameskipInhibitedFrames--;

    const bool behind = lines > 0 && !fastForward && FrameskipInhibitedFrames == 0
        && FrameLimitError <= -(frameTimeStep * 0.5);

    if (Current.FrameskipMode == 1 && Current.FrameskipManualValue > 0 && lines > 0 && !fastForward
        && FrameskipInhibitedFrames == 0)
    {
        // a request the emulator refused does not advance the manual cycle
        if (!(requestedThisFrame && !grantedThisFrame))
            FrameskipManualCycle = (FrameskipManualCycle + 1) % (Current.FrameskipManualValue + 1);
    }
    else
    {
        FrameskipManualCycle = 0;
    }

    DrsDebt = behind;
    if (Current.FrameskipMode == 0)
        FrameskipRequested = false;
    else if (Current.FrameskipMode == 1)
        FrameskipRequested = FrameskipManualCycle != 0;
    else
        FrameskipRequested = behind;

    if (FrameLimitError >= 0.5)
    {
        svcSleepThread((s64)(FrameLimitError * 1e6));
        const double afterSleep = NowMs();
        FrameLimitError -= afterSleep - currentTick;
        currentTick = afterSleep;
    }
    LastTickMs = currentTick;
}

void SetPaused(bool paused)
{
    if (!Instance)
        return;

    Instance->setVulkanRepresentPreviousFrameWhenIdle(paused);
    if (!paused)
    {
        // the time spent paused must not look like the game running behind
        FrameLimitError = 0.0;
        LastTickMs = NowMs();
        FrameskipRequested = false;
        FrameskipInhibitedFrames = 2;
        FrameskipManualCycle = 0;
        DrsDebt = false;
    }
}

void IdleFrame()
{
    NextFrameNs += kFrameNs;
    u64 nowNs = armTicksToNs(armGetSystemTick());
    if (nowNs < NextFrameNs)
        svcSleepThread(NextFrameNs - nowNs);
    else
        NextFrameNs = nowNs;
}

void ApplySettings(const Settings& settings)
{
    const bool surfaceChanged = settings.Filter != Current.Filter
        || settings.Display.Layout != Current.Display.Layout
        || settings.Display.SwapScreens != Current.Display.SwapScreens
        || settings.Display.Size != Current.Display.Size;
    const bool speedChanged = settings.FastForwardSpeed != Current.FastForwardSpeed
        || settings.FrameLimitSpeed != Current.FrameLimitSpeed;

    Current = settings;
    PublishRuntimeState();
    if (!Instance)
        return;

    Instance->updateConfiguration(MakeConfiguration(EffectiveSettings()));

    if (surfaceChanged && SurfaceId > 0)
    {
        const VulkanSurfaceConfig surfaceConfig = MakeSurfaceConfig(Current, TouchRect);
        Instance->configureVulkanSurface(SurfaceId, surfaceConfig, VulkanBackgroundImage {});
    }
    if (speedChanged)
    {
        FrameLimitError = 0.0;
        LastTickMs = NowMs();
    }
}

Settings GetSettings()
{
    return Current;
}

void SetDisplayOptions(const DisplayOptions& options)
{
    Current.Display = options;
    if (!Instance || SurfaceId <= 0)
        return;

    const VulkanSurfaceConfig surfaceConfig = MakeSurfaceConfig(Current, TouchRect);
    Instance->configureVulkanSurface(SurfaceId, surfaceConfig, VulkanBackgroundImage {});
}

DisplayOptions GetDisplayOptions()
{
    return Current.Display;
}

void SetFastForward(bool enabled)
{
    if (!Instance || SwitchFrontend::FastForwardActive.load() == enabled)
        return;

    SwitchFrontend::FastForwardActive = enabled;
    Instance->requestVulkanFastForwardPresentationTransition();
    PublishRuntimeState();
    FrameLimitError = 0.0;
    LastTickMs = NowMs();
    FrameskipRequested = false;
    FrameskipInhibitedFrames = 2;
}

bool IsFastForward()
{
    return SwitchFrontend::FastForwardActive.load();
}

void SetMicActive(bool active)
{
    MicHeld = active;
    SwitchFrontend::MicBlowing = MicHeld && Current.MicSource == 1;
}

void Reset()
{
    if (!Instance)
        return;

    Instance->reset();
    FrameLimitError = 0.0;
    LastTickMs = NowMs();
    FrameskipRequested = false;
    FrameskipInhibitedFrames = 2;
}

std::vector<int> GetRewindPoints()
{
    std::vector<int> points;
    if (!Instance || !Current.RewindEnabled)
        return points;

    RewindWindow window = Instance->getRewindWindow();
    for (const RewindSaveState& state : window.rewindStates)
        points.push_back(std::max(0, (window.currentFrame - state.frame) / 60));
    // nearest first
    std::sort(points.begin(), points.end());
    return points;
}

bool LoadRewindPoint(size_t index)
{
    if (!Instance)
        return false;

    RewindWindow window = Instance->getRewindWindow();
    std::vector<RewindSaveState> states(window.rewindStates.begin(), window.rewindStates.end());
    std::sort(states.begin(), states.end(),
        [](const RewindSaveState& a, const RewindSaveState& b) { return a.frame > b.frame; });
    if (index >= states.size())
        return false;

    if (!Instance->loadRewindState(states[index]))
        return false;
    Instance->requestVulkanPresentationResync();
    return true;
}

bool SaveState(const std::string& path)
{
    if (!Instance)
        return false;

    Savestate state;
    if (state.Error || !Instance->saveState(&state, false) || state.Error)
    {
        SwitchFrontend::Log("save state: could not serialize\n");
        return false;
    }

    FILE* file = fopen(path.c_str(), "wb");
    if (!file)
    {
        SwitchFrontend::Log("save state: could not open %s\n", path.c_str());
        return false;
    }
    const bool written = fwrite(state.Buffer(), 1, state.Length(), file) == state.Length();
    const bool closed = fclose(file) == 0;
    if (!written || !closed)
    {
        SwitchFrontend::Log("save state: could not write %s\n", path.c_str());
        remove(path.c_str());
        return false;
    }
    return true;
}

bool LoadState(const std::string& path)
{
    if (!Instance || !Instance->areSaveStatesAllowed())
        return false;

    FILE* file = fopen(path.c_str(), "rb");
    if (!file)
        return false;
    fseeko(file, 0, SEEK_END);
    const off_t size = ftello(file);
    fseeko(file, 0, SEEK_SET);
    std::vector<u8> buffer(size > 0 ? (size_t)size : 0);
    const bool read = !buffer.empty() && fread(buffer.data(), 1, buffer.size(), file) == buffer.size();
    fclose(file);
    if (!read)
    {
        SwitchFrontend::Log("load state: could not read %s\n", path.c_str());
        return false;
    }

    // keep the current state so a rejected file does not leave the core half loaded
    auto backup = std::make_unique<Savestate>(Savestate::DEFAULT_SIZE);
    if (backup->Error || !Instance->saveState(backup.get(), false) || backup->Error)
    {
        SwitchFrontend::Log("load state: could not back up the current state\n");
        return false;
    }

    auto state = std::make_unique<Savestate>(buffer.data(), (u32)buffer.size(), false);
    if (!Instance->loadState(state.get()) || state->Error)
    {
        SwitchFrontend::Log("load state: %s was rejected\n", path.c_str());
        auto restore = std::make_unique<Savestate>(backup->Buffer(), backup->Length(), false);
        if (!Instance->loadState(restore.get()) || restore->Error)
            SwitchFrontend::Log("load state: could not restore the previous state\n");
        return false;
    }

    Instance->requestVulkanPresentationResync();
    return true;
}

void SetCheats(const std::vector<std::vector<u32>>& codes)
{
    if (!Instance)
        return;

    std::list<Cheat> cheats;
    for (const std::vector<u32>& code : codes)
        cheats.push_back(Cheat { code });
    Instance->loadCheats(std::move(cheats));
}

Stats GetStats()
{
    Stats stats { Frames, PresentedFrames.load(), RunFrameTicks, Instance ? Instance->getVulkanEscalaRenderizada() : 0, {} };
    for (int core = 0; core < 3; core++)
        svcGetInfo(&stats.IdleTicks[core], InfoType_IdleTickCount, INVALID_HANDLE, core);
    return stats;
}

bool StopRequested()
{
    return SwitchFrontend::StopRequested;
}

void Shutdown()
{
    if (!Instance)
        return;

    Running = false;
    Instance->cancelVulkanPresentationWaits();
    threadWaitForExit(&PresentWorker);
    threadClose(&PresentWorker);
    if (AudioStarted)
    {
        threadWaitForExit(&AudioWorker);
        threadClose(&AudioWorker);
        AudioStarted = false;
    }

    Instance->detachVulkanSurface(SurfaceId);
    Instance->stop();
    Instance.reset();
}

}
