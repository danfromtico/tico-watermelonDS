// WatermelonDS entrypoint for the tico frontend.
//
// tico chainloads this NRO with the ROM path in argv[1] and the game's display
// title in argv[2]; on exit it chainloads back to tico with --resume. Started
// without a ROM, it falls back to the first .nds in sdmc:/switch/watermelonds/
// so the interface can be tested without tico. This file
// wires the tico pieces (overlay and quick menu, per-core config, save states,
// cheats, layout hotkeys) to the shared SwitchEmulator runtime.

#include <switch.h>

#include <dirent.h>
#include <sys/stat.h>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "ARCodeFile.h"

#include "SwitchEmulator.h"
#include "SwitchFrontend.h"
#include "tico/overlay/overlay_ui.h"
#include "tico/overlay/tico_config.h"
#include "tico/overlay/vulkan_overlay.h"

extern "C"
{
u32 __nx_applet_type = AppletType_Application;
}

namespace
{
// tico's SD card layout
constexpr const char* kSystemDir = "sdmc:/tico/system/nds/";
constexpr const char* kCheatDir = "sdmc:/tico/system/nds/cheats/";
constexpr const char* kSaveDir = "sdmc:/tico/saves/nds/";
constexpr const char* kStateDir = "sdmc:/tico/states/nds/";
constexpr const char* kLogPath = "sdmc:/tico/system/nds/debug/watermelonds.txt";
constexpr const char* kTicoLauncherPath = "sdmc:/switch/tico/tico.nro";
// where a ROM is picked from when the NRO is started without tico
constexpr const char* kFallbackRomDir = "sdmc:/switch/watermelonds/";

// if the overlay is open but has not been drawn for this many frames, the
// paused image is not being presented and emulation is resumed underneath it
constexpr u32 kOverlayStallFrames = 30;

PadState pad {};

// The rewind points as of the moment the menu opened. RewindManager is not
// thread-safe, so the emulation thread takes this copy and the overlay, which
// renders on the presentation thread, only ever reads the copy.
std::mutex RewindPointsMutex;
std::vector<int> RewindPoints;

void SnapshotRewindPoints()
{
    std::vector<int> points = SwitchEmulator::GetRewindPoints();
    std::lock_guard lock(RewindPointsMutex);
    RewindPoints = std::move(points);
}

void EnsureDirs()
{
    mkdir("sdmc:/tico", 0777);
    mkdir("sdmc:/tico/system", 0777);
    mkdir("sdmc:/tico/system/nds", 0777);
    mkdir("sdmc:/tico/system/nds/debug", 0777);
    mkdir("sdmc:/tico/system/nds/cheats", 0777);
    mkdir("sdmc:/tico/saves", 0777);
    mkdir("sdmc:/tico/saves/nds", 0777);
    mkdir("sdmc:/tico/states", 0777);
    mkdir("sdmc:/tico/states/nds", 0777);
}

// Queues tico to be loaded when this process exits.
bool QueueTicoReturn()
{
    if (!envHasNextLoad())
    {
        SwitchFrontend::Log("tico return: the loader does not support envSetNextLoad\n");
        return false;
    }

    struct stat info;
    if (stat(kTicoLauncherPath, &info) != 0)
    {
        SwitchFrontend::Log("tico return: no launcher at %s\n", kTicoLauncherPath);
        return false;
    }

    char args[512];
    snprintf(args, sizeof(args), "%s --resume", kTicoLauncherPath);
    Result rc = envSetNextLoad(kTicoLauncherPath, args);
    SwitchFrontend::Log("tico return: envSetNextLoad rc=0x%x\n", rc);
    return R_SUCCEEDED(rc);
}

// Lets the interface be tested without tico: the alphabetically first .nds in
// the standalone build's folder.
std::string FindFallbackRom()
{
    DIR* dir = opendir(kFallbackRomDir);
    if (!dir)
        return {};

    std::string found;
    while (dirent* entry = readdir(dir))
    {
        const std::string name = entry->d_name;
        if (name.size() < 4 || strcasecmp(name.c_str() + name.size() - 4, ".nds") != 0)
            continue;
        if (found.empty() || name < found)
            found = name;
    }
    closedir(dir);

    return found.empty() ? found : kFallbackRomDir + found;
}

std::string GetRomPath(int argc, char** argv)
{
    for (int i = 1; i < argc; i++)
    {
        // some tico launch paths pass this guard word before the ROM
        if (!argv[i] || !argv[i][0] || strcmp(argv[i], "ticoSetup") == 0)
            continue;
        return argv[i];
    }

    const std::string fallback = FindFallbackRom();
    if (!fallback.empty())
        SwitchFrontend::Log("no ROM in argv, using fallback %s\n", fallback.c_str());
    return fallback;
}

std::string FileStem(const std::string& path)
{
    size_t slash = path.find_last_of("/:");
    std::string name = slash == std::string::npos ? path : path.substr(slash + 1);
    size_t dot = name.find_last_of('.');
    return dot == std::string::npos ? name : name.substr(0, dot);
}

std::string GetDisplayTitle(int argc, char** argv, const std::string& romPath)
{
    if (argc > 2 && argv[2] && argv[2][0])
        return argv[2];
    return FileStem(romPath);
}

std::string StatePath(const std::string& romStem, int slot)
{
    return std::string(kStateDir) + romStem + ".state" + std::to_string(slot);
}

// Cheats come from a melonDS cheat file (.mch) named after the ROM. The overlay
// lists and toggles them on the presentation thread; the emulation thread
// picks the change up between frames.
class CheatList
{
public:
    explicit CheatList(const std::string& path) : File(path)
    {
        File.Load();
        Collect();
    }

    std::vector<SwitchFrontend::OverlayUI::CheatMenuEntry> List()
    {
        std::lock_guard lock(Mutex);
        std::vector<SwitchFrontend::OverlayUI::CheatMenuEntry> entries;
        entries.reserve(Codes.size());
        for (size_t i = 0; i < Codes.size(); i++)
        {
            std::string name = Codes[i]->Name;
            if (name.empty())
                name = "Cheat " + std::to_string(i + 1);
            entries.push_back({ std::move(name), Codes[i]->Enabled, true, (int)i });
        }
        return entries;
    }

    bool Toggle(int index)
    {
        std::lock_guard lock(Mutex);
        if (index < 0 || (size_t)index >= Codes.size())
            return false;
        Codes[index]->Enabled = !Codes[index]->Enabled;
        File.Save();
        Dirty = true;
        return true;
    }

    // Sends the enabled codes to the emulator if they changed.
    void Apply(bool force)
    {
        std::vector<std::vector<melonDS::u32>> enabled;
        {
            std::lock_guard lock(Mutex);
            if (!Dirty && !force)
                return;
            Dirty = false;
            for (const melonDS::ARCode* code : Codes)
            {
                if (code->Enabled)
                    enabled.push_back(code->Code);
            }
        }
        SwitchEmulator::SetCheats(enabled);
    }

    size_t Count() const { return Codes.size(); }

private:
    void Collect()
    {
        for (auto& item : File.RootCat.Children)
        {
            if (auto* category = std::get_if<melonDS::ARCodeCat>(&item))
            {
                for (auto& child : category->Children)
                {
                    if (auto* code = std::get_if<melonDS::ARCode>(&child))
                        Codes.push_back(code);
                }
            }
            else if (auto* code = std::get_if<melonDS::ARCode>(&item))
            {
                Codes.push_back(code);
            }
        }
    }

    std::mutex Mutex;
    melonDS::ARCodeFile File;
    std::vector<melonDS::ARCode*> Codes;
    bool Dirty = false;
};

void ConfigureOverlay(const std::string& displayTitle, const std::string& romStem, CheatList& cheats)
{
    using namespace SwitchFrontend;

    OverlayUI::SetGameTitle(displayTitle.empty() ? std::string { "WatermelonDS" } : displayTitle);

    OverlayUI::SetSlotOccupiedCallback([romStem](int slot) {
        struct stat info;
        return slot > 0 && stat(StatePath(romStem, slot).c_str(), &info) == 0 && info.st_size > 0;
    });

    OverlayUI::SetCheatCallbacks(
        [&cheats]() { return cheats.List(); },
        [&cheats](int index) { return cheats.Toggle(index); });

    OverlayUI::SetRewindCallback([]() {
        std::lock_guard lock(RewindPointsMutex);
        return RewindPoints;
    });
}

// Opens the system keyboard for a text setting (firmware nickname, message).
void EditTextOption(const SwitchFrontend::TicoConfig::OptionDef& option)
{
    using namespace SwitchFrontend;

    SwkbdConfig keyboard;
    if (R_FAILED(swkbdCreate(&keyboard, 0)))
    {
        SwitchFrontend::Log("text entry: swkbdCreate failed\n");
        return;
    }
    swkbdConfigMakePresetDefault(&keyboard);
    const std::string current = TicoConfig::GetOptionValue(option);
    swkbdConfigSetInitialText(&keyboard, current.c_str());
    swkbdConfigSetHeaderText(&keyboard, option.fallback);
    if (option.max_length > 0)
        swkbdConfigSetStringLenMax(&keyboard, option.max_length);

    char text[128] = {};
    const Result rc = swkbdShow(&keyboard, text, sizeof(text));
    swkbdClose(&keyboard);
    if (R_FAILED(rc) || text[0] == '\0')
        return;

    TicoConfig::SetOptionValue(option, text);
    OverlayUI::NotifyOptionEdited(option);
}

// Returns true when the overlay asked to leave the game.
bool HandleOverlayAction(SwitchFrontend::OverlayUI::Action action, const std::string& romStem)
{
    using namespace SwitchFrontend;
    using OverlayUI::Action;

    if (action == Action::None)
        return false;
    if (action == Action::Exit || VulkanOverlay::ShouldExit())
    {
        SwitchFrontend::Log("overlay requested exit\n");
        return true;
    }
    if (action == Action::Reset)
    {
        SwitchFrontend::Log("overlay requested reset\n");
        SwitchEmulator::Reset();
        OverlayUI::ShowToast("Reset", OverlayUI::ToastCorner::TopRight);
        return false;
    }
    if (action == Action::Rewind)
    {
        const int index = OverlayUI::ConsumeRewindIndex();
        const bool rewound = index >= 0 && SwitchEmulator::LoadRewindPoint((size_t)index);
        SwitchFrontend::Log("rewind to point %d: %s\n", index, rewound ? "ok" : "failed");
        OverlayUI::ShowToast(rewound ? "Rewound" : "Rewind failed", OverlayUI::ToastCorner::TopRight);
        return false;
    }
    if (action == Action::EditText)
    {
        if (const TicoConfig::OptionDef* option = OverlayUI::ConsumeTextEditOption())
            EditTextOption(*option);
        return false;
    }

    const int slot = OverlayUI::GetStateSlotForAction(action);
    if (slot <= 0)
        return false;

    const std::string path = StatePath(romStem, slot);
    if (OverlayUI::IsSaveStateAction(action))
    {
        const bool saved = SwitchEmulator::SaveState(path);
        SwitchFrontend::Log("save state slot %d: %s\n", slot, saved ? "ok" : "failed");
        OverlayUI::ShowToast(saved ? "State saved" : "Save failed", OverlayUI::ToastCorner::TopRight);
    }
    else if (OverlayUI::IsLoadStateAction(action))
    {
        const bool loaded = SwitchEmulator::LoadState(path);
        SwitchFrontend::Log("load state slot %d: %s\n", slot, loaded ? "ok" : "failed");
        OverlayUI::ShowToast(loaded ? "State loaded" : "Load failed", OverlayUI::ToastCorner::TopRight);
    }
    return false;
}

// Rising-edge detection so each press advances a single layout instead of
// cycling every frame the combo is held.
bool LayoutComboPressed(u64 held)
{
    static bool wasDown = false;
    const bool down = (held & HidNpadButton_ZL) && (held & HidNpadButton_Minus);
    const bool triggered = down && !wasDown;
    wasDown = down;
    return triggered;
}

bool SwapScreensHotkeyPressed(u64 held)
{
    static bool wasDown = false;
    const bool down = (held & HidNpadButton_StickL) != 0;
    const bool triggered = down && !wasDown;
    wasDown = down;
    return triggered;
}

// Re-reads every setting from the config and hands it to the emulator.
void ApplyConfigToEmulator()
{
    SwitchEmulator::ApplySettings(SwitchFrontend::TicoConfig::BuildSettings());
}

void CycleScreenLayout()
{
    using namespace SwitchFrontend;

    for (const auto& category : TicoConfig::GetCategories())
    {
        for (size_t i = 0; i < category.option_count; i++)
        {
            if (strcmp(category.options[i].key, "layout") == 0)
                TicoConfig::StepOption(category.options[i], 1);
        }
    }
    ApplyConfigToEmulator();
}

void ToggleSwapScreens()
{
    using namespace SwitchFrontend;

    const bool swapped = TicoConfig::GetConfigValue("swap_screens", "false") == "true";
    TicoConfig::SetConfigValue("swap_screens", swapped ? "false" : "true");
    TicoConfig::SaveConfig();
    ApplyConfigToEmulator();
}

bool IsEnabled(const char* key)
{
    return SwitchFrontend::TicoConfig::GetConfigValue(key, "false") == "true";
}

// Feeds the FPS counter and rendered-resolution display twice a second.
void UpdateHud()
{
    static u64 lastTick = 0;
    static u64 lastPresented = 0;

    const u64 now = armGetSystemTick();
    if (lastTick == 0)
    {
        lastTick = now;
        lastPresented = SwitchEmulator::GetStats().PresentedFrames;
        return;
    }
    const double seconds = armTicksToNs(now - lastTick) / 1e9;
    if (seconds < 0.5)
        return;

    const SwitchEmulator::Stats stats = SwitchEmulator::GetStats();
    SwitchFrontend::OverlayUI::HudStats hud;
    // frames actually shown: with frameskip the emulator can run 60 frames a
    // second while drawing far fewer, and that is what the player sees
    hud.fps = (float)((stats.PresentedFrames - lastPresented) / seconds);
    hud.rendered_scale = stats.RenderedScale;
    hud.fast_forward = SwitchEmulator::IsFastForward();
    SwitchFrontend::OverlayUI::SetHudStats(hud);
    lastTick = now;
    lastPresented = stats.PresentedFrames;
}

void RunLoop(const std::string& romStem, CheatList& cheats)
{
    using namespace SwitchFrontend;

    bool wasVisible = false;
    u64 lastDrawCount = 0;
    u32 stalledFrames = 0;
    bool stallLogged = false;

    while (appletMainLoop() && !SwitchEmulator::StopRequested())
    {
        padUpdate(&pad);
        const u64 held = padGetButtons(&pad);

        VulkanOverlay::Update(&pad);
        if (HandleOverlayAction(static_cast<OverlayUI::Action>(VulkanOverlay::ConsumeAction()), romStem))
            break;

        // settings changed in the quick menu
        if (OverlayUI::ConsumeSettingsChanged())
            ApplyConfigToEmulator();
        cheats.Apply(false);

        const bool visible = VulkanOverlay::IsVisible();
        if (visible != wasVisible)
        {
            if (visible)
                SnapshotRewindPoints();
            SwitchEmulator::SetPaused(visible);
            wasVisible = visible;
            stalledFrames = 0;
        }

        if (!visible)
        {
            if (LayoutComboPressed(held))
                CycleScreenLayout();
            if (SwapScreensHotkeyPressed(held))
                ToggleSwapScreens();
        }

        // ZR held fast forwards; the right stick click held blows into the microphone
        SwitchEmulator::SetFastForward(!visible && (held & HidNpadButton_ZR));
        SwitchEmulator::SetMicActive(!visible && (held & HidNpadButton_StickR));

        // The game is frozen while the quick menu is open. If the frozen frame
        // is not being presented the menu would never appear, so fall back to
        // running the game underneath it.
        bool emulate = !visible;
        if (visible)
        {
            const u64 drawCount = VulkanOverlay::GetDrawCount();
            stalledFrames = drawCount == lastDrawCount ? stalledFrames + 1 : 0;
            lastDrawCount = drawCount;
            if (stalledFrames >= kOverlayStallFrames)
            {
                emulate = true;
                if (!stallLogged)
                {
                    SwitchFrontend::Log("overlay: paused frame is not being presented, running the game under the menu\n");
                    stallLogged = true;
                }
            }
        }

        if (emulate)
        {
            // the game sees no input while the menu is open, and never the combo that opens it
            const bool menuCombo = (held & HidNpadButton_Plus) && (held & HidNpadButton_Minus);
            SwitchEmulator::UpdateInput((visible || menuCombo) ? 0 : held);
            SwitchEmulator::RunFrame();
            UpdateHud();
        }
        else
        {
            SwitchEmulator::IdleFrame();
        }
    }
}

int Run(int argc, char** argv)
{
    using namespace SwitchFrontend;

    EnsureDirs();
    SwitchFrontend::OpenLog(kLogPath);
    SwitchFrontend::Log("WatermelonDS tico entrypoint\n");
    for (int i = 0; i < argc; i++)
        SwitchFrontend::Log("argv[%d]=%s\n", i, argv[i] ? argv[i] : "(null)");

    const std::string romPath = GetRomPath(argc, argv);
    if (romPath.empty())
    {
        SwitchFrontend::Log("no ROM path supplied and no .nds in %s\n", kFallbackRomDir);
        return 1;
    }
    const std::string romStem = FileStem(romPath);
    const std::string displayTitle = GetDisplayTitle(argc, argv, romPath);
    SwitchFrontend::Log("rom: %s\ntitle: %s\n", romPath.c_str(), displayTitle.c_str());

    // fonts, translations and assets for the overlay
    const bool romfsReady = R_SUCCEEDED(romfsInit());
    SwitchFrontend::Log("romfs %s\n", romfsReady ? "mounted" : "unavailable");

    padConfigureInput(1, HidNpadStyleSet_NpadStandard);
    padInitializeDefault(&pad);

    SwitchEmulator::BootOptions options;
    options.RomPath = romPath;
    options.SavePath = std::string(kSaveDir) + romStem + ".sav";
    options.DataDir = kSystemDir;
    TicoConfig::ReloadConfig();
    options.Config = TicoConfig::BuildSettings();
    OverlayUI::ReloadSettings();

    std::string error;
    int result = 0;
    if (SwitchEmulator::Boot(options, error))
    {
        CheatList cheats(std::string(kCheatDir) + romStem + ".mch");
        SwitchFrontend::Log("cheats: %zu loaded\n", cheats.Count());
        cheats.Apply(true);

        ConfigureOverlay(displayTitle, romStem, cheats);
        const bool overlayReady = VulkanOverlay::Init();

        const std::string autoStatePath = std::string(kStateDir) + romStem + ".stateauto";
        if (IsEnabled("auto_load_state_on_launch"))
        {
            const bool loaded = SwitchEmulator::LoadState(autoStatePath);
            SwitchFrontend::Log("auto load state: %s\n", loaded ? "ok" : "none");
            if (loaded)
                OverlayUI::ShowToast("State loaded", OverlayUI::ToastCorner::TopRight);
        }

        RunLoop(romStem, cheats);

        if (IsEnabled("auto_save_state_on_exit"))
        {
            SwitchEmulator::SetPaused(false);
            const bool saved = SwitchEmulator::SaveState(autoStatePath);
            SwitchFrontend::Log("auto save state: %s\n", saved ? "ok" : "failed");
        }

        SwitchFrontend::Log("exiting after %llu frames\n", (unsigned long long)SwitchEmulator::GetStats().Frames);
        if (overlayReady)
            VulkanOverlay::Shutdown();
        // the overlay's callbacks point at this scope's cheat list
        OverlayUI::SetCheatCallbacks(nullptr, nullptr);
        OverlayUI::SetSlotOccupiedCallback(nullptr);
        OverlayUI::SetRewindCallback(nullptr);
        SwitchEmulator::Shutdown();
    }
    else
    {
        SwitchFrontend::Log("boot failed: %s\n", error.c_str());
        result = 1;
    }

    if (romfsReady)
        romfsExit();
    return result;
}
}

int main(int argc, char** argv)
{
    // keep the process alive through shutdown so saves are flushed and the return to tico is queued
    appletLockExit();

    int result = Run(argc, argv);

    const bool queued = QueueTicoReturn();
    SwitchFrontend::CloseLog();
    appletUnlockExit();
    return (result == 0 && queued) ? EXIT_SUCCESS : EXIT_FAILURE;
}
