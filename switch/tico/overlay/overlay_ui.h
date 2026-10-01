// Copyright 2026 Azahar Emulator Project
// Copyright 2026 Dan | ticoverse.com
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include <functional>
#include <string>
#include <vector>

#include "tico/overlay/tico_config.h"

namespace SwitchFrontend::OverlayUI {

enum class Action {
    None,
    Resume,
    Exit,
    Reset,
    // the point to go back to is given by ConsumeRewindIndex
    Rewind,
    // a text setting needs the system keyboard; see ConsumeTextEditOption
    EditText,
    SaveStateSlot1,
    SaveStateSlot2,
    SaveStateSlot3,
    SaveStateSlot4,
    LoadStateSlot1,
    LoadStateSlot2,
    LoadStateSlot3,
    LoadStateSlot4,
};

enum class ToastCorner {
    TopLeft,
    TopRight,
    BottomLeft,
    BottomRight,
};

bool IsSaveStateAction(Action action);
bool IsLoadStateAction(Action action);
int GetStateSlotForAction(Action action);

void SetVisible(bool visible);

Action Render(int display_w, int display_h);

void SetGameTitle(std::string title);
void SetNickname(std::string nickname);
void SetAvatarTextureId(unsigned long long texture_id);
void ShowToast(std::string message, ToastCorner corner = ToastCorner::TopLeft);
bool HasTransientContent();

// Predicate the overlay uses to label save/load slots as "In Use" or "Empty".
// The Switch frontend registers one that checks for the slot's state file.
using SlotOccupiedFn = std::function<bool(int slot)>;
void SetSlotOccupiedCallback(SlotOccupiedFn callback);

struct CheatMenuEntry {
    std::string name;
    bool enabled = false;
    bool toggleable = true;
    int source_index = -1;
};

using CheatListFn = std::function<std::vector<CheatMenuEntry>()>;
using CheatToggleFn = std::function<bool(int source_index)>;
void SetCheatCallbacks(CheatListFn list_callback, CheatToggleFn toggle_callback);

// Lists the rewind points as seconds before now, nearest first.
using RewindListFn = std::function<std::vector<int>()>;
void SetRewindCallback(RewindListFn callback);

// The overlay renders on the presentation thread and never calls into the
// emulator. The emulation thread polls these after each frame.

// True once after settings were changed from the menu.
bool ConsumeSettingsChanged();
// The rewind point chosen with Action::Rewind, or -1.
int ConsumeRewindIndex();
// The option chosen with Action::EditText, or nullptr.
const TicoConfig::OptionDef* ConsumeTextEditOption();
// Call after storing a value for an option outside the menu (text entry), so
// the change is picked up like any other.
void NotifyOptionEdited(const TicoConfig::OptionDef& option);

// Figures shown by the FPS counter and rendered-resolution display.
struct HudStats {
    float fps = 0.0f;
    int rendered_scale = 0;
    bool fast_forward = false;
};
void SetHudStats(const HudStats& stats);
// Re-reads the HUD positions from the config; call once it has been loaded.
void ReloadSettings();

struct NavInput {
    bool up;
    bool down;
    bool left;
    bool right;
    bool accept;
    bool cancel;
};
void FeedNav(const NavInput& nav);

} // namespace SwitchFrontend::OverlayUI
