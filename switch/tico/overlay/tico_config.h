// Copyright 2026 Azahar Emulator Project
// Copyright 2026 Dan | ticoverse.com
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

#include "SwitchEmulator.h"

namespace SwitchFrontend::TicoConfig {

// Loads the tico config for the WatermelonDS core from the first existing path
// in the search list (sdmc:/tico/config/cores/watermelonds.jsonc and fallbacks).
// Safe to call repeatedly; a missing file simply yields an empty option set.
void ReloadConfig();

// Returns the string value for `key`, or `default_value` if the key is absent.
std::string GetConfigValue(std::string_view key, std::string_view default_value = {});

// Sets an option in memory (call SaveConfig to persist it to the writable path).
void SetConfigValue(const std::string& key, const std::string& value);

// Writes the current option set back to the writable config path as JSON.
bool SaveConfig();

// The emulator settings described by the loaded options; anything not present
// keeps its default. Unknown keys are ignored so the same file can carry
// options this core doesn't use.
SwitchEmulator::Settings BuildSettings();

std::string GetLoadedConfigPath();
std::size_t GetLoadedOptionCount();

// ---------------------------------------------------------------------------
// Option catalogue: one table describes every setting, and drives both the
// config file and the overlay's settings menu.

enum class OptionType {
    // true / false
    Toggle,
    // one of a fixed list of values
    Choice,
    // an integer between Min and Max, changed in Step increments
    Range,
    // free text, edited with the system keyboard
    Text,
};

struct OptionChoice {
    // the value stored in the config file
    const char* value;
    // translation key and English text for the menu
    const char* label_key;
    const char* fallback;
};

struct OptionDef {
    const char* key;
    // translation key and English text for the menu
    const char* label_key;
    const char* fallback;
    OptionType type;
    const char* default_value;
    const OptionChoice* choices;
    std::size_t choice_count;
    int min;
    int max;
    int step;
    // only read when a game starts
    bool needs_restart;
    // maximum length for Text options
    int max_length;
};

struct OptionCategory {
    const char* label_key;
    const char* fallback;
    const OptionDef* options;
    std::size_t option_count;
};

const std::vector<OptionCategory>& GetCategories();

// The stored value of an option, or its default when the file does not set it.
std::string GetOptionValue(const OptionDef& option);

// The text the menu shows for the option's current value (the translation key
// and English text of a choice, or the value itself for ranges and text).
struct OptionValueLabel {
    const char* label_key;
    std::string fallback;
};
OptionValueLabel GetOptionValueLabel(const OptionDef& option);

// Moves a Toggle, Choice or Range option to its next (direction > 0) or
// previous value and saves the config. Toggles and choices wrap around; ranges
// stop at their limits.
void StepOption(const OptionDef& option, int direction);

// Stores a new value for any option and saves the config.
void SetOptionValue(const OptionDef& option, const std::string& value);

} // namespace SwitchFrontend::TicoConfig
