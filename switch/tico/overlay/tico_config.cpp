// Copyright 2026 Azahar Emulator Project
// Copyright 2026 Dan | ticoverse.com
// SPDX-License-Identifier: GPL-2.0-or-later

#include "tico/overlay/tico_config.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdlib>
#include <cstdio>
#include <initializer_list>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <sys/stat.h>

#include <json.hpp>

#include "SwitchFrontend.h"

namespace SwitchFrontend::TicoConfig {
namespace {

using OptionMap = std::map<std::string, std::string, std::less<>>;

constexpr std::array<const char*, 4> kConfigPaths = {{
    "sdmc:/tico/config/cores/watermelonds.jsonc",
    "sdmc:/tico/config/cores/watermelonds.json",
    "romfs:/config/watermelonds.jsonc",
    "sdmc:/tico/system/nds/tico.jsonc",
}};

constexpr const char* kDefaultWritableConfigPath = "sdmc:/tico/config/cores/watermelonds.jsonc";

void EnsureWritableConfigDirectory() {
    mkdir("sdmc:/tico", 0777);
    mkdir("sdmc:/tico/config", 0777);
    mkdir("sdmc:/tico/config/cores", 0777);
}

// Strips // line and /* */ block comments so a .jsonc file parses as plain JSON.
std::string StripJsonComments(std::string_view input) {
    std::string output;
    output.reserve(input.size());

    bool in_string = false;
    bool escaped = false;
    for (std::size_t i = 0; i < input.size(); ++i) {
        const char c = input[i];
        if (in_string) {
            output.push_back(c);
            if (escaped) {
                escaped = false;
            } else if (c == '\\') {
                escaped = true;
            } else if (c == '"') {
                in_string = false;
            }
            continue;
        }
        if (c == '"') {
            in_string = true;
            output.push_back(c);
            continue;
        }
        if (c == '/' && i + 1 < input.size()) {
            if (input[i + 1] == '/') {
                i += 2;
                while (i < input.size() && input[i] != '\n') {
                    ++i;
                }
                if (i < input.size()) {
                    output.push_back('\n');
                }
                continue;
            }
            if (input[i + 1] == '*') {
                i += 2;
                while (i + 1 < input.size() && !(input[i] == '*' && input[i + 1] == '/')) {
                    ++i;
                }
                if (i + 1 < input.size()) {
                    ++i;
                }
                continue;
            }
        }
        output.push_back(c);
    }
    return output;
}

bool ReadWholeFile(const char* path, std::string& out) {
    std::FILE* fp = std::fopen(path, "rb");
    if (!fp) {
        return false;
    }
    std::fseek(fp, 0, SEEK_END);
    const long size = std::ftell(fp);
    std::fseek(fp, 0, SEEK_SET);
    if (size < 0) {
        std::fclose(fp);
        return false;
    }
    out.resize(static_cast<std::size_t>(size));
    const std::size_t read = std::fread(out.data(), 1, out.size(), fp);
    std::fclose(fp);
    out.resize(read);
    return true;
}

// Converts a JSON scalar into the canonical string we store internally.
std::string JsonScalarToString(const nlohmann::json& value) {
    if (value.is_string()) {
        return value.get<std::string>();
    }
    if (value.is_boolean()) {
        return value.get<bool>() ? "true" : "false";
    }
    if (value.is_number_integer()) {
        return std::to_string(value.get<long long>());
    }
    if (value.is_number_unsigned()) {
        return std::to_string(value.get<unsigned long long>());
    }
    if (value.is_number_float()) {
        return std::to_string(value.get<double>());
    }
    return {};
}

std::string LowerCopy(std::string_view value) {
    std::string out(value);
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
}

std::optional<bool> ParseBool(std::string_view value) {
    const std::string lower = LowerCopy(value);
    if (lower == "true" || lower == "1" || lower == "on" || lower == "yes" ||
        lower == "enabled") {
        return true;
    }
    if (lower == "false" || lower == "0" || lower == "off" || lower == "no" ||
        lower == "disabled") {
        return false;
    }
    return std::nullopt;
}

std::optional<int> ParseInt(std::string_view value) {
    if (value.empty()) {
        return std::nullopt;
    }
    try {
        std::size_t consumed = 0;
        const int result = std::stoi(std::string(value), &consumed);
        if (consumed == value.size()) {
            return result;
        }
    } catch (...) {
    }
    return std::nullopt;
}

// ---------------------------------------------------------------------------
// Option catalogue. Keys and values follow the Android app's preferences where
// one exists, so a setting means the same thing in both.

#define CHOICES(name) name, sizeof(name) / sizeof(name[0])
#define NO_CHOICES nullptr, 0

constexpr OptionChoice kLayoutChoices[] = {
    {"side_by_side", "emulator_side_by_side", "Side by Side"},
    {"stacked", "emulator_stacked", "Stacked"},
    {"single", "emulator_single_screen", "Single Screen"},
    {"large", "emulator_large_screen", "Large Screen"},
    {"large_inverted", "emulator_large_screen_inverted", "Large Screen Inverted"},
};
constexpr OptionChoice kDisplaySizeChoices[] = {
    {"Fill", "emulator_fill", "Fill"},
    {"Stretch", "emulator_stretch", "Stretch"},
    {"Original", "emulator_original", "Original"},
};
constexpr OptionChoice kFilterChoices[] = {
    {"none", "emulator_filter_none", "None"},
    {"linear", "emulator_filter_linear", "Linear"},
    {"xbr2", "emulator_filter_xbr2", "xBR 2x"},
    {"hq2x", "emulator_filter_hq2x", "HQ2x"},
    {"hq4x", "emulator_filter_hq4x", "HQ4x"},
    {"quilez", "emulator_filter_quilez", "Quilez"},
    {"lcd", "emulator_filter_lcd", "LCD"},
    {"scanlines", "emulator_filter_scanlines", "Scanlines"},
};
constexpr OptionChoice kHudPositionChoices[] = {
    {"hidden", "emulator_hidden", "Hidden"},
    {"top_left", "emulator_top_left", "Top Left"},
    {"top_right", "emulator_top_right", "Top Right"},
    {"bottom_left", "emulator_bottom_left", "Bottom Left"},
    {"bottom_right", "emulator_bottom_right", "Bottom Right"},
};
constexpr OptionChoice kInterpolationChoices[] = {
    {"none", "emulator_interpolation_none", "None"},
    {"linear", "emulator_interpolation_linear", "Linear"},
    {"cosine", "emulator_interpolation_cosine", "Cosine"},
    {"cubic", "emulator_interpolation_cubic", "Cubic"},
};
constexpr OptionChoice kBitrateChoices[] = {
    {"auto", "emulator_auto", "Auto"},
    {"bit10", "emulator_bitrate_10", "10-bit"},
    {"bit16", "emulator_bitrate_16", "16-bit"},
};
constexpr OptionChoice kMicSourceChoices[] = {
    {"none", "emulator_mic_none", "None"},
    {"blow", "emulator_mic_blow", "Blow Noise"},
};
constexpr OptionChoice kFrameskipChoices[] = {
    {"off", "emulator_off", "Off"},
    {"manual", "emulator_manual", "Manual"},
    {"auto", "emulator_auto", "Auto"},
};
constexpr OptionChoice kFastForwardChoices[] = {
    {"-1", "emulator_unlimited", "Unlimited"},
    {"1.5", nullptr, "1.5x"},
    {"2", nullptr, "2x"},
    {"3", nullptr, "3x"},
    {"4", nullptr, "4x"},
    {"8", nullptr, "8x"},
};
constexpr OptionChoice kFrameLimitChoices[] = {
    {"1", nullptr, "100%"},
    {"0.75", nullptr, "75%"},
    {"0.5", nullptr, "50%"},
};
constexpr OptionChoice kLanguageChoices[] = {
    {"0", nullptr, "日本語"},
    {"1", nullptr, "English"},
    {"2", nullptr, "Français"},
    {"3", nullptr, "Deutsch"},
    {"4", nullptr, "Italiano"},
    {"5", nullptr, "Español"},
};

//                key, label key, English label, type, default, choices, min, max, step, restart, text length
constexpr OptionDef kDisplayOptions[] = {
    {"layout", "emulator_display_layout", "Display Layout", OptionType::Choice, "side_by_side",
     CHOICES(kLayoutChoices), 0, 0, 0, false, 0},
    {"swap_screens", "emulator_swap_screens", "Swap Screens", OptionType::Toggle, "false",
     NO_CHOICES, 0, 0, 0, false, 0},
    {"display_size", "emulator_display_size", "Display Size", OptionType::Choice, "Fill",
     CHOICES(kDisplaySizeChoices), 0, 0, 0, false, 0},
    {"video_filtering", "emulator_filter", "Filter", OptionType::Choice, "none",
     CHOICES(kFilterChoices), 0, 0, 0, false, 0},
    {"fps_counter_position", "emulator_fps_counter", "FPS Counter", OptionType::Choice, "hidden",
     CHOICES(kHudPositionChoices), 0, 0, 0, false, 0},
    {"rendered_ir_position", "emulator_rendered_resolution", "Rendered Resolution",
     OptionType::Choice, "hidden", CHOICES(kHudPositionChoices), 0, 0, 0, false, 0},
};
constexpr OptionDef kVideoOptions[] = {
    {"video_internal_resolution", "emulator_internal_resolution", "Internal Resolution",
     OptionType::Range, "1", NO_CHOICES, 1, 4, 1, true, 0},
    {"enable_threaded_rendering", "emulator_threaded_rendering", "Threaded Rendering",
     OptionType::Toggle, "true", NO_CHOICES, 0, 0, 0, false, 0},
    {"video_better_polygons", "emulator_better_polygons", "Improved Polygon Splitting",
     OptionType::Toggle, "false", NO_CHOICES, 0, 0, 0, false, 0},
    {"video_vulkan_drs", "emulator_dynamic_resolution", "Dynamic Resolution",
     OptionType::Toggle, "false", NO_CHOICES, 0, 0, 0, false, 0},
    {"video_conservative_coverage_enabled", "emulator_conservative_coverage",
     "Conservative Coverage", OptionType::Toggle, "false", NO_CHOICES, 0, 0, 0, false, 0},
    {"video_conservative_coverage_px", "emulator_conservative_coverage_px",
     "Coverage Size (1/100 px)", OptionType::Range, "150", NO_CHOICES, 0, 400, 25, false, 0},
    {"video_conservative_coverage_apply_repeat", "emulator_conservative_coverage_repeat",
     "Coverage on Repeating Textures", OptionType::Toggle, "true", NO_CHOICES, 0, 0, 0, false, 0},
    {"video_conservative_coverage_apply_clamp", "emulator_conservative_coverage_clamp",
     "Coverage on Clamped Textures", OptionType::Toggle, "false", NO_CHOICES, 0, 0, 0, false, 0},
    {"video_conservative_coverage_depth_bias", "emulator_conservative_coverage_depth_bias",
     "Coverage Depth Bias", OptionType::Range, "0", NO_CHOICES, 0, 100, 5, false, 0},
};
constexpr OptionDef kAudioOptions[] = {
    {"sound_enabled", "emulator_enable_sound", "Enable Sound", OptionType::Toggle, "true",
     NO_CHOICES, 0, 0, 0, false, 0},
    {"volume", "emulator_volume", "Volume", OptionType::Range, "256", NO_CHOICES, 0, 256, 16,
     false, 0},
    {"audio_interpolation", "emulator_interpolation", "Interpolation", OptionType::Choice, "none",
     CHOICES(kInterpolationChoices), 0, 0, 0, false, 0},
    {"audio_bitrate", "emulator_bitrate", "Bitrate", OptionType::Choice, "auto",
     CHOICES(kBitrateChoices), 0, 0, 0, false, 0},
    {"audio_mute_on_fast_forward", "emulator_mute_on_fast_forward", "Mute on Fast Forward",
     OptionType::Toggle, "false", NO_CHOICES, 0, 0, 0, false, 0},
    {"mic_source", "emulator_microphone_source", "Microphone Source", OptionType::Choice, "blow",
     CHOICES(kMicSourceChoices), 0, 0, 0, false, 0},
};
constexpr OptionDef kSystemOptions[] = {
    {"enable_jit", "emulator_enable_jit", "JIT Recompiler", OptionType::Toggle, "true",
     NO_CHOICES, 0, 0, 0, true, 0},
    {"threaded_2d", "emulator_threaded_2d", "Threaded 2D (experimental)", OptionType::Toggle, "false",
     NO_CHOICES, 0, 0, 0, true, 0},
    {"frameskip_mode", "emulator_frameskip", "Frameskip", OptionType::Choice, "auto",
     CHOICES(kFrameskipChoices), 0, 0, 0, false, 0},
    {"frameskip_manual_value", "emulator_frameskip_value", "Frames to Skip", OptionType::Range,
     "1", NO_CHOICES, 0, 4, 1, false, 0},
    {"fast_forward_speed_multiplier", "emulator_fast_forward_speed", "Fast Forward Speed",
     OptionType::Choice, "-1", CHOICES(kFastForwardChoices), 0, 0, 0, false, 0},
    {"frame_limit_speed_multiplier", "emulator_emulation_speed", "Emulation Speed",
     OptionType::Choice, "1", CHOICES(kFrameLimitChoices), 0, 0, 0, false, 0},
    {"enable_rewind", "emulator_enable_rewind", "Rewind", OptionType::Toggle, "false",
     NO_CHOICES, 0, 0, 0, false, 0},
    {"rewind_period", "emulator_rewind_period", "Rewind Interval (s)", OptionType::Range, "10",
     NO_CHOICES, 1, 60, 1, false, 0},
    {"rewind_window", "emulator_rewind_window", "Rewind Length (x10 s)", OptionType::Range, "6",
     NO_CHOICES, 1, 30, 1, false, 0},
    {"auto_save_state_on_exit", "emulator_auto_save_on_exit", "Save State on Exit",
     OptionType::Toggle, "false", NO_CHOICES, 0, 0, 0, false, 0},
    {"auto_load_state_on_launch", "emulator_auto_load_on_launch", "Load State on Launch",
     OptionType::Toggle, "false", NO_CHOICES, 0, 0, 0, false, 0},
};
constexpr OptionDef kFirmwareOptions[] = {
    {"firmware_settings_nickname", "emulator_firmware_nickname", "Nickname", OptionType::Text,
     "Player", NO_CHOICES, 0, 0, 0, true, 10},
    {"firmware_settings_message", "emulator_firmware_message", "Message", OptionType::Text,
     "Hello!", NO_CHOICES, 0, 0, 0, true, 26},
    {"firmware_settings_language", "emulator_firmware_language", "Language", OptionType::Choice,
     "1", CHOICES(kLanguageChoices), 0, 0, 0, true, 0},
    {"firmware_settings_colour", "emulator_firmware_colour", "Favourite Colour",
     OptionType::Range, "0", NO_CHOICES, 0, 15, 1, true, 0},
    {"firmware_settings_birthday_month", "emulator_firmware_birthday_month", "Birthday Month",
     OptionType::Range, "1", NO_CHOICES, 1, 12, 1, true, 0},
    {"firmware_settings_birthday_day", "emulator_firmware_birthday_day", "Birthday Day",
     OptionType::Range, "1", NO_CHOICES, 1, 31, 1, true, 0},
};

#define OPTIONS(name) name, sizeof(name) / sizeof(name[0])

const std::vector<OptionCategory> kCategories = {
    {"emulator_category_display", "Display", OPTIONS(kDisplayOptions)},
    {"emulator_category_video", "Video", OPTIONS(kVideoOptions)},
    {"emulator_category_audio", "Audio", OPTIONS(kAudioOptions)},
    {"emulator_category_system", "System", OPTIONS(kSystemOptions)},
    {"emulator_category_firmware", "Firmware", OPTIONS(kFirmwareOptions)},
};

// Index of the choice whose stored value matches, or 0 when none does.
std::size_t FindChoice(const OptionDef& option, std::string_view value) {
    const std::string lower = LowerCopy(value);
    for (std::size_t i = 0; i < option.choice_count; ++i) {
        if (LowerCopy(option.choices[i].value) == lower) {
            return i;
        }
    }
    return 0;
}

class Manager {
public:
    void ReloadConfig() {
        options.clear();
        loaded_path.clear();

        for (const char* path : kConfigPaths) {
            std::string content;
            if (!ReadWholeFile(path, content)) {
                continue;
            }
            const std::string stripped = StripJsonComments(content);
            nlohmann::json root = nlohmann::json::parse(stripped, nullptr, false);
            if (root.is_discarded() || !root.is_object()) {
                SwitchFrontend::Log("tico config at %s is not a JSON object\n", path);
                continue;
            }
            for (auto it = root.begin(); it != root.end(); ++it) {
                if (it.value().is_object() || it.value().is_array()) {
                    continue;
                }
                options[it.key()] = JsonScalarToString(it.value());
            }
            loaded_path = path;
            SwitchFrontend::Log("tico config loaded from %s (%zu options)\n", path, options.size());
            return;
        }
        SwitchFrontend::Log("no tico config found; using defaults\n");
    }

    std::string GetConfigValue(std::string_view key, std::string_view default_value) const {
        const auto it = options.find(key);
        if (it != options.end()) {
            return it->second;
        }
        return std::string(default_value);
    }

    void SetConfigValue(const std::string& key, const std::string& value) {
        options[key] = value;
    }

    bool SaveConfig() {
        nlohmann::json root = nlohmann::json::object();
        for (const auto& [key, value] : options) {
            if (const auto b = ParseBool(value)) {
                root[key] = *b;
            } else if (const auto i = ParseInt(value)) {
                root[key] = *i;
            } else {
                root[key] = value;
            }
        }
        const std::string serialized = root.dump(2);

        EnsureWritableConfigDirectory();

        const char* target = kDefaultWritableConfigPath;
        std::FILE* fp = std::fopen(target, "wb");
        if (!fp) {
            SwitchFrontend::Log("failed to open tico config for write: %s\n", target);
            return false;
        }
        const std::size_t written = std::fwrite(serialized.data(), 1, serialized.size(), fp);
        std::fclose(fp);
        if (written != serialized.size()) {
            SwitchFrontend::Log("failed to write full tico config: %s\n", target);
            return false;
        }
        loaded_path = target;
        return true;
    }

    std::string GetOptionValue(const OptionDef& option) const {
        return GetConfigValue(option.key, option.default_value);
    }

    bool GetBool(const OptionDef& option) const {
        if (const auto b = ParseBool(GetOptionValue(option))) {
            return *b;
        }
        return ParseBool(option.default_value).value_or(false);
    }

    int GetInt(const OptionDef& option) const {
        int value = ParseInt(GetOptionValue(option)).value_or(ParseInt(option.default_value).value_or(0));
        if (option.type == OptionType::Range) {
            value = std::clamp(value, option.min, option.max);
        }
        return value;
    }

    // For Choice options: the position of the stored value in the choice list.
    int GetChoiceIndex(const OptionDef& option) const {
        return static_cast<int>(FindChoice(option, GetOptionValue(option)));
    }

    float GetChoiceFloat(const OptionDef& option) const {
        const std::string value = option.choices[GetChoiceIndex(option)].value;
        return static_cast<float>(std::atof(value.c_str()));
    }

    const std::string& GetLoadedPath() const {
        return loaded_path;
    }

    std::size_t GetOptionCount() const {
        return options.size();
    }

private:
    std::optional<std::string> GetOptional(std::string_view key) const {
        const auto it = options.find(key);
        if (it == options.end()) {
            return std::nullopt;
        }
        return it->second;
    }

    // Returns the first of several alias keys that is present.
    std::optional<std::string> GetFirstOptional(std::initializer_list<std::string_view> keys) const {
        for (const std::string_view key : keys) {
            if (auto value = GetOptional(key)) {
                return value;
            }
        }
        return std::nullopt;
    }

    OptionMap options;
    std::string loaded_path;
};

Manager& GetManager() {
    static Manager manager;
    return manager;
}

} // namespace

void ReloadConfig() {
    GetManager().ReloadConfig();
}

std::string GetConfigValue(std::string_view key, std::string_view default_value) {
    return GetManager().GetConfigValue(key, default_value);
}

void SetConfigValue(const std::string& key, const std::string& value) {
    GetManager().SetConfigValue(key, value);
}

bool SaveConfig() {
    return GetManager().SaveConfig();
}

const std::vector<OptionCategory>& GetCategories() {
    return kCategories;
}

namespace {

// The catalogue entry for a key. Every key BuildSettings asks for is in the table.
const OptionDef& Option(std::string_view key) {
    for (const OptionCategory& category : kCategories) {
        for (std::size_t i = 0; i < category.option_count; ++i) {
            if (key == category.options[i].key) {
                return category.options[i];
            }
        }
    }
    return kCategories.front().options[0];
}

} // namespace

SwitchEmulator::Settings BuildSettings() {
    const Manager& config = GetManager();
    SwitchEmulator::Settings settings;

    settings.Display.Layout =
        static_cast<SwitchEmulator::ScreenLayout>(config.GetChoiceIndex(Option("layout")));
    settings.Display.SwapScreens = config.GetBool(Option("swap_screens"));
    settings.Display.Size =
        static_cast<SwitchEmulator::DisplaySize>(config.GetChoiceIndex(Option("display_size")));
    settings.Filter = config.GetChoiceIndex(Option("video_filtering"));

    settings.RenderScale = config.GetInt(Option("video_internal_resolution"));
    settings.ThreadedRendering = config.GetBool(Option("enable_threaded_rendering"));
    settings.BetterPolygons = config.GetBool(Option("video_better_polygons"));
    settings.DynamicResolution = config.GetBool(Option("video_vulkan_drs"));
    settings.ConservativeCoverage = config.GetBool(Option("video_conservative_coverage_enabled"));
    settings.ConservativeCoveragePx =
        static_cast<float>(config.GetInt(Option("video_conservative_coverage_px"))) / 100.0f;
    settings.ConservativeCoverageRepeat =
        config.GetBool(Option("video_conservative_coverage_apply_repeat"));
    settings.ConservativeCoverageClamp =
        config.GetBool(Option("video_conservative_coverage_apply_clamp"));
    settings.ConservativeCoverageDepthBias =
        static_cast<float>(config.GetInt(Option("video_conservative_coverage_depth_bias")));

    settings.SoundEnabled = config.GetBool(Option("sound_enabled"));
    settings.Volume = config.GetInt(Option("volume"));
    settings.AudioInterpolation = config.GetChoiceIndex(Option("audio_interpolation"));
    settings.AudioBitDepth = config.GetChoiceIndex(Option("audio_bitrate"));
    settings.MuteOnFastForward = config.GetBool(Option("audio_mute_on_fast_forward"));
    settings.MicSource = config.GetChoiceIndex(Option("mic_source"));

    settings.UseJit = config.GetBool(Option("enable_jit"));
    settings.Threaded2D = config.GetBool(Option("threaded_2d"));
    settings.FrameskipMode = config.GetChoiceIndex(Option("frameskip_mode"));
    settings.FrameskipManualValue = config.GetInt(Option("frameskip_manual_value"));
    settings.FastForwardSpeed = config.GetChoiceFloat(Option("fast_forward_speed_multiplier"));
    settings.FrameLimitSpeed = config.GetChoiceFloat(Option("frame_limit_speed_multiplier"));
    settings.RewindEnabled = config.GetBool(Option("enable_rewind"));
    settings.RewindPeriodSeconds = config.GetInt(Option("rewind_period"));
    settings.RewindWindowSeconds = config.GetInt(Option("rewind_window")) * 10;

    settings.Nickname = config.GetOptionValue(Option("firmware_settings_nickname"));
    settings.Message = config.GetOptionValue(Option("firmware_settings_message"));
    settings.Language = config.GetChoiceIndex(Option("firmware_settings_language"));
    settings.FavouriteColour = config.GetInt(Option("firmware_settings_colour"));
    settings.BirthdayMonth = config.GetInt(Option("firmware_settings_birthday_month"));
    settings.BirthdayDay = config.GetInt(Option("firmware_settings_birthday_day"));
    return settings;
}

std::string GetOptionValue(const OptionDef& option) {
    return GetManager().GetOptionValue(option);
}

OptionValueLabel GetOptionValueLabel(const OptionDef& option) {
    const Manager& config = GetManager();
    switch (option.type) {
    case OptionType::Toggle:
        return config.GetBool(option) ? OptionValueLabel{"emulator_on", "On"}
                                      : OptionValueLabel{"emulator_off", "Off"};
    case OptionType::Choice: {
        const OptionChoice& choice = option.choices[config.GetChoiceIndex(option)];
        return {choice.label_key, choice.fallback};
    }
    case OptionType::Range:
        return {nullptr, std::to_string(config.GetInt(option))};
    case OptionType::Text:
    default:
        return {nullptr, config.GetOptionValue(option)};
    }
}

void SetOptionValue(const OptionDef& option, const std::string& value) {
    GetManager().SetConfigValue(option.key, value);
    GetManager().SaveConfig();
}

void StepOption(const OptionDef& option, int direction) {
    if (direction == 0) {
        return;
    }
    const Manager& config = GetManager();
    switch (option.type) {
    case OptionType::Toggle:
        SetOptionValue(option, config.GetBool(option) ? "false" : "true");
        break;
    case OptionType::Choice: {
        const int count = static_cast<int>(option.choice_count);
        const int index = (config.GetChoiceIndex(option) + (direction > 0 ? 1 : count - 1)) % count;
        SetOptionValue(option, option.choices[index].value);
        break;
    }
    case OptionType::Range: {
        const int value = std::clamp(
            config.GetInt(option) + (direction > 0 ? option.step : -option.step), option.min,
            option.max);
        SetOptionValue(option, std::to_string(value));
        break;
    }
    case OptionType::Text:
    default:
        break;
    }
}

std::string GetLoadedConfigPath() {
    return GetManager().GetLoadedPath();
}

std::size_t GetLoadedOptionCount() {
    return GetManager().GetOptionCount();
}

} // namespace SwitchFrontend::TicoConfig
