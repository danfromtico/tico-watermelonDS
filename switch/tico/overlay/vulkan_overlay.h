// Copyright 2026 Azahar Emulator Project
// Copyright 2026 Dan | ticoverse.com
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include <switch.h>

namespace SwitchFrontend::VulkanOverlay {

// Registers the overlay with the emulator's Vulkan presenter. The ImGui backend
// itself is created on the presentation thread the first time a frame is drawn.
bool Init();

// Reads the pad: Plus + Minus toggles the quick menu, and while it is open the
// d-pad, A and B navigate it.
void Update(PadState* pad);

bool IsVisible();
bool ShouldExit();
int ConsumeAction();

// How many presented frames the overlay has been given the chance to draw on.
// The caller uses it to notice that nothing is being presented while paused.
u64 GetDrawCount();

void Shutdown();

} // namespace SwitchFrontend::VulkanOverlay
