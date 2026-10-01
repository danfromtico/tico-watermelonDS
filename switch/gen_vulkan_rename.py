#!/usr/bin/env python3
"""Regenerates VulkanSwitchRename.h from the core's VulkanDispatch.h."""
import pathlib
import re

root = pathlib.Path(__file__).resolve().parent
header = (root / "../melonDS-android-lib/src/VulkanDispatch.h").read_text()
names = [a for a, b in re.findall(r"extern PFN_(vk\w+) (vk\w+);", header) if a == b]

lines = [
    "// Generated from VulkanDispatch.h by switch/gen_vulkan_rename.py - do not edit.",
    "//",
    "// NVK is linked statically on the Switch, so libvulkan.a already defines the",
    "// real vk* functions. The core keeps its own table of function pointers under",
    "// the same names; this header gives those pointers private names so the two",
    "// do not collide at link time.",
    "#pragma once",
    "",
]
lines += [f"#define {name} melon_{name}" for name in names]
(root / "VulkanSwitchRename.h").write_text("\n".join(lines) + "\n")
