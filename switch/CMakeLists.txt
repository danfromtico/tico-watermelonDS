cmake_minimum_required(VERSION 3.16)

# Nintendo Switch test target: the melonDS core plus a single NWindow entrypoint.
# Configure with -DCMAKE_TOOLCHAIN_FILE=$DEVKITPRO/cmake/Switch.cmake (see build.sh).

project(WatermelonDS_nx VERSION 0.1.0 LANGUAGES C CXX ASM)

if (NOT NINTENDO_SWITCH)
    message(FATAL_ERROR "This target only builds with the devkitPro Switch toolchain")
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

set(MELONDS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../melonDS-android-lib")
list(APPEND CMAKE_MODULE_PATH "${MELONDS_ROOT}/cmake")

include(CheckLibraryExists)

set(ARCHITECTURE ARM64)
add_definitions(-DARCHITECTURE_ARM64=1)

set(ENABLE_JIT ON)
set(ENABLE_JIT_PROFILING OFF)
set(ENABLE_GDBSTUB OFF)
# The shared frontend code refers to the OpenGL renderers too; they build
# against Mesa's GLES headers but are never selected on the Switch.
set(ENABLE_OGLRENDERER ON)
add_compile_definitions(MELONDS_GL_HEADER=<GLES3/gl32.h> EGL_EGLEXT_PROTOTYPES MELONDS_ANDROID_DEBUG_BUILD=0)
set(ENABLE_VULKANRENDERER ON CACHE BOOL "" FORCE)

set(SWITCH_PORTLIBS "$ENV{DEVKITPRO}/portlibs/switch")

add_subdirectory("${MELONDS_ROOT}/src" core)

# The core calls Vulkan through its own function-pointer table. Those pointers
# are renamed (VulkanSwitchRename.h) because NVK is linked statically and
# already defines the real vk* symbols.
target_compile_definitions(core PUBLIC VK_NO_PROTOTYPES MELONDS_HAS_ADRENOTOOLS=0)

# Cheaper sound-capture stepping in the mixer (see SPU::Mix); the per-channel
# edge stepping costs about a quarter of the emulation thread on the Switch.
target_compile_definitions(core PRIVATE MELONDS_SPU_CAPTURE_EDGE_STEPPING)
target_compile_options(core PUBLIC -include "${CMAKE_CURRENT_SOURCE_DIR}/VulkanSwitchRename.h")
target_include_directories(core PUBLIC "${SWITCH_PORTLIBS}/include")

# The Android app's native frontend, built unchanged: the emulator driver and
# the Vulkan output/presentation pipeline. The Android-only pieces (JNI, Oboe,
# RetroAchievements, networking) are replaced by the files in this directory.
set(FRONTEND_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../app/src/main/cpp")

# Everything except an entrypoint: the Switch runtime, the glue, and the shared
# native frontend. Compiled once and linked into each NRO below.
add_library(watermelon_switch OBJECT
    SwitchEmulator.cpp
    Platform_Switch.cpp
    Frontend_Switch.cpp

    "${FRONTEND_DIR}/MelonInstance.cpp"
    "${FRONTEND_DIR}/RewindManager.cpp"
    "${FRONTEND_DIR}/SaveManager.cpp"
    "${FRONTEND_DIR}/EmulatorArgsBuilder.cpp"
    "${FRONTEND_DIR}/SDCardArgsBuilder.cpp"
    "${FRONTEND_DIR}/DSiSupport.cpp"
    "${FRONTEND_DIR}/CartLoader.cpp"
    "${FRONTEND_DIR}/ndz/NdzReader.cpp"
    "${FRONTEND_DIR}/ndz/NdzRomLoader.cpp"
    "${FRONTEND_DIR}/AudioOutputCapture.cpp"
    "${FRONTEND_DIR}/OpenGLContext.cpp"
    "${FRONTEND_DIR}/renderer/FrameQueue.cpp"
    "${FRONTEND_DIR}/renderer/ScreenshotRenderer.cpp"
    "${FRONTEND_DIR}/renderer/ShaderDiagnostics.cpp"
    "${FRONTEND_DIR}/renderer/VulkanOutput.cpp"
    "${FRONTEND_DIR}/renderer/VulkanSurfacePresenter.cpp"

    "${MELONDS_ROOT}/src/net/Net.cpp"
    "${MELONDS_ROOT}/src/net/PacketDispatcher.cpp")

target_include_directories(watermelon_switch PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/android_shim"
    "${FRONTEND_DIR}"
    "${FRONTEND_DIR}/renderer"
    "${FRONTEND_DIR}/retroachievements"
    "${FRONTEND_DIR}/rcheevos/include"
    "${FRONTEND_DIR}/zstd/lib"
    "${MELONDS_ROOT}/src/frontend"
    "${MELONDS_ROOT}/src/net/libslirp/src")

target_link_libraries(watermelon_switch PUBLIC core)

# NVK (Mesa) is a static ICD: keep its entry points and resolve the Mesa
# support archives in one rescan group.
function(watermelon_link_switch target)
    target_link_directories(${target} PRIVATE "${SWITCH_PORTLIBS}/lib")
    target_link_libraries(${target} PRIVATE
        watermelon_switch
        core
        -Wl,--wrap=threadCreate
        -Wl,-u,vk_icdGetInstanceProcAddr
        -Wl,-u,vk_icdNegotiateLoaderICDInterfaceVersion
        -Wl,--start-group
        -l:libGLESv2.a -l:libEGL.a -l:libglapi.a
        -l:libvulkan.a -l:libmesa_util_c11.a -l:libblake3.a -l:libmesa_util.a
        -l:libmesa_util_simd.a -l:libxmlconfig.a
        -lexpat -lzstd -lz -lnx -lstdc++ -lm
        -Wl,--end-group)
endfunction()

# Standalone test build: no UI, picks a ROM from sdmc:/switch/watermelonds/.
add_executable(watermelonds main.cpp)
watermelon_link_switch(watermelonds)

nx_generate_nacp(
    OUTPUT watermelonds.nacp
    NAME "WatermelonDS"
    AUTHOR "WatermelonDS"
    VERSION "${PROJECT_VERSION}")

nx_create_nro(watermelonds
    OUTPUT "${CMAKE_BINARY_DIR}/WatermelonDS.nro"
    NACP watermelonds.nacp)

# tico build: launched by the tico frontend. Mirrors tico-azahar's src/tico:
# the entrypoint, the ImGui overlay drawn through the Vulkan presenter, the
# per-core config and translations, with fonts, lang and assets packed as romfs.
add_library(imgui STATIC
    externals/imgui/imgui/imgui.cpp
    externals/imgui/imgui/imgui_draw.cpp
    externals/imgui/imgui/imgui_tables.cpp
    externals/imgui/imgui/imgui_widgets.cpp
    externals/imgui/imgui/backends/imgui_impl_vulkan.cpp)
target_include_directories(imgui PUBLIC
    externals/imgui
    externals/imgui/imgui
    externals/imgui/imgui/backends
    "${SWITCH_PORTLIBS}/include")
target_compile_definitions(imgui PUBLIC
    IMGUI_USER_CONFIG="imgui_user_config.h"
    IMGUI_IMPL_VULKAN_NO_PROTOTYPES
    IMGUI_DISABLE_DEFAULT_SHELL_FUNCTIONS
    VK_NO_PROTOTYPES)

add_executable(tico
    tico/main.cpp
    tico/overlay/overlay_ui.cpp
    tico/overlay/tico_config.cpp
    tico/overlay/translation_manager.cpp
    tico/overlay/vulkan_overlay.cpp)
set_target_properties(tico PROPERTIES OUTPUT_NAME "tico-watermelonds")
target_include_directories(tico PRIVATE externals/stb externals/json)
target_link_libraries(tico PRIVATE imgui)
watermelon_link_switch(tico)

# romfs is staged from the same folders tico-azahar keeps next to its sources
set(TICO_ROMFS_DIR "${CMAKE_BINARY_DIR}/tico_romfs")
set(TICO_ROMFS_SOURCES fonts lang assets config)
# nx_create_nro checks for the directory when configuring
file(MAKE_DIRECTORY "${TICO_ROMFS_DIR}")
add_custom_target(tico_romfs
    COMMAND "${CMAKE_COMMAND}" -E remove_directory "${TICO_ROMFS_DIR}"
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${TICO_ROMFS_DIR}"
    VERBATIM)
foreach(dir IN LISTS TICO_ROMFS_SOURCES)
    if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tico/${dir}")
        add_custom_command(TARGET tico_romfs POST_BUILD
            COMMAND "${CMAKE_COMMAND}" -E copy_directory
                "${CMAKE_CURRENT_SOURCE_DIR}/tico/${dir}" "${TICO_ROMFS_DIR}/${dir}"
            VERBATIM)
    endif()
endforeach()
add_dependencies(tico tico_romfs)

nx_generate_nacp(
    OUTPUT tico-watermelonds.nacp
    NAME "tico WatermelonDS"
    AUTHOR "ticoverse.com"
    VERSION "${PROJECT_VERSION}")

nx_create_nro(tico
    OUTPUT "${CMAKE_BINARY_DIR}/tico-watermelonds.nro"
    NACP tico-watermelonds.nacp
    ROMFS "${TICO_ROMFS_DIR}")
