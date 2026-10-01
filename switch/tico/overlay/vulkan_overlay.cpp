// Copyright 2026 Azahar Emulator Project
// Copyright 2026 Dan | ticoverse.com
// SPDX-License-Identifier: GPL-2.0-or-later

// VulkanDispatch.h provides the emulator's Vulkan function-pointer table (the
// build is VK_NO_PROTOTYPES) and must come before the ImGui Vulkan backend so
// the backend's <vulkan/vulkan.h> sees the same prototype-less configuration.
#include "VulkanDispatch.h"

#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <vector>

#include <imgui.h>
#include <imgui_impl_vulkan.h>
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include "VulkanContext.h"
#include "renderer/VulkanSurfacePresenter.h"

#include "SwitchFrontend.h"
#include "tico/overlay/overlay_ui.h"
#include "tico/overlay/vulkan_overlay.h"

namespace SwitchFrontend::VulkanOverlay {
namespace {

using OverlayDrawContext = MelonDSAndroid::VulkanSurfacePresenter::OverlayDrawContext;

constexpr const char* TAG = "[tico-overlay]";
constexpr std::array<const char*, 3> kFontPaths = {{
    "romfs:/fonts/font.ttf",
    "sdmc:/tico/fonts/font.ttf",
    "sdmc:/tico/system/nds/fonts/font.ttf",
}};
constexpr std::array<const char*, 6> kAvatarPaths = {{
    "sdmc:/tico/assets/avatar.jpg",
    "sdmc:/tico/assets/avatar.jpeg",
    "sdmc:/tico/assets/avatar.png",
    "romfs:/assets/avatar.jpg",
    "romfs:/assets/avatar.jpeg",
    "romfs:/assets/avatar.png",
}};

std::atomic_bool s_initialized{false};
std::atomic_bool s_visible{false};
std::atomic_bool s_exit_requested{false};
std::atomic_int s_pending_action{0};
std::atomic_uint s_pending_nav_mask{0};
std::atomic<u64> s_draw_count{0};

bool s_was_combo_down = false;
bool s_psm_initialized = false;

// Present-thread-only state (touched exclusively inside the draw callback, or
// in Shutdown after the callback has been removed).
bool s_backend_ready = false;
VkInstance s_instance = VK_NULL_HANDLE;
VkPhysicalDevice s_physical_device = VK_NULL_HANDLE;
VkDevice s_device = VK_NULL_HANDLE;
VkQueue s_queue = VK_NULL_HANDLE;
u32 s_queue_family = 0;
VkFormat s_color_format = VK_FORMAT_UNDEFINED;

VkDescriptorPool s_descriptor_pool = VK_NULL_HANDLE;
VkCommandPool s_command_pool = VK_NULL_HANDLE;

struct OverlayTextureResource {
    VkImage image = VK_NULL_HANDLE;
    VkDeviceMemory memory = VK_NULL_HANDLE;
    VkImageView view = VK_NULL_HANDLE;
    VkSampler sampler = VK_NULL_HANDLE;
    VkDescriptorSet descriptor = VK_NULL_HANDLE;
};
std::vector<OverlayTextureResource> s_overlay_textures;
bool s_avatar_load_attempted = false;

struct NavPrev {
    bool up;
    bool down;
    bool left;
    bool right;
    bool a;
    bool b;
};
NavPrev s_nav_prev{};

enum NavBits : unsigned int {
    NavBit_Up = 1u << 0,
    NavBit_Down = 1u << 1,
    NavBit_Left = 1u << 2,
    NavBit_Right = 1u << 3,
    NavBit_Accept = 1u << 4,
    NavBit_Cancel = 1u << 5,
};

// The emulator's renderer threads submit to the same queues, so every queue
// operation the overlay or the ImGui backend makes takes the emulator's lock.
std::mutex& QueueLockFor(VkQueue queue) {
    melonDS::VulkanContext& context = melonDS::VulkanContext::Get();
    return queue == context.GetPresentQueue() ? context.GetPresentQueueLock()
                                              : context.GetQueueLock();
}

VKAPI_ATTR VkResult VKAPI_CALL LockedQueueSubmit(VkQueue queue, uint32_t submit_count,
                                                 const VkSubmitInfo* submits, VkFence fence) {
    std::scoped_lock lock(QueueLockFor(queue));
    return vkQueueSubmit(queue, submit_count, submits, fence);
}

VKAPI_ATTR VkResult VKAPI_CALL LockedQueueWaitIdle(VkQueue queue) {
    std::scoped_lock lock(QueueLockFor(queue));
    return vkQueueWaitIdle(queue);
}

PFN_vkVoidFunction OverlayLoader(const char* name, void* user_data) {
    if (std::strcmp(name, "vkQueueSubmit") == 0) {
        return reinterpret_cast<PFN_vkVoidFunction>(&LockedQueueSubmit);
    }
    if (std::strcmp(name, "vkQueueWaitIdle") == 0) {
        return reinterpret_cast<PFN_vkVoidFunction>(&LockedQueueWaitIdle);
    }
    if (!vkGetInstanceProcAddr) {
        return nullptr;
    }
    return vkGetInstanceProcAddr(static_cast<VkInstance>(user_data), name);
}

bool CreateDescriptorPool() {
    VkDescriptorPoolSize pool_size{};
    pool_size.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    pool_size.descriptorCount = 64;

    VkDescriptorPoolCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    info.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    info.maxSets = 64;
    info.poolSizeCount = 1;
    info.pPoolSizes = &pool_size;
    return vkCreateDescriptorPool(s_device, &info, nullptr, &s_descriptor_pool) == VK_SUCCESS;
}

bool CreateCommandPool() {
    VkCommandPoolCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    info.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT |
                 VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    info.queueFamilyIndex = s_queue_family;
    return vkCreateCommandPool(s_device, &info, nullptr, &s_command_pool) == VK_SUCCESS;
}

bool FindMemoryType(u32 type_filter, VkMemoryPropertyFlags properties, u32& out_index) {
    VkPhysicalDeviceMemoryProperties memory_properties{};
    vkGetPhysicalDeviceMemoryProperties(s_physical_device, &memory_properties);
    for (u32 i = 0; i < memory_properties.memoryTypeCount; ++i) {
        if ((type_filter & (1u << i)) &&
            (memory_properties.memoryTypes[i].propertyFlags & properties) == properties) {
            out_index = i;
            return true;
        }
    }
    return false;
}

void TransitionImage(VkCommandBuffer cmd, VkImage image, VkImageLayout old_layout,
                     VkImageLayout new_layout, VkAccessFlags src_access, VkAccessFlags dst_access,
                     VkPipelineStageFlags src_stage, VkPipelineStageFlags dst_stage) {
    VkImageMemoryBarrier barrier{};
    barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    barrier.srcAccessMask = src_access;
    barrier.dstAccessMask = dst_access;
    barrier.oldLayout = old_layout;
    barrier.newLayout = new_layout;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.image = image;
    barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    barrier.subresourceRange.levelCount = 1;
    barrier.subresourceRange.layerCount = 1;
    vkCmdPipelineBarrier(cmd, src_stage, dst_stage, VK_DEPENDENCY_BY_REGION_BIT, 0, nullptr, 0,
                         nullptr, 1, &barrier);
}

void DestroyOverlayTexture(OverlayTextureResource& texture) {
    if (texture.descriptor && s_backend_ready) {
        ImGui_ImplVulkan_RemoveTexture(texture.descriptor);
        texture.descriptor = VK_NULL_HANDLE;
    }
    if (texture.sampler) {
        vkDestroySampler(s_device, texture.sampler, nullptr);
        texture.sampler = VK_NULL_HANDLE;
    }
    if (texture.view) {
        vkDestroyImageView(s_device, texture.view, nullptr);
        texture.view = VK_NULL_HANDLE;
    }
    if (texture.image) {
        vkDestroyImage(s_device, texture.image, nullptr);
        texture.image = VK_NULL_HANDLE;
    }
    if (texture.memory) {
        vkFreeMemory(s_device, texture.memory, nullptr);
        texture.memory = VK_NULL_HANDLE;
    }
}

void DestroyOverlayTextures() {
    for (auto& texture : s_overlay_textures) {
        DestroyOverlayTexture(texture);
    }
    s_overlay_textures.clear();
    OverlayUI::SetAvatarTextureId(0);
}

unsigned long long CreateOverlayTextureRGBA(const unsigned char* rgba, int width, int height) {
    if (!rgba || width <= 0 || height <= 0 || !s_backend_ready || !s_command_pool) {
        return 0;
    }

    const VkDeviceSize upload_size =
        static_cast<VkDeviceSize>(width) * static_cast<VkDeviceSize>(height) * 4;

    VkBuffer staging_buffer = VK_NULL_HANDLE;
    VkDeviceMemory staging_memory = VK_NULL_HANDLE;
    VkCommandBuffer command_buffer = VK_NULL_HANDLE;
    OverlayTextureResource texture{};

    auto cleanup = [&] {
        if (command_buffer)
            vkFreeCommandBuffers(s_device, s_command_pool, 1, &command_buffer);
        if (staging_buffer)
            vkDestroyBuffer(s_device, staging_buffer, nullptr);
        if (staging_memory)
            vkFreeMemory(s_device, staging_memory, nullptr);
        DestroyOverlayTexture(texture);
    };
    auto fail = [&](const char* what) -> unsigned long long {
        SwitchFrontend::Log("%s avatar upload failed: %s\n", TAG, what);
        cleanup();
        return 0;
    };

    VkBufferCreateInfo buffer_info{};
    buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    buffer_info.size = upload_size;
    buffer_info.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    if (vkCreateBuffer(s_device, &buffer_info, nullptr, &staging_buffer) != VK_SUCCESS) {
        return fail("staging buffer");
    }

    VkMemoryRequirements buffer_requirements{};
    vkGetBufferMemoryRequirements(s_device, staging_buffer, &buffer_requirements);
    VkMemoryAllocateInfo buffer_alloc_info{};
    buffer_alloc_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    buffer_alloc_info.allocationSize = buffer_requirements.size;
    if (!FindMemoryType(buffer_requirements.memoryTypeBits,
                        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                        buffer_alloc_info.memoryTypeIndex)) {
        return fail("no host-visible coherent memory");
    }
    void* mapped = nullptr;
    if (vkAllocateMemory(s_device, &buffer_alloc_info, nullptr, &staging_memory) != VK_SUCCESS ||
        vkBindBufferMemory(s_device, staging_buffer, staging_memory, 0) != VK_SUCCESS ||
        vkMapMemory(s_device, staging_memory, 0, upload_size, 0, &mapped) != VK_SUCCESS) {
        return fail("staging memory");
    }
    std::memcpy(mapped, rgba, static_cast<std::size_t>(upload_size));
    vkUnmapMemory(s_device, staging_memory);

    VkImageCreateInfo image_info{};
    image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    image_info.imageType = VK_IMAGE_TYPE_2D;
    image_info.format = VK_FORMAT_R8G8B8A8_UNORM;
    image_info.extent = {static_cast<u32>(width), static_cast<u32>(height), 1};
    image_info.mipLevels = 1;
    image_info.arrayLayers = 1;
    image_info.samples = VK_SAMPLE_COUNT_1_BIT;
    image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
    image_info.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    if (vkCreateImage(s_device, &image_info, nullptr, &texture.image) != VK_SUCCESS) {
        return fail("image");
    }

    VkMemoryRequirements image_requirements{};
    vkGetImageMemoryRequirements(s_device, texture.image, &image_requirements);
    VkMemoryAllocateInfo image_alloc_info{};
    image_alloc_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    image_alloc_info.allocationSize = image_requirements.size;
    if (!FindMemoryType(image_requirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                        image_alloc_info.memoryTypeIndex)) {
        return fail("no device-local memory");
    }
    if (vkAllocateMemory(s_device, &image_alloc_info, nullptr, &texture.memory) != VK_SUCCESS ||
        vkBindImageMemory(s_device, texture.image, texture.memory, 0) != VK_SUCCESS) {
        return fail("image memory");
    }

    VkImageViewCreateInfo view_info{};
    view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    view_info.image = texture.image;
    view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
    view_info.format = VK_FORMAT_R8G8B8A8_UNORM;
    view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    view_info.subresourceRange.levelCount = 1;
    view_info.subresourceRange.layerCount = 1;
    if (vkCreateImageView(s_device, &view_info, nullptr, &texture.view) != VK_SUCCESS) {
        return fail("image view");
    }

    VkSamplerCreateInfo sampler_info{};
    sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    sampler_info.magFilter = VK_FILTER_LINEAR;
    sampler_info.minFilter = VK_FILTER_LINEAR;
    sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    if (vkCreateSampler(s_device, &sampler_info, nullptr, &texture.sampler) != VK_SUCCESS) {
        return fail("sampler");
    }

    VkCommandBufferAllocateInfo command_alloc_info{};
    command_alloc_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    command_alloc_info.commandPool = s_command_pool;
    command_alloc_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    command_alloc_info.commandBufferCount = 1;
    if (vkAllocateCommandBuffers(s_device, &command_alloc_info, &command_buffer) != VK_SUCCESS) {
        return fail("command buffer");
    }

    VkCommandBufferBeginInfo begin_info{};
    begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    begin_info.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(command_buffer, &begin_info);

    TransitionImage(command_buffer, texture.image, VK_IMAGE_LAYOUT_UNDEFINED,
                    VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 0, VK_ACCESS_TRANSFER_WRITE_BIT,
                    VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT);

    VkBufferImageCopy copy_region{};
    copy_region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    copy_region.imageSubresource.layerCount = 1;
    copy_region.imageExtent = {static_cast<u32>(width), static_cast<u32>(height), 1};
    vkCmdCopyBufferToImage(command_buffer, staging_buffer, texture.image,
                           VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy_region);

    TransitionImage(command_buffer, texture.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_ACCESS_TRANSFER_WRITE_BIT,
                    VK_ACCESS_SHADER_READ_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
                    VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);

    vkEndCommandBuffer(command_buffer);

    VkSubmitInfo submit_info{};
    submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submit_info.commandBufferCount = 1;
    submit_info.pCommandBuffers = &command_buffer;
    if (LockedQueueSubmit(s_queue, 1, &submit_info, VK_NULL_HANDLE) != VK_SUCCESS) {
        return fail("queue submit");
    }
    LockedQueueWaitIdle(s_queue);

    vkFreeCommandBuffers(s_device, s_command_pool, 1, &command_buffer);
    command_buffer = VK_NULL_HANDLE;
    vkDestroyBuffer(s_device, staging_buffer, nullptr);
    staging_buffer = VK_NULL_HANDLE;
    vkFreeMemory(s_device, staging_memory, nullptr);
    staging_memory = VK_NULL_HANDLE;

    texture.descriptor = ImGui_ImplVulkan_AddTexture(texture.sampler, texture.view,
                                                     VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    const unsigned long long texture_id =
        static_cast<unsigned long long>(reinterpret_cast<std::uintptr_t>(texture.descriptor));
    s_overlay_textures.push_back(texture);
    return texture_id;
}

bool LoadAvatarFromRGBA(const unsigned char* rgba, int width, int height, const char* source) {
    const unsigned long long texture_id = CreateOverlayTextureRGBA(rgba, width, height);
    if (texture_id == 0) {
        return false;
    }
    OverlayUI::SetAvatarTextureId(texture_id);
    SwitchFrontend::Log("%s loaded avatar: %s (%dx%d)\n", TAG, source, width, height);
    return true;
}

bool LoadAvatarFromFile(const char* path) {
    int width = 0;
    int height = 0;
    int channels = 0;
    unsigned char* rgba = stbi_load(path, &width, &height, &channels, 4);
    if (!rgba) {
        return false;
    }
    const bool loaded = LoadAvatarFromRGBA(rgba, width, height, path);
    stbi_image_free(rgba);
    return loaded;
}

bool LoadAvatarFromMemory(const unsigned char* data, std::size_t size, const char* source) {
    int width = 0;
    int height = 0;
    int channels = 0;
    unsigned char* rgba =
        stbi_load_from_memory(data, static_cast<int>(size), &width, &height, &channels, 4);
    if (!rgba) {
        return false;
    }
    const bool loaded = LoadAvatarFromRGBA(rgba, width, height, source);
    stbi_image_free(rgba);
    return loaded;
}

bool LoadAvatarFromAccount() {
    if (R_FAILED(accountInitialize(AccountServiceType_Application))) {
        return false;
    }

    AccountUid uid{};
    bool found = false;
    if (R_SUCCEEDED(accountGetPreselectedUser(&uid)) && accountUidIsValid(&uid)) {
        found = true;
    }
    if (!found && R_SUCCEEDED(accountGetLastOpenedUser(&uid)) && accountUidIsValid(&uid)) {
        found = true;
    }
    if (!found) {
        s32 user_count = 0;
        if (R_SUCCEEDED(accountGetUserCount(&user_count)) && user_count > 0) {
            AccountUid uids[ACC_USER_LIST_SIZE]{};
            s32 actual_total = 0;
            if (R_SUCCEEDED(accountListAllUsers(uids, ACC_USER_LIST_SIZE, &actual_total)) &&
                actual_total > 0) {
                uid = uids[0];
                found = accountUidIsValid(&uid);
            }
        }
    }

    bool loaded = false;
    if (found) {
        AccountProfile profile{};
        AccountProfileBase profile_base{};
        if (R_SUCCEEDED(accountGetProfile(&profile, uid))) {
            if (R_SUCCEEDED(accountProfileGet(&profile, nullptr, &profile_base)) &&
                profile_base.nickname[0] != '\0') {
                OverlayUI::SetNickname(profile_base.nickname);
            }

            u32 image_size = 0;
            if (R_SUCCEEDED(accountProfileGetImageSize(&profile, &image_size)) && image_size > 0) {
                std::vector<unsigned char> jpeg_data(image_size);
                u32 actual_size = 0;
                if (R_SUCCEEDED(accountProfileLoadImage(&profile, jpeg_data.data(), image_size,
                                                        &actual_size)) &&
                    actual_size > 0) {
                    loaded = LoadAvatarFromMemory(jpeg_data.data(), actual_size,
                                                  "switch-account-avatar");
                }
            }
            accountProfileClose(&profile);
        }
    }

    accountExit();
    return loaded;
}

void LoadAvatarTexture() {
    if (s_avatar_load_attempted) {
        return;
    }
    s_avatar_load_attempted = true;

    for (const char* path : kAvatarPaths) {
        if (LoadAvatarFromFile(path)) {
            return;
        }
    }
    if (LoadAvatarFromAccount()) {
        return;
    }
    SwitchFrontend::Log("%s no avatar image found\n", TAG);
}

void DestroyBackend() {
    DestroyOverlayTextures();
    if (s_backend_ready) {
        ImGui_ImplVulkan_Shutdown();
        s_backend_ready = false;
    }
    if (s_descriptor_pool) {
        vkDestroyDescriptorPool(s_device, s_descriptor_pool, nullptr);
        s_descriptor_pool = VK_NULL_HANDLE;
    }
    if (s_command_pool) {
        vkDestroyCommandPool(s_device, s_command_pool, nullptr);
        s_command_pool = VK_NULL_HANDLE;
    }
    s_color_format = VK_FORMAT_UNDEFINED;
    s_avatar_load_attempted = false;
}

// The backend draws inside the presenter's own render pass, so it is built
// against that pass. A swapchain that comes back with a different format makes
// the ImGui pipeline incompatible and the backend is rebuilt.
bool EnsureBackend(const OverlayDrawContext& context) {
    if (s_backend_ready && context.format == s_color_format && context.device == s_device) {
        return true;
    }
    if (s_backend_ready) {
        std::scoped_lock lock(QueueLockFor(s_queue));
        vkQueueWaitIdle(s_queue);
    }
    DestroyBackend();

    s_instance = context.instance;
    s_physical_device = context.physicalDevice;
    s_device = context.device;
    s_queue = context.queue;
    s_queue_family = context.queueFamilyIndex;

    if (!CreateCommandPool() || !CreateDescriptorPool()) {
        SwitchFrontend::Log("%s could not create the Vulkan pools\n", TAG);
        DestroyBackend();
        return false;
    }

    if (!ImGui_ImplVulkan_LoadFunctions(VK_API_VERSION_1_1, OverlayLoader, s_instance)) {
        SwitchFrontend::Log("%s ImGui_ImplVulkan_LoadFunctions failed\n", TAG);
        DestroyBackend();
        return false;
    }

    const u32 image_count = context.imageCount >= 2 ? context.imageCount : 2;
    ImGui_ImplVulkan_InitInfo init_info{};
    init_info.ApiVersion = VK_API_VERSION_1_1;
    init_info.Instance = s_instance;
    init_info.PhysicalDevice = s_physical_device;
    init_info.Device = s_device;
    init_info.QueueFamily = s_queue_family;
    init_info.Queue = s_queue;
    init_info.DescriptorPool = s_descriptor_pool;
    init_info.RenderPass = context.renderPass;
    init_info.MinImageCount = image_count;
    init_info.ImageCount = image_count;
    init_info.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
    init_info.PipelineCache = VK_NULL_HANDLE;
    init_info.Subpass = 0;

    if (!ImGui_ImplVulkan_Init(&init_info)) {
        SwitchFrontend::Log("%s ImGui_ImplVulkan_Init failed\n", TAG);
        DestroyBackend();
        return false;
    }
    s_backend_ready = true;
    s_color_format = context.format;
    SwitchFrontend::Log("%s ImGui backend ready (format=%d, images=%u)\n", TAG,
                        static_cast<int>(context.format), image_count);
    LoadAvatarTexture();
    return true;
}

// Runs on the presentation thread, inside the presenter's render pass, after
// the emulator's screens have been drawn into the swapchain image.
void DrawCallback(const OverlayDrawContext& context) {
    if (!s_initialized.load()) {
        return;
    }
    s_draw_count.fetch_add(1);
    if (!EnsureBackend(context)) {
        return;
    }

    const bool visible = s_visible.load();
    const bool has_transient = OverlayUI::HasTransientContent();
    if (!visible && !has_transient) {
        return;
    }

    ImGui_ImplVulkan_NewFrame();

    ImGuiIO& io = ImGui::GetIO();
    io.DisplaySize = ImVec2(static_cast<float>(context.extent.width),
                            static_cast<float>(context.extent.height));
    io.DeltaTime = 1.0f / 60.0f;

    const unsigned int nav_mask = s_pending_nav_mask.exchange(0);
    OverlayUI::FeedNav({
        .up = (nav_mask & NavBit_Up) != 0,
        .down = (nav_mask & NavBit_Down) != 0,
        .left = (nav_mask & NavBit_Left) != 0,
        .right = (nav_mask & NavBit_Right) != 0,
        .accept = (nav_mask & NavBit_Accept) != 0,
        .cancel = (nav_mask & NavBit_Cancel) != 0,
    });

    ImGui::NewFrame();
    const OverlayUI::Action action = OverlayUI::Render(static_cast<int>(context.extent.width),
                                                       static_cast<int>(context.extent.height));
    ImGui::Render();

    if (action != OverlayUI::Action::None) {
        s_pending_action.store(static_cast<int>(action));
        if (action == OverlayUI::Action::Exit) {
            s_exit_requested.store(true);
        }
        if (action == OverlayUI::Action::Resume || action == OverlayUI::Action::Exit ||
            OverlayUI::IsSaveStateAction(action) || OverlayUI::IsLoadStateAction(action)) {
            s_visible.store(false);
            s_pending_nav_mask.store(0);
            OverlayUI::SetVisible(false);
        }
    }

    ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), context.commandBuffer);
}

} // namespace

bool Init() {
    if (s_initialized.load()) {
        return true;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.LogFilename = nullptr;
    io.ConfigFlags |= ImGuiConfigFlags_NoMouseCursorChange;

    for (const char* font_path : kFontPaths) {
        if (ImFont* font = io.Fonts->AddFontFromFileTTF(font_path, 32.0f)) {
            io.FontDefault = font;
            SwitchFrontend::Log("%s loaded font: %s\n", TAG, font_path);
            break;
        }
    }
    if (!io.FontDefault) {
        SwitchFrontend::Log("%s could not load overlay font, using ImGui default\n", TAG);
    }
    ImGui::StyleColorsDark();

    if (!s_psm_initialized && R_SUCCEEDED(psmInitialize())) {
        s_psm_initialized = true;
    }

    s_visible.store(false);
    s_exit_requested.store(false);
    s_pending_action.store(0);
    s_pending_nav_mask.store(0);
    s_draw_count.store(0);
    s_backend_ready = false;
    s_avatar_load_attempted = false;

    // Register the present-time hook last so no callback fires mid-initialization.
    s_initialized.store(true);
    MelonDSAndroid::VulkanSurfacePresenter::setOverlayDrawCallback(&DrawCallback);

    SwitchFrontend::Log("%s initialized\n", TAG);
    return true;
}

void Update(PadState* pad) {
    if (!s_initialized.load() || !pad) {
        return;
    }

    const u64 held = padGetButtons(pad);
    const bool plus = (held & HidNpadButton_Plus) != 0;
    const bool minus = (held & HidNpadButton_Minus) != 0;
    const bool combo_down = plus && minus;

    if (combo_down && !s_was_combo_down) {
        const bool new_visible = !s_visible.load();
        s_visible.store(new_visible);
        if (!new_visible) {
            s_pending_nav_mask.store(0);
        }
    }
    s_was_combo_down = combo_down;

    const bool visible = s_visible.load();
    OverlayUI::SetVisible(visible);

    const bool up = (held & (HidNpadButton_Up | HidNpadButton_StickLUp)) != 0;
    const bool down = (held & (HidNpadButton_Down | HidNpadButton_StickLDown)) != 0;
    const bool left = (held & (HidNpadButton_Left | HidNpadButton_StickLLeft)) != 0;
    const bool right = (held & (HidNpadButton_Right | HidNpadButton_StickLRight)) != 0;
    const bool a = (held & HidNpadButton_A) != 0;
    const bool b = (held & HidNpadButton_B) != 0;

    unsigned int nav_mask = 0;
    if (up && !s_nav_prev.up)
        nav_mask |= NavBit_Up;
    if (down && !s_nav_prev.down)
        nav_mask |= NavBit_Down;
    if (left && !s_nav_prev.left)
        nav_mask |= NavBit_Left;
    if (right && !s_nav_prev.right)
        nav_mask |= NavBit_Right;
    if (a && !s_nav_prev.a)
        nav_mask |= NavBit_Accept;
    if (b && !s_nav_prev.b)
        nav_mask |= NavBit_Cancel;

    if (visible && nav_mask != 0) {
        s_pending_nav_mask.fetch_or(nav_mask);
    }

    s_nav_prev = {up, down, left, right, a, b};
}

bool IsVisible() {
    return s_visible.load();
}

bool ShouldExit() {
    return s_exit_requested.load();
}

int ConsumeAction() {
    return s_pending_action.exchange(0);
}

u64 GetDrawCount() {
    return s_draw_count.load();
}

void Shutdown() {
    if (!s_initialized.load()) {
        return;
    }

    // Stop the presentation thread from calling the draw callback, then tear down.
    MelonDSAndroid::VulkanSurfacePresenter::setOverlayDrawCallback(nullptr);
    s_initialized.store(false);

    if (s_device && s_queue) {
        std::scoped_lock lock(QueueLockFor(s_queue));
        vkQueueWaitIdle(s_queue);
    }

    DestroyBackend();
    ImGui::DestroyContext();

    OverlayUI::SetVisible(false);
    OverlayUI::ShowToast(std::string{});

    s_visible.store(false);
    s_exit_requested.store(false);
    s_pending_action.store(0);
    s_pending_nav_mask.store(0);
    s_device = VK_NULL_HANDLE;
    s_queue = VK_NULL_HANDLE;

    if (s_psm_initialized) {
        psmExit();
        s_psm_initialized = false;
    }
}

} // namespace SwitchFrontend::VulkanOverlay
