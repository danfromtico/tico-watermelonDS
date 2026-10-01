#pragma once

#include <switch.h>

#include "VulkanDispatch.h"

// Presents software-rendered frames: the two 256x192 BGRA screens are
// uploaded to a texture and scaled into the swapchain with a blit, then the
// overlay (if any) draws on top in a render pass over the same image.
// Uses the emulator's VulkanContext device and its Vulkan dispatch table.
class SoftwarePresenter
{
public:
    struct Rect
    {
        bool Shown;
        s32 X, Y, Width, Height;
    };

    bool Init(NWindow* window);
    void Shutdown();

    // screens holds the top screen followed by the bottom screen
    bool Present(const u32* screens, const Rect& top, const Rect& bottom, bool linear);

    const char* LastError() const { return Error; }

private:
    bool Fail(const char* step, VkResult result);
    bool CreateSwapchain();
    bool CreateScreenTexture();
    bool CreateRenderPass();
    void Transition(VkImage image, VkImageLayout from, VkImageLayout to,
        VkAccessFlags srcAccess, VkAccessFlags dstAccess,
        VkPipelineStageFlags srcStage, VkPipelineStageFlags dstStage);

    static constexpr u32 kMaxSwapImages = 8;

    char Error[128] = "";
    bool ContextAcquired = false;

    VkInstance Instance = VK_NULL_HANDLE;
    VkPhysicalDevice PhysicalDevice = VK_NULL_HANDLE;
    VkDevice Device = VK_NULL_HANDLE;
    VkQueue Queue = VK_NULL_HANDLE;
    u32 QueueFamily = 0;
    VkSurfaceKHR Surface = VK_NULL_HANDLE;

    VkSwapchainKHR Swapchain = VK_NULL_HANDLE;
    VkFormat SwapFormat = VK_FORMAT_UNDEFINED;
    VkExtent2D SwapExtent {};
    VkImage SwapImages[kMaxSwapImages] {};
    VkImageView SwapViews[kMaxSwapImages] {};
    VkFramebuffer SwapFramebuffers[kMaxSwapImages] {};
    u32 SwapImageCount = 0;
    VkRenderPass OverlayRenderPass = VK_NULL_HANDLE;

    VkCommandPool CommandPool = VK_NULL_HANDLE;
    VkCommandBuffer CommandBuffer = VK_NULL_HANDLE;
    VkSemaphore AcquireSemaphore = VK_NULL_HANDLE;
    VkSemaphore RenderSemaphore = VK_NULL_HANDLE;
    VkFence FrameFence = VK_NULL_HANDLE;
    bool FrameInFlight = false;

    VkBuffer StagingBuffer = VK_NULL_HANDLE;
    VkDeviceMemory StagingMemory = VK_NULL_HANDLE;
    void* StagingMap = nullptr;

    VkImage ScreenImage = VK_NULL_HANDLE;
    VkDeviceMemory ScreenMemory = VK_NULL_HANDLE;
    bool ScreenImageInitialized = false;
};
