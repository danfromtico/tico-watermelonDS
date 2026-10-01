/*
    Copyright 2016-2025 melonDS team

    This file is part of melonDS.

    melonDS is free software: you can redistribute it and/or modify it under
    the terms of the GNU General Public License as published by the Free
    Software Foundation, either version 3 of the License, or (at your option)
    any later version.

    melonDS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with melonDS. If not, see http://www.gnu.org/licenses/.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>
#include <cassert>
#include <limits>
#include "NDS.h"
#include "GPU.h"

#include "ARMJIT.h"

#include "GPU2D_Soft.h"
#include "GPU3D.h"
#include "VulkanPerfStats.h"

namespace MelonDSAndroid
{
bool areRendererDebugToolsEnabled();
bool isVulkanGpu2DPerfLoggingEnabled();
}

namespace melonDS
{
using Platform::Log;
using Platform::LogLevel;

#define LINE_CYCLES  (355*6)
#define HBLANK_CYCLES (48+(256*6))
#define FRAME_CYCLES  (LINE_CYCLES * 263)

enum
{
    LCD_StartHBlank = 0,
    LCD_StartScanline,
    LCD_FinishFrame,
};

namespace
{
PerfSampleWindow<120> gStructured2DDrawWindow;
PerfSampleWindow<120> gStructured2DSpriteWindow;
PerfSampleWindow<120> gStructured2DDrawAWindow;
PerfSampleWindow<120> gStructured2DDrawBWindow;
PerfSampleWindow<120> gStructured2DSpriteAWindow;
PerfSampleWindow<120> gStructured2DSpriteBWindow;
u64 gStructured2DFrameDrawNs = 0;
u64 gStructured2DFrameSpriteNs = 0;
u64 gStructured2DFrameDrawANs = 0;
u64 gStructured2DFrameDrawBNs = 0;
u64 gStructured2DFrameSpriteANs = 0;
u64 gStructured2DFrameSpriteBNs = 0;

bool ShouldMeasureStructured2D(const GPU& gpu)
{
    return MelonDSAndroid::isVulkanGpu2DPerfLoggingEnabled()
        && gpu.GPU3D.GetCurrentRenderer().UsesStructured2DMetadata();
}

void LogStructured2DPerfIfReady()
{
    if (!gStructured2DDrawWindow.Ready()
        || !gStructured2DSpriteWindow.Ready()
        || !gStructured2DDrawAWindow.Ready()
        || !gStructured2DDrawBWindow.Ready()
        || !gStructured2DSpriteAWindow.Ready()
        || !gStructured2DSpriteBWindow.Ready())
    {
        return;
    }

    const auto drawSummary = gStructured2DDrawWindow.SummarizeAndReset();
    const auto spriteSummary = gStructured2DSpriteWindow.SummarizeAndReset();
    const auto drawASummary = gStructured2DDrawAWindow.SummarizeAndReset();
    const auto drawBSummary = gStructured2DDrawBWindow.SummarizeAndReset();
    const auto spriteASummary = gStructured2DSpriteAWindow.SummarizeAndReset();
    const auto spriteBSummary = gStructured2DSpriteBWindow.SummarizeAndReset();
    Platform::Log(
        Platform::LogLevel::Warn,
        "VulkanPerf[GPU2D]: draw cpu avg=%.3fms p95=%.3fms max=%.3fms a=%.3f/%.3fms b=%.3f/%.3fms sprites cpu avg=%.3fms p95=%.3fms max=%.3fms a=%.3f/%.3fms b=%.3f/%.3fms",
        PerfNsToMs(drawSummary.MeanNs),
        PerfNsToMs(drawSummary.P95Ns),
        PerfNsToMs(drawSummary.MaxNs),
        PerfNsToMs(drawASummary.MeanNs),
        PerfNsToMs(drawASummary.P95Ns),
        PerfNsToMs(drawBSummary.MeanNs),
        PerfNsToMs(drawBSummary.P95Ns),
        PerfNsToMs(spriteSummary.MeanNs),
        PerfNsToMs(spriteSummary.P95Ns),
        PerfNsToMs(spriteSummary.MaxNs),
        PerfNsToMs(spriteASummary.MeanNs),
        PerfNsToMs(spriteASummary.P95Ns),
        PerfNsToMs(spriteBSummary.MeanNs),
        PerfNsToMs(spriteBSummary.P95Ns)
    );
}
}


/*
    VRAM invalidation tracking

    - we want to know when a VRAM region used for graphics changed
    - for some regions unmapping is mandatory to modify them (Texture, TexPal and ExtPal) and
        we don't want to completely invalidate them every time they're unmapped and remapped

    For this reason we don't track the dirtyness per mapping region, but instead per VRAM bank
    with VRAMDirty.

    This is more or less a description of VRAMTrackingSet::DeriveState
        Each time before the memory is read two things could have happened
        to each 16kb piece (16kb is the smallest unit in which mappings can
        be made thus also the size VRAMMap_* use):
            - this piece was remapped compared to last time we checked,
                which means this location in memory is invalid.
            - this piece wasn't remapped, which means we need to check whether
                it was changed. This can be archived by checking VRAMDirty.
                VRAMDirty need to be reset for the respective VRAM bank.
*/

GPU::GPU(melonDS::NDS& nds, std::unique_ptr<Renderer3D>&& renderer3d, std::unique_ptr<GPU2D::Renderer2D>&& renderer2d) noexcept :
    NDS(nds),
    GPU2D_A(0, *this),
    GPU2D_B(1, *this),
    GPU3D(nds, renderer3d ? std::move(renderer3d) : std::make_unique<SoftRenderer>()),
    GPU2D_Renderer(renderer2d ? std::move(renderer2d) : std::make_unique<GPU2D::SoftRenderer>(*this))
{
    for (u32 bank = 0; bank < 9; bank++)
    {
        const size_t halfwordCount = (static_cast<size_t>(VRAMMask[bank]) + 1u) / 2u;
        FaithfulVramCaptureTags[bank] =
            std::make_unique<FaithfulVramCaptureTag[]>(halfwordCount);
        assert((halfwordCount % FaithfulVramSummaryPageHalfwords) == 0u);
        const size_t summaryPageCount =
            halfwordCount / FaithfulVramSummaryPageHalfwords;
        FaithfulVramCapturePageSummaries[bank] =
            std::make_unique<FaithfulVramCaptureSpan[]>(summaryPageCount);
    }

    NDS.RegisterEventFuncs(Event_LCD, this,
    {
            MakeEventThunk(GPU, StartHBlank),
            MakeEventThunk(GPU, StartScanline),
            MakeEventThunk(GPU, FinishFrame)
    });
    NDS.RegisterEventFuncs(Event_DisplayFIFO, this, {MakeEventThunk(GPU, DisplayFIFO)});

    InitFramebuffers();

    SetThreaded2D(DefaultThreaded2D);
}

u64 GPU::MintFaithfulCaptureProductId() noexcept
{
    if (!FaithfulVramCaptureTrackingEnabled)
        return 0;
    if (FaithfulCaptureProductCounter == std::numeric_limits<u64>::max())
        return 0;
    return ++FaithfulCaptureProductCounter;
}

u32 GPU::GetFaithfulCaptureProductPhysicalRefCount(
    u64 productEpoch, u64 productId) const noexcept
{
    if (!FaithfulVramCaptureTrackingEnabled
        || productEpoch == 0u
        || productEpoch != FaithfulCaptureProductEpoch
        || productId == 0u)
    {
        return 0u;
    }

    if (FaithfulCaptureProductPhysicalRefsDirty)
        return std::numeric_limits<u32>::max();
    const auto found = FaithfulCaptureProductPhysicalRefs.find(productId);
    return found != FaithfulCaptureProductPhysicalRefs.end()
        ? found->second : 0u;
}

bool GPU::RefreshFaithfulCaptureProductPhysicalRefs() noexcept
{
    if (!FaithfulVramCaptureTrackingEnabled)
    {
        FaithfulCaptureProductPhysicalRefs.clear();
        FaithfulCaptureProductPhysicalRefsDirty = false;
        return true;
    }
    if (!FaithfulCaptureProductPhysicalRefsDirty)
        return true;

    try
    {
        std::unordered_map<u64, u32> rebuilt;
        rebuilt.reserve(FaithfulCaptureProductPhysicalRefs.size() + 8u);
        u64 lastProductId = 0u;
        u32* lastProductCount = nullptr;
        const auto addRef = [&](u64 productId, u32 count = 1u) {
            if (productId == 0u)
                return;
            if (productId != lastProductId || lastProductCount == nullptr)
            {

                lastProductId = productId;
                lastProductCount = &rebuilt[productId];
            }
            const u32 available = std::numeric_limits<u32>::max() - *lastProductCount;
            assert(count <= available);
            *lastProductCount += std::min(count, available);
        };

#ifndef NDEBUG
        u32 observedVramTags = 0u;
        u32 observedMainRamPages = 0u;
        u64 observedMainRamTags = 0u;
#endif
        for (u32 bank = 0u; bank < 9u; bank++)
        {
            if (FaithfulVramValidTagCounts[bank] == 0u)
                continue;
            const size_t halfwordCount =
                (static_cast<size_t>(VRAMMask[bank]) + 1u) / 2u;
            const FaithfulVramCaptureTag* tags =
                FaithfulVramCaptureTags[bank].get();
            for (size_t first = 0u; first < halfwordCount;
                 first += FaithfulVramSummaryPageHalfwords)
            {
                const FaithfulVramCaptureSpan& summary =
                    FaithfulVramCapturePageSummaries[bank]
                        [first / FaithfulVramSummaryPageHalfwords];
                if (summary.Valid()
                    && summary.ProductEpoch == FaithfulCaptureProductEpoch
                    && summary.StorageBank == bank)
                {

                    addRef(summary.ProductId, FaithfulVramSummaryPageHalfwords);
#ifndef NDEBUG
                    for (size_t x = 0u; x < FaithfulVramSummaryPageHalfwords; x++)
                    {
                        assert(tags[first + x].Valid());
                        assert(tags[first + x].ProductEpoch == summary.ProductEpoch);
                        assert(tags[first + x].ProductId == summary.ProductId);
                    }
                    observedVramTags += FaithfulVramSummaryPageHalfwords;
#endif
                    continue;
                }
                for (size_t x = 0u; x < FaithfulVramSummaryPageHalfwords; x++)
                {
                    const FaithfulVramCaptureTag& tag = tags[first + x];
                    if (!tag.Valid()
                        || tag.ProductEpoch != FaithfulCaptureProductEpoch)
                        continue;
                    addRef(tag.ProductId);
#ifndef NDEBUG
                    observedVramTags++;
#endif
                }
            }
        }

        for (const auto& pagePtr : FaithfulMainRamCapturePages)
        {
            if (pagePtr == nullptr)
                continue;
#ifndef NDEBUG
            observedMainRamPages++;
            assert(pagePtr->ValidCount > 0u);
            u32 observedPageTags = 0u;
#endif
            for (const FaithfulMainRamCaptureTag& tag : pagePtr->Tags)
            {
                if (!tag.Valid())
                    continue;
                addRef(tag.ProductId);
#ifndef NDEBUG
                observedMainRamTags++;
                observedPageTags++;
#endif
            }
#ifndef NDEBUG
            assert(observedPageTags == pagePtr->ValidCount);
#endif
        }

#ifndef NDEBUG
        assert(observedVramTags == FaithfulVramTotalValidTagCount);
        assert(observedMainRamPages == FaithfulMainRamTaggedPageCount);
        u64 rebuiltRefCount = 0u;
        for (const auto& [productId, count] : rebuilt)
        {
            (void)productId;
            rebuiltRefCount += count;
        }
        assert(rebuiltRefCount
            == static_cast<u64>(observedVramTags) + observedMainRamTags);
#endif
        FaithfulCaptureProductPhysicalRefs.swap(rebuilt);
        FaithfulCaptureProductPhysicalRefsDirty = false;
        return true;
    }
    catch (const std::bad_alloc&)
    {

        return false;
    }
}

void GPU::InvalidateFaithfulCaptureLineage() noexcept
{
    ClearFaithfulCaptureTags();
    if (FaithfulCaptureProductEpoch
            == std::numeric_limits<u64>::max())
    {

        FaithfulCaptureProductCounter = std::numeric_limits<u64>::max();
        return;
    }
    FaithfulCaptureProductEpoch++;
    FaithfulCaptureProductCounter = 0u;
}

void GPU::SetFaithfulVramCaptureTrackingEnabled(bool enabled) noexcept
{
    if (FaithfulVramCaptureTrackingEnabled == enabled)
        return;

    ClearFaithfulCaptureTags();
    FaithfulVramCaptureTrackingEnabled = enabled;
    if (enabled && FaithfulCaptureProductEpoch != std::numeric_limits<u64>::max())
    {
        FaithfulCaptureProductEpoch++;
        FaithfulCaptureProductCounter = 0u;
    }
}

void GPU::ClearFaithfulCaptureTags() noexcept
{

    FaithfulDmaLineageBatchActive = false;
    FaithfulDmaLineageSource = {};
    FaithfulDmaLineageDestination = {};
    FaithfulDmaLineageMaximumByteCount = 0u;
#ifndef NDEBUG
    u32 observedVramTagCount = 0u;
    u32 observedMainRamPageCount = 0u;
    u64 observedMainRamTagCount = 0u;
#endif
    for (u32 bank = 0; bank < 9; bank++)
    {
#ifndef NDEBUG
        observedVramTagCount += FaithfulVramValidTagCounts[bank];
#endif
        const size_t halfwordCount = (static_cast<size_t>(VRAMMask[bank]) + 1u) / 2u;
        std::fill_n(
            FaithfulVramCaptureTags[bank].get(),
            halfwordCount,
            FaithfulVramCaptureTag{});
        const size_t summaryPageCount =
            halfwordCount / FaithfulVramSummaryPageHalfwords;
        std::fill_n(
            FaithfulVramCapturePageSummaries[bank].get(),
            summaryPageCount,
            FaithfulVramCaptureSpan{});
    }
#ifndef NDEBUG
    assert(observedVramTagCount == FaithfulVramTotalValidTagCount);
#endif
    FaithfulVramValidTagCounts.fill(0u);
    FaithfulVramTotalValidTagCount = 0u;
    for (u32 pageIndex = 0u;
         pageIndex < FaithfulMainRamCapturePages.size();
         pageIndex++)
    {
        std::unique_ptr<FaithfulMainRamCapturePage>& page =
            FaithfulMainRamCapturePages[pageIndex];
        if (page == nullptr)
            continue;

#ifndef NDEBUG
        observedMainRamPageCount++;
        observedMainRamTagCount += page->ValidCount;
#endif
        assert(page->ValidCount > 0u);
        (void)NDS.JIT.Memory.NotifyMainRAMLineagePageTransition(
            pageIndex << FaithfulMainRamPageShift, false);
        page.reset();
    }
#ifndef NDEBUG
    assert(observedMainRamPageCount == FaithfulMainRamTaggedPageCount);
    (void)observedMainRamTagCount;
#endif
    FaithfulMainRamTaggedPageCount = 0u;
    FaithfulCaptureProductPhysicalRefs.clear();
    FaithfulCaptureProductPhysicalRefsDirty = false;
}

void GPU::InvalidateFaithfulVramCapturePageSummaries(
    u32 bank, u32 firstHalfwordOffset,
    u32 halfwordCount) noexcept
{
    if (bank >= 9u || halfwordCount == 0u)
        return;

    const u32 bankHalfwordCount = (VRAMMask[bank] + 1u) >> 1u;
    const u32 summaryPageCount =
        bankHalfwordCount / FaithfulVramSummaryPageHalfwords;
    if (halfwordCount >= bankHalfwordCount)
    {
        std::fill_n(
            FaithfulVramCapturePageSummaries[bank].get(),
            summaryPageCount,
            FaithfulVramCaptureSpan{});
        return;
    }

    u32 current = firstHalfwordOffset & (bankHalfwordCount - 1u);
    u32 remaining = halfwordCount;
    while (remaining != 0u)
    {
        const u32 pageIndex =
            current / FaithfulVramSummaryPageHalfwords;
        FaithfulVramCapturePageSummaries[bank][pageIndex] = {};
        const u32 untilPageEnd = FaithfulVramSummaryPageHalfwords
            - (current % FaithfulVramSummaryPageHalfwords);
        const u32 untilBankEnd = bankHalfwordCount - current;
        const u32 consumed = std::min(
            remaining, std::min(untilPageEnd, untilBankEnd));
        remaining -= consumed;
        current = (current + consumed) & (bankHalfwordCount - 1u);
    }
}

void GPU::InvalidateFaithfulVramCaptureRange(
    u32 bank, u32 byteOffset, u32 byteCount) noexcept
{
    if (ShouldDeferFaithfulDmaVramInvalidation(
            bank, byteOffset, byteCount)
        || !FaithfulVramCaptureTrackingEnabled || bank >= 9u || byteCount == 0u
        || FaithfulVramValidTagCounts[bank] == 0u)
        return;

    const u32 halfwordMask = VRAMMask[bank] >> 1u;
    const u32 firstHalfword = (byteOffset & VRAMMask[bank]) >> 1u;
    const u32 touchedHalfwords = ((byteOffset & 1u) + byteCount + 1u) >> 1u;
    for (u32 i = 0u; i < touchedHalfwords; i++)
        SetFaithfulVramCaptureTag(
            bank, (firstHalfword + i) & halfwordMask, {});
}

bool GPU::ShouldDeferFaithfulDmaVramInvalidation(
    u32 bank, u32 byteOffset, u32 byteCount) const noexcept
{
    if (!FaithfulDmaLineageBatchActive || byteCount == 0u
        || FaithfulDmaLineageDestination.Kind
            != FaithfulDmaLineageStorageKind::Vram
        || bank != FaithfulDmaLineageDestination.Bank)
    {
        return false;
    }

    const u64 writeBegin = byteOffset;
    const u64 writeEnd = writeBegin + byteCount;
    const u64 batchBegin = FaithfulDmaLineageDestination.ByteOffset;
    const u64 batchEnd = batchBegin + FaithfulDmaLineageMaximumByteCount;
    return writeBegin >= batchBegin && writeEnd <= batchEnd;
}

bool GPU::GetFaithfulVramCaptureTag(
    u32 bank, u32 halfwordOffset,
    FaithfulVramCaptureTag& outTag) const noexcept
{
    outTag = {};
    if (!FaithfulVramCaptureTrackingEnabled || bank >= 9u)
        return false;
    const u32 halfwordMask = VRAMMask[bank] >> 1u;
    outTag = FaithfulVramCaptureTags[bank][halfwordOffset & halfwordMask];
    if (!outTag.Valid()
        || outTag.ProductEpoch != FaithfulCaptureProductEpoch)
    {
        outTag = {};
        return false;
    }
    return true;
}

void GPU::SetFaithfulVramCaptureTag(
    u32 bank, u32 halfwordOffset,
    const FaithfulVramCaptureTag& tag) noexcept
{
    if (!FaithfulVramCaptureTrackingEnabled || bank >= 9u)
        return;
    const u32 halfwordMask = VRAMMask[bank] >> 1u;
    const u32 normalizedHalfwordOffset = halfwordOffset & halfwordMask;

    FaithfulVramCaptureSpan& pageSummary =
        FaithfulVramCapturePageSummaries[bank]
            [normalizedHalfwordOffset
                / FaithfulVramSummaryPageHalfwords];
    if (pageSummary.Valid())
        pageSummary = {};
    FaithfulVramCaptureTag& stored =
        FaithfulVramCaptureTags[bank][normalizedHalfwordOffset];
    FaithfulVramCaptureTag normalized =
        tag.Valid() && tag.ProductEpoch == FaithfulCaptureProductEpoch
            ? tag : FaithfulVramCaptureTag{};
    const bool storedValid = stored.Valid()
        && stored.ProductEpoch == FaithfulCaptureProductEpoch;
    const u64 oldProductId = storedValid ? stored.ProductId : 0u;
    if (oldProductId != normalized.ProductId)
        FaithfulCaptureProductPhysicalRefsDirty = true;
    const bool normalizedValid = normalized.Valid();
    if (storedValid != normalizedValid)
    {
        u32& validCount = FaithfulVramValidTagCounts[bank];
        if (normalizedValid)
        {

            assert(validCount < std::numeric_limits<u32>::max());
            assert(FaithfulVramTotalValidTagCount
                < std::numeric_limits<u32>::max());
            validCount++;
            FaithfulVramTotalValidTagCount++;
        }
        else
        {
            assert(validCount > 0u);
            if (validCount > 0u)
                validCount--;
            assert(FaithfulVramTotalValidTagCount > 0u);
            if (FaithfulVramTotalValidTagCount > 0u)
                FaithfulVramTotalValidTagCount--;
        }
    }
    stored = normalized;
}

void GPU::StampFaithfulVramCaptureTagLine(
    u32 bank, u32 firstHalfwordOffset, u32 halfwordCount,
    u64 productEpoch, u64 productId, u16 sourceY) noexcept
{
    if (!FaithfulVramCaptureTrackingEnabled || bank >= 9u
        || halfwordCount == 0u)
    {
        return;
    }

    const u32 halfwordMask = VRAMMask[bank] >> 1u;
    const u32 normalizedFirstHalfword = firstHalfwordOffset & halfwordMask;
    bool oldLineFullyTagged = false;
    if (halfwordCount == FaithfulVramSummaryPageHalfwords
        && (normalizedFirstHalfword % FaithfulVramSummaryPageHalfwords) == 0u)
    {
        const FaithfulVramCaptureSpan& oldSummary =
            FaithfulVramCapturePageSummaries[bank]
                [normalizedFirstHalfword / FaithfulVramSummaryPageHalfwords];
        oldLineFullyTagged = oldSummary.Valid()
            && oldSummary.ProductEpoch == FaithfulCaptureProductEpoch
            && oldSummary.StorageBank == bank;
    }

    InvalidateFaithfulVramCapturePageSummaries(
        bank, firstHalfwordOffset, halfwordCount);

    constexpr u32 kMaxCaptureLineHalfwords = 256u;
    if (halfwordCount > kMaxCaptureLineHalfwords)
    {
        for (u32 x = 0u; x < halfwordCount; x++)
        {
            FaithfulVramCaptureTag tag {};
            if (productEpoch == FaithfulCaptureProductEpoch
                && productId != 0u)
            {
                tag.ProductEpoch = productEpoch;
                tag.ProductId = productId;
                tag.SourceX = static_cast<u16>(x);
                tag.SourceY = sourceY;
            }
            SetFaithfulVramCaptureTag(bank, firstHalfwordOffset + x, tag);
        }
        return;
    }

    const bool publishValid = productEpoch == FaithfulCaptureProductEpoch
        && productEpoch != 0u && productId != 0u;
    u32 oldValidCount = oldLineFullyTagged ? halfwordCount : 0u;

    if (!oldLineFullyTagged)
    {
        for (u32 x = 0u; x < halfwordCount; x++)
        {
            const FaithfulVramCaptureTag& stored =
                FaithfulVramCaptureTags[bank]
                    [(firstHalfwordOffset + x) & halfwordMask];
            if (!stored.Valid()
                || stored.ProductEpoch != FaithfulCaptureProductEpoch)
            {
                continue;
            }

            oldValidCount++;
        }
    }

    const u32 newValidCount = publishValid ? halfwordCount : 0u;
    if (newValidCount >= oldValidCount)
    {
        const u32 added = newValidCount - oldValidCount;
        assert(FaithfulVramValidTagCounts[bank]
            <= std::numeric_limits<u32>::max() - added);
        assert(FaithfulVramTotalValidTagCount
            <= std::numeric_limits<u32>::max() - added);
        FaithfulVramValidTagCounts[bank] += added;
        FaithfulVramTotalValidTagCount += added;
    }
    else
    {
        const u32 removed = oldValidCount - newValidCount;
        assert(FaithfulVramValidTagCounts[bank] >= removed);
        assert(FaithfulVramTotalValidTagCount >= removed);
        FaithfulVramValidTagCounts[bank] -= removed;
        FaithfulVramTotalValidTagCount -= removed;
    }

    for (u32 x = 0u; x < halfwordCount; x++)
    {
        FaithfulVramCaptureTag& stored =
            FaithfulVramCaptureTags[bank]
                [(firstHalfwordOffset + x) & halfwordMask];
        if (publishValid)
        {
            stored.ProductEpoch = productEpoch;
            stored.ProductId = productId;
            stored.SourceX = static_cast<u16>(x);
            stored.SourceY = sourceY;
        }
        else
        {
            stored = {};
        }
    }
    if (publishValid
        && halfwordCount == FaithfulVramSummaryPageHalfwords
        && (normalizedFirstHalfword
            % FaithfulVramSummaryPageHalfwords) == 0u)
    {
        FaithfulVramCaptureSpan summary {};
        summary.ProductEpoch = productEpoch;
        summary.ProductId = productId;
        summary.SourceXBase = 0u;
        summary.SourceY = sourceY;
        summary.StorageBank = static_cast<u8>(bank);
        FaithfulVramCapturePageSummaries[bank]
            [normalizedFirstHalfword
                / FaithfulVramSummaryPageHalfwords] = summary;
    }
    FaithfulCaptureProductPhysicalRefsDirty = true;
}

bool GPU::ResolveFaithfulVramCaptureUniformSpan(
    u32 bank, u32 firstHalfwordOffset, u32 halfwordCount,
    FaithfulVramCaptureSpan& outSpan) const noexcept
{
    outSpan = {};
    if (!FaithfulVramCaptureTrackingEnabled || bank >= 9u
        || halfwordCount == 0u)
    {
        return false;
    }

    const u32 bankHalfwordCount = (VRAMMask[bank] + 1u) >> 1u;

    if (firstHalfwordOffset >= bankHalfwordCount
        || halfwordCount > bankHalfwordCount - firstHalfwordOffset)
    {
        return false;
    }

    const u32 firstPage =
        firstHalfwordOffset / FaithfulVramSummaryPageHalfwords;
    const FaithfulVramCaptureSpan& firstSummary =
        FaithfulVramCapturePageSummaries[bank][firstPage];
    if (!firstSummary.Valid()
        || firstSummary.ProductEpoch != FaithfulCaptureProductEpoch
        || firstSummary.StorageBank != bank)
    {
        return false;
    }

    const u32 firstInPage =
        firstHalfwordOffset % FaithfulVramSummaryPageHalfwords;
    const u32 sourceXBase =
        static_cast<u32>(firstSummary.SourceXBase) + firstInPage;
    if (sourceXBase > std::numeric_limits<u16>::max())
        return false;

    u32 consumed = 0u;
    while (consumed < halfwordCount)
    {
        const u32 physicalOffset = firstHalfwordOffset + consumed;
        const u32 pageIndex =
            physicalOffset / FaithfulVramSummaryPageHalfwords;
        const u32 inPage =
            physicalOffset % FaithfulVramSummaryPageHalfwords;
        const FaithfulVramCaptureSpan& summary =
            FaithfulVramCapturePageSummaries[bank][pageIndex];
        if (!summary.Valid()
            || summary.ProductEpoch != firstSummary.ProductEpoch
            || summary.ProductId != firstSummary.ProductId
            || summary.SourceY != firstSummary.SourceY
            || summary.StorageBank != bank)
        {
            return false;
        }

        const u32 actualSourceX =
            static_cast<u32>(summary.SourceXBase) + inPage;
        const u32 expectedSourceX = sourceXBase + consumed;
        if (expectedSourceX > std::numeric_limits<u16>::max()
            || actualSourceX != expectedSourceX)
        {
            return false;
        }

        const u32 pageRemaining =
            FaithfulVramSummaryPageHalfwords - inPage;
        consumed += std::min(halfwordCount - consumed, pageRemaining);
    }

    outSpan = firstSummary;
    outSpan.SourceXBase = static_cast<u16>(sourceXBase);

#ifndef NDEBUG

    for (u32 x = 0u; x < halfwordCount; x++)
    {
        FaithfulVramCaptureTag denseTag {};
        const bool denseValid = GetFaithfulVramCaptureTag(
            bank, firstHalfwordOffset + x, denseTag);
        const u32 expectedSourceX = sourceXBase + x;
        if (!denseValid
            || denseTag.ProductEpoch != outSpan.ProductEpoch
            || denseTag.ProductId != outSpan.ProductId
            || denseTag.SourceY != outSpan.SourceY
            || denseTag.SourceX != expectedSourceX)
        {
            assert(false && "stale faithful VRAM page summary");
            outSpan = {};
            return false;
        }
    }
#endif
    return true;
}

bool GPU::ResolveFaithfulMainRamAddress(
    u32 cpu, u32 addr, u32& outByteOffset) const noexcept
{
    outByteOffset = 0u;
    (void)cpu;
    if (!FaithfulVramCaptureTrackingEnabled)
        return false;

    const u32 region = addr & 0xFF000000u;
    const bool regularMainRam = region == 0x02000000u;
    const bool dsiMainRamMirror = NDS.ConsoleType == 1
        && region == 0x0C000000u;
    if (!regularMainRam && !dsiMainRamMirror)
        return false;

    outByteOffset = addr & NDS.MainRAMMask;
    return true;
}

bool GPU::GetFaithfulMainRamCaptureTag(
    u32 byteOffset, FaithfulVramCaptureTag& outTag) const noexcept
{
    outTag = {};
    if (!FaithfulVramCaptureTrackingEnabled)
        return false;

    const u32 physicalOffset = byteOffset & NDS.MainRAMMask;
    const u32 pageIndex = physicalOffset >> FaithfulMainRamPageShift;
    if (pageIndex >= FaithfulMainRamCapturePages.size()
        || FaithfulMainRamCapturePages[pageIndex] == nullptr)
        return false;

    const u32 halfwordIndex =
        (physicalOffset & (FaithfulMainRamPageSize - 1u)) >> 1u;
    const FaithfulMainRamCaptureTag& stored =
        FaithfulMainRamCapturePages[pageIndex]->Tags[halfwordIndex];
    if (!stored.Valid())
        return false;

    outTag.ProductEpoch = FaithfulCaptureProductEpoch;
    outTag.ProductId = stored.ProductId;
    outTag.SourceX = stored.SourceX;
    outTag.SourceY = stored.SourceY;
    return true;
}

void GPU::SetFaithfulMainRamCaptureTag(
    u32 byteOffset, const FaithfulVramCaptureTag& tag) noexcept
{
    if (!FaithfulVramCaptureTrackingEnabled)
        return;

    const u32 physicalOffset = byteOffset & NDS.MainRAMMask;
    const u32 pageIndex = physicalOffset >> FaithfulMainRamPageShift;
    const u32 halfwordIndex =
        (physicalOffset & (FaithfulMainRamPageSize - 1u)) >> 1u;
    const bool valid = tag.Valid()
        && tag.ProductEpoch == FaithfulCaptureProductEpoch;

    if (pageIndex >= FaithfulMainRamCapturePages.size())
        return;
    std::unique_ptr<FaithfulMainRamCapturePage>& pagePtr =
        FaithfulMainRamCapturePages[pageIndex];
    bool allocatedPage = false;
    if (pagePtr == nullptr)
    {
        if (!valid)
            return;
        try
        {
            pagePtr = std::make_unique<FaithfulMainRamCapturePage>();
            allocatedPage = true;
        }
        catch (const std::bad_alloc&)
        {

            return;
        }
    }

    FaithfulMainRamCapturePage& page = *pagePtr;
    FaithfulMainRamCaptureTag& stored = page.Tags[halfwordIndex];
    const bool wasValid = stored.Valid();
    bool publishValid = valid;
    if (publishValid && !wasValid && page.ValidCount == 0u
        && !NDS.JIT.Memory.NotifyMainRAMLineagePageTransition(
            pageIndex << FaithfulMainRamPageShift, true))
    {

        if (allocatedPage)
            pagePtr.reset();
        return;
    }
    const u64 oldProductId = wasValid ? stored.ProductId : 0u;
    const u64 newProductId = publishValid ? tag.ProductId : 0u;
    if (oldProductId != newProductId)
        FaithfulCaptureProductPhysicalRefsDirty = true;
    if (publishValid)
    {
        stored.ProductId = tag.ProductId;
        stored.SourceX = tag.SourceX;
        stored.SourceY = tag.SourceY;
        if (!wasValid)
        {
            if (page.ValidCount == 0u)
            {
                assert(FaithfulMainRamTaggedPageCount
                    < FaithfulMainRamCapturePages.size());
                FaithfulMainRamTaggedPageCount++;
            }
            page.ValidCount++;
        }
    }
    else if (wasValid)
    {
        stored = {};
        assert(page.ValidCount > 0u);
        page.ValidCount--;
    }
    if (page.ValidCount == 0u)
    {
        if (wasValid)
        {
            assert(FaithfulMainRamTaggedPageCount > 0u);
            FaithfulMainRamTaggedPageCount--;
        }
        (void)NDS.JIT.Memory.NotifyMainRAMLineagePageTransition(
            pageIndex << FaithfulMainRamPageShift, false);
        pagePtr.reset();
    }
}

void GPU::InvalidateFaithfulMainRamCaptureRange(
    u32 byteOffset, u32 byteCount) noexcept
{
    if (!HasFaithfulMainRamCaptureTags() || byteCount == 0u)
        return;

    const u32 firstHalfword = (byteOffset & NDS.MainRAMMask) >> 1u;
    const u32 halfwordMask = NDS.MainRAMMask >> 1u;
    const u32 touchedHalfwords = ((byteOffset & 1u) + byteCount + 1u) >> 1u;
    for (u32 i = 0u; i < touchedHalfwords; i++)
    {
        const u32 halfword = (firstHalfword + i) & halfwordMask;
        const u32 physicalOffset = halfword << 1u;
        const u32 pageIndex = physicalOffset >> FaithfulMainRamPageShift;
        if (pageIndex >= FaithfulMainRamCapturePages.size()
            || FaithfulMainRamCapturePages[pageIndex] == nullptr)
        {
            continue;
        }
        SetFaithfulMainRamCaptureTag(physicalOffset, {});
    }
}

void GPU::InvalidateFaithfulMainRamCaptureRangeForCpuAddress(
    u32 cpu, u32 addr, u32 byteCount) noexcept
{
    Sync2D();
    if (!HasFaithfulMainRamCaptureTags())
        return;
    u32 byteOffset = 0u;
    if (ResolveFaithfulMainRamAddress(cpu, addr, byteOffset))
        InvalidateFaithfulMainRamCaptureRange(byteOffset, byteCount);
}

bool GPU::ResolveFaithfulVramAddress(
    u32 cpu, u32 addr, u8& outBank, u32& outByteOffset) const noexcept
{
    outBank = 0xFFu;
    outByteOffset = 0u;
    if (!FaithfulVramCaptureTrackingEnabled)
        return false;

    u32 mask = 0u;
    if (cpu == 1u)
    {
        if ((addr & 0xFF800000u) != 0x06000000u
            && (addr & 0xFF800000u) != 0x06800000u)
            return false;
        mask = VRAMMap_ARM7[(addr >> 17u) & 0x1u] & ((1u << 2u) | (1u << 3u));
    }
    else
    {
        if ((addr & 0xFF000000u) != 0x06000000u)
            return false;
        switch (addr & 0x00E00000u)
        {
        case 0x00000000u:
            mask = VRAMMap_ABG[(addr >> 14u) & 0x1Fu] & 0x7Fu;
            break;
        case 0x00200000u:
            mask = VRAMMap_BBG[(addr >> 14u) & 0x7u]
                & ((1u << 2u) | (1u << 7u) | (1u << 8u));
            break;
        case 0x00400000u:
            mask = VRAMMap_AOBJ[(addr >> 14u) & 0xFu]
                & ((1u << 0u) | (1u << 1u) | (1u << 4u)
                    | (1u << 5u) | (1u << 6u));
            break;
        case 0x00600000u:
            mask = VRAMMap_BOBJ[(addr >> 14u) & 0x7u]
                & ((1u << 3u) | (1u << 8u));
            break;
        default:
            switch (addr & 0xFF8FC000u)
            {
            case 0x06800000u: case 0x06804000u: case 0x06808000u: case 0x0680C000u:
            case 0x06810000u: case 0x06814000u: case 0x06818000u: case 0x0681C000u:
                outBank = 0u; outByteOffset = addr & 0x1FFFFu; break;
            case 0x06820000u: case 0x06824000u: case 0x06828000u: case 0x0682C000u:
            case 0x06830000u: case 0x06834000u: case 0x06838000u: case 0x0683C000u:
                outBank = 1u; outByteOffset = addr & 0x1FFFFu; break;
            case 0x06840000u: case 0x06844000u: case 0x06848000u: case 0x0684C000u:
            case 0x06850000u: case 0x06854000u: case 0x06858000u: case 0x0685C000u:
                outBank = 2u; outByteOffset = addr & 0x1FFFFu; break;
            case 0x06860000u: case 0x06864000u: case 0x06868000u: case 0x0686C000u:
            case 0x06870000u: case 0x06874000u: case 0x06878000u: case 0x0687C000u:
                outBank = 3u; outByteOffset = addr & 0x1FFFFu; break;
            case 0x06880000u: case 0x06884000u: case 0x06888000u: case 0x0688C000u:
                outBank = 4u; outByteOffset = addr & 0xFFFFu; break;
            case 0x06890000u:
                outBank = 5u; outByteOffset = addr & 0x3FFFu; break;
            case 0x06894000u:
                outBank = 6u; outByteOffset = addr & 0x3FFFu; break;
            case 0x06898000u: case 0x0689C000u:
                outBank = 7u; outByteOffset = addr & 0x7FFFu; break;
            case 0x068A0000u:
                outBank = 8u; outByteOffset = addr & 0x3FFFu; break;
            default:
                return false;
            }
            return (VRAMMap_LCDC & (1u << outBank)) != 0u;
        }
    }

    if (mask == 0u || (mask & (mask - 1u)) != 0u)
        return false;
    outBank = static_cast<u8>(__builtin_ctz(mask));
    outByteOffset = addr & VRAMMask[outBank];
    return true;
}

bool GPU::ResolveFaithfulDmaLineageRange(
    u32 cpu, u32 addr, u32 byteCount,
    FaithfulDmaLineageStorage& outStorage) const noexcept
{
    outStorage = {};
    if (!FaithfulVramCaptureTrackingEnabled || byteCount < 2u
        || (addr & 1u) != 0u
        || byteCount > std::numeric_limits<u32>::max() - addr + 1u)
    {
        return false;
    }

    const u32 lastAddress = addr + byteCount - 2u;
    u32 firstMainRamOffset = 0u;
    u32 lastMainRamOffset = 0u;
    if (ResolveFaithfulMainRamAddress(cpu, addr, firstMainRamOffset)
        && ResolveFaithfulMainRamAddress(
            cpu, lastAddress, lastMainRamOffset)
        && byteCount - 2u <= NDS.MainRAMMask
        && firstMainRamOffset <= NDS.MainRAMMask - (byteCount - 2u)
        && lastMainRamOffset == firstMainRamOffset + byteCount - 2u)
    {
        outStorage.Kind = FaithfulDmaLineageStorageKind::MainRam;
        outStorage.ByteOffset = firstMainRamOffset;
        return true;
    }

    u8 firstBank = 0xFFu;
    u8 lastBank = 0xFFu;
    u32 firstVramOffset = 0u;
    u32 lastVramOffset = 0u;
    if (ResolveFaithfulVramAddress(
            cpu, addr, firstBank, firstVramOffset)
        && ResolveFaithfulVramAddress(
            cpu, lastAddress, lastBank, lastVramOffset)
        && firstBank == lastBank
        && byteCount - 2u <= VRAMMask[firstBank]
        && firstVramOffset <= VRAMMask[firstBank] - (byteCount - 2u)
        && lastVramOffset == firstVramOffset + byteCount - 2u)
    {

        u64 probeAddress =
            (static_cast<u64>(addr) + 0x4000u) & ~0x3FFFull;
        while (probeAddress < lastAddress)
        {
            u8 probeBank = 0xFFu;
            u32 probeOffset = 0u;
            if (!ResolveFaithfulVramAddress(
                    cpu, static_cast<u32>(probeAddress),
                    probeBank, probeOffset)
                || probeBank != firstBank
                || probeOffset != firstVramOffset
                    + static_cast<u32>(probeAddress - addr))
            {
                return false;
            }
            probeAddress += 0x4000u;
        }
        outStorage.Kind = FaithfulDmaLineageStorageKind::Vram;
        outStorage.Bank = firstBank;
        outStorage.ByteOffset = firstVramOffset;
        return true;
    }
    return false;
}

bool GPU::ShouldDeferFaithfulDmaMainRamInvalidation(
    u32 cpu, u32 addr, u32 byteCount) const noexcept
{
    Sync2D();
    if (!FaithfulDmaLineageBatchActive || byteCount == 0u
        || FaithfulDmaLineageDestination.Kind
            != FaithfulDmaLineageStorageKind::MainRam)
    {
        return false;
    }

    u32 byteOffset = 0u;
    if (!ResolveFaithfulMainRamAddress(cpu, addr, byteOffset))
        return false;

    const u64 writeBegin = byteOffset;
    const u64 writeEnd = writeBegin + byteCount;
    const u64 batchBegin = FaithfulDmaLineageDestination.ByteOffset;
    const u64 batchEnd = batchBegin + FaithfulDmaLineageMaximumByteCount;
    return writeBegin >= batchBegin && writeEnd <= batchEnd;
}

bool GPU::BeginFaithfulDmaLineageBatch(
    u32 cpu, u32 sourceAddress, u32 destinationAddress,
    u32 byteCount) noexcept
{
    Sync2D();
    if (FaithfulDmaLineageBatchActive || byteCount < 2u
        || !HasFaithfulCapturePhysicalTags())
    {
        return false;
    }

    FaithfulDmaLineageStorage source {};
    FaithfulDmaLineageStorage destination {};
    if (!ResolveFaithfulDmaLineageRange(
            cpu, sourceAddress, byteCount, source)
        || !ResolveFaithfulDmaLineageRange(
            cpu, destinationAddress, byteCount, destination))
    {
        return false;
    }

    const bool sameStorage = source.Kind == destination.Kind
        && (source.Kind != FaithfulDmaLineageStorageKind::Vram
            || source.Bank == destination.Bank);
    if (sameStorage)
    {
        const u64 sourceBegin = source.ByteOffset;
        const u64 sourceEnd = sourceBegin + byteCount;
        const u64 destinationBegin = destination.ByteOffset;
        const u64 destinationEnd = destinationBegin + byteCount;
        if (sourceBegin < destinationEnd
            && destinationBegin < sourceEnd)
        {

            return false;
        }
    }

    if (destination.Kind == FaithfulDmaLineageStorageKind::Vram)
    {

        InvalidateFaithfulVramCapturePageSummaries(
            destination.Bank,
            destination.ByteOffset >> 1u,
            (byteCount >> 1u) + (byteCount & 1u));
    }

    FaithfulDmaLineageSource = source;
    FaithfulDmaLineageDestination = destination;
    FaithfulDmaLineageMaximumByteCount = byteCount;

    FaithfulCaptureProductPhysicalRefsDirty = true;
    FaithfulDmaLineageBatchActive = true;
    return true;
}

void GPU::FinishFaithfulDmaLineageBatch(
    u32 transferredByteCount) noexcept
{
    Sync2D();
    if (!FaithfulDmaLineageBatchActive)
        return;

    const FaithfulDmaLineageStorage source = FaithfulDmaLineageSource;
    const FaithfulDmaLineageStorage destination =
        FaithfulDmaLineageDestination;
    const u32 maximumByteCount = FaithfulDmaLineageMaximumByteCount;
    FaithfulDmaLineageBatchActive = false;
    FaithfulDmaLineageSource = {};
    FaithfulDmaLineageDestination = {};
    FaithfulDmaLineageMaximumByteCount = 0u;

    assert((transferredByteCount & 1u) == 0u);
    assert(transferredByteCount <= maximumByteCount);
    if ((transferredByteCount & 1u) != 0u
        || transferredByteCount > maximumByteCount)
    {

        transferredByteCount = maximumByteCount & ~1u;
        for (u32 offset = 0u; offset < transferredByteCount; offset += 2u)
        {
            if (destination.Kind == FaithfulDmaLineageStorageKind::Vram)
                SetFaithfulVramCaptureTag(
                    destination.Bank,
                    (destination.ByteOffset + offset) >> 1u, {});
            else if (destination.Kind
                == FaithfulDmaLineageStorageKind::MainRam)
                SetFaithfulMainRamCaptureTag(
                    destination.ByteOffset + offset, {});
        }
        return;
    }

    const bool sourceMayHaveTags =
        source.Kind == FaithfulDmaLineageStorageKind::Vram
            ? FaithfulVramValidTagCounts[source.Bank] != 0u
            : FaithfulMainRamTaggedPageCount != 0u;
    if (!sourceMayHaveTags)
    {
        if (destination.Kind == FaithfulDmaLineageStorageKind::Vram
            && FaithfulVramValidTagCounts[destination.Bank] == 0u)
        {

            std::fill_n(FaithfulVramCaptureTags[destination.Bank].get()
                    + (destination.ByteOffset >> 1u),
                transferredByteCount >> 1u, FaithfulVramCaptureTag{});
            return;
        }
        if (destination.Kind == FaithfulDmaLineageStorageKind::MainRam
            && FaithfulMainRamTaggedPageCount == 0u)
        {

            return;
        }
    }

    if (destination.Kind == FaithfulDmaLineageStorageKind::Vram)
    {
        FaithfulVramCaptureTag* output =
            FaithfulVramCaptureTags[destination.Bank].get()
                + (destination.ByteOffset >> 1u);
        const u64 epoch = FaithfulCaptureProductEpoch;
        u32 oldValidCount = 0u;
        u32 newValidCount = 0u;
        const auto copyTag = [&](const FaithfulVramCaptureTag& tag) {
            oldValidCount += output->Valid()
                && output->ProductEpoch == epoch;
            const bool valid = tag.Valid() && tag.ProductEpoch == epoch;
            newValidCount += valid;
            *output++ = valid ? tag : FaithfulVramCaptureTag{};
        };

        if (!sourceMayHaveTags)
        {
            for (u32 i = 0u; i < transferredByteCount / 2u; i++)
                copyTag({});
        }
        else if (source.Kind == FaithfulDmaLineageStorageKind::Vram)
        {
            const FaithfulVramCaptureTag* input =
                FaithfulVramCaptureTags[source.Bank].get()
                    + (source.ByteOffset >> 1u);
            for (u32 i = 0u; i < transferredByteCount / 2u; i++)
                copyTag(input[i]);
        }
        else
        {
            u32 offset = source.ByteOffset;
            const u32 end = offset + transferredByteCount;
            while (offset < end)
            {
                const u32 pageIndex = offset >> FaithfulMainRamPageShift;
                const u32 pageOffset = offset & (FaithfulMainRamPageSize - 1u);
                const u32 count = std::min(end - offset,
                    FaithfulMainRamPageSize - pageOffset) >> 1u;
                const FaithfulMainRamCapturePage* page =
                    FaithfulMainRamCapturePages[pageIndex].get();
                if (page != nullptr)
                {
                    const FaithfulMainRamCaptureTag* input =
                        page->Tags.data() + (pageOffset >> 1u);
                    for (u32 i = 0u; i < count; i++)
                        copyTag({epoch, input[i].ProductId,
                            input[i].SourceX, input[i].SourceY});
                }
                else
                {
                    for (u32 i = 0u; i < count; i++)
                        copyTag({});
                }
                offset += count * 2u;
            }
        }

        assert(FaithfulVramValidTagCounts[destination.Bank] >= oldValidCount);
        assert(FaithfulVramTotalValidTagCount >= oldValidCount);
        FaithfulVramValidTagCounts[destination.Bank] =
            FaithfulVramValidTagCounts[destination.Bank]
                - oldValidCount + newValidCount;
        FaithfulVramTotalValidTagCount =
            FaithfulVramTotalValidTagCount - oldValidCount + newValidCount;
        return;
    }

    for (u32 offset = 0u; offset < transferredByteCount; offset += 2u)
    {
        FaithfulVramCaptureTag tag {};
        if (sourceMayHaveTags
            && source.Kind == FaithfulDmaLineageStorageKind::Vram)
        {
            (void)GetFaithfulVramCaptureTag(
                source.Bank, (source.ByteOffset + offset) >> 1u, tag);
        }
        else if (sourceMayHaveTags
            && source.Kind == FaithfulDmaLineageStorageKind::MainRam)
        {
            (void)GetFaithfulMainRamCaptureTag(
                source.ByteOffset + offset, tag);
        }

        if (destination.Kind == FaithfulDmaLineageStorageKind::Vram)
        {
            SetFaithfulVramCaptureTag(
                destination.Bank,
                (destination.ByteOffset + offset) >> 1u, tag);
        }
        else if (destination.Kind
            == FaithfulDmaLineageStorageKind::MainRam)
        {
            SetFaithfulMainRamCaptureTag(
                destination.ByteOffset + offset, tag);
        }
    }
}

bool GPU::GetFaithfulCaptureTagForCpuAddress(
    u32 cpu, u32 addr, FaithfulVramCaptureTag& outTag,
    u8& outStorageBank) const noexcept
{
    Sync2D();
    if (!HasFaithfulCapturePhysicalTags())
    {
        outTag = {};
        outStorageBank = 0xFFu;
        return false;
    }
    u32 byteOffset = 0u;
    if (ResolveFaithfulMainRamAddress(cpu, addr, byteOffset))
    {
        outStorageBank = 0xFEu;
        if (FaithfulMainRamTaggedPageCount == 0u)
        {
            outTag = {};
            return false;
        }
        return GetFaithfulMainRamCaptureTag(byteOffset, outTag);
    }
    if (FaithfulVramTotalValidTagCount == 0u)
    {
        outTag = {};
        outStorageBank = 0xFFu;
        return false;
    }
    if (!ResolveFaithfulVramAddress(cpu, addr, outStorageBank, byteOffset))
    {
        outTag = {};
        outStorageBank = 0xFFu;
        return false;
    }
    return GetFaithfulVramCaptureTag(outStorageBank, byteOffset >> 1u, outTag);
}

void GPU::PropagateFaithfulCaptureTagForDmaDestination(
    u32 cpu, u32 addr, const FaithfulVramCaptureTag& tag) noexcept
{
    Sync2D();
    if (!FaithfulVramCaptureTrackingEnabled)
        return;
    u32 mainRamByteOffset = 0u;
    if (ResolveFaithfulMainRamAddress(cpu, addr, mainRamByteOffset))
    {
        SetFaithfulMainRamCaptureTag(mainRamByteOffset, tag);
        return;
    }
    u32 mask = 0u;
    if (cpu == 1u)
    {
        if ((addr & 0xFF800000u) != 0x06000000u
            && (addr & 0xFF800000u) != 0x06800000u)
            return;
        mask = VRAMMap_ARM7[(addr >> 17u) & 0x1u] & ((1u << 2u) | (1u << 3u));
    }
    else
    {
        if ((addr & 0xFF000000u) != 0x06000000u)
            return;
        switch (addr & 0x00E00000u)
        {
        case 0x00000000u: mask = VRAMMap_ABG[(addr >> 14u) & 0x1Fu] & 0x7Fu; break;
        case 0x00200000u: mask = VRAMMap_BBG[(addr >> 14u) & 0x7u]
            & ((1u << 2u) | (1u << 7u) | (1u << 8u)); break;
        case 0x00400000u: mask = VRAMMap_AOBJ[(addr >> 14u) & 0xFu]
            & ((1u << 0u) | (1u << 1u) | (1u << 4u) | (1u << 5u) | (1u << 6u)); break;
        case 0x00600000u: mask = VRAMMap_BOBJ[(addr >> 14u) & 0x7u]
            & ((1u << 3u) | (1u << 8u)); break;
        default:
        {
            u8 bank = 0xFFu;
            u32 byteOffset = 0u;
            if (ResolveFaithfulVramAddress(cpu, addr, bank, byteOffset))
                SetFaithfulVramCaptureTag(bank, byteOffset >> 1u, tag);
            return;
        }
        }
    }

    if (mask == 0u)
        return;
    while (mask != 0u)
    {
        const u32 bank = static_cast<u32>(__builtin_ctz(mask));
        mask &= mask - 1u;
        SetFaithfulVramCaptureTag(
            bank, (addr & VRAMMask[bank]) >> 1u, tag);
    }
}

bool GPU::HasFaithfulVramCaptureTags(u32 physicalBankMask) const noexcept
{
    if (!FaithfulVramCaptureTrackingEnabled)
        return false;
    for (u32 bank = 0u; bank < FaithfulVramValidTagCounts.size(); bank++)
    {
        if ((physicalBankMask & (1u << bank)) != 0u
            && FaithfulVramValidTagCounts[bank] != 0u)
        {
            return true;
        }
    }
    return false;
}

GPU::FaithfulVramCaptureTagResolution
GPU::ResolveFaithfulVramCaptureTagForBGAddress(
    u32 engine, u32 flatByteAddress,
    FaithfulVramCaptureTag& outTag,
    u8& outStorageBank) const noexcept
{
    FaithfulVramCaptureTagAddressResolver resolver {};
    PrepareFaithfulVramCaptureTagResolverForBG(engine, resolver);
    return ResolveFaithfulVramCaptureTagWithPreparedAddress(
        resolver, flatByteAddress, outTag, outStorageBank);
}

void GPU::PrepareFaithfulVramCaptureTagResolverForBG(
    u32 engine,
    FaithfulVramCaptureTagAddressResolver& resolver) const noexcept
{
    resolver = {};
    if (!FaithfulVramCaptureTrackingEnabled || engine >= 2u)
        return;

    resolver.ProductEpoch = FaithfulCaptureProductEpoch;
    if (engine == 0u)
    {
        resolver.Mapping = VRAMMap_ABG;
        resolver.MappingIndexMask = 0x1Fu;
        resolver.AddressMask = 0x7FFFFu;
        resolver.AllowedBankMask = 0x7Fu;
    }
    else
    {
        resolver.Mapping = VRAMMap_BBG;
        resolver.MappingIndexMask = 0x7u;
        resolver.AddressMask = 0x1FFFFu;
        resolver.AllowedBankMask =
            (1u << 2u) | (1u << 7u) | (1u << 8u);
    }
}

void GPU::PrepareFaithfulVramCaptureTagResolverForOBJ(
    u32 engine,
    FaithfulVramCaptureTagAddressResolver& resolver) const noexcept
{
    resolver = {};
    if (!FaithfulVramCaptureTrackingEnabled || engine >= 2u)
        return;

    resolver.ProductEpoch = FaithfulCaptureProductEpoch;
    if (engine == 0u)
    {
        resolver.Mapping = VRAMMap_AOBJ;
        resolver.MappingIndexMask = 0xFu;
        resolver.AddressMask = 0x3FFFFu;
        resolver.AllowedBankMask = (1u << 0u) | (1u << 1u)
            | (1u << 4u) | (1u << 5u) | (1u << 6u);
    }
    else
    {
        resolver.Mapping = VRAMMap_BOBJ;
        resolver.MappingIndexMask = 0x7u;
        resolver.AddressMask = 0x1FFFFu;
        resolver.AllowedBankMask = (1u << 3u) | (1u << 8u);
    }
}

bool GPU::ResolveFaithfulVramCaptureUniformMappedSpan(
    const FaithfulVramCaptureTagAddressResolver& resolver,
    u32 firstFlatByteAddress, u32 halfwordCount,
    FaithfulVramCaptureSpan& outSpan) const noexcept
{
    outSpan = {};
    if (!FaithfulVramCaptureTrackingEnabled
        || resolver.Mapping == nullptr || halfwordCount == 0u
        || resolver.ProductEpoch == 0u
        || resolver.ProductEpoch != FaithfulCaptureProductEpoch)
    {
        return false;
    }

    const u32 address = firstFlatByteAddress & resolver.AddressMask;
    const u64 lastAddress = static_cast<u64>(address)
        + (static_cast<u64>(halfwordCount) - 1u) * 2u;
    if ((address & 1u) != 0u || lastAddress > resolver.AddressMask
        || (address >> 14u) != (lastAddress >> 14u))
    {
        return false;
    }
    const u32 mask = resolver.Mapping[
        (address >> 14u) & resolver.MappingIndexMask]
        & resolver.AllowedBankMask;
    if (mask == 0u || (mask & (mask - 1u)) != 0u)
        return false;

    const u32 bank = static_cast<u32>(__builtin_ctz(mask));
    if (!ResolveFaithfulVramCaptureUniformSpan(
            bank, (address & VRAMMask[bank]) >> 1u,
            halfwordCount, outSpan))
    {
        return false;
    }
    if (static_cast<u64>(outSpan.SourceXBase) + halfwordCount - 1u
        > std::numeric_limits<u16>::max())
    {
        outSpan = {};
        return false;
    }
    return true;
}

GPU::FaithfulVramCaptureTagResolution
GPU::ResolveFaithfulVramCaptureTagWithPreparedAddress(
    FaithfulVramCaptureTagAddressResolver& resolver,
    u32 flatByteAddress,
    FaithfulVramCaptureTag& outTag,
    u8& outStorageBank) const noexcept
{
    outTag = {};
    outStorageBank = 0xFFu;
    if (!FaithfulVramCaptureTrackingEnabled
        || resolver.Mapping == nullptr
        || resolver.ProductEpoch == 0u
        || resolver.ProductEpoch != FaithfulCaptureProductEpoch)
    {
        return FaithfulVramCaptureTagResolution::Native;
    }

    const u32 address = flatByteAddress & resolver.AddressMask;
    const u32 segment = (address >> 14u) & resolver.MappingIndexMask;
    const u32 mask = resolver.Mapping[segment]
        & resolver.AllowedBankMask;
    if (segment != resolver.CachedSegment
        || mask != resolver.CachedPhysicalBankMask)
    {
        resolver.CachedSegment = segment;
        resolver.CachedPhysicalBankMask = mask;
        resolver.CachedUniqueBank =
            mask != 0u && (mask & (mask - 1u)) == 0u
                ? static_cast<u8>(__builtin_ctz(mask))
                : 0xFFu;
    }
    if (mask == 0u)
        return FaithfulVramCaptureTagResolution::Native;
    if (resolver.CachedUniqueBank == 0xFFu)
    {
        u32 remaining = mask;
        while (remaining != 0u)
        {
            const u32 bank = static_cast<u32>(__builtin_ctz(remaining));
            remaining &= remaining - 1u;
            const FaithfulVramCaptureTag& candidate =
                FaithfulVramCaptureTags[bank]
                    [(address & VRAMMask[bank]) >> 1u];
            if (candidate.Valid()
                && candidate.ProductEpoch == FaithfulCaptureProductEpoch)
            {
                return FaithfulVramCaptureTagResolution::Ambiguous;
            }
        }
        return FaithfulVramCaptureTagResolution::Native;
    }

    const u8 bank = resolver.CachedUniqueBank;
    const FaithfulVramCaptureTag& tag = FaithfulVramCaptureTags[bank]
        [(address & VRAMMask[bank]) >> 1u];
    if (!tag.Valid()
        || tag.ProductEpoch != FaithfulCaptureProductEpoch)
    {
        return FaithfulVramCaptureTagResolution::Native;
    }
    outTag = tag;
    outStorageBank = bank;
    return FaithfulVramCaptureTagResolution::Exact;
}

bool GPU::GetFaithfulVramCaptureTagForBBGAddress(
    u32 flatByteAddress, FaithfulVramCaptureTag& outTag,
    u8& outStorageBank) const noexcept
{
    return ResolveFaithfulVramCaptureTagForBGAddress(
        1u, flatByteAddress, outTag, outStorageBank)
        == FaithfulVramCaptureTagResolution::Exact;
}

namespace
{
thread_local bool sIsRender2DWorker = false;
}

bool GPU::IsRender2DWorkerThread() noexcept
{
    return sIsRender2DWorker;
}

// One visible line's 2D work, in the order the emulation thread used to do it.
void GPU::DrawScanline2D(u32 line, u32 vcount) noexcept
{
    GPU2D_Renderer->DrawScanline(line, &GPU2D_A);
    GPU2D_Renderer->DrawScanline(line, &GPU2D_B);
    if (line < 191)
    {
        GPU2D_Renderer->DrawSprites(line + 1, &GPU2D_A);
        GPU2D_Renderer->DrawSprites(line + 1, &GPU2D_B);
    }
    GPU2D_A.CheckWindows(vcount);
    GPU2D_B.CheckWindows(vcount);
}

void GPU::SetThreaded2D(bool enabled) noexcept
{
    if (enabled == Threaded2D)
        return;

    if (!enabled)
    {
        Sync2D();
        Threaded2D = false;
        StopRender2DWorker();
        return;
    }

    Render2DPosted.store(0, std::memory_order_relaxed);
    Render2DDone.store(0, std::memory_order_relaxed);
    Render2DExit.store(false, std::memory_order_relaxed);
    Render2DWorker = std::thread([this] { Render2DWorkerLoop(); });
    Threaded2D = true;
}

void GPU::StopRender2DWorker() noexcept
{
    if (!Render2DWorker.joinable())
        return;
    {
        std::lock_guard lock(Render2DMutex);
        Render2DExit.store(true, std::memory_order_release);
    }
    Render2DWake.notify_all();
    Render2DWorker.join();
}

void GPU::PostRender2DLine(u32 line, u32 vcount) noexcept
{
    const u32 posted = Render2DPosted.load(std::memory_order_relaxed);
    // keep the queue bounded; the worker is normally at most a line behind
    while (posted - Render2DDone.load(std::memory_order_acquire) >= Render2DQueueSize)
        std::this_thread::yield();

    Render2DQueue[posted % Render2DQueueSize] = (vcount << 16) | line;
    Render2DPosted.store(posted + 1, std::memory_order_release);

    if (Render2DSleeping.load(std::memory_order_acquire))
    {
        std::lock_guard lock(Render2DMutex);
        Render2DWake.notify_one();
    }
}

void GPU::WaitRender2D() const noexcept
{
    const u32 target = Render2DPosted.load(std::memory_order_relaxed);
    while (Render2DDone.load(std::memory_order_acquire) != target)
    {
        if (Render2DSleeping.load(std::memory_order_acquire))
        {
            std::lock_guard lock(Render2DMutex);
            Render2DWake.notify_one();
        }
#if defined(__aarch64__)
        asm volatile("yield");
#endif
    }
}

void GPU::Render2DWorkerLoop() noexcept
{
    sIsRender2DWorker = true;
    if (Render2DWorkerInit)
        Render2DWorkerInit();

    // Lines arrive about every 60 microseconds while the screen is drawn.
    // Spinning that long is far cheaper than a kernel wakeup per line; the
    // worker only sleeps over VBlank and pauses.
    constexpr u64 kSpinUs = 300;

    u32 done = Render2DDone.load(std::memory_order_relaxed);
    while (!Render2DExit.load(std::memory_order_acquire))
    {
        if (done != Render2DPosted.load(std::memory_order_acquire))
        {
            const u32 entry = Render2DQueue[done % Render2DQueueSize];
            DrawScanline2D(entry & 0xFFFF, entry >> 16);
            done++;
            Render2DDone.store(done, std::memory_order_release);
            continue;
        }

        const u64 spinStart = Platform::GetUSCount();
        bool gotWork = false;
        while (Platform::GetUSCount() - spinStart < kSpinUs)
        {
            if (done != Render2DPosted.load(std::memory_order_acquire)
                || Render2DExit.load(std::memory_order_acquire))
            {
                gotWork = true;
                break;
            }
#if defined(__aarch64__)
            asm volatile("yield");
#endif
        }
        if (gotWork)
            continue;

        std::unique_lock lock(Render2DMutex);
        Render2DSleeping.store(true, std::memory_order_release);
        Render2DWake.wait(lock, [&] {
            return done != Render2DPosted.load(std::memory_order_acquire)
                || Render2DExit.load(std::memory_order_acquire);
        });
        Render2DSleeping.store(false, std::memory_order_release);
    }
}

GPU::~GPU() noexcept
{
    StopRender2DWorker();

    ClearFaithfulCaptureTags();

    NDS.UnregisterEventFuncs(Event_LCD);
    NDS.UnregisterEventFuncs(Event_DisplayFIFO);
}

void GPU::ResetVRAMCache() noexcept
{
    VRAMCacheEpoch++;
    InvalidateFaithfulCaptureLineage();
    for (int i = 0; i < 9; i++)
        VRAMDirty[i] = NonStupidBitField<128*1024/VRAMDirtyGranularity>();

    for (auto& dirty : VRAMDirty_LCDC)
        dirty.Clear();

    VRAMDirty_ABG.Reset();
    VRAMDirty_BBG.Reset();
    VRAMDirty_AOBJ.Reset();
    VRAMDirty_BOBJ.Reset();
    VRAMDirty_ABGExtPal.Reset();
    VRAMDirty_BBGExtPal.Reset();
    VRAMDirty_AOBJExtPal.Reset();
    VRAMDirty_BOBJExtPal.Reset();
    VRAMDirty_Texture.Reset();
    VRAMDirty_TexPal.Reset();

    memset(VRAMFlat_ABG, 0, sizeof(VRAMFlat_ABG));
    memset(VRAMFlat_BBG, 0, sizeof(VRAMFlat_BBG));
    memset(VRAMFlat_AOBJ, 0, sizeof(VRAMFlat_AOBJ));
    memset(VRAMFlat_BOBJ, 0, sizeof(VRAMFlat_BOBJ));
    memset(VRAMFlat_ABGExtPal, 0, sizeof(VRAMFlat_ABGExtPal));
    memset(VRAMFlat_BBGExtPal, 0, sizeof(VRAMFlat_BBGExtPal));
    memset(VRAMFlat_AOBJExtPal, 0, sizeof(VRAMFlat_AOBJExtPal));
    memset(VRAMFlat_BOBJExtPal, 0, sizeof(VRAMFlat_BOBJExtPal));
    memset(VRAMFlat_Texture, 0, sizeof(VRAMFlat_Texture));
    memset(VRAMFlat_TexPal, 0, sizeof(VRAMFlat_TexPal));
}

void GPU::Reset() noexcept
{
    Sync2D();
    VCount = 0;
    NextVCount = -1;
    TotalScanlines = 0;

    DispStat[0] = 0;
    DispStat[1] = 0;
    VMatch[0] = 0;
    VMatch[1] = 0;

    memset(Palette, 0, 2*1024);
    memset(OAM, 0, 2*1024);

    memset(VRAM_A, 0, 128*1024);
    memset(VRAM_B, 0, 128*1024);
    memset(VRAM_C, 0, 128*1024);
    memset(VRAM_D, 0, 128*1024);
    memset(VRAM_E, 0,  64*1024);
    memset(VRAM_F, 0,  16*1024);
    memset(VRAM_G, 0,  16*1024);
    memset(VRAM_H, 0,  32*1024);
    memset(VRAM_I, 0,  16*1024);

    memset(VRAMCNT, 0, 9);
    VRAMSTAT = 0;

    VRAMMap_LCDC = 0;

    memset(VRAMMap_ABG, 0, sizeof(VRAMMap_ABG));
    memset(VRAMMap_AOBJ, 0, sizeof(VRAMMap_AOBJ));
    memset(VRAMMap_BBG, 0, sizeof(VRAMMap_BBG));
    memset(VRAMMap_BOBJ, 0, sizeof(VRAMMap_BOBJ));

    memset(VRAMMap_ABGExtPal, 0, sizeof(VRAMMap_ABGExtPal));
    VRAMMap_AOBJExtPal = 0;
    memset(VRAMMap_BBGExtPal, 0, sizeof(VRAMMap_BBGExtPal));
    VRAMMap_BOBJExtPal = 0;

    memset(VRAMMap_Texture, 0, sizeof(VRAMMap_Texture));
    memset(VRAMMap_TexPal, 0, sizeof(VRAMMap_TexPal));

    VRAMMap_ARM7[0] = 0;
    VRAMMap_ARM7[1] = 0;

    memset(VRAMPtr_ABG, 0, sizeof(VRAMPtr_ABG));
    memset(VRAMPtr_AOBJ, 0, sizeof(VRAMPtr_AOBJ));
    memset(VRAMPtr_BBG, 0, sizeof(VRAMPtr_BBG));
    memset(VRAMPtr_BOBJ, 0, sizeof(VRAMPtr_BOBJ));

    size_t fbsize;
    if (GPU3D.IsRendererAccelerated())
        fbsize = (256*3 + 1) * 192;
    else
        fbsize = 256 * 192;

    for (size_t i = 0; i < fbsize; i++)
    {
        Framebuffer[0][0][i] = 0xFFFFFFFF;
        Framebuffer[1][0][i] = 0xFFFFFFFF;
    }
    for (size_t i = 0; i < fbsize; i++)
    {
        Framebuffer[0][1][i] = 0xFFFFFFFF;
        Framebuffer[1][1][i] = 0xFFFFFFFF;
    }

    GPU2D_A.Reset();
    if (GPU2D_Renderer)
        GPU2D_Renderer->ResetFrameskipState();
    GPU2D_B.Reset();
    GPU3D.Reset();

    int backbuf = FrontBuffer ? 0 : 1;
    GPU2D_Renderer->SetFramebuffer(
        Framebuffer[backbuf][1].get(),
        Framebuffer[backbuf][0].get(),
        static_cast<u8>(backbuf),
        GPU2D::PhysicalScreen::Bottom);

    ResetVRAMCache();

    OAMDirty = 0x3;
    PaletteDirty = 0xF;
}

void GPU::Stop() noexcept
{
    Sync2D();
    int fbsize;
    if (GPU3D.IsRendererAccelerated())
        fbsize = (256*3 + 1) * 192;
    else
        fbsize = 256 * 192;

    memset(Framebuffer[0][0].get(), 0, fbsize*4);
    memset(Framebuffer[0][1].get(), 0, fbsize*4);
    memset(Framebuffer[1][0].get(), 0, fbsize*4);
    memset(Framebuffer[1][1].get(), 0, fbsize*4);

    GPU3D.Stop(*this);
}

void GPU::DoSavestate(Savestate* file) noexcept
{
    Sync2D();
    file->Section("GPUG");

    file->Var16(&VCount);
    file->Var32(&NextVCount);
    file->Var16(&TotalScanlines);

    file->Var16(&DispStat[0]);
    file->Var16(&DispStat[1]);
    file->Var16(&VMatch[0]);
    file->Var16(&VMatch[1]);

    file->VarArray(Palette, 2*1024);
    file->VarArray(OAM, 2*1024);

    file->VarArray(VRAM_A, 128*1024);
    file->VarArray(VRAM_B, 128*1024);
    file->VarArray(VRAM_C, 128*1024);
    file->VarArray(VRAM_D, 128*1024);
    file->VarArray(VRAM_E,  64*1024);
    file->VarArray(VRAM_F,  16*1024);
    file->VarArray(VRAM_G,  16*1024);
    file->VarArray(VRAM_H,  32*1024);
    file->VarArray(VRAM_I,  16*1024);

    file->VarArray(VRAMCNT, 9);
    file->Var8(&VRAMSTAT);

    file->Var32(&VRAMMap_LCDC);

    file->VarArray(VRAMMap_ABG, sizeof(VRAMMap_ABG));
    file->VarArray(VRAMMap_AOBJ, sizeof(VRAMMap_AOBJ));
    file->VarArray(VRAMMap_BBG, sizeof(VRAMMap_BBG));
    file->VarArray(VRAMMap_BOBJ, sizeof(VRAMMap_BOBJ));

    file->VarArray(VRAMMap_ABGExtPal, sizeof(VRAMMap_ABGExtPal));
    file->Var32(&VRAMMap_AOBJExtPal);
    file->VarArray(VRAMMap_BBGExtPal, sizeof(VRAMMap_BBGExtPal));
    file->Var32(&VRAMMap_BOBJExtPal);

    file->VarArray(VRAMMap_Texture, sizeof(VRAMMap_Texture));
    file->VarArray(VRAMMap_TexPal, sizeof(VRAMMap_TexPal));

    file->Var32(&VRAMMap_ARM7[0]);
    file->Var32(&VRAMMap_ARM7[1]);

    if (!file->Saving)
    {
        for (int i = 0; i < 0x20; i++)
            VRAMPtr_ABG[i] = GetUniqueBankPtr(VRAMMap_ABG[i], i << 14);
        for (int i = 0; i < 0x10; i++)
            VRAMPtr_AOBJ[i] = GetUniqueBankPtr(VRAMMap_AOBJ[i], i << 14);
        for (int i = 0; i < 0x8; i++)
            VRAMPtr_BBG[i] = GetUniqueBankPtr(VRAMMap_BBG[i], i << 14);
        for (int i = 0; i < 0x8; i++)
            VRAMPtr_BOBJ[i] = GetUniqueBankPtr(VRAMMap_BOBJ[i], i << 14);
    }

    GPU2D_A.DoSavestate(file);
    GPU2D_B.DoSavestate(file);
    GPU3D.DoSavestate(file);

    if (!file->Saving)
        ResetVRAMCache();
}

namespace
{

bool LogVuelcoActivo()
{
    static const bool activo = getenv("MELON_LOG_VUELCO") != nullptr;
    return activo;
}
u32 sVuelcoFotograma = 0;
u32 sVuelcoSwapAsignado = 0;
}

void GPU::AssignFramebuffers() noexcept
{
    int backbuf = FrontBuffer ? 0 : 1;
    if (NDS.PowerControl9 & (1<<15))
    {
        GPU2D_Renderer->SetFramebuffer(
            Framebuffer[backbuf][0].get(),
            Framebuffer[backbuf][1].get(),
            static_cast<u8>(backbuf),
            GPU2D::PhysicalScreen::Top);
    }
    else
    {
        GPU2D_Renderer->SetFramebuffer(
            Framebuffer[backbuf][1].get(),
            Framebuffer[backbuf][0].get(),
            static_cast<u8>(backbuf),
            GPU2D::PhysicalScreen::Bottom);
    }
    sVuelcoSwapAsignado = (NDS.PowerControl9 >> 15) & 1u;

    SwapSelladoBuffer[backbuf] = sVuelcoSwapAsignado;
}

void GPU::SetRenderer3D(std::unique_ptr<Renderer3D>&& renderer) noexcept
{
    Sync2D();
    if (renderer == nullptr)
        GPU3D.SetCurrentRenderer(std::make_unique<SoftRenderer>());
    else
        GPU3D.SetCurrentRenderer(std::move(renderer));

    InitFramebuffers();
}

void GPU::InitFramebuffers() noexcept
{
    int fbsize;
    if (GPU3D.IsRendererAccelerated())
        fbsize = (256*3 + 1) * 192;
    else
        fbsize = 256 * 192;

    Framebuffer[0][0] = std::make_unique<u32[]>(fbsize);
    Framebuffer[1][0] = std::make_unique<u32[]>(fbsize);
    Framebuffer[0][1] = std::make_unique<u32[]>(fbsize);
    Framebuffer[1][1] = std::make_unique<u32[]>(fbsize);

    memset(Framebuffer[0][0].get(), 0, fbsize*4);
    memset(Framebuffer[1][0].get(), 0, fbsize*4);
    memset(Framebuffer[0][1].get(), 0, fbsize*4);
    memset(Framebuffer[1][1].get(), 0, fbsize*4);

    AssignFramebuffers();
}


// VRAM mapping notes
//
// mirroring:
// unmapped range reads zero
// LCD is mirrored every 0x100000 bytes, the gap between each mirror reads zero
// ABG:
//   bank A,B,C,D,E mirror every 0x80000 bytes
//   bank F,G mirror at base+0x8000, mirror every 0x80000 bytes
// AOBJ:
//   bank A,B,E mirror every 0x40000 bytes
//   bank F,G mirror at base+0x8000, mirror every 0x40000 bytes
// BBG:
//   bank C mirrors every 0x20000 bytes
//   bank H mirrors every 0x10000 bytes
//   bank I mirrors at base+0x4000, mirrors every 0x10000 bytes
// BOBJ:
//   bank D mirrors every 0x20000 bytes
//   bank I mirrors every 0x4000 bytes
//
// untested:
//   ARM7 (TODO)
//   extended palette (mirroring doesn't apply)
//   texture/texpal (does mirroring apply?)
//   -> trying to use extpal/texture/texpal with no VRAM mapped.
//      would likely read all black, but has to be tested.
//
// overlap:
// when reading: values are read from each bank and ORed together
// when writing: value is written to each bank

u8* GPU::GetUniqueBankPtr(u32 mask, u32 offset) noexcept
{
    if (!mask || (mask & (mask - 1)) != 0) return NULL;
    int num = __builtin_ctz(mask);
    return &VRAM[num][offset & VRAMMask[num]];
}

const u8* GPU::GetUniqueBankPtr(u32 mask, u32 offset) const noexcept
{
    if (!mask || (mask & (mask - 1)) != 0) return NULL;
    int num = __builtin_ctz(mask);
    return &VRAM[num][offset & VRAMMask[num]];
}

#define MAP_RANGE(map, base, n)    for (int i = 0; i < n; i++) VRAMMap_##map[(base)+i] |= bankmask;
#define UNMAP_RANGE(map, base, n)  for (int i = 0; i < n; i++) VRAMMap_##map[(base)+i] &= ~bankmask;

#define MAP_RANGE_PTR(map, base, n) \
    for (int i = 0; i < n; i++) { VRAMMap_##map[(base)+i] |= bankmask; VRAMPtr_##map[(base)+i] = GetUniqueBankPtr(VRAMMap_##map[(base)+i], ((base)+i)<<14); }
#define UNMAP_RANGE_PTR(map, base, n) \
    for (int i = 0; i < n; i++) { VRAMMap_##map[(base)+i] &= ~bankmask; VRAMPtr_##map[(base)+i] = GetUniqueBankPtr(VRAMMap_##map[(base)+i], ((base)+i)<<14); }

void GPU::MapVRAM_AB(u32 bank, u8 cnt) noexcept
{
    cnt &= 0x9B;

    u8 oldcnt = VRAMCNT[bank];
    VRAMCNT[bank] = cnt;

    if (oldcnt == cnt) return;

    u8 oldofs = (oldcnt >> 3) & 0x3;
    u8 ofs = (cnt >> 3) & 0x3;
    u32 bankmask = 1 << bank;

    if (oldcnt & (1<<7))
    {
        switch (oldcnt & 0x3)
        {
        case 0: // LCDC
            VRAMMap_LCDC &= ~bankmask;
            break;

        case 1: // ABG
            UNMAP_RANGE_PTR(ABG, oldofs<<3, 8);
            break;

        case 2: // AOBJ
            oldofs &= 0x1;
            UNMAP_RANGE_PTR(AOBJ, oldofs<<3, 8);
            break;

        case 3: // texture
            VRAMMap_Texture[oldofs] &= ~bankmask;
            break;
        }
    }

    if (cnt & (1<<7))
    {
        switch (cnt & 0x3)
        {
        case 0: // LCDC
            VRAMMap_LCDC |= bankmask;

            if ((oldcnt & 0x83u) != 0x80u)
                VRAMDirty_LCDC[bank].SetRange(0, 128*1024/VRAMDirtyGranularity);
            break;

        case 1: // ABG
            MAP_RANGE_PTR(ABG, ofs<<3, 8);
            break;

        case 2: // AOBJ
            ofs &= 0x1;
            MAP_RANGE_PTR(AOBJ, ofs<<3, 8);
            break;

        case 3: // texture
            VRAMMap_Texture[ofs] |= bankmask;
            if (bank < 4) BancoFueTextura[bank] = true;
            break;
        }
    }
}

void GPU::MapVRAM_CD(u32 bank, u8 cnt) noexcept
{
    cnt &= 0x9F;

    u8 oldcnt = VRAMCNT[bank];
    VRAMCNT[bank] = cnt;

    VRAMSTAT &= ~(1 << (bank-2));

    if (oldcnt == cnt) return;

    if (LogVuelcoActivo())
        fprintf(stderr, "[vuelco] f=%u vc=%u VRAMCNT %c %02X->%02X\n",
                sVuelcoFotograma, VCount, "ABCD"[bank], oldcnt, cnt);

    u8 oldofs = (oldcnt >> 3) & 0x7;
    u8 ofs = (cnt >> 3) & 0x7;
    u32 bankmask = 1 << bank;

    if (oldcnt & (1<<7))
    {
        switch (oldcnt & 0x7)
        {
        case 0: // LCDC
            VRAMMap_LCDC &= ~bankmask;
            break;

        case 1: // ABG
            UNMAP_RANGE_PTR(ABG, oldofs<<3, 8);
            break;

        case 2: // ARM7 VRAM
            oldofs &= 0x1;
            VRAMMap_ARM7[oldofs] &= ~bankmask;
            break;

        case 3: // texture
            VRAMMap_Texture[oldofs] &= ~bankmask;
            break;

        case 4: // BBG/BOBJ
            if (bank == 2)
            {
                UNMAP_RANGE_PTR(BBG, 0, 8);
            }
            else
            {
                UNMAP_RANGE_PTR(BOBJ, 0, 8);
            }
            break;
        }
    }

    if (cnt & (1<<7))
    {
        switch (cnt & 0x7)
        {
        case 0: // LCDC
            VRAMMap_LCDC |= bankmask;
            if ((oldcnt & 0x87u) != 0x80u)
                VRAMDirty_LCDC[bank].SetRange(0, 128*1024/VRAMDirtyGranularity);
            break;

        case 1: // ABG
            MAP_RANGE_PTR(ABG, ofs<<3, 8);
            break;

        case 2: // ARM7 VRAM
            ofs &= 0x1;
            VRAMMap_ARM7[ofs] |= bankmask;
            memset(VRAMDirty[bank].Data, 0xFF, sizeof(VRAMDirty[bank].Data));
            VRAMSTAT |= (1 << (bank-2));
            NDS.JIT.CheckAndInvalidateWVRAM(ofs);
            break;

        case 3: // texture
            VRAMMap_Texture[ofs] |= bankmask;
            if (bank < 4) BancoFueTextura[bank] = true;
            break;

        case 4: // BBG/BOBJ
            if (bank == 2)
            {
                MAP_RANGE_PTR(BBG, 0, 8);
            }
            else
            {
                MAP_RANGE_PTR(BOBJ, 0, 8);
            }
            break;
        }
    }
}

void GPU::MapVRAM_E(u32 bank, u8 cnt) noexcept
{
    cnt &= 0x87;

    u8 oldcnt = VRAMCNT[bank];
    VRAMCNT[bank] = cnt;

    if (oldcnt == cnt) return;

    u32 bankmask = 1 << bank;

    if (oldcnt & (1<<7))
    {
        switch (oldcnt & 0x7)
        {
        case 0: // LCDC
            VRAMMap_LCDC &= ~bankmask;
            break;

        case 1: // ABG
            UNMAP_RANGE_PTR(ABG, 0, 4);
            break;

        case 2: // AOBJ
            UNMAP_RANGE_PTR(AOBJ, 0, 4);
            break;

        case 3: // texture palette
            UNMAP_RANGE(TexPal, 0, 4);
            break;

        case 4: // ABG ext palette
            UNMAP_RANGE(ABGExtPal, 0, 4);
            break;
        }
    }

    if (cnt & (1<<7))
    {
        switch (cnt & 0x7)
        {
        case 0: // LCDC
            VRAMMap_LCDC |= bankmask;
            break;

        case 1: // ABG
            MAP_RANGE_PTR(ABG, 0, 4);
            break;

        case 2: // AOBJ
            MAP_RANGE_PTR(AOBJ, 0, 4);
            break;

        case 3: // texture palette
            MAP_RANGE(TexPal, 0, 4);
            break;

        case 4: // ABG ext palette
            MAP_RANGE(ABGExtPal, 0, 4);
            break;
        }
    }
}

void GPU::MapVRAM_FG(u32 bank, u8 cnt) noexcept
{
    cnt &= 0x9F;

    u8 oldcnt = VRAMCNT[bank];
    VRAMCNT[bank] = cnt;

    if (oldcnt == cnt) return;

    u8 oldofs = (oldcnt >> 3) & 0x7;
    u8 ofs = (cnt >> 3) & 0x7;
    u32 bankmask = 1 << bank;

    if (oldcnt & (1<<7))
    {
        switch (oldcnt & 0x7)
        {
        case 0: // LCDC
            VRAMMap_LCDC &= ~bankmask;
            break;

        case 1: // ABG
            {
                u32 base = (oldofs & 0x1) + ((oldofs & 0x2) << 1);
                VRAMMap_ABG[base] &= ~bankmask;
                VRAMMap_ABG[base + 2] &= ~bankmask;
                VRAMPtr_ABG[base] = GetUniqueBankPtr(VRAMMap_ABG[base], base << 14);
                VRAMPtr_ABG[base + 2] = GetUniqueBankPtr(VRAMMap_ABG[base + 2], (base + 2) << 14);
            }
            break;

        case 2: // AOBJ
            {
                u32 base = (oldofs & 0x1) + ((oldofs & 0x2) << 1);
                VRAMMap_AOBJ[base] &= ~bankmask;
                VRAMMap_AOBJ[base + 2] &= ~bankmask;
                VRAMPtr_AOBJ[base] = GetUniqueBankPtr(VRAMMap_AOBJ[base], base << 14);
                VRAMPtr_AOBJ[base + 2] = GetUniqueBankPtr(VRAMMap_AOBJ[base + 2], (base + 2) << 14);
            }
            break;

        case 3: // texture palette
            VRAMMap_TexPal[(oldofs & 0x1) + ((oldofs & 0x2) << 1)] &= ~bankmask;
            break;

        case 4: // ABG ext palette
            VRAMMap_ABGExtPal[((oldofs & 0x1) << 1)] &= ~bankmask;
            VRAMMap_ABGExtPal[((oldofs & 0x1) << 1) + 1] &= ~bankmask;
            break;

        case 5: // AOBJ ext palette
            VRAMMap_AOBJExtPal &= ~bankmask;
            break;
        }
    }

    if (cnt & (1<<7))
    {
        switch (cnt & 0x7)
        {
        case 0: // LCDC
            VRAMMap_LCDC |= bankmask;
            break;

        case 1: // ABG
            {
                u32 base = (ofs & 0x1) + ((ofs & 0x2) << 1);
                VRAMMap_ABG[base] |= bankmask;
                VRAMMap_ABG[base + 2] |= bankmask;
                VRAMPtr_ABG[base] = GetUniqueBankPtr(VRAMMap_ABG[base], base << 14);
                VRAMPtr_ABG[base + 2] = GetUniqueBankPtr(VRAMMap_ABG[base + 2], (base + 2) << 14);
            }
            break;

        case 2: // AOBJ
            {
                u32 base = (ofs & 0x1) + ((ofs & 0x2) << 1);
                VRAMMap_AOBJ[base] |= bankmask;
                VRAMMap_AOBJ[base + 2] |= bankmask;
                VRAMPtr_AOBJ[base] = GetUniqueBankPtr(VRAMMap_AOBJ[base], base << 14);
                VRAMPtr_AOBJ[base + 2] = GetUniqueBankPtr(VRAMMap_AOBJ[base + 2], (base + 2) << 14);
            }
            break;

        case 3: // texture palette
            VRAMMap_TexPal[(ofs & 0x1) + ((ofs & 0x2) << 1)] |= bankmask;
            break;

        case 4: // ABG ext palette
            VRAMMap_ABGExtPal[((ofs & 0x1) << 1)] |= bankmask;
            VRAMMap_ABGExtPal[((ofs & 0x1) << 1) + 1] |= bankmask;
            break;

        case 5: // AOBJ ext palette
            VRAMMap_AOBJExtPal |= bankmask;
            break;
        }
    }
}

void GPU::MapVRAM_H(u32 bank, u8 cnt) noexcept
{
    cnt &= 0x83;

    u8 oldcnt = VRAMCNT[bank];
    VRAMCNT[bank] = cnt;

    if (oldcnt == cnt) return;

    if (LogVuelcoActivo())
        fprintf(stderr, "[vuelco] f=%u vc=%u VRAMCNT H %02X->%02X\n",
                sVuelcoFotograma, VCount, oldcnt, cnt);

    u32 bankmask = 1 << bank;

    if (oldcnt & (1<<7))
    {
        switch (oldcnt & 0x3)
        {
        case 0: // LCDC
            VRAMMap_LCDC &= ~bankmask;
            break;

        case 1: // BBG
            VRAMMap_BBG[0] &= ~bankmask;
            VRAMMap_BBG[1] &= ~bankmask;
            VRAMMap_BBG[4] &= ~bankmask;
            VRAMMap_BBG[5] &= ~bankmask;
            VRAMPtr_BBG[0] = GetUniqueBankPtr(VRAMMap_BBG[0], 0 << 14);
            VRAMPtr_BBG[1] = GetUniqueBankPtr(VRAMMap_BBG[1], 1 << 14);
            VRAMPtr_BBG[4] = GetUniqueBankPtr(VRAMMap_BBG[4], 4 << 14);
            VRAMPtr_BBG[5] = GetUniqueBankPtr(VRAMMap_BBG[5], 5 << 14);
            break;

        case 2: // BBG ext palette
            UNMAP_RANGE(BBGExtPal, 0, 4);
            break;
        }
    }

    if (cnt & (1<<7))
    {
        switch (cnt & 0x3)
        {
        case 0: // LCDC
            VRAMMap_LCDC |= bankmask;
            break;

        case 1: // BBG
            VRAMMap_BBG[0] |= bankmask;
            VRAMMap_BBG[1] |= bankmask;
            VRAMMap_BBG[4] |= bankmask;
            VRAMMap_BBG[5] |= bankmask;
            VRAMPtr_BBG[0] = GetUniqueBankPtr(VRAMMap_BBG[0], 0 << 14);
            VRAMPtr_BBG[1] = GetUniqueBankPtr(VRAMMap_BBG[1], 1 << 14);
            VRAMPtr_BBG[4] = GetUniqueBankPtr(VRAMMap_BBG[4], 4 << 14);
            VRAMPtr_BBG[5] = GetUniqueBankPtr(VRAMMap_BBG[5], 5 << 14);
            break;

        case 2: // BBG ext palette
            MAP_RANGE(BBGExtPal, 0, 4);
            break;
        }
    }
}

void GPU::MapVRAM_I(u32 bank, u8 cnt) noexcept
{
    cnt &= 0x83;

    u8 oldcnt = VRAMCNT[bank];
    VRAMCNT[bank] = cnt;

    if (oldcnt == cnt) return;

    if (LogVuelcoActivo())
        fprintf(stderr, "[vuelco] f=%u vc=%u VRAMCNT I %02X->%02X\n",
                sVuelcoFotograma, VCount, oldcnt, cnt);

    u32 bankmask = 1 << bank;

    if (oldcnt & (1<<7))
    {
        switch (oldcnt & 0x3)
        {
        case 0: // LCDC
            VRAMMap_LCDC &= ~bankmask;
            break;

        case 1: // BBG
            VRAMMap_BBG[2] &= ~bankmask;
            VRAMMap_BBG[3] &= ~bankmask;
            VRAMMap_BBG[6] &= ~bankmask;
            VRAMMap_BBG[7] &= ~bankmask;
            VRAMPtr_BBG[2] = GetUniqueBankPtr(VRAMMap_BBG[2], 2 << 14);
            VRAMPtr_BBG[3] = GetUniqueBankPtr(VRAMMap_BBG[3], 3 << 14);
            VRAMPtr_BBG[6] = GetUniqueBankPtr(VRAMMap_BBG[6], 6 << 14);
            VRAMPtr_BBG[7] = GetUniqueBankPtr(VRAMMap_BBG[7], 7 << 14);
            break;

        case 2: // BOBJ
            UNMAP_RANGE_PTR(BOBJ, 0, 8);
            break;

        case 3: // BOBJ ext palette
            VRAMMap_BOBJExtPal &= ~bankmask;
            break;
        }
    }

    if (cnt & (1<<7))
    {
        switch (cnt & 0x3)
        {
        case 0: // LCDC
            VRAMMap_LCDC |= bankmask;
            break;

        case 1: // BBG
            VRAMMap_BBG[2] |= bankmask;
            VRAMMap_BBG[3] |= bankmask;
            VRAMMap_BBG[6] |= bankmask;
            VRAMMap_BBG[7] |= bankmask;
            VRAMPtr_BBG[2] = GetUniqueBankPtr(VRAMMap_BBG[2], 2 << 14);
            VRAMPtr_BBG[3] = GetUniqueBankPtr(VRAMMap_BBG[3], 3 << 14);
            VRAMPtr_BBG[6] = GetUniqueBankPtr(VRAMMap_BBG[6], 6 << 14);
            VRAMPtr_BBG[7] = GetUniqueBankPtr(VRAMMap_BBG[7], 7 << 14);
            break;

        case 2: // BOBJ
            MAP_RANGE_PTR(BOBJ, 0, 8);
            break;

        case 3: // BOBJ ext palette
            VRAMMap_BOBJExtPal |= bankmask;
            break;
        }
    }
}


void GPU::SetPowerCnt(u32 val) noexcept
{
    if (LogVuelcoActivo())
        fprintf(stderr, "[vuelco] f=%u vc=%u escribe pow=%04X pow15=%u\n",
                sVuelcoFotograma, VCount, val & 0xFFFFu,
                (u32)((val >> 15) & 1u));
    // POWCNT1 effects:
    // * bit0: asplodes hardware??? not tested.
    // * bit1: disables engine A palette and OAM (zero-filled) (TODO: affects mem timings???)
    // * bit2: disables rendering engine, resets its internal state (polygons and registers)
    // * bit3: disables geometry engine
    // * bit9: disables engine B palette, OAM and rendering (screen turns white)
    // * bit15: screen swap

    if (!(val & (1<<0))) Log(LogLevel::Warn, "!!! CLEARING POWCNT BIT0. DANGER\n");

    GPU2D_A.SetEnabled(val & (1<<1));
    GPU2D_B.SetEnabled(val & (1<<9));
    GPU3D.SetEnabled(val & (1<<3), val & (1<<2));

    AssignFramebuffers();
}


void GPU::DisplayFIFO(u32 x) noexcept
{
    Sync2D();
    // sample the FIFO
    // as this starts 16 cycles (~3 pixels) before display start,
    // we aren't aligned to the 8-pixel grid
    if (x > 0)
    {
        if (x == 8)
            GPU2D_A.SampleFIFO(0, 5);
        else
            GPU2D_A.SampleFIFO(x-11, 8);
    }

    if (x < 256)
    {
        // transfer the next 8 pixels
        NDS.CheckDMAs(0, 0x04);
        NDS.ScheduleEvent(Event_DisplayFIFO, true, 6*8, 0, x+8);
    }
    else
        GPU2D_A.SampleFIFO(253, 3); // sample the remaining pixels
}

void GPU::StartFrame() noexcept
{
    // only run the display FIFO if needed:
    // * if it is used for display or capture
    // * if we have display FIFO DMA
    RunFIFO = GPU2D_A.UsesFIFO() || NDS.DMAsInMode(0, 0x04);

    TotalScanlines = 0;
    StartScanline(0);
}

void GPU::StartHBlank(u32 line) noexcept
{
    DispStat[0] |= (1<<1);
    DispStat[1] |= (1<<1);

    if (VCount < 192)
    {

        if (VCount == 96 && LogVuelcoActivo())
            fprintf(stderr, "[vuelco] f=%u linea96 asignado=%u front=%d\n",
                    sVuelcoFotograma, sVuelcoSwapAsignado, FrontBuffer);
        // draw
        // note: this should start 48 cycles after the scanline start
        if (Threaded2D && line < 192)
        {
            // drawing, the next line's sprites and the window update run on
            // the worker in this order; see DrawScanline2D
            PostRender2DLine(line, VCount);
        }
        else if (line < 192)
        {
            const bool measureStructured2D = ShouldMeasureStructured2D(*this);
            const u64 drawAStartNs = measureStructured2D ? PerfNowNs() : 0;
            GPU2D_Renderer->DrawScanline(line, &GPU2D_A);
            const u64 drawBStartNs = measureStructured2D ? PerfNowNs() : 0;
            GPU2D_Renderer->DrawScanline(line, &GPU2D_B);
            if (measureStructured2D)
            {
                const u64 drawEndNs = PerfNowNs();
                gStructured2DFrameDrawANs += drawBStartNs - drawAStartNs;
                gStructured2DFrameDrawBNs += drawEndNs - drawBStartNs;
                gStructured2DFrameDrawNs += drawEndNs - drawAStartNs;
            }
        }

        // sprites are pre-rendered one scanline in advance
        if (!Threaded2D && line < 191)
        {
            const bool measureStructured2D = ShouldMeasureStructured2D(*this);
            const u64 spriteAStartNs = measureStructured2D ? PerfNowNs() : 0;
            GPU2D_Renderer->DrawSprites(line+1, &GPU2D_A);
            const u64 spriteBStartNs = measureStructured2D ? PerfNowNs() : 0;
            GPU2D_Renderer->DrawSprites(line+1, &GPU2D_B);
            if (measureStructured2D)
            {
                const u64 spriteEndNs = PerfNowNs();
                gStructured2DFrameSpriteANs += spriteBStartNs - spriteAStartNs;
                gStructured2DFrameSpriteBNs += spriteEndNs - spriteBStartNs;
                gStructured2DFrameSpriteNs += spriteEndNs - spriteAStartNs;
            }
        }

        if (line == 191)
        {
            if (ShouldMeasureStructured2D(*this))
            {
                gStructured2DDrawWindow.Add(gStructured2DFrameDrawNs);
                gStructured2DSpriteWindow.Add(gStructured2DFrameSpriteNs);
                gStructured2DDrawAWindow.Add(gStructured2DFrameDrawANs);
                gStructured2DDrawBWindow.Add(gStructured2DFrameDrawBNs);
                gStructured2DSpriteAWindow.Add(gStructured2DFrameSpriteANs);
                gStructured2DSpriteBWindow.Add(gStructured2DFrameSpriteBNs);
                LogStructured2DPerfIfReady();
            }
            gStructured2DFrameDrawNs = 0;
            gStructured2DFrameSpriteNs = 0;
            gStructured2DFrameDrawANs = 0;
            gStructured2DFrameDrawBNs = 0;
            gStructured2DFrameSpriteANs = 0;
            gStructured2DFrameSpriteBNs = 0;
        }

        NDS.CheckDMAs(0, 0x02);
    }
    else if (VCount == 215)
    {
        GPU3D.VCount215(*this);
    }
    else if (VCount == 262)
    {
        Sync2D();
        GPU2D_Renderer->DrawSprites(0, &GPU2D_A);
        GPU2D_Renderer->DrawSprites(0, &GPU2D_B);
    }

    if (DispStat[0] & (1<<4)) NDS.SetIRQ(0, IRQ_HBlank);
    if (DispStat[1] & (1<<4)) NDS.SetIRQ(1, IRQ_HBlank);

    if (VCount < 262)
        NDS.ScheduleEvent(Event_LCD, true, (LINE_CYCLES - HBLANK_CYCLES), LCD_StartScanline, line+1);
    else
        NDS.ScheduleEvent(Event_LCD, true, (LINE_CYCLES - HBLANK_CYCLES), LCD_FinishFrame, line+1);
}

void GPU::FinishFrame(u32 lines) noexcept
{
    Sync2D();
    FrontBuffer = FrontBuffer ? 0 : 1;
    AssignFramebuffers();

    if (LogVuelcoActivo())
        fprintf(stderr,
                "[vuelco] f=%u FIN pow15=%u dispA=%08X dispB=%08X capA=%08X\n",
                sVuelcoFotograma, (u32)((NDS.PowerControl9 >> 15) & 1u),
                GPU2D_A.DispCnt, GPU2D_B.DispCnt, GPU2D_A.CaptureCnt);
    sVuelcoFotograma++;

    TotalScanlines = lines;

    if (GPU3D.AbortFrame)
    {
        GPU3D.RestartFrame(*this);
        GPU3D.AbortFrame = false;
    }
}

void GPU::BlankFrame() noexcept
{
    int backbuf = FrontBuffer ? 0 : 1;
    int fbsize;
    if (GPU3D.IsRendererAccelerated())
        fbsize = (256*3 + 1) * 192;
    else
        fbsize = 256 * 192;

    memset(Framebuffer[backbuf][0].get(), 0, fbsize*4);
    memset(Framebuffer[backbuf][1].get(), 0, fbsize*4);

    FrontBuffer = backbuf;
    AssignFramebuffers();

    TotalScanlines = 263;
}

void GPU::StartScanline(u32 line) noexcept
{
    // VBlank and the frame swap read everything the 2D worker writes
    if (line >= 192 || line == 0)
        Sync2D();

    if (line == 0)
        VCount = 0;
    else if (NextVCount != 0xFFFFFFFF)
        VCount = NextVCount;
    else
        VCount++;

    NextVCount = -1;

    DispStat[0] &= ~(1<<1);
    DispStat[1] &= ~(1<<1);

    if (VCount == VMatch[0])
    {
        DispStat[0] |= (1<<2);

        if (DispStat[0] & (1<<5)) NDS.SetIRQ(0, IRQ_VCount);
    }
    else
        DispStat[0] &= ~(1<<2);

    if (VCount == VMatch[1])
    {
        DispStat[1] |= (1<<2);

        if (DispStat[1] & (1<<5)) NDS.SetIRQ(1, IRQ_VCount);
    }
    else
        DispStat[1] &= ~(1<<2);

    if (!(Threaded2D && VCount < 192 && line < 192))
    {
        Sync2D();
        GPU2D_A.CheckWindows(VCount);
        GPU2D_B.CheckWindows(VCount);
    }

    if (VCount >= 2 && VCount < 194)
        NDS.CheckDMAs(0, 0x03);
    else if (VCount == 194)
        NDS.StopDMAs(0, 0x03);

    if (line < 192)
    {
        if (line == 0)
        {
            Sync2D();
            GPU2D_Renderer->VBlankEnd(&GPU2D_A, &GPU2D_B);
            GPU2D_A.VBlankEnd();
            GPU2D_B.VBlankEnd();
        }

        if (RunFIFO)
            NDS.ScheduleEvent(Event_DisplayFIFO, false, 32, 0, 0);
    }

    if (VCount == 262)
    {
        // frame end

        DispStat[0] &= ~(1<<0);
        DispStat[1] &= ~(1<<0);
    }
    else
    {
        if (VCount == 192)
        {
            // in reality rendering already finishes at line 144
            // and games might already start to modify texture memory.
            // That doesn't matter for us because we cache the entire
            // texture memory anyway and only update it before the start
            //of the next frame.
            // So we can give the rasteriser a bit more headroom
            GPU3D.VCount144(*this);

            // VBlank
            DispStat[0] |= (1<<0);
            DispStat[1] |= (1<<0);

            NDS.StopDMAs(0, 0x04);

            NDS.CheckDMAs(0, 0x01);
            NDS.CheckDMAs(1, 0x11);

            if (DispStat[0] & (1<<3)) NDS.SetIRQ(0, IRQ_VBlank);
            if (DispStat[1] & (1<<3)) NDS.SetIRQ(1, IRQ_VBlank);

            GPU2D_A.VBlank();
            GPU2D_B.VBlank();
            GPU3D.VBlank();

            // Need a better way to identify the openGL renderer in particular
            if (GPU3D.IsRendererAccelerated())
                GPU3D.Blit(*this);
        }
    }

    NDS.ScheduleEvent(Event_LCD, true, HBLANK_CYCLES, LCD_StartHBlank, line);
}


void GPU::SetDispStat(u32 cpu, u16 val) noexcept
{
    val &= 0xFFB8;
    DispStat[cpu] &= 0x0047;
    DispStat[cpu] |= val;

    VMatch[cpu] = (val >> 8) | ((val & 0x80) << 1);
}

void GPU::SetVCount(u16 val) noexcept
{
    // VCount write is delayed until the next scanline

    // TODO: how does the 3D engine react to VCount writes while it's rendering?
    // 3D engine seems to give up on the current frame in that situation, repeating the last two scanlines
    // TODO: also check the various DMA types that can be involved

    GPU3D.AbortFrame |= NextVCount != val;
    NextVCount = val;
}

template <u32 Size, u32 MappingGranularity>
NonStupidBitField<Size/VRAMDirtyGranularity> VRAMTrackingSet<Size, MappingGranularity>::DeriveState(const u32* currentMappings, GPU& gpu)
{
    NonStupidBitField<Size/VRAMDirtyGranularity> result;
    u16 banksToBeZeroed = 0;
    for (u32 i = 0; i < Size / MappingGranularity; i++)
    {
        if (currentMappings[i] != Mapping[i])
        {
            result.SetRange(i*VRAMBitsPerMapping, VRAMBitsPerMapping);
            banksToBeZeroed |= currentMappings[i];
            Mapping[i] = currentMappings[i];
        }
        else
        {
            u32 mapping = Mapping[i];

            banksToBeZeroed |= mapping;

            while (mapping != 0)
            {
                u32 num = __builtin_ctz(mapping);
                mapping &= ~(1 << num);

                // hack for **speed**
                // this could probably be done less ugly but then we would rely
                // on the compiler for vectorisation
                static_assert(VRAMDirtyGranularity == 512, "");
                if (MappingGranularity == 16*1024)
                {
                    u32 dirty = ((u32*)gpu.VRAMDirty[num].Data)[i & (gpu.VRAMMask[num] >> 14)];
                    result.Data[i / 2] |= (u64)dirty << ((i&1)*32);
                }
                else if (MappingGranularity == 8*1024)
                {
                    u16 dirty = ((u16*)gpu.VRAMDirty[num].Data)[i & (gpu.VRAMMask[num] >> 13)];
                    result.Data[i / 4] |= (u64)dirty << ((i&3)*16);
                }
                else if (MappingGranularity == 128*1024)
                {
                    result.Data[i * 4 + 0] |= gpu.VRAMDirty[num].Data[0];
                    result.Data[i * 4 + 1] |= gpu.VRAMDirty[num].Data[1];
                    result.Data[i * 4 + 2] |= gpu.VRAMDirty[num].Data[2];
                    result.Data[i * 4 + 3] |= gpu.VRAMDirty[num].Data[3];
                }
                else
                {
                    // welp
                    abort();
                }
            }
        }
    }

    while (banksToBeZeroed != 0)
    {
        u32 num = __builtin_ctz(banksToBeZeroed);
        banksToBeZeroed &= ~(1 << num);
        gpu.VRAMDirty[num].Clear();
    }

    return result;
}

template NonStupidBitField<32*1024/VRAMDirtyGranularity> VRAMTrackingSet<32*1024, 8*1024>::DeriveState(const u32*, GPU& gpu);
template NonStupidBitField<8*1024/VRAMDirtyGranularity> VRAMTrackingSet<8*1024, 8*1024>::DeriveState(const u32*, GPU& gpu);
template NonStupidBitField<512*1024/VRAMDirtyGranularity> VRAMTrackingSet<512*1024, 128*1024>::DeriveState(const u32*, GPU& gpu);
template NonStupidBitField<128*1024/VRAMDirtyGranularity> VRAMTrackingSet<128*1024, 16*1024>::DeriveState(const u32*, GPU& gpu);
template NonStupidBitField<256*1024/VRAMDirtyGranularity> VRAMTrackingSet<256*1024, 16*1024>::DeriveState(const u32*, GPU& gpu);
template NonStupidBitField<512*1024/VRAMDirtyGranularity> VRAMTrackingSet<512*1024, 16*1024>::DeriveState(const u32*, GPU& gpu);



bool GPU::MakeVRAMFlat_TextureCoherent(NonStupidBitField<512*1024/VRAMDirtyGranularity>& dirty) noexcept
{
    return CopyLinearVRAM<128*1024>(VRAMFlat_Texture, VRAMMap_Texture, dirty, &GPU::ReadVRAM_Texture<u64>);
}
bool GPU::MakeVRAMFlat_TexPalCoherent(NonStupidBitField<128*1024/VRAMDirtyGranularity>& dirty) noexcept
{
    return CopyLinearVRAM<16*1024>(VRAMFlat_TexPal, VRAMMap_TexPal, dirty, &GPU::ReadVRAM_TexPal<u64>);
}

bool GPU::MakeVRAMFlat_ABGCoherent(NonStupidBitField<512*1024/VRAMDirtyGranularity>& dirty) noexcept
{
    return CopyLinearVRAM<16*1024>(VRAMFlat_ABG, VRAMMap_ABG, dirty, &GPU::ReadVRAM_ABG<u64>);
}
bool GPU::MakeVRAMFlat_BBGCoherent(NonStupidBitField<128*1024/VRAMDirtyGranularity>& dirty) noexcept
{
    return CopyLinearVRAM<16*1024>(VRAMFlat_BBG, VRAMMap_BBG, dirty, &GPU::ReadVRAM_BBG<u64>);
}

bool GPU::MakeVRAMFlat_AOBJCoherent(NonStupidBitField<256*1024/VRAMDirtyGranularity>& dirty) noexcept
{
    return CopyLinearVRAM<16*1024>(VRAMFlat_AOBJ, VRAMMap_AOBJ, dirty, &GPU::ReadVRAM_AOBJ<u64>);
}
bool GPU::MakeVRAMFlat_BOBJCoherent(NonStupidBitField<128*1024/VRAMDirtyGranularity>& dirty) noexcept
{
    return CopyLinearVRAM<16*1024>(VRAMFlat_BOBJ, VRAMMap_BOBJ, dirty, &GPU::ReadVRAM_BOBJ<u64>);
}

bool GPU::MakeVRAMFlat_ABGExtPalCoherent(NonStupidBitField<32*1024/VRAMDirtyGranularity>& dirty) noexcept
{
    return CopyLinearVRAM<8*1024>(VRAMFlat_ABGExtPal, VRAMMap_ABGExtPal, dirty, &GPU::ReadVRAM_ABGExtPal<u64>);
}
bool GPU::MakeVRAMFlat_BBGExtPalCoherent(NonStupidBitField<32*1024/VRAMDirtyGranularity>& dirty) noexcept
{
    return CopyLinearVRAM<8*1024>(VRAMFlat_BBGExtPal, VRAMMap_BBGExtPal, dirty, &GPU::ReadVRAM_BBGExtPal<u64>);
}

bool GPU::MakeVRAMFlat_AOBJExtPalCoherent(NonStupidBitField<8*1024/VRAMDirtyGranularity>& dirty) noexcept
{
    return CopyLinearVRAM<8*1024>(VRAMFlat_AOBJExtPal, &VRAMMap_AOBJExtPal, dirty, &GPU::ReadVRAM_AOBJExtPal<u64>);
}
bool GPU::MakeVRAMFlat_BOBJExtPalCoherent(NonStupidBitField<8*1024/VRAMDirtyGranularity>& dirty) noexcept
{
    return CopyLinearVRAM<8*1024>(VRAMFlat_BOBJExtPal, &VRAMMap_BOBJExtPal, dirty, &GPU::ReadVRAM_BOBJExtPal<u64>);
}
}
