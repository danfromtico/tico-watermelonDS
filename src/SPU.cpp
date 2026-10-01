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
#include <string.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <new>
#include <numeric>
#include "Platform.h"
#include "NDS.h"
#include "Mic.h"
#include "DSi.h"
#include "DSi_I2S.h"
#include "SPU.h"
#include "AudioOutputExactMath.h"

#include "blip-buf/blip_buf.h"

#define INTERNAL_SAMPLE_RATE 16756991.f

namespace melonDS
{
using Platform::Log;
using Platform::LogLevel;

struct OutputAdaptiveGeometry
{
    u32 controlCapacity;
    u32 block;
    u32 rateWindow;
    u32 physicalReserve;
    u32 controlTarget;
    u32 target;
    u32 low;
    u32 high;
    u32 safe;
};

static OutputAdaptiveGeometry GetOutputAdaptiveGeometry(u32 physicalCapacity)
{

    const u32 controlCapacity = physicalCapacity / 2;
    const u32 block = controlCapacity / 8;
    const u32 rateWindow = 2 * block;
    const u32 physicalReserve = 4 * block;
    const u32 controlTarget = controlCapacity / 4;
    const u32 target = controlTarget + physicalReserve;
    return {
        controlCapacity,
        block,
        rateWindow,
        physicalReserve,
        controlTarget,
        target,
        target - physicalReserve,
        target + (2 * block),
        controlCapacity - block + physicalReserve,
    };
}

struct SPU::OutputSpillNode
{
    std::unique_ptr<OutputSpillNode> next;
    u32 readPos = 0;
    u32 writePos = 0;
    s16 samples[blip_max_frame * 2] {};
};

u32 SPU::OutputRingLevelLocked() const
{
    const u32 sampleLevel = OutputBufferWritePos >= OutputBufferReadPos
        ? OutputBufferWritePos - OutputBufferReadPos
        : (OutputBufferSize * 2) - OutputBufferReadPos
            + OutputBufferWritePos;
    return sampleLevel >> 1;
}

u64 SPU::OutputLogicalLevelLocked() const
{
    return static_cast<u64>(OutputRingLevelLocked()) + OutputSpillFrames;
}

std::unique_ptr<SPU::OutputSpillNode> SPU::TakeOutputSpillFreeLocked()
{
    if (!OutputSpillFree)
        return nullptr;

    auto node = std::move(OutputSpillFree);
    OutputSpillFree = std::move(node->next);
    node->next.reset();
    node->readPos = 0;
    node->writePos = 0;
    --OutputSpillFreeNodes;
    return node;
}

void SPU::ReturnOutputSpillFreeLocked(
    std::unique_ptr<OutputSpillNode> node)
{
    if (!node)
        return;

    node->readPos = 0;
    node->writePos = 0;
    node->next = std::move(OutputSpillFree);
    OutputSpillFree = std::move(node);
    ++OutputSpillFreeNodes;
}

void SPU::PromoteOutputSpillLocked(u32 maxFrames)
{
    while (maxFrames > 0 && OutputSpillActive)
    {
        const u32 ringFree = OutputBufferSize - 1
            - OutputRingLevelLocked();
        if (ringFree == 0)
            break;

        OutputSpillNode* head = OutputSpillActive.get();
        const u32 available = head->writePos - head->readPos;
        const u32 count = std::min({maxFrames, ringFree, available});
        for (u32 i = 0; i < count; ++i)
        {
            const u32 source = (head->readPos + i) * 2;
            OutputBuffer[OutputBufferWritePos++] = head->samples[source];
            OutputBuffer[OutputBufferWritePos++] = head->samples[source + 1];
            OutputBufferWritePos &= ((2 * OutputBufferSize) - 1);
        }
        head->readPos += count;
        OutputSpillFrames -= count;
        maxFrames -= count;

        if (head->readPos == head->writePos)
        {
            auto exhausted = std::move(OutputSpillActive);
            OutputSpillActive = std::move(exhausted->next);
            --OutputSpillActiveNodes;
            if (!OutputSpillActive)
                OutputSpillTail = nullptr;
            ReturnOutputSpillFreeLocked(std::move(exhausted));
        }
    }
}

int SPU::ReadOutputQueueLocked(s16* data, int samples)
{
    int read = 0;
    while (read < samples && OutputBufferReadPos != OutputBufferWritePos)
    {
        data[read * 2] = OutputBuffer[OutputBufferReadPos++];
        data[read * 2 + 1] = OutputBuffer[OutputBufferReadPos++];
        OutputBufferReadPos &= ((2 * OutputBufferSize) - 1);
        ++read;
    }

    while (read < samples && OutputSpillActive)
    {
        OutputSpillNode* head = OutputSpillActive.get();
        while (read < samples && head->readPos < head->writePos)
        {
            const u32 source = head->readPos++ * 2;
            data[read * 2] = head->samples[source];
            data[read * 2 + 1] = head->samples[source + 1];
            ++read;
            --OutputSpillFrames;
        }
        if (head->readPos == head->writePos)
        {
            auto exhausted = std::move(OutputSpillActive);
            OutputSpillActive = std::move(exhausted->next);
            --OutputSpillActiveNodes;
            if (!OutputSpillActive)
                OutputSpillTail = nullptr;
            ReturnOutputSpillFreeLocked(std::move(exhausted));
        }
    }

    PromoteOutputSpillLocked(static_cast<u32>(read));
    OutputProvenanceRemovedFrames += static_cast<u64>(read);
    OutputProvenanceRealDrainedFrames += static_cast<u64>(read);
    return read;
}

void SPU::DiscardOutputQueueLocked(u64 frames)
{
    const u64 availableBefore = OutputLogicalLevelLocked();
    const u64 discardRequested = std::min(frames, availableBefore);
    frames = discardRequested;
    const u32 ringDiscard = static_cast<u32>(std::min<u64>(
        frames, OutputRingLevelLocked()));
    OutputBufferReadPos += ringDiscard * 2;
    OutputBufferReadPos &= ((2 * OutputBufferSize) - 1);
    frames -= ringDiscard;

    while (frames > 0 && OutputSpillActive)
    {
        OutputSpillNode* head = OutputSpillActive.get();
        const u32 available = head->writePos - head->readPos;
        const u32 count = static_cast<u32>(std::min<u64>(frames, available));
        head->readPos += count;
        OutputSpillFrames -= count;
        frames -= count;
        if (head->readPos == head->writePos)
        {
            auto exhausted = std::move(OutputSpillActive);
            OutputSpillActive = std::move(exhausted->next);
            --OutputSpillActiveNodes;
            if (!OutputSpillActive)
                OutputSpillTail = nullptr;
            ReturnOutputSpillFreeLocked(std::move(exhausted));
        }
    }
    PromoteOutputSpillLocked(std::numeric_limits<u32>::max());
    const u64 discarded = discardRequested - frames;
    OutputProvenanceRemovedFrames += discarded;
    OutputProvenanceDiscardedFrames += discarded;
    if (discarded > 0)
        ++OutputProvenanceLineageEpoch;
}

void SPU::ClearOutputQueueLocked()
{
    const u64 discarded = OutputLogicalLevelLocked();
    OutputProvenanceRemovedFrames += discarded;
    OutputProvenanceDiscardedFrames += discarded;
    ++OutputProvenanceLineageEpoch;
    OutputBufferReadPos = 0;
    OutputBufferWritePos = 0;
    if (OutputSpillActive)
    {
        OutputSpillTail->next = std::move(OutputSpillFree);
        OutputSpillFree = std::move(OutputSpillActive);
        OutputSpillFreeNodes += OutputSpillActiveNodes;
    }
    OutputSpillTail = nullptr;
    OutputSpillFrames = 0;
    OutputSpillActiveNodes = 0;
}


// SPU TODO
// * channel hold

static s32 CaptureTruncateTowardZero(s64 value, u32 fractionalBits)
{
    const s64 magnitude = value < 0 ? -value : value;
    const s32 truncated = static_cast<s32>(magnitude >> fractionalBits);
    return value < 0 ? -truncated : truncated;
}

static s32 CaptureMixerToPCM16(s32 mixer)
{

    const s32 clipped = std::clamp(mixer, -0x800000, 0x7FFFFF);
    return CaptureTruncateTowardZero(clipped, 8);
}

static s32 CaptureDirectToPCM16(s32 channelA, s32 channelB, bool add)
{

    if (!add)
    {
        if (channelA < 0 && channelB < 0)
            return -0x8000;
        return CaptureTruncateTowardZero(channelA, 11);
    }

    const s32 sum = CaptureTruncateTowardZero(
        static_cast<s64>(channelA) + channelB, 11);
    const u32 wrapped = static_cast<u32>(sum) & 0xFFFF;
    return wrapped >= 0x8000
        ? static_cast<s32>(wrapped) - 0x10000
        : static_cast<s32>(wrapped);
}


const s8 SPUChannel::ADPCMIndexTable[8] = {-1, -1, -1, -1, 2, 4, 6, 8};

const u16 SPUChannel::ADPCMTable[89] =
{
    0x0007, 0x0008, 0x0009, 0x000A, 0x000B, 0x000C, 0x000D, 0x000E,
    0x0010, 0x0011, 0x0013, 0x0015, 0x0017, 0x0019, 0x001C, 0x001F,
    0x0022, 0x0025, 0x0029, 0x002D, 0x0032, 0x0037, 0x003C, 0x0042,
    0x0049, 0x0050, 0x0058, 0x0061, 0x006B, 0x0076, 0x0082, 0x008F,
    0x009D, 0x00AD, 0x00BE, 0x00D1, 0x00E6, 0x00FD, 0x0117, 0x0133,
    0x0151, 0x0173, 0x0198, 0x01C1, 0x01EE, 0x0220, 0x0256, 0x0292,
    0x02D4, 0x031C, 0x036C, 0x03C3, 0x0424, 0x048E, 0x0502, 0x0583,
    0x0610, 0x06AB, 0x0756, 0x0812, 0x08E0, 0x09C3, 0x0ABD, 0x0BD0,
    0x0CFF, 0x0E4C, 0x0FBA, 0x114C, 0x1307, 0x14EE, 0x1706, 0x1954,
    0x1BDC, 0x1EA5, 0x21B6, 0x2515, 0x28CA, 0x2CDF, 0x315B, 0x364B,
    0x3BB9, 0x41B2, 0x4844, 0x4F7E, 0x5771, 0x602F, 0x69CE, 0x7462,
    0x7FFF
};

const s16 SPUChannel::PSGTable[8][8] =
{
    {-0x7FFF, -0x7FFF, -0x7FFF, -0x7FFF, -0x7FFF, -0x7FFF, -0x7FFF,  0x7FFF},
    {-0x7FFF, -0x7FFF, -0x7FFF, -0x7FFF, -0x7FFF, -0x7FFF,  0x7FFF,  0x7FFF},
    {-0x7FFF, -0x7FFF, -0x7FFF, -0x7FFF, -0x7FFF,  0x7FFF,  0x7FFF,  0x7FFF},
    {-0x7FFF, -0x7FFF, -0x7FFF, -0x7FFF,  0x7FFF,  0x7FFF,  0x7FFF,  0x7FFF},
    {-0x7FFF, -0x7FFF, -0x7FFF,  0x7FFF,  0x7FFF,  0x7FFF,  0x7FFF,  0x7FFF},
    {-0x7FFF, -0x7FFF,  0x7FFF,  0x7FFF,  0x7FFF,  0x7FFF,  0x7FFF,  0x7FFF},
    {-0x7FFF,  0x7FFF,  0x7FFF,  0x7FFF,  0x7FFF,  0x7FFF,  0x7FFF,  0x7FFF},
    {-0x7FFF, -0x7FFF, -0x7FFF, -0x7FFF, -0x7FFF, -0x7FFF, -0x7FFF, -0x7FFF}
};

template <typename T>
constexpr T ipow(T num, unsigned int pow)
{
    T product = 1;
    for (int i = 0; i < pow; ++i)
    {
        product *= num;
    }

    return product;
}

template <typename T>
constexpr T factorial(T num)
{
    T product = 1;
    for (T i = 1; i <= num; ++i)
    {
        product *= i;
    }

    return product;
}

// We can't use std::cos in constexpr functions until C++26,
// so we need to compute the cosine ourselves with the Taylor series.
// Code adapted from https://prosepoetrycode.potterpcs.net/2015/07/a-simple-constexpr-power-function-c/
template <int Iterations = 10>
constexpr double cosine (double theta)
{
    return (ipow(-1, Iterations) * ipow(theta, 2 * Iterations)) /
            static_cast<double>(factorial(2ull * Iterations))
        + cosine<Iterations-1>(theta);
}

template <>
constexpr double cosine<0> (double theta)
{
    return 1.0;
}

// generate interpolation tables
// values are 1:1:14 fixed-point
constexpr std::array<s16, 0x100> InterpCos = []() constexpr {
    std::array<s16, 0x100> interp {};

    for (int i = 0; i < 0x100; i++)
    {
        float ratio = (i * M_PI) / 255.0f;
        ratio = 1.0f - cosine(ratio);

        interp[i] = (s16)(ratio * 0x2000);
    }

    return interp;
}();

constexpr array2d<s16, 0x100, 4> InterpCubic = []() constexpr {
    array2d<s16, 0x100, 4> interp {};

    for (int i = 0; i < 0x100; i++)
    {
        s32 i1 = i << 6;
        s32 i2 = (i * i) >> 2;
        s32 i3 = (i * i * i) >> 10;

        interp[i][0] = -i3 + 2*i2 - i1;
        interp[i][1] = i3 - 2*i2 + 0x4000;
        interp[i][2] = -i3 + i2 + i1;
        interp[i][3] = i3 - i2;
    }

    return interp;
}();

const std::array<s16, 0x200> InterpSNESGauss = {
    0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000,
    0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x002, 0x002, 0x002, 0x002, 0x002,
    0x002, 0x002, 0x003, 0x003, 0x003, 0x003, 0x003, 0x004, 0x004, 0x004, 0x004, 0x004, 0x005, 0x005, 0x005, 0x005,
    0x006, 0x006, 0x006, 0x006, 0x007, 0x007, 0x007, 0x008, 0x008, 0x008, 0x009, 0x009, 0x009, 0x00A, 0x00A, 0x00A,
    0x00B, 0x00B, 0x00B, 0x00C, 0x00C, 0x00D, 0x00D, 0x00E, 0x00E, 0x00F, 0x00F, 0x00F, 0x010, 0x010, 0x011, 0x011,
    0x012, 0x013, 0x013, 0x014, 0x014, 0x015, 0x015, 0x016, 0x017, 0x017, 0x018, 0x018, 0x019, 0x01A, 0x01B, 0x01B,
    0x01C, 0x01D, 0x01D, 0x01E, 0x01F, 0x020, 0x020, 0x021, 0x022, 0x023, 0x024, 0x024, 0x025, 0x026, 0x027, 0x028,
    0x029, 0x02A, 0x02B, 0x02C, 0x02D, 0x02E, 0x02F, 0x030, 0x031, 0x032, 0x033, 0x034, 0x035, 0x036, 0x037, 0x038,
    0x03A, 0x03B, 0x03C, 0x03D, 0x03E, 0x040, 0x041, 0x042, 0x043, 0x045, 0x046, 0x047, 0x049, 0x04A, 0x04C, 0x04D,
    0x04E, 0x050, 0x051, 0x053, 0x054, 0x056, 0x057, 0x059, 0x05A, 0x05C, 0x05E, 0x05F, 0x061, 0x063, 0x064, 0x066,
    0x068, 0x06A, 0x06B, 0x06D, 0x06F, 0x071, 0x073, 0x075, 0x076, 0x078, 0x07A, 0x07C, 0x07E, 0x080, 0x082, 0x084,
    0x086, 0x089, 0x08B, 0x08D, 0x08F, 0x091, 0x093, 0x096, 0x098, 0x09A, 0x09C, 0x09F, 0x0A1, 0x0A3, 0x0A6, 0x0A8,
    0x0AB, 0x0AD, 0x0AF, 0x0B2, 0x0B4, 0x0B7, 0x0BA, 0x0BC, 0x0BF, 0x0C1, 0x0C4, 0x0C7, 0x0C9, 0x0CC, 0x0CF, 0x0D2,
    0x0D4, 0x0D7, 0x0DA, 0x0DD, 0x0E0, 0x0E3, 0x0E6, 0x0E9, 0x0EC, 0x0EF, 0x0F2, 0x0F5, 0x0F8, 0x0FB, 0x0FE, 0x101,
    0x104, 0x107, 0x10B, 0x10E, 0x111, 0x114, 0x118, 0x11B, 0x11E, 0x122, 0x125, 0x129, 0x12C, 0x130, 0x133, 0x137,
    0x13A, 0x13E, 0x141, 0x145, 0x148, 0x14C, 0x150, 0x153, 0x157, 0x15B, 0x15F, 0x162, 0x166, 0x16A, 0x16E, 0x172,
    0x176, 0x17A, 0x17D, 0x181, 0x185, 0x189, 0x18D, 0x191, 0x195, 0x19A, 0x19E, 0x1A2, 0x1A6, 0x1AA, 0x1AE, 0x1B2,
    0x1B7, 0x1BB, 0x1BF, 0x1C3, 0x1C8, 0x1CC, 0x1D0, 0x1D5, 0x1D9, 0x1DD, 0x1E2, 0x1E6, 0x1EB, 0x1EF, 0x1F3, 0x1F8,
    0x1FC, 0x201, 0x205, 0x20A, 0x20F, 0x213, 0x218, 0x21C, 0x221, 0x226, 0x22A, 0x22F, 0x233, 0x238, 0x23D, 0x241,
    0x246, 0x24B, 0x250, 0x254, 0x259, 0x25E, 0x263, 0x267, 0x26C, 0x271, 0x276, 0x27B, 0x280, 0x284, 0x289, 0x28E,
    0x293, 0x298, 0x29D, 0x2A2, 0x2A6, 0x2AB, 0x2B0, 0x2B5, 0x2BA, 0x2BF, 0x2C4, 0x2C9, 0x2CE, 0x2D3, 0x2D8, 0x2DC,
    0x2E1, 0x2E6, 0x2EB, 0x2F0, 0x2F5, 0x2FA, 0x2FF, 0x304, 0x309, 0x30E, 0x313, 0x318, 0x31D, 0x322, 0x326, 0x32B,
    0x330, 0x335, 0x33A, 0x33F, 0x344, 0x349, 0x34E, 0x353, 0x357, 0x35C, 0x361, 0x366, 0x36B, 0x370, 0x374, 0x379,
    0x37E, 0x383, 0x388, 0x38C, 0x391, 0x396, 0x39B, 0x39F, 0x3A4, 0x3A9, 0x3AD, 0x3B2, 0x3B7, 0x3BB, 0x3C0, 0x3C5,
    0x3C9, 0x3CE, 0x3D2, 0x3D7, 0x3DC, 0x3E0, 0x3E5, 0x3E9, 0x3ED, 0x3F2, 0x3F6, 0x3FB, 0x3FF, 0x403, 0x408, 0x40C,
    0x410, 0x415, 0x419, 0x41D, 0x421, 0x425, 0x42A, 0x42E, 0x432, 0x436, 0x43A, 0x43E, 0x442, 0x446, 0x44A, 0x44E,
    0x452, 0x455, 0x459, 0x45D, 0x461, 0x465, 0x468, 0x46C, 0x470, 0x473, 0x477, 0x47A, 0x47E, 0x481, 0x485, 0x488,
    0x48C, 0x48F, 0x492, 0x496, 0x499, 0x49C, 0x49F, 0x4A2, 0x4A6, 0x4A9, 0x4AC, 0x4AF, 0x4B2, 0x4B5, 0x4B7, 0x4BA,
    0x4BD, 0x4C0, 0x4C3, 0x4C5, 0x4C8, 0x4CB, 0x4CD, 0x4D0, 0x4D2, 0x4D5, 0x4D7, 0x4D9, 0x4DC, 0x4DE, 0x4E0, 0x4E3,
    0x4E5, 0x4E7, 0x4E9, 0x4EB, 0x4ED, 0x4EF, 0x4F1, 0x4F3, 0x4F5, 0x4F6, 0x4F8, 0x4FA, 0x4FB, 0x4FD, 0x4FF, 0x500,
    0x502, 0x503, 0x504, 0x506, 0x507, 0x508, 0x50A, 0x50B, 0x50C, 0x50D, 0x50E, 0x50F, 0x510, 0x511, 0x511, 0x512,
    0x513, 0x514, 0x514, 0x515, 0x516, 0x516, 0x517, 0x517, 0x517, 0x518, 0x518, 0x518, 0x518, 0x518, 0x519, 0x519
};

SPU::SPU(melonDS::NDS& nds, AudioBitDepth bitdepth, AudioInterpolation interpolation, double outputSampleRate) :
    NDS(nds),
    Channels {
        SPUChannel(0, nds, interpolation),
        SPUChannel(1, nds, interpolation),
        SPUChannel(2, nds, interpolation),
        SPUChannel(3, nds, interpolation),
        SPUChannel(4, nds, interpolation),
        SPUChannel(5, nds, interpolation),
        SPUChannel(6, nds, interpolation),
        SPUChannel(7, nds, interpolation),
        SPUChannel(8, nds, interpolation),
        SPUChannel(9, nds, interpolation),
        SPUChannel(10, nds, interpolation),
        SPUChannel(11, nds, interpolation),
        SPUChannel(12, nds, interpolation),
        SPUChannel(13, nds, interpolation),
        SPUChannel(14, nds, interpolation),
        SPUChannel(15, nds, interpolation),
    },
    Capture {
        SPUCaptureUnit(0, nds),
        SPUCaptureUnit(1, nds),
    },
    AudioLock(Platform::Mutex_Create()),
    Degrade10Bit(bitdepth == AudioBitDepth::_10Bit),
    OutputSampleRate(outputSampleRate),
    OutputBuffer(nullptr)
{
    NDS.RegisterEventFuncs(Event_SPU, this, {MakeEventThunk(SPU, Mix)});

    ApplyBias = true;

    BlipLeft = blip_new(blip_max_frame);
    BlipRight = blip_new(blip_max_frame);

    OutputBufferReadPos = 0;
    OutputBufferWritePos = 0;

    SetSampleRate(AudioSampleRate::_32KHz);
}

SPU::~SPU()
{
    auto destroyNodes = [](std::unique_ptr<OutputSpillNode> nodes) {
        while (nodes)
        {
            auto next = std::move(nodes->next);
            nodes.reset();
            nodes = std::move(next);
        }
    };
    destroyNodes(std::move(OutputSpillActive));
    destroyNodes(std::move(OutputSpillFree));
    Platform::Mutex_Free(AudioLock);
    AudioLock = nullptr;
    blip_delete(BlipLeft);
    blip_delete(BlipRight);

    NDS.UnregisterEventFuncs(Event_SPU);
}

void SPU::Reset()
{
    InitOutput();
    ResetOutputAdaptivo();

    Cnt = 0;
    MasterVolume = 0;
    Bias = 0;
    Mute = false;

    for (int i = 0; i < 16; i++)
        Channels[i].Reset();

    Capture[0].Reset();
    Capture[1].Reset();

    NDS.ScheduleEvent(Event_SPU, false, 1024, 0, 0);
}

void SPU::Stop()
{
    Platform::Mutex_Lock(AudioLock);
    memset(OutputBuffer, 0, 2*OutputBufferSize*2);

    blip_clear(BlipLeft);
    blip_clear(BlipRight);
    BlipTimer = 0;

    ClearOutputQueueLocked();
    ResetOutputAdaptivo();
    Platform::Mutex_Unlock(AudioLock);
}

void SPU::DoSavestate(Savestate* file)
{
    file->Section("SPU.");

    file->Var16(&Cnt);
    file->Var8(&MasterVolume);
    file->Var16(&Bias);

    file->VarArray(OutputLastSamples, sizeof(OutputLastSamples));

    file->Var32(&MixInterval);

    file->Bool32(&Mute);

    for (SPUChannel& channel : Channels)
        channel.DoSavestate(file);

    for (SPUCaptureUnit& capture : Capture)
        capture.DoSavestate(file);
}


void SPU::SetPowerCnt(u32 val)
{
    Mute = !(val & (1<<0));
}


void SPU::SetSampleRate(AudioSampleRate rate)
{
    if (rate == AudioSampleRate::_47KHz)
    {
        MixInterval = 704;
    }
    else
    {
        MixInterval = 1024;
    }

    memset(OutputLastSamples, 0, sizeof(OutputLastSamples));
}


void SPU::SetInterpolation(AudioInterpolation type)
{
    for (SPUChannel& channel : Channels)
        channel.InterpType = type;
}

void SPU::SetBias(u16 bias)
{
    Bias = bias;
}

void SPU::SetApplyBias(bool enable)
{
    ApplyBias = enable;
}

void SPU::SetDegrade10Bit(bool enable)
{
    Degrade10Bit = enable;
}

void SPU::SetDegrade10Bit(AudioBitDepth depth)
{
    switch (depth)
    {
    case AudioBitDepth::Auto:

        Degrade10Bit = false;
        break;
    case AudioBitDepth::_10Bit:
        Degrade10Bit = true;
        break;
    case AudioBitDepth::_16Bit:
        Degrade10Bit = false;
        break;
    }
}

SPUChannel::SPUChannel(u32 num, melonDS::NDS& nds, AudioInterpolation interpolation) :
    NDS(nds),
    Num(num),
    InterpType(interpolation)
{
}

void SPUChannel::Reset()
{
    KeyOn = false;
    HoldRestartPending = false;

    SetCnt(0);
    SrcAddr = 0;
    TimerReload = 0;
    LoopPos = 0;
    Length = 0;

    Timer = 0;

    Pos = 0;
    FIFOReadPos = 0;
    FIFOWritePos = 0;
    FIFOReadOffset = 0;
    FIFOLevel = 0;
}

void SPUChannel::DoSavestate(Savestate* file)
{
    file->Var32(&Cnt);
    file->Var32(&SrcAddr);
    file->Var16(&TimerReload);
    file->Var32(&LoopPos);
    file->Var32(&Length);

    file->Var8(&Volume);
    file->Var8(&VolumeShift);
    file->Var8(&Pan);

    file->Var8((u8*)&KeyOn);
    if (file->IsAtLeastVersion(13, 1))
        file->Var8((u8*)&HoldRestartPending);
    else
        HoldRestartPending = false;
    file->Var32(&Timer);
    file->Var32((u32*)&Pos);
    file->VarArray(PrevSample, sizeof(PrevSample));
    file->Var16((u16*)&CurSample);
    file->Var16(&NoiseVal);

    file->Var32((u32*)&ADPCMVal);
    file->Var32((u32*)&ADPCMIndex);
    file->Var32((u32*)&ADPCMValLoop);
    file->Var32((u32*)&ADPCMIndexLoop);
    file->Var8(&ADPCMCurByte);

    file->Var32(&FIFOReadPos);
    file->Var32(&FIFOWritePos);
    file->Var32(&FIFOReadOffset);
    file->Var32(&FIFOLevel);
    file->VarArray(FIFO, sizeof(FIFO));
}

void SPUChannel::FIFO_BufferData()
{
    u32 totallen = LoopPos + Length;

    if (FIFOReadOffset >= totallen)
    {
        u32 repeatmode = (Cnt >> 27) & 0x3;
        if      (repeatmode & 1) FIFOReadOffset = LoopPos;
        else if (repeatmode & 2) return; // one-shot sound, we're done
    }

    u32 burstlen = 16;
    if ((FIFOReadOffset + 16) > totallen)
        burstlen = totallen - FIFOReadOffset;

    // sound DMA can't read from the ARM7 BIOS
    if ((SrcAddr + FIFOReadOffset) >= 0x00004000)
    {
        for (u32 i = 0; i < burstlen; i += 4)
        {
            FIFO[FIFOWritePos] = NDS.ARM7Read32(SrcAddr + FIFOReadOffset);
            FIFOReadOffset += 4;
            FIFOWritePos++;
            FIFOWritePos &= 0x7;
        }
    }
    else
    {
        for (u32 i = 0; i < burstlen; i += 4)
        {
            FIFO[FIFOWritePos] = 0;
            FIFOReadOffset += 4;
            FIFOWritePos++;
            FIFOWritePos &= 0x7;
        }
    }

    FIFOLevel += burstlen;
}

template<typename T>
T SPUChannel::FIFO_ReadData()
{
    T ret = *(T*)&((u8*)FIFO)[FIFOReadPos];

    FIFOReadPos += sizeof(T);
    FIFOReadPos &= 0x1F;
    FIFOLevel -= sizeof(T);

    if (FIFOLevel <= 16)
        FIFO_BufferData();

    return ret;
}

void SPUChannel::Start()
{
    const s16 heldSample = CurSample;
    Timer = TimerReload;

    if (((Cnt >> 29) & 0x3) == 3)
        Pos = -1;
    else
        Pos = -3;

    NoiseVal = 0x7FFF;
    PrevSample[0] = 0;
    PrevSample[1] = 0;
    PrevSample[2] = 0;
    CurSample = HoldRestartPending ? heldSample : 0;

    FIFOReadPos = 0;
    FIFOWritePos = 0;
    FIFOReadOffset = 0;
    FIFOLevel = 0;

    // when starting a channel, buffer data
    if (((Cnt >> 29) & 0x3) != 3)
    {
        FIFO_BufferData();
        FIFO_BufferData();
    }
}

void SPUChannel::NextSample_PCM8()
{
    Pos++;
    if (Pos < 0)
    {
        if (HoldRestartPending)
        {
            CurSample = 0;
            HoldRestartPending = false;
        }
        return;
    }
    if (Pos >= (LoopPos + Length))
    {
        u32 repeat = (Cnt >> 27) & 0x3;
        if (repeat & 1)
        {
            Pos = LoopPos;
        }
        else if (repeat & 2)
        {
            CurSample = 0;
            Cnt &= ~(1<<31);
            return;
        }
    }

    s8 val = FIFO_ReadData<s8>();
    CurSample = val << 8;
    const u32 repeat = (Cnt >> 27) & 0x3;
    if ((repeat & 2) && static_cast<u32>(Pos + 1) >= (LoopPos + Length))
        Cnt &= ~(1<<31);
}

void SPUChannel::NextSample_PCM16()
{
    Pos++;
    if (Pos < 0)
    {
        if (HoldRestartPending)
        {
            CurSample = 0;
            HoldRestartPending = false;
        }
        return;
    }
    if ((Pos<<1) >= (LoopPos + Length))
    {
        u32 repeat = (Cnt >> 27) & 0x3;
        if (repeat & 1)
        {
            Pos = LoopPos>>1;
        }
        else if (repeat & 2)
        {
            CurSample = 0;
            Cnt &= ~(1<<31);
            return;
        }
    }

    s16 val = FIFO_ReadData<s16>();
    CurSample = val;
    const u32 repeat = (Cnt >> 27) & 0x3;
    if ((repeat & 2) && (static_cast<u32>(Pos + 1) << 1) >= (LoopPos + Length))
        Cnt &= ~(1<<31);
}

void SPUChannel::NextSample_ADPCM()
{
    Pos++;
    if (Pos < 8)
    {
        if (HoldRestartPending)
        {
            CurSample = 0;
            HoldRestartPending = false;
        }
        if (Pos == 0)
        {
            // setup ADPCM
            u32 header = FIFO_ReadData<u32>();
            ADPCMVal = (s32)(s16)(header & 0xFFFF);
            ADPCMIndex = (header >> 16) & 0x7F;
            if (ADPCMIndex > 88) ADPCMIndex = 88;

            ADPCMValLoop = ADPCMVal;
            ADPCMIndexLoop = ADPCMIndex;
        }

        return;
    }

    if ((Pos>>1) >= (LoopPos + Length))
    {
        u32 repeat = (Cnt >> 27) & 0x3;
        if (repeat & 1)
        {
            Pos = LoopPos<<1;
            ADPCMVal = ADPCMValLoop;
            ADPCMIndex = ADPCMIndexLoop;
            ADPCMCurByte = FIFO_ReadData<u8>();
        }
        else if (repeat & 2)
        {
            CurSample = 0;
            Cnt &= ~(1<<31);
            return;
        }
    }
    else
    {
        if (!(Pos & 0x1))
            ADPCMCurByte = FIFO_ReadData<u8>();
        else
            ADPCMCurByte >>= 4;

        u16 val = ADPCMTable[ADPCMIndex];
        u16 diff = val >> 3;
        if (ADPCMCurByte & 0x1) diff += (val >> 2);
        if (ADPCMCurByte & 0x2) diff += (val >> 1);
        if (ADPCMCurByte & 0x4) diff += val;

        if (ADPCMCurByte & 0x8)
        {
            ADPCMVal -= diff;
            if (ADPCMVal < -0x7FFF) ADPCMVal = -0x7FFF;
        }
        else
        {
            ADPCMVal += diff;
            if (ADPCMVal > 0x7FFF) ADPCMVal = 0x7FFF;
        }

        ADPCMIndex += ADPCMIndexTable[ADPCMCurByte & 0x7];
        if      (ADPCMIndex < 0)  ADPCMIndex = 0;
        else if (ADPCMIndex > 88) ADPCMIndex = 88;

        if (Pos == (LoopPos<<1))
        {
            ADPCMValLoop = ADPCMVal;
            ADPCMIndexLoop = ADPCMIndex;
        }
    }

    CurSample = ADPCMVal;
    const u32 repeat = (Cnt >> 27) & 0x3;
    if ((repeat & 2) && static_cast<u32>((Pos + 1) >> 1) >= (LoopPos + Length))
        Cnt &= ~(1<<31);
}

void SPUChannel::NextSample_PSG()
{
    Pos++;
    CurSample = PSGTable[(Cnt >> 24) & 0x7][Pos & 0x7];
}

void SPUChannel::NextSample_Noise()
{
    if (NoiseVal & 0x1)
    {
        NoiseVal = (NoiseVal >> 1) ^ 0x6000;
        CurSample = -0x7FFF;
    }
    else
    {
        NoiseVal >>= 1;
        CurSample = 0x7FFF;
    }
}

template<u32 type>
s32 SPUChannel::Run(u32 cycles)
{
    if (!(Cnt & (1<<31)))
    {
        if (!(Cnt & (1<<15))) return 0;
        s32 held = static_cast<s32>(CurSample);
        held <<= VolumeShift;
        held *= Volume;
        return held;
    }

    if ((type < 3) && ((Length+LoopPos) < 16)) return 0;

    if (KeyOn)
    {
        Start();
        KeyOn = false;
    }

    // 1 sample = 512 cycles at 16MHz
    // (or 352 cycles at 47KHz)
    Timer += cycles;

    while (Timer >> 16)
    {
        Timer = TimerReload + (Timer - 0x10000);

        // for optional interpolation: save previous samples
        // the interpolated audio will be delayed by a couple samples,
        // but it's easier to deal with this way
        if ((type < 3) && (InterpType != AudioInterpolation::None))
        {
            PrevSample[2] = PrevSample[1];
            PrevSample[1] = PrevSample[0];
            PrevSample[0] = CurSample;
        }

        switch (type)
        {
        case 0: NextSample_PCM8(); break;
        case 1: NextSample_PCM16(); break;
        case 2: NextSample_ADPCM(); break;
        case 3: NextSample_PSG(); break;
        case 4: NextSample_Noise(); break;
        }

        if (!(Cnt & (1<<31))) break;
    }

    s32 val = (s32)CurSample;

    // interpolation (emulation improvement, not a hardware feature)
    if ((type < 3) && (InterpType != AudioInterpolation::None))
    {
        s32 samplepos = ((Timer - TimerReload) * 0x100) / (0x10000 - TimerReload);
        if (samplepos > 0xFF) samplepos = 0xFF;

        switch (InterpType)
        {
        case AudioInterpolation::Linear:
            val = ((val           * samplepos) +
                   (PrevSample[0] * (0xFF-samplepos))) >> 8;
            break;

        case AudioInterpolation::Cosine:
            val = ((val           * InterpCos[samplepos]) +
                   (PrevSample[0] * InterpCos[0xFF-samplepos])) >> 14;
            break;

        case AudioInterpolation::Cubic:
            val = ((PrevSample[2] * InterpCubic[samplepos][0]) +
                   (PrevSample[1] * InterpCubic[samplepos][1]) +
                   (PrevSample[0] * InterpCubic[samplepos][2]) +
                   (val           * InterpCubic[samplepos][3])) >> 14;
            break;

        case AudioInterpolation::SNESGaussian: {
                // Avoid clipping (from fullsnes)
#define CLAMP(s) (std::clamp((s) >> 1, -0x3FFA, 0x3FF8))
                s32 out =    (InterpSNESGauss[0x0FF - samplepos] * CLAMP(PrevSample[2]) >> 10);
                out = out + ((InterpSNESGauss[0x1FF - samplepos] * CLAMP(PrevSample[1])) >> 10);
                out = out + ((InterpSNESGauss[0x100 + samplepos] * CLAMP(PrevSample[0])) >> 10);
                out = out + ((InterpSNESGauss[0x000 + samplepos] * CLAMP(val)) >> 10);
                val = std::clamp(out, -0x8000, 0x7FFF);
#undef CLAMP
                break;
            }

        default:
            break;
        }
    }

    val <<= VolumeShift;
    val *= Volume;
    return val;
}

void SPUChannel::PanOutput(s32 in, s32& left, s32& right)
{
    left += ((s64)in * (128-Pan)) >> 10;
    right += ((s64)in * Pan) >> 10;
}


SPUCaptureUnit::SPUCaptureUnit(u32 num, melonDS::NDS& nds) : NDS(nds), Num(num)
{
}

void SPUCaptureUnit::Reset()
{
    SetCnt(0);
    DstAddr = 0;
    TimerReload = 0;
    Length = 0;

    Timer = 0;

    Pos = 0;
    FIFOReadPos = 0;
    FIFOWritePos = 0;
    FIFOWriteOffset = 0;
    FIFOLevel = 0;
}

void SPUCaptureUnit::DoSavestate(Savestate* file)
{
    file->Var8(&Cnt);
    file->Var32(&DstAddr);
    file->Var16(&TimerReload);
    file->Var32(&Length);

    file->Var32(&Timer);
    file->Var32((u32*)&Pos);

    file->Var32(&FIFOReadPos);
    file->Var32(&FIFOWritePos);
    file->Var32(&FIFOWriteOffset);
    file->Var32(&FIFOLevel);
    file->VarArray(FIFO, 4*4);
}

void SPUCaptureUnit::FIFO_FlushData()
{
    for (u32 i = 0; i < 4; i++)
    {
        NDS.ARM7Write32(DstAddr + FIFOWriteOffset, FIFO[FIFOReadPos]);
        // Calls the NDS or DSi version, depending on the class

        FIFOReadPos++;
        FIFOReadPos &= 0x3;
        FIFOLevel -= 4;

        FIFOWriteOffset += 4;
        if (FIFOWriteOffset >= Length)
        {
            FIFOWriteOffset = 0;
            break;
        }
    }
}

template<typename T>
void SPUCaptureUnit::FIFO_WriteData(T val)
{
    *(T*)&((u8*)FIFO)[FIFOWritePos] = val;

    FIFOWritePos += sizeof(T);
    FIFOWritePos &= 0xF;
    FIFOLevel += sizeof(T);

    if (FIFOLevel >= 16)
        FIFO_FlushData();
}

void SPUCaptureUnit::Run(u32 cycles, s32 sample)
{
    Timer += cycles;

    if (Cnt & 0x08)
    {
        while (Timer >> 16)
        {
            Timer = TimerReload + (Timer - 0x10000);

            FIFO_WriteData<s8>((s8)CaptureTruncateTowardZero(sample, 8));
            Pos++;
            if (Pos >= Length)
            {
                if (FIFOLevel >= 4)
                    FIFO_FlushData();

                if (Cnt & 0x04)
                {
                    Cnt &= 0x7F;
                    return;
                }
                else
                    Pos = 0;
            }
        }
    }
    else
    {
        while (Timer >> 16)
        {
            Timer = TimerReload + (Timer - 0x10000);

            FIFO_WriteData<s16>((s16)sample);
            Pos += 2;
            if (Pos >= Length)
            {
                if (FIFOLevel >= 4)
                    FIFO_FlushData();

                if (Cnt & 0x04)
                {
                    Cnt &= 0x7F;
                    return;
                }
                else
                    Pos = 0;
            }
        }
    }
}


void SPU::Mix(u32 spucycles)
{
    s32 left = 0, right = 0;
    s32 leftoutput = 0, rightoutput = 0;

    if (Cnt & (1<<15))
    {
        s32 ch0, ch1, ch2, ch3;
        bool add01;
        bool add23;

        if (!(Capture[0].Cnt & (1<<7)) && !(Capture[1].Cnt & (1<<7)))
        {

            ch0 = Channels[0].DoRun(spucycles);
            ch1 = Channels[1].DoRun(spucycles);
            ch2 = Channels[2].DoRun(spucycles);
            ch3 = Channels[3].DoRun(spucycles);

            add01 = false;
            add23 = false;

            Channels[0].PanOutput(ch0, left, right);
            Channels[2].PanOutput(ch2, left, right);

            if (!(Cnt & (1<<12))) Channels[1].PanOutput(ch1, left, right);
            if (!(Cnt & (1<<13))) Channels[3].PanOutput(ch3, left, right);

            for (int i = 4; i < 16; i++)
            {
                SPUChannel* chan = &Channels[i];

                s32 channel = chan->DoRun(spucycles);
                chan->PanOutput(channel, left, right);
            }
        }
        else
        {
            std::array<s32, 16> channelSamples;
            for (u32 i = 0; i < channelSamples.size(); i++)
                channelSamples[i] = Channels[i].DoRun(0);

            auto mixChannels = [&](bool mixAdd01, bool mixAdd23,
                                   s32& mixLeft, s32& mixRight) {
                mixLeft = 0;
                mixRight = 0;

                s32 ch0Mixer = channelSamples[0];
                s32 ch2Mixer = channelSamples[2];
                if (mixAdd01 && (Channels[0].Cnt & (1<<31)) && !(Cnt & (1<<12)))
                    ch0Mixer += channelSamples[1];
                if (mixAdd23 && (Channels[2].Cnt & (1<<31)) && !(Cnt & (1<<13)))
                    ch2Mixer += channelSamples[3];

                Channels[0].PanOutput(ch0Mixer, mixLeft, mixRight);
                Channels[2].PanOutput(ch2Mixer, mixLeft, mixRight);
                if (!mixAdd01 && !(Cnt & (1<<12)))
                    Channels[1].PanOutput(channelSamples[1], mixLeft, mixRight);
                if (!mixAdd23 && !(Cnt & (1<<13)))
                    Channels[3].PanOutput(channelSamples[3], mixLeft, mixRight);

                for (u32 i = 4; i < channelSamples.size(); i++)
                    Channels[i].PanOutput(channelSamples[i], mixLeft, mixRight);
            };

            u32 cyclesLeft = spucycles;
            while (cyclesLeft > 0)
            {
                if (!(Capture[0].Cnt & (1<<7)) && !(Capture[1].Cnt & (1<<7)))
                {
                    for (u32 i = 0; i < channelSamples.size(); i++)
                        channelSamples[i] = Channels[i].DoRun(cyclesLeft);
                    cyclesLeft = 0;
                    break;
                }

                u32 step = cyclesLeft;
                auto includeTimer = [&](u32 timer) {
                    const u32 untilOverflow = 0x10000 - (timer & 0xFFFF);
                    step = std::min(step, untilOverflow);
                };
#if defined(MELONDS_SPU_CAPTURE_EDGE_STEPPING)
                // Step from capture edge to capture edge instead of stopping at
                // every channel edge. A capture unit only samples the mixer when
                // its own timer overflows, and the value it must see is the one
                // made of every channel edge strictly before that cycle. Running
                // the channels up to one cycle short of the step gives exactly
                // that state, at a fraction of the sub-steps.
                if (Capture[0].Cnt & (1<<7)) includeTimer(Capture[0].Timer);
                if (Capture[1].Cnt & (1<<7)) includeTimer(Capture[1].Timer);

                if (step > 1)
                {
                    for (u32 i = 0; i < channelSamples.size(); i++)
                        channelSamples[i] = Channels[i].DoRun(step - 1);
                }
#else
                for (u32 i = 0; i < channelSamples.size(); i++)
                {
                    if (Channels[i].Cnt & (1<<31))
                        includeTimer(Channels[i].Timer);
                }
                if (Capture[0].Cnt & (1<<7)) includeTimer(Capture[0].Timer);
                if (Capture[1].Cnt & (1<<7)) includeTimer(Capture[1].Timer);
#endif

                const bool edgeAdd01 = (Capture[0].Cnt & 0x81) == 0x81;
                const bool edgeAdd23 = (Capture[1].Cnt & 0x81) == 0x81;
                s32 edgeLeft, edgeRight;
                mixChannels(edgeAdd01, edgeAdd23, edgeLeft, edgeRight);

                const s32 capture0Value = (Capture[0].Cnt & (1<<1))
                    ? CaptureDirectToPCM16(
                        channelSamples[0], channelSamples[1], edgeAdd01)
                    : CaptureMixerToPCM16(edgeLeft);
                const s32 capture1Value = (Capture[1].Cnt & (1<<1))
                    ? CaptureDirectToPCM16(
                        channelSamples[2], channelSamples[3], edgeAdd23)
                    : CaptureMixerToPCM16(edgeRight);

                if (Capture[0].Cnt & (1<<7))
                    Capture[0].Run(step, capture0Value);
                if (Capture[1].Cnt & (1<<7))
                    Capture[1].Run(step, capture1Value);

#if defined(MELONDS_SPU_CAPTURE_EDGE_STEPPING)
                for (u32 i = 0; i < channelSamples.size(); i++)
                    channelSamples[i] = Channels[i].DoRun(1);
#else
                for (u32 i = 0; i < channelSamples.size(); i++)
                    channelSamples[i] = Channels[i].DoRun(step);
#endif
                cyclesLeft -= step;
            }

            ch0 = channelSamples[0];
            ch1 = channelSamples[1];
            ch2 = channelSamples[2];
            ch3 = channelSamples[3];
            add01 = (Capture[0].Cnt & 0x81) == 0x81;
            add23 = (Capture[1].Cnt & 0x81) == 0x81;
            mixChannels(add01, add23, left, right);
        }

        // final output

        switch (Cnt & 0x0300)
        {
        case 0x0000: // left mixer
            leftoutput = left;
            break;
        case 0x0100: // channel 1
            {
                s32 pan = 128 - Channels[1].Pan;
                leftoutput = ((s64)ch1 * pan) >> 10;
            }
            break;
        case 0x0200: // channel 3
            {
                s32 pan = 128 - Channels[3].Pan;
                leftoutput = ((s64)ch3 * pan) >> 10;
            }
            break;
        case 0x0300: // channel 1+3
            {
                s32 pan1 = 128 - Channels[1].Pan;
                s32 pan3 = 128 - Channels[3].Pan;
                leftoutput = (((s64)ch1 * pan1) >> 10) + (((s64)ch3 * pan3) >> 10);
            }
            break;
        }

        switch (Cnt & 0x0C00)
        {
        case 0x0000: // right mixer
            rightoutput = right;
            break;
        case 0x0400: // channel 1
            {
                s32 pan = Channels[1].Pan;
                rightoutput = ((s64)ch1 * pan) >> 10;
            }
            break;
        case 0x0800: // channel 3
            {
                s32 pan = Channels[3].Pan;
                rightoutput = ((s64)ch3 * pan) >> 10;
            }
            break;
        case 0x0C00: // channel 1+3
            {
                s32 pan1 = Channels[1].Pan;
                s32 pan3 = Channels[3].Pan;
                rightoutput = (((s64)ch1 * pan1) >> 10) + (((s64)ch3 * pan3) >> 10);
            }
            break;
        }
    }

    leftoutput = ((s64)leftoutput * MasterVolume) >> 7;
    rightoutput = ((s64)rightoutput * MasterVolume) >> 7;

    leftoutput >>= 8;
    rightoutput >>= 8;

    // Add SOUNDBIAS value
    // The value used by all commercial games is 0x200, so we subtract that so it won't offset the final sound output.
    if (ApplyBias)
    {
        leftoutput += (Bias << 6) - 0x8000;
        rightoutput += (Bias << 6) - 0x8000;
    }

    s16 output[2];
    if (Mute)
    {
        // on the DSi, POWCNT2 bit 0 only disables NITRO mixer output
        output[0] = 0;
        output[1] = 0;
    }
    else
    {
        output[0] = (s16)std::clamp(leftoutput, -0x8000, 0x7FFF);
        output[1] = (s16)std::clamp(rightoutput, -0x8000, 0x7FFF);
    }

    NDS.Mic.Advance(spucycles << 1);

    if (NDS.ConsoleType == 1)
    {
        // for the DSi, we run the I2S interface here, so it can mix in DSP audio
        // this isn't the cleanest, but it's the easiest, since the audio output apparatus is here
        ((DSi&)NDS).I2S.SampleClock(output);
    }

    // The original DS and DS lite degrade the output from 16 to 10 bit before output
    if (Degrade10Bit)
    {
        output[0] &= 0xFFC0;
        output[1] &= 0xFFC0;
    }

    BlipTimer += spucycles;

    if (output[0] != OutputLastSamples[0])
        blip_add_delta(BlipLeft, BlipTimer, (int) output[0] - OutputLastSamples[0]);
    if (output[1] != OutputLastSamples[1])
        blip_add_delta(BlipRight, BlipTimer, (int) output[1] - OutputLastSamples[1]);

    OutputLastSamples[0] = output[0];
    OutputLastSamples[1] = output[1];

    if (BlipTimer >= 512 * 128)
        BufferAudio();

    NDS.ScheduleEvent(Event_SPU, true, MixInterval, 0, MixInterval >> 1);
}

void SPU::BufferAudio()
{

    const double sourceAppliedSkew =
        OutputSkewPublicado.load(std::memory_order_relaxed);
    const u32 ticksTerminados = (u32)BlipTimer;
    blip_end_frame(BlipLeft, ticksTerminados);
    blip_end_frame(BlipRight, ticksTerminados);
    BlipTimer = 0;

    int avail = blip_samples_avail(BlipLeft);

    s16 temp[blip_max_frame * 2];
    blip_read_samples(BlipLeft, temp, avail, true);
    blip_read_samples(BlipRight, temp + 1, avail, true);

    std::unique_ptr<OutputSpillNode> spillCandidate;
    bool spillAllocationAttempted = false;
    bool spillAllocationSucceeded = false;
    if (avail > 0)
    {
        bool nodeMayBeNeeded;
        Platform::Mutex_Lock(AudioLock);
        if (OutputSpillActive)
        {

            nodeMayBeNeeded = true;
        }
        else
        {
            nodeMayBeNeeded = static_cast<u32>(avail)
                > OutputBufferSize - 1 - OutputRingLevelLocked();
        }
        if (nodeMayBeNeeded)
            spillCandidate = TakeOutputSpillFreeLocked();
        Platform::Mutex_Unlock(AudioLock);

        if (nodeMayBeNeeded && !spillCandidate)
        {
            spillAllocationAttempted = true;
            spillCandidate.reset(new (std::nothrow) OutputSpillNode());
            spillAllocationSucceeded = spillCandidate != nullptr;
        }
    }

    u64 hostConsumed;
    u64 ticksAlConsumo;
    u64 framesAlConsumo;
    int nivelPostConsumo;
    int nivelAntesEscritura;
    int nivelPostEscritura;
    u64 nivelPostConsumoLogico;
    u64 nivelAntesEscrituraLogico;
    u64 nivelPostEscrituraLogico;
    u32 modoDrenado;
    u32 resetSolicitado;
    u32 resetConfirmado;
    bool priming;
    u64 continuidadEpoch;
    u32 dropsEstaEscritura = 0;
    u32 spillDroppedPacketsEstaEscritura = 0;
    u32 spillDroppedFramesEstaEscritura = 0;
    AudioOutputProducerPacketObservation producerObservation {};
    Platform::Mutex_Lock(AudioLock);
    const u64 provenanceAcceptedBefore =
        OutputProvenanceAcceptedFrames;
    if (spillAllocationAttempted)
    {
        if (spillAllocationSucceeded)
            ++OutputSpillAllocationCount;
    }
    OutputSpillBirthThisWrite = false;
    PromoteOutputSpillLocked(std::numeric_limits<u32>::max());
    nivelAntesEscritura = static_cast<int>(OutputRingLevelLocked());
    nivelAntesEscrituraLogico = OutputLogicalLevelLocked();
    modoDrenado = AdaptModoDrenado.load(std::memory_order_acquire);
    AudioOutputAdaptiveTelemetrySnapshot& telemetry = AdaptTelemetryState;
    ++telemetry.updateId;
    const u32 resetAntesEscritura =
        AdaptResetEpoch.load(std::memory_order_acquire);
    const u32 hintAntesEscritura =
        AdaptHintEpoch.load(std::memory_order_acquire);
    const bool parentCapacityPreWriteVigente =
        modoDrenado == 2
        && AdaptParentCapacityActivo
        && !AdaptParentCapacityPrimerRiesgoVisto
        && AdaptParentCapacityCapacidad == OutputBufferSize - 1
        && AdaptParentCapacityContinuidadEpoch == AdaptContinuidadEpoch
        && AdaptParentCapacityResetEpoch == resetAntesEscritura
        && AdaptParentCapacityHintSeenEpoch == hintAntesEscritura;
    const u32 capacidadUtil = OutputBufferSize - 1;
    if (parentCapacityPreWriteVigente
        && nivelAntesEscrituraLogico <= capacidadUtil
        && static_cast<u32>(avail)
           > capacidadUtil - nivelAntesEscrituraLogico)
    {

        AdaptParentCapacityPrimerRiesgoVisto = true;
        ++telemetry.parentCapacityFirstRiskCount;
        telemetry.lastParentCapacityFirstRiskUpdateId = telemetry.updateId;
        telemetry.lastParentCapacityFirstRiskGeneration =
            AdaptParentCapacityGeneracion;
        telemetry.lastParentCapacityFirstRiskLevelBeforeWrite =
            static_cast<u32>(nivelAntesEscrituraLogico);
        telemetry.lastParentCapacityFirstRiskFramesProduced =
            static_cast<u32>(avail);
        telemetry.lastParentCapacityFirstRiskCapacity = capacidadUtil;
    }
    u32 framesRemaining = static_cast<u32>(avail);
    u32 sourceFrame = 0;
    const bool needsNode = OutputSpillActive
        ? framesRemaining > static_cast<u32>(blip_max_frame)
            - OutputSpillTail->writePos
        : framesRemaining > OutputBufferSize - 1 - OutputRingLevelLocked();
    if (needsNode && !spillCandidate)
    {

        spillCandidate = TakeOutputSpillFreeLocked();
    }
    if (needsNode && !spillCandidate)
    {

        AdaptDescartes.fetch_add(1, std::memory_order_relaxed);
        ++dropsEstaEscritura;
        ++OutputSpillDroppedPacketsTotal;
        OutputSpillDroppedFramesTotal += framesRemaining;
        if (spillAllocationAttempted && !spillAllocationSucceeded)
            ++OutputSpillAllocationFailureCount;
        spillDroppedPacketsEstaEscritura = 1;
        spillDroppedFramesEstaEscritura = framesRemaining;
        framesRemaining = 0;
    }
    else
    {
        if (!OutputSpillActive)
        {
            const u32 ringFrames = std::min(
                framesRemaining,
                OutputBufferSize - 1 - OutputRingLevelLocked());
            for (u32 i = 0; i < ringFrames; ++i)
            {
                OutputBuffer[OutputBufferWritePos++] =
                    temp[(sourceFrame + i) * 2];
                OutputBuffer[OutputBufferWritePos++] =
                    temp[(sourceFrame + i) * 2 + 1];
                OutputBufferWritePos &= ((2 * OutputBufferSize) - 1);
            }
            sourceFrame += ringFrames;
            framesRemaining -= ringFrames;
        }

        u32 spillPublished = 0;
        while (framesRemaining > 0)
        {
            if (!OutputSpillActive)
            {
                spillCandidate->readPos = 0;
                spillCandidate->writePos = 0;
                OutputSpillActive = std::move(spillCandidate);
                OutputSpillTail = OutputSpillActive.get();
                ++OutputSpillActiveNodes;
                ++OutputSpillGeneration;
                ++OutputSpillBirthCount;
                OutputSpillBirthThisWrite = true;
                OutputSpillLastBirthContinuityEpoch = AdaptContinuidadEpoch;
                OutputSpillLastBirthResetEpoch = resetAntesEscritura;
                OutputSpillLastBirthOwnerGeneration =
                    AdaptSostenidoOwnerActivo
                    ? AdaptSostenidoOwnerGeneracion
                    : parentCapacityPreWriteVigente
                        ? AdaptParentCapacitySourceOwnerGeneracion : 0;
            }

            u32 tailFree = static_cast<u32>(blip_max_frame)
                - OutputSpillTail->writePos;
            if (tailFree == 0)
            {
                spillCandidate->readPos = 0;
                spillCandidate->writePos = 0;
                OutputSpillTail->next = std::move(spillCandidate);
                OutputSpillTail = OutputSpillTail->next.get();
                ++OutputSpillActiveNodes;
                tailFree = static_cast<u32>(blip_max_frame);
            }
            const u32 count = std::min(framesRemaining, tailFree);
            for (u32 i = 0; i < count; ++i)
            {
                const u32 destination =
                    (OutputSpillTail->writePos + i) * 2;
                OutputSpillTail->samples[destination] =
                    temp[(sourceFrame + i) * 2];
                OutputSpillTail->samples[destination + 1] =
                    temp[(sourceFrame + i) * 2 + 1];
            }
            OutputSpillTail->writePos += count;
            OutputSpillFrames += count;
            sourceFrame += count;
            framesRemaining -= count;
            spillPublished += count;
        }
        if (spillPublished > 0)
        {
            ++OutputSpillPublicationCount;
            OutputSpillFramesPublishedTotal += spillPublished;
            OutputSpillPeakFrames = std::max(
                OutputSpillPeakFrames, OutputSpillFrames);
            if (OutputSpillBirthThisWrite)
                OutputSpillLastBirthFrames = spillPublished;
        }
    }
    ReturnOutputSpillFreeLocked(std::move(spillCandidate));
    OutputProvenanceAcceptedFrames += static_cast<u64>(sourceFrame);
    nivelPostEscritura = static_cast<int>(OutputRingLevelLocked());
    nivelPostEscrituraLogico = OutputLogicalLevelLocked();

    AdaptTicksTotal += ticksTerminados;
    AdaptFramesProducidosTotal += (u64)avail;
    AdaptTicksPublicados.store(AdaptTicksTotal, std::memory_order_relaxed);
    hostConsumed = AdaptHostConsumedTotal;
    ticksAlConsumo = AdaptTicksAlConsumo;
    framesAlConsumo = AdaptFramesAlConsumo;
    nivelPostConsumo = AdaptNivelPostConsumo;
    nivelPostConsumoLogico = AdaptNivelPostConsumoLogico;
    resetSolicitado = AdaptResetEpoch.load(std::memory_order_acquire);
    resetConfirmado = AdaptResetConfirmado;
    priming = AdaptPriming;
    continuidadEpoch = AdaptContinuidadEpoch;

    telemetry.hostConsumed = hostConsumed;
    telemetry.ticksAtConsumption = ticksAlConsumo;
    telemetry.framesAtConsumption = framesAlConsumo;
    telemetry.estimatorDeltaConsumed = 0;
    telemetry.estimatorDeltaTicks = 0;
    telemetry.estimatorRawRatio = 0.0;
    telemetry.levelPostConsumption = static_cast<u32>(nivelPostConsumo);
    telemetry.levelBeforeWrite = static_cast<u32>(nivelAntesEscritura);
    telemetry.levelPostWrite = static_cast<u32>(nivelPostEscritura);
    telemetry.framesProduced = static_cast<u32>(avail);
    const u32 underrunsActuales =
        AdaptUnderruns.load(std::memory_order_relaxed);
    telemetry.dropsThisWrite = dropsEstaEscritura;
    telemetry.underruns = underrunsActuales;
    telemetry.outputBufferSize = OutputBufferSize;
    telemetry.resetRequestedEpoch = resetSolicitado;
    telemetry.resetConfirmedEpoch = resetConfirmado;
    telemetry.decisionFlags = modoDrenado == 2
        ? AudioOutputAdaptiveMode
        : AudioOutputAdaptiveModeBlocked;

    if (modoDrenado == 2)
        UpdateOutputAdaptivo(hostConsumed, ticksAlConsumo, framesAlConsumo,
                             nivelPostConsumo, nivelPostEscritura,
                             nivelPostConsumoLogico,
                             nivelPostEscrituraLogico,
                             resetSolicitado,
                             resetConfirmado, priming, continuidadEpoch,
                             dropsEstaEscritura, underrunsActuales,
                             telemetry);

    const bool fronteraPhaseCambioTargetVisible = modoDrenado == 2
        && AdaptFastCambioTargetPendiente;
    telemetry.fastTargetChangePhaseFrontierOrigin =
        fronteraPhaseCambioTargetVisible
        ? AdaptFastCambioTargetFronteraPhaseOrigen
        : AudioOutputAdaptiveFastTargetChangePhaseFrontierNone;
    telemetry.fastTargetChangePhaseFrontierValid =
        fronteraPhaseCambioTargetVisible
        && AdaptFastCambioTargetFronteraPhaseValida;
    telemetry.fastTargetChangePhaseEscrowActive = modoDrenado == 2
        && AdaptFastCambioTargetPhaseEscrowActivo;
    telemetry.fastTargetChangePhaseEscrowGrantFrames =
        AdaptFastCambioTargetPhaseEscrowGrantFrames;
    telemetry.fastTargetChangePhaseEscrowGuardSkew =
        AdaptFastCambioTargetPhaseEscrowGuardSkew;
    telemetry.fastTargetChangePhaseEscrowOverlaySkew =
        AdaptFastCambioTargetPhaseEscrowOverlaySkew;
    telemetry.fastTargetChangePhaseEscrowReferenceRate =
        AdaptFastCambioTargetPhaseEscrowRateReferencia;
    telemetry.fastTargetChangePhaseEscrowStartTicks =
        AdaptFastCambioTargetPhaseEscrowTicksInicio;
    telemetry.fastTargetChangePhaseEscrowStartConsumed =
        AdaptFastCambioTargetPhaseEscrowConsInicio;
    telemetry.fastTargetChangePhaseEscrowStartConsumedProduced =
        AdaptFastCambioTargetPhaseEscrowFramesConsumoInicio;
    telemetry.fastTargetChangePhaseEscrowStartPublishedProduced =
        AdaptFastCambioTargetPhaseEscrowFramesPublicadosInicio;
    telemetry.fastTargetChangePhaseEscrowStartLevelPostConsumption =
        AdaptFastCambioTargetPhaseEscrowNivelConsumoInicio;
    telemetry.fastTargetChangePhaseEscrowStartLevelPostWrite =
        AdaptFastCambioTargetPhaseEscrowNivelEscrituraInicio;
    telemetry.fastTargetChangePhaseEscrowStartContinuityEpoch =
        AdaptFastCambioTargetPhaseEscrowContinuidadEpoch;
    telemetry.fastTargetChangePhaseEscrowStartResetEpoch =
        AdaptFastCambioTargetPhaseEscrowResetEpoch;
    telemetry.fastTargetChangePhaseEscrowStartHintSeenEpoch =
        AdaptFastCambioTargetPhaseEscrowHintSeenEpoch;
    telemetry.fastTargetChangePhaseEscrowStartCandidateGeneration =
        AdaptFastCambioTargetPhaseEscrowCandidatoGeneracion;
    telemetry.fastTargetChangePhaseEscrowElapsedPublishedTicks =
        AdaptFastCambioTargetPhaseEscrowActivo
        && AdaptTicksTotal >= AdaptFastCambioTargetPhaseEscrowTicksInicio
        ? AdaptTicksTotal - AdaptFastCambioTargetPhaseEscrowTicksInicio : 0;
    telemetry.fastTargetChangePhaseEscrowCurrentPhysicalPrefix =
        AdaptFastCambioTargetPhaseEscrowActivo
        ? static_cast<u64>(std::max(0, nivelPostEscritura)) : 0;
    telemetry.sustainedProvisionalPhaseActive = modoDrenado == 2
        && AdaptSostenidoPhaseProvisionalActivo;
    telemetry.sustainedProvisionalPhaseCandidateGeneration =
        AdaptSostenidoPhaseProvisionalCandidatoGeneracion;
    telemetry.sustainedProvisionalPhaseOwnerGeneration =
        AdaptSostenidoPhaseProvisionalOwnerGeneracion;
    telemetry.sustainedProvisionalPhaseParentRatio =
        AdaptSostenidoPhaseProvisionalParentRatio;
    telemetry.sustainedProvisionalPhaseSkew =
        AdaptSostenidoPhaseProvisionalSkew;
    telemetry.sustainedProvisionalPhaseFrontierFrames =
        AdaptSostenidoPhaseProvisionalFronteraFrames;
    telemetry.sustainedProvisionalPhaseBackingFrames =
        AdaptSostenidoPhaseProvisionalBackingFrames;
    telemetry.sustainedProvisionalPhasePublicationReserveFrames =
        AdaptSostenidoPhaseProvisionalReservaPublicacionFrames;
    telemetry.sustainedProvisionalPhaseReservedPublicationCount =
        AdaptSostenidoPhaseProvisionalPublicacionesReservadas;
    telemetry.provisionalFastStateFlags = modoDrenado == 2
        ? (AdaptFastLowPendiente
           ? AudioOutputAdaptiveProvisionalFastLowPending : 0u)
          | (AdaptFastCambioTargetPendiente
             ? AudioOutputAdaptiveProvisionalFastTargetChangePending : 0u)
        : 0u;
    telemetry.provisionalFastAnchorConsumed = AdaptFastConsInicio;
    telemetry.provisionalFastAnchorTicks = AdaptFastTicksInicio;
    telemetry.provisionalFastAnchorProduced = AdaptFastFramesInicio;
    telemetry.provisionalFastAnchorPublishedProduced =
        AdaptFastFramesPublicadosInicio;
    telemetry.provisionalFastAnchorLevelPostConsumption =
        AdaptFastNivelInicioLogico;
    telemetry.provisionalFastAnchorLevelPostWrite =
        AdaptFastNivelEscrituraInicioLogico;
    telemetry.provisionalFastAnchorContinuityEpoch =
        AdaptFastContinuidadEpochInicio;
    telemetry.provisionalFastAnchorResetEpoch = AdaptFastResetEpochInicio;
    telemetry.provisionalFastAnchorHintSeenEpoch =
        AdaptFastHintSeenEpochInicio;
    telemetry.provisionalFastAnchorCandidateGeneration =
        AdaptFastCandidatoGeneracionInicio;
    telemetry.provisionalFastAnchorOwnerGeneration =
        AdaptFastAnchorOwnerGeneracion;
    telemetry.provisionalCandidateStartConsumed = AdaptSostenidoConsInicio;
    telemetry.provisionalCandidateStartTicks = AdaptSostenidoTicksInicio;
    telemetry.provisionalCandidateStartProduced = AdaptSostenidoFramesInicio;
    telemetry.provisionalCandidateStartPublishedProduced =
        AdaptSostenidoFramesPublicadosInicio;
    telemetry.provisionalCandidateStartLevelPostConsumption =
        AdaptSostenidoNivelInicioLogico;
    telemetry.provisionalCandidateStartLevelPostWrite =
        AdaptSostenidoNivelEscrituraInicioLogico;
    telemetry.provisionalCandidateStartContinuityEpoch =
        AdaptSostenidoContinuidadEpochInicio;
    telemetry.provisionalCandidateStartResetEpoch =
        AdaptSostenidoResetEpochInicio;
    telemetry.provisionalCandidateStartHintSeenEpoch =
        AdaptSostenidoHintSeenEpochInicio;
    telemetry.provisionalCandidateDirection = AdaptSostenidoDireccion;

    const bool rollbackVisible = modoDrenado == 2
        && AdaptRateRollbackPendiente;
    telemetry.rollbackPending = rollbackVisible;
    telemetry.rollbackVerifying = rollbackVisible
        && AdaptRateRollbackVerificando;
    telemetry.rollbackParentRatio = rollbackVisible
        ? AdaptRateRollbackRatio : 0.0;
    telemetry.rollbackTargetFrames = rollbackVisible
        ? static_cast<u32>(std::max(0, AdaptRateRollbackObjetivoFrames))
        : 0;
    telemetry.hintParentCertified = modoDrenado == 2
        && AdaptHintPadreCertificado;
    telemetry.rollbackParentIsHint = rollbackVisible
        && AdaptRateRollbackPadreEsHint;
    telemetry.actuatorActive = modoDrenado == 2
        && AdaptRateActuadorPendiente;
    telemetry.sustainedCandidateGeneration =
        AdaptSostenidoCandidatoGeneracion;
    telemetry.sustainedOwnerGeneration = AdaptSostenidoOwnerGeneracion;
    telemetry.sustainedOwnerCandidateGeneration =
        AdaptSostenidoOwnerCandidatoGeneracion;
    telemetry.sustainedCandidateActive = modoDrenado == 2
        && AdaptSostenidoCandidatoActivo;
    telemetry.sustainedOwnerActive = modoDrenado == 2
        && AdaptSostenidoOwnerActivo;
    telemetry.sustainedOwnerRatio = telemetry.sustainedOwnerActive
        ? AdaptSostenidoOwnerRatio : 0.0;
    telemetry.sustainedStateFlags = 0;
    if (modoDrenado == 2 && AdaptSostenidoSegmentoUnoCompleto)
        telemetry.sustainedStateFlags |=
            AudioOutputAdaptiveSustainedSegmentOneComplete;
    if (modoDrenado == 2 && AdaptSostenidoCandidatoReemplazo)
        telemetry.sustainedStateFlags |=
            AudioOutputAdaptiveSustainedCandidateReplacement;
    if (modoDrenado == 2 && AdaptSostenidoCandidatoReemplazo
        && AdaptSostenidoSegmentoUnoCompleto)
    {
        telemetry.sustainedStateFlags |=
            AudioOutputAdaptiveSustainedReplacementVerification;
    }
    if (modoDrenado == 2 && AdaptSostenidoOverrideActivo)
        telemetry.sustainedStateFlags |=
            AudioOutputAdaptiveSustainedFastOverride;
    if (modoDrenado == 2 && AdaptSostenidoParentValido
        && AdaptSostenidoParentEsHint)
    {
        telemetry.sustainedStateFlags |=
            AudioOutputAdaptiveSustainedParentIsHint;
    }
    if (modoDrenado == 2 && AdaptPhaseObjetivoActivo)
        telemetry.sustainedStateFlags |=
            AudioOutputAdaptiveSustainedEffectivePhase;
    if (modoDrenado == 2 && AdaptSostenidoFastHighWitnessActivo)
        telemetry.sustainedStateFlags |=
            AudioOutputAdaptiveSustainedFastHighWitness;
    if (modoDrenado == 2
        && AdaptSostenidoFastHighWitnessActivo
        && AdaptSostenidoFastHighWitnessCapacidad)
    {
        telemetry.sustainedStateFlags |=
            AudioOutputAdaptiveSustainedFastHighCapacity;
    }
    if (modoDrenado == 2
        && AdaptSostenidoFastHighGuardSkew > 0.0)
    {
        telemetry.sustainedStateFlags |=
            AudioOutputAdaptiveSustainedFastHighGuard;
    }
    if (modoDrenado == 2 && AdaptSostenidoFastLowWitnessActivo)
    {
        telemetry.sustainedStateFlags |=
            AudioOutputAdaptiveSustainedFastLowWitness;
    }
    if (modoDrenado == 2
        && AdaptSostenidoFastHighEscrowInicializado)
    {
        telemetry.sustainedStateFlags |=
            AudioOutputAdaptiveSustainedFastHighEscrowInitialized;
    }
    if (modoDrenado == 2 && AdaptSostenidoFastHighEscrowActivo)
    {
        telemetry.sustainedStateFlags |=
            AudioOutputAdaptiveSustainedFastHighEscrowActive;
    }
    telemetry.fastLowWitnessOwnerGeneration =
        AdaptSostenidoFastLowWitnessOwnerGeneracion;
    telemetry.fastLowWitnessEndConsumed =
        AdaptSostenidoFastLowWitnessConsFin;
    telemetry.fastLowWitnessEndTicks =
        AdaptSostenidoFastLowWitnessTicksFin;
    telemetry.fastLowWitnessEndProduced =
        AdaptSostenidoFastLowWitnessFramesFin;
    telemetry.fastLowWitnessEndLevel = static_cast<u32>(
        std::max(0, AdaptSostenidoFastLowWitnessNivelFin));
    telemetry.framesPublishedTotal = AdaptFramesProducidosTotal;
    telemetry.parentCapacityActive = modoDrenado == 2
        && AdaptParentCapacityActivo;
    telemetry.parentCapacityCapacity = AdaptParentCapacityCapacidad;
    telemetry.parentCapacityGeneration =
        AdaptParentCapacityGeneracion;
    telemetry.parentCapacitySourceOwnerGeneration =
        AdaptParentCapacitySourceOwnerGeneracion;
    telemetry.parentCapacityParentRatio =
        AdaptParentCapacityParentRatio;
    telemetry.parentCapacityStartContinuityEpoch =
        AdaptParentCapacityContinuidadEpoch;
    telemetry.parentCapacityStartResetEpoch =
        AdaptParentCapacityResetEpoch;
    telemetry.parentCapacityStartHintSeenEpoch =
        AdaptParentCapacityHintSeenEpoch;
    telemetry.parentCapacityStartConsumed =
        AdaptParentCapacityConsInicio;
    telemetry.parentCapacityStartTicks =
        AdaptParentCapacityTicksInicio;
    telemetry.parentCapacityStartConsumedProduced =
        AdaptParentCapacityFramesConsumoInicio;
    telemetry.parentCapacityStartPublishedProduced =
        AdaptParentCapacityFramesPublicadosInicio;
    telemetry.parentCapacityStartLevelPostConsumption =
        static_cast<u32>(std::max(
            0, AdaptParentCapacityNivelConsumoInicio));
    telemetry.parentCapacityStartLevelPostWrite =
        static_cast<u32>(std::max(
            0, AdaptParentCapacityNivelEscrituraInicio));
    telemetry.logicalLevelPostConsumption = nivelPostConsumoLogico;
    telemetry.logicalLevelBeforeWrite = nivelAntesEscrituraLogico;
    telemetry.logicalLevelPostWrite = nivelPostEscrituraLogico;
    telemetry.spillFrames = OutputSpillFrames;
    telemetry.spillPeakFrames = OutputSpillPeakFrames;
    telemetry.spillActiveNodes = static_cast<u32>(std::min<u64>(
        OutputSpillActiveNodes,
        std::numeric_limits<u32>::max()));
    telemetry.spillFreeNodes = static_cast<u32>(std::min<u64>(
        OutputSpillFreeNodes,
        std::numeric_limits<u32>::max()));
    telemetry.spillGeneration = OutputSpillGeneration;
    telemetry.spillBirthCount = OutputSpillBirthCount;
    if (OutputSpillBirthThisWrite)
    {
        telemetry.lastSpillBirthUpdateId = telemetry.updateId;
        telemetry.lastSpillBirthContinuityEpoch =
            OutputSpillLastBirthContinuityEpoch;
        telemetry.lastSpillBirthResetEpoch =
            OutputSpillLastBirthResetEpoch;
        telemetry.lastSpillBirthOwnerGeneration =
            OutputSpillLastBirthOwnerGeneration;
        telemetry.lastSpillBirthFrames = OutputSpillLastBirthFrames;
    }
    telemetry.spillPublicationCount = OutputSpillPublicationCount;
    telemetry.spillFramesPublishedTotal =
        OutputSpillFramesPublishedTotal;
    telemetry.spillAllocationCount = OutputSpillAllocationCount;
    telemetry.spillAllocationFailureCount =
        OutputSpillAllocationFailureCount;
    telemetry.spillDroppedPacketsTotal = OutputSpillDroppedPacketsTotal;
    telemetry.spillDroppedFramesTotal = OutputSpillDroppedFramesTotal;
    telemetry.spillDroppedPacketsThisWrite =
        spillDroppedPacketsEstaEscritura;
    telemetry.spillDroppedFramesThisWrite =
        spillDroppedFramesEstaEscritura;
    telemetry.ticksPublishedTotal = AdaptTicksTotal;
    telemetry.fastHighEscrowInitialized = modoDrenado == 2
        && AdaptSostenidoFastHighEscrowInicializado;
    telemetry.fastHighEscrowActive = modoDrenado == 2
        && AdaptSostenidoFastHighEscrowActivo;
    telemetry.fastHighEscrowReferenceIsOverride =
        AdaptSostenidoFastHighEscrowReferenciaEsOverride;
    telemetry.fastHighEscrowOwnerGeneration =
        AdaptSostenidoFastHighEscrowOwnerGeneracion;
    telemetry.fastHighEscrowReferenceTicks =
        AdaptSostenidoFastHighEscrowReferenciaTicks;
    telemetry.fastHighEscrowReferenceConsumed =
        AdaptSostenidoFastHighEscrowReferenciaCons;
    telemetry.fastHighEscrowRate = AdaptSostenidoFastHighEscrowRate;
    telemetry.fastHighEscrowGuardSkew =
        AdaptSostenidoFastHighEscrowGuardSkew;
    telemetry.fastHighEscrowHorizonFrames =
        AdaptSostenidoFastHighEscrowHorizonteRestanteFrames;
    telemetry.fastHighEscrowStartTicks =
        AdaptSostenidoFastHighEscrowTicksInicio;
    telemetry.fastHighEscrowStartPublishedProduced =
        AdaptSostenidoFastHighEscrowFramesPublicadosInicio;
    telemetry.fastHighEscrowStartConsumed =
        AdaptSostenidoFastHighEscrowConsInicio;
    telemetry.fastHighEscrowStartConsumedProduced =
        AdaptSostenidoFastHighEscrowFramesConsumoInicio;
    telemetry.fastHighEscrowStartLevelPostConsumption =
        AdaptSostenidoFastHighEscrowNivelConsumoInicioLogico;
    telemetry.fastHighEscrowStartLevelPostWrite =
        AdaptSostenidoFastHighEscrowNivelEscrituraInicioLogico;
    telemetry.fastHighEscrowStartContinuityEpoch =
        AdaptSostenidoFastHighEscrowContinuidadEpoch;
    telemetry.fastHighEscrowStartResetEpoch =
        AdaptSostenidoFastHighEscrowResetEpoch;
    telemetry.fastHighEscrowStartHintSeenEpoch =
        AdaptSostenidoFastHighEscrowHintSeenEpoch;
    telemetry.fastHighEscrowGrantFrames =
        AdaptSostenidoFastHighEscrowGrantFrames;
    telemetry.fastHighEscrowSpentFrames =
        AdaptSostenidoFastHighEscrowSpentFrames;
    telemetry.fastHighEscrowRemainingFrames =
        AdaptSostenidoFastHighEscrowGrantFrames
            >= AdaptSostenidoFastHighEscrowSpentFrames
        ? AdaptSostenidoFastHighEscrowGrantFrames
            - AdaptSostenidoFastHighEscrowSpentFrames
        : 0;
    telemetry.sustainedEpisodeOriginUpdateId =
        AdaptSostenidoEpisodioOrigenUpdateId;
    telemetry.sustainedParentRatio =
        modoDrenado == 2 && AdaptSostenidoParentValido
        ? AdaptSostenidoParentRatio : 0.0;
    telemetry.sustainedParentDeltaConsumed =
        modoDrenado == 2 && AdaptSostenidoParentValido
        ? AdaptSostenidoParentCons : 0;
    telemetry.sustainedParentDeltaTicks =
        modoDrenado == 2 && AdaptSostenidoParentValido
        ? AdaptSostenidoParentTicks : 0;
    telemetry.sustainedRecoveryBoundaryFrames =
        modoDrenado == 2 && (AdaptSostenidoOwnerActivo
                            || AdaptSostenidoCandidatoReemplazo)
        ? static_cast<u32>(std::max(
              0, AdaptSostenidoPhaseObjetivoFrames)) : 0;
    telemetry.effectivePhaseTargetFrames =
        modoDrenado == 2 && AdaptPhaseObjetivoActivo
        ? static_cast<u32>(std::max(0, AdaptPhaseObjetivoFrames)) : 0;

    producerObservation.packetId = ++OutputProvenancePacketId;
    producerObservation.updateId = telemetry.updateId;
    producerObservation.ticksBefore = AdaptTicksTotal - ticksTerminados;
    producerObservation.ticksAfter = AdaptTicksTotal;
    producerObservation.sourceProducedFramesBefore =
        AdaptFramesProducidosTotal - static_cast<u64>(avail);
    producerObservation.sourceProducedFramesAfter =
        AdaptFramesProducidosTotal;
    producerObservation.sourceFrames = static_cast<u32>(avail);
    producerObservation.acceptedSourcePrefixFrames = sourceFrame;
    producerObservation.droppedFrames =
        spillDroppedFramesEstaEscritura;
    producerObservation.droppedPackets =
        spillDroppedPacketsEstaEscritura;
    producerObservation.transportAcceptedFramesBefore =
        provenanceAcceptedBefore;
    producerObservation.transportAcceptedFramesAfter =
        OutputProvenanceAcceptedFrames;
    producerObservation.transportRemovedFrames =
        OutputProvenanceRemovedFrames;
    producerObservation.transportRealDrainedFrames =
        OutputProvenanceRealDrainedFrames;
    producerObservation.transportDiscardedFrames =
        OutputProvenanceDiscardedFrames;
    producerObservation.transportLineageEpoch =
        OutputProvenanceLineageEpoch;
    producerObservation.hostConsumed = hostConsumed;
    producerObservation.ticksAtConsumption = ticksAlConsumo;
    producerObservation.framesAtConsumption = framesAlConsumo;
    producerObservation.physicalLevelBeforeWrite =
        static_cast<u64>(std::max(0, nivelAntesEscritura));
    producerObservation.physicalLevelPostWrite =
        static_cast<u64>(std::max(0, nivelPostEscritura));
    producerObservation.logicalLevelBeforeWrite =
        nivelAntesEscrituraLogico;
    producerObservation.logicalLevelPostWrite =
        nivelPostEscrituraLogico;
    producerObservation.continuityEpoch = continuidadEpoch;
    producerObservation.resetRequestedEpoch = resetSolicitado;
    producerObservation.resetConfirmedEpoch = resetConfirmado;
    producerObservation.hintSeenEpoch = AdaptHintVistoProductor;
    producerObservation.sustainedCandidateGeneration =
        AdaptSostenidoCandidatoGeneracion;
    producerObservation.sustainedOwnerGeneration =
        AdaptSostenidoOwnerGeneracion;
    producerObservation.controllerRatio = AdaptRatio;
    producerObservation.desiredSkew = AdaptSkew;
    producerObservation.sourceAppliedSkew = sourceAppliedSkew;
    producerObservation.appliedSkew =
        OutputSkewPublicado.load(std::memory_order_relaxed);
    producerObservation.sustainedProvisionalPhaseActive =
        telemetry.sustainedProvisionalPhaseActive;
    producerObservation.sustainedProvisionalPhaseCandidateGeneration =
        telemetry.sustainedProvisionalPhaseCandidateGeneration;
    producerObservation.sustainedProvisionalPhaseOwnerGeneration =
        telemetry.sustainedProvisionalPhaseOwnerGeneration;
    producerObservation.sustainedProvisionalPhaseParentRatio =
        telemetry.sustainedProvisionalPhaseParentRatio;
    producerObservation.sustainedProvisionalPhaseSkew =
        telemetry.sustainedProvisionalPhaseSkew;
    producerObservation.sustainedProvisionalPhaseFrontierFrames =
        telemetry.sustainedProvisionalPhaseFrontierFrames;
    producerObservation.sustainedProvisionalPhaseBackingFrames =
        telemetry.sustainedProvisionalPhaseBackingFrames;
    producerObservation.sustainedProvisionalPhasePublicationReserveFrames =
        telemetry.sustainedProvisionalPhasePublicationReserveFrames;
    producerObservation.sustainedProvisionalPhaseReservedPublicationCount =
        telemetry.sustainedProvisionalPhaseReservedPublicationCount;
    producerObservation.sustainedProvisionalPhaseBirthCount =
        telemetry.sustainedProvisionalPhaseBirthCount;
    producerObservation.lastSustainedProvisionalPhaseBirthUpdateId =
        telemetry.lastSustainedProvisionalPhaseBirthUpdateId;
    producerObservation.sustainedProvisionalPhaseStepCount =
        telemetry.sustainedProvisionalPhaseStepCount;
    producerObservation.lastSustainedProvisionalPhaseStepUpdateId =
        telemetry.lastSustainedProvisionalPhaseStepUpdateId;
    producerObservation.lastSustainedProvisionalPhaseStepPreviousSkew =
        telemetry.lastSustainedProvisionalPhaseStepPreviousSkew;
    producerObservation.lastSustainedProvisionalPhaseStepSkew =
        telemetry.lastSustainedProvisionalPhaseStepSkew;
    producerObservation.sustainedProvisionalPhaseClearCount =
        telemetry.sustainedProvisionalPhaseClearCount;
    producerObservation.lastSustainedProvisionalPhaseClearUpdateId =
        telemetry.lastSustainedProvisionalPhaseClearUpdateId;
    producerObservation.lastSustainedProvisionalPhaseClearReasons =
        telemetry.lastSustainedProvisionalPhaseClearReasons;
    producerObservation.sustainedProvisionalPhaseApplicationConflictCount =
        telemetry.sustainedProvisionalPhaseApplicationConflictCount;
    producerObservation.lastSustainedProvisionalPhaseApplicationConflictUpdateId =
        telemetry.lastSustainedProvisionalPhaseApplicationConflictUpdateId;

    Platform::Mutex_Unlock(AudioLock);

    telemetry.primingFrames =
        AdaptPrimingFrames.load(std::memory_order_relaxed);
    telemetry.controllerRatio = AdaptRatio;
    telemetry.desiredSkew =
        AdaptSkewDeseado.load(std::memory_order_relaxed);

    telemetry.appliedSkew =
        OutputSkewPublicado.load(std::memory_order_relaxed);
    telemetry.speedHint = AdaptHint;
    telemetry.droppedBlocks = AdaptDescartes.load(std::memory_order_relaxed);
    telemetry.resetSeenEpoch = AdaptResetVistoProductor;
    telemetry.hintEpoch = AdaptHintEpoch.load(std::memory_order_acquire);
    telemetry.hintSeenEpoch = AdaptHintVistoProductor;

    if (dropsEstaEscritura > 0)
    {
        telemetry.lastDropUpdateId = telemetry.updateId;
        telemetry.lastDropLevelBeforeWrite = telemetry.levelBeforeWrite;
        telemetry.lastDropLevelPostWrite = telemetry.levelPostWrite;
        telemetry.lastDropFramesProduced = telemetry.framesProduced;
        telemetry.lastDropDropsThisWrite = dropsEstaEscritura;
    }

    AdaptTelemetryMailbox.Publish(telemetry);
    if (OutputObservationSink)
    {
        OutputObservationSink->OnAudioOutputProducerPacket(
            temp, static_cast<u32>(avail), producerObservation);
    }
}

void SPU::TrimOutput()
{
    Platform::Mutex_Lock(AudioLock);
    const u64 halflimit = OutputBufferSize / 2;
    const u64 level = OutputLogicalLevelLocked();
    if (level > halflimit)
        DiscardOutputQueueLocked(level - halflimit);
    Platform::Mutex_Unlock(AudioLock);
}

void SPU::DrainOutput()
{
    Platform::Mutex_Lock(AudioLock);
    ClearOutputQueueLocked();
    Platform::Mutex_Unlock(AudioLock);
}

void SPU::DrainAndResetOutputAdaptivo()
{
    Platform::Mutex_Lock(AudioLock);

    ClearOutputQueueLocked();
    AdaptReingresoPrimingReserva.store(0, std::memory_order_release);
    AdaptResetEpoch.fetch_add(1, std::memory_order_release);
    Platform::Mutex_Unlock(AudioLock);
}

void SPU::InitOutput()
{
    Platform::Mutex_Lock(AudioLock);

    blip_set_rates(BlipLeft, INTERNAL_SAMPLE_RATE * OutputSkew, OutputSampleRate);
    blip_set_rates(BlipRight, INTERNAL_SAMPLE_RATE * OutputSkew, OutputSampleRate);

    u32 needSamples = (u32) ceil(INTERNAL_SAMPLE_RATE / 60 / INTERNAL_SAMPLE_RATE * OutputSampleRate);
    u32 newBufferSize = 512;
    while (newBufferSize < needSamples)
        newBufferSize <<= 1;
    newBufferSize <<= 1;
    newBufferSize <<= 2;

    newBufferSize <<= 1;

    while (newBufferSize <= 4u * (u32)blip_max_frame)
        newBufferSize <<= 1;

    if (newBufferSize != OutputBufferSize)
    {
        if (OutputBuffer != nullptr)
            free(OutputBuffer);
        OutputBuffer = (s16*) malloc(2 * newBufferSize * 2);
        OutputBufferSize = newBufferSize;
    }

    memset(OutputBuffer, 0, 2*OutputBufferSize*2);
    ClearOutputQueueLocked();
    Platform::Mutex_Unlock(AudioLock);
}

int SPU::GetOutputSize() const
{
    Platform::Mutex_Lock(AudioLock);

    const u64 level = OutputLogicalLevelLocked();
    const int ret = level > static_cast<u64>(std::numeric_limits<int>::max())
        ? std::numeric_limits<int>::max()
        : static_cast<int>(level);

    Platform::Mutex_Unlock(AudioLock);
    return ret;
}

void SPU::Sync(bool wait)
{
    // this function is currently not used anywhere
    // depending on the usage context the thread safety measures could be made
    // a lot faster

    // sync to audio output in case the core is running too fast
    // * wait=true: wait until enough audio data has been played
    // * wait=false: merely skip some audio data to avoid a FIFO overflow

    const int halflimit = (OutputBufferSize / 2);

    if (wait)
    {
        // TODO: less CPU-intensive wait?
        while (GetOutputSize() > halflimit);
    }
    else if (GetOutputSize() > halflimit)
    {
        Platform::Mutex_Lock(AudioLock);
        const u64 level = OutputLogicalLevelLocked();
        if (level > static_cast<u64>(halflimit))
            DiscardOutputQueueLocked(level - halflimit);
        Platform::Mutex_Unlock(AudioLock);
    }
}

int SPU::ReadOutput(s16* data, int samples)
{

    u32 modoEsperado = 0;
    AdaptModoDrenado.compare_exchange_strong(
        modoEsperado, 1, std::memory_order_acq_rel,
        std::memory_order_acquire);
    if (modoEsperado == 2)
        return 0;

    Platform::Mutex_Lock(AudioLock);
    if (OutputLogicalLevelLocked() == 0)
    {
        Platform::Mutex_Unlock(AudioLock);
        return 0;
    }

    const int read = ReadOutputQueueLocked(data, samples);
    Platform::Mutex_Unlock(AudioLock);
    return read;
}

int SPU::ReadOutputAdaptivo(
    s16* data, int samples, AudioOutputDrainObservation* observation)
{
    if (observation)
    {
        *observation = {};
        observation->valid = true;
        observation->requestedFrames = samples > 0
            ? static_cast<u32>(samples) : 0;
    }
    if (samples <= 0)
        return 0;

    Platform::Mutex_Lock(AudioLock);
    const int capacidad = (int)OutputBufferSize;
    Platform::Mutex_Unlock(AudioLock);
    const OutputAdaptiveGeometry geometry =
        GetOutputAdaptiveGeometry((u32)capacidad);
    if (capacidad > 1 && samples >= capacidad)
    {
        AudioOutputDrainObservation aggregate {};
        aggregate.valid = true;
        aggregate.requestedFrames = static_cast<u32>(samples);
        bool first = true;
        const int trozoMaximo = (int)geometry.target;
        int total = 0;
        while (total < samples)
        {
            const int restantes = samples - total;
            const int trozo = restantes > trozoMaximo
                ? trozoMaximo : restantes;
            AudioOutputDrainObservation child {};
            const int leidos = ReadOutputAdaptivo(
                data + ((size_t)total * 2), trozo,
                observation ? &child : nullptr);
            if (observation)
            {
                if (first)
                {
                    aggregate.hostConsumedBefore = child.hostConsumedBefore;
                    aggregate.physicalLevelBefore = child.physicalLevelBefore;
                    aggregate.logicalLevelBefore = child.logicalLevelBefore;
                    aggregate.transportRemovedFramesBefore =
                        child.transportRemovedFramesBefore;
                    aggregate.transportRealDrainedFramesBefore =
                        child.transportRealDrainedFramesBefore;
                    aggregate.transportDiscardedFramesBefore =
                        child.transportDiscardedFramesBefore;
                    aggregate.transportLineageEpochBefore =
                        child.transportLineageEpochBefore;
                    aggregate.continuityEpochBefore =
                        child.continuityEpochBefore;
                    aggregate.resetRequestedEpochBefore =
                        child.resetRequestedEpochBefore;
                    aggregate.resetSeenEpochBefore =
                        child.resetSeenEpochBefore;
                    aggregate.resetConfirmedEpochBefore =
                        child.resetConfirmedEpochBefore;
                    aggregate.underrunsBefore = child.underrunsBefore;
                    aggregate.primingFramesBefore =
                        child.primingFramesBefore;
                    first = false;
                }
                aggregate.returnedFrames += child.returnedFrames;
                aggregate.inactiveZeroFrames += child.inactiveZeroFrames;
                aggregate.primingZeroFrames += child.primingZeroFrames;
                aggregate.realRampInFrames += child.realRampInFrames;
                aggregate.realUnmodifiedFrames += child.realUnmodifiedFrames;
                aggregate.underrunRampOutFrames +=
                    child.underrunRampOutFrames;
                aggregate.underrunZeroFrames += child.underrunZeroFrames;
                aggregate.unclassifiedFrames += child.unclassifiedFrames;
                aggregate.hostConsumedAfter = child.hostConsumedAfter;
                aggregate.ticksAtDemand = child.ticksAtDemand;
                aggregate.framesProducedAtDemand =
                    child.framesProducedAtDemand;
                aggregate.physicalLevelAfter = child.physicalLevelAfter;
                aggregate.logicalLevelAfter = child.logicalLevelAfter;
                aggregate.transportAcceptedFrames =
                    child.transportAcceptedFrames;
                aggregate.transportRemovedFramesAfter =
                    child.transportRemovedFramesAfter;
                aggregate.transportRealDrainedFramesAfter =
                    child.transportRealDrainedFramesAfter;
                aggregate.transportDiscardedFramesAfter =
                    child.transportDiscardedFramesAfter;
                aggregate.transportLineageEpochAfter =
                    child.transportLineageEpochAfter;
                aggregate.continuityEpochAfter =
                    child.continuityEpochAfter;
                aggregate.resetRequestedEpochAfter =
                    child.resetRequestedEpochAfter;
                aggregate.resetSeenEpochAfter = child.resetSeenEpochAfter;
                aggregate.resetConfirmedEpochAfter =
                    child.resetConfirmedEpochAfter;
                aggregate.underrunsAfter = child.underrunsAfter;
                aggregate.primingFramesAfter = child.primingFramesAfter;
            }
            if (leidos <= 0)
                break;
            total += leidos;
            if (leidos != trozo)
                break;
        }
        if (observation)
        {
            const u32 classified =
                AudioOutputDrainClassifiedFrames(aggregate);
            if (classified < static_cast<u32>(samples))
                aggregate.unclassifiedFrames +=
                    static_cast<u32>(samples) - classified;
            *observation = aggregate;
        }
        return total;
    }

    const u32 modoActual = AdaptModoDrenado.load(std::memory_order_acquire);
    if (modoActual == 1)
    {
        if (observation)
            observation->unclassifiedFrames = static_cast<u32>(samples);
        return 0;
    }

    const bool activar = modoActual == 0;
    if (activar)
    {

        ResetOutputAdaptivo();
    }
    int objetivo;
    int nivelCebado;
    int nivel;
    u64 nivelLogico;
    int leidos = 0;

    Platform::Mutex_Lock(AudioLock);
    const u32 resetEpoch = AdaptResetEpoch.load(std::memory_order_acquire);
    if (observation)
    {
        observation->hostConsumedBefore = AdaptHostConsumedTotal;
        observation->physicalLevelBefore = OutputRingLevelLocked();
        observation->logicalLevelBefore = OutputLogicalLevelLocked();
        observation->transportAcceptedFrames =
            OutputProvenanceAcceptedFrames;
        observation->transportRemovedFramesBefore =
            OutputProvenanceRemovedFrames;
        observation->transportRealDrainedFramesBefore =
            OutputProvenanceRealDrainedFrames;
        observation->transportDiscardedFramesBefore =
            OutputProvenanceDiscardedFrames;
        observation->transportLineageEpochBefore =
            OutputProvenanceLineageEpoch;
        observation->continuityEpochBefore = AdaptContinuidadEpoch;
        observation->resetRequestedEpochBefore = resetEpoch;
        observation->resetSeenEpochBefore = AdaptResetVistoConsumidor;
        observation->resetConfirmedEpochBefore = AdaptResetConfirmado;
        observation->underrunsBefore =
            AdaptUnderruns.load(std::memory_order_relaxed);
        observation->primingFramesBefore =
            AdaptPrimingFrames.load(std::memory_order_relaxed);
    }
    if (resetEpoch != AdaptResetVistoConsumidor)
    {
        AdaptResetVistoConsumidor = resetEpoch;
        AdaptPriming = true;
        AdaptEnHueco = true;
        AdaptUltima[0] = 0;
        AdaptUltima[1] = 0;
    }
    objetivo = (int)geometry.target;
    const u64 reservaReingreso =
        AdaptReingresoPrimingReserva.load(std::memory_order_acquire);
    const u32 reservaResetEpoch = (u32)(reservaReingreso >> 32);
    const u32 reservaTarget = (u32)reservaReingreso;
    if (reservaTarget > 0 && reservaResetEpoch == resetEpoch)
    {

        objetivo = std::min((int)reservaTarget, (int)geometry.safe);
    }
    nivelCebado = samples > objetivo ? samples : objetivo;
    if (nivelCebado >= (int)OutputBufferSize)
        nivelCebado = (int)OutputBufferSize - 1;
    nivel = static_cast<int>(OutputRingLevelLocked());
    nivelLogico = OutputLogicalLevelLocked();

    AdaptHostConsumedTotal += (u64)samples;
    AdaptTicksAlConsumo = AdaptTicksPublicados.load(std::memory_order_relaxed);
    AdaptFramesAlConsumo = AdaptFramesProducidosTotal;
    if (observation)
    {
        observation->hostConsumedAfter = AdaptHostConsumedTotal;
        observation->ticksAtDemand = AdaptTicksAlConsumo;
        observation->framesProducedAtDemand = AdaptFramesAlConsumo;
    }
    if (activar)
    {
        u32 modoEsperado = 0;
        const bool activado = AdaptModoDrenado.compare_exchange_strong(
            modoEsperado, 2, std::memory_order_release,
            std::memory_order_acquire);
        if (!activado && modoEsperado != 2)
        {
            if (observation)
            {
                observation->unclassifiedFrames = static_cast<u32>(samples);
                observation->hostConsumedAfter = AdaptHostConsumedTotal;
            }
            Platform::Mutex_Unlock(AudioLock);
            return 0;
        }
    }

    auto finishObservationLocked = [&]() {
        if (!observation)
            return;
        observation->physicalLevelAfter = OutputRingLevelLocked();
        observation->logicalLevelAfter = OutputLogicalLevelLocked();
        observation->transportAcceptedFrames =
            OutputProvenanceAcceptedFrames;
        observation->transportRemovedFramesAfter =
            OutputProvenanceRemovedFrames;
        observation->transportRealDrainedFramesAfter =
            OutputProvenanceRealDrainedFrames;
        observation->transportDiscardedFramesAfter =
            OutputProvenanceDiscardedFrames;
        observation->transportLineageEpochAfter =
            OutputProvenanceLineageEpoch;
        observation->continuityEpochAfter = AdaptContinuidadEpoch;
        observation->resetRequestedEpochAfter =
            AdaptResetEpoch.load(std::memory_order_acquire);
        observation->resetSeenEpochAfter = AdaptResetVistoConsumidor;
        observation->resetConfirmedEpochAfter = AdaptResetConfirmado;
        observation->underrunsAfter =
            AdaptUnderruns.load(std::memory_order_relaxed);
        observation->primingFramesAfter =
            AdaptPrimingFrames.load(std::memory_order_relaxed);
    };

    if (AdaptPriming && nivelLogico < static_cast<u64>(nivelCebado))
    {
        AdaptNivelPostConsumo = nivel;
        AdaptNivelPostConsumoLogico = nivelLogico;
        finishObservationLocked();
        Platform::Mutex_Unlock(AudioLock);
        memset(data, 0, (size_t)samples * sizeof(s16) * 2);
        const u64 primingBefore = AdaptPrimingFrames.fetch_add(
            (u64)samples, std::memory_order_relaxed);
        if (observation)
        {
            observation->primingZeroFrames = static_cast<u32>(samples);
            observation->returnedFrames = static_cast<u32>(samples);
            observation->primingFramesBefore = primingBefore;
            observation->primingFramesAfter =
                primingBefore + static_cast<u64>(samples);
        }
        return samples;
    }

    if (AdaptPriming)
    {
        const u32 resetAntesDrenar =
            AdaptResetEpoch.load(std::memory_order_acquire);
        if (resetAntesDrenar != AdaptResetVistoConsumidor)
        {
            AdaptResetVistoConsumidor = resetAntesDrenar;
            AdaptEnHueco = true;
            AdaptUltima[0] = 0;
            AdaptUltima[1] = 0;
            AdaptNivelPostConsumo = nivel;
            AdaptNivelPostConsumoLogico = nivelLogico;
            finishObservationLocked();
            Platform::Mutex_Unlock(AudioLock);
            memset(data, 0, (size_t)samples * sizeof(s16) * 2);
            const u64 primingBefore = AdaptPrimingFrames.fetch_add(
                (u64)samples, std::memory_order_relaxed);
            if (observation)
            {
                observation->primingZeroFrames = static_cast<u32>(samples);
                observation->returnedFrames = static_cast<u32>(samples);
                observation->primingFramesBefore = primingBefore;
                observation->primingFramesAfter =
                    primingBefore + static_cast<u64>(samples);
                observation->resetSeenEpochAfter = resetAntesDrenar;
            }
            return samples;
        }
    }

    const bool reingresando = AdaptPriming;
    leidos = ReadOutputQueueLocked(data, samples);
    AdaptNivelPostConsumo = static_cast<int>(OutputRingLevelLocked());
    AdaptNivelPostConsumoLogico = OutputLogicalLevelLocked();

    if (reingresando && leidos == samples)
    {

        AdaptPriming = false;
        AdaptEnHueco = true;
        AdaptResetConfirmado = AdaptResetVistoConsumidor;
        ++AdaptContinuidadEpoch;
    }
    else if (!reingresando && leidos < samples)
    {

        AdaptPriming = true;
        ++AdaptContinuidadEpoch;
        AdaptUnderruns.fetch_add(1, std::memory_order_relaxed);
    }
    const bool aplicarRampaEntrada = leidos > 0 && AdaptEnHueco;
    finishObservationLocked();
    Platform::Mutex_Unlock(AudioLock);

    int rampaEntrada = 0;
    if (leidos > 0)
    {
        if (aplicarRampaEntrada)
        {
            rampaEntrada = leidos < 64 ? leidos : 64;
            for (int i = 0; i < rampaEntrada; i++)
            {
                data[i*2] = (s16)((s32)data[i*2] * (i + 1) / rampaEntrada);
                data[i*2+1] = (s16)((s32)data[i*2+1] * (i + 1) / rampaEntrada);
            }
            AdaptEnHueco = false;
        }
        AdaptUltima[0] = data[(leidos-1)*2];
        AdaptUltima[1] = data[(leidos-1)*2+1];
    }
    if (observation)
    {
        observation->realRampInFrames = static_cast<u32>(rampaEntrada);
        observation->realUnmodifiedFrames =
            static_cast<u32>(leidos - rampaEntrada);
    }

    if (leidos < samples)
    {
        s32 l = AdaptEnHueco ? 0 : AdaptUltima[0];
        s32 r = AdaptEnHueco ? 0 : AdaptUltima[1];
        const int falta = samples - leidos;
        const int rampa = falta < 64 ? falta : 64;
        for (int i = 0; i < falta; i++)
        {
            if (i < rampa)
            {
                l = (l * (rampa - 1 - i)) / rampa;
                r = (r * (rampa - 1 - i)) / rampa;
            }
            else
            {
                l = 0;
                r = 0;
            }
            data[(leidos + i)*2] = (s16)l;
            data[(leidos + i)*2+1] = (s16)r;
        }
        AdaptEnHueco = true;
        AdaptUltima[0] = 0;
        AdaptUltima[1] = 0;
        if (observation)
        {
            observation->underrunRampOutFrames = static_cast<u32>(rampa);
            observation->underrunZeroFrames =
                static_cast<u32>(falta - rampa);
            observation->returnedFrames = static_cast<u32>(samples);
        }
        return samples;
    }

    if (observation)
        observation->returnedFrames = static_cast<u32>(leidos);
    return leidos;
}

void SPU::UpdateOutputAdaptivo(u64 hostConsumed, u64 ticksAlConsumo,
                               u64 framesAlConsumo,
                               int nivelPostConsumo, int nivelPostEscritura,
                               u64 nivelPostConsumoLogico,
                               u64 nivelPostEscrituraLogico,
                               u32 resetSolicitado, u32 resetConfirmado,
                               bool priming, u64 continuidadEpoch,
                               u32 dropsEstaEscritura,
                               u32 underrunsActuales,
                               AudioOutputAdaptiveTelemetrySnapshot& telemetry)
{
    if (OutputBufferSize == 0)
        return;

    u32 decisionFlags = AudioOutputAdaptiveMode
        | AudioOutputAdaptiveControllerRan;

    auto provisionalFastStateActual = [&]() -> u32 {
        return (AdaptFastLowPendiente
                ? AudioOutputAdaptiveProvisionalFastLowPending : 0u)
             | (AdaptFastCambioTargetPendiente
                ? AudioOutputAdaptiveProvisionalFastTargetChangePending : 0u);
    };
    const u32 provisionalFastStateAntes = provisionalFastStateActual();
    const u64 provisionalFastAnchorConsAntes = AdaptFastConsInicio;
    const u64 provisionalFastAnchorTicksAntes = AdaptFastTicksInicio;
    const u64 provisionalFastAnchorFramesAntes = AdaptFastFramesInicio;
    const u64 provisionalFastAnchorFramesPublicadosAntes =
        AdaptFastFramesPublicadosInicio;
    const u64 provisionalFastAnchorNivelConsAntes =
        AdaptFastNivelInicioLogico;
    const u64 provisionalFastAnchorNivelEscrituraAntes =
        AdaptFastNivelEscrituraInicioLogico;
    const u64 provisionalFastAnchorContinuidadAntes =
        AdaptFastContinuidadEpochInicio;
    const u32 provisionalFastAnchorResetAntes =
        AdaptFastResetEpochInicio;
    const u32 provisionalFastAnchorHintAntes =
        AdaptFastHintSeenEpochInicio;
    const u64 provisionalFastAnchorCandidatoAntes =
        AdaptFastCandidatoGeneracionInicio;
    const u64 provisionalFastAnchorOwnerAntes =
        AdaptFastAnchorOwnerGeneracion;
    const bool provisionalCandidatoActivoAntes =
        AdaptSostenidoCandidatoActivo;
    const u64 provisionalCandidatoGeneracionAntes =
        AdaptSostenidoCandidatoGeneracion;
    const u64 provisionalCandidatoConsAntes = AdaptSostenidoConsInicio;
    const u64 provisionalCandidatoTicksAntes = AdaptSostenidoTicksInicio;
    const u64 provisionalCandidatoFramesAntes = AdaptSostenidoFramesInicio;
    const u64 provisionalCandidatoFramesPublicadosAntes =
        AdaptSostenidoFramesPublicadosInicio;
    const u64 provisionalCandidatoNivelConsAntes =
        AdaptSostenidoNivelInicioLogico;
    const u64 provisionalCandidatoNivelEscrituraAntes =
        AdaptSostenidoNivelEscrituraInicioLogico;
    const u64 provisionalCandidatoContinuidadAntes =
        AdaptSostenidoContinuidadEpochInicio;
    const u32 provisionalCandidatoResetAntes =
        AdaptSostenidoResetEpochInicio;
    const u32 provisionalCandidatoHintAntes =
        AdaptSostenidoHintSeenEpochInicio;
    const int provisionalCandidatoDireccionAntes = AdaptSostenidoDireccion;
    const u64 provisionalObservacionesAntes =
        telemetry.sustainedObservationCount;
    const u64 provisionalTransicionesAntes =
        telemetry.sustainedTransitionCount;
    const u64 provisionalRateRecoveryAntes =
        telemetry.rateRecoveryRebaseCount;
    const u64 provisionalContinuidadVistaAntes =
        AdaptContinuidadVistoProductor;
    const u32 provisionalResetVistoAntes = AdaptResetVistoProductor;
    const u32 provisionalHintVistoAntes = AdaptHintVistoProductor;

    bool provisionalPhaseIntervalInfeasibleEsteUpdate = false;

    auto publicarEventoProvisional = [&]() {
        const u32 fastStateDespues = provisionalFastStateActual();
        u32 eventFlags = 0;
        if (fastStateDespues != provisionalFastStateAntes)
        {
            eventFlags |=
                AudioOutputAdaptiveProvisionalFastStateChanged;
        }
        const bool fastAnchorCambio =
            AdaptFastConsInicio != provisionalFastAnchorConsAntes
            || AdaptFastTicksInicio != provisionalFastAnchorTicksAntes
            || AdaptFastFramesInicio != provisionalFastAnchorFramesAntes
            || AdaptFastFramesPublicadosInicio
               != provisionalFastAnchorFramesPublicadosAntes
            || AdaptFastNivelInicioLogico
               != provisionalFastAnchorNivelConsAntes
            || AdaptFastNivelEscrituraInicioLogico
               != provisionalFastAnchorNivelEscrituraAntes
            || AdaptFastContinuidadEpochInicio
               != provisionalFastAnchorContinuidadAntes
            || AdaptFastResetEpochInicio != provisionalFastAnchorResetAntes
            || AdaptFastHintSeenEpochInicio != provisionalFastAnchorHintAntes
            || AdaptFastCandidatoGeneracionInicio
               != provisionalFastAnchorCandidatoAntes
            || AdaptFastAnchorOwnerGeneracion
               != provisionalFastAnchorOwnerAntes;
        if (fastAnchorCambio)
        {
            eventFlags |=
                AudioOutputAdaptiveProvisionalFastAnchorChanged;
        }
        const bool candidatoNacido = AdaptSostenidoCandidatoGeneracion
            != provisionalCandidatoGeneracionAntes;
        const bool candidatoLimpiado = provisionalCandidatoActivoAntes
            && (!AdaptSostenidoCandidatoActivo || candidatoNacido);
        const bool candidatoReemplazado = candidatoNacido
            && candidatoLimpiado;
        if (candidatoNacido)
        {
            eventFlags |=
                AudioOutputAdaptiveProvisionalCandidateBorn;
        }
        if (candidatoLimpiado)
        {
            eventFlags |=
                AudioOutputAdaptiveProvisionalCandidateCleared;
        }
        if (telemetry.sustainedObservationCount
            != provisionalObservacionesAntes)
        {
            eventFlags |=
                AudioOutputAdaptiveProvisionalCandidateObserved;
        }
        if ((decisionFlags
             & AudioOutputAdaptiveFastAccepted) != 0)
        {
            eventFlags |= AudioOutputAdaptiveProvisionalFastAccepted;
        }
        if (telemetry.sustainedTransitionCount
            != provisionalTransicionesAntes)
        {
            eventFlags |=
                AudioOutputAdaptiveProvisionalSustainedTransition;
        }
        if (telemetry.rateRecoveryRebaseCount
            != provisionalRateRecoveryAntes)
        {
            eventFlags |= AudioOutputAdaptiveProvisionalRateRecovery;
        }
        if (AdaptResetVistoProductor != provisionalResetVistoAntes)
            eventFlags |= AudioOutputAdaptiveProvisionalResetChanged;
        if (AdaptContinuidadVistoProductor
            != provisionalContinuidadVistaAntes)
        {
            eventFlags |=
                AudioOutputAdaptiveProvisionalContinuityChanged;
        }
        if (AdaptHintVistoProductor != provisionalHintVistoAntes)
            eventFlags |= AudioOutputAdaptiveProvisionalHintChanged;
        if (dropsEstaEscritura > 0)
            eventFlags |= AudioOutputAdaptiveProvisionalDrop;
        if (provisionalPhaseIntervalInfeasibleEsteUpdate)
        {
            eventFlags |=
                AudioOutputAdaptiveProvisionalPhaseIntervalInfeasible;
        }

        if (eventFlags == 0)
            return;

        const bool usarCandidatoAntes = candidatoLimpiado
            && !candidatoNacido;
        ++telemetry.provisionalEventCount;
        telemetry.lastProvisionalEventUpdateId = telemetry.updateId;
        telemetry.lastProvisionalEventFlags = eventFlags;
        telemetry.lastProvisionalEventFastStateBefore =
            provisionalFastStateAntes;
        telemetry.lastProvisionalEventFastStateAfter = fastStateDespues;
        telemetry.lastProvisionalEventFastAnchorConsumed =
            provisionalFastAnchorConsAntes;
        telemetry.lastProvisionalEventFastAnchorTicks =
            provisionalFastAnchorTicksAntes;
        telemetry.lastProvisionalEventFastAnchorProduced =
            provisionalFastAnchorFramesAntes;
        telemetry.lastProvisionalEventFastAnchorPublishedProduced =
            provisionalFastAnchorFramesPublicadosAntes;
        telemetry.lastProvisionalEventFastAnchorLevelPostConsumption =
            provisionalFastAnchorNivelConsAntes;
        telemetry.lastProvisionalEventFastAnchorLevelPostWrite =
            provisionalFastAnchorNivelEscrituraAntes;
        telemetry.lastProvisionalEventFastAnchorContinuityEpoch =
            provisionalFastAnchorContinuidadAntes;
        telemetry.lastProvisionalEventFastAnchorResetEpoch =
            provisionalFastAnchorResetAntes;
        telemetry.lastProvisionalEventFastAnchorHintSeenEpoch =
            provisionalFastAnchorHintAntes;
        telemetry.lastProvisionalEventFastAnchorCandidateGeneration =
            provisionalFastAnchorCandidatoAntes;
        telemetry.lastProvisionalEventFastAnchorOwnerGeneration =
            provisionalFastAnchorOwnerAntes;
        telemetry.lastProvisionalEventCandidateGeneration =
            usarCandidatoAntes
            ? provisionalCandidatoGeneracionAntes
            : AdaptSostenidoCandidatoGeneracion;
        telemetry.lastProvisionalEventCandidateStartConsumed =
            usarCandidatoAntes
            ? provisionalCandidatoConsAntes : AdaptSostenidoConsInicio;
        telemetry.lastProvisionalEventCandidateStartTicks =
            usarCandidatoAntes
            ? provisionalCandidatoTicksAntes : AdaptSostenidoTicksInicio;
        telemetry.lastProvisionalEventCandidateStartProduced =
            usarCandidatoAntes
            ? provisionalCandidatoFramesAntes : AdaptSostenidoFramesInicio;
        telemetry.lastProvisionalEventCandidateStartPublishedProduced =
            usarCandidatoAntes
            ? provisionalCandidatoFramesPublicadosAntes
            : AdaptSostenidoFramesPublicadosInicio;
        telemetry.lastProvisionalEventCandidateStartLevelPostConsumption =
            usarCandidatoAntes
            ? provisionalCandidatoNivelConsAntes
            : AdaptSostenidoNivelInicioLogico;
        telemetry.lastProvisionalEventCandidateStartLevelPostWrite =
            usarCandidatoAntes
            ? provisionalCandidatoNivelEscrituraAntes
            : AdaptSostenidoNivelEscrituraInicioLogico;
        telemetry.lastProvisionalEventCandidateStartContinuityEpoch =
            usarCandidatoAntes
            ? provisionalCandidatoContinuidadAntes
            : AdaptSostenidoContinuidadEpochInicio;
        telemetry.lastProvisionalEventCandidateStartResetEpoch =
            usarCandidatoAntes
            ? provisionalCandidatoResetAntes
            : AdaptSostenidoResetEpochInicio;
        telemetry.lastProvisionalEventCandidateStartHintSeenEpoch =
            usarCandidatoAntes
            ? provisionalCandidatoHintAntes
            : AdaptSostenidoHintSeenEpochInicio;
        telemetry.lastProvisionalEventCandidateDirection =
            usarCandidatoAntes
            ? provisionalCandidatoDireccionAntes : AdaptSostenidoDireccion;
        telemetry.lastProvisionalEventOwnerGeneration =
            AdaptSostenidoOwnerGeneracion;
        telemetry.lastProvisionalEventEndpointConsumed = hostConsumed;
        telemetry.lastProvisionalEventEndpointTicks = ticksAlConsumo;
        telemetry.lastProvisionalEventEndpointProduced = framesAlConsumo;
        telemetry.lastProvisionalEventEndpointPublishedProduced =
            AdaptFramesProducidosTotal;
        telemetry.lastProvisionalEventEndpointLevelPostConsumption =
            nivelPostConsumoLogico;
        telemetry.lastProvisionalEventEndpointLevelPostWrite =
            nivelPostEscrituraLogico;
        telemetry.lastProvisionalEventEndpointContinuityEpoch =
            continuidadEpoch;
        telemetry.lastProvisionalEventResetEpoch = resetSolicitado;
        telemetry.lastProvisionalEventHintSeenEpoch =
            AdaptHintVistoProductor;
        telemetry.lastProvisionalEventDecisionFlags =
            decisionFlags;

        telemetry.lastProvisionalEventPreviousCandidateGeneration =
            candidatoReemplazado ? provisionalCandidatoGeneracionAntes : 0;
        telemetry.lastProvisionalEventPreviousCandidateStartConsumed =
            candidatoReemplazado ? provisionalCandidatoConsAntes : 0;
        telemetry.lastProvisionalEventPreviousCandidateStartTicks =
            candidatoReemplazado ? provisionalCandidatoTicksAntes : 0;
        telemetry.lastProvisionalEventPreviousCandidateStartProduced =
            candidatoReemplazado ? provisionalCandidatoFramesAntes : 0;
        telemetry.lastProvisionalEventPreviousCandidateStartPublishedProduced =
            candidatoReemplazado
            ? provisionalCandidatoFramesPublicadosAntes : 0;
        telemetry.lastProvisionalEventPreviousCandidateStartLevelPostConsumption =
            candidatoReemplazado ? provisionalCandidatoNivelConsAntes : 0;
        telemetry.lastProvisionalEventPreviousCandidateStartLevelPostWrite =
            candidatoReemplazado
            ? provisionalCandidatoNivelEscrituraAntes : 0;
        telemetry.lastProvisionalEventPreviousCandidateStartContinuityEpoch =
            candidatoReemplazado ? provisionalCandidatoContinuidadAntes : 0;
        telemetry.lastProvisionalEventPreviousCandidateStartResetEpoch =
            candidatoReemplazado ? provisionalCandidatoResetAntes : 0;
        telemetry.lastProvisionalEventPreviousCandidateStartHintSeenEpoch =
            candidatoReemplazado ? provisionalCandidatoHintAntes : 0;
        telemetry.lastProvisionalEventPreviousCandidateDirection =
            candidatoReemplazado ? provisionalCandidatoDireccionAntes : 0;
    };

    constexpr u64 maxTicksPublicacion = 512ULL * 128ULL + 1024ULL;
    const double minSkew =
        ((double)maxTicksPublicacion * OutputSampleRate) /
        ((double)INTERNAL_SAMPLE_RATE * (double)(blip_max_frame - 1));
    const double maxSkew =
        (OutputSampleRate * (double)blip_max_ratio) /
        (double)INTERNAL_SAMPLE_RATE;
    const OutputAdaptiveGeometry geometry =
        GetOutputAdaptiveGeometry(OutputBufferSize);
    const u64 bloque = geometry.block;
    const u64 ventanaRate = geometry.rateWindow;
    const double presupuestoRate = (double)geometry.rateWindow;
    const int objetivoControl = (int)geometry.controlTarget;
    const int objetivo = (int)geometry.target;
    const int nivelBajo = (int)geometry.low;
    const int nivelAlto = (int)geometry.high;
    const int nivelSeguro = (int)geometry.safe;
    const u64 nivelPostEscrituraFisico = static_cast<u64>(
        std::max(0, nivelPostEscritura));

    double fastHighOverlaySkew = 0.0;
    const u64 ventanaLenta = 12 * (u64)objetivoControl;

    constexpr u64 ticksUnFrameDS = 355ULL * 263ULL * 3ULL;
    constexpr u64 ticksDosFrames = 2ULL * ticksUnFrameDS;
    constexpr double fpsDS =
        (double)INTERNAL_SAMPLE_RATE / (double)ticksUnFrameDS;
    const u64 quantumTicksSPU = std::max<u64>(1, MixInterval >> 1);
    const u64 ticksUnFramePublicables =
        (ticksUnFrameDS / quantumTicksSPU) * quantumTicksSPU;
    const u64 ticksDosFramesPublicables =
        (ticksDosFrames / quantumTicksSPU) * quantumTicksSPU;

    auto pisoSkew = [this, minSkew, maxSkew]() {
        const double hint = AdaptHint;
        const double piso = hint > 0.0 && std::isfinite(hint)
            ? std::min(hint, 1.0) : 1.0;
        return std::min(std::max(minSkew, piso), maxSkew);
    };
    auto limitar = [=](double value) {
        const double piso = pisoSkew();
        if (value < piso) return piso;
        if (value > maxSkew) return maxSkew;
        return value;
    };
    auto limpiarFastHighEscrow = [&](u32 razones) {
        if (AdaptSostenidoFastHighEscrowInicializado)
        {
            ++telemetry.fastHighEscrowClearCount;
            telemetry.lastFastHighEscrowClearUpdateId = telemetry.updateId;
            telemetry.lastFastHighEscrowClearReasons = razones;
            telemetry.lastFastHighEscrowClearOwnerGeneration =
                AdaptSostenidoFastHighEscrowOwnerGeneracion;
            telemetry.lastFastHighEscrowClearGrantFrames =
                AdaptSostenidoFastHighEscrowGrantFrames;
            telemetry.lastFastHighEscrowClearSpentFrames =
                AdaptSostenidoFastHighEscrowSpentFrames;
        }
        AdaptSostenidoFastHighEscrowInicializado = false;
        AdaptSostenidoFastHighEscrowActivo = false;
        AdaptSostenidoFastHighEscrowReferenciaEsOverride = false;
        AdaptSostenidoFastHighEscrowOwnerGeneracion = 0;
        AdaptSostenidoFastHighEscrowReferenciaTicks = 0;
        AdaptSostenidoFastHighEscrowReferenciaCons = 0;
        AdaptSostenidoFastHighEscrowRate = 0.0;
        AdaptSostenidoFastHighEscrowGuardSkew = 0.0;
        AdaptSostenidoFastHighEscrowHorizonteNacimientoFrames = 0;
        AdaptSostenidoFastHighEscrowHorizonteRestanteFrames = 0;
        AdaptSostenidoFastHighEscrowTicksInicio = AdaptTicksTotal;
        AdaptSostenidoFastHighEscrowFramesPublicadosInicio =
            AdaptFramesProducidosTotal;
        AdaptSostenidoFastHighEscrowConsInicio = hostConsumed;
        AdaptSostenidoFastHighEscrowFramesConsumoInicio = framesAlConsumo;
        AdaptSostenidoFastHighEscrowNivelConsumoInicioLogico =
            nivelPostConsumoLogico;
        AdaptSostenidoFastHighEscrowNivelEscrituraInicioLogico =
            nivelPostEscrituraLogico;
        AdaptSostenidoFastHighEscrowContinuidadEpoch = continuidadEpoch;
        AdaptSostenidoFastHighEscrowResetEpoch = resetSolicitado;
        AdaptSostenidoFastHighEscrowHintSeenEpoch = AdaptHintVistoProductor;
        AdaptSostenidoFastHighEscrowGrantFrames = 0;
        AdaptSostenidoFastHighEscrowSpentFrames = 0;
    };
    auto limpiarFastHighWitness = [&]() {

        AdaptSostenidoFastHighWitnessActivo = false;
        AdaptSostenidoFastHighWitnessReferenciaEsOverride = false;
        AdaptSostenidoFastHighWitnessOwnerGeneracion = 0;
        AdaptSostenidoFastHighWitnessReferenciaTicks = 0;
        AdaptSostenidoFastHighWitnessReferenciaCons = 0;
        AdaptSostenidoFastHighWitnessTicksInicio = ticksAlConsumo;
        AdaptSostenidoFastHighWitnessConsInicio = hostConsumed;
        AdaptSostenidoFastHighWitnessFramesInicio = framesAlConsumo;
        AdaptSostenidoFastHighWitnessNivelInicio = nivelPostConsumo;
        AdaptSostenidoFastHighWitnessNivelInicioLogico =
            nivelPostConsumoLogico;
        AdaptSostenidoFastHighWitnessUltimoNivelConsumo = nivelPostConsumo;
        AdaptSostenidoFastHighWitnessUltimoNivelEscritura =
            nivelPostEscritura;
        AdaptSostenidoFastHighWitnessCapacidad = false;
        AdaptSostenidoFastHighGuardSkew = 0.0;
    };
    auto armarFastHighWitness = [&](bool referenciaEsOverride,
                                     u64 referenciaTicks,
                                     u64 referenciaCons,
                                     bool capacidadFisica) {

        AdaptSostenidoFastHighWitnessActivo = true;
        AdaptSostenidoFastHighWitnessReferenciaEsOverride =
            referenciaEsOverride;
        AdaptSostenidoFastHighWitnessOwnerGeneracion =
            AdaptSostenidoOwnerGeneracion;
        AdaptSostenidoFastHighWitnessReferenciaTicks = referenciaTicks;
        AdaptSostenidoFastHighWitnessReferenciaCons = referenciaCons;
        AdaptSostenidoFastHighWitnessTicksInicio = ticksAlConsumo;
        AdaptSostenidoFastHighWitnessConsInicio = hostConsumed;
        AdaptSostenidoFastHighWitnessFramesInicio = framesAlConsumo;
        AdaptSostenidoFastHighWitnessNivelInicio = nivelPostConsumo;
        AdaptSostenidoFastHighWitnessNivelInicioLogico =
            nivelPostConsumoLogico;
        AdaptSostenidoFastHighWitnessUltimoNivelConsumo = nivelPostConsumo;
        AdaptSostenidoFastHighWitnessUltimoNivelEscritura =
            nivelPostEscritura;
        AdaptSostenidoFastHighWitnessCapacidad = capacidadFisica;
    };
    auto limpiarFastLowWitness = [&](u32 razones) {
        if (AdaptSostenidoFastLowWitnessActivo)
        {
            ++telemetry.fastLowWitnessClearCount;
            telemetry.lastFastLowWitnessClearUpdateId = telemetry.updateId;
            telemetry.lastFastLowWitnessClearReasons = razones;
            telemetry.lastFastLowWitnessClearOwnerGeneration =
                AdaptSostenidoFastLowWitnessOwnerGeneracion;
            telemetry.lastFastLowWitnessClearWitnessConsumed =
                AdaptSostenidoFastLowWitnessConsFin;
            telemetry.lastFastLowWitnessClearWitnessTicks =
                AdaptSostenidoFastLowWitnessTicksFin;
            telemetry.lastFastLowWitnessClearWitnessProduced =
                AdaptSostenidoFastLowWitnessFramesFin;
            telemetry.lastFastLowWitnessClearWitnessLevel =
                static_cast<u32>(std::max(
                    0, AdaptSostenidoFastLowWitnessNivelFin));
            telemetry.lastFastLowWitnessClearConsumed = hostConsumed;
            telemetry.lastFastLowWitnessClearTicks = ticksAlConsumo;
            telemetry.lastFastLowWitnessClearProduced = framesAlConsumo;
            telemetry.lastFastLowWitnessClearLevel =
                static_cast<u32>(std::max(0, nivelPostConsumo));
            telemetry.lastFastLowWitnessClearLevelPostWrite =
                static_cast<u32>(std::max(0, nivelPostEscritura));
        }
        AdaptSostenidoFastLowWitnessActivo = false;
        AdaptSostenidoFastLowWitnessOwnerGeneracion = 0;
        AdaptSostenidoFastLowWitnessTicksNacimiento = 0;
        AdaptSostenidoFastLowWitnessConsNacimiento = 0;
        AdaptSostenidoFastLowWitnessFramesNacimiento = 0;
        AdaptSostenidoFastLowWitnessNivelNacimiento = 0;
        AdaptSostenidoFastLowWitnessNivelNacimientoLogico = 0;
        AdaptSostenidoFastLowWitnessTicksFin = ticksAlConsumo;
        AdaptSostenidoFastLowWitnessConsFin = hostConsumed;
        AdaptSostenidoFastLowWitnessFramesFin = framesAlConsumo;
        AdaptSostenidoFastLowWitnessNivelFin = nivelPostConsumo;
        AdaptSostenidoFastLowWitnessNivelFinLogico =
            nivelPostConsumoLogico;
    };
    auto armarFastLowWitness = [&](u32 tipoNacimiento) {
        AdaptSostenidoFastLowWitnessActivo = true;
        AdaptSostenidoFastLowWitnessOwnerGeneracion =
            AdaptSostenidoOwnerGeneracion;
        AdaptSostenidoFastLowWitnessTicksNacimiento = ticksAlConsumo;
        AdaptSostenidoFastLowWitnessConsNacimiento = hostConsumed;
        AdaptSostenidoFastLowWitnessFramesNacimiento = framesAlConsumo;
        AdaptSostenidoFastLowWitnessNivelNacimiento = nivelPostConsumo;
        AdaptSostenidoFastLowWitnessNivelNacimientoLogico =
            nivelPostConsumoLogico;
        AdaptSostenidoFastLowWitnessTicksFin = ticksAlConsumo;
        AdaptSostenidoFastLowWitnessConsFin = hostConsumed;
        AdaptSostenidoFastLowWitnessFramesFin = framesAlConsumo;
        AdaptSostenidoFastLowWitnessNivelFin = nivelPostConsumo;
        AdaptSostenidoFastLowWitnessNivelFinLogico =
            nivelPostConsumoLogico;
        ++telemetry.fastLowWitnessBirthCount;
        telemetry.lastFastLowWitnessBirthUpdateId = telemetry.updateId;
        telemetry.lastFastLowWitnessBirthKind = tipoNacimiento;
        telemetry.lastFastLowWitnessBirthOwnerGeneration =
            AdaptSostenidoOwnerGeneracion;
        telemetry.lastFastLowWitnessBirthConsumed = hostConsumed;
        telemetry.lastFastLowWitnessBirthTicks = ticksAlConsumo;
        telemetry.lastFastLowWitnessBirthProduced = framesAlConsumo;
        telemetry.lastFastLowWitnessBirthLevel =
            static_cast<u32>(std::max(0, nivelPostConsumo));
        telemetry.lastFastLowWitnessBirthLevelPostWrite =
            static_cast<u32>(std::max(0, nivelPostEscritura));
    };
    auto limpiarPhaseProvisionalOwnerFree = [&](u32 razones) {
        if (AdaptSostenidoPhaseProvisionalActivo)
        {
            ++telemetry.sustainedProvisionalPhaseClearCount;
            telemetry.lastSustainedProvisionalPhaseClearUpdateId =
                telemetry.updateId;
            telemetry.lastSustainedProvisionalPhaseClearReasons = razones;
            telemetry.lastSustainedProvisionalPhaseClearCandidateGeneration =
                AdaptSostenidoPhaseProvisionalCandidatoGeneracion;
            telemetry.lastSustainedProvisionalPhaseClearSkew =
                AdaptSostenidoPhaseProvisionalSkew;
            telemetry.lastSustainedProvisionalPhaseClearReservedPublicationCount =
                AdaptSostenidoPhaseProvisionalPublicacionesReservadas;
        }
        AdaptSostenidoPhaseProvisionalActivo = false;
        AdaptSostenidoPhaseProvisionalCandidatoGeneracion = 0;
        AdaptSostenidoPhaseProvisionalOwnerGeneracion = 0;
        AdaptSostenidoPhaseProvisionalParentRatio = 0.0;
        AdaptSostenidoPhaseProvisionalSkew = 0.0;
        AdaptSostenidoPhaseProvisionalNivelInicio = 0;
        AdaptSostenidoPhaseProvisionalFronteraFrames = 0;
        AdaptSostenidoPhaseProvisionalBackingFrames = 0;
        AdaptSostenidoPhaseProvisionalReservaPublicacionFrames = 0;
        AdaptSostenidoPhaseProvisionalPublicacionesReservadas = 0;
        AdaptSostenidoPhaseProvisionalContinuidadEpoch = 0;
        AdaptSostenidoPhaseProvisionalResetEpoch = 0;
        AdaptSostenidoPhaseProvisionalHintSeenEpoch = 0;
        AdaptSostenidoPhaseProvisionalUnderrunsInicio = 0;
        AdaptSostenidoPhaseProvisionalSpillDropsInicio = 0;
        AdaptSostenidoPhaseProvisionalSpillAllocationFailuresInicio = 0;
    };
    auto limpiarParentCapacity = [&](u32 razones) {
        if (!AdaptParentCapacityActivo)
            return;
        ++telemetry.parentCapacityClearCount;
        telemetry.lastParentCapacityClearUpdateId = telemetry.updateId;
        telemetry.lastParentCapacityClearReasons = razones;
        telemetry.lastParentCapacityClearGeneration =
            AdaptParentCapacityGeneracion;
        telemetry.lastParentCapacityClearConsumed = hostConsumed;
        telemetry.lastParentCapacityClearTicks = ticksAlConsumo;
        telemetry.lastParentCapacityClearPublishedProduced =
            AdaptFramesProducidosTotal;
        telemetry.lastParentCapacityClearLevelPostWrite =
            static_cast<u32>(std::max(0, nivelPostEscritura));
        AdaptParentCapacityActivo = false;
    };
    auto armarParentCapacity = [&](u64 sourceOwnerGeneracion,
                                    double parentRatio) {
        if (AdaptParentCapacityActivo)
        {
            limpiarParentCapacity(
                AudioOutputAdaptiveParentCapacityClearRateChange);
        }
        ++AdaptParentCapacityGeneracion;
        AdaptParentCapacityActivo = true;
        AdaptParentCapacityPrimerRiesgoVisto = false;
        AdaptParentCapacitySourceOwnerGeneracion = sourceOwnerGeneracion;
        AdaptParentCapacityParentRatio = parentRatio;
        AdaptParentCapacityContinuidadEpoch = continuidadEpoch;
        AdaptParentCapacityResetEpoch = resetSolicitado;
        AdaptParentCapacityHintSeenEpoch = AdaptHintVistoProductor;
        AdaptParentCapacityCapacidad = OutputBufferSize - 1;
        AdaptParentCapacityConsInicio = hostConsumed;
        AdaptParentCapacityTicksInicio = ticksAlConsumo;
        AdaptParentCapacityFramesConsumoInicio = framesAlConsumo;
        AdaptParentCapacityFramesPublicadosInicio =
            AdaptFramesProducidosTotal;
        AdaptParentCapacityNivelConsumoInicio = nivelPostConsumo;
        AdaptParentCapacityNivelEscrituraInicio = nivelPostEscritura;
        AdaptParentCapacityNivelConsumoInicioLogico =
            nivelPostConsumoLogico;
        AdaptParentCapacityNivelEscrituraInicioLogico =
            nivelPostEscrituraLogico;
        ++telemetry.parentCapacityBirthCount;
        telemetry.lastParentCapacityBirthUpdateId = telemetry.updateId;
    };
    auto invalidarFronteraPhaseCambioTarget = [&](u32 razones) {
        if (AdaptFastCambioTargetPendiente
            && AdaptFastCambioTargetFronteraPhaseValida)
        {
            ++telemetry.fastTargetChangePhaseFrontierInvalidationCount;
            telemetry.lastFastTargetChangePhaseFrontierInvalidationUpdateId =
                telemetry.updateId;
            telemetry.lastFastTargetChangePhaseFrontierInvalidationReasons =
                razones;
        }
        AdaptFastCambioTargetFronteraPhaseValida = false;
    };
    auto limpiarPhaseEscrowCambioTarget = [&](u32 razones) {
        if (AdaptFastCambioTargetPhaseEscrowActivo)
        {
            ++telemetry.fastTargetChangePhaseEscrowClearCount;
            telemetry.lastFastTargetChangePhaseEscrowClearUpdateId =
                telemetry.updateId;
            telemetry.lastFastTargetChangePhaseEscrowClearReasons = razones;
            telemetry.lastFastTargetChangePhaseEscrowClearGrantFrames =
                AdaptFastCambioTargetPhaseEscrowGrantFrames;
            telemetry.lastFastTargetChangePhaseEscrowClearOverlaySkew =
                AdaptFastCambioTargetPhaseEscrowOverlaySkew;
        }
        AdaptFastCambioTargetPhaseEscrowActivo = false;
        AdaptFastCambioTargetPhaseEscrowGrantFrames = 0;
        AdaptFastCambioTargetPhaseEscrowGuardSkew = 0.0;
        AdaptFastCambioTargetPhaseEscrowOverlaySkew = 0.0;
        AdaptFastCambioTargetPhaseEscrowRateReferencia = 0.0;
        AdaptFastCambioTargetPhaseEscrowTicksInicio = 0;
        AdaptFastCambioTargetPhaseEscrowConsInicio = 0;
        AdaptFastCambioTargetPhaseEscrowFramesConsumoInicio = 0;
        AdaptFastCambioTargetPhaseEscrowFramesPublicadosInicio = 0;
        AdaptFastCambioTargetPhaseEscrowNivelConsumoInicio = 0;
        AdaptFastCambioTargetPhaseEscrowNivelEscrituraInicio = 0;
        AdaptFastCambioTargetPhaseEscrowContinuidadEpoch = 0;
        AdaptFastCambioTargetPhaseEscrowResetEpoch = 0;
        AdaptFastCambioTargetPhaseEscrowHintSeenEpoch = 0;
        AdaptFastCambioTargetPhaseEscrowCandidatoGeneracion = 0;
        AdaptFastCambioTargetPhaseEscrowUnderrunsInicio = 0;
        AdaptFastCambioTargetPhaseEscrowSpillDropsInicio = 0;
        AdaptFastCambioTargetPhaseEscrowSpillAllocationFailuresInicio = 0;
    };
    auto rebasarVentanaFast = [&](bool anchorOwnerLimpio) {

        limpiarPhaseEscrowCambioTarget(
            AudioOutputAdaptiveFastTargetChangePhaseEscrowClearFastRebase);
        AdaptFastTicksInicio = ticksAlConsumo;
        AdaptFastConsInicio = hostConsumed;
        AdaptFastFramesInicio = framesAlConsumo;
        AdaptFastFramesPublicadosInicio = AdaptFramesProducidosTotal;
        AdaptFastNivelInicio = nivelPostConsumo;
        AdaptFastNivelInicioLogico = nivelPostConsumoLogico;
        AdaptFastNivelEscrituraInicioLogico = nivelPostEscrituraLogico;
        AdaptFastContinuidadEpochInicio = continuidadEpoch;
        AdaptFastResetEpochInicio = resetSolicitado;
        AdaptFastHintSeenEpochInicio = AdaptHintVistoProductor;
        AdaptFastCandidatoGeneracionInicio =
            AdaptSostenidoCandidatoGeneracion;
        AdaptFastAnchorOwnerValido =
            anchorOwnerLimpio && AdaptSostenidoOwnerActivo;
        AdaptFastAnchorOwnerGeneracion = AdaptFastAnchorOwnerValido
            ? AdaptSostenidoOwnerGeneracion : 0;

        invalidarFronteraPhaseCambioTarget(
            AudioOutputAdaptiveFastTargetChangePhaseFrontierInvalidationAnchorRebase);
        AdaptFastCambioTargetFronteraPhaseOrigen =
            AudioOutputAdaptiveFastTargetChangePhaseFrontierNone;
        AdaptFastCambioTargetCruzoIntervaloHost = false;
    };
    auto reiniciarVentanas = [&](bool restaurarOverrideSostenido,
                                  u32 razonLowWitness,
                                  u32 razonParentCapacity,
                                  u32 razonFastHighEscrow) {

        limpiarParentCapacity(razonParentCapacity);
        if (AdaptSostenidoOwnerActivo)
        {

            if (restaurarOverrideSostenido
                && AdaptSostenidoOverrideActivo)
            {
                AdaptRatio = limitar(AdaptSostenidoOwnerRatio);
                AdaptSkew = AdaptRatio;
            }
            ++telemetry.sustainedTransitionCount;
            telemetry.lastSustainedTransitionUpdateId = telemetry.updateId;
            telemetry.lastSustainedTransitionKind =
                AudioOutputAdaptiveSustainedInvalidate;
            telemetry.lastSustainedTransitionOwnerGeneration =
                AdaptSostenidoOwnerGeneracion;
        }
        rebasarVentanaFast(false);
        AdaptSlowTicksInicio = ticksAlConsumo;
        AdaptSlowConsInicio = hostConsumed;
        AdaptSostenidoCandidatoActivo = false;
        AdaptSostenidoOwnerActivo = false;
        AdaptSostenidoOwnerRatio = 0.0;
        AdaptSostenidoEpisodioOrigenUpdateId = 0;
        AdaptSostenidoTicksInicio = ticksAlConsumo;
        AdaptSostenidoConsInicio = hostConsumed;
        AdaptSostenidoFramesInicio = framesAlConsumo;
        AdaptSostenidoFramesPublicadosInicio = AdaptFramesProducidosTotal;
        AdaptSostenidoUltimaPublicacionCons = hostConsumed;
        AdaptSostenidoNivelInicio = nivelPostConsumo;
        AdaptSostenidoNivelInicioLogico = nivelPostConsumoLogico;
        AdaptSostenidoNivelEscrituraInicioLogico =
            nivelPostEscrituraLogico;
        AdaptSostenidoContinuidadEpochInicio = continuidadEpoch;
        AdaptSostenidoResetEpochInicio = resetSolicitado;
        AdaptSostenidoHintSeenEpochInicio = AdaptHintVistoProductor;
        AdaptSostenidoNivelMinimo = nivelPostConsumo;
        AdaptSostenidoNivelMaximo = nivelPostEscritura;
        AdaptSostenidoNivelMinimoLogico = nivelPostConsumoLogico;
        AdaptSostenidoNivelMaximoLogico = nivelPostEscrituraLogico;
        AdaptSostenidoSegmentoUnoCompleto = false;
        AdaptSostenidoCandidatoReemplazo = false;
        AdaptSostenidoSegmentoUnoTicks = ticksAlConsumo;
        AdaptSostenidoSegmentoUnoCons = hostConsumed;
        AdaptSostenidoSegmentoUnoFrames = framesAlConsumo;
        AdaptSostenidoSegmentoUnoFramesPublicados =
            AdaptFramesProducidosTotal;
        AdaptSostenidoSegmentoUnoNivel = nivelPostConsumo;
        AdaptSostenidoSegmentoUnoNivelLogico = nivelPostConsumoLogico;
        AdaptSostenidoSegmentoUnoNivelEscrituraLogico =
            nivelPostEscrituraLogico;
        AdaptSostenidoSegmentoUnoContinuidadEpoch = continuidadEpoch;
        AdaptSostenidoSegmentoUnoResetEpoch = resetSolicitado;
        AdaptSostenidoSegmentoUnoHintSeenEpoch = AdaptHintVistoProductor;
        AdaptSostenidoReferenciaRatio = 0.0;
        AdaptSostenidoDireccion = 0;
        AdaptSostenidoOwnerTicks = 0;
        AdaptSostenidoOwnerCons = 0;
        AdaptSostenidoPhaseObjetivoFrames = 0;
        AdaptSostenidoParentValido = false;
        AdaptSostenidoParentEsHint = false;
        AdaptSostenidoParentRatio = 0.0;
        AdaptSostenidoParentTicks = 0;
        AdaptSostenidoParentCons = 0;
        limpiarPhaseProvisionalOwnerFree(
            AudioOutputAdaptiveSustainedProvisionalPhaseClearWindowReset);
        AdaptSostenidoOverrideActivo = false;
        AdaptSostenidoOverrideDireccion = 0;
        AdaptSostenidoOverrideTicks = 0;
        AdaptSostenidoOverrideCons = 0;
        limpiarFastLowWitness(razonLowWitness);
        limpiarFastHighEscrow(razonFastHighEscrow);
        limpiarFastHighWitness();
        AdaptSostenidoRecoveryVerificando = false;
        AdaptSostenidoRecoveryTicksInicio = ticksAlConsumo;
        AdaptSostenidoRecoveryConsInicio = hostConsumed;
        AdaptSostenidoRecoveryFramesInicio = framesAlConsumo;
        AdaptSostenidoRecoveryNivelInicio = nivelPostConsumo;
        AdaptSostenidoRecoveryFrontera = 0;
        AdaptActConsAnterior = hostConsumed;
        AdaptActTicksAnterior = ticksAlConsumo;
        AdaptActFramesAnterior = framesAlConsumo;
        AdaptActCreditoFrames = 0.0;
        AdaptRateCreditoFrames = 0.0;
        AdaptRateActuadorPendiente = false;
        AdaptRateActuadorPreferirRapido = false;
        AdaptRateActuadorCons = 0;
        AdaptRateActuadorTicks = 0;
        AdaptPhaseBalanceFrames = 0.0;
        AdaptPhaseEpisodioActivo = false;
        AdaptPhaseObjetivoFrames = 0;
        AdaptPhaseObjetivoActivo = false;
        AdaptPhaseRebasePendiente = false;
        AdaptPhaseRebaseBandViolada = false;
        AdaptPhaseRebaseMinFrames = -1;
        AdaptPhaseOrigenRecuperacionRate = false;
        AdaptRateBajoConfirmado = false;
        AdaptRatePadreCertificado = false;
        AdaptRatePadreTicks = 0;
        AdaptRatePadreCons = 0;
        AdaptHintPadreCertificado = false;
        AdaptHintCertTicksInicio = ticksAlConsumo;
        AdaptHintCertConsInicio = hostConsumed;
        AdaptRateRollbackPendiente = false;
        AdaptRateRollbackVerificando = false;
        AdaptRateRollbackPadreEsHint = false;
        AdaptRateRollbackRatio = 1.0;
        AdaptRateRollbackObjetivoFrames = 0;
        AdaptRateRollbackFronteraFrames = 0;
        AdaptRateRollbackPadreTicks = 0;
        AdaptRateRollbackPadreCons = 0;
        AdaptRateRollbackTicksInicio = ticksAlConsumo;
        AdaptRateRollbackConsInicio = hostConsumed;
        AdaptFastLowPendiente = false;
        AdaptFastCambioTargetPendiente = false;
        AdaptFastCambioTargetCruzoIntervaloHost = false;
    };
    auto medirRatio = [&](u64 deltaTicks, u64 deltaConsumo) {
        return ((double)deltaTicks * OutputSampleRate) /
               ((double)INTERNAL_SAMPLE_RATE * (double)deltaConsumo);
    };
    auto mismaRelacionHT = [](u64 ticksA, u64 consA,
                              u64 ticksB, u64 consB) {
        if (ticksA == 0 || consA == 0 || ticksB == 0 || consB == 0)
            return false;
        const u64 divisorA = std::gcd(ticksA, consA);
        const u64 divisorB = std::gcd(ticksB, consB);
        return ticksA / divisorA == ticksB / divisorB
            && consA / divisorA == consB / divisorB;
    };
    auto medirDeficit = [&](u64 deltaTicks, u64 deltaConsumo) {
        const double produccionAlaTasaActual =
            ((double)deltaTicks * OutputSampleRate) /
            ((double)INTERNAL_SAMPLE_RATE * AdaptRatio);
        return (double)deltaConsumo - produccionAlaTasaActual;
    };
    auto certificarHintPadre = [&](u64 deltaConsumo, u64 deltaTicks) {
        AdaptHintPadreCertificado = true;
        ++telemetry.hintParentCertificationCount;
        telemetry.lastHintParentCertificationUpdateId = telemetry.updateId;
        telemetry.lastHintParentCertificationRatio = AdaptHint;
        telemetry.lastHintParentCertificationDeltaConsumed = deltaConsumo;
        telemetry.lastHintParentCertificationDeltaTicks = deltaTicks;
        AdaptHintCertTicksInicio = ticksAlConsumo;
        AdaptHintCertConsInicio = hostConsumed;
        AdaptHintCertRebasePostDropPendiente = false;
    };
    auto medirPresupuestoTransitorio = [&]() {

        const double demandaHostUnFrame =
            OutputSampleRate / (fpsDS * AdaptRatio);
        return std::max(presupuestoRate, std::ceil(demandaHostUnFrame));
    };
    auto medirReservaPublicacion = [&](double skew) {
        if (!(skew > 0.0) || !std::isfinite(skew))
            return std::numeric_limits<double>::infinity();
        return std::ceil(
            ((double)maxTicksPublicacion * OutputSampleRate)
            / ((double)INTERNAL_SAMPLE_RATE * skew));
    };
    auto proyectarPhaseProvisionalOwnerFree = [&](double objetivoSkew,
                                                   u64 deltaConsumo,
                                                   u64 deltaTicks,
                                                   bool& intervaloFactible) {
        intervaloFactible = false;

        const double h = (double)deltaConsumo;
        const double k =
            ((double)deltaTicks * OutputSampleRate)
            / (double)INTERNAL_SAMPLE_RATE;
        const double kPublicacion =
            ((double)maxTicksPublicacion * OutputSampleRate)
            / (double)INTERNAL_SAMPLE_RATE;
        const double objetivo = limitar(objetivoSkew);
        const double presupuestoTransitorio = medirPresupuestoTransitorio();
        if (!(h > 0.0) || !(k > 0.0) || !std::isfinite(k)
            || !(kPublicacion > 0.0) || !std::isfinite(kPublicacion)
            || !(objetivo > 0.0) || !std::isfinite(objetivo)
            || !(presupuestoTransitorio >= 0.0)
            || !std::isfinite(presupuestoTransitorio)
            || OutputBufferSize == 0
            || nivelPostConsumo < 0
            || nivelPostEscritura < nivelPostConsumo
            || static_cast<u64>(nivelPostEscritura)
               > static_cast<u64>(OutputBufferSize - 1))
        {
            return AdaptSkew;
        }

        const double capacidad = (double)(OutputBufferSize - 1);
        const double nivel = (double)nivelPostEscritura;
        const double fronteraConsumo = (double)nivelPostConsumo;
        const double reservaMinimaRaw = medirReservaPublicacion(maxSkew);
        const double reservaMaximaRaw = medirReservaPublicacion(pisoSkew());
        if (!std::isfinite(reservaMinimaRaw)
            || !std::isfinite(reservaMaximaRaw)
            || reservaMinimaRaw < 1.0
            || reservaMaximaRaw < reservaMinimaRaw
            || reservaMaximaRaw > capacidad
            || reservaMaximaRaw
               > (double)std::numeric_limits<u64>::max())
        {
            return AdaptSkew;
        }

        const u64 reservaMinima = static_cast<u64>(reservaMinimaRaw);
        const u64 reservaMaxima = static_cast<u64>(reservaMaximaRaw);
        double skewElegido = AdaptSkew;
        double distanciaElegida = std::numeric_limits<double>::infinity();
        bool encontrado = false;

        for (u64 reserva = reservaMinima;
             reserva <= reservaMaxima; ++reserva)
        {
            const double r = (double)reserva;
            const double reservaBacking = std::max({
                fronteraConsumo, r, presupuestoTransitorio});
            if (!std::isfinite(reservaBacking)
                || r > capacidad
                || reservaBacking > capacidad - r)
            {
                continue;
            }

            const double produccionMinima = std::max(
                0.0, h + reservaBacking - nivel);
            const double produccionMaxima =
                h + capacidad - nivel - r;
            if (!(produccionMaxima > 0.0)
                || produccionMinima > produccionMaxima)
            {
                continue;
            }

            double skewMinimo = std::nextafter(
                k / produccionMaxima,
                std::numeric_limits<double>::infinity());
            double skewMaximo = produccionMinima > 0.0
                ? std::nextafter(k / produccionMinima, 0.0)
                : maxSkew;
            const double celdaMinima = std::nextafter(
                kPublicacion / r,
                std::numeric_limits<double>::infinity());
            const double celdaMaxima = reserva > 1
                ? std::nextafter(
                    kPublicacion / (double)(reserva - 1), 0.0)
                : maxSkew;
            skewMinimo = std::max({pisoSkew(), skewMinimo, celdaMinima});
            skewMaximo = std::min({maxSkew, skewMaximo, celdaMaxima});
            if (!std::isfinite(skewMinimo)
                || !std::isfinite(skewMaximo)
                || skewMinimo > skewMaximo)
            {
                continue;
            }

            const double candidato = std::min(
                skewMaximo, std::max(skewMinimo, AdaptSkew));
            const double reservaFinal = medirReservaPublicacion(candidato);
            const double nivelTerminal =
                nivel + k / candidato - h;
            const double backingFinal = std::max({
                fronteraConsumo, reservaFinal, presupuestoTransitorio});
            if (!std::isfinite(candidato) || !(candidato > 0.0)
                || reservaFinal != r
                || !std::isfinite(nivelTerminal)
                || !std::isfinite(backingFinal)
                || nivelTerminal < backingFinal
                || nivelTerminal > capacidad - reservaFinal)
            {
                continue;
            }

            const double distancia = std::abs(candidato - AdaptSkew);
            if (!encontrado || distancia < distanciaElegida
                || (distancia == distanciaElegida
                    && candidato > skewElegido))
            {
                encontrado = true;
                skewElegido = candidato;
                distanciaElegida = distancia;
            }
        }
        if (!encontrado)
            return AdaptSkew;

        intervaloFactible = true;
        return skewElegido;
    };
    auto conservaPCM = [](u64 deltaProducido, u64 nivelInicio,
                          u64 deltaConsumido, u64 nivelFinal) {
        return deltaProducido
                   <= std::numeric_limits<u64>::max() - nivelInicio
            && deltaConsumido
                   <= std::numeric_limits<u64>::max() - nivelFinal
            && deltaProducido + nivelInicio
                   == deltaConsumido + nivelFinal;
    };

    if (AdaptParentCapacityActivo)
    {
        u32 razonesParentCapacity =
            AudioOutputAdaptiveParentCapacityClearNone;
        if (continuidadEpoch != AdaptParentCapacityContinuidadEpoch)
        {
            razonesParentCapacity |=
                AudioOutputAdaptiveParentCapacityClearContinuity;
        }
        if (resetSolicitado != AdaptParentCapacityResetEpoch
            || resetConfirmado != AdaptParentCapacityResetEpoch)
        {
            razonesParentCapacity |=
                AudioOutputAdaptiveParentCapacityClearReset;
        }
        if (priming)
        {
            razonesParentCapacity |=
                AudioOutputAdaptiveParentCapacityClearPriming;
        }
        if (AdaptRatio != AdaptParentCapacityParentRatio)
        {
            razonesParentCapacity |=
                AudioOutputAdaptiveParentCapacityClearRateChange;
        }
        const bool contadoresParentCapacityMonotonos =
            hostConsumed >= AdaptParentCapacityConsInicio
            && ticksAlConsumo >= AdaptParentCapacityTicksInicio
            && framesAlConsumo
               >= AdaptParentCapacityFramesConsumoInicio
            && AdaptFramesProducidosTotal
               >= AdaptParentCapacityFramesPublicadosInicio;
        if (!contadoresParentCapacityMonotonos)
        {
            razonesParentCapacity |=
                AudioOutputAdaptiveParentCapacityClearCounterRegression;
        }
        else
        {
            const u64 deltaCons =
                hostConsumed - AdaptParentCapacityConsInicio;
            const u64 deltaFramesConsumo =
                framesAlConsumo
                - AdaptParentCapacityFramesConsumoInicio;
            const bool conservaBoundaryConsumo = conservaPCM(
                deltaFramesConsumo,
                AdaptParentCapacityNivelConsumoInicioLogico,
                deltaCons,
                nivelPostConsumoLogico);
            bool conservaBoundaryPublicacion = true;
            if (dropsEstaEscritura == 0)
            {
                const u64 deltaFramesPublicados =
                    AdaptFramesProducidosTotal
                    - AdaptParentCapacityFramesPublicadosInicio;
                conservaBoundaryPublicacion = conservaPCM(
                    deltaFramesPublicados,
                    AdaptParentCapacityNivelEscrituraInicioLogico,
                    deltaCons,
                    nivelPostEscrituraLogico);
            }
            if (!conservaBoundaryConsumo
                || !conservaBoundaryPublicacion)
            {
                razonesParentCapacity |=
                    AudioOutputAdaptiveParentCapacityClearPcmConservation;
            }
        }
        if (dropsEstaEscritura > 0)
        {
            razonesParentCapacity |=
                AudioOutputAdaptiveParentCapacityClearDrop;
        }
        if (razonesParentCapacity
            != AudioOutputAdaptiveParentCapacityClearNone)
        {
            limpiarParentCapacity(razonesParentCapacity);
        }
    }
    auto produccionIntegral = [&](u64 deltaTicks, double ratio) {
        if (deltaTicks == 0 || !(ratio > 0.0) || !std::isfinite(ratio))
            return -1.0;
        const double produccion =
            ((double)deltaTicks * OutputSampleRate)
            / ((double)INTERNAL_SAMPLE_RATE * ratio);
        return std::isfinite(produccion) && produccion >= 0.0
            ? std::ceil(produccion) : -1.0;
    };
    auto cambioSostenidoAgotaFrontera = [&](u64 deltaTicks,
                                             u64 deltaConsumo,
                                             int nivelInicio,
                                             double referencia,
                                             int direccion) {
        const double produccion = produccionIntegral(
            deltaTicks, referencia);
        if (!(produccion >= 0.0))
            return false;
        const double inicio = (double)std::max(0, nivelInicio);
        if (direccion < 0)
        {

            return (double)deltaConsumo > produccion + inicio;
        }
        if (direccion > 0)
        {

            const double headroom = (double)std::max(
                0, nivelSeguro - std::max(0, nivelInicio));
            return produccion > (double)deltaConsumo + headroom;
        }
        return false;
    };
    auto compararRelacionHT = [](u64 ticksA, u64 consA,
                                  u64 ticksB, u64 consB) {
        if (ticksA == 0 || consA == 0 || ticksB == 0 || consB == 0)
            return 0;
        return AudioOutputExactMath::CompareProducts(
            ticksA, consB, ticksB, consA);
    };
    auto cambioSostenidoAgotaRelacionExacta = [&](u64 deltaTicks,
                                                   u64 deltaConsumo,
                                                   int nivelInicio,
                                                   int direccion,
                                                   u64 referenciaTicks,
                                                   u64 referenciaCons) {
        if (referenciaTicks == 0 || referenciaCons == 0)
        {
            return false;
        }

        const u64 inicio = static_cast<u64>(std::max(0, nivelInicio));
        if (direccion < 0)
        {
            if (deltaConsumo <= inicio)
                return false;
            const u64 consumoSinReserva = deltaConsumo - inicio;
            return consumoSinReserva > 0
                && AudioOutputExactMath::CompareProducts(
                       consumoSinReserva - 1, referenciaTicks,
                       deltaTicks, referenciaCons) >= 0;
        }
        if (direccion > 0)
        {
            const u64 headroom = static_cast<u64>(std::max(
                0, nivelSeguro - std::max(0, nivelInicio)));
            if (deltaConsumo > std::numeric_limits<u64>::max() - headroom - 1)
                return false;
            return AudioOutputExactMath::CompareProducts(
                deltaTicks, referenciaCons,
                deltaConsumo + headroom + 1, referenciaTicks) >= 0;
        }
        return false;
    };
    auto cambioSostenidoAgotaCapacidadExacta = [&](u64 deltaTicks,
                                                    u64 deltaConsumo,
                                                    int nivelPostEscritura,
                                                    u64 horizonteConsumo,
                                                    u64 referenciaTicks,
                                                    u64 referenciaCons) {
        if (referenciaTicks == 0 || referenciaCons == 0
            || horizonteConsumo == 0 || OutputBufferSize == 0)
        {
            return false;
        }

        const u64 capacidadUtil = static_cast<u64>(OutputBufferSize - 1);
        const u64 nivel = std::min(
            capacidadUtil,
            static_cast<u64>(std::max(0, nivelPostEscritura)));
        const u64 headroom = capacidadUtil - nivel;
        return AudioOutputExactMath::ExhaustsHeadroomOverConsumptionHorizon(
            deltaTicks, deltaConsumo, headroom, horizonteConsumo,
            referenciaTicks, referenciaCons);
    };

    bool aplicarInmediato = false;
    const bool faseRatePoseidaAlEntrar = AdaptRateBajoConfirmado
        || AdaptSostenidoOwnerActivo
        || (AdaptPhaseObjetivoActivo
            && AdaptPhaseOrigenRecuperacionRate);
    if (continuidadEpoch != AdaptContinuidadVistoProductor)
    {
        AdaptContinuidadVistoProductor = continuidadEpoch;

        const bool reingresoTrasUnderrun = priming
            && resetSolicitado == resetConfirmado;
        const bool prepararReservaReingreso = reingresoTrasUnderrun
            && faseRatePoseidaAlEntrar;
        if (prepararReservaReingreso)
        {

            AdaptReingresoRatePendiente = true;
        }
        const bool confirmarReingresoRate = !priming
            && resetSolicitado == resetConfirmado
            && AdaptReingresoRatePendiente;
        reiniciarVentanas(
            true, AudioOutputAdaptiveFastLowWitnessClearContinuity,
            AudioOutputAdaptiveParentCapacityClearContinuity,
            AudioOutputAdaptiveFastHighEscrowClearContinuity);
        if (reingresoTrasUnderrun)
        {

            const u64 reservaReingreso =
                ((u64)resetSolicitado << 32) | (u64)(u32)ventanaRate;
            AdaptReingresoPrimingReserva.store(
                reservaReingreso, std::memory_order_release);
        }
        if (confirmarReingresoRate)
        {

            AdaptPhaseObjetivoFrames = std::min(
                nivelSeguro, std::max(0, nivelPostConsumo));
            AdaptPhaseObjetivoActivo = true;
            AdaptPhaseOrigenRecuperacionRate = true;
        }
        if (!priming)
        {
            AdaptReingresoRatePendiente = false;
            AdaptReingresoPrimingReserva.store(
                0, std::memory_order_release);
        }
        decisionFlags |= AudioOutputAdaptiveContinuityRebased;
    }
    if (resetConfirmado != AdaptResetVistoProductor)
    {
        AdaptResetVistoProductor = resetConfirmado;
        reiniciarVentanas(
            true, AudioOutputAdaptiveFastLowWitnessClearReset,
            AudioOutputAdaptiveParentCapacityClearReset,
            AudioOutputAdaptiveFastHighEscrowClearReset);
        AdaptReingresoRatePendiente = false;
        AdaptReingresoPrimingReserva.store(0, std::memory_order_release);
        AdaptRateBajoConfirmado = false;
        decisionFlags |= AudioOutputAdaptiveResetRebased;
    }

    const u32 hintEpoch = AdaptHintEpoch.load(std::memory_order_acquire);
    if (hintEpoch != AdaptHintVistoProductor)
    {
        AdaptHintVistoProductor = hintEpoch;
        decisionFlags |= AudioOutputAdaptiveHintConsumed;
        double hintNuevo = AdaptSpeedHint.load(std::memory_order_relaxed);

        if (hintNuevo > 0.0)
            hintNuevo = std::min(std::max(hintNuevo, minSkew), maxSkew);

        if (hintNuevo != AdaptHint)
        {
            limpiarParentCapacity(
                AudioOutputAdaptiveParentCapacityClearHintChange);
            AdaptReingresoRatePendiente = false;
            AdaptRateBajoConfirmado = false;
            AdaptHint = hintNuevo;
            const bool rebaseFaseDescendente =
                AdaptHint > 0.0 && AdaptHint < AdaptRatio;
            if (AdaptHint > 0.0 && AdaptHint <= AdaptRatio)
            {

                AdaptRatio = AdaptHint;
                if (AdaptHint <= AdaptSkew)
                    AdaptSkew = AdaptHint;
                AdaptProbe = false;
                aplicarInmediato = AdaptSkew == AdaptHint;
                decisionFlags |=
                    AudioOutputAdaptiveHintDescendingImmediate;
            }
            else
            {

                AdaptProbe = true;
                decisionFlags |=
                    AudioOutputAdaptiveHintProbeOrUpward;
            }
            reiniciarVentanas(
                false, AudioOutputAdaptiveFastLowWitnessClearHintChange,
                AudioOutputAdaptiveParentCapacityClearHintChange,
                AudioOutputAdaptiveFastHighEscrowClearHintChange);
            AdaptPhaseRebasePendiente = rebaseFaseDescendente;
        }
        else
        {

            if (AdaptParentCapacityActivo
                && hintNuevo == AdaptParentCapacityParentRatio)
            {
                AdaptParentCapacityHintSeenEpoch = AdaptHintVistoProductor;
            }

            AdaptFastHintSeenEpochInicio = AdaptHintVistoProductor;
            if (AdaptFastCambioTargetPhaseEscrowActivo)
            {
                AdaptFastCambioTargetPhaseEscrowHintSeenEpoch =
                    AdaptHintVistoProductor;
            }
            if (AdaptSostenidoFastHighEscrowInicializado)
            {
                AdaptSostenidoFastHighEscrowHintSeenEpoch =
                    AdaptHintVistoProductor;
            }
            if (AdaptSostenidoPhaseProvisionalActivo)
            {
                AdaptSostenidoPhaseProvisionalHintSeenEpoch =
                    AdaptHintVistoProductor;
            }
        }
    }

    auto publicarSalida = [&]() {
        AdaptSkew = limitar(AdaptSkew);
        const double skewSalida = limitar(std::max(
            AdaptSkew, fastHighOverlaySkew));
        AdaptActConsAnterior = hostConsumed;
        AdaptActTicksAnterior = ticksAlConsumo;
        AdaptActFramesAnterior = framesAlConsumo;
        AdaptSkewDeseado.store(skewSalida, std::memory_order_relaxed);
        telemetry.desiredSkew = skewSalida;

        if (skewSalida > OutputSkew * 1.000001
            || skewSalida < OutputSkew * 0.999999)
        {
            SetOutputSkew(skewSalida);
            decisionFlags |=
                AudioOutputAdaptiveOutputSkewChanged;
        }
        telemetry.appliedSkew =
            OutputSkewPublicado.load(std::memory_order_relaxed);

        if (skewSalida > AdaptSkew)
        {
            decisionFlags |=
                AudioOutputAdaptiveGuardApplied
                | AudioOutputAdaptiveApplyImmediate;
        }
        if (aplicarInmediato)
            decisionFlags |= AudioOutputAdaptiveApplyImmediate;
        if (AdaptProbe)
            decisionFlags |= AudioOutputAdaptiveProbeAfter;
        telemetry.decisionFlags = decisionFlags;
    };

    if (priming || resetSolicitado != resetConfirmado)
    {
        if (priming)
            decisionFlags |= AudioOutputAdaptivePrimingHintOnly;
        if (resetSolicitado != resetConfirmado)
            decisionFlags |= AudioOutputAdaptiveResetBlocked;
        if (dropsEstaEscritura > 0)
        {
            AdaptReingresoRatePendiente = false;
            AdaptReingresoPrimingReserva.store(
                0, std::memory_order_release);
        }
        reiniciarVentanas(
            true, AudioOutputAdaptiveFastLowWitnessClearPriming,
            AudioOutputAdaptiveParentCapacityClearPriming,
            AudioOutputAdaptiveFastHighEscrowClearPriming);
        publicarSalida();
        publicarEventoProvisional();
        return;
    }

    u64 deltaActCons = 0;
    u64 deltaActTicks = 0;
    u64 deltaActFrames = 0;
    if (hostConsumed >= AdaptActConsAnterior
        && ticksAlConsumo >= AdaptActTicksAnterior
        && framesAlConsumo >= AdaptActFramesAnterior)
    {
        deltaActCons = hostConsumed - AdaptActConsAnterior;
        deltaActTicks = ticksAlConsumo - AdaptActTicksAnterior;
        deltaActFrames = framesAlConsumo - AdaptActFramesAnterior;
    }
    else
    {

        reiniciarVentanas(
            true,
            AudioOutputAdaptiveFastLowWitnessClearGlobalCounterRegression,
            AudioOutputAdaptiveParentCapacityClearCounterRegression,
            AudioOutputAdaptiveFastHighEscrowClearCounterRegression);
    }

    const double creditoActuadorNuevo = (double)deltaActFrames;
    const double produccionRateNueva =
        ((double)deltaActTicks * OutputSampleRate)
        / ((double)INTERNAL_SAMPLE_RATE * AdaptRatio);
    if (std::isfinite(creditoActuadorNuevo)
        && creditoActuadorNuevo >= 0.0
        && std::isfinite(AdaptActCreditoFrames)
        && AdaptActCreditoFrames >= 0.0)
    {
        AdaptActCreditoFrames += creditoActuadorNuevo;
        if (!std::isfinite(AdaptActCreditoFrames))
            AdaptActCreditoFrames = 0.0;
    }
    else
    {
        AdaptActCreditoFrames = 0.0;
    }
    if (std::isfinite(produccionRateNueva)
        && produccionRateNueva >= 0.0
        && std::isfinite(AdaptRateCreditoFrames)
        && AdaptRateCreditoFrames >= 0.0)
    {
        AdaptRateCreditoFrames += produccionRateNueva;
        if (!std::isfinite(AdaptRateCreditoFrames))
            AdaptRateCreditoFrames = 0.0;
    }
    else
    {
        AdaptRateCreditoFrames = 0.0;
    }
    if (dropsEstaEscritura > 0)
    {

        reiniciarVentanas(
            true, AudioOutputAdaptiveFastLowWitnessClearDrop,
            AudioOutputAdaptiveParentCapacityClearDrop,
            AudioOutputAdaptiveFastHighEscrowClearDrop);

        AdaptHintCertRebasePostDropPendiente = true;
        AdaptReingresoRatePendiente = false;
        AdaptReingresoPrimingReserva.store(0, std::memory_order_release);
    }
    if (AdaptHintCertRebasePostDropPendiente
        && dropsEstaEscritura == 0
        && hostConsumed > AdaptHintCertConsInicio
        && ticksAlConsumo > AdaptHintCertTicksInicio)
    {
        AdaptHintCertTicksInicio = ticksAlConsumo;
        AdaptHintCertConsInicio = hostConsumed;
        AdaptHintCertRebasePostDropPendiente = false;
    }
    const double creditoAntesServicio = AdaptActCreditoFrames;
    const double deltaActControl = std::min(
        (double)deltaActCons, creditoAntesServicio);
    AdaptActCreditoFrames -= deltaActControl;
    const double creditoRateAntesServicio = AdaptRateCreditoFrames;
    const double deltaRateControl = std::min(
        (double)deltaActCons, creditoRateAntesServicio);
    AdaptRateCreditoFrames -= deltaRateControl;

    bool actuadorRateRebasado = false;
    u32 actuadorRateReservaWriteInicialFrames = 0;
    auto iniciarActuadorRate = [&](u64 deltaConsumo, u64 deltaTicks,
                                   bool preferirRapido = false,
                                   u32 reservaWriteInicialFrames = 0) {
        AdaptRateActuadorPendiente =
            deltaConsumo > 0
            && deltaTicks > 0
            && std::isfinite(AdaptRatio)
            && std::isfinite(AdaptSkew)
            && AdaptRatio != AdaptSkew;
        AdaptRateActuadorCons = AdaptRateActuadorPendiente
            ? deltaConsumo : 0;
        AdaptRateActuadorTicks = AdaptRateActuadorPendiente
            ? deltaTicks : 0;
        AdaptRateActuadorPreferirRapido =
            AdaptRateActuadorPendiente && preferirRapido;
        actuadorRateReservaWriteInicialFrames = AdaptRateActuadorPendiente
            ? reservaWriteInicialFrames : 0;
        actuadorRateRebasado = AdaptRateActuadorPendiente;
        if (AdaptRateActuadorPendiente)
        {
            ++telemetry.actuatorTransitionCount;
            telemetry.lastActuatorTransitionUpdateId = telemetry.updateId;
            telemetry.lastActuatorStartSkew = AdaptSkew;
            telemetry.lastActuatorTargetRatio = AdaptRatio;
            telemetry.lastActuatorFirstAppliedSkew = 0.0;
            telemetry.lastActuatorFirstControlFrames = 0.0;
            telemetry.lastActuatorDeltaConsumed = deltaConsumo;
            telemetry.lastActuatorDeltaTicks = deltaTicks;
            telemetry.lastActuatorBackingFrames = 0;
            telemetry.lastActuatorLevelPostWrite = 0;
            telemetry.lastActuatorStepCount = 0;
            telemetry.lastActuatorMaxStepRatio = 0.0;
            telemetry.lastActuatorMonotonicViolationCount = 0;
            telemetry.lastActuatorConvergedUpdateId = 0;
        }
    };
    auto aplicarActuadorRate = [&]() {
        if (!AdaptRateActuadorPendiente || deltaActControl <= 0.0)
            return false;

        const double h = (double)AdaptRateActuadorCons;
        const double k =
            ((double)AdaptRateActuadorTicks * OutputSampleRate)
            / (double)INTERNAL_SAMPLE_RATE;
        if (!(h > 0.0) || !(k > 0.0) || !std::isfinite(k))
        {
            AdaptRateActuadorPendiente = false;
            AdaptRateActuadorPreferirRapido = false;
            AdaptRateActuadorCons = 0;
            AdaptRateActuadorTicks = 0;
            return false;
        }

        const int objetivoRateActuador = AdaptPhaseObjetivoActivo
            ? std::min(nivelSeguro,
                       std::max(0, AdaptPhaseObjetivoFrames))
            : objetivo;

        const int reservaFase = objetivoRateActuador;
        const double respaldo = std::max(
            0.0, (double)nivelPostEscritura - (double)reservaFase);
        const double headroom = std::max(
            0.0, (double)nivelSeguro - (double)nivelPostEscritura);

        const double skewMasProductivo = std::min(AdaptSkew, AdaptRatio);
        const double reservaPublicacion =
            medirReservaPublicacion(skewMasProductivo);

        const u64 reservaWriteInicial = std::min<u64>(
            (u64)nivelSeguro,
            (u64)objetivoRateActuador
                + (u64)actuadorRateReservaWriteInicialFrames);
        const double reservaBacking = std::max(
            (double)reservaWriteInicial, reservaPublicacion);
        const double respaldoGastable = std::max(
            0.0, (double)nivelPostEscritura - reservaBacking);
        const double headroomGastable = std::max(
            0.0, headroom - reservaPublicacion);
        const double produccionMinima = std::max(
            0.0, h - respaldoGastable);
        const double produccionMaxima = h + headroomGastable;
        double skewMinimo = produccionMaxima > 0.0
            ? k / produccionMaxima : maxSkew;
        double skewMaximo = produccionMinima > 0.0
            ? k / produccionMinima : maxSkew;
        skewMinimo = limitar(skewMinimo);
        skewMaximo = limitar(skewMaximo);

        const double ratioVentanaActuador = medirRatio(
            AdaptRateActuadorTicks, AdaptRateActuadorCons);
        if (AdaptRatio == ratioVentanaActuador)
        {
            if (produccionMinima == h && skewMaximo < AdaptRatio)
                skewMaximo = AdaptRatio;
            if (produccionMaxima == h && skewMinimo > AdaptRatio)
                skewMinimo = AdaptRatio;
        }

        const bool ascendente = AdaptRatio > AdaptSkew;
        const double lento = ascendente
            ? AdaptSkew * std::pow(1.001, deltaActControl / 512.0)
            : AdaptSkew * std::pow(0.999, deltaActControl / 512.0);
        const double rapido = ascendente
            ? AdaptSkew * std::pow(1.02, deltaActControl / 512.0)
            : AdaptSkew * std::pow(0.98, deltaActControl / 512.0);
        const double segmentoMin = std::min(AdaptSkew, AdaptRatio);
        const double segmentoMax = std::max(AdaptSkew, AdaptRatio);
        const double factibleMin = std::max(skewMinimo, segmentoMin);
        const double factibleMax = std::min(skewMaximo, segmentoMax);
        const bool proyeccionAscendenteDominada =
            factibleMin > factibleMax
            && ascendente
            && skewMinimo > segmentoMax;
        if (factibleMin > factibleMax
            && !proyeccionAscendenteDominada)
        {

            decisionFlags |= AudioOutputAdaptiveSlewBranch;
            return true;
        }
        auto acotarSegmento = [&](double candidato) {
            return std::min(segmentoMax,
                            std::max(segmentoMin, limitar(candidato)));
        };
        auto fisicamenteSeguro = [&](double candidato) {
            return candidato >= factibleMin && candidato <= factibleMax;
        };

        const double primario = AdaptRateActuadorPreferirRapido
            ? rapido : lento;
        const double secundario = AdaptRateActuadorPreferirRapido
            ? lento : rapido;
        double candidato;
        if (proyeccionAscendenteDominada)
        {

            candidato = segmentoMax;
        }
        else
        {
            candidato = acotarSegmento(primario);
            if (!fisicamenteSeguro(candidato))
            {
                candidato = acotarSegmento(secundario);
                if (!fisicamenteSeguro(candidato))
                {
                    candidato = std::min(
                        factibleMax, std::max(factibleMin, candidato));
                }
            }
        }

        if (candidato == AdaptSkew)
        {

            decisionFlags |= AudioOutputAdaptiveSlewBranch;
            return true;
        }

        const double skewAnterior = AdaptSkew;
        const bool monotono = ascendente
            ? (candidato >= skewAnterior && candidato <= AdaptRatio)
            : (candidato <= skewAnterior && candidato >= AdaptRatio);
        if (!monotono)
            ++telemetry.lastActuatorMonotonicViolationCount;
        const double pasoRelativo = skewAnterior > 0.0
            ? std::abs(candidato / skewAnterior - 1.0) : 0.0;
        if (pasoRelativo > telemetry.lastActuatorMaxStepRatio)
            telemetry.lastActuatorMaxStepRatio = pasoRelativo;
        if (proyeccionAscendenteDominada)
        {

            ++telemetry.dominatedAscendingEndpointCount;
            telemetry.lastDominatedAscendingEndpointUpdateId =
                telemetry.updateId;
            telemetry.lastDominatedAscendingEndpointTransitionUpdateId =
                telemetry.lastActuatorTransitionUpdateId;
            telemetry.lastDominatedAscendingEndpointStartSkew = skewAnterior;
            telemetry.lastDominatedAscendingEndpointTargetRatio = AdaptRatio;
            telemetry.lastDominatedAscendingEndpointAppliedSkew = candidato;
            telemetry.lastDominatedAscendingEndpointDeltaConsumed =
                AdaptRateActuadorCons;
            telemetry.lastDominatedAscendingEndpointDeltaTicks =
                AdaptRateActuadorTicks;
            telemetry.lastDominatedAscendingEndpointBackingFrames =
                static_cast<u32>(respaldo);
            telemetry.lastDominatedAscendingEndpointLevelPostWrite =
                static_cast<u32>(std::max(0, nivelPostEscritura));
            telemetry.lastDominatedAscendingEndpointOutputBufferSize =
                OutputBufferSize;
            telemetry.lastDominatedAscendingEndpointOutputSampleRate =
                OutputSampleRate;
        }
        if (telemetry.lastActuatorStepCount == 0)
        {

            telemetry.lastActuatorDeltaConsumed =
                AdaptRateActuadorCons;
            telemetry.lastActuatorDeltaTicks =
                AdaptRateActuadorTicks;
            telemetry.lastActuatorFirstAppliedSkew = candidato;
            telemetry.lastActuatorFirstControlFrames = deltaActControl;
            telemetry.lastActuatorBackingFrames =
                static_cast<u32>(respaldo);
            telemetry.lastActuatorLevelPostWrite = static_cast<u32>(
                std::max(0, nivelPostEscritura));
        }
        ++telemetry.lastActuatorStepCount;

        decisionFlags |= AudioOutputAdaptiveSlewBranch;
        if (candidato > AdaptSkew)
            decisionFlags |= AudioOutputAdaptiveSlewUp;
        else if (candidato < AdaptSkew)
            decisionFlags |= AudioOutputAdaptiveSlewDown;
        AdaptSkew = candidato;

        actuadorRateReservaWriteInicialFrames = 0;
        if (AdaptSkew == AdaptRatio)
        {
            AdaptRateActuadorPendiente = false;
            AdaptRateActuadorPreferirRapido = false;
            AdaptRateActuadorCons = 0;
            AdaptRateActuadorTicks = 0;
            telemetry.lastActuatorConvergedUpdateId = telemetry.updateId;
            decisionFlags |= AudioOutputAdaptiveSlewSnap;
        }
        return true;
    };
    const double balanceRateNuevo =
        produccionRateNueva - (double)deltaActCons;
    const double balanceFaseNuevo =
        creditoActuadorNuevo - (double)deltaActCons;
    const bool intervaloHostPerdido = std::isfinite(produccionRateNueva)
        && -balanceRateNuevo > deltaRateControl;
    if (AdaptFastCambioTargetPendiente && intervaloHostPerdido)
        AdaptFastCambioTargetCruzoIntervaloHost = true;
    if (AdaptFastCambioTargetPendiente
        && (intervaloHostPerdido || dropsEstaEscritura > 0))
    {

        u32 razonesFronteraPhase =
            AudioOutputAdaptiveFastTargetChangePhaseFrontierInvalidationNone;
        if (intervaloHostPerdido)
        {
            razonesFronteraPhase |=
                AudioOutputAdaptiveFastTargetChangePhaseFrontierInvalidationHostInterval;
        }
        if (dropsEstaEscritura > 0)
        {
            razonesFronteraPhase |=
                AudioOutputAdaptiveFastTargetChangePhaseFrontierInvalidationDrop;
        }
        invalidarFronteraPhaseCambioTarget(razonesFronteraPhase);
    }
    if (AdaptFastCambioTargetPhaseEscrowActivo)
    {
        const bool contadoresEscrowMonotonos =
            hostConsumed >= AdaptFastCambioTargetPhaseEscrowConsInicio
            && framesAlConsumo
               >= AdaptFastCambioTargetPhaseEscrowFramesConsumoInicio
            && AdaptFramesProducidosTotal
               >= AdaptFastCambioTargetPhaseEscrowFramesPublicadosInicio
            && AdaptTicksTotal
               >= AdaptFastCambioTargetPhaseEscrowTicksInicio;
        const u64 deltaEscrowCons = contadoresEscrowMonotonos
            ? hostConsumed - AdaptFastCambioTargetPhaseEscrowConsInicio : 0;
        const u64 deltaEscrowFramesCons = contadoresEscrowMonotonos
            ? framesAlConsumo
                - AdaptFastCambioTargetPhaseEscrowFramesConsumoInicio
            : 0;
        const u64 deltaEscrowFramesPublicados = contadoresEscrowMonotonos
            ? AdaptFramesProducidosTotal
                - AdaptFastCambioTargetPhaseEscrowFramesPublicadosInicio
            : 0;
        const bool conservaEscrowConsumo = contadoresEscrowMonotonos
            && conservaPCM(
                deltaEscrowFramesCons,
                AdaptFastCambioTargetPhaseEscrowNivelConsumoInicio,
                deltaEscrowCons,
                nivelPostConsumoLogico);
        const bool conservaEscrowPublicacion = contadoresEscrowMonotonos
            && conservaPCM(
                deltaEscrowFramesPublicados,
                AdaptFastCambioTargetPhaseEscrowNivelEscrituraInicio,
                deltaEscrowCons,
                nivelPostEscrituraLogico);
        const u64 deltaEscrowTicks = contadoresEscrowMonotonos
            ? AdaptTicksTotal - AdaptFastCambioTargetPhaseEscrowTicksInicio
            : 0;
        const bool lineageEscrowVigente =
            AdaptFastCambioTargetPendiente
            && !AdaptSostenidoCandidatoActivo
            && !AdaptSostenidoOwnerActivo
            && !AdaptFastAnchorOwnerValido
            && AdaptFastAnchorOwnerGeneracion == 0
            && continuidadEpoch
               == AdaptFastCambioTargetPhaseEscrowContinuidadEpoch
            && resetSolicitado
               == AdaptFastCambioTargetPhaseEscrowResetEpoch
            && resetConfirmado
               == AdaptFastCambioTargetPhaseEscrowResetEpoch
            && AdaptHintVistoProductor
               == AdaptFastCambioTargetPhaseEscrowHintSeenEpoch
            && AdaptSostenidoCandidatoGeneracion
               == AdaptFastCambioTargetPhaseEscrowCandidatoGeneracion
            && AdaptRatio
               == AdaptFastCambioTargetPhaseEscrowRateReferencia;
        const bool storageEscrowVigente =
            nivelPostConsumo >= 0
            && nivelPostEscritura >= nivelPostConsumo
            && nivelPostConsumoLogico
               == static_cast<u64>(nivelPostConsumo)
            && nivelPostEscrituraLogico
               == static_cast<u64>(nivelPostEscritura)
            && OutputSpillFrames == 0
            && OutputSpillDroppedFramesTotal
               == AdaptFastCambioTargetPhaseEscrowSpillDropsInicio
            && OutputSpillAllocationFailureCount
               == AdaptFastCambioTargetPhaseEscrowSpillAllocationFailuresInicio;
        const bool transporteEscrowVigente =
            dropsEstaEscritura == 0
            && underrunsActuales
               == AdaptFastCambioTargetPhaseEscrowUnderrunsInicio;
        u32 razonesClearEscrow =
            AudioOutputAdaptiveFastTargetChangePhaseEscrowClearNone;
        if (!AdaptFastCambioTargetPendiente)
        {
            razonesClearEscrow |=
                AudioOutputAdaptiveFastTargetChangePhaseEscrowClearPending;
        }
        if (AdaptSostenidoCandidatoActivo
            || AdaptSostenidoOwnerActivo
            || AdaptFastAnchorOwnerValido
            || AdaptFastAnchorOwnerGeneracion != 0)
        {
            razonesClearEscrow |=
                AudioOutputAdaptiveFastTargetChangePhaseEscrowClearOwnership;
        }
        if (continuidadEpoch
            != AdaptFastCambioTargetPhaseEscrowContinuidadEpoch)
        {
            razonesClearEscrow |=
                AudioOutputAdaptiveFastTargetChangePhaseEscrowClearContinuity;
        }
        if (resetSolicitado != AdaptFastCambioTargetPhaseEscrowResetEpoch
            || resetConfirmado != AdaptFastCambioTargetPhaseEscrowResetEpoch)
        {
            razonesClearEscrow |=
                AudioOutputAdaptiveFastTargetChangePhaseEscrowClearReset;
        }
        if (AdaptHintVistoProductor
            != AdaptFastCambioTargetPhaseEscrowHintSeenEpoch)
        {
            razonesClearEscrow |=
                AudioOutputAdaptiveFastTargetChangePhaseEscrowClearHintChange;
        }
        if (AdaptSostenidoCandidatoGeneracion
            != AdaptFastCambioTargetPhaseEscrowCandidatoGeneracion)
        {
            razonesClearEscrow |=
                AudioOutputAdaptiveFastTargetChangePhaseEscrowClearCandidateGeneration;
        }
        if (AdaptRatio != AdaptFastCambioTargetPhaseEscrowRateReferencia)
        {
            razonesClearEscrow |=
                AudioOutputAdaptiveFastTargetChangePhaseEscrowClearRateReference;
        }
        if (!contadoresEscrowMonotonos)
        {
            razonesClearEscrow |=
                AudioOutputAdaptiveFastTargetChangePhaseEscrowClearCounterRegression;
        }
        if (contadoresEscrowMonotonos && !conservaEscrowConsumo)
        {
            razonesClearEscrow |=
                AudioOutputAdaptiveFastTargetChangePhaseEscrowClearConsumedPcm;
        }
        if (contadoresEscrowMonotonos && !conservaEscrowPublicacion)
        {
            razonesClearEscrow |=
                AudioOutputAdaptiveFastTargetChangePhaseEscrowClearPublishedPcm;
        }
        if (!storageEscrowVigente)
        {
            razonesClearEscrow |=
                AudioOutputAdaptiveFastTargetChangePhaseEscrowClearStorage;
        }
        if (dropsEstaEscritura != 0)
        {
            razonesClearEscrow |=
                AudioOutputAdaptiveFastTargetChangePhaseEscrowClearDrop;
        }
        if (underrunsActuales
            != AdaptFastCambioTargetPhaseEscrowUnderrunsInicio)
        {
            razonesClearEscrow |=
                AudioOutputAdaptiveFastTargetChangePhaseEscrowClearUnderrun;
        }
        if (deltaEscrowTicks >= ticksDosFramesPublicables)
        {
            razonesClearEscrow |=
                AudioOutputAdaptiveFastTargetChangePhaseEscrowClearHorizon;
        }
        if (!lineageEscrowVigente
            || !storageEscrowVigente
            || !transporteEscrowVigente
            || !conservaEscrowConsumo
            || !conservaEscrowPublicacion
            || deltaEscrowTicks >= ticksDosFramesPublicables)
        {
            limpiarPhaseEscrowCambioTarget(razonesClearEscrow);
        }
    }
    const bool actuadorHintCertificadoAntesP3 =
        AdaptHintPadreCertificado && AdaptRateActuadorPendiente;
    if (intervaloHostPerdido)
    {
        AdaptHintPadreCertificado = false;

        AdaptHintCertTicksInicio = ticksAlConsumo;
        AdaptHintCertConsInicio = hostConsumed;
        if (actuadorHintCertificadoAntesP3)
        {

            AdaptRateActuadorPreferirRapido = false;
        }
    }
    if (AdaptRateRollbackPendiente && intervaloHostPerdido)
    {
        if (AdaptRateRollbackPadreEsHint)
        {

            AdaptRateRollbackPendiente = false;
            AdaptRateRollbackVerificando = false;
            AdaptRateRollbackPadreEsHint = false;
            AdaptRateRollbackRatio = 1.0;
            AdaptRateRollbackObjetivoFrames = 0;
            AdaptRateRollbackFronteraFrames = 0;
            AdaptRateRollbackPadreTicks = 0;
            AdaptRateRollbackPadreCons = 0;
            AdaptRateRollbackTicksInicio = ticksAlConsumo;
            AdaptRateRollbackConsInicio = hostConsumed;
            AdaptRateActuadorPreferirRapido = false;
        }
        else
        {

            AdaptRateRollbackVerificando = false;
            AdaptRateRollbackFronteraFrames = 0;
            AdaptRateRollbackTicksInicio = ticksAlConsumo;
            AdaptRateRollbackConsInicio = hostConsumed;
        }
    }
    if (AdaptRateRollbackPendiente
        && !AdaptSostenidoCandidatoActivo
        && !AdaptSostenidoOwnerActivo
        && !intervaloHostPerdido)
    {
        if (!AdaptRateRollbackVerificando)
        {
            const bool reservaPadreRecuperada =
                deltaActCons > 0
                && nivelPostConsumo
                   >= AdaptRateRollbackObjetivoFrames
                && nivelPostEscritura
                   >= AdaptRateRollbackObjetivoFrames;
            if (reservaPadreRecuperada)
            {

                AdaptRateRollbackVerificando = true;

                AdaptRateRollbackFronteraFrames =
                    AdaptRateRollbackObjetivoFrames;
                AdaptRateRollbackTicksInicio = ticksAlConsumo;
                AdaptRateRollbackConsInicio = hostConsumed;
            }
        }
        else if (nivelPostConsumo < AdaptRateRollbackFronteraFrames
                 || nivelPostEscritura
                    < AdaptRateRollbackFronteraFrames)
        {

            AdaptRateRollbackVerificando = false;
            AdaptRateRollbackFronteraFrames = 0;
            AdaptRateRollbackTicksInicio = ticksAlConsumo;
            AdaptRateRollbackConsInicio = hostConsumed;
        }
        else if (deltaActCons > 0
                 && hostConsumed >= AdaptRateRollbackConsInicio
                 && ticksAlConsumo >= AdaptRateRollbackTicksInicio)
        {
            const u64 deltaRollbackCons =
                hostConsumed - AdaptRateRollbackConsInicio;
            const u64 deltaRollbackTicks =
                ticksAlConsumo - AdaptRateRollbackTicksInicio;
            const bool ventanaRollback =
                deltaRollbackCons >= ventanaRate
                && deltaRollbackTicks >= ticksDosFramesPublicables;
            const bool ratePadreExacto = ventanaRollback
                && !AdaptRateRollbackPadreEsHint
                && mismaRelacionHT(
                    deltaRollbackTicks, deltaRollbackCons,
                    AdaptRateRollbackPadreTicks,
                    AdaptRateRollbackPadreCons);
            const double produccionIntegralPadreHint =
                AdaptRateRollbackPadreEsHint && ventanaRollback
                ? ((double)deltaRollbackTicks * OutputSampleRate)
                  / ((double)INTERNAL_SAMPLE_RATE
                     * AdaptRateRollbackRatio)
                : 0.0;
            const bool ratePadreHintSuficiente =
                AdaptRateRollbackPadreEsHint
                && ventanaRollback
                && std::isfinite(produccionIntegralPadreHint)
                && (double)deltaRollbackCons
                   <= std::ceil(produccionIntegralPadreHint);
            if (ratePadreExacto || ratePadreHintSuficiente)
            {

                const double ratioPadre = AdaptRateRollbackRatio;
                const int objetivoPadre =
                    AdaptRateRollbackObjetivoFrames;
                const u64 ticksPadre = AdaptRateRollbackPadreTicks;
                const u64 consPadre = AdaptRateRollbackPadreCons;
                const bool padreEsHint =
                    AdaptRateRollbackPadreEsHint;
                const double ratioPadreLimitado = limitar(ratioPadre);
                const int objetivoPadreLimitado = std::min(
                    nivelSeguro, std::max(0, objetivoPadre));
                ++telemetry.rollbackCount;
                telemetry.lastRollbackUpdateId = telemetry.updateId;
                telemetry.lastRollbackParentRatio = ratioPadreLimitado;
                telemetry.lastRollbackTargetFrames =
                    static_cast<u32>(objetivoPadreLimitado);
                telemetry.lastRollbackDeltaConsumed = deltaRollbackCons;
                telemetry.lastRollbackDeltaTicks = deltaRollbackTicks;
                telemetry.lastRollbackParentWasHint = padreEsHint;
                const double skewAntesRollback = AdaptSkew;
                reiniciarVentanas(
                    true, AudioOutputAdaptiveFastLowWitnessClearRollback,
                    AudioOutputAdaptiveParentCapacityClearRateChange,
                    AudioOutputAdaptiveFastHighEscrowClearRate);
                AdaptRatio = ratioPadreLimitado;
                AdaptSkew = skewAntesRollback;
                AdaptProbe = false;
                AdaptRateBajoConfirmado = !padreEsHint;
                AdaptRatePadreCertificado = !padreEsHint;
                AdaptRatePadreTicks = padreEsHint ? 0 : ticksPadre;
                AdaptRatePadreCons = padreEsHint ? 0 : consPadre;
                if (padreEsHint)
                    certificarHintPadre(deltaRollbackCons,
                                        deltaRollbackTicks);
                AdaptPhaseObjetivoFrames = objetivoPadreLimitado;
                AdaptPhaseObjetivoActivo = true;
                AdaptPhaseOrigenRecuperacionRate = padreEsHint;
                iniciarActuadorRate(
                    deltaRollbackCons, deltaRollbackTicks, true);
                aplicarInmediato = !AdaptRateActuadorPendiente;
                aplicarActuadorRate();
                if (!AdaptRateActuadorPendiente
                    && AdaptSkew != AdaptRatio)
                {
                    AdaptSkew = AdaptRatio;
                    aplicarInmediato = true;
                }
                publicarSalida();
                publicarEventoProvisional();
                return;
            }
            if (ventanaRollback)
            {

                AdaptRateRollbackTicksInicio = ticksAlConsumo;
                AdaptRateRollbackConsInicio = hostConsumed;
            }
        }
    }

    const bool targetRatePoseido = AdaptPhaseObjetivoActivo
        && (AdaptRateBajoConfirmado
            || AdaptSostenidoOwnerActivo
            || AdaptPhaseOrigenRecuperacionRate);
    int objetivoRateCausal = objetivo;
    int nivelRecuperacionPostConsumo = nivelBajo;
    int nivelRecuperacionAscendente = objetivo;
    if (targetRatePoseido)
    {
        objetivoRateCausal = std::min(
            nivelSeguro, std::max(0, AdaptPhaseObjetivoFrames));
        nivelRecuperacionPostConsumo = objetivoRateCausal;
        nivelRecuperacionAscendente = std::max(
            objetivo, objetivoRateCausal);
    }
    const int reservaTargetSobreObjetivo = targetRatePoseido
        ? std::max(0, objetivoRateCausal - objetivo) : 0;
    const int reservaFaseNoElegible =
        targetRatePoseido && !intervaloHostPerdido
        ? reservaTargetSobreObjetivo : 0;
    const double respaldoRateElegible = std::max(
        0.0, (double)nivelPostEscritura - reservaFaseNoElegible);
    const double creditoLibreMax = (double)nivelSeguro;
    if (!std::isfinite(AdaptActCreditoFrames)
        || AdaptActCreditoFrames < 0.0)
    {
        AdaptActCreditoFrames = 0.0;
    }
    else if (AdaptActCreditoFrames > creditoLibreMax)
    {
        AdaptActCreditoFrames = creditoLibreMax;
    }
    if (!std::isfinite(AdaptRateCreditoFrames)
        || AdaptRateCreditoFrames < 0.0)
    {
        AdaptRateCreditoFrames = 0.0;
    }
    else if (AdaptRateCreditoFrames > creditoLibreMax)
    {
        AdaptRateCreditoFrames = creditoLibreMax;
    }
    const bool intervaloUnFramePublicable =
        deltaActTicks >= ticksUnFramePublicables
        && deltaActTicks < ticksDosFramesPublicables;
    const bool demandaIntervaloSuperaProduccionEntera =
        intervaloUnFramePublicable
        && std::isfinite(produccionRateNueva)
        && (double)deltaActCons > std::ceil(produccionRateNueva);
    const double deficitIntervalo =
        demandaIntervaloSuperaProduccionEntera
        ? (double)deltaActCons - produccionRateNueva : 0.0;
    const bool targetCambioRateObservable = AdaptHint > 0.0
        && !AdaptProbe
        && (AdaptRatio == AdaptHint || targetRatePoseido);
    const bool fastLowWitnessContadoresMonotonos =
        AdaptSostenidoFastLowWitnessActivo
        && ticksAlConsumo >= AdaptSostenidoFastLowWitnessTicksFin
        && hostConsumed >= AdaptSostenidoFastLowWitnessConsFin
        && framesAlConsumo >= AdaptSostenidoFastLowWitnessFramesFin;

    if (AdaptSostenidoFastLowWitnessActivo
        && (!AdaptSostenidoOwnerActivo
            || AdaptSostenidoFastLowWitnessOwnerGeneracion
               != AdaptSostenidoOwnerGeneracion
            || !fastLowWitnessContadoresMonotonos
            || dropsEstaEscritura > 0))
    {
        u32 razones = AudioOutputAdaptiveFastLowWitnessClearNone;
        if (!AdaptSostenidoOwnerActivo)
            razones |= AudioOutputAdaptiveFastLowWitnessClearOwnerInactive;
        if (AdaptSostenidoFastLowWitnessOwnerGeneracion
            != AdaptSostenidoOwnerGeneracion)
        {
            razones |=
                AudioOutputAdaptiveFastLowWitnessClearOwnerGeneration;
        }
        if (!fastLowWitnessContadoresMonotonos)
        {
            razones |=
                AudioOutputAdaptiveFastLowWitnessClearWitnessCounterRegression;
        }
        if (dropsEstaEscritura > 0)
            razones |= AudioOutputAdaptiveFastLowWitnessClearDrop;
        limpiarFastLowWitness(razones);
    }
    const bool transaccionRateRecuperada = targetRatePoseido
        && !AdaptSostenidoOwnerActivo
        && (AdaptFastLowPendiente || AdaptFastCambioTargetPendiente)
        && nivelPostConsumo >= objetivoRateCausal
        && nivelPostEscritura >= objetivoRateCausal;
    if (transaccionRateRecuperada)
    {
        ++telemetry.rateRecoveryRebaseCount;
        telemetry.lastRateRecoveryRebaseUpdateId = telemetry.updateId;
        telemetry.lastRateRecoveryRebaseDeltaConsumed =
            hostConsumed - AdaptFastConsInicio;
        telemetry.lastRateRecoveryRebaseDeltaTicks =
            ticksAlConsumo - AdaptFastTicksInicio;
        telemetry.lastRateRecoveryRebaseLevelPostConsumption =
            static_cast<u32>(std::max(0, nivelPostConsumo));
        telemetry.lastRateRecoveryRebaseLevelPostWrite =
            static_cast<u32>(std::max(0, nivelPostEscritura));
        telemetry.lastRateRecoveryRebaseTargetRatio = AdaptRatio;
        telemetry.lastRateRecoveryRebaseBoundaryFrames =
            static_cast<u32>(std::max(0, objetivoRateCausal));
        telemetry.lastRateRecoveryRebaseFastLowPending =
            AdaptFastLowPendiente;
        telemetry.lastRateRecoveryRebaseFastTargetChangePending =
            AdaptFastCambioTargetPendiente;
        AdaptFastLowPendiente = false;
        AdaptFastCambioTargetPendiente = false;
        rebasarVentanaFast(
            !intervaloHostPerdido && dropsEstaEscritura == 0);
        AdaptSlowTicksInicio = ticksAlConsumo;
        AdaptSlowConsInicio = hostConsumed;
        decisionFlags |= AudioOutputAdaptiveLowRejectedByLevel;
    }
    const u64 deltaFastAntesFronteraCons =
        hostConsumed - AdaptFastConsInicio;
    const u64 deltaFastAntesFronteraTicks =
        ticksAlConsumo - AdaptFastTicksInicio;
    const bool endpointLowSostenidoAntesFrontera =
        AdaptSostenidoOwnerActivo
        && deltaFastAntesFronteraCons >= ventanaRate
        && deltaFastAntesFronteraTicks >= ticksDosFramesPublicables
        && !intervaloHostPerdido
        && dropsEstaEscritura == 0
        && nivelPostConsumo < nivelSeguro
        && nivelPostEscritura < nivelSeguro
        && compararRelacionHT(
            deltaFastAntesFronteraTicks,
            deltaFastAntesFronteraCons,
            AdaptSostenidoOwnerTicks,
            AdaptSostenidoOwnerCons) < 0;
    const bool armarLowSostenidoUnFrame =
        AdaptSostenidoOwnerActivo
        && !AdaptSostenidoFastLowWitnessActivo
        && !(AdaptSostenidoOverrideActivo
             && AdaptSostenidoOverrideDireccion < 0)
        && targetCambioRateObservable
        && dropsEstaEscritura == 0
        && !intervaloHostPerdido
        && intervaloUnFramePublicable
        && demandaIntervaloSuperaProduccionEntera
        && std::isfinite(deficitIntervalo)
        && deficitIntervalo > 0.0
        && deficitIntervalo < respaldoRateElegible
        && nivelPostConsumo < nivelSeguro
        && nivelPostEscritura < nivelSeguro
        && compararRelacionHT(
            deltaActTicks,
            deltaActCons,
            AdaptSostenidoOwnerTicks,
            AdaptSostenidoOwnerCons) < 0;
    if (armarLowSostenidoUnFrame)
    {

        rebasarVentanaFast(true);
        AdaptSlowTicksInicio = ticksAlConsumo;
        AdaptSlowConsInicio = hostConsumed;
        armarFastLowWitness(
            AudioOutputAdaptiveFastLowWitnessBirthOneFrameFrontier);
        AdaptFastLowPendiente = false;
        AdaptFastCambioTargetPendiente = false;
        decisionFlags |= AudioOutputAdaptiveLowDeferred;
    }
    const bool armarFronteraUnFrame =
        targetCambioRateObservable
        && !armarLowSostenidoUnFrame
        && !endpointLowSostenidoAntesFrontera
        && (!AdaptFastLowPendiente || targetRatePoseido)
        && !AdaptSostenidoFastLowWitnessActivo
        && !AdaptFastCambioTargetPendiente
        && dropsEstaEscritura == 0
        && !intervaloHostPerdido
        && demandaIntervaloSuperaProduccionEntera
        && std::isfinite(deficitIntervalo)
        && deficitIntervalo > 0.0
        && deficitIntervalo < respaldoRateElegible;
    if (armarFronteraUnFrame)
    {

        AdaptFastLowPendiente = false;
        rebasarVentanaFast(true);

        AdaptSlowTicksInicio = ticksAlConsumo;
        AdaptSlowConsInicio = hostConsumed;
        AdaptFastCambioTargetPendiente = true;
        AdaptFastCambioTargetFronteraPhaseValida = true;
        AdaptFastCambioTargetFronteraPhaseOrigen =
            AudioOutputAdaptiveFastTargetChangePhaseFrontierOneFrame;
        decisionFlags |= AudioOutputAdaptiveLowDeferred;
    }
    const u64 deltaFastCons = hostConsumed - AdaptFastConsInicio;
    const u64 deltaFastTicks = ticksAlConsumo - AdaptFastTicksInicio;
    const bool nivelFuera = nivelPostConsumo < nivelBajo
                         || nivelPostConsumo > nivelAlto;
    const bool ventanaFast = deltaFastCons >= ventanaRate
                          && deltaFastTicks >= ticksDosFrames;
    const bool urgenciaFast = nivelPostEscritura >= nivelAlto
                           && deltaFastCons > 0
                           && deltaFastTicks >= ticksDosFrames;
    const bool ventanaFastPublicable = deltaFastCons >= ventanaRate
                                    && deltaFastTicks
                                       >= ticksDosFramesPublicables;
    const bool urgenciaSostenidaBaja = AdaptSostenidoOwnerActivo
        && nivelPostConsumo < nivelBajo
        && ventanaFastPublicable;
    bool fastHighEscrowInvalidadoEsteUpdate = false;
    if (AdaptSostenidoFastHighEscrowInicializado)
    {
        u32 razonesEscrow = AudioOutputAdaptiveFastHighEscrowClearNone;
        if (AdaptSostenidoFastHighEscrowOwnerGeneracion
            != AdaptSostenidoOwnerGeneracion)
        {
            razonesEscrow |=
                AudioOutputAdaptiveFastHighEscrowClearOwnerGeneration;
        }
        if (AdaptSostenidoFastHighEscrowReferenciaTicks == 0
            || AdaptSostenidoFastHighEscrowReferenciaCons == 0)
        {
            razonesEscrow |=
                AudioOutputAdaptiveFastHighEscrowClearReference;
        }
        if (AdaptSostenidoFastHighEscrowRate != AdaptRatio)
            razonesEscrow |= AudioOutputAdaptiveFastHighEscrowClearRate;
        if (AdaptSostenidoFastHighEscrowContinuidadEpoch != continuidadEpoch)
        {
            razonesEscrow |=
                AudioOutputAdaptiveFastHighEscrowClearContinuity;
        }
        if (AdaptSostenidoFastHighEscrowResetEpoch != resetSolicitado
            || AdaptSostenidoFastHighEscrowResetEpoch != resetConfirmado)
        {
            razonesEscrow |= AudioOutputAdaptiveFastHighEscrowClearReset;
        }
        if (AdaptSostenidoFastHighEscrowHintSeenEpoch
            != AdaptHintVistoProductor)
        {
            razonesEscrow |=
                AudioOutputAdaptiveFastHighEscrowClearHintChange;
        }
        if (priming)
            razonesEscrow |= AudioOutputAdaptiveFastHighEscrowClearPriming;
        if (intervaloHostPerdido)
        {
            razonesEscrow |=
                AudioOutputAdaptiveFastHighEscrowClearHostIntervalLost;
        }
        if (dropsEstaEscritura > 0)
            razonesEscrow |= AudioOutputAdaptiveFastHighEscrowClearDrop;

        const bool contadoresEscrowMonotonos =
            AdaptTicksTotal >= AdaptSostenidoFastHighEscrowTicksInicio
            && AdaptFramesProducidosTotal
                >= AdaptSostenidoFastHighEscrowFramesPublicadosInicio
            && hostConsumed >= AdaptSostenidoFastHighEscrowConsInicio
            && framesAlConsumo
                >= AdaptSostenidoFastHighEscrowFramesConsumoInicio;
        if (!contadoresEscrowMonotonos)
        {
            razonesEscrow |=
                AudioOutputAdaptiveFastHighEscrowClearCounterRegression;
        }
        else
        {
            const u64 deltaEscrowCons = hostConsumed
                - AdaptSostenidoFastHighEscrowConsInicio;
            const u64 deltaEscrowFramesCons = framesAlConsumo
                - AdaptSostenidoFastHighEscrowFramesConsumoInicio;
            const u64 deltaEscrowFramesPub = AdaptFramesProducidosTotal
                - AdaptSostenidoFastHighEscrowFramesPublicadosInicio;
            const bool conservaEscrowCons = conservaPCM(
                deltaEscrowFramesCons,
                AdaptSostenidoFastHighEscrowNivelConsumoInicioLogico,
                deltaEscrowCons,
                nivelPostConsumoLogico);
            const bool conservaEscrowPub = conservaPCM(
                deltaEscrowFramesPub,
                AdaptSostenidoFastHighEscrowNivelEscrituraInicioLogico,
                deltaEscrowCons,
                nivelPostEscrituraLogico);
            if (!conservaEscrowCons || !conservaEscrowPub)
            {
                razonesEscrow |=
                    AudioOutputAdaptiveFastHighEscrowClearPcmConservation;
            }
            if (deltaEscrowFramesPub
                > std::numeric_limits<u64>::max()
                    - AdaptSostenidoFastHighEscrowGrantFrames)
            {
                razonesEscrow |=
                    AudioOutputAdaptiveFastHighEscrowClearCounterRange;
            }
            if (AdaptSostenidoFastHighEscrowGrantFrames == 0
                || AdaptSostenidoFastHighEscrowSpentFrames
                   > AdaptSostenidoFastHighEscrowGrantFrames)
            {
                razonesEscrow |=
                    AudioOutputAdaptiveFastHighEscrowClearCounterRange;
            }
            if (!std::isfinite(AdaptSostenidoFastHighEscrowGuardSkew)
                || AdaptSostenidoFastHighEscrowGuardSkew < 0.0)
            {
                razonesEscrow |=
                    AudioOutputAdaptiveFastHighEscrowClearReference;
            }
            if (AdaptSostenidoFastHighEscrowHorizonteNacimientoFrames == 0
                || AdaptSostenidoFastHighEscrowHorizonteRestanteFrames == 0
                || AdaptSostenidoFastHighEscrowHorizonteRestanteFrames
                   > AdaptSostenidoFastHighEscrowHorizonteNacimientoFrames)
            {
                razonesEscrow |=
                    AudioOutputAdaptiveFastHighEscrowClearCounterRange;
            }
        }
        const int fronteraEscrow = std::min(
            nivelSeguro,
            std::max(0, AdaptSostenidoPhaseObjetivoFrames));
        const bool escrowAgotado =
            AdaptSostenidoFastHighEscrowGrantFrames > 0
            && AdaptSostenidoFastHighEscrowSpentFrames
               >= AdaptSostenidoFastHighEscrowGrantFrames;
        const bool episodioEscrowCerrado = escrowAgotado
            && nivelPostConsumoLogico
               < static_cast<u64>(fronteraEscrow)
            && nivelPostEscrituraLogico
               < static_cast<u64>(fronteraEscrow);
        if (episodioEscrowCerrado)
        {
            razonesEscrow |=
                AudioOutputAdaptiveFastHighEscrowClearEpisodeExhausted;
        }
        if (razonesEscrow != AudioOutputAdaptiveFastHighEscrowClearNone)
        {

            fastHighEscrowInvalidadoEsteUpdate = true;
            limpiarFastHighEscrow(razonesEscrow);
            limpiarFastHighWitness();
        }
    }
    if (AdaptSostenidoFastHighWitnessActivo)
    {
        const int fronteraWitness = std::min(
            nivelSeguro,
            std::max(0, AdaptSostenidoPhaseObjetivoFrames));
        if (!AdaptSostenidoOwnerActivo
            || AdaptSostenidoFastHighWitnessOwnerGeneracion
               != AdaptSostenidoOwnerGeneracion
            || nivelPostConsumo < fronteraWitness
            || nivelPostEscritura < fronteraWitness)
        {
            limpiarFastHighWitness();
        }
    }
    const double produccionVentanaFast = ventanaFastPublicable
        ? ((double)deltaFastTicks * OutputSampleRate)
          / ((double)INTERNAL_SAMPLE_RATE * AdaptRatio)
        : 0.0;

    const bool demandaSuperaProduccionEntera = ventanaFastPublicable
        && std::isfinite(produccionVentanaFast)
        && (double)deltaFastCons > std::ceil(produccionVentanaFast);
    const bool cambioTargetPendienteAntes =
        AdaptFastCambioTargetPendiente;
    const bool cambioTargetCruzoIntervaloHostAntes =
        cambioTargetPendienteAntes
        && AdaptFastCambioTargetCruzoIntervaloHost;
    const bool fronteraPhaseContadoresMonotonos =
        ticksAlConsumo >= AdaptFastTicksInicio
        && hostConsumed >= AdaptFastConsInicio
        && framesAlConsumo >= AdaptFastFramesInicio
        && AdaptFramesProducidosTotal
           >= AdaptFastFramesPublicadosInicio;
    const u64 fronteraPhaseDeltaCons = fronteraPhaseContadoresMonotonos
        ? hostConsumed - AdaptFastConsInicio : 0;
    const u64 fronteraPhaseDeltaFramesCons =
        fronteraPhaseContadoresMonotonos
        ? framesAlConsumo - AdaptFastFramesInicio : 0;
    const u64 fronteraPhaseDeltaFramesPublicados =
        fronteraPhaseContadoresMonotonos
        ? AdaptFramesProducidosTotal
            - AdaptFastFramesPublicadosInicio
        : 0;
    const bool fronteraPhasePcmConsumoConservado =
        fronteraPhaseContadoresMonotonos
        && conservaPCM(
            fronteraPhaseDeltaFramesCons,
            AdaptFastNivelInicioLogico,
            fronteraPhaseDeltaCons,
            nivelPostConsumoLogico);
    const bool fronteraPhasePcmPublicacionConservado =
        fronteraPhaseContadoresMonotonos
        && conservaPCM(
            fronteraPhaseDeltaFramesPublicados,
            AdaptFastNivelEscrituraInicioLogico,
            fronteraPhaseDeltaCons,
            nivelPostEscrituraLogico);
    const u32 cambioTargetFronteraPhaseOrigenAntes =
        cambioTargetPendienteAntes
        ? AdaptFastCambioTargetFronteraPhaseOrigen
        : AudioOutputAdaptiveFastTargetChangePhaseFrontierNone;
    u32 cambioTargetFronteraPhaseRejectMask = 0;
    if (!cambioTargetPendienteAntes)
        cambioTargetFronteraPhaseRejectMask |=
            AudioOutputAdaptiveFastTargetChangePhaseFrontierRejectNoPending;
    if (!AdaptFastCambioTargetFronteraPhaseValida)
        cambioTargetFronteraPhaseRejectMask |=
            AudioOutputAdaptiveFastTargetChangePhaseFrontierRejectInvalid;
    if (targetRatePoseido)
        cambioTargetFronteraPhaseRejectMask |=
            AudioOutputAdaptiveFastTargetChangePhaseFrontierRejectTargetRateOwned;
    if (AdaptPhaseObjetivoActivo)
        cambioTargetFronteraPhaseRejectMask |=
            AudioOutputAdaptiveFastTargetChangePhaseFrontierRejectPhaseTargetActive;
    if (AdaptPhaseEpisodioActivo)
        cambioTargetFronteraPhaseRejectMask |=
            AudioOutputAdaptiveFastTargetChangePhaseFrontierRejectPhaseEpisodeActive;
    if (AdaptRateBajoConfirmado)
        cambioTargetFronteraPhaseRejectMask |=
            AudioOutputAdaptiveFastTargetChangePhaseFrontierRejectLowRateConfirmed;
    if (AdaptSostenidoCandidatoActivo)
        cambioTargetFronteraPhaseRejectMask |=
            AudioOutputAdaptiveFastTargetChangePhaseFrontierRejectCandidateActive;
    if (AdaptSostenidoOwnerActivo)
        cambioTargetFronteraPhaseRejectMask |=
            AudioOutputAdaptiveFastTargetChangePhaseFrontierRejectOwnerActive;
    if (AdaptFastAnchorOwnerValido)
        cambioTargetFronteraPhaseRejectMask |=
            AudioOutputAdaptiveFastTargetChangePhaseFrontierRejectAnchorOwnerValid;
    if (AdaptFastAnchorOwnerGeneracion != 0)
        cambioTargetFronteraPhaseRejectMask |=
            AudioOutputAdaptiveFastTargetChangePhaseFrontierRejectAnchorOwnerGeneration;
    if (AdaptFastCandidatoGeneracionInicio
        != AdaptSostenidoCandidatoGeneracion)
    {
        cambioTargetFronteraPhaseRejectMask |=
            AudioOutputAdaptiveFastTargetChangePhaseFrontierRejectCandidateGeneration;
    }
    if (continuidadEpoch != AdaptFastContinuidadEpochInicio)
        cambioTargetFronteraPhaseRejectMask |=
            AudioOutputAdaptiveFastTargetChangePhaseFrontierRejectContinuity;
    if (resetSolicitado != AdaptFastResetEpochInicio)
        cambioTargetFronteraPhaseRejectMask |=
            AudioOutputAdaptiveFastTargetChangePhaseFrontierRejectResetRequested;
    if (resetConfirmado != AdaptFastResetEpochInicio)
        cambioTargetFronteraPhaseRejectMask |=
            AudioOutputAdaptiveFastTargetChangePhaseFrontierRejectResetConfirmed;
    if (AdaptHintVistoProductor != AdaptFastHintSeenEpochInicio)
        cambioTargetFronteraPhaseRejectMask |=
            AudioOutputAdaptiveFastTargetChangePhaseFrontierRejectHint;
    if (intervaloHostPerdido)
        cambioTargetFronteraPhaseRejectMask |=
            AudioOutputAdaptiveFastTargetChangePhaseFrontierRejectHostInterval;
    if (dropsEstaEscritura > 0)
        cambioTargetFronteraPhaseRejectMask |=
            AudioOutputAdaptiveFastTargetChangePhaseFrontierRejectDrop;
    if (!fronteraPhaseContadoresMonotonos)
        cambioTargetFronteraPhaseRejectMask |=
            AudioOutputAdaptiveFastTargetChangePhaseFrontierRejectCounterRegression;
    else
    {

        if (!fronteraPhasePcmConsumoConservado)
            cambioTargetFronteraPhaseRejectMask |=
                AudioOutputAdaptiveFastTargetChangePhaseFrontierRejectConsumedPcm;
        if (!fronteraPhasePcmPublicacionConservado)
            cambioTargetFronteraPhaseRejectMask |=
                AudioOutputAdaptiveFastTargetChangePhaseFrontierRejectPublishedPcm;
    }
    const bool cambioTargetFronteraPhaseElegible =
        cambioTargetPendienteAntes
        && AdaptFastCambioTargetFronteraPhaseValida
        && !targetRatePoseido
        && !AdaptPhaseObjetivoActivo
        && !AdaptPhaseEpisodioActivo
        && !AdaptRateBajoConfirmado
        && !AdaptSostenidoCandidatoActivo
        && !AdaptSostenidoOwnerActivo
        && !AdaptFastAnchorOwnerValido
        && AdaptFastAnchorOwnerGeneracion == 0
        && AdaptFastCandidatoGeneracionInicio
           == AdaptSostenidoCandidatoGeneracion
        && continuidadEpoch == AdaptFastContinuidadEpochInicio
        && resetSolicitado == AdaptFastResetEpochInicio
        && resetConfirmado == AdaptFastResetEpochInicio
        && AdaptHintVistoProductor == AdaptFastHintSeenEpochInicio
        && !intervaloHostPerdido
        && dropsEstaEscritura == 0
        && fronteraPhaseContadoresMonotonos
        && conservaPCM(
            fronteraPhaseDeltaFramesCons,
            AdaptFastNivelInicioLogico,
            fronteraPhaseDeltaCons,
            nivelPostConsumoLogico)
        && conservaPCM(
            fronteraPhaseDeltaFramesPublicados,
            AdaptFastNivelEscrituraInicioLogico,
            fronteraPhaseDeltaCons,
            nivelPostEscrituraLogico);
    const bool cambioTargetEscrowWriteInicialElegible =
        cambioTargetPendienteAntes
        && cambioTargetCruzoIntervaloHostAntes
        && cambioTargetFronteraPhaseOrigenAntes
           == AudioOutputAdaptiveFastTargetChangePhaseFrontierFastWindow
        && !AdaptFastCambioTargetFronteraPhaseValida
        && !targetRatePoseido
        && !AdaptPhaseObjetivoActivo
        && AdaptPhaseEpisodioActivo
        && !AdaptRateBajoConfirmado
        && !AdaptSostenidoCandidatoActivo
        && !AdaptSostenidoOwnerActivo
        && !AdaptFastAnchorOwnerValido
        && AdaptFastAnchorOwnerGeneracion == 0
        && AdaptFastCandidatoGeneracionInicio
           == AdaptSostenidoCandidatoGeneracion
        && continuidadEpoch == AdaptFastContinuidadEpochInicio
        && resetSolicitado == AdaptFastResetEpochInicio
        && resetConfirmado == AdaptFastResetEpochInicio
        && AdaptHintVistoProductor == AdaptFastHintSeenEpochInicio
        && !intervaloHostPerdido
        && dropsEstaEscritura == 0
        && deltaActControl > 0.0
        && fronteraPhaseContadoresMonotonos
        && fronteraPhasePcmConsumoConservado
        && fronteraPhasePcmPublicacionConservado
        && nivelPostConsumo >= 0
        && nivelPostEscritura > nivelPostConsumo
        && nivelPostEscritura <= nivelSeguro

        && nivelPostConsumoLogico
           == static_cast<u64>(nivelPostConsumo)
        && nivelPostEscrituraLogico
           == static_cast<u64>(nivelPostEscritura);
    const bool cambioTargetPhaseProvisionalP3Elegible =
        cambioTargetPendienteAntes
        && cambioTargetFronteraPhaseOrigenAntes
           == AudioOutputAdaptiveFastTargetChangePhaseFrontierFastWindow

        && !AdaptFastCambioTargetFronteraPhaseValida
        && cambioTargetCruzoIntervaloHostAntes
        && intervaloHostPerdido
        && !targetRatePoseido
        && !AdaptPhaseObjetivoActivo
        && !AdaptRateBajoConfirmado
        && !AdaptSostenidoCandidatoActivo
        && !AdaptSostenidoOwnerActivo
        && !AdaptFastAnchorOwnerValido
        && AdaptFastAnchorOwnerGeneracion == 0
        && AdaptFastCandidatoGeneracionInicio
           == AdaptSostenidoCandidatoGeneracion
        && continuidadEpoch == AdaptFastContinuidadEpochInicio
        && resetSolicitado == AdaptFastResetEpochInicio
        && resetConfirmado == AdaptFastResetEpochInicio
        && AdaptHintVistoProductor == AdaptFastHintSeenEpochInicio
        && dropsEstaEscritura == 0
        && fronteraPhaseContadoresMonotonos
        && fronteraPhasePcmConsumoConservado
        && fronteraPhasePcmPublicacionConservado;
    const int cambioTargetFronteraPhaseFrames = AdaptFastNivelInicio;
    const u64 deltaHintCertCons =
        hostConsumed - AdaptHintCertConsInicio;
    const u64 deltaHintCertTicks =
        ticksAlConsumo - AdaptHintCertTicksInicio;
    const bool ventanaHintCertPublicable =
        deltaHintCertCons >= ventanaRate
        && deltaHintCertTicks >= ticksDosFramesPublicables;
    const bool hintFinitoCertificable =
        AdaptHint > 0.0 && std::isfinite(AdaptHint);
    const double produccionVentanaHint = hintFinitoCertificable
        && ventanaHintCertPublicable
        ? ((double)deltaHintCertTicks * OutputSampleRate)
          / ((double)INTERNAL_SAMPLE_RATE * AdaptHint)
        : 0.0;
    const bool demandaHintSuperaProduccionEntera =
        ventanaHintCertPublicable
        && std::isfinite(produccionVentanaHint)
        && (double)deltaHintCertCons
           > std::ceil(produccionVentanaHint);
    const bool colaEnBanda = nivelPostConsumo >= nivelBajo
                          && nivelPostConsumo <= nivelAlto
                          && nivelPostEscritura <= nivelAlto;
    const bool ventanaCertificaHint =
        !AdaptHintPadreCertificado
        && !AdaptSostenidoCandidatoActivo
        && !AdaptSostenidoOwnerActivo
        && hintFinitoCertificable
        && !AdaptProbe
        && AdaptRatio == AdaptHint
        && !AdaptRateActuadorPendiente
        && !AdaptRateRollbackPendiente
        && !AdaptFastLowPendiente
        && !AdaptFastCambioTargetPendiente
        && !AdaptHintCertRebasePostDropPendiente
        && ventanaHintCertPublicable
        && !urgenciaFast
        && (!ventanaFastPublicable
            || !demandaSuperaProduccionEntera)
        && !intervaloHostPerdido
        && dropsEstaEscritura == 0
        && !demandaHintSuperaProduccionEntera
        && colaEnBanda;
    if (ventanaCertificaHint)
    {

        certificarHintPadre(deltaHintCertCons, deltaHintCertTicks);
        rebasarVentanaFast(true);
        AdaptSlowTicksInicio = ticksAlConsumo;
        AdaptSlowConsInicio = hostConsumed;
    }
    const bool ventanaCambioTarget = targetCambioRateObservable
                                  && ventanaFastPublicable
                                  && (cambioTargetPendienteAntes
                                      || (colaEnBanda
                                          && demandaSuperaProduccionEntera));
    const double deficitVentanaFast = ventanaCambioTarget
        ? medirDeficit(deltaFastTicks, deltaFastCons)
        : 0.0;
    const double presupuestoVentanaFast = ventanaCambioTarget
        ? medirPresupuestoTransitorio()
        : 0.0;
    const bool deficitAgotaReserva = ventanaCambioTarget
                                  && demandaSuperaProduccionEntera
                                  && std::isfinite(deficitVentanaFast)
                                  && deficitVentanaFast
                                     > presupuestoVentanaFast;
    const bool deficitRespaldadoPorPCM = ventanaCambioTarget
                                      && std::isfinite(deficitVentanaFast)
                                      && deficitVentanaFast
                                         < respaldoRateElegible;
    const bool ventanaInicialConservada =
        !intervaloHostPerdido && dropsEstaEscritura == 0;
    const double respaldoSiguienteObservacionInicial =
        deficitVentanaFast + presupuestoVentanaFast;
    const bool siguienteObservacionInicialRespaldada =
        std::isfinite(respaldoSiguienteObservacionInicial)
        && respaldoSiguienteObservacionInicial
           < respaldoRateElegible;
    const bool iniciarCambioTarget = ventanaCambioTarget
                                  && !cambioTargetPendienteAntes
                                  && demandaSuperaProduccionEntera
                                  && std::isfinite(deficitVentanaFast)
                                  && (deficitVentanaFast
                                         <= presupuestoVentanaFast
                                      || (deficitRespaldadoPorPCM
                                          && (!ventanaInicialConservada
                                              || siguienteObservacionInicialRespaldada)));
    if (AdaptProbe)
        decisionFlags |= AudioOutputAdaptiveProbeAtFastGate;
    if (nivelFuera)
        decisionFlags |= AudioOutputAdaptiveLevelOutside;
    if (ventanaFast || ventanaCambioTarget || urgenciaSostenidaBaja)
        decisionFlags |= AudioOutputAdaptiveFastWindow;
    if (urgenciaFast)
        decisionFlags |= AudioOutputAdaptiveFastUrgency;
    if (nivelPostEscritura >= nivelSeguro)
        decisionFlags |= AudioOutputAdaptiveLevelAtOrAboveSafe;
    const double ratioAntesEstimadores = AdaptRatio;
    const bool rateBajoAntesEstimadores = AdaptRateBajoConfirmado;
    const bool ratePadreCertificadoAntesEstimadores =
        AdaptRatePadreCertificado;
    const bool hintPadreCertificadoAntesEstimadores =
        AdaptHintPadreCertificado;
    const bool rebaseFaseAntesEstimadores = AdaptPhaseRebasePendiente;
    const bool rebaseFaseOrigenRateAntes =
        AdaptPhaseOrigenRecuperacionRate;
    bool ratioFastAceptado = false;
    bool conservarVentanaFast = false;
    bool fastTargetChangePhaseFrontierBornThisUpdate = false;
    bool estimadorFastEvaluado = false;
    bool fastLowConfirmadoSinRespaldoFuturo = false;
    bool fastLowOwnerFreePhaseProvisional = false;
    double fastLowOwnerFreePhaseObjetivo = 0.0;
    bool fastOverrideSostenidoAceptado = false;
    bool fastOverrideSostenidoAscendente = false;
    u64 fastOverrideSostenidoTicks = 0;
    u64 fastOverrideSostenidoCons = 0;
    bool observacionAltaSostenida = false;
    double ratioFastObservable = 0.0;
    if (!ventanaCertificaHint
        && (ventanaFast || urgenciaFast || ventanaCambioTarget
            || urgenciaSostenidaBaja))
    {
        ratioFastObservable = medirRatio(deltaFastTicks, deltaFastCons);
        const bool recuperacionFastAscendenteObservable =
            targetRatePoseido
            && ventanaFast
            && !intervaloHostPerdido
            && dropsEstaEscritura == 0
            && std::isfinite(ratioFastObservable)
            && ratioFastObservable > AdaptRatio
            && nivelPostConsumo >= nivelRecuperacionAscendente
            && nivelPostEscritura >= nivelRecuperacionAscendente;
        if (AdaptProbe || AdaptFastLowPendiente
            || AdaptSostenidoFastLowWitnessActivo || nivelFuera
            || nivelPostEscritura >= nivelSeguro
            || ventanaCambioTarget
            || urgenciaSostenidaBaja
            || recuperacionFastAscendenteObservable)
        {
            estimadorFastEvaluado = true;
            double ratio = ratioFastObservable;
            decisionFlags |= AudioOutputAdaptiveEstimatorFast;
            telemetry.estimatorDeltaConsumed = deltaFastCons;
            telemetry.estimatorDeltaTicks = deltaFastTicks;
            telemetry.estimatorRawRatio = ratio;
            if (std::isfinite(ratio) && ratio > 0.0)
            {
                decisionFlags |=
                    AudioOutputAdaptiveMeasurementValid;

                bool aceptar = true;
                if (cambioTargetPendienteAntes)
                {
                    const double deficit =
                        medirDeficit(deltaFastTicks, deltaFastCons);
                    const double respaldoSiguienteObservacion =
                        deficit + presupuestoVentanaFast;
                    const bool refinamientoRateConfirmado =
                        AdaptRateBajoConfirmado
                        && ratio < AdaptRatio
                        && !intervaloHostPerdido
                        && !AdaptPhaseEpisodioActivo
                        && dropsEstaEscritura == 0;
                    const bool siguienteObservacionRespaldada =
                        std::isfinite(respaldoSiguienteObservacion)
                        && respaldoSiguienteObservacion
                           < respaldoRateElegible;
                    const bool colaRecuperada =
                        nivelPostConsumo >= nivelRecuperacionPostConsumo
                        && nivelPostEscritura >= objetivoRateCausal;

                    if (colaRecuperada
                        || !demandaSuperaProduccionEntera)
                    {

                        aceptar = false;
                        AdaptFastLowPendiente = false;
                        AdaptFastCambioTargetPendiente = false;
                        decisionFlags |=
                            AudioOutputAdaptiveLowRejectedByLevel;
                        AdaptSlowTicksInicio = ticksAlConsumo;
                        AdaptSlowConsInicio = hostConsumed;
                    }
                    else if (std::isfinite(deficit)
                             && deficitRespaldadoPorPCM
                             && (!refinamientoRateConfirmado
                                 || siguienteObservacionRespaldada))
                    {

                        aceptar = false;
                        AdaptFastCambioTargetPendiente = true;
                        conservarVentanaFast = true;
                        decisionFlags |=
                            AudioOutputAdaptiveLowDeferred;
                        AdaptSlowTicksInicio = ticksAlConsumo;
                        AdaptSlowConsInicio = hostConsumed;
                    }
                    else
                    {
                        AdaptFastLowPendiente = false;
                        AdaptFastCambioTargetPendiente = false;
                        decisionFlags |=
                            AudioOutputAdaptiveLowConfirmed;
                    }
                }
                else if (targetRatePoseido
                         && ratio > AdaptRatio
                         && (nivelPostConsumo
                             < nivelRecuperacionAscendente
                             || nivelPostEscritura
                                < nivelRecuperacionAscendente))
                {

                    aceptar = false;
                    AdaptFastLowPendiente = false;
                    AdaptFastCambioTargetPendiente = false;
                    decisionFlags |=
                        AudioOutputAdaptiveLowRejectedByLevel;
                    AdaptSlowTicksInicio = ticksAlConsumo;
                    AdaptSlowConsInicio = hostConsumed;
                }
                else if (AdaptFastLowPendiente
                         || (AdaptHint > 0.0 && ratio < AdaptRatio))
                {

                    const double deficit =
                        medirDeficit(deltaFastTicks, deltaFastCons);
                    const double presupuestoTransitorio =
                        medirPresupuestoTransitorio();
                    const bool refinamientoRateConfirmadoOwnerFree =
                        targetRatePoseido
                        && AdaptRateBajoConfirmado
                        && !AdaptSostenidoOwnerActivo
                        && ratio < AdaptRatio
                        && !intervaloHostPerdido
                        && dropsEstaEscritura == 0;
                    const double respaldoSiguienteRefinamiento =
                        deficit + presupuestoTransitorio;
                    const bool siguienteRefinamientoRespaldado =
                        std::isfinite(respaldoSiguienteRefinamiento)
                        && respaldoSiguienteRefinamiento
                           < respaldoRateElegible;

                    if (iniciarCambioTarget)
                    {

                        aceptar = false;
                        AdaptFastLowPendiente = false;
                        AdaptFastCambioTargetPendiente = true;
                        AdaptFastCambioTargetFronteraPhaseValida = false;
                        AdaptFastCambioTargetFronteraPhaseOrigen =
                            AudioOutputAdaptiveFastTargetChangePhaseFrontierFastWindow;
                        fastTargetChangePhaseFrontierBornThisUpdate = true;
                        decisionFlags |=
                            AudioOutputAdaptiveLowDeferred;
                        AdaptSlowTicksInicio = ticksAlConsumo;
                        AdaptSlowConsInicio = hostConsumed;
                    }
                    else if (!deficitAgotaReserva
                             && nivelPostConsumo
                                >= nivelRecuperacionPostConsumo
                             && nivelPostEscritura >= objetivoRateCausal)
                    {

                        aceptar = false;
                        AdaptFastLowPendiente = false;
                        decisionFlags |=
                            AudioOutputAdaptiveLowRejectedByLevel;
                        AdaptSlowTicksInicio = ticksAlConsumo;
                        AdaptSlowConsInicio = hostConsumed;
                    }
                    else if (std::isfinite(deficit)
                             && deficit <= presupuestoTransitorio
                             && (!refinamientoRateConfirmadoOwnerFree
                                 || siguienteRefinamientoRespaldado))
                    {
                        aceptar = false;
                        AdaptFastLowPendiente = true;
                        conservarVentanaFast = true;
                        decisionFlags |=
                            AudioOutputAdaptiveLowDeferred;

                        AdaptSlowTicksInicio = ticksAlConsumo;
                        AdaptSlowConsInicio = hostConsumed;
                    }
                    else
                    {
                        AdaptFastLowPendiente = false;
                        fastLowConfirmadoSinRespaldoFuturo =
                            refinamientoRateConfirmadoOwnerFree
                            && !siguienteRefinamientoRespaldado;
                        decisionFlags |=
                            AudioOutputAdaptiveLowConfirmed;
                    }
                }
                else
                {
                    AdaptFastLowPendiente = false;
                    AdaptFastCambioTargetPendiente = false;
                }

                if (AdaptSostenidoOwnerActivo)
                {
                    const double produccionOwner = produccionIntegral(
                        deltaFastTicks, AdaptSostenidoOwnerRatio);
                    const int fronteraOwnerSostenido = std::min(
                        nivelSeguro,
                        std::max(0, AdaptSostenidoPhaseObjetivoFrames));
                    const bool lowWitnessActivoAntes =
                        AdaptSostenidoFastLowWitnessActivo
                        && AdaptSostenidoFastLowWitnessOwnerGeneracion
                           == AdaptSostenidoOwnerGeneracion;
                    const bool fastAnchorOwnerVigente =
                        AdaptFastAnchorOwnerValido
                        && AdaptFastAnchorOwnerGeneracion
                           == AdaptSostenidoOwnerGeneracion
                        && ticksAlConsumo >= AdaptFastTicksInicio
                        && hostConsumed >= AdaptFastConsInicio
                        && framesAlConsumo >= AdaptFastFramesInicio;
                    const bool fastAnchorConservaPCM =
                        fastAnchorOwnerVigente
                        && conservaPCM(
                            framesAlConsumo - AdaptFastFramesInicio,
                            AdaptFastNivelInicioLogico,
                            hostConsumed - AdaptFastConsInicio,
                            nivelPostConsumoLogico);
                    const bool p3CierraFastLowTipado =
                        intervaloHostPerdido
                        && !lowWitnessActivoAntes
                        && deltaFastCons >= ventanaRate
                        && deltaFastTicks >= ticksDosFramesPublicables
                        && fastAnchorConservaPCM;
                    const bool observacionBajaSostenida =
                        ratioFastObservable < AdaptSostenidoOwnerRatio
                        && (!intervaloHostPerdido
                            || lowWitnessActivoAntes
                            || p3CierraFastLowTipado)
                        && dropsEstaEscritura == 0
                        && nivelPostConsumo < nivelSeguro
                        && nivelPostEscritura < nivelSeguro;
                    const bool lowWitnessAvanzo = lowWitnessActivoAntes
                        && ticksAlConsumo
                             - AdaptSostenidoFastLowWitnessTicksFin
                           >= ticksDosFramesPublicables
                        && hostConsumed
                             - AdaptSostenidoFastLowWitnessConsFin
                           >= ventanaRate
                        && framesAlConsumo
                           >= AdaptSostenidoFastLowWitnessFramesFin;
                    const bool lowWitnessConservaPCM = lowWitnessAvanzo
                        && conservaPCM(
                            framesAlConsumo
                                - AdaptSostenidoFastLowWitnessFramesFin,
                            AdaptSostenidoFastLowWitnessNivelFinLogico,
                            hostConsumed
                                - AdaptSostenidoFastLowWitnessConsFin,
                            nivelPostConsumoLogico);
                    const bool lowWitnessNacimientoMonotono =
                        lowWitnessActivoAntes
                        && ticksAlConsumo
                           >= AdaptSostenidoFastLowWitnessTicksNacimiento
                        && hostConsumed
                           >= AdaptSostenidoFastLowWitnessConsNacimiento
                        && framesAlConsumo
                           >= AdaptSostenidoFastLowWitnessFramesNacimiento;
                    const bool lowWitnessNacimientoConservaPCM =
                        lowWitnessNacimientoMonotono
                        && conservaPCM(
                            framesAlConsumo
                                - AdaptSostenidoFastLowWitnessFramesNacimiento,
                            AdaptSostenidoFastLowWitnessNivelNacimientoLogico,
                            hostConsumed
                                - AdaptSostenidoFastLowWitnessConsNacimiento,
                            nivelPostConsumoLogico);
                    const bool lowWitnessFinMonotonoDesdeNacimiento =
                        lowWitnessActivoAntes
                        && AdaptSostenidoFastLowWitnessTicksFin
                           >= AdaptSostenidoFastLowWitnessTicksNacimiento
                        && AdaptSostenidoFastLowWitnessConsFin
                           >= AdaptSostenidoFastLowWitnessConsNacimiento
                        && AdaptSostenidoFastLowWitnessFramesFin
                           >= AdaptSostenidoFastLowWitnessFramesNacimiento;
                    const u64 lowWitnessFinDeltaTicks =
                        lowWitnessFinMonotonoDesdeNacimiento
                        ? AdaptSostenidoFastLowWitnessTicksFin
                              - AdaptSostenidoFastLowWitnessTicksNacimiento
                        : 0;
                    const u64 lowWitnessFinDeltaCons =
                        lowWitnessFinMonotonoDesdeNacimiento
                        ? AdaptSostenidoFastLowWitnessConsFin
                              - AdaptSostenidoFastLowWitnessConsNacimiento
                        : 0;
                    const u64 lowWitnessFinDeltaFrames =
                        lowWitnessFinMonotonoDesdeNacimiento
                        ? AdaptSostenidoFastLowWitnessFramesFin
                              - AdaptSostenidoFastLowWitnessFramesNacimiento
                        : 0;
                    const bool lowWitnessFinPublicableDesdeNacimiento =
                        lowWitnessFinMonotonoDesdeNacimiento
                        && lowWitnessFinDeltaTicks
                           >= ticksDosFramesPublicables
                        && lowWitnessFinDeltaCons >= ventanaRate;
                    const bool lowWitnessFinConservaPCMDesdeNacimiento =
                        lowWitnessFinPublicableDesdeNacimiento
                        && conservaPCM(
                            lowWitnessFinDeltaFrames,
                            AdaptSostenidoFastLowWitnessNivelNacimientoLogico,
                            lowWitnessFinDeltaCons,
                            AdaptSostenidoFastLowWitnessNivelFinLogico);
                    const bool lowWitnessFinEsLowDesdeNacimiento =
                        lowWitnessFinPublicableDesdeNacimiento
                        && compararRelacionHT(
                            lowWitnessFinDeltaTicks,
                            lowWitnessFinDeltaCons,
                            AdaptSostenidoOwnerTicks,
                            AdaptSostenidoOwnerCons) < 0;
                    const bool lowWitnessProgresoDespuesDelFin =
                        lowWitnessActivoAntes
                        && ticksAlConsumo
                           > AdaptSostenidoFastLowWitnessTicksFin
                        && hostConsumed
                           > AdaptSostenidoFastLowWitnessConsFin
                        && framesAlConsumo
                           > AdaptSostenidoFastLowWitnessFramesFin;
                    const u64 lowWitnessNacimientoDeltaTicks =
                        lowWitnessNacimientoMonotono
                        ? ticksAlConsumo
                              - AdaptSostenidoFastLowWitnessTicksNacimiento
                        : 0;
                    const u64 lowWitnessNacimientoDeltaCons =
                        lowWitnessNacimientoMonotono
                        ? hostConsumed
                              - AdaptSostenidoFastLowWitnessConsNacimiento
                        : 0;
                    const bool lowWitnessNacimientoPublicable =
                        lowWitnessNacimientoMonotono
                        && lowWitnessNacimientoDeltaTicks
                           >= ticksDosFramesPublicables
                        && lowWitnessNacimientoDeltaCons >= ventanaRate;
                    const bool lowWitnessNacimientoEsLow =
                        lowWitnessNacimientoPublicable
                        && compararRelacionHT(
                            lowWitnessNacimientoDeltaTicks,
                            lowWitnessNacimientoDeltaCons,
                            AdaptSostenidoOwnerTicks,
                            AdaptSostenidoOwnerCons) < 0;
                    const double produccionOwnerDesdeNacimiento =
                        lowWitnessNacimientoPublicable
                        ? produccionIntegral(
                            lowWitnessNacimientoDeltaTicks,
                            AdaptSostenidoOwnerRatio)
                        : -1.0;
                    const bool lowWitnessNacimientoCierraLiveness =
                        lowWitnessFinConservaPCMDesdeNacimiento
                        && lowWitnessFinEsLowDesdeNacimiento
                        && lowWitnessProgresoDespuesDelFin
                        && lowWitnessNacimientoConservaPCM
                        && lowWitnessNacimientoEsLow
                        && produccionOwnerDesdeNacimiento >= 0.0
                        && (double)lowWitnessNacimientoDeltaCons
                           > produccionOwnerDesdeNacimiento
                             + (double)std::max(0, nivelPostConsumo)
                        && dropsEstaEscritura == 0
                        && nivelPostConsumo < nivelSeguro
                        && nivelPostEscritura < nivelSeguro;
                    const bool deficitOwnerAgotaConsumo =
                        observacionBajaSostenida
                        && produccionOwner >= 0.0
                        && (double)deltaFastCons
                           > produccionOwner
                             + (double)std::max(0, nivelPostConsumo);
                    const bool overrideBajoVigente =
                        AdaptSostenidoOverrideActivo
                        && AdaptSostenidoOverrideDireccion < 0;
                    bool overrideBajo =
                        !overrideBajoVigente
                        && ((deficitOwnerAgotaConsumo
                             && ((lowWitnessActivoAntes
                                  && lowWitnessAvanzo
                                  && lowWitnessConservaPCM)
                                 || (!lowWitnessActivoAntes
                                     && !intervaloHostPerdido
                                     && nivelPostEscritura
                                        < fronteraOwnerSostenido)))
                            || lowWitnessNacimientoCierraLiveness);
                    const bool lowWitnessRecuperoFrontera =
                        lowWitnessActivoAntes
                        && lowWitnessAvanzo
                        && lowWitnessConservaPCM
                        && !deficitOwnerAgotaConsumo
                        && nivelPostConsumo
                           >= AdaptSostenidoFastLowWitnessNivelFin
                        && nivelPostConsumo >= fronteraOwnerSostenido
                        && nivelPostEscritura >= fronteraOwnerSostenido;
                    const bool lowWitnessRefutadoEsteUpdate =
                        lowWitnessActivoAntes
                        && (!observacionBajaSostenida
                            || (lowWitnessAvanzo
                                && !lowWitnessConservaPCM)
                            || lowWitnessRecuperoFrontera);
                    observacionAltaSostenida =
                        compararRelacionHT(
                            deltaFastTicks, deltaFastCons,
                            AdaptSostenidoOwnerTicks,
                            AdaptSostenidoOwnerCons) > 0
                        && nivelPostEscritura >= nivelSeguro;
                    const bool overrideAltoVigente =
                        AdaptSostenidoOverrideActivo
                        && AdaptSostenidoOverrideDireccion > 0;
                    const bool overrideAltoTieneIdentidad =
                        overrideAltoVigente
                        && AdaptSostenidoOverrideTicks > 0
                        && AdaptSostenidoOverrideCons > 0;
                    const u64 referenciaAltoTicks = overrideAltoVigente
                        ? AdaptSostenidoOverrideTicks
                        : AdaptSostenidoOwnerTicks;
                    const u64 referenciaAltoCons = overrideAltoVigente
                        ? AdaptSostenidoOverrideCons
                        : AdaptSostenidoOwnerCons;
                    const bool referenciaAltoDisponible =
                        referenciaAltoTicks > 0
                        && referenciaAltoCons > 0
                        && (!overrideAltoVigente
                            || overrideAltoTieneIdentidad);
                    const bool endpointFastAltoRelacion =
                        referenciaAltoDisponible
                        && compararRelacionHT(
                            deltaFastTicks, deltaFastCons,
                            referenciaAltoTicks,
                            referenciaAltoCons) > 0;
                    const bool endpointFastAltoArmable =
                        endpointFastAltoRelacion
                        && !intervaloHostPerdido
                        && dropsEstaEscritura == 0;
                    const bool endpointFastAltoPreSeguro =
                        endpointFastAltoArmable
                        && nivelPostConsumo < nivelSeguro
                        && nivelPostEscritura < nivelSeguro;
                    const bool endpointFastAltoSobreSeguro =
                        endpointFastAltoArmable
                        && nivelPostEscritura >= nivelSeguro;
                    const bool endpointFastAltoAgotaCapacidad =
                        endpointFastAltoSobreSeguro
                        && cambioSostenidoAgotaCapacidadExacta(
                            deltaFastTicks,
                            deltaFastCons,
                            nivelPostEscritura,
                            ventanaLenta,
                            referenciaAltoTicks,
                            referenciaAltoCons);
                    if (overrideBajo)
                    {
                        if (lowWitnessActivoAntes
                            && ((lowWitnessAvanzo
                                 && lowWitnessConservaPCM)
                                || lowWitnessNacimientoCierraLiveness)
                            && lowWitnessNacimientoConservaPCM)
                        {

                            fastOverrideSostenidoTicks =
                                ticksAlConsumo
                                - AdaptSostenidoFastLowWitnessTicksNacimiento;
                            fastOverrideSostenidoCons =
                                hostConsumed
                                - AdaptSostenidoFastLowWitnessConsNacimiento;
                            ratio = medirRatio(
                                fastOverrideSostenidoTicks,
                                fastOverrideSostenidoCons);
                        }
                        else
                        {
                            fastOverrideSostenidoTicks = deltaFastTicks;
                            fastOverrideSostenidoCons = deltaFastCons;
                        }

                        overrideBajo =
                            fastOverrideSostenidoTicks > 0
                            && fastOverrideSostenidoCons > 0
                            && compararRelacionHT(
                                fastOverrideSostenidoTicks,
                                fastOverrideSostenidoCons,
                                AdaptSostenidoOwnerTicks,
                                AdaptSostenidoOwnerCons) < 0;
                        if (overrideBajo)
                        {

                            limpiarFastHighWitness();
                        }
                    }
                    else if (!lowWitnessRefutadoEsteUpdate)
                    {
                        const bool referenciaWitnessVigente =
                            AdaptSostenidoFastHighWitnessActivo
                            && AdaptSostenidoFastHighWitnessOwnerGeneracion
                               == AdaptSostenidoOwnerGeneracion
                            && AdaptSostenidoFastHighWitnessReferenciaTicks
                               == referenciaAltoTicks
                            && AdaptSostenidoFastHighWitnessReferenciaCons
                               == referenciaAltoCons
                            && AdaptSostenidoFastHighWitnessReferenciaEsOverride
                               == overrideAltoVigente;
                        const bool contadoresWitnessMonotonos =
                            AdaptSostenidoFastHighWitnessActivo
                            && ticksAlConsumo
                               >= AdaptSostenidoFastHighWitnessTicksInicio
                            && hostConsumed
                               >= AdaptSostenidoFastHighWitnessConsInicio
                            && framesAlConsumo
                               >= AdaptSostenidoFastHighWitnessFramesInicio;
                        bool witnessLineageInvalidadoEsteUpdate =
                            fastHighEscrowInvalidadoEsteUpdate;
                        if (AdaptSostenidoFastHighWitnessActivo
                            && (!referenciaWitnessVigente
                                || !contadoresWitnessMonotonos
                                || intervaloHostPerdido
                                || dropsEstaEscritura > 0))
                        {
                            limpiarFastHighWitness();
                            witnessLineageInvalidadoEsteUpdate = true;
                        }

                        if (AdaptSostenidoFastHighWitnessActivo)
                        {
                            const int fronteraOwner = std::min(
                                nivelSeguro,
                                std::max(
                                    0, AdaptSostenidoPhaseObjetivoFrames));
                            const bool perdioFronteraAlta =
                                nivelPostConsumo < fronteraOwner
                                || nivelPostEscritura < fronteraOwner;
                            const bool episodioFillRevertido =
                                nivelPostConsumo
                                    < AdaptSostenidoFastHighWitnessUltimoNivelConsumo
                                && nivelPostEscritura
                                    < AdaptSostenidoFastHighWitnessUltimoNivelEscritura;
                            const u64 witnessDeltaTicks =
                                ticksAlConsumo
                                - AdaptSostenidoFastHighWitnessTicksInicio;
                            const u64 witnessDeltaCons =
                                hostConsumed
                                - AdaptSostenidoFastHighWitnessConsInicio;
                            const u64 witnessDeltaFrames =
                                framesAlConsumo
                                - AdaptSostenidoFastHighWitnessFramesInicio;
                            const bool witnessConservaPCM = conservaPCM(
                                witnessDeltaFrames,
                                AdaptSostenidoFastHighWitnessNivelInicioLogico,
                                witnessDeltaCons,
                                nivelPostConsumoLogico);
                            const bool witnessPublicable =
                                witnessDeltaCons >= ventanaRate
                                && witnessDeltaTicks >= ticksDosFrames
                                && nivelPostConsumo
                                   > AdaptSostenidoFastHighWitnessNivelInicio;
                            const bool witnessReversionPublicable =
                                episodioFillRevertido
                                && witnessDeltaCons >= ventanaRate
                                && witnessDeltaTicks >= ticksDosFrames
                                && witnessConservaPCM;
                            const bool witnessSuperaReferencia =
                                witnessPublicable
                                && compararRelacionHT(
                                    witnessDeltaTicks,
                                    witnessDeltaCons,
                                    AdaptSostenidoFastHighWitnessReferenciaTicks,
                                    AdaptSostenidoFastHighWitnessReferenciaCons)
                                   > 0;
                            const bool witnessAgotaCapacidad =
                                witnessPublicable
                                && endpointFastAltoSobreSeguro
                                && cambioSostenidoAgotaCapacidadExacta(
                                    witnessDeltaTicks,
                                    witnessDeltaCons,
                                    nivelPostEscritura,
                                    ventanaLenta,
                                    AdaptSostenidoFastHighWitnessReferenciaTicks,
                                    AdaptSostenidoFastHighWitnessReferenciaCons);
                            const bool witnessCapacidadTrasEndpoint =
                                AdaptSostenidoFastHighWitnessCapacidad
                                || endpointFastAltoAgotaCapacidad
                                || witnessAgotaCapacidad;
                            if (witnessConservaPCM
                                && !perdioFronteraAlta
                                && !episodioFillRevertido
                                && endpointFastAltoArmable)
                            {

                                AdaptSostenidoFastHighWitnessCapacidad =
                                    witnessCapacidadTrasEndpoint;
                            }

                            if (!witnessConservaPCM || perdioFronteraAlta)
                            {
                                limpiarFastHighWitness();
                            }
                            else if (witnessReversionPublicable)
                            {

                                const bool rearmarDesdeEndpoint =
                                    endpointFastAltoArmable
                                    && (endpointFastAltoPreSeguro
                                        || endpointFastAltoAgotaCapacidad);
                                limpiarFastHighWitness();
                                if (rearmarDesdeEndpoint)
                                {
                                    armarFastHighWitness(
                                        overrideAltoVigente,
                                        referenciaAltoTicks,
                                        referenciaAltoCons,
                                        endpointFastAltoAgotaCapacidad);
                                }
                            }
                            else if (episodioFillRevertido)
                            {

                            }
                            else if (!endpointFastAltoArmable)
                            {
                                limpiarFastHighWitness();
                            }
                            else if (endpointFastAltoPreSeguro
                                     && !AdaptSostenidoFastHighWitnessCapacidad)
                            {

                                armarFastHighWitness(
                                    overrideAltoVigente,
                                    referenciaAltoTicks,
                                    referenciaAltoCons,
                                    false);
                            }
                            else if (!witnessPublicable)
                            {

                                AdaptSostenidoFastHighWitnessUltimoNivelConsumo =
                                    nivelPostConsumo;
                                AdaptSostenidoFastHighWitnessUltimoNivelEscritura =
                                    nivelPostEscritura;
                            }
                            else if (!witnessSuperaReferencia)
                            {

                                if (endpointFastAltoPreSeguro
                                    || endpointFastAltoAgotaCapacidad
                                    || AdaptSostenidoFastHighWitnessCapacidad)
                                {
                                    armarFastHighWitness(
                                        overrideAltoVigente,
                                        referenciaAltoTicks,
                                        referenciaAltoCons,
                                        witnessCapacidadTrasEndpoint);
                                }
                                else
                                {
                                    limpiarFastHighWitness();
                                }
                            }
                            else
                            {

                                AdaptSostenidoFastHighWitnessUltimoNivelConsumo =
                                    nivelPostConsumo;
                                AdaptSostenidoFastHighWitnessUltimoNivelEscritura =
                                    nivelPostEscritura;
                            }
                        }
                        else if ((endpointFastAltoAgotaCapacidad
                                  || endpointFastAltoPreSeguro)
                                 && !witnessLineageInvalidadoEsteUpdate)
                        {

                            limpiarFastHighWitness();
                            armarFastHighWitness(
                                overrideAltoVigente,
                                referenciaAltoTicks,
                                referenciaAltoCons,
                                endpointFastAltoAgotaCapacidad);
                        }
                    }

                    if (overrideBajo)
                    {

                        aceptar = true;
                        conservarVentanaFast = false;
                        limpiarFastLowWitness(
                            AudioOutputAdaptiveFastLowWitnessClearOverrideAccepted);
                        AdaptFastLowPendiente = false;
                        AdaptFastCambioTargetPendiente = false;
                        decisionFlags &=
                            ~(AudioOutputAdaptiveLowDeferred
                              | AudioOutputAdaptiveLowRejectedByLevel);
                        fastOverrideSostenidoAceptado = true;
                        fastOverrideSostenidoAscendente = false;
                    }
                    else if (lowWitnessRefutadoEsteUpdate)
                    {

                        aceptar = false;
                        conservarVentanaFast = false;
                        u32 razones =
                            AudioOutputAdaptiveFastLowWitnessClearNone;
                        if (!observacionBajaSostenida)
                        {
                            razones |=
                                AudioOutputAdaptiveFastLowWitnessClearObservationNotLow;
                        }
                        if (lowWitnessAvanzo && !lowWitnessConservaPCM)
                        {
                            razones |=
                                AudioOutputAdaptiveFastLowWitnessClearPcmConservation;
                        }
                        if (lowWitnessRecuperoFrontera)
                        {
                            razones |=
                                AudioOutputAdaptiveFastLowWitnessClearRecoveredFrontier;
                        }
                        limpiarFastLowWitness(razones);
                        AdaptFastLowPendiente = false;
                        AdaptFastCambioTargetPendiente = false;
                        decisionFlags |=
                            AudioOutputAdaptiveLowRejectedByLevel;
                    }
                    else if (observacionBajaSostenida
                             && !overrideBajoVigente)
                    {

                        aceptar = false;
                        conservarVentanaFast = true;
                        if (!lowWitnessActivoAntes)
                            armarFastLowWitness(
                                AudioOutputAdaptiveFastLowWitnessBirthFastObservation);
                        else if (lowWitnessAvanzo
                                 && lowWitnessConservaPCM)
                        {

                            AdaptSostenidoFastLowWitnessTicksFin =
                                ticksAlConsumo;
                            AdaptSostenidoFastLowWitnessConsFin =
                                hostConsumed;
                            AdaptSostenidoFastLowWitnessFramesFin =
                                framesAlConsumo;
                            AdaptSostenidoFastLowWitnessNivelFin =
                                nivelPostConsumo;
                            AdaptSostenidoFastLowWitnessNivelFinLogico =
                                nivelPostConsumoLogico;
                        }

                        AdaptFastLowPendiente = false;
                        AdaptFastCambioTargetPendiente = false;
                        decisionFlags |=
                            AudioOutputAdaptiveLowDeferred;
                        AdaptSlowTicksInicio = ticksAlConsumo;
                        AdaptSlowConsInicio = hostConsumed;
                    }
                    else if (aceptar)
                    {

                        aceptar = false;
                        conservarVentanaFast = false;
                        AdaptFastLowPendiente = false;
                        AdaptFastCambioTargetPendiente = false;
                        decisionFlags |=
                            AudioOutputAdaptiveLowRejectedByLevel;
                    }
                }

                const bool evaluaL2SostenidoEsteUpdate =
                    AdaptSostenidoSegmentoUnoCompleto
                    && hostConsumed >= AdaptSostenidoSegmentoUnoCons
                    && ticksAlConsumo > AdaptSostenidoSegmentoUnoTicks
                    && hostConsumed - AdaptSostenidoSegmentoUnoCons
                       >= ventanaLenta;
                const bool retargetAscendenteOwnerFree =
                    aceptar
                    && ratio > AdaptRatio
                    && AdaptSostenidoCandidatoActivo
                    && !AdaptSostenidoCandidatoReemplazo
                    && !AdaptSostenidoOwnerActivo
                    && AdaptSostenidoCandidatoGeneracion != 0
                    && AdaptSostenidoDireccion < 0
                    && AdaptSostenidoParentValido
                    && !evaluaL2SostenidoEsteUpdate
                    && !intervaloHostPerdido
                    && dropsEstaEscritura == 0;
                if (retargetAscendenteOwnerFree)
                {

                    const bool contadoresCandidatoMonotonos =
                        hostConsumed > AdaptSostenidoConsInicio
                        && ticksAlConsumo > AdaptSostenidoTicksInicio
                        && framesAlConsumo >= AdaptSostenidoFramesInicio;
                    const bool fastContenidoEnCandidato =
                        AdaptFastCandidatoGeneracionInicio
                           == AdaptSostenidoCandidatoGeneracion
                        && AdaptFastConsInicio >= AdaptSostenidoConsInicio
                        && AdaptFastTicksInicio >= AdaptSostenidoTicksInicio
                        && AdaptFastFramesInicio
                           >= AdaptSostenidoFramesInicio;
                    const bool fastConservaPCM =
                        fastContenidoEnCandidato
                        && hostConsumed >= AdaptFastConsInicio
                        && ticksAlConsumo >= AdaptFastTicksInicio
                        && framesAlConsumo >= AdaptFastFramesInicio
                        && conservaPCM(
                            framesAlConsumo - AdaptFastFramesInicio,
                            AdaptFastNivelInicioLogico,
                            hostConsumed - AdaptFastConsInicio,
                            nivelPostConsumoLogico);
                    if (!contadoresCandidatoMonotonos
                        || !fastConservaPCM)
                    {
                        aceptar = false;
                    }
                    else
                    {
                        const u64 deltaCandidatoCons =
                            hostConsumed - AdaptSostenidoConsInicio;
                        const u64 deltaCandidatoTicks =
                            ticksAlConsumo - AdaptSostenidoTicksInicio;
                        const u64 deltaCandidatoFrames =
                            framesAlConsumo - AdaptSostenidoFramesInicio;
                        const bool candidatoPublicable =
                            deltaCandidatoCons >= ventanaRate
                            && deltaCandidatoTicks
                               >= ticksDosFramesPublicables;
                        const bool candidatoConservaPCM =
                            candidatoPublicable
                            && conservaPCM(
                                deltaCandidatoFrames,
                                AdaptSostenidoNivelInicioLogico,
                                deltaCandidatoCons,
                                nivelPostConsumoLogico);
                        const double ratioCandidato = candidatoConservaPCM
                            ? medirRatio(
                                deltaCandidatoTicks, deltaCandidatoCons)
                            : 0.0;
                        const bool candidatoSigueBajoParentMedido =
                            !AdaptSostenidoParentEsHint
                            && AdaptSostenidoParentTicks > 0
                            && AdaptSostenidoParentCons > 0
                            && compararRelacionHT(
                                deltaCandidatoTicks,
                                deltaCandidatoCons,
                                AdaptSostenidoParentTicks,
                                AdaptSostenidoParentCons) < 0;
                        const bool candidatoSigueBajoParentHint =
                            AdaptSostenidoParentEsHint
                            && AdaptSostenidoParentRatio > 0.0
                            && std::isfinite(AdaptSostenidoParentRatio)
                            && ratioCandidato
                               < AdaptSostenidoParentRatio;
                        if (!candidatoConservaPCM
                            || !std::isfinite(ratioCandidato)
                            || (!candidatoSigueBajoParentMedido
                                && !candidatoSigueBajoParentHint)
                            || ratioCandidato <= AdaptRatio)
                        {
                            aceptar = false;
                        }
                        else if (compararRelacionHT(
                                     deltaCandidatoTicks,
                                     deltaCandidatoCons,
                                     deltaFastTicks,
                                     deltaFastCons) < 0)
                        {
                            ratio = ratioCandidato;
                        }
                    }
                }

                if (aceptar && AdaptHint > 0.0 && ratio > AdaptHint)
                {
                    decisionFlags |=
                        AudioOutputAdaptiveRatioCappedByHint;
                    ratio = AdaptHint;
                }
                if (aceptar
                    && AdaptSostenidoPhaseProvisionalActivo
                    && AdaptSostenidoCandidatoActivo
                    && !AdaptSostenidoCandidatoReemplazo
                    && !AdaptSostenidoOwnerActivo
                    && AdaptSostenidoPhaseProvisionalCandidatoGeneracion
                       == AdaptSostenidoCandidatoGeneracion)
                {

                    aceptar = false;
                    conservarVentanaFast = false;
                    AdaptFastLowPendiente = false;
                    AdaptFastCambioTargetPendiente = false;
                    decisionFlags |=
                        AudioOutputAdaptiveLowDeferred;
                    AdaptSlowTicksInicio = ticksAlConsumo;
                    AdaptSlowConsInicio = hostConsumed;
                }
                if (aceptar)
                {
                    const bool parentMedidoVigente =
                        rateBajoAntesEstimadores
                        && ratePadreCertificadoAntesEstimadores
                        && AdaptRatePadreTicks > 0
                        && AdaptRatePadreCons > 0;
                    const bool parentHintVigente =
                        !parentMedidoVigente
                        && AdaptHint > 0.0
                        && !AdaptProbe
                        && ratioAntesEstimadores == AdaptHint;
                    const bool primerLowOwnerFree =
                        !AdaptSostenidoPhaseProvisionalActivo
                        && !AdaptSostenidoCandidatoActivo
                        && !AdaptSostenidoOwnerActivo
                        && !AdaptRateActuadorPendiente
                        && (cambioTargetFronteraPhaseElegible
                            || cambioTargetEscrowWriteInicialElegible
                            || cambioTargetPhaseProvisionalP3Elegible)
                        && (decisionFlags
                            & AudioOutputAdaptiveLowConfirmed) != 0
                        && ratio < ratioAntesEstimadores
                        && (parentMedidoVigente || parentHintVigente);
                    if (primerLowOwnerFree)
                    {

                        fastLowOwnerFreePhaseProvisional = true;
                        fastLowOwnerFreePhaseObjetivo = limitar(ratio);
                    }
                    else
                    {
                        AdaptRatio = limitar(ratio);
                        AdaptProbe = false;
                        ratioFastAceptado = true;
                        decisionFlags |=
                            AudioOutputAdaptiveFastAccepted;
                        aplicarInmediato = true;
                    }
                    AdaptSlowTicksInicio = ticksAlConsumo;
                    AdaptSlowConsInicio = hostConsumed;
                }
            }
            else
            {
                limpiarFastLowWitness(
                    AudioOutputAdaptiveFastLowWitnessClearFastWindowUnavailable);
                AdaptFastLowPendiente = false;
                AdaptFastCambioTargetPendiente = false;
                limpiarFastHighWitness();
            }
        }
        if (!conservarVentanaFast)
        {

            ++telemetry.fastAnchorRebaseCount;
            telemetry.lastFastAnchorRebaseUpdateId = telemetry.updateId;
            telemetry.lastFastAnchorRebaseDeltaConsumed = deltaFastCons;
            telemetry.lastFastAnchorRebaseDeltaTicks = deltaFastTicks;
            telemetry.lastFastAnchorRebaseRawRatio = deltaFastCons > 0
                ? medirRatio(deltaFastTicks, deltaFastCons) : 0.0;
            telemetry.lastFastAnchorRebaseRatioBeforeEstimators =
                ratioAntesEstimadores;
            telemetry.lastFastAnchorRebaseRatioAfter = AdaptRatio;
            telemetry.lastFastAnchorRebaseLevelPostConsumption =
                static_cast<u32>(std::max(0, nivelPostConsumo));
            telemetry.lastFastAnchorRebaseLevelPostWrite =
                static_cast<u32>(std::max(0, nivelPostEscritura));
            telemetry.lastFastAnchorRebaseRecoveryAscendingFrames =
                static_cast<u32>(std::max(0, nivelRecuperacionAscendente));
            telemetry.lastFastAnchorRebaseDecisionFlags =
                decisionFlags;
            telemetry.lastFastAnchorRebaseTargetRateOwned =
                targetRatePoseido;
            telemetry.lastFastAnchorRebaseEstimatorEvaluated =
                estimadorFastEvaluado;
            rebasarVentanaFast(
                !intervaloHostPerdido && dropsEstaEscritura == 0);
            if (fastTargetChangePhaseFrontierBornThisUpdate
                && AdaptFastCambioTargetPendiente)
            {

                AdaptFastCambioTargetFronteraPhaseOrigen =
                    AudioOutputAdaptiveFastTargetChangePhaseFrontierFastWindow;
                AdaptFastCambioTargetFronteraPhaseValida = true;

                const double obligacionSiguienteObservacion =
                    deficitVentanaFast + presupuestoVentanaFast;
                const double faltaBacking =
                    obligacionSiguienteObservacion
                    - (double)nivelPostEscritura;
                const u64 safe = static_cast<u64>(
                    std::max(0, nivelSeguro));
                const u64 writeLevel = nivelPostEscritura >= 0
                    ? static_cast<u64>(nivelPostEscritura) : 0;
                const u64 headroomFisico = writeLevel <= safe
                    ? safe - writeLevel : 0;
                const double horizonteProduccion =
                    ((double)ticksDosFramesPublicables
                     * OutputSampleRate)
                    / (double)INTERNAL_SAMPLE_RATE;
                const bool obligacionCompletamenteRespaldada =
                    std::isfinite(obligacionSiguienteObservacion)
                    && obligacionSiguienteObservacion >= 0.0
                    && obligacionSiguienteObservacion
                       <= (double)writeLevel;
                const double produccionEndpoint =
                    ratioFastObservable > 0.0
                    && std::isfinite(ratioFastObservable)
                    ? horizonteProduccion / ratioFastObservable : 0.0;
                const double produccionPadre =
                    AdaptRatio > 0.0 && std::isfinite(AdaptRatio)
                    ? horizonteProduccion / AdaptRatio : 0.0;
                const double grantFinitoRaw =
                    obligacionCompletamenteRespaldada
                    ? produccionEndpoint - produccionPadre
                    : faltaBacking;
                const bool rootFisicoOwnerFree =
                    intervaloHostPerdido
                    && !targetRatePoseido
                    && !AdaptSostenidoCandidatoActivo
                    && !AdaptSostenidoOwnerActivo
                    && !AdaptFastAnchorOwnerValido
                    && AdaptFastAnchorOwnerGeneracion == 0
                    && nivelPostConsumo >= 0
                    && nivelPostEscritura >= nivelPostConsumo
                    && nivelPostConsumoLogico
                       == static_cast<u64>(nivelPostConsumo)
                    && nivelPostEscrituraLogico
                       == static_cast<u64>(nivelPostEscritura)
                    && OutputSpillFrames == 0
                    && dropsEstaEscritura == 0
                    && std::isfinite(AdaptRatio)
                    && AdaptRatio > 0.0
                    && std::isfinite(ratioFastObservable)
                    && ratioFastObservable > 0.0
                    && ratioFastObservable < AdaptRatio
                    && std::isfinite(grantFinitoRaw)
                    && grantFinitoRaw > 0.0

                    && writeLevel <= safe
                    && grantFinitoRaw
                       <= static_cast<double>(headroomFisico);
                if (rootFisicoOwnerFree)
                {
                    const u64 grant = static_cast<u64>(
                        std::ceil(grantFinitoRaw));
                    const bool grantCabeFisicamente = grant > 0
                        && grant <= headroomFisico;
                    const double produccionBaseExacta =
                        horizonteProduccion / AdaptRatio;
                    const double produccionBaseConservadora = std::floor(
                        std::nextafter(
                            produccionBaseExacta,
                            -std::numeric_limits<double>::infinity()));
                    const double produccionMaxima =
                        produccionBaseConservadora + (double)grant;
                    double guardSkew = produccionMaxima > 0.0
                        ? horizonteProduccion / produccionMaxima
                        : 0.0;
                    guardSkew = std::nextafter(guardSkew, AdaptRatio);
                    guardSkew = limitar(guardSkew);
                    if (grantCabeFisicamente
                        && std::isfinite(guardSkew)
                        && guardSkew > 0.0
                        && guardSkew < AdaptRatio

                        && AdaptSkew >= guardSkew)
                    {
                        AdaptFastCambioTargetPhaseEscrowActivo = true;
                        AdaptFastCambioTargetPhaseEscrowGrantFrames = grant;
                        AdaptFastCambioTargetPhaseEscrowGuardSkew =
                            guardSkew;
                        AdaptFastCambioTargetPhaseEscrowOverlaySkew = 0.0;
                        AdaptFastCambioTargetPhaseEscrowRateReferencia =
                            AdaptRatio;
                        AdaptFastCambioTargetPhaseEscrowTicksInicio =
                            AdaptTicksTotal;
                        AdaptFastCambioTargetPhaseEscrowConsInicio =
                            hostConsumed;
                        AdaptFastCambioTargetPhaseEscrowFramesConsumoInicio =
                            framesAlConsumo;
                        AdaptFastCambioTargetPhaseEscrowFramesPublicadosInicio =
                            AdaptFramesProducidosTotal;
                        AdaptFastCambioTargetPhaseEscrowNivelConsumoInicio =
                            nivelPostConsumoLogico;
                        AdaptFastCambioTargetPhaseEscrowNivelEscrituraInicio =
                            nivelPostEscrituraLogico;
                        AdaptFastCambioTargetPhaseEscrowContinuidadEpoch =
                            continuidadEpoch;
                        AdaptFastCambioTargetPhaseEscrowResetEpoch =
                            resetSolicitado;
                        AdaptFastCambioTargetPhaseEscrowHintSeenEpoch =
                            AdaptHintVistoProductor;
                        AdaptFastCambioTargetPhaseEscrowCandidatoGeneracion =
                            AdaptSostenidoCandidatoGeneracion;
                        AdaptFastCambioTargetPhaseEscrowUnderrunsInicio =
                            underrunsActuales;
                        AdaptFastCambioTargetPhaseEscrowSpillDropsInicio =
                            OutputSpillDroppedFramesTotal;
                        AdaptFastCambioTargetPhaseEscrowSpillAllocationFailuresInicio =
                            OutputSpillAllocationFailureCount;
                        ++telemetry.fastTargetChangePhaseEscrowBirthCount;
                        telemetry.lastFastTargetChangePhaseEscrowBirthUpdateId =
                            telemetry.updateId;
                        telemetry.lastFastTargetChangePhaseEscrowBirthGrantFrames =
                            grant;
                        telemetry.lastFastTargetChangePhaseEscrowBirthGuardSkew =
                            guardSkew;
                    }
                }
            }
        }
    }
    bool ratioSlowAplicado = false;
    const u64 deltaSlowCons = hostConsumed - AdaptSlowConsInicio;
    const u64 deltaSlowTicks = ticksAlConsumo - AdaptSlowTicksInicio;
    if (!ratioFastAceptado && !fastLowOwnerFreePhaseProvisional
        && !AdaptSostenidoPhaseProvisionalActivo
        && !AdaptFastLowPendiente
        && !AdaptFastCambioTargetPendiente
        && !AdaptRateBajoConfirmado
        && !AdaptSostenidoOwnerActivo
        && deltaSlowCons >= ventanaLenta
        && deltaSlowTicks > 0)
    {
        decisionFlags &= ~AudioOutputAdaptiveEstimatorFast;
        decisionFlags |= AudioOutputAdaptiveEstimatorSlow;
        telemetry.estimatorDeltaConsumed = deltaSlowCons;
        telemetry.estimatorDeltaTicks = deltaSlowTicks;
        double ratio = medirRatio(deltaSlowTicks, deltaSlowCons);
        telemetry.estimatorRawRatio = ratio;
        bool conservarVentanaSlow = false;
        if (std::isfinite(ratio) && ratio > 0.0)
        {
            decisionFlags |= AudioOutputAdaptiveMeasurementValid;
            bool aplicar = true;
            if (AdaptHint > 0.0 && ratio < AdaptRatio)
            {
                const double deficit =
                    medirDeficit(deltaSlowTicks, deltaSlowCons);
                const double presupuestoTransitorio =
                    medirPresupuestoTransitorio();

                if (nivelPostConsumo >= nivelBajo
                    && nivelPostEscritura >= objetivo)
                {

                    aplicar = false;
                    decisionFlags |=
                        AudioOutputAdaptiveLowRejectedByLevel;
                }
                else if (std::isfinite(deficit)
                         && deficit <= presupuestoTransitorio)
                {

                    aplicar = false;
                    conservarVentanaSlow = true;
                    decisionFlags |=
                        AudioOutputAdaptiveLowDeferred;
                }
                else
                {
                    decisionFlags |=
                        AudioOutputAdaptiveLowConfirmed;
                }
            }
            if (aplicar && AdaptHint > 0.0 && ratio > AdaptHint)
            {
                decisionFlags |=
                    AudioOutputAdaptiveRatioCappedByHint;
                ratio = AdaptHint;
            }
            if (aplicar)
            {
                AdaptRatio = 0.5 * AdaptRatio + 0.5 * limitar(ratio);
                ratioSlowAplicado = true;
                decisionFlags |= AudioOutputAdaptiveSlowApplied;
            }
        }
        if (!conservarVentanaSlow)
        {
            AdaptSlowTicksInicio = ticksAlConsumo;
            AdaptSlowConsInicio = hostConsumed;
        }
    }

    const bool medidaRateBajaConfirmada =
        (decisionFlags
         & AudioOutputAdaptiveLowConfirmed) != 0;
    if ((ratioFastAceptado || fastLowOwnerFreePhaseProvisional)
        && medidaRateBajaConfirmada)
    {

        ++telemetry.fastTargetChangePhaseFrontierEvaluationCount;
        telemetry.lastFastTargetChangePhaseFrontierEvaluationUpdateId =
            telemetry.updateId;
        telemetry.lastFastTargetChangePhaseFrontierEvaluationOrigin =
            cambioTargetFronteraPhaseOrigenAntes;
        telemetry.lastFastTargetChangePhaseFrontierEvaluationRejectMask =
            cambioTargetFronteraPhaseRejectMask;
    }
    const bool fastCambioTargetConfirmadoConFaseRaiz =
        cambioTargetFronteraPhaseElegible
        && ratioFastAceptado
        && medidaRateBajaConfirmada;
    const bool fastCambioTargetConfirmadoConEscrowWrite =
        cambioTargetEscrowWriteInicialElegible
        && ratioFastAceptado
        && medidaRateBajaConfirmada
        && AdaptRatio < ratioAntesEstimadores;
    const u32 fastCambioTargetEscrowWriteFrames =
        fastCambioTargetConfirmadoConEscrowWrite
        ? static_cast<u32>(nivelPostEscritura - nivelPostConsumo) : 0;
    const bool fastRateIdempotente = ratioFastAceptado
        && !medidaRateBajaConfirmada
        && AdaptRatio == ratioAntesEstimadores;
    if (fastRateIdempotente)
    {

        aplicarInmediato = false;
    }
    enum class TransaccionSostenida
    {
        Ninguna,
        AdquirirOwner,
        ReemplazarOwner,
        OverrideFast,
        RestaurarOwner,
        LiberarParent,
    };
    TransaccionSostenida transaccionSostenida =
        TransaccionSostenida::Ninguna;
    bool parentCapacityTransferenciaTipada = false;
    bool parentCapacityNacidaEsteUpdate = false;
    double ratioRateSostenido = 0.0;
    u64 rateSostenidoVentanaCons = 0;
    u64 rateSostenidoVentanaTicks = 0;
    int rateSostenidoPhaseFrames = 0;
    const bool parentSostenidoMedido =
        rateBajoAntesEstimadores
        && ratePadreCertificadoAntesEstimadores
        && AdaptRatePadreTicks > 0
        && AdaptRatePadreCons > 0;
    const bool parentSostenidoHint =
        !parentSostenidoMedido
        && AdaptHint > 0.0
        && !AdaptProbe
        && ratioAntesEstimadores == AdaptHint;
    const bool transicionRateParaSostenido =
        medidaRateBajaConfirmada
        && ((ratioFastAceptado && !fastRateIdempotente)
            || fastLowOwnerFreePhaseProvisional
            || (ratioSlowAplicado && AdaptRatio != ratioAntesEstimadores));
    auto abrirCandidatoSostenido = [&](bool reemplazo,
                                        double referencia,
                                        int direccion) {
        ++AdaptSostenidoCandidatoGeneracion;
        AdaptSostenidoCandidatoActivo = true;
        AdaptSostenidoCandidatoReemplazo = reemplazo;
        AdaptSostenidoTicksInicio = ticksAlConsumo;
        AdaptSostenidoConsInicio = hostConsumed;
        AdaptSostenidoFramesInicio = framesAlConsumo;
        AdaptSostenidoFramesPublicadosInicio = AdaptFramesProducidosTotal;
        AdaptSostenidoUltimaPublicacionCons = hostConsumed;
        AdaptSostenidoNivelInicio = nivelPostConsumo;
        AdaptSostenidoNivelInicioLogico = nivelPostConsumoLogico;
        AdaptSostenidoNivelEscrituraInicioLogico =
            nivelPostEscrituraLogico;
        AdaptSostenidoContinuidadEpochInicio = continuidadEpoch;
        AdaptSostenidoResetEpochInicio = resetSolicitado;
        AdaptSostenidoHintSeenEpochInicio = AdaptHintVistoProductor;
        AdaptSostenidoNivelMinimo = nivelPostConsumo;
        AdaptSostenidoNivelMaximo = nivelPostEscritura;
        AdaptSostenidoNivelMinimoLogico = nivelPostConsumoLogico;
        AdaptSostenidoNivelMaximoLogico = nivelPostEscrituraLogico;
        AdaptSostenidoSegmentoUnoCompleto = false;
        AdaptSostenidoSegmentoUnoTicks = ticksAlConsumo;
        AdaptSostenidoSegmentoUnoCons = hostConsumed;
        AdaptSostenidoSegmentoUnoFrames = framesAlConsumo;
        AdaptSostenidoSegmentoUnoFramesPublicados =
            AdaptFramesProducidosTotal;
        AdaptSostenidoSegmentoUnoNivel = nivelPostConsumo;
        AdaptSostenidoSegmentoUnoNivelLogico = nivelPostConsumoLogico;
        AdaptSostenidoSegmentoUnoNivelEscrituraLogico =
            nivelPostEscrituraLogico;
        AdaptSostenidoSegmentoUnoContinuidadEpoch = continuidadEpoch;
        AdaptSostenidoSegmentoUnoResetEpoch = resetSolicitado;
        AdaptSostenidoSegmentoUnoHintSeenEpoch = AdaptHintVistoProductor;
        AdaptSostenidoReferenciaRatio = referencia;
        AdaptSostenidoDireccion = direccion;
    };
    auto publicarObservacionSostenida = [&](u64 deltaCons,
                                             u64 deltaTicks,
                                             u64 deltaFrames,
                                             u64 nivelInicioLogico,
                                             u32 tipo) {
        auto nivelTelemetry = [](u64 nivel) {
            return static_cast<u32>(std::min<u64>(
                nivel, std::numeric_limits<u32>::max()));
        };
        ++telemetry.sustainedObservationCount;
        telemetry.lastSustainedObservationUpdateId = telemetry.updateId;
        telemetry.lastSustainedObservationCandidateGeneration =
            AdaptSostenidoCandidatoGeneracion;
        telemetry.lastSustainedObservationEndConsumed = hostConsumed;
        telemetry.lastSustainedObservationEndTicks = ticksAlConsumo;
        telemetry.lastSustainedObservationEndProduced = framesAlConsumo;
        telemetry.lastSustainedObservationDeltaConsumed = deltaCons;
        telemetry.lastSustainedObservationDeltaTicks = deltaTicks;
        telemetry.lastSustainedObservationDeltaProduced = deltaFrames;
        telemetry.lastSustainedObservationRawRatio =
            medirRatio(deltaTicks, deltaCons);
        telemetry.lastSustainedObservationStartLevel =
            nivelTelemetry(nivelInicioLogico);
        telemetry.lastSustainedObservationEndLevel =
            nivelTelemetry(nivelPostConsumoLogico);
        telemetry.lastSustainedObservationMinLevel =
            nivelTelemetry(AdaptSostenidoNivelMinimoLogico);
        telemetry.lastSustainedObservationMaxLevel =
            nivelTelemetry(AdaptSostenidoNivelMaximoLogico);
        telemetry.lastSustainedObservationType = tipo;
        AdaptSostenidoUltimaPublicacionCons = hostConsumed;
    };

    bool candidatoAbiertoEsteUpdate = false;
    if (AdaptSostenidoOwnerActivo && fastOverrideSostenidoAceptado)
    {
        const int direccionOverride = fastOverrideSostenidoAscendente ? 1 : -1;
        if (!fastRateIdempotente)
        {
            AdaptSostenidoOverrideActivo = true;
            AdaptSostenidoOverrideDireccion = direccionOverride;
            const bool identidadOverrideAlta = direccionOverride > 0
                && AdaptRatio == ratioFastObservable;
            AdaptSostenidoOverrideTicks = identidadOverrideAlta
                ? fastOverrideSostenidoTicks : 0;
            AdaptSostenidoOverrideCons = identidadOverrideAlta
                ? fastOverrideSostenidoCons : 0;
            if (identidadOverrideAlta)
            {

                limpiarFastHighEscrow(
                    AudioOutputAdaptiveFastHighEscrowClearRate);
                limpiarFastHighWitness();
                armarFastHighWitness(
                    true,
                    AdaptSostenidoOverrideTicks,
                    AdaptSostenidoOverrideCons,
                    true);
            }
            else
            {

                limpiarFastHighEscrow(
                    AudioOutputAdaptiveFastHighEscrowClearRate);
                limpiarFastHighWitness();
            }
            if (!AdaptSostenidoCandidatoActivo
                || !AdaptSostenidoCandidatoReemplazo
                || AdaptSostenidoDireccion != direccionOverride)
            {

                abrirCandidatoSostenido(
                    true, AdaptSostenidoOwnerRatio, direccionOverride);
                candidatoAbiertoEsteUpdate = true;
            }
            transaccionSostenida = TransaccionSostenida::OverrideFast;
        }
        else if (direccionOverride > 0
                 && !AdaptSostenidoCandidatoActivo)
        {

            abrirCandidatoSostenido(
                true, AdaptSostenidoOwnerRatio, direccionOverride);
            candidatoAbiertoEsteUpdate = true;
        }
    }
    else if (AdaptSostenidoOwnerActivo
             && observacionAltaSostenida
             && !AdaptSostenidoCandidatoActivo)
    {

        abrirCandidatoSostenido(
            true, AdaptSostenidoOwnerRatio, 1);
        candidatoAbiertoEsteUpdate = true;
    }
    else if (transicionRateParaSostenido
        && !AdaptSostenidoCandidatoActivo
        && !AdaptSostenidoOwnerActivo
        && (parentSostenidoMedido || parentSostenidoHint))
    {

        abrirCandidatoSostenido(false, ratioAntesEstimadores, -1);
        candidatoAbiertoEsteUpdate = true;
        AdaptSostenidoEpisodioOrigenUpdateId = telemetry.updateId;
        AdaptSostenidoParentValido = true;
        AdaptSostenidoParentEsHint = parentSostenidoHint;
        AdaptSostenidoParentRatio = ratioAntesEstimadores;
        AdaptSostenidoParentTicks = parentSostenidoMedido
            ? AdaptRatePadreTicks : 0;
        AdaptSostenidoParentCons = parentSostenidoMedido
            ? AdaptRatePadreCons : 0;
        if (fastLowOwnerFreePhaseProvisional)
        {
            const double skewPublicadoActual =
                OutputSkewPublicado.load(std::memory_order_relaxed);
            const double reservaPhaseActualRaw =
                medirReservaPublicacion(AdaptSkew);
            const double presupuestoTransitorioRaw =
                medirPresupuestoTransitorio();
            const double capacidadRaw = (double)(OutputBufferSize - 1);
            const bool postcondicionRepresentable =
                (cambioTargetEscrowWriteInicialElegible
                 || cambioTargetPhaseProvisionalP3Elegible)
                && AdaptSkew == skewPublicadoActual
                && AdaptSkew > 0.0
                && std::isfinite(AdaptSkew)
                && AdaptSkew < ratioAntesEstimadores
                && std::isfinite(reservaPhaseActualRaw)
                && reservaPhaseActualRaw >= 0.0
                && reservaPhaseActualRaw <= capacidadRaw
                && reservaPhaseActualRaw
                   <= (double)std::numeric_limits<u64>::max()
                && std::isfinite(presupuestoTransitorioRaw)
                && presupuestoTransitorioRaw >= 0.0
                && presupuestoTransitorioRaw <= capacidadRaw
                && presupuestoTransitorioRaw
                   <= (double)std::numeric_limits<u64>::max()
                && nivelPostConsumo >= 0
                && nivelPostEscritura >= nivelPostConsumo
                && nivelPostConsumoLogico
                   == static_cast<u64>(nivelPostConsumo)
                && nivelPostEscrituraLogico
                   == static_cast<u64>(nivelPostEscritura)
                && OutputSpillFrames == 0
                && dropsEstaEscritura == 0
                && !AdaptRateActuadorPendiente;
            const u64 reservaPhaseActual = postcondicionRepresentable
                ? static_cast<u64>(reservaPhaseActualRaw) : 0;
            const u64 presupuestoTransitorioFrames =
                postcondicionRepresentable
                ? static_cast<u64>(presupuestoTransitorioRaw) : 0;
            const u64 fronteraPhaseActual = postcondicionRepresentable
                ? static_cast<u64>(std::min(
                    objetivoControl, std::max(0, nivelPostConsumo))) : 0;
            const u64 backingPhaseActual = std::max(
                fronteraPhaseActual, presupuestoTransitorioFrames);
            const bool conservarPostcondicionPhase =
                postcondicionRepresentable
                && backingPhaseActual
                   <= static_cast<u64>(std::numeric_limits<int>::max())
                && AudioOutputExactMath::FitsPhasePostcondition(
                    static_cast<u64>(nivelPostEscritura),
                    backingPhaseActual, reservaPhaseActual,
                    static_cast<u64>(OutputBufferSize - 1));
            bool intervaloPhaseFactible = false;
            const double skewPhaseProyectado = conservarPostcondicionPhase
                ? AdaptSkew : proyectarPhaseProvisionalOwnerFree(
                    fastLowOwnerFreePhaseObjetivo,
                    deltaFastCons, deltaFastTicks,
                    intervaloPhaseFactible);

            limpiarPhaseEscrowCambioTarget(
                AudioOutputAdaptiveFastTargetChangePhaseEscrowClearOwnership);
            limpiarPhaseProvisionalOwnerFree(
                AudioOutputAdaptiveSustainedProvisionalPhaseClearReplacement);
            const bool nacimientoPhaseValido =
                (conservarPostcondicionPhase || intervaloPhaseFactible)
                && skewPhaseProyectado > 0.0
                && std::isfinite(skewPhaseProyectado)
                && ratioAntesEstimadores > 0.0
                && std::isfinite(ratioAntesEstimadores)
                && !AdaptRateActuadorPendiente;
            if (nacimientoPhaseValido)
            {
                AdaptSostenidoPhaseProvisionalActivo = true;
                AdaptSostenidoPhaseProvisionalCandidatoGeneracion =
                    AdaptSostenidoCandidatoGeneracion;
                AdaptSostenidoPhaseProvisionalOwnerGeneracion =
                    AdaptSostenidoOwnerGeneracion;
                AdaptSostenidoPhaseProvisionalParentRatio =
                    ratioAntesEstimadores;
                AdaptSostenidoPhaseProvisionalSkew =
                    skewPhaseProyectado;
                AdaptSostenidoPhaseProvisionalNivelInicio =
                    conservarPostcondicionPhase
                    ? static_cast<int>(backingPhaseActual)
                    : nivelPostConsumo;
                const double reservaNacimientoRaw =
                    medirReservaPublicacion(skewPhaseProyectado);
                const u64 reservaNacimiento =
                    std::isfinite(reservaNacimientoRaw)
                    && reservaNacimientoRaw >= 0.0
                    && reservaNacimientoRaw
                       <= (double)std::numeric_limits<u64>::max()
                    ? static_cast<u64>(reservaNacimientoRaw) : 0;
                const u64 presupuestoNacimiento =
                    std::isfinite(presupuestoTransitorioRaw)
                    && presupuestoTransitorioRaw >= 0.0
                    && presupuestoTransitorioRaw
                       <= (double)std::numeric_limits<u64>::max()
                    ? static_cast<u64>(presupuestoTransitorioRaw) : 0;
                AdaptSostenidoPhaseProvisionalFronteraFrames =
                    conservarPostcondicionPhase
                    ? fronteraPhaseActual
                    : static_cast<u64>(std::max(0, nivelPostConsumo));
                AdaptSostenidoPhaseProvisionalBackingFrames =
                    conservarPostcondicionPhase
                    ? backingPhaseActual
                    : std::max({
                        AdaptSostenidoPhaseProvisionalFronteraFrames,
                        reservaNacimiento, presupuestoNacimiento});
                AdaptSostenidoPhaseProvisionalReservaPublicacionFrames =
                    reservaNacimiento;
                AdaptSostenidoPhaseProvisionalPublicacionesReservadas = 1;
                AdaptSostenidoPhaseProvisionalContinuidadEpoch =
                    continuidadEpoch;
                AdaptSostenidoPhaseProvisionalResetEpoch = resetSolicitado;
                AdaptSostenidoPhaseProvisionalHintSeenEpoch =
                    AdaptHintVistoProductor;
                AdaptSostenidoPhaseProvisionalUnderrunsInicio =
                    underrunsActuales;
                AdaptSostenidoPhaseProvisionalSpillDropsInicio =
                    OutputSpillDroppedFramesTotal;
                AdaptSostenidoPhaseProvisionalSpillAllocationFailuresInicio =
                    OutputSpillAllocationFailureCount;
                ++telemetry.sustainedProvisionalPhaseBirthCount;
                telemetry.lastSustainedProvisionalPhaseBirthUpdateId =
                    telemetry.updateId;
                telemetry.lastSustainedProvisionalPhaseBirthCandidateGeneration =
                    AdaptSostenidoPhaseProvisionalCandidatoGeneracion;
                telemetry.lastSustainedProvisionalPhaseBirthParentRatio =
                    AdaptSostenidoPhaseProvisionalParentRatio;
                telemetry.lastSustainedProvisionalPhaseBirthSkew =
                    AdaptSostenidoPhaseProvisionalSkew;
                telemetry.lastSustainedProvisionalPhaseBirthFrontierFrames =
                    AdaptSostenidoPhaseProvisionalFronteraFrames;
                telemetry.lastSustainedProvisionalPhaseBirthBackingFrames =
                    AdaptSostenidoPhaseProvisionalBackingFrames;
                telemetry.lastSustainedProvisionalPhaseBirthPublicationReserveFrames =
                    AdaptSostenidoPhaseProvisionalReservaPublicacionFrames;
                telemetry.lastSustainedProvisionalPhaseBirthReservedPublicationCount =
                    AdaptSostenidoPhaseProvisionalPublicacionesReservadas;
            }
            else
            {

                decisionFlags |=
                    AudioOutputAdaptiveSlewBranch;
                provisionalPhaseIntervalInfeasibleEsteUpdate = true;
            }

            if (nacimientoPhaseValido)
            {
                AdaptPhaseBalanceFrames = 0.0;
                AdaptPhaseEpisodioActivo = false;
                AdaptPhaseObjetivoFrames = 0;
                AdaptPhaseObjetivoActivo = false;
                AdaptPhaseRebasePendiente = false;
                AdaptPhaseRebaseBandViolada = false;
                AdaptPhaseRebaseMinFrames = -1;
                AdaptPhaseOrigenRecuperacionRate = false;
                AdaptActCreditoFrames = 0.0;
                AdaptRateCreditoFrames = 0.0;
                aplicarInmediato = true;
            }
        }
    }

    if (AdaptSostenidoOwnerActivo
        && AdaptSostenidoOverrideActivo
        && transaccionSostenida != TransaccionSostenida::OverrideFast)
    {
        const int fronteraOwner = std::min(
            nivelSeguro,
            std::max(0, AdaptSostenidoPhaseObjetivoFrames));
        const bool overrideAltoPerdioFrontera =
            AdaptSostenidoOverrideDireccion > 0
            && (nivelPostConsumo < fronteraOwner
                || nivelPostEscritura < fronteraOwner);
        const bool overrideBajoRecuperoFrontera =
            AdaptSostenidoOverrideDireccion < 0
            && !AdaptSostenidoCandidatoActivo
            && nivelPostConsumo >= fronteraOwner
            && nivelPostEscritura >= fronteraOwner;
        if (overrideAltoPerdioFrontera
            || overrideBajoRecuperoFrontera)
        {

            AdaptSostenidoOverrideActivo = false;
            AdaptSostenidoOverrideDireccion = 0;
            AdaptSostenidoOverrideTicks = 0;
            AdaptSostenidoOverrideCons = 0;
            limpiarFastHighEscrow(
                AudioOutputAdaptiveFastHighEscrowClearRate);
            limpiarFastHighWitness();
            transaccionSostenida = TransaccionSostenida::RestaurarOwner;
        }
    }

    if (AdaptSostenidoCandidatoActivo && !candidatoAbiertoEsteUpdate)
    {
        AdaptSostenidoNivelMinimo = std::min(
            AdaptSostenidoNivelMinimo, nivelPostConsumo);
        AdaptSostenidoNivelMaximo = std::max(
            AdaptSostenidoNivelMaximo, nivelPostEscritura);
        AdaptSostenidoNivelMinimoLogico = std::min(
            AdaptSostenidoNivelMinimoLogico, nivelPostConsumoLogico);
        AdaptSostenidoNivelMaximoLogico = std::max(
            AdaptSostenidoNivelMaximoLogico, nivelPostEscrituraLogico);

        const bool contadoresSostenidosMonotonos =
            hostConsumed >= AdaptSostenidoConsInicio
            && ticksAlConsumo >= AdaptSostenidoTicksInicio
            && framesAlConsumo >= AdaptSostenidoFramesInicio;
        if (!contadoresSostenidosMonotonos || dropsEstaEscritura > 0)
        {
            AdaptSostenidoCandidatoActivo = false;
            AdaptSostenidoCandidatoReemplazo = false;
            AdaptSostenidoSegmentoUnoCompleto = false;
            if (!AdaptSostenidoOwnerActivo)
                AdaptSostenidoParentValido = false;

        }
        else if (intervaloHostPerdido)
        {

        }
        else if (!AdaptSostenidoSegmentoUnoCompleto)
        {
            const u64 deltaCons = hostConsumed - AdaptSostenidoConsInicio;
            if (deltaCons >= ventanaLenta
                && ticksAlConsumo > AdaptSostenidoTicksInicio)
            {
                const u64 deltaTicks =
                    ticksAlConsumo - AdaptSostenidoTicksInicio;
                const u64 deltaFrames =
                    framesAlConsumo - AdaptSostenidoFramesInicio;
                const bool referenciaInicialMedida =
                    !AdaptSostenidoCandidatoReemplazo
                    && AdaptSostenidoParentValido
                    && !AdaptSostenidoParentEsHint;
                const int direccion = AdaptSostenidoCandidatoReemplazo
                    ? compararRelacionHT(
                        deltaTicks, deltaCons,
                        AdaptSostenidoOwnerTicks,
                        AdaptSostenidoOwnerCons)
                    : referenciaInicialMedida
                    ? compararRelacionHT(
                        deltaTicks, deltaCons,
                        AdaptSostenidoParentTicks,
                        AdaptSostenidoParentCons)
                    : (medirRatio(deltaTicks, deltaCons)
                           < AdaptSostenidoReferenciaRatio ? -1
                       : medirRatio(deltaTicks, deltaCons)
                           > AdaptSostenidoReferenciaRatio ? 1 : 0);
                const bool segmentoValido =
                    conservaPCM(
                        deltaFrames,
                        AdaptSostenidoNivelInicioLogico,
                        deltaCons,
                        nivelPostConsumoLogico)
                    && direccion == AdaptSostenidoDireccion;
                const double produccionParent =
                    AdaptSostenidoParentValido
                    && AdaptSostenidoParentEsHint
                    ? produccionIntegral(
                        deltaTicks, AdaptSostenidoParentRatio) : -1.0;
                const bool parentHintSuficiente =
                    AdaptSostenidoCandidatoReemplazo
                    && AdaptSostenidoParentValido
                    && AdaptSostenidoParentEsHint
                    && produccionParent >= 0.0
                    && (double)deltaCons <= produccionParent;
                const bool parentMedidoExacto =
                    AdaptSostenidoCandidatoReemplazo
                    && AdaptSostenidoParentValido
                    && !AdaptSostenidoParentEsHint
                    && mismaRelacionHT(
                        deltaTicks, deltaCons,
                        AdaptSostenidoParentTicks,
                        AdaptSostenidoParentCons);
                const bool fronteraParentContinua =
                    AdaptSostenidoCandidatoReemplazo
                    && AdaptSostenidoNivelMinimo
                       >= AdaptSostenidoPhaseObjetivoFrames;
                if (segmentoValido
                    && fronteraParentContinua
                    && (parentHintSuficiente || parentMedidoExacto))
                {
                    publicarObservacionSostenida(
                        deltaCons, deltaTicks, deltaFrames,
                        AdaptSostenidoNivelInicioLogico,
                        AudioOutputAdaptiveSustainedParentRecovery);
                    ratioRateSostenido =
                        limitar(AdaptSostenidoParentRatio);
                    rateSostenidoVentanaCons = deltaCons;
                    rateSostenidoVentanaTicks = deltaTicks;
                    rateSostenidoPhaseFrames = std::min(
                        nivelSeguro, std::max(0, nivelPostConsumo));
                    parentCapacityTransferenciaTipada =
                        AdaptSostenidoCandidatoReemplazo
                        && AdaptSostenidoDireccion > 0
                        && AdaptSostenidoFastHighWitnessActivo
                        && AdaptSostenidoFastHighWitnessCapacidad
                        && AdaptSostenidoFastHighWitnessOwnerGeneracion
                           == AdaptSostenidoOwnerGeneracion
                        && !intervaloHostPerdido
                        && dropsEstaEscritura == 0;
                    transaccionSostenida =
                        TransaccionSostenida::LiberarParent;
                    AdaptSostenidoCandidatoActivo = false;
                    AdaptSostenidoSegmentoUnoCompleto = false;
                }
                else if (segmentoValido)
                {
                    publicarObservacionSostenida(
                        deltaCons, deltaTicks, deltaFrames,
                        AdaptSostenidoNivelInicioLogico,
                        AdaptSostenidoCandidatoReemplazo
                        ? AudioOutputAdaptiveSustainedReplacementSegmentOne
                        : AudioOutputAdaptiveSustainedAcquireSegmentOne);
                    AdaptSostenidoSegmentoUnoCompleto = true;
                    AdaptSostenidoSegmentoUnoTicks = ticksAlConsumo;
                    AdaptSostenidoSegmentoUnoCons = hostConsumed;
                    AdaptSostenidoSegmentoUnoFrames = framesAlConsumo;
                    AdaptSostenidoSegmentoUnoFramesPublicados =
                        AdaptFramesProducidosTotal;
                    AdaptSostenidoSegmentoUnoNivel = nivelPostConsumo;
                    AdaptSostenidoSegmentoUnoNivelLogico =
                        nivelPostConsumoLogico;
                    AdaptSostenidoSegmentoUnoNivelEscrituraLogico =
                        nivelPostEscrituraLogico;
                    AdaptSostenidoSegmentoUnoContinuidadEpoch =
                        continuidadEpoch;
                    AdaptSostenidoSegmentoUnoResetEpoch = resetSolicitado;
                    AdaptSostenidoSegmentoUnoHintSeenEpoch =
                        AdaptHintVistoProductor;
                    AdaptSostenidoNivelMinimo = nivelPostConsumo;
                    AdaptSostenidoNivelMaximo = nivelPostEscritura;
                    AdaptSostenidoNivelMinimoLogico =
                        nivelPostConsumoLogico;
                    AdaptSostenidoNivelMaximoLogico =
                        nivelPostEscrituraLogico;
                }
                else
                {
                    AdaptSostenidoCandidatoActivo = false;
                    AdaptSostenidoCandidatoReemplazo = false;
                    AdaptSostenidoSegmentoUnoCompleto = false;
                    if (!AdaptSostenidoOwnerActivo)
                        AdaptSostenidoParentValido = false;

                }
            }
        }
        else if (AdaptSostenidoSegmentoUnoCons >= AdaptSostenidoConsInicio
                 && AdaptSostenidoSegmentoUnoTicks >= AdaptSostenidoTicksInicio
                 && AdaptSostenidoSegmentoUnoFrames >= AdaptSostenidoFramesInicio
                 && hostConsumed >= AdaptSostenidoSegmentoUnoCons
                 && ticksAlConsumo >= AdaptSostenidoSegmentoUnoTicks
                 && framesAlConsumo >= AdaptSostenidoSegmentoUnoFrames)
        {
            const u64 deltaDosCons =
                hostConsumed - AdaptSostenidoSegmentoUnoCons;
            if (deltaDosCons >= ventanaLenta
                && ticksAlConsumo > AdaptSostenidoSegmentoUnoTicks)
            {
                const u64 deltaDosTicks =
                    ticksAlConsumo - AdaptSostenidoSegmentoUnoTicks;
                const u64 deltaDosFrames =
                    framesAlConsumo - AdaptSostenidoSegmentoUnoFrames;
                const bool referenciaInicialMedida =
                    !AdaptSostenidoCandidatoReemplazo
                    && AdaptSostenidoParentValido
                    && !AdaptSostenidoParentEsHint;
                const int direccionDos = AdaptSostenidoCandidatoReemplazo
                    ? compararRelacionHT(
                        deltaDosTicks, deltaDosCons,
                        AdaptSostenidoOwnerTicks,
                        AdaptSostenidoOwnerCons)
                    : referenciaInicialMedida
                    ? compararRelacionHT(
                        deltaDosTicks, deltaDosCons,
                        AdaptSostenidoParentTicks,
                        AdaptSostenidoParentCons)
                    : (medirRatio(deltaDosTicks, deltaDosCons)
                           < AdaptSostenidoReferenciaRatio ? -1
                       : medirRatio(deltaDosTicks, deltaDosCons)
                           > AdaptSostenidoReferenciaRatio ? 1 : 0);
                const bool segmentoDosValido =
                    conservaPCM(
                        deltaDosFrames,
                        AdaptSostenidoSegmentoUnoNivelLogico,
                        deltaDosCons,
                        nivelPostConsumoLogico)
                    && direccionDos == AdaptSostenidoDireccion;
                const u64 deltaUnoCons =
                    AdaptSostenidoSegmentoUnoCons
                    - AdaptSostenidoConsInicio;
                const u64 deltaUnoTicks =
                    AdaptSostenidoSegmentoUnoTicks
                    - AdaptSostenidoTicksInicio;
                const int direccionDosContraUno = compararRelacionHT(
                    deltaDosTicks, deltaDosCons,
                    deltaUnoTicks, deltaUnoCons);

                const int nivelUnoFisico = std::max(
                    0, AdaptSostenidoSegmentoUnoNivel);
                const int headroomUno = std::max(
                    0, nivelSeguro - nivelUnoFisico);
                const int suffixProductorPendiente = std::max(
                    0, nivelPostEscritura - nivelPostConsumo);
                const int reservaSuffix = std::min(
                    headroomUno, suffixProductorPendiente);
                const int nivelUnoConSuffix =
                    nivelUnoFisico + reservaSuffix;
                const bool segmentosFisicamenteCompatibles =
                    !AdaptSostenidoCandidatoReemplazo
                    || direccionDosContraUno == 0
                    || !cambioSostenidoAgotaRelacionExacta(
                        deltaDosTicks, deltaDosCons,
                        nivelUnoConSuffix,
                        direccionDosContraUno,
                        deltaUnoTicks, deltaUnoCons);
                const bool overrideAltoVigenteSlide =
                    AdaptSostenidoOverrideActivo
                    && AdaptSostenidoOverrideDireccion > 0
                    && AdaptSostenidoOverrideTicks > 0
                    && AdaptSostenidoOverrideCons > 0;
                const bool referenciaWitnessSlideVigente =
                    AdaptSostenidoFastHighWitnessActivo
                    && AdaptSostenidoFastHighWitnessOwnerGeneracion
                       == AdaptSostenidoOwnerGeneracion
                    && AdaptSostenidoFastHighWitnessReferenciaEsOverride
                       == overrideAltoVigenteSlide
                    && (overrideAltoVigenteSlide
                        ? AdaptSostenidoFastHighWitnessReferenciaTicks
                             == AdaptSostenidoOverrideTicks
                          && AdaptSostenidoFastHighWitnessReferenciaCons
                             == AdaptSostenidoOverrideCons
                        : AdaptSostenidoOwnerActivo
                          && AdaptSostenidoFastHighWitnessReferenciaTicks
                             == AdaptSostenidoOwnerTicks
                          && AdaptSostenidoFastHighWitnessReferenciaCons
                             == AdaptSostenidoOwnerCons);
                const bool contadoresWitnessSlideMonotonos =
                    referenciaWitnessSlideVigente
                    && ticksAlConsumo
                       >= AdaptSostenidoFastHighWitnessTicksInicio
                    && hostConsumed
                       >= AdaptSostenidoFastHighWitnessConsInicio
                    && framesAlConsumo
                       >= AdaptSostenidoFastHighWitnessFramesInicio;
                const bool witnessSlideConservaPCM =
                    contadoresWitnessSlideMonotonos
                    && conservaPCM(
                        framesAlConsumo
                            - AdaptSostenidoFastHighWitnessFramesInicio,
                        AdaptSostenidoFastHighWitnessNivelInicioLogico,
                        hostConsumed
                            - AdaptSostenidoFastHighWitnessConsInicio,
                        nivelPostConsumoLogico);
                const bool slidePruebaCapacidad =
                    AdaptSostenidoCandidatoReemplazo
                    && witnessSlideConservaPCM
                    && !intervaloHostPerdido
                    && dropsEstaEscritura == 0
                    && direccionDosContraUno > 0
                    && cambioSostenidoAgotaCapacidadExacta(
                        deltaDosTicks, deltaDosCons,
                        nivelPostEscritura,
                        ventanaLenta,
                        deltaUnoTicks, deltaUnoCons);
                const u64 deltaCons =
                    hostConsumed - AdaptSostenidoConsInicio;
                const u64 deltaTicks =
                    ticksAlConsumo - AdaptSostenidoTicksInicio;
                const u64 deltaFrames =
                    framesAlConsumo - AdaptSostenidoFramesInicio;
                const double ratioSostenido = medirRatio(
                    deltaTicks, deltaCons);
                const bool conservacionTotal = conservaPCM(
                    deltaFrames,
                    AdaptSostenidoNivelInicioLogico,
                    deltaCons,
                    nivelPostConsumoLogico);
                double ratioOwnerCandidato = limitar(ratioSostenido);
                if (AdaptHint > 0.0 && ratioOwnerCandidato > AdaptHint)
                    ratioOwnerCandidato = AdaptHint;
                const bool identidadOwnerAplicable =
                    ratioOwnerCandidato == ratioSostenido;
                const bool fronteraAgotada =
                    AdaptSostenidoCandidatoReemplazo
                    ? cambioSostenidoAgotaRelacionExacta(
                        deltaTicks, deltaCons,
                        AdaptSostenidoNivelInicio,
                        AdaptSostenidoDireccion,
                        AdaptSostenidoOwnerTicks,
                        AdaptSostenidoOwnerCons)
                    : referenciaInicialMedida
                    ? cambioSostenidoAgotaRelacionExacta(
                        deltaTicks, deltaCons,
                        AdaptSostenidoNivelInicio,
                        AdaptSostenidoDireccion,
                        AdaptSostenidoParentTicks,
                        AdaptSostenidoParentCons)
                    : cambioSostenidoAgotaFrontera(
                        deltaTicks, deltaCons,
                        AdaptSostenidoNivelInicio,
                        AdaptSostenidoReferenciaRatio,
                        AdaptSostenidoDireccion);
                bool deslizarSegmentoDosComoUno = false;
                if (segmentoDosValido
                    && segmentosFisicamenteCompatibles
                    && conservacionTotal
                    && fronteraAgotada
                    && identidadOwnerAplicable
                    && std::isfinite(ratioSostenido)
                    && ratioSostenido > 0.0)
                {
                    publicarObservacionSostenida(
                        deltaDosCons, deltaDosTicks, deltaDosFrames,
                        AdaptSostenidoSegmentoUnoNivelLogico,
                        AdaptSostenidoCandidatoReemplazo
                        ? AudioOutputAdaptiveSustainedReplacementSegmentTwo
                        : AudioOutputAdaptiveSustainedAcquireSegmentTwo);

                    ++AdaptSostenidoOwnerGeneracion;
                    AdaptSostenidoOwnerActivo = true;

                    AdaptSostenidoOwnerRatio = ratioOwnerCandidato;
                    AdaptSostenidoOwnerTicks = deltaTicks;
                    AdaptSostenidoOwnerCons = deltaCons;
                    AdaptSostenidoOwnerCandidatoGeneracion =
                        AdaptSostenidoCandidatoGeneracion;
                    AdaptSostenidoPhaseObjetivoFrames = std::min(
                        nivelSeguro, std::max(0, nivelPostConsumo));
                    AdaptSostenidoOverrideActivo = false;
                    AdaptSostenidoOverrideDireccion = 0;
                    AdaptSostenidoOverrideTicks = 0;
                    AdaptSostenidoOverrideCons = 0;
                    limpiarFastLowWitness(
                        AudioOutputAdaptiveFastLowWitnessClearSustainedTransition);
                    limpiarFastHighEscrow(
                        AudioOutputAdaptiveFastHighEscrowClearOwnerGeneration);
                    limpiarFastHighWitness();
                    AdaptSostenidoRecoveryVerificando = false;
                    ratioRateSostenido = ratioOwnerCandidato;
                    rateSostenidoVentanaCons = deltaCons;
                    rateSostenidoVentanaTicks = deltaTicks;
                    rateSostenidoPhaseFrames =
                        AdaptSostenidoPhaseObjetivoFrames;
                    transaccionSostenida =
                        AdaptSostenidoCandidatoReemplazo
                        ? TransaccionSostenida::ReemplazarOwner
                        : TransaccionSostenida::AdquirirOwner;
                }
                else if (AdaptSostenidoCandidatoReemplazo
                         && segmentoDosValido
                         && !segmentosFisicamenteCompatibles)
                {

                    publicarObservacionSostenida(
                        deltaDosCons, deltaDosTicks, deltaDosFrames,
                        AdaptSostenidoSegmentoUnoNivelLogico,
                        AudioOutputAdaptiveSustainedReplacementSegmentOne);
                    AdaptSostenidoTicksInicio =
                        AdaptSostenidoSegmentoUnoTicks;
                    AdaptSostenidoConsInicio =
                        AdaptSostenidoSegmentoUnoCons;
                    AdaptSostenidoFramesInicio =
                        AdaptSostenidoSegmentoUnoFrames;
                    AdaptSostenidoFramesPublicadosInicio =
                        AdaptSostenidoSegmentoUnoFramesPublicados;
                    AdaptSostenidoNivelInicio =
                        AdaptSostenidoSegmentoUnoNivel;
                    AdaptSostenidoNivelInicioLogico =
                        AdaptSostenidoSegmentoUnoNivelLogico;
                    AdaptSostenidoNivelEscrituraInicioLogico =
                        AdaptSostenidoSegmentoUnoNivelEscrituraLogico;
                    AdaptSostenidoContinuidadEpochInicio =
                        AdaptSostenidoSegmentoUnoContinuidadEpoch;
                    AdaptSostenidoResetEpochInicio =
                        AdaptSostenidoSegmentoUnoResetEpoch;
                    AdaptSostenidoHintSeenEpochInicio =
                        AdaptSostenidoSegmentoUnoHintSeenEpoch;
                    AdaptSostenidoSegmentoUnoTicks = ticksAlConsumo;
                    AdaptSostenidoSegmentoUnoCons = hostConsumed;
                    AdaptSostenidoSegmentoUnoFrames = framesAlConsumo;
                    AdaptSostenidoSegmentoUnoFramesPublicados =
                        AdaptFramesProducidosTotal;
                    AdaptSostenidoSegmentoUnoNivel = nivelPostConsumo;
                    AdaptSostenidoSegmentoUnoNivelLogico =
                        nivelPostConsumoLogico;
                    AdaptSostenidoSegmentoUnoNivelEscrituraLogico =
                        nivelPostEscrituraLogico;
                    AdaptSostenidoSegmentoUnoContinuidadEpoch =
                        continuidadEpoch;
                    AdaptSostenidoSegmentoUnoResetEpoch = resetSolicitado;
                    AdaptSostenidoSegmentoUnoHintSeenEpoch =
                        AdaptHintVistoProductor;
                    AdaptSostenidoNivelMinimo = std::min(
                        AdaptSostenidoNivelInicio, nivelPostConsumo);
                    AdaptSostenidoNivelMaximo = std::max(
                        std::max(AdaptSostenidoNivelInicio,
                                 nivelPostConsumo),
                        nivelPostEscritura);
                    AdaptSostenidoNivelMinimoLogico = std::min(
                        AdaptSostenidoNivelInicioLogico,
                        nivelPostConsumoLogico);
                    AdaptSostenidoNivelMaximoLogico = std::max(
                        std::max(AdaptSostenidoNivelInicioLogico,
                                 nivelPostConsumoLogico),
                        nivelPostEscrituraLogico);
                    if (slidePruebaCapacidad)
                    {

                        AdaptSostenidoFastHighWitnessCapacidad = true;
                    }
                    deslizarSegmentoDosComoUno = true;
                }
                else if (!AdaptSostenidoOwnerActivo)
                {
                    AdaptSostenidoParentValido = false;
                }
                if (!deslizarSegmentoDosComoUno)
                {
                    AdaptSostenidoCandidatoActivo = false;
                    AdaptSostenidoCandidatoReemplazo = false;
                    AdaptSostenidoSegmentoUnoCompleto = false;
                }
            }
        }
        else
        {
            AdaptSostenidoCandidatoActivo = false;
            AdaptSostenidoCandidatoReemplazo = false;
            AdaptSostenidoSegmentoUnoCompleto = false;
            if (!AdaptSostenidoOwnerActivo)
                AdaptSostenidoParentValido = false;

        }
    }

    bool transaccionRateSostenida = false;
    if (transaccionSostenida != TransaccionSostenida::Ninguna)
    {
        const bool liberarRateSostenido = transaccionSostenida
            == TransaccionSostenida::LiberarParent;
        const bool usarOwner = transaccionSostenida
                == TransaccionSostenida::AdquirirOwner
            || transaccionSostenida
                == TransaccionSostenida::ReemplazarOwner
            || transaccionSostenida
                == TransaccionSostenida::RestaurarOwner;
        const bool liberarAHint = liberarRateSostenido
            && AdaptSostenidoParentEsHint;
        const u64 parentTicks = AdaptSostenidoParentTicks;
        const u64 parentCons = AdaptSostenidoParentCons;
        u32 tipoTransicionSostenida =
            AudioOutputAdaptiveSustainedTransitionNone;
        switch (transaccionSostenida)
        {
        case TransaccionSostenida::AdquirirOwner:
            tipoTransicionSostenida =
                AudioOutputAdaptiveSustainedAcquire;
            break;
        case TransaccionSostenida::ReemplazarOwner:
            tipoTransicionSostenida =
                AudioOutputAdaptiveSustainedReplace;
            break;
        case TransaccionSostenida::OverrideFast:
            tipoTransicionSostenida =
                AudioOutputAdaptiveSustainedFastOverrideTransition;
            break;
        case TransaccionSostenida::RestaurarOwner:
            tipoTransicionSostenida =
                AudioOutputAdaptiveSustainedRestoreOwner;
            break;
        case TransaccionSostenida::LiberarParent:
            tipoTransicionSostenida =
                AudioOutputAdaptiveSustainedReleaseParent;
            break;
        case TransaccionSostenida::Ninguna:
            break;
        }
        ++telemetry.sustainedTransitionCount;
        telemetry.lastSustainedTransitionUpdateId = telemetry.updateId;
        telemetry.lastSustainedTransitionKind = tipoTransicionSostenida;
        telemetry.lastSustainedTransitionOwnerGeneration =
            AdaptSostenidoOwnerGeneracion;
        if (ratioFastAceptado
            && transaccionSostenida != TransaccionSostenida::OverrideFast)
        {

            decisionFlags &= ~AudioOutputAdaptiveFastAccepted;
        }

        if (usarOwner)
        {
            ratioRateSostenido = AdaptSostenidoOwnerRatio;
            rateSostenidoVentanaCons = AdaptSostenidoOwnerCons;
            rateSostenidoVentanaTicks = AdaptSostenidoOwnerTicks;
            rateSostenidoPhaseFrames =
                AdaptSostenidoPhaseObjetivoFrames;
            AdaptRatio = limitar(AdaptSostenidoOwnerRatio);
        }
        else if (liberarRateSostenido)
        {
            AdaptRatio = limitar(ratioRateSostenido);
        }
        else
        {

            rateSostenidoVentanaCons = fastOverrideSostenidoCons;
            rateSostenidoVentanaTicks = fastOverrideSostenidoTicks;
            rateSostenidoPhaseFrames =
                AdaptSostenidoPhaseObjetivoFrames;
        }
        if (liberarAHint
            && parentCapacityTransferenciaTipada
            && AdaptSostenidoOwnerGeneracion != 0
            && AdaptRatio == AdaptHint)
        {

            armarParentCapacity(
                AdaptSostenidoOwnerGeneracion, AdaptRatio);
            parentCapacityNacidaEsteUpdate = true;
        }
        AdaptProbe = false;
        limpiarFastLowWitness(
            AudioOutputAdaptiveFastLowWitnessClearSustainedTransition);
        AdaptFastLowPendiente = false;
        AdaptFastCambioTargetPendiente = false;
        rebasarVentanaFast(
            !intervaloHostPerdido && dropsEstaEscritura == 0);
        AdaptSlowTicksInicio = ticksAlConsumo;
        AdaptSlowConsInicio = hostConsumed;

        AdaptRateRollbackPendiente = false;
        AdaptRateRollbackVerificando = false;
        AdaptRateRollbackPadreEsHint = false;
        AdaptRateRollbackRatio = 1.0;
        AdaptRateRollbackObjetivoFrames = 0;
        AdaptRateRollbackFronteraFrames = 0;
        AdaptRateRollbackPadreTicks = 0;
        AdaptRateRollbackPadreCons = 0;
        AdaptRateRollbackTicksInicio = ticksAlConsumo;
        AdaptRateRollbackConsInicio = hostConsumed;
        AdaptRateBajoConfirmado = false;
        AdaptRatePadreCertificado = false;
        AdaptRatePadreTicks = 0;
        AdaptRatePadreCons = 0;
        if (!liberarRateSostenido
            && AdaptSostenidoParentEsHint)
        {
            AdaptHintPadreCertificado = false;
            AdaptHintCertTicksInicio = ticksAlConsumo;
            AdaptHintCertConsInicio = hostConsumed;
        }

        AdaptPhaseBalanceFrames = 0.0;
        AdaptPhaseEpisodioActivo = false;
        AdaptPhaseObjetivoFrames = std::min(
            nivelSeguro, std::max(0, rateSostenidoPhaseFrames));
        AdaptPhaseObjetivoActivo = true;
        AdaptPhaseRebasePendiente = false;
        AdaptPhaseRebaseBandViolada = false;
        AdaptPhaseRebaseMinFrames = -1;
        AdaptPhaseOrigenRecuperacionRate = false;

        if (liberarRateSostenido)
        {
            AdaptSostenidoCandidatoActivo = false;
            AdaptSostenidoCandidatoReemplazo = false;
            AdaptSostenidoSegmentoUnoCompleto = false;
            AdaptSostenidoOwnerActivo = false;
            AdaptSostenidoOwnerRatio = 0.0;
            AdaptSostenidoOwnerTicks = 0;
            AdaptSostenidoOwnerCons = 0;
            AdaptSostenidoPhaseObjetivoFrames = 0;
            AdaptSostenidoParentValido = false;
            AdaptSostenidoParentEsHint = false;
            AdaptSostenidoParentRatio = 0.0;
            AdaptSostenidoParentTicks = 0;
            AdaptSostenidoParentCons = 0;
            AdaptSostenidoOverrideActivo = false;
            AdaptSostenidoOverrideDireccion = 0;
            AdaptSostenidoOverrideTicks = 0;
            AdaptSostenidoOverrideCons = 0;
            limpiarFastLowWitness(
                AudioOutputAdaptiveFastLowWitnessClearSustainedTransition);
            limpiarFastHighEscrow(
                AudioOutputAdaptiveFastHighEscrowClearOwnerGeneration);
            limpiarFastHighWitness();
            AdaptSostenidoRecoveryVerificando = false;
            AdaptRateBajoConfirmado = !liberarAHint;
            AdaptRatePadreCertificado = !liberarAHint;
            AdaptRatePadreTicks = liberarAHint ? 0 : parentTicks;
            AdaptRatePadreCons = liberarAHint ? 0 : parentCons;
            if (liberarAHint)
                certificarHintPadre(rateSostenidoVentanaCons,
                                    rateSostenidoVentanaTicks);
            AdaptPhaseOrigenRecuperacionRate = liberarAHint;
        }
        else if (transaccionSostenida
                 == TransaccionSostenida::RestaurarOwner)
        {

            AdaptSostenidoOverrideActivo = false;
            AdaptSostenidoOverrideDireccion = 0;
            AdaptSostenidoOverrideTicks = 0;
            AdaptSostenidoOverrideCons = 0;
            limpiarFastLowWitness(
                AudioOutputAdaptiveFastLowWitnessClearSustainedTransition);
            limpiarFastHighEscrow(
                AudioOutputAdaptiveFastHighEscrowClearRate);
            limpiarFastHighWitness();
        }

        const bool preferirRapido = liberarRateSostenido
            || transaccionSostenida
               == TransaccionSostenida::OverrideFast;
        iniciarActuadorRate(
            rateSostenidoVentanaCons > 0
                ? rateSostenidoVentanaCons : deltaFastCons,
            rateSostenidoVentanaTicks > 0
                ? rateSostenidoVentanaTicks : deltaFastTicks,
            preferirRapido);
        aplicarInmediato = !AdaptRateActuadorPendiente;
        transaccionRateSostenida = true;
    }

    if (AdaptSostenidoPhaseProvisionalActivo)
    {
        const double reservaPhaseObservadaRaw = medirReservaPublicacion(
            AdaptSostenidoPhaseProvisionalSkew);
        AdaptSostenidoPhaseProvisionalReservaPublicacionFrames =
            std::isfinite(reservaPhaseObservadaRaw)
            && reservaPhaseObservadaRaw >= 0.0
            && reservaPhaseObservadaRaw
               <= (double)std::numeric_limits<u64>::max()
            ? static_cast<u64>(reservaPhaseObservadaRaw) : 0;

        AdaptSostenidoPhaseProvisionalPublicacionesReservadas =
            AdaptSostenidoSegmentoUnoCompleto ? 2u : 1u;
        const bool generacionVigente =
            AdaptSostenidoCandidatoActivo
            && !AdaptSostenidoCandidatoReemplazo
            && !AdaptSostenidoOwnerActivo
            && !AdaptRateActuadorPendiente
            && AdaptSostenidoCandidatoGeneracion
               == AdaptSostenidoPhaseProvisionalCandidatoGeneracion
            && AdaptSostenidoOwnerGeneracion
               == AdaptSostenidoPhaseProvisionalOwnerGeneracion;
        const bool epochsVigentes =
            continuidadEpoch
               == AdaptSostenidoPhaseProvisionalContinuidadEpoch
            && resetSolicitado
               == AdaptSostenidoPhaseProvisionalResetEpoch
            && resetConfirmado
               == AdaptSostenidoPhaseProvisionalResetEpoch
            && AdaptHintVistoProductor
               == AdaptSostenidoPhaseProvisionalHintSeenEpoch;
        const bool transporteVigente =
            dropsEstaEscritura == 0
            && underrunsActuales
               == AdaptSostenidoPhaseProvisionalUnderrunsInicio
            && OutputSpillDroppedFramesTotal
               == AdaptSostenidoPhaseProvisionalSpillDropsInicio
            && OutputSpillAllocationFailureCount
               == AdaptSostenidoPhaseProvisionalSpillAllocationFailuresInicio
            && OutputSpillFrames == 0
            && nivelPostConsumo >= 0
            && nivelPostEscritura >= nivelPostConsumo
            && nivelPostConsumoLogico
               == static_cast<u64>(nivelPostConsumo)
            && nivelPostEscrituraLogico
               == static_cast<u64>(nivelPostEscritura);

        const u64 ledgerTicksInicio = AdaptSostenidoSegmentoUnoCompleto
            ? AdaptSostenidoSegmentoUnoTicks : AdaptSostenidoTicksInicio;
        const u64 ledgerConsInicio = AdaptSostenidoSegmentoUnoCompleto
            ? AdaptSostenidoSegmentoUnoCons : AdaptSostenidoConsInicio;
        const u64 ledgerFramesInicio = AdaptSostenidoSegmentoUnoCompleto
            ? AdaptSostenidoSegmentoUnoFrames : AdaptSostenidoFramesInicio;
        const u64 ledgerFramesPublicadosInicio =
            AdaptSostenidoSegmentoUnoCompleto
            ? AdaptSostenidoSegmentoUnoFramesPublicados
            : AdaptSostenidoFramesPublicadosInicio;
        const u64 ledgerNivelConsumoInicio =
            AdaptSostenidoSegmentoUnoCompleto
            ? AdaptSostenidoSegmentoUnoNivelLogico
            : AdaptSostenidoNivelInicioLogico;
        const u64 ledgerNivelEscrituraInicio =
            AdaptSostenidoSegmentoUnoCompleto
            ? AdaptSostenidoSegmentoUnoNivelEscrituraLogico
            : AdaptSostenidoNivelEscrituraInicioLogico;
        const bool contadoresLedgerMonotonos =
            ticksAlConsumo >= ledgerTicksInicio
            && hostConsumed >= ledgerConsInicio
            && framesAlConsumo >= ledgerFramesInicio
            && AdaptFramesProducidosTotal >= ledgerFramesPublicadosInicio;
        const u64 deltaLedgerCons = contadoresLedgerMonotonos
            ? hostConsumed - ledgerConsInicio : 0;
        const u64 deltaLedgerTicks = contadoresLedgerMonotonos
            ? ticksAlConsumo - ledgerTicksInicio : 0;
        const u64 deltaLedgerFrames = contadoresLedgerMonotonos
            ? framesAlConsumo - ledgerFramesInicio : 0;
        const u64 deltaLedgerFramesPublicados = contadoresLedgerMonotonos
            ? AdaptFramesProducidosTotal - ledgerFramesPublicadosInicio : 0;
        const bool ledgerConservaConsumo = contadoresLedgerMonotonos
            && conservaPCM(
                deltaLedgerFrames, ledgerNivelConsumoInicio,
                deltaLedgerCons, nivelPostConsumoLogico);
        const bool ledgerConservaPublicacion = contadoresLedgerMonotonos
            && conservaPCM(
                deltaLedgerFramesPublicados, ledgerNivelEscrituraInicio,
                deltaLedgerCons, nivelPostEscrituraLogico);

        if (!generacionVigente || !epochsVigentes || !transporteVigente
            || !ledgerConservaConsumo || !ledgerConservaPublicacion)
        {

            u32 razonesClearPhase =
                AudioOutputAdaptiveSustainedProvisionalPhaseClearNone;
            if (!AdaptSostenidoCandidatoActivo)
                razonesClearPhase |=
                    AudioOutputAdaptiveSustainedProvisionalPhaseClearCandidateInactive;
            if (AdaptSostenidoCandidatoReemplazo)
                razonesClearPhase |=
                    AudioOutputAdaptiveSustainedProvisionalPhaseClearCandidateReplacement;
            if (AdaptSostenidoOwnerActivo)
                razonesClearPhase |=
                    AudioOutputAdaptiveSustainedProvisionalPhaseClearOwnerActive;
            if (AdaptRateActuadorPendiente)
                razonesClearPhase |=
                    AudioOutputAdaptiveSustainedProvisionalPhaseClearRateActuator;
            if (AdaptSostenidoCandidatoGeneracion
                != AdaptSostenidoPhaseProvisionalCandidatoGeneracion)
            {
                razonesClearPhase |=
                    AudioOutputAdaptiveSustainedProvisionalPhaseClearCandidateGeneration;
            }
            if (AdaptSostenidoOwnerGeneracion
                != AdaptSostenidoPhaseProvisionalOwnerGeneracion)
            {
                razonesClearPhase |=
                    AudioOutputAdaptiveSustainedProvisionalPhaseClearOwnerGeneration;
            }
            if (continuidadEpoch
                != AdaptSostenidoPhaseProvisionalContinuidadEpoch)
            {
                razonesClearPhase |=
                    AudioOutputAdaptiveSustainedProvisionalPhaseClearContinuity;
            }
            if (resetSolicitado
                    != AdaptSostenidoPhaseProvisionalResetEpoch
                || resetConfirmado
                    != AdaptSostenidoPhaseProvisionalResetEpoch)
            {
                razonesClearPhase |=
                    AudioOutputAdaptiveSustainedProvisionalPhaseClearReset;
            }
            if (AdaptHintVistoProductor
                != AdaptSostenidoPhaseProvisionalHintSeenEpoch)
            {
                razonesClearPhase |=
                    AudioOutputAdaptiveSustainedProvisionalPhaseClearHintChange;
            }
            if (dropsEstaEscritura != 0)
                razonesClearPhase |=
                    AudioOutputAdaptiveSustainedProvisionalPhaseClearDrop;
            if (underrunsActuales
                != AdaptSostenidoPhaseProvisionalUnderrunsInicio)
            {
                razonesClearPhase |=
                    AudioOutputAdaptiveSustainedProvisionalPhaseClearUnderrun;
            }
            if (OutputSpillDroppedFramesTotal
                    != AdaptSostenidoPhaseProvisionalSpillDropsInicio
                || OutputSpillAllocationFailureCount
                    != AdaptSostenidoPhaseProvisionalSpillAllocationFailuresInicio
                || OutputSpillFrames != 0)
            {
                razonesClearPhase |=
                    AudioOutputAdaptiveSustainedProvisionalPhaseClearSpill;
            }
            if (nivelPostConsumo < 0
                || nivelPostEscritura < nivelPostConsumo
                || nivelPostConsumoLogico
                   != static_cast<u64>(std::max(0, nivelPostConsumo))
                || nivelPostEscrituraLogico
                   != static_cast<u64>(std::max(0, nivelPostEscritura)))
            {
                razonesClearPhase |=
                    AudioOutputAdaptiveSustainedProvisionalPhaseClearStorage;
            }
            if (!ledgerConservaConsumo)
                razonesClearPhase |=
                    AudioOutputAdaptiveSustainedProvisionalPhaseClearConsumedPcm;
            if (!ledgerConservaPublicacion)
                razonesClearPhase |=
                    AudioOutputAdaptiveSustainedProvisionalPhaseClearPublishedPcm;
            limpiarPhaseProvisionalOwnerFree(razonesClearPhase);
        }
        else if (deltaLedgerCons >= ventanaRate
                 && deltaLedgerTicks >= ticksDosFramesPublicables
                 && deltaLedgerCons < ventanaLenta)
        {

            const double ratioLedger = medirRatio(
                deltaLedgerTicks, deltaLedgerCons);
            const double reservaPublicacion = medirReservaPublicacion(
                AdaptSostenidoPhaseProvisionalSkew);
            const u64 capacidadFrames =
                static_cast<u64>(OutputBufferSize - 1);
            const u64 nivelFrames = std::min<u64>(
                capacidadFrames,
                static_cast<u64>(std::max(0, nivelPostEscritura)));
            const double capacidad = (double)capacidadFrames;
            const double nivel = (double)nivelFrames;
            const double fronteraPhase = std::min(
                capacidad,
                (double)std::max(
                    0, AdaptSostenidoPhaseProvisionalNivelInicio));
            const bool reservaPublicacionValida =
                std::isfinite(reservaPublicacion)
                && reservaPublicacion >= 0.0
                && reservaPublicacion <= capacidad;
            const u64 reservaPublicacionFrames =
                reservaPublicacionValida
                ? static_cast<u64>(reservaPublicacion) : 0;
            const double headroomGastable = reservaPublicacionValida
                ? static_cast<double>(
                    AudioOutputExactMath::SpendablePhaseHeadroom(
                        nivelFrames, reservaPublicacionFrames,
                        AdaptSostenidoPhaseProvisionalPublicacionesReservadas,
                        capacidadFrames))
                : 0.0;
            const double backingGastable = std::isfinite(reservaPublicacion)
                ? std::max(
                    0.0, nivel - std::max(
                        fronteraPhase, reservaPublicacion))
                : 0.0;
            const double horizonte =
                (double)(ventanaLenta - deltaLedgerCons);
            double floorHeadroom = ratioLedger
                / (1.0 + headroomGastable / horizonte);
            if (headroomGastable > 0.0)
            {
                floorHeadroom = std::nextafter(
                    floorHeadroom,
                    std::numeric_limits<double>::infinity());
            }
            double ceilingBacking =
                AdaptSostenidoPhaseProvisionalParentRatio;
            if (backingGastable == 0.0)
            {

                ceilingBacking = ratioLedger;
            }
            else if (backingGastable < horizonte)
            {
                const double ceilingRaw = ratioLedger * horizonte
                    / (horizonte - backingGastable);
                ceilingBacking = std::nextafter(
                    ceilingRaw, ratioLedger);
            }

            const double factibleMinFisico = std::max(
                pisoSkew(), floorHeadroom);
            const double factibleMax = std::min({
                maxSkew,
                AdaptSostenidoPhaseProvisionalParentRatio,
                ceilingBacking});
            if (ratioLedger > 0.0
                && std::isfinite(ratioLedger)
                && std::isfinite(reservaPublicacion)
                && AdaptSostenidoPhaseProvisionalSkew > 0.0
                && std::isfinite(
                    AdaptSostenidoPhaseProvisionalSkew)
                && AdaptSostenidoPhaseProvisionalParentRatio > 0.0
                && std::isfinite(
                    AdaptSostenidoPhaseProvisionalParentRatio)
                && std::isfinite(floorHeadroom)
                && std::isfinite(ceilingBacking)
                && std::isfinite(factibleMinFisico)
                && std::isfinite(factibleMax)
                && factibleMinFisico <= factibleMax)
            {

                const double skewAnteriorPhase =
                    AdaptSostenidoPhaseProvisionalSkew;
                double skewFactible = std::clamp(
                    skewAnteriorPhase, factibleMinFisico, factibleMax);
                if (skewAnteriorPhase > factibleMax)
                {

                    skewFactible = std::clamp(
                        ratioLedger, factibleMinFisico, factibleMax);
                }
                AdaptSostenidoPhaseProvisionalSkew = skewFactible;
                const double reservaStepRaw =
                    medirReservaPublicacion(skewFactible);
                AdaptSostenidoPhaseProvisionalReservaPublicacionFrames =
                    std::isfinite(reservaStepRaw)
                    && reservaStepRaw >= 0.0
                    && reservaStepRaw
                       <= (double)std::numeric_limits<u64>::max()
                    ? static_cast<u64>(reservaStepRaw) : 0;
                if (skewFactible != skewAnteriorPhase)
                {
                    ++telemetry.sustainedProvisionalPhaseStepCount;
                    telemetry.lastSustainedProvisionalPhaseStepUpdateId =
                        telemetry.updateId;
                    telemetry.lastSustainedProvisionalPhaseStepCandidateGeneration =
                        AdaptSostenidoPhaseProvisionalCandidatoGeneracion;
                    telemetry.lastSustainedProvisionalPhaseStepPreviousSkew =
                        skewAnteriorPhase;
                    telemetry.lastSustainedProvisionalPhaseStepSkew =
                        skewFactible;
                    telemetry.lastSustainedProvisionalPhaseStepFrontierFrames =
                        AdaptSostenidoPhaseProvisionalFronteraFrames;
                    telemetry.lastSustainedProvisionalPhaseStepBackingFrames =
                        AdaptSostenidoPhaseProvisionalBackingFrames;
                    telemetry.lastSustainedProvisionalPhaseStepPublicationReserveFrames =
                        AdaptSostenidoPhaseProvisionalReservaPublicacionFrames;
                    telemetry.lastSustainedProvisionalPhaseStepReservedPublicationCount =
                        AdaptSostenidoPhaseProvisionalPublicacionesReservadas;
                }
            }
            else
            {

                decisionFlags |=
                    AudioOutputAdaptiveSlewBranch;
                provisionalPhaseIntervalInfeasibleEsteUpdate = true;
            }
        }
    }
    const bool faseRatePoseidaAntes = rateBajoAntesEstimadores
        || AdaptSostenidoOwnerActivo
        || (AdaptPhaseObjetivoActivo
            && AdaptPhaseOrigenRecuperacionRate)
        || (rebaseFaseAntesEstimadores
            && rebaseFaseOrigenRateAntes);
    const bool fastTransicionConFaseRate = !transaccionRateSostenida
        && ratioFastAceptado
        && !fastRateIdempotente
        && (medidaRateBajaConfirmada || faseRatePoseidaAntes);
    bool lineageRateRebasado = false;
    if (!transaccionRateSostenida && ratioFastAceptado)
    {
        if (!fastRateIdempotente)
        {
            const bool armarRollbackRatePadre =
                medidaRateBajaConfirmada
                && ((rateBajoAntesEstimadores
                     && ratePadreCertificadoAntesEstimadores)
                    || (AdaptHint > 0.0
                        && !AdaptProbe
                        && hintPadreCertificadoAntesEstimadores
                        && ratioAntesEstimadores == AdaptHint))
                && AdaptRatio < ratioAntesEstimadores;
            const bool padreMedidoCertificado =
                rateBajoAntesEstimadores
                && ratePadreCertificadoAntesEstimadores;
            const bool padreHintCertificado =
                !padreMedidoCertificado
                && hintPadreCertificadoAntesEstimadores
                && AdaptHint > 0.0
                && !AdaptProbe
                && ratioAntesEstimadores == AdaptHint;
            const bool reemplazarHintPorPadreMedido =
                AdaptRateRollbackPendiente
                && AdaptRateRollbackPadreEsHint
                && padreMedidoCertificado;
            if (armarRollbackRatePadre
                && (!AdaptRateRollbackPendiente
                    || reemplazarHintPorPadreMedido))
            {

                AdaptRateRollbackPendiente = true;
                AdaptRateRollbackVerificando = false;
                AdaptRateRollbackPadreEsHint =
                    padreHintCertificado;
                if (padreHintCertificado)
                {

                    AdaptHintPadreCertificado = false;
                    AdaptHintCertTicksInicio = ticksAlConsumo;
                    AdaptHintCertConsInicio = hostConsumed;
                }
                AdaptRateRollbackRatio = ratioAntesEstimadores;
                AdaptRateRollbackObjetivoFrames = objetivoRateCausal;
                AdaptRateRollbackFronteraFrames = 0;
                AdaptRateRollbackPadreTicks = padreMedidoCertificado
                    ? AdaptRatePadreTicks : 0;
                AdaptRateRollbackPadreCons = padreMedidoCertificado
                    ? AdaptRatePadreCons : 0;
                AdaptRateRollbackTicksInicio = ticksAlConsumo;
                AdaptRateRollbackConsInicio = hostConsumed;
            }
            else if (AdaptRateRollbackPendiente
                     && !medidaRateBajaConfirmada
                     && AdaptRatio > AdaptRateRollbackRatio)
            {

                AdaptRateRollbackPendiente = false;
                AdaptRateRollbackVerificando = false;
                AdaptRateRollbackPadreEsHint = false;
                AdaptRateRollbackFronteraFrames = 0;
                AdaptRateRollbackPadreTicks = 0;
                AdaptRateRollbackPadreCons = 0;
            }
            if (hintPadreCertificadoAntesEstimadores
                && AdaptHint > 0.0
                && ratioAntesEstimadores == AdaptHint
                && AdaptRatio < ratioAntesEstimadores)
            {

                AdaptHintPadreCertificado = false;
                AdaptHintCertTicksInicio = ticksAlConsumo;
                AdaptHintCertConsInicio = hostConsumed;
            }
            u64 ticksCandidata = 0;
            u64 consumoCandidata = 0;
            if (deltaFastTicks > deltaActTicks
                && deltaFastCons > deltaActCons)
            {
                ticksCandidata = deltaFastTicks - deltaActTicks;
                consumoCandidata = deltaFastCons - deltaActCons;
            }
            const bool extensionHTIdempotente = mismaRelacionHT(
                ticksCandidata, consumoCandidata,
                deltaActTicks, deltaActCons);
            AdaptRatePadreCertificado =
                medidaRateBajaConfirmada
                && cambioTargetPendienteAntes
                && !intervaloHostPerdido
                && dropsEstaEscritura == 0
                && extensionHTIdempotente;
            AdaptRatePadreTicks = AdaptRatePadreCertificado
                ? deltaFastTicks : 0;
            AdaptRatePadreCons = AdaptRatePadreCertificado
                ? deltaFastCons : 0;
            AdaptRateBajoConfirmado = medidaRateBajaConfirmada;
            lineageRateRebasado = true;
        }
    }
    else if (!transaccionRateSostenida
             && ratioSlowAplicado
             && (decisionFlags
                 & AudioOutputAdaptiveLowConfirmed) != 0)
    {
        if (hintPadreCertificadoAntesEstimadores
            && AdaptHint > 0.0
            && ratioAntesEstimadores == AdaptHint
            && AdaptRatio < ratioAntesEstimadores)
        {
            AdaptHintPadreCertificado = false;
            AdaptHintCertTicksInicio = ticksAlConsumo;
            AdaptHintCertConsInicio = hostConsumed;
        }
        AdaptRateBajoConfirmado = true;
        AdaptRatePadreCertificado = false;
        AdaptRatePadreTicks = 0;
        AdaptRatePadreCons = 0;
        lineageRateRebasado = true;
    }
    else if (!transaccionRateSostenida
             && ratioSlowAplicado
             && AdaptRateBajoConfirmado
             && (decisionFlags
                 & AudioOutputAdaptiveRatioCappedByHint) != 0)
    {
        AdaptRateBajoConfirmado = false;
        AdaptRatePadreCertificado = false;
        AdaptRatePadreTicks = 0;
        AdaptRatePadreCons = 0;
        lineageRateRebasado = true;
    }

    if (AdaptHint > 0.0
        && !AdaptProbe
        && ratioAntesEstimadores != AdaptHint
        && AdaptRatio == AdaptHint)
    {

        AdaptHintPadreCertificado = false;
        AdaptHintCertTicksInicio = ticksAlConsumo;
        AdaptHintCertConsInicio = hostConsumed;
    }

    const bool conservarRebaseFase =
        (ratioFastAceptado || ratioSlowAplicado)
        && !medidaRateBajaConfirmada
        && !fastRateIdempotente
        && !fastTransicionConFaseRate
        && AdaptHint > 0.0
        && AdaptRatio > ratioAntesEstimadores
        && (rateBajoAntesEstimadores || rebaseFaseAntesEstimadores);
    const bool rebaseFaseOrigenRate = conservarRebaseFase
        && (rateBajoAntesEstimadores
            || (rebaseFaseAntesEstimadores
                && rebaseFaseOrigenRateAntes));
    const bool faseFueraBanda = nivelPostConsumo < nivelBajo
                             || nivelPostConsumo > nivelAlto
                             || nivelPostEscritura > nivelAlto;
    const bool rebaseHintDescendenteFisicamenteSeguro =
        AdaptPhaseRebasePendiente
        && !AdaptPhaseOrigenRecuperacionRate
        && !AdaptPhaseEpisodioActivo
        && AdaptHint > 0.0
        && std::isfinite(AdaptHint)
        && AdaptRatio == AdaptHint
        && !AdaptProbe
        && nivelPostEscritura < nivelSeguro
        && dropsEstaEscritura == 0;

    if (AdaptSostenidoPhaseProvisionalActivo)
    {

        AdaptPhaseBalanceFrames = 0.0;
        AdaptPhaseEpisodioActivo = false;
        AdaptPhaseObjetivoFrames = 0;
        AdaptPhaseObjetivoActivo = false;
        AdaptPhaseRebasePendiente = false;
        AdaptPhaseRebaseBandViolada = false;
        AdaptPhaseRebaseMinFrames = -1;
        AdaptPhaseOrigenRecuperacionRate = false;
    }
    else if (transaccionRateSostenida)
    {

    }
    else if (fastTransicionConFaseRate)
    {

        int fronteraFaseRate = nivelPostConsumo;
        if (fastCambioTargetConfirmadoConFaseRaiz)
        {
            fronteraFaseRate = std::max(
                cambioTargetFronteraPhaseFrames, nivelPostConsumo);
        }
        else if (fastLowConfirmadoSinRespaldoFuturo
                 && AdaptPhaseObjetivoActivo)
        {
            fronteraFaseRate = std::max(
                AdaptPhaseObjetivoFrames, nivelPostConsumo);
        }
        AdaptPhaseBalanceFrames = 0.0;
        AdaptPhaseEpisodioActivo = false;
        AdaptPhaseObjetivoFrames = std::min(
            nivelSeguro, std::max(0, fronteraFaseRate));
        AdaptPhaseObjetivoActivo = true;
        AdaptPhaseRebasePendiente = false;
        AdaptPhaseRebaseBandViolada = false;
        AdaptPhaseRebaseMinFrames = -1;
        AdaptPhaseOrigenRecuperacionRate =
            !medidaRateBajaConfirmada
            && AdaptRatio > ratioAntesEstimadores
            && faseRatePoseidaAntes;
    }
    else if (lineageRateRebasado)
    {
        AdaptPhaseBalanceFrames = 0.0;
        AdaptPhaseEpisodioActivo = false;
        AdaptPhaseObjetivoFrames = 0;
        AdaptPhaseObjetivoActivo = false;
        AdaptPhaseRebasePendiente = conservarRebaseFase;
        AdaptPhaseRebaseBandViolada = conservarRebaseFase
            && ((rebaseFaseAntesEstimadores
                 && AdaptPhaseRebaseBandViolada)
                || faseFueraBanda);
        AdaptPhaseOrigenRecuperacionRate = rebaseFaseOrigenRate;
        if (conservarRebaseFase)
        {
            if (!rebaseFaseAntesEstimadores
                || AdaptPhaseRebaseMinFrames < 0)
            {
                AdaptPhaseRebaseMinFrames = nivelPostConsumo;
            }
            else if (nivelPostConsumo < AdaptPhaseRebaseMinFrames)
            {
                AdaptPhaseRebaseMinFrames = nivelPostConsumo;
            }
        }
        else
        {
            AdaptPhaseRebaseMinFrames = -1;
        }
    }
    else if (AdaptRateBajoConfirmado)
    {

        AdaptPhaseBalanceFrames = 0.0;
        AdaptPhaseEpisodioActivo = false;
    }
    else
    {
        if (AdaptPhaseRebasePendiente)
        {
            AdaptPhaseRebaseBandViolada |= faseFueraBanda;
            if (AdaptPhaseRebaseMinFrames < 0
                || nivelPostConsumo < AdaptPhaseRebaseMinFrames)
            {
                AdaptPhaseRebaseMinFrames = nivelPostConsumo;
            }
        }

        const bool ventanaRebaseSegura =
            AdaptPhaseOrigenRecuperacionRate
            ? !AdaptPhaseRebaseBandViolada
            : (rebaseHintDescendenteFisicamenteSeguro
               || !faseFueraBanda);
        if (AdaptPhaseRebasePendiente
            && ventanaRebaseSegura
            && AdaptHint > 0.0
            && AdaptRatio == AdaptHint
            && !AdaptProbe
            && ventanaFastPublicable
            && !demandaSuperaProduccionEntera
            && !AdaptFastLowPendiente
            && !AdaptFastCambioTargetPendiente
            && !ratioFastAceptado
            && !ratioSlowAplicado
            && dropsEstaEscritura == 0)
        {

            AdaptPhaseObjetivoFrames =
                AdaptPhaseOrigenRecuperacionRate
                ? AdaptPhaseRebaseMinFrames : nivelPostConsumo;
            AdaptPhaseObjetivoActivo = true;
            AdaptPhaseRebasePendiente = false;
            AdaptPhaseRebaseBandViolada = false;
            AdaptPhaseRebaseMinFrames = -1;
            AdaptPhaseBalanceFrames = 0.0;
            AdaptPhaseEpisodioActivo = false;
        }
        else if (AdaptPhaseRebasePendiente && ventanaFastPublicable)
        {

            AdaptPhaseRebaseBandViolada = faseFueraBanda;
            AdaptPhaseRebaseMinFrames = nivelPostConsumo;
        }
    }

    if (transaccionRateSostenida)
    {

    }
    else if (fastTransicionConFaseRate)
    {
        const bool hijoConPadreRollback = AdaptRateRollbackPendiente
            && AdaptRatio < AdaptRateRollbackRatio;
        iniciarActuadorRate(
            deltaFastCons, deltaFastTicks, hijoConPadreRollback,
            fastCambioTargetEscrowWriteFrames);
        aplicarInmediato = !AdaptRateActuadorPendiente;
    }
    else if (fastRateIdempotente && AdaptRateActuadorPendiente)
    {

        AdaptRateActuadorCons = deltaFastCons;
        AdaptRateActuadorTicks = deltaFastTicks;
    }
    else if (ratioFastAceptado && !fastRateIdempotente)
    {

        AdaptRateActuadorPendiente = false;
        AdaptRateActuadorPreferirRapido = false;
        AdaptRateActuadorCons = 0;
        AdaptRateActuadorTicks = 0;
    }
    else if (ratioSlowAplicado
             && AdaptRateActuadorPendiente
             && AdaptRatio != ratioAntesEstimadores)
    {

        const bool hijoConPadreRollback = AdaptRateRollbackPendiente
            && AdaptRatio < AdaptRateRollbackRatio;
        iniciarActuadorRate(
            deltaSlowCons, deltaSlowTicks, hijoConPadreRollback);
    }

    const int objetivoBase = AdaptPhaseObjetivoActivo
        ? AdaptPhaseObjetivoFrames : objetivo;
    if (!lineageRateRebasado
        && !AdaptRateBajoConfirmado
        && !AdaptSostenidoPhaseProvisionalActivo
        && dropsEstaEscritura == 0
        && std::isfinite(creditoActuadorNuevo))
    {
        if (!AdaptPhaseEpisodioActivo)
        {
            if (intervaloHostPerdido)
            {
                AdaptPhaseBalanceFrames = std::max(
                    -(double)bloque, balanceFaseNuevo);
                AdaptPhaseEpisodioActivo = true;
            }
        }
        else
        {
            AdaptPhaseBalanceFrames += balanceFaseNuevo;
            if (AdaptPhaseBalanceFrames > (double)bloque)
                AdaptPhaseBalanceFrames = (double)bloque;
            else if (AdaptPhaseBalanceFrames < -(double)bloque)
                AdaptPhaseBalanceFrames = -(double)bloque;

            if ((AdaptPhaseBalanceFrames > 0.0
                 && nivelPostConsumo <= objetivoBase)
                || (AdaptPhaseBalanceFrames < 0.0
                    && nivelPostConsumo >= objetivoBase)
                || std::abs(AdaptPhaseBalanceFrames) < 1.0)
            {
                AdaptPhaseBalanceFrames = 0.0;
                AdaptPhaseEpisodioActivo = false;
            }
        }
    }

    const double phaseOffset = AdaptPhaseEpisodioActivo
        ? std::min(0.0, AdaptPhaseBalanceFrames) : 0.0;
    const double objetivoFase = std::max(
        (double)nivelBajo, (double)objetivoBase + phaseOffset);
    double error = ((double)nivelPostConsumo - objetivoFase)
                 / (double)objetivoControl;
    if (error > 1.0) error = 1.0;
    else if (error < -1.0) error = -1.0;
    const bool phasePoseidaPorRate = AdaptRateBajoConfirmado
        || AdaptSostenidoOwnerActivo
        || (AdaptPhaseObjetivoActivo
            && AdaptPhaseOrigenRecuperacionRate);

    const bool phaseConservadaPorRebaseHint =
        rebaseHintDescendenteFisicamenteSeguro
        && AdaptPhaseRebasePendiente;
    const double objetivoPhaseOrdinario = limitar(
        AdaptRatio * (1.0 + 0.02 * error));
    double objetivoSkew = (phasePoseidaPorRate
                           || phaseConservadaPorRebaseHint)
        ? AdaptRatio
        : objetivoPhaseOrdinario;

    if (AdaptFastCambioTargetPhaseEscrowActivo)
    {

        const double candidatoEscrow =
            AdaptFastCambioTargetPhaseEscrowGuardSkew;
        if (AdaptFastCambioTargetPhaseEscrowOverlaySkew > 0.0)
        {
            AdaptFastCambioTargetPhaseEscrowOverlaySkew = std::min(
                AdaptFastCambioTargetPhaseEscrowOverlaySkew,
                candidatoEscrow);
        }
        else
        {
            AdaptFastCambioTargetPhaseEscrowOverlaySkew = candidatoEscrow;
        }
        if (AdaptFastCambioTargetPhaseEscrowOverlaySkew < AdaptSkew)
        {
            AdaptSkew = AdaptFastCambioTargetPhaseEscrowOverlaySkew;
            decisionFlags |=
                AudioOutputAdaptiveGuardApplied
                | AudioOutputAdaptiveSlewBranch
                | AudioOutputAdaptiveSlewDown;
        }
        objetivoSkew = AdaptSkew;
        aplicarInmediato = true;
    }

    if (AdaptProbe && AdaptHint > 0.0)
    {
        int nivelGuardia = nivelPostEscritura;
        if (nivelGuardia < 0) nivelGuardia = 0;
        if (nivelGuardia > nivelSeguro) nivelGuardia = nivelSeguro;
        const double margenBloques =
            (double)(nivelSeguro - nivelGuardia) / (double)bloque;
        const double guardia = limitar(AdaptHint / (1.0 + margenBloques));
        if (guardia > AdaptSkew)
        {
            AdaptSkew = guardia;
            aplicarInmediato = true;
            decisionFlags |= AudioOutputAdaptiveGuardApplied;
        }
    }

    const bool fastHighWitnessPuedeConceder =
        AdaptSostenidoFastHighWitnessActivo
        && AdaptSostenidoFastHighWitnessCapacidad
        && !AdaptProbe
        && !intervaloHostPerdido
        && dropsEstaEscritura == 0
        && AdaptHint > 0.0
        && std::isfinite(AdaptHint);
    const double reservaRateEscrow = medirReservaPublicacion(AdaptRatio);
    const bool reservaRateEscrowValida =
        std::isfinite(reservaRateEscrow)
        && reservaRateEscrow >= 0.0
        && reservaRateEscrow
           <= static_cast<double>(std::numeric_limits<u64>::max());
    const u64 horizonteFisicoEscrow = reservaRateEscrowValida
        ? std::max(
              ventanaRate,
              static_cast<u64>(std::ceil(reservaRateEscrow)))
        : 0;
    if (fastHighWitnessPuedeConceder
        || AdaptSostenidoFastHighEscrowInicializado)
    {
        if (fastHighWitnessPuedeConceder)
        {

        const double skewMasProductivo = std::min(AdaptSkew, AdaptRatio);
        const double reservaPublicacion =
            medirReservaPublicacion(skewMasProductivo);
        const double capacidadUtil = (double)(OutputBufferSize - 1);
        const double nivel = std::min(
            capacidadUtil, (double)std::max(0, nivelPostEscritura));
        const double headroom = capacidadUtil - nivel;
        const double headroomGastable = std::max(
            0.0, headroom - reservaPublicacion);
        u64 horizonteGuardia = ventanaRate;
        if (AdaptSostenidoCandidatoActivo
            && AdaptSostenidoCandidatoReemplazo
            && !AdaptSostenidoSegmentoUnoCompleto
            && AdaptSostenidoDireccion > 0
            && hostConsumed >= AdaptSostenidoConsInicio
            && ticksAlConsumo >= AdaptSostenidoTicksInicio
            && framesAlConsumo >= AdaptSostenidoFramesInicio)
        {
            const u64 deltaL1Cons =
                hostConsumed - AdaptSostenidoConsInicio;
            const u64 deltaL1Frames =
                framesAlConsumo - AdaptSostenidoFramesInicio;
            const bool l1ConservaPCM = deltaL1Cons < ventanaLenta
                && conservaPCM(
                    deltaL1Frames,
                    AdaptSostenidoNivelInicioLogico,
                    deltaL1Cons,
                    nivelPostConsumoLogico);
            if (l1ConservaPCM)
            {

                horizonteGuardia = std::max(
                    ventanaRate, ventanaLenta - deltaL1Cons);
            }
        }
        const double ventanasFinanciables =
            headroomGastable / (double)horizonteGuardia;
        double guardia = limitar(
            AdaptHint / (1.0 + ventanasFinanciables));

        if (AdaptSostenidoCandidatoActivo
            && AdaptSostenidoCandidatoReemplazo
            && AdaptSostenidoSegmentoUnoCompleto
            && hostConsumed >= AdaptSostenidoSegmentoUnoCons
            && ticksAlConsumo >= AdaptSostenidoSegmentoUnoTicks
            && framesAlConsumo >= AdaptSostenidoSegmentoUnoFrames)
        {
            const u64 deltaLedgerCons =
                hostConsumed - AdaptSostenidoSegmentoUnoCons;
            const u64 deltaLedgerTicks =
                ticksAlConsumo - AdaptSostenidoSegmentoUnoTicks;
            const u64 deltaLedgerFrames =
                framesAlConsumo - AdaptSostenidoSegmentoUnoFrames;
            const bool ledgerPublicable =
                deltaLedgerCons >= ventanaRate
                && deltaLedgerCons < ventanaLenta
                && deltaLedgerTicks >= ticksDosFrames;
            const bool ledgerConservaPCM = ledgerPublicable
                && conservaPCM(
                    deltaLedgerFrames,
                    AdaptSostenidoSegmentoUnoNivelLogico,
                    deltaLedgerCons,
                    nivelPostConsumoLogico);
            const bool ledgerSuperaOwner = ledgerConservaPCM
                && compararRelacionHT(
                    deltaLedgerTicks, deltaLedgerCons,
                    AdaptSostenidoOwnerTicks,
                    AdaptSostenidoOwnerCons) > 0;
            if (ledgerSuperaOwner)
            {
                const u64 restanteLedger =
                    ventanaLenta - deltaLedgerCons;
                horizonteGuardia = std::max(
                    ventanaRate, restanteLedger);
                const double ratioLedger = std::min(
                    AdaptHint,
                    medirRatio(deltaLedgerTicks, deltaLedgerCons));
                const double guardiaLedger = limitar(
                    ratioLedger
                    / (1.0 + headroomGastable
                               / (double)restanteLedger));
                guardia = std::max(guardia, guardiaLedger);
            }
        }
        if (AdaptSostenidoFastHighGuardSkew > 0.0)
        {
            AdaptSostenidoFastHighGuardSkew = std::max(
                AdaptSostenidoFastHighGuardSkew, guardia);
        }
        else if (guardia > AdaptSkew)
        {
            AdaptSostenidoFastHighGuardSkew = guardia;
        }

        const u64 suffixFrames =
            nivelPostEscrituraLogico > nivelPostEscrituraFisico
            ? nivelPostEscrituraLogico - nivelPostEscrituraFisico : 0;
        if (AdaptSostenidoFastHighWitnessActivo
            && AdaptSostenidoFastHighWitnessCapacidad
            && !AdaptSostenidoFastHighEscrowInicializado
            && !fastHighEscrowInvalidadoEsteUpdate
            && suffixFrames > 0
            && AdaptSostenidoFastHighWitnessReferenciaTicks > 0
            && AdaptSostenidoFastHighWitnessReferenciaCons > 0
            && AdaptRatio > 0.0
            && std::isfinite(AdaptRatio)
            && AdaptSostenidoFastHighGuardSkew >= 0.0
            && std::isfinite(AdaptSostenidoFastHighGuardSkew)
            && reservaRateEscrowValida
            && horizonteFisicoEscrow > 0
            && medirRatio(
                   AdaptSostenidoFastHighWitnessReferenciaTicks,
                   AdaptSostenidoFastHighWitnessReferenciaCons)
               == AdaptRatio)
        {
            AdaptSostenidoFastHighEscrowInicializado = true;
            AdaptSostenidoFastHighEscrowActivo = true;
            AdaptSostenidoFastHighEscrowReferenciaEsOverride =
                AdaptSostenidoFastHighWitnessReferenciaEsOverride;
            AdaptSostenidoFastHighEscrowOwnerGeneracion =
                AdaptSostenidoOwnerGeneracion;
            AdaptSostenidoFastHighEscrowReferenciaTicks =
                AdaptSostenidoFastHighWitnessReferenciaTicks;
            AdaptSostenidoFastHighEscrowReferenciaCons =
                AdaptSostenidoFastHighWitnessReferenciaCons;
            AdaptSostenidoFastHighEscrowRate = AdaptRatio;
            AdaptSostenidoFastHighEscrowGuardSkew =
                AdaptSostenidoFastHighGuardSkew;
            const u64 horizonteNacimiento = std::max(
                horizonteGuardia, horizonteFisicoEscrow);
            AdaptSostenidoFastHighEscrowHorizonteNacimientoFrames =
                horizonteNacimiento;
            AdaptSostenidoFastHighEscrowHorizonteRestanteFrames =
                horizonteNacimiento;
            AdaptSostenidoFastHighEscrowTicksInicio = AdaptTicksTotal;
            AdaptSostenidoFastHighEscrowFramesPublicadosInicio =
                AdaptFramesProducidosTotal;
            AdaptSostenidoFastHighEscrowConsInicio = hostConsumed;
            AdaptSostenidoFastHighEscrowFramesConsumoInicio = framesAlConsumo;
            AdaptSostenidoFastHighEscrowNivelConsumoInicioLogico =
                nivelPostConsumoLogico;
            AdaptSostenidoFastHighEscrowNivelEscrituraInicioLogico =
                nivelPostEscrituraLogico;
            AdaptSostenidoFastHighEscrowContinuidadEpoch = continuidadEpoch;
            AdaptSostenidoFastHighEscrowResetEpoch = resetSolicitado;
            AdaptSostenidoFastHighEscrowHintSeenEpoch =
                AdaptHintVistoProductor;
            AdaptSostenidoFastHighEscrowGrantFrames = suffixFrames;
            AdaptSostenidoFastHighEscrowSpentFrames = 0;

            ++telemetry.fastHighEscrowBirthCount;
            telemetry.lastFastHighEscrowBirthUpdateId = telemetry.updateId;
            telemetry.lastFastHighEscrowBirthOwnerGeneration =
                AdaptSostenidoFastHighEscrowOwnerGeneracion;
            telemetry.lastFastHighEscrowBirthReferenceTicks =
                AdaptSostenidoFastHighEscrowReferenciaTicks;
            telemetry.lastFastHighEscrowBirthReferenceConsumed =
                AdaptSostenidoFastHighEscrowReferenciaCons;
            telemetry.lastFastHighEscrowBirthReferenceWasOverride =
                AdaptSostenidoFastHighEscrowReferenciaEsOverride;
            telemetry.lastFastHighEscrowBirthRate =
                AdaptSostenidoFastHighEscrowRate;
            telemetry.lastFastHighEscrowBirthGuardSkew =
                AdaptSostenidoFastHighEscrowGuardSkew;
            telemetry.lastFastHighEscrowBirthHorizonFrames =
                AdaptSostenidoFastHighEscrowHorizonteNacimientoFrames;
            telemetry.lastFastHighEscrowBirthStartTicks =
                AdaptSostenidoFastHighEscrowTicksInicio;
            telemetry.lastFastHighEscrowBirthStartPublishedProduced =
                AdaptSostenidoFastHighEscrowFramesPublicadosInicio;
            telemetry.lastFastHighEscrowBirthStartConsumed =
                AdaptSostenidoFastHighEscrowConsInicio;
            telemetry.lastFastHighEscrowBirthStartConsumedProduced =
                AdaptSostenidoFastHighEscrowFramesConsumoInicio;
            telemetry.lastFastHighEscrowBirthStartLevelPostConsumption =
                AdaptSostenidoFastHighEscrowNivelConsumoInicioLogico;
            telemetry.lastFastHighEscrowBirthStartLevelPostWrite =
                AdaptSostenidoFastHighEscrowNivelEscrituraInicioLogico;
            telemetry.lastFastHighEscrowBirthStartContinuityEpoch =
                AdaptSostenidoFastHighEscrowContinuidadEpoch;
            telemetry.lastFastHighEscrowBirthStartResetEpoch =
                AdaptSostenidoFastHighEscrowResetEpoch;
            telemetry.lastFastHighEscrowBirthStartHintSeenEpoch =
                AdaptSostenidoFastHighEscrowHintSeenEpoch;
            telemetry.lastFastHighEscrowBirthGrantFrames = suffixFrames;
        }
        }

        if (AdaptSostenidoFastHighEscrowInicializado)
        {
            const u64 deltaEscrowTicks = AdaptTicksTotal
                - AdaptSostenidoFastHighEscrowTicksInicio;
            const u64 deltaEscrowFrames = AdaptFramesProducidosTotal
                - AdaptSostenidoFastHighEscrowFramesPublicadosInicio;
            const u64 grantEscrow =
                AdaptSostenidoFastHighEscrowGrantFrames;
            u64 spentEscrow = 0;
            if (AudioOutputExactMath::CompareProducts(
                    deltaEscrowTicks,
                    AdaptSostenidoFastHighEscrowReferenciaCons,
                    deltaEscrowFrames,
                    AdaptSostenidoFastHighEscrowReferenciaTicks) > 0)
            {

                u64 low = 1;
                u64 high = grantEscrow;
                if (AudioOutputExactMath::CompareProducts(
                        deltaEscrowTicks,
                        AdaptSostenidoFastHighEscrowReferenciaCons,
                        deltaEscrowFrames + grantEscrow,
                        AdaptSostenidoFastHighEscrowReferenciaTicks) > 0)
                {
                    spentEscrow = grantEscrow;
                }
                else
                {
                    while (low < high)
                    {
                        const u64 middle = low + (high - low) / 2;
                        if (AudioOutputExactMath::CompareProducts(
                                deltaEscrowTicks,
                                AdaptSostenidoFastHighEscrowReferenciaCons,
                                deltaEscrowFrames + middle,
                                AdaptSostenidoFastHighEscrowReferenciaTicks)
                            <= 0)
                        {
                            high = middle;
                        }
                        else
                        {
                            low = middle + 1;
                        }
                    }
                    spentEscrow = low;
                }
            }
            spentEscrow = std::max(
                spentEscrow,
                AdaptSostenidoFastHighEscrowSpentFrames);
            if (spentEscrow
                != AdaptSostenidoFastHighEscrowSpentFrames)
            {
                AdaptSostenidoFastHighEscrowSpentFrames = spentEscrow;
                ++telemetry.fastHighEscrowSpendCount;
                telemetry.lastFastHighEscrowSpendUpdateId =
                    telemetry.updateId;
                telemetry.lastFastHighEscrowSpentFrames = spentEscrow;
                telemetry.lastFastHighEscrowRemainingFrames =
                    grantEscrow - spentEscrow;
            }
            if (spentEscrow >= grantEscrow)
                AdaptSostenidoFastHighEscrowActivo = false;
        }
        if (AdaptSostenidoFastHighEscrowInicializado
            && reservaRateEscrowValida
            && horizonteFisicoEscrow > 0
            && AdaptSostenidoFastHighEscrowHorizonteNacimientoFrames
               >= horizonteFisicoEscrow
            && AdaptSostenidoFastHighEscrowHorizonteRestanteFrames > 0
            && hostConsumed >= AdaptSostenidoFastHighEscrowConsInicio)
        {
            const u64 elapsedEscrow = hostConsumed
                - AdaptSostenidoFastHighEscrowConsInicio;
            const u64 restantePorTiempo =
                elapsedEscrow
                    < AdaptSostenidoFastHighEscrowHorizonteNacimientoFrames
                ? AdaptSostenidoFastHighEscrowHorizonteNacimientoFrames
                    - elapsedEscrow
                : 0;
            const u64 horizonteRestante = std::max(
                horizonteFisicoEscrow, restantePorTiempo);
            AdaptSostenidoFastHighEscrowHorizonteRestanteFrames = std::min(
                AdaptSostenidoFastHighEscrowHorizonteRestanteFrames,
                horizonteRestante);
        }
        const u64 escrowRemaining =
            AdaptSostenidoFastHighEscrowInicializado
            && AdaptSostenidoFastHighEscrowGrantFrames
               >= AdaptSostenidoFastHighEscrowSpentFrames
            ? AdaptSostenidoFastHighEscrowGrantFrames
                - AdaptSostenidoFastHighEscrowSpentFrames
            : 0;
        if (AdaptSostenidoFastHighEscrowActivo
            && escrowRemaining > 0
            && reservaRateEscrowValida
            && horizonteFisicoEscrow > 0
            && AdaptSostenidoFastHighEscrowHorizonteRestanteFrames
               >= horizonteFisicoEscrow
            && !AdaptProbe
            && !intervaloHostPerdido
            && dropsEstaEscritura == 0
            && AdaptHint > 0.0
            && std::isfinite(AdaptHint)
            && AdaptRatio > 0.0
            && std::isfinite(AdaptRatio)
            && AdaptSostenidoFastHighEscrowGuardSkew > AdaptSkew)
        {

            const u64 horizonteEscrow =
                AdaptSostenidoFastHighEscrowHorizonteRestanteFrames;
            double capSuffix = AdaptHint;
            if (escrowRemaining < horizonteEscrow)
            {
                capSuffix = AdaptRatio
                    * (double)horizonteEscrow
                    / (double)(horizonteEscrow - escrowRemaining);

                capSuffix = std::nextafter(capSuffix, AdaptRatio);
            }
            const double guardiaFinita = limitar(std::min(
                AdaptHint,
                std::min(
                    AdaptSostenidoFastHighEscrowGuardSkew, capSuffix)));
            if (guardiaFinita > AdaptSkew)
            {
                fastHighOverlaySkew = guardiaFinita;
                ++telemetry.fastHighEscrowOverlayCount;
                telemetry.lastFastHighEscrowOverlayUpdateId =
                    telemetry.updateId;
                telemetry.lastFastHighEscrowOverlayRemainingFrames =
                    escrowRemaining;
                telemetry.lastFastHighEscrowOverlaySkew = guardiaFinita;
            }
        }
    }

    if (AdaptSostenidoPhaseProvisionalActivo && AdaptRateActuadorPendiente)
    {
        ++telemetry.sustainedProvisionalPhaseApplicationConflictCount;
        telemetry.lastSustainedProvisionalPhaseApplicationConflictUpdateId =
            telemetry.updateId;
    }

    if (AdaptSostenidoPhaseProvisionalActivo)
    {
        const double skewAnterior = AdaptSkew;
        AdaptSkew = AdaptSostenidoPhaseProvisionalSkew;
        objetivoSkew = AdaptSkew;
        aplicarInmediato = true;
        decisionFlags |=
            AudioOutputAdaptiveGuardApplied
            | AudioOutputAdaptiveSlewBranch;
        if (AdaptSkew > skewAnterior)
            decisionFlags |= AudioOutputAdaptiveSlewUp;
        else if (AdaptSkew < skewAnterior)
            decisionFlags |= AudioOutputAdaptiveSlewDown;
    }

    if (AdaptRateActuadorPendiente)
    {
        aplicarActuadorRate();
        if (actuadorRateRebasado)
        {

            AdaptActCreditoFrames = 0.0;
            AdaptRateCreditoFrames = 0.0;
        }
    }
    else if (ratioFastAceptado && !fastRateIdempotente)
    {
        AdaptSkew = objetivoSkew;
    }
    else if (!aplicarInmediato && deltaRateControl > 0.0)
    {
        decisionFlags |= AudioOutputAdaptiveSlewBranch;

        const double subir = std::pow(
            1.001, deltaRateControl / 512.0);
        const double bajar = std::pow(
            0.999, deltaRateControl / 512.0);
        if (objetivoSkew > AdaptSkew * subir)
        {
            AdaptSkew *= subir;
            decisionFlags |= AudioOutputAdaptiveSlewUp;
        }
        else if (objetivoSkew < AdaptSkew * bajar)
        {
            AdaptSkew *= bajar;
            decisionFlags |= AudioOutputAdaptiveSlewDown;
        }
        else
        {
            AdaptSkew = objetivoSkew;
            decisionFlags |= AudioOutputAdaptiveSlewSnap;
        }
    }

    if (AdaptHint > 0.0
        && AdaptRatio == AdaptHint
        && !AdaptProbe
        && !AdaptRateActuadorPendiente
        && !AdaptFastLowPendiente
        && !AdaptFastCambioTargetPendiente
        && nivelPostConsumo >= nivelBajo
        && nivelPostEscritura <= nivelAlto)
    {

        AdaptActCreditoFrames = 0.0;
        AdaptRateCreditoFrames = 0.0;
    }
    if (AdaptParentCapacityActivo
        && !parentCapacityNacidaEsteUpdate
        && (AdaptRatio != AdaptParentCapacityParentRatio
            || transaccionSostenida != TransaccionSostenida::Ninguna))
    {
        limpiarParentCapacity(
            AudioOutputAdaptiveParentCapacityClearRateChange);
    }
    publicarSalida();

    if ((decisionFlags & AudioOutputAdaptiveFastAccepted) != 0)
    {
        telemetry.lastFastAcceptedUpdateId = telemetry.updateId;
        telemetry.lastFastAcceptedRawRatio = telemetry.estimatorRawRatio;
        telemetry.lastFastAcceptedEffectiveRatio = AdaptRatio;
        telemetry.lastFastAcceptedLevelPostConsumption =
            telemetry.levelPostConsumption;
        telemetry.lastFastAcceptedLevelBeforeWrite =
            telemetry.levelBeforeWrite;
        telemetry.lastFastAcceptedLevelPostWrite = telemetry.levelPostWrite;
        telemetry.lastFastAcceptedFlags = decisionFlags;
        ++telemetry.fastAcceptedCount;
    }

    if ((decisionFlags & AudioOutputAdaptiveSlowApplied) != 0)
    {
        telemetry.lastSlowAppliedUpdateId = telemetry.updateId;
        telemetry.lastSlowAppliedRawRatio = telemetry.estimatorRawRatio;
        telemetry.lastSlowAppliedEffectiveRatio = AdaptRatio;
        ++telemetry.slowAppliedCount;
    }
    publicarEventoProvisional();
}

void SPU::SetOutputSpeedHint(double speed)
{
    if (!std::isfinite(speed) || speed <= 0.0)
        speed = 0.0;

    AdaptSpeedHint.store(speed, std::memory_order_relaxed);
    AdaptHintEpoch.fetch_add(1, std::memory_order_release);
}

void SPU::ResetOutputAdaptivo()
{
    AdaptReingresoPrimingReserva.store(0, std::memory_order_release);
    AdaptResetEpoch.fetch_add(1, std::memory_order_release);
}

void SPU::SetOutputSampleRate(double rate)
{
    OutputSampleRate = rate;
    InitOutput();
    ResetOutputAdaptivo();
}

void SPU::SetOutputSkew(double skew)
{
    blip_set_rates(BlipLeft, INTERNAL_SAMPLE_RATE * skew, OutputSampleRate);
    blip_set_rates(BlipRight, INTERNAL_SAMPLE_RATE * skew, OutputSampleRate);
    OutputSkew = skew;
    OutputSkewPublicado.store(skew, std::memory_order_relaxed);
}


u8 SPU::Read8(u32 addr)
{
    if (addr < 0x04000500)
    {
        SPUChannel* chan = &Channels[(addr >> 4) & 0xF];

        switch (addr & 0xF)
        {
        case 0x0: return chan->Cnt & 0xFF;
        case 0x1: return (chan->Cnt >> 8) & 0xFF;
        case 0x2: return (chan->Cnt >> 16) & 0xFF;
        case 0x3: return chan->Cnt >> 24;
        }
    }
    else
    {
        switch (addr)
        {
        case 0x04000500: return Cnt & 0x7F;
        case 0x04000501: return Cnt >> 8;

        case 0x04000508: return Capture[0].Cnt;
        case 0x04000509: return Capture[1].Cnt;
        }
    }

    Log(LogLevel::Warn, "unknown SPU read8 %08X\n", addr);
    return 0;
}

u16 SPU::Read16(u32 addr)
{
    if (addr < 0x04000500)
    {
        SPUChannel* chan = &Channels[(addr >> 4) & 0xF];

        switch (addr & 0xF)
        {
        case 0x0: return chan->Cnt & 0xFFFF;
        case 0x2: return chan->Cnt >> 16;
        }
    }
    else
    {
        switch (addr)
        {
        case 0x04000500: return Cnt;
        case 0x04000504: return Bias;

        case 0x04000508: return Capture[0].Cnt | (Capture[1].Cnt << 8);
        }
    }

    Log(LogLevel::Warn, "unknown SPU read16 %08X\n", addr);
    return 0;
}

u32 SPU::Read32(u32 addr)
{
    if (addr < 0x04000500)
    {
        SPUChannel* chan = &Channels[(addr >> 4) & 0xF];

        switch (addr & 0xF)
        {
        case 0x0: return chan->Cnt;
        }
    }
    else
    {
        switch (addr)
        {
        case 0x04000500: return Cnt;
        case 0x04000504: return Bias;

        case 0x04000508: return Capture[0].Cnt | (Capture[1].Cnt << 8);

        case 0x04000510: return Capture[0].DstAddr;
        case 0x04000518: return Capture[1].DstAddr;
        }
    }

    Log(LogLevel::Warn, "unknown SPU read32 %08X\n", addr);
    return 0;
}

void SPU::Write8(u32 addr, u8 val)
{
    if (addr < 0x04000500)
    {
        SPUChannel* chan = &Channels[(addr >> 4) & 0xF];

        switch (addr & 0xF)
        {
        case 0x0: chan->SetCnt((chan->Cnt & 0xFFFFFF00) | val); return;
        case 0x1: chan->SetCnt((chan->Cnt & 0xFFFF00FF) | (val << 8)); return;
        case 0x2: chan->SetCnt((chan->Cnt & 0xFF00FFFF) | (val << 16)); return;
        case 0x3: chan->SetCnt((chan->Cnt & 0x00FFFFFF) | (val << 24)); return;
        }
    }
    else
    {
        switch (addr)
        {
        case 0x04000500:
            Cnt = (Cnt & 0xBF00) | (val & 0x7F);
            MasterVolume = Cnt & 0x7F;
            if (MasterVolume == 127) MasterVolume++;
            return;
        case 0x04000501:
            Cnt = (Cnt & 0x007F) | ((val & 0xBF) << 8);
            return;

        case 0x04000508:
            Capture[0].SetCnt(val);
            return;
        case 0x04000509:
            Capture[1].SetCnt(val);
            return;
        }
    }

    Log(LogLevel::Warn, "unknown SPU write8 %08X %02X\n", addr, val);
}

void SPU::Write16(u32 addr, u16 val)
{
    if (addr < 0x04000500)
    {
        SPUChannel* chan = &Channels[(addr >> 4) & 0xF];

        switch (addr & 0xF)
        {
        case 0x0: chan->SetCnt((chan->Cnt & 0xFFFF0000) | val); return;
        case 0x2: chan->SetCnt((chan->Cnt & 0x0000FFFF) | (val << 16)); return;
        case 0x8:
            chan->SetTimerReload(val);
            if      ((addr & 0xF0) == 0x10) Capture[0].SetTimerReload(val);
            else if ((addr & 0xF0) == 0x30) Capture[1].SetTimerReload(val);
            return;
        case 0xA: chan->SetLoopPos(val); return;

        case 0xC: chan->SetLength(((chan->Length >> 2) & 0xFFFF0000) | val); return;
        case 0xE: chan->SetLength(((chan->Length >> 2) & 0x0000FFFF) | (val << 16)); return;
        }
    }
    else
    {
        switch (addr)
        {
        case 0x04000500:
            Cnt = val & 0xBF7F;
            MasterVolume = Cnt & 0x7F;
            if (MasterVolume == 127) MasterVolume++;
            return;

        case 0x04000504:
            Bias = val & 0x3FF;
            return;

        case 0x04000508:
            Capture[0].SetCnt(val & 0xFF);
            Capture[1].SetCnt(val >> 8);
            return;

        case 0x04000514: Capture[0].SetLength(val); return;
        case 0x0400051C: Capture[1].SetLength(val); return;
        }
    }

    Log(LogLevel::Warn, "unknown SPU write16 %08X %04X\n", addr, val);
}

void SPU::Write32(u32 addr, u32 val)
{
    if (addr < 0x04000500)
    {
        SPUChannel* chan = &Channels[(addr >> 4) & 0xF];

        switch (addr & 0xF)
        {
        case 0x0: chan->SetCnt(val); return;
        case 0x4: chan->SetSrcAddr(val); return;
        case 0x8:
            chan->SetLoopPos(val >> 16);
            val &= 0xFFFF;
            chan->SetTimerReload(val);
            if      ((addr & 0xF0) == 0x10) Capture[0].SetTimerReload(val);
            else if ((addr & 0xF0) == 0x30) Capture[1].SetTimerReload(val);
            return;
        case 0xC: chan->SetLength(val); return;
        }
    }
    else
    {
        switch (addr)
        {
        case 0x04000500:
            Cnt = val & 0xBF7F;
            MasterVolume = Cnt & 0x7F;
            if (MasterVolume == 127) MasterVolume++;
            return;

        case 0x04000504:
            Bias = val & 0x3FF;
            return;

        case 0x04000508:
            Capture[0].SetCnt(val & 0xFF);
            Capture[1].SetCnt(val >> 8);
            return;

        case 0x04000510: Capture[0].SetDstAddr(val); return;
        case 0x04000514: Capture[0].SetLength(val & 0xFFFF); return;
        case 0x04000518: Capture[1].SetDstAddr(val); return;
        case 0x0400051C: Capture[1].SetLength(val & 0xFFFF); return;
        }
    }
}

}
