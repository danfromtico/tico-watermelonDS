#pragma once

#include <switch.h>
#include <switch/nro.h>
#include <string>

extern "C" void _start();

namespace SwitchFrontend
{
inline std::string BuildId()
{
    const auto* header = reinterpret_cast<const NroHeader*>(
        reinterpret_cast<const u8*>(&_start) + sizeof(NroStart));
    if (header->magic != NROHEADER_MAGIC)
        return "unavailable";
    constexpr char hex[] = "0123456789abcdef";
    std::string result;
    for (u8 byte : header->build_id)
    {
        result += hex[byte >> 4];
        result += hex[byte & 15];
    }
    return result;
}
}
