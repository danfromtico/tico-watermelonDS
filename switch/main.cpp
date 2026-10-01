// WatermelonDS Nintendo Switch test entrypoint.
//
// No launcher and no UI: boots one cartridge and logs frame timing. The tico
// build lives in tico/main.cpp; both share the runtime in SwitchEmulator.
//
// ROM: argv[1] when launched with one, otherwise the first .nds found in
// sdmc:/switch/watermelonds/. Hold Plus + Minus to quit.

#include <switch.h>

#include <dirent.h>
#include <sys/stat.h>

#include <array>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <atomic>
#include <mutex>

#include "ARMJIT.h"
#include "SwitchEmulator.h"
#include "SwitchFrontend.h"
#include "BuildIdentity.h"

extern "C" void _start();

namespace
{
const std::string kBaseDir = "sdmc:/switch/watermelonds/";

[[noreturn]] void Fatal(const char* message)
{
    SwitchFrontend::Log("fatal: %s\n", message);
    SwitchFrontend::CloseLog();

    consoleInit(nullptr);
    printf("WatermelonDS\n\n%s\n\nPress + to exit.\n", message);
    consoleUpdate(nullptr);

    PadState pad;
    padInitializeDefault(&pad);
    while (appletMainLoop())
    {
        padUpdate(&pad);
        if (padGetButtonsDown(&pad) & HidNpadButton_Plus)
            break;
        consoleUpdate(nullptr);
    }

    consoleExit(nullptr);
    exit(1);
}

bool HasNdsExtension(const std::string& name)
{
    if (name.size() < 4)
        return false;
    return strcasecmp(name.c_str() + name.size() - 4, ".nds") == 0;
}

std::string FindRom(int argc, char** argv)
{
    if (argc > 1 && argv[1] && argv[1][0])
        return argv[1];

    DIR* dir = opendir(kBaseDir.c_str());
    if (!dir)
        return {};

    std::string found;
    while (dirent* entry = readdir(dir))
    {
        std::string name = entry->d_name;
        if (!HasNdsExtension(name))
            continue;
        // pick the alphabetically first one so the choice is stable
        if (found.empty() || name < found)
            found = name;
    }
    closedir(dir);

    return found.empty() ? found : kBaseDir + found;
}

// Sampling profiler for the emulation thread: every 5 ms a thread on
// another core records where the main thread is. Samples are bucketed by NRO
// offset and written to profile.txt so they can be resolved against the ELF.
constexpr u32 kProfileBucketShift = 4;
constexpr u32 kProfileBuckets = 0x2000000 >> kProfileBucketShift;
constexpr u64 kProfileIntervalNs = 5000000;
constexpr size_t kProfileStackDepth = 16;
constexpr size_t kProfileMaxStacks = 16384;

// Walks the frame-pointer chain of a paused thread: each frame record holds
// the caller's frame pointer and the return address. Only addresses inside
// the thread's own stack are followed.
void CollectStack(const ThreadContext& ctx, u64 base, std::array<u64, kProfileStackDepth>& out)
{
    out.fill(0);
    size_t n = 0;
    auto push = [&](u64 address) {
        if (n < out.size())
            out[n++] = address >= base ? address - base : 0xFFFFFFFFull;
    };
    push(ctx.pc.x);
    push(ctx.lr);

    const u64 stackLow = ctx.sp;
    const u64 stackHigh = ctx.sp + 0x100000;
    u64 fp = ctx.fp;
    while (n < out.size() && fp >= stackLow && fp + 16 <= stackHigh && (fp & 0xF) == 0)
    {
        const u64* record = reinterpret_cast<const u64*>(fp);
        const u64 next = record[0];
        const u64 ret = record[1];
        if (ret == 0)
            break;
        push(ret);
        if (next <= fp)
            break;
        fp = next;
    }
}

struct Profiler
{
    ::Thread Worker {};
    Handle Target = INVALID_HANDLE;
    std::atomic<bool> Running { false };
    std::mutex DataMutex;
    u32* Buckets = nullptr;
    u32 Outside = 0;
    u32 Total = 0;
    u32 Failed = 0;
    // call stacks, newest frame first, as NRO offsets (0 ends a stack)
    std::vector<std::array<u64, kProfileStackDepth>> Stacks;
};

Profiler Prof;

void ProfilerThread(void*)
{
    u64 base = (u64)&_start;
    while (Prof.Running)
    {
        svcSleepThread(kProfileIntervalNs);
        if (!Prof.Running)
            break;

        ThreadContext ctx;
        std::array<u64, kProfileStackDepth> stack;
        if (R_FAILED(svcSetThreadActivity(Prof.Target, ThreadActivity_Paused)))
        {
            std::lock_guard lock(Prof.DataMutex);
            Prof.Failed++;
            continue;
        }
        Result rc = svcGetThreadContext3(&ctx, Prof.Target);
        // the stack is only stable while the thread is paused
        if (R_SUCCEEDED(rc))
            CollectStack(ctx, base, stack);
        svcSetThreadActivity(Prof.Target, ThreadActivity_Runnable);
        // Never acquire the data mutex while the target thread is paused.
        std::lock_guard lock(Prof.DataMutex);
        if (R_FAILED(rc))
        {
            Prof.Failed++;
            continue;
        }

        Prof.Total++;
        if (Prof.Stacks.size() < kProfileMaxStacks)
            Prof.Stacks.push_back(stack);
        u64 offset = ctx.pc.x - base;
        if (ctx.pc.x >= base && (offset >> kProfileBucketShift) < kProfileBuckets)
            Prof.Buckets[offset >> kProfileBucketShift]++;
        else
            Prof.Outside++;
    }
}

void WriteProfile();

void StartProfiler()
{
    Prof.Buckets = (u32*)calloc(kProfileBuckets, sizeof(u32));
    Prof.Stacks.reserve(kProfileMaxStacks);
    if (!Prof.Buckets)
    {
        SwitchFrontend::Log("profiler allocation failed\n");
        return;
    }
    Prof.Target = threadGetCurHandle();
    Prof.Running = true;
    int core;
    u32 mask;
    SwitchFrontend::Cores::NextBackgroundCore(core, mask);
    const Result created = threadCreate(&Prof.Worker, ProfilerThread, nullptr, nullptr, 0x10000, 0x2C, core);
    if (R_SUCCEEDED(created))
        svcSetThreadCoreMask(Prof.Worker.handle, core, mask);
    if (R_FAILED(created) || R_FAILED(threadStart(&Prof.Worker)))
    {
        if (R_SUCCEEDED(created))
            threadClose(&Prof.Worker);
        Prof.Running = false;
        SwitchFrontend::Log("profiler thread could not be started\n");
    }
    // Replace an older run's profile immediately, even if this run is killed.
    WriteProfile();
}

void WriteProfile()
{
    if (!Prof.Buckets)
        return;

    std::lock_guard lock(Prof.DataMutex);
    FILE* f = fopen((kBaseDir + "profile.txt").c_str(), "w");
    if (!f)
        return;
    fprintf(f, "total %u outside %u failed %u\n", Prof.Total, Prof.Outside, Prof.Failed);
    fprintf(f, "# build_id %s interval_ns %llu\n", SwitchFrontend::BuildId().c_str(),
        (unsigned long long)kProfileIntervalNs);
    for (u32 i = 0; i < kProfileBuckets; i++)
    {
        if (Prof.Buckets[i])
            fprintf(f, "%x %u\n", i << kProfileBucketShift, Prof.Buckets[i]);
    }
    fclose(f);

    // one sample per line: hex NRO offsets, innermost first
    FILE* stacks = fopen((kBaseDir + "stacks.txt").c_str(), "w");
    if (!stacks)
        return;
    fprintf(stacks, "# build_id %s\n", SwitchFrontend::BuildId().c_str());
    for (const auto& stack : Prof.Stacks)
    {
        for (u64 address : stack)
        {
            if (address == 0)
                break;
            fprintf(stacks, "%llx ", (unsigned long long)address);
        }
        fputc('\n', stacks);
    }
    fclose(stacks);
}

void StopProfiler()
{
    if (Prof.Running)
    {
        Prof.Running = false;
        threadWaitForExit(&Prof.Worker);
        threadClose(&Prof.Worker);
    }
    WriteProfile();
}
}

int main(int argc, char** argv)
{
    mkdir("sdmc:/switch", 0777);
    mkdir("sdmc:/switch/watermelonds", 0777);
    SwitchFrontend::OpenLog(kBaseDir + "log.txt");
    SwitchFrontend::Log("WatermelonDS Switch test entrypoint (Vulkan)\n");

    std::string romPath = FindRom(argc, argv);
    if (romPath.empty())
        Fatal("No ROM found.\nPut a .nds file in sdmc:/switch/watermelonds/");
    SwitchFrontend::Log("rom: %s\n", romPath.c_str());

    SwitchEmulator::BootOptions options;
    options.RomPath = romPath;
    options.SavePath = romPath.substr(0, romPath.find_last_of('.')) + ".sav";
    options.DataDir = kBaseDir;
    // create threaded2d.on to try drawing 2D on a worker core
    struct stat threadedFlag;
    options.Config.Threaded2D = stat((kBaseDir + "threaded2d.on").c_str(), &threadedFlag) == 0;

    std::string error;
    if (!SwitchEmulator::Boot(options, error))
        Fatal(error.c_str());

    padConfigureInput(1, HidNpadStyleSet_NpadStandard);
    PadState pad;
    padInitializeDefault(&pad);

    // Opt-in sampling covers frames 1200-1800: the first frames are loading
    // and JIT warm-up, which is not what a game costs once it is running.
    // profile.on may hold the first frame to sample (default 600); 600 frames are sampled
    struct stat profileFlag;
    const bool profiling = stat((kBaseDir + "profile.on").c_str(), &profileFlag) == 0;
    u64 kProfileStartFrame = 600;
    if (profiling)
    {
        if (FILE* flag = fopen((kBaseDir + "profile.on").c_str(), "r"))
        {
            unsigned long long start = 0;
            if (fscanf(flag, "%llu", &start) == 1)
                kProfileStartFrame = start;
            fclose(flag);
        }
    }
    const u64 kProfileEndFrame = kProfileStartFrame + 600;
    bool profileTaken = false;
    SwitchFrontend::Log("profiler %s\n", profiling
        ? "armed (5 ms samples over 600 frames; start frame in profile.on, default 600)"
        : "off (create profile.on to enable)");

    SwitchEmulator::Stats last = SwitchEmulator::GetStats();
    u64 lastReport = armGetSystemTick();
    while (appletMainLoop() && !SwitchEmulator::StopRequested())
    {
        padUpdate(&pad);
        u64 held = padGetButtons(&pad);
        if ((held & HidNpadButton_Plus) && (held & HidNpadButton_Minus))
            break;

        SwitchEmulator::UpdateInput(held);
        SwitchEmulator::RunFrame();

        SwitchEmulator::Stats stats = SwitchEmulator::GetStats();
        if (stats.Frames - last.Frames >= 300)
        {
            u64 now = armGetSystemTick();
            double seconds = armTicksToNs(now - lastReport) / 1e9;
            const u64 elapsedTicks = now - lastReport;
            double busy[3];
            for (int core = 0; core < 3; core++)
                busy[core] = 100.0 * (1.0 - (double)(stats.IdleTicks[core] - last.IdleTicks[core]) / elapsedTicks);
            SwitchFrontend::Log("%.1f fps emulated, %.1f fps presented (runFrame %.2f ms) cores busy %.0f%% %.0f%% %.0f%%\n",
                300.0 / seconds, (stats.PresentedFrames - last.PresentedFrames) / seconds,
                armTicksToNs(stats.RunFrameTicks - last.RunFrameTicks) / 300e6, busy[0], busy[1], busy[2]);
            {
                static u64 lastCompiled = 0, lastResets = 0, lastInvalidations = 0;
                const u64 compiled = melonDS::JitStatCompiledBlocks.load();
                const u64 resets = melonDS::JitStatCacheResets.load();
                const u64 invalidations = melonDS::JitStatInvalidations.load();
                SwitchFrontend::Log("JIT: %llu blocks compiled, %llu cache resets, %llu invalidations in the last 300 frames\n",
                    (unsigned long long)(compiled - lastCompiled), (unsigned long long)(resets - lastResets),
                    (unsigned long long)(invalidations - lastInvalidations));
                lastCompiled = compiled;
                lastResets = resets;
                lastInvalidations = invalidations;
            }
            if (Prof.Running && stats.Frames >= kProfileEndFrame)
            {
                StopProfiler();
                profileTaken = true;
                SwitchFrontend::Log("profile saved (frames %llu-%llu)\n",
                    (unsigned long long)kProfileStartFrame, (unsigned long long)stats.Frames);
            }
            else if (Prof.Running)
            {
                // closing from HOME skips the clean exit, so keep what was sampled so far
                WriteProfile();
            }
            else if (profiling && !profileTaken && !Prof.Running && stats.Frames >= kProfileStartFrame)
            {
                StartProfiler();
                SwitchFrontend::Log("profiler started at frame %llu\n", (unsigned long long)stats.Frames);
            }
            lastReport = now;
            last = stats;
        }
    }

    SwitchFrontend::Log("exiting after %llu frames\n", (unsigned long long)SwitchEmulator::GetStats().Frames);

    StopProfiler();
    SwitchEmulator::Shutdown();
    SwitchFrontend::CloseLog();
    return 0;
}
