// melonDS Platform layer for the Nintendo Switch test entrypoint.
// Only what the core needs to boot a cartridge: files, threads, timing and saves.

#include <switch.h>

#include <sys/stat.h>

#include <atomic>
#include <condition_variable>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <chrono>
#include <cstdlib>
#include <functional>

#include "Platform.h"
#include "SPI_Firmware.h"
#include "mic_blow.h"

#include "MelonInstance.h"

#include "SwitchFrontend.h"

namespace SwitchFrontend
{
std::string BaseDir = "sdmc:/switch/watermelonds/";
bool StopRequested = false;
std::atomic<bool> FastForwardActive { false };
std::atomic<bool> MicBlowing { false };

static FILE* LogFile = nullptr;
static std::mutex LogMutex;

void OpenLog(const std::string& path)
{
    LogFile = fopen(path.c_str(), "w");
}

void CloseLog()
{
    if (LogFile)
        fclose(LogFile);
    LogFile = nullptr;
}

void LogV(const char* fmt, va_list args)
{
    std::lock_guard<std::mutex> lock(LogMutex);
    va_list copy;
    va_copy(copy, args);
    vprintf(fmt, copy);
    va_end(copy);
    if (LogFile)
    {
        vfprintf(LogFile, fmt, args);
        // every flush is an fs IPC round trip on the emulation thread; once a
        // second keeps the log current enough, and crashes close the file
        static u64 lastFlushTick = 0;
        const u64 now = armGetSystemTick();
        if (now - lastFlushTick >= armGetSystemTickFreq())
        {
            fflush(LogFile);
            lastFlushTick = now;
        }
    }
}

void Log(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    LogV(fmt, args);
    va_end(args);
}
}

// Debug toggles the core expects from the Android frontend.
namespace MelonDSAndroid
{
bool areRendererDebugToolsEnabled() { return false; }
bool areRendererDebugBgObjLogsEnabled() { return false; }
bool areRenderer2DDebugControlsActive() { return false; }
bool isVulkanGpu2DPerfLoggingEnabled() { return false; }
int getRenderer2DDebugForcedMode(melonDS::u32) { return -1; }
bool isRenderer2DDebugBgLayerEnabled(melonDS::u32, melonDS::u32) { return true; }
bool isRenderer2DDebugBgPriorityEnabled(melonDS::u32, melonDS::u32) { return true; }
bool isRenderer2DDebugBackgroundKindEnabled(melonDS::u32) { return true; }
bool areRenderer2DDebugObjectsEnabled(melonDS::u32) { return true; }
bool isRenderer2DDebugObjectPriorityEnabled(melonDS::u32, melonDS::u32) { return true; }
bool isRenderer2DDebugObjectOrderEnabled(melonDS::u32, melonDS::u32) { return true; }
bool isRenderer2DDebugObjectFeatureEnabled(melonDS::u32) { return true; }
}

namespace melonDS::Platform
{

void SignalStop(StopReason reason, void* userdata)
{
    SwitchFrontend::Log("core requested stop (reason %d)\n", (int)reason);
    SwitchFrontend::StopRequested = true;
}

std::string GetLocalFilePath(const std::string& filename)
{
    if (filename.find(':') != std::string::npos || (!filename.empty() && filename[0] == '/'))
        return filename;
    return SwitchFrontend::BaseDir + filename;
}

static const char* ModeString(FileMode mode, bool exists)
{
    if (mode & FileMode::Append)
        return (mode & FileMode::Text) ? "a" : "ab";

    if (mode & FileMode::Write)
    {
        if ((mode & FileMode::Preserve) && exists)
            return (mode & FileMode::Text) ? "r+" : "rb+";
        if (mode & FileMode::Read)
            return (mode & FileMode::Text) ? "w+" : "wb+";
        return (mode & FileMode::Text) ? "w" : "wb";
    }

    return (mode & FileMode::Text) ? "r" : "rb";
}

FileHandle* OpenFile(const std::string& path, FileMode mode)
{
    if ((mode & (FileMode::ReadWrite | FileMode::Append)) == FileMode::None)
        return nullptr;

    bool exists = FileExists(path);
    if ((mode & FileMode::Write) && (mode & FileMode::NoCreate) && !exists)
        return nullptr;

    return reinterpret_cast<FileHandle*>(fopen(path.c_str(), ModeString(mode, exists)));
}

FileHandle* OpenLocalFile(const std::string& path, FileMode mode)
{
    return OpenFile(GetLocalFilePath(path), mode);
}

bool FileExists(const std::string& name)
{
    FILE* f = fopen(name.c_str(), "rb");
    if (!f)
        return false;
    fclose(f);
    return true;
}

bool LocalFileExists(const std::string& name)
{
    return FileExists(GetLocalFilePath(name));
}

bool CheckFileWritable(const std::string& filepath)
{
    // Horizon refuses to open a file for writing while it is open elsewhere,
    // and callers probe files they are still reading. An existing file on the
    // SD card is writable, so only a missing one needs the create attempt.
    struct stat info;
    if (stat(filepath.c_str(), &info) == 0)
        return S_ISREG(info.st_mode);

    FILE* f = fopen(filepath.c_str(), "ab");
    if (!f)
        return false;
    fclose(f);
    return true;
}

bool CheckLocalFileWritable(const std::string& filepath)
{
    return CheckFileWritable(GetLocalFilePath(filepath));
}

bool CloseFile(FileHandle* file)
{
    return fclose(reinterpret_cast<FILE*>(file)) == 0;
}

bool IsEndOfFile(FileHandle* file)
{
    return feof(reinterpret_cast<FILE*>(file)) != 0;
}

bool FileReadLine(char* str, int count, FileHandle* file)
{
    return fgets(str, count, reinterpret_cast<FILE*>(file)) != nullptr;
}

u64 FilePosition(FileHandle* file)
{
    return ftello(reinterpret_cast<FILE*>(file));
}

bool FileSeek(FileHandle* file, s64 offset, FileSeekOrigin origin)
{
    int whence;
    switch (origin)
    {
    case FileSeekOrigin::Start: whence = SEEK_SET; break;
    case FileSeekOrigin::Current: whence = SEEK_CUR; break;
    case FileSeekOrigin::End: whence = SEEK_END; break;
    default: return false;
    }
    return fseeko(reinterpret_cast<FILE*>(file), offset, whence) == 0;
}

void FileRewind(FileHandle* file)
{
    rewind(reinterpret_cast<FILE*>(file));
}

u64 FileRead(void* data, u64 size, u64 count, FileHandle* file)
{
    return fread(data, size, count, reinterpret_cast<FILE*>(file));
}

bool FileFlush(FileHandle* file)
{
    return fflush(reinterpret_cast<FILE*>(file)) == 0;
}

u64 FileWrite(const void* data, u64 size, u64 count, FileHandle* file)
{
    return fwrite(data, size, count, reinterpret_cast<FILE*>(file));
}

u64 FileWriteFormatted(FileHandle* file, const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int ret = vfprintf(reinterpret_cast<FILE*>(file), fmt, args);
    va_end(args);
    return ret < 0 ? 0 : ret;
}

u64 FileLength(FileHandle* file)
{
    FILE* f = reinterpret_cast<FILE*>(file);
    off_t pos = ftello(f);
    fseeko(f, 0, SEEK_END);
    off_t len = ftello(f);
    fseeko(f, pos, SEEK_SET);
    return len < 0 ? 0 : len;
}

void Log(LogLevel level, const char* fmt, ...)
{
    if (level == LogLevel::Debug)
        return;

    // the shared frontend logs without trailing newlines, the core with them
    char line[1024];
    va_list args;
    va_start(args, fmt);
    int length = vsnprintf(line, sizeof(line) - 1, fmt, args);
    va_end(args);
    if (length < 0)
        return;
    if (length > (int)sizeof(line) - 2)
        length = sizeof(line) - 2;
    if (length == 0 || line[length - 1] != '\n')
    {
        line[length] = '\n';
        line[length + 1] = '\0';
    }
    SwitchFrontend::Log("%s", line);
}

// main thread's core, where they only compete with the emulated CPUs.
struct Thread
{
    ::Thread Handle {};
    std::function<void()> Func;
    bool Started = false;
    bool Joined = false;
};

static void ThreadEntry(void* arg)
{
    static_cast<Thread*>(arg)->Func();
}

Thread* Thread_Create(std::function<void()> func)
{
    static int nextCore = 0;

    Thread* thread = new Thread;
    thread->Func = std::move(func);

    s32 priority = 0x2C;
    svcGetThreadPriority(&priority, CUR_THREAD_HANDLE);

    int core = 1 + (nextCore++ & 1);
    if (R_SUCCEEDED(threadCreate(&thread->Handle, ThreadEntry, thread, nullptr, 0x100000, priority, core))
        && R_SUCCEEDED(threadStart(&thread->Handle)))
    {
        thread->Started = true;
    }
    else
    {
        SwitchFrontend::Log("failed to start a worker thread\n");
        abort();
    }
    return thread;
}

void Thread_Wait(Thread* thread)
{
    if (!thread->Started || thread->Joined)
        return;
    threadWaitForExit(&thread->Handle);
    threadClose(&thread->Handle);
    thread->Joined = true;
}

void Thread_Free(Thread* thread)
{
    Thread_Wait(thread);
    delete thread;
}

struct Semaphore
{
    std::mutex Lock;
    std::condition_variable Cond;
    int Count = 0;
};

Semaphore* Semaphore_Create()
{
    return new Semaphore;
}

void Semaphore_Free(Semaphore* sema)
{
    delete sema;
}

void Semaphore_Reset(Semaphore* sema)
{
    std::lock_guard<std::mutex> lock(sema->Lock);
    sema->Count = 0;
}

void Semaphore_Wait(Semaphore* sema)
{
    std::unique_lock<std::mutex> lock(sema->Lock);
    sema->Cond.wait(lock, [sema] { return sema->Count > 0; });
    sema->Count--;
}

bool Semaphore_TryWait(Semaphore* sema, int timeout_ms)
{
    std::unique_lock<std::mutex> lock(sema->Lock);
    if (timeout_ms == 0)
    {
        if (sema->Count <= 0)
            return false;
    }
    else if (!sema->Cond.wait_for(lock, std::chrono::milliseconds(timeout_ms), [sema] { return sema->Count > 0; }))
    {
        return false;
    }
    sema->Count--;
    return true;
}

void Semaphore_Post(Semaphore* sema, int count)
{
    {
        std::lock_guard<std::mutex> lock(sema->Lock);
        sema->Count += count;
    }
    sema->Cond.notify_all();
}

struct Mutex
{
    std::mutex Handle;
};

Mutex* Mutex_Create()
{
    return new Mutex;
}

void Mutex_Free(Mutex* mutex)
{
    delete mutex;
}

void Mutex_Lock(Mutex* mutex)
{
    mutex->Handle.lock();
}

void Mutex_Unlock(Mutex* mutex)
{
    mutex->Handle.unlock();
}

bool Mutex_TryLock(Mutex* mutex)
{
    return mutex->Handle.try_lock();
}

void Sleep(u64 usecs)
{
    svcSleepThread(usecs * 1000);
}

u64 GetMSCount()
{
    return armTicksToNs(armGetSystemTick()) / 1000000;
}

u64 GetUSCount()
{
    return armTicksToNs(armGetSystemTick()) / 1000;
}

// The core's userdata is the MelonInstance that owns it; saves go through its SaveManager.
void WriteNDSSave(const u8* savedata, u32 savelen, u32 writeoffset, u32 writelen, void* userdata)
{
    if (auto instance = static_cast<MelonDSAndroid::MelonInstance*>(userdata))
        instance->requestNdsSaveWrite(savedata, savelen, writeoffset, writelen);
}

void WriteGBASave(const u8* savedata, u32 savelen, u32 writeoffset, u32 writelen, void* userdata)
{
    if (auto instance = static_cast<MelonDSAndroid::MelonInstance*>(userdata))
        instance->requestGbaSaveWrite(savedata, savelen, writeoffset, writelen);
}

// only the generated firmware is used, and it is not persisted
void WriteFirmware(const Firmware& firmware, u32 writeoffset, u32 writelen, void* userdata) {}
void WriteDateTime(int year, int month, int day, int hour, int minute, int second, void* userdata) {}

void MP_Begin(void* userdata) {}
void MP_End(void* userdata) {}
int MP_SendPacket(u8* data, int len, u64 timestamp, void* userdata) { return 0; }
int MP_RecvPacket(u8* data, u64* timestamp, void* userdata) { return 0; }
int MP_SendCmd(u8* data, int len, u64 timestamp, void* userdata) { return 0; }
int MP_SendReply(u8* data, int len, u64 timestamp, u16 aid, void* userdata) { return 0; }
int MP_SendAck(u8* data, int len, u64 timestamp, void* userdata) { return 0; }
int MP_RecvHostPacket(u8* data, u64* timestamp, void* userdata) { return 0; }
u16 MP_RecvReplies(u8* data, u64 timestamp, u16 aidmask, void* userdata) { return 0; }

int Net_SendPacket(u8* data, int len, void* userdata)
{
    auto instance = static_cast<MelonDSAndroid::MelonInstance*>(userdata);
    return instance ? instance->sendNetPacket(data, len) : 0;
}

int Net_RecvPacket(u8* data, void* userdata)
{
    auto instance = static_cast<MelonDSAndroid::MelonInstance*>(userdata);
    return instance ? instance->receiveNetPacket(data) : 0;
}

void Camera_Start(int num, void* userdata) {}
void Camera_Stop(int num, void* userdata) {}
void Camera_CaptureFrame(int num, u32* frame, int width, int height, bool yuv, void* userdata) {}

void Mic_Start(void* userdata) {}
void Mic_Stop(void* userdata) {}

// There is no microphone to record from; the "blow" source loops the core's
// blow sample while the microphone button is held, and is silent otherwise.
int Mic_ReadInput(s16* data, int maxlength, void* userdata)
{
    static int readPos = 0;

    if (!SwitchFrontend::MicBlowing.load())
    {
        memset(data, 0, maxlength * sizeof(s16));
        return maxlength;
    }

    const int sampleCount = sizeof(mic_blow) / sizeof(s16);
    for (int i = 0; i < maxlength; i++)
    {
        data[i] = mic_blow[readPos];
        readPos = (readPos + 1) % sampleCount;
    }
    return maxlength;
}

AACDecoder* AAC_Init() { return nullptr; }
void AAC_DeInit(AACDecoder* dec) {}
bool AAC_Configure(AACDecoder* dec, int frequency, int channels) { return false; }
bool AAC_DecodeFrame(AACDecoder* dec, const void* input, int inputlen, void* output, int outputlen) { return false; }

bool Addon_KeyDown(KeyType type, void* userdata) { return false; }
void Addon_RumbleStart(u32 len, void* userdata) {}
void Addon_RumbleStop(void* userdata) {}
float Addon_MotionQuery(MotionQueryType type, void* userdata) { return 0.0f; }

DynamicLibrary* DynamicLibrary_Load(const char* lib) { return nullptr; }
void DynamicLibrary_Unload(DynamicLibrary* lib) {}
void* DynamicLibrary_LoadFunction(DynamicLibrary* lib, const char* name) { return nullptr; }

}

extern "C" void _start();

namespace melonDS
{
// Called by the JIT fault handler for faults that are not fastmem accesses.
void HandleFault(u64 pc, u64 lr, u64 fp, u64 faultAddr, u32 desc)
{
    u64 base = (u64)&_start;
    SwitchFrontend::Log("crash: desc=%x pc=%llx (nro+%llx) lr=%llx (nro+%llx) far=%llx\n", desc,
        (unsigned long long)pc, (unsigned long long)(pc - base),
        (unsigned long long)lr, (unsigned long long)(lr - base),
        (unsigned long long)faultAddr);
    SwitchFrontend::CloseLog();
    svcExitProcess();
}
}

// Core roles, following the DrasticDS port (pthr.c) and the Cemu and Vita3K
// ports: the first granted core runs the emulated CPUs alone, every other
// thread is spread round-robin over the remaining granted cores with a mask
// that lets it migrate between them, and a fourth core, when the process is
// given one, takes the background threads (audio, presentation).
namespace SwitchFrontend::Cores
{
namespace
{
std::once_flag InitOnce;
int CoreCount = 0;
int EmulationCore = 0;
int WorkList[4] = {};
int WorkCount = 0;
u32 WorkMask = 0;
int BackgroundCore = -1;
std::atomic<unsigned> WorkNext { 0 };
std::atomic<unsigned> BackgroundNext { 0 };

void Init()
{
    std::call_once(InitOnce, [] {
        u64 mask = 0;
        if (R_FAILED(svcGetInfo(&mask, InfoType_CoreMask, CUR_PROCESS_HANDLE, 0)) || mask == 0)
            mask = 0x7; // the usual application allotment: cores 0, 1 and 2

        int cores[4];
        for (int core = 0; core < 4; core++)
        {
            if (mask & (1u << core))
                cores[CoreCount++] = core;
        }

        // with 4 cores the top one is reserved for background threads; with 3
        // or fewer every core is needed for the heavy threads
        int hotCount = CoreCount;
        if (CoreCount >= 4)
        {
            hotCount = CoreCount - 1;
            BackgroundCore = cores[CoreCount - 1];
        }

        EmulationCore = cores[0];
        for (int i = 1; i < hotCount; i++)
        {
            WorkList[WorkCount++] = cores[i];
            WorkMask |= 1u << cores[i];
        }
        if (WorkCount == 0)
        {
            WorkList[WorkCount++] = EmulationCore;
            WorkMask = 1u << EmulationCore;
        }

        SwitchFrontend::Log("cores: %d granted (mask 0x%llx), emulation %d, workers 0x%x, background %d\n",
            CoreCount, (unsigned long long)mask, EmulationCore, WorkMask, BackgroundCore);
    });
}
}

void PinEmulationThread()
{
    Init();
    if (R_FAILED(svcSetThreadCoreMask(CUR_THREAD_HANDLE, EmulationCore, 1u << EmulationCore)))
        SwitchFrontend::Log("cores: could not pin the emulation thread\n");
}

void NextWorkCore(int& core, u32& mask)
{
    Init();
    core = WorkList[WorkNext++ % (unsigned)WorkCount];
    mask = WorkMask;
}

void NextBackgroundCore(int& core, u32& mask)
{
    Init();
    if (BackgroundCore >= 0)
    {
        core = BackgroundCore;
        mask = 1u << BackgroundCore;
        return;
    }
    core = WorkList[BackgroundNext++ % (unsigned)WorkCount];
    mask = WorkMask;
}

void PinCurrentThreadToWorkCores()
{
    int core;
    u32 mask;
    NextWorkCore(core, mask);
    svcSetThreadCoreMask(CUR_THREAD_HANDLE, core, mask);
}
}

// libnx starts std::thread and pthread workers on the process's default core,
// which is the emulation thread's. Those created without an explicit core
// (the frontend's and the Vulkan renderer's workers) go to the worker cores.
extern "C" Result __real_threadCreate(::Thread* t, ThreadFunc entry, void* arg, void* stack_mem, size_t stack_sz, int prio, int cpuid);

extern "C" Result __wrap_threadCreate(::Thread* t, ThreadFunc entry, void* arg, void* stack_mem, size_t stack_sz, int prio, int cpuid)
{
    if (cpuid != -2)
        return __real_threadCreate(t, entry, arg, stack_mem, stack_sz, prio, cpuid);

    int core;
    u32 mask;
    SwitchFrontend::Cores::NextWorkCore(core, mask);
    Result result = __real_threadCreate(t, entry, arg, stack_mem, stack_sz, prio, core);
    if (R_SUCCEEDED(result))
        svcSetThreadCoreMask(t->handle, core, mask);
    return result;
}
