// KizuriPhysics - Core/JobSystem.cpp
#include "Kizuri/Core/JobSystem.h"

#include <algorithm>

namespace kizuri {

thread_local JobSystem* JobSystem::tlsCurrentSystem = nullptr;
thread_local u32 JobSystem::tlsWorkerIndex = 0xFFFFFFFFu;

namespace {
JobSystem* gJobSystem = nullptr;

/// Pending job array for ParallelFor, stored per-call on the caller's stack.
struct ParallelForContext {
    void (*fn)(void*, u32) = nullptr;
    void* data = nullptr;
    u32 begin = 0;
    u32 end = 0;
};

void ParallelForJobEntry(void* rawContext) {
    auto* ctx = static_cast<ParallelForContext*>(rawContext);
    for (u32 i = ctx->begin; i < ctx->end; ++i) ctx->fn(ctx->data, i);
}
} // namespace

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
void JobSystem::Init(u32 numThreads, u32 maxJobs) {
    if (gJobSystem) return;
    gJobSystem = new JobSystem(numThreads, maxJobs);
}

void JobSystem::Shutdown() {
    delete gJobSystem;
    gJobSystem = nullptr;
}

bool JobSystem::IsInitialized() { return gJobSystem != nullptr; }

JobSystem& JobSystem::Get() {
    KZ_ASSERT(gJobSystem != nullptr && "JobSystem not initialized");
    return *gJobSystem;
}

JobSystem::JobSystem(u32 numThreads, u32 maxJobs) : mMaxJobs(maxJobs) {
    KZ_ASSERT((maxJobs & (maxJobs - 1)) == 0 && "maxJobs must be a power of two");

    mJobPool = static_cast<Job*>(AllocateAligned(sizeof(Job) * maxJobs, 64));
    for (u32 i = 0; i < maxJobs; ++i) {
        new (&mJobPool[i]) Job();
        mJobPool[i].nextFree = (i + 1 < maxJobs) ? i32(i + 1) : -1;
    }
    mJobPoolTop.store(0, std::memory_order_relaxed);

    mQueue = new MPMCQueue<Job*>(maxJobs);

    if (numThreads == 0) {
        u32 hw = std::thread::hardware_concurrency();
        numThreads = hw > 1 ? hw - 1 : 1;
    }
    if (numThreads > 256) numThreads = 256;

    mThreads.Reserve(numThreads);
    for (u32 i = 0; i < numThreads; ++i) {
        mThreads.EmplaceBack([this, i]() { WorkerLoop(i); });
    }
}

JobSystem::~JobSystem() {
    mQuit.store(true, std::memory_order_release);
    {
        std::lock_guard<std::mutex> lock(mWaitMutex);
        mWaitCv.notify_all();
    }
    for (auto& t : mThreads) {
        if (t.joinable()) t.join();
    }
    for (u32 i = 0; i < mMaxJobs; ++i) mJobPool[i].~Job();
    FreeAligned(mJobPool);
    delete mQueue;
}

// ---------------------------------------------------------------------------
// Job pool
// ---------------------------------------------------------------------------
JobSystem::Job* JobSystem::CreateJob(const char* name, JobFunction fn, void* data) {
    // Lock-free pop from the free-list stack.
    i32 top = mJobPoolTop.load(std::memory_order_acquire);
    for (;;) {
        if (top == -1) return nullptr; // pool exhausted
        Job* job = &mJobPool[top];
        i32 next = job->nextFree;
        if (mJobPoolTop.compare_exchange_weak(top, next, std::memory_order_acq_rel)) {
            job->function = fn;
            job->data = data;
            job->name = name;
            job->numDependencies.store(0, std::memory_order_relaxed);
            job->finished.store(false, std::memory_order_relaxed);
            job->submitted.store(false, std::memory_order_relaxed);
            job->dependents.Clear();
            job->owner = this;
            return job;
        }
    }
}

void JobSystem::FreeJob(Job* job) {
    if (!job) return;
    KZ_ASSERT(job->finished.load(std::memory_order_acquire));
    job->dependents.ClearAndFree();
    job->function = nullptr;
    job->data = nullptr;
    i32 top = mJobPoolTop.load(std::memory_order_acquire);
    for (;;) {
        job->nextFree = top;
        if (mJobPoolTop.compare_exchange_weak(top, i32(job - mJobPool), std::memory_order_acq_rel)) return;
    }
}

// ---------------------------------------------------------------------------
// Dependency graph
// ---------------------------------------------------------------------------
void JobSystem::AddDependency(Job* job, Job* dependency) {
    KZ_ASSERT(job && dependency);
    KZ_ASSERT(!job->submitted.load(std::memory_order_relaxed));
    KZ_ASSERT(!dependency->submitted.load(std::memory_order_relaxed));
    job->numDependencies.fetch_add(1, std::memory_order_relaxed);
    dependency->dependents.PushBack(job);
}

void JobSystem::AddJob(Job* job) {
    KZ_ASSERT(job);
    KZ_ASSERT(!job->submitted.load(std::memory_order_relaxed));
    job->submitted.store(true, std::memory_order_release);
    mOutstandingJobs.fetch_add(1, std::memory_order_acq_rel);
    if (job->numDependencies.load(std::memory_order_acquire) == 0) {
        while (!mQueue->Push(job)) std::this_thread::yield();
    }
}

// ---------------------------------------------------------------------------
// Execution
// ---------------------------------------------------------------------------
void JobSystem::ExecuteJob(Job* job) {
    if (job->function) job->function(job->data);
    FinishJob(job);
}

void JobSystem::FinishJob(Job* job) {
    job->finished.store(true, std::memory_order_release);

    for (Job* dependent : job->dependents) {
        // Release: the decrement must happen-after this job's writes.
        if (dependent->numDependencies.fetch_sub(1, std::memory_order_acq_rel) == 1) {
            while (!mQueue->Push(dependent)) std::this_thread::yield();
        }
    }

    mOutstandingJobs.fetch_sub(1, std::memory_order_acq_rel);
    std::lock_guard<std::mutex> lock(mWaitMutex);
    mWaitCv.notify_all();
}

bool JobSystem::TryGetJob(Job*& outJob) {
    return mQueue->Pop(outJob);
}

void JobSystem::WorkerLoop(u32 workerIndex) {
    tlsCurrentSystem = this;
    tlsWorkerIndex = workerIndex;

    for (;;) {
        Job* job = nullptr;
        if (mQueue->Pop(job)) {
            ExecuteJob(job);
            continue;
        }

        if (mQuit.load(std::memory_order_acquire)) break;

        // Idle: wait briefly for work.
        std::unique_lock<std::mutex> lock(mWaitMutex);
        mWaitCv.wait_for(lock, std::chrono::microseconds(200));
    }
}

void JobSystem::WaitFor(Job* job) {
    if (!job) return;
    KZ_ASSERT(job->submitted.load(std::memory_order_acquire));

    while (!job->finished.load(std::memory_order_acquire)) {
        Job* j = nullptr;
        if (mQueue->Pop(j)) {
            ExecuteJob(j);
        } else {
            std::this_thread::yield();
        }
    }
}

void JobSystem::WaitForAll() {
    while (mOutstandingJobs.load(std::memory_order_acquire) > 0) {
        Job* j = nullptr;
        if (mQueue->Pop(j)) {
            ExecuteJob(j);
        } else {
            std::this_thread::yield();
        }
    }
}

bool JobSystem::IsWorkerThread() const { return tlsCurrentSystem == this; }

// ---------------------------------------------------------------------------
// ParallelFor
// ---------------------------------------------------------------------------
void JobSystem::ParallelFor(u32 begin, u32 end, u32 minBatch,
                            void (*fn)(void* data, u32 index), void* data) {
    if (end <= begin) return;

    u32 total = end - begin;
    u32 numThreads = NumThreads() + 1; // include calling thread
    if (numThreads < 1) numThreads = 1;

    u32 numBatches = (total + minBatch - 1) / minBatch;
    if (numBatches == 0) numBatches = 1;
    if (numBatches > numThreads) numBatches = numThreads;
    if (numBatches > mMaxJobs / 2) numBatches = mMaxJobs / 2;

    u32 perBatch = (total + numBatches - 1) / numBatches;

    // Stack-allocated contexts (numBatches is bounded by thread count).
    constexpr u32 kMaxBatches = 256;
    KZ_ASSERT(numBatches <= kMaxBatches);
    ParallelForContext contexts[kMaxBatches];
    Job* jobs[kMaxBatches];

    u32 created = 0;
    for (u32 b = 0; b < numBatches; ++b) {
        u32 lo = begin + b * perBatch;
        if (lo >= end) break;
        u32 hi = lo + perBatch;
        if (hi > end) hi = end;
        contexts[created] = { fn, data, lo, hi };
        jobs[created] = CreateJob("ParallelFor", ParallelForJobEntry, &contexts[created]);
        if (!jobs[created]) break;
        ++created;
    }

    for (u32 i = 0; i < created; ++i) AddJob(jobs[i]);
    for (u32 i = 0; i < created; ++i) WaitFor(jobs[i]);
    for (u32 i = 0; i < created; ++i) FreeJob(jobs[i]);
}

} // namespace kizuri
