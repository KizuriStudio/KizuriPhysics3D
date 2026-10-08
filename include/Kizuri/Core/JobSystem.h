// KizuriPhysics - Core/JobSystem.h
// Work-stealing job scheduler used to parallelize the physics pipeline.
#pragma once

#include "Types.h"
#include "Memory.h"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace kizuri {

// ---------------------------------------------------------------------------
// Bounded MPMC queue (Vyukov's algorithm).
// ---------------------------------------------------------------------------
template <typename T>
class MPMCQueue {
public:
    explicit MPMCQueue(usize capacityPow2)
        : mCapacity(capacityPow2), mMask(capacityPow2 - 1) {
        KZ_ASSERT((capacityPow2 & (capacityPow2 - 1)) == 0 && "capacity must be a power of two");
        mBuffer = static_cast<Cell*>(AllocateAligned(sizeof(Cell) * mCapacity, 64));
        for (usize i = 0; i < mCapacity; ++i) mBuffer[i].sequence.store(i, std::memory_order_relaxed);
        mEnqueuePos.store(0, std::memory_order_relaxed);
        mDequeuePos.store(0, std::memory_order_relaxed);
    }
    ~MPMCQueue() { FreeAligned(mBuffer); }

    MPMCQueue(const MPMCQueue&) = delete;
    MPMCQueue& operator=(const MPMCQueue&) = delete;

    /// Push a value. Returns false if the queue is full.
    bool Push(const T& value) {
        usize pos = mEnqueuePos.load(std::memory_order_relaxed);
        for (;;) {
            Cell* cell = &mBuffer[pos & mMask];
            usize seq = cell->sequence.load(std::memory_order_acquire);
            iptr diff = iptr(seq) - iptr(pos);
            if (diff == 0) {
                if (mEnqueuePos.compare_exchange_weak(pos, pos + 1, std::memory_order_relaxed)) {
                    cell->data = value;
                    cell->sequence.store(pos + 1, std::memory_order_release);
                    return true;
                }
            } else if (diff < 0) {
                return false; // full
            } else {
                pos = mEnqueuePos.load(std::memory_order_relaxed);
            }
        }
    }

    /// Pop a value. Returns false if the queue is empty.
    bool Pop(T& out) {
        usize pos = mDequeuePos.load(std::memory_order_relaxed);
        for (;;) {
            Cell* cell = &mBuffer[pos & mMask];
            usize seq = cell->sequence.load(std::memory_order_acquire);
            iptr diff = iptr(seq) - iptr(pos + 1);
            if (diff == 0) {
                if (mDequeuePos.compare_exchange_weak(pos, pos + 1, std::memory_order_relaxed)) {
                    out = cell->data;
                    cell->sequence.store(pos + mMask + 1, std::memory_order_release);
                    return true;
                }
            } else if (diff < 0) {
                return false; // empty
            } else {
                pos = mDequeuePos.load(std::memory_order_relaxed);
            }
        }
    }

    usize Capacity() const { return mCapacity; }

private:
    using iptr = std::intptr_t;
    struct Cell {
        std::atomic<usize> sequence;
        T data;
    };
    Cell* mBuffer;
    usize mCapacity;
    usize mMask;
    alignas(64) std::atomic<usize> mEnqueuePos;
    alignas(64) std::atomic<usize> mDequeuePos;
};

// ---------------------------------------------------------------------------
// JobSystem
// ---------------------------------------------------------------------------
class JobSystem {
public:
    using JobFunction = void (*)(void* data);

    struct Job {
        JobFunction function = nullptr;
        void* data = nullptr;
        const char* name = nullptr;

        std::atomic<i32> numDependencies{0};
        std::atomic<bool> finished{false};
        std::atomic<bool> submitted{false};
        Vector<Job*> dependents;
        JobSystem* owner = nullptr;
        i32 nextFree = -1; // used by the job pool free list
    };

    /// Create and start the global job system with `numThreads` worker threads.
    /// numThreads == 0 selects hardware_concurrency - 1. Safe to call once.
    static void Init(u32 numThreads = 0, u32 maxJobs = 1u << 16);
    static void Shutdown();
    static bool IsInitialized();
    static JobSystem& Get();

    u32 NumThreads() const { return u32(mThreads.Size()); }
    u32 MaxJobs() const { return mMaxJobs; }

    /// Allocate a job from the pool. Thread-safe.
    Job* CreateJob(const char* name, JobFunction fn, void* data);
    /// Return a job to the pool. Only valid once the job has finished.
    void FreeJob(Job* job);

    /// Make `job` wait for `dependency`. Call before AddJob on either.
    void AddDependency(Job* job, Job* dependency);

    /// Submit a job to the scheduler. If it still has unfinished dependencies
    /// it becomes runnable automatically once they complete.
    void AddJob(Job* job);

    /// Block until the job has completed, helping execute queued work meanwhile.
    void WaitFor(Job* job);

    /// Block until all submitted jobs have completed.
    void WaitForAll();

    /// Convenience: run a range [begin, end) in parallel, chunked.
    /// fn(void* data, u32 index) is invoked for each index.
    void ParallelFor(u32 begin, u32 end, u32 minBatch,
                     void (*fn)(void* data, u32 index), void* data);

    /// True when the calling thread is one of the worker threads.
    bool IsWorkerThread() const;

private:
    JobSystem(u32 numThreads, u32 maxJobs);
    ~JobSystem();

    void WorkerLoop(u32 workerIndex);
    void ExecuteJob(Job* job);
    void FinishJob(Job* job);
    bool TryGetJob(Job*& outJob);

    static thread_local JobSystem* tlsCurrentSystem;
    static thread_local u32 tlsWorkerIndex;

    u32 mMaxJobs = 0;
    Job* mJobPool = nullptr;
    std::atomic<i32> mJobPoolTop{0};
    MPMCQueue<Job*>* mQueue = nullptr;

    std::atomic<i32> mOutstandingJobs{0};
    std::atomic<bool> mQuit{false};
    Vector<std::thread> mThreads;
    std::mutex mWaitMutex;
    std::condition_variable mWaitCv;
};

} // namespace kizuri
