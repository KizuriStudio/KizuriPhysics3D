// KizuriPhysics - Core/Memory.cpp
#include "Kizuri/Core/Memory.h"

#include <cstdio>

namespace kizuri {

// ---------------------------------------------------------------------------
// Aligned allocation
// ---------------------------------------------------------------------------
void* AllocateAligned(usize size, usize alignment) {
    if (alignment < sizeof(void*)) alignment = sizeof(void*);
    KZ_ASSERT((alignment & (alignment - 1)) == 0);
#if defined(KZ_PLATFORM_WINDOWS)
    void* ptr = _aligned_malloc(size, alignment);
    if (!ptr) KZ_FATAL("AllocateAligned failed");
    return ptr;
#else
    usize aligned = alignment < sizeof(void*) ? sizeof(void*) : alignment;
    void* ptr = nullptr;
    if (posix_memalign(&ptr, aligned, size) != 0) KZ_FATAL("AllocateAligned failed");
    return ptr;
#endif
}

void FreeAligned(void* ptr) {
    if (!ptr) return;
#if defined(KZ_PLATFORM_WINDOWS)
    _aligned_free(ptr);
#else
    free(ptr);
#endif
}

// ---------------------------------------------------------------------------
// HeapAllocator / default
// ---------------------------------------------------------------------------
HeapAllocator& HeapAllocator::Get() {
    static HeapAllocator instance;
    return instance;
}

static Allocator* gDefaultAllocator = nullptr;

Allocator& GetDefaultAllocator() {
    return gDefaultAllocator ? *gDefaultAllocator : static_cast<Allocator&>(HeapAllocator::Get());
}

void SetDefaultAllocator(Allocator* allocator) { gDefaultAllocator = allocator; }

// ---------------------------------------------------------------------------
// LinearAllocator
// ---------------------------------------------------------------------------
LinearAllocator::LinearAllocator(usize capacity, Allocator& backing)
    : mCapacity(capacity), mPeak(0), mBacking(backing) {
    mBase = static_cast<u8*>(backing.Allocate(capacity, 64));
    mCurrent = mBase;
    mEnd = mBase + capacity;
}

LinearAllocator::~LinearAllocator() {
    if (mBase) mBacking.Free(mBase);
}

void* LinearAllocator::Allocate(usize size, usize alignment) {
    uptr addr = reinterpret_cast<uptr>(mCurrent);
    uptr alignedAddr = (addr + alignment - 1) & ~uptr(alignment - 1);
    u8* result = reinterpret_cast<u8*>(alignedAddr);
    if (result + size > mEnd) KZ_FATAL("LinearAllocator out of memory");
    mCurrent = result + size;
    usize used = usize(mCurrent - mBase);
    if (used > mPeak) mPeak = used;
    return result;
}

void LinearAllocator::Free(void*) {}

// ---------------------------------------------------------------------------
// StackAllocator
// ---------------------------------------------------------------------------
StackAllocator::StackAllocator(usize capacity, Allocator& backing)
    : mCapacity(capacity), mBacking(backing) {
    mBase = static_cast<u8*>(backing.Allocate(capacity, 64));
    mCurrent = mBase;
    mEnd = mBase + capacity;
}

StackAllocator::~StackAllocator() {
    if (mBase) mBacking.Free(mBase);
}

void* StackAllocator::Allocate(usize size, usize alignment) {
    uptr addr = reinterpret_cast<uptr>(mCurrent);
    uptr alignedAddr = (addr + alignment - 1) & ~uptr(alignment - 1);
    u8* result = reinterpret_cast<u8*>(alignedAddr);
    if (result + size > mEnd) KZ_FATAL("StackAllocator out of memory");
    mCurrent = result + size;
    return result;
}

void StackAllocator::Free(void*) {}

// ---------------------------------------------------------------------------
// FreeListAllocator
//
// Physical blocks are laid out end to end in the backing buffer. Each block
// begins with a 48-byte header (a multiple of 16) and its payload starts right
// after, which keeps payloads 16-byte aligned as long as every block size is a
// multiple of 16. That invariant is maintained by rounding every request up.
// ---------------------------------------------------------------------------
struct FreeListAllocator::Block {
    usize  size;          // total block size including this header
    bool   isFree;
    Block* prevPhysical;
    Block* nextPhysical;
    Block* prevFree;
    Block* nextFree;
};

namespace {
constexpr usize kBlockAlign = 16;
KZ_FORCEINLINE usize RoundUp16(usize v) { return (v + (kBlockAlign - 1)) & ~usize(kBlockAlign - 1); }
} // namespace

FreeListAllocator::FreeListAllocator(usize capacity, Allocator& backing)
    : mCapacity(RoundUp16(capacity)), mUsed(0), mFreeHead(nullptr), mBacking(backing) {
    mBase = static_cast<u8*>(backing.Allocate(mCapacity, 64));
    Block* initial = reinterpret_cast<Block*>(mBase);
    initial->size = mCapacity;
    initial->isFree = true;
    initial->prevPhysical = nullptr;
    initial->nextPhysical = nullptr;
    initial->prevFree = nullptr;
    initial->nextFree = nullptr;
    mFreeHead = initial;
}

FreeListAllocator::~FreeListAllocator() {
    if (mBase) mBacking.Free(mBase);
}

FreeListAllocator::Block* FreeListAllocator::FindFirstFit(usize size, usize /*alignment*/) {
    const usize headerSize = sizeof(Block);
    for (Block* b = mFreeHead; b != nullptr; b = b->nextFree) {
        if (b->size >= headerSize + size) return b;
    }
    return nullptr;
}

void* FreeListAllocator::Allocate(usize size, usize alignment) {
    KZ_ASSERT(alignment <= kBlockAlign && "FreeListAllocator supports alignment up to 16");
    const usize headerSize = sizeof(Block);
    usize rounded = RoundUp16(size);

    Block* block = FindFirstFit(rounded, alignment);
    if (!block) return nullptr;

    // Unlink from the free list.
    if (block->prevFree) block->prevFree->nextFree = block->nextFree;
    else mFreeHead = block->nextFree;
    if (block->nextFree) block->nextFree->prevFree = block->prevFree;
    block->prevFree = block->nextFree = nullptr;

    // Split off the remainder when it can hold another block plus payload.
    if (block->size >= headerSize + rounded + headerSize + kBlockAlign) {
        u8* restAddr = reinterpret_cast<u8*>(block) + headerSize + rounded;
        Block* rest = reinterpret_cast<Block*>(restAddr);
        rest->size = block->size - (headerSize + rounded);
        rest->isFree = true;
        rest->prevPhysical = block;
        rest->nextPhysical = block->nextPhysical;
        if (rest->nextPhysical) rest->nextPhysical->prevPhysical = rest;
        block->nextPhysical = rest;
        block->size = headerSize + rounded;

        rest->prevFree = nullptr;
        rest->nextFree = mFreeHead;
        if (mFreeHead) mFreeHead->prevFree = rest;
        mFreeHead = rest;
    }

    block->isFree = false;
    mUsed += block->size;
    return reinterpret_cast<void*>(reinterpret_cast<u8*>(block) + headerSize);
}

void FreeListAllocator::Coalesce() {
    const usize headerSize = sizeof(Block);
    KZ_UNUSED(headerSize);
    Block* b = reinterpret_cast<Block*>(mBase);
    while (b != nullptr) {
        if (b->isFree && b->nextPhysical && b->nextPhysical->isFree) {
            Block* next = b->nextPhysical;
            // Unlink `next` from the free list.
            if (next->prevFree) next->prevFree->nextFree = next->nextFree;
            else mFreeHead = next->nextFree;
            if (next->nextFree) next->nextFree->prevFree = next->prevFree;
            // Absorb.
            b->size += next->size;
            b->nextPhysical = next->nextPhysical;
            if (b->nextPhysical) b->nextPhysical->prevPhysical = b;
            // Do not advance; try to absorb further free neighbours.
        } else {
            b = b->nextPhysical;
        }
    }
}

void FreeListAllocator::Free(void* ptr) {
    if (!ptr) return;
    const usize headerSize = sizeof(Block);
    Block* block = reinterpret_cast<Block*>(reinterpret_cast<u8*>(ptr) - headerSize);
    KZ_ASSERT(!block->isFree && "double free detected");
    block->isFree = true;
    mUsed -= block->size;
    block->prevFree = nullptr;
    block->nextFree = mFreeHead;
    if (mFreeHead) mFreeHead->prevFree = block;
    mFreeHead = block;
    Coalesce();
}

} // namespace kizuri
