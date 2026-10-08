// KizuriPhysics - Core/Memory.h
// Allocators and aligned allocation helpers.
#pragma once

#include "Types.h"

#include <cstdlib>
#include <cstring>
#include <new>

namespace kizuri {

// ---------------------------------------------------------------------------
// Low-level aligned allocation
// ---------------------------------------------------------------------------
/// Aligned allocation; alignment must be a power of two.
void* AllocateAligned(usize size, usize alignment);
/// Free memory returned by AllocateAligned.
void FreeAligned(void* ptr);

/// Zero-initializing aligned allocation.
KZ_FORCEINLINE void* AllocateAlignedZeroed(usize size, usize alignment) {
    void* p = AllocateAligned(size, alignment);
    if (p) std::memset(p, 0, size);
    return p;
}

// ---------------------------------------------------------------------------
// Allocator interface
// ---------------------------------------------------------------------------
class Allocator {
public:
    virtual ~Allocator() = default;
    virtual void* Allocate(usize size, usize alignment) = 0;
    virtual void  Free(void* ptr) = 0;
    virtual const char* GetName() const = 0;
};

/// Default heap allocator.
class HeapAllocator final : public Allocator {
public:
    static HeapAllocator& Get();
    void* Allocate(usize size, usize alignment) override { return AllocateAligned(size, alignment); }
    void  Free(void* ptr) override { FreeAligned(ptr); }
    const char* GetName() const override { return "HeapAllocator"; }
};

/// Global allocator used by the engine when none is supplied.
Allocator& GetDefaultAllocator();
void SetDefaultAllocator(Allocator* allocator);

// ---------------------------------------------------------------------------
// LinearAllocator - bump allocator, freed all at once.
// Ideal for per-frame scratch memory (contact points, broadphase pairs, ...).
// ---------------------------------------------------------------------------
class LinearAllocator final : public Allocator {
public:
    LinearAllocator(usize capacity, Allocator& backing = GetDefaultAllocator());
    ~LinearAllocator() override;

    LinearAllocator(const LinearAllocator&) = delete;
    LinearAllocator& operator=(const LinearAllocator&) = delete;

    void* Allocate(usize size, usize alignment) override;
    void  Free(void* ptr) override; // no-op

    /// Reset the allocator, freeing everything at once. O(1).
    void Reset() { mCurrent = mBase; mPeak = 0; }

    usize Used() const { return usize(mCurrent - mBase); }
    usize Capacity() const { return mCapacity; }
    usize Peak() const { return mPeak; }

    const char* GetName() const override { return "LinearAllocator"; }

private:
    u8* mBase;
    u8* mCurrent;
    u8* mEnd;
    usize mCapacity;
    usize mPeak;
    Allocator& mBacking;
};

// ---------------------------------------------------------------------------
// StackAllocator - linear with marker-based rollback (for nested scopes).
// ---------------------------------------------------------------------------
class StackAllocator final : public Allocator {
public:
    using Marker = usize;

    StackAllocator(usize capacity, Allocator& backing = GetDefaultAllocator());
    ~StackAllocator() override;

    StackAllocator(const StackAllocator&) = delete;
    StackAllocator& operator=(const StackAllocator&) = delete;

    void* Allocate(usize size, usize alignment) override;
    void  Free(void* ptr) override; // no-op; use Rollback

    Marker GetMarker() const { return usize(mCurrent - mBase); }
    void   Rollback(Marker marker) { mCurrent = mBase + marker; }
    void   Reset() { mCurrent = mBase; }

    usize Used() const { return usize(mCurrent - mBase); }
    usize Capacity() const { return mCapacity; }

    const char* GetName() const override { return "StackAllocator"; }

private:
    u8* mBase;
    u8* mCurrent;
    u8* mEnd;
    usize mCapacity;
    Allocator& mBacking;
};

// ---------------------------------------------------------------------------
// FreeListAllocator - general purpose, reuses freed blocks. Good for
// long-lived objects with mixed lifetimes (bodies, shapes).
// ---------------------------------------------------------------------------
class FreeListAllocator final : public Allocator {
public:
    FreeListAllocator(usize capacity, Allocator& backing = GetDefaultAllocator());
    ~FreeListAllocator() override;

    FreeListAllocator(const FreeListAllocator&) = delete;
    FreeListAllocator& operator=(const FreeListAllocator&) = delete;

    void* Allocate(usize size, usize alignment) override;
    void  Free(void* ptr) override;

    usize Capacity() const { return mCapacity; }
    usize Used() const { return mUsed; }
    const char* GetName() const override { return "FreeListAllocator"; }

private:
    struct Block;
    Block* FindFirstFit(usize size, usize alignment);
    void   Coalesce();

    u8* mBase;
    usize mCapacity;
    usize mUsed;
    Block* mFreeHead;
    Allocator& mBacking;
};

// ---------------------------------------------------------------------------
// Allocator-aware containers: a minimal vector that can use any Allocator.
// ---------------------------------------------------------------------------
template <typename T>
class Vector {
public:
    Vector() = default;
    explicit Vector(Allocator& allocator) : mAllocator(&allocator) {}
    Vector(usize initialSize, Allocator& allocator = GetDefaultAllocator()) : mAllocator(&allocator) {
        Resize(initialSize);
    }

    Vector(const Vector& o) : mAllocator(o.mAllocator) { Assign(o.Data(), o.Size()); }
    Vector& operator=(const Vector& o) {
        if (this != &o) { mAllocator = o.mAllocator; Assign(o.Data(), o.Size()); }
        return *this;
    }
    Vector(Vector&& o) noexcept
        : mData(o.mData), mSize(o.mSize), mCapacity(o.mCapacity), mAllocator(o.mAllocator) {
        o.mData = nullptr; o.mSize = 0; o.mCapacity = 0;
    }
    Vector& operator=(Vector&& o) noexcept {
        if (this != &o) {
            ClearAndFree();
            mData = o.mData; mSize = o.mSize; mCapacity = o.mCapacity; mAllocator = o.mAllocator;
            o.mData = nullptr; o.mSize = 0; o.mCapacity = 0;
        }
        return *this;
    }
    ~Vector() { ClearAndFree(); }

    void Reserve(usize newCapacity) {
        if (newCapacity <= mCapacity) return;
        usize bytes = newCapacity * sizeof(T);
        T* newData = static_cast<T*>(mAllocator->Allocate(bytes, alignof(T)));
        for (usize i = 0; i < mSize; ++i) {
            new (&newData[i]) T(std::move(mData[i]));
            mData[i].~T();
        }
        if (mData) mAllocator->Free(mData);
        mData = newData;
        mCapacity = newCapacity;
    }

    void Resize(usize newSize) {
        if (newSize > mSize) {
            Reserve(newSize);
            for (usize i = mSize; i < newSize; ++i) new (&mData[i]) T();
        } else {
            for (usize i = newSize; i < mSize; ++i) mData[i].~T();
        }
        mSize = newSize;
    }

    void PushBack(const T& value) {
        if (mSize >= mCapacity) Reserve(mCapacity == 0 ? 8 : mCapacity * 2);
        new (&mData[mSize]) T(value);
        ++mSize;
    }
    void PushBack(T&& value) {
        if (mSize >= mCapacity) Reserve(mCapacity == 0 ? 8 : mCapacity * 2);
        new (&mData[mSize]) T(std::move(value));
        ++mSize;
    }
    template <typename... Args>
    T& EmplaceBack(Args&&... args) {
        if (mSize >= mCapacity) Reserve(mCapacity == 0 ? 8 : mCapacity * 2);
        new (&mData[mSize]) T(std::forward<Args>(args)...);
        return mData[mSize++];
    }

    void PopBack() { if (mSize > 0) { mData[--mSize].~T(); } }

    void Clear() {
        for (usize i = 0; i < mSize; ++i) mData[i].~T();
        mSize = 0;
    }
    void ClearAndFree() {
        Clear();
        if (mData) { mAllocator->Free(mData); mData = nullptr; }
        mCapacity = 0;
    }

    void EraseAt(usize index) {
        KZ_ASSERT(index < mSize);
        mData[index].~T();
        for (usize i = index; i + 1 < mSize; ++i) {
            new (&mData[i]) T(std::move(mData[i + 1]));
            mData[i + 1].~T();
        }
        --mSize;
    }
    /// Swap-remove; O(1) but does not preserve order.
    void RemoveSwap(usize index) {
        KZ_ASSERT(index < mSize);
        mData[index].~T();
        if (index != mSize - 1) {
            new (&mData[index]) T(std::move(mData[mSize - 1]));
            mData[mSize - 1].~T();
        }
        --mSize;
    }

    KZ_FORCEINLINE T&       operator[](usize i)       { KZ_ASSERT(i < mSize); return mData[i]; }
    KZ_FORCEINLINE const T& operator[](usize i) const { KZ_ASSERT(i < mSize); return mData[i]; }
    KZ_FORCEINLINE T*       Data()       { return mData; }
    KZ_FORCEINLINE const T* Data() const { return mData; }
    KZ_FORCEINLINE usize    Size() const { return mSize; }
    KZ_FORCEINLINE usize    Capacity() const { return mCapacity; }
    KZ_FORCEINLINE bool     Empty() const { return mSize == 0; }
    KZ_FORCEINLINE T&       Front() { return mData[0]; }
    KZ_FORCEINLINE T&       Back()  { return mData[mSize - 1]; }
    KZ_FORCEINLINE const T& Back() const { return mData[mSize - 1]; }

    T* begin() { return mData; }
    T* end()   { return mData + mSize; }
    const T* begin() const { return mData; }
    const T* end()   const { return mData + mSize; }

private:
    void Assign(const T* data, usize count) {
        Clear();
        Reserve(count);
        for (usize i = 0; i < count; ++i) new (&mData[i]) T(data[i]);
        mSize = count;
    }

    T* mData = nullptr;
    usize mSize = 0;
    usize mCapacity = 0;
    Allocator* mAllocator = &GetDefaultAllocator();
};

// ---------------------------------------------------------------------------
// Intrusive ref-counted pointer
// ---------------------------------------------------------------------------
class RefCounted {
public:
    void AddRef() const { ++mRefCount; }
    void Release() const { if (--mRefCount == 0) delete this; }
    u32  RefCount() const { return mRefCount; }
protected:
    RefCounted() = default;
    virtual ~RefCounted() = default;
private:
    mutable u32 mRefCount = 0;
};

template <typename T>
class Ref {
public:
    Ref() = default;
    Ref(std::nullptr_t) {}
    explicit Ref(T* ptr) : mPtr(ptr) { if (mPtr) mPtr->AddRef(); }
    Ref(const Ref& o) : mPtr(o.mPtr) { if (mPtr) mPtr->AddRef(); }
    Ref(Ref&& o) noexcept : mPtr(o.mPtr) { o.mPtr = nullptr; }
    /// Converting constructor: Ref<Derived> -> Ref<Base>.
    template <typename U, typename = std::enable_if_t<std::is_convertible_v<U*, T*>>>
    Ref(const Ref<U>& o) : mPtr(o.Get()) { if (mPtr) mPtr->AddRef(); }
    ~Ref() { if (mPtr) mPtr->Release(); }

    Ref& operator=(const Ref& o) {
        if (this != &o) { if (o.mPtr) o.mPtr->AddRef(); if (mPtr) mPtr->Release(); mPtr = o.mPtr; }
        return *this;
    }
    template <typename U, typename = std::enable_if_t<std::is_convertible_v<U*, T*>>>
    Ref& operator=(const Ref<U>& o) {
        T* p = o.Get();
        if (p) p->AddRef();
        if (mPtr) mPtr->Release();
        mPtr = p;
        return *this;
    }
    Ref& operator=(Ref&& o) noexcept {
        if (this != &o) { if (mPtr) mPtr->Release(); mPtr = o.mPtr; o.mPtr = nullptr; }
        return *this;
    }
    Ref& operator=(T* ptr) {
        if (ptr) ptr->AddRef();
        if (mPtr) mPtr->Release();
        mPtr = ptr;
        return *this;
    }
    bool operator==(std::nullptr_t) const noexcept { return mPtr == nullptr; }
    bool operator!=(std::nullptr_t) const noexcept { return mPtr != nullptr; }

    T* operator->() const { KZ_ASSERT(mPtr); return mPtr; }
    T& operator*() const  { KZ_ASSERT(mPtr); return *mPtr; }
    T* Get() const { return mPtr; }
    explicit operator bool() const { return mPtr != nullptr; }
    void Reset() { if (mPtr) mPtr->Release(); mPtr = nullptr; }

private:
    T* mPtr = nullptr;
};

template <typename T, typename... Args>
Ref<T> MakeRef(Args&&... args) {
    return Ref<T>(new T(std::forward<Args>(args)...));
}

// ---------------------------------------------------------------------------
// ArrayView - non-owning view over a contiguous range.
// ---------------------------------------------------------------------------
template <typename T>
class ArrayView {
public:
    ArrayView() = default;
    ArrayView(T* data, usize size) : mData(data), mSize(size) {}
    template <usize N>
    ArrayView(T (&arr)[N]) : mData(arr), mSize(N) {}

    T* begin() const { return mData; }
    T* end() const { return mData + mSize; }
    T& operator[](usize i) const { KZ_ASSERT(i < mSize); return mData[i]; }
    T* Data() const { return mData; }
    usize Size() const { return mSize; }
    bool Empty() const { return mSize == 0; }

private:
    T* mData = nullptr;
    usize mSize = 0;
};

} // namespace kizuri
