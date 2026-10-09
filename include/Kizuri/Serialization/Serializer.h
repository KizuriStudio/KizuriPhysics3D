// KizuriPhysics - Serialization/Serializer.h
// Binary save/load of a PhysicsWorld (bodies + shapes).
#pragma once

#include "Kizuri/World/PhysicsWorld.h"

#include <cstddef>

namespace kizuri {

// ---------------------------------------------------------------------------
// Little-endian byte stream helpers.
// ---------------------------------------------------------------------------
class BinaryWriter {
public:
    void WriteBytes(const void* data, size_t size) {
        const u8* p = static_cast<const u8*>(data);
        for (size_t i = 0; i < size; ++i) mBuffer.PushBack(p[i]);
    }
    template <typename T>
    void Write(const T& value) { WriteBytes(&value, sizeof(T)); }

    const Vector<u8>& GetBuffer() const { return mBuffer; }
    size_t Size() const { return mBuffer.Size(); }

private:
    Vector<u8> mBuffer;
};

class BinaryReader {
public:
    BinaryReader(const u8* data, size_t size) : mData(data), mSize(size) {}

    bool ReadBytes(void* out, size_t size) {
        if (mOffset + size > mSize) return false;
        u8* p = static_cast<u8*>(out);
        for (size_t i = 0; i < size; ++i) p[i] = mData[mOffset + i];
        mOffset += size;
        return true;
    }
    template <typename T>
    bool Read(T& value) { return ReadBytes(&value, sizeof(T)); }

    bool Ok() const { return mOffset <= mSize; }
    size_t Remaining() const { return mSize - mOffset; }

private:
    const u8* mData = nullptr;
    size_t mSize = 0;
    size_t mOffset = 0;
};

// ---------------------------------------------------------------------------
// World serialization.
// ---------------------------------------------------------------------------
/// Serialize the bodies (and their shapes) of a world into `out`.
bool SerializeWorld(const PhysicsWorld& world, Vector<u8>& out);

/// Restore a world from a buffer produced by SerializeWorld. The world is
/// cleared of its current bodies first. Returns false on a malformed buffer.
bool DeserializeWorld(PhysicsWorld& world, const u8* data, size_t size);

/// File convenience wrappers.
bool SaveWorldToFile(const PhysicsWorld& world, const char* path);
bool LoadWorldFromFile(PhysicsWorld& world, const char* path);

} // namespace kizuri
