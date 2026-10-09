// KizuriPhysics - Serialization/Serializer.cpp
#include "Kizuri/Serialization/Serializer.h"

#include <cstdint>
#include <cstdio>

namespace kizuri {

namespace {

constexpr u32 kMagic = 0x31505A4Bu; // 'KZP1'
constexpr u32 kVersion = 1;

void WriteVec3(BinaryWriter& w, const Vec3& v) {
    w.Write(v.x);
    w.Write(v.y);
    w.Write(v.z);
}
bool ReadVec3(BinaryReader& r, Vec3& v) {
    return r.Read(v.x) && r.Read(v.y) && r.Read(v.z);
}

void WriteQuat(BinaryWriter& w, const Quat& q) {
    w.Write(q.x);
    w.Write(q.y);
    w.Write(q.z);
    w.Write(q.w);
}
bool ReadQuat(BinaryReader& r, Quat& q) {
    return r.Read(q.x) && r.Read(q.y) && r.Read(q.z) && r.Read(q.w);
}

bool WriteShape(BinaryWriter& w, const Shape& shape) {
    const ShapeType type = shape.GetType();
    w.Write<u8>(u8(type));
    switch (type) {
        case ShapeType::Sphere: {
            w.Write(static_cast<const SphereShape&>(shape).GetRadius());
            break;
        }
        case ShapeType::Box: {
            WriteVec3(w, static_cast<const BoxShape&>(shape).GetHalfExtent());
            break;
        }
        case ShapeType::Capsule: {
            const CapsuleShape& c = static_cast<const CapsuleShape&>(shape);
            w.Write(c.GetHalfHeight());
            w.Write(c.GetRadius());
            break;
        }
        case ShapeType::Cylinder: {
            const CylinderShape& c = static_cast<const CylinderShape&>(shape);
            w.Write(c.GetHalfHeight());
            w.Write(c.GetRadius());
            break;
        }
        case ShapeType::Plane: {
            const Plane& p = static_cast<const PlaneShape&>(shape).GetPlane();
            WriteVec3(w, p.normal);
            w.Write(p.distance);
            break;
        }
        case ShapeType::ConvexHull: {
            const ArrayView<const Vec3> points = static_cast<const ConvexHullShape&>(shape).GetPoints();
            w.Write<u32>(u32(points.Size()));
            for (u32 i = 0; i < u32(points.Size()); ++i) WriteVec3(w, points[i]);
            break;
        }
        default:
            // Mesh / Compound / HeightField are not supported yet.
            return false;
    }
    return true;
}

Ref<Shape> ReadShape(BinaryReader& r) {
    u8 rawType = 0;
    if (!r.Read(rawType)) return nullptr;
    const ShapeType type = ShapeType(rawType);
    switch (type) {
        case ShapeType::Sphere: {
            Real radius = 0;
            if (!r.Read(radius)) return nullptr;
            return MakeRef<SphereShape>(radius);
        }
        case ShapeType::Box: {
            Vec3 half;
            if (!ReadVec3(r, half)) return nullptr;
            return MakeRef<BoxShape>(half);
        }
        case ShapeType::Capsule: {
            Real halfHeight = 0, radius = 0;
            if (!r.Read(halfHeight) || !r.Read(radius)) return nullptr;
            return MakeRef<CapsuleShape>(halfHeight, radius);
        }
        case ShapeType::Cylinder: {
            Real halfHeight = 0, radius = 0;
            if (!r.Read(halfHeight) || !r.Read(radius)) return nullptr;
            return MakeRef<CylinderShape>(halfHeight, radius);
        }
        case ShapeType::Plane: {
            Vec3 normal;
            Real distance = 0;
            if (!ReadVec3(r, normal) || !r.Read(distance)) return nullptr;
            return MakeRef<PlaneShape>(Plane(normal, distance));
        }
        case ShapeType::ConvexHull: {
            u32 count = 0;
            if (!r.Read(count)) return nullptr;
            if (count > 100000) return nullptr;
            Vector<Vec3> points;
            points.Resize(count);
            for (u32 i = 0; i < count; ++i) {
                if (!ReadVec3(r, points[i])) return nullptr;
            }
            return ConvexHullShape::Create(ArrayView<const Vec3>(points.Data(), points.Size()));
        }
        default:
            return nullptr;
    }
}

void WriteBodySettings(BinaryWriter& w, const BodySettings& s) {
    WriteVec3(w, s.position);
    WriteQuat(w, s.rotation);
    WriteVec3(w, s.linearVelocity);
    WriteVec3(w, s.angularVelocity);
    w.Write<u8>(u8(s.motionType));
    w.Write<u8>(u8(s.motionQuality));
    w.Write(s.ccdMotionThreshold);
    w.Write(s.friction);
    w.Write(s.restitution);
    w.Write(s.linearDamping);
    w.Write(s.angularDamping);
    w.Write(s.gravityFactor);
    w.Write(s.maxLinearVelocity);
    w.Write(s.maxAngularVelocity);
    w.Write<u32>(s.objectLayer);
    w.Write<u32>(s.collisionMask);
    w.Write<u8>(s.allowSleeping ? 1 : 0);
    w.Write<u8>(s.isSensor ? 1 : 0);
    w.Write<u64>(u64(reinterpret_cast<uintptr_t>(s.userData)));
    w.Write(s.massOverride);
    w.Write(s.density);
}

bool ReadBodySettings(BinaryReader& r, BodySettings& s) {
    u8 motionType = 0, motionQuality = 0, allowSleeping = 1, isSensor = 0;
    if (!ReadVec3(r, s.position)) return false;
    if (!ReadQuat(r, s.rotation)) return false;
    if (!ReadVec3(r, s.linearVelocity)) return false;
    if (!ReadVec3(r, s.angularVelocity)) return false;
    if (!r.Read(motionType)) return false;
    if (!r.Read(motionQuality)) return false;
    if (!r.Read(s.ccdMotionThreshold)) return false;
    if (!r.Read(s.friction)) return false;
    if (!r.Read(s.restitution)) return false;
    if (!r.Read(s.linearDamping)) return false;
    if (!r.Read(s.angularDamping)) return false;
    if (!r.Read(s.gravityFactor)) return false;
    if (!r.Read(s.maxLinearVelocity)) return false;
    if (!r.Read(s.maxAngularVelocity)) return false;
    if (!r.Read(s.objectLayer)) return false;
    if (!r.Read(s.collisionMask)) return false;
    if (!r.Read(allowSleeping)) return false;
    if (!r.Read(isSensor)) return false;
    {
        u64 userData = 0;
        if (!r.Read(userData)) return false;
        s.userData = reinterpret_cast<void*>(uintptr_t(userData));
    }
    if (!r.Read(s.massOverride)) return false;
    if (!r.Read(s.density)) return false;
    s.motionType = MotionType(motionType);
    s.motionQuality = MotionQuality(motionQuality);
    s.allowSleeping = allowSleeping != 0;
    s.isSensor = isSensor != 0;
    return true;
}

} // namespace

bool SerializeWorld(const PhysicsWorld& world, Vector<u8>& out) {
    BinaryWriter writer;
    writer.Write<u32>(kMagic);
    writer.Write<u32>(kVersion);

    const BodyManager& bodies = world.GetBodyManager();
    const u32 slots = bodies.GetBodySlotCount();

    // Count serialisable bodies first.
    u32 count = 0;
    for (u32 i = 0; i < slots; ++i) {
        const Body* b = bodies.GetBodyBySlot(i);
        if (b && b->GetShape()) ++count;
    }
    writer.Write<u32>(count);

    for (u32 i = 0; i < slots; ++i) {
        const Body* b = bodies.GetBodyBySlot(i);
        if (!b || !b->GetShape()) continue;
        if (!WriteShape(writer, *b->GetShape())) return false;
        WriteBodySettings(writer, b->CaptureSettings());
        writer.Write<u8>(b->IsActive() ? 1 : 0);
    }

    out = writer.GetBuffer();
    return true;
}

bool DeserializeWorld(PhysicsWorld& world, const u8* data, size_t size) {
    BinaryReader reader(data, size);
    u32 magic = 0, version = 0;
    if (!reader.Read(magic) || magic != kMagic) return false;
    if (!reader.Read(version) || version != kVersion) return false;

    u32 count = 0;
    if (!reader.Read(count)) return false;

    world.Clear();

    for (u32 i = 0; i < count; ++i) {
        Ref<Shape> shape = ReadShape(reader);
        if (!shape) return false;

        BodySettings settings;
        if (!ReadBodySettings(reader, settings)) return false;
        settings.shape = shape;

        u8 active = 1;
        if (!reader.Read(active)) return false;

        const BodyID id = world.CreateBody(settings);
        if (active == 0) world.SetBodyActive(id, false);
    }
    return reader.Ok();
}

bool SaveWorldToFile(const PhysicsWorld& world, const char* path) {
    Vector<u8> buffer;
    if (!SerializeWorld(world, buffer)) return false;
    std::FILE* f = std::fopen(path, "wb");
    if (!f) return false;
    const size_t written = buffer.Size() > 0
        ? std::fwrite(buffer.Data(), 1, buffer.Size(), f) : 0;
    std::fclose(f);
    return written == buffer.Size();
}

bool LoadWorldFromFile(PhysicsWorld& world, const char* path) {
    std::FILE* f = std::fopen(path, "rb");
    if (!f) return false;
    std::fseek(f, 0, SEEK_END);
    const long length = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    if (length <= 0) {
        std::fclose(f);
        return false;
    }
    Vector<u8> buffer;
    buffer.Resize(size_t(length));
    const size_t read = std::fread(buffer.Data(), 1, size_t(length), f);
    std::fclose(f);
    if (read != size_t(length)) return false;
    return DeserializeWorld(world, buffer.Data(), buffer.Size());
}

} // namespace kizuri
