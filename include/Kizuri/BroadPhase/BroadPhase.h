// KizuriPhysics - BroadPhase/BroadPhase.h
// Dynamic AABB tree broad phase with fat bounds and layer filtering.
#pragma once

#include "Kizuri/Core/Memory.h"
#include "Kizuri/Core/Math.h"

namespace kizuri {

using BroadPhaseProxyId = u32;
inline constexpr BroadPhaseProxyId kInvalidProxy = 0xFFFFFFFFu;

struct BroadPhasePair {
    BroadPhaseProxyId proxyA = kInvalidProxy;
    BroadPhaseProxyId proxyB = kInvalidProxy;
};

// ---------------------------------------------------------------------------
// Dynamic AABB tree (surface area heuristic insertion, Box2D style).
// ---------------------------------------------------------------------------
class DynamicAABBTree {
public:
    static constexpr u32 kNullNode = 0xFFFFFFFFu;

    DynamicAABBTree();

    u32 CreateProxy(const AABB& aabb, u32 userData);
    void DestroyProxy(u32 proxyId);
    /// Move a proxy. Returns true when the new AABB escapes the fat bounds and
    /// the tree was updated.
    bool MoveProxy(u32 proxyId, const AABB& aabb, const Vec3& displacement);
    void SetFatAABB(u32 proxyId, const AABB& aabb);

    const AABB& GetFatAABB(u32 proxyId) const { return mNodes[proxyId].aabb; }
    const AABB& GetTightAABB(u32 proxyId) const { return mNodes[proxyId].tightAABB; }
    u32 GetUserData(u32 proxyId) const { return mNodes[proxyId].userData; }
    void SetUserData(u32 proxyId, u32 data) { mNodes[proxyId].userData = data; }

    template <typename Fn>
    void Query(const AABB& aabb, Fn&& callback) const {
        u32 stack[128];
        u32 stackCount = 0;
        stack[stackCount++] = mRoot;
        while (stackCount > 0) {
            u32 nodeId = stack[--stackCount];
            if (nodeId == kNullNode) continue;
            const Node& node = mNodes[nodeId];
            if (!node.aabb.Overlaps(aabb)) continue;
            if (node.IsLeaf()) {
                callback(nodeId);
            } else {
                if (stackCount + 2 <= 128) {
                    stack[stackCount++] = node.child1;
                    stack[stackCount++] = node.child2;
                }
            }
        }
    }

    /// Enumerate all overlapping leaf pairs (each pair once).
    template <typename Fn>
    void QueryPairs(Fn&& callback) const {
        if (mRoot == kNullNode) return;
        struct Entry { u32 n1, n2; };
        Vector<Entry> stack;
        stack.Reserve(1024);

        auto push = [&](u32 a, u32 b) { stack.PushBack(Entry{a, b}); };

        push(mRoot, mRoot);
        while (stack.Size() > 0) {
            Entry e = stack[stack.Size() - 1];
            stack.PopBack();
            u32 n1 = e.n1, n2 = e.n2;
            const Node& a = mNodes[n1];

            if (n1 == n2) {
                // Self pair: descend into the node's children.
                if (a.IsLeaf()) continue;
                push(a.child1, a.child1);
                push(a.child2, a.child2);
                push(a.child1, a.child2);
                continue;
            }

            const Node& b = mNodes[n2];
            // Prune node pairs whose fat bounds do not overlap.
            if (!a.aabb.Overlaps(b.aabb)) continue;

            if (a.IsLeaf() && b.IsLeaf()) {
                if (n1 < n2) callback(n1, n2);
                continue;
            }

            // Split the larger node to keep the traversal balanced.
            if (a.IsLeaf() || (!b.IsLeaf() && a.height < b.height)) {
                push(n1, b.child1);
                push(n1, b.child2);
            } else {
                push(a.child1, n2);
                push(a.child2, n2);
            }
        }
    }

    /// Cast a ray through the tree. Callback(proxyId, fraction) may return a
    /// reduced max fraction. Returns the hit fraction.
    template <typename Fn>
    Real RayCast(const Ray& ray, Real maxFraction, Fn&& callback) const {
        if (mRoot == kNullNode) return maxFraction;
        Real tMin = Real(0), tMax = maxFraction;
        if (!RayAABB(ray, mNodes[mRoot].aabb, tMin, tMax)) return maxFraction;

        u32 stack[128];
        Real fracStack[128];
        u32 count = 0;
        stack[count] = mRoot;
        fracStack[count] = tMin;
        ++count;
        Real best = maxFraction;
        while (count > 0) {
            --count;
            u32 nodeId = stack[count];
            Real t = fracStack[count];
            if (t > best) continue;
            const Node& node = mNodes[nodeId];
            if (node.IsLeaf()) {
                Real f = callback(nodeId, best);
                if (f < best) best = f;
            } else {
                for (u32 ci = 0; ci < 2; ++ci) {
                    u32 child = ci == 0 ? node.child1 : node.child2;
                    Real a = Real(0), b = best;
                    if (RayAABB(ray, mNodes[child].aabb, a, b) && count < 128) {
                        stack[count] = child;
                        fracStack[count] = a;
                        ++count;
                    }
                }
            }
        }
        return best;
    }

    u32 NumProxies() const { return mNumProxies; }
    u32 NumNodes() const { return mNodeCount; }
    u32 Height() const;
    Real GetAreaRatio() const;

    /// Rebuild the tree from scratch for optimal quality (call occasionally).
    void Rebuild();

private:
    struct Node {
        AABB aabb;       // fat bounds
        AABB tightAABB;  // tight bounds
        u32 parent = kNullNode;
        u32 child1 = kNullNode;
        u32 child2 = kNullNode;
        u32 userData = 0;
        u32 next = kNullNode;
        i32 height = -1; // -1 for free nodes

        bool IsLeaf() const { return child1 == kNullNode; }
    };

    u32 AllocateNode();
    void FreeNode(u32 nodeId);
    void InsertLeaf(u32 leaf);
    void RemoveLeaf(u32 leaf);
    u32 Balance(u32 indexA);
    void UpdateHeights(u32 index);

    static bool RayAABB(const Ray& ray, const AABB& aabb, Real& tMin, Real& tMax);

    Vector<Node> mNodes;
    u32 mRoot = kNullNode;
    u32 mNodeCount = 0;   // high-water mark
    u32 mFreeList = kNullNode;
    u32 mNumProxies = 0;
    u32 mInsertionCount = 0;
};

// ---------------------------------------------------------------------------
// Broad phase: AABB tree + object layer filtering + pair list.
// ---------------------------------------------------------------------------
class BroadPhase {
public:
    BroadPhase();

    BroadPhaseProxyId CreateProxy(const AABB& aabb, u32 userData, u32 objectLayer, u32 collisionMask);
    void DestroyProxy(BroadPhaseProxyId id);
    bool MoveProxy(BroadPhaseProxyId id, const AABB& aabb, const Vec3& displacement);
    void SetLayer(BroadPhaseProxyId id, u32 objectLayer, u32 collisionMask);

    const AABB& GetAABB(BroadPhaseProxyId id) const { return mTree.GetTightAABB(id); }
    u32 GetUserData(BroadPhaseProxyId id) const { return mTree.GetUserData(id); }
    u32 GetObjectLayer(BroadPhaseProxyId id) const { return mLayers[id]; }
    u32 GetCollisionMask(BroadPhaseProxyId id) const { return mMasks[id]; }

    /// Recompute the overlapping pair list (layer filtered, deduplicated).
    void UpdatePairs();

    ArrayView<const BroadPhasePair> GetPairs() const { return { mPairs.Data(), mPairs.Size() }; }
    u32 NumProxies() const { return mTree.NumProxies(); }
    const DynamicAABBTree& GetTree() const { return mTree; }

    template <typename Fn>
    void Query(const AABB& aabb, Fn&& callback) const { mTree.Query(aabb, callback); }

    template <typename Fn>
    Real RayCast(const Ray& ray, Real maxFraction, Fn&& callback) const {
        return mTree.RayCast(ray, maxFraction, callback);
    }

private:
    DynamicAABBTree mTree;
    Vector<u32> mLayers;   // indexed by proxy id
    Vector<u32> mMasks;
    Vector<BroadPhasePair> mPairs;
};

} // namespace kizuri
