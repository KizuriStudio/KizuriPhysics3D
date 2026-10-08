// KizuriPhysics - BroadPhase/BroadPhase.cpp
#include "Kizuri/BroadPhase/BroadPhase.h"

namespace kizuri {

// ---------------------------------------------------------------------------
// DynamicAABBTree
// ---------------------------------------------------------------------------
DynamicAABBTree::DynamicAABBTree() {
    mNodes.Reserve(1024);
}

u32 DynamicAABBTree::AllocateNode() {
    if (mFreeList != kNullNode) {
        u32 id = mFreeList;
        mFreeList = mNodes[id].next;
        Node& n = mNodes[id];
        n = Node();
        n.next = kNullNode;
        return id;
    }
    mNodes.PushBack(Node());
    ++mNodeCount;
    return u32(mNodes.Size()) - 1;
}

void DynamicAABBTree::FreeNode(u32 nodeId) {
    Node& n = mNodes[nodeId];
    n = Node();
    n.next = mFreeList;
    n.height = -1;
    mFreeList = nodeId;
}

u32 DynamicAABBTree::CreateProxy(const AABB& aabb, u32 userData) {
    u32 id = AllocateNode();
    // Fat AABB: expanded margin for stability.
    Vec3 margin(Real(0.1));
    mNodes[id].aabb = AABB(aabb.min - margin, aabb.max + margin);
    mNodes[id].tightAABB = aabb;
    mNodes[id].userData = userData;
    mNodes[id].height = 0;
    mNodes[id].child1 = kNullNode;
    mNodes[id].child2 = kNullNode;
    mNodes[id].parent = kNullNode;
    InsertLeaf(id);
    ++mNumProxies;
    return id;
}

void DynamicAABBTree::DestroyProxy(u32 proxyId) {
    KZ_ASSERT(proxyId < mNodes.Size() && mNodes[proxyId].IsLeaf());
    RemoveLeaf(proxyId);
    FreeNode(proxyId);
    --mNumProxies;
}

bool DynamicAABBTree::MoveProxy(u32 proxyId, const AABB& aabb, const Vec3& displacement) {
    Node& leaf = mNodes[proxyId];
    leaf.tightAABB = aabb;

    // If the new tight AABB is still inside the fat AABB, no tree update needed.
    if (leaf.aabb.Contains(aabb)) return false;

    RemoveLeaf(proxyId);

    // New fat AABB: expand by the displacement plus a margin.
    Vec3 margin(Real(0.1));
    Vec3 d = displacement.Abs();
    leaf.aabb = AABB(aabb.min - margin - d, aabb.max + margin + d);

    InsertLeaf(proxyId);
    ++mInsertionCount;
    return true;
}

void DynamicAABBTree::SetFatAABB(u32 proxyId, const AABB& aabb) {
    mNodes[proxyId].aabb = aabb;
}

void DynamicAABBTree::InsertLeaf(u32 leaf) {
    if (mRoot == kNullNode) {
        mRoot = leaf;
        mNodes[leaf].parent = kNullNode;
        return;
    }

    // Find the best sibling using the surface area heuristic.
    AABB leafAABB = mNodes[leaf].aabb;
    u32 index = mRoot;
    while (!mNodes[index].IsLeaf()) {
        u32 child1 = mNodes[index].child1;
        u32 child2 = mNodes[index].child2;

        Real area = mNodes[index].aabb.SurfaceArea();
        AABB combined = Merge(mNodes[index].aabb, leafAABB);
        Real combinedArea = combined.SurfaceArea();

        // Cost of creating a new parent for this node and the leaf.
        Real cost = Real(2) * combinedArea;
        Real inheritanceCost = Real(2) * (combinedArea - area);

        auto childCost = [&](u32 child) -> Real {
            AABB merged = Merge(leafAABB, mNodes[child].aabb);
            if (mNodes[child].IsLeaf()) return merged.SurfaceArea() + inheritanceCost;
            return (merged.SurfaceArea() - mNodes[child].aabb.SurfaceArea()) + inheritanceCost;
        };

        Real cost1 = childCost(child1);
        Real cost2 = childCost(child2);

        if (cost < cost1 && cost < cost2) break;
        index = (cost1 < cost2) ? child1 : child2;
    }

    u32 sibling = index;

    // Create a new parent.
    u32 oldParent = mNodes[sibling].parent;
    u32 newParent = AllocateNode();
    mNodes[newParent].parent = oldParent;
    mNodes[newParent].aabb = Merge(leafAABB, mNodes[sibling].aabb);
    mNodes[newParent].tightAABB = mNodes[newParent].aabb;
    mNodes[newParent].height = mNodes[sibling].height + 1;
    mNodes[newParent].userData = 0;

    if (oldParent != kNullNode) {
        if (mNodes[oldParent].child1 == sibling) mNodes[oldParent].child1 = newParent;
        else mNodes[oldParent].child2 = newParent;
    } else {
        mRoot = newParent;
    }

    mNodes[newParent].child1 = sibling;
    mNodes[newParent].child2 = leaf;
    mNodes[sibling].parent = newParent;
    mNodes[leaf].parent = newParent;

    // Walk up refitting AABBs and balancing.
    index = mNodes[leaf].parent;
    while (index != kNullNode) {
        index = Balance(index);
        u32 c1 = mNodes[index].child1;
        u32 c2 = mNodes[index].child2;
        mNodes[index].height = 1 + math::Max(mNodes[c1].height, mNodes[c2].height);
        mNodes[index].aabb = Merge(mNodes[c1].aabb, mNodes[c2].aabb);
        index = mNodes[index].parent;
    }
}

void DynamicAABBTree::RemoveLeaf(u32 leaf) {
    if (leaf == mRoot) {
        mRoot = kNullNode;
        return;
    }
    u32 parent = mNodes[leaf].parent;
    u32 grandParent = mNodes[parent].parent;
    u32 sibling = (mNodes[parent].child1 == leaf) ? mNodes[parent].child2 : mNodes[parent].child1;

    if (grandParent != kNullNode) {
        if (mNodes[grandParent].child1 == parent) mNodes[grandParent].child1 = sibling;
        else mNodes[grandParent].child2 = sibling;
        mNodes[sibling].parent = grandParent;
        FreeNode(parent);

        u32 index = grandParent;
        while (index != kNullNode) {
            index = Balance(index);
            u32 c1 = mNodes[index].child1;
            u32 c2 = mNodes[index].child2;
            mNodes[index].aabb = Merge(mNodes[c1].aabb, mNodes[c2].aabb);
            mNodes[index].height = 1 + math::Max(mNodes[c1].height, mNodes[c2].height);
            index = mNodes[index].parent;
        }
    } else {
        mRoot = sibling;
        mNodes[sibling].parent = kNullNode;
        FreeNode(parent);
    }
}

u32 DynamicAABBTree::Balance(u32 iA) {
    Node& A = mNodes[iA];
    if (A.IsLeaf() || A.height < 2) return iA;

    u32 iB = A.child1;
    u32 iC = A.child2;
    Node& B = mNodes[iB];
    Node& C = mNodes[iC];

    i32 balance = C.height - B.height;

    // Rotate C up.
    if (balance > 1) {
        u32 iF = C.child1;
        u32 iG = C.child2;
        Node& F = mNodes[iF];
        Node& G = mNodes[iG];

        C.child1 = iA;
        C.parent = A.parent;
        A.parent = iC;

        if (C.parent != kNullNode) {
            if (mNodes[C.parent].child1 == iA) mNodes[C.parent].child1 = iC;
            else mNodes[C.parent].child2 = iC;
        } else {
            mRoot = iC;
        }

        if (F.height > G.height) {
            C.child2 = iF;
            A.child2 = iG;
            G.parent = iA;
            A.aabb = Merge(B.aabb, G.aabb);
            C.aabb = Merge(A.aabb, F.aabb);
            A.height = 1 + math::Max(B.height, G.height);
            C.height = 1 + math::Max(A.height, F.height);
        } else {
            C.child2 = iG;
            A.child2 = iF;
            F.parent = iA;
            A.aabb = Merge(B.aabb, F.aabb);
            C.aabb = Merge(A.aabb, G.aabb);
            A.height = 1 + math::Max(B.height, F.height);
            C.height = 1 + math::Max(A.height, G.height);
        }
        return iC;
    }

    // Rotate B up.
    if (balance < -1) {
        u32 iD = B.child1;
        u32 iE = B.child2;
        Node& D = mNodes[iD];
        Node& E = mNodes[iE];

        B.child1 = iA;
        B.parent = A.parent;
        A.parent = iB;

        if (B.parent != kNullNode) {
            if (mNodes[B.parent].child1 == iA) mNodes[B.parent].child1 = iB;
            else mNodes[B.parent].child2 = iB;
        } else {
            mRoot = iB;
        }

        if (D.height > E.height) {
            B.child2 = iD;
            A.child1 = iE;
            E.parent = iA;
            A.aabb = Merge(C.aabb, E.aabb);
            B.aabb = Merge(A.aabb, D.aabb);
            A.height = 1 + math::Max(C.height, E.height);
            B.height = 1 + math::Max(A.height, D.height);
        } else {
            B.child2 = iE;
            A.child1 = iD;
            D.parent = iA;
            A.aabb = Merge(C.aabb, D.aabb);
            B.aabb = Merge(A.aabb, E.aabb);
            A.height = 1 + math::Max(C.height, D.height);
            B.height = 1 + math::Max(A.height, E.height);
        }
        return iB;
    }

    return iA;
}

void DynamicAABBTree::UpdateHeights(u32 index) {
    while (index != kNullNode) {
        u32 c1 = mNodes[index].child1;
        u32 c2 = mNodes[index].child2;
        if (c1 != kNullNode) {
            mNodes[index].aabb = Merge(mNodes[c1].aabb, mNodes[c2].aabb);
            mNodes[index].height = 1 + math::Max(mNodes[c1].height, mNodes[c2].height);
        }
        index = mNodes[index].parent;
    }
}

u32 DynamicAABBTree::Height() const {
    if (mRoot == kNullNode) return 0;
    return u32(mNodes[mRoot].height);
}

Real DynamicAABBTree::GetAreaRatio() const {
    if (mRoot == kNullNode) return 0;
    Real rootArea = mNodes[mRoot].aabb.SurfaceArea();
    Real totalArea = 0;
    for (u32 i = 0; i < mNodes.Size(); ++i) {
        if (mNodes[i].height < 0) continue; // free
        totalArea += mNodes[i].aabb.SurfaceArea();
    }
    return rootArea > Real(0) ? totalArea / rootArea : 0;
}

void DynamicAABBTree::Rebuild() {
    // Collect all leaf proxies.
    Vector<u32> leaves;
    for (u32 i = 0; i < mNodes.Size(); ++i) {
        if (mNodes[i].height == 0) leaves.PushBack(i);
    }
    mRoot = kNullNode;
    for (u32 i = 0; i < mNodes.Size(); ++i) {
        if (mNodes[i].height >= 0) {
            mNodes[i].parent = kNullNode;
            mNodes[i].child1 = kNullNode;
            mNodes[i].child2 = kNullNode;
            mNodes[i].height = 0;
        }
    }
    for (u32 leaf : leaves) InsertLeaf(leaf);
}

bool DynamicAABBTree::RayAABB(const Ray& ray, const AABB& aabb, Real& tMin, Real& tMax) {
    Real t0 = tMin, t1 = tMax;
    for (int i = 0; i < 3; ++i) {
        Real d = ray.direction[i];
        if (math::Abs(d) < math::kEpsilon) {
            if (ray.origin[i] < aabb.min[i] || ray.origin[i] > aabb.max[i]) return false;
        } else {
            Real inv = Real(1) / d;
            Real ta = (aabb.min[i] - ray.origin[i]) * inv;
            Real tb = (aabb.max[i] - ray.origin[i]) * inv;
            if (ta > tb) { Real tmp = ta; ta = tb; tb = tmp; }
            t0 = math::Max(t0, ta);
            t1 = math::Min(t1, tb);
            if (t0 > t1) return false;
        }
    }
    tMin = t0;
    tMax = t1;
    return true;
}

// ---------------------------------------------------------------------------
// BroadPhase
// ---------------------------------------------------------------------------
BroadPhase::BroadPhase() = default;

BroadPhaseProxyId BroadPhase::CreateProxy(const AABB& aabb, u32 userData, u32 objectLayer, u32 collisionMask) {
    BroadPhaseProxyId id = mTree.CreateProxy(aabb, userData);
    if (id >= mLayers.Size()) {
        while (mLayers.Size() <= id) { mLayers.PushBack(0); mMasks.PushBack(0); }
    }
    mLayers[id] = objectLayer;
    mMasks[id] = collisionMask;
    return id;
}

void BroadPhase::DestroyProxy(BroadPhaseProxyId id) {
    mTree.DestroyProxy(id);
}

bool BroadPhase::MoveProxy(BroadPhaseProxyId id, const AABB& aabb, const Vec3& displacement) {
    return mTree.MoveProxy(id, aabb, displacement);
}

void BroadPhase::SetLayer(BroadPhaseProxyId id, u32 objectLayer, u32 collisionMask) {
    if (id >= mLayers.Size()) return;
    mLayers[id] = objectLayer;
    mMasks[id] = collisionMask;
}

void BroadPhase::UpdatePairs() {
    mPairs.Clear();
    mTree.QueryPairs([&](u32 n1, u32 n2) {
        // Layer filtering: both must agree to collide.
        if ((mLayers[n1] & mMasks[n2]) == 0) return;
        if ((mLayers[n2] & mMasks[n1]) == 0) return;
        // Also require the tight AABBs to actually overlap.
        if (!mTree.GetTightAABB(n1).Overlaps(mTree.GetTightAABB(n2))) return;
        BroadPhasePair p;
        p.proxyA = n1;
        p.proxyB = n2;
        mPairs.PushBack(p);
    });
}

} // namespace kizuri
