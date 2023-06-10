
inline bool IntersectP000BOX(const Bounds3f& bounds, const Ray &ray, const Vector3f &invDir
                                   #ifdef CACHE_INVDIR_BY_GAMMA
                                   , const Vector3f &invDirByGamma
                                   #endif
                                   )  {
    #ifndef CACHE_INVDIR_BY_GAMMA
    const Vector3f &invDirByGamma = invDir;
    #endif
    // This will also be removed in optimization
    
    // Check for ray intersection against $x$ and $y$ slabs
    Float tMin = (bounds.pMin.x - ray.o.x) * invDir.x;
    Float tMax = (bounds.pMax.x - ray.o.x) * invDirByGamma.x;
    Float tyMin = (bounds.pMin.y - ray.o.y) * invDir.y;
    Float tyMax = (bounds.pMax.y - ray.o.y) * invDirByGamma.y;

    #ifndef CACHE_INVDIR_BY_GAMMA
    // Update _tMax_ and _tyMax_ to ensure robust bounds intersection
    tMax *= 1 + 2 * gamma(3);
    tyMax *= 1 + 2 * gamma(3);
    #endif
    if (tMin > tyMax || tyMin > tMax) return false;
    if (tyMin > tMin) tMin = tyMin;
    if (tyMax < tMax) tMax = tyMax;

    // Check for ray intersection against $z$ slab
    Float tzMin = (bounds.pMin.z - ray.o.z) * invDir.z;
    Float tzMax = (bounds.pMax.z - ray.o.z) * invDirByGamma.z;
    
    #ifndef CACHE_INVDIR_BY_GAMMA
    // Update _tzMax_ to ensure robust bounds intersection
    tzMax *= 1 + 2 * gamma(3);
    #endif
    if (tMin > tzMax || tzMin > tMax) return false;
    if (tzMin > tMin) tMin = tzMin;
    if (tzMax < tMax) tMax = tzMax;
    return (tMin < ray.tMax) && (tMax > 0);
}
inline bool IntersectP000 (const std::vector<std::shared_ptr<Primitive>> &primitives, const LinearBVHNode *nodes, const Ray &ray){
    if (!nodes) return false;
    #ifdef PBRT_8_CLASSES_DEBUG
    CHECK_EQ((ray.d.x < 0), 0);
    CHECK_EQ((ray.d.y < 0), 0);
    CHECK_EQ((ray.d.z < 0), 0);
    #endif
    Vector3f invDir(1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z);
    #ifdef CACHE_INVDIR_BY_GAMMA
    Vector3f invDirByGamma = invDir * (1 + 2 * gamma(3));
    #endif
    //int dirIsNeg[3] = {invDir.x < 0, invDir.y < 0, invDir.z < 0};
    int nodesToVisit[64];
    int toVisitOffset = 0, currentNodeIndex = 0;
    while (true) {
        const LinearBVHNode *node = &nodes[currentNodeIndex];
        if (
            #ifndef CACHE_INVDIR_BY_GAMMA
            IntersectP000BOX(node->bounds, ray, invDir)
            #else
            IntersectP000BOX(node->bounds, ray, invDir, invDirByGamma)
            #endif
            ) {
            // Process BVH node _node_ for traversal
            if (node->nPrimitives > 0) {
                for (int i = 0; i < node->nPrimitives; ++i) {
                    if (primitives[node->primitivesOffset + i]->IntersectP(
                            ray)) {
                        return true;
                    }
                }
                if (toVisitOffset == 0) break;
                currentNodeIndex = nodesToVisit[--toVisitOffset];
            } else {
                if (0){
                    /// second child first
                    nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
                    currentNodeIndex = node->secondChildOffset;
                } else {
                    nodesToVisit[toVisitOffset++] = node->secondChildOffset;
                    currentNodeIndex = currentNodeIndex + 1;
                }
            }
        } else {
            if (toVisitOffset == 0) break;
            currentNodeIndex = nodesToVisit[--toVisitOffset];
        }
    }
    return false;
}
inline bool Intersect000 (const std::vector<std::shared_ptr<Primitive>> &primitives, const LinearBVHNode *nodes, const Ray &ray, SurfaceInteraction *isect){
    if (!nodes) return false;
    #ifdef PBRT_8_CLASSES_DEBUG
    CHECK_EQ((ray.d.x < 0), 0);
    CHECK_EQ((ray.d.y < 0), 0);
    CHECK_EQ((ray.d.z < 0), 0);
    #endif
    bool hit = false;
    Vector3f invDir(1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z);
    #ifdef CACHE_INVDIR_BY_GAMMA
    Vector3f invDirByGamma = invDir * (1 + 2 * gamma(3));
    #endif
    //int dirIsNeg[3] = {invDir.x < 0, invDir.y < 0, invDir.z < 0};
    int nodesToVisit[64];
    int toVisitOffset = 0, currentNodeIndex = 0;
    while (true) {
        const LinearBVHNode *node = &nodes[currentNodeIndex];
        if (
            #ifndef CACHE_INVDIR_BY_GAMMA
            IntersectP000BOX(node->bounds, ray, invDir)
            #else
            IntersectP000BOX(node->bounds, ray, invDir, invDirByGamma)
            #endif
            ) {
            // Process BVH node _node_ for traversal
            if (node->nPrimitives > 0) {
                // Intersect ray with primitives in leaf BVH node
                for (int i = 0; i < node->nPrimitives; ++i)
                    if (primitives[node->primitivesOffset + i]->Intersect(
                            ray, isect))
                        hit = true;
                if (toVisitOffset == 0) break;
                currentNodeIndex = nodesToVisit[--toVisitOffset];
            } else {
                if (0){
                    /// second child first
                    nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
                    currentNodeIndex = node->secondChildOffset;
                } else {
                    nodesToVisit[toVisitOffset++] = node->secondChildOffset;
                    currentNodeIndex = currentNodeIndex + 1;
                }
            }
        } else {
            if (toVisitOffset == 0) break;
            currentNodeIndex = nodesToVisit[--toVisitOffset];
        }
    }
    return hit;
}

inline bool IntersectP001BOX(const Bounds3f& bounds, const Ray &ray, const Vector3f &invDir
                                   #ifdef CACHE_INVDIR_BY_GAMMA
                                   , const Vector3f &invDirByGamma
                                   #endif
                                   )  {
    #ifndef CACHE_INVDIR_BY_GAMMA
    const Vector3f &invDirByGamma = invDir;
    #endif
    // This will also be removed in optimization
    
    // Check for ray intersection against $x$ and $y$ slabs
    Float tMin = (bounds.pMin.x - ray.o.x) * invDir.x;
    Float tMax = (bounds.pMax.x - ray.o.x) * invDirByGamma.x;
    Float tyMin = (bounds.pMin.y - ray.o.y) * invDir.y;
    Float tyMax = (bounds.pMax.y - ray.o.y) * invDirByGamma.y;

    #ifndef CACHE_INVDIR_BY_GAMMA
    // Update _tMax_ and _tyMax_ to ensure robust bounds intersection
    tMax *= 1 + 2 * gamma(3);
    tyMax *= 1 + 2 * gamma(3);
    #endif
    if (tMin > tyMax || tyMin > tMax) return false;
    if (tyMin > tMin) tMin = tyMin;
    if (tyMax < tMax) tMax = tyMax;

    // Check for ray intersection against $z$ slab
    Float tzMin = (bounds.pMax.z - ray.o.z) * invDir.z;
    Float tzMax = (bounds.pMin.z - ray.o.z) * invDirByGamma.z;
    
    #ifndef CACHE_INVDIR_BY_GAMMA
    // Update _tzMax_ to ensure robust bounds intersection
    tzMax *= 1 + 2 * gamma(3);
    #endif
    if (tMin > tzMax || tzMin > tMax) return false;
    if (tzMin > tMin) tMin = tzMin;
    if (tzMax < tMax) tMax = tzMax;
    return (tMin < ray.tMax) && (tMax > 0);
}
inline bool IntersectP001 (const std::vector<std::shared_ptr<Primitive>> &primitives, const LinearBVHNode *nodes, const Ray &ray){
    if (!nodes) return false;
    #ifdef PBRT_8_CLASSES_DEBUG
    CHECK_EQ((ray.d.x < 0), 0);
    CHECK_EQ((ray.d.y < 0), 0);
    CHECK_EQ((ray.d.z < 0), 1);
    #endif
    Vector3f invDir(1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z);
    #ifdef CACHE_INVDIR_BY_GAMMA
    Vector3f invDirByGamma = invDir * (1 + 2 * gamma(3));
    #endif
    //int dirIsNeg[3] = {invDir.x < 0, invDir.y < 0, invDir.z < 0};
    int nodesToVisit[64];
    int toVisitOffset = 0, currentNodeIndex = 0;
    while (true) {
        const LinearBVHNode *node = &nodes[currentNodeIndex];
        if (
            #ifndef CACHE_INVDIR_BY_GAMMA
            IntersectP001BOX(node->bounds, ray, invDir)
            #else
            IntersectP001BOX(node->bounds, ray, invDir, invDirByGamma)
            #endif
            ) {
            // Process BVH node _node_ for traversal
            if (node->nPrimitives > 0) {
                for (int i = 0; i < node->nPrimitives; ++i) {
                    if (primitives[node->primitivesOffset + i]->IntersectP(
                            ray)) {
                        return true;
                    }
                }
                if (toVisitOffset == 0) break;
                currentNodeIndex = nodesToVisit[--toVisitOffset];
            } else {
                if (node->axis == 2){
                    /// second child first
                    nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
                    currentNodeIndex = node->secondChildOffset;
                } else {
                    nodesToVisit[toVisitOffset++] = node->secondChildOffset;
                    currentNodeIndex = currentNodeIndex + 1;
                }
            }
        } else {
            if (toVisitOffset == 0) break;
            currentNodeIndex = nodesToVisit[--toVisitOffset];
        }
    }
    return false;
}
inline bool Intersect001 (const std::vector<std::shared_ptr<Primitive>> &primitives, const LinearBVHNode *nodes, const Ray &ray, SurfaceInteraction *isect){
    if (!nodes) return false;
    #ifdef PBRT_8_CLASSES_DEBUG
    CHECK_EQ((ray.d.x < 0), 0);
    CHECK_EQ((ray.d.y < 0), 0);
    CHECK_EQ((ray.d.z < 0), 1);
    #endif
    bool hit = false;
    Vector3f invDir(1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z);
    #ifdef CACHE_INVDIR_BY_GAMMA
    Vector3f invDirByGamma = invDir * (1 + 2 * gamma(3));
    #endif
    //int dirIsNeg[3] = {invDir.x < 0, invDir.y < 0, invDir.z < 0};
    int nodesToVisit[64];
    int toVisitOffset = 0, currentNodeIndex = 0;
    while (true) {
        const LinearBVHNode *node = &nodes[currentNodeIndex];
        if (
            #ifndef CACHE_INVDIR_BY_GAMMA
            IntersectP001BOX(node->bounds, ray, invDir)
            #else
            IntersectP001BOX(node->bounds, ray, invDir, invDirByGamma)
            #endif
            ) {
            // Process BVH node _node_ for traversal
            if (node->nPrimitives > 0) {
                // Intersect ray with primitives in leaf BVH node
                for (int i = 0; i < node->nPrimitives; ++i)
                    if (primitives[node->primitivesOffset + i]->Intersect(
                            ray, isect))
                        hit = true;
                if (toVisitOffset == 0) break;
                currentNodeIndex = nodesToVisit[--toVisitOffset];
            } else {
                if (node->axis == 2){
                    /// second child first
                    nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
                    currentNodeIndex = node->secondChildOffset;
                } else {
                    nodesToVisit[toVisitOffset++] = node->secondChildOffset;
                    currentNodeIndex = currentNodeIndex + 1;
                }
            }
        } else {
            if (toVisitOffset == 0) break;
            currentNodeIndex = nodesToVisit[--toVisitOffset];
        }
    }
    return hit;
}

inline bool IntersectP010BOX(const Bounds3f& bounds, const Ray &ray, const Vector3f &invDir
                                   #ifdef CACHE_INVDIR_BY_GAMMA
                                   , const Vector3f &invDirByGamma
                                   #endif
                                   )  {
    #ifndef CACHE_INVDIR_BY_GAMMA
    const Vector3f &invDirByGamma = invDir;
    #endif
    // This will also be removed in optimization
    
    // Check for ray intersection against $x$ and $y$ slabs
    Float tMin = (bounds.pMin.x - ray.o.x) * invDir.x;
    Float tMax = (bounds.pMax.x - ray.o.x) * invDirByGamma.x;
    Float tyMin = (bounds.pMax.y - ray.o.y) * invDir.y;
    Float tyMax = (bounds.pMin.y - ray.o.y) * invDirByGamma.y;

    #ifndef CACHE_INVDIR_BY_GAMMA
    // Update _tMax_ and _tyMax_ to ensure robust bounds intersection
    tMax *= 1 + 2 * gamma(3);
    tyMax *= 1 + 2 * gamma(3);
    #endif
    if (tMin > tyMax || tyMin > tMax) return false;
    if (tyMin > tMin) tMin = tyMin;
    if (tyMax < tMax) tMax = tyMax;

    // Check for ray intersection against $z$ slab
    Float tzMin = (bounds.pMin.z - ray.o.z) * invDir.z;
    Float tzMax = (bounds.pMax.z - ray.o.z) * invDirByGamma.z;
    
    #ifndef CACHE_INVDIR_BY_GAMMA
    // Update _tzMax_ to ensure robust bounds intersection
    tzMax *= 1 + 2 * gamma(3);
    #endif
    if (tMin > tzMax || tzMin > tMax) return false;
    if (tzMin > tMin) tMin = tzMin;
    if (tzMax < tMax) tMax = tzMax;
    return (tMin < ray.tMax) && (tMax > 0);
}
inline bool IntersectP010 (const std::vector<std::shared_ptr<Primitive>> &primitives, const LinearBVHNode *nodes, const Ray &ray){
    if (!nodes) return false;
    #ifdef PBRT_8_CLASSES_DEBUG
    CHECK_EQ((ray.d.x < 0), 0);
    CHECK_EQ((ray.d.y < 0), 1);
    CHECK_EQ((ray.d.z < 0), 0);
    #endif
    Vector3f invDir(1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z);
    #ifdef CACHE_INVDIR_BY_GAMMA
    Vector3f invDirByGamma = invDir * (1 + 2 * gamma(3));
    #endif
    //int dirIsNeg[3] = {invDir.x < 0, invDir.y < 0, invDir.z < 0};
    int nodesToVisit[64];
    int toVisitOffset = 0, currentNodeIndex = 0;
    while (true) {
        const LinearBVHNode *node = &nodes[currentNodeIndex];
        if (
            #ifndef CACHE_INVDIR_BY_GAMMA
            IntersectP010BOX(node->bounds, ray, invDir)
            #else
            IntersectP010BOX(node->bounds, ray, invDir, invDirByGamma)
            #endif
            ) {
            // Process BVH node _node_ for traversal
            if (node->nPrimitives > 0) {
                for (int i = 0; i < node->nPrimitives; ++i) {
                    if (primitives[node->primitivesOffset + i]->IntersectP(
                            ray)) {
                        return true;
                    }
                }
                if (toVisitOffset == 0) break;
                currentNodeIndex = nodesToVisit[--toVisitOffset];
            } else {
                if (node->axis == 1){
                    /// second child first
                    nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
                    currentNodeIndex = node->secondChildOffset;
                } else {
                    nodesToVisit[toVisitOffset++] = node->secondChildOffset;
                    currentNodeIndex = currentNodeIndex + 1;
                }
            }
        } else {
            if (toVisitOffset == 0) break;
            currentNodeIndex = nodesToVisit[--toVisitOffset];
        }
    }
    return false;
}
inline bool Intersect010 (const std::vector<std::shared_ptr<Primitive>> &primitives, const LinearBVHNode *nodes, const Ray &ray, SurfaceInteraction *isect){
    if (!nodes) return false;
    #ifdef PBRT_8_CLASSES_DEBUG
    CHECK_EQ((ray.d.x < 0), 0);
    CHECK_EQ((ray.d.y < 0), 1);
    CHECK_EQ((ray.d.z < 0), 0);
    #endif
    bool hit = false;
    Vector3f invDir(1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z);
    #ifdef CACHE_INVDIR_BY_GAMMA
    Vector3f invDirByGamma = invDir * (1 + 2 * gamma(3));
    #endif
    //int dirIsNeg[3] = {invDir.x < 0, invDir.y < 0, invDir.z < 0};
    int nodesToVisit[64];
    int toVisitOffset = 0, currentNodeIndex = 0;
    while (true) {
        const LinearBVHNode *node = &nodes[currentNodeIndex];
        if (
            #ifndef CACHE_INVDIR_BY_GAMMA
            IntersectP010BOX(node->bounds, ray, invDir)
            #else
            IntersectP010BOX(node->bounds, ray, invDir, invDirByGamma)
            #endif
            ) {
            // Process BVH node _node_ for traversal
            if (node->nPrimitives > 0) {
                // Intersect ray with primitives in leaf BVH node
                for (int i = 0; i < node->nPrimitives; ++i)
                    if (primitives[node->primitivesOffset + i]->Intersect(
                            ray, isect))
                        hit = true;
                if (toVisitOffset == 0) break;
                currentNodeIndex = nodesToVisit[--toVisitOffset];
            } else {
                if (node->axis == 1){
                    /// second child first
                    nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
                    currentNodeIndex = node->secondChildOffset;
                } else {
                    nodesToVisit[toVisitOffset++] = node->secondChildOffset;
                    currentNodeIndex = currentNodeIndex + 1;
                }
            }
        } else {
            if (toVisitOffset == 0) break;
            currentNodeIndex = nodesToVisit[--toVisitOffset];
        }
    }
    return hit;
}

inline bool IntersectP011BOX(const Bounds3f& bounds, const Ray &ray, const Vector3f &invDir
                                   #ifdef CACHE_INVDIR_BY_GAMMA
                                   , const Vector3f &invDirByGamma
                                   #endif
                                   )  {
    #ifndef CACHE_INVDIR_BY_GAMMA
    const Vector3f &invDirByGamma = invDir;
    #endif
    // This will also be removed in optimization
    
    // Check for ray intersection against $x$ and $y$ slabs
    Float tMin = (bounds.pMin.x - ray.o.x) * invDir.x;
    Float tMax = (bounds.pMax.x - ray.o.x) * invDirByGamma.x;
    Float tyMin = (bounds.pMax.y - ray.o.y) * invDir.y;
    Float tyMax = (bounds.pMin.y - ray.o.y) * invDirByGamma.y;

    #ifndef CACHE_INVDIR_BY_GAMMA
    // Update _tMax_ and _tyMax_ to ensure robust bounds intersection
    tMax *= 1 + 2 * gamma(3);
    tyMax *= 1 + 2 * gamma(3);
    #endif
    if (tMin > tyMax || tyMin > tMax) return false;
    if (tyMin > tMin) tMin = tyMin;
    if (tyMax < tMax) tMax = tyMax;

    // Check for ray intersection against $z$ slab
    Float tzMin = (bounds.pMax.z - ray.o.z) * invDir.z;
    Float tzMax = (bounds.pMin.z - ray.o.z) * invDirByGamma.z;
    
    #ifndef CACHE_INVDIR_BY_GAMMA
    // Update _tzMax_ to ensure robust bounds intersection
    tzMax *= 1 + 2 * gamma(3);
    #endif
    if (tMin > tzMax || tzMin > tMax) return false;
    if (tzMin > tMin) tMin = tzMin;
    if (tzMax < tMax) tMax = tzMax;
    return (tMin < ray.tMax) && (tMax > 0);
}
inline bool IntersectP011 (const std::vector<std::shared_ptr<Primitive>> &primitives, const LinearBVHNode *nodes, const Ray &ray){
    if (!nodes) return false;
    #ifdef PBRT_8_CLASSES_DEBUG
    CHECK_EQ((ray.d.x < 0), 0);
    CHECK_EQ((ray.d.y < 0), 1);
    CHECK_EQ((ray.d.z < 0), 1);
    #endif
    Vector3f invDir(1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z);
    #ifdef CACHE_INVDIR_BY_GAMMA
    Vector3f invDirByGamma = invDir * (1 + 2 * gamma(3));
    #endif
    //int dirIsNeg[3] = {invDir.x < 0, invDir.y < 0, invDir.z < 0};
    int nodesToVisit[64];
    int toVisitOffset = 0, currentNodeIndex = 0;
    while (true) {
        const LinearBVHNode *node = &nodes[currentNodeIndex];
        if (
            #ifndef CACHE_INVDIR_BY_GAMMA
            IntersectP011BOX(node->bounds, ray, invDir)
            #else
            IntersectP011BOX(node->bounds, ray, invDir, invDirByGamma)
            #endif
            ) {
            // Process BVH node _node_ for traversal
            if (node->nPrimitives > 0) {
                for (int i = 0; i < node->nPrimitives; ++i) {
                    if (primitives[node->primitivesOffset + i]->IntersectP(
                            ray)) {
                        return true;
                    }
                }
                if (toVisitOffset == 0) break;
                currentNodeIndex = nodesToVisit[--toVisitOffset];
            } else {
                if (node->axis != 0){
                    /// second child first
                    nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
                    currentNodeIndex = node->secondChildOffset;
                } else {
                    nodesToVisit[toVisitOffset++] = node->secondChildOffset;
                    currentNodeIndex = currentNodeIndex + 1;
                }
            }
        } else {
            if (toVisitOffset == 0) break;
            currentNodeIndex = nodesToVisit[--toVisitOffset];
        }
    }
    return false;
}
inline bool Intersect011 (const std::vector<std::shared_ptr<Primitive>> &primitives, const LinearBVHNode *nodes, const Ray &ray, SurfaceInteraction *isect){
    if (!nodes) return false;
    #ifdef PBRT_8_CLASSES_DEBUG
    CHECK_EQ((ray.d.x < 0), 0);
    CHECK_EQ((ray.d.y < 0), 1);
    CHECK_EQ((ray.d.z < 0), 1);
    #endif
    bool hit = false;
    Vector3f invDir(1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z);
    #ifdef CACHE_INVDIR_BY_GAMMA
    Vector3f invDirByGamma = invDir * (1 + 2 * gamma(3));
    #endif
    //int dirIsNeg[3] = {invDir.x < 0, invDir.y < 0, invDir.z < 0};
    int nodesToVisit[64];
    int toVisitOffset = 0, currentNodeIndex = 0;
    while (true) {
        const LinearBVHNode *node = &nodes[currentNodeIndex];
        if (
            #ifndef CACHE_INVDIR_BY_GAMMA
            IntersectP011BOX(node->bounds, ray, invDir)
            #else
            IntersectP011BOX(node->bounds, ray, invDir, invDirByGamma)
            #endif
            ) {
            // Process BVH node _node_ for traversal
            if (node->nPrimitives > 0) {
                // Intersect ray with primitives in leaf BVH node
                for (int i = 0; i < node->nPrimitives; ++i)
                    if (primitives[node->primitivesOffset + i]->Intersect(
                            ray, isect))
                        hit = true;
                if (toVisitOffset == 0) break;
                currentNodeIndex = nodesToVisit[--toVisitOffset];
            } else {
                if (node->axis != 0){
                    /// second child first
                    nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
                    currentNodeIndex = node->secondChildOffset;
                } else {
                    nodesToVisit[toVisitOffset++] = node->secondChildOffset;
                    currentNodeIndex = currentNodeIndex + 1;
                }
            }
        } else {
            if (toVisitOffset == 0) break;
            currentNodeIndex = nodesToVisit[--toVisitOffset];
        }
    }
    return hit;
}

inline bool IntersectP100BOX(const Bounds3f& bounds, const Ray &ray, const Vector3f &invDir
                                   #ifdef CACHE_INVDIR_BY_GAMMA
                                   , const Vector3f &invDirByGamma
                                   #endif
                                   )  {
    #ifndef CACHE_INVDIR_BY_GAMMA
    const Vector3f &invDirByGamma = invDir;
    #endif
    // This will also be removed in optimization
    
    // Check for ray intersection against $x$ and $y$ slabs
    Float tMin = (bounds.pMax.x - ray.o.x) * invDir.x;
    Float tMax = (bounds.pMin.x - ray.o.x) * invDirByGamma.x;
    Float tyMin = (bounds.pMin.y - ray.o.y) * invDir.y;
    Float tyMax = (bounds.pMax.y - ray.o.y) * invDirByGamma.y;

    #ifndef CACHE_INVDIR_BY_GAMMA
    // Update _tMax_ and _tyMax_ to ensure robust bounds intersection
    tMax *= 1 + 2 * gamma(3);
    tyMax *= 1 + 2 * gamma(3);
    #endif
    if (tMin > tyMax || tyMin > tMax) return false;
    if (tyMin > tMin) tMin = tyMin;
    if (tyMax < tMax) tMax = tyMax;

    // Check for ray intersection against $z$ slab
    Float tzMin = (bounds.pMin.z - ray.o.z) * invDir.z;
    Float tzMax = (bounds.pMax.z - ray.o.z) * invDirByGamma.z;
    
    #ifndef CACHE_INVDIR_BY_GAMMA
    // Update _tzMax_ to ensure robust bounds intersection
    tzMax *= 1 + 2 * gamma(3);
    #endif
    if (tMin > tzMax || tzMin > tMax) return false;
    if (tzMin > tMin) tMin = tzMin;
    if (tzMax < tMax) tMax = tzMax;
    return (tMin < ray.tMax) && (tMax > 0);
}
inline bool IntersectP100 (const std::vector<std::shared_ptr<Primitive>> &primitives, const LinearBVHNode *nodes, const Ray &ray){
    if (!nodes) return false;
    #ifdef PBRT_8_CLASSES_DEBUG
    CHECK_EQ((ray.d.x < 0), 1);
    CHECK_EQ((ray.d.y < 0), 0);
    CHECK_EQ((ray.d.z < 0), 0);
    #endif
    Vector3f invDir(1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z);
    #ifdef CACHE_INVDIR_BY_GAMMA
    Vector3f invDirByGamma = invDir * (1 + 2 * gamma(3));
    #endif
    //int dirIsNeg[3] = {invDir.x < 0, invDir.y < 0, invDir.z < 0};
    int nodesToVisit[64];
    int toVisitOffset = 0, currentNodeIndex = 0;
    while (true) {
        const LinearBVHNode *node = &nodes[currentNodeIndex];
        if (
            #ifndef CACHE_INVDIR_BY_GAMMA
            IntersectP100BOX(node->bounds, ray, invDir)
            #else
            IntersectP100BOX(node->bounds, ray, invDir, invDirByGamma)
            #endif
            ) {
            // Process BVH node _node_ for traversal
            if (node->nPrimitives > 0) {
                for (int i = 0; i < node->nPrimitives; ++i) {
                    if (primitives[node->primitivesOffset + i]->IntersectP(
                            ray)) {
                        return true;
                    }
                }
                if (toVisitOffset == 0) break;
                currentNodeIndex = nodesToVisit[--toVisitOffset];
            } else {
                if (node->axis == 0){
                    /// second child first
                    nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
                    currentNodeIndex = node->secondChildOffset;
                } else {
                    nodesToVisit[toVisitOffset++] = node->secondChildOffset;
                    currentNodeIndex = currentNodeIndex + 1;
                }
            }
        } else {
            if (toVisitOffset == 0) break;
            currentNodeIndex = nodesToVisit[--toVisitOffset];
        }
    }
    return false;
}
inline bool Intersect100 (const std::vector<std::shared_ptr<Primitive>> &primitives, const LinearBVHNode *nodes, const Ray &ray, SurfaceInteraction *isect){
    if (!nodes) return false;
    #ifdef PBRT_8_CLASSES_DEBUG
    CHECK_EQ((ray.d.x < 0), 1);
    CHECK_EQ((ray.d.y < 0), 0);
    CHECK_EQ((ray.d.z < 0), 0);
    #endif
    bool hit = false;
    Vector3f invDir(1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z);
    #ifdef CACHE_INVDIR_BY_GAMMA
    Vector3f invDirByGamma = invDir * (1 + 2 * gamma(3));
    #endif
    //int dirIsNeg[3] = {invDir.x < 0, invDir.y < 0, invDir.z < 0};
    int nodesToVisit[64];
    int toVisitOffset = 0, currentNodeIndex = 0;
    while (true) {
        const LinearBVHNode *node = &nodes[currentNodeIndex];
        if (
            #ifndef CACHE_INVDIR_BY_GAMMA
            IntersectP100BOX(node->bounds, ray, invDir)
            #else
            IntersectP100BOX(node->bounds, ray, invDir, invDirByGamma)
            #endif
            ) {
            // Process BVH node _node_ for traversal
            if (node->nPrimitives > 0) {
                // Intersect ray with primitives in leaf BVH node
                for (int i = 0; i < node->nPrimitives; ++i)
                    if (primitives[node->primitivesOffset + i]->Intersect(
                            ray, isect))
                        hit = true;
                if (toVisitOffset == 0) break;
                currentNodeIndex = nodesToVisit[--toVisitOffset];
            } else {
                if (node->axis == 0){
                    /// second child first
                    nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
                    currentNodeIndex = node->secondChildOffset;
                } else {
                    nodesToVisit[toVisitOffset++] = node->secondChildOffset;
                    currentNodeIndex = currentNodeIndex + 1;
                }
            }
        } else {
            if (toVisitOffset == 0) break;
            currentNodeIndex = nodesToVisit[--toVisitOffset];
        }
    }
    return hit;
}

inline bool IntersectP101BOX(const Bounds3f& bounds, const Ray &ray, const Vector3f &invDir
                                   #ifdef CACHE_INVDIR_BY_GAMMA
                                   , const Vector3f &invDirByGamma
                                   #endif
                                   )  {
    #ifndef CACHE_INVDIR_BY_GAMMA
    const Vector3f &invDirByGamma = invDir;
    #endif
    // This will also be removed in optimization
    
    // Check for ray intersection against $x$ and $y$ slabs
    Float tMin = (bounds.pMax.x - ray.o.x) * invDir.x;
    Float tMax = (bounds.pMin.x - ray.o.x) * invDirByGamma.x;
    Float tyMin = (bounds.pMin.y - ray.o.y) * invDir.y;
    Float tyMax = (bounds.pMax.y - ray.o.y) * invDirByGamma.y;

    #ifndef CACHE_INVDIR_BY_GAMMA
    // Update _tMax_ and _tyMax_ to ensure robust bounds intersection
    tMax *= 1 + 2 * gamma(3);
    tyMax *= 1 + 2 * gamma(3);
    #endif
    if (tMin > tyMax || tyMin > tMax) return false;
    if (tyMin > tMin) tMin = tyMin;
    if (tyMax < tMax) tMax = tyMax;

    // Check for ray intersection against $z$ slab
    Float tzMin = (bounds.pMax.z - ray.o.z) * invDir.z;
    Float tzMax = (bounds.pMin.z - ray.o.z) * invDirByGamma.z;
    
    #ifndef CACHE_INVDIR_BY_GAMMA
    // Update _tzMax_ to ensure robust bounds intersection
    tzMax *= 1 + 2 * gamma(3);
    #endif
    if (tMin > tzMax || tzMin > tMax) return false;
    if (tzMin > tMin) tMin = tzMin;
    if (tzMax < tMax) tMax = tzMax;
    return (tMin < ray.tMax) && (tMax > 0);
}
inline bool IntersectP101 (const std::vector<std::shared_ptr<Primitive>> &primitives, const LinearBVHNode *nodes, const Ray &ray){
    if (!nodes) return false;
    #ifdef PBRT_8_CLASSES_DEBUG
    CHECK_EQ((ray.d.x < 0), 1);
    CHECK_EQ((ray.d.y < 0), 0);
    CHECK_EQ((ray.d.z < 0), 1);
    #endif
    Vector3f invDir(1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z);
    #ifdef CACHE_INVDIR_BY_GAMMA
    Vector3f invDirByGamma = invDir * (1 + 2 * gamma(3));
    #endif
    //int dirIsNeg[3] = {invDir.x < 0, invDir.y < 0, invDir.z < 0};
    int nodesToVisit[64];
    int toVisitOffset = 0, currentNodeIndex = 0;
    while (true) {
        const LinearBVHNode *node = &nodes[currentNodeIndex];
        if (
            #ifndef CACHE_INVDIR_BY_GAMMA
            IntersectP101BOX(node->bounds, ray, invDir)
            #else
            IntersectP101BOX(node->bounds, ray, invDir, invDirByGamma)
            #endif
            ) {
            // Process BVH node _node_ for traversal
            if (node->nPrimitives > 0) {
                for (int i = 0; i < node->nPrimitives; ++i) {
                    if (primitives[node->primitivesOffset + i]->IntersectP(
                            ray)) {
                        return true;
                    }
                }
                if (toVisitOffset == 0) break;
                currentNodeIndex = nodesToVisit[--toVisitOffset];
            } else {
                if (node->axis != 1){
                    /// second child first
                    nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
                    currentNodeIndex = node->secondChildOffset;
                } else {
                    nodesToVisit[toVisitOffset++] = node->secondChildOffset;
                    currentNodeIndex = currentNodeIndex + 1;
                }
            }
        } else {
            if (toVisitOffset == 0) break;
            currentNodeIndex = nodesToVisit[--toVisitOffset];
        }
    }
    return false;
}
inline bool Intersect101 (const std::vector<std::shared_ptr<Primitive>> &primitives, const LinearBVHNode *nodes, const Ray &ray, SurfaceInteraction *isect){
    if (!nodes) return false;
    #ifdef PBRT_8_CLASSES_DEBUG
    CHECK_EQ((ray.d.x < 0), 1);
    CHECK_EQ((ray.d.y < 0), 0);
    CHECK_EQ((ray.d.z < 0), 1);
    #endif
    bool hit = false;
    Vector3f invDir(1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z);
    #ifdef CACHE_INVDIR_BY_GAMMA
    Vector3f invDirByGamma = invDir * (1 + 2 * gamma(3));
    #endif
    //int dirIsNeg[3] = {invDir.x < 0, invDir.y < 0, invDir.z < 0};
    int nodesToVisit[64];
    int toVisitOffset = 0, currentNodeIndex = 0;
    while (true) {
        const LinearBVHNode *node = &nodes[currentNodeIndex];
        if (
            #ifndef CACHE_INVDIR_BY_GAMMA
            IntersectP101BOX(node->bounds, ray, invDir)
            #else
            IntersectP101BOX(node->bounds, ray, invDir, invDirByGamma)
            #endif
            ) {
            // Process BVH node _node_ for traversal
            if (node->nPrimitives > 0) {
                // Intersect ray with primitives in leaf BVH node
                for (int i = 0; i < node->nPrimitives; ++i)
                    if (primitives[node->primitivesOffset + i]->Intersect(
                            ray, isect))
                        hit = true;
                if (toVisitOffset == 0) break;
                currentNodeIndex = nodesToVisit[--toVisitOffset];
            } else {
                if (node->axis != 1){
                    /// second child first
                    nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
                    currentNodeIndex = node->secondChildOffset;
                } else {
                    nodesToVisit[toVisitOffset++] = node->secondChildOffset;
                    currentNodeIndex = currentNodeIndex + 1;
                }
            }
        } else {
            if (toVisitOffset == 0) break;
            currentNodeIndex = nodesToVisit[--toVisitOffset];
        }
    }
    return hit;
}

inline bool IntersectP110BOX(const Bounds3f& bounds, const Ray &ray, const Vector3f &invDir
                                   #ifdef CACHE_INVDIR_BY_GAMMA
                                   , const Vector3f &invDirByGamma
                                   #endif
                                   )  {
    #ifndef CACHE_INVDIR_BY_GAMMA
    const Vector3f &invDirByGamma = invDir;
    #endif
    // This will also be removed in optimization
    
    // Check for ray intersection against $x$ and $y$ slabs
    Float tMin = (bounds.pMax.x - ray.o.x) * invDir.x;
    Float tMax = (bounds.pMin.x - ray.o.x) * invDirByGamma.x;
    Float tyMin = (bounds.pMax.y - ray.o.y) * invDir.y;
    Float tyMax = (bounds.pMin.y - ray.o.y) * invDirByGamma.y;

    #ifndef CACHE_INVDIR_BY_GAMMA
    // Update _tMax_ and _tyMax_ to ensure robust bounds intersection
    tMax *= 1 + 2 * gamma(3);
    tyMax *= 1 + 2 * gamma(3);
    #endif
    if (tMin > tyMax || tyMin > tMax) return false;
    if (tyMin > tMin) tMin = tyMin;
    if (tyMax < tMax) tMax = tyMax;

    // Check for ray intersection against $z$ slab
    Float tzMin = (bounds.pMin.z - ray.o.z) * invDir.z;
    Float tzMax = (bounds.pMax.z - ray.o.z) * invDirByGamma.z;
    
    #ifndef CACHE_INVDIR_BY_GAMMA
    // Update _tzMax_ to ensure robust bounds intersection
    tzMax *= 1 + 2 * gamma(3);
    #endif
    if (tMin > tzMax || tzMin > tMax) return false;
    if (tzMin > tMin) tMin = tzMin;
    if (tzMax < tMax) tMax = tzMax;
    return (tMin < ray.tMax) && (tMax > 0);
}
inline bool IntersectP110 (const std::vector<std::shared_ptr<Primitive>> &primitives, const LinearBVHNode *nodes, const Ray &ray){
    if (!nodes) return false;
    #ifdef PBRT_8_CLASSES_DEBUG
    CHECK_EQ((ray.d.x < 0), 1);
    CHECK_EQ((ray.d.y < 0), 1);
    CHECK_EQ((ray.d.z < 0), 0);
    #endif
    Vector3f invDir(1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z);
    #ifdef CACHE_INVDIR_BY_GAMMA
    Vector3f invDirByGamma = invDir * (1 + 2 * gamma(3));
    #endif
    //int dirIsNeg[3] = {invDir.x < 0, invDir.y < 0, invDir.z < 0};
    int nodesToVisit[64];
    int toVisitOffset = 0, currentNodeIndex = 0;
    while (true) {
        const LinearBVHNode *node = &nodes[currentNodeIndex];
        if (
            #ifndef CACHE_INVDIR_BY_GAMMA
            IntersectP110BOX(node->bounds, ray, invDir)
            #else
            IntersectP110BOX(node->bounds, ray, invDir, invDirByGamma)
            #endif
            ) {
            // Process BVH node _node_ for traversal
            if (node->nPrimitives > 0) {
                for (int i = 0; i < node->nPrimitives; ++i) {
                    if (primitives[node->primitivesOffset + i]->IntersectP(
                            ray)) {
                        return true;
                    }
                }
                if (toVisitOffset == 0) break;
                currentNodeIndex = nodesToVisit[--toVisitOffset];
            } else {
                if (node->axis != 2){
                    /// second child first
                    nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
                    currentNodeIndex = node->secondChildOffset;
                } else {
                    nodesToVisit[toVisitOffset++] = node->secondChildOffset;
                    currentNodeIndex = currentNodeIndex + 1;
                }
            }
        } else {
            if (toVisitOffset == 0) break;
            currentNodeIndex = nodesToVisit[--toVisitOffset];
        }
    }
    return false;
}
inline bool Intersect110 (const std::vector<std::shared_ptr<Primitive>> &primitives, const LinearBVHNode *nodes, const Ray &ray, SurfaceInteraction *isect){
    if (!nodes) return false;
    #ifdef PBRT_8_CLASSES_DEBUG
    CHECK_EQ((ray.d.x < 0), 1);
    CHECK_EQ((ray.d.y < 0), 1);
    CHECK_EQ((ray.d.z < 0), 0);
    #endif
    bool hit = false;
    Vector3f invDir(1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z);
    #ifdef CACHE_INVDIR_BY_GAMMA
    Vector3f invDirByGamma = invDir * (1 + 2 * gamma(3));
    #endif
    //int dirIsNeg[3] = {invDir.x < 0, invDir.y < 0, invDir.z < 0};
    int nodesToVisit[64];
    int toVisitOffset = 0, currentNodeIndex = 0;
    while (true) {
        const LinearBVHNode *node = &nodes[currentNodeIndex];
        if (
            #ifndef CACHE_INVDIR_BY_GAMMA
            IntersectP110BOX(node->bounds, ray, invDir)
            #else
            IntersectP110BOX(node->bounds, ray, invDir, invDirByGamma)
            #endif
            ) {
            // Process BVH node _node_ for traversal
            if (node->nPrimitives > 0) {
                // Intersect ray with primitives in leaf BVH node
                for (int i = 0; i < node->nPrimitives; ++i)
                    if (primitives[node->primitivesOffset + i]->Intersect(
                            ray, isect))
                        hit = true;
                if (toVisitOffset == 0) break;
                currentNodeIndex = nodesToVisit[--toVisitOffset];
            } else {
                if (node->axis != 2){
                    /// second child first
                    nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
                    currentNodeIndex = node->secondChildOffset;
                } else {
                    nodesToVisit[toVisitOffset++] = node->secondChildOffset;
                    currentNodeIndex = currentNodeIndex + 1;
                }
            }
        } else {
            if (toVisitOffset == 0) break;
            currentNodeIndex = nodesToVisit[--toVisitOffset];
        }
    }
    return hit;
}

inline bool IntersectP111BOX(const Bounds3f& bounds, const Ray &ray, const Vector3f &invDir
                                   #ifdef CACHE_INVDIR_BY_GAMMA
                                   , const Vector3f &invDirByGamma
                                   #endif
                                   )  {
    #ifndef CACHE_INVDIR_BY_GAMMA
    const Vector3f &invDirByGamma = invDir;
    #endif
    // This will also be removed in optimization
    
    // Check for ray intersection against $x$ and $y$ slabs
    Float tMin = (bounds.pMax.x - ray.o.x) * invDir.x;
    Float tMax = (bounds.pMin.x - ray.o.x) * invDirByGamma.x;
    Float tyMin = (bounds.pMax.y - ray.o.y) * invDir.y;
    Float tyMax = (bounds.pMin.y - ray.o.y) * invDirByGamma.y;

    #ifndef CACHE_INVDIR_BY_GAMMA
    // Update _tMax_ and _tyMax_ to ensure robust bounds intersection
    tMax *= 1 + 2 * gamma(3);
    tyMax *= 1 + 2 * gamma(3);
    #endif
    if (tMin > tyMax || tyMin > tMax) return false;
    if (tyMin > tMin) tMin = tyMin;
    if (tyMax < tMax) tMax = tyMax;

    // Check for ray intersection against $z$ slab
    Float tzMin = (bounds.pMax.z - ray.o.z) * invDir.z;
    Float tzMax = (bounds.pMin.z - ray.o.z) * invDirByGamma.z;
    
    #ifndef CACHE_INVDIR_BY_GAMMA
    // Update _tzMax_ to ensure robust bounds intersection
    tzMax *= 1 + 2 * gamma(3);
    #endif
    if (tMin > tzMax || tzMin > tMax) return false;
    if (tzMin > tMin) tMin = tzMin;
    if (tzMax < tMax) tMax = tzMax;
    return (tMin < ray.tMax) && (tMax > 0);
}
inline bool IntersectP111 (const std::vector<std::shared_ptr<Primitive>> &primitives, const LinearBVHNode *nodes, const Ray &ray){
    if (!nodes) return false;
    #ifdef PBRT_8_CLASSES_DEBUG
    CHECK_EQ((ray.d.x < 0), 1);
    CHECK_EQ((ray.d.y < 0), 1);
    CHECK_EQ((ray.d.z < 0), 1);
    #endif
    Vector3f invDir(1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z);
    #ifdef CACHE_INVDIR_BY_GAMMA
    Vector3f invDirByGamma = invDir * (1 + 2 * gamma(3));
    #endif
    //int dirIsNeg[3] = {invDir.x < 0, invDir.y < 0, invDir.z < 0};
    int nodesToVisit[64];
    int toVisitOffset = 0, currentNodeIndex = 0;
    while (true) {
        const LinearBVHNode *node = &nodes[currentNodeIndex];
        if (
            #ifndef CACHE_INVDIR_BY_GAMMA
            IntersectP111BOX(node->bounds, ray, invDir)
            #else
            IntersectP111BOX(node->bounds, ray, invDir, invDirByGamma)
            #endif
            ) {
            // Process BVH node _node_ for traversal
            if (node->nPrimitives > 0) {
                for (int i = 0; i < node->nPrimitives; ++i) {
                    if (primitives[node->primitivesOffset + i]->IntersectP(
                            ray)) {
                        return true;
                    }
                }
                if (toVisitOffset == 0) break;
                currentNodeIndex = nodesToVisit[--toVisitOffset];
            } else {
                if (1){
                    /// second child first
                    nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
                    currentNodeIndex = node->secondChildOffset;
                } else {
                    nodesToVisit[toVisitOffset++] = node->secondChildOffset;
                    currentNodeIndex = currentNodeIndex + 1;
                }
            }
        } else {
            if (toVisitOffset == 0) break;
            currentNodeIndex = nodesToVisit[--toVisitOffset];
        }
    }
    return false;
}
inline bool Intersect111 (const std::vector<std::shared_ptr<Primitive>> &primitives, const LinearBVHNode *nodes, const Ray &ray, SurfaceInteraction *isect){
    if (!nodes) return false;
    #ifdef PBRT_8_CLASSES_DEBUG
    CHECK_EQ((ray.d.x < 0), 1);
    CHECK_EQ((ray.d.y < 0), 1);
    CHECK_EQ((ray.d.z < 0), 1);
    #endif
    bool hit = false;
    Vector3f invDir(1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z);
    #ifdef CACHE_INVDIR_BY_GAMMA
    Vector3f invDirByGamma = invDir * (1 + 2 * gamma(3));
    #endif
    //int dirIsNeg[3] = {invDir.x < 0, invDir.y < 0, invDir.z < 0};
    int nodesToVisit[64];
    int toVisitOffset = 0, currentNodeIndex = 0;
    while (true) {
        const LinearBVHNode *node = &nodes[currentNodeIndex];
        if (
            #ifndef CACHE_INVDIR_BY_GAMMA
            IntersectP111BOX(node->bounds, ray, invDir)
            #else
            IntersectP111BOX(node->bounds, ray, invDir, invDirByGamma)
            #endif
            ) {
            // Process BVH node _node_ for traversal
            if (node->nPrimitives > 0) {
                // Intersect ray with primitives in leaf BVH node
                for (int i = 0; i < node->nPrimitives; ++i)
                    if (primitives[node->primitivesOffset + i]->Intersect(
                            ray, isect))
                        hit = true;
                if (toVisitOffset == 0) break;
                currentNodeIndex = nodesToVisit[--toVisitOffset];
            } else {
                if (1){
                    /// second child first
                    nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
                    currentNodeIndex = node->secondChildOffset;
                } else {
                    nodesToVisit[toVisitOffset++] = node->secondChildOffset;
                    currentNodeIndex = currentNodeIndex + 1;
                }
            }
        } else {
            if (toVisitOffset == 0) break;
            currentNodeIndex = nodesToVisit[--toVisitOffset];
        }
    }
    return hit;
}
