import string
#bounds.pMax : bounds.pMin;
code = string.Template("""
inline bool ${INTERSECTP_FUNCTION_NAME}BOX(const Bounds3f& bounds, const Ray &ray, const Vector3f &invDir
                                   #ifdef CACHE_INVDIR_BY_GAMMA
                                   , const Vector3f &invDirByGamma
                                   #endif
                                   )  {
    #ifndef CACHE_INVDIR_BY_GAMMA
    const Vector3f &invDirByGamma = invDir;
    #endif
    // This will also be removed in optimization
    
    // Check for ray intersection against $$x$$ and $$y$$ slabs
    Float tMin = ($pmin_x.x - ray.o.x) * invDir.x;
    Float tMax = ($pmax_x.x - ray.o.x) * invDirByGamma.x;
    Float tyMin = ($pmin_y.y - ray.o.y) * invDir.y;
    Float tyMax = ($pmax_y.y - ray.o.y) * invDirByGamma.y;

    #ifndef CACHE_INVDIR_BY_GAMMA
    // Update _tMax_ and _tyMax_ to ensure robust bounds intersection
    tMax *= 1 + 2 * gamma(3);
    tyMax *= 1 + 2 * gamma(3);
    #endif
    if (tMin > tyMax || tyMin > tMax) return false;
    if (tyMin > tMin) tMin = tyMin;
    if (tyMax < tMax) tMax = tyMax;

    // Check for ray intersection against $$z$$ slab
    Float tzMin = ($pmin_z.z - ray.o.z) * invDir.z;
    Float tzMax = ($pmax_z.z - ray.o.z) * invDirByGamma.z;
    
    #ifndef CACHE_INVDIR_BY_GAMMA
    // Update _tzMax_ to ensure robust bounds intersection
    tzMax *= 1 + 2 * gamma(3);
    #endif
    if (tMin > tzMax || tzMin > tMax) return false;
    if (tzMin > tMin) tMin = tzMin;
    if (tzMax < tMax) tMax = tzMax;
    return (tMin < ray.tMax) && (tMax > 0);
}
inline bool $INTERSECTP_FUNCTION_NAME (const std::vector<std::shared_ptr<Primitive>> &primitives, const LinearBVHNode *nodes, const Ray &ray){
    if (!nodes) return false;
    #ifdef PBRT_8_CLASSES_DEBUG
    CHECK_EQ((ray.d.x < 0), $x_neg);
    CHECK_EQ((ray.d.y < 0), $y_neg);
    CHECK_EQ((ray.d.z < 0), $z_neg);
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
            ${INTERSECTP_FUNCTION_NAME}BOX(node->bounds, ray, invDir)
            #else
            ${INTERSECTP_FUNCTION_NAME}BOX(node->bounds, ray, invDir, invDirByGamma)
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
                if ($second_child_condition){
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
inline bool $INTERSECT_FUNCTION_NAME (const std::vector<std::shared_ptr<Primitive>> &primitives, const LinearBVHNode *nodes, const Ray &ray, SurfaceInteraction *isect){
    if (!nodes) return false;
    #ifdef PBRT_8_CLASSES_DEBUG
    CHECK_EQ((ray.d.x < 0), $x_neg);
    CHECK_EQ((ray.d.y < 0), $y_neg);
    CHECK_EQ((ray.d.z < 0), $z_neg);
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
            ${INTERSECTP_FUNCTION_NAME}BOX(node->bounds, ray, invDir)
            #else
            ${INTERSECTP_FUNCTION_NAME}BOX(node->bounds, ray, invDir, invDirByGamma)
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
                if ($second_child_condition){
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
""")
result=""
x_neg = 0
while x_neg < 2:
    y_neg = 0
    while y_neg < 2:
        z_neg = 0
        while z_neg < 2:
            tag = str(x_neg) + str(y_neg) + str(z_neg)
            IntersectP = "IntersectP" + tag
            Intersect = "Intersect" + tag
            pmin_x = pmin_y = pmin_z = "bounds.pMin"
            pmax_x = pmax_y = pmax_z = "bounds.pMax"
            if(x_neg):
                pmin_x, pmax_x = pmax_x, pmin_x
            if(y_neg):
                pmin_y, pmax_y = pmax_y, pmin_y
            if(z_neg):
                pmin_z, pmax_z = pmax_z, pmin_z
            second_child_condition = ""
            if(tag.count("1") == 0): #all positive
                second_child_condition = "0"
            elif(tag.count("0") == 0): #all negative
                second_child_condition = "1"
            elif(tag.count("1") == 1): #one negative
                negative_axis = tag.find("1")
                second_child_condition = "node->axis == " + str(negative_axis)
            elif(tag.count("0") == 1): #one positive
                positive_axis = tag.find("0")
                second_child_condition = "node->axis != " + str(positive_axis)
            result += code.substitute(INTERSECTP_FUNCTION_NAME = IntersectP, INTERSECT_FUNCTION_NAME = Intersect, pmin_x = pmin_x, pmax_x = pmax_x, pmin_y = pmin_y, pmax_y = pmax_y, pmin_z = pmin_z, pmax_z = pmax_z, second_child_condition = second_child_condition, x_neg = x_neg, y_neg = y_neg, z_neg = z_neg)
            z_neg += 1
        y_neg += 1
    x_neg += 1
with open("./bvhintersect_auto.cpp", 'w') as f:
    f.write(result)
            
