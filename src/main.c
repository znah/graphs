#include <wasm_simd128.h>
#ifdef WASM
#define sqrtf __builtin_sqrtf
#else
#include <math.h>
#endif

static inline float wasm_f32x4_hsum(v128_t v) {
#if defined(__wasm_simd128__)
    v = wasm_f32x4_add(v, wasm_i32x4_shuffle(v, v, 1, 0, 3, 2));
    v = wasm_f32x4_add(v, wasm_i32x4_shuffle(v, v, 2, 3, 0, 1));
    return wasm_f32x4_extract_lane(v, 0);
#else
    return wasm_f32x4_extract_lane(v, 0) + wasm_f32x4_extract_lane(v, 1) + 
           wasm_f32x4_extract_lane(v, 2) + wasm_f32x4_extract_lane(v, 3);
#endif
}

#ifdef WASM
    #define WASM_EXPORT(name) __attribute__((export_name(name)))
#else
    #define WASM_EXPORT(name)
#endif

void * alloc(int size);

#ifdef WASM
#define PAGE_SIZE   0x10000
extern unsigned char __heap_base;
static int _heap_end = (int)&__heap_base;
typedef unsigned long long uint64_t;

WASM_EXPORT("alloc")
void * alloc(int size) {
    int ptr = (_heap_end+0xf) & ~0xf; // round up to 16 byte alignment
    _heap_end = ptr + size;
    int pages_needed = (_heap_end+PAGE_SIZE-1) / PAGE_SIZE;
    int pages_n = __builtin_wasm_memory_size(0);
    if (pages_n<pages_needed) {
        __builtin_wasm_memory_grow(0, pages_needed-pages_n);
    }
    return (void *)ptr;
}
#endif

#define BUFFER(name, type, size) type name[size] __attribute__((aligned(16))); \
  WASM_EXPORT("_get_"#name) type* get_##name() {return name;} \
  WASM_EXPORT("_len_"#name"__"#type) int get_##name##_len() {return (size);}

#define DYNAMIC_BUFFER(name, type) type * name = 0; int name##_len = 0; \
  WASM_EXPORT("_get_"#name) type* get_##name() {return name;} \
  WASM_EXPORT("_len_"#name"__"#type) int get_##name##_len() {return (name##_len);} \
  void name##_alloc(int size) {name = (type*)alloc(size*sizeof(type)); name##_len = size;}


DYNAMIC_BUFFER(sorted_x, v128_t);
DYNAMIC_BUFFER(sorted_y, v128_t);
DYNAMIC_BUFFER(sorted_z, v128_t);
DYNAMIC_BUFFER(force_x, v128_t);
DYNAMIC_BUFFER(force_y, v128_t);
DYNAMIC_BUFFER(force_z, v128_t);

DYNAMIC_BUFFER(node_start, int);
DYNAMIC_BUFFER(node_end, int);
DYNAMIC_BUFFER(node_level, int);
DYNAMIC_BUFFER(node_parent, int);
DYNAMIC_BUFFER(node_next, int);

DYNAMIC_BUFFER(node_center, float);
DYNAMIC_BUFFER(node_min, float);
DYNAMIC_BUFFER(node_max, float);
DYNAMIC_BUFFER(node_mass, float);
DYNAMIC_BUFFER(node_extent, float);
DYNAMIC_BUFFER(node_force, float);

DYNAMIC_BUFFER(points, float);
DYNAMIC_BUFFER(vel, float);
DYNAMIC_BUFFER(links, int);
DYNAMIC_BUFFER(indices, int);
DYNAMIC_BUFFER(sorted_morton, unsigned int);
DYNAMIC_BUFFER(morton_and_indices, uint64_t);

WASM_EXPORT("init")
void init(int max_point_n, int max_node_n, int max_link_n) {
    // ensure alignment of vector types
    max_point_n = (max_point_n + 3) & ~3;

    sorted_x_alloc(max_point_n/4);
    sorted_y_alloc(max_point_n/4);
    sorted_z_alloc(max_point_n/4);
    force_x_alloc(max_point_n/4);
    force_y_alloc(max_point_n/4);
    force_z_alloc(max_point_n/4);

    node_start_alloc(max_node_n);
    node_end_alloc(max_node_n);
    node_level_alloc(max_node_n);
    node_parent_alloc(max_node_n);
    node_next_alloc(max_node_n);

    node_center_alloc(max_node_n*3);
    node_min_alloc(max_node_n*3);
    node_max_alloc(max_node_n*3);
    node_mass_alloc(max_node_n);
    node_extent_alloc(max_node_n);
    node_force_alloc(max_node_n*3);

    points_alloc(max_point_n*3);
    vel_alloc(max_point_n*3);
    links_alloc(max_link_n);
    indices_alloc(max_point_n);
    sorted_morton_alloc(max_point_n);
    morton_and_indices_alloc(max_point_n);
}
BUFFER(tree_center, float, 3);

static float last_tree_extent = 0.0f;
WASM_EXPORT("get_tree_extent") float get_tree_extent() {return last_tree_extent;}

static inline unsigned int dilate3(unsigned int x) {
    x &= 0x3ff;
    x = (x | (x << 16)) & 0x30000ff;
    x = (x | (x << 8))  & 0x300f00f;
    x = (x | (x << 4))  & 0x30c30c3;
    x = (x | (x << 2))  & 0x9249249;
    return x;
}

static void quicksort_u64(unsigned long long* arr, int left, int right) {
    if (left >= right) return;
    unsigned long long pivot = arr[(left + right) / 2];
    int i = left, j = right;
    while (i <= j) {
        while (arr[i] < pivot) i++;
        while (arr[j] > pivot) j--;
        if (i <= j) {
            unsigned long long tmp = arr[i];
            arr[i] = arr[j];
            arr[j] = tmp;
            i++; j--;
        }
    }
    if (left < j) quicksort_u64(arr, left, j);
    if (i < right) quicksort_u64(arr, i, right);
}

static int _leafSize = 16;
static int _maxLevel = 10;
static int _nodeCount = 0;

static void _buildNode(int level, int start, int end, int parentIdx) {
    int ni = _nodeCount++;
    node_start[ni] = start;
    node_end[ni] = end;
    node_level[ni] = level;
    node_parent[ni] = parentIdx;

    if (end - start <= _leafSize || level >= _maxLevel) {
        node_next[ni] = _nodeCount;
        return;
    }

    int count[8] = {0,0,0,0,0,0,0,0};
    int shift = (_maxLevel - level - 1) * 3;
    for (int i = start; i < end; ++i) {
        int octant = (sorted_morton[i] >> shift) & 7;
        count[octant]++;
    }

    for (int i = 0; i < 8; i++) {
        if (count[i]) {
            _buildNode(level + 1, start, start + count[i], ni);
            start += count[i];
        }
    }
    node_next[ni] = _nodeCount;
}

WASM_EXPORT("buildOctree")
int buildOctree(int pointN, int leafSize, int maxLevel) {
    if (pointN <= 0) return 0;
    _leafSize = leafSize;
    _maxLevel = maxLevel;
    _nodeCount = 0;

    float minX = 1e30f, minY = 1e30f, minZ = 1e30f;
    float maxX = -1e30f, maxY = -1e30f, maxZ = -1e30f;
    for (int i = 0; i < pointN; i++) {
        float x = points[i*3], y = points[i*3+1], z = points[i*3+2];
        if (x < minX) minX = x; if (x > maxX) maxX = x;
        if (y < minY) minY = y; if (y > maxY) maxY = y;
        if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
    }

    float dx = maxX - minX, dy = maxY - minY, dz = maxZ - minZ;
    float extent = dx;
    if (dy > extent) extent = dy;
    if (dz > extent) extent = dz;
    last_tree_extent = extent;

    float centerX = (minX + maxX) * 0.5f;
    float centerY = (minY + maxY) * 0.5f;
    float centerZ = (minZ + maxZ) * 0.5f;
    tree_center[0] = centerX; tree_center[1] = centerY; tree_center[2] = centerZ;

    float loX = centerX - extent * 0.5f;
    float loY = centerY - extent * 0.5f;
    float loZ = centerZ - extent * 0.5f;

    float scale = 1023.0f / (extent + 1e-8f);

    for (int i = 0; i < pointN; i++) {
        unsigned int ix = (unsigned int)((points[i*3] - loX) * scale);
        unsigned int iy = (unsigned int)((points[i*3+1] - loY) * scale);
        unsigned int iz = (unsigned int)((points[i*3+2] - loZ) * scale);
        unsigned int code = dilate3(ix) | (dilate3(iy) << 1) | (dilate3(iz) << 2);
        morton_and_indices[i] = ((unsigned long long)code << 32) | i;
    }

    quicksort_u64(morton_and_indices, 0, pointN - 1);

    for (int i = 0; i < pointN; i++) {
        unsigned int code = (unsigned int)(morton_and_indices[i] >> 32);
        int idx = (int)(morton_and_indices[i] & 0xFFFFFFFF);
        sorted_morton[i] = code;
        indices[i] = idx;
        ((float*)sorted_x)[i] = points[idx*3];
        ((float*)sorted_y)[i] = points[idx*3+1];
        ((float*)sorted_z)[i] = points[idx*3+2];
    }

    _buildNode(0, 0, pointN, 0);

    return _nodeCount;
}


WASM_EXPORT("linkForce")
void linkForce(int linkN, float linkStrength, float linkDistance) {
    for (int p = 0; p < linkN * 2; p += 2) {
        int i = links[p], j = links[p+1];
        float dx = points[j*3]   + vel[j*3]   - points[i*3]   - vel[i*3];
        float dy = points[j*3+1] + vel[j*3+1] - points[i*3+1] - vel[i*3+1];
        float dz = points[j*3+2] + vel[j*3+2] - points[i*3+2] - vel[i*3+2];
        float l2 = dx*dx + dy*dy + dz*dz;
        if (l2 < 1.0f) l2 = 1.0f;
        float l = sqrtf(l2);
        float s = (l - linkDistance) / l * linkStrength;
        dx *= s; dy *= s; dz *= s;
        vel[j*3] -= dx; vel[j*3+1] -= dy; vel[j*3+2] -= dz;
        vel[i*3] += dx; vel[i*3+1] += dy; vel[i*3+2] += dz;
    }
}

WASM_EXPORT("updateNodes")
void updateNodes(int pointN, float velocityDecay) {
    for (int i = 0; i < pointN * 3; i++) {
        points[i] += vel[i];
        vel[i] *= velocityDecay;
    }
}

WASM_EXPORT("applyChargeForces")
void applyChargeForces(int pointN, float strength) {
    for (int i = 0; i < pointN; i++) {
        int k = indices[i];
        vel[k * 3]     += strength * ((float*)force_x)[i];
        vel[k * 3 + 1] += strength * ((float*)force_y)[i];
        vel[k * 3 + 2] += strength * ((float*)force_z)[i];
    }
}

WASM_EXPORT("accumPoints")
void accumPoints(int nodeN, float treeExtent) {
    for (int i=0; i<nodeN*3; ++i) {
        node_center[i] = 0.0f;
        node_min[i] = 1e30f;
        node_max[i] = -1e30f;
    }
    for (int ni=nodeN-1; ni>=0; --ni) {
        if (node_next[ni] == ni+1) { // leaf
            for (int i=node_start[ni]; i<node_end[ni]; ++i) {
                float px = ((float*)sorted_x)[i];
                float py = ((float*)sorted_y)[i];
                float pz = ((float*)sorted_z)[i];
                node_center[ni*3]   += px;
                node_center[ni*3+1] += py;
                node_center[ni*3+2] += pz;
                
                if (px < node_min[ni*3]) node_min[ni*3] = px;
                if (px > node_max[ni*3]) node_max[ni*3] = px;
                if (py < node_min[ni*3+1]) node_min[ni*3+1] = py;
                if (py > node_max[ni*3+1]) node_max[ni*3+1] = py;
                if (pz < node_min[ni*3+2]) node_min[ni*3+2] = pz;
                if (pz > node_max[ni*3+2]) node_max[ni*3+2] = pz;
            }
        } 
        int parent = node_parent[ni];
        if (parent == ni) continue;
        node_center[parent*3]   += node_center[ni*3];
        node_center[parent*3+1] += node_center[ni*3+1];
        node_center[parent*3+2] += node_center[ni*3+2];
        
        if (node_min[ni*3]   < node_min[parent*3])   node_min[parent*3]   = node_min[ni*3];
        if (node_max[ni*3]   > node_max[parent*3])   node_max[parent*3]   = node_max[ni*3];
        if (node_min[ni*3+1] < node_min[parent*3+1]) node_min[parent*3+1] = node_min[ni*3+1];
        if (node_max[ni*3+1] > node_max[parent*3+1]) node_max[parent*3+1] = node_max[ni*3+1];
        if (node_min[ni*3+2] < node_min[parent*3+2]) node_min[parent*3+2] = node_min[ni*3+2];
        if (node_max[ni*3+2] > node_max[parent*3+2]) node_max[parent*3+2] = node_max[ni*3+2];
    }
    for (int i=0; i<nodeN; ++i) {
        const float mass = (float)(node_end[i] - node_start[i]);
        node_mass[i] = mass;
        node_center[i*3]   /= mass;
        node_center[i*3+1] /= mass;
        node_center[i*3+2] /= mass;
        
        float dx = node_max[i*3] - node_min[i*3];
        float dy = node_max[i*3+1] - node_min[i*3+1];
        float dz = node_max[i*3+2] - node_min[i*3+2];
        float max_d = dx;
        if (dy > max_d) max_d = dy;
        if (dz > max_d) max_d = dz;
        node_extent[i] = max_d;
    }
}


WASM_EXPORT("calcMultibodyForce")
void calcMultibodyForce(int pointN, int nodeN, float maxDist) {
    const float theta2 = 0.81f; // Barnes-Hut theta squared
    const float maxDist2 = maxDist * maxDist;
    const float min_c = 1.0f / (1.0f + maxDist2);

    for (int pointI = 0; pointI < pointN; ++pointI) {
        const float x = ((float*)sorted_x)[pointI];
        const float y = ((float*)sorted_y)[pointI];
        const float z = ((float*)sorted_z)[pointI];
        float fx = 0.0f, fy = 0.0f, fz = 0.0f;

        for (int nodeI = 0; nodeI < nodeN;) {
            const float dx = node_center[nodeI * 3] - x;
            const float dy = node_center[nodeI * 3 + 1] - y;
            const float dz = node_center[nodeI * 3 + 2] - z;
            const float l2 = dx * dx + dy * dy + dz * dz;
            const float w = node_extent[nodeI];

            if (w * w < theta2 * l2) { // Far enough, treat as a single body (Barnes-Hut approximation)
                if (l2 < maxDist2) { // but not too far
                    const float mass = (float)(node_end[nodeI] - node_start[nodeI]); // mass is number of points
                    const float c = mass * (1.0f / (1.0f + l2) - min_c); // force magnitude
                    fx += c*dx; fy += c*dy; fz += c*dz;
                }
                nodeI = node_next[nodeI]; // Skip to next sibling or ancestor's sibling
            } else { // Too close, traverse children or individual points
                if (node_next[nodeI] == nodeI + 1 && l2 < maxDist2) { // It's a leaf node and not too far
                    for (int i = node_start[nodeI]; i < node_end[nodeI]; ++i) {
                        const float p_dx = ((float*)sorted_x)[i] - x;
                        const float p_dy = ((float*)sorted_y)[i] - y;
                        const float p_dz = ((float*)sorted_z)[i] - z;
                        const float p_l2 = p_dx * p_dx + p_dy * p_dy + p_dz * p_dz;
                        if (p_l2 < maxDist2) {
                            const float c = 1.0f / (1.0f + p_l2) - min_c;
                            fx += c * p_dx; fy += c * p_dy; fz += c * p_dz;
                        }
                    }
                }
                ++nodeI;
            }
        }
        ((float*)force_x)[pointI] = fx;
        ((float*)force_y)[pointI] = fy;
        ((float*)force_z)[pointI] = fz;
    }
}


WASM_EXPORT("calcMultibodyForceDual")
void calcMultibodyForceDual(int pointN, int nodeN, float maxDist) {
    const float theta2 = 0.81f; // Barnes-Hut theta squared
    const float maxDist2 = maxDist * maxDist;
    const v128_t v_maxDist2 = wasm_f32x4_splat(maxDist2);
    const v128_t v_one = wasm_f32x4_splat(1.0f);
    const v128_t v_zero = wasm_f32x4_splat(0.0f);
    const float min_c = 1.0f / (1.0f + maxDist2);
    const v128_t v_min_c = wasm_f32x4_splat(min_c);
    const v128_t v_lane_offsets = wasm_f32x4_make(0.0f, 1.0f, 2.0f, 3.0f);

    // Reset forces using SIMD
    int vN = (pointN + 3) / 4;
    for (int i = 0; i < vN; i++) {
        force_x[i] = force_y[i] = force_z[i] = v_zero;
    }

    int n3 = nodeN * 3;
    for (int i = 0; i < (n3 & ~3); i += 4) {
        wasm_v128_store(&node_force[i], v_zero);
    }
    for (int i = (n3 & ~3); i < n3; ++i) {
        node_force[i] = 0.0f;
    }

    if (nodeN <= 0) return;

    // Stack for dual tree traversal pairs
    typedef struct { int a; int b; } NodePair;
    enum { MAX_STACK = 256 };
    NodePair stack[MAX_STACK];
    int top = 0;

    // Start with root vs root
    stack[top++] = (NodePair){0, 0};

    while (top > 0) {
        NodePair pair = stack[--top];
        int niA = pair.a;
        int niB = pair.b;

        const float dx = node_center[niB * 3]     - node_center[niA * 3];
        const float dy = node_center[niB * 3 + 1] - node_center[niA * 3 + 1];
        const float dz = node_center[niB * 3 + 2] - node_center[niA * 3 + 2];
        const float l2 = dx * dx + dy * dy + dz * dz;

        const float wA = node_extent[niA];
        const float wB = node_extent[niB];
        const float combined_w = wA + wB;

        // Dual-tree Barnes-Hut criteria
        if (niA != niB && (combined_w * combined_w < theta2 * l2)) {
            if (l2 < maxDist2) {
                const float massA = (float)(node_end[niA] - node_start[niA]);
                const float massB = (float)(node_end[niB] - node_start[niB]);
                const float common = 1.0f / (1.0f + l2) - min_c;
                
                const float ca = massB * common;
                node_force[niA * 3]     += ca * dx;
                node_force[niA * 3 + 1] += ca * dy;
                node_force[niA * 3 + 2] += ca * dz;

                const float cb = massA * common;
                node_force[niB * 3]     -= cb * dx;
                node_force[niB * 3 + 1] -= cb * dy;
                node_force[niB * 3 + 2] -= cb * dz;
            }
        } else {
            const bool leafA = (node_next[niA] == niA + 1);
            const bool leafB = (node_next[niB] == niB + 1);

            if (leafA && leafB) {
                for (int i = node_start[niA]; i < node_end[niA]; ++i) {
                    const float ix = ((float*)sorted_x)[i];
                    const float iy = ((float*)sorted_y)[i];
                    const float iz = ((float*)sorted_z)[i];
                    const v128_t v_ix = wasm_f32x4_splat(ix);
                    const v128_t v_iy = wasm_f32x4_splat(iy);
                    const v128_t v_iz = wasm_f32x4_splat(iz);

                    v128_t v_ifx = v_zero;
                    v128_t v_ify = v_zero;
                    v128_t v_ifz = v_zero;

                    int j_start = (niA == niB) ? i + 1 : node_start[niB];
                    int j_end = node_end[niB];

                    int vj_start = j_start >> 2;
                    int vj_end = (j_end + 3) >> 2;

                    v128_t v_j_start = wasm_f32x4_splat((float)j_start);
                    v128_t v_j_end = wasm_f32x4_splat((float)j_end);

                    for (int vj = vj_start; vj < vj_end; ++vj) {
                        v128_t v_jx = sorted_x[vj];
                        v128_t v_jy = sorted_y[vj];
                        v128_t v_jz = sorted_z[vj];

                        v128_t v_dx = wasm_f32x4_sub(v_jx, v_ix);
                        v128_t v_dy = wasm_f32x4_sub(v_jy, v_iy);
                        v128_t v_dz = wasm_f32x4_sub(v_jz, v_iz);

                        v128_t v_l2 = wasm_f32x4_add(wasm_f32x4_mul(v_dx, v_dx),
                                      wasm_f32x4_add(wasm_f32x4_mul(v_dy, v_dy),
                                                     wasm_f32x4_mul(v_dz, v_dz)));

                        v128_t v_j_idx = wasm_f32x4_add(wasm_f32x4_splat((float)(vj * 4)), v_lane_offsets);
                        v128_t v_range_mask = wasm_v128_and(
                            wasm_f32x4_ge(v_j_idx, v_j_start),
                            wasm_f32x4_lt(v_j_idx, v_j_end)
                        );

                        v128_t v_dist_mask = wasm_f32x4_lt(v_l2, v_maxDist2);
                        v128_t v_mask = wasm_v128_and(v_range_mask, v_dist_mask);

                        v128_t v_c = wasm_f32x4_div(v_one, wasm_f32x4_add(v_one, v_l2));
                        v_c = wasm_f32x4_sub(v_c, v_min_c);
                        v_c = wasm_v128_and(v_c, v_mask);

                        v128_t v_dfx = wasm_f32x4_mul(v_c, v_dx);
                        v128_t v_dfy = wasm_f32x4_mul(v_c, v_dy);
                        v128_t v_dfz = wasm_f32x4_mul(v_c, v_dz);

                        v_ifx = wasm_f32x4_add(v_ifx, v_dfx);
                        v_ify = wasm_f32x4_add(v_ify, v_dfy);
                        v_ifz = wasm_f32x4_add(v_ifz, v_dfz);

                        force_x[vj] = wasm_f32x4_sub(force_x[vj], v_dfx);
                        force_y[vj] = wasm_f32x4_sub(force_y[vj], v_dfy);
                        force_z[vj] = wasm_f32x4_sub(force_z[vj], v_dfz);
                    }

                    // Horizontal sum for point i
                    ((float*)force_x)[i] += wasm_f32x4_hsum(v_ifx);
                    ((float*)force_y)[i] += wasm_f32x4_hsum(v_ify);
                    ((float*)force_z)[i] += wasm_f32x4_hsum(v_ifz);
                }
            } else if (niA == niB) {
                // Self-interaction: recurse on unique children pairs
                for (int m = niA + 1; m < node_next[niA]; m = node_next[m]) {
                    for (int n = m; n < node_next[niA]; n = node_next[n]) {
                        if (top < MAX_STACK) stack[top++] = (NodePair){m, n};
                    }
                }
            } else if (!leafA && (leafB || wA >= wB)) {
                // Split larger node A
                for (int m = niA + 1; m < node_next[niA]; m = node_next[m]) {
                    if (top < MAX_STACK) stack[top++] = (NodePair){m, niB};
                }
            } else {
                // Split larger node B
                for (int m = niB + 1; m < node_next[niB]; m = node_next[m]) {
                    if (top < MAX_STACK) stack[top++] = (NodePair){niA, m};
                }
            }
        }
    }

    // Downward pass: propagate node-level forces to children
    for (int ni = 1; ni < nodeN; ++ni) {
        int p = node_parent[ni];
        if (p == ni) continue;
        node_force[ni * 3]     += node_force[p * 3];
        node_force[ni * 3 + 1] += node_force[p * 3 + 1];
        node_force[ni * 3 + 2] += node_force[p * 3 + 2];
    }

    // Add accumulated node forces to point forces
    for (int ni = 0; ni < nodeN; ++ni) {
        if (node_next[ni] == ni + 1) { // leaf
            const float fx = node_force[ni * 3];
            const float fy = node_force[ni * 3 + 1];
            const float fz = node_force[ni * 3 + 2];
            const v128_t v_fx = wasm_f32x4_splat(fx);
            const v128_t v_fy = wasm_f32x4_splat(fy);
            const v128_t v_fz = wasm_f32x4_splat(fz);

            int i = node_start[ni];
            int i_end = node_end[ni];
            for (; i + 3 < i_end; i += 4) {
                int vi = i / 4;
                force_x[vi] = wasm_f32x4_add(force_x[vi], v_fx);
                force_y[vi] = wasm_f32x4_add(force_y[vi], v_fy);
                force_z[vi] = wasm_f32x4_add(force_z[vi], v_fz);
            }
            for (; i < i_end; ++i) {
                ((float*)force_x)[i] += fx; ((float*)force_y)[i] += fy; ((float*)force_z)[i] += fz;
            }
        }
    }
}
