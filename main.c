#include <wasm_simd128.h>
#ifdef WASM
#define sqrtf __builtin_sqrtf
#endif

#ifdef WASM
    #define WASM_EXPORT(name) __attribute__((export_name(name)))
    void * alloc(int size);
#else
    #define WASM_EXPORT(name)
#endif

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

#define BUFFER(name, type, size) type name[size]; \
  WASM_EXPORT("_get_"#name) type* get_##name() {return name;} \
  WASM_EXPORT("_len_"#name"__"#type) int get_##name##_len() {return (size);}

#define DYNAMIC_BUFFER(name, type) type * name = NULL; int name##_len = 0; \
  WASM_EXPORT("_get_"#name) type* get_##name() {return name;} \
  WASM_EXPORT("_len_"#name"__"#type) int get_##name##_len() {return (name##_len);} \
  void name##_alloc(int size) {name = (type*)alloc(size*sizeof(type)); name##_len = size;}


#define max_point_n (1<<16)
#define max_node_n (1<<16)

BUFFER(sorted_points, float, max_point_n*3);
BUFFER(forces, float, max_point_n*3);

BUFFER(node_start, int, max_node_n);
BUFFER(node_end, int, max_node_n);
BUFFER(node_level, int, max_node_n);
BUFFER(node_parent, int, max_node_n);
BUFFER(node_next, int, max_node_n);

BUFFER(node_center, float, max_node_n*3);
BUFFER(node_extent, float, max_node_n);
BUFFER(node_force, float, max_node_n*3);

BUFFER(points, float, max_point_n*3);
BUFFER(vel, float, max_point_n*3);
BUFFER(links, int, max_point_n*8); // Assuming max 4 links per node on average
BUFFER(indices, int, max_point_n);
BUFFER(sorted_morton, unsigned int, max_point_n);
BUFFER(morton_and_indices, uint64_t, max_point_n);
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
        sorted_points[i*3]   = points[idx*3];
        sorted_points[i*3+1] = points[idx*3+1];
        sorted_points[i*3+2] = points[idx*3+2];
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
        vel[k * 3]     += strength * forces[i * 3];
        vel[k * 3 + 1] += strength * forces[i * 3 + 1];
        vel[k * 3 + 2] += strength * forces[i * 3 + 2];
    }
}


// WASM_EXPORT("add_vectors")
// void add_vectors(const float* a, const float* b, float* result, int len) {
//     const v128_t * va = (const v128_t *)a;
//     const v128_t * vb = (const v128_t *)b;
//     v128_t * vr = (v128_t *)result;
//     for (int i=0; i<len; ++i) {
//         vr[i] = wasm_f32x4_add(va[i], vb[i]);
//     }
// }

WASM_EXPORT("accumPoints")
void accumPoints(int nodeN, float treeExtent) {
    for (int i=0; i<nodeN*3; ++i) {
        node_center[i] = 0.0f;
    }
    for (int ni=nodeN-1; ni>=0; --ni) {
        if (node_next[ni] == ni+1) { // leaf
            for (int i=node_start[ni]; i<node_end[ni]; ++i) {
                node_center[ni*3]   += sorted_points[i*3];
                node_center[ni*3+1] += sorted_points[i*3+1];
                node_center[ni*3+2] += sorted_points[i*3+2];
            }
        } 
        int parent = node_parent[ni];
        if (parent == ni) continue;
        node_center[parent*3]   += node_center[ni*3];
        node_center[parent*3+1] += node_center[ni*3+1];
        node_center[parent*3+2] += node_center[ni*3+2];
    }
    for (int i=0; i<nodeN; ++i) {
        const float mass = (float)(node_end[i] - node_start[i]);
        node_center[i*3]   /= mass;
        node_center[i*3+1] /= mass;
        node_center[i*3+2] /= mass;
        node_extent[i] = treeExtent / (1<<node_level[i]);
    }
}


WASM_EXPORT("calcMultibodyForce")
void calcMultibodyForce(int pointN, int nodeN, float maxDist) {
    const float theta2 = 0.81f; // Barnes-Hut theta squared
    const float maxDist2 = maxDist * maxDist;

    for (int pointI = 0; pointI < pointN; ++pointI) {
        const float x = sorted_points[pointI * 3];
        const float y = sorted_points[pointI * 3 + 1];
        const float z = sorted_points[pointI * 3 + 2];
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
                    const float c = mass / (1.0f + l2); // force magnitude
                    fx += c*dx; fy += c*dy; fz += c*dz;
                }
                nodeI = node_next[nodeI]; // Skip to next sibling or ancestor's sibling
            } else { // Too close, traverse children or individual points
                if (node_next[nodeI] == nodeI + 1 && l2 < maxDist2) { // It's a leaf node and not too far
                    for (int i = node_start[nodeI]; i < node_end[nodeI]; ++i) {
                        const float p_dx = sorted_points[i * 3] - x;
                        const float p_dy = sorted_points[i * 3 + 1] - y;
                        const float p_dz = sorted_points[i * 3 + 2] - z;
                        const float p_l2 = p_dx * p_dx + p_dy * p_dy + p_dz * p_dz;
                        const float c = 1.0f / (1.0f + p_l2);
                        fx += c * p_dx; fy += c * p_dy; fz += c * p_dz;
                    }
                }
                ++nodeI;
            }
        }
        forces[pointI * 3]     = fx;
        forces[pointI * 3 + 1] = fy;
        forces[pointI * 3 + 2] = fz;
    }
}


WASM_EXPORT("calcMultibodyForceDual")
void calcMultibodyForceDual(int pointN, int nodeN, float maxDist) {
    const float theta2 = 0.81f; // Barnes-Hut theta squared
    const float maxDist2 = maxDist * maxDist;

    // Reset forces
    for (int i = 0; i < pointN * 3; ++i) {
        forces[i] = 0.0f;
    }
    for (int i = 0; i < nodeN * 3; ++i) {
        node_force[i] = 0.0f;
    }

    if (nodeN <= 0) return;

    // Stack for dual tree traversal pairs
    typedef struct { int a; int b; } NodePair;
    NodePair stack[4096];
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
                const float common = 1.0f / (1.0f + l2);
                
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
                    const float ix = sorted_points[i * 3];
                    const float iy = sorted_points[i * 3 + 1];
                    const float iz = sorted_points[i * 3 + 2];
                    const int jStart = (niA == niB) ? i + 1 : node_start[niB];
                    for (int j = jStart; j < node_end[niB]; ++j) {
                        const float pdx = sorted_points[j * 3]     - ix;
                        const float pdy = sorted_points[j * 3 + 1] - iy;
                        const float pdz = sorted_points[j * 3 + 2] - iz;
                        const float pl2 = pdx * pdx + pdy * pdy + pdz * pdz;
                        if (pl2 < maxDist2) {
                            const float c = 1.0f / (1.0f + pl2);
                            forces[i * 3]     += c * pdx;
                            forces[i * 3 + 1] += c * pdy;
                            forces[i * 3 + 2] += c * pdz;
                            forces[j * 3]     -= c * pdx;
                            forces[j * 3 + 1] -= c * pdy;
                            forces[j * 3 + 2] -= c * pdz;
                        }
                    }
                }
            } else if (niA == niB) {
                // Self-interaction: recurse on unique children pairs
                int childI = niA + 1;
                while (childI < node_next[niA]) {
                    if (top < 4096) stack[top++] = (NodePair){childI, childI};
                    int childJ = node_next[childI];
                    while (childJ < node_next[niA]) {
                        if (top < 4096) stack[top++] = (NodePair){childI, childJ};
                        childJ = node_next[childJ];
                    }
                    childI = node_next[childI];
                }
            } else if (!leafA && (leafB || wA > wB)) {
                // Split larger node A
                int childA = niA + 1;
                while (childA < node_next[niA]) {
                    if (top < 4096) stack[top++] = (NodePair){childA, niB};
                    childA = node_next[childA];
                }
            } else {
                // Split larger node B
                int childB = niB + 1;
                while (childB < node_next[niB]) {
                    if (top < 4096) stack[top++] = (NodePair){niA, childB};
                    childB = node_next[childB];
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
            for (int i = node_start[ni]; i < node_end[ni]; ++i) {
                forces[i * 3]     += fx;
                forces[i * 3 + 1] += fy;
                forces[i * 3 + 2] += fz;
            }
        }
    }
}