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


const int max_point_n = 1<<16;
const int max_node_n = 1<<16;

BUFFER(points, float, max_point_n*3);
BUFFER(forces, float, max_point_n*3);

BUFFER(node_start, int, max_node_n);
BUFFER(node_end, int, max_node_n);
BUFFER(node_next, int, max_node_n);
BUFFER(node_center_size, float, max_node_n*4);

WASM_EXPORT("calcMultibodyForce")
void calcMultibodyForce(int pointN, int nodeN, float maxDist) {
    const float theta2 = 0.81f; // Barnes-Hut theta squared
    const float maxDist2 = maxDist * maxDist;

    for (int pointI = 0; pointI < pointN; ++pointI) {
        const float x = points[pointI * 3];
        const float y = points[pointI * 3 + 1];
        const float z = points[pointI * 3 + 2];
        float fx = 0.0f, fy = 0.0f, fz = 0.0f;

        for (int nodeI = 0; nodeI < nodeN;) {
            const float dx = node_center_size[nodeI * 4] - x;
            const float dy = node_center_size[nodeI * 4 + 1] - y;
            const float dz = node_center_size[nodeI * 4 + 2] - z;
            const float l2 = dx * dx + dy * dy + dz * dz;
            const float w = node_center_size[nodeI * 4 + 3];

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
                        const float p_dx = points[i * 3] - x;
                        const float p_dy = points[i * 3 + 1] - y;
                        const float p_dz = points[i * 3 + 2] - z;
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