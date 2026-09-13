/*
 * VP8L (WebP Lossless) decoder for RinOS
 * Spec: https://developers.google.com/speed/webp/docs/webp_lossless_bitstream_spec
 */

#include <stdint.h>
#include <stddef.h>

/* Freestanding: declare libc functions manually to avoid C++ header conflicts */
extern void* malloc(size_t);
extern void  free(void*);
extern void* memset(void*, int, size_t);
extern void* memcpy(void*, const void*, size_t);
extern int snprintf(char*, size_t, const char*, ...);

/* ══════════════════════════════════════════════════════════════════════
 *  Constants
 * ══════════════════════════════════════════════════════════════════════ */

#define VP8L_MAX_CODE_LENGTH    15
#define VP8L_HTREE_TABLE_BITS   8
#define VP8L_LENGTHS_TABLE_BITS 7
#define VP8L_HTREE_TABLE_SIZE   (1 << VP8L_HTREE_TABLE_BITS)
#define VP8L_NUM_CODE_TYPES     5
#define VP8L_MAX_TRANSFORMS     4

#define VP8L_GREEN_ALPHA_SIZE   256
#define VP8L_RED_SIZE           256
#define VP8L_BLUE_SIZE          256
#define VP8L_ALPHA_SIZE         256
#define VP8L_NUM_DISTANCE_CODES 40

#define VP8L_TRANSFORM_PREDICTOR      0
#define VP8L_TRANSFORM_CROSS_COLOR    1
#define VP8L_TRANSFORM_SUBTRACT_GREEN 2
#define VP8L_TRANSFORM_COLOR_INDEXING 3

#define VP8L_MAX_CACHE_BITS     11
#define VP8L_MAX_NUM_HTREE_GROUPS 65536

/* code length code order */
static const int kCodeLengthOrder[19] = {
    17, 18, 0, 1, 2, 3, 4, 5, 16, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15
};

/* Spec plane-code remapping for short backward distances 1..120. */
static const uint8_t kCodeToPlane[120] = {
    0x18, 0x07, 0x17, 0x19, 0x28, 0x06, 0x27, 0x29, 0x16, 0x1a, 0x26, 0x2a,
    0x38, 0x05, 0x37, 0x39, 0x15, 0x1b, 0x36, 0x3a, 0x25, 0x2b, 0x48, 0x04,
    0x47, 0x49, 0x14, 0x1c, 0x35, 0x3b, 0x46, 0x4a, 0x24, 0x2c, 0x58, 0x45,
    0x4b, 0x34, 0x3c, 0x03, 0x57, 0x59, 0x13, 0x1d, 0x56, 0x5a, 0x23, 0x2d,
    0x44, 0x4c, 0x55, 0x5b, 0x33, 0x3d, 0x68, 0x02, 0x67, 0x69, 0x12, 0x1e,
    0x66, 0x6a, 0x22, 0x2e, 0x54, 0x5c, 0x43, 0x4d, 0x65, 0x6b, 0x32, 0x3e,
    0x78, 0x01, 0x77, 0x79, 0x53, 0x5d, 0x11, 0x1f, 0x64, 0x6c, 0x42, 0x4e,
    0x76, 0x7a, 0x21, 0x2f, 0x75, 0x7b, 0x31, 0x3f, 0x63, 0x6d, 0x52, 0x5e,
    0x00, 0x74, 0x7c, 0x41, 0x4f, 0x10, 0x20, 0x62, 0x6e, 0x30, 0x73, 0x7d,
    0x51, 0x5f, 0x40, 0x72, 0x7e, 0x61, 0x6f, 0x50, 0x71, 0x7f, 0x60, 0x70
};

/* ══════════════════════════════════════════════════════════════════════
 *  Bit Reader (LSB-first)
 * ══════════════════════════════════════════════════════════════════════ */

typedef struct {
    const uint8_t* data;
    size_t         size;
    size_t         byte_pos;
    uint64_t       val;
    int            bits;
    int            eos;
} VP8LBitReader;

static const char* g_vp8l_last_stage = "idle";
static const char* g_vp8l_huff_context = NULL;
static int g_vp8l_last_pos = -1;
static int g_vp8l_last_dist = -1;
static int g_vp8l_last_length = -1;
static int g_vp8l_last_width = -1;
static int g_vp8l_last_dist_symbol = -1;
static int g_vp8l_last_huff_simple = -1;
static int g_vp8l_last_num_cl_codes = -1;
static int g_vp8l_last_max_symbol = -1;
static int g_vp8l_last_cl_lengths[19];
static int g_vp8l_last_transform_type = -1;
static int g_vp8l_last_transform_bits = -1;
static int g_vp8l_last_sub_cache_flag = -1;
static int g_vp8l_last_sub_cache_bits = -1;
static int g_vp8l_last_transform_index = -1;
static char g_vp8l_last_success_context[96] = "none";
static int g_vp8l_last_success_alphabet_size = -1;
static int g_vp8l_last_success_simple = -1;
static int g_vp8l_last_success_num_cl_codes = -1;
static int g_vp8l_last_success_max_symbol = -1;
static int g_vp8l_last_success_start_bit_pos = -1;
static int g_vp8l_last_success_end_bit_pos = -1;

static void vp8l_set_stage(const char* stage) {
    g_vp8l_last_stage = stage ? stage : "unknown";
}

static void vp8l_set_stage_with_prefix(const char* prefix, const char* suffix) {
    static char stage_buf[96];
    size_t pos = 0;
    size_t i = 0;
    if (!prefix || !*prefix) {
        vp8l_set_stage(suffix);
        return;
    }
    while (prefix[i] != '\0' && pos + 1 < sizeof(stage_buf)) {
        stage_buf[pos++] = prefix[i++];
    }
    if (pos + 1 < sizeof(stage_buf)) {
        stage_buf[pos++] = '-';
    }
    i = 0;
    while (suffix && suffix[i] != '\0' && pos + 1 < sizeof(stage_buf)) {
        stage_buf[pos++] = suffix[i++];
    }
    stage_buf[pos] = '\0';
    vp8l_set_stage(stage_buf);
}

static void vp8l_set_huff_context(const char* context) {
    g_vp8l_huff_context = context;
}

static void vp8l_set_huff_stage(const char* suffix) {
    if (g_vp8l_huff_context && *g_vp8l_huff_context) {
        vp8l_set_stage_with_prefix(g_vp8l_huff_context, suffix);
    } else {
        vp8l_set_stage(suffix);
    }
}

const char* vp8l_get_last_stage(void) {
    return g_vp8l_last_stage;
}

int vp8l_get_last_pos(void) { return g_vp8l_last_pos; }
int vp8l_get_last_dist(void) { return g_vp8l_last_dist; }
int vp8l_get_last_length(void) { return g_vp8l_last_length; }
int vp8l_get_last_width(void) { return g_vp8l_last_width; }
int vp8l_get_last_dist_symbol(void) { return g_vp8l_last_dist_symbol; }
int vp8l_get_last_huff_simple(void) { return g_vp8l_last_huff_simple; }
int vp8l_get_last_num_cl_codes(void) { return g_vp8l_last_num_cl_codes; }
int vp8l_get_last_max_symbol(void) { return g_vp8l_last_max_symbol; }
const int* vp8l_get_last_cl_lengths(void) { return g_vp8l_last_cl_lengths; }
int vp8l_get_last_transform_type(void) { return g_vp8l_last_transform_type; }
int vp8l_get_last_transform_bits(void) { return g_vp8l_last_transform_bits; }
int vp8l_get_last_sub_cache_flag(void) { return g_vp8l_last_sub_cache_flag; }
int vp8l_get_last_sub_cache_bits(void) { return g_vp8l_last_sub_cache_bits; }
int vp8l_get_last_transform_index(void) { return g_vp8l_last_transform_index; }
const char* vp8l_get_last_success_context(void) { return g_vp8l_last_success_context; }
int vp8l_get_last_success_alphabet_size(void) { return g_vp8l_last_success_alphabet_size; }
int vp8l_get_last_success_simple(void) { return g_vp8l_last_success_simple; }
int vp8l_get_last_success_num_cl_codes(void) { return g_vp8l_last_success_num_cl_codes; }
int vp8l_get_last_success_max_symbol(void) { return g_vp8l_last_success_max_symbol; }
int vp8l_get_last_success_start_bit_pos(void) { return g_vp8l_last_success_start_bit_pos; }
int vp8l_get_last_success_end_bit_pos(void) { return g_vp8l_last_success_end_bit_pos; }

static void vp8l_br_init(VP8LBitReader* br, const uint8_t* data, size_t size) {
    br->data = data;
    br->size = size;
    br->byte_pos = 0;
    br->val = 0;
    br->bits = 0;
    br->eos = 0;
}

static inline void vp8l_br_fill(VP8LBitReader* br) {
    while (br->bits < 56 && br->byte_pos < br->size) {
        br->val |= (uint64_t)br->data[br->byte_pos++] << br->bits;
        br->bits += 8;
    }
    if (br->byte_pos >= br->size && br->bits == 0) br->eos = 1;
}

static inline int vp8l_br_ensure(VP8LBitReader* br, int n) {
    if (n < 0 || n > 31) {
        br->eos = 1;
        return 0;
    }
    if (n == 0) return 1;
    if (br->bits < n) vp8l_br_fill(br);
    if (br->bits < n) {
        br->eos = 1;
        return 0;
    }
    return 1;
}

static inline uint32_t vp8l_br_read(VP8LBitReader* br, int n) {
    uint32_t result;
    if (n == 0) return 0;
    if (!vp8l_br_ensure(br, n)) return 0;
    result = (uint32_t)(br->val & (((uint64_t)1 << n) - 1));
    br->val >>= n;
    br->bits -= n;
    return result;
}

static inline uint32_t vp8l_br_peek(VP8LBitReader* br, int n) {
    if (!vp8l_br_ensure(br, n)) return 0;
    return (uint32_t)(br->val & (((uint64_t)1 << n) - 1));
}

static inline void vp8l_br_advance(VP8LBitReader* br, int n) {
    if (!vp8l_br_ensure(br, n)) return;
    br->val >>= n;
    br->bits -= n;
}

/* ══════════════════════════════════════════════════════════════════════
 *  Huffman Table
 * ══════════════════════════════════════════════════════════════════════ */

typedef struct {
    uint8_t  bits;   /* number of bits consumed (0 = second-level redirect) */
    uint16_t value;  /* symbol value, or offset to second-level table */
} VP8LHuffEntry;

typedef struct {
    VP8LHuffEntry* table;
    int            total_size;  /* total entries (root + secondary) */
    int            single_symbol;
    uint16_t       single_symbol_value;
    uint8_t        single_symbol_bits;
    uint8_t*       code_lengths;
    uint32_t*      codes;
    int            alphabet_size;
    int            max_code_length;
    int            root_bits;
} VP8LHuffTree;

/* Returns reverse(reverse(key, len) + 1, len) in upstream libwebp terms. */
static uint32_t vp8l_get_next_key(uint32_t key, int len) {
    uint32_t step = 1u << (len - 1);
    while (key & step) {
        step >>= 1;
    }
    return step ? ((key & (step - 1u)) + step) : key;
}

static uint32_t vp8l_reverse_bits(uint32_t bits, int n) {
    uint32_t out = 0;
    int i;
    for (i = 0; i < n; ++i) {
        out = (out << 1) | (bits & 1u);
        bits >>= 1;
    }
    return out;
}

static void vp8l_replicate_value(VP8LHuffEntry* table, int step, int end,
                                 VP8LHuffEntry code) {
    do {
        end -= step;
        table[end] = code;
    } while (end > 0);
}

static int vp8l_next_table_bit_size(const int* count, int len, int root_bits) {
    int left = 1 << (len - root_bits);
    while (len < VP8L_MAX_CODE_LENGTH) {
        left -= count[len];
        if (left <= 0) break;
        ++len;
        left <<= 1;
    }
    return len - root_bits;
}

static size_t vp8l_huff_table_capacity(int alphabet_size, int root_bits) {
    int extra_bits = VP8L_MAX_CODE_LENGTH - root_bits;
    size_t cap = (size_t)1u << root_bits;
    if (extra_bits < 0) {
        extra_bits = 0;
    }
    cap += (size_t)alphabet_size << extra_bits;
    return cap;
}

static int vp8l_build_lookup_table(VP8LHuffEntry* root_table, int root_bits,
                                   const int* code_lengths,
                                   int code_lengths_size,
                                   uint16_t* sorted) {
    int count[VP8L_MAX_CODE_LENGTH + 1];
    int offset[VP8L_MAX_CODE_LENGTH + 1];
    VP8LHuffEntry* table = root_table;
    int total_size = 1 << root_bits;
    int table_bits = root_bits;
    int table_size = 1 << table_bits;
    int symbol;
    int len;
    uint32_t key = 0;
    uint32_t low = 0xffffffffu;
    int num_nodes = 1;
    int num_open = 1;

    memset(count, 0, sizeof(count));
    for (symbol = 0; symbol < code_lengths_size; ++symbol) {
        int cl = code_lengths[symbol];
        if (cl < 0 || cl > VP8L_MAX_CODE_LENGTH) {
            vp8l_set_huff_stage("huff-build-invalid-length");
            return 0;
        }
        ++count[cl];
    }
    if (count[0] == code_lengths_size) {
        vp8l_set_huff_stage("huff-build-all-zero");
        return 0;
    }

    offset[0] = 0;
    offset[1] = 0;
    for (len = 1; len < VP8L_MAX_CODE_LENGTH; ++len) {
        if (count[len] > (1 << len)) {
            vp8l_set_huff_stage("huff-build-overfull");
            return 0;
        }
        offset[len + 1] = offset[len] + count[len];
    }

    for (symbol = 0; symbol < code_lengths_size; ++symbol) {
        int cl = code_lengths[symbol];
        if (cl > 0) {
            if (sorted != NULL) {
                sorted[offset[cl]++] = (uint16_t)symbol;
            } else {
                offset[cl]++;
            }
        }
    }

    if (offset[VP8L_MAX_CODE_LENGTH] == 1) {
        if (root_table != NULL) {
            VP8LHuffEntry code;
            code.bits = 0;
            code.value = sorted[0];
            vp8l_replicate_value(root_table, 1, total_size, code);
        }
        return total_size;
    }

    symbol = 0;
    for (len = 1; len <= root_bits; ++len) {
        int step = 1 << len;
        VP8LHuffEntry code;
        num_open <<= 1;
        num_nodes += num_open;
        num_open -= count[len];
        if (num_open < 0) {
            vp8l_set_huff_stage("huff-build-root-open");
            return 0;
        }
        code.bits = (uint8_t)len;
        for (; count[len] > 0; --count[len]) {
            if (root_table != NULL) {
                code.value = sorted[symbol++];
                vp8l_replicate_value(&table[key], step, table_size, code);
            }
            key = vp8l_get_next_key(key, len);
        }
    }

    {
        int step = 2;
        for (len = root_bits + 1; len <= VP8L_MAX_CODE_LENGTH; ++len, step <<= 1) {
            VP8LHuffEntry code;
            num_open <<= 1;
            num_nodes += num_open;
            num_open -= count[len];
            if (num_open < 0) {
                vp8l_set_huff_stage("huff-build-sub-open");
                return 0;
            }
            for (; count[len] > 0; --count[len]) {
                if ((key & ((1u << root_bits) - 1u)) != low) {
                    table_bits = vp8l_next_table_bit_size(count, len, root_bits);
                    table_size = 1 << table_bits;
                    total_size += table_size;
                    low = key & ((1u << root_bits) - 1u);
                    if (root_table != NULL) {
                        table = root_table + (total_size - table_size);
                        root_table[low].bits = (uint8_t)(table_bits + root_bits);
                        root_table[low].value =
                            (uint16_t)((table - root_table) - (int)low);
                    }
                }
                if (root_table != NULL) {
                    code.bits = (uint8_t)(len - root_bits);
                    code.value = sorted[symbol++];
                    vp8l_replicate_value(&table[key >> root_bits], step, table_size, code);
                }
                key = vp8l_get_next_key(key, len);
            }
        }
    }

    return total_size;
}

static int vp8l_huff_build_with_root_bits(VP8LHuffTree* tree,
                                          const int* code_lengths,
                                          int alphabet_size,
                                          int root_bits) {
    uint16_t* sorted = NULL;
    int total_size = 0;
    int symbol;
    int non_zero = 0;

    if (!tree || !code_lengths || alphabet_size <= 0) return 0;
    if (tree->table) {
        free(tree->table);
        tree->table = NULL;
    }
    if (tree->code_lengths) {
        free(tree->code_lengths);
        tree->code_lengths = NULL;
    }
    if (tree->codes) {
        free(tree->codes);
        tree->codes = NULL;
    }
    tree->total_size = 0;
    tree->single_symbol = 0;
    tree->single_symbol_value = 0;
    tree->single_symbol_bits = 0;
    tree->alphabet_size = alphabet_size;
    tree->max_code_length = 0;
    tree->root_bits = root_bits;

    for (symbol = 0; symbol < alphabet_size; ++symbol) {
        int cl = code_lengths[symbol];
        if (cl < 0 || cl > VP8L_MAX_CODE_LENGTH) {
            vp8l_set_huff_stage("huff-build-invalid-length");
            goto fail;
        }
        if (cl > 0) {
            ++non_zero;
        }
    }
    if (non_zero == 0) {
        vp8l_set_huff_stage("huff-build-all-zero");
        goto fail;
    }

    tree->table = (VP8LHuffEntry*)malloc(
        vp8l_huff_table_capacity(alphabet_size, root_bits) * sizeof(VP8LHuffEntry));
    sorted = (uint16_t*)malloc((size_t)alphabet_size * sizeof(uint16_t));
    if (!tree->table || !sorted) goto fail;
    memset(tree->table, 0,
           vp8l_huff_table_capacity(alphabet_size, root_bits) * sizeof(VP8LHuffEntry));

    total_size = vp8l_build_lookup_table(tree->table, root_bits,
                                         code_lengths, alphabet_size, sorted);
    if (total_size <= 0) {
        goto fail;
    }

    tree->code_lengths = (uint8_t*)malloc((size_t)alphabet_size * sizeof(uint8_t));
    tree->codes = (uint32_t*)malloc((size_t)alphabet_size * sizeof(uint32_t));
    if (!tree->code_lengths || !tree->codes) {
        goto fail;
    }
    {
        int count[VP8L_MAX_CODE_LENGTH + 1];
        uint32_t next_code[VP8L_MAX_CODE_LENGTH + 1];
        uint32_t code = 0;
        int len;
        memset(count, 0, sizeof(count));
        memset(next_code, 0, sizeof(next_code));
        tree->max_code_length = 0;
        for (symbol = 0; symbol < alphabet_size; ++symbol) {
            int cl = code_lengths[symbol];
            tree->code_lengths[symbol] = (uint8_t)cl;
            tree->codes[symbol] = 0;
            if (cl > 0) {
                ++count[cl];
                if (cl > tree->max_code_length) {
                    tree->max_code_length = cl;
                }
            }
        }
        for (len = 1; len <= VP8L_MAX_CODE_LENGTH; ++len) {
            code = (code + (uint32_t)count[len - 1]) << 1;
            next_code[len] = code;
        }
        for (symbol = 0; symbol < alphabet_size; ++symbol) {
            int cl = code_lengths[symbol];
            if (cl > 0) {
                tree->codes[symbol] = vp8l_reverse_bits(next_code[cl], cl);
                next_code[cl]++;
            }
        }
    }

    if (non_zero == 1) {
        for (symbol = 0; symbol < alphabet_size; ++symbol) {
            if (code_lengths[symbol] > 0) {
                tree->single_symbol = 1;
                tree->single_symbol_value = (uint16_t)symbol;
                tree->single_symbol_bits = 0;
                break;
            }
        }
    }

    tree->total_size = total_size;
    free(sorted);
    return 1;

fail:
    if (sorted) free(sorted);
    if (tree->table) {
        free(tree->table);
        tree->table = NULL;
    }
    if (tree->code_lengths) {
        free(tree->code_lengths);
        tree->code_lengths = NULL;
    }
    if (tree->codes) {
        free(tree->codes);
        tree->codes = NULL;
    }
    tree->total_size = 0;
    tree->single_symbol = 0;
    tree->single_symbol_value = 0;
    tree->single_symbol_bits = 0;
    tree->alphabet_size = 0;
    tree->max_code_length = 0;
    tree->root_bits = 0;
    return 0;
}

static int vp8l_huff_build(VP8LHuffTree* tree, const int* code_lengths,
                           int alphabet_size) {
    return vp8l_huff_build_with_root_bits(tree, code_lengths, alphabet_size,
                                          VP8L_HTREE_TABLE_BITS);
}

static inline int vp8l_huff_read(VP8LBitReader* br, const VP8LHuffTree* tree) {
    const int root_bits =
        (tree && tree->root_bits > 0) ? tree->root_bits : VP8L_HTREE_TABLE_BITS;
    const uint32_t root_mask = ((uint32_t)1 << root_bits) - 1u;
    uint32_t low;
    if (tree && tree->single_symbol) {
        return (int)tree->single_symbol_value;
    }

    VP8LHuffEntry entry;
    int nbits;

    if (!br || !tree || !tree->table || tree->total_size <= 0) return -1;

    vp8l_br_fill(br);
    low = (uint32_t)(br->val & root_mask);
    entry = tree->table[low];
    nbits = (int)entry.bits - root_bits;
    if (nbits > 0) {
        const int table_offset = (int)entry.value;
        if (!vp8l_br_ensure(br, root_bits)) return -1;
        vp8l_br_advance(br, root_bits);
        vp8l_br_fill(br);
        entry = tree->table[(int)low + table_offset +
                            (int)(br->val & (((uint64_t)1 << nbits) - 1u))];
    }

    if (!vp8l_br_ensure(br, entry.bits)) return -1;
    vp8l_br_advance(br, entry.bits);
    return (int)entry.value;
}

static inline int vp8l_huff_read_canonical(VP8LBitReader* br,
                                           const VP8LHuffTree* tree) {
    uint32_t code = 0;
    int len;
    int symbol;
    if (!br || !tree) return -1;
    if (tree->single_symbol) {
        return (int)tree->single_symbol_value;
    }
    if (!tree->code_lengths || !tree->codes || tree->max_code_length <= 0) {
        return vp8l_huff_read(br, tree);
    }
    for (len = 1; len <= tree->max_code_length; ++len) {
        if (!vp8l_br_ensure(br, 1)) return -1;
        code |= vp8l_br_read(br, 1) << (len - 1);
        for (symbol = 0; symbol < tree->alphabet_size; ++symbol) {
            if ((int)tree->code_lengths[symbol] == len &&
                tree->codes[symbol] == code) {
                return symbol;
            }
        }
    }
    return -1;
}

static void vp8l_huff_free(VP8LHuffTree* tree) {
    if (tree->table) { free(tree->table); tree->table = NULL; }
    if (tree->code_lengths) { free(tree->code_lengths); tree->code_lengths = NULL; }
    if (tree->codes) { free(tree->codes); tree->codes = NULL; }
    tree->total_size = 0;
    tree->single_symbol = 0;
    tree->single_symbol_value = 0;
    tree->single_symbol_bits = 0;
    tree->alphabet_size = 0;
    tree->max_code_length = 0;
    tree->root_bits = 0;
}

/* ══════════════════════════════════════════════════════════════════════
 *  Prefix Code Reading
 * ══════════════════════════════════════════════════════════════════════ */

typedef struct {
    VP8LHuffTree htrees[VP8L_NUM_CODE_TYPES];
} VP8LCodeGroup;

/* Read a Huffman code from the bitstream */
static int vp8l_read_huffman_code(VP8LBitReader* br, int alphabet_size,
                                   VP8LHuffTree* tree) {
    if (!br || !tree || alphabet_size <= 0) return 0;
    const int start_bit_pos = (int)((int)br->byte_pos * 8 - br->bits);
    int simple = (int)vp8l_br_read(br, 1);
    int idx;
    g_vp8l_last_huff_simple = simple;
    g_vp8l_last_num_cl_codes = -1;
    g_vp8l_last_max_symbol = -1;
    for (idx = 0; idx < 19; ++idx) g_vp8l_last_cl_lengths[idx] = -1;

    if (simple) {
        /* Simple code: 1 or 2 symbols */
        int num_symbols = (int)vp8l_br_read(br, 1) + 1;
        int first_bits = (int)vp8l_br_read(br, 1) ? 8 : 1;
        int first_sym = (int)vp8l_br_read(br, first_bits);

        if (first_sym >= alphabet_size) {
            vp8l_set_huff_stage("huffman-simple-first-symbol");
            return 0;
        }

        if (num_symbols == 1) {
            /* Single symbol: zero-length code */
            /* Build a tree where every entry maps to first_sym */
            if (tree->table) {
                free(tree->table);
                tree->table = NULL;
            }
            tree->total_size = VP8L_HTREE_TABLE_SIZE;
            tree->table = (VP8LHuffEntry*)malloc(
                (size_t)VP8L_HTREE_TABLE_SIZE * sizeof(VP8LHuffEntry));
            if (!tree->table) return 0;
            for (int i = 0; i < VP8L_HTREE_TABLE_SIZE; i++) {
                tree->table[i].bits = 0;
                tree->table[i].value = (uint16_t)first_sym;
            }
            tree->single_symbol = 1;
            tree->single_symbol_bits = 0;
            tree->single_symbol_value = (uint16_t)first_sym;
            snprintf(g_vp8l_last_success_context, sizeof(g_vp8l_last_success_context), "%s",
                     (g_vp8l_huff_context && *g_vp8l_huff_context) ? g_vp8l_huff_context : "huffman");
            g_vp8l_last_success_alphabet_size = alphabet_size;
            g_vp8l_last_success_simple = simple;
            g_vp8l_last_success_num_cl_codes = g_vp8l_last_num_cl_codes;
            g_vp8l_last_success_max_symbol = g_vp8l_last_max_symbol;
            g_vp8l_last_success_start_bit_pos = start_bit_pos;
            g_vp8l_last_success_end_bit_pos = (int)((int)br->byte_pos * 8 - br->bits);
            return 1;
        } else {
            int second_sym = (int)vp8l_br_read(br, 8);
            int code_lengths_buf[512];
            int* cl = code_lengths_buf;
            int need_alloc = (alphabet_size > 512);
            if (need_alloc) {
                cl = (int*)malloc((size_t)alphabet_size * sizeof(int));
                if (!cl) {
                    vp8l_set_huff_stage("huffman-simple-alloc");
                    return 0;
                }
            }
            memset(cl, 0, (size_t)alphabet_size * sizeof(int));
            if (second_sym >= alphabet_size) {
                vp8l_set_huff_stage("huffman-simple-second-symbol");
                if (need_alloc) free(cl);
                return 0;
            }
            cl[first_sym] = 1;
            cl[second_sym] = 1;
            {
                int ok = vp8l_huff_build(tree, cl, alphabet_size);
                if (need_alloc) free(cl);
                if (ok) {
                    snprintf(g_vp8l_last_success_context, sizeof(g_vp8l_last_success_context), "%s",
                             (g_vp8l_huff_context && *g_vp8l_huff_context) ? g_vp8l_huff_context : "huffman");
                    g_vp8l_last_success_alphabet_size = alphabet_size;
                    g_vp8l_last_success_simple = simple;
                    g_vp8l_last_success_num_cl_codes = g_vp8l_last_num_cl_codes;
                    g_vp8l_last_success_max_symbol = g_vp8l_last_max_symbol;
                    g_vp8l_last_success_start_bit_pos = start_bit_pos;
                    g_vp8l_last_success_end_bit_pos = (int)((int)br->byte_pos * 8 - br->bits);
                }
                return ok;
            }
        }
    } else {
        /* Normal code */
        int num_cl_codes = (int)vp8l_br_read(br, 4) + 4;
        int cl_lengths[19];
        VP8LHuffTree cl_tree;
        int code_lengths_buf[512];
        int* cl = code_lengths_buf;
        int need_alloc = (alphabet_size > 512);
        int i, ok;

        memset(cl_lengths, 0, sizeof(cl_lengths));
        memset(&cl_tree, 0, sizeof(cl_tree));

        if (num_cl_codes > 19) num_cl_codes = 19;
        g_vp8l_last_num_cl_codes = num_cl_codes;
        for (i = 0; i < num_cl_codes; i++) {
            cl_lengths[kCodeLengthOrder[i]] = (int)vp8l_br_read(br, 3);
        }
        for (i = 0; i < 19; ++i) g_vp8l_last_cl_lengths[i] = cl_lengths[i];

        if (!vp8l_huff_build_with_root_bits(&cl_tree, cl_lengths, 19,
                                            VP8L_LENGTHS_TABLE_BITS)) {
            return 0;
        }

        if (need_alloc) {
            cl = (int*)malloc((size_t)alphabet_size * sizeof(int));
            if (!cl) {
                vp8l_set_huff_stage("huffman-cl-alloc");
                vp8l_huff_free(&cl_tree);
                return 0;
            }
        }
        memset(cl, 0, (size_t)alphabet_size * sizeof(int));

        /* Decode code lengths using the meta-Huffman tree */
        {
            int max_symbol = alphabet_size;
            int codes_to_read;
            /* Check for max_symbol flag */
            if (vp8l_br_read(br, 1)) {
                int length_nbits = 2 + 2 * (int)vp8l_br_read(br, 3);
                max_symbol = 2 + (int)vp8l_br_read(br, length_nbits);
                if (max_symbol > alphabet_size) {
                    vp8l_set_huff_stage("huffman-max-symbol-range");
                    if (need_alloc) free(cl);
                    vp8l_huff_free(&cl_tree);
                    return 0;
                }
            }
            g_vp8l_last_max_symbol = max_symbol;

            int sym_idx = 0;
            int prev_len = 8;
            codes_to_read = max_symbol;
            while (sym_idx < alphabet_size) {
                int code;
                int repeat = 0;
                int fill = 0;
                if (codes_to_read-- == 0) {
                    break;
                }
                code = vp8l_huff_read_canonical(br, &cl_tree);
                if (code < 0 || code > 18) {
                    vp8l_set_huff_stage("huffman-code-length-symbol");
                    if (need_alloc) free(cl);
                    vp8l_huff_free(&cl_tree);
                    return 0;
                }
                if (code < 16) {
                    cl[sym_idx++] = code;
                    if (code > 0) prev_len = code;
                } else {
                    if (code == 16) {
                        repeat = 3 + (int)vp8l_br_read(br, 2);
                        fill = prev_len;
                    } else if (code == 17) {
                        repeat = 3 + (int)vp8l_br_read(br, 3);
                    } else {
                        repeat = 11 + (int)vp8l_br_read(br, 7);
                    }
                    if (sym_idx + repeat > alphabet_size) {
                        vp8l_set_huff_stage("huffman-repeat-range");
                        if (need_alloc) free(cl);
                        vp8l_huff_free(&cl_tree);
                        return 0;
                    }
                    while (repeat-- > 0) {
                        cl[sym_idx++] = fill;
                    }
                }
            }
        }

        vp8l_huff_free(&cl_tree);
        ok = vp8l_huff_build(tree, cl, alphabet_size);
        if (need_alloc) free(cl);
        if (ok) {
            snprintf(g_vp8l_last_success_context, sizeof(g_vp8l_last_success_context), "%s",
                     (g_vp8l_huff_context && *g_vp8l_huff_context) ? g_vp8l_huff_context : "huffman");
            g_vp8l_last_success_alphabet_size = alphabet_size;
            g_vp8l_last_success_simple = simple;
            g_vp8l_last_success_num_cl_codes = g_vp8l_last_num_cl_codes;
            g_vp8l_last_success_max_symbol = g_vp8l_last_max_symbol;
            g_vp8l_last_success_start_bit_pos = start_bit_pos;
            g_vp8l_last_success_end_bit_pos = (int)((int)br->byte_pos * 8 - br->bits);
        }
        return ok;
    }
}

typedef struct {
    int color_cache_bits;
    uint32_t* color_cache;
    VP8LCodeGroup* code_groups;
    int num_code_groups;
    uint32_t* meta_data;
    int meta_bits;
    int meta_xsize;
} VP8LImageCodingState;

static int vp8l_decode_image_data(VP8LBitReader* br,
                                  VP8LCodeGroup* code_groups,
                                  int num_code_groups,
                                  uint32_t* meta_data,
                                  int meta_bits, int meta_xsize,
                                  uint32_t* color_cache, int cache_bits,
                                  uint32_t* pixels, int width, int height,
                                  const char* stage_prefix);

static void vp8l_free_code_group(VP8LCodeGroup* group) {
    int i;
    if (!group) return;
    for (i = 0; i < VP8L_NUM_CODE_TYPES; ++i) {
        vp8l_huff_free(&group->htrees[i]);
    }
}

static void vp8l_free_image_coding_state(VP8LImageCodingState* state) {
    int g;
    if (!state) return;
    if (state->code_groups) {
        for (g = 0; g < state->num_code_groups; ++g) {
            vp8l_free_code_group(&state->code_groups[g]);
        }
        free(state->code_groups);
    }
    if (state->meta_data) free(state->meta_data);
    if (state->color_cache) free(state->color_cache);
    memset(state, 0, sizeof(*state));
}

static int vp8l_read_code_group_set(VP8LBitReader* br,
                                    VP8LImageCodingState* state,
                                    const char* stage_prefix) {
    int g;
    int green_alpha_size;

    if (!br || !state || state->num_code_groups <= 0 ||
        state->num_code_groups > VP8L_MAX_NUM_HTREE_GROUPS) {
        return 0;
    }

    green_alpha_size = 256 + 24;
    if (state->color_cache_bits > 0) {
        green_alpha_size += (1 << state->color_cache_bits);
    }

    state->code_groups = (VP8LCodeGroup*)malloc(
        (size_t)state->num_code_groups * sizeof(VP8LCodeGroup));
    if (!state->code_groups) return 0;
    memset(state->code_groups, 0,
           (size_t)state->num_code_groups * sizeof(VP8LCodeGroup));

    for (g = 0; g < state->num_code_groups; ++g) {
        int j;
        char huff_context[96];
        vp8l_set_stage_with_prefix(stage_prefix, "green-huffman");
        snprintf(huff_context, sizeof(huff_context), "%s-%s",
                 stage_prefix ? stage_prefix : "huffman", "green-huffman");
        vp8l_set_huff_context(huff_context);
        if (!vp8l_read_huffman_code(br, green_alpha_size,
                                    &state->code_groups[g].htrees[0])) {
            return 0;
        }
        for (j = 1; j < 4; ++j) {
            vp8l_set_stage_with_prefix(stage_prefix, "rgba-huffman");
            snprintf(huff_context, sizeof(huff_context), "%s-%s",
                     stage_prefix ? stage_prefix : "huffman", "rgba-huffman");
            vp8l_set_huff_context(huff_context);
            if (!vp8l_read_huffman_code(br, 256,
                                        &state->code_groups[g].htrees[j])) {
                return 0;
            }
        }
        vp8l_set_stage_with_prefix(stage_prefix, "distance-huffman");
        snprintf(huff_context, sizeof(huff_context), "%s-%s",
                 stage_prefix ? stage_prefix : "huffman", "distance-huffman");
        vp8l_set_huff_context(huff_context);
        if (!vp8l_read_huffman_code(br, VP8L_NUM_DISTANCE_CODES,
                                    &state->code_groups[g].htrees[4])) {
            return 0;
        }
    }
    vp8l_set_huff_context(NULL);
    return 1;
}

static int vp8l_read_image_coding_state_internal(VP8LBitReader* br,
                                                 int width,
                                                 int height,
                                                 int allow_meta,
                                                 VP8LImageCodingState* state,
                                                 const char* stage_prefix) {
    VP8LImageCodingState meta_state;
    int use_meta = 0;

    if (!br || !state || width <= 0 || height <= 0) return 0;
    memset(state, 0, sizeof(*state));
    memset(&meta_state, 0, sizeof(meta_state));

    vp8l_set_stage_with_prefix(stage_prefix, "color-cache");
    g_vp8l_last_sub_cache_flag = (int)vp8l_br_read(br, 1);
    if (g_vp8l_last_sub_cache_flag) {
        state->color_cache_bits = (int)vp8l_br_read(br, 4);
        g_vp8l_last_sub_cache_bits = state->color_cache_bits;
        if (state->color_cache_bits < 1 ||
            state->color_cache_bits > VP8L_MAX_CACHE_BITS) {
            goto fail;
        }
        state->color_cache = (uint32_t*)malloc(
            (size_t)(1 << state->color_cache_bits) * sizeof(uint32_t));
        if (!state->color_cache) goto fail;
        memset(state->color_cache, 0,
               (size_t)(1 << state->color_cache_bits) * sizeof(uint32_t));
    } else {
        g_vp8l_last_sub_cache_bits = 0;
    }

    if (allow_meta) {
        vp8l_set_stage_with_prefix(stage_prefix, "meta-flag");
        use_meta = (int)vp8l_br_read(br, 1);
    }
    if (use_meta) {
        int meta_ysize;
        int meta_total;
        int i;
        int max_idx = 0;

        state->meta_bits = (int)vp8l_br_read(br, 3) + 2;
        state->meta_xsize =
            (width + ((1 << state->meta_bits) - 1)) >> state->meta_bits;
        meta_ysize =
            (height + ((1 << state->meta_bits) - 1)) >> state->meta_bits;
        meta_total = state->meta_xsize * meta_ysize;
        if (state->meta_xsize <= 0 || meta_ysize <= 0 || meta_total <= 0) {
            goto fail;
        }

        state->meta_data = (uint32_t*)malloc((size_t)meta_total * sizeof(uint32_t));
        if (!state->meta_data) goto fail;

        if (!vp8l_read_image_coding_state_internal(br,
                                                   state->meta_xsize,
                                                   meta_ysize,
                                                   0,
                                                   &meta_state,
                                                   "meta-huffman")) {
            goto fail;
        }
        vp8l_set_stage_with_prefix(stage_prefix, "meta-image");
        if (vp8l_decode_image_data(br,
                                   meta_state.code_groups,
                                   meta_state.num_code_groups,
                                   meta_state.meta_data,
                                   meta_state.meta_bits,
                                   meta_state.meta_xsize,
                                   meta_state.color_cache,
                                   meta_state.color_cache_bits,
                                   state->meta_data,
                                   state->meta_xsize,
                                   meta_ysize,
                                   "meta-image") != 0) {
            goto fail;
        }
        vp8l_free_image_coding_state(&meta_state);

        for (i = 0; i < meta_total; ++i) {
            int idx = (int)((state->meta_data[i] >> 8) & 0xffff);
            if (idx > max_idx) max_idx = idx;
        }
        state->num_code_groups = max_idx + 1;
    } else {
        state->num_code_groups = 1;
    }

    if (!vp8l_read_code_group_set(br, state, stage_prefix)) {
        goto fail;
    }
    return 1;

fail:
    vp8l_free_image_coding_state(&meta_state);
    vp8l_free_image_coding_state(state);
    return 0;
}

static int vp8l_read_image_coding_state(VP8LBitReader* br,
                                        int width,
                                        int height,
                                        VP8LImageCodingState* state,
                                        const char* stage_prefix) {
    return vp8l_read_image_coding_state_internal(br, width, height, 0,
                                                 state, stage_prefix);
}

/* ══════════════════════════════════════════════════════════════════════
 *  LZ77 Prefix Code Helpers
 * ══════════════════════════════════════════════════════════════════════ */

static inline int vp8l_prefix_decode(VP8LBitReader* br, int prefix_code) {
    if (!br || prefix_code < 0) return -1;
    if (prefix_code < 4) return prefix_code + 1;
    {
        int extra_bits = (prefix_code - 2) >> 1;
        int offset = (2 + (prefix_code & 1)) << extra_bits;
        if (extra_bits < 0 || extra_bits > 24) return -1;
        return offset + (int)vp8l_br_read(br, extra_bits) + 1;
    }
}

static inline int vp8l_distance_map(int plane_code, int xsize) {
    if (plane_code <= 0 || xsize <= 0) return -1;
    if (plane_code > 120) return plane_code - 120;
    {
        int dist_code = kCodeToPlane[plane_code - 1];
        int yoffset = dist_code >> 4;
        int xoffset = 8 - (dist_code & 0x0f);
        int dist = yoffset * xsize + xoffset;
        return dist >= 1 ? dist : 1;
    }
}

/* ══════════════════════════════════════════════════════════════════════
 *  Color Cache
 * ══════════════════════════════════════════════════════════════════════ */

static inline void vp8l_cache_insert(uint32_t* cache, int bits, uint32_t argb) {
    int key = (int)((argb * 0x1e35a7bdU) >> (32 - bits));
    cache[key] = argb;
}

static inline uint32_t vp8l_cache_lookup(const uint32_t* cache, int key) {
    return cache[key];
}

/* ══════════════════════════════════════════════════════════════════════
 *  Transform Structures
 * ══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int       type;
    int       bits;
    int       xsize, ysize;
    uint32_t* data;    /* malloc'd sub-image data */
} VP8LTransform;

/* ══════════════════════════════════════════════════════════════════════
 *  Image Data Decode (main LZ77 + literal loop)
 * ══════════════════════════════════════════════════════════════════════ */

static int vp8l_decode_image_data(VP8LBitReader* br,
                                   VP8LCodeGroup* code_groups,
                                   int num_code_groups,
                                   uint32_t* meta_data,
                                   int meta_bits, int meta_xsize,
                                   uint32_t* color_cache, int cache_bits,
                                   uint32_t* pixels, int width, int height,
                                   const char* stage_prefix) {
    if (!br || !code_groups || num_code_groups <= 0 || !pixels ||
        width <= 0 || height <= 0) {
        return -1;
    }
#define VP8L_STAGE(name) vp8l_set_stage_with_prefix(stage_prefix, name)
    g_vp8l_last_pos = -1;
    g_vp8l_last_dist = -1;
    g_vp8l_last_length = -1;
    g_vp8l_last_width = width;
    g_vp8l_last_dist_symbol = -1;
    int total = width * height;
    int pos = 0;
    int cache_size = cache_bits > 0 ? (1 << cache_bits) : 0;
    int len_code_limit = 256 + 24;
    int cache_code_base = len_code_limit;

    while (pos < total) {
        VP8LCodeGroup* group;
        int green_sym;

        /* Select code group */
        if (meta_data && num_code_groups > 1) {
            int mx = (pos % width) >> meta_bits;
            int my = (pos / width) >> meta_bits;
            uint32_t meta_pixel = meta_data[my * meta_xsize + mx];
            int group_idx = ((meta_pixel >> 8) & 0xffff);
            if (group_idx >= num_code_groups) group_idx = 0;
            group = &code_groups[group_idx];
        } else {
            group = &code_groups[0];
        }

        green_sym = vp8l_huff_read(br, &group->htrees[0]);
        if (green_sym < 0) {
            VP8L_STAGE("green-symbol");
            return -1;
        }

        if (green_sym < 256) {
            /* Literal pixel */
            int red   = vp8l_huff_read(br, &group->htrees[1]);
            int blue  = vp8l_huff_read(br, &group->htrees[2]);
            int alpha = vp8l_huff_read(br, &group->htrees[3]);
            if (red < 0 || red > 255 || blue < 0 || blue > 255 ||
                alpha < 0 || alpha > 255) {
                VP8L_STAGE("literal-symbol");
                return -1;
            }
            uint32_t argb = ((uint32_t)alpha << 24) | ((uint32_t)red << 16) |
                            ((uint32_t)green_sym << 8) | (uint32_t)blue;
            pixels[pos] = argb;
            if (cache_bits > 0) vp8l_cache_insert(color_cache, cache_bits, argb);
            pos++;
        } else if (green_sym < cache_code_base) {
            /* LZ77 back-reference */
            int length_prefix = green_sym - 256;
            int length = vp8l_prefix_decode(br, length_prefix);
            int dist_sym = vp8l_huff_read(br, &group->htrees[4]);
            int dist;
            if (length <= 0) {
                VP8L_STAGE("length-prefix");
                return -1;
            }
            if (dist_sym < 0 || dist_sym >= VP8L_NUM_DISTANCE_CODES) {
                g_vp8l_last_pos = pos;
                g_vp8l_last_length = length;
                g_vp8l_last_dist_symbol = dist_sym;
                VP8L_STAGE("distance-symbol");
                return -1;
            }
            dist = vp8l_prefix_decode(br, dist_sym);
            if (dist <= 0) {
                g_vp8l_last_pos = pos;
                g_vp8l_last_length = length;
                g_vp8l_last_dist_symbol = dist_sym;
                VP8L_STAGE("distance-prefix");
                return -1;
            }
            dist = vp8l_distance_map(dist, width);
            if (dist <= 0) {
                g_vp8l_last_pos = pos;
                g_vp8l_last_dist = dist;
                g_vp8l_last_length = length;
                g_vp8l_last_dist_symbol = dist_sym;
                VP8L_STAGE("distance-map");
                return -1;
            }
            if (dist > pos) {
                g_vp8l_last_pos = pos;
                g_vp8l_last_dist = dist;
                g_vp8l_last_length = length;
                g_vp8l_last_dist_symbol = dist_sym;
                VP8L_STAGE("distance-range");
                return -1;
            }
            if (pos + length > total) {
                g_vp8l_last_pos = pos;
                g_vp8l_last_dist = dist;
                g_vp8l_last_length = length;
                g_vp8l_last_dist_symbol = dist_sym;
                VP8L_STAGE("length-range");
                return -1;
            }

            {
                int i;
                for (i = 0; i < length; i++) {
                    pixels[pos + i] = pixels[pos + i - dist];
                    if (cache_bits > 0)
                        vp8l_cache_insert(color_cache, cache_bits, pixels[pos + i]);
                }
            }
            pos += length;
        } else {
            /* Color cache reference */
            int cache_idx = green_sym - cache_code_base;
            if (cache_bits <= 0 || !color_cache) {
                VP8L_STAGE("cache-disabled");
                return -1;
            }
            if (cache_idx < 0 || cache_idx >= cache_size) {
                VP8L_STAGE("cache-range");
                return -1;
            }
            {
                uint32_t argb = vp8l_cache_lookup(color_cache, cache_idx);
                pixels[pos] = argb;
                if (cache_bits > 0)
                    vp8l_cache_insert(color_cache, cache_bits, argb);
                pos++;
            }
        }

        if (br->eos && pos < total) {
            VP8L_STAGE("eos");
            return -1;
        }
    }
#undef VP8L_STAGE
    return 0;
}

/* ══════════════════════════════════════════════════════════════════════
 *  Inverse Transforms
 * ══════════════════════════════════════════════════════════════════════ */

static inline uint8_t vp8l_clamp_byte(int v) {
    return (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
}

static inline uint32_t vp8l_add_pixels(uint32_t a, uint32_t b) {
    uint32_t ag = ((a >> 8) & 0x00ff00ff) + ((b >> 8) & 0x00ff00ff);
    uint32_t rb = (a & 0x00ff00ff) + (b & 0x00ff00ff);
    return ((ag & 0x00ff00ff) << 8) | (rb & 0x00ff00ff);
}

static inline int vp8l_average2(int a, int b) {
    return (a + b) / 2;
}

static inline uint32_t vp8l_select(uint32_t L, uint32_t T, uint32_t TL) {
    int pL = (int)((L >> 24) & 0xff) + (int)((L >> 16) & 0xff) +
             (int)((L >> 8) & 0xff) + (int)(L & 0xff);
    int pT = (int)((T >> 24) & 0xff) + (int)((T >> 16) & 0xff) +
             (int)((T >> 8) & 0xff) + (int)(T & 0xff);
    int pTL = (int)((TL >> 24) & 0xff) + (int)((TL >> 16) & 0xff) +
              (int)((TL >> 8) & 0xff) + (int)(TL & 0xff);
    int pT_minus_pTL = pT - pTL;
    int pL_minus_pTL = pL - pTL;
    if (pL_minus_pTL < 0) pL_minus_pTL = -pL_minus_pTL;
    if (pT_minus_pTL < 0) pT_minus_pTL = -pT_minus_pTL;
    return (pL_minus_pTL < pT_minus_pTL) ? L : T;
}

static inline uint32_t vp8l_clamp_add_sub_full(uint32_t a, uint32_t b, uint32_t c) {
    int aa = (int)((a >> 24) & 0xff), ar = (int)((a >> 16) & 0xff),
        ag = (int)((a >> 8) & 0xff), ab = (int)(a & 0xff);
    int ba = (int)((b >> 24) & 0xff), br_ = (int)((b >> 16) & 0xff),
        bg = (int)((b >> 8) & 0xff), bb = (int)(b & 0xff);
    int ca = (int)((c >> 24) & 0xff), cr = (int)((c >> 16) & 0xff),
        cg = (int)((c >> 8) & 0xff), cb = (int)(c & 0xff);
    return ((uint32_t)vp8l_clamp_byte(aa + ba - ca) << 24) |
           ((uint32_t)vp8l_clamp_byte(ar + br_ - cr) << 16) |
           ((uint32_t)vp8l_clamp_byte(ag + bg - cg) << 8) |
           (uint32_t)vp8l_clamp_byte(ab + bb - cb);
}

static inline uint32_t vp8l_clamp_add_sub_half(uint32_t avg, uint32_t val) {
    int aa = (int)((avg >> 24) & 0xff), ar = (int)((avg >> 16) & 0xff),
        ag = (int)((avg >> 8) & 0xff), ab = (int)(avg & 0xff);
    int va = (int)((val >> 24) & 0xff), vr = (int)((val >> 16) & 0xff),
        vg = (int)((val >> 8) & 0xff), vb = (int)(val & 0xff);
    return ((uint32_t)vp8l_clamp_byte(aa + (aa - va) / 2) << 24) |
           ((uint32_t)vp8l_clamp_byte(ar + (ar - vr) / 2) << 16) |
           ((uint32_t)vp8l_clamp_byte(ag + (ag - vg) / 2) << 8) |
           (uint32_t)vp8l_clamp_byte(ab + (ab - vb) / 2);
}

static inline uint32_t vp8l_average2_pixels(uint32_t a, uint32_t b) {
    uint32_t ag = (((a >> 8) & 0x00ff00ff) + ((b >> 8) & 0x00ff00ff)) >> 1;
    uint32_t rb = ((a & 0x00ff00ff) + (b & 0x00ff00ff)) >> 1;
    return (ag << 8) | (rb & 0x00ff00ff);
}

static uint32_t vp8l_predict(int mode, uint32_t L, uint32_t T,
                              uint32_t TL, uint32_t TR) {
    switch (mode) {
    case 0:  return 0xff000000u;           /* black */
    case 1:  return L;                     /* left */
    case 2:  return T;                     /* top */
    case 3:  return TR;                    /* top-right */
    case 4:  return TL;                    /* top-left */
    case 5:  return vp8l_average2_pixels(vp8l_average2_pixels(L, TR), T);
    case 6:  return vp8l_average2_pixels(L, TL);
    case 7:  return vp8l_average2_pixels(L, T);
    case 8:  return vp8l_average2_pixels(TL, T);
    case 9:  return vp8l_average2_pixels(T, TR);
    case 10: return vp8l_average2_pixels(vp8l_average2_pixels(L, TL),
                                          vp8l_average2_pixels(T, TR));
    case 11: return vp8l_select(L, T, TL);
    case 12: return vp8l_clamp_add_sub_full(L, T, TL);
    case 13: return vp8l_clamp_add_sub_half(vp8l_average2_pixels(L, T), TL);
    default: return 0xff000000u;
    }
}

static void vp8l_inverse_predictor(uint32_t* pixels, int width, int height,
                                    const VP8LTransform* tr) {
    int block_size = 1 << tr->bits;
    int bw = (width + block_size - 1) / block_size;
    int x, y;

    /* First pixel (0,0): add prediction 0 (black) */
    /* Actually, top-left pixel has no prediction context -> mode 0 */

    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            uint32_t pred;
            if (x == 0 && y == 0) {
                pred = 0xff000000u;
            } else if (y == 0) {
                pred = pixels[y * width + x - 1]; /* left */
            } else if (x == 0) {
                pred = pixels[(y - 1) * width + x]; /* top */
            } else {
                int bx = x / block_size;
                int by = y / block_size;
                int mode = (tr->data[by * bw + bx] >> 8) & 0xff; /* green channel = mode */
                uint32_t L  = pixels[y * width + x - 1];
                uint32_t T  = pixels[(y - 1) * width + x];
                uint32_t TL = pixels[(y - 1) * width + x - 1];
                uint32_t TR = (x + 1 < width) ? pixels[(y - 1) * width + x + 1] : T;
                pred = vp8l_predict(mode, L, T, TL, TR);
            }
            pixels[y * width + x] = vp8l_add_pixels(pixels[y * width + x], pred);
        }
    }
}

static inline int8_t vp8l_color_transform_delta(int8_t t, int8_t c) {
    return (int8_t)(((int)t * (int)c) >> 5);
}

static void vp8l_inverse_cross_color(uint32_t* pixels, int width, int height,
                                      const VP8LTransform* tr) {
    int block_size = 1 << tr->bits;
    int bw = (width + block_size - 1) / block_size;
    int x, y;

    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            int bx = x / block_size;
            int by = y / block_size;
            uint32_t t = tr->data[by * bw + bx];
            /* RFC 9649: the subimage pixel stores
             *   red   = red_to_blue
             *   green = green_to_blue
             *   blue  = green_to_red
             */
            int8_t green_to_red  = (int8_t)(t & 0xff);           /* blue channel */
            int8_t green_to_blue = (int8_t)((t >> 8) & 0xff);   /* green channel */
            int8_t red_to_blue   = (int8_t)((t >> 16) & 0xff);  /* red channel */

            uint32_t p = pixels[y * width + x];
            int green = (int)((p >> 8) & 0xff);
            int red   = (int)((p >> 16) & 0xff);
            int blue  = (int)(p & 0xff);

            red  = (red + vp8l_color_transform_delta(green_to_red, (int8_t)green)) & 0xff;
            blue = (blue + vp8l_color_transform_delta(green_to_blue, (int8_t)green)) & 0xff;
            blue = (blue + vp8l_color_transform_delta(red_to_blue, (int8_t)red)) & 0xff;

            pixels[y * width + x] = (p & 0xff00ff00u) | ((uint32_t)red << 16) | (uint32_t)blue;
        }
    }
}

static void vp8l_inverse_subtract_green(uint32_t* pixels, int width, int height) {
    int total = width * height;
    int i;
    for (i = 0; i < total; i++) {
        uint32_t p = pixels[i];
        int green = (int)((p >> 8) & 0xff);
        int red   = ((int)((p >> 16) & 0xff) + green) & 0xff;
        int blue  = ((int)(p & 0xff) + green) & 0xff;
        pixels[i] = (p & 0xff00ff00u) | ((uint32_t)red << 16) | (uint32_t)blue;
    }
}

static void vp8l_inverse_color_indexing(uint32_t* pixels, int width, int height,
                                         const VP8LTransform* tr) {
    int palette_size = tr->ysize; /* stored in ysize */
    int width_bits = tr->bits;
    int src_width = tr->xsize;   /* encoded width (possibly packed) */
    int x, y;

    /* Unpack and palette-lookup */
    if (width_bits > 0) {
        /* Pixels are packed: multiple palette indices per green channel */
        int pixels_per_entry = 1 << width_bits;
        int mask = (1 << (8 >> width_bits)) - 1;
        /* Hmm, actually for color indexing:
         * bits_per_pixel = 8 / pixels_per_entry
         * For palette_size <= 2:  pixels_per_entry = 8, bits_per_pixel = 1
         * For palette_size <= 4:  pixels_per_entry = 4, bits_per_pixel = 2
         * For palette_size <= 16: pixels_per_entry = 2, bits_per_pixel = 4
         */
        int bits_per_pixel;
        if (palette_size <= 2) bits_per_pixel = 1;
        else if (palette_size <= 4) bits_per_pixel = 2;
        else bits_per_pixel = 4;

        mask = (1 << bits_per_pixel) - 1;

        /* Work backwards to avoid overwriting */
        for (y = height - 1; y >= 0; y--) {
            for (x = width - 1; x >= 0; x--) {
                int src_x = x / (8 / bits_per_pixel);
                int bit_offset = (x % (8 / bits_per_pixel)) * bits_per_pixel;
                uint32_t packed = pixels[y * src_width + src_x];
                int idx = ((int)(packed >> 8) >> bit_offset) & mask; /* green channel */
                if (idx < palette_size)
                    pixels[y * width + x] = tr->data[idx];
                else
                    pixels[y * width + x] = 0;
            }
        }
    } else {
        /* No packing, simple palette lookup via green channel */
        int total = width * height;
        int i;
        for (i = 0; i < total; i++) {
            int idx = (int)((pixels[i] >> 8) & 0xff);
            if (idx < palette_size)
                pixels[i] = tr->data[idx];
            else
                pixels[i] = 0;
        }
    }
}

/* ══════════════════════════════════════════════════════════════════════
 *  Top-Level Decoder
 * ══════════════════════════════════════════════════════════════════════ */

static int vp8l_decode_impl(const uint8_t* data, size_t data_size,
                            int width, int height,
                            int has_header,
                            uint8_t* output, size_t output_size, int output_stride,
                            uint8_t* green_output, size_t green_output_size,
                            int green_output_stride) {
    VP8LBitReader br;
    VP8LTransform transforms[VP8L_MAX_TRANSFORMS];
    int num_transforms = 0;
    int color_cache_bits = 0;
    uint32_t* color_cache = NULL;
    VP8LCodeGroup* code_groups = NULL;
    int num_code_groups = 0;
    uint32_t* meta_data = NULL;
    int meta_bits = 0, meta_xsize = 0;
    uint32_t* argb = NULL;
    int xsize = width, ysize = height;
    int ok = 0;
    int i;
    VP8LImageCodingState main_state;

    memset(transforms, 0, sizeof(transforms));
    memset(&main_state, 0, sizeof(main_state));

    vp8l_set_stage(has_header ? "header" : "alph-stream");
    if (!data) return -1;
    if (width <= 0 || height <= 0) return -1;
    if (has_header) {
        if (data_size < 5) return -1;
        if (data[0] != 0x2f) return -1;
        vp8l_br_init(&br, data + 5, data_size - 5);
    } else {
        if (data_size == 0) return -1;
        vp8l_br_init(&br, data, data_size);
    }

    /* Read transforms */
    vp8l_set_stage("transforms");
    while (vp8l_br_read(&br, 1)) {
        int type;
        VP8LTransform* tr;

        if (num_transforms >= VP8L_MAX_TRANSFORMS) goto fail;
        tr = &transforms[num_transforms];
        type = (int)vp8l_br_read(&br, 2);
        tr->type = type;
        tr->xsize = xsize;
        tr->ysize = ysize;
        g_vp8l_last_transform_index = num_transforms;
        g_vp8l_last_transform_type = type;
        g_vp8l_last_transform_bits = -1;
        g_vp8l_last_sub_cache_flag = -1;
        g_vp8l_last_sub_cache_bits = -1;
        /* Claim the slot before any allocation. The shared failure path must
         * also release a partially decoded transform. */
        num_transforms++;

        switch (type) {
        case VP8L_TRANSFORM_PREDICTOR:
        case VP8L_TRANSFORM_CROSS_COLOR: {
            int bits = (int)vp8l_br_read(&br, 3) + 2;
            int bw = (xsize + ((1 << bits) - 1)) >> bits;
            int bh = (ysize + ((1 << bits) - 1)) >> bits;
            int sub_total = bw * bh;
            VP8LImageCodingState sub_state;

            tr->bits = bits;
            g_vp8l_last_transform_bits = bits;
            tr->data = (uint32_t*)malloc((size_t)sub_total * sizeof(uint32_t));
            if (!tr->data) goto fail;
            memset(&sub_state, 0, sizeof(sub_state));
            if (!vp8l_read_image_coding_state(&br, bw, bh, &sub_state,
                                              "transform-subimage")) {
                vp8l_free_image_coding_state(&sub_state);
                goto fail;
            }

            /* Decode sub-image */
            vp8l_set_stage("transform-subimage");
            if (vp8l_decode_image_data(&br,
                                        sub_state.code_groups,
                                        sub_state.num_code_groups,
                                        sub_state.meta_data,
                                        sub_state.meta_bits,
                                        sub_state.meta_xsize,
                                        sub_state.color_cache,
                                        sub_state.color_cache_bits,
                                        tr->data, bw, bh,
                                        "transform-subimage-image") != 0) {
                vp8l_free_image_coding_state(&sub_state);
                goto fail;
            }
            vp8l_free_image_coding_state(&sub_state);
            break;
        }
        case VP8L_TRANSFORM_SUBTRACT_GREEN:
            tr->bits = 0;
            tr->data = NULL;
            break;
        case VP8L_TRANSFORM_COLOR_INDEXING: {
            int palette_size = (int)vp8l_br_read(&br, 8) + 1;
            VP8LImageCodingState palette_state;

            tr->data = (uint32_t*)malloc((size_t)palette_size * sizeof(uint32_t));
            if (!tr->data) goto fail;
            tr->ysize = palette_size;
            memset(&palette_state, 0, sizeof(palette_state));
            if (!vp8l_read_image_coding_state(&br, palette_size, 1,
                                              &palette_state, "palette")) {
                vp8l_free_image_coding_state(&palette_state);
                goto fail;
            }

            if (vp8l_decode_image_data(&br,
                                        palette_state.code_groups,
                                        palette_state.num_code_groups,
                                        palette_state.meta_data,
                                        palette_state.meta_bits,
                                        palette_state.meta_xsize,
                                        palette_state.color_cache,
                                        palette_state.color_cache_bits,
                                        tr->data, palette_size, 1,
                                        "palette-image") != 0) {
                vp8l_free_image_coding_state(&palette_state);
                goto fail;
            }
            vp8l_free_image_coding_state(&palette_state);

            /* Apply inverse palette delta: palette[i] += palette[i-1] per component */
            for (i = 1; i < palette_size; i++) {
                tr->data[i] = vp8l_add_pixels(tr->data[i], tr->data[i - 1]);
            }

            /* Update working width for pixel packing */
            if (palette_size <= 2) {
                tr->bits = 3; /* 8 pixels per entry */
                xsize = (xsize + 7) >> 3;
            } else if (palette_size <= 4) {
                tr->bits = 2; /* 4 pixels per entry */
                xsize = (xsize + 3) >> 2;
            } else if (palette_size <= 16) {
                tr->bits = 1; /* 2 pixels per entry */
                xsize = (xsize + 1) >> 1;
            } else {
                tr->bits = 0; /* no packing */
            }
            tr->xsize = xsize;
            break;
        }
        }
    }

    vp8l_set_stage("main-coding-state");
    if (!vp8l_read_image_coding_state_internal(&br, xsize, ysize, 1,
                                               &main_state, "main")) {
        goto fail;
    }
    color_cache_bits = main_state.color_cache_bits;
    color_cache = main_state.color_cache;
    main_state.color_cache = NULL;
    num_code_groups = main_state.num_code_groups;
    code_groups = main_state.code_groups;
    main_state.code_groups = NULL;
    meta_bits = main_state.meta_bits;
    meta_xsize = main_state.meta_xsize;
    meta_data = main_state.meta_data;
    main_state.meta_data = NULL;

    /* Allocate ARGB pixel buffer */
    vp8l_set_stage("main-image-alloc");
    argb = (uint32_t*)malloc((size_t)width * (size_t)height * sizeof(uint32_t));
    if (!argb) goto fail;
    memset(argb, 0, (size_t)width * (size_t)height * sizeof(uint32_t));

    /* Decode main image data */
    vp8l_set_stage("main-image");
    if (vp8l_decode_image_data(&br, code_groups, num_code_groups,
                                meta_data, meta_bits, meta_xsize,
                                color_cache, color_cache_bits,
                                argb, xsize, ysize,
                                "main-image") != 0)
        goto fail;

    /* Apply inverse transforms in reverse order */
    for (i = num_transforms - 1; i >= 0; i--) {
        VP8LTransform* tr = &transforms[i];
        switch (tr->type) {
        case VP8L_TRANSFORM_COLOR_INDEXING:
            vp8l_set_stage("inverse-color-indexing");
            vp8l_inverse_color_indexing(argb, width, height, tr);
            xsize = width;
            break;
        case VP8L_TRANSFORM_SUBTRACT_GREEN:
            vp8l_set_stage("inverse-subtract-green");
            vp8l_inverse_subtract_green(argb, xsize, ysize);
            break;
        case VP8L_TRANSFORM_PREDICTOR:
            vp8l_set_stage("inverse-predictor");
            vp8l_inverse_predictor(argb, xsize, ysize, tr);
            break;
        case VP8L_TRANSFORM_CROSS_COLOR:
            vp8l_set_stage("inverse-cross-color");
            vp8l_inverse_cross_color(argb, xsize, ysize, tr);
            break;
        }
    }

    if (green_output) {
        int y;
        vp8l_set_stage("extract-green");
        if (green_output_stride < width) goto fail;
        if ((size_t)green_output_stride > (size_t)-1 / (size_t)height) goto fail;
        if ((size_t)green_output_stride * (size_t)height > green_output_size) goto fail;
        for (y = 0; y < height; y++) {
            uint8_t* row = green_output + (size_t)y * (size_t)green_output_stride;
            int x;
            for (x = 0; x < width; x++) {
                row[x] = (uint8_t)((argb[(size_t)y * (size_t)width + (size_t)x] >> 8) & 0xffu);
            }
        }
    } else {
        /* Copy ARGB to output as BGRA */
        /* On little-endian, ARGB uint32 stores as bytes [B,G,R,A] = BGRA. */
        int y;
        vp8l_set_stage("copy-output");
        if (!output || output_stride < width * 4) goto fail;
        if ((size_t)output_stride > (size_t)-1 / (size_t)height) goto fail;
        if ((size_t)output_stride * (size_t)height > output_size) goto fail;
        for (y = 0; y < height; y++) {
            memcpy(output + y * output_stride,
                   argb + y * width,
                   (size_t)width * 4);
        }
    }

    ok = 1;
    vp8l_set_stage("ok");

fail:
    if (argb) free(argb);
    vp8l_free_image_coding_state(&main_state);
    if (color_cache) free(color_cache);
    if (meta_data) free(meta_data);
    if (code_groups) {
        int g;
        for (g = 0; g < num_code_groups; g++) {
            int j;
            for (j = 0; j < 5; j++)
                vp8l_huff_free(&code_groups[g].htrees[j]);
        }
        free(code_groups);
    }
    for (i = 0; i < num_transforms; i++) {
        if (transforms[i].data) free(transforms[i].data);
    }

    return ok ? 0 : -1;
}

int vp8l_decode(const uint8_t* data, size_t data_size,
                int width, int height,
                uint8_t* output, size_t output_size, int output_stride) {
    return vp8l_decode_impl(data, data_size, width, height, 1,
                            output, output_size, output_stride,
                            NULL, 0, 0);
}

int vp8l_decode_alpha_image(const uint8_t* data, size_t data_size,
                            int width, int height,
                            uint8_t* alpha, size_t alpha_size, int alpha_stride) {
    return vp8l_decode_impl(data, data_size, width, height, 0,
                            NULL, 0, 0,
                            alpha, alpha_size, alpha_stride);
}
