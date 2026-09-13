/*
 * VP8 (WebP Lossy) decoder for RinOS
 * Based on RFC 6386 - VP8 Data Format and Decoding Guide
 * Key frames only (sufficient for WebP still images)
 */

#include <stdint.h>
#include <stddef.h>

/* Freestanding: declare libc functions manually to avoid C++ header conflicts */
extern void* malloc(size_t);
extern void  free(void*);
extern void* memset(void*, int, size_t);
extern void* memcpy(void*, const void*, size_t);

/* ══════════════════════════════════════════════════════════════════════
 *  Constants & Lookup Tables
 * ══════════════════════════════════════════════════════════════════════ */

#define VP8_MAX_WIDTH   4096
#define VP8_MAX_MB_W    256

static inline uint8_t vp8_clip8(int v) {
    return (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
}

static inline uint8_t vp8_avg2(uint8_t x, uint8_t y) {
    return (uint8_t)((x + y + 1) >> 1);
}

static inline uint8_t vp8_avg3(uint8_t x, uint8_t y, uint8_t z) {
    return (uint8_t)((x + 2 * y + z + 2) >> 2);
}

/* DC quantizer lookup (RFC 6386 Section 14.1) */
static const uint16_t kDcQLookup[128] = {
      4,   5,   6,   7,   8,   9,  10,  10,  11,  12,  13,  14,  15,  16,  17,  17,
     18,  19,  20,  20,  21,  21,  22,  22,  23,  23,  24,  25,  25,  26,  27,  28,
     29,  30,  31,  32,  33,  34,  35,  36,  37,  37,  38,  39,  40,  41,  42,  43,
     44,  45,  46,  46,  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,
     59,  60,  61,  62,  63,  64,  65,  66,  67,  68,  69,  70,  71,  72,  73,  74,
     75,  76,  76,  77,  78,  79,  80,  81,  82,  83,  84,  85,  86,  87,  88,  89,
     91,  93,  95,  96,  98, 100, 101, 102, 104, 106, 108, 110, 112, 114, 116, 118,
    122, 124, 126, 128, 130, 132, 134, 136, 138, 140, 143, 145, 148, 151, 154, 157,
};

/* AC quantizer lookup */
static const uint16_t kAcQLookup[128] = {
      4,   5,   6,   7,   8,   9,  10,  11,  12,  13,  14,  15,  16,  17,  18,  19,
     20,  21,  22,  23,  24,  25,  26,  27,  28,  29,  30,  31,  32,  33,  34,  35,
     36,  37,  38,  39,  40,  41,  42,  43,  44,  45,  46,  47,  48,  49,  50,  51,
     52,  53,  54,  55,  56,  57,  58,  60,  62,  64,  66,  68,  70,  72,  74,  76,
     78,  80,  82,  84,  86,  88,  90,  92,  94,  96,  98, 100, 102, 104, 106, 108,
    110, 112, 114, 116, 119, 122, 125, 128, 131, 134, 137, 140, 143, 146, 149, 152,
    155, 158, 161, 164, 167, 170, 173, 177, 181, 185, 189, 193, 197, 201, 205, 209,
    213, 217, 221, 225, 229, 234, 239, 245, 249, 254, 259, 264, 269, 274, 279, 284,
};

/* Zigzag scan order for VP8 4x4 DCT */
static const uint8_t kZigzag[16] = {
    0, 1, 4, 8, 5, 2, 3, 6, 9, 12, 13, 10, 7, 11, 14, 15
};

/* Coefficient band for each DCT position */
static const uint8_t kBands[17] = {
    0, 1, 2, 3, 6, 4, 5, 6, 6, 6, 6, 6, 6, 6, 6, 7, 0
};

/* Key-frame Y mode probabilities */
static const uint8_t kKfYModeProb[4] = { 145, 156, 163, 128 };

/* Key-frame UV mode probabilities */
static const uint8_t kKfUvModeProb[3] = { 142, 114, 183 };

/* Key-frame B mode context probabilities [above_mode][left_mode][9 tree nodes] */
static const uint8_t kKfBModeProb[10][10][9] = {
    { {231,120, 48, 89,115,113,120,152,112}, {152,179, 64,126,170,118, 46, 70, 95},
      {175, 69,143, 80, 85, 82, 72,155,103}, { 56, 58, 10,171,218,189, 17, 13,152},
      {144, 71, 10, 38,171,213,144, 34, 26}, {114, 26, 17,163, 44,195, 21, 10,173},
      {121, 24, 80,195, 26, 62, 44, 64, 85}, {170, 46, 55, 19,136,160, 33,206, 71},
      { 63, 20,  8,114,114,208, 12,  9,226}, {81,40,11,96,182,84,29,16,36} },
    { {134,183, 89,137, 98,101,106,165,148}, {72,187,100,130,157,111, 32, 75, 80},
      {66,102,167, 99, 74, 62, 40,234,128}, {41, 53,  9,178,241,141, 26,  8,107},
      {104, 79, 12, 27,217,255, 87, 17,  7}, { 74, 43, 26,146, 73,166, 49, 23,157},
      { 65, 38, 105,160, 51, 52, 31,115,128}, {87, 68, 71, 44,114, 51, 15,186, 23},
      { 47, 41, 14,110,182,183, 21, 17,194}, {66,45,25,102,197,189,23,18,22} },
    { {88,88,147,150, 42, 46, 45,196,205}, {43,97,183,117, 85, 38, 35,179, 61},
      {39, 53,200, 87, 26, 21, 43,232,171}, {56, 34, 51,104,114,102, 29, 93,77},
      { 107, 54, 32, 26, 51,  1, 81, 43, 31}, {39, 28, 85,171, 58, 165, 41, 18,142},
      { 52, 16, 133,101, 20, 23, 26,121,221}, {57, 24, 42, 66,100, 54, 14,194, 72},
      {22, 17, 17,140,172,183, 24, 10,245}, {85,39,68,64,145,35,22,20,9} },
    { { 65, 70, 60,155,159,145, 68, 47,156}, { 57,44,69,140,155,118, 32, 34,147},
      { 49,25,137,108, 55, 41, 50,207,164}, { 29,27,21,136,219,164, 27, 11,107},
      { 100,42,17, 41,221,247, 88, 21,  7}, { 71,20, 38,119, 68,179, 90, 14,193},
      {  61,21,102,104, 27, 48, 36,128,  9}, { 67,44,59, 29,137,108, 21,212, 49},
      {  31,24, 12,  9,129,180, 34, 13,241}, {54,29,15,79,189,169,22,13,29} },
    { {  86,143, 45,115,145,172, 17, 21,114}, { 55,116, 76,135,172,111, 19, 20, 80},
      { 49, 40,152,141, 71, 31, 48,169,183}, { 30,43,  9,140,203,236, 18,  7, 92},
      {  95,40, 15, 20,220,243, 67, 15,  9}, { 57,28, 28,136, 75,166, 35, 10,146},
      {  82,23, 44,154, 42, 32, 44,118,122}, { 63,39, 40, 24,141,173, 15,194, 40},
      {  22,24, 13, 74,174,199, 30, 13,226}, {73,30,22,67,179,179,27,13,29} },
    { {  57, 47, 42,156,155,196, 60, 16,168}, { 30, 62, 66,157,161,112, 31, 16,114},
      { 29, 20,116,110, 64, 55, 31,184,233}, { 12, 19, 14,181,232,253, 15,  4, 98},
      {  72, 33, 23, 27,212,255,109, 27,  5}, { 25, 17, 36,158, 82,163, 55,  9,176},
      {  51, 10, 70,110, 34, 39, 32,118,231}, { 44, 25, 50, 20,175,176, 12,209, 43},
      {  12, 16,  3,103,194,201, 28,  6,248}, {47,18,14,80,196,195,27,12,35} },
    { { 117, 62, 84, 79,116,135, 54, 40,128}, { 64, 80, 88,126,147, 88, 23, 38,101},
      { 42, 38,149, 85, 62, 47, 30,172,148}, { 45, 29, 14,165,218,236, 21,  9,100},
      { 120,36, 11, 31,199,238, 82, 15,  6}, { 59, 22, 36,162, 87,174, 57, 10,162},
      {  62, 13, 92,126, 48, 38, 37,102,202}, { 92, 33, 67, 18,137,126, 18,222, 51},
      {  29, 18, 12, 85,185,199, 32, 12,234}, {82,28,24,72,162,167,24,16,27} },
    { {  91, 56, 56,108,125,157, 47, 93,139}, { 53, 62, 61,126,145,109, 19, 49,101},
      { 41, 36,154, 95, 47, 36, 31,198,187}, { 26, 38, 12,173,232,253, 13,  6,100},
      { 101,42, 14, 27,211,245, 71, 17,  3}, { 37, 20, 39,149, 68,178, 56,  7,162},
      {  52, 15,100,119, 38, 38, 35,123,208}, { 69, 35, 56, 17,163,159, 13,208, 45},
      {  13, 13,  5, 95,172,208, 30,  8,243}, {67,29,19,73,178,189,25,12,28} },
    { {  56, 41, 51,150,173,203, 42, 14,183}, { 27, 45, 48,146,174,122, 20, 14,117},
      { 23, 15,113,112, 63, 51, 28,196,225}, {  8, 16,  4,164,233,255, 19,  4,106},
      {  60, 36, 17, 17,215,255,109, 31,  5}, { 20, 14, 24,143, 71,167, 49,  4,189},
      {  39,  8, 67,118, 37, 35, 27,120,235}, { 43, 17, 50, 14,188,189, 15,227, 48},
      {   9, 13,  4, 96,183,214, 36,  8,248}, {41,16,10,67,188,209,27,10,42} },
    { { 102, 61, 71,123,113,148, 46, 50,135}, { 64, 67, 77,117,136,103, 23, 41, 98},
      { 48, 31,141, 98, 59, 44, 34,185,162}, { 30, 37, 17,159,210,227, 17, 10,110},
      { 107,42, 16, 27,218,250, 85, 18,  5}, { 52, 22, 34,148, 67,176, 49,  9,163},
      {  65, 16, 97,126, 39, 38, 36,114,190}, { 75, 36, 58, 22,147,148, 14,207, 50},
      {  19, 17,  9, 89,176,200, 30, 11,238}, {71,30,17,76,177,177,24,13,28} },
};

/* Default coefficient probabilities (RFC 6386 Section 13.5) */
static const uint8_t kDefaultCoeffProbs[4][8][3][11] = {
  { { {128,128,128,128,128,128,128,128,128,128,128},
      {128,128,128,128,128,128,128,128,128,128,128},
      {128,128,128,128,128,128,128,128,128,128,128} },
    { {253,136,254,255,228,219,128,128,128,128,128},
      {189,129,242,255,227,213,255,219,128,128,128},
      {106,126,227,252,214,209,255,255,128,128,128} },
    { {1,98,248,255,236,226,255,255,128,128,128},
      {181,133,238,254,221,234,255,154,128,128,128},
      {78,134,202,247,198,180,255,219,128,128,128} },
    { {1,185,249,255,243,255,128,128,128,128,128},
      {184,150,247,255,236,224,128,128,128,128,128},
      {77,110,216,255,236,230,128,128,128,128,128} },
    { {1,101,251,255,241,255,128,128,128,128,128},
      {170,139,241,252,236,209,255,255,128,128,128},
      {37,116,196,243,228,255,255,255,128,128,128} },
    { {1,204,254,255,245,255,128,128,128,128,128},
      {207,160,250,255,238,128,128,128,128,128,128},
      {102,103,231,255,211,171,128,128,128,128,128} },
    { {1,152,252,255,240,255,128,128,128,128,128},
      {177,135,243,255,234,225,128,128,128,128,128},
      {80,129,211,255,194,224,128,128,128,128,128} },
    { {1,1,255,128,128,128,128,128,128,128,128},
      {246,1,255,128,128,128,128,128,128,128,128},
      {255,128,128,128,128,128,128,128,128,128,128} } },
  { { {198,35,237,223,193,187,162,160,145,155,62},
      {131,45,198,221,172,176,220,157,252,221,1},
      {68,47,146,208,149,167,221,162,255,223,128} },
    { {1,149,241,255,221,224,255,255,128,128,128},
      {184,141,234,253,222,220,255,199,128,128,128},
      {81,99,181,242,176,190,249,202,255,255,128} },
    { {1,129,232,253,214,197,242,196,255,255,128},
      {99,121,210,250,201,198,255,202,128,128,128},
      {23,91,163,242,170,187,247,210,255,255,128} },
    { {1,200,246,255,234,255,128,128,128,128,128},
      {109,178,241,255,231,245,255,255,128,128,128},
      {44,130,201,253,205,192,255,255,128,128,128} },
    { {1,132,239,251,219,209,255,165,128,128,128},
      {94,136,225,251,218,190,255,255,128,128,128},
      {22,100,174,245,186,161,255,199,128,128,128} },
    { {1,182,249,255,232,235,128,128,128,128,128},
      {124,143,241,255,227,234,128,128,128,128,128},
      {35,77,181,251,193,211,255,205,128,128,128} },
    { {1,157,247,255,236,231,255,255,128,128,128},
      {121,141,235,255,225,227,255,255,128,128,128},
      {45,99,188,251,195,217,255,224,128,128,128} },
    { {1,1,251,255,213,255,128,128,128,128,128},
      {203,1,248,255,255,128,128,128,128,128,128},
      {137,1,177,255,224,255,128,128,128,128,128} } },
  { { {253,9,248,251,207,208,255,192,128,128,128},
      {175,13,224,243,193,185,249,198,255,255,128},
      {73,17,171,221,161,179,236,167,255,234,128} },
    { {1,95,247,253,212,183,255,255,128,128,128},
      {239,90,244,250,211,209,255,255,128,128,128},
      {155,77,195,248,188,195,255,255,128,128,128} },
    { {1,24,239,251,218,219,255,205,128,128,128},
      {201,51,219,255,196,186,128,128,128,128,128},
      {69,46,190,239,201,218,255,228,128,128,128} },
    { {1,191,251,255,255,128,128,128,128,128,128},
      {223,165,249,255,213,255,128,128,128,128,128},
      {141,124,248,255,255,128,128,128,128,128,128} },
    { {1,16,248,255,255,128,128,128,128,128,128},
      {190,36,230,255,236,255,128,128,128,128,128},
      {149,1,255,128,128,128,128,128,128,128,128} },
    { {1,226,255,128,128,128,128,128,128,128,128},
      {247,192,255,128,128,128,128,128,128,128,128},
      {240,128,255,128,128,128,128,128,128,128,128} },
    { {1,134,252,255,255,128,128,128,128,128,128},
      {213,62,250,255,255,128,128,128,128,128,128},
      {55,93,255,128,128,128,128,128,128,128,128} },
    { {128,128,128,128,128,128,128,128,128,128,128},
      {128,128,128,128,128,128,128,128,128,128,128},
      {128,128,128,128,128,128,128,128,128,128,128} } },
  { { {202,24,213,235,186,191,220,160,240,175,255},
      {126,38,182,232,169,184,228,174,255,187,128},
      {61,46,138,219,151,178,240,170,255,216,128} },
    { {1,112,230,250,199,191,247,159,255,255,128},
      {166,109,228,252,211,215,255,174,128,128,128},
      {39,77,162,232,172,180,245,178,255,255,128} },
    { {1,52,220,246,198,199,249,220,255,255,128},
      {124,74,191,243,183,193,250,221,255,255,128},
      {24,71,130,219,154,170,243,182,255,255,128} },
    { {1,182,225,249,219,240,255,224,128,128,128},
      {149,150,226,252,216,205,255,171,128,128,128},
      {28,108,170,242,183,194,254,223,255,255,128} },
    { {1,81,230,252,204,203,255,192,128,128,128},
      {123,102,209,247,188,196,255,233,128,128,128},
      {20,95,153,243,164,173,255,203,128,128,128} },
    { {1,222,248,255,216,213,128,128,128,128,128},
      {168,175,246,252,235,205,255,255,128,128,128},
      {47,116,215,255,211,212,255,255,128,128,128} },
    { {1,121,236,253,212,214,255,255,128,128,128},
      {141,84,213,252,201,202,255,219,128,128,128},
      {42,80,160,240,162,185,255,205,128,128,128} },
    { {1,1,255,128,128,128,128,128,128,128,128},
      {244,1,255,128,128,128,128,128,128,128,128},
      {238,1,255,128,128,128,128,128,128,128,128} } }
};

/* Probability update table for coefficient probs */
static const uint8_t kCoeffUpdateProbs[4][8][3][11] = {
  { { {255,255,255,255,255,255,255,255,255,255,255},
      {255,255,255,255,255,255,255,255,255,255,255},
      {255,255,255,255,255,255,255,255,255,255,255} },
    { {176,246,255,255,255,255,255,255,255,255,255},
      {223,241,252,255,255,255,255,255,255,255,255},
      {249,253,253,255,255,255,255,255,255,255,255} },
    { {255,244,252,255,255,255,255,255,255,255,255},
      {234,254,254,255,255,255,255,255,255,255,255},
      {253,255,255,255,255,255,255,255,255,255,255} },
    { {255,246,254,255,255,255,255,255,255,255,255},
      {239,253,254,255,255,255,255,255,255,255,255},
      {254,255,254,255,255,255,255,255,255,255,255} },
    { {255,248,254,255,255,255,255,255,255,255,255},
      {251,255,254,255,255,255,255,255,255,255,255},
      {255,255,255,255,255,255,255,255,255,255,255} },
    { {255,253,254,255,255,255,255,255,255,255,255},
      {251,254,254,255,255,255,255,255,255,255,255},
      {254,255,254,255,255,255,255,255,255,255,255} },
    { {255,254,253,255,254,255,255,255,255,255,255},
      {250,255,254,255,254,255,255,255,255,255,255},
      {254,255,255,255,255,255,255,255,255,255,255} },
    { {255,255,255,255,255,255,255,255,255,255,255},
      {255,255,255,255,255,255,255,255,255,255,255},
      {255,255,255,255,255,255,255,255,255,255,255} } },
  { { {217,255,255,255,255,255,255,255,255,255,255},
      {225,252,241,253,255,255,254,255,255,255,255},
      {234,250,241,250,253,255,253,254,255,255,255} },
    { {255,254,255,255,255,255,255,255,255,255,255},
      {223,254,254,255,255,255,255,255,255,255,255},
      {238,253,254,254,255,255,255,255,255,255,255} },
    { {255,248,254,255,255,255,255,255,255,255,255},
      {249,254,255,255,255,255,255,255,255,255,255},
      {255,255,255,255,255,255,255,255,255,255,255} },
    { {255,253,255,255,255,255,255,255,255,255,255},
      {247,254,255,255,255,255,255,255,255,255,255},
      {255,255,255,255,255,255,255,255,255,255,255} },
    { {255,253,254,255,255,255,255,255,255,255,255},
      {252,255,255,255,255,255,255,255,255,255,255},
      {255,255,255,255,255,255,255,255,255,255,255} },
    { {255,254,254,255,255,255,255,255,255,255,255},
      {253,255,255,255,255,255,255,255,255,255,255},
      {255,255,255,255,255,255,255,255,255,255,255} },
    { {255,254,253,255,255,255,255,255,255,255,255},
      {250,255,255,255,255,255,255,255,255,255,255},
      {254,255,255,255,255,255,255,255,255,255,255} },
    { {255,255,255,255,255,255,255,255,255,255,255},
      {255,255,255,255,255,255,255,255,255,255,255},
      {255,255,255,255,255,255,255,255,255,255,255} } },
  { { {186,251,250,255,255,255,255,255,255,255,255},
      {234,251,244,254,255,255,255,255,255,255,255},
      {251,251,243,253,254,255,254,255,255,255,255} },
    { {255,253,254,255,255,255,255,255,255,255,255},
      {236,253,254,255,255,255,255,255,255,255,255},
      {251,253,253,254,254,255,255,255,255,255,255} },
    { {255,254,254,255,255,255,255,255,255,255,255},
      {254,254,254,255,255,255,255,255,255,255,255},
      {255,255,255,255,255,255,255,255,255,255,255} },
    { {255,254,255,255,255,255,255,255,255,255,255},
      {254,254,255,255,255,255,255,255,255,255,255},
      {254,255,255,255,255,255,255,255,255,255,255} },
    { {255,255,255,255,255,255,255,255,255,255,255},
      {254,255,255,255,255,255,255,255,255,255,255},
      {255,255,255,255,255,255,255,255,255,255,255} },
    { {255,255,255,255,255,255,255,255,255,255,255},
      {255,255,255,255,255,255,255,255,255,255,255},
      {255,255,255,255,255,255,255,255,255,255,255} },
    { {255,255,255,255,255,255,255,255,255,255,255},
      {255,255,255,255,255,255,255,255,255,255,255},
      {255,255,255,255,255,255,255,255,255,255,255} },
    { {255,255,255,255,255,255,255,255,255,255,255},
      {255,255,255,255,255,255,255,255,255,255,255},
      {255,255,255,255,255,255,255,255,255,255,255} } },
  { { {248,255,255,255,255,255,255,255,255,255,255},
      {250,254,252,254,255,255,255,255,255,255,255},
      {248,254,249,253,255,255,255,255,255,255,255} },
    { {255,253,253,255,255,255,255,255,255,255,255},
      {246,253,253,255,255,255,255,255,255,255,255},
      {252,254,251,254,254,255,255,255,255,255,255} },
    { {255,254,252,255,255,255,255,255,255,255,255},
      {248,254,253,255,255,255,255,255,255,255,255},
      {253,255,254,254,255,255,255,255,255,255,255} },
    { {255,251,254,255,255,255,255,255,255,255,255},
      {245,251,254,255,255,255,255,255,255,255,255},
      {253,253,254,255,255,255,255,255,255,255,255} },
    { {255,251,253,255,255,255,255,255,255,255,255},
      {252,253,254,255,255,255,255,255,255,255,255},
      {255,254,255,255,255,255,255,255,255,255,255} },
    { {255,252,255,255,255,255,255,255,255,255,255},
      {249,255,254,255,255,255,255,255,255,255,255},
      {255,255,254,255,255,255,255,255,255,255,255} },
    { {255,255,253,255,255,255,255,255,255,255,255},
      {250,255,255,255,255,255,255,255,255,255,255},
      {255,255,255,255,255,255,255,255,255,255,255} },
    { {255,255,255,255,255,255,255,255,255,255,255},
      {254,255,255,255,255,255,255,255,255,255,255},
      {255,255,255,255,255,255,255,255,255,255,255} } }
};

/* ══════════════════════════════════════════════════════════════════════
 *  Boolean (Arithmetic) Decoder - RFC 6386 Section 7
 * ══════════════════════════════════════════════════════════════════════ */

typedef struct {
    const uint8_t* buf;
    const uint8_t* buf_end;
    uint32_t range;
    uint32_t value;
    int      bits_left;
    int      eof;
} VP8BoolReader;

static void vp8_br_init(VP8BoolReader* br, const uint8_t* buf, size_t size) {
    br->buf = buf + (size >= 2 ? 2 : size);
    br->buf_end = buf + size;
    br->range = 255;
    br->value = size >= 2
        ? ((uint32_t)buf[0] << 8) | (uint32_t)buf[1]
        : 0;
    br->bits_left = 0;
    br->eof = size < 2;
}

static int vp8_br_get(VP8BoolReader* br, int prob) {
    uint32_t split = 1u + (((br->range - 1u) * (uint32_t)prob) >> 8);
    uint32_t bigsplit = split << 8;
    int bit;

    if (br->value >= bigsplit) {
        bit = 1;
        br->range -= split;
        br->value -= bigsplit;
    } else {
        bit = 0;
        br->range = split;
    }

    while (br->range < 128u) {
        br->value <<= 1;
        br->range <<= 1;
        if (++br->bits_left == 8) {
            br->bits_left = 0;
            if (br->buf < br->buf_end) {
                br->value |= *br->buf++;
            } else {
                br->eof = 1;
            }
        }
    }
    return bit;
}

static int vp8_br_get_bit(VP8BoolReader* br) {
    return vp8_br_get(br, 128);
}

static int vp8_br_get_value(VP8BoolReader* br, int bits) {
    int v = 0, i;
    for (i = bits - 1; i >= 0; i--) {
        v |= vp8_br_get_bit(br) << i;
    }
    return v;
}

static int vp8_br_get_signed(VP8BoolReader* br, int bits) {
    int v = vp8_br_get_value(br, bits);
    return vp8_br_get_bit(br) ? -v : v;
}

/* ══════════════════════════════════════════════════════════════════════
 *  Decoder State
 * ══════════════════════════════════════════════════════════════════════ */

typedef struct {
    uint16_t y1_dc, y1_ac;
    uint16_t y2_dc, y2_ac;
    uint16_t uv_dc, uv_ac;
} VP8QuantMat;

typedef struct {
    /* Frame info */
    int width, height;
    int mb_w, mb_h;

    /* Bool readers */
    VP8BoolReader br_hdr;    /* header partition */
    VP8BoolReader br_parts[8]; /* token partitions */
    int num_parts;

    /* Segment */
    int use_segment;
    int segment_update_map;
    int segment_abs_delta;
    int8_t seg_quant[4];
    int8_t seg_lf[4];
    uint8_t seg_prob[3];

    /* Loop filter */
    int filter_simple;
    int filter_level;
    int filter_sharpness;
    int filter_use_delta;
    int8_t filter_ref_delta[4];
    int8_t filter_mode_delta[4];

    /* Quantization */
    int base_q;
    int y1_dc_delta, y2_dc_delta, y2_ac_delta, uv_dc_delta, uv_ac_delta;
    VP8QuantMat quant[4]; /* per segment */

    /* Coefficient probabilities */
    uint8_t coeff_probs[4][8][3][11];

    /* Per-MB context */
    uint8_t top_nz_y[VP8_MAX_MB_W * 4];
    uint8_t top_nz_u[VP8_MAX_MB_W * 2];
    uint8_t top_nz_v[VP8_MAX_MB_W * 2];
    uint8_t top_nz_dc[VP8_MAX_MB_W];
    uint8_t top_bmode[VP8_MAX_MB_W * 4];
    uint8_t left_nz_y_ctx[4];
    uint8_t left_nz_u_ctx[2];
    uint8_t left_nz_v_ctx[2];
    uint8_t left_nz_dc_ctx;
    uint8_t left_bmode[4];

    /* MB info for current row */
    uint8_t mb_segment[VP8_MAX_MB_W];
    uint8_t mb_skip[VP8_MAX_MB_W];
    uint8_t mb_y_mode[VP8_MAX_MB_W];
    uint8_t mb_uv_mode[VP8_MAX_MB_W];
    uint8_t mb_is_i4x4[VP8_MAX_MB_W];
    uint8_t mb_has_coeff[VP8_MAX_MB_W];
    uint8_t mb_imodes[VP8_MAX_MB_W][16];
    int16_t mb_coeffs[VP8_MAX_MB_W][25][16];

    /* Skip coeff feature */
    int skip_enabled;
    uint8_t skip_prob;

    /* Reconstructed pixel rows */
    uint8_t* y_row;  /* malloc'd, mb_w*16 * 16 */
    uint8_t* u_row;
    uint8_t* v_row;
    uint8_t* y_prev; /* previous reconstructed MB row, retained for filtering */
    uint8_t* u_prev;
    uint8_t* v_prev;
    /* Top reference pixels */
    uint8_t* top_y;  /* malloc'd, mb_w*16 + 8 */
    uint8_t* top_u;
    uint8_t* top_v;
    /* Left reference pixels */
    uint8_t left_y[16];
    uint8_t left_u[8];
    uint8_t left_v[8];
    uint8_t top_left_y, top_left_u, top_left_v;
} VP8Decoder;

/* ══════════════════════════════════════════════════════════════════════
 *  Frame Header Parsing
 * ══════════════════════════════════════════════════════════════════════ */

static int vp8_parse_frame_header(VP8Decoder* dec, const uint8_t* data, size_t size,
                                   size_t* header_offset) {
    uint32_t tag;
    int key_frame, version, first_part_size;

    if (size < 10) return -1;

    tag = (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16);
    key_frame = !(tag & 1);
    version = (tag >> 1) & 7;
    /* show_frame = (tag >> 4) & 1; -- not needed */
    first_part_size = (tag >> 5);

    (void)version;

    if (!key_frame) return -1; /* Only key frames for WebP */

    /* Check start code */
    if (data[3] != 0x9d || data[4] != 0x01 || data[5] != 0x2a) return -1;

    dec->width  = ((int)data[6] | ((int)data[7] << 8)) & 0x3fff;
    dec->height = ((int)data[8] | ((int)data[9] << 8)) & 0x3fff;

    if (dec->width <= 0 || dec->height <= 0) return -1;
    if (dec->width > VP8_MAX_WIDTH || dec->height > VP8_MAX_WIDTH) return -1;

    dec->mb_w = (dec->width + 15) >> 4;
    dec->mb_h = (dec->height + 15) >> 4;

    *header_offset = 10;
    {
        size_t part0_end = 10 + (size_t)first_part_size;
        if (part0_end > size) return -1;

        /* Initialize header bool reader */
        vp8_br_init(&dec->br_hdr, data + 10, (size_t)first_part_size);

        /* Parse color space & clamping (ignored) */
        vp8_br_get_bit(&dec->br_hdr); /* color_space */
        vp8_br_get_bit(&dec->br_hdr); /* clamping_type */

        /* Parse segments */
        dec->use_segment = vp8_br_get_bit(&dec->br_hdr);
        if (dec->use_segment) {
            dec->segment_update_map = vp8_br_get_bit(&dec->br_hdr);
            {
                int update_data = vp8_br_get_bit(&dec->br_hdr);
                if (update_data) {
                    int s;
                    dec->segment_abs_delta = vp8_br_get_bit(&dec->br_hdr);
                    for (s = 0; s < 4; s++) {
                        if (vp8_br_get_bit(&dec->br_hdr))
                            dec->seg_quant[s] = (int8_t)vp8_br_get_signed(&dec->br_hdr, 7);
                        else
                            dec->seg_quant[s] = 0;
                    }
                    for (s = 0; s < 4; s++) {
                        if (vp8_br_get_bit(&dec->br_hdr))
                            dec->seg_lf[s] = (int8_t)vp8_br_get_signed(&dec->br_hdr, 6);
                        else
                            dec->seg_lf[s] = 0;
                    }
                }
            }
            if (dec->segment_update_map) {
                int s;
                for (s = 0; s < 3; s++) {
                    if (vp8_br_get_bit(&dec->br_hdr))
                        dec->seg_prob[s] = (uint8_t)vp8_br_get_value(&dec->br_hdr, 8);
                    else
                        dec->seg_prob[s] = 255;
                }
            }
        }

        /* Parse filter */
        dec->filter_simple = vp8_br_get_bit(&dec->br_hdr);
        dec->filter_level = vp8_br_get_value(&dec->br_hdr, 6);
        dec->filter_sharpness = vp8_br_get_value(&dec->br_hdr, 3);
        dec->filter_use_delta = vp8_br_get_bit(&dec->br_hdr);
        if (dec->filter_use_delta) {
            if (vp8_br_get_bit(&dec->br_hdr)) {
                int i;
                for (i = 0; i < 4; i++) {
                    if (vp8_br_get_bit(&dec->br_hdr))
                        dec->filter_ref_delta[i] = (int8_t)vp8_br_get_signed(&dec->br_hdr, 6);
                }
                for (i = 0; i < 4; i++) {
                    if (vp8_br_get_bit(&dec->br_hdr))
                        dec->filter_mode_delta[i] = (int8_t)vp8_br_get_signed(&dec->br_hdr, 6);
                }
            }
        }

        /* Parse partitions */
        {
            int log2_parts = vp8_br_get_value(&dec->br_hdr, 2);
            int p;
            dec->num_parts = 1 << log2_parts;

            /* Partition sizes are after the first partition */
            {
                const uint8_t* part_data = data + part0_end;
                size_t part_remain = size - part0_end;
                size_t sizes_bytes = (size_t)(dec->num_parts - 1) * 3;

                if (part_remain < sizes_bytes) return -1;

                size_t offset = sizes_bytes;
                for (p = 0; p < dec->num_parts - 1; p++) {
                    uint32_t psz = (uint32_t)part_data[p * 3] |
                                   ((uint32_t)part_data[p * 3 + 1] << 8) |
                                   ((uint32_t)part_data[p * 3 + 2] << 16);
                    if (offset + psz > part_remain) return -1;
                    vp8_br_init(&dec->br_parts[p], part_data + offset, psz);
                    offset += psz;
                }
                /* Last partition gets remaining data */
                if (offset > part_remain) return -1;
                vp8_br_init(&dec->br_parts[dec->num_parts - 1],
                             part_data + offset, part_remain - offset);
            }
        }

        /* Parse quantization */
        dec->base_q = vp8_br_get_value(&dec->br_hdr, 7);
        dec->y1_dc_delta = vp8_br_get_bit(&dec->br_hdr) ? vp8_br_get_signed(&dec->br_hdr, 4) : 0;
        dec->y2_dc_delta = vp8_br_get_bit(&dec->br_hdr) ? vp8_br_get_signed(&dec->br_hdr, 4) : 0;
        dec->y2_ac_delta = vp8_br_get_bit(&dec->br_hdr) ? vp8_br_get_signed(&dec->br_hdr, 4) : 0;
        dec->uv_dc_delta = vp8_br_get_bit(&dec->br_hdr) ? vp8_br_get_signed(&dec->br_hdr, 4) : 0;
        dec->uv_ac_delta = vp8_br_get_bit(&dec->br_hdr) ? vp8_br_get_signed(&dec->br_hdr, 4) : 0;

        /* Build quantization matrices */
        {
            int s;
            for (s = 0; s < 4; s++) {
                int q = dec->base_q;
                if (dec->use_segment) {
                    if (dec->segment_abs_delta) q = dec->seg_quant[s];
                    else q += dec->seg_quant[s];
                }
                if (q < 0) q = 0;
                if (q > 127) q = 127;
                {
                    int dc, ac;
                    dc = q + dec->y1_dc_delta; if (dc < 0) dc = 0; if (dc > 127) dc = 127;
                    dec->quant[s].y1_dc = kDcQLookup[dc];
                    dec->quant[s].y1_ac = kAcQLookup[q];

                    dc = q + dec->y2_dc_delta; if (dc < 0) dc = 0; if (dc > 127) dc = 127;
                    ac = q + dec->y2_ac_delta; if (ac < 0) ac = 0; if (ac > 127) ac = 127;
                    dec->quant[s].y2_dc = kDcQLookup[dc] * 2;
                    dec->quant[s].y2_ac = kAcQLookup[ac] * 155 / 100;
                    if (dec->quant[s].y2_ac < 8) dec->quant[s].y2_ac = 8;

                    dc = q + dec->uv_dc_delta; if (dc < 0) dc = 0; if (dc > 127) dc = 127;
                    ac = q + dec->uv_ac_delta; if (ac < 0) ac = 0; if (ac > 127) ac = 127;
                    dec->quant[s].uv_dc = kDcQLookup[dc];
                    if (dec->quant[s].uv_dc > 132) dec->quant[s].uv_dc = 132;
                    dec->quant[s].uv_ac = kAcQLookup[ac];
                }
            }
        }

        /*
         * RFC 6386 section 9.6:
         * refresh_entropy_probs is present in the frame header even for key
         * frames. We keep per-frame probabilities only, but still must
         * consume the flag bit before parsing skip/coefficient probability
         * updates. Missing this bit misaligns the remaining partition-0
         * stream and breaks chroma-heavy real-world VP8 WebP images.
         */
        (void)vp8_br_get_bit(&dec->br_hdr);

        /* Coefficient probabilities */
        memcpy(dec->coeff_probs, kDefaultCoeffProbs, sizeof(dec->coeff_probs));
        {
            int t, b, c, p;
            for (t = 0; t < 4; t++)
                for (b = 0; b < 8; b++)
                    for (c = 0; c < 3; c++)
                        for (p = 0; p < 11; p++) {
                            if (vp8_br_get(&dec->br_hdr, kCoeffUpdateProbs[t][b][c][p]))
                                dec->coeff_probs[t][b][c][p] =
                                    (uint8_t)vp8_br_get_value(&dec->br_hdr, 8);
                        }
        }

        /* Coefficient skip mode follows the complete probability update
         * table in the VP8 entropy header. */
        dec->skip_enabled = vp8_br_get_bit(&dec->br_hdr);
        if (dec->skip_enabled) {
            dec->skip_prob = (uint8_t)vp8_br_get_value(&dec->br_hdr, 8);
        }
        if (dec->br_hdr.eof) return -1;
    }
    return 0;
}

/* ══════════════════════════════════════════════════════════════════════
 *  Macroblock Mode Decoding
 * ══════════════════════════════════════════════════════════════════════ */

static int vp8_read_tree(VP8BoolReader* br, const uint8_t* probs) {
    /* RFC 6386 Section 11.2: B_PRED, DC, V, H, TM */
    if (!vp8_br_get(br, probs[0])) {
        return 4; /* B_PRED */
    }
    if (!vp8_br_get(br, probs[1])) {
        return vp8_br_get(br, probs[2]) ? 1 : 0; /* V or DC */
    }
    return vp8_br_get(br, probs[3]) ? 3 : 2; /* TM or H */
}

static int vp8_read_bmode(VP8BoolReader* br, const uint8_t* probs) {
    /*
     * RFC 6386 Section 11.2 / 11.3:
     * B_DC, B_TM, B_VE, B_HE, B_LD, B_RD, B_VR, B_VL, B_HD, B_HU
     */
    if (!vp8_br_get(br, probs[0])) return 0; /* B_DC */
    if (!vp8_br_get(br, probs[1])) return 1; /* B_TM */
    if (!vp8_br_get(br, probs[2])) return 2; /* B_VE */
    if (!vp8_br_get(br, probs[3])) {
        if (!vp8_br_get(br, probs[4])) return 3; /* B_HE */
        return vp8_br_get(br, probs[5]) ? 6 : 5; /* B_VR or B_RD */
    }
    if (!vp8_br_get(br, probs[6])) return 4; /* B_LD */
    if (!vp8_br_get(br, probs[7])) return 7; /* B_VL */
    return vp8_br_get(br, probs[8]) ? 9 : 8; /* B_HU or B_HD */
}

static int vp8_read_uv_mode(VP8BoolReader* br, const uint8_t* probs) {
    if (!vp8_br_get(br, probs[0])) return 0; /* DC */
    if (!vp8_br_get(br, probs[1])) return 1; /* V */
    return vp8_br_get(br, probs[2]) ? 3 : 2; /* TM or H */
}

static uint8_t vp8_ymode_to_bmode(int y_mode) {
    switch (y_mode) {
        case 0: return 0; /* DC_PRED -> B_DC_PRED */
        case 1: return 2; /* V_PRED  -> B_VE_PRED */
        case 2: return 3; /* H_PRED  -> B_HE_PRED */
        case 3: return 1; /* TM_PRED -> B_TM_PRED */
        default: return 0;
    }
}

static void vp8_decode_mb_modes(VP8Decoder* dec, int mb_x, int mb_y) {
    VP8BoolReader* br = &dec->br_hdr;
    int seg = 0;
    int skip = 0;

    (void)mb_y;

    if (dec->use_segment && dec->segment_update_map) {
        seg = vp8_br_get(br, dec->seg_prob[0]) ?
              (2 + vp8_br_get(br, dec->seg_prob[2])) :
              vp8_br_get(br, dec->seg_prob[1]);
    }
    dec->mb_segment[mb_x] = (uint8_t)seg;

    if (dec->skip_enabled) {
        skip = vp8_br_get(br, dec->skip_prob);
    }
    dec->mb_skip[mb_x] = (uint8_t)skip;

    {
        int y_mode = vp8_read_tree(br, kKfYModeProb);
        dec->mb_y_mode[mb_x] = (uint8_t)y_mode;

        if (y_mode == 4) { /* B_PRED */
            int i;
            dec->mb_is_i4x4[mb_x] = 1;
            for (i = 0; i < 16; i++) {
                /* Context: above and left sub-block modes */
                int above, left;
                int bx = i & 3, by = i >> 2;
                if (by == 0) {
                    above = dec->top_bmode[mb_x * 4 + bx];
                } else {
                    above = dec->mb_imodes[mb_x][(by - 1) * 4 + bx];
                }
                if (bx == 0) {
                    left = dec->left_bmode[by];
                } else {
                    left = dec->mb_imodes[mb_x][by * 4 + bx - 1];
                }
                dec->mb_imodes[mb_x][i] =
                    (uint8_t)vp8_read_bmode(br, kKfBModeProb[above][left]);
            }
            for (i = 0; i < 4; ++i) {
                dec->top_bmode[mb_x * 4 + i] = dec->mb_imodes[mb_x][12 + i];
                dec->left_bmode[i] = dec->mb_imodes[mb_x][i * 4 + 3];
            }
        } else {
            uint8_t derived_bmode = vp8_ymode_to_bmode(y_mode);
            int i;
            dec->mb_is_i4x4[mb_x] = 0;
            for (i = 0; i < 16; ++i) {
                dec->mb_imodes[mb_x][i] = derived_bmode;
            }
            for (i = 0; i < 4; ++i) {
                dec->top_bmode[mb_x * 4 + i] = derived_bmode;
                dec->left_bmode[i] = derived_bmode;
            }
        }
    }

    dec->mb_uv_mode[mb_x] = (uint8_t)vp8_read_uv_mode(br, kKfUvModeProb);
}

/* ══════════════════════════════════════════════════════════════════════
 *  Coefficient Decoding (Token Trees)
 * ══════════════════════════════════════════════════════════════════════ */

/* Token categories and extra bits (RFC 6386 section 13.2). */
static const int kCat1[] = {159};
static const int kCat2[] = {165, 145};
static const int kCat3[] = {173, 148, 140};
static const int kCat4[] = {176, 155, 140, 135};
static const int kCat5[] = {180, 157, 141, 134, 130};
static const int kCat6[] = {254, 254, 243, 230, 196, 177, 153, 140, 133, 130, 129};

static int vp8_read_coeff(VP8BoolReader* br, const int* probs, int n) {
    int v = 0, i;
    for (i = 0; i < n; i++) {
        v = (v << 1) | vp8_br_get(br, probs[i]);
    }
    return v;
}

static int vp8_read_token(VP8BoolReader* br, const uint8_t* probs,
                          int* next_context) {
    int v;

    if (!vp8_br_get(br, probs[2])) {
        *next_context = 1;
        return 1;
    }

    *next_context = 2;
    if (!vp8_br_get(br, probs[3])) {
        v = vp8_br_get(br, probs[4]);
        if (v) v += vp8_br_get(br, probs[5]);
        return v + 2;
    }

    if (!vp8_br_get(br, probs[6])) {
        if (!vp8_br_get(br, probs[7])) {
            return 5 + vp8_read_coeff(br, kCat1, 1);
        }
        return 7 + vp8_read_coeff(br, kCat2, 2);
    }

    {
        int a = vp8_br_get(br, probs[8]);
        int b = vp8_br_get(br, probs[9 + a]);
        int cat = (a << 1) + b;

        switch (cat) {
        case 0: return 11 + vp8_read_coeff(br, kCat3, 3);
        case 1: return 19 + vp8_read_coeff(br, kCat4, 4);
        case 2: return 35 + vp8_read_coeff(br, kCat5, 5);
        default: return 67 + vp8_read_coeff(br, kCat6, 11);
        }
    }
}

/*
 * Decode one 4x4 coefficient block. In VP8 a zero token advances directly to
 * the next token's ZERO/non-zero node; there is no EOB decision between a run
 * of zeroes. EOB is tested initially and after each non-zero coefficient.
 */
static int vp8_decode_block(VP8Decoder* dec, VP8BoolReader* br,
                            int type, int first, int initial_context,
                            int16_t* coeffs, int dc_quant, int ac_quant) {
    int n = first;
    int context = initial_context;
    const uint8_t* probs = dec->coeff_probs[type][kBands[n]][context];
    int last = 0;

    if (!vp8_br_get(br, probs[0])) return 0;

    while (n < 16) {
        while (!vp8_br_get(br, probs[1])) {
            if (++n == 16) return last;
            probs = dec->coeff_probs[type][kBands[n]][0];
        }

        {
            int v = vp8_read_token(br, probs, &context);
            if (vp8_br_get_bit(br)) v = -v;
            coeffs[kZigzag[n]] = (int16_t)(v * (n == 0 ? dc_quant : ac_quant));
            last = n + 1;
        }

        if (++n == 16) break;
        probs = dec->coeff_probs[type][kBands[n]][context];
        if (!vp8_br_get(br, probs[0])) break;
    }
    return last;
}

static void vp8_decode_mb_coeffs(VP8Decoder* dec, VP8BoolReader* br, int mb_x) {
    int seg = dec->mb_segment[mb_x];
    VP8QuantMat* qm = &dec->quant[seg];
    int is_i4x4 = dec->mb_is_i4x4[mb_x];
    uint8_t left_y[4];
    uint8_t above_y[4];
    uint8_t left_u[2];
    uint8_t above_u[2];
    uint8_t left_v[2];
    uint8_t above_v[2];
    int has_coeff = 0;
    int i;

    for (i = 0; i < 4; ++i) {
        left_y[i] = dec->left_nz_y_ctx[i];
        above_y[i] = dec->top_nz_y[mb_x * 4 + i];
    }
    for (i = 0; i < 2; ++i) {
        left_u[i] = dec->left_nz_u_ctx[i];
        above_u[i] = dec->top_nz_u[mb_x * 2 + i];
        left_v[i] = dec->left_nz_v_ctx[i];
        above_v[i] = dec->top_nz_v[mb_x * 2 + i];
    }

    memset(dec->mb_coeffs[mb_x], 0, sizeof(dec->mb_coeffs[mb_x]));
    dec->mb_has_coeff[mb_x] = 0;

    if (dec->mb_skip[mb_x]) {
        dec->top_nz_dc[mb_x] = 0;
        dec->left_nz_dc_ctx = 0;
        for (i = 0; i < 4; ++i) {
            dec->top_nz_y[mb_x * 4 + i] = 0;
            dec->left_nz_y_ctx[i] = 0;
        }
        for (i = 0; i < 2; ++i) {
            dec->top_nz_u[mb_x * 2 + i] = 0;
            dec->left_nz_u_ctx[i] = 0;
            dec->top_nz_v[mb_x * 2 + i] = 0;
            dec->left_nz_v_ctx[i] = 0;
        }
        return;
    }

    /* Decode Y2 (DC block) if not i4x4. */
    if (!is_i4x4) {
        int16_t* coeffs = dec->mb_coeffs[mb_x][24];
        int context = (dec->top_nz_dc[mb_x] ? 1 : 0) +
                      (dec->left_nz_dc_ctx ? 1 : 0);
        int nz = vp8_decode_block(dec, br, 1, 0, context, coeffs,
                                  qm->y2_dc, qm->y2_ac);

        has_coeff |= nz > 0;
        dec->top_nz_dc[mb_x] = (nz > 0) ? 1 : 0;
        dec->left_nz_dc_ctx = dec->top_nz_dc[mb_x];
    }

    /* Decode Y sub-blocks (0..15). */
    for (i = 0; i < 16; i++) {
        int16_t* coeffs = dec->mb_coeffs[mb_x][i];
        int bx = i & 3, by = i >> 2;
        int first = is_i4x4 ? 0 : 1; /* skip DC if not i4x4 (comes from Y2) */
        int type = is_i4x4 ? 3 : 0;
        int context = (left_y[by] ? 1 : 0) + (above_y[bx] ? 1 : 0);
        int nz = vp8_decode_block(dec, br, type, first, context, coeffs,
                                  qm->y1_dc, qm->y1_ac);

        has_coeff |= nz > 0;
        above_y[bx] = (nz > 0) ? 1 : 0;
        left_y[by] = (nz > 0) ? 1 : 0;
    }

    /* Decode U sub-blocks (16..19). */
    for (i = 0; i < 4; i++) {
        int16_t* coeffs = dec->mb_coeffs[mb_x][16 + i];
        int bx = i & 1, by = i >> 1;
        int context = (left_u[by] ? 1 : 0) + (above_u[bx] ? 1 : 0);
        int nz = vp8_decode_block(dec, br, 2, 0, context, coeffs,
                                  qm->uv_dc, qm->uv_ac);

        has_coeff |= nz > 0;
        above_u[bx] = (nz > 0) ? 1 : 0;
        left_u[by] = (nz > 0) ? 1 : 0;
    }

    /* Decode V sub-blocks (20..23). */
    for (i = 0; i < 4; i++) {
        int16_t* coeffs = dec->mb_coeffs[mb_x][20 + i];
        int bx = i & 1, by = i >> 1;
        int context = (left_v[by] ? 1 : 0) + (above_v[bx] ? 1 : 0);
        int nz = vp8_decode_block(dec, br, 2, 0, context, coeffs,
                                  qm->uv_dc, qm->uv_ac);

        has_coeff |= nz > 0;
        above_v[bx] = (nz > 0) ? 1 : 0;
        left_v[by] = (nz > 0) ? 1 : 0;
    }

    for (i = 0; i < 4; ++i) {
        dec->top_nz_y[mb_x * 4 + i] = above_y[i];
        dec->left_nz_y_ctx[i] = left_y[i];
    }
    for (i = 0; i < 2; ++i) {
        dec->top_nz_u[mb_x * 2 + i] = above_u[i];
        dec->left_nz_u_ctx[i] = left_u[i];
        dec->top_nz_v[mb_x * 2 + i] = above_v[i];
        dec->left_nz_v_ctx[i] = left_v[i];
    }
    dec->mb_has_coeff[mb_x] = (uint8_t)(has_coeff != 0);
}

/* ══════════════════════════════════════════════════════════════════════
 *  Inverse Transforms
 * ══════════════════════════════════════════════════════════════════════ */

/* Inverse Walsh-Hadamard Transform 4x4 */
static void vp8_iwht4x4(int16_t* in, int16_t* out) {
    int i;
    int16_t tmp[16];

    /* Rows */
    for (i = 0; i < 4; i++) {
        int a0 = in[i * 4 + 0] + in[i * 4 + 3];
        int a1 = in[i * 4 + 1] + in[i * 4 + 2];
        int a2 = in[i * 4 + 1] - in[i * 4 + 2];
        int a3 = in[i * 4 + 0] - in[i * 4 + 3];
        tmp[i * 4 + 0] = (int16_t)(a0 + a1);
        tmp[i * 4 + 1] = (int16_t)(a3 + a2);
        tmp[i * 4 + 2] = (int16_t)(a0 - a1);
        tmp[i * 4 + 3] = (int16_t)(a3 - a2);
    }

    /* Columns */
    for (i = 0; i < 4; i++) {
        int a0 = tmp[0 * 4 + i] + tmp[3 * 4 + i];
        int a1 = tmp[1 * 4 + i] + tmp[2 * 4 + i];
        int a2 = tmp[1 * 4 + i] - tmp[2 * 4 + i];
        int a3 = tmp[0 * 4 + i] - tmp[3 * 4 + i];
        out[0 * 4 + i] = (int16_t)((a0 + a1 + 3) >> 3);
        out[1 * 4 + i] = (int16_t)((a3 + a2 + 3) >> 3);
        out[2 * 4 + i] = (int16_t)((a0 - a1 + 3) >> 3);
        out[3 * 4 + i] = (int16_t)((a3 - a2 + 3) >> 3);
    }
}

/* Inverse DCT 4x4 - VP8 uses specific fixed-point cosine approximation */
static void vp8_idct4x4_add(const int16_t* in, uint8_t* dst, int stride) {
    int i;
    int tmp[16];

    /* Columns first */
    for (i = 0; i < 4; i++) {
        int a = in[0 * 4 + i] + in[2 * 4 + i];
        int b = in[0 * 4 + i] - in[2 * 4 + i];
        int t1 = (in[1 * 4 + i] * 35468 >> 16) - in[3 * 4 + i] - (in[3 * 4 + i] * 20091 >> 16);
        int t2 = in[1 * 4 + i] + (in[1 * 4 + i] * 20091 >> 16) + (in[3 * 4 + i] * 35468 >> 16);
        tmp[0 * 4 + i] = a + t2;
        tmp[1 * 4 + i] = b + t1;
        tmp[2 * 4 + i] = b - t1;
        tmp[3 * 4 + i] = a - t2;
    }

    /* Rows */
    for (i = 0; i < 4; i++) {
        int a = tmp[i * 4 + 0] + tmp[i * 4 + 2];
        int b = tmp[i * 4 + 0] - tmp[i * 4 + 2];
        int t1 = (tmp[i * 4 + 1] * 35468 >> 16) - tmp[i * 4 + 3] - (tmp[i * 4 + 3] * 20091 >> 16);
        int t2 = tmp[i * 4 + 1] + (tmp[i * 4 + 1] * 20091 >> 16) + (tmp[i * 4 + 3] * 35468 >> 16);
        int r0 = (a + t2 + 4) >> 3;
        int r1 = (b + t1 + 4) >> 3;
        int r2 = (b - t1 + 4) >> 3;
        int r3 = (a - t2 + 4) >> 3;
        dst[i * stride + 0] = vp8_clip8(dst[i * stride + 0] + r0);
        dst[i * stride + 1] = vp8_clip8(dst[i * stride + 1] + r1);
        dst[i * stride + 2] = vp8_clip8(dst[i * stride + 2] + r2);
        dst[i * stride + 3] = vp8_clip8(dst[i * stride + 3] + r3);
    }
}

static void vp8_idct4x4_dc_add(int dc, uint8_t* dst, int stride) {
    int i, j;
    int d = (dc + 4) >> 3;
    for (i = 0; i < 4; i++)
        for (j = 0; j < 4; j++)
            dst[i * stride + j] = vp8_clip8(dst[i * stride + j] + d);
}

/* ══════════════════════════════════════════════════════════════════════
 *  Intra Prediction
 * ══════════════════════════════════════════════════════════════════════ */

static void vp8_predict_16x16(uint8_t* dst, int stride,
                                const uint8_t* top, const uint8_t* left,
                                uint8_t tl, int mode, int has_top, int has_left) {
    int i, j;
    switch (mode) {
    case 0: /* DC_PRED */ {
        int sum = 0, count = 0;
        if (has_top) { for (i = 0; i < 16; i++) sum += top[i]; count += 16; }
        if (has_left) { for (i = 0; i < 16; i++) sum += left[i]; count += 16; }
        {
            int dc = count > 0 ? (sum + count / 2) / count : 128;
            for (i = 0; i < 16; i++)
                memset(dst + i * stride, dc, 16);
        }
        break;
    }
    case 1: /* V_PRED */
        for (i = 0; i < 16; i++)
            memcpy(dst + i * stride, top, 16);
        break;
    case 2: /* H_PRED */
        for (i = 0; i < 16; i++)
            memset(dst + i * stride, left[i], 16);
        break;
    case 3: /* TM_PRED */
        for (i = 0; i < 16; i++)
            for (j = 0; j < 16; j++)
                dst[i * stride + j] = vp8_clip8((int)top[j] + (int)left[i] - (int)tl);
        break;
    }
}

static void vp8_predict_4x4(uint8_t* dst, int stride,
                              const uint8_t* top, const uint8_t* left,
                              uint8_t tl, const uint8_t* tr, int mode) {
    int i, j;
    switch (mode) {
    case 0: /* DC_PRED */ {
        int sum = 0;
        for (i = 0; i < 4; i++) sum += top[i] + left[i];
        {
            int dc = (sum + 4) >> 3;
            for (i = 0; i < 4; i++)
                memset(dst + i * stride, dc, 4);
        }
        break;
    }
    case 1: /* TM_PRED */
        for (i = 0; i < 4; i++)
            for (j = 0; j < 4; j++)
                dst[i * stride + j] = vp8_clip8((int)top[j] + (int)left[i] - (int)tl);
        break;
    case 2: /* VE (vertical) */ {
        uint8_t v[4];
        v[0] = (tl + 2 * top[0] + top[1] + 2) >> 2;
        v[1] = (top[0] + 2 * top[1] + top[2] + 2) >> 2;
        v[2] = (top[1] + 2 * top[2] + top[3] + 2) >> 2;
        v[3] = (top[2] + 2 * top[3] + tr[0] + 2) >> 2;
        for (i = 0; i < 4; i++) memcpy(dst + i * stride, v, 4);
        break;
    }
    case 3: /* HE (horizontal) */ {
        uint8_t h[4];
        h[0] = (tl + 2 * left[0] + left[1] + 2) >> 2;
        h[1] = (left[0] + 2 * left[1] + left[2] + 2) >> 2;
        h[2] = (left[1] + 2 * left[2] + left[3] + 2) >> 2;
        h[3] = (left[2] + 2 * left[3] + left[3] + 2) >> 2;
        for (i = 0; i < 4; i++) memset(dst + i * stride, h[i], 4);
        break;
    }
    case 4: /* LD (left-down) */ {
        uint8_t p[8];
        p[0] = top[0]; p[1] = top[1]; p[2] = top[2]; p[3] = top[3];
        p[4] = tr[0]; p[5] = tr[1]; p[6] = tr[2]; p[7] = tr[3];
        for (i = 0; i < 4; i++)
            for (j = 0; j < 4; j++) {
                int idx = i + j;
                if (idx >= 7) dst[i * stride + j] = p[7];
                else dst[i * stride + j] = (p[idx] + 2 * p[idx + 1] + p[idx + 2] + 2) >> 2;
            }
        break;
    }
    case 5: /* RD (right-down) */ {
        uint8_t p[9]; /* tl=p[4], top=p[3..0], left=p[5..8] */
        p[0] = top[3]; p[1] = top[2]; p[2] = top[1]; p[3] = top[0]; p[4] = tl;
        p[5] = left[0]; p[6] = left[1]; p[7] = left[2]; p[8] = left[3];
        for (i = 0; i < 4; i++)
            for (j = 0; j < 4; j++)
                dst[i * stride + j] = (p[4 - j + i - 1] + 2 * p[4 - j + i] + p[4 - j + i + 1] + 2) >> 2;
        break;
    }
    case 6: /* VR (vertical-right) */ {
        uint8_t e[9];
        e[0] = left[3]; e[1] = left[2]; e[2] = left[1]; e[3] = left[0];
        e[4] = tl;
        e[5] = top[0];  e[6] = top[1];  e[7] = top[2];  e[8] = top[3];
        dst[3 * stride + 0] = vp8_avg3(e[1], e[2], e[3]);
        dst[2 * stride + 0] = vp8_avg3(e[2], e[3], e[4]);
        dst[3 * stride + 1] = dst[1 * stride + 0] = vp8_avg3(e[3], e[4], e[5]);
        dst[2 * stride + 1] = dst[0 * stride + 0] = vp8_avg2(e[4], e[5]);
        dst[3 * stride + 2] = dst[1 * stride + 1] = vp8_avg3(e[4], e[5], e[6]);
        dst[2 * stride + 2] = dst[0 * stride + 1] = vp8_avg2(e[5], e[6]);
        dst[3 * stride + 3] = dst[1 * stride + 2] = vp8_avg3(e[5], e[6], e[7]);
        dst[2 * stride + 3] = dst[0 * stride + 2] = vp8_avg2(e[6], e[7]);
        dst[1 * stride + 3] = vp8_avg3(e[6], e[7], e[8]);
        dst[0 * stride + 3] = vp8_avg2(e[7], e[8]);
        break;
    }
    case 7: /* VL (vertical-left) */ {
        uint8_t a[8];
        a[0] = top[0]; a[1] = top[1]; a[2] = top[2]; a[3] = top[3];
        a[4] = tr[0];  a[5] = tr[1];  a[6] = tr[2];  a[7] = tr[3];
        dst[0 * stride + 0] = vp8_avg2(a[0], a[1]);
        dst[1 * stride + 0] = vp8_avg3(a[0], a[1], a[2]);
        dst[2 * stride + 0] = dst[0 * stride + 1] = vp8_avg2(a[1], a[2]);
        dst[1 * stride + 1] = dst[3 * stride + 0] = vp8_avg3(a[1], a[2], a[3]);
        dst[2 * stride + 1] = dst[0 * stride + 2] = vp8_avg2(a[2], a[3]);
        dst[3 * stride + 1] = dst[1 * stride + 2] = vp8_avg3(a[2], a[3], a[4]);
        dst[2 * stride + 2] = dst[0 * stride + 3] = vp8_avg2(a[3], a[4]);
        dst[3 * stride + 2] = dst[1 * stride + 3] = vp8_avg3(a[3], a[4], a[5]);
        dst[2 * stride + 3] = vp8_avg3(a[4], a[5], a[6]);
        dst[3 * stride + 3] = vp8_avg3(a[5], a[6], a[7]);
        break;
    }
    case 8: /* HD (horizontal-down) */ {
        uint8_t e[9];
        e[0] = left[3]; e[1] = left[2]; e[2] = left[1]; e[3] = left[0];
        e[4] = tl;
        e[5] = top[0];  e[6] = top[1];  e[7] = top[2];  e[8] = top[3];
        dst[3 * stride + 0] = vp8_avg2(e[0], e[1]);
        dst[3 * stride + 1] = vp8_avg3(e[0], e[1], e[2]);
        dst[2 * stride + 0] = dst[3 * stride + 2] = vp8_avg2(e[1], e[2]);
        dst[2 * stride + 1] = dst[3 * stride + 3] = vp8_avg3(e[1], e[2], e[3]);
        dst[2 * stride + 2] = dst[1 * stride + 0] = vp8_avg2(e[2], e[3]);
        dst[2 * stride + 3] = dst[1 * stride + 1] = vp8_avg3(e[2], e[3], e[4]);
        dst[1 * stride + 2] = dst[0 * stride + 0] = vp8_avg2(e[3], e[4]);
        dst[1 * stride + 3] = dst[0 * stride + 1] = vp8_avg3(e[3], e[4], e[5]);
        dst[0 * stride + 2] = vp8_avg3(e[4], e[5], e[6]);
        dst[0 * stride + 3] = vp8_avg3(e[5], e[6], e[7]);
        break;
    }
    case 9: /* HU (horizontal-up) */ {
        dst[0 * stride + 0] = vp8_avg2(left[0], left[1]);
        dst[0 * stride + 1] = vp8_avg3(left[0], left[1], left[2]);
        dst[0 * stride + 2] = dst[1 * stride + 0] = vp8_avg2(left[1], left[2]);
        dst[0 * stride + 3] = dst[1 * stride + 1] = vp8_avg3(left[1], left[2], left[3]);
        dst[1 * stride + 2] = dst[2 * stride + 0] = vp8_avg2(left[2], left[3]);
        dst[1 * stride + 3] = dst[2 * stride + 1] = vp8_avg3(left[2], left[3], left[3]);
        dst[2 * stride + 2] = dst[2 * stride + 3] =
        dst[3 * stride + 0] = dst[3 * stride + 1] =
        dst[3 * stride + 2] = dst[3 * stride + 3] = left[3];
        break;
    }
    default:
        for (i = 0; i < 4; i++)
            memset(dst + i * stride, 128, 4);
        break;
    }
}

static void vp8_predict_8x8_uv(uint8_t* dst, int stride,
                                 const uint8_t* top, const uint8_t* left,
                                 uint8_t tl, int mode, int has_top, int has_left) {
    int i, j;
    switch (mode) {
    case 0: /* DC */ {
        int sum = 0, count = 0;
        if (has_top) { for (i = 0; i < 8; i++) sum += top[i]; count += 8; }
        if (has_left) { for (i = 0; i < 8; i++) sum += left[i]; count += 8; }
        {
            int dc = count > 0 ? (sum + count / 2) / count : 128;
            for (i = 0; i < 8; i++) memset(dst + i * stride, dc, 8);
        }
        break;
    }
    case 1: /* V */
        for (i = 0; i < 8; i++) memcpy(dst + i * stride, top, 8);
        break;
    case 2: /* H */
        for (i = 0; i < 8; i++) memset(dst + i * stride, left[i], 8);
        break;
    case 3: /* TM */
        for (i = 0; i < 8; i++)
            for (j = 0; j < 8; j++)
                dst[i * stride + j] = vp8_clip8((int)top[j] + (int)left[i] - (int)tl);
        break;
    }
}

/* ══════════════════════════════════════════════════════════════════════
 *  Macroblock Reconstruction
 * ══════════════════════════════════════════════════════════════════════ */

static void vp8_reconstruct_mb(VP8Decoder* dec, int mb_x, int mb_y) {
    int y_stride = dec->mb_w * 16;
    int uv_stride = dec->mb_w * 8;
    uint8_t* y_dst = dec->y_row + mb_x * 16;
    uint8_t* u_dst = dec->u_row + mb_x * 8;
    uint8_t* v_dst = dec->v_row + mb_x * 8;
    int mode = dec->mb_y_mode[mb_x];
    int uv_mode = dec->mb_uv_mode[mb_x];
    int has_top = (mb_y > 0);
    int has_left = (mb_x > 0);
    int i;

    /* Y prediction */
    if (!dec->mb_is_i4x4[mb_x]) {
        /* 16x16 prediction */
        vp8_predict_16x16(y_dst, y_stride,
                           dec->top_y + mb_x * 16, dec->left_y,
                           dec->top_left_y, mode, has_top, has_left);

        /* Apply WHT to get DC values */
        {
            int16_t dc16[16];
            vp8_iwht4x4(dec->mb_coeffs[mb_x][24], dc16);
            for (i = 0; i < 16; i++)
                dec->mb_coeffs[mb_x][i][0] = dc16[i];
        }

        /* Add residual with IDCT for each 4x4 block */
        for (i = 0; i < 16; i++) {
            int bx = (i & 3) * 4, by = (i >> 2) * 4;
            int16_t* c = dec->mb_coeffs[mb_x][i];
            int has_ac = 0, n;
            for (n = 1; n < 16; n++) { if (c[n]) { has_ac = 1; break; } }
            if (has_ac || c[0]) {
                if (has_ac) vp8_idct4x4_add(c, y_dst + by * y_stride + bx, y_stride);
                else vp8_idct4x4_dc_add(c[0], y_dst + by * y_stride + bx, y_stride);
            }
        }
    } else {
        /* 4x4 prediction + IDCT for each sub-block */
        for (i = 0; i < 16; i++) {
            int bx_pix = (i & 3) * 4, by_pix = (i >> 2) * 4;
            uint8_t* dst4 = y_dst + by_pix * y_stride + bx_pix;
            uint8_t top_buf[4], left_buf[4], tr_buf[4];
            uint8_t tl4;
            int bmode = dec->mb_imodes[mb_x][i];
            int j;

            /* Gather reference pixels */
            for (j = 0; j < 4; j++) {
                if (by_pix > 0) top_buf[j] = y_dst[(by_pix - 1) * y_stride + bx_pix + j];
                else if (has_top) top_buf[j] = dec->top_y[mb_x * 16 + bx_pix + j];
                else top_buf[j] = 127;
            }
            for (j = 0; j < 4; j++) {
                if (bx_pix > 0) left_buf[j] = y_dst[(by_pix + j) * y_stride + bx_pix - 1];
                else if (has_left) left_buf[j] = dec->left_y[by_pix + j];
                else left_buf[j] = 129;
            }
            if (bx_pix > 0 && by_pix > 0) tl4 = y_dst[(by_pix - 1) * y_stride + bx_pix - 1];
            else if (by_pix > 0 && has_left) tl4 = y_dst[(by_pix - 1) * y_stride + bx_pix - 1];
            else if (bx_pix > 0 && has_top) tl4 = dec->top_y[mb_x * 16 + bx_pix - 1];
            else if (has_top && has_left) tl4 = dec->top_left_y;
            else tl4 = 127;

            /* Top-right */
            for (j = 0; j < 4; j++) {
                if (bx_pix + 4 + j < 16) {
                    if (by_pix > 0) tr_buf[j] = y_dst[(by_pix - 1) * y_stride + bx_pix + 4 + j];
                    else if (has_top) tr_buf[j] = dec->top_y[mb_x * 16 + bx_pix + 4 + j];
                    else tr_buf[j] = 127;
                } else {
                    if (has_top) {
                        if (mb_x + 1 < dec->mb_w) {
                            tr_buf[j] = dec->top_y[(mb_x + 1) * 16 + j];
                        } else {
                            tr_buf[j] = dec->top_y[mb_x * 16 + 15];
                        }
                    } else {
                        tr_buf[j] = 127;
                    }
                }
            }

            vp8_predict_4x4(dst4, y_stride, top_buf, left_buf, tl4, tr_buf, bmode);

            /* Add residual */
            {
                int16_t* c = dec->mb_coeffs[mb_x][i];
                int has_ac = 0, n;
                for (n = 1; n < 16; n++) { if (c[n]) { has_ac = 1; break; } }
                if (has_ac || c[0]) {
                    if (has_ac) vp8_idct4x4_add(c, dst4, y_stride);
                    else vp8_idct4x4_dc_add(c[0], dst4, y_stride);
                }
            }
        }
    }

    /* UV prediction */
    vp8_predict_8x8_uv(u_dst, uv_stride, dec->top_u + mb_x * 8, dec->left_u,
                        dec->top_left_u, uv_mode, has_top, has_left);
    vp8_predict_8x8_uv(v_dst, uv_stride, dec->top_v + mb_x * 8, dec->left_v,
                        dec->top_left_v, uv_mode, has_top, has_left);

    /* Add UV residual */
    for (i = 0; i < 4; i++) {
        int bx = (i & 1) * 4, by = (i >> 1) * 4;
        {
            int16_t* cu = dec->mb_coeffs[mb_x][16 + i];
            int has_ac = 0, n;
            for (n = 1; n < 16; n++) { if (cu[n]) { has_ac = 1; break; } }
            if (has_ac || cu[0]) {
                if (has_ac) vp8_idct4x4_add(cu, u_dst + by * uv_stride + bx, uv_stride);
                else vp8_idct4x4_dc_add(cu[0], u_dst + by * uv_stride + bx, uv_stride);
            }
        }
        {
            int16_t* cv = dec->mb_coeffs[mb_x][20 + i];
            int has_ac = 0, n;
            for (n = 1; n < 16; n++) { if (cv[n]) { has_ac = 1; break; } }
            if (has_ac || cv[0]) {
                if (has_ac) vp8_idct4x4_add(cv, v_dst + by * uv_stride + bx, uv_stride);
                else vp8_idct4x4_dc_add(cv[0], v_dst + by * uv_stride + bx, uv_stride);
            }
        }
    }

    /* Update left reference pixels */
    for (i = 0; i < 16; i++) dec->left_y[i] = y_dst[i * y_stride + 15];
    for (i = 0; i < 8; i++) dec->left_u[i] = u_dst[i * uv_stride + 7];
    for (i = 0; i < 8; i++) dec->left_v[i] = v_dst[i * uv_stride + 7];
    /*
     * The top-left predictors for the next macroblock come from the previous
     * row's top reference arrays, not from the current macroblock's bottom-
     * right reconstructed pixels. Using the current MB tail here corrupts
     * intra prediction across macroblock boundaries, which shows up most
     * clearly as wrong chroma on multi-MB VP8 WebP images.
     */
    dec->top_left_y = dec->top_y[mb_x * 16 + 15];
    dec->top_left_u = dec->top_u[mb_x * 8 + 7];
    dec->top_left_v = dec->top_v[mb_x * 8 + 7];
}

/* ══════════════════════════════════════════════════════════════════════
 *  Loop Filters - RFC 6386 Section 15
 * ═════════════════════════════════════════════════════════════════════ */

enum VP8FilterKind {
    VP8_FILTER_SIMPLE,
    VP8_FILTER_SUBBLOCK,
    VP8_FILTER_MACROBLOCK
};

typedef struct {
    int level;
    int interior_limit;
    int hev_threshold;
    int mb_edge_limit;
    int sub_edge_limit;
} VP8FilterParams;

static int vp8_clip_s8(int v) {
    if (v < -128) return -128;
    if (v > 127) return 127;
    return v;
}

static uint8_t vp8_signed_to_pixel(int v) {
    return (uint8_t)(vp8_clip_s8(v) + 128);
}

static int vp8_abs_diff(int a, int b) {
    int d = a - b;
    return d < 0 ? -d : d;
}

static int vp8_filter_yes(int interior_limit, int edge_limit,
                          const uint8_t* p3, const uint8_t* p2,
                          const uint8_t* p1, const uint8_t* p0,
                          const uint8_t* q0, const uint8_t* q1,
                          const uint8_t* q2, const uint8_t* q3) {
    return 2 * vp8_abs_diff(*p0, *q0) +
               vp8_abs_diff(*p1, *q1) / 2 <= edge_limit &&
           vp8_abs_diff(*p3, *p2) <= interior_limit &&
           vp8_abs_diff(*p2, *p1) <= interior_limit &&
           vp8_abs_diff(*p1, *p0) <= interior_limit &&
           vp8_abs_diff(*q3, *q2) <= interior_limit &&
           vp8_abs_diff(*q2, *q1) <= interior_limit &&
           vp8_abs_diff(*q1, *q0) <= interior_limit;
}

static int vp8_high_edge_variance(int threshold,
                                   const uint8_t* p1, const uint8_t* p0,
                                   const uint8_t* q0, const uint8_t* q1) {
    return vp8_abs_diff(*p1, *p0) > threshold ||
           vp8_abs_diff(*q1, *q0) > threshold;
}

static int vp8_common_adjust(int use_outer_taps,
                             const uint8_t* p1p, uint8_t* p0p,
                             uint8_t* q0p, const uint8_t* q1p) {
    int p1 = (int)*p1p - 128;
    int p0 = (int)*p0p - 128;
    int q0 = (int)*q0p - 128;
    int q1 = (int)*q1p - 128;
    int outer = use_outer_taps ? vp8_clip_s8(p1 - q1) : 0;
    int a = vp8_clip_s8(outer + 3 * (q0 - p0));
    int p_adjust = vp8_clip_s8(a + 3) >> 3;

    a = vp8_clip_s8(a + 4) >> 3;
    *q0p = vp8_signed_to_pixel(q0 - a);
    *p0p = vp8_signed_to_pixel(p0 + p_adjust);
    return a;
}

static void vp8_simple_segment(int edge_limit,
                               uint8_t* p1, uint8_t* p0,
                               uint8_t* q0, uint8_t* q1) {
    if (2 * vp8_abs_diff(*p0, *q0) +
            vp8_abs_diff(*p1, *q1) / 2 <= edge_limit) {
        (void)vp8_common_adjust(1, p1, p0, q0, q1);
    }
}

static void vp8_subblock_segment(int hev_threshold, int interior_limit,
                                 int edge_limit,
                                 uint8_t* p3, uint8_t* p2,
                                 uint8_t* p1, uint8_t* p0,
                                 uint8_t* q0, uint8_t* q1,
                                 uint8_t* q2, uint8_t* q3) {
    int p1_value;
    int q1_value;
    int hev;
    int adjustment;

    if (!vp8_filter_yes(interior_limit, edge_limit,
                        p3, p2, p1, p0, q0, q1, q2, q3)) {
        return;
    }

    p1_value = (int)*p1 - 128;
    q1_value = (int)*q1 - 128;
    hev = vp8_high_edge_variance(hev_threshold, p1, p0, q0, q1);
    adjustment = (vp8_common_adjust(hev, p1, p0, q0, q1) + 1) >> 1;
    if (!hev) {
        *q1 = vp8_signed_to_pixel(q1_value - adjustment);
        *p1 = vp8_signed_to_pixel(p1_value + adjustment);
    }
}

static void vp8_macroblock_segment(int hev_threshold, int interior_limit,
                                   int edge_limit,
                                   uint8_t* p3, uint8_t* p2,
                                   uint8_t* p1, uint8_t* p0,
                                   uint8_t* q0, uint8_t* q1,
                                   uint8_t* q2, uint8_t* q3) {
    int p2_value;
    int p1_value;
    int p0_value;
    int q0_value;
    int q1_value;
    int q2_value;
    int w;
    int adjustment;

    if (!vp8_filter_yes(interior_limit, edge_limit,
                        p3, p2, p1, p0, q0, q1, q2, q3)) {
        return;
    }
    if (vp8_high_edge_variance(hev_threshold, p1, p0, q0, q1)) {
        (void)vp8_common_adjust(1, p1, p0, q0, q1);
        return;
    }

    p2_value = (int)*p2 - 128;
    p1_value = (int)*p1 - 128;
    p0_value = (int)*p0 - 128;
    q0_value = (int)*q0 - 128;
    q1_value = (int)*q1 - 128;
    q2_value = (int)*q2 - 128;
    w = vp8_clip_s8(vp8_clip_s8(p1_value - q1_value) +
                     3 * (q0_value - p0_value));

    adjustment = vp8_clip_s8((27 * w + 63) >> 7);
    *p0 = vp8_signed_to_pixel(p0_value + adjustment);
    *q0 = vp8_signed_to_pixel(q0_value - adjustment);
    adjustment = vp8_clip_s8((18 * w + 63) >> 7);
    *p1 = vp8_signed_to_pixel(p1_value + adjustment);
    *q1 = vp8_signed_to_pixel(q1_value - adjustment);
    adjustment = vp8_clip_s8((9 * w + 63) >> 7);
    *p2 = vp8_signed_to_pixel(p2_value + adjustment);
    *q2 = vp8_signed_to_pixel(q2_value - adjustment);
}

static void vp8_filter_segment(enum VP8FilterKind kind,
                               const VP8FilterParams* params,
                               uint8_t* p3, uint8_t* p2,
                               uint8_t* p1, uint8_t* p0,
                               uint8_t* q0, uint8_t* q1,
                               uint8_t* q2, uint8_t* q3) {
    if (kind == VP8_FILTER_SIMPLE) {
        vp8_simple_segment(params->mb_edge_limit, p1, p0, q0, q1);
    } else if (kind == VP8_FILTER_SUBBLOCK) {
        vp8_subblock_segment(params->hev_threshold, params->interior_limit,
                             params->sub_edge_limit,
                             p3, p2, p1, p0, q0, q1, q2, q3);
    } else {
        vp8_macroblock_segment(params->hev_threshold, params->interior_limit,
                               params->mb_edge_limit,
                               p3, p2, p1, p0, q0, q1, q2, q3);
    }
}

static void vp8_filter_vertical(uint8_t* q0, int stride, int length,
                                enum VP8FilterKind kind,
                                const VP8FilterParams* params) {
    int i;
    for (i = 0; i < length; ++i) {
        uint8_t* q = q0 + i * stride;
        vp8_filter_segment(kind, params,
                           q - 4, q - 3, q - 2, q - 1,
                           q, q + 1, q + 2, q + 3);
    }
}

static void vp8_filter_horizontal(uint8_t* q0, int stride, int length,
                                  enum VP8FilterKind kind,
                                  const VP8FilterParams* params) {
    int i;
    for (i = 0; i < length; ++i) {
        uint8_t* q = q0 + i;
        vp8_filter_segment(kind, params,
                           q - 4 * stride, q - 3 * stride,
                           q - 2 * stride, q - stride,
                           q, q + stride, q + 2 * stride, q + 3 * stride);
    }
}

static void vp8_filter_row_boundary(uint8_t* prev, uint8_t* current,
                                    int stride, int row_height,
                                    int x, int length,
                                    enum VP8FilterKind kind,
                                    const VP8FilterParams* params) {
    int i;
    for (i = 0; i < length; ++i) {
        uint8_t* p = prev + x + i;
        uint8_t* q = current + x + i;
        vp8_filter_segment(kind, params,
                           p + (row_height - 4) * stride,
                           p + (row_height - 3) * stride,
                           p + (row_height - 2) * stride,
                           p + (row_height - 1) * stride,
                           q, q + stride, q + 2 * stride, q + 3 * stride);
    }
}

static void vp8_filter_parameters(const VP8Decoder* dec, int mb_x,
                                  VP8FilterParams* params) {
    int level = dec->filter_level;
    int interior;

    if (dec->use_segment) {
        int segment_level = dec->seg_lf[dec->mb_segment[mb_x]];
        level = dec->segment_abs_delta ? segment_level : level + segment_level;
    }
    if (level < 0) level = 0;
    if (level > 63) level = 63;

    if (dec->filter_use_delta) {
        /* A key-frame macroblock uses CURRENT_FRAME (reference index zero). */
        level += dec->filter_ref_delta[0];
        if (dec->mb_is_i4x4[mb_x]) level += dec->filter_mode_delta[0];
    }
    if (level < 0) level = 0;
    if (level > 63) level = 63;

    interior = level;
    if (dec->filter_sharpness > 0) {
        interior >>= dec->filter_sharpness > 4 ? 2 : 1;
        if (interior > 9 - dec->filter_sharpness) {
            interior = 9 - dec->filter_sharpness;
        }
    }
    if (interior < 1) interior = 1;

    params->level = level;
    params->interior_limit = interior;
    params->hev_threshold = level >= 40 ? 2 : (level >= 15 ? 1 : 0);
    params->mb_edge_limit = 2 * (level + 2) + interior;
    params->sub_edge_limit = 2 * level + interior;
}

static void vp8_filter_mb_row(VP8Decoder* dec, int mb_y,
                              int y_stride, int uv_stride) {
    int mb_x;

    for (mb_x = 0; mb_x < dec->mb_w; ++mb_x) {
        uint8_t* y = dec->y_row + mb_x * 16;
        uint8_t* u = dec->u_row + mb_x * 8;
        uint8_t* v = dec->v_row + mb_x * 8;
        VP8FilterParams params;
        int filter_subblocks = dec->mb_has_coeff[mb_x] ||
                               dec->mb_is_i4x4[mb_x];

        vp8_filter_parameters(dec, mb_x, &params);
        if (params.level == 0) continue;

        if (dec->filter_simple) {
            if (mb_x > 0) {
                vp8_filter_vertical(y, y_stride, 16,
                                    VP8_FILTER_SIMPLE, &params);
            }
            if (filter_subblocks) {
                VP8FilterParams sub_params = params;
                sub_params.mb_edge_limit = params.sub_edge_limit;
                vp8_filter_vertical(y + 4, y_stride, 16,
                                    VP8_FILTER_SIMPLE, &sub_params);
                vp8_filter_vertical(y + 8, y_stride, 16,
                                    VP8_FILTER_SIMPLE, &sub_params);
                vp8_filter_vertical(y + 12, y_stride, 16,
                                    VP8_FILTER_SIMPLE, &sub_params);
            }
            if (mb_y > 0) {
                vp8_filter_row_boundary(dec->y_prev, dec->y_row,
                                        y_stride, 16, mb_x * 16, 16,
                                        VP8_FILTER_SIMPLE, &params);
            }
            if (filter_subblocks) {
                VP8FilterParams sub_params = params;
                sub_params.mb_edge_limit = params.sub_edge_limit;
                vp8_filter_horizontal(y + 4 * y_stride, y_stride, 16,
                                      VP8_FILTER_SIMPLE, &sub_params);
                vp8_filter_horizontal(y + 8 * y_stride, y_stride, 16,
                                      VP8_FILTER_SIMPLE, &sub_params);
                vp8_filter_horizontal(y + 12 * y_stride, y_stride, 16,
                                      VP8_FILTER_SIMPLE, &sub_params);
            }
            continue;
        }

        if (mb_x > 0) {
            vp8_filter_vertical(y, y_stride, 16,
                                VP8_FILTER_MACROBLOCK, &params);
            vp8_filter_vertical(u, uv_stride, 8,
                                VP8_FILTER_MACROBLOCK, &params);
            vp8_filter_vertical(v, uv_stride, 8,
                                VP8_FILTER_MACROBLOCK, &params);
        }
        if (filter_subblocks) {
            vp8_filter_vertical(y + 4, y_stride, 16,
                                VP8_FILTER_SUBBLOCK, &params);
            vp8_filter_vertical(y + 8, y_stride, 16,
                                VP8_FILTER_SUBBLOCK, &params);
            vp8_filter_vertical(y + 12, y_stride, 16,
                                VP8_FILTER_SUBBLOCK, &params);
            vp8_filter_vertical(u + 4, uv_stride, 8,
                                VP8_FILTER_SUBBLOCK, &params);
            vp8_filter_vertical(v + 4, uv_stride, 8,
                                VP8_FILTER_SUBBLOCK, &params);
        }
        if (mb_y > 0) {
            vp8_filter_row_boundary(dec->y_prev, dec->y_row,
                                    y_stride, 16, mb_x * 16, 16,
                                    VP8_FILTER_MACROBLOCK, &params);
            vp8_filter_row_boundary(dec->u_prev, dec->u_row,
                                    uv_stride, 8, mb_x * 8, 8,
                                    VP8_FILTER_MACROBLOCK, &params);
            vp8_filter_row_boundary(dec->v_prev, dec->v_row,
                                    uv_stride, 8, mb_x * 8, 8,
                                    VP8_FILTER_MACROBLOCK, &params);
        }
        if (filter_subblocks) {
            vp8_filter_horizontal(y + 4 * y_stride, y_stride, 16,
                                  VP8_FILTER_SUBBLOCK, &params);
            vp8_filter_horizontal(y + 8 * y_stride, y_stride, 16,
                                  VP8_FILTER_SUBBLOCK, &params);
            vp8_filter_horizontal(y + 12 * y_stride, y_stride, 16,
                                  VP8_FILTER_SUBBLOCK, &params);
            vp8_filter_horizontal(u + 4 * uv_stride, uv_stride, 8,
                                  VP8_FILTER_SUBBLOCK, &params);
            vp8_filter_horizontal(v + 4 * uv_stride, uv_stride, 8,
                                  VP8_FILTER_SUBBLOCK, &params);
        }
    }
}

/* ══════════════════════════════════════════════════════════════════════
 *  YUV to BGRA Conversion
 * ══════════════════════════════════════════════════════════════════════ */

static void vp8_yuv_to_bgra_row(const uint8_t* y_row, int y_stride,
                                  const uint8_t* u_row, const uint8_t* v_row,
                                  int uv_stride, uint8_t* bgra, int bgra_stride,
                                  int width, int num_rows) {
    int row, x;
    for (row = 0; row < num_rows; row++) {
        const uint8_t* y = y_row + row * y_stride;
        const uint8_t* u = u_row + (row >> 1) * uv_stride;
        const uint8_t* v = v_row + (row >> 1) * uv_stride;
        uint8_t* dst = bgra + row * bgra_stride;
        for (x = 0; x < width; x++) {
            /* VP8 key frames use studio-range BT.601 YUV. */
            int Y = (int)y[x] - 16;
            int U = (int)u[x >> 1] - 128;
            int V = (int)v[x >> 1] - 128;
            int r = (298 * Y + 409 * V + 128) >> 8;
            int g = (298 * Y - 100 * U - 208 * V + 128) >> 8;
            int b = (298 * Y + 516 * U + 128) >> 8;
            dst[x * 4 + 0] = vp8_clip8(b);
            dst[x * 4 + 1] = vp8_clip8(g);
            dst[x * 4 + 2] = vp8_clip8(r);
            dst[x * 4 + 3] = 0xFF;
        }
    }
}

/* ══════════════════════════════════════════════════════════════════════
 *  Top-Level VP8 Decoder
 * ══════════════════════════════════════════════════════════════════════ */

int vp8_decode(const uint8_t* data, size_t data_size,
               int exp_width, int exp_height,
               uint8_t* output, size_t output_size, int output_stride) {
    VP8Decoder dec;
    int mb_x, mb_y;
    size_t header_off;
    int y_stride, uv_stride;
    size_t output_row_bytes;
    size_t required_output_size;

    memset(&dec, 0, sizeof(dec));

    if (!data || !output || exp_width <= 0 || exp_height <= 0 ||
        output_stride <= 0) {
        return -1;
    }

    if (vp8_parse_frame_header(&dec, data, data_size, &header_off) != 0)
        return -1;

    if (dec.width != exp_width || dec.height != exp_height)
        return -1;

    output_row_bytes = (size_t)dec.width * 4u;
    if ((size_t)output_stride < output_row_bytes) return -1;
    if ((size_t)(dec.height - 1) >
        (SIZE_MAX - output_row_bytes) / (size_t)output_stride) {
        return -1;
    }
    required_output_size = (size_t)(dec.height - 1) *
                           (size_t)output_stride + output_row_bytes;
    if (output_size < required_output_size) return -1;

    y_stride = dec.mb_w * 16;
    uv_stride = dec.mb_w * 8;

    /* Allocate row buffers */
    dec.y_row = (uint8_t*)malloc((size_t)y_stride * 16);
    dec.u_row = (uint8_t*)malloc((size_t)uv_stride * 8);
    dec.v_row = (uint8_t*)malloc((size_t)uv_stride * 8);
    dec.y_prev = (uint8_t*)malloc((size_t)y_stride * 16);
    dec.u_prev = (uint8_t*)malloc((size_t)uv_stride * 8);
    dec.v_prev = (uint8_t*)malloc((size_t)uv_stride * 8);
    dec.top_y = (uint8_t*)malloc((size_t)y_stride + 8);
    dec.top_u = (uint8_t*)malloc((size_t)uv_stride + 8);
    dec.top_v = (uint8_t*)malloc((size_t)uv_stride + 8);
    if (!dec.y_row || !dec.u_row || !dec.v_row ||
        !dec.y_prev || !dec.u_prev || !dec.v_prev ||
        !dec.top_y || !dec.top_u || !dec.top_v) {
        goto fail;
    }
    memset(dec.top_y, 127, (size_t)y_stride + 8);
    memset(dec.top_u, 127, (size_t)uv_stride + 8);
    memset(dec.top_v, 127, (size_t)uv_stride + 8);
    memset(dec.left_y, 129, 16);
    memset(dec.left_u, 129, 8);
    memset(dec.left_v, 129, 8);
    dec.top_left_y = 127;
    dec.top_left_u = 127;
    dec.top_left_v = 127;
    memset(dec.top_nz_y, 0, sizeof(dec.top_nz_y));
    memset(dec.top_nz_u, 0, sizeof(dec.top_nz_u));
    memset(dec.top_nz_v, 0, sizeof(dec.top_nz_v));
    memset(dec.top_nz_dc, 0, sizeof(dec.top_nz_dc));

    /* Decode macroblock rows */
    for (mb_y = 0; mb_y < dec.mb_h; mb_y++) {
        int part_idx = mb_y % dec.num_parts;

        memset(dec.y_row, 0, (size_t)y_stride * 16);
        memset(dec.u_row, 0, (size_t)uv_stride * 8);
        memset(dec.v_row, 0, (size_t)uv_stride * 8);

        /* Reset left context */
        memset(dec.left_y, 129, 16);
        memset(dec.left_u, 129, 8);
        memset(dec.left_v, 129, 8);
        memset(dec.left_nz_y_ctx, 0, sizeof(dec.left_nz_y_ctx));
        memset(dec.left_nz_u_ctx, 0, sizeof(dec.left_nz_u_ctx));
        memset(dec.left_nz_v_ctx, 0, sizeof(dec.left_nz_v_ctx));
        dec.left_nz_dc_ctx = 0;
        memset(dec.left_bmode, 0, sizeof(dec.left_bmode));
        dec.top_left_y = dec.top_y[0];
        dec.top_left_u = dec.top_u[0];
        dec.top_left_v = dec.top_v[0];

        for (mb_x = 0; mb_x < dec.mb_w; mb_x++) {
            /* Decode modes from header partition */
            vp8_decode_mb_modes(&dec, mb_x, mb_y);
            if (dec.br_hdr.eof) goto fail;

            /* Decode coefficients from token partition */
            vp8_decode_mb_coeffs(&dec, &dec.br_parts[part_idx], mb_x);
            if (dec.br_parts[part_idx].eof) goto fail;

            /* Reconstruct MB */
            vp8_reconstruct_mb(&dec, mb_x, mb_y);
        }

        /* Preserve the unfiltered bottom pixels for prediction of the next
         * row. RFC 6386 requires intra prediction to precede loop filtering. */
        memcpy(dec.top_y, dec.y_row + 15 * y_stride, (size_t)y_stride);
        memcpy(dec.top_u, dec.u_row + 7 * uv_stride, (size_t)uv_stride);
        memcpy(dec.top_v, dec.v_row + 7 * uv_stride, (size_t)uv_stride);

        /* Filtering the top macroblock edge can change the previous row, so
         * retain two rows and emit a row only after its bottom edge is final. */
        vp8_filter_mb_row(&dec, mb_y, y_stride, uv_stride);
        if (mb_y > 0) {
            vp8_yuv_to_bgra_row(dec.y_prev, y_stride,
                                dec.u_prev, dec.v_prev, uv_stride,
                                output + (mb_y - 1) * 16 * output_stride,
                                output_stride, dec.width, 16);
        }

        {
            uint8_t* tmp;
            tmp = dec.y_prev; dec.y_prev = dec.y_row; dec.y_row = tmp;
            tmp = dec.u_prev; dec.u_prev = dec.u_row; dec.u_row = tmp;
            tmp = dec.v_prev; dec.v_prev = dec.v_row; dec.v_row = tmp;
        }
    }

    /* The final macroblock row has no lower neighbor and is now complete. */
    {
        int last_y = (dec.mb_h - 1) * 16;
        int num_rows = dec.height - last_y;
        vp8_yuv_to_bgra_row(dec.y_prev, y_stride,
                            dec.u_prev, dec.v_prev, uv_stride,
                            output + last_y * output_stride, output_stride,
                            dec.width, num_rows);
    }

    /* Cleanup */
    free(dec.y_row); free(dec.u_row); free(dec.v_row);
    free(dec.y_prev); free(dec.u_prev); free(dec.v_prev);
    free(dec.top_y); free(dec.top_u); free(dec.top_v);
    return 0;

fail:
    if (dec.y_row) free(dec.y_row);
    if (dec.u_row) free(dec.u_row);
    if (dec.v_row) free(dec.v_row);
    if (dec.y_prev) free(dec.y_prev);
    if (dec.u_prev) free(dec.u_prev);
    if (dec.v_prev) free(dec.v_prev);
    if (dec.top_y) free(dec.top_y);
    if (dec.top_u) free(dec.top_u);
    if (dec.top_v) free(dec.top_v);
    return -1;
}
