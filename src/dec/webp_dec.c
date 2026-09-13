/*
 * RinOS local WebP decoder (container parser)
 * - Parses RIFF/WebP container
 * - Supports VP8 / VP8L and VP8X container metadata
 * - Supports ALPH mode0/mode1 for VP8 payload
 * - Supports animated WebP first-frame decode (ANMF only)
 *
 * Public API signatures stay compatible with decode.h.
 */

#include "../webp/decode.h"

/* VP8L (lossless) and VP8 (lossy) decoders */
extern int vp8l_decode(const uint8_t* data, size_t data_size,
                       int width, int height,
                       uint8_t* output, size_t output_size, int output_stride);
extern int vp8l_decode_alpha_image(const uint8_t* data, size_t data_size,
                                   int width, int height,
                                   uint8_t* alpha, size_t alpha_size,
                                   int alpha_stride);
extern const char* vp8l_get_last_stage(void);
extern int vp8_decode(const uint8_t* data, size_t data_size,
                      int width, int height,
                      uint8_t* output, size_t output_size, int output_stride);

/* Freestanding: declare libc functions manually */
extern void* malloc(size_t);
extern void  free(void*);
extern void* memset(void*, int, size_t);
extern void* memcpy(void*, const void*, size_t);

typedef enum WebPDecodeError {
    WEBP_DEC_OK = 0,
    WEBP_DEC_ERR_INVALID_HEADER,
    WEBP_DEC_ERR_INVALID_DIMENSIONS,
    WEBP_DEC_ERR_CHUNK_BOUNDS,
    WEBP_DEC_ERR_MISSING_IMAGE_PAYLOAD,
    WEBP_DEC_ERR_UNSUPPORTED_FEATURE,
    WEBP_DEC_ERR_ALPH_MODE,
    WEBP_DEC_ERR_ALPH_PAYLOAD,
    WEBP_DEC_ERR_FRAME_BOUNDS,
    WEBP_DEC_ERR_DECODE_BACKEND,
    WEBP_DEC_ERR_OOM
} WebPDecodeError;

static WebPDecodeError g_last_error = WEBP_DEC_OK;
static uint32_t g_last_chunk_tag = 0;
static const char* g_last_backend_stage = "none";

static uint32_t read_u32_le(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int read_u24_le(const uint8_t* p) {
    return (int)p[0] | ((int)p[1] << 8) | ((int)p[2] << 16);
}

static uint32_t tag_u32(const uint8_t* tag4) {
    return read_u32_le(tag4);
}

static void set_error(WebPDecodeError err, const uint8_t* chunk_tag) {
    g_last_error = err;
    g_last_chunk_tag = chunk_tag ? tag_u32(chunk_tag) : 0;
    if (err == WEBP_DEC_OK) {
        g_last_backend_stage = "ok";
    }
}

const char* rin_webp_get_last_error_reason(void) {
    switch (g_last_error) {
        case WEBP_DEC_OK: return "ok";
        case WEBP_DEC_ERR_INVALID_HEADER: return "invalid-header";
        case WEBP_DEC_ERR_INVALID_DIMENSIONS: return "invalid-dimensions";
        case WEBP_DEC_ERR_CHUNK_BOUNDS: return "chunk-bounds";
        case WEBP_DEC_ERR_MISSING_IMAGE_PAYLOAD: return "missing-image-payload";
        case WEBP_DEC_ERR_UNSUPPORTED_FEATURE: return "unsupported-feature";
        case WEBP_DEC_ERR_ALPH_MODE: return "alph-unsupported-mode";
        case WEBP_DEC_ERR_ALPH_PAYLOAD: return "alph-invalid-payload";
        case WEBP_DEC_ERR_FRAME_BOUNDS: return "frame-bounds";
        case WEBP_DEC_ERR_DECODE_BACKEND: return "decode-backend-failed";
        case WEBP_DEC_ERR_OOM: return "oom";
        default: return "unknown";
    }
}

const char* rin_webp_get_last_error_chunk(void) {
    static char tag[5];
    tag[0] = (char)(g_last_chunk_tag & 0xffu);
    tag[1] = (char)((g_last_chunk_tag >> 8) & 0xffu);
    tag[2] = (char)((g_last_chunk_tag >> 16) & 0xffu);
    tag[3] = (char)((g_last_chunk_tag >> 24) & 0xffu);
    tag[4] = '\0';
    if (g_last_chunk_tag == 0) return "----";
    return tag;
}

const char* rin_webp_get_last_backend_stage(void) {
    return g_last_backend_stage ? g_last_backend_stage : "none";
}

static int compute_chunk_step(size_t pos, size_t size, uint32_t chunk_size,
                              size_t* payload_off, size_t* advance) {
    size_t padded;
    if (!payload_off || !advance) return 0;
    if (pos > size || size - pos < 8) return 0;
    *payload_off = pos + 8;
    if (*payload_off > size || (size - *payload_off) < (size_t)chunk_size) return 0;
    padded = (size_t)chunk_size + (size_t)(chunk_size & 1u);
    if (padded < (size_t)chunk_size) return 0;
    *advance = 8 + padded;
    if (*advance < 8) return 0;
    if (*advance > size - pos) return 0;
    return 1;
}

static int parse_vp8_dims(const uint8_t* payload, size_t payload_size, int* w, int* h) {
    if (payload_size < 10) return 0;
    if (!(payload[3] == 0x9d && payload[4] == 0x01 && payload[5] == 0x2a)) return 0;
    *w = ((int)payload[6] | ((int)payload[7] << 8)) & 0x3fff;
    *h = ((int)payload[8] | ((int)payload[9] << 8)) & 0x3fff;
    return (*w > 0 && *h > 0);
}

static int parse_vp8l_dims(const uint8_t* payload, size_t payload_size, int* w, int* h) {
    if (payload_size < 5) return 0;
    if (payload[0] != 0x2f) return 0;
    {
        uint32_t bits = read_u32_le(payload + 1);
        *w = 1 + (int)(bits & 0x3fffu);
        *h = 1 + (int)((bits >> 14) & 0x3fffu);
    }
    return (*w > 0 && *h > 0);
}

static int tag_eq(const uint8_t* tag, const char a, const char b,
                  const char c, const char d) {
    return tag[0] == (uint8_t)a && tag[1] == (uint8_t)b &&
           tag[2] == (uint8_t)c && tag[3] == (uint8_t)d;
}

typedef struct WebPContainerInfo {
    int canvas_w;
    int canvas_h;
    int has_vp8x;
    uint8_t vp8x_flags;

    const uint8_t* vp8_payload;
    uint32_t vp8_size;
    const uint8_t* vp8l_payload;
    uint32_t vp8l_size;
    const uint8_t* alph_payload;
    uint32_t alph_size;

    int has_anim;
    const uint8_t* first_anmf_payload;
    uint32_t first_anmf_size;
} WebPContainerInfo;

static int parse_container_info(const uint8_t* data, size_t data_size,
                                WebPContainerInfo* info) {
    size_t pos;
    if (!data || !info || data_size < 20) {
        set_error(WEBP_DEC_ERR_INVALID_HEADER, NULL);
        return 0;
    }

    if (!(data[0] == 'R' && data[1] == 'I' && data[2] == 'F' && data[3] == 'F')) {
        set_error(WEBP_DEC_ERR_INVALID_HEADER, NULL);
        return 0;
    }
    if (!(data[8] == 'W' && data[9] == 'E' && data[10] == 'B' && data[11] == 'P')) {
        set_error(WEBP_DEC_ERR_INVALID_HEADER, NULL);
        return 0;
    }

    memset(info, 0, sizeof(*info));
    pos = 12;

    while (pos + 8 <= data_size) {
        const uint8_t* chunk = data + pos;
        const uint32_t chunk_size = read_u32_le(chunk + 4);
        size_t payload_off = 0;
        size_t advance = 0;
        if (!compute_chunk_step(pos, data_size, chunk_size, &payload_off, &advance)) {
            set_error(WEBP_DEC_ERR_CHUNK_BOUNDS, chunk);
            return 0;
        }

        if (tag_eq(chunk, 'V', 'P', '8', 'X')) {
            if (chunk_size < 10) {
                set_error(WEBP_DEC_ERR_CHUNK_BOUNDS, chunk);
                return 0;
            }
            info->has_vp8x = 1;
            info->vp8x_flags = data[payload_off + 0];
            info->canvas_w = 1 + read_u24_le(data + payload_off + 4);
            info->canvas_h = 1 + read_u24_le(data + payload_off + 7);
            if (info->canvas_w <= 0 || info->canvas_h <= 0) {
                set_error(WEBP_DEC_ERR_INVALID_DIMENSIONS, chunk);
                return 0;
            }
        } else if (tag_eq(chunk, 'V', 'P', '8', 'L')) {
            if (!info->vp8l_payload) {
                info->vp8l_payload = data + payload_off;
                info->vp8l_size = chunk_size;
            }
        } else if (tag_eq(chunk, 'V', 'P', '8', ' ')) {
            if (!info->vp8_payload) {
                info->vp8_payload = data + payload_off;
                info->vp8_size = chunk_size;
            }
        } else if (tag_eq(chunk, 'A', 'L', 'P', 'H')) {
            if (!info->alph_payload) {
                info->alph_payload = data + payload_off;
                info->alph_size = chunk_size;
            }
        } else if (tag_eq(chunk, 'A', 'N', 'I', 'M')) {
            info->has_anim = 1;
        } else if (tag_eq(chunk, 'A', 'N', 'M', 'F')) {
            info->has_anim = 1;
            if (!info->first_anmf_payload) {
                info->first_anmf_payload = data + payload_off;
                info->first_anmf_size = chunk_size;
            }
        }

        pos += advance;
    }

    if (!info->vp8_payload && !info->vp8l_payload && !info->first_anmf_payload) {
        set_error(WEBP_DEC_ERR_MISSING_IMAGE_PAYLOAD, NULL);
        return 0;
    }

    set_error(WEBP_DEC_OK, NULL);
    return 1;
}

static int parse_webp_dims(const uint8_t* data, size_t size, int* out_w, int* out_h) {
    WebPContainerInfo info;
    int w = 0, h = 0;

    if (!out_w || !out_h) return 0;
    if (!parse_container_info(data, size, &info)) return 0;

    if (info.has_vp8x && info.canvas_w > 0 && info.canvas_h > 0) {
        *out_w = info.canvas_w;
        *out_h = info.canvas_h;
        return 1;
    }

    if (info.vp8l_payload && parse_vp8l_dims(info.vp8l_payload, info.vp8l_size, &w, &h)) {
        *out_w = w;
        *out_h = h;
        return 1;
    }
    if (info.vp8_payload && parse_vp8_dims(info.vp8_payload, info.vp8_size, &w, &h)) {
        *out_w = w;
        *out_h = h;
        return 1;
    }

    if (info.first_anmf_payload && info.first_anmf_size >= 16) {
        int fw = 1 + read_u24_le(info.first_anmf_payload + 6);
        int fh = 1 + read_u24_le(info.first_anmf_payload + 9);
        if (fw > 0 && fh > 0) {
            *out_w = fw;
            *out_h = fh;
            return 1;
        }
    }

    set_error(WEBP_DEC_ERR_INVALID_DIMENSIONS, NULL);
    return 0;
}

static void fill_alpha_opaque(uint8_t* output, int width, int height, int stride) {
    int y;
    for (y = 0; y < height; ++y) {
        uint8_t* row = output + (size_t)y * (size_t)stride;
        int x;
        for (x = 0; x < width; ++x) {
            row[(size_t)x * 4u + 3u] = 0xff;
        }
    }
}

static uint8_t clip_alpha_byte(int v) {
    return (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
}

static int decode_alph_plane(const uint8_t* alph_payload, size_t alph_size,
                             int width, int height,
                             uint8_t* alpha_plane, size_t alpha_plane_size,
                             int alpha_stride,
                             uint8_t* out_filter) {
    size_t needed_alpha;
    uint8_t header;
    uint8_t method;
    uint8_t filter;
    const uint8_t* alpha_data;

    if (!alph_payload || alph_size < 1 || !alpha_plane ||
        width <= 0 || height <= 0 || alpha_stride < width) {
        return 0;
    }

    needed_alpha = (size_t)alpha_stride * (size_t)height;
    if (needed_alpha == 0 || needed_alpha > alpha_plane_size) {
        set_error(WEBP_DEC_ERR_ALPH_PAYLOAD, (const uint8_t*)"ALPH");
        g_last_backend_stage = "alph-plane-size";
        return 0;
    }

    header = alph_payload[0];
    method = (uint8_t)(header & 0x03u);
    filter = (uint8_t)((header >> 2) & 0x03u);
    if (out_filter) *out_filter = filter;

    if (method == 0) {
        size_t raw_size = (size_t)width * (size_t)height;
        int y;
        g_last_backend_stage = "alph-raw";
        if (alph_size < 1u + raw_size) {
            set_error(WEBP_DEC_ERR_ALPH_PAYLOAD, (const uint8_t*)"ALPH");
            return 0;
        }
        alpha_data = alph_payload + 1;
        for (y = 0; y < height; ++y) {
            memcpy(alpha_plane + (size_t)y * (size_t)alpha_stride,
                   alpha_data + (size_t)y * (size_t)width,
                   (size_t)width);
        }
        return 1;
    }

    if (method == 1) {
        g_last_backend_stage = "alph-lossless";
        if (vp8l_decode_alpha_image(alph_payload + 1, alph_size - 1,
                                    width, height,
                                    alpha_plane, alpha_plane_size,
                                    alpha_stride) != 0) {
            set_error(WEBP_DEC_ERR_DECODE_BACKEND, (const uint8_t*)"ALPH");
            g_last_backend_stage = vp8l_get_last_stage();
            return 0;
        }
        return 1;
    }

    set_error(WEBP_DEC_ERR_ALPH_MODE, (const uint8_t*)"ALPH");
    g_last_backend_stage = "alph-mode";
    return 0;
}

static int unfilter_alph_plane(uint8_t* alpha_plane, size_t alpha_plane_size,
                               int width, int height, int alpha_stride,
                               uint8_t filter) {
    int x, y;
    if (!alpha_plane || width <= 0 || height <= 0 || alpha_stride < width) return 0;
    if ((size_t)alpha_stride * (size_t)height > alpha_plane_size) return 0;
    g_last_backend_stage = "alph-filter";
    if (filter == 0) return 1;
    if (filter > 3) {
        set_error(WEBP_DEC_ERR_UNSUPPORTED_FEATURE, (const uint8_t*)"ALPH");
        return 0;
    }

    for (y = 0; y < height; ++y) {
        uint8_t* row = alpha_plane + (size_t)y * (size_t)alpha_stride;
        const uint8_t* prev = (y > 0) ? (alpha_plane + (size_t)(y - 1) * (size_t)alpha_stride) : NULL;
        for (x = 0; x < width; ++x) {
            int predictor = 0;
            switch (filter) {
                case 1:
                    predictor = (x > 0) ? row[x - 1] : (prev ? prev[0] : 0);
                    break;
                case 2:
                    predictor = prev ? prev[x] : (x > 0 ? row[x - 1] : 0);
                    break;
                case 3:
                    if (x == 0 && !prev) {
                        predictor = 0;
                    } else if (!prev) {
                        predictor = row[x - 1];
                    } else if (x == 0) {
                        predictor = prev[0];
                    } else {
                        predictor = clip_alpha_byte((int)row[x - 1] + (int)prev[x] - (int)prev[x - 1]);
                    }
                    break;
                default:
                    predictor = 0;
                    break;
            }
            row[x] = (uint8_t)((row[x] + predictor) & 0xff);
        }
    }
    return 1;
}

static int apply_alph_payload(const uint8_t* alph_payload, size_t alph_size,
                              int width, int height,
                              uint8_t* output, size_t output_size, int stride) {
    size_t alpha_stride;
    size_t alpha_size;
    uint8_t filter = 0;
    uint8_t* alpha_plane;
    int y;

    if (!alph_payload || alph_size < 1 || !output || width <= 0 || height <= 0 || stride <= 0) {
        return 0;
    }
    if ((size_t)stride * (size_t)height > output_size) {
        set_error(WEBP_DEC_ERR_ALPH_PAYLOAD, (const uint8_t*)"ALPH");
        g_last_backend_stage = "alph-output-size";
        return 0;
    }

    alpha_stride = (size_t)width;
    alpha_size = alpha_stride * (size_t)height;
    alpha_plane = (uint8_t*)malloc(alpha_size);
    if (!alpha_plane) {
        set_error(WEBP_DEC_ERR_OOM, (const uint8_t*)"ALPH");
        g_last_backend_stage = "alph-oom";
        return 0;
    }

    if (!decode_alph_plane(alph_payload, alph_size, width, height,
                           alpha_plane, alpha_size, (int)alpha_stride,
                           &filter)) {
        free(alpha_plane);
        return 0;
    }
    if (!unfilter_alph_plane(alpha_plane, alpha_size, width, height,
                             (int)alpha_stride, filter)) {
        free(alpha_plane);
        return 0;
    }

    g_last_backend_stage = "alph-apply";
    for (y = 0; y < height; ++y) {
        uint8_t* dst = output + (size_t)y * (size_t)stride;
        const uint8_t* src = alpha_plane + (size_t)y * alpha_stride;
        int x;
        for (x = 0; x < width; ++x) {
            dst[(size_t)x * 4u + 3u] = src[x];
        }
    }

    free(alpha_plane);
    return 1;
}

static int decode_static_payload(const WebPContainerInfo* info,
                                 int width, int height,
                                 uint8_t* output_buffer,
                                 size_t output_buffer_size,
                                 int output_stride) {
    if (!info || !output_buffer) return 0;

    if (info->vp8l_payload) {
        if (vp8l_decode(info->vp8l_payload, info->vp8l_size,
                        width, height, output_buffer,
                        output_buffer_size, output_stride) == 0) {
            g_last_backend_stage = "vp8l";
            set_error(WEBP_DEC_OK, (const uint8_t*)"VP8L");
            return 1;
        }
        g_last_backend_stage = vp8l_get_last_stage();
        set_error(WEBP_DEC_ERR_DECODE_BACKEND, (const uint8_t*)"VP8L");
        return 0;
    }

    if (info->vp8_payload) {
        if (vp8_decode(info->vp8_payload, info->vp8_size,
                       width, height, output_buffer,
                       output_buffer_size, output_stride) != 0) {
            g_last_backend_stage = "vp8";
            set_error(WEBP_DEC_ERR_DECODE_BACKEND, (const uint8_t*)"VP8 ");
            return 0;
        }
        if (info->alph_payload) {
            if (!apply_alph_payload(info->alph_payload, info->alph_size,
                                    width, height,
                                    output_buffer, output_buffer_size,
                                    output_stride)) {
                return 0;
            }
            set_error(WEBP_DEC_OK, (const uint8_t*)"ALPH");
            return 1;
        }
        g_last_backend_stage = "vp8";
        fill_alpha_opaque(output_buffer, width, height, output_stride);
        set_error(WEBP_DEC_OK, (const uint8_t*)"VP8 ");
        return 1;
    }

    set_error(WEBP_DEC_ERR_MISSING_IMAGE_PAYLOAD, NULL);
    return 0;
}

static int decode_first_anmf(const uint8_t* anmf_payload, size_t anmf_size,
                             int canvas_w, int canvas_h,
                             uint8_t* output_buffer,
                             size_t output_buffer_size,
                             int output_stride) {
    int frame_x, frame_y, frame_w, frame_h;
    const uint8_t* vp8 = NULL;
    uint32_t vp8_size = 0;
    const uint8_t* vp8l = NULL;
    uint32_t vp8l_size = 0;
    const uint8_t* alph = NULL;
    uint32_t alph_size = 0;
    size_t pos;
    uint8_t* frame = NULL;
    int ok = 0;

    if (!anmf_payload || anmf_size < 16) {
        set_error(WEBP_DEC_ERR_CHUNK_BOUNDS, (const uint8_t*)"ANMF");
        return 0;
    }

    frame_x = 2 * read_u24_le(anmf_payload + 0);
    frame_y = 2 * read_u24_le(anmf_payload + 3);
    frame_w = 1 + read_u24_le(anmf_payload + 6);
    frame_h = 1 + read_u24_le(anmf_payload + 9);

    if (frame_w <= 0 || frame_h <= 0 || frame_x < 0 || frame_y < 0) {
        set_error(WEBP_DEC_ERR_FRAME_BOUNDS, (const uint8_t*)"ANMF");
        return 0;
    }
    if (frame_x + frame_w > canvas_w || frame_y + frame_h > canvas_h) {
        set_error(WEBP_DEC_ERR_FRAME_BOUNDS, (const uint8_t*)"ANMF");
        return 0;
    }

    pos = 16;
    while (pos + 8 <= anmf_size) {
        const uint8_t* chunk = anmf_payload + pos;
        uint32_t chunk_size = read_u32_le(chunk + 4);
        size_t payload_off = 0;
        size_t advance = 0;
        if (!compute_chunk_step(pos, anmf_size, chunk_size, &payload_off, &advance)) {
            set_error(WEBP_DEC_ERR_CHUNK_BOUNDS, chunk);
            return 0;
        }

        if (tag_eq(chunk, 'V', 'P', '8', 'L') && !vp8l) {
            vp8l = anmf_payload + payload_off;
            vp8l_size = chunk_size;
        } else if (tag_eq(chunk, 'V', 'P', '8', ' ') && !vp8) {
            vp8 = anmf_payload + payload_off;
            vp8_size = chunk_size;
        } else if (tag_eq(chunk, 'A', 'L', 'P', 'H') && !alph) {
            alph = anmf_payload + payload_off;
            alph_size = chunk_size;
        }

        pos += advance;
    }

    if (!vp8 && !vp8l) {
        set_error(WEBP_DEC_ERR_MISSING_IMAGE_PAYLOAD, (const uint8_t*)"ANMF");
        return 0;
    }

    {
        size_t frame_stride = (size_t)frame_w * 4u;
        size_t frame_size = frame_stride * (size_t)frame_h;
        if (frame_size == 0 || frame_size > (size_t)-1 / 2u) {
            set_error(WEBP_DEC_ERR_OOM, (const uint8_t*)"ANMF");
            return 0;
        }
        frame = (uint8_t*)malloc(frame_size);
        if (!frame) {
            set_error(WEBP_DEC_ERR_OOM, (const uint8_t*)"ANMF");
            return 0;
        }
        memset(frame, 0, frame_size);

        if (vp8l) {
            if (vp8l_decode(vp8l, vp8l_size, frame_w, frame_h,
                            frame, frame_size, (int)frame_stride) != 0) {
                g_last_backend_stage = vp8l_get_last_stage();
                set_error(WEBP_DEC_ERR_DECODE_BACKEND, (const uint8_t*)"VP8L");
                goto done;
            }
        } else {
            if (vp8_decode(vp8, vp8_size, frame_w, frame_h,
                           frame, frame_size, (int)frame_stride) != 0) {
                g_last_backend_stage = "vp8";
                set_error(WEBP_DEC_ERR_DECODE_BACKEND, (const uint8_t*)"VP8 ");
                goto done;
            }
            if (alph) {
                if (!apply_alph_payload(alph, alph_size, frame_w, frame_h,
                                        frame, frame_size, (int)frame_stride)) {
                    goto done;
                }
            } else {
                fill_alpha_opaque(frame, frame_w, frame_h, (int)frame_stride);
            }
        }

        memset(output_buffer, 0, output_buffer_size);
        {
            int y;
            for (y = 0; y < frame_h; ++y) {
                uint8_t* dst = output_buffer + (size_t)(frame_y + y) * (size_t)output_stride +
                               (size_t)frame_x * 4u;
                const uint8_t* src = frame + (size_t)y * frame_stride;
                memcpy(dst, src, (size_t)frame_w * 4u);
            }
        }
        set_error(WEBP_DEC_OK, (const uint8_t*)"ANMF");
        ok = 1;
    }

done:
    if (frame) free(frame);
    return ok;
}

int WebPGetInfo(const uint8_t* data, size_t data_size, int* width, int* height) {
    int w = 0, h = 0;
    g_last_backend_stage = "container";
    if (!parse_webp_dims(data, data_size, &w, &h)) return 0;
    if (width) *width = w;
    if (height) *height = h;
    return 1;
}

uint8_t* WebPDecodeBGRAInto(const uint8_t* data, size_t data_size,
                            uint8_t* output_buffer, size_t output_buffer_size,
                            int output_stride) {
    WebPContainerInfo info;
    int w = 0, h = 0;
    int min_stride;
    size_t needed_bytes;

    g_last_backend_stage = "container";

    if (!output_buffer || output_stride <= 0) {
        set_error(WEBP_DEC_ERR_INVALID_DIMENSIONS, NULL);
        return 0;
    }

    if (!parse_webp_dims(data, data_size, &w, &h)) return 0;
    if (!parse_container_info(data, data_size, &info)) return 0;

    if (w <= 0 || h <= 0 || w > 0x1fffffff) {
        set_error(WEBP_DEC_ERR_INVALID_DIMENSIONS, NULL);
        return 0;
    }

    min_stride = w * 4;
    if (output_stride < min_stride) {
        set_error(WEBP_DEC_ERR_INVALID_DIMENSIONS, NULL);
        return 0;
    }
    if ((size_t)output_stride > (size_t)-1 / (size_t)h) {
        set_error(WEBP_DEC_ERR_INVALID_DIMENSIONS, NULL);
        return 0;
    }

    needed_bytes = (size_t)output_stride * (size_t)h;
    if (needed_bytes > output_buffer_size) {
        set_error(WEBP_DEC_ERR_INVALID_DIMENSIONS, NULL);
        return 0;
    }

    if (info.has_anim && info.first_anmf_payload) {
        if (decode_first_anmf(info.first_anmf_payload, info.first_anmf_size,
                              w, h, output_buffer, needed_bytes, output_stride)) {
            return output_buffer;
        }
        return 0;
    }

    if (decode_static_payload(&info, w, h,
                              output_buffer, needed_bytes,
                              output_stride)) {
        return output_buffer;
    }

    return 0;
}
