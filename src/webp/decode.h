/*
 * RinOS local WebP decode API (minimal subset used by browser)
 */

#ifndef RIN_WEBP_DECODE_H_
#define RIN_WEBP_DECODE_H_

#include "./types.h"

#ifdef __cplusplus
extern "C" {
#endif

WEBP_EXTERN int WebPGetInfo(const uint8_t* data, size_t data_size,
                            int* width, int* height);

WEBP_EXTERN uint8_t* WebPDecodeBGRAInto(const uint8_t* data, size_t data_size,
                                        uint8_t* output_buffer,
                                        size_t output_buffer_size,
                                        int output_stride);

WEBP_EXTERN const char* rin_webp_get_last_error_reason(void);
WEBP_EXTERN const char* rin_webp_get_last_error_chunk(void);
WEBP_EXTERN const char* rin_webp_get_last_backend_stage(void);

#ifdef __cplusplus
}
#endif

#endif /* RIN_WEBP_DECODE_H_ */
