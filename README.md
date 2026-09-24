RinWebP is RinOS's in-tree WebP decoder subset used through RinImage and by
RinBrowser compatibility code. It is not a claim of complete WebP
specification compatibility and does not depend on V8's packaged libraries at
runtime.

## Supported API and profile

The public subset in `src/webp/decode.h` provides `WebPGetInfo`,
`WebPDecodeBGRAInto`, and last-error diagnostic accessors. It parses RIFF/WebP
containers and decodes VP8 lossy or VP8L lossless image payloads. VP8X is
handled for the container metadata needed to locate image data; supported
`ALPH` modes are accepted. For an animated container, the decoder returns the
first `ANMF` frame only. `WebPDecodeBGRAInto` writes BGRA bytes into the
caller-provided stride and buffer.

This API returns pixels only. It does not expose an animation timeline,
per-frame timing/disposal, ICC/EXIF/XMP metadata, or a complete
color-management interface. Do not infer support for a feature merely because
the container can be probed. RinImage is the preferred API when a consumer
needs centralized limits and canonical ARGB output.

## Ownership, errors, and concurrency

Encoded input and output storage are caller-owned. `WebPDecodeBGRAInto`
validates stride and output capacity; the caller must provide the entire
destination buffer. Some decode paths allocate temporary memory and report
allocation failure through a null result. `rin_webp_get_last_error_reason`,
`rin_webp_get_last_error_chunk`, and `rin_webp_get_last_backend_stage`
describe the most recent process-global decode state. Calls that need these
diagnostics must be serialized, with the accessors read immediately after the
corresponding call; concurrent calls race on that state.

The direct API has no configurable input, pixel, allocation, or CPU budget.
Use RinImage's default limits (64 MiB input, 4096 by 4096 dimensions,
16,777,216 pixels, and 64 MiB output) for untrusted image input, and lower
them where the caller's resource budget requires it. RinWebP does not offer a
cancellation/deadline API.

## ABI, build, and tests

`decode.h` is a C source interface; no separately versioned binary ABI is
published. RinOS integrates these sources through the parent image build.
This repository has no standalone build or test target; parent test evidence
must be tracked separately.

