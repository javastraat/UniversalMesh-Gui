#pragma once
// ESP8266 stub — mbedtls encryption path in UniversalMesh is never invoked
// without a key set, so these no-op stubs satisfy the compiler/linker.
// On ESP32 (which has real mbedtls), #include_next forwards to the real header.
#ifdef ESP8266
#include <stdint.h>
#include <stddef.h>
#define MBEDTLS_AES_ENCRYPT 1
#define MBEDTLS_AES_DECRYPT 0
typedef struct { int nr; uint32_t buf[68]; } mbedtls_aes_context;
#ifdef __cplusplus
extern "C" {
#endif
inline void mbedtls_aes_init(mbedtls_aes_context *ctx) {}
inline void mbedtls_aes_free(mbedtls_aes_context *ctx) {}
inline int  mbedtls_aes_setkey_enc(mbedtls_aes_context *ctx,
                                    const unsigned char *key,
                                    unsigned int keybits) { return 0; }
inline int  mbedtls_aes_crypt_cfb128(mbedtls_aes_context *ctx, int mode,
                                      size_t length, size_t *iv_off,
                                      unsigned char *iv,
                                      const unsigned char *input,
                                      unsigned char *output) { return 0; }
#ifdef __cplusplus
}
#endif
#else
#include_next <mbedtls/aes.h>
#endif
