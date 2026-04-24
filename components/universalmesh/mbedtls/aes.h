#pragma once
// ESP8266 stub — UniversalMesh encryption path is never called without a key set
#ifdef ESP8266

#include <stdint.h>
#include <stddef.h>

#define MBEDTLS_AES_ENCRYPT 1
#define MBEDTLS_AES_DECRYPT 0

typedef struct {
    int nr;
    uint32_t buf[68];
} mbedtls_aes_context;

#ifdef __cplusplus
extern "C" {
#endif

void mbedtls_aes_init(mbedtls_aes_context *ctx);
void mbedtls_aes_free(mbedtls_aes_context *ctx);
int  mbedtls_aes_setkey_enc(mbedtls_aes_context *ctx, const unsigned char *key, unsigned int keybits);
int  mbedtls_aes_crypt_cfb128(mbedtls_aes_context *ctx, int mode, size_t length,
                               size_t *iv_off, unsigned char *iv,
                               const unsigned char *input, unsigned char *output);

#ifdef __cplusplus
}
#endif

#endif  // ESP8266
