#ifdef ESP8266
// Stub implementations — encryption is never invoked without a mesh key set,
// so these exist only to satisfy the linker.
#include "mbedtls/aes.h"

extern "C" {

void mbedtls_aes_init(mbedtls_aes_context *ctx) {}
void mbedtls_aes_free(mbedtls_aes_context *ctx) {}

int mbedtls_aes_setkey_enc(mbedtls_aes_context *ctx, const unsigned char *key,
                            unsigned int keybits) {
    return 0;
}

int mbedtls_aes_crypt_cfb128(mbedtls_aes_context *ctx, int mode, size_t length,
                              size_t *iv_off, unsigned char *iv,
                              const unsigned char *input, unsigned char *output) {
    return 0;
}

}  // extern "C"
#endif  // ESP8266
