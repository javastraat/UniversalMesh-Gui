import os
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.components import sensor
from esphome.core import CORE

CODEOWNERS = ["@johestephan"]
DEPENDENCIES = ["wifi"]

universalmesh_ns = cg.esphome_ns.namespace("universalmesh")
UniversalMeshComponent = universalmesh_ns.class_(
    "UniversalMeshComponent", cg.Component
)

CONF_NODE_NAME = "node_name"
CONF_HEARTBEAT_INTERVAL = "heartbeat_interval"
CONF_SENSORS = "sensors"
CONF_SENSOR_ID = "sensor_id"
CONF_KEY = "key"

# ESP8266 stub — written to build src/ so it's on the default include path.
# #include_next forwards to the real mbedtls on platforms that have it (ESP32).
MBEDTLS_AES_STUB = """\
#pragma once
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
"""

SENSOR_ENTRY_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_SENSOR_ID): cv.use_id(sensor.Sensor),
        cv.Required(CONF_KEY): cv.string,
    }
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(UniversalMeshComponent),
        cv.Required(CONF_NODE_NAME): cv.string,
        cv.Optional(CONF_UPDATE_INTERVAL, default="60s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_HEARTBEAT_INTERVAL, default="120s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_SENSORS, default=[]): cv.ensure_list(SENSOR_ENTRY_SCHEMA),
    }
).extend(cv.COMPONENT_SCHEMA)

CONF_UPDATE_INTERVAL = "update_interval"


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_node_name(config[CONF_NODE_NAME]))
    cg.add(var.set_update_interval_ms(config[CONF_UPDATE_INTERVAL]))
    cg.add(var.set_heartbeat_interval_ms(config[CONF_HEARTBEAT_INTERVAL]))

    for entry in config[CONF_SENSORS]:
        sens = await cg.get_variable(entry[CONF_SENSOR_ID])
        cg.add(var.register_sensor(entry[CONF_KEY], sens))

    # Write mbedtls/aes.h stub into build src/ — always on the include path.
    stub_dir = os.path.join(CORE.build_path, "src", "mbedtls")
    os.makedirs(stub_dir, exist_ok=True)
    stub_path = os.path.join(stub_dir, "aes.h")
    with open(stub_path, "w") as f:
        f.write(MBEDTLS_AES_STUB)

    cg.add_library(
        "UniversalMesh",
        None,
        "https://github.com/johestephan/UniversalMesh",
    )
