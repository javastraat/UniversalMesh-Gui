import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_UPDATE_INTERVAL
from esphome.components import sensor

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


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_node_name(config[CONF_NODE_NAME]))
    cg.add(var.set_update_interval_ms(config[CONF_UPDATE_INTERVAL]))
    cg.add(var.set_heartbeat_interval_ms(config[CONF_HEARTBEAT_INTERVAL]))

    for entry in config[CONF_SENSORS]:
        sens = await cg.get_variable(entry[CONF_SENSOR_ID])
        cg.add(var.register_sensor(entry[CONF_KEY], sens))

    cg.add_library(
        "UniversalMesh",
        None,
        "https://github.com/johestephan/UniversalMesh",
    )
