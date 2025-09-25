from . import hall_filament_switch_sensor

def load_config(config):
    return hall_filament_switch_sensor.HallFilamentSwitchSensor(config)