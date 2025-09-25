from . import hall_filament_cut_sensor

def load_config(config):
    return hall_filament_cut_sensor.HallFilamentCutSensor(config)