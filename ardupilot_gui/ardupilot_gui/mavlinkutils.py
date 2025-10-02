"""
Helper functions from pymavlink
"""


def mode_string(vehtype, mode):
    '''Return the text string name of a flight mode'''
    mode_map = mode_mapping_bynumber(vehtype)
    if mode_map and mode in mode_map:
        return mode_map[mode]
    return "Mode(%u)" % mode


def mode_int(vehtype, modestr):
    '''Return the mode number for a given mode string, or None if not found'''
    mode_map = mode_mapping_bynumber(vehtype)
    if mode_map:
        for k, v in mode_map.items():
            if v == modestr:
                return k
    return None


def mode_mapping_bynumber(mav_type):
    '''return dictionary mapping mode numbers to name, or None if unknown'''
    return AP_MAV_TYPE_MODE_MAP[mav_type] if mav_type in AP_MAV_TYPE_MODE_MAP else None


FAILSAFE_ID = {
    21: "Radio ",
    22: "Battery ",
    23: "GCS ",
    24: "EKF ",
}

VEHICLE_TYPES = {
    0: "Generic",
    1: "Rover",
    2: "Copter",
    3: "Plane",
    4: "Antenna Tracker",
    5: "Unknown",
    6: "Replay",
    7: "Sub",
    8: "IOFirmware",
    9: "Peripth",
    10: "DAL Standalone",
    11: "Bootloader",
    12: "Blimp",
    13: "Heli",
}

mode_mapping_apm = {
    0: 'MANUAL',
    1: 'CIRCLE',
    2: 'STABILIZE',
    3: 'TRAINING',
    4: 'ACRO',
    5: 'FBWA',
    6: 'FBWB',
    7: 'CRUISE',
    8: 'AUTOTUNE',
    10: 'AUTO',
    11: 'RTL',
    12: 'LOITER',
    13: 'TAKEOFF',
    14: 'AVOID_ADSB',
    15: 'GUIDED',
    16: 'INITIALISING',
    17: 'QSTABILIZE',
    18: 'QHOVER',
    19: 'QLOITER',
    20: 'QLAND',
    21: 'QRTL',
    22: 'QAUTOTUNE',
    23: 'QACRO',
    24: 'THERMAL',
    25: 'LOITERALTQLAND',
}

mode_mapping_acm = {
    0: 'STABILIZE',
    1: 'ACRO',
    2: 'ALT_HOLD',
    3: 'AUTO',
    4: 'GUIDED',
    5: 'LOITER',
    6: 'RTL',
    7: 'CIRCLE',
    8: 'POSITION',
    9: 'LAND',
    10: 'OF_LOITER',
    11: 'DRIFT',
    13: 'SPORT',
    14: 'FLIP',
    15: 'AUTOTUNE',
    16: 'POSHOLD',
    17: 'BRAKE',
    18: 'THROW',
    19: 'AVOID_ADSB',
    20: 'GUIDED_NOGPS',
    21: 'SMART_RTL',
    22: 'FLOWHOLD',
    23: 'FOLLOW',
    24: 'ZIGZAG',
    25: 'SYSTEMID',
    26: 'AUTOROTATE',
    27: 'AUTO_RTL',
}

mode_mapping_rover = {
    0: 'MANUAL',
    1: 'ACRO',
    2: 'LEARNING',
    3: 'STEERING',
    4: 'HOLD',
    5: 'LOITER',
    6: 'FOLLOW',
    7: 'SIMPLE',
    8: 'DOCK',
    9: 'CIRCLE',
    10: 'AUTO',
    11: 'RTL',
    12: 'SMART_RTL',
    15: 'GUIDED',
    16: 'INITIALISING'
}

mode_mapping_tracker = {
    0: 'MANUAL',
    1: 'STOP',
    2: 'SCAN',
    4: 'GUIDED',
    10: 'AUTO',
    16: 'INITIALISING'
}

mode_mapping_sub = {
    0: 'STABILIZE',
    1: 'ACRO',
    2: 'ALT_HOLD',
    3: 'AUTO',
    4: 'GUIDED',
    7: 'CIRCLE',
    9: 'SURFACE',
    16: 'POSHOLD',
    19: 'MANUAL',
}

mode_mapping_blimp = {
    0: 'LAND',
    1: 'MANUAL',
    2: 'VELOCITY',
    3: 'LOITER',
    4: 'RTL',
}

AP_MAV_TYPE_MODE_MAP = {
    1: mode_mapping_rover,
    2: mode_mapping_acm,
    3: mode_mapping_apm,
    4: mode_mapping_tracker,
    7: mode_mapping_sub,
    12: mode_mapping_blimp,
    13: mode_mapping_acm,  # Helicopter uses same modes as APM Copter
    }
