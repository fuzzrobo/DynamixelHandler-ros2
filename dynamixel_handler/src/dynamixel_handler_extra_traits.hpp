#ifndef DYNAMIXEL_HANDLER_EXTRA_TRAITS_HPP
#define DYNAMIXEL_HANDLER_EXTRA_TRAITS_HPP

#include "dynamixel_communicator.h"

#include <array>

enum class ExtraStorage {
    ROM,
    RAM,
    INSTRUCTION,
};

enum class ExtraField {
    PROTOCOL_TYPE,
    DRIVE_MODE,
    SHUTDOWN,
    RESTORE_CONFIGURATION,
    PWM_SLOPE,
    HOMING_OFFSET,
    RETURN_DELAY_TIME,
    BUS_WATCHDOG,
    LED,
    SHADOW_ID,
    MOVING_THRESHOLD,
    STATUS_RETURN_LEVEL,
    MOVING_STATUS,
    REALTIME_TICK,
    MOVING,
    REGISTERED_INSTRUCTION,
    BACKUP_READY,
    REBOOT,
};

struct ExtraFieldTrait {
    ExtraField id;
    const char* name;
    ExtraStorage storage;
    bool support_x;
    bool support_p;
    bool support_pro;
    bool writable_hw;
    bool writable_policy;
    const char* bit_note; // e-Manual memo (bit fields / cross-series differences)
};

inline constexpr std::array<ExtraFieldTrait, 18> extra_field_traits = {{
    {ExtraField::PROTOCOL_TYPE,          "protocol_type",          ExtraStorage::ROM,         true,  true,  false, true,  false, ""},
    {ExtraField::DRIVE_MODE,             "drive_mode",             ExtraStorage::ROM,         true,  true,  false, true,  true ,
        "X/P: bit0 reverse, bit2 time-based, bit3 auto-activate. Y: bit6 dynamic-brake, bit3 unused. Pro: no field."},
    {ExtraField::SHUTDOWN,               "shutdown",               ExtraStorage::ROM,         true,  true,  true,  true,  true ,
        "bit0/2/3/4/5 meaning is consistent across X/P/Pro. bit1 motor-hall-sensor exists on P/Pro (X unused)."},
    {ExtraField::RESTORE_CONFIGURATION,  "restore_configuration",  ExtraStorage::ROM,         true,  true,  false, true,  true ,
        "startup_configuration: bit0 torque-on is shared. bit1 RAM-restore is X/P only (Y bit1 unused, Pro no field)."},
    {ExtraField::PWM_SLOPE,              "pwm_slope_percent",      ExtraStorage::ROM,         true,  false, false, true,  true ,
        "X330/XL330 only (address 62). XD/XM/XH/XW/XC430/XL430/Pro: no field."},
    {ExtraField::HOMING_OFFSET,          "homing_offset_deg",      ExtraStorage::ROM,         true,  true,  true,  true,  true , ""},
    {ExtraField::RETURN_DELAY_TIME,      "return_delay_time_us",   ExtraStorage::ROM,         true,  true,  true,  true,  true , ""},
    {ExtraField::BUS_WATCHDOG,           "bus_watchdog_ms",        ExtraStorage::RAM,         true,  true,  false, true,  true , ""},
    {ExtraField::LED,                    "led",                    ExtraStorage::RAM,         true,  true,  true,  true,  true , ""},
    {ExtraField::SHADOW_ID,              "shadow_id",              ExtraStorage::ROM,         true,  true,  false, true,  true , ""},
    {ExtraField::MOVING_THRESHOLD,       "moving_threshold_deg_s", ExtraStorage::ROM,         true,  true,  true,  true,  true , ""},
    {ExtraField::STATUS_RETURN_LEVEL,    "status_return_level",    ExtraStorage::RAM,         true,  true,  true,  true,  false, ""},
    {ExtraField::MOVING_STATUS,          "moving_status",          ExtraStorage::RAM,         true,  true,  false, false, false,
        "No shared-bit meaning collision. X: bit3/1/0 + bits5-4 profile. P: bit3/1 unused. Y adds bit2 moving, bit6 position-in-range."},
    {ExtraField::REALTIME_TICK,          "realtime_tick_s",        ExtraStorage::RAM,         true,  true,  false, false, false, ""},
    {ExtraField::MOVING,                 "moving",                 ExtraStorage::RAM,         true,  true,  true,  false, false, ""},
    {ExtraField::REGISTERED_INSTRUCTION, "registered_instruction", ExtraStorage::RAM,         true,  true,  true,  false, false, ""},
    {ExtraField::BACKUP_READY,           "backup_ready",           ExtraStorage::RAM,         true,  true,  false, false, false, ""},
    {ExtraField::REBOOT,                 "reboot",                 ExtraStorage::INSTRUCTION, true,  true,  true,  true,  true , ""},
}};

inline const ExtraFieldTrait& Trait(ExtraField field) {
    for (const auto& t : extra_field_traits) if (t.id == field) return t;
    return extra_field_traits[0];
}

inline bool IsSupported(const ExtraFieldTrait& trait, DynamixelSeries series) {
    if (series == SERIES_X) return trait.support_x;
    if (series == SERIES_P) return trait.support_p;
    if (series == SERIES_PRO) return trait.support_pro;
    if (series == SERIES_UNKNOWN) return true;  // dummy: treat all fields as supported
    return false;
}

#endif  // DYNAMIXEL_HANDLER_EXTRA_TRAITS_HPP
