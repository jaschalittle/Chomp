void selfRightSafe();
void selfRightExtendLeft();
void selfRightExtendRight();
void selfRightExtendBoth();
void selfRightRetractLeft();
void selfRightRetractRight();
void selfRightRetractBoth();
void selfRightOff();
void autoSelfRight(bool enabled);
void manualSelfRight(uint16_t current_rc_bitfield, uint16_t diff);
void telemetrySelfRight();
void setSelfRightParameters(
        uint16_t p_min_hammer_self_right_angle,
        uint16_t p_max_hammer_self_right_angle,
        uint32_t p_max_hammer_move_duration,
        uint32_t p_max_reorient_duration,
        uint32_t p_min_retract_duration,
        uint32_t p_min_vent_duration,
        uint32_t p_manual_self_right_retract_duration,
        uint32_t p_manual_self_right_dead_duration
        );
void restoreSelfRightParameters();
