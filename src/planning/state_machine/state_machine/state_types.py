import enum

class TopLevelMode(enum.Enum):
    NORMAL_RACING = 'NORMAL_RACING'
    RECOVERY = 'RECOVERY'
    SAFETY = 'SAFETY'

class StateType(enum.Enum):
    # Normal Racing Sub-states
    GB_TRACK = 'GB_TRACK' 
    TRAILING = 'TRAILING' 
    OVERTAKE = 'OVERTAKE' 
    FTGONLY = 'FTGONLY'

    # Recovery Sub-states
    RECOVERY_STUCK = 'RECOVERY_STUCK'
    RECOVERY_REVERSE = 'RECOVERY_REVERSE'
    RECOVERY_ALIGN = 'RECOVERY_ALIGN'
    
    # Safety Sub-states
    LOW_BAT = 'LOW_BAT'

