#define V5_EN(a)        _TRISF0=(!a)
#define V5_ON(a)        _LATF0=a

#define SYS_BUS_EN(a)   _TRISE1=(!a)
#define SYS_BUS_ON(a)   _LATE1=a

#define PWR_BUTTON()    _RB11

//battery charger pins
#define BQ24745_EN(a)   _TRISE3=(!a)
#define BQ24745_ON(a)   _LATE3=a

#define BQ24745_ACOK()    _RE2

//motor controller pins
// input-capture inputs
#define A_HI_R_RPN_PIN        (_RP11R)
#define A_LO_R_RPN_PIN        (_RP24R)
#define B_HI_R_RPN_PIN        (_RP23R)//TEST1_RPN_PIN
#define B_LO_R_RPN_PIN        (_RP22R)//TEST2_RPN_PIN

#define A_HI_L_RPN_PIN        (_RP25R)
#define A_LO_L_RPN_PIN        (_RP20R)
#define B_HI_L_RPN_PIN        (_RP2R)//TEST1_RPN_PIN
#define B_LO_L_RPN_PIN        (_RP12R)//TEST2_RPN_PIN
