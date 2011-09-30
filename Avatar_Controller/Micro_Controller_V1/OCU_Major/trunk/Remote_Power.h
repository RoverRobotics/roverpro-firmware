#define LP_DATALINK_POWER_TRIS TRISDbits.TRISD6
#define LP_DATALINK_POWER_LAT	LATDbits.LATD6
//#define LP_DATALINK_RX_PIN		20
#define LP_DATALINK_POR			RPOR10bits.RP20R

void Remote_Power_Test(void);
void Remote_Power_Init(void);
char Power_On_Message_Received(void);
void Send_Turnon_Message(void);
void Send_Turnoff_Message(void);

