extern unsigned char CONTROLLER1,CONTROLLER2,ROBOT1,ROBOT2;
extern unsigned char binding_to_OCU;
void display_robot_address(unsigned char MSB, unsigned char LSB, unsigned char col, unsigned char row);
void store_address(unsigned char MSB, unsigned char LSB);
void bind_to_OCU(void);
void set_stored_address(void);
void unbind_robot(void);
