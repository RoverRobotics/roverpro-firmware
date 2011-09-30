
#define ZOOM_TX_LENGTH 6
extern unsigned char zoom_tx[ZOOM_TX_LENGTH];

void init_zoom(void);
void zoom_in(void);
void zoom_out(void);
void zoom_stop(void);
void digital_zoom_on(void);
