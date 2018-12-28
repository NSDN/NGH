#ifndef __NGH_BIOS_H_
#define __NGH_BIOS_H_


void ngh_setup();
void ngh_loop();

#ifdef __cplusplus
extern "C" {
#endif

void delay(int t);
void processEvent();

#ifdef __cplusplus
}
#endif


#endif
