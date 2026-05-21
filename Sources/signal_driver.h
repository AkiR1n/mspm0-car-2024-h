#ifndef SIGNAL_DRIVER_H_
#define SIGNAL_DRIVER_H_

#include "app_state.h"

void signal_driver_init(void);
void signal_emit(app_event_id_t event_id);
void signal_driver_update(uint16_t elapsed_ms);

#endif
