#ifndef APP_INIT_H
#define APP_INIT_H

#include "vehicle_comm.h"

int systemInit(VehicleComm_CommandCallback command_callback, const volatile int *program_running);

#endif /* APP_INIT_H */