#ifndef CONFIGURATION_STORE_H
#define CONFIGURATION_STORE_H

#include "MarlinConfig.h"

void Config_ResetDefault();

FORCE_INLINE void Config_StoreSettings() {}
FORCE_INLINE void Config_RetrieveSettings() { Config_ResetDefault(); }

#endif //CONFIGURATION_STORE_H
