/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef APP_UPDATE_H_
#define APP_UPDATE_H_

#include <stdbool.h>

struct app_update_status {
	bool mcuboot_enabled;
	bool confirmed;
	int active_slot;
	int swap_type;
};

void app_update_init(void);
void app_update_get_status(struct app_update_status *status);
const char *app_update_swap_type_to_string(int swap_type);
int app_update_confirm(void);
int app_update_request_test(void);
int app_update_request_permanent(void);

#endif /* APP_UPDATE_H_ */
