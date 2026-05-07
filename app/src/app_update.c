/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include "app_update.h"

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#ifdef CONFIG_BOOTLOADER_MCUBOOT
#include <zephyr/dfu/mcuboot.h>
#endif

LOG_MODULE_REGISTER(app_update, CONFIG_APP_LOG_LEVEL);

#if defined(CONFIG_BOOTLOADER_MCUBOOT) && defined(CONFIG_APP_UPDATE_AUTO_CONFIRM_AFTER_BOOT)
static void app_update_auto_confirm_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(app_update_auto_confirm_work,
			       app_update_auto_confirm_work_handler);
#endif

const char *app_update_swap_type_to_string(int swap_type)
{
#ifdef CONFIG_BOOTLOADER_MCUBOOT
	switch (swap_type) {
	case BOOT_SWAP_TYPE_NONE:
		return "none";
	case BOOT_SWAP_TYPE_TEST:
		return "test";
	case BOOT_SWAP_TYPE_PERM:
		return "permanent";
	case BOOT_SWAP_TYPE_REVERT:
		return "revert";
	case BOOT_SWAP_TYPE_FAIL:
		return "fail";
	default:
		return "unknown";
	}
#else
	ARG_UNUSED(swap_type);
	return "disabled";
#endif
}

void app_update_get_status(struct app_update_status *status)
{
	if (!status) {
		return;
	}

	*status = (struct app_update_status){
		.mcuboot_enabled = IS_ENABLED(CONFIG_BOOTLOADER_MCUBOOT),
		.confirmed = true,
		.active_slot = -1,
		.swap_type = 0,
	};

#ifdef CONFIG_BOOTLOADER_MCUBOOT
	status->confirmed = boot_is_img_confirmed();
	status->active_slot = boot_fetch_active_slot();
	status->swap_type = mcuboot_swap_type();
#endif
}

int app_update_confirm(void)
{
#ifdef CONFIG_BOOTLOADER_MCUBOOT
	if (boot_is_img_confirmed()) {
		return 0;
	}

	return boot_write_img_confirmed();
#else
	return -ENOTSUP;
#endif
}

int app_update_request_test(void)
{
#ifdef CONFIG_BOOTLOADER_MCUBOOT
	return boot_request_upgrade(BOOT_UPGRADE_TEST);
#else
	return -ENOTSUP;
#endif
}

int app_update_request_permanent(void)
{
#ifdef CONFIG_BOOTLOADER_MCUBOOT
	return boot_request_upgrade(BOOT_UPGRADE_PERMANENT);
#else
	return -ENOTSUP;
#endif
}

#if defined(CONFIG_BOOTLOADER_MCUBOOT) && defined(CONFIG_APP_UPDATE_AUTO_CONFIRM_AFTER_BOOT)
static void app_update_auto_confirm_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (boot_is_img_confirmed()) {
		LOG_INF("MCUboot image already confirmed");
		return;
	}

	int rc = boot_write_img_confirmed();

	if (rc == 0) {
		LOG_INF("MCUboot test image auto-confirmed");
	} else {
		LOG_ERR("Failed to auto-confirm MCUboot test image: %d", rc);
	}
}
#endif

void app_update_init(void)
{
#ifdef CONFIG_BOOTLOADER_MCUBOOT
	struct app_update_status status;

	app_update_get_status(&status);
	LOG_INF("MCUboot update state: confirmed=%s active_slot=%d swap=%s",
		status.confirmed ? "yes" : "no", status.active_slot,
		app_update_swap_type_to_string(status.swap_type));

#ifdef CONFIG_APP_UPDATE_AUTO_CONFIRM_AFTER_BOOT
	if (!status.confirmed) {
		k_work_schedule(&app_update_auto_confirm_work,
				K_MSEC(CONFIG_APP_UPDATE_AUTO_CONFIRM_DELAY_MS));
		LOG_WRN("MCUboot test image will auto-confirm in %d ms",
			CONFIG_APP_UPDATE_AUTO_CONFIRM_DELAY_MS);
	}
#else
	if (!status.confirmed) {
		LOG_WRN("MCUboot test image is unconfirmed; run `mcuboot confirm` after validation");
	}
#endif
#else
	LOG_INF("MCUboot update support disabled");
#endif
}
