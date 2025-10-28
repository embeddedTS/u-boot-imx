// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2024 Technologic Systems, Inc. (dba embeddedTS)
 */

#include <env.h>
#include <net.h>
#include <stdbool.h>
#include <stdio.h>
#include <vsprintf.h>

#include "ts-macs.h"
#include "wizard.h"

static void increment_mac(uint8_t *);

/*
 * Obtain at least the first MAC then increment it to assign each additional
 * MAC needed by this platform.
 */
void setup_mac_addresses(int n_macs)
{
	unsigned char enetaddr[6];
	char mac_str[16];
	int ret, i;

	ret = wizard_read_mac(enetaddr);
	if (ret) {
		printf("Error reading MAC Address from the Wizard!\n");
		return;
	}

	if (!is_valid_ethaddr(enetaddr)) {
		printf("Read an invalid MAC Address from the Wizard!\n");
		return;
	}

	eth_env_set_enetaddr("ethaddr", enetaddr);

	for (i = 1; i < n_macs; i++) {
		increment_mac(enetaddr);
		snprintf(mac_str, sizeof(mac_str), "eth%daddr", i);
		eth_env_set_enetaddr(mac_str, enetaddr);
	}
}

static void increment_mac(uint8_t *mac)
{
	int i;

	for (i = 5; i >= 3; i--) {
		mac[i] += 1;
		if (mac[i])
			break;
	}
}
