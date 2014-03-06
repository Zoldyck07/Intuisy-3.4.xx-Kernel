/*
 * hostapd / RADIUS Accounting
 * Copyright (c) 2002-2005, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#ifndef ACCOUNTING_H
#define ACCOUNTING_H

void accounting_sta_interim(struct hostapd_data *hapd, struct sta_info *sta);
#ifdef CONFIG_NO_ACCOUNTING
static inline void accounting_sta_start(struct hostapd_data *hapd,
					struct sta_info *sta)
{
}

static inline void accounting_sta_stop(struct hostapd_data *hapd,
				       struct sta_info *sta)
{
}

static inline int accounting_init(struct hostapd_data *hapd)
{
	return 0;
}

static inline void accounting_deinit(struct hostapd_data *hapd)
{
}
#else /* CONFIG_NO_ACCOUNTING */
void accounting_sta_start(struct hostapd_data *hapd, struct sta_info *sta);
void accounting_sta_stop(struct hostapd_data *hapd, struct sta_info *sta);
int accounting_init(struct hostapd_data *hapd);
void accounting_deinit(struct hostapd_data *hapd);
#endif /* CONFIG_NO_ACCOUNTING */

#endif /* ACCOUNTING_H */