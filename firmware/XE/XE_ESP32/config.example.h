#pragma once
// Define local credentials in ignored config.local.h; never commit real values.
#ifndef YEAR2_WIFI_SSID
#define YEAR2_WIFI_SSID ""
#endif
#ifndef YEAR2_WIFI_PASSWORD
#define YEAR2_WIFI_PASSWORD ""
#endif
// Network control is disabled in the archive's default configuration.
#ifndef YEAR2_ENABLE_REMOTE_CONTROL
#define YEAR2_ENABLE_REMOTE_CONTROL 0
#endif

#if YEAR2_ENABLE_REMOTE_CONTROL
#error "Network prototype is archived only: upgrade SDK and implement authorization/HTTPS before enabling it"
#endif
