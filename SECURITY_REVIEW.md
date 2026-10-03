# Security review scope

This publication contains source archives only. It does not deploy a website, flash firmware, run training, publish group datasets, or validate native CAD. The exact reviewed Git commit and full 20-item publication checklist are recorded in the consolidated Year2 report.

Current and historical text was scanned with Gitleaks 8.30.1 with secret redaction. Python dependencies are constrained to the resolver snapshot audited against OSV; browser packages are pinned and audited with npm. Python source and notebook syntax and the documented local smoke tests were checked. Hardware/model/CAD verification remains an explicit limitation.

No database, cloud billing account, user-account service, server-side upload endpoint, or sensitive session cookie is introduced. Production HTTPS, hosting headers, quotas, multi-instance authorization and hardware safety require their own release checks before deployment or flashing.

Wi-Fi and HTTP are disabled in the default firmware; enabling the incomplete network prototype is a compile-time error. The archived PlatformIO ESP32 SDK family (Arduino 2.0.17) must not be used to certify a network release. Before enabling any HTTP control, upgrade and recheck Espressif advisories, authorization, request limits and transport security. This archive has no upload handler or WebServer password authentication, so those vulnerable APIs are not used.

- https://github.com/espressif/arduino-esp32/security/advisories/GHSA-8cmm-3887-r32j
- https://github.com/espressif/arduino-esp32/security/advisories/GHSA-w887-pg2w-mfj2
