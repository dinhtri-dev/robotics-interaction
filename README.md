# Robotics Interaction Experiments

Year2 source archive by dinhtri-dev. Original workspace files are unchanged.

## Contents and status

firmware/UNO is the arm controller; firmware/XE contains ESP32/Nano vehicle experiments. Wi-Fi credentials are local configuration. Remote motor control is disabled by default; do not expose these prototypes to a public network.

## Run / inspect

Open the appropriate .ino directory in Arduino IDE. Configure ignored config.local.h before Wi-Fi experiments. No physical controller is exercised by archive checks.

Install only this project's listed dependencies in a separate virtual environment. Model binaries, large datasets, private configuration and generated output are excluded. Check DATA_AND_MODELS.md when present.

## Archive validation

See ARCHIVE_STATUS.md and SECURITY_REVIEW.md for the exact publication scope, tests and remaining limitations. A successful source-archive check does not certify production deployment, firmware flashing, model accuracy or CAD geometry.

## Future work

The consolidated Google document records project-specific fixes, missing functions and suggested features. This commit focuses on reproducible archiving, not implementing that roadmap.

## Directory guide

- `firmware`

## Archived Wi-Fi firmware

The default firmware does not connect Wi-Fi or start HTTP. Network control is blocked at compile time when enabled: the prototype lacks deployment authorization/transport security, and older Arduino-ESP32 dependencies must be upgraded before enabling network code. See SECURITY_REVIEW.md. Serial experiments remain separate and have not been flashed or connected to hardware by archive QA.

## Tests actually run

3 complete firmware targets compiled with PlatformIO (build only, never upload).
