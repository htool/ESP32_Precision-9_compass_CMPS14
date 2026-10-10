# PlatformIO extra script: compile-time include of gitignored wifi_secrets.h
# at the project root (not under src/). Missing file → AP provisioning.
#
# A content stamp is added to the compile flags so editing wifi_secrets.h
# actually rebuilds firmware (plain -include is not a SCons dependency).

import hashlib
from pathlib import Path

Import("env")  # type: ignore  # PlatformIO

secrets = Path(env["PROJECT_DIR"]) / "wifi_secrets.h"
if secrets.is_file():
    digest = hashlib.sha256(secrets.read_bytes()).hexdigest()
    # Keep this in signed 32-bit so it is only a rebuild cookie, never logged
    # as a 64-bit printf argument (that panic'd with LoadProhibited).
    stamp = int(digest[:7], 16)
    env.Append(CCFLAGS=["-include", str(secrets)])
    env.Append(CPPDEFINES=[("WIFI_SECRETS_STAMP", str(stamp))])
    print(f"Including Wi-Fi secrets from {secrets} (stamp {digest[:7]})")
else:
    print("No wifi_secrets.h; firmware will start a config AP")
