---
name: elrs-build-flash
description: Build ExpressLRS firmware with hardware config selection and WiFi upload. Use when user wants to compile and flash ELRS firmware to a device over WiFi. Variables: DEVICE_IP, JSON_NUM, TARGET_NAME.
---

# ELRS Build & Flash

Build ExpressLRS Unified firmware, select hardware config by number, and upload via WiFi.

## Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `DEVICE_IP` | IP address of the target ELRS device | (required, no default) |
| `JSON_NUM` | Interactive list number of the hardware config in `targets.json` | (required, no default) |
| `TARGET_NAME` | PlatformIO build target name | `Unified_ESP8285_2400_RX_via_WIFI` |

The user can set these by saying things like:
- "IP 是 192.168.3.32，用第 20 个 json"
- "DEVICE_IP=192.168.3.32 JSON_NUM=20"
- "build and flash, target=Unified_ESP8285_2400_RX_via_WIFI, ip=..., json=20"

If any required variable is missing, ask the user for it before proceeding.

## Workflow

All commands run from `src/` directory.

### Step 1: Build firmware

Use a Python pty to simulate an interactive terminal so the `UnifiedConfiguration.py` post-build script receives the JSON selection number automatically:

```bash
cd /home/ncer/ExpressLRS/src && python3 -c "
import pty, os, select, time, sys
pid, fd = pty.fork()
if pid == 0:
    os.execvp('pio', ['pio', 'run', '-e', '${TARGET_NAME}'])
else:
    sent = False
    while True:
        try:
            r, w, e = select.select([fd], [], [], 1.0)
            if r:
                data = os.read(fd, 1024)
                if not data: break
                sys.stdout.buffer.write(data); sys.stdout.flush()
                if not sent and b'Choose a configuration' in data:
                    time.sleep(0.2); os.write(fd, b'${JSON_NUM}\n'); sent = True
        except OSError: break
    os.waitpid(pid, 0)
"
```

Replace `${TARGET_NAME}` and `${JSON_NUM}` with actual values.

**Note:** If the user needs a specific layout file rather than a number, look up the corresponding number first:
```bash
cd /home/ncer/ExpressLRS/src && python3 -c "
import json
with open('hardware/targets.json') as f:
    targets = json.load(f)
results = []
for cat_name, cat in targets.items():
    if cat_name == 'misc': continue
    for mod_type in ['rx_2400', 'rx_900', 'rx_dual', 'tx_2400', 'tx_900']:
        devs = cat.get(mod_type, {})
        if isinstance(devs, dict):
            for var_name, var in devs.items():
                if isinstance(var, dict):
                    results.append(var.get('product_name', ''))
for i, name in enumerate(results, 1):
    print(f'{i}) {name}')
"
```

### Step 2: Upload via WiFi

```bash
cd /home/ncer/ExpressLRS/src/.pio/build/${TARGET_NAME} && \
FILESIZE=$(stat -c%s firmware.bin) && \
curl -X POST "http://${DEVICE_IP}/update" \
  -H "X-FileSize: $FILESIZE" \
  -F "data=@firmware.bin" \
  -F "force=1" \
  --noproxy "*" \
  --max-time 300
```

Replace `${TARGET_NAME}` and `${DEVICE_IP}` with actual values.

The response should be: `{"status": "ok", "msg": "Update complete. ..."}`

If the response is `"mismatch"`, add `-F "force=1"` (already included above) to bypass the target name check.

### Step 3: Verify

```bash
curl -s --noproxy "*" --max-time 10 "http://${DEVICE_IP}/hardware.json"
```

Wait ~10 seconds after upload for the device to reboot before verifying.

## Important Notes

- The `--noproxy "*"` flag is essential — the local HTTP proxy will fail to reach LAN devices
- `force=1` bypasses target name mismatch check (e.g. `_via_WIFI` vs `_via_UART` suffix difference)
- The firmware binary is gzip-compressed — the device's `/update` endpoint handles decompression
- Upload uses **multipart form data** (`-F`), NOT raw body (`--data-binary`)
- If upload speed is extremely slow (< 10 KB/s), the device may be in normal RC mode rather than WiFi update mode; trigger WiFi update on the device first
