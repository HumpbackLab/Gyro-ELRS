---
name: elrs-build-flash
description: Build and WiFi-flash ExpressLRS firmware by calling the repository script `elrs_build_flash.sh`. Use when working in this ExpressLRS repository and the user wants to build a PlatformIO environment, choose a Unified hardware config number, upload over WiFi, and verify the device after reboot. Expected inputs usually include `DEVICE_IP`, `JSON_NUM`, and optionally `TARGET_NAME`.
---

# ELRS Build And Flash

Use this skill by calling the repository script [`elrs_build_flash.sh`](/home/ncer/ExpressLRS/elrs_build_flash.sh:1). Do not reimplement the build and upload flow inline when the script is available.

`JSON_NUM` means the numeric choice shown by the actual Unified build prompt for the selected `TARGET_NAME`.

## Required Inputs

- `DEVICE_IP`: target device IP address
- `JSON_NUM`: numeric selection shown by the Unified build prompt for the chosen environment

## Optional Input

- `TARGET_NAME`: PlatformIO environment name
  Default: `Unified_ESP8285_2400_RX_via_WIFI`

If a required input is missing, ask for it before running the script.

## Workflow

1. Confirm the repository root contains `elrs_build_flash.sh`.
2. Confirm `DEVICE_IP`, `JSON_NUM`, and optional `TARGET_NAME`.
3. Run the script from the repository root.
4. Report the script result, including the upload response and post-reboot verification.

## Command

Default target:

```bash
./elrs_build_flash.sh 192.168.3.32 20
```

Explicit target:

```bash
./elrs_build_flash.sh 192.168.3.32 20 Unified_ESP8285_2400_RX_via_WIFI
```

## Rules

- Prefer the repository script over ad hoc `python3 -c`, inline PTY wrappers, or hand-written `curl` upload commands.
- If the script succeeds, trust its verification path and report its output.
- Only fall back to manual build/upload steps if the script is missing or the user explicitly asks not to use it.
- If the user provides a number, pass it through directly; do not reinterpret it through a separate `targets.json` enumeration.
