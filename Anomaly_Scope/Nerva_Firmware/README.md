# Nerva Firmware

Independent export of the **verified operational STWIN.box firmware**, not a diagnostic-only or experimental build. No connected board was accessed during this export.

- **Hardware:** STEVAL-STWINBX1 / STM32U585AIIxQ / EMW3080 firmware V2.3.4.
- **Base:** FP-SNS-DATALOG2 V3.3.0 plus the existing SD/lossless Wi-Fi cycle implementation.
- **Verified provisioned ELF SHA-256:** `41fea4e1a72cba4253b7beb1e98bcace92550143d18b821dc167543cf3d8b3eb`.

The original verified ELF/BIN are **not included because they contain private Wi-Fi credentials**. Their identities are recorded in `verification/selected-operational-version.json`. The ELF/BIN in `firmware/` are fresh, credential-free builds from the same operational source. They have different hashes and have not been flashed or hardware-qualified.

## Contents

- `SDK/`: original directory layout, complete application, sensor/board drivers, STM32U5 HAL, CMSIS, ThreadX, FileX, NetX Duo, USBX, eLooM and supporting middleware. Linker script and CubeIDE project are included.
- `tools/build_firmware.py`: isolated build helper; no board operations.
- `firmware/`: rebuilt ELF/BIN and linker map.
- `verification/`: source provenance, dependency/build checks and SHA-256 hashes.
- `Host/`: optional existing PC receiver/decoder; see its README. No Dashboard code.

Old builds/caches, datasets, logs, vendor tests, unrelated board families and unused alternate IDE projects are excluded. Integrated helpers belonging to the operational application remain unchanged. Original component licenses are retained.

## Build

Tested: **STM32CubeIDE 2.2.0**, bundled **GNU Tools for STM32 14.3.rel1 / GCC 14.3.1**. Python 3.10+ is required only for the helper. SDK dependencies are bundled; install the IDE/compiler separately.

From this folder:

```powershell
python tools/build_firmware.py --headless-build "C:/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/headless-build.bat"
```

Adjust the IDE path or set `STM32CUBEIDE_HEADLESS`. The helper builds a fresh temporary SDK copy, audits compiler dependencies, saves ELF/BIN/MAP and build proof, then removes its own build cache. It never flashes.

Alternatively import `SDK/Projects/STM32U585AI-STWIN.box/Applications/DATALOG2/STM32CubeIDE` into CubeIDE and build **DATALOG2-STWIN.box / Release**. Move the whole export, not the IDE subfolder alone. CubeMX regeneration and other build configurations are not qualified here.

## Configure and deploy

Fill the empty `cycle30_ssid` and `cycle30_password` arrays in `SDK/Projects/STM32U585AI-STWIN.box/Applications/DATALOG2/NetX/app/cycle30_network_private.h`, then rebuild. The adjacent `.example.h` is the public template. Keep configured headers and provisioned images private.

The existing PC endpoint is `192.168.8.4:19195` in `NetX/app/cycle30.inc`; configure that reachable PC address or change it and rebuild.

**Automatic startup behavior is preserved.** The compiled duration is 1,800 seconds; CRC-protected `CYCLEA.CFG` / `CYCLEB.CFG` SD records can override duration and cycle limits. The verified one-hour test used a 3,600-second SD/RAM configuration, not different source code. Completed counters can prevent recording; pending-upload discovery precedes the limit check. This export adds no diagnostic startup barrier or single-attempt controller. Preserve existing recordings and configure deliberately before running.

For flashing, install **STM32CubeProgrammer** (tested 2.23.0) and the ST-LINK driver. First confirm acquisition and SD writing are inactive; connect through SWD **under reset**. Program and verify `firmware/DATALOG2-STWIN.box.elf`, then deliberately choose when to run. ELF covers both flash regions, including `.bench_text` at `0x08100000`. BIN, if used, starts at `0x08000000` and includes the gap. These are instructions only; packaging did not flash or reset hardware.

## Current functionality and verification

Ten unchanged sensor streams; native HSD 2.0 SD files; proper closure; lossless TCP-over-Wi-Fi delivery with CRC, per-file SHA-256, persisted reconstruction and durable device-confirmed receipt; original SD files retained. It is not MQTT. The source version recorded 3,600.212 seconds and delivered all twelve files (2.677 GB) in 2,154.880 seconds including verification; that timing excludes controlled pauses. Zero physical sensor loss is not proven: one magnetometer IRQ was unaccounted at shutdown.

The exported Release build passed with zero errors/warnings and no dependencies on the original project. Byte-for-byte reproduction of the original image is not claimed: private deployment strings were removed and ELF debug paths differ. All other operational firmware source and project settings are preserved. See `verification/build-verification.json` for the rebuilt artifact hashes.
