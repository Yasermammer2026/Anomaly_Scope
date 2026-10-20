# STWIN.box sensor logger — compact checkout

Target: **STEVAL-STWINBX1 (STWIN.box), STM32U585AI**.
Upstream release: **FP-SNS-DATALOG2 v3.4.0**, commit `16f61ea1e0ad0dae529545b1e1ba50b874cbaacc`.

This shallow, partial Git clone checks out only the STWIN.box DATALOG2 application and its STM32CubeIDE source/header dependencies. Other boards, UltrasoundFFT/PDetect applications, IAR/Keil projects, middleware tests/examples, and general PC utilities are omitted. The original firmware behavior and sensor configuration are retained, including optional sensor support referenced by this board's build. This reduces the source checkout; it does not reduce firmware flash/RAM usage.

## Open and build

1. Open STM32CubeIDE (the upstream project names version 2.2.0).
2. Choose **File > Import > Existing Projects into Workspace**.
3. Select `Projects/STM32U585AI-STWIN.box/Applications/DATALOG2/STM32CubeIDE`.
4. Leave **Copy projects into workspace** unchecked to preserve relative dependency paths.
5. Select `DATALOG2-STWIN.box`, then **Project > Build Project** (Debug or Release).

The upstream application README remains in the DATALOG2 directory. Prebuilt vendor firmware is retained in its `Binary` directory. No board has been flashed by this task.

## Validation and maintenance

`STWINBOX-MANIFEST.json` records the upstream revision, retained file count/bytes, and linked-resource validation. All CubeIDE linked resources were checked for existence. One upstream metadata link to `HWDriverMap.h` was corrected to its actual location under `Middlewares/ST/eLooM/Inc/drivers`; source code is unchanged.

A compiler build and hardware acquisition test have not been run for this checkout. STM32CubeIDE's headless build tool was not found in the checked installation locations.

Run `python select-stwinbox.py` from this directory to regenerate the dependency selection and repeat the link checks. It requires Git, Python and network access for blobs not already cached. Keep the sparse checkout enabled; disabling it downloads the other boards again. Only STM32CubeIDE is covered by the dependency selection.

The older projects elsewhere in the workspace are preserved.
