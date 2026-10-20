# Sensor recording GUI (WSL/Linux)

Compact STDATALOG-PYSDK 1.5.0 bundle from https://github.com/STMicroelectronics/stdatalog-pysdk.

One-time system prerequisites (Ubuntu):

```bash
sudo apt update
sudo apt install python3-venv libasound-dev portaudio19-dev libportaudio2 libportaudiocpp0 libxcb-cursor-dev
bash setup.sh
```

USB permissions: retained original scripts in `linux_setup/`. Run its setup script with the privileges it requires. In WSL, forward the board with Windows usbipd; GUI display requires WSLg. The board needs compatible DATALOG firmware.

Start each session:

```bash
bash start-gui.sh
```

The four wheels include the GUI and its required SDK libraries. Python dependencies are downloaded during setup; the installed environment will be larger than this bundle. Source trees, older wheels, extra examples and Git history were removed. Re-clone the upstream repository for development or updates.

Setup and hardware recording have not been verified on this machine.
