# Optional PC receiver

This is the existing single-connection receiver and lossless decoder, copied for protocol integration. Only packaging paths were adjusted, and standalone corpus benchmark entrypoints/fixture discovery were omitted. Codec algorithms and the transfer/durable-receipt protocol are unchanged.

Install Python 3.10+, native host GCC (not ARM GCC), and the packages in `requirements.txt`. From the Nerva_Firmware folder:

```powershell
python -m pip install -r Host/requirements.txt
python Host/tools/receive_once.py --listen 192.168.8.4 --storage Host/artifacts/new-run --max-storage-bytes 3489660928 --storage-approval Host/storage-approval.example.json
```

The PC needs the reachable address configured in the firmware, sufficient free disk space, and inbound TCP port 19195 as appropriate. Use a new storage directory. The example approval sets a 3.25 GiB local archive cap. The receiver accepts one connection, verifies reconstruction/CRC/SHA-256 and persisted archive readback, sends a durable receipt, waits for device confirmation, and exits. It does not delete board data. No receiver was started during packaging.
