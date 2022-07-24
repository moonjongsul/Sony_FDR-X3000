import pyudev

context = pyudev.Context()
subsystem = "video4linux"

for device in context.list_devices(subsystem=subsystem):
    id_v4l_product = device.get("ID_V4L_PRODUCT")
    id_serial       = device.get("ID_SERIAL")
    id_serial_short = device.get("ID_SERIAL_SHORT")
    id_vendor = device.get("ID_VENDOR_ID")
    id_product = device.get("ID_MODEL_ID")
    dev_name = device.get("DEVNAME")
    dev_port = dev_name[-1]

    print("")
    print("=" * 70)
    print(f"[INFO] Detected Camera Info.")
    print(f"[INFO]   - V4L Product:          {id_v4l_product}")
    print(f"[INFO]   - Full serial:          {id_serial}")
    print(f"[INFO]   - Short serial:         {id_serial_short}")
    print(f"[INFO]   - idVendor & idProduct: {id_vendor}:{id_product}")
    print(f"[INFO]   - /dev name:            {dev_name}")
    print("=" * 70)

