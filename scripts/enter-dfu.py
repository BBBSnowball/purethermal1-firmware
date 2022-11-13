#! /usr/bin/env nix-shell
#! nix-shell -i python -p python3.withPackages(p:[p.pyusb])

# Example: make SYSTEM=arm-none-eabihf- && ./scripts/enter-dfu.py dfu-util -a 0 -D main.bin -s 0x08000000:leave

import sys, time, subprocess
import usb.core

def send_vendor_read_request(dev, bRequest, expect_reset=False):
    try:
        if dev.is_kernel_driver_active(0):
            dev.detach_kernel_driver(0)
    except usb.core.USBError as e:
        if e.errno == 2:
            # "[Errno 2] Entity not found" means that no driver is attached. This is ok.
            pass
        else:
            raise

    try:
        print(dev.ctrl_transfer(0xc0, bRequest, 0, 0, 32))
    except usb.core.USBError as e:
        # We expect "[Errno 32] Pipe error".
        if not expect_reset or e.errno != 32:
            raise

def enter_dfu():
    dev = usb.core.find(idVendor=0x0483, idProduct=0xdf11)
    if dev:
        print("Already in bootloader")
        return True

    dev = usb.core.find(idVendor=0x1e4e, idProduct=0x0100)
    if dev:
        send_vendor_read_request(dev, 123, expect_reset=True)
        return True

    return False

def reset_device(dev):
    send_vendor_read_request(dev, 122, expect_reset=True)
    time.sleep(1)
    dev2 = wait_for_device(0.5, idVendor=0x1e4e, idProduct=0x0100)
    print(dev)
    if dev.bus == dev2.bus and dev.address == dev2.address:
        print("Reset didn't work. Adress has remained the same.")

def ping(dev):
    send_vendor_read_request(dev, 120)

def wait_for_device(timeout, **kwargs):
    stepsize = min(timeout/10, 0.1)
    for _ in range(round(timeout/stepsize)):
        dev = usb.core.find(**kwargs)
        if dev:
            return dev
        time.sleep(stepsize)
    return None


if sys.argv[1:] == [] or sys.argv[1:] == ["bootloader"]:
    if not enter_dfu():
        raise Exception("Device not found")
elif sys.argv[1:] == ["reset"]:
    dev = usb.core.find(idVendor=0x1e4e, idProduct=0x0100)
    reset_device(dev)
elif sys.argv[1:] == ["ping"]:
    dev = usb.core.find(idVendor=0x1e4e, idProduct=0x0100)
    ping(dev)
else:
    if not enter_dfu():
        raise Exception("Device not found")

    if len(sys.argv) > 1:
        dev = wait_for_device(1, idVendor=0x0483, idProduct=0xdf11)
        if not dev:
            raise Exception("Timeout while waiting for device to enter bootloader")

        subprocess.run(sys.argv[1:])

        dev = wait_for_device(2, idVendor=0x1e4e, idProduct=0x0100)
        print(dev)
        if not dev:
            raise Exception("Timeout while waiting for device to leave bootloader")
        reset_device(dev)
