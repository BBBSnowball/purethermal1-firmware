#! /usr/bin/env nix-shell
#! nix-shell -i python -p python3.withPackages(p:[p.pyusb])

# Example: make SYSTEM=arm-none-eabihf- && ./scripts/enter-dfu.py dfu-util -a 0 -D main.bin -s 0x08000000:leave

import sys, time, subprocess
import usb.core, usb.control

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
        return dev.ctrl_transfer(0xc0, bRequest, 0, 0, 32)
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
    send_vendor_read_request(dev, 124, expect_reset=True)

    if not wait_for_device(1, expect_present=False, bus=dev.bus, address=dev.address, idVendor=0x1e4e, idProduct=0x0100):
        raise Exception("Timeout while waiting for device to disappear from USB")

    dev2 = wait_for_device(2, idVendor=0x1e4e, idProduct=0x0100)
    if not dev2:
        raise Exception("Timeout while waiting for device to restart.")

    #print((dev2.bus, dev2.address))
    if dev.bus == dev2.bus and dev.address == dev2.address:
        print("Reset didn't work. Adress has remained the same.")

    return dev2

def ping(dev):
    #print(usb.control.get_configuration(dev))

    send_vendor_read_request(dev, 121)

    if False:
        for i in range(256):
            if i == 123 or i == 124:
                continue
            try:
                x = send_vendor_read_request(dev, i)
                print((i, "ok", x))
            except usb.core.USBError as e:
                print((i, e.errno, str(e)))

def wait_for_device(timeout, expect_present=True, **kwargs):
    stepsize = min(timeout/10, 0.1)
    for _ in range(round(timeout/stepsize)):
        dev = usb.core.find(**kwargs)
        if expect_present and dev:
            return dev
        elif not expect_present and not dev:
            return True
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

        dev = wait_for_device(3, idVendor=0x1e4e, idProduct=0x0100)
        if not dev:
            raise Exception("Timeout while waiting for device to leave bootloader")

        # UVC doesn't work after DFU so we trigger a real reset, as well.
        reset_device(dev)
