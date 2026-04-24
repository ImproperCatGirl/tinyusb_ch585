#!/usr/bin/env python3
# Install python3 HID package: pip install hid
import hid
import time
import sys

# Define the Vendor IDs to look for
# Default is TinyUSB (0xcafe), Adafruit (0x239a), RaspberryPi (0x2e8a), Espressif (0x303a)
USB_VENDORS = (0xcafe, 0x239a, 0x2e8a, 0x303a)

# Configuration for the benchmark
BENCHMARK_DURATION_SECONDS = 10  # How long to run the benchmark
REPORT_SIZE_BYTES = 64           # Expected size of the HID report (from your device descriptor)
READ_TIMEOUT_MS = 100            # Timeout for each read operation in milliseconds

def find_and_open_device():
    """
    Scans for a HID device with the specified VIDs and opens the first one found.
    """
    print("Searching for HID device...")
    for vid in USB_VENDORS:
        for dev_info in hid.enumerate(vid):
            # Print device information for debugging
            print(f"  Found device: Vendor ID=0x{dev_info['vendor_id']:04x}, "
                  f"Product ID=0x{dev_info['product_id']:04x}, "
                  f"Usage Page=0x{dev_info['usage_page']:04x}, "
                  f"Usage=0x{dev_info['usage']:04x}, "
                  f"Path={dev_info['path']}")

            try:
                # Attempt to open the device
                dev = hid.Device(dev_info['vendor_id'], dev_info['product_id'])
                if dev:
                    print(f"\nSuccessfully opened device: "
                          f"'{dev.get_product_string()}' by '{dev.get_manufacturer_string()}'")
                    # Set the device to non-blocking mode for continuous reading
                    dev.set_nonblocking(1)
                    return dev
            except hid.HIDException as e:
                print(f"  Error opening device {dev_info['path']}: {e}")
    print("\nNo compatible HID device found. Make sure your device is connected and enumerated.")
    return None

def run_benchmark(dev):
    """
    Runs the HID report rate benchmark by continuously reading from the device.
    """
    print(f"\nStarting benchmark for {BENCHMARK_DURATION_SECONDS} seconds...")
    print("Ensure your device is configured to send reports continuously.")

    start_time = time.time()
    reports_received = 0

    try:
        while (time.time() - start_time) < BENCHMARK_DURATION_SECONDS:
            # Read a report with a timeout. If no report is available, it returns an empty bytearray
            # The timeout prevents the script from blocking indefinitely if the device stops sending.
            report = dev.read(REPORT_SIZE_BYTES, timeout_ms=READ_TIMEOUT_MS)

            if report:
                reports_received += 1
                # Optional: print received report data for verification
                # print(f"Received ({len(report)} bytes): {report}")
                
    except hid.HIDException as e:
        print(f"\nError during read operation: {e}")
    except KeyboardInterrupt:
        print("\nBenchmark interrupted by user.")
    finally:
        end_time = time.time()
        elapsed_time = end_time - start_time

        print("\n--- Benchmark Results ---")
        print(f"Total reports received: {reports_received}")
        print(f"Total elapsed time: {elapsed_time:.2f} seconds")

        if elapsed_time > 0:
            reports_per_second = reports_received / elapsed_time
            print(f"Average report rate: {reports_per_second:.2f} reports/second")
        else:
            print("Elapsed time is zero, cannot calculate report rate.")

        dev.close()
        print("Device closed.")

if __name__ == "__main__":
    device = find_and_open_device()
    if device:
        run_benchmark(device)

