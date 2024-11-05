import asyncio
import struct
from bleak import BleakClient, BleakScanner
from typing import Optional

# Constants for the ESP32 BLE setup
ESP32_NAME = "ESP32-BLE" 
ESP32_SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"  
ESP32_CHARACTERISTIC_UUID = "12345678-1234-5678-1234-56789abcdef1"  

class BLETester:
    def __init__(self):
        self.client: Optional[BleakClient] = None
        self.connected = False

    async def connect_ble(self):
        try:
            print("Scanning for ESP32...")
            dev = await BleakScanner.find_device_by_filter(
                lambda d, ad: d.name and d.name.lower() == ESP32_NAME.lower()
            )

            if not dev:
                print("ESP32 not found!")
                return

            # Connect to the ESP32 device
            print(f"Connecting to {dev.name}...")
            self.client = BleakClient(dev)
            await self.client.connect()
            self.connected = True
            print(f"Connected to {dev.name}!")

            # Start notifications for the characteristic UUID
            await self.client.start_notify(
                ESP32_CHARACTERISTIC_UUID,
                self.ble_data_handler
            )

        except Exception as err:
            print(f"Connection error: {str(err)}")
            self.connected = False

    async def disconnect_ble(self):
        if self.client:
            await self.client.disconnect()
            self.connected = False
            print("Disconnected from ESP32")

    def ble_data_handler(self, characteristic, data):
        try:
            # Parse the data received (expecting two floats: x and y)
            x, y = struct.unpack('ff', data)
            print(f"Received Data - X: {x}, Y: {y}")
        except struct.error as err:
            print(f"Error parsing data: {str(err)}")

    async def run(self):
        await self.connect_ble()

        # Wait for notifications indefinitely (can be stopped manually)
        try:
            while self.connected:
                await asyncio.sleep(1)  # Just keep the connection alive for testing
        except KeyboardInterrupt:
            print("Test finished by user")

        await self.disconnect_ble()

if __name__ == '__main__':
    # Run the BLE tester
    tester = BLETester()
    asyncio.run(tester.run())
