    import sys
    import asyncio
    import struct
    from PyQt5.QtWidgets import QApplication, QMainWindow, QGraphicsScene, QGraphicsEllipseItem, QLineEdit, QPushButton, QVBoxLayout, QWidget, QGraphicsView, QLabel
    from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QObject
    from PyQt5.QtGui import QBrush, QPen
    from bleak import BleakClient, BleakScanner
    from typing import Optional
    # import bluetooth

    # async bridge between BLE and Qt
    class SignalHandler(QObject):
        data_received = pyqtSignal(float, float)
        connection_status = pyqtSignal(str)

    class MainWindow(QMainWindow):
        def __init__(self):
            super().__init__()

            # Window settings
            self.setWindowTitle("Any Screen Touch Screen Device")
            self.setGeometry(100, 100, 800, 600)

            # Setup UI Components
            self.setupUI()

            # # Setup Bluetooth connection
            # self.setupBluetooth()

            # Setup BLE config
            self.ESP32_NAME = "ESP32-BLE"
            self.ESP32_SERVICE_UUID = "180F"
            self.ESP32_CHARACTERISTIC_UUID = "2A19"
            self.client: Optional[BleakClient] = None
            self.connected = False

            # Setup sig handler
            self.signal_handler = SignalHandler()
            self.signal_handler.data_received.connect(self.displayTouchPoint)
            self.signal_handler.connection_status.connect(self.update_status)

            # Setup BLE connection
            self.setup_ble()

            # Placeholder for the last touch point
            self.lastTouchPoint = None

        def setupUI(self):
            # Main widget and layout
            main_widget = QWidget(self)
            self.setCentralWidget(main_widget)
            layout = QVBoxLayout(main_widget)

            # Touch display area
            self.graphicsView = QGraphicsView(self)
            self.touchScene = QGraphicsScene(self)
            self.graphicsView.setScene(self.touchScene)
            layout.addWidget(self.graphicsView)

            # Connect button for BLE 
            self.connectButton = QPushButton("Connect to ESP32", self)
            self.connectButton.clicked.connect(self.toggle_connect)
            layout.addWidget(self.connectButton)

            # Custom parameter input
            self.customParamInput = QLineEdit(self)
            self.customParamInput.setPlaceholderText("Enter custom parameters here")
            layout.addWidget(self.customParamInput)

            # Apply button for custom parameters
            applyButton = QPushButton("Apply", self)
            applyButton.clicked.connect(self.applyCustomParameters)
            layout.addWidget(applyButton)

            # Status label
            self.statusLabel = QLabel("", self)
            layout.addWidget(self.statusLabel)

        # def setupBluetooth(self):
        #     # Set up Bluetooth connection to ESP32 device
        #     self.bt_sock = None
        #     self.timer = QTimer(self)
        #     self.timer.timeout.connect(self.connectBluetooth)
        #     self.timer.start(1000)  # Try connecting every second until successful

        def setup_ble(self):
            self.ble_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.ble_loop)
            self.ble_timer = QTimer(self)
            self.ble_timer.timeout.connect(self.run_ble_loop)
            self.ble_timer.start(10) # run every 10 ms
        
        def run_ble_loop(self):
            pending = asyncio.all_tasks(self.ble_loop)
            if pending:
                self.ble_loop.run_until_complete(asyncio.gather(*pending))
        
        def toggle_connect(self):
            if not self.connected:
                asyncio.run_coroutine_threadsafe(self.connect_ble(), self.ble_loop)
            else:
                asyncio.run_coroutine_threadsafe(self.disconnect_ble(), self.ble_loop)
        
        async def connect_ble(self):
            try:
                self.signal_handler.connection_status.emit("Scanning for ESP32...")
                dev = await BleakScanner.find_device_by_filter(
                    lambda d, ad: d.name and d.name.lower() == self.ESP32_NAME.lower()
                )

                if not dev:
                    self.signal_handler.connection_status.emit("ESP32 not found!")
                    return
                
                # connect to dev
                self.signal_handler.connection_status.emit(f"Connecting to {dev.name}...")
                self.client = BleakClient(dev)
                await self.client.connect()
                self.connected = True

                await self.client.start_notify(
                    self.ESP32_CHARACTERISTIC_UUID,
                    self.ble_data_handler
                )

                self.signal_handler.connection_status.emit(f"Connected to {dev.name}!")
                self.connectButton.setText("Disconnect")
            
            except Exception as err:
                self.signal_handler.connection_status.emit(f"Connection error {str(err)}")
                self.connected = False

        async def disconnect_ble(self):
            if self.client:
                await self.client.disconnect()
                self.connected = False
                self.signal_handler.connection_status.emit("Disconnected")
                self.connectButton.setText("Connect to Device")

        def ble_data_handler(self, characteristic, data):
            try:
                # parse 32-bit data
                x, y = struct.unpack('ff', data)
                self.signal_handler.data_received.emit(x, y)
            except struct.error as err:
                self.signal_handler.connection_status.emit(f"BLE data handler error {str(err)}")

        def displayTouchPoint(self, x, y):
            # Display touch point on the graphics view
            if self.lastTouchPoint:
                self.touchScene.removeItem(self.lastTouchPoint)
            self.lastTouchPoint = self.touchScene.addEllipse(x - 5, y - 5, 10, 10, QPen(), QBrush(Qt.red))
        
        # status handler
        def update_status(self, message):
            self.statusLabel.setText(message)

        def applyCustomParameters(self):
            # Retrieve and display custom parameters entered by the user
            customParams = self.customParamInput.text()
            self.statusLabel.setText(f"Custom Parameters Applied: {customParams}")
        
        # connection tear down
        def closeEvent(self, event):
            asyncio.run_coroutine_threadsafe(self.disconnect_ble(), self.ble_loop)
            self.ble_timer.stop()
            self.ble_loop.stop()
            event.accept()

    if __name__ == '__main__':
        # Run the application
        app = QApplication(sys.argv)
        window = MainWindow()
        window.show()
        sys.exit(app.exec_())

        # def connectBluetooth(self):
        #     # Find and connect to Bluetooth device
        #     target_name = "ESP32_S3"  # Replace with your device's Bluetooth name
        #     target_address = None
        #     nearby_devices = bluetooth.discover_devices(duration=5, lookup_names=True)

        #     for addr, name in nearby_devices:
        #         if name == target_name:
        #             target_address = addr
        #             break

        #     if target_address:
        #         self.statusLabel.setText(f"Connecting to {target_name} at {target_address}...")
        #         try:
        #             self.bt_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        #             self.bt_sock.connect((target_address, 1))
        #             self.statusLabel.setText("Connected to Bluetooth device")
        #             self.timer.stop()
        #             # Start listening for data
        #             self.timer = QTimer(self)
        #             self.timer.timeout.connect(self.readBluetoothData)
        #             self.timer.start(100)
        #         except bluetooth.BluetoothError as e:
        #             self.statusLabel.setText(f"Failed to connect: {str(e)}")
        #     else:
        #         self.statusLabel.setText("Device not found. Retrying...")

        # def readBluetoothData(self):
        #     # Read incoming Bluetooth data for coordinates
        #     if self.bt_sock:
        #         try:
        #             data = self.bt_sock.recv(1024)
        #             if data:
        #                 x, y = int(data[:4]), int(data[4:8])  # Parse x, y from the data received
        #                 self.displayTouchPoint(x, y)
        #         except bluetooth.BluetoothError as e:
        #             self.statusLabel.setText(f"Error reading data: {str(e)}")