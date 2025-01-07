- Real-time BLE packet scanning
- Accelerometer data extraction
- Motion detection using variance analysis
- Timestamp-based logging
- Device address tracking

bash
# Compile the program
gcc -o ble_motion ble detection.c -lbluetooth -lm


## Running the Program

1. Ensure Bluetooth is enabled:
bash
sudo hciconfig hci0 up


2. Run the program:
bash
sudo ./ble detection



The program uses these default parameters (can be modified in code):
- `WINDOW_SIZE`: 10 samples for motion detection
- `MOVEMENT_THRESHOLD`: 0.5 (motion detection sensitivity)
- `MANUFACTURER_ID`: 0xE1 (target device manufacturer ID)

## Output Format

The program outputs:

[HH:MM:SS.mmm] Device: XX:XX:XX:XX:XX:XX
Accelerometer Data (x,y,z): X, Y, Z
Motion State: Moving/Stationary


2. Bluetooth Issues:
- Check adapter status: `hciconfig`
- Verify adapter recognition: `hcitool dev`
- Reset adapter if needed: `hciconfig hci0 reset`


The program expects accelerometer data packets in this format:

0x0201060303E1FF1216E1FFA10364FFXX00YYFF00ZZZZ

Where XX, YY, ZZ represent accelerometer values.
