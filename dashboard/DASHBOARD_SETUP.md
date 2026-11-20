# Dashboard Setup — RFID Smart Inventory

## 1. Install Mosquitto (MQTT Broker)

```powershell
winget install EclipseFoundation.Mosquitto
# After install, start the broker:
mosquitto -v
```

## 2. Install Node-RED (already done via npm)

```powershell
# Start Node-RED:
node-red

# Open browser: http://localhost:1880
# Import the flow: node_red_flow.json  (Menu → Import → Select file)
# Install dashboard: Manage Palette → Install → "node-red-dashboard"
# Open dashboard: http://localhost:1880/ui
```

## 3. Run the Python BLE Bridge

```powershell
# From the dashboard/ folder:
python ble_mqtt_bridge.py

# Optional flags:
python ble_mqtt_bridge.py --device-name RFID_INVENTORY --broker localhost --port 1883
```

## 4. Connect from Phone (alternative — no laptop needed)

Use **NXP IoT Toolbox** (Android/iOS):
1. Open → Wireless UART  
2. Scan → connect to **RFID_INVENTORY**  
3. See ADD/REMOVE/STATUS messages in real time

Or use **nRF Connect**:
1. Connect to **RFID_INVENTORY**  
2. Subscribe to characteristic UUID `01ff0100-ba5e-f4ee-5ca1-eb1e5e4b1ce0`

## 5. Data Flow

```
RFID Card → RC522 (SPI) → MCXW71 → BLE (Wireless UART) → Python Bridge → MQTT → Node-RED → Dashboard
```

## 6. MQTT Topics

| Topic | Content |
|-------|---------|
| `inventory/update` | Raw string: `ADD:Laptop:11` |
| `inventory/update/json` | JSON: `{"action":"ADD","product":"Laptop","quantity":11}` |
| `inventory/status/json` | JSON: `{"inventory":{"Laptop":11,"Keyboard":19}}` |
| `inventory/log` | Alerts and bridge status |
