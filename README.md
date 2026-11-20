# IoT-Based Smart Inventory Management System

**FRDM-MCXW71 + RC522 RFID Reader + BLE Wireless UART**

An automated inventory tracking and management system using the **FRDM-MCXW71** development board (Arm Cortex-M33, BLE 5.3), **RC522 RFID reader** (13.56 MHz, SPI), active **buzzer** audio feedback, and **Bluetooth LE Wireless UART** telemetry to a phone or laptop.

---

## 🔌 Hardware Connections

### RC522 RFID Reader → FRDM-MCXW71

| RC522 Pin | FRDM-MCXW71 Pin | Note |
|-----------|-----------------|------|
| **3.3V**  | 3V3             | Power |
| **GND**   | GND             | Ground |
| **SDA (CS)** | **PTA18** (GPIOA pin 18) | Software CS |
| **SCK**   | **PTB2** (LPSPI1_SCK)    | SPI Clock |
| **MOSI**  | **PTB3** (LPSPI1_SOUT)   | SPI MOSI |
| **MISO**  | **PTB1** (LPSPI1_SIN)    | SPI MISO |
| **RST**   | **PTC4** (GPIOC pin 4)   | Reset (active low) |
| **IRQ**   | Not connected   | Optional |

### Buzzer → FRDM-MCXW71

| Buzzer Pin | FRDM-MCXW71 Pin | Note |
|------------|-----------------|------|
| **+**      | **PTC5** via 100Ω resistor | GPIOC pin 5 |
| **−**      | GND             | Ground |

> ⚠️ **Use 3.3V only for RC522 power.** Never connect 5V signals to the MCU GPIO.

---

## 🏗️ Software Architecture

```
RFID Card (13.56 MHz)
       │ SPI (LPSPI1, 1 MHz, Mode 0)
       ▼
  RC522 Driver (source/rc522_drv.c)
       │ RC522_DetectCard()
       ▼
  Inventory Engine (source/rfid_inventory.c)
   • Add / Remove toggle on consecutive scans
   • 3 pre-registered products (Laptop, Keyboard, Mouse)
   • Unknown tag detection
   • Buzzer feedback (success: 1 beep, error: 3 beeps)
       │ BleApp_SendUartStream()
       ▼
  BLE Wireless UART Service (bluetooth/profiles/wireless_uart/)
       │ BLE 5.3 (MCXW71 built-in radio)
       ▼
  Phone / Laptop BLE client
  (NXP IoT Toolbox / nRF Connect / Web Bluetooth)
       │ MQTT (via Python bridge)
       ▼
  Node-RED Dashboard  →  Real-time stock gauges & table
```

---

## 📡 BLE Data Format

| Message | Meaning |
|---------|---------|
| `ADD:Laptop:11\n`         | Laptop added, stock now 11 |
| `REMOVE:Keyboard:19\n`   | Keyboard removed, stock 19 |
| `ALERT:UNKNOWN_TAG:A1B2C3D4\n` | Unregistered RFID tag |
| `STATUS:Laptop:11,Keyboard:19,Mouse:15\n` | Full inventory (button press) |

BLE device name: **`RFID_INVENTORY`**  
TX UUID: `01ff0100-ba5e-f4ee-5ca1-eb1e5e4b1ce0`

---

## 📦 Project Structure

```
miniprojectiot/
├── source/
│   ├── rfid_inventory.c   ← Main application (RFID + BLE + inventory logic)
│   ├── rfid_inventory.h
│   ├── rc522_drv.c        ← RC522 SPI driver
│   ├── rc522_drv.h
│   ├── gatt_db.h          ← BLE GATT database (device name: RFID_INVENTORY)
│   ├── wireless_uart.h    ← Compatibility shim → includes rfid_inventory.h
│   └── ...
├── board/
│   ├── pin_mux.c / pin_mux.h  ← Pin multiplexing
│   └── ...
├── dashboard/
│   ├── ble_mqtt_bridge.py ← Python: BLE → MQTT bridge
│   ├── node_red_flow.json ← Node-RED flow to import
│   └── DASHBOARD_SETUP.md ← Step-by-step dashboard setup
└── CMakeLists.txt
```

---

## 🔨 Building

### Prerequisites
- MCUXpresso for VS Code (NXP toolchain at `~/.mcuxpressotools/`)
- arm-gnu-toolchain-14.2, CMake 3.30, Ninja

### Build

```powershell
# Configure
cmake --preset Debug

# Build
cmake --build --preset Debug

# Output: Debug/frdmmcxw71_wireless_uart_bm_light_sensor.elf
```

Flash via VS Code debugger (MCUXpresso Debug launch config) or LinkServer CLI.

---

## 📱 Dashboard Setup

See [`dashboard/DASHBOARD_SETUP.md`](dashboard/DASHBOARD_SETUP.md) for full instructions.

**Quick start:**
```powershell
# 1. Start Mosquitto broker
mosquitto -v

# 2. Start Node-RED
node-red
# → open http://localhost:1880, import dashboard/node_red_flow.json

# 3. Start BLE bridge (after flashing firmware)
cd dashboard
python ble_mqtt_bridge.py
```

---

## 🃏 Registering Your RFID Cards

The default UIDs are placeholders. To use your own cards:

1. Flash firmware and connect serial console (115200 baud).
2. Scan your card — you will see `[RFID] UNKNOWN TAG: AABBCCDD`.
3. Update `Inventory_Init()` in `source/rfid_inventory.c`:
   ```c
   mInventory[0].uid[0] = 0xAA; mInventory[0].uid[1] = 0xBB;
   mInventory[0].uid[2] = 0xCC; mInventory[0].uid[3] = 0xDD;
   FLib_MemCpy(mInventory[0].productName, "Laptop", 7);
   ```
4. Rebuild and re-flash.

---

## 💡 Presentation Q&A

| Question | Answer |
|----------|--------|
| Why RFID over barcode? | No line-of-sight needed, faster bulk scanning |
| Why FRDM-MCXW71? | Arm Cortex-M33 with built-in BLE 5.3 — ideal for IoT |
| Why BLE and not WiFi? | MCXW71 has no WiFi. BLE is low-power, phone acts as gateway |
| Why RC522 over EM-18? | 3.3V compatible, faster SPI, industry-standard MIFARE |
| Why MQTT? | Lightweight pub/sub protocol designed for IoT |

---

*SPDX-License-Identifier: BSD-3-Clause*  
*Copyright 2024 NXP — Modified for RFID Inventory System*
