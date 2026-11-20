#!/usr/bin/env python3
"""
BLE-to-MQTT Bridge for RFID Smart Inventory System
====================================================
Device  : FRDM-MCXW71 running rfid_inventory firmware
BLE name: RFID_INVENTORY
TX UUID : 01ff0100-ba5e-f4ee-5ca1-eb1e5e4b1ce0  (Wireless UART TX notifications)
RX UUID : 01ff0200-ba5e-f4ee-5ca1-eb1e5e4b1ce0  (Wireless UART RX write)

MQTT topics published:
  inventory/update  → ADD:Product:qty / REMOVE:Product:qty / ALERT:UNKNOWN_TAG:...
  inventory/status  → STATUS:Product1:qty,Product2:qty,...

Usage:
  python ble_mqtt_bridge.py [--device-name RFID_INVENTORY] [--broker localhost] [--port 1883]

Requirements:
  pip install bleak paho-mqtt

"""

import asyncio
import argparse
import json
import logging
import sys
from datetime import datetime

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    print("ERROR: bleak not installed. Run: pip install bleak")
    sys.exit(1)

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("ERROR: paho-mqtt not installed. Run: pip install paho-mqtt")
    sys.exit(1)

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
DEVICE_NAME      = "RFID_INVENTORY"
UART_TX_UUID     = "01ff0101-ba5e-f4ee-5ca1-eb1e5e4b1ce0"   # board → host (notify char_uart_stream)
UART_RX_UUID     = "01ff0101-ba5e-f4ee-5ca1-eb1e5e4b1ce0"   # host → board (write char_uart_stream)
MQTT_BROKER      = "localhost"
MQTT_PORT        = 1883
MQTT_TOPIC_DATA  = "inventory/update"
MQTT_TOPIC_STATUS= "inventory/status"
MQTT_TOPIC_LOG   = "inventory/log"

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# MQTT helpers
# ---------------------------------------------------------------------------
def make_mqtt_client(broker: str, port: int) -> mqtt.Client:
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="ble_bridge")

    def on_connect(c, _ud, _flags, rc, _prop):
        if rc == 0:
            log.info("MQTT connected to %s:%d", broker, port)
        else:
            log.error("MQTT connection failed: rc=%d", rc)

    def on_disconnect(c, _ud, _flags, rc, _prop):
        log.warning("MQTT disconnected: rc=%d", rc)

    client.on_connect    = on_connect
    client.on_disconnect = on_disconnect
    client.connect(broker, port, keepalive=60)
    client.loop_start()
    return client


def mqtt_publish(client: mqtt.Client, topic: str, payload: str):
    result = client.publish(topic, payload, qos=0, retain=False)
    if result.rc != mqtt.MQTT_ERR_SUCCESS:
        log.warning("MQTT publish failed on %s: %s", topic, result.rc)


# ---------------------------------------------------------------------------
# BLE notification callback
# ---------------------------------------------------------------------------
def make_notification_handler(mqtt_client: mqtt.Client):
    buffer = ""

    def handler(sender, data: bytearray):
        nonlocal buffer
        try:
            text = data.decode("utf-8", errors="replace")
        except Exception:
            return

        buffer += text
        # process complete newline-terminated messages
        while "\n" in buffer:
            line, buffer = buffer.split("\n", 1)
            line = line.strip()
            if not line:
                continue

            log.info("BLE RX: %s", line)
            ts = datetime.now().isoformat()

            if line.startswith("STATUS:"):
                mqtt_publish(mqtt_client, MQTT_TOPIC_STATUS, line)
                # also publish as structured JSON
                parts = line[7:].split(",")
                inventory = {}
                for p in parts:
                    if ":" in p:
                        name, qty = p.rsplit(":", 1)
                        try:
                            inventory[name.strip()] = int(qty.strip())
                        except ValueError:
                            pass
                mqtt_publish(
                    mqtt_client,
                    MQTT_TOPIC_STATUS + "/json",
                    json.dumps({"timestamp": ts, "inventory": inventory})
                )

            elif line.startswith("ADD:") or line.startswith("REMOVE:"):
                mqtt_publish(mqtt_client, MQTT_TOPIC_DATA, line)
                parts = line.split(":")
                if len(parts) >= 3:
                    action  = parts[0]
                    product = parts[1]
                    try:
                        qty = int(parts[2])
                    except ValueError:
                        qty = -1
                    mqtt_publish(
                        mqtt_client,
                        MQTT_TOPIC_DATA + "/json",
                        json.dumps({
                            "timestamp": ts,
                            "action": action,
                            "product": product,
                            "quantity": qty
                        })
                    )

            elif line.startswith("ALERT:"):
                mqtt_publish(mqtt_client, MQTT_TOPIC_LOG, line)
                log.warning("RFID ALERT: %s", line)

            else:
                # pass through any other messages
                mqtt_publish(mqtt_client, MQTT_TOPIC_LOG, line)

    return handler


# ---------------------------------------------------------------------------
# Main BLE loop
# ---------------------------------------------------------------------------
async def run(device_name: str, mqtt_client: mqtt.Client):
    log.info("Scanning for BLE device: '%s' ...", device_name)

    while True:
        device = await BleakScanner.find_device_by_name(device_name, timeout=10.0)
        if device is None:
            log.warning("Device '%s' not found. Retrying in 5 s ...", device_name)
            await asyncio.sleep(5)
            continue

        log.info("Found device: %s  [%s]", device.name, device.address)

        try:
            async with BleakClient(device) as client:
                log.info("BLE connected. Subscribing to TX notifications ...")
                handler = make_notification_handler(mqtt_client)
                await client.start_notify(UART_TX_UUID, handler)
                log.info("Bridge active. Press Ctrl-C to stop.")
                mqtt_publish(mqtt_client, MQTT_TOPIC_LOG,
                             f"BRIDGE_CONNECTED:{device.address}")

                # keep alive until disconnected
                while client.is_connected:
                    await asyncio.sleep(1)

                log.warning("BLE disconnected. Reconnecting ...")
                mqtt_publish(mqtt_client, MQTT_TOPIC_LOG, "BRIDGE_DISCONNECTED")

        except Exception as exc:
            log.error("BLE error: %s. Retrying in 5 s ...", exc)
            await asyncio.sleep(5)


def main():
    parser = argparse.ArgumentParser(description="BLE-to-MQTT bridge for RFID Inventory")
    parser.add_argument("--device-name", default=DEVICE_NAME,
                        help="BLE device name (default: %(default)s)")
    parser.add_argument("--broker",      default=MQTT_BROKER,
                        help="MQTT broker host (default: %(default)s)")
    parser.add_argument("--port",        default=MQTT_PORT, type=int,
                        help="MQTT broker port (default: %(default)s)")
    args = parser.parse_args()

    mqtt_client = make_mqtt_client(args.broker, args.port)
    try:
        asyncio.run(run(args.device_name, mqtt_client))
    except KeyboardInterrupt:
        log.info("Stopped by user.")
    finally:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()


if __name__ == "__main__":
    main()
