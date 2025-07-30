# -*- coding: utf-8 -*-
import smbus
import time
import paho.mqtt.client as mqtt
import json
import logging
import os
from datetime import datetime

# --- LOGGING SETUP ---
log_file_path = os.path.join(os.path.expanduser("~"), "ups_hat.log")
logging.basicConfig(
    filename=log_file_path,
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# --- REGISTER DEFINITIONS ---
_REG_CONFIG = 0x00          # Config Register (R/W)
_REG_SHUNTVOLTAGE = 0x01    # Shunt Voltage Register (R)
_REG_BUSVOLTAGE = 0x02      # Bus Voltage Register (R)
_REG_POWER = 0x03           # Power Register (R)
_REG_CURRENT = 0x04         # Current Register (R)
_REG_CALIBRATION = 0x05     # Calibration Register (R/W)

class BusVoltageRange:
    """Constants for the Bus Voltage Range setting."""
    RANGE_16V = 0x00  # Set bus voltage range to 16V
    RANGE_32V = 0x01  # Set bus voltage range to 32V (default)

class Gain:
    """Constants for the Shunt PGA Gain setting."""
    DIV_1_40MV = 0x00  # Shunt prog. gain set to 1, 40 mV range
    DIV_2_80MV = 0x01  # Shunt prog. gain set to /2, 80 mV range
    DIV_4_160MV = 0x02 # Shunt prog. gain set to /4, 160 mV range
    DIV_8_320MV = 0x03 # Shunt prog. gain set to /8, 320 mV range

class ADCResolution:
    """Constants for the Bus and Shunt ADC settings."""
    ADCRES_9BIT_1S = 0x00   # 9bit,   1 sample,   84us
    ADCRES_10BIT_1S = 0x01  # 10bit,  1 sample,   148us
    ADCRES_11BIT_1S = 0x02  # 11 bit, 1 sample,   276us
    ADCRES_12BIT_1S = 0x03  # 12 bit, 1 sample,   532us
    ADCRES_12BIT_2S = 0x09  # 12 bit, 2 samples,  1.06ms
    ADCRES_12BIT_4S = 0x0A  # 12 bit, 4 samples,  2.13ms
    ADCRES_12BIT_8S = 0x0B  # 12 bit, 8 samples,  4.26ms
    ADCRES_12BIT_16S = 0x0C # 12 bit, 16 samples, 8.51ms
    ADCRES_12BIT_32S = 0x0D # 12 bit, 32 samples, 17.02ms
    ADCRES_12BIT_64S = 0x0E # 12 bit, 64 samples, 34.05ms
    ADCRES_12BIT_128S = 0x0F# 12 bit, 128 samples, 68.10ms

class Mode:
    """Constants for the Operating Mode setting."""
    POWERDOWN = 0x00            # Power-Down
    SVOLT_TRIGGERED = 0x01      # Shunt Voltage, Triggered
    BVOLT_TRIGGERED = 0x02      # Bus Voltage, Triggered
    SANDBVOLT_TRIGGERED = 0x03  # Shunt and Bus, Triggered
    ADCOFF = 0x04               # ADC Off (disabled)
    SVOLT_CONTINUOUS = 0x05     # Shunt Voltage, Continuous
    BVOLT_CONTINUOUS = 0x06     # Bus Voltage, Continuous
    SANDBVOLT_CONTINUOUS = 0x07 # Shunt and Bus, Continuous (default)


class INA219:
    """
    Python driver for the INA219 current and power sensor.
    """
    def __init__(self, i2c_bus=1, addr=0x40):
        """
        Initializes the INA219 sensor.

        Args:
            i2c_bus (int): The I2C bus number (e.g., 1 for Raspberry Pi).
            addr (int): The I2C address of the INA219 sensor.
        """
        self.bus = smbus.SMBus(i2c_bus)
        self.addr = addr

        # Internal scaling factors and calibration value
        self._cal_value = 0
        self._current_lsb_mA = 0
        self._power_lsb_W = 0

        # Set chip to a known default configuration (32V, 2A)
        self.set_calibration_32V_2A()

    def _read_register(self, address):
        """Reads 2 bytes from a given I2C address."""
        data = self.bus.read_i2c_block_data(self.addr, address, 2)
        return (data[0] << 8) | data[1]

    def _write_register(self, address, data):
        """Writes 2 bytes to a given I2C address."""
        temp = [0, 0]
        temp[0] = (data & 0xFF00) >> 8
        temp[1] = data & 0xFF
        self.bus.write_i2c_block_data(self.addr, address, temp)

    def set_calibration_32V_2A(self):
        """
        Configures the INA219 to be able to measure up to 32V and 2A of current.
        With a 0.1 ohm shunt resistor, the counter will overflow at 3.2A.
        """
        current_lsb_A = 0.0001
        self._current_lsb_mA = current_lsb_A * 1000
        self._cal_value = 4096
        self._power_lsb_W = 0.002
        self._write_register(_REG_CALIBRATION, self._cal_value)
        bus_voltage_range = BusVoltageRange.RANGE_32V
        gain = Gain.DIV_8_320MV
        bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        mode = Mode.SANDBVOLT_CONTINUOUS
        config = (bus_voltage_range << 13 |
                  gain << 11 |
                  bus_adc_resolution << 7 |
                  shunt_adc_resolution << 3 |
                  mode)
        self._write_register(_REG_CONFIG, config)

    def get_shunt_voltage_mV(self):
        """Reads and returns the shunt voltage in millivolts (mV)."""
        value = self._read_register(_REG_SHUNTVOLTAGE)
        if value > 32767:
            value -= 65536
        return value * 0.01

    def get_bus_voltage_V(self):
        """Reads and returns the bus voltage in volts (V)."""
        raw_val = self._read_register(_REG_BUSVOLTAGE)
        shifted_val = raw_val >> 3
        return shifted_val * 0.004

    def get_current_mA(self):
        """Reads and returns the current in milliamps (mA)."""
        value = self._read_register(_REG_CURRENT)
        if value > 32767:
            value -= 65536
        return value * self._current_lsb_mA

    def get_power_W(self):
        """Reads and returns the power in watts (W)."""
        value = self._read_register(_REG_POWER)
        return value * self._power_lsb_W

if __name__ == '__main__':
    # --- MQTT Configuration ---
    MQTT_BROKER = "192.168.1.203" #ip of broker
    MQTT_PORT = 1883
    MQTT_USER = "username" #username
    MQTT_PASS = "pass" #password
    MQTT_CLIENT_ID = "UPS-HAT" # Static Client ID
    # Tasmota-like topic structure for Sonoff adapter compatibility
    MQTT_TELE_TOPIC = f"tele/{MQTT_CLIENT_ID}/SENSOR"
    MQTT_CMND_TOPIC = f"cmnd/{MQTT_CLIENT_ID}/safeshutdown"
    
    # --- Global flags ---
    connected = False
    # This global variable holds the state of our virtual switch as a proper boolean
    safe_shutdown_triggered = False 

    def on_connect(client, userdata, flags, reason_code, properties):
        """Callback-Funktion, die beim Verbinden mit dem Broker aufgerufen wird."""
        global connected
        logging.info("on_connect Callback wurde ausgeloest.")
        if reason_code == 0:
            logging.info(f"Erfolgreich mit MQTT Broker verbunden! (Client-ID: {MQTT_CLIENT_ID})")
            # Subscribe to the command topic upon successful connection
            client.subscribe(MQTT_CMND_TOPIC)
            logging.info(f"Subscribed auf Topic: {MQTT_CMND_TOPIC}")
            connected = True
        else:
            logging.error(f"Verbindung zum MQTT Broker fehlgeschlagen! Grund: {reason_code}")
            connected = False

    def on_disconnect(client, userdata, disconnect_flags, reason_code, properties):
        """Callback-Funktion, die bei einer Trennung vom Broker aufgerufen wird."""
        global connected
        connected = False
        logging.warning(f"Verbindung zum MQTT Broker getrennt! Grund: {reason_code}")
        if reason_code != 0:
            logging.info("Dies war eine unerwartete Trennung.")

    def on_publish(client, userdata, mid, reason_code, properties):
        """Callback-Funktion, die nach dem Senden einer Nachricht aufgerufen wird."""
        pass

    def on_message(client, userdata, msg):
        """Callback-Funktion, die bei einer eingehenden Nachricht aufgerufen wird."""
        global safe_shutdown_triggered, ina219
        logging.info(f"Nachricht empfangen auf Topic '{msg.topic}': {msg.payload.decode()}")
        if msg.topic == MQTT_CMND_TOPIC:
            payload = msg.payload.decode().upper()
            if payload in ["ON", "TRUE"]:
                logging.warning("Shutdown-Befehl empfangen! Sende finalen Status und fahre in 10s herunter...")
                safe_shutdown_triggered = True
                
                # Read sensor values one last time to include in the final message
                bus_voltage = round(ina219.get_bus_voltage_V(), 3)
                current_A = round(ina219.get_current_mA() / 1000, 3)
                power_W = round(ina219.get_power_W(), 3)
                percentage = (bus_voltage - 6)/2.4*100
                if(percentage > 100): percentage = 100
                if(percentage < 0): percentage = 0
                
                # Create and publish the final payload reflecting the shutdown state
                final_payload = {
                    "Time": datetime.utcnow().isoformat(),
                    "SafeShutdown": safe_shutdown_triggered, # This will be true
                    "ENERGY": {
                        "Voltage": bus_voltage,
                        "Current": current_A,
                        "Power": power_W,
                        "Battery": round(percentage, 1)
                    }
                }
                client.publish(MQTT_TELE_TOPIC, json.dumps(final_payload), retain=True)
                client.publish(f"tele/{MQTT_CLIENT_ID}/LWT", "Offline", retain=True)
                
                # Wait for 10 seconds before shutting down
                time.sleep(10)
                logging.warning("Fahre das System jetzt herunter.")
                os.system('sudo shutdown -h now')

    # Setup MQTT Client
    logging.info(f"Initialisiere MQTT Client mit ID: {MQTT_CLIENT_ID}")
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=MQTT_CLIENT_ID)
    client.username_pw_set(MQTT_USER, MQTT_PASS)
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_publish = on_publish
    client.on_message = on_message

    try:
        logging.info(f"Starte Verbindungsversuch zu {MQTT_BROKER}:{MQTT_PORT}...")
        client.connect(MQTT_BROKER, MQTT_PORT)
    except Exception as e:
        logging.critical(f"Verbindung konnte nicht initiiert werden: {e}")
        exit()
        
    logging.info("MQTT-Netzwerkschleife wird im Hintergrund gestartet.")
    client.loop_start()
    
    # Make sensor object accessible in on_message callback
    global ina219
    ina219 = INA219(addr=0x42)
    logging.info("INA219 Sensor Reader Initialized.")
    
    try:
        while True:
            if connected:
                # Read sensor values
                bus_voltage = round(ina219.get_bus_voltage_V(), 3)
                current_A = round(ina219.get_current_mA() / 1000, 3)
                power_W = round(ina219.get_power_W(), 3)
                
                # Calculate battery percentage as requested
                percentage = (bus_voltage - 6)/2.4*100
                if(percentage > 100): percentage = 100
                if(percentage < 0): percentage = 0
                
                # --- Create a Tasmota-like JSON payload ---
                tasmota_payload = {
                    "Time": datetime.utcnow().isoformat(),
                    "SafeShutdown": safe_shutdown_triggered, # This will be false
                    "ENERGY": {
                        "Voltage": bus_voltage,
                        "Current": current_A,
                        "Power": power_W,
                        "Battery": round(percentage, 1)
                    }
                }
                
                # Convert dictionary to JSON string
                payload_str = json.dumps(tasmota_payload)
                
                # Publish the single JSON payload
                result = client.publish(MQTT_TELE_TOPIC, payload_str)
                status = result[0]

                if status == 0:
                    logging.info(f"Sende Payload an Topic '{MQTT_TELE_TOPIC}': {payload_str}")
                else:
                    logging.warning(f"Senden an Topic '{MQTT_TELE_TOPIC}' fehlgeschlagen mit Code {status}")

            else:
                logging.info("Nicht verbunden, warte auf Verbindung...")
                time.sleep(1)

            # Wait for the next major cycle
            time.sleep(30)

    except IOError as e:
        logging.error(f"Verbindung zum INA219 Sensor fehlgeschlagen: {e}")
    except KeyboardInterrupt:
        logging.info("Programm durch Benutzer beendet.")
    except Exception as e:
        logging.critical(f"Ein unerwarteter Fehler ist aufgetreten: {e}", exc_info=True)
    finally:
        logging.info("Beende MQTT-Netzwerkschleife und trenne Verbindung.")
        client.loop_stop()
        client.disconnect()
        logging.info("MQTT-Verbindung sauber getrennt.")
