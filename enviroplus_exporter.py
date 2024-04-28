#!/usr/bin/env python3
import datetime
import os
import random
import requests
import time, traceback
import logging
import argparse
import subprocess
import serial
from threading import Thread
import json
import uuid

import board
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS
from prometheus_client import start_http_server, Gauge, Histogram

from periphery import Serial

from bme280 import BME280
from enviroplus import gas
from enviroplus.noise import Noise
from pms5003 import PMS5003, ReadTimeoutError as pmsReadTimeoutError, SerialTimeoutError as pmsSerialTimeoutError

from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS

import colorsys
import ST7735
from PIL import Image, ImageDraw, ImageFont
from fonts.ttf import RobotoMedium as UserFont
from pms5003 import PMS5003
from adafruit_lc709203f import LC709203F, PackSize
import paho.mqtt.client as mqtt

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus

try:
    # Transitional fix for breaking change in LTR559
    from ltr559 import LTR559
    ltr559 = LTR559()
except ImportError:
    import ltr559

logging.basicConfig(
    format='%(asctime)s.%(msecs)03d %(levelname)-8s %(message)s',
    level=logging.INFO,
    handlers=[logging.FileHandler("enviroplus_exporter.log"),
              logging.StreamHandler()],
    datefmt='%Y-%m-%d %H:%M:%S')

logging.info("""enviroplus_exporter.py - Expose readings from the Enviro+ sensor by Pimoroni in Prometheus format

Press Ctrl+C to exit!

""")

DEBUG = os.getenv('DEBUG', 'false') == 'true'

bus = SMBus(1)
bme280 = BME280(i2c_dev=bus)
noise = Noise()

st7735 = ST7735.ST7735(
    port=0,
    cs=1,
    dc=9,
    backlight=12,
    rotation=270,
    spi_speed_hz=10000000
)

# Initialize display
st7735 = ST7735.ST7735(
    port=0,
    cs=1,
    dc=9,
    backlight=12,
    rotation=270,
    spi_speed_hz=10000000
)

# Initialize display
st7735.begin()

WIDTH = st7735.width
HEIGHT = st7735.height

# Set up canvas and font
img = Image.new('RGB', (WIDTH, HEIGHT), color=(0, 0, 0))
draw = ImageDraw.Draw(img)
font_size_small = 10
font_size_large = 20
font = ImageFont.truetype(UserFont, font_size_large)
smallfont = ImageFont.truetype(UserFont, font_size_small)
x_offset = 2
y_offset = 2

message = ""

# The position of the top bar
top_pos = 25


# Define your own warning limits
# The limits definition follows the order of the variables array
# Example limits explanation for temperature:
# [4,18,28,35] means
# [-273.15 .. 4] -> Dangerously Low
# (4 .. 18]      -> Low
# (18 .. 28]     -> Normal
# (28 .. 35]     -> High
# (35 .. MAX]    -> Dangerously High
# DISCLAIMER: The limits provided here are just examples and come
# with NO WARRANTY. The authors of this example code claim
# NO RESPONSIBILITY if reliance on the following values or this
# code in general leads to ANY DAMAGES or DEATH.
limits = [[4, 18, 28, 35],
          [250, 650, 1013.25, 1015],
          [20, 30, 60, 70],
          [-1, -1, 30000, 100000],
          [-1, -1, 40, 50],
          [-1, -1, 450, 550],
          [-1, -1, 200, 300],
          [-1, -1, 50, 100],
          [-1, -1, 50, 100],
          [-1, -1, 50, 100]]

# RGB palette for values on the combined screen
palette = [(0, 0, 255),           # Dangerously Low
           (0, 255, 255),         # Low
           (0, 255, 0),           # Normal
           (255, 255, 0),         # High
           (255, 0, 0)]           # Dangerously High

try:
    pms5003 = PMS5003()
except serial.serialutil.SerialException:
    logging.warning("Failed to initialise PMS5003.")

battery_sensor = False
try:
    sensor = LC709203F(board.I2C())
    battery_sensor = True
except ValueError:
    pass

TEMPERATURE = Gauge('temperature','Temperature measured (*C)')
PRESSURE = Gauge('pressure','Pressure measured (hPa)')
HUMIDITY = Gauge('humidity','Relative humidity measured (%)')
OXIDISING = Gauge('oxidising','Mostly nitrogen dioxide but could include NO and Hydrogen (Ohms)')
REDUCING = Gauge('reducing', 'Mostly carbon monoxide but could include H2S, Ammonia, Ethanol, Hydrogen, Methane, Propane, Iso-butane (Ohms)')
NH3 = Gauge('NH3', 'mostly Ammonia but could also include Hydrogen, Ethanol, Propane, Iso-butane (Ohms)') 
LUX = Gauge('lux', 'current ambient light level (lux)')
PROXIMITY = Gauge('proximity', 'proximity, with larger numbers being closer proximity and vice versa')
PM1 = Gauge('PM1', 'Particulate Matter of diameter less than 1 micron. Measured in micrograms per cubic metre (ug/m3)')
PM25 = Gauge('PM25', 'Particulate Matter of diameter less than 2.5 microns. Measured in micrograms per cubic metre (ug/m3)')
PM10 = Gauge('PM10', 'Particulate Matter of diameter less than 10 microns. Measured in micrograms per cubic metre (ug/m3)')
CPU_TEMPERATURE = Gauge('cpu_temperature','CPU temperature measured (*C)')
BATTERY_VOLTAGE = Gauge('battery_voltage','Voltage of the battery (Volts)')
BATTERY_PERCENTAGE = Gauge('battery_percentage','Percentage of the battery remaining (%)')

OXIDISING_HIST = Histogram('oxidising_measurements', 'Histogram of oxidising measurements', buckets=(0, 10000, 15000, 20000, 25000, 30000, 35000, 40000, 45000, 50000, 55000, 60000, 65000, 70000, 75000, 80000, 85000, 90000, 100000))
REDUCING_HIST = Histogram('reducing_measurements', 'Histogram of reducing measurements', buckets=(0, 100000, 200000, 300000, 400000, 500000, 600000, 700000, 800000, 900000, 1000000, 1100000, 1200000, 1300000, 1400000, 1500000))
NH3_HIST = Histogram('nh3_measurements', 'Histogram of nh3 measurements', buckets=(0, 10000, 110000, 210000, 310000, 410000, 510000, 610000, 710000, 810000, 910000, 1010000, 1110000, 1210000, 1310000, 1410000, 1510000, 1610000, 1710000, 1810000, 1910000, 2000000))

PM1_HIST = Histogram('pm1_measurements', 'Histogram of Particulate Matter of diameter less than 1 micron measurements', buckets=(0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100))
PM25_HIST = Histogram('pm25_measurements', 'Histogram of Particulate Matter of diameter less than 2.5 micron measurements', buckets=(0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100))
PM10_HIST = Histogram('pm10_measurements', 'Histogram of Particulate Matter of diameter less than 10 micron measurements', buckets=(0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100))

NOISE_PROFILE_MID_FREQ = Gauge('noise_profile_mid_freq', 'Noise profile of mid frequency noise (db)')
NOISE_PROFILE_LOW_FREQ = Gauge('noise_profile_low_freq', 'Noise profile of low frequency noise (db)')
NOISE_PROFILE_HIGH_FREQ = Gauge('noise_profile_high_freq', 'Noise profile of high frequency noise (db)')
NOISE_PROFILE_AMP = Gauge('noise_profile_amp', 'Noise profile of amplitude (db)')

# Setup InfluxDB
# You can generate an InfluxDB Token from the Tokens Tab in the InfluxDB Cloud UI
# INFLUXDB_URL = os.getenv('INFLUXDB_URL', '')
# INFLUXDB_TOKEN = os.getenv('INFLUXDB_TOKEN', '')
# INFLUXDB_ORG_ID = os.getenv('INFLUXDB_ORG_ID', '')
# INFLUXDB_BUCKET = os.getenv('INFLUXDB_BUCKET', '')
# INFLUXDB_SENSOR_LOCATION = os.getenv('INFLUXDB_SENSOR_LOCATION', 'Adelaide')
# INFLUXDB_TIME_BETWEEN_POSTS = int(os.getenv('INFLUXDB_TIME_BETWEEN_POSTS', '5'))
# influxdb_client = InfluxDBClient(url=INFLUXDB_URL, token=INFLUXDB_TOKEN, org=INFLUXDB_ORG_ID)
# influxdb_api = influxdb_client.write_api(write_options=SYNCHRONOUS)

# delay between each write to lcd
WRITE_TO_LCD_TIME = int(os.getenv('WRITE_TO_LCD_TIME', '4'))

# Sometimes the sensors can't be read. Resetting the i2c 
def reset_i2c():
    subprocess.run(['i2cdetect', '-y', '1'])
    time.sleep(2)

# Setup LC709203F battery monitor
if battery_sensor:
    if DEBUG:
        logging.info('## LC709203F battery monitor ##')
    try:
        if DEBUG:
            logging.info("Sensor IC version: {}".format(hex(sensor.ic_version)))
        # Set the battery pack size to 3000 mAh
        sensor.pack_size = PackSize.MAH3000
        sensor.init_RSOC()
        if DEBUG:
            logging.info("Battery size: {}".format(PackSize.string[sensor.pack_sizes]))
    except RuntimeError as exception:
        logging.error("Failed to read sensor with error: {}".format(exception))
        logging.info("Try setting the I2C clock speed to 10000Hz")

def get_cpu_temperature():
    """Get the temperature from the Raspberry Pi CPU"""
    with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
        temp = f.read()
        temp = int(temp) / 1000.0
        CPU_TEMPERATURE.set(temp)

def get_temperature(factor_usr):
    """Get temperature from the weather sensor"""
    # Tuning factor for compensation. Decrease this number to adjust the
    # temperature down, and increase to adjust up
    raw_temp = bme280.get_temperature()

    factor = 1.25



    if factor_usr:
        factor = factor_usr
        # factor = factor_usr
        cpu_temps = [get_cpu_temperature()] * 5
        cpu_temp = get_cpu_temperature()
        # Smooth out with some averaging to decrease jitter
        cpu_temps = cpu_temps[1:] + [cpu_temp]
        avg_cpu_temp = sum(cpu_temps) / float(len(cpu_temps))
        temperature = raw_temp - ((avg_cpu_temp - raw_temp) / factor)
    else:
        temperature = raw_temp

    
    # cpu_temps = [get_cpu_temperature()] * 5
    # cpu_temp = get_cpu_temperature()
    # # Smooth out with some averaging to decrease jitter
    # cpu_temps = cpu_temps[1:] + [cpu_temp]
    # avg_cpu_temp = sum(cpu_temps) / float(len(cpu_temps))
    # temperature = raw_temp - ((avg_cpu_temp - raw_temp) / factor)

    TEMPERATURE.set(temperature)   # Set to a given value

def get_pressure():
    """Get pressure from the weather sensor"""
    try:
        pressure = bme280.get_pressure()
        PRESSURE.set(pressure)
    except IOError:
        logging.error("Could not get pressure readings. Resetting i2c.")
        reset_i2c()

def get_humidity(humidity_compensation):
    """Get humidity from the weather sensor"""
    try:
        humidity = bme280.get_humidity()
        HUMIDITY.set(humidity)
    except IOError:
        logging.error("Could not get humidity readings. Resetting i2c.")
        reset_i2c()

def get_gas():
    """Get all gas readings"""
    try:
        readings = gas.read_all()
        OXIDISING.set(readings.oxidising)
        OXIDISING_HIST.observe(readings.oxidising)

        REDUCING.set(readings.reducing)
        REDUCING_HIST.observe(readings.reducing)

        NH3.set(readings.nh3)
        NH3_HIST.observe(readings.nh3)
    except IOError:
        logging.error("Could not get gas readings. Resetting i2c.")
        reset_i2c()

def get_noise_profile():
    """Get the noise profile"""
    try:
        low = noise.get_amplitude_at_frequency_range(20, 200)
        mid = noise.get_amplitude_at_frequency_range(200, 2000)
        high = noise.get_amplitude_at_frequency_range(2000, 8000)
        amp = noise.get_amplitude_at_frequency_range(20, 8000)
        NOISE_PROFILE_LOW_FREQ.set(low)
        NOISE_PROFILE_MID_FREQ.set(mid)
        NOISE_PROFILE_HIGH_FREQ.set(high)
        NOISE_PROFILE_AMP.set(amp)
    except IOError:
        logging.error("Could not get noise profile. Resetting i2c.")
        reset_i2c()

def get_light():
    """Get all light readings"""
    try:
       lux = ltr559.get_lux()
       prox = ltr559.get_proximity()

       LUX.set(lux)
       PROXIMITY.set(prox)
    except IOError:
        logging.error("Could not get lux and proximity readings. Resetting i2c.")
        reset_i2c()

def get_particulates():
    """Get the particulate matter readings"""
    try:
        pms_data = pms5003.read()
    except pmsReadTimeoutError:
        logging.warning("Timed out reading PMS5003.")
    except (IOError, pmsSerialTimeoutError):
        logging.warning("Could not get particulate matter readings.")
    else:
        PM1.set(pms_data.pm_ug_per_m3(1.0))
        PM25.set(pms_data.pm_ug_per_m3(2.5))
        PM10.set(pms_data.pm_ug_per_m3(10))

        PM1_HIST.observe(pms_data.pm_ug_per_m3(1.0))
        PM25_HIST.observe(pms_data.pm_ug_per_m3(2.5) - pms_data.pm_ug_per_m3(1.0))
        PM10_HIST.observe(pms_data.pm_ug_per_m3(10) - pms_data.pm_ug_per_m3(2.5))

def get_battery():
    """Get the battery voltage and percentage left"""
    try:
        voltage_reading = sensor.cell_voltage
        percentage_reading = sensor.cell_percent
        BATTERY_VOLTAGE.set(voltage_reading)
        BATTERY_PERCENTAGE.set(percentage_reading)
        if DEBUG:
            logging.info("Battery: {} Volts / {} %".format(sensor.cell_voltage, sensor.cell_percent))
    except (RuntimeError, OSError) as exception:
        logging.warning("Failed to read battery monitor with error: {}".format(exception))

def collect_all_data():
    """Collects all the data currently set"""
    sensor_data = {}
    sensor_data['temperature'] = TEMPERATURE.collect()[0].samples[0].value
    sensor_data['humidity'] = HUMIDITY.collect()[0].samples[0].value
    sensor_data['pressure'] = PRESSURE.collect()[0].samples[0].value
    sensor_data['oxidising'] = OXIDISING.collect()[0].samples[0].value
    sensor_data['reducing'] = REDUCING.collect()[0].samples[0].value
    sensor_data['nh3'] = NH3.collect()[0].samples[0].value
    sensor_data['lux'] = LUX.collect()[0].samples[0].value
    sensor_data['proximity'] = PROXIMITY.collect()[0].samples[0].value
    sensor_data['pm1'] = PM1.collect()[0].samples[0].value
    sensor_data['pm25'] = PM25.collect()[0].samples[0].value
    sensor_data['pm10'] = PM10.collect()[0].samples[0].value
    sensor_data['cpu_temperature'] = CPU_TEMPERATURE.collect()[0].samples[0].value
    sensor_data['battery_voltage'] = BATTERY_VOLTAGE.collect()[0].samples[0].value
    sensor_data['battery_percentage'] = BATTERY_PERCENTAGE.collect()[0].samples[0].value
    sensor_data['noise_profile_low_freq'] = NOISE_PROFILE_LOW_FREQ.collect()[0].samples[0].value
    sensor_data['noise_profile_mid_freq'] = NOISE_PROFILE_MID_FREQ.collect()[0].samples[0].value
    sensor_data['noise_profile_high_freq'] = NOISE_PROFILE_HIGH_FREQ.collect()[0].samples[0].value
    sensor_data['noise_profile_amp'] = NOISE_PROFILE_AMP.collect()[0].samples[0].value
    return sensor_data

#mqtt
MQTT_HOST = ""
MQTT_PORT = ""
MQTT_USER = ""
MQTT_PASS = ""
MQTT_TOPIC = ""
MQTT_POST_INTERVAL_MILLIS = 500.0
MQTT_CONNECTED = False
def connect_mqtt():
    global mqtt_client
    logging.info(f"Connecting to MQTT server {MQTT_HOST} on port {MQTT_PORT}")
    mqtt_client = mqtt.Client(client_id=f"enviroplus_{uuid.uuid4()}", clean_session=True, userdata=None, protocol=mqtt.MQTTv311, transport="tcp")
    mqtt_client.on_connect = on_connect
    mqtt_client.on_disconnect = on_disconnect
    mqtt_client.on_message = on_message
    mqtt_client.username_pw_set(MQTT_USER, MQTT_PASS)
    mqtt_client.will_set(f"{MQTT_TOPIC}/enviro_plus/availability", "offline", 1, True)
    mqtt_client.connect(MQTT_HOST, int(MQTT_PORT), 60)
    mqtt_client.loop_start() #start the loop

def on_message(client, userdata, msg):
    logging.info("Message received-> " + msg.topic + " " + str(msg.payload))

def on_connect(client, userdata, flags, rc):
    logging.info("Connected to MQTT with result code "+str(rc))
    logging.info("Subscribing to topic: " + MQTT_TOPIC + "/#")
    home_assistant_discovery()
    global MQTT_CONNECTED
    MQTT_CONNECTED = True

def on_disconnect(client, userdata, rc):
    logging.info("Disconnected from MQTT with result code "+str(rc))
    global MQTT_CONNECTED
    MQTT_CONNECTED = False

def post_data_mqtt():
    last_mqtt_post = time.time()
    global MQTT_CONNECTED
    while True:
        global mqtt_client
        if MQTT_CONNECTED and (time.time() - last_mqtt_post) > (MQTT_POST_INTERVAL_MILLIS / 1000.0):
            try:    
                sensor_data = collect_all_data()
                mqtt_client.publish(f"{MQTT_TOPIC}/enviro_plus_temperature", json.dumps({"temperature": sensor_data['temperature']}), qos=1, retain=True)
                mqtt_client.publish(f"{MQTT_TOPIC}/enviro_plus_humidity", json.dumps({"humidity": sensor_data['humidity']}), qos=1, retain=True)
                mqtt_client.publish(f"{MQTT_TOPIC}/enviro_plus_pressure", json.dumps({"pressure": sensor_data['pressure']}), qos=1, retain=True)
                mqtt_client.publish(f"{MQTT_TOPIC}/enviro_plus_oxidising", json.dumps({"oxidising": sensor_data['oxidising']}), qos=1, retain=True)
                mqtt_client.publish(f"{MQTT_TOPIC}/enviro_plus_reducing", json.dumps({"reducing": sensor_data['reducing']}), qos=1, retain=True)
                mqtt_client.publish(f"{MQTT_TOPIC}/enviro_plus_nh3", json.dumps({"nh3": sensor_data['nh3']}), qos=1, retain=True)
                mqtt_client.publish(f"{MQTT_TOPIC}/enviro_plus_lux", json.dumps({"lux": sensor_data['lux']}), qos=1, retain=True)
                mqtt_client.publish(f"{MQTT_TOPIC}/enviro_plus_proximity", json.dumps({"proximity": sensor_data['proximity']}), qos=1, retain=True)
                mqtt_client.publish(f"{MQTT_TOPIC}/enviro_plus_pm1", json.dumps({"pm1": sensor_data['pm1']}), qos=1, retain=True)
                mqtt_client.publish(f"{MQTT_TOPIC}/enviro_plus_pm25", json.dumps({"pm25": sensor_data['pm25']}), qos=1, retain=True)
                mqtt_client.publish(f"{MQTT_TOPIC}/enviro_plus_pm10", json.dumps({"pm10": sensor_data['pm10']}), qos=1, retain=True)
                mqtt_client.publish(f"{MQTT_TOPIC}/enviro_plus_noise_profile_low", json.dumps({"noise_profile_low": sensor_data['noise_profile_low_freq']}), qos=1, retain=True)
                mqtt_client.publish(f"{MQTT_TOPIC}/enviro_plus_noise_profile_mid", json.dumps({"noise_profile_mid": sensor_data['noise_profile_mid_freq']}), qos=1, retain=True)
                mqtt_client.publish(f"{MQTT_TOPIC}/enviro_plus_noise_profile_high", json.dumps({"noise_profile_high": sensor_data['noise_profile_high_freq']}), qos=1, retain=True)
                mqtt_client.publish(f"{MQTT_TOPIC}/enviro_plus_noise_profile_amp", json.dumps({"noise_profile_amp": sensor_data['noise_profile_amp']}), qos=1, retain=True)

            except (RuntimeError, OSError) as exception:
                logging.warning("Failed to post data to mqtt with error: {}".format(exception))
            last_mqtt_post = time.time()

def home_assistant_discovery():
    """Publish home assistant discovery messages"""
    temp_discovery_topic = f"homeassistant/sensor/enviro_plus_temperature/config"
    temp_configuration = {
        "name": "Enviro plus Temperature",
        "unique_id": "enviro_plus_temperature",
        "availability_topic": f"{MQTT_TOPIC}/enviro_plus/availability",
        "state_topic": f"{MQTT_TOPIC}/enviro_plus_temperature",
        "unit_of_measurement": "C",
        "value_template": "{{ value_json.temperature | round(1) }}",
        "unit_of_measurement": "C",
        "device_class": "temperature",
    }
    mqtt_client.publish(temp_discovery_topic, json.dumps(temp_configuration), retain=True)

    humidity_discovery_topic = f"homeassistant/sensor/enviro_plus_humidity/config"
    humidity_configuration = {
        "name": "Enviro plus Humidity",
        "unique_id": "enviro_plus_humidity",
        "availability_topic": f"{MQTT_TOPIC}/enviro_plus/availability",
        "state_topic": f"{MQTT_TOPIC}/enviro_plus_humidity",
        "unit_of_measurement": "%",   
        "value_template": "{{ value_json.humidity | round(1) }}",
        "device_class": "humidity",
    }
    mqtt_client.publish(humidity_discovery_topic, json.dumps(humidity_configuration), retain=True)

    pressure_discovery_topic = f"homeassistant/sensor/enviro_plus_pressure/config"
    pressure_configuration = {
        "name": "Enviro plus Pressure",
        "unique_id": "enviro_plus_pressure",
        "availability_topic": f"{MQTT_TOPIC}/enviro_plus/availability",
        "state_topic": f"{MQTT_TOPIC}/enviro_plus_pressure",
        "unit_of_measurement": "hPa",
        "value_template": "{{ value_json.pressure | round(1) }}",
        "device_class": "pressure",
    }
    mqtt_client.publish(pressure_discovery_topic, json.dumps(pressure_configuration), retain=True)

    pm1_discovery_topic = f"homeassistant/sensor/enviro_plus_pm1/config"
    pm1_configuration = {
        "name": "Enviro plus PM1",
        "unique_id": "enviro_plus_pm1",
        "availability_topic": f"{MQTT_TOPIC}/enviro_plus/availability",
        "state_topic": f"{MQTT_TOPIC}/enviro_plus_pm1",
        "unit_of_measurement": "ug/m3",
        "value_template": "{{ value_json.pm1 | round(1) }}",
        "device_class": "pm1",
    }
    mqtt_client.publish(pm1_discovery_topic, json.dumps(pm1_configuration), retain=True)

    pm25_discovery_topic = f"homeassistant/sensor/enviro_plus_pm25/config"
    pm25_configuration = {
        "name": "Enviro plus PM2.5",
        "unique_id": "enviro_plus_pm25",
        "availability_topic": f"{MQTT_TOPIC}/enviro_plus/availability",
        "state_topic": f"{MQTT_TOPIC}/enviro_plus_pm25",
        "unit_of_measurement": "ug/m3",
        "value_template": "{{ value_json.pm25 | round(1) }}",
        "device_class": "pm25",
    }
    mqtt_client.publish(pm25_discovery_topic, json.dumps(pm25_configuration), retain=True)

    pm10_discovery_topic = f"homeassistant/sensor/enviro_plus_pm10/config"
    pm10_configuration = {
        "name": "Enviro plus PM10",
        "unique_id": "enviro_plus_pm10",
        "availability_topic": f"{MQTT_TOPIC}/enviro_plus/availability",
        "state_topic": f"{MQTT_TOPIC}/enviro_plus_pm10",
        "unit_of_measurement": "ug/m3",
        "value_template": "{{ value_json.pm10 | round(1) }}",
        "device_class": "pm10",
    }
    mqtt_client.publish(pm10_discovery_topic, json.dumps(pm10_configuration), retain=True)

    oxidising_discovery_topic = f"homeassistant/sensor/enviro_plus_oxidising/config"
    oxidising_configuration = {
        "name": "Enviro plus Oxidising",
        "unique_id": "enviro_plus_oxidising",
        "availability_topic": f"{MQTT_TOPIC}/enviro_plus/availability",
        "state_topic": f"{MQTT_TOPIC}/enviro_plus_oxidising",
        "unit_of_measurement": "kO",
        "value_template": "{{ value_json.oxidising | round(1) }}",
    }

    mqtt_client.publish(oxidising_discovery_topic, json.dumps(oxidising_configuration), retain=True)

    reducing_discovery_topic = f"homeassistant/sensor/enviro_plus_reducing/config"
    reducing_configuration = {
        "name": "Enviro plus Reducing",
        "unique_id": "enviro_plus_reducing",
        "availability_topic": f"{MQTT_TOPIC}/enviro_plus/availability",
        "state_topic": f"{MQTT_TOPIC}/enviro_plus_reducing",
        "unit_of_measurement": "kO",
        "value_template": "{{ value_json.reducing | round(1) }}",
    }
    mqtt_client.publish(reducing_discovery_topic, json.dumps(reducing_configuration), retain=True)

    nh3_discovery_topic = f"homeassistant/sensor/enviro_plus_nh3/config"
    nh3_configuration = {
        "name": "Enviro plus NH3",
        "unique_id": "enviro_plus_nh3",
        "availability_topic": f"{MQTT_TOPIC}/enviro_plus/availability",
        "state_topic": f"{MQTT_TOPIC}/enviro_plus_nh3",
        "unit_of_measurement": "kO",
        "value_template": "{{ value_json.nh3 | round(1) }}",
    }
    mqtt_client.publish(nh3_discovery_topic, json.dumps(nh3_configuration), retain=True)

    lux_discovery_topic = f"homeassistant/sensor/enviro_plus_lux/config"
    lux_configuration = {
        "name": "Enviro plus Lux",
        "unique_id": "enviro_plus_lux",
        "availability_topic": f"{MQTT_TOPIC}/enviro_plus/availability",
        "state_topic": f"{MQTT_TOPIC}/enviro_plus_lux",
        "unit_of_measurement": "lx",
        "value_template": "{{ value_json.lux | round(1) }}",
        "device_class": "illuminance",
    }
    mqtt_client.publish(lux_discovery_topic, json.dumps(lux_configuration), retain=True)

    proximity_discovery_topic = f"homeassistant/sensor/enviro_plus_proximity/config"
    proximity_configuration = {
        "name": "Enviro plus Proximity",
        "unique_id": "enviro_plus_proximity",
        "availability_topic": f"{MQTT_TOPIC}/enviro_plus/availability",
        "state_topic": f"{MQTT_TOPIC}/enviro_plus_proximity",
        "unit_of_measurement": "cm",
        "value_template": "{{ value_json.proximity | round(1) }}",
        "device_class": "distance",
    }
    mqtt_client.publish(proximity_discovery_topic, json.dumps(proximity_configuration), retain=True)

    noise_profile_low_discovery_topic = f"homeassistant/sensor/enviro_plus_noise_profile_low/config"
    noise_profile_low_configuration = {
        "name": "Enviro plus Noise Profile Low",
        "unique_id": "enviro_plus_noise_profile_low",
        "availability_topic": f"{MQTT_TOPIC}/enviro_plus/availability",
        "state_topic": f"{MQTT_TOPIC}/enviro_plus_noise_profile_low",
        "unit_of_measurement": "db",
        "value_template": "{{ value_json.noise_profile_low | round(1) }}",
        "device_class": "sound_pressure",
    }
    mqtt_client.publish(noise_profile_low_discovery_topic, json.dumps(noise_profile_low_configuration), retain=True)

    noise_profile_mid_discovery_topic = f"homeassistant/sensor/enviro_plus_noise_profile_mid/config"
    noise_profile_mid_configuration = {
        "name": "Enviro plus Noise Profile Mid",
        "unique_id": "enviro_plus_noise_profile_mid",
        "availability_topic": f"{MQTT_TOPIC}/enviro_plus/availability",
        "state_topic": f"{MQTT_TOPIC}/enviro_plus_noise_profile_mid",
        "unit_of_measurement": "db",
        "value_template": "{{ value_json.noise_profile_mid | round(1) }}",
        "device_class": "sound_pressure",
    }
    mqtt_client.publish(noise_profile_mid_discovery_topic, json.dumps(noise_profile_mid_configuration), retain=True)

    noise_profile_high_discovery_topic = f"homeassistant/sensor/enviro_plus_noise_profile_high/config"
    noise_profile_high_configuration = {
        "name": "Enviro plus Noise Profile High",
        "unique_id": "enviro_plus_noise_profile_high",
        "availability_topic": f"{MQTT_TOPIC}/enviro_plus/availability",
        "state_topic": f"{MQTT_TOPIC}/enviro_plus_noise_profile_high",
        "unit_of_measurement": "db",
        "value_template": "{{ value_json.noise_profile_high | round(1) }}",
        "device_class": "sound_pressure",
    }
    mqtt_client.publish(noise_profile_high_discovery_topic, json.dumps(noise_profile_high_configuration), retain=True)

    noise_profile_amp_discovery_topic = f"homeassistant/sensor/enviro_plus_noise_profile_amp/config"
    noise_profile_amp_configuration = {
        "name": "Enviro plus Noise Profile Amp",
        "unique_id": "enviro_plus_noise_profile_amp",
        "availability_topic": f"{MQTT_TOPIC}/enviro_plus/availability",
        "state_topic": f"{MQTT_TOPIC}/enviro_plus_noise_profile_amp",
        "unit_of_measurement": "db",
        "value_template": "{{ value_json.noise_profile_amp | round(1) }}",
        "device_class": "sound_pressure",
    }
    mqtt_client.publish(noise_profile_amp_discovery_topic, json.dumps(noise_profile_amp_configuration), retain=True)

    mqtt_client.publish(f"{MQTT_TOPIC}/enviro_plus/availability", "online", 1, retain=True)

    

def write_to_lcd():
    """Write dta to eniro lcd"""
    while True:
        time.sleep(WRITE_TO_LCD_TIME)
        sensor_data = collect_all_data()

        try:
            variables = [
                "temperature",
                "pressure",
                "humidity",
            ]

            units = [
                    "C",
                    "hPa",
                    "%",
            ]
            for i in range(len(variables)):
                variable = variables[i]
                data_value = sensor_data[variable]
                unit = units[i]
                message = "{}: {:.1f} {}".format(variable[:1].capitalize(), data_value, unit)
                logging.debug('Writing to LCD: {}'.format(message))
                img = Image.new('RGB', (WIDTH, HEIGHT), color=(0, 0, 0))
                draw = ImageDraw.Draw(img)
                font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 30)
                size_x, size_y = draw.textsize(message, font)
                while size_x > WIDTH:
                    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", font.size - 2)
                    size_x, size_y = draw.textsize(message, font)
                draw.text((0, 0), message, font=font, fill=(255, 255, 255))
                st7735.display(img)
                time.sleep(WRITE_TO_LCD_TIME)
        except Exception as exception:
            logging.warning('Exception writing to LCD: {}'.format(exception))

# def post_to_influxdb():
#     """Post all sensor data to InfluxDB"""
#     name = 'enviroplus'
#     tag = ['location', 'adelaide']
#     while True:
#         time.sleep(INFLUXDB_TIME_BETWEEN_POSTS)
#         data_points = []
#         epoch_time_now = round(time.time())
#         sensor_data = collect_all_data()
#         for field_name in sensor_data:
#             data_points.append(Point('enviroplus').tag('location', INFLUXDB_SENSOR_LOCATION).field(field_name, sensor_data[field_name]))
#         try:
#             influxdb_api.write(bucket=INFLUXDB_BUCKET, record=data_points)
#             if DEBUG:
#                 logging.info('InfluxDB response: OK')
#         except Exception as exception:
#             logging.warning('Exception sending to InfluxDB: {}'.format(exception))

def str_to_bool(value):
    if value.lower() in {'false', 'f', '0', 'no', 'n'}:
        return False
    elif value.lower() in {'true', 't', '1', 'yes', 'y'}:
        return True
    raise ValueError('{} is not a valid boolean value'.format(value))

def every(delay, task):
  next_time = time.time() + delay
  while True:
    time.sleep(max(0, next_time - time.time()))
    try:
      task()
    except Exception:
      traceback.print_exc()
      # in production code you might want to have this instead of course:
      # logger.exception("Problem while executing repetitive task.")
    # skip tasks if we are behind schedule:
    next_time += (time.time() - next_time) // delay * delay + delay

def get_data():
    get_temperature(args.temp)
    get_cpu_temperature()
    get_humidity(args.humid)
    get_pressure()
    get_light()
    get_gas()
    get_noise_profile()
    get_gas()
    get_particulates()
    if DEBUG:
        logging.info('Sensor data: {}'.format(collect_all_data()))

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-b", "--bind", metavar='ADDRESS', default='0.0.0.0', help="Specify alternate bind address [default: 0.0.0.0]")
    parser.add_argument("-p", "--port", metavar='PORT', default=8000, type=int, help="Specify alternate port [default: 8000]")
    parser.add_argument("-e", "--enviro", metavar='ENVIRO', type=str_to_bool, default='false', help="Device is an Enviro (not Enviro+) so don't fetch data from particulate sensor as it doesn't exist [default: false]")
    parser.add_argument("-t", "--temp", metavar='TEMPERATURE', type=float, help="The temperature compensation value to get better temperature results when the Enviro+ pHAT is too close to the Raspberry Pi board")
    parser.add_argument("-u", "--humid", metavar='HUMIDITY', type=float, help="The humidity compensation value to get better humidity results when the Enviro+ pHAT is too close to the Raspberry Pi board")
    parser.add_argument("-d", "--debug", metavar='DEBUG', type=str_to_bool, help="Turns on more vebose logging, showing sensor output and post responses [default: false]")
    # parser.add_argument("-i", "--influxdb", metavar='INFLUXDB', type=str_to_bool, default='false', help="Post sensor data to InfluxDB Cloud [default: false]")
    parser.add_argument("-m", "--mqtt", metavar='MQTT', type=str, default=None, help="MQTT configuration localhost:1883:username:password:topic:interval [default: none]")
    parser.add_argument("-P", "--polling", metavar='POLLING', type=int, default=2, help="Polling interval in seconds, to fetch data from sensor [default: 2]")
    parser.add_argument("-L", "--lcd", metavar='LCD', type=str_to_bool, default='false', help="Display sensor data on LCD [default: false]")
    args = parser.parse_args()

    # Start up the server to expose the metrics.
    start_http_server(addr=args.bind, port=args.port)
    # Generate some requests.
    
    polling_interval = args.polling

    if args.debug:
        DEBUG = True

    if args.temp:
        logging.info("Using temperature compensation, reducing the output value by {}° to account for heat leakage from Raspberry Pi board".format(args.temp))

    if args.humid:
        logging.info("Using humidity compensation, increasing the output value by {}% to account for heat leakage from Raspberry Pi board".format(args.humid))

    # if args.influxdb:
    #     # Post to InfluxDB in another thread
    #     logging.info("Sensor data will be posted to InfluxDB every {} seconds".format(INFLUXDB_TIME_BETWEEN_POSTS))
    #     influx_thread = Thread(target=post_to_influxdb)
    #     influx_thread.start()

    if args.mqtt is not None:
        # Post to MQTT in another thread
        mqttdata = args.mqtt.split(':')
        MQTT_HOST = mqttdata[0]
        MQTT_PORT = mqttdata[1]
        MQTT_USER = mqttdata[2]
        MQTT_PASS = mqttdata[3]
        MQTT_TOPIC = mqttdata[4]
        MQTT_POST_INTERVAL_MILLIS = float(mqttdata[5])
        logging.info("Sensor data will be posted to MQTT every {} seconds to topic {}/+".format(MQTT_POST_INTERVAL_MILLIS, MQTT_TOPIC))
        
        connect_mqtt()
        mqtt_publisher__thread = Thread(target=post_data_mqtt)
        mqtt_publisher__thread.start()

    lcd_thread = Thread(target=write_to_lcd)
    lcd_thread.start()

    logging.info("Listening on http://{}:{}".format(args.bind, args.port))
    Thread(target=lambda: every(polling_interval, get_data)).start()