#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# Author: Krzysztof Markiewicz
# 2019, www.obbo.pl
# v.0.8.20190428
#
# This program is distributed under the terms of the GNU General Public License v3.0
#

import yaml
import time
import datetime
import logging
import logging.config
import sys
from threading import Thread, Timer
from os import path
from subprocess import Popen, PIPE
from operator import itemgetter
import smbus 
import shlex
import signal
import RPi.GPIO as GPIO
import schedule
import rrdtool
# https://pypi.org/project/paho-mqtt/
import paho.mqtt.client as mqtt


bus = smbus.SMBus(1)       # 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)

# https://fangpenlin.com/posts/2012/08/26/good-logging-practice-in-python/
# https://docs.python.org/3/howto/logging-cookbook.html
logger = logging.getLogger('pi-powerman')
logger_sched = logging.getLogger('schedule')
logging.config.dictConfig({
    'version': 1,
    'disable_existing_loggers': False,
    'formatters': {
        'console': {
            'format': '%(asctime)s pi-powerman.py[%(process)d]: [%(levelname)s]%(name)s: %(message)s'
        },
        'syslog': {
            'format': 'pi-powerman.py[%(process)d]: [%(levelname)s]%(name)s: %(message)s'
        },
    },
    'handlers': {
        'console': {
            'class':'logging.StreamHandler',
            'level':'DEBUG',
            'formatter': 'console',
            'stream': 'ext://sys.stdout'
        },
        'syslog': {
            'class': 'logging.handlers.SysLogHandler',
            'level': 'DEBUG',
            'formatter': 'syslog',
            'address': '/dev/log',
        },
    },
    'loggers': {
        'pi-powerman': {
            'handlers': ['console', 'syslog'],
            'level': 'INFO',
            'propagate': True
        },
        'schedule': {
            'handlers': ['console', 'syslog'],
            'level': 'WARNING',
            'propagate': True
        }
   }
})

def main():
    logger.info('Pi-Powerman daemon starting...')
    #https://elinux.org/RPi_GPIO_Code_Samples#RPi.GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    GPIO.setup(13, GPIO.OUT)     # RasPi daemon state, physical pin 13
    GPIO.output(13, GPIO.HIGH)   # A high level means that the daemon is running
    GPIO.setup(11, GPIO.IN)      # Listening PiPower request, physical pin 11
    
    daemon_killer = GracefulKillDaemon()
    # load configuration
    hardware = PiPowerHardware()
    result = hardware.load()
    if result:
        config = PiPowerConfig()
        result = config.load(hardware)
    if not(result):
        logger.critical('Braking daemon... Can\'t load configuration')
        exit(2)
    # set logging levels
    logger.setLevel(config.log['level'])
    if config.log['level'] == 'DEBUG':
        logger_sched.setLevel(config.log['level'])
    # handle I2C bus
    i2c = PiPowerI2C(hardware.i2c)
    # get hardware info
    logger.info(get_hardware_info(i2c, hardware.requests))
    # get the firmware configuration, check and update if necessary  
    hardware_setup = PiPowerSetup(hardware)
    hardware_setup.load(hardware.setup)
    if (hardware_setup.check(i2c)):
        logger.info('The hardware configuration is up-to-date')
    else:
        logger.info('Sending a new setup')
        hardware_setup.send(i2c)
        time.sleep(1)
    
    data = PiPowerData()
    export = PiPowerExport(hardware, config)
    export.create_rrd()        
    # run on boot task
    if 'on_boot' in config.jobs:
        if not(config.jobs['on_boot'] is None):
            run_job(config.jobs['on_boot'], i2c.buffer, hardware, 'job on boot')
    # add maintenance job
    if 'maintenance' in config.jobs:    
        if not(config.jobs['maintenance'] is None):
            add_job(config.jobs['maintenance'], 'maintenance', i2c.buffer, hardware)
    # add check jobs        
    if 'action' in config.buttons:
        if not(config.buttons['action'] is None):
            for i in config.buttons['action']:
                if not(config.buttons['action'][i] is None):
                    if 'check' in config.buttons['action'][i]:
                        if not(config.buttons['action'][i]['check'] is None):
                            add_job(config.buttons['action'][i]['check'], ''.join(['check ', i]), i2c.buffer, hardware)

    i2c_command = PiPowerCommand(i2c.buffer, hardware, ';')
    check_pipower_timer = PiPowerTimeout(config.common['check_pipower_period_sec'])
    check_pipower_timer.set_counter(0)
    keepalive_timer = PiPowerTimeout(config.common['keepalive_period_sec'])
    loop_delay = config.common['check_request_period_sec']
    # main loop
    while not(daemon_killer.kill_now):
        # run scheduled jobs
        schedule.run_pending()
        # get data from hardware module
        check_pipower_timer.update(loop_delay)
        if check_pipower_timer.check():
            check_pipower(i2c, 'schedule', data, hardware, config, export)
            check_pipower_timer.reset()
            keepalive_timer.reset()
        # check request from Raspberry
        if check_request():
            check_pipower(i2c, 'hardware request', data, hardware, config, export)     
            keepalive_timer.reset()
        # check buttons state
        if data.ready:
            config.buttons['history'] = check_button_change(i2c.buffer, data.buttons, hardware, config.buttons)
        # send keepalive
        keepalive_timer.update(loop_delay)
        if keepalive_timer.check():
            i2c_command.run('$send_keepalive')
            keepalive_timer.reset()
        # flush buffer
        if len(i2c.buffer) > 0:
            i2c.send_buffer()
        time.sleep(loop_delay)
    # detecting shutdown/reboot https://stackoverflow.com/questions/2832376/how-to-detect-pending-system-shutdown-on-linux
    logger.info('Signal {} detected, stoping Pi-Powerman daemon ...'.format(daemon_killer.kill_reason))
    if config.common['power_off_after_shutdown']:
        logger.debug('Power off after shutdown enabled')
        if sys.platform.startswith('linux'):
            proc = Popen(shlex.split('/bin/systemctl list-jobs'), stdout=PIPE, stderr=PIPE)
            stdout, stderr = proc.communicate()
            logger.debug('list-jobs: {}'.format(stdout.decode('utf-8')))
            systemctl_result = stdout.decode('utf-8').splitlines() 
            poweroff = False
            for i in systemctl_result:
                if i.find('shutdown.target') >= 0:
                    poweroff = True
                if i.find('reboot.target') >= 0:
                    poweroff = False
                    break
            if poweroff:
                logger.debug('Requesting PWR OFF')
                i2c_command.run('$press_button PWR OFF')
                i2c.send_buffer()
    logger.info('Bye!')
    # END
   
def get_hardware_info(i2c, requests):
    info = []
    info.extend(i2c.read_bus(requests['DAEMON_REQUEST_INFO1'])[1:])
    info.extend(i2c.read_bus(requests['DAEMON_REQUEST_INFO2'])[1:])
    result = ''
    for i in info:
        if (i >= 0x20) and (i < 0x80) :
            result = ''.join([result, chr(i)])
    return result
    
def add_job(job, job_name, buffer, hardware):
    if 'period' in job:
        period = safe_cast(job['period'], int, -1)
        if period > 0:
            schedule.every(period).seconds.do(run_job, job, buffer, hardware)
            logger.info('Schedule: add new job ({}) every {} sec'.format(job_name, period))
        else:
            logger.error('Set \"period\" ({}) for job \"{}\"'.format(period, job_name))
    else:
        logger.error('No \"period\" for shedule job \"{}\"'.format(job_name))     
        
def run_job(job, buffer, hardware, source='schedule'):
    logger.debug('Starting new job ({}): {}'.format(source, job))
    command_thread = PiPowerScript(buffer, hardware, job, source)
    command_thread.start()
        
def check_request():
    request = GPIO.input(11)
    logger.debug('Checking hardware request on pin 11, state ({})'.format(request))
    # low state on pin 11 => activate request
    return not(request)
       
def check_pipower(i2c, source, data, hardware, config, export):
    logger.debug('Checking I2C bus on: {}'.format(source))
    data.set(i2c.read_bus(hardware.requests['DAEMON_REQUEST_READ_STATUS']), config.factors)
    if data.ready:
        export.run(source, data)

def check_button_change(buffer, new, hardware, config):
    new_button_history = config['history']
    for button in hardware.buttons['ports']:
        last_state = config['history'][button]
        new_state = new[hardware.buttons['ports'][button]][0]
        if not(last_state == new_state):
            logger.debug('Button ({})state change: old state: {}, new state: {}'.format(button, last_state, new_state))
            if new_state < len(hardware.buttons['state']):
                new_state_name = hardware.buttons['state'][new_state]
            else:
                new_state_name = 'unknown'
            if button in config['action']:
                if not(config['action'][button] is None):
                    if new_state_name in config['action'][button]:
                        logger.debug('Invoking command set: ({}) for: ({})'.format(config['action'][button][new_state_name], button))
                        command_thread = PiPowerScript(buffer, hardware, config['action'][button][new_state_name], button)
                        command_thread.start()
            new_button_history[button] = new_state
    return new_button_history

def safe_cast(val, to_type, default=None):
    try:
        result = to_type(val)
    except (ValueError, TypeError):
        logger.warning('Conversion problem, value: "{}"; to type: {}'.format(val, to_type))
        result = default  
    return result


class PiPowerMQTT(object):
    def __init__(self, config):
        counter = 0    
        self.configured = False
        self.connected = False
        self.host = config['host']
        if len(self.host) > 0:
            counter += 1
        self.connection_type = config['connection_type']
        self.credential_file = config['credential_file']
        if self.connection_type == 0:
            self.transport = 'tcp'
            self.port = 1883
            self.tls = None
            counter += 1
        if self.connection_type == 1:
            self.transport = 'websockets'
            self.port = 80
            self.tls = None
            counter += 1
        if self.connection_type == 2:
            import ssl
            self.transport = 'websockets'
            self.port = 443
            self.tls = {'ca_certs':"/etc/ssl/certs/ca-certificates.crt", 'tls_version':ssl.PROTOCOL_TLSv1}
            counter += 1
        self.topic = ''
        self.channel_ID = None
        self.channel_write_API_key = None
        self.username = None
        self.API_key = None
        self.mqttc = None
        counter += self._load_credentials()
        if counter == 6:
            logger.info('MQTT protocol configured')
            self.topic = 'channels/{}/publish/{}'.format(self.channel_ID, self.channel_write_API_key)
            self.mqttc = mqtt.Client(transport=self.transport) 
            self.mqttc.enable_logger(logger=None) 
            if not(self.tls is None):
                self.mqttc.tls_set(ca_certs=self.tls['ca_certs'], tls_version=self.tls['tls_version'])
            self.mqttc.reconnect_delay_set(min_delay=20, max_delay=120)
            self.mqttc.username_pw_set(self.username, self.API_key) 
            self.mqttc.on_connect = self._on_connect
            self.mqttc.on_disconnect = self._on_disconnect
            self.configured = True
        else:
            logger.error('MQTT protocol not configured')
            
    def connect(self):
        if self.configured:
            self.mqttc.connect_async(self.host, port=self.port, keepalive=60, bind_address="")
            self.mqttc.loop_start()       
           
    def disconnect(self):
        if self.connected:
            self.mqttc.disable_logger()
            self.mqttc.loop_stop(force=False)    
            self.mqttc.disconnect()        
    
    def publish(self, values):
        if self.connected:
            payload = ''
            for i in range(0, len(values), 1):
                if not(values[i] is None):
                    payload += 'field{}={}&'.format(i + 1, values[i].strip())
            if payload.index('=') > 0:
                payload = payload[:-1]
                logger.debug('MQTT publish payload: {}'.format(payload))
                self.mqttc.publish(self.topic, payload)        
    
    def is_connected(self):
        return self.connected
    
    def _load_credentials(self):
        counter = 0
        if path.isfile(self.credential_file):
            credentials = open(self.credential_file, 'r')
            lines = credentials.read().splitlines()
            for i in lines:
                if i.startswith('channel_ID'):
                    self.channel_ID = self._extract_value(i)
                    if not(self.channel_ID is None):
                        counter += 1
                if i.startswith('channel_write_API_key'):
                    self.channel_write_API_key = self._extract_value(i)
                    if not(self.channel_write_API_key is None):
                        counter += 1
                if i.startswith('username'):
                    self.username = self._extract_value(i)
                    if not(self.username is None):
                        counter += 1
                if i.startswith('API_key'):
                    self.API_key = self._extract_value(i)
                    if not(self.API_key is None):
                        counter += 1
            credentials.close()
        else:
            logger.error('Can\'t open MQTT credential file') 
        return counter
        
    def _extract_value(self, line):
        result = None
        if line.index('=') > 0:
            result = line[line.index('=') + 1:]
            result = result.replace('\'', '')
            result = result.replace('"', '')
            result = result.strip()
        return result
    
    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.connected = True
            logger.info('MQTT client connected')
        else:
            self.connected = False
            logger.error('MQTT client connection error code {}'.format(rc))

    def _on_disconnect(self, client, userdata, rc):
        self.connected = False
        if rc == 0:
            logger.info('MQTT client disconnected')
        else:
            logger.error('MQTT client disconnection error code {}'.format(rc))

class PiPowerI2C(object):
    def __init__(self, hardware):
        self.buffer = []
        self.config = hardware
        
    def send_buffer(self, count=-1):
        logger.debug('Send buffer: "{}"'.format(self.buffer))
        i2c_ready = True
        while (i2c_ready) and (len(self.buffer) > 0) and ((count > 0) or (count == -1)):
            command = self.buffer[0][0]
            data = []
            data.extend(self.buffer[0])
            data.pop(0)
            # fulfill minimum buffer length
            while len(data) < 2:
                data.append(0x00)
            logger.debug('Sending buffer[0]: {} as command, {} as data'.format(hex(command), data))          
            try:
                bus.write_i2c_block_data(self.config['ADDRESS'], command, data)
            except IOError as e:
                logger.critical('Write I2C bus problem: "{}"'.format(e))    
                i2c_ready = False    
            if i2c_ready or self.config['DROP_SEND_BUFFER_ON_FAILED']:
                logger.debug('Remove from buffer current item')
                self.buffer.pop(0)
            if count > 0:
                count -= 1
        logger.debug('Leaving buffer: "{}"'.format(self.buffer))

    def read_bus(self, command):
        raw_data = [0,0]
        logger.debug('Reading I2C device at address ' + str(hex(self.config['ADDRESS'])))
        try:
            raw_data = bus.read_i2c_block_data(self.config['ADDRESS'], command, self.config['RECEIVE_BUFFER_SIZE'])
        except IOError as e:
            logger.critical('Read I2C bus problem: "{}"'.format(e)) 
            raw_data = [0,0]            
        return raw_data

class PiPowerExport(object):
    def __init__(self, hardware, config):
        self.hardware = hardware
        self.setup = config.export
        self.mqtt = config.mqtt
        self.active_rrd = []
        self.indent = '\t'
        self.temperature_unit = 'K'
        self.aliases = dict()
        self.aliases['outputs'] = config.outputs['aliases']
        self.aliases['buttons'] = config.buttons['aliases']
        self.temperature_unit_string = self._convert_temperature()
        self.csv_separator = ';'
        self.iot = None
        if 'iot' in self.setup['destination']:
            logger.info('MQTT start')
            self.iot = PiPowerMQTT(self.mqtt)
            self.iot.connect()

    def __del__(self):
        if 'iot' in self.setup['destination']:
            logger.info('MQTT stop')
            self.iot.disconnect()
        
    def run(self, source, data):
        if 'destination' in self.setup:
            if not(self.setup['destination'] is None):
                destination = self.setup['destination']
                if len(destination) > 0:
                    if ('show' in destination) or ('save' in destination) or ('log' in destination):
                        report = self._create_report(source, data)
                        if 'show' in destination:
                            self._print_report(report)
                        if 'save' in destination:
                            self._save_report(report) 
                        if 'log' in destination:
                            self._log_report(report) 
                    if 'rrd' in destination:
                        self._rrd_data_update(data)
                        if self.setup['rrd_graph_update']:
                            self._rrd_graph_update()
                    if 'csv' in destination:
                        self._csv_save(self._csv_update(data))      
                    if 'iot' in destination:
                        self._iot_publish(data)
        else:
            logger.error('Export destination is not defined')
            
    def create_rrd(self):
        folder = self.setup['rrd_folder']
        if path.isdir(folder):
            step = ['--step', '300']
            option = ['--no-overwrite']
            archives = ['RRA:AVERAGE:0.5:1:300', 'RRA:AVERAGE:0.5:1h:31d', 'RRA:AVERAGE:0.5:12h:1y']
            max_voltage = 100 # volt
            max_current = 10000 # milliampere
            max_power = 500 # watt
            min_temperature = -150 # kelvin, celsius, fahrenheit
            max_temperature = 400 # kelvin, celsius, fahrenheit
            self._create_rrd_database(folder, 'pipo_supply-voltage.rrd','DS:voltage:GAUGE:600:0:' + str(max_voltage), step, option, archives)
            self._create_rrd_database(folder, 'pipo_bus-voltage.rrd','DS:voltage:GAUGE:600:0:' + str(max_voltage), step, option, archives)
            self._create_rrd_database(folder, 'pipo_current.rrd','DS:current:GAUGE:600:0:' + str(max_current), step, option, archives)
            self._create_rrd_database(folder, 'pipo_power.rrd','DS:power:GAUGE:600:0:' + str(max_power), step, option, archives)
            if self.hardware.features['ups']:
                self._create_rrd_database(folder, 'pipo_battery-voltage.rrd','DS:voltage:GAUGE:600:0:' + str(max_voltage), step, option, archives)
                self._create_rrd_database(folder, 'pipo_battery-temperature.rrd','DS:temperature:GAUGE:600:' + str(min_temperature)+ ':' + str(max_temperature), step, option, archives)
        else:
            logger.error('Can\'t find folder: "{}"'.format(folder))

    def _create_rrd_database(self, folder, file_name, data_source, step, option, archives):
        full_name = path.join(folder, file_name)
        if path.isfile(full_name):
            self.active_rrd.append(file_name)
        else:
            try:
                rrdtool.create(full_name, step, option, data_source, archives)
            except rrdtool.OperationalError as e:
                logger.error('Creating RRD datafile error, value: ({})'.format(e))
            else:
                logger.info('New RRD datafile "{}" created'.format(full_name))
                self.active_rrd.append(file_name)

    def _rrd_data_update(self, data):
        folder = self.setup['rrd_folder']
        if path.isdir(folder):
            if 'pipo_supply-voltage.rrd' in self.active_rrd:
                rrdtool.update(path.join(folder, 'pipo_supply-voltage.rrd'), 'N:' + str(data.supply_voltage))
            if 'pipo_bus-voltage.rrd' in self.active_rrd:
                rrdtool.update(path.join(folder, 'pipo_bus-voltage.rrd'), 'N:' + str(data.bus_voltage))
            if 'pipo_current.rrd' in self.active_rrd:
                rrdtool.update(path.join(folder, 'pipo_current.rrd'), 'N:' + str(data.current))
            if 'pipo_power.rrd' in self.active_rrd:
                rrdtool.update(path.join(folder, 'pipo_power.rrd'), 'N:' + str(data.power))
            if self.hardware.features['ups']:
                if 'pipo_battery-voltage.rrd' in self.active_rrd:
                    rrdtool.update(path.join(folder, 'pipo_battery-voltage.rrd'), 'N:' + str(data.battery_voltage))
                if 'pipo_battery-temperature.rrd' in self.active_rrd:
                    rrdtool.update(path.join(folder, 'pipo_battery-temperature.rrd'), 'N:' + str(self._convert_temperature(data.battery_temperature)))

    def _rrd_graph_update(self):
        folder = self.setup['rrd_folder']
        if path.isdir(folder):
            width = '-w 640'
            mode = ['--slope-mode', '-l 0',]
            # daily
            battery_voltage = []
            if self.hardware.features['ups']:
                battery_voltage = ['DEF:battery=' + path.join(folder, 'pipo_battery-voltage.rrd') + ':voltage:AVERAGE',
                    'LINE3:battery#00ffff:Battery voltage(V)\t',
                    'GPRINT:battery:LAST:Cur\: %5.1lf(V)',
                    'GPRINT:battery:MAX:Max\: %5.1lf(V)',
                    'GPRINT:battery:MIN:Min\: %5.1lf(V)\t\t\t']
            ret = rrdtool.graph(path.join(folder, 'pipo_voltage_d.png'), '--start', '-1d', width, mode,
                'DEF:supply=' + path.join(folder, 'pipo_supply-voltage.rrd') + ':voltage:AVERAGE',
                'DEF:bus=' + path.join(folder, 'pipo_bus-voltage.rrd') + ':voltage:AVERAGE',
                'LINE1:supply#0000ff:Supply voltage(V)\t',
                'GPRINT:supply:LAST:Cur\: %5.1lf(V)',
                'GPRINT:supply:MAX:Max\: %5.1lf(V)',
                'GPRINT:supply:MIN:Min\: %5.1lf(V)\t\t\t',
                'LINE2:bus#00ff00:Bus voltage(V)\t',
                'GPRINT:bus:LAST:Cur\: %5.1lf(V)',
                'GPRINT:bus:MAX:Max\: %5.1lf(V)',
                'GPRINT:bus:MIN:Min\: %5.1lf(V)\t\t\t',
                battery_voltage)
                
            ret = rrdtool.graph(path.join(folder, 'pipo_current_d.png'), '--start', '-1d', width, mode, 
                'DEF:current=' + path.join(folder, 'pipo_current.rrd') + ':current:AVERAGE',
                'LINE1:current#0000ff:Current(mA)',
                'GPRINT:current:LAST:Cur\: %5.1lf(mA)',
                'GPRINT:current:MAX:Max\: %5.1lf(mA)',
                'GPRINT:current:MIN:Min\: %5.1lf(mA)\t\t\t')

            ret = rrdtool.graph(path.join(folder, 'pipo_power_d.png'), '--start', '-1d', width, mode,
                'DEF:power=' + path.join(folder, 'pipo_power.rrd') + ':power:AVERAGE',
                'LINE1:power#0000ff:Power(W)',
                'GPRINT:power:LAST:Cur\: %5.1lf(W)',
                'GPRINT:power:MAX:Max\: %5.1lf(W)',
                'GPRINT:power:MIN:Min\: %5.1lf(W)\t\t\t')

            if self.hardware.features['ups']:
                ret = rrdtool.graph(path.join(folder, 'pipo_temperature_d.png'), '--start', '-1d', width, mode,
                    'DEF:temperature=' + path.join(folder, 'pipo_battery-temperature.rrd') + ':temperature:AVERAGE', 
                    'LINE1:temperature#0000ff:Battery temperature(' + self.temperature_unit_string + ')',
                    'GPRINT:temperature:LAST:Cur\: %5.1lf(' + self.temperature_unit_string + ')',
                    'GPRINT:temperature:MAX:Max\: %5.1lf(' + self.temperature_unit_string + ')',
                    'GPRINT:temperature:MIN:Min\: %5.1lf(' + self.temperature_unit_string + ')\t\t\t')

            # monthly
            battery_voltage = []
            if self.hardware.features['ups']:
                battery_voltage = ['DEF:battery=' + path.join(folder, 'pipo_battery-voltage.rrd') + ':voltage:AVERAGE',
                    'LINE3:battery#00ffff:Battery voltage(V)\t',
                    'GPRINT:battery:LAST:Cur\: %5.1lf(V)',
                    'GPRINT:battery:MAX:Max\: %5.1lf(V)',
                    'GPRINT:battery:MIN:Min\: %5.1lf(V)\t\t\t']
            ret = rrdtool.graph(path.join(folder, 'pipo_voltage_m.png'), '--start', '-1m', width, mode,
                'DEF:supply=' + path.join(folder, 'pipo_supply-voltage.rrd') + ':voltage:AVERAGE',
                'DEF:bus=' + path.join(folder, 'pipo_bus-voltage.rrd') + ':voltage:AVERAGE',          
                'LINE1:supply#0000ff:Supply voltage(V)\t',
                'GPRINT:supply:LAST:Cur\: %5.1lf(V)',
                'GPRINT:supply:MAX:Max\: %5.1lf(V)',
                'GPRINT:supply:MIN:Min\: %5.1lf(V)\t\t\t',
                'LINE2:bus#00ff00:Bus voltage(V)\t',
                'GPRINT:bus:LAST:Cur\: %5.1lf(V)',
                'GPRINT:bus:MAX:Max\: %5.1lf(V)',
                'GPRINT:bus:MIN:Min\: %5.1lf(V)\t\t\t',
                battery_voltage)
                
            ret = rrdtool.graph(path.join(folder, 'pipo_current_m.png'), '--start', '-1m', width, mode,
                'DEF:current=' + path.join(folder, 'pipo_current.rrd') + ':current:AVERAGE',
                'LINE1:current#0000ff:Current(mA)',
                'GPRINT:current:LAST:Cur\: %5.1lf(mA)',
                'GPRINT:current:MAX:Max\: %5.1lf(mA)',
                'GPRINT:current:MIN:Min\: %5.1lf(mA)\t\t\t')

            ret = rrdtool.graph(path.join(folder, 'pipo_power_m.png'), '--start', '-1m', width, mode,
                'DEF:power=' + path.join(folder, 'pipo_power.rrd') + ':power:AVERAGE',
                'LINE1:power#0000ff:Power(W)',
                'GPRINT:power:LAST:Cur\: %5.1lf(W)',
                'GPRINT:power:MAX:Max\: %5.1lf(W)',
                'GPRINT:power:MIN:Min\: %5.1lf(W)\t\t\t')

            if self.hardware.features['ups']:
                ret = rrdtool.graph(path.join(folder, 'pipo_temperature_m.png'), '--start', '-1m', width, mode,
                    'DEF:temperature=' + path.join(folder, 'pipo_battery-temperature.rrd') + ':temperature:AVERAGE',
                    'LINE1:temperature#0000ff:Battery temperature(' + self.temperature_unit_string + ')',
                    'GPRINT:temperature:LAST:Cur\: %5.1lf(' + self.temperature_unit_string + ')',
                    'GPRINT:temperature:MAX:Max\: %5.1lf(' + self.temperature_unit_string + ')',
                    'GPRINT:temperature:MIN:Min\: %5.1lf(' + self.temperature_unit_string + ')\t\t\t')

    def _iot_publish(self, data):
        values = []
        for i in self.setup['iot_publish']:
            value = None
            if i == 'supply_voltage':
                value = data.supply_voltage
            if i == 'bus_voltage':
                value = data.bus_voltage
            if i == 'shunt_voltage':
                value = data.shunt_voltage
            if i == 'current':
                value = data.current
            if i == 'battery_voltage':
                value = data.battery_voltage
            if i == 'battery_temperature':
                value = self._convert_temperature(data.battery_temperature)
            if i == 'power':
                value = data.power
            if not(value is None):
                values.append('{:4.1f}'.format(value))
            else:
                values.append(None)
        self.iot.publish(values)
                    
    def _create_report(self, source, data):
        result = []
        date_time = '%I:%M:%S %p on %B %d, %Y'
        if not(self.setup['report_date_time_format'] is None):
            date_time = self.setup['report_date_time_format']
        result.append(''.join([datetime.datetime.now().strftime(date_time), '; requested by: ', source]))
        if 'report_raw_data' in self.setup:
            if self.setup['report_raw_data']:
                result.append(self._to_string_raw_data(data.raw))
        if 'report_buttons_state' in self.setup:
            if self.setup['report_buttons_state']:
                result.append(self._to_string_buttons_state(data.buttons, self.hardware.buttons))
        if 'report_pi_daemon_state' in self.setup:
            if self.setup['report_pi_daemon_state']:
                result.append(self._to_string_pi_daemon_state(data.pi_daemon_state, self.hardware.daemon))
        if 'report_supply_voltage' in self.setup:
            if self.setup['report_supply_voltage']:
                result.append(self._to_string_measure('Supply voltage', data.supply_voltage, 'V'))
        if 'report_bus_voltage' in self.setup:
            if self.setup['report_bus_voltage']:
                result.append(self._to_string_measure('Bus voltage', data.bus_voltage, 'V'))
        if 'report_shunt_voltage' in self.setup:
            if self.setup['report_shunt_voltage']:
                result.append(self._to_string_measure('Shunt voltage', data.shunt_voltage, 'mV'))
        if 'report_current' in self.setup:
            if self.setup['report_current']:
                result.append(self._to_string_measure('Current', data.current, 'mA'))
        if 'report_power' in self.setup:
            if self.setup['report_power']:
                result.append(self._to_string_measure('Power', data.power, 'W'))
        if 'report_outputs_state' in self.setup:
            if self.setup['report_outputs_state']:
                result.append(self._to_string_power_outputs_state(data.output_state, self.hardware.outputs))
        if self.hardware.features['ups']:
            if 'report_ups_state' in self.setup:
                if self.setup['report_ups_state']:
                    result.append(self._to_string_ups_state(data.ups_state, self.hardware.ups))
            if 'report_battery_voltage' in self.setup:
                if self.setup['report_battery_voltage']:
                    result.append(self._to_string_measure('Battery voltage', data.battery_voltage, 'V'))
            if 'report_battery_temperature' in self.setup:
                if self.setup['report_battery_temperature']:
                    result.append(self._to_string_measure('Battery temperature', self._convert_temperature(data.battery_temperature), self.temperature_unit_string))
        if 'report_errors' in self.setup:
            if self.setup['report_errors']:
                result.append(self._to_string_errors(data.errors, self.hardware.errors))
        result.append('')
        return result
                
    def _save_report(self, report):
        if 'save_folder' in self.setup:
            if not(self.setup['save_folder'] is None):
                if path.isdir(self.setup['save_folder']):
                    file_name = path.join(self.setup['save_folder'], datetime.datetime.now().strftime(self.setup['save_file']))
                    if not(path.isfile(file_name)) and not(self.setup['save_last']):
                        try:
                            with open(file_name, 'w') as file:
                                file.close()
                        except IOError as e:
                            logger.error('Save report with error, ({})'.format(e))
                            return
                    if self.setup['save_last']:
                        method = 'w'
                    else: 
                        method = 'a'
                    try:    
                        with open(file_name, method) as file:
                            for i in report:
                                file.write(i + '\n')
                            file.flush()
                            file.close()
                    except IOError as e:
                        logger.error('Save report with error, ({})'.format(e))
                else:
                    logger.error('Can\'t find folder: "{}"'.format(self.setup['save_folder']))
            else:
                logger.error('Save folder is empty')
        else:
            logger.error('Save folder is not defined')
            
    def _print_report(self, report):
        for i in report:
            print (i)
        
    def _log_report(self, report):
        for i in report:
            logger.info(i)

    def _convert_temperature(self, temperature_k=None):
        unit = self.setup['temperature_unit']
        temperature = 0
        if not(temperature_k is None):
            temperature = temperature_k
        if unit == 'C':
            temperature = temperature - 273.15
            unit = '°C'
        if unit == 'F':
            temperature = (temperature - 273.15) * 9 / 5  + 32
            unit = '°F'
        if temperature_k is None:
            return unit
        else:
            return temperature

    def _csv_update(self, data):
        result = []
        sep = self.csv_separator
        result.append(datetime.datetime.now().strftime('%Y-%m-%d') + sep)
        result.append(datetime.datetime.now().strftime('%H:%M:%S') + sep)
        result.append(self._to_csv_buttons_state(data.buttons, self.hardware.buttons) + sep)
        result.append(self._to_csv_measure(data.supply_voltage) + sep)
        result.append(self._to_csv_measure(data.bus_voltage) + sep)
        result.append(self._to_csv_measure(data.shunt_voltage) + sep)
        result.append(self._to_csv_measure(data.current) + sep)
        result.append(self._to_csv_measure(data.power) + sep)
        result.append(self._to_csv_power_outputs_state(data.output_state, self.hardware.outputs) + sep)
        if self.hardware.features['ups']:
            result.append(self._to_csv_ups_state(data.ups_state, self.hardware.ups) + sep)
            result.append(self._to_csv_measure(data.battery_voltage) + sep)
            result.append(self._to_csv_measure(self._convert_temperature(data.battery_temperature)) + sep)
        result.append(self._to_csv_errors(data.errors, self.hardware.errors) + sep)
        return ''.join(result)
        
    def _csv_save(self, report):
        if path.isdir(self.setup['csv_folder']):
            file_name = path.join(self.setup['csv_folder'], datetime.datetime.now().strftime(self.setup['csv_file']))
            if not(path.isfile(file_name)):
                try:
                    with open(file_name, 'w') as file:
                        file.write('Date;Time;Buttons;Supply voltage [V];Bus voltage [V];Shunt voltage [mV];Current [mA];Power [W];Outputs;')
                        if self.hardware.features['ups']:
                            file.write('UPS state;Battery voltage [V];Battery temperature [' + self.temperature_unit_string + '];')
                        file.write('Errors;\n')
                        file.close()
                except IOError as e:
                    logger.error('Save CSV with error, ({})'.format(e))
                    return
            try:    
                with open(file_name, 'a') as file:
                    file.write(report + '\n')
                    file.flush()
                    file.close()
            except IOError as e:
                logger.error('Save CSV with error, ({})'.format(e))
        else:
            logger.error('Can\'t find folder: "{}"'.format(self.setup['csv_folder']))

    def _to_string_raw_data(self, raw):
        result = [self.indent + 'Raw data: ']
        sep = ' '
        for i in raw:
            result.extend([self._hex_to_str(i), sep])
        return ''.join(result)[:-len(sep)] 
        
    def _to_string_ups_state(self, state, hardware):
        result = []
        if 'state' in hardware:
            label = hardware['state']
            result = [self.indent + 'UPS state: ']
            no_info = True
            sep = ', '
            for i in range(0, len(label), 1):
                if not(state & (1 << i)):
                    no_info = False
                    result.extend([label[i], sep])
            if no_info:
                result.extend(['NOT READY', len(sep) * '-'])
        else:
            result.extend(['No hardware state definition', len(sep) * '-'])
        return ''.join(result)[:-len(sep)]

    def _to_string_button_state(self, name, state, hardware_states, short_info):
        if name in self.aliases['buttons']:
            name = self.aliases['buttons'][name]
        if state[0] < len(hardware_states):
            state_string = hardware_states[state[0]]
        else:
            logger.error('Button ({}): unknown state'.format(name))
            state_string = 'unknown'            
        if short_info:
            return '({}):{}'.format(name, state_string)
        else:
            return '({}) state: {}'.format(name, state_string)
        
    def _to_string_buttons_state(self, data, hardware):
        result = []
        sep = '; '
        if 'state' in hardware:
            result = [self.indent + 'Buttons: ']
            hardware_ports_sorted = sorted(hardware['ports'].items(), key=itemgetter(1))
            for i in hardware_ports_sorted:                               
                result.append(self._to_string_button_state(i[0], data[hardware['ports'][i[0]]], hardware['state'], False) + sep)
        else:
            result.append('No hardware state definition' + len(sep) * '-')
        return ''.join(result)[:-len(sep)]
        
    def _to_string_pi_daemon_state(self, state, hardware):
        result = []
        if 'state' in hardware:
            if state < len(hardware['state']):
                label = hardware['state'][state]
            else:
                logger.error('Daemon state unknown')
                label = 'unknown'
            result.extend([self.indent, 'Pi deamon state: {}'.format(label)])
        else:
            result.append('No hardware state definition')
        return ''.join(result)
        
    def _to_string_power_output_state(self, name, state, hardware, short_info):
        label = hardware['state']
        if state[0] & (1 << hardware['ports'][name]):
            result = label[1]
        else:
            result = label[0]        
        if name in self.aliases['outputs']:
            name = self.aliases['outputs'][name]
        if short_info:
            return '({}):{}'.format(name, result)
        else:
            return '({}) state: {}'.format(name, result)
        
    def _to_string_power_outputs_state(self, state, hardware):
        result =[]
        sep = '; '
        if 'state' in hardware:        
            result = [self.indent + 'Outputs: ']
            hardware_ports_sorted = sorted(hardware['ports'].items(), key=itemgetter(1))
            for i in hardware_ports_sorted:                            
                result.extend([self._to_string_power_output_state(i[0], state, hardware, False), sep])
        else:
            result.extend(['No hardware state definition', len(sep) * '-'])
        return ''.join(result)[:-len(sep)]
        
    def _to_string_measure(self, name, value, unit):
        return self.indent + '{} = {:4.1f}{}'.format(name, value, unit)
      
    def _to_string_errors(self, state, hardware):
        sep = ', '
        label = hardware
        result = [self.indent + 'Errors: ']
        no_errors = True
        for i in range(0, len(label), 1):
            if state & (1 << i):
                no_errors = False
                result.extend([label[i], sep])
        if no_errors:
            result.extend(['No errors', len(sep) * '-'])
        return ''.join(result)[:-len(sep)]
        
    def _to_csv_measure(self, value):
        result = '{:.1f}'.format(value)
        if 'csv_decimal_mark' in self.setup:
            if not(self.setup['csv_decimal_mark'] == '.'):
                result = result.replace('.', self.setup['csv_decimal_mark'])
        return result
        
    def _to_csv_errors(self, state, hardware):
        sep = ','
        result = []
        label = hardware
        no_errors = True
        for i in range(0, len(label), 1):
            if state & (1 << i):
                no_errors = False
                result.extend([label[i], sep])
        if no_errors:
            result.extend(['No errors', len(sep) * '-'])
        return ''.join(result)[:-len(sep)]  
      
    def _to_csv_ups_state(self, state, hardware):
        sep = ','
        result = []
        label = hardware['state']
        no_info = True
        for i in range(0, len(label), 1):
            if not(state & (1 << i)):
                no_info = False
                result.extend([label[i], sep])
        if no_info:
            result.extend(['NOT READY', len(sep) * '-'])
        return ''.join(result)[:-len(sep)]
      
    def _to_csv_power_outputs_state(self, state, hardware):
        sep = ','
        result = []
        hardware_ports_sorted = sorted(hardware['ports'].items(), key=itemgetter(1))
        for i in hardware_ports_sorted:                            
            result.extend([self._to_string_power_output_state(i[0], state, hardware, True), sep])
        return ''.join(result)[:-len(sep)]

    def _to_csv_buttons_state(self, data, hardware):
        sep = ','
        result = []
        hardware_ports_sorted = sorted(hardware['ports'].items(), key=itemgetter(1))
        for i in hardware_ports_sorted:                               
            result.extend([self._to_string_button_state(i[0], data[hardware['ports'][i[0]]], hardware['state'], True), sep])
        return ''.join(result)[:-len(sep)]

    def _hex_to_str(self, h, length=2):
        s = ''.join([length * '0', str(hex(h))[2:]])
        return s[len(s) - length:]
        
class PiPowerScript (Thread):
    def __init__(self, buffer, hardware, script, source):
        Thread.__init__(self)
        self.buffer = buffer
        self.script = script
        self.source = source
        self.hardware = hardware
        self.separator = ';'
        
    def run(self):
        failed = False
        code = None
        command_list = []
        if 'commands' in self.script:
            i2c_command = PiPowerCommand(self.buffer, self.hardware, self.separator)
            if not(self.script['commands'] is None):
                command_all = ';'.join(self.script['commands'])
                command_list = command_all.split(self.separator)
                logger.debug('New thread starting... {} command(s) to do'.format(len(command_list)))
                while (len(command_list) > 0) and not(failed):
                    command = command_list[0].strip()
                    if command.find(i2c_command.INTERNAL_COMMAND) == 0:
                        logger.debug('Executing as internal task: {}'.format(command))
                        i2c_command.run(command)
                    else:
                        logger.debug('Executing as external script: {}'.format(command))
                        failed, code = self.run_external_script(command)
                    command_list.pop(0)
            else:
                logger.debug('In thread, nothing to do, skiping')
            if failed:
                logger.critical('For source ({}) script failed'.format(self.source))
                if 'on_failed' in self.script:
                    i2c_command.run(self.script['on_failed'])
                #i2c_command.run('$set_daemon_error')
            else:
                logger.debug('Command queue ({}) finished'.format(self.source))    
                if 'on_success' in self.script:
                    i2c_command.run(self.script['on_success'])
            if not(code is None):
                on_exit_code_command = ''.join(['exit(', str(code), ')'])
                if on_exit_code_command in self.script:
                    i2c_command.run(self.script[on_exit_code_command])   
        else:
            logger.debug('In thread, no commands definition, skiping')
        logger.debug('Thread finished')
            
    def run_external_script(self, external_command):
        failed = False  
        timeout = 60
        exit_code = -1
        if ('timeout' in self.script) and not(self.script['timeout'] is None):
            timeout = self.safe_cast(self.script['timeout'], int, timeout)
        file_name = external_command.split(' ')[0]
        logger.debug('Checking file: "{}"'.format(file_name))
        if path.isfile(file_name):
            logger.debug('In thread, executing: "{}" with timeout: "{}" sec'.format(external_command, timeout))
            try:
                proc = Popen(shlex.split(external_command), stdout=PIPE, stderr=PIPE)
                # Popen(..., cwd='path\to\somewhere')
                # Popen(..., shell=True)
            except OSError as e:
                failed = True
                logger.critical('({}); OSError ({}): {}'.format(external_command, e.errno, e.strerror))
            else:
                timer = Timer(timeout, proc.kill)
                try:
                    timer.start()
                    stdout, stderr = proc.communicate()
                finally:
                    timer.cancel()
                exit_code = proc.returncode
                logger.debug('Command "{}" return with code: "{}"'.format(external_command, exit_code))                 
        else:
            failed = True
            logger.error('Can\'t find file: "{}"'.format(file_name))
        if not(exit_code == 0):
            failed = True
        if exit_code < 0:
            logger.critical('Command "{}" was killed due to the timeout'.format(external_command))
        return failed, exit_code
      
    def safe_cast(self, val, to_type, default=None):
        try:
            result = to_type(val)
        except (ValueError, TypeError):
            logger.warning('Conversion problem, value: "{}"; to type: {}'.format(val, to_type))
            result = default  
        return result

class PiPowerCommand(object):
    def __init__(self, buffer, hardware, separator):
        self.buffer = buffer
        self.hardware = hardware
        self.i2c_request = hardware.requests
        self.INTERNAL_COMMAND = '$'   
        self.separator = separator
            
    def run(self, command):
        if len(self.buffer) < self.hardware.i2c['MAX_SEND_QUEUE']:
            logger.debug('Looking for internal command ({})'.format(command))
            command_list = []
            if command.find(self.separator) > 0:
                command_list = command.split(self.separator)
            else:
                command_list.append(command)
            for cmd in command_list:
                cmd = cmd.strip()
                if cmd.find(self.INTERNAL_COMMAND) == 0:
                    cmd = cmd[1:]
                    if cmd.find('wait ') == 0:
                        self.wait(cmd)
                    elif cmd.find('set_output ') == 0:
                        self.set_output(cmd)
                    elif cmd.find('set_button ') == 0:
                        self.set_button(cmd)
                    elif cmd == 'set_daemon_error':
                        self.set_daemon_error(cmd)
                    elif cmd == 'set_system_error':
                        self.set_system_error(cmd)
                    elif cmd == 'send_keepalive':
                        self.send_keepalive(cmd)
                    elif cmd.find('press_button ') == 0:
                        self.press_button(cmd)
                    # elif cmd == 'disable_battery_charge':
                        # self.disable_battery_charge(cmd)
                    # elif cmd == 'enable_battery_charge':
                        # self.enable_battery_charge(cmd)
                    # elif cmd == 'discharge_battery_start':
                        # self.discharge_battery_start(cmd)
                    # elif cmd == 'discharge_battery_stop':
                        # self.discharge_battery_stop(cmd)
                    elif cmd.find('clear_error ') == 0:
                        self.clear_error(cmd)
                    else:
                        logger.warning('Unknown internal command ({})'.format(cmd))
        else:
            logger.error('Command queue in buffer is too long')
        
    def wait(self, command):
        # $sleep <sec>
        params = command.split(' ')
        value = self.safe_cast(params[1], int, 0)
        logger.debug('Goes "wait ({})"'.format(value))
        time.sleep(value)
        
    def set_output(self, command):
        # $set_output <output> <value>
        params = command.split(' ')
        success = False
        if len(params) >= 3:
            if params[1] in self.hardware.outputs['ports']:
                port = 1 << self.hardware.outputs['ports'][params[1]]
                if params[2] in self.hardware.outputs['state']:
                    state = 0
                    state = self.hardware.outputs['state'].index(params[2]) << self.hardware.outputs['ports'][params[1]]
                    self.buffer.append([self.i2c_request['DAEMON_REQUEST_OUTPUT'], port, state]) 
                    success = True
                    logger.debug('Command: ({}) added to buffer'.format(command))
        if not(success):
            logger.warning('Command: ({}) can\'t be added to buffer'.format(command))
       
    def set_button(self, command):
        # $set_button <button> <new_state>
        # this command for PWR button will be ignored
        params = command.split(' ')
        success = False
        if len(params) >= 3:
            if params[1] in self.hardware.buttons['ports']:
                port = self.hardware.buttons['ports'][params[1]]
                if params[2] in self.hardware.buttons['state']:
                    new_state = self.hardware.buttons['state'].index(params[2])
                    self.buffer.append([self.i2c_request['DAEMON_REQUEST_BUTTON'], port, new_state])
                    success = True
                    logger.debug('Command: ({}) added to buffer'.format(command))
        if not(success):
            logger.warning('Command: ({}) can\'t be added to buffer'.format(command))
        
    def set_daemon_error(self, command):
        # $set_daemon_error
        self.buffer.append([self.i2c_request['DAEMON_REQUEST_ERROR']])
        logger.debug('Command: ({}) added to buffer'.format(command))
    
    def set_system_error(self, command):
        # $set_system_error
        self.buffer.append([self.i2c_request['DAEMON_REQUEST_SYSTEM_ERROR']])
        logger.debug('Command: ({}) added to buffer'.format(command))
 
    def send_keepalive(self, command):
        # $send_keepalive
        self.buffer.append([self.i2c_request['DAEMON_REQUEST_KEEPALIVE']])
        logger.debug('Command: ({}) added to buffer'.format(command))
 
    def press_button(self, command):
        # $press_button <button> [<new_state {ON, OFF} or default, toggle {PRESS}>]
        params = command.split(' ')
        new_state = 0
        success = False
        if len(params) >= 2:
            if params[1] in self.hardware.buttons['ports']:
                port = self.hardware.buttons['ports'][params[1]]
                if len(params) >= 3:
                    if params[2] in self.hardware.buttons['state']:
                        new_state = self.hardware.buttons['state'].index(params[2])
                self.buffer.append([self.i2c_request['DAEMON_REQUEST_PRESS'], port, new_state])
                success = True
                logger.debug('Command: ({}) added to buffer'.format(command))
        if not(success):
            logger.warning('Command: ({}) can\'t be added to buffer'.format(command))
        
    def disable_battery_charge(self, command):
        # $disable_battery_charge 
        self.buffer.append([self.i2c_request['DAEMON_REQUEST_DISABLE_CHARGE']])
        logger.debug('Command: ({}) added to buffer'.format(command))

    def enable_battery_charge(self, command):
        # $enable_battery_charge 
        self.buffer.append([self.i2c_request['DAEMON_REQUEST_ENABLE_CHARGE']])
        logger.debug('Command: ({}) added to buffer'.format(command))

    def discharge_battery_start(self, command):
        # $discharge_battery_start 
        self.buffer.append([self.i2c_request['DAEMON_REQUEST_DISCHARGE_START']])
        logger.debug('Command: ({}) added to buffer'.format(command))

    def discharge_battery_stop(self, command):
        # $discharge_battery_stop 
        self.buffer.append([self.i2c_request['DAEMON_REQUEST_DISCHARGE_STOP']])
        logger.debug('Command: ({}) added to buffer'.format(command))
                
    def clear_error(self, command):
        # $clear_error <error>
        params = command.split(' ')
        success = False
        if len(params) >= 2:
            if params[1] in self.hardware.errors:
                error = self.hardware.errors.index(params[1])
                self.buffer.append([self.i2c_request['DAEMON_REQUEST_CLEAR_ERROR'], error])
                success = True
                logger.debug('Command: ({}) added to buffer'.format(command))
        if not(success):
            logger.warning('Command: ({}) can\'t be added to buffer'.format(command))
        
    def safe_cast(self, val, to_type, default=None):
        try:
            result = to_type(val)
        except (ValueError, TypeError):
            logger.warning('Conversion problem, value: "{}"; to type: {}'.format(val, to_type))
            result = default  
        return result

class GracefulKillDaemon(object):
    # https://www.raspberrypi.org/forums/viewtopic.php?t=149939
    def __init__(self):
        self.kill_now = False
        self.kill_reason = []
        signal.signal(signal.SIGCONT, self.exit_gracefully) # 18 daemon restart
        signal.signal(signal.SIGINT, self.exit_gracefully)  #  2 Ctrl+C
        signal.signal(signal.SIGTERM, self.exit_gracefully) # 15 daemon restart, shutdown -r 
        signal.signal(signal.SIGUSR1, self.exit_gracefully) # 10
        signal.signal(signal.SIGUSR2, self.exit_gracefully) # 12

    def exit_gracefully(self, signum, frame):
        self.kill_now = True
        self.kill_reason.append(signum)

class PiPowerTimeout(object):
    def __init__(self, length):
        self.counter = length
        self.length = length
        self.pause = False

    def set(self, length):
        self.length = length
        
    def set_counter(self, value):
        if value > self.length:
            self.counter = self.length
        elif value < 0:
            self.counter = 0
        else:
            self.counter = value
        
    def reset(self):
        self.counter = self.length
        self.pause = False
        
    def update(self, step):
        if not(self.pause):
            if self.counter > step:
                self.counter -= step
            else:
                self.counter = 0
            
    def check(self):
        if self.counter == 0:
            return True
        else:
            return False
    
    def pause(self, state=True):
        self.pause = state

class PiPowerConfig(object):
    LOGGING_LEVELS = ('CRITICAL', 'ERROR', 'WARNING', 'INFO', 'DEBUG')
    def __init__(self):
        self.common = {
            'check_request_period_sec': 2,
            'check_pipower_period_sec': 300,
            'power_off_after_shutdown': False,
            'keepalive_period_sec': 120}
        self.export = {
            'destination': [],
            'report_date_time_format': '%H:%M:%S on %Y %B %d',
            'report_raw_data': True,
            'report_buttons_state': True,
            'report_pi_daemon_state': True,
            'report_supply_voltage': True,
            'report_bus_voltage': True,
            'report_shunt_voltage': True,
            'report_current': True,
            'report_power': True,
            'report_outputs_state': True,
            'report_ups_state': True,
            'report_battery_test': True,
            'report_battery_status': True,
            'report_battery_voltage': True,
            'report_battery_temperature': True,
            'report_errors': True,
            'save_folder': '',
            'save_file': 'pi-powerman-status_%Y-%m-%d.log',
            'save_last': False,
            'csv_folder': '',
            'csv_file': 'pi-powerman-status_%Y-%m-%d.csv',
            'csv_decimal_mark': ',',            
            'rrd_folder': '',
            'rrd_graph_update': False,
            'temperature_unit': 'K',
            'iot_protocol': 'mqtt',
            'iot_publish': []}
        self.mqtt = {
            'connection_type': 0,
            'host': '',
            'credential_file': ''}
        self.log = {
            'level': 'INFO'}
        self.factors = {
            'supply_voltage': 0.1,
            'battery_voltage': 0.1,
            'battery_temperature': 0.1,
            'bus_voltage': 0.1,
            'shunt_voltage': 0.1,
            'current': 0.1,
            'power': 0.1}
        self.outputs = {}
        self.buttons = {}
        self.jobs = {}
                    
    def load(self, hardware, config_file_name='pi-powerman-config.yaml'):
        result = True
        if not(path.isfile(config_file_name)) and (config_file_name.find('/') < 0):
            base_path = '/'.join(path.realpath(__file__).split('/')[:-1])
            config_file_name = path.join(base_path, config_file_name)
        if path.isfile(config_file_name):
            with open(config_file_name, 'r') as ymlfile:
                try:
                    cfg = yaml.load(ymlfile)
                except yaml.YAMLError as exc:
                    logger.error('When opennig {} YAML config file: {}'.format(config_file_name, exc))
                    result = False
                else:
                    temp = self.get_section('common', cfg, config_file_name)
                    if not(temp is None):
                        for i in temp:
                            self.common[i] = temp[i]
                    temp = self.get_section('export', cfg, config_file_name)
                    if not(temp is None):
                        for i in temp:
                            self.export[i] = temp[i]
                    if not(self.export['temperature_unit'] in ('K', 'C', 'F')):
                        self.export['temperature_unit'] = 'K'
                    self.log = self.get_section('log', cfg, config_file_name)
                    if not(self.log['level'] in self.LOGGING_LEVELS):
                        logger.error('Wrong log level, setting to default')
                        self.log = {'level': 'INFO'}
                    self.mqtt = self.get_section('mqtt', cfg, config_file_name)
                    temp = self.get_section('factors', cfg, config_file_name)
                    if not(temp is None):
                        for i in temp:
                            self.factors[i] = temp[i]
                    self.outputs = self.get_section('outputs', cfg, config_file_name)
                    self.buttons = self.get_section('buttons', cfg, config_file_name)
                    self.buttons['history'] = {}
                    for i in hardware.buttons['ports']:
                        self.buttons['history'][i] = 0
                    temp = self.get_section('jobs', cfg, config_file_name)
                    if not(temp is None):
                        for i in temp:
                            self.jobs[i] = temp[i]
                    logger.info('Config file ({}) loaded'.format(config_file_name))
        else:
            logger.error('Can\'t find file: "{}"'.format(config_file_name))
            result = False
        return result

    def get_section(self, section, cfg, file):
        if section in cfg:
            return cfg[section]
        else:
            logger.error('No section {} in file: {}'.format(section, file))
            return {}

class PiPowerHardware(object):
    def __init__(self):
        self.features = {}
        self.i2c = {}
        self.outputs = {}
        self.daemon = {}
        self.errors = {}
        self.ups = {}
        self.buttons = {}
        self.requests = {}
        self.setup = {}
                    
    def load(self, config_file_name='pi-powerman-hardware.yaml'):
        result = True
        if not(path.isfile(config_file_name)) and (config_file_name.find('/') < 0):
            base_path = '/'.join(path.realpath(__file__).split('/')[:-1])
            config_file_name = path.join(base_path, config_file_name)
        if path.isfile(config_file_name):
            with open(config_file_name, 'r') as ymlfile:
                try:
                    cfg = yaml.load(ymlfile)
                except yaml.YAMLError as exc:
                    logger.error('When opennig {} YAML config file: {}'.format(config_file_name, exc)) 
                    result = False
            self.features = self.get_section('features', cfg, config_file_name)
            self.i2c = self.get_section('i2c', cfg, config_file_name)
            self.outputs = self.get_section('outputs', cfg, config_file_name)
            self.daemon = self.get_section('daemon', cfg, config_file_name)
            self.errors = self.get_section('errors', cfg, config_file_name)
            self.ups = self.get_section('ups', cfg, config_file_name)
            self.buttons = self.get_section('buttons', cfg, config_file_name)
            self.requests = self.get_section('requests', cfg, config_file_name)
            self.setup = self.get_section('setup', cfg, config_file_name)
            logger.info('Hardware setup ({}) loaded'.format(config_file_name))
        else:
            logger.error('Can\'t find file: "{}"'.format(config_file_name))
            result = False
        return result
        
    def get_section(self, section, cfg, file):
        if section in cfg:
            return cfg[section]
        else:
            logger.error('No section {} in file: {}'.format(section, file))
            return {}

class PiPowerData(object):
    def __init__(self):
        self.raw = []
        self.buttons = []
        self.pi_daemon_state = 0
        self.supply_voltage = 0
        self.battery_voltage = 0
        self.battery_temp = 0
        self.output_state = []
        self.ups_state = 0
        self.bus_voltage = 0
        self.shunt_voltage = 0
        self.current = 0
        self.power = 0
        self.errors = 0
        self.requst = 0
        self.ready = False
            
    def set(self, data, factors):
        logger.debug('Extracting RAW data: "{}"'.format(data))
        self.ready = False
        if len(data) == 32:
            self.raw = []
            for i in data:
                self.raw.append(i)
            self.buttons = [[data[1], data[5]], [data[2], data[6]], [data[3], data[7]], [data[4], data[8]]]
            self.pi_daemon_state = data[9]
            self.supply_voltage = (data[11] * 0x100 + data[10]) * factors['supply_voltage']
            self.battery_voltage = (data[13] * 0x100 + data[12]) * factors['battery_voltage']
            self.battery_temperature = (data[15] * 0x100 + data[14]) * factors['battery_temperature']
            self.output_state = [data[16]]
            self.ups_state = data[17]
            self.bus_voltage = (data[19] * 0x100 + data[18]) * factors['bus_voltage']
            self.shunt_voltage = (data[21] * 0x100 + data[20]) * factors['shunt_voltage']
            self.current = (data[23] * 0x100 + data[22]) * factors['current']
            self.power = (data[25] * 0x100 + data[24]) * factors['power']
            self.errors = data[26]
            self.requst = data[27]
            self.ready = True

class PiPowerSetup(object):
    def __init__(self, hardware):
        self.setup = []
        self.hw_i2c = hardware.i2c
        self.hw_errors = hardware.errors
        self.hw_outputs = hardware.outputs
        self.hw_ups = hardware.ups
        self.hw_requests = hardware.requests
        
    def load(self, setup):
        self.setup.append(self.hw_requests['DAEMON_REQUEST_WRITE_SETUP'])
        self.setup.append(setup['calibration_vin'] & 0xff)
        self.setup.append(setup['calibration_vin'] >> 8)
        self.setup.append(setup['calibration_vbat'] & 0xff)
        self.setup.append(setup['calibration_vbat'] >> 8)
        self.setup.append(self._outputs_to_byte(setup['on_boot_output']))
        self.setup.append(setup['on_boot_delay_s'] & 0xff)
        self.setup.append(setup['on_boot_delay_s'] >> 8)
        self.setup.append(setup['daemon_timeout_s'] & 0xff)
        self.setup.append(setup['daemon_timeout_s'] >> 8)
        self.setup.append(self._errors_to_byte(setup['error_recoverable']))
        self.setup.append(self._ups_to_byte(setup['ups_behavior']))
        self.setup.append(setup['ups_shut_delay_s'] & 0xff)
        self.setup.append(setup['ups_shut_delay_s'] >> 8)
        self.setup.append(setup['ups_cut_level_mv'] & 0xff)
        self.setup.append(setup['ups_cut_level_mv'] >> 8)
        while len(self.setup) < 32:
            self.setup.append(0x00)
       
    def check(self, i2c):
        equal = False
        setup = []
        setup = i2c.read_bus(self.hw_requests['DAEMON_REQUEST_READ_SETUP'])
        if len(setup) >= len(self.setup):
            equal = True
            for i in range (1, len(self.setup), 1):
                if not(self.setup[i] == setup[i]):
                    equal = False
        return equal
    
    def send(self, i2c):
        i2c.buffer.extend([self.setup])
        i2c.send_buffer()

    def _errors_to_byte(self, list):
        result = 0
        error_list = list.split(',')
        for i in error_list:
            result += 1 << self.hw_errors.index(i.strip())
        return result
 
    def _outputs_to_byte(self, list):
        result = 0
        outputs_list = list.split(',')
        for i in outputs_list:
            result += 1 << self.hw_outputs['ports'][i.strip()]
        return result
       
    def _ups_to_byte(self, list):
        ups_list = list.split(',')
        result = self.hw_ups['behavior'].index(ups_list[0].strip())
        return result


        
main()


