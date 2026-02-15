import time
import serial
import uuid

import numpy as np
import pandas as pd

import serial.tools.list_ports as lsp

from datetime import datetime
from tqdm import tqdm

from nadamq.NadaMq import cPacket, cPacketParser, PACKET_TYPES
from ..hardware_test import test_i2c
from base_node import BaseNode
from .. import metadata

def dump_capacitance(signal):
    tqdm.write(f'{datetime.now()}: Capacitance: {signal["new_value"]:.2e} - Voltage: {signal["V_a"]:.2f}')

def dump(signal):
    tqdm.write(f'{signal}' )

def dump_event(signal):
    event = signal.get('event', None)
    if event == 'output_disabled':
        tqdm.write('\033[91m' + f'{event}' + '\033[0m')
    elif event == 'output_enabled':
        tqdm.write('\033[92m' + f'{event}' + '\033[0m')
    else:
        tqdm.write(f'{signal}')

def print_event(signal):
    event = signal.get('event', None)
    if event == 'shorts-detected':
        tqdm.write('\033[91m' + f'{event}' + '\033[0m')
    elif event == 'halted':
        tqdm.write('\033[92m' + f'{event}' + '\033[0m')
    else:
        tqdm.write(f'{signal}')

port = None
for port in lsp.comports():
    # check if the hwid matches teensy 3.2
    if 'USB VID:PID=16C0:0483' in port.hwid:
        print('\033[92m' + 'Found Teensy 3.2' + '\033[0m')
        break
    else:
        port = None

if port:
    ID_REQUEST = cPacket(type_=PACKET_TYPES.ID_REQUEST).tobytes()
    ser = serial.Serial(port=port.device)
    if ser.is_open:
        ser.write(ID_REQUEST)
        time.sleep(0.1)
        bytes_to_read = ser.in_waiting
        if bytes_to_read > 0:
            print(f'Bytes to read:\033[95m{bytes_to_read}\033[0m')
            raw_packet = ser.read(bytes_to_read)
            packet = cPacketParser().parse(raw_packet)
            print('\033[94m', packet, '\033[0m', sep='')
            ser.close()

            import dropbot as db
            try:
                print('\033[92m' + 'Received Response, Attempting to Connect to Dropbot' + '\033[0m')
                proxy = db.SerialProxy(ignore=True)
            except Exception as e:
                print('\033[91m' + f'[Error] {e}' + '\033[0m')
                proxy = db.SerialProxy(port=port.device, ignore=True)
            try:
                title = '=' * 32 + '\033[93m' + ' Config ' + '\033[0m' + '=' * 32
                print(title)
                print(proxy.config)
                print('='*(len(title)-9))
                time.sleep(0.1)

                title = '=' * 28 + '\033[94m' + ' Control Board ' + '\033[0m' + '=' * 29
                print(title)
                print(f'uuid : {str(proxy.uuid)}')
                print(f'soft_i2c_scan : {proxy.soft_i2c_scan().tolist()}')
                print(f'number_of_channels : {proxy.number_of_channels}')
                print(f'temperature : {proxy.measure_temperature():.2f} C')
                print(f'analog reference : {proxy.measure_aref():.2f} V')
                print(f'voltage limit : {proxy.voltage_limit:.2f} V')
                print('='*(len(title)-9))
                time.sleep(0.1)

                # proxy.update_state(event_mask=proxy.state.event_mask | db.EVENT_CHANNELS_UPDATED | db.EVENT_ENABLE)
                # proxy.signals.signal('shorts-detected').connect(print_event)
                # proxy.signals.signal('halted').connect(print_event)
                
                title = '='*30 + '\033[91m' + ' Properties ' + '\033[0m' + '='*30
                print(title)
                print(proxy.properties)
                print('='*(len(title)-9))
                time.sleep(0.1)

                title = '='*32 + '\033[96m' + ' State ' + '\033[0m' + '='*33
                print(title)
                print(proxy.state)
                print('='*(len(title)-9))
                time.sleep(0.1)

                title = '='*31 + '\033[93m' + ' I2C Test ' + '\033[0m' + '='*31
                print(title)
                print('i2c_scan (Wire):', proxy.i2c_scan().tolist())
                print('soft_i2c_scan (digipot):', proxy.soft_i2c_scan().tolist())
                # result = test_i2c(proxy)["i2c_scan"]
                        # Inlined from test_i2c with prints to see where it hangs
                result = {}
                for address in proxy.i2c_scan():
                    if address == proxy.config.i2c_address:
                        continue
                    print(f'  address {address}...', flush=True)
                    if address in [32, 33, 34]:
                        print(f'    BaseNode({address}).name()', flush=True)
                        node = BaseNode(proxy, int(address))
                        name = node.name().split(b'\0', 1)[0].decode("utf-8")
                        print(f'    BaseNode({address}).hardware_version()', flush=True)
                        hardware_version = node.hardware_version().split(b'\0', 1)[0].decode("utf-8")
                        print(f'    BaseNode({address}).software_version()', flush=True)
                        software_version = node.software_version().split(b'\0', 1)[0].decode("utf-8")
                        print(f'    BaseNode({address}).uuid', flush=True)
                        node_uuid = str(node.uuid)
                        result[int(address)] = {'name': name, 'hardware_version': hardware_version,
                                               'software_version': software_version, 'uuid': node_uuid}
                    elif address in [80, 81]:
                        print(f'    i2c_eeprom_read({address}, 0, 1)', flush=True)
                        n_bytes = proxy.i2c_eeprom_read(address, 0, 1)
                        print(f'    i2c_eeprom_read({address}, 1, {n_bytes})', flush=True)
                        data = proxy.i2c_eeprom_read(address, 1, n_bytes)
                        board = metadata.Hardware.FromString(data.tobytes())
                        result[int(address)] = {'name': board.name, 'hardware_version': board.version,
                                                'uuid': str(uuid.UUID(bytes=board.uuid))}
                    else:
                        result[int(address)] = {}
                df_res = pd.DataFrame.from_dict(result, orient='index')
                df_res.index.name = 'i2c_address'
                print(df_res)
                print('='*(len(title)-9))
                time.sleep(0.1)

                proxy.signals.signal('output_disabled').connect(dump_event)
                proxy.signals.signal('output_enabled').connect(dump_event)
                
                print('='*(len(title)-9))
                for _ in tqdm(range(5), desc='Test Signals'):
                    time.sleep(2)
                print('=' * (len(title) - 9))

                print("Updating state...")
                proxy.update_state(
                    capacitance_update_interval_ms=500,
                    hv_output_selected=True,
                    hv_output_enabled=True,
                    voltage=80,
                    frequency=1000
                )

                title = '='*28 + '\033[96m' + ' Updated State ' + '\033[0m' + '='*29
                print(title)
                print(proxy.state)
                print('='*(len(title)-9))
                time.sleep(0.1)

                # Turn off all channels
                proxy.state_of_channels = np.zeros(proxy.number_of_channels, dtype='uint8')

                print("Testing capacitance measurement...")
                proxy.signals.signal('capacitance-updated').connect(dump_capacitance)

                for i in tqdm(range(6), desc='Measuring capacitance'):
                    if i == 2:
                        print("Updating voltage to 100V")
                        proxy.update_state(
                            capacitance_update_interval_ms=500,
                            hv_output_selected=True,
                            hv_output_enabled=True,
                            voltage=100,
                            frequency=10000
                        )
                        time.sleep(1)
                    if i > 1:
                        # After 1 iteration, we will turn on some channels
                        channels = np.random.choice([0, 1], size=proxy.number_of_channels,
                                                    p=[1-i/100, i/100]).astype('uint8')
                        proxy.state_of_channels = channels
                        tqdm.write('\033[92m' +
                                   f"Turned on {sum(channels==1)} channels {np.where(channels == 1)[0]}" +
                                   '\033[0m')
                    time.sleep(0.6) # wait for capacitance measurement (500ms update interval)

                proxy.state_of_channels = np.zeros(proxy.number_of_channels, dtype='uint8')
                proxy.update_state(
                    hv_output_selected=False,
                    hv_output_enabled=False)

                print('='*(len(title)-9))
                print("Turning off HV output...")

            except Exception as e:
                print(f'[Error] {e}')
            finally:
                time.sleep(1)
                print("Closing connection...")
                try:
                    proxy.terminate()
                    print('Proxy terminated')
                except Exception as e:
                    print(f'[Error] {e}')
                time.sleep(0.1)
        else:
            print('No bytes to read')
        if ser.is_open:
            ser.close()
            print('Serial port closed')
    else:
        print('\033[93m' + 'Serial port is not open' + '\033[0m')
else:
    print('\033[91m' + 'No Teensy 3.2 found' + '\033[0m')
