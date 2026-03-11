"""DropBot connection and hardware integration test.

Usage:
    python -m dropbot.tests.dropbot_connect
    python -m dropbot.tests.dropbot_connect --con-only
    python -m dropbot.tests.dropbot_connect --from-reboot
    python -m dropbot.tests.dropbot_connect --from-reboot --con-only
"""
import argparse
import time
import serial

import numpy as np
import pandas as pd

import serial.tools.list_ports as lsp

from datetime import datetime
from tqdm import tqdm

from nadamq.NadaMq import cPacket, cPacketParser, PACKET_TYPES
from ..hardware_test import test_i2c

def dump_capacitance(signal):
    tqdm.write(f'{datetime.now()}: Capacitance: {signal["new_value"]:.2e} - Voltage: {signal["V_a"]:.2f}')

def dump(signal):
    tqdm.write(f'{signal}' )

halt_events = []
cap_sat_events = []

def on_halted(signal):
    halt_events.append(signal)
    tqdm.write('\033[91m' + f'HALTED: {signal["error"]["name"]}' + '\033[0m')

def on_cap_saturated(signal):
    cap_sat_events.append(signal)
    saturation = signal['error']
    tqdm.write('\033[93m' + f'CAP SATURATED: {saturation["chip_load"]} ({saturation["margin"]}) ' + '\033[0m', end='>')

def dump_event(signal):
    event = signal.get('event', None)
    if event == 'output_disabled':
        tqdm.write('\033[91m' + f'{event}' + '\033[0m')
    elif event == 'output_enabled':
        tqdm.write('\033[92m' + f'{event}' + '\033[0m')
    else:
        tqdm.write(f'{signal}')

parser = argparse.ArgumentParser(description="DropBot connection and hardware test.")
parser.add_argument("--con-only", action="store_true",
                    help="Connect and display info only, skip signal tests")
parser.add_argument("--from-reboot", action="store_true",
                    help="Add extra delays for device settling after flash/reboot")
args = parser.parse_args()

settle_time = 10 if args.from_reboot else .5
print(f"Waiting {settle_time}s for device to settle...")
time.sleep(settle_time)

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

            connect_delay = 5 if args.from_reboot else .5
            time.sleep(connect_delay)

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
                result = test_i2c(proxy)["i2c_scan"]
                df_res = pd.DataFrame.from_dict(result, orient='index')
                df_res.index.name = 'i2c_address'
                print(df_res)
                print('='*(len(title)-9))
                time.sleep(0.1)

                if not args.con_only:
                    proxy.signals.signal('output_disabled').connect(dump_event)
                    proxy.signals.signal('output_enabled').connect(dump_event)

                    proxy.signals.signal('halted').connect(on_halted, weak=False)
                    proxy.signals.signal('capacitance_saturated').connect(on_cap_saturated, weak=False)

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
                        if i == 3:
                            print('Updating Voltage to 100V')
                            proxy.update_state(voltage=100)
                            time.sleep(0.1)
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

                    # ── Test halt signals ──
                    # Feature detection: try toggling the field — if old firmware
                    # ignores it, the readback stays at the protobuf default (True).
                    proxy.update_state(halt_on_chip_load_saturated=False)
                    has_halt_flag = not proxy.state['halt_on_chip_load_saturated']
                    if has_halt_flag:
                        proxy.update_state(halt_on_chip_load_saturated=True)  # restore
                    if has_halt_flag and proxy.number_of_channels > 0:
                        title = '='*26 + '\033[91m' + ' Halt Signal Tests ' + '\033[0m' + '='*27
                        print(title)

                        # --- Test 1: chip_load saturation WITHOUT halt ---
                        print('\n\033[96mTest 1: Chip load saturation (halt disabled)\033[0m')
                        halt_events.clear()
                        cap_sat_events.clear()
                        proxy.update_state(
                            halt_on_chip_load_saturated=False,
                            chip_load_range_margin=0.6,
                            hv_output_selected=True,
                            hv_output_enabled=True,
                            voltage=80,
                            frequency=1000
                        )
                        time.sleep(0.5)  # wait for 25ms timer to fire

                        for i in tqdm(range(11), desc='Testing Random Electrodes'):
                            if not proxy.hv_output_enabled:
                                break
                            channels = np.random.choice([0, 1], size=proxy.number_of_channels,
                                                        p=[1 - i / 10, i / 10]).astype('uint8')
                            proxy.state_of_channels = channels
                            if len(cap_sat_events):
                                tqdm.write(f"\n\033[92m Turned on {sum(channels == 1)} channels > {sum(channels == 1)*1e-11:.2e}\033[0m")
                            else:
                                tqdm.write(f"\033[92m Turned on {sum(channels==1)} channels > {sum(channels == 1)*1e-11:.2e}\033[0m")
                            time.sleep(0.25) # wait enough time for capacitance saturation trigger

                        # HV should still be on (no halt)
                        assert proxy.hv_output_enabled, 'FAIL: HV was halted but should not have been'
                        assert len(cap_sat_events) > 0, 'FAIL: No capacitance_saturated events received'
                        assert len(halt_events) == 0, '\nFAIL: Received halted event when halt was disabled'
                        print(f'\n\033[92m  PASS: {len(cap_sat_events)} capacitance_saturated events, '
                              f'no halt, HV still on\033[0m')

                        # Clean up
                        proxy.state_of_channels = np.zeros(proxy.number_of_channels, dtype='uint8')
                        proxy.update_state(
                            hv_output_enabled=False,
                            hv_output_selected=False
                        )
                        time.sleep(0.2)

                        # --- Test 2: chip_load saturation WITH halt ---
                        print('\n\033[96mTest 2: Chip load saturation (halt enabled)\033[0m')
                        halt_events.clear()
                        cap_sat_events.clear()
                        proxy.update_state(
                            halt_on_chip_load_saturated=True,
                            hv_output_selected=True,
                            hv_output_enabled=True,
                            voltage=80,
                            frequency=1000
                        )
                        time.sleep(0.5)

                        for i in tqdm(range(11), desc='Testing Random Electrodes'):
                            if not proxy.hv_output_enabled:
                                break
                            channels = np.random.choice([0, 1], size=proxy.number_of_channels,
                                                        p=[1 - i / 10, i / 10]).astype('uint8')
                            proxy.state_of_channels = channels
                            tqdm.write(f"\033[92m Turned on {sum(channels==1)} channels > {sum(channels == 1)*1e-11:.2e} \033[0m")
                            time.sleep(0.25) # wait enough time for capacitance saturation trigger

                        # HV should be OFF (halted)
                        assert not proxy.hv_output_enabled, 'FAIL: HV still on after halt'
                        assert len(halt_events) > 0, 'FAIL: No halted events received'
                        print(f'\033[92m  PASS: {len(halt_events)} halted events, '
                              f'HV correctly disabled\033[0m')

                        if len(halt_events) > 0:
                            print('Rebooting...')
                            proxy.reboot()
                            print('Booted!')
                            time.sleep(1)

                            title = '='*26 + '\033[96m' + ' State After Reboot' + '\033[0m' + '='*27
                            print(title)
                            print(proxy.state)
                            print('='*(len(title)-9))
                            time.sleep(0.1)

                        print('='*(len(title)-9))

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
