import time
import serial
import serial.tools.list_ports as lsp
from datetime import datetime
from tqdm import tqdm
from nadamq.NadaMq import cPacket, cPacketParser, PACKET_TYPES


def dump_capacitance(signal):
    tqdm.write(f'{datetime.now()}: Capacitance: {signal["new_value"]:.2e} - Voltage: {signal["V_a"]:.2f}')

def dump(signal):
    tqdm.write(f'{signal}')

port = None
for port in lsp.comports():
    # check if the hwid matches teensy 3.2
    if 'USB VID:PID=16C0:0483' in port.hwid:
        print('Found Teensy 3.2')
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
            print(f'Bytes to read: {bytes_to_read}')
            raw_packet = ser.read(bytes_to_read)
            packet = cPacketParser().parse(raw_packet)
            print(packet)
            ser.close()

            import dropbot as db
            try:
                proxy = db.SerialProxy(ignore=True)
            except Exception as e:
                print(f'[Error] {e}')
                proxy = db.SerialProxy(port=port.device, ignore=True)
            try:
                proxy.signals.signal('output_disabled').connect(dump)
                print('=' * len('='*72))
                for _ in tqdm(range(5), desc='Test Signals'):
                    time.sleep(2)

                title = '='*30 + ' Properties ' + '='*30
                print(title)
                print(proxy.properties)
                print('='*len(title))
                time.sleep(0.1)

                title = '='*32 + ' State ' + '='*33
                print(title)
                print(proxy.state)
                print('='*len(title))
                print("Updating state...")
                proxy.update_state(
                    capacitance_update_interval_ms=500,
                    hv_output_selected=True,
                    hv_output_enabled=True,
                    voltage=80,
                    frequency=1000
                )

                title = '='*28 + ' Updated State ' + '='*29
                print(title)
                print(proxy.state)
                print('='*len(title))

                print("Testing capacitance measurement...")
                proxy.signals.signal('capacitance-updated').connect(dump_capacitance)
                for _ in tqdm(range(5), desc='Measuring capacitance'):
                    time.sleep(0.6)

                proxy.update_state(
                    hv_output_selected=False,
                    hv_output_enabled=False)

                print('='*len(title))
                print("Turning off HV output...")

            except Exception as e:
                print(f'[Error] {e}')
            finally:
                time.sleep(0.1)
                print("Closing connection...")
                try:
                    proxy.terminate()
                except Exception as e:
                    print(f'[Error] {e}')
                time.sleep(0.1)
        else:
            print('No bytes to read')
        if ser.is_open:
            ser.close()
            print('Serial port closed')
    else:
        print('Serial port is not open')