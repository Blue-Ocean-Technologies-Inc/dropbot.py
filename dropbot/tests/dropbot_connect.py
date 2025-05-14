import time
import serial
import serial.tools.list_ports as lsp
from nadamq.NadaMq import cPacket, cPacketParser, PACKET_TYPES

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
                proxy = db.SerialProxy()
            except Exception as e:
                print(f'[Error] {e}')
                proxy = db.SerialProxy(port=port.device)
            title = '='*30 + ' Properties ' + '='*30
            print(title)
            print(proxy.properties)
            print('='*len(title))
            title = '='*32 + ' State ' + '='*33
            print(title)
            print(proxy.state)
            print('='*len(title))

        else:
            print('No bytes to read')
    else:
        print('Serial port is not open')