"""
A simple USB to nRF module data sender
"""

import serial
import serial.tools.list_ports
import multiprocessing
from multiprocessing import Process, Queue
import time
import platform
from datetime import datetime
import json
import glob
import sys
from pathlib import Path

SRC_ROOT = Path(__file__).resolve().parent.parent  # src/
CONFIG_DIR = SRC_ROOT / "config"

if 'Linux' in platform.system():
    from getch import getch
elif 'Windows' in platform.system():
    from msvcrt import getwch as getch
else:
    from getch import getch

KEY_COMMANDS = [
    ('N', 'Move / idle'),
    ('u', 'Left Foot Shoot'),
    ('i', 'Right Foot Shoot'),
    ('o', 'Left Foot Pass'),
    ('p', 'Right Foot Pass'),
    ('y', 'Defence / Dodge'),
    ('Y', 'Stand Up'),
    ('x', 'Rest / Dodge'),
    ('W', 'Fast Forward'),
    ('w', 'Forward'),
    ('s', 'Backward'),
    ('a', 'Shift Left'),
    ('d', 'Shift Right'),
    ('A', 'Fast Shift Left'),
    ('D', 'Fast Shift Right'),
    ('q', 'Turn Left'),
    ('e', 'Turn Right'),
]

cfg = None
DEF_port = 'COM14'
DEF_baud = 9600

def print_keyboard_help():
    """Print available keyboard commands once serial port is ready."""
    print("\n=== 鍵盤控制對照 ===")
    for key, desc in KEY_COMMANDS:
        print(f"  {key:>4} -> {desc}")
    print("  SPACE -> 切換 FREE/ROBOT 模式")
    print("  Ctrl+C -> 結束程式\n")

def inputFunc(q):
    """
    Multi processing function, get input

    Parameters
        param1 - A multiprocess queue to push new input data

    Raise
        input character not valid

    Description
        a new character will be pushed only when queue is empty
    """
    while True:
        try:
            input_data = getch()
            if q.empty():
                q.put(input_data)
                time.sleep(0.001)
            if ord(input_data) in [3]:
                break
        except OverflowError:
            print('enter invalid value')

def read_config():
    """
    Read configuration json file

    Return
        retva1 - config dictionary data structure
    """
    global cfg
    cfg_path = CONFIG_DIR / 'config.json'
    with open(cfg_path,'r') as f:
        cfg = json.load(fp = f)

def load_cfg(config):
    """
    Load config from other program

    Parameters
        param1 - outer config dictionary data structure
    """
    global cfg
    cfg = config

def device_read(device, delay=0):
    """
    Reading nRF input after delay

    Parameters
        param1 - nRF serial device
        param2 - read after this delay

    Return
        retva1 - decode data (in gbk)
        retva2 - length

    Raise
        decode error
    """
    time.sleep(delay)
    a = device.in_waiting
    if a > 0:
        data = device.read(a)
        try:
            decode_data = bytes.decode(data, "gbk")
        except Exception:
            print('Decode error. Maybe data too long or reading time too short')
            return "None", a
        else:
            return decode_data,a
    return "None", a

def download_cfg(device, choose_cfg=None):
    """
    Download configuration to device.
    If choose_cfg is None or invalid, fall back to default1 to avoid blocking input() in GUI/threads.
    """
    try:
        for i in range(len(cfg["RESET"]) - 3):
            print(str(i+1) + ' ' + str(cfg["RESET"]["default"+str(i+1)]))
    except Exception:
        pass

    # If running in GUI/threaded mode, avoid blocking input()
    if choose_cfg is None:
        try:
            if sys.stdin.isatty():
                choose_cfg = input("Enter desired address >>> ")
            else:
                choose_cfg = "1"
        except Exception:
            choose_cfg = "1"

    if not str(choose_cfg).isdigit() or int(choose_cfg) > len(cfg["RESET"]) - 3:
        print("No such cfg, use default1")
        choose_cfg = "1"

    address_select = "default" + str(choose_cfg)

    device.write(bytes(cfg["CMD"]["Freq"] + cfg["RESET"]["Freq"], encoding='utf8'))
    data, __ = device_read(device,0.5)
    if "成功" in data:
        print('freq setting success')
    print(data)
    device.write(bytes(cfg["CMD"]["Rate"] + cfg["RESET"]["Rate"], encoding='utf8'))
    data, __ = device_read(device,0.5)
    if "成功" in data:
        print('rate setting success')
    print(data)
    device.write(bytes(cfg["CMD"]["RXA"] + cfg["RESET"][address_select]["RXA"], encoding='utf8'))
    data, __ = device_read(device,0.5)
    if "成功" in data:
        print('rxa setting success')
    print(data)
    device.write(bytes(cfg["CMD"]["TXA"] + cfg["RESET"][address_select]["TXA"], encoding='utf8'))
    data, __ = device_read(device,0.5)
    if "成功" in data:
        print('txa setting success')
    print(data)

def RF_sendCmd(input_data, device, delay=0, mode=0):
    """
    Send command through RF

    Parameters
        param1 - data to send
        param2 - serial device to u33se
        param3 - delay before sending
        param4 - sending mode

    Description
        For mode(@param4) 0, program will extend input character into robot command format and then send
        For mode(@param4) 1, program will directly send what it received
    """
    time.sleep(delay)
    current_time = datetime.now().strftime("%H-%M-%S-%f")
    if mode == 0:
        send_data = '#'+input_data+'1'+input_data+'2'+input_data+'9' +'$'
        print(current_time + " Send=", send_data.encode())
        device.write(bytes(send_data, encoding='utf8'))
    else:
        print(current_time + " Send=", input_data.encode())
        device.write(bytes(input_data, encoding='utf8'))

def get_device():
    """
    Obtain the device connected to computer
    """
    # 抓取所有 /dev/ttyUSB* 和 /dev/ttyACM*（常見於某些 Arduino 裝置）和 /dev/ttyS*
    port_list = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyS*')
    port_list.sort()  # 排序顯示比較整齊
    return port_list

def open_device(portname):
    """
    Scanning through baudrate in the cfg file and existing port

    Parameters
        param1 - name of the port to test

    Return
        retva1 - device open or not
        retva2 - serial device object (None if not opened)
        retva3 - opened device baudrate (0 if not opened)
    """
    for baud in cfg['Scan']:
        print(portname,' Scanning Baudrate: ',baud,end="")
        device = None
        try:
            device = serial.Serial(portname, timeout = 0.5, baudrate = baud)
            device.write(bytes(cfg['CMD']['Info'],encoding = 'utf8'))
            data, __ = device_read(device, 0.5)
            if 'OK' == data[0:2]:
                print('......Success')
                print(data)
                print("Device %s Baudrate: %s found"%(device.name,baud))
                result_device = device
                device = None  # avoid closing in finally
                return 1, result_device, baud
            else:
                print('......Failed')
        except serial.SerialException as err:
            print(f"......Failed ({err})")
        except Exception as err:
            print(f"......Failed ({err})")
        finally:
            if device is not None and device.is_open:
                device.close()
    print('Error --Cannot open serial device')
    return 0, None, 0

def _auto_scan_ports(port_list):
    """
    Automatically scan every available serial port and report usable ones.
    Return first usable device so the caller can keep using it.
    """
    if not port_list:
        print("No serial device found")
        return 0, None, 0

    print("自動掃描序列埠中...")
    usable = []
    chosen_device = None
    chosen_baud = 0

    for port in port_list:
        print(f"👉 嘗試開啟 {port}")
        state, device, baud = open_device(port)
        if state:
            print(f"✅ {port} 可用 (baud={baud})")
            usable.append((port, baud))
            if chosen_device is None:
                chosen_device = device
                chosen_baud = baud
            else:
                device.close()
        else:
            print(f"❌ {port} 無法連線")

    if not usable:
        print("沒有可用的序列埠")
        return 0, None, 0

    print("\n=== 可用序列埠列表 ===")
    for port, baud in usable:
        print(f"- {port} @ {baud} bps")
    print("======================\n")

    return 1, chosen_device, chosen_baud


def device_chose(auto=True):
    """
    The program to teach user how to choose

    Return
        retva1 - device open or not
        retva2 - serial device object (None if not opened)
        retva3 - opened device baudrate (0 if not opened)
    """
    port_list = get_device()
    list_len = len(port_list)

    if auto:
        return _auto_scan_ports(port_list)

    if list_len == 0:
        print("No serial device found")
        return 0, None, 0
    else:
        print("Choose device below......")

    for i in range(0,list_len):
        print('%s %s'%(i+1,port_list[i]))

    choose_device = None
    while(True):
        # Get user input
        input_num = input("Enter device number >>> ")

        if input_num.isdigit():
            choose = int(input_num)
            if choose > list_len:
                print('Device not found')
                continue
            else:
                choose_device = choose - 1
                break
        else:
            if 'q' == input:
                print('Aborting......')
                return 0, None, 0
    state, device, baud = open_device(port_list[choose_device])
    return state, device, baud

def main_procedure(device, input_data=None):
    """
    Main function of the nRF control

    Parameters
        param1 - port to open. If not open then use COM14
        param2 - baud to use. If not given then use 9600

    Description
        Press 'space' to seitch between FREE mode and robot controller mode
        One can use FREE mode to send any character, string or even send AT
        command to currently using module
    """
    # Compatibility: if move_data/list is provided, send once and return
    if input_data is not None:
        try:
            if isinstance(input_data, (list, tuple)) and len(input_data) >= 3:
                send_data = f"#{input_data[0]}{input_data[1]}{input_data[2]}$"
                current_time = datetime.now().strftime("%H-%M-%S-%f")
                print(current_time + " Send=", send_data.encode())
                device.write(bytes(send_data, encoding='utf8'))
                return
            elif isinstance(input_data, str):
                RF_sendCmd(input_data, device, 0.05, mode=1)
                return
        except Exception as err:
            print(f"[main_procedure] send error: {err}")
            return

    q = Queue(5)
    pro = Process(target=inputFunc, args=(q,))
    pro.daemon = True

    pro.start()

    mode = 0
    saved = ''
    while True:
        tstart = time.time()
        current_time = datetime.now().strftime("%H-%M-%S-%f")
        if not q.empty():
            # Get input from queue
            input_data = q.get()

            # End of program
            if ord(input_data) in [3]:
                print(current_time + ' End of program')
                break
            # Switch mode
            elif input_data == ' ':
                saved = ''
                mode = 1 if mode == 0 else 0
                print(current_time + ' Switch to {} mode'.format('FREE' if mode == 1 else 'ROBOT COMMAND'))
            else:
                if mode == 0:
                    print(current_time + ' Has new input: ',ord(input_data), input_data)
                    RF_sendCmd(input_data, device, 0.05, mode)
                else:
                    if ord(input_data) == 10 or ord(input_data) == 13:
                        RF_sendCmd(saved, device, 0.05, mode)
                        saved = ''
                        time.sleep(0.5)
                    else:
                        saved = saved + input_data
                        print(input_data)

        while time.time() - tstart < 1/30:
            # Read input
            data,length = device_read(device)
            if length != 0:
                current_time = datetime.now().strftime("%H-%M-%S-%f")
                print(current_time + ' ' + data)

def device_close(device):
    if device is None:
        return
    else:
        device.close()

if __name__ == '__main__':
    read_config()
    state, device, baud = device_chose()
    if state == 1:
        print_keyboard_help()
        download_cfg(device)
        main_procedure(device)
