import time
import os
import asyncio
import threading

from pymavlink import mavutil

from ipc import IPC, IPCMessage, ThreadID
from gps import gps_thread_exec
from flight_controller import flight_controller_thread_exec, FCMsg

conn = mavutil.mavlink_connection('udpout:' + os.environ['MAV_HOST'] + ':14540')

def esc_test():
    print('Running ESC test')
    print('Arming')
    esc.arm()
    print('Armed')
    max_val = 200

    for x in range(max_val):
        esc.throttle = (math.sin(time.time())+1)/2/1
        time.sleep(.05)
    esc.throttle = 0

#gps_serial_test()

async def wait_conn():
    """
    Sends a ping to stabilish the UDP communication and awaits for a response
    """
    print('waiting')
    msg = None
    while not msg:
        conn.mav.ping_send(
            int(time.time() * 1e6), # Unix time in microseconds
            0, # Ping number
            0, # Request ping of all systems
            0 # Request ping of all components
        )
        msg = conn.recv_match()
        time.sleep(0.5)

async def mavhost():
    await wait_conn()

    while True:
        try:
            print(conn.recv_match().to_dict())
        except:
            pass
        time.sleep(0.1)


def main():
    print('Starting Helios II Avionics')

    ipc = IPC(ThreadID.MAIN, host=True)

    ipc_links = {}

    gps_thread = threading.Thread(target=gps_thread_exec)
    fc_thread = threading.Thread(target=flight_controller_thread_exec)

    gps_thread.start()
    fc_thread.start()

    alive = True

    while True:
        msg, addr = ipc.recv()
        if not(msg.from_id == ThreadID.FLIGHT_CONTROLLER and msg.msg_type == FCMsg.IMU_DATA.value):
            print(msg.data)

        ipc_links[msg.from_id] = addr

        if msg.to_id == ThreadID.ALL:
            for key in ipc_links.keys():
                ipc.send_to(msg.data, msg.to_id, ipc_links[key], msg.msg_type)
        elif msg.to_id != ThreadID.MAIN:
            link = ipc_links.get(msg.to_id)
            if link != None:
                ipc.send_to(msg.data, msg.to_id, link, msg.msg_type)
        
        time.sleep(.01)

main()
