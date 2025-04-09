import os
import time
from typing import List
import can
import keyboard
from queue import Queue
import pygame
from mks_api import *
from game_pad import *

pygame.init()
pygame.joystick.init()


joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Name of joystick: {joystick.get_name()}")

axis_state = {0: 0, 1: 0}
previous_axis_values = {0: 0, 1: 0}
max_reached = {0: False, 1: False}
isStoppedBuffer = [False, False, False, False, False, False]

isStopped = False
motorBusy = { i : {"busy": False, "rotating": False, "timeWaitedAck": 0} for i in range(1, 7)}
commandQueue = Queue(maxsize=10)

def cyclicRead():
    if (not commandQueue.full()):
        commandQueue.put(prepareCanMessage(0x01, prepareReadEncoderValue()))
        commandQueue.put(prepareCanMessage(0x02, prepareReadEncoderValue()))

def prepareCanMessage(arbitrationId: int, data: list[int]) -> can.Message:
    """
    Prepares a CAN message with the specified arbitration ID and data bytes. automatically calculates the CRC.
    """
    crc = sum(data) + arbitrationId & 0xFF
    data.append(crc)
    return can.Message(arbitration_id=arbitrationId, data=data, is_extended_id=False)

def processSendMessage(commandQueue: Queue[can.Message], motorBusy) -> List[can.Message]:
    """
    Processes a list of CAN commandQueue, checking if the motor is busy before sending them.

    Args:
        commandQueue: A list of `can.Message` objects to be sent.
        motorBusy: A list of dictionaries indicating the busy status of each motor.

    Note:
        This function checks if the motor is busy before sending commandQueue and updates the motor's status accordingly.
    """
    _processedMesssage = []
    while not commandQueue.empty():
        msg = commandQueue.get()
        send = False
        # if command is controlling and speed > 0
        if (msg.data[0] in [0xf4, 0xf5, 0xf6]) and ((msg.data[1] | msg.data[2]) > 0):
            # check if the motor is busy, if yes, discard the command
            if motorBusy[msg.arbitration_id]["busy"] == True:
                print("Motor busy, command discarded")
                continue
            elif motorBusy[msg.arbitration_id]["rotating"] == True and msg.data[0] == 0xf6:
                print("Motor rotating, command discarded")
                continue
            else:
                # send it.
                motorBusy[msg.arbitration_id]["busy"] = True
                motorBusy[msg.arbitration_id]["timeWaitedAck"] = time.time()
                if msg.data[0] == 0xf4:
                    motorBusy[msg.arbitration_id]["rotating"] = True
                send = True
        else:
            send = True
        if send:
            _processedMesssage.append(msg)
    return _processedMesssage
            

def canSendMessage(bus: can.interface.Bus, messages: list) -> None:
    """
    Sends a list of CAN messages through a specified CAN bus and waits for responses.

    Args:
        bus: The `can.interface.Bus` instance representing the CAN bus to send messages on.
        messages: A list of `can.Message` objects to be sent.

    Note:
        This function waits for responses from expected motors after sending messages
        and prints out the status of the sent and received messages.
    """
    if len(messages) == 0:
        return
    for msg in messages:
        bus.send(msg)
        data_bytes = ", ".join([f"0x{byte:02X}" for byte in msg.data])
        print(
            f"Sent: arbitration_id=0x{msg.arbitration_id:X}, data=[{data_bytes}], is_extended_id=False"
        )

def initializeMotor(bus: can.interface.Bus, currentID: int, newID: int) -> None:
    """
    Initializes the motor by sending a series of commands to set its ID, working mode, protection function,
    subdivision interpolation, and home command.

    Args:
        bus: The `can.interface.Bus` instance representing the CAN bus to send messages on.
        currentID: The current CAN ID of the motor.
        newID: The new CAN ID to set for the motor.

    Note:
        This function sends a series of commands to the motor and waits for responses.
    """
    messages = prepareInitializeMotor(currentID, newID)
    for i in range(len(messages)):
            canSendMessage(bus, [prepareCanMessage(currentID, messages[i])], motorBusy)

def processReceivedMessage(buffReader: can.BufferedReader) -> None:
    while buffReader.buffer.qsize() > 0:
        receivedMsg = buffReader.get_message()
        if receivedMsg is not None:
            receivedCommand = receivedMsg.data[0]
            if receivedCommand in commandAnswer.keys():
                value = 0
                start, end = commandAnswer[receivedCommand]
                for i in range(start-1, end):
                    value |= (receivedMsg.data[i] << (8 * (i - (start - 1))))
                print(f'Received: arbitration_id=0x{receivedMsg.arbitration_id:X}: {receivedCommand:X} {value}')
                if receivedCommand == 0xf6:
                    if value == 2:
                        motorBusy[receivedMsg.arbitration_id]["rotating"] = False
                    else:
                        motorBusy[receivedMsg.arbitration_id]["busy"] = False
                elif receivedCommand in [0xf4, 0xf5]:
                    if value == 2:
                        motorBusy[receivedMsg.arbitration_id]["busy"] = False
                        print("Run axis completed")
                    elif value == 3:
                        motorBusy[receivedMsg.arbitration_id]["busy"] = False
                        print("Stopped due to end limit")
            else:
                received_data_bytes = ", ".join(
                [f"0x{byte:02X}" for byte in receivedMsg.data]
                )
                print(
                    f"Received: arbitration_id=0x{receivedMsg.arbitration_id:X}, data=[{received_data_bytes}], is_extended_id=False"
                )
        else:
            break

def checkAckTimeout():
    """
    Check the ack timeout, if the motor take too long to answer, release the lock "busy" and "rotating" to allow 
    control.
    """
    global motorBusy
    for id, motor in motorBusy.items():
        if motor["busy"] or motor["rotating"]:
            duration = time.time() - motor["timeWaitedAck"]
            if duration >= 5:
                motor["busy"] = False
                motor["rotating"] = False
                print(f"motorID:{id} ack timeout" )


def main() -> None:
    global isStopped
    """
    Main function to read CAN messages from a .txt file, send them through a CAN bus, and adjust speeds within packets.
    """
    # real bus
    # bus = can.interface.Bus(interface="slcan", channel="COM3", bitrate=500000)  
    # virtual bus
    bus = can.interface.Bus(interface="virtual", receive_own_messages=True)  

    print("Press arrow keys to call functions. Press ESC to exit.")

    buffReader = can.BufferedReader()

    delay_100ms = 0

    while (True):
        pygame.event.pump()
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                handle_button_press(event.button, buffer)
            if event.type == pygame.JOYBUTTONUP:
                handle_button_release(event.button, buffer)
            if event.type == pygame.JOYAXISMOTION:
                handle_axis_motion(event.axis, event.value, buffer)
        
        for i in range(len(buffer)):
            if buffer[i] == -1:
                if (not commandQueue.full()):
                    commandQueue.put(prepareCanMessage(i+1, prepareSpeedmodeCommand(run = True, direction = 0, speed = 100, acceleration = 2)))
                    isStoppedBuffer[i] = False
            elif buffer[i] == 1:
                if (not commandQueue.full()):
                    commandQueue.put(prepareCanMessage(i+1, prepareSpeedmodeCommand(run = True, direction = 1, speed = 100, acceleration = 2)))
                    isStoppedBuffer[i] = False
            elif buffer[i] == 0:
                if (isStoppedBuffer[i] == False and not commandQueue.full()):
                    commandQueue.put(prepareCanMessage(i+1, prepareSpeedmodeCommand(run = False, direction = 0, speed = 0, acceleration = 2)))
                    isStoppedBuffer[i] = True
            else:
                raise ValueError("Wtf?")


        # a bunch of function that read the status of motors:
        # if delay_100ms < 20:
        #     delay_100ms += 1
        # else:
        #     cyclicRead()
        #     delay_100ms = 0

        
        processedMessage = processSendMessage(commandQueue, motorBusy)
        canSendMessage(bus, messages=processedMessage)
        time.sleep(0.01)
        processReceivedMessage(buffReader)
        checkAckTimeout()
        if keyboard.is_pressed("esc"):
            break
    bus.shutdown()


if __name__ == "__main__":
    main()
