import os
import time
from typing import List
import can
import keyboard
from queue import Queue
import pygame

pygame.init()
pygame.joystick.init()


joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Name of joystick: {joystick.get_name()}")

axis_state = {0: 0, 1: 0}
previous_axis_values = {0: 0, 1: 0}
max_reached = {0: False, 1: False}

def on_up_arrow():
    if (not commandQueue.full()):
        commandQueue.put(prepareCanMessage(0x02, prepareSpeedmodeCommand(run = True, direction = 0, speed = 100, acceleration = 2)))
        print("Up arrow key pressed")

def on_down_arrow():
    if (not commandQueue.full()):
        commandQueue.put(prepareCanMessage(0x02, prepareSpeedmodeCommand(run = True, direction = 1, speed = 100, acceleration = 2)))
        print("Down arrow key pressed")

def on_left_arrow():
    if (not commandQueue.full()):
        commandQueue.put(prepareCanMessage(0x01, prepareSpeedmodeCommand(run = True, direction = 0, speed = 100, acceleration = 2)))
        print("Left arrow key pressed")

def on_right_arrow():
    if (not commandQueue.full()):
        commandQueue.put(prepareCanMessage(0x01, prepareSpeedmodeCommand(run = True, direction = 1, speed = 100, acceleration = 2)))
        print("Right arrow key pressed")

def stop():
    if (not commandQueue.full()):
        commandQueue.put(prepareCanMessage(0x01, prepareSpeedmodeCommand(run = False, direction = 0, speed = 0, acceleration = 2)))
        commandQueue.put(prepareCanMessage(0x02, prepareSpeedmodeCommand(run = False, direction = 0, speed = 0, acceleration = 2)))
        print("no key pressed")

def cyclicRead():
    if (not commandQueue.full()):
        commandQueue.put(prepareCanMessage(0x01, prepareReadEncoderValue()))
        commandQueue.put(prepareCanMessage(0x02, prepareReadEncoderValue()))

# maximum speed is 3000
yMotorSpeed = 3000
xMotorSpeed = 3000

# to rotate 1 cycle, currentPosition + 0x4000
absolutePos = 0x4000
relativePos = 0x0f00

# current position = currentPosition + relativePos
moveRelative = 0xf4
# current position = albolutePos
moveAbsolute = 0xf5

isStopped = False


motorBusy = [{"busy": False, "rotating": False, "timeWaitedAck": 0} for i in range(6)]

# dictionary that conntains expected answer from encoder for each command. 
# for each answer, the [start, end] byte is specified.
# when processing, remember that the start byte is Most Significant Byte.
commandAnswer = {
    # relative position axis
    0xf4: [2, 2],
    # absolute position axis
    0xf5: [2, 2],
    # speed mode
    0xf6: [2, 2],
    # read encoder value
    0x31: [2, 7],
    # read motor real-time speed (RPM)
    0x32: [2, 3],
    # read go back to zero status
    0x3B: [2, 2]
}

commandQueue = Queue(maxsize=10)

def prepareCanMessage(arbitrationId: int, data: list[int]) -> can.Message:
    """
    Prepares a CAN message with the specified arbitration ID and data bytes. automatically calculates the CRC.
    """
    crc = sum(data) + arbitrationId & 0xFF
    data.append(crc)
    return can.Message(arbitration_id=arbitrationId, data=data, is_extended_id=False)

def preparePositionModeAxisCommand(relative, speed, acceleration, axis) -> list[int]:
    """
    Prepare the data to:\n
    Move the motor to the specified axis with specified speed and acceleration.\n
    relative: True for relative, False for absolute\n
    speed: The speed of the motor (0-3000) (2 byte [2-3])\n
    acceleration: Acceleration/Deceleration of the motor (0-255) (1 byte [4])\n
    axis: The axis to move the motor to (-8388607 - +8388607) (3 byte [5-7])\n
    Need to append the arbitration ID and CRC with "prepareCanMessage" before sending.\n

    Relative response: \n
    F4 [status], with [status] = 0 (failed), 1 (start running), 2 (run completed), 3 (end stopper reached)\n

    Absolute response: \n
    F5 [status], with [status] = 0 (failed), 1 (start running), 2 (run completed), 3 (end stopper reached)\n
    """
    if relative == True:
        return [0xf4, speed >> 8, speed & 0xFF, acceleration & 0xFF, (axis >> 16) & 0xFF, (axis >> 8) & 0xFF, axis & 0xFF]
    else:
        return [0xf5, speed >> 8, speed & 0xFF, acceleration & 0xFF, (axis >> 16) & 0xFF, (axis >> 8) & 0xFF, axis & 0xFF]

def prepareSpeedmodeCommand(run: bool, direction:int, speed: int, acceleration: int) -> list[int]:
    """
    Prepare the data to spin the motor freely in direction, with speed and acceleration:\n
    run: True to run, False to stop\n
    Run the motor (True)
        direction: True for clockwise, False for counterclockwise\n
        speed: The speed of the motor (0-3000) (4 low bit of byte 2 and full byte 3)\n
        acceleration: Acceleration of the motor (0-255) (1 byte [4])\n
    Stop the motor (False)
        direction: False\n
        speed: 0x00\n
        acceleration: Deceleration of the motor (0-255) (1 byte [4])\n
    
    Response for run\n
    F6 [status], with [status] = 0 (failed), 1 (run completed)\n
    Response for stop\n
    F6 [status], with [status] = 0 (failed), 1 (stopping), 2 (stopped)\n
    """
    if run:
        return [0xf6, (0x00 | (direction << 7)) | (speed >> 8)  ,speed & 0xFF, acceleration & 0xFF]
    else:
        return [0xf6, 0x00, 0x00, acceleration & 0xFF]
    
def prepareReadEncoderValue() -> list[int]:
    """
    Prepare the data to read the encoder value:\n
    Response:\n
    31 [value], with [value] (6 bytes), signed int.\n
    """
    return [0x31]

def prepareReadMotorSpeed() -> list[int]:
    """
    Prepare the data to read the motor speed:\n
    Response:\n
    32 [value], with [value] (2 bytes), signed int.\n
    """
    return [0x32]

def prepareReadGoBackZero() -> list[int]:
    """
    Prepare the data to read the go back to zero:\n
    Response:\n
    3B [value], with [value] (1 byte): 0 (going back to zero), 1 (zero reached), 2 (failed)\n
    """
    return [0x3B]

def prepareSetCanID(id: int) -> list[int]:
    """
    Prepare the data to set the CAN ID:\n
    id: new ID for the Can slave\n
    Response:\n
    8B,  [value], with [value] (1 byte): 1 (success), 0 (failed)\n
    """
    return [0x8B, (id >> 8) & 0xFF, id & 0xFF]

def prepareGoHome() -> list[int]:
    """
    Prepare the data to go back to zero:\n
    Response:\n
    3B [value], with [value] (1 byte): 0 (go home failed), 1 (go home start), 2 (go home success)\n
    """
    return [0x91]

def prepareInitializeMotor(currentID: int, newID: int) -> list[int]:
    """
    Conduct a list of steps to initialize the motor:\n
    """

    # 2. Set working mode (SR_vFOC)
    setWorkingMode = [0x82, 5]
    # 3. Set Protection function (on)
    setProtection = [0x88, 1]
    # 4. Set subdivision interpolation to enable (255)
    setMplyer = [0x89, 1]
    # 5. Set home command
    # 90 [homeTrigger (low)] [homeDirection (CW)] [homeSpeed 00 60 (dec)] [Endlimit (enabled)]
    setHomeCommand = [90, 0, 0, 0, 60, 1]
    # 5. enable limit port mapping (only for 42D motor, which has CanID > 2)
    setLimitPortRemap = [0x9E, 1]
    # 99. Set the new ID
    setID = prepareSetCanID(newID)
    if (newID > 2):
        return [setWorkingMode, setProtection, setMplyer, setHomeCommand, setID]
    else:
        return [setWorkingMode, setProtection, setMplyer, setHomeCommand, setLimitPortRemap,setID]

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


def canReceiveMessage(buffReader: can.BufferedReader) -> None:
    while buffReader.buffer.qsize() > 0:
        received_msg = buffReader.get_message()
        if received_msg is not None:
            receivedCommand = received_msg.data[0]
            if receivedCommand in commandAnswer.keys():
                value = 0
                start, end = commandAnswer[receivedCommand]
                for i in range(start-1, end):
                    value |= (received_msg.data[i] << (8 * (i - (start - 1))))
                print(f'Received: arbitration_id=0x{received_msg.arbitration_id:X}: {receivedCommand:X} {value}')
                if receivedCommand == 0xf6:
                    if value == 2:
                        motorBusy[received_msg.arbitration_id]["rotating"] = False
                    else:
                        motorBusy[received_msg.arbitration_id]["busy"] = False
                elif receivedCommand in [0xf4, 0xf5]:
                    if value == 2:
                        motorBusy[received_msg.arbitration_id]["busy"] = False
                        print("Run axis completed")
                    elif value == 3:
                        motorBusy[received_msg.arbitration_id]["busy"] = False
                        print("Stopped due to end limit")
            else:
                received_data_bytes = ", ".join(
                [f"0x{byte:02X}" for byte in received_msg.data]
                )
                print(
                    f"Received: arbitration_id=0x{received_msg.arbitration_id:X}, data=[{received_data_bytes}], is_extended_id=False"
                )
        else:
            break


def main() -> None:
    """
    Main function to read CAN messages from a .txt file, send them through a CAN bus, and adjust speeds within packets.
    """
    bus = can.interface.Bus(interface="slcan", channel="COM3", bitrate=500000)  

    print("Press arrow keys to call functions. Press ESC to exit.")

    buffReader = can.BufferedReader()

    DEADZONE = 0.1

    while (True):
        pygame.event.pump()

        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                axis_value = joystick.get_axis(event.axis)
                print(f"Trục {event.axis} di chuyển với giá trị: {axis_value:.2f}") 
                if event.axis == 0:  # Trục X (trái/phải)
                    if axis_value < -DEADZONE and axis_value != previous_axis_values[0]:
                        on_left_arrow()
                        previous_axis_values[0] = axis_value  
                        isStopped = False
                    elif axis_value > DEADZONE and axis_value != previous_axis_values[0]:
                        on_right_arrow()
                        previous_axis_values[0] = axis_value 
                        isStopped = False 
                elif event.axis == 1:  # Trục Y (lên/xuống)
                    if axis_value < -DEADZONE and axis_value != previous_axis_values[1]:
                        on_up_arrow()
                        previous_axis_values[1] = axis_value  
                        isStopped = False
                    elif axis_value > DEADZONE and axis_value != previous_axis_values[1]:
                        on_down_arrow()
                        previous_axis_values[1] = axis_value 
                        isStopped = False
                else:
                    if not isStopped:
                        stop()
                        isStopped = True
            else:
                if not isStopped:
                    stop()
                    isStopped = True
        cyclicRead()

        
        processedMessage = processSendMessage(commandQueue, motorBusy)
        canSendMessage(bus, messages=processedMessage)
        time.sleep(0.004)
        canReceiveMessage(buffReader)

        if keyboard.is_pressed("esc"):
            break
    bus.shutdown()


if __name__ == "__main__":
    main()
