from enum import Enum


class Descriptors(Enum):
    ACK_CCCDESCRITPTOR = "000002902-0000-1000-8000-00805f9b34fb"
    TX_CCCDESCRITPTOR = "000002902-0000-1000-8000-00805f9b34fb"
    # public static final UUID RX_SERVICE_UUID = UUID.fromString("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
    # public static final UUID RX_CHAR_UUID = UUID.fromString("6e400002-b5a3-f393-e0a9-e50e24dcca9e");
    # public static final UUID TX_CHAR_UUID = UUID.fromString("6e400003-b5a3-f393-e0a9-e50e24dcca9e");
    RX_SERVICE_UUID = "00002760-08c2-11e1-9073-0e8ac72e1011"
    RX_CHAR_UUID = "00002760-08c2-11e1-9073-0e8ac72e0011"
    TX_CHAR_UUID = "00002760-08c2-11e1-9073-0e8ac72e0012"
    ACK_CHAR_UUID = "00002760-08c2-11e1-9073-0e8ac72e0013"


ACK_VALUE = bytes([0x05, 0x00, 0x00, 0x20, 0x00,
                   0x8d, 0xef, 0x02, 0xd2])


class Commands(Enum):
    NOT_A_COMMAND = 0
    SET_MOTORS = 1
    GET_MOTORS = 2
    RES_MOTORS = 3
    SET_GAINS = 4
    GET_ODOM = 5
    SET_VEL = 6
    GET_VEL = 7
    SER_TX = 8
    SER_RX = 9
    PING = 10
    PONG = 11
    START_BYTESTREAM_TX = 12
    STOP_BYTESTREAM_TX = 13
    BYTESTREAM_TX = 14
    BYTESTREAM_RX = 15


def getCommandName(name):
    try:
        return Commands(name).name
    except ValueError:
        return "invalid command " + str(name)
