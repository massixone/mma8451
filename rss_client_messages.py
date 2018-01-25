
import datetime
import raspidata


#
# All client messages are handled in here.
#
# Message Formats
# ---------------
# 1 - Timestamp
# 2 - Device ID
# 3 - Message Type
#
# Client message types
# --------------------
# 0001  CHB - Client Heart Bit (Client is alive)
# 0002  CHM - Client Hello Message (Client is online)
# 0003  CZM - Client Zap Message (Client is shut down)
# 0004  ADM - Acceleration Data Message (Acceleration data)
# 0005  CCA - Client Config Affirm
# ... (other to come)
#
# Data message Format for all Client Message Types
# ------------------------------------------------
#
# # 0001 - CHB - Client Heart Bit
# +--------------+-------+--------------------------------------------------------+
# | Field        |Length | Description                                            |
# +--------------+-------+--------------------------------------------------------+
# | Datetime     |  16   | 'HH/MM/DD hh:mi:ss.uuuuuu'                             |
# | Device ID    |  16   | 16 byte device ID                                      |
# | 0001         |   2   | Literally '0001'                                       |
# # +-------------------------------------------------------------------------------+
#
# 0002 CHM - Client Hello Message
# +--------------+-------+--------------------------------------------------------+
# | Field        |Length | Description                                            |
# +--------------+-------+--------------------------------------------------------+
# | Datetime     |  16   | 'HH/MM/DD hh:mi:ss.uuuuuu'                             |
# | Device ID    |  16   | 16 byte device ID                                      |
# | 0002         |   2   | Literally '0002'                                       |
# +--------------+-------+--------------------------------------------------------+
#
# 0003  CZM - Client Zap Message (Client is shut down)
# +--------------+-------+--------------------------------------------------------+
# | Field        |Length | Description                                            |
# +--------------+-------+--------------------------------------------------------+
# | Datetime     |  16   | 'HH/MM/DD hh:mi:ss.uuuuuu'                             |
# | Device ID    |  16   | 16 byte device ID                                      |
# | 0003         |   2   | Literally '0003'                                       |
# +--------------+-------+--------------------------------------------------------+
#
# 0004  ADM - Acceleration Data Message (Acceleration data)
# +--------------+-------+--------------------------------------------------------+
# | Field        |Length | Description                                            |
# +--------------+-------+--------------------------------------------------------+
# | Datetime     |  16   | 'HH/MM/DD hh:mi:ss.uuuuuu'                             |
# | Device ID    |  16   | 16 byte device ID                                      |
# | 0004         |   2   | Literally '0004'                                       |
# | Accel Data   |  Var  | Array containing acceleration data (X,Y,Z.O)           |
# +--------------+-------+--------------------------------------------------------+
#
# 0005  CCA - Client Configuration Affirm
# +--------------+-------+--------------------------------------------------------+
# | Field        |Length | Description                                            |
# +--------------+-------+--------------------------------------------------------+
# | Datetime     |  16   | 'HH/MM/DD hh:mi:ss.uuuuuu'                             |
# | Device ID    |  16   | 16 byte device ID                                      |
# | 0004         |   2   | Literally '0005'                                       |
# | Config Data  |  Var  | Array containing client configuration                  |
# +--------------+-------+--------------------------------------------------------+

def heart_bit():
    """Client Heart Bit"""
    msg = {'cmd': '0001', 'timestamp': str(datetime.datetime.now()), 'clientid': raspidata.get_serial()}
    return msg


def hello_message():
    """Client Hello Message"""
    msg = {'cmd': '0002', 'timestamp': str(datetime.datetime.now()), 'clientid': raspidata.get_serial()}
    return msg


def zap_message():
    """Client Zap Message"""
    msg = {'cmd': '0003', 'timestamp': str(datetime.datetime.now()), 'clientid': raspidata.get_serial()}
    return msg


def accel_data_message(data):
    msg = {'cmd': '0004', 'timestamp': str(datetime.datetime.now()), 'clientid': raspidata.get_serial(), 'acceldata': str(data)}
    return msg


def config_affirm_message(data):
    msg = {'cmd': '0005', 'timestamp': str(datetime.datetime.now()), 'clientid': raspidata.get_serial(), 'config': str(data)}
    return msg