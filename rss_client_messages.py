
import datetime
import raspidata
import json
import rss_cli_config as ccfg


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
# | 0001         |  Var  | Literally 'CHB'                                        |
# # +-------------------------------------------------------------------------------+
#
# 0002 CHM - Client Hello Message
# +--------------+-------+--------------------------------------------------------+
# | Field        |Length | Description                                            |
# +--------------+-------+--------------------------------------------------------+
# | Datetime     |  16   | 'HH/MM/DD hh:mi:ss.uuuuuu'                             |
# | Device ID    |  16   | 16 byte device ID                                      |
# | 0002         |  Var  | Literally 'CHM'                                        |
# +--------------+-------+--------------------------------------------------------+
#
# 0003  CZM - Client Zap Message (Client is shut down)
# +--------------+-------+--------------------------------------------------------+
# | Field        |Length | Description                                            |
# +--------------+-------+--------------------------------------------------------+
# | Datetime     |  16   | 'HH/MM/DD hh:mi:ss.uuuuuu'                             |
# | Device ID    |  16   | 16 byte device ID                                      |
# | 0003         |  Var  | Literally 'CZM'                                        |
# +--------------+-------+--------------------------------------------------------+
#
# 0004  ADM - Acceleration Data Message (Acceleration data)
# +--------------+-------+--------------------------------------------------------+
# | Field        |Length | Description                                            |
# +--------------+-------+--------------------------------------------------------+
# | Datetime     |  16   | 'HH/MM/DD hh:mi:ss.uuuuuu'                             |
# | Device ID    |  16   | 16 byte device ID                                      |
# | 0004         |  Var  | Literally 'ADM'                                        |
# | Accel Data   |  Var  | Array containing acceleration data (X,Y,Z.O)           |
# +--------------+-------+--------------------------------------------------------+
#
# 0005  CCA - Client Configuration Affirm
# +--------------+-------+--------------------------------------------------------+
# | Field        |Length | Description                                            |
# +--------------+-------+--------------------------------------------------------+
# | Datetime     |  16   | 'HH/MM/DD hh:mi:ss.uuuuuu'                             |
# | Device ID    |  16   | 16 byte device ID                                      |
# | 0004         |  Var  | Literally 'CCA'                                        |
# | Config Data  |  Var  | Array containing client configuration                  |
# +--------------+-------+--------------------------------------------------------+

###########################################################################
#   WARNIG ! THIS FILE IS NOT CURRENTLY USED !
#   Its presence it is for debugging/testing only
#   Althought, it will be used in some future version
###########################################################################

#def heart_bit():
#    """Client Heart Bit"""
#    msg = {'cmd': 'CHB', 'timestamp': str(datetime.datetime.now()), 'clid': raspidata.get_serial()}
#    return msg
#
#
#def hello_message():
#    """Client Hello Message"""
#    msg = {'cmd': 'CHM', 'timestamp': str(datetime.datetime.now()), 'clid': raspidata.get_serial()}
#    return msg
#
#
#def zap_message():
#    """Client Zap Message"""
#    msg = {'cmd': 'CZM', 'timestamp': str(datetime.datetime.now()), 'clid': raspidata.get_serial()}
#    return msg
#
#
#def accel_data_message(data):
#    """Acceleration Data Message"""
#    msg = {'cmd': 'ADM', 'timestamp': str(datetime.datetime.now()), 'clid': raspidata.get_serial(), 'acceldata': str(data)}
#    return msg
#
#
#def config_affirm_message(data):    #def config_affirm_message(data):
#    """Client Configuration Affirm"""
#    ###
#    pkt_hdr = {'cmd': 'CCA', 'timestamp': str(datetime.datetime.now()), 'clid': raspidata.get_serial(), 'data' : data}
#
#    msg = dict(city = ccfg.cityname, latitude = ccfg.latitude, longitude = ccfg.longitude, config = data)
#    return msg

