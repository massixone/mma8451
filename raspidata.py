""" A Raspberry Pi class.
To access some of the most useful hardware information
"""


#class RaspiData():


    # Get Raspberry Serial Number
    # You can do it in bash:
    #   cat /proc/cpuinfo | perl -n -e '/^Serial[ ]*: ([0-9a-f]{16})$/ && print "$1\n"'
    # Or with perl:
    #   cat cpuinfo | perl -n -e '/^Serial[ ]*: ([0-9a-f]{16})$/ && print "$1\n"'
    #
    #


#    def __init__(self):
#       pass
from uuid import getnode as get_mac

#    @staticmethod
# def get_serial(self):
def get_serial():
    """Extract serial from cpuinfo file"""
    cpu_serial= "0000000000000000"
    try:
        f = open('/proc/cpuinfo','r')
        for line in f:
            if line[0:6]=='Serial':
                cpu_serial = line[10:26]
        f.close()
    except:
        cpu_serial = "ERROR000000000"
    return cpu_serial


def mac_address():
    """Extract rth0 MAC address"""
    # mac = '00:00:00:00:00:00'
    # mac = '00:00:00:00:00:00'
    try:
        t = get_mac()
    except:
        t = "000000000000"
    mac = ':'.join(("%012X" % t)[i:i+2] for i in range(0, 12, 2))
    return mac

