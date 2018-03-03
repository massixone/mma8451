#!/usr/bin/env python
# -*- coding:utf-8, indent=tab, tabstop=4 -*-
#
# See 'LICENSE'  for copying
#
# This file contains the code for the client module of 'accel.py'
#
# Revision history
# Date          Author                  Version     Details
# ----------------------------------------------------------------------------------
# 2018-01-18    Massimo Di Primio       0.06        1st file implementation

"""Client thread worker - This is a simple client code example for 'accel'.py' program"""

import logging
import time
import datetime
import socket
import json
import rss_client_messages as climsg
import rss_cli_config as ccfg
import raspidata


#def cli_connect(params):
def cli_connect():
    """Open connection to the server"""
    server_address = (str(ccfg.serveraddress), int(ccfg.servertcpport))
    logging.debug('Trying to connect to server ' + str(server_address))
    # Create a TCP/IP socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        # Connect the socket to the port where the server is listening
        s.connect(server_address)
        logging.debug('Connection Established to server ' + str(server_address))
    except:
        logging.debug(
            "Failed to open connection: " + str(ccfg.serverprotocol) +
            ", to IP: " + str(ccfg.serveraddress) +
            ", on port: " + str(ccfg.servertcpport)
            )
        return(-1)
    return(s)


def cli_close(s):
    """Close the server connection"""
    if s > -1:
        s.close()


# def cli_worker(stopEvent, config, accelBuffer):
def cli_worker(stopEvent, accelBuffer):
    """A client worker as thread"""
    logging.debug('Thread Starting')
    s = cli_connect()     # s = cli_connect(config)
    send_client_hello(s)
    time.sleep(0.5)
    send_config_affirm_message(s)

    ts = int(time.time())
    te = ts
    while not stopEvent.wait(0.3):
        if len(accelBuffer) > 0:
            send_accel_data(s, accelBuffer)
            te = int(time.time())
        if (te - ts) > 10:
            send_client_heartbit(s)
            ts = int(time.time())

        time.sleep(0.5)
    send_zap_message(s)
    cli_close(s)
    logging.debug("Thread cliWorker is terminating as per your request.")

    
def send_accel_data(s, accelBuffer):
    """Send acceleration data to the server"""
    #msg = dict(cmd='ADM', timestamp=str(datetime.datetime.now()), clid=raspidata.get_serial())
    #a = 123 #str(datetime.datetime.now())
    pbuf = parse_accel_data(accelBuffer)
    msg = dict(cmd = 'ADM', timestamp = str(datetime.datetime.now()), clid = raspidata.get_serial(), data = pbuf)
    # if len(pbuf) > 0: # this sometimes returns error (when buf is empty, it has None type)
    if (pbuf is not None) and (len(pbuf) > 0):
        #str = climsg.accel_data_message(pbuf)
        try:
            logging.debug("Sending Acceleration data to the server")
            s.sendall(str(json.dumps(msg)) + "\n")  #s.sendall(json.dumps(str) + "\n")
        except:
            logging.debug("Failed to send Acceleration-Data to the server")


def send_client_hello(s):
    """Send Hello message to the server"""
    msg = dict(cmd = 'CHM', timestamp = str(datetime.datetime.now()), clid = raspidata.get_serial())
    try:
        logging.debug("Sending Hello to the server")
        s.sendall(str(json.dumps(msg)) + "\n")  #s.sendall(json.dumps(climsg.hello_message()) + "\n")
    except:
        logging.debug("Failed to send Hello to the server")


def send_zap_message(s):
    """Send Zap message to the server"""
    msg = dict(cmd = 'CZM', timestamp = str(datetime.datetime.now()), clid = raspidata.get_serial())
    try:
        logging.debug("Sending Zap to the server")
        s.sendall(str(json.dumps(msg)) + "\n")  #s.sendall(json.dumps(climsg.zap_message()) + "\n")
    except:
        logging.debug("Failed to send Zap to the server")


def send_config_affirm_message(s):      #def send_config_affirm_message(s, config):
    msg_data = dict(city = ccfg.cityname, latitude = ccfg.latitude,longitude = ccfg.longitude)
    msg = dict(cmd = 'CCA', timestamp = str(datetime.datetime.now()), clid = raspidata.get_serial(), config = msg_data)
    try:
        logging.debug("Sending client configuration to the server")
        s.sendall(str(json.dumps(msg)) + "\n")    #s.sendall(climsg.config_affirm_message(cfg_data))
    except:
        logging.debug("Failed to send client configuration to the server")


def send_client_heartbit(s):
    """Send Heartbit to the server"""
    msg = dict(cmd = 'CHB', timestamp = str(datetime.datetime.now()), clid=raspidata.get_serial())
    try:
        logging.debug("Sending Heartbit to the server")
        s.sendall(str(json.dumps(msg)) + "\n")  #s.sendall(json.dumps(climsg.heart_bit()) + "\n")
    except:
        logging.debug("Failed to send Heartbit to the server")


def parse_accel_data(b):
    """Parse acceleration data to make sure we only send meaningfull data to the server"""
    tsh = 10
    tbuf = []
    # tbuf.append([0, 0, 0, 0, 0])
    # bLength = len(b)
    # logging.debug("parseAccelData(b) # of elements   = " + str(len(b)))
    if len(b) > 1:
        logging.debug("parseAccelData: In  AccelData/BufLen: " + str(len(b)) + "/" +str(len(tbuf)))
        firstTime = 1
        prow = None
        for row in b:
            crow = b.pop(0)     # Get the oldest record
            if firstTime == 1:
                prow = crow
                firstTime = 0
            if ( (abs(abs(int(crow[1])) - abs(int(prow[1]))) > tsh) or
                 (abs(abs(int(crow[2])) - abs(int(prow[2]))) > tsh) or
                 (abs(abs(int(crow[3])) - abs(int(prow[3]))) > tsh)
                ):
                tbuf.append(crow)
                prow = crow
                print ("Again PROW/CROW/TBUFLEN:" + str(prow) + " / " + str(crow) + " / " +  str(len(tbuf)))
        
        logging.debug("parseAccelData: Out AccelData/BufLen: " + str(len(b)) + "/" +str(len(tbuf)))
        return(tbuf)

