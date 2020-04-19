#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
from enum import Enum

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

# Settings constants.
DEVICE_TYPE_SETTING = 'Device type'
I2C_BUS_ADDRESS_SETTING = 'I2C bus address'

DEVICE_TYPE_CHOICES = {
  'Sensirion SFM3000': 0
}

class SensorMode(Enum):
    GET_SERIAL = 1
    GET_MEASUREMENT = 2

# This HLA only uses I2C analyzer frames as input and will produce frames for every valid transaction (from I2C start condition to stop condition).
class FlowMeterHla():

    # declare and initialize member variables
    temp_frame = None
    frame_started = False
    frame_data = None
    sensor_mode = None
    device_busaddress = None

    def __init__(self):
        pass

    def get_capabilities(self):
        return {
            'settings': {
                I2C_BUS_ADDRESS_SETTING : {
                    'type': 'number',
                    'minimum': 0,
                    'maximum': 255
                },
                DEVICE_TYPE_SETTING : {
                    'type': 'choices',
                    'choices': DEVICE_TYPE_CHOICES.keys()
                }
            }
        }

    def set_settings(self, settings):
        # handle settings

        if I2C_BUS_ADDRESS_SETTING in settings.keys():
            self.device_busaddress = int( settings[I2C_BUS_ADDRESS_SETTING] )
        self.temp_frame = None
        self.frame_started = False
        self.frame_data = None
        self.sensor_mode = None

        # return formatting strings
        return {
            'result_types': {
                'error': {
                    'format': 'ERROR!'
                },
                'command': {
                    'format': 'CMD: {{data.command}}'
                },
                'serialno': {
                    'format': 'SERIAL NO: {{data.serialno}}'
                },
                'measurement': {
                    'format': 'MEASUREMENT: flow: {{data.flow}}; raw: {{data.raw}}'
                },
                'unknown': {
                    'format': 'UNKNOWN'
                }
            }
        }

    def cleanNullTerms(self, d):
        clean = {}
        for k, v in d.items():
            if isinstance(v, dict):
                nested = self.cleanNullTerms(v)
                if len(nested.keys()) > 0:
                    clean[k] = nested
            elif v is not None:
                clean[k] = v

        return clean

    def decode(self, data):
        if data["type"] == "start":
            # ignore if already started -> always re-start new packet

            # mark frame as started
            self.frame_started = True
            # reset frame data
            self.temp_frame = {
                'type' : 'invalid',
                'start_time' : data['start_time'],
                'end_time' : data['end_time'],
                'data' : {
                    'address' : None,
                    'command' : None,
                    'raw' : None,
                    'flow' : None
                }
            }
            self.frame_data = bytearray()

        if data["type"] == "address":
            # sanity-check if frame has started
            if self.frame_started:
                # TODO: sanity-check if address has already been set for the frame;
                #       if so, set packet to error
                
                # set address
                address_byte = data['data']['address'][0]
                self.temp_frame['data']['address'] = address_byte

            #self.logger.debug(f"decode(); detected address; temp_frame = '{self.temp_frame}'")

        if data["type"] == "data":
            # sanity-check if frame has started
            if self.frame_started:
                # TODO: sanity-check if frame's address has been set

                # append data
                data_byte = data['data']['data'][0]
                self.frame_data.append( data_byte )

        if data["type"] == "stop":
            # sanity-check if frame has started
            if self.frame_started:
                # TODO: sanity-check if frame's address has been set
                # TODO: sanity-check if frame's data is not empty

                self.temp_frame['end_time'] = data['end_time']
                
                # do the real decoding
                #self.logger.debug(f"decode(); detected stop")
                if self.temp_frame['data']['address']//2 == self.device_busaddress: # I2C address matches
                    #self.logger.debug(f"decode(): address matches")
                    if self.temp_frame['data']['address'] & 1 == 0: # I2C master writes command to sensor: identified by write direction (address)
                        #self.logger.debug(f"decode(): I2C master write")

                        # sanity-check if this can be a serial: check length
                        if len( self.frame_data ) is not 2: # commands are 16 bit, i.e. 2 bytes
                            self.temp_frame['type'] = 'error'
                        else:
                            #self.logger.debug(f"decode(): detected valid command: {self.frame_data}")
                            self.temp_frame['type'] = 'command'
                            if self.frame_data == b"\x31\xAE":
                                #self.logger.debug(f"decode(): detected 'get serial number' command")
                                self.temp_frame['data']['command'] = "get serial number"
                                self.sensor_mode = SensorMode.GET_SERIAL
                            elif self.frame_data == b"\x10\x00":
                                #self.logger.debug(f"decode(): detected 'get measurement' command")
                                self.temp_frame['data']['command'] = "get measurement"
                                self.sensor_mode = SensorMode.GET_MEASUREMENT
                            elif self.frame_data == b"\x20\x00":
                                #self.logger.debug(f"decode(): detected 'soft reset' command")
                                self.temp_frame['data']['command'] = "soft reset"
                                self.sensor_mode = None
                            else:
                                #self.logger.debug(f"decode(): detected unknown command")
                                self.temp_frame['data']['command'] = "unknown"
                                self.sensor_mode = None
                    else: # I2C master reads command results from sensor: identified by read direction (address)
                        #self.logger.debug(f"decode(): I2C master read, sensor mode: {self.sensor_mode}, frame data length: {len( self.frame_data )}")

                        if self.sensor_mode == SensorMode.GET_SERIAL:
                            self.temp_frame['type'] = "serialno"
                            # sanity-check if this can be a serial: check length
                            if len( self.frame_data ) is not 6:
                                self.temp_frame['type'] = "error"
                                self.temp_frame["data"]["raw"] = "invalid"
                            else:
                                # TODO: sanity-check if this can be a serial: check CRC
                                # TODO: add serial number to frame
                                #       serial number is sent MSByte and MSBit first: serialno[31:16],crc[7:0],serialno[15:0],crc[7:0]
                                self.temp_frame['data']['serialno'] = "serial number"
                        elif self.sensor_mode == SensorMode.GET_MEASUREMENT:
                            self.temp_frame["type"] = "measurement"
                            # sanity-check if this can be a measurement: check length
                            if len( self.frame_data ) is not 3:
                                self.temp_frame["type"] = "error"
                                self.temp_frame["data"]["raw"] = "invalid"
                            else:
                                # TODO: sanity-check if this can be a measurement: check CRC
                                # TODO: check measurement value itself, according to protocol measurement[1:0] is always zero
                                # TODO: add measurement data to frame
                                #       measurement data is sent MSByte and MSBit first: measurement[15:0],crc[7:0]
                                raw = int.from_bytes(self.frame_data[:2], byteorder='big', signed=False)
                                self.temp_frame["data"]["raw"] = raw
                                flow = (float(raw) - 32000.0)/140.0;
                                self.temp_frame["data"]["flow"] = "{:+.4f}".format(flow)
                                
                                # TODO: calculate flow from raw value, dependent on type
                                #       flow [slm] = (measured value - offset flow) / scale factor flow
                        else:
                            self.temp_frame["type"] = "unknown"
                            # TODO: might still want to add/display data

                    new_frame = self.cleanNullTerms(self.temp_frame)
                else:
                    #self.logger.debug(f"decode(): address does not match")
                    # address does not match
                    new_frame = None

                self.temp_frame = None
                self.frame_started = False
                self.frame_data = None

                #self.logger.debug(f"decode(); detected stop; returning frame = '{new_frame}'")

                return new_frame
            else:
                #self.logger.debug(f"decode(); detected stop; no frame")
                return None

