from enum import Enum

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
        self.temp_frame = None
        self.frame_started = False
        self.frame_data = None
        self.sensor_mode = None

        return {
            'result_types': {
                'error': {
                    'format': 'Error!'
                },
                'command': {
                    'format': 'address: {{data.address}}; command: {{data.command}}'
                },
                'serialno': {
                    'format': 'address: {{data.address}}; serialno: {{data.serialno}}'
                },
                'measurement': {
                    'format': 'address: {{data.address}}; flow: {{data.flow}}; raw: {{data.raw}}'
                },
                'unknown': {
                    'format': 'address: {{data.address}}; unknown data'
                }
            }
        }

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
                self.temp_frame['data']['address'] = hex(address_byte)

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
                
                # TODO: do the real decoding
                if "I2C address matches": # TODO should depend on I2C_BUS_ADDRESS_SETTING
                    if "isacommand": # FIXME: I2C master writes command to sensor: identified by write direction (address)
                    
                        # sanity-check if this can be a serial: check length
                        if len( self.frame_data ) is not 2: # commands are 16 bit, i.e. 2 bytes
                            self.temp_frame['type'] = "error"
                        else:
                            self.temp_frame['type'] = "command"
                            if "getserialcmd": # FIXME: check against b"\x10\x00"
                                self.temp_frame['data']['command'] = "get serial number"
                                self.sensor_mode = SensorMode.GET_SERIAL
                            elif "setmeasurementmodecmd": # FIXME: check against b"\x31\xAE"
                                self.temp_frame['data']['command'] = "get measurement"
                                self.sensor_mode = SensorMode.GET_MEASUREMENT
                            elif "softresetcmd": # FIXME: check against b"\x20\x00"
                                self.temp_frame['data']['command'] = "soft reset"
                                self.sensor_mode = None
                            else:
                                self.temp_frame['data']['command'] = "unknown"
                                self.sensor_mode = None
                    else: # I2C master reads command results from sensor: identified by read direction (address)
                        if self.sensor_mode == SensorMode.GET_SERIAL:
                            self.temp_frame['type'] = "serialno"
                            # sanity-check if this can be a serial: check length
                            if len( self.frame_data ) is not 6:
                                self.temp_frame['type'] = "error"
                            else:
                                # TODO: sanity-check if this can be a serial: check CRC
                                # TODO: add serial number to frame
                                #       serial number is sent MSByte and MSBit first: serialno[31:16],crc[7:0],serialno[15:0],crc[7:0]
                                self.temp_frame['data']['serialno'] = "serial number"
                        elif self.sensor_mode == SensorMode.GET_MEASUREMENT:
                            self.temp_frame["type"] = "measurement"
                            # sanity-check if this can be a measurement: check length
                            if len( self.frame_data ) is not 6:
                                self.temp_frame["type"] = "error"
                            else:
                                # TODO: sanity-check if this can be a measurement: check CRC
                                # TODO: check measurement value itself, according to protocol measurement[1:0] is always zero
                                # TODO: add measurement data to frame
                                #       measurement data is sent MSByte and MSBit first: measurement[15:0],crc[7:0]
                                self.temp_frame["data"]["raw"] = 42
                                self.temp_frame["data"]["flow"] = 23.42
                                
                                # TODO: calculate flow from raw value, dependent on type
                                #       flow [slm] = (measured value - offset flow) / scale factor flow
                        else:
                            self.temp_frame["type"] = "unknown"
                            # TODO: might still want to add/display data

                    new_frame = self.temp_frame
                else:
                    new_frame = {}

                self.temp_frame = None
                self.frame_started = False
                self.frame_data = None

                return new_frame

