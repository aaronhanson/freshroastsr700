# -*- coding: utf-8 -*-
# Copyright (c) 2015-2016 Mark Spicer
# Made available under the MIT license.

import time
import serial
import multiprocessing as mp
from multiprocessing import sharedctypes

from yoctopuce import yocto_api
from yoctopuce import yocto_temperature

from freshroastsr700 import pid
from freshroastsr700 import utils
from freshroastsr700 import exceptions


class freshroastsr700(object):
    """A class to interface with a freshroastsr700 coffee roaster."""
    def __init__(self, update_data_func=None, state_transition_func=None,
                 use_software_pid=False, external_thermocouple=None):
        """Create variables used to send in packets to the roaster. The update
        data function is called when a packet is opened. The state transistion
        function is used by the timer thread to know what to do next. See wiki
        for more information on packet structure and fields."""
        self.update_data_func = update_data_func
        self.state_transition_func = state_transition_func
        self.use_software_pid = use_software_pid
        self.external_thermocouple = external_thermocouple

        self._header = sharedctypes.Array('c', b'\xAA\xAA')
        self._temp_unit = sharedctypes.Array('c', b'\x61\x74')
        self._flags = sharedctypes.Array('c', b'\x63')
        self._current_state = sharedctypes.Array('c', b'\x02\x01')
        self._footer = sharedctypes.Array('c', b'\xAA\xFA')

        self._fan_speed = sharedctypes.Value('i', 1)
        self._heat_setting = sharedctypes.Value('i', 0)

        self._current_temp = sharedctypes.Value('i', 150)
        self._time_remaining = sharedctypes.Value('i', 0)
        self._total_time = sharedctypes.Value('i', 0)

        self._bean_temp = sharedctypes.Value('d', 0.0)
        self._environment_temp = sharedctypes.Value('d', 0.0)
        self._target_temp = sharedctypes.Value('i', 150)

    def yocto_thermocouple(self):
        errmsg = yocto_api.YRefParam()
        if yocto_api.YAPI.RegisterHub("usb", errmsg) != yocto_api.YAPI.SUCCESS:
            raise exceptions.ExternalThermocoupleError(errmsg.value)

        sensor = yocto_temperature.YTemperature.FirstTemperature()
        if sensor is None :
            raise exceptions.ExternalThermocoupleError("No module connected")

        if not(sensor.isOnline()):
            raise exceptions.ExternalThermocoupleError("device not connected")

        yocto_serial = sensor.get_module().get_serialNumber()

        channel1 = yocto_temperature.YTemperature.FindTemperature(
            yocto_serial + '.temperature1')
        channel2 = yocto_temperature.YTemperature.FindTemperature(
            yocto_serial + '.temperature2')

        while(True):
            self.bean_temp = channel1.get_currentValue()
            self.environment_temp = channel2.get_currentValue()
            yocto_api.YAPI.Sleep(1000)

    @property
    def fan_speed(self):
        """A getter method for _fan_speed."""
        return self._fan_speed.value

    @fan_speed.setter
    def fan_speed(self, value):
        """Verifies the value is between 1 and 9 inclusively."""
        if value not in range(1, 10):
            raise exceptions.RoasterValueError

        self._fan_speed.value = value

    @property
    def heat_setting(self):
        """A getter method for _heat_setting."""
        return self._heat_setting.value

    @heat_setting.setter
    def heat_setting(self, value):
        """Verifies that the heat setting is between 0 and 3."""
        if value not in range(0, 4):
            raise exceptions.RoasterValueError

        self._heat_setting.value = value

    @property
    def target_temp(self):
        return self._target_temp.value

    @target_temp.setter
    def target_temp(self, value):
        if value not in range(150, 550):
            raise exceptions.RoasterValueError

        self._target_temp.value = value

    @property
    def bean_temp(self):
        return self._bean_temp.value

    @bean_temp.setter
    def bean_temp(self, value):
#        if value not in range(0, 550):
#            raise exceptions.RoasterValueError

        self._bean_temp.value = value

    @property
    def environment_temp(self):
        return self._environment_temp.value

    @environment_temp.setter
    def environment_temp(self, value):
#        if value not in range(0, 550):
#            raise exceptions.RoasterValueError

        self._environment_temp.value = value

    @property
    def current_temp(self):
        return self._current_temp.value

    @current_temp.setter
    def current_temp(self, value):
        if value not in range(150, 550):
            raise exceptions.RoasterValueError

        self._current_temp.value = value

    @property
    def time_remaining(self):
        return self._time_remaining.value

    @time_remaining.setter
    def time_remaining(self, value):
        self._time_remaining.value = value

    @property
    def total_time(self):
        return self._total_time.value

    @total_time.setter
    def total_time(self, value):
        self._total_time.value = value

    def connect(self):
        """Connects to the roaster and creates communication thread."""
        port = utils.find_device('1A86:5523')
        self._ser = serial.Serial(
            port=port,
            baudrate=9600,
            bytesize=8,
            parity='N',
            stopbits=1.5,
            timeout=.25,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False)

        self._initialize()

        self.comm_process = mp.Process(target=self.comm)
        self.comm_process.start()
        self.time_process = mp.Process(target=self.timer)
        self.time_process.start()

        if(self.use_software_pid is True):
            self._p = 4.000
            self._i = 0.045
            self._d = 2.200
            self._pid = pid.PID(self._p, self._i, self._d)

            self.software_pid_process = mp.Process(target=self.software_pid)
            self.software_pid_process.start()

        if(self.external_thermocouple == 'yocto-thermocouple'):
            self.external_thermocouple_process = mp.Process(
                target=self.yocto_thermocouple)
            self.external_thermocouple_process.start()

    def _initialize(self):
        """Sends the initialization packet to the roaster."""
        self._header.value = b'\xAA\x55'
        self._current_state.value = b'\x00\x00'
        s = self.generate_packet()
        self._ser.write(s)
        self._header.value = b'\xAA\xAA'
        self._current_state.value = b'\x02\x01'

        # The readline is used here to get the entirety of the current recipe
        # currently on the roaster.
        r = self._ser.readline()
        return r

    def auto_connect(self):
        """Starts a thread that will automatically connect to the roaster when
        it is plugged in."""
        self.connected = False
        self.auto_connect_process = mp.Process(target=self._auto_connect)
        self.auto_connect_process.start()

    def _auto_connect(self):
        """Attempts to connect to the roaster every quarter of a second."""
        while(True):
            try:
                self.connect()
                self.connected = True
                break
            except exceptions.RoasterLookupError:
                time.sleep(.25)

    def disconnect(self):
        """Stops the communication loop to the roaster. Note that this will not
        actually stop the roaster itself, but will allow the program to exit
        cleanly."""
        self.auto_connect_process.join()
        self.comm_process.join()
        self.time_process.join()

        if(self.use_software_pid is True):
            self.software_pid_process.join()

        if(self.external_thermocouple is not None):
            self.external_thermocouple_process.join()

    def comm(self):
        """Main communications loop to the roaster. If the packet is not 14
        bytes exactly, the packet will not be opened. If an update data
        function is available, it will be called when the packet is opened."""
        while(True):
            s = self.generate_packet()
            try:
                self._ser.write(s)
            except serial.serialutil.SerialException:
                self._ser.close()
                self.auto_connect()
                return

            try:
                r = self._ser.read(14)
            except serial.serialutil.SerialException:
                self._ser.close()
                self.auto_connect()
                return

            if(r[-2:] == self._footer):
                temp = int.from_bytes(bytes(r[10:-2]), byteorder='big')

                if(temp == 65280):
                    self.current_temp = 150
                elif(temp > 550 or temp < 150):
                    self._initialize()
                    continue
                else:
                    self.current_temp = temp

                if(self.update_data_func is not None):
                    self.update_data_func()

            time.sleep(.25)

        self._ser.close()

    def timer(self):
        """Timer loop used to keep track of the time while roasting or
        cooling. If the time remaining reaches zero, the roaster will call the
        supplied state transistion function or the roaster will be set to
        the idle state."""
        while(True):
            state = self.get_roaster_state()
            if(state == 'roasting' or state == 'cooling'):
                time.sleep(1)
                self.total_time += 1
                if(self.time_remaining > 0):
                    self.time_remaining -= 1
                else:
                    if(self.state_transition_func is not None):
                        self.state_transition_func()
                    else:
                        self.idle()

    def get_roaster_state(self):
        """Returns a string based upon the current state of the roaster. Will
        raise an exception if the state is unknown."""
        if(self._current_state == b'\x02\x01'):
            return 'idle'
        elif(self._current_state == b'\x04\x04'):
            return 'cooling'
        elif(self._current_state == b'\x08\x01'):
            return 'sleeping'
        elif(self._current_state == b'\x00\x00'):
            return 'connecting'
        elif(self._current_state == b'\x04\x02'):
            return 'roasting'
        else:
            return 'unknown'

    def generate_packet(self):
        """Generates a packet based upon the current class variables. Note that
        current temperature is not sent, as the original application sent zeros
        to the roaster for the current temperature."""
        roaster_time = utils.seconds_to_float(self.time_remaining)
        packet = (
            self._header.value +
            self._temp_unit.value +
            self._flags.value +
            self._current_state.value +
            self.fan_speed.to_bytes(1, byteorder='big') +
            int(float(roaster_time * 10)).to_bytes(1, byteorder='big') +
            self.heat_setting.to_bytes(1, byteorder='big') +
            b'\x00\x00' +
            self._footer.value)

        return packet

    def idle(self):
        """Sets the current state of the roaster to idle."""
        self._current_state.value = b'\x02\x01'

    def roast(self):
        """Sets the current state of the roaster to roast and begins
        roasting."""
        self._current_state.value = b'\x04\x02'

    def cool(self):
        """Sets the current state of the roaster to cool. The roaster expects
        that cool will be run after roast, and will not work as expected if ran
        before."""
        self._current_state.value = b'\x04\x04'

    def sleep(self):
        """Sets the current state of the roaster to sleep. Different than idle
        in that this will set double dashes on the roaster display rather than
        digits."""
        self._current_state.value = b'\x08\x01'

    def software_pid(self):
        """Utilizes a software PID controller to set the heat setting on the
        roaster given the current temperature and a target temperture."""
        while(True):
            if(self.external_thermocouple is not None):
                output = self._pid.update(self.bean_temp, self.target_temp)
            else:
                output = self._pid.update(self.current_temp, self.target_temp)

            if(self.target_temp >= 460):
                if(output >= 30):
                    self.heat_setting = 3
                else:
                    if(self.heat_setting == 2):
                        self.heat_setting = 3
                    else:
                        self.heat_setting = 2
            elif(self.target_temp >= 430):
                if(output >= 30):
                    self.heat_setting = 3
                elif(output >= 20):
                    self.heat_setting = 2
                else:
                    if(self.heat_setting == 1):
                        self.heat_setting = 2
                    else:
                        self.heat_setting = 1
            elif(self.target_temp >= 350):
                if(output >= 30):
                    self.heat_setting = 3
                elif(output >= 20):
                    self.heat_setting = 2
                elif(output >= 10):
                    self.heat_setting = 1
                else:
                    if(self.heat_setting == 0):
                        self.heat_setting = 1
                    else:
                        self.heat_setting = 0
            else:
                if(output >= 30):
                    self.heat_setting = 3
                elif(output >= 20):
                    self.heat_setting = 2
                elif(output >= 10):
                    self.heat_setting = 1
                else:
                    self.heat_setting = 0

            time.sleep(.25)
