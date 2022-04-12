# Importing Ping Protocol dependencies libraries
# Libraries are located in Python3 folder in #!/usr/bin/env python3
from brping import definitions
from brping import PingDevice
from brping import pingmessage


class Ping360(PingDevice):
    def initialize(self):
        if not PingDevice.initialize(self):
            return False
        if (self.readDeviceInformation() is None):
            return False
        self._speed_of_sound = 1500         # Initializes sonar with 1500 m/s speed of sound
        self._step = 2
        return True

    ##
    # @brief Get a device_data message from the device\n
    # Message description:\n
    # This message is used to communicate the current sonar state. If the data field is populated, the other fields indicate the sonar state when the data was captured. The time taken before the response to the command is sent depends on the difference between the last angle scanned and the new angle in the parameters as well as the number of samples and sample interval (range). To allow for the worst case reponse time the command timeout should be set to 4000 msec.
    #
    # @return None if there is no reply from the device, otherwise a dictionary with the following keys:\n
    # mode: Operating mode (1 for Ping360)\n
    # gain_setting: Analog gain setting (0 = low, 1 = normal, 2 = high)\n
    # angle: Units: gradian Head angle\n
    # transmit_duration: Units: microsecond Acoustic transmission duration (1~1000 microseconds)\n
    # sample_period: Time interval between individual signal intensity samples in 25nsec increments (80 to 40000 == 2 microseconds to 1000 microseconds)\n
    # transmit_frequency: Units: kHz Acoustic operating frequency. Frequency range is 500kHz to 1000kHz, however it is only practical to use say 650kHz to 850kHz due to the narrow bandwidth of the acoustic receiver.\n
    # number_of_samples: Number of samples per reflected signal\n
    # data: 8 bit binary data array representing sonar echo strength\n
    def get_device_data(self):
        if self.request(definitions.PING360_DEVICE_DATA, 4) is None:
            print("empty request")
            return None
        data = ({
            "mode": self._mode,  # Operating mode (1 for Ping360)
            "gain_setting": self._gain_setting,  # Analog gain setting (0 = low, 1 = normal, 2 = high)
            "angle": self._angle,  # Units: gradian Head angle
            "transmit_duration": self._transmit_duration,  # Units: microsecond Acoustic transmission duration (1~1000 microseconds)
            "sample_period": self._sample_period,  # Time interval between individual signal intensity samples in 25nsec increments (80 to 40000 == 2 microseconds to 1000 microseconds)
            "transmit_frequency": self._transmit_frequency,  # Units: kHz Acoustic operating frequency. Frequency range is 500kHz to 1000kHz, however it is only practical to use say 650kHz to 850kHz due to the narrow bandwidth of the acoustic receiver.
            "number_of_samples": self._number_of_samples,  # Number of samples per reflected signal
            "data": self._data,  # 8 bit binary data array representing sonar echo strength
        })
        return data

    ##
    # @brief Send a device_id message to the device\n
    # Message description:\n
    # Change the device id\n
    # Send the message to write the device parameters, then read the values back from the device\n
    #
    # @param id - Device ID (1-254). 0 and 255 are reserved.
    # @param reserved - reserved
    #
    # @return If verify is False, True on successful communication with the device. If verify is False, True if the new device parameters are verified to have been written correctly. False otherwise (failure to read values back or on verification failure)
    def device_id(self, id, reserved, verify=True):
        m = pingmessage.PingMessage(definitions.PING360_DEVICE_ID)
        m.id = id
        m.reserved = reserved
        m.pack_msg_data()
        self.write(m.msg_data)
        if self.request(definitions.PING360_DEVICE_ID) is None:
            return False
        # Read back the data and check that changes have been applied
        if (verify
                and (self._id != id or self._reserved != reserved)):
            return False
        return True  # success        m.id = id
        m.reserved = reserved
        m.pack_msg_data()
        self.write(m.msg_data)

    def control_reset(self, bootloader, reserved):
        m = pingmessage.PingMessage(definitions.PING360_RESET)
        m.bootloader = bootloader
        m.reserved = reserved
        m.pack_msg_data()
        self.write(m.msg_data)

    def control_transducer(self, mode, gain_setting, angle, transmit_duration, sample_period, transmit_frequency, number_of_samples, transmit, reserved):
        m = pingmessage.PingMessage(definitions.PING360_TRANSDUCER)
        m.mode = mode
        m.gain_setting = gain_setting
        m.angle = angle
        m.transmit_duration = transmit_duration
        m.sample_period = sample_period
        m.transmit_frequency = transmit_frequency
        m.number_of_samples = number_of_samples
        m.transmit = transmit
        m.reserved = reserved
        m.pack_msg_data()
        self.write(m.msg_data)


    def set_mode(self, mode):
        self.control_transducer(
            mode,
            self._gain_setting,
            self._angle,
            self._transmit_duration,
            self._sample_period,
            self._transmit_frequency,
            self._number_of_samples,
            0,
            0
        )
        return self.wait_message([definitions.PING360_DEVICE_DATA, definitions.COMMON_NACK], 4.0)

    def set_gain_setting(self, gain_setting):
        self.control_transducer(
            self._mode,
            gain_setting,
            self._angle,
            self._transmit_duration,
            self._sample_period,
            self._transmit_frequency,
            self._number_of_samples,
            0,
            0
        )
        return self.wait_message([definitions.PING360_DEVICE_DATA, definitions.COMMON_NACK], 4.0)

    def set_angle(self, angle):
        self.control_transducer(
            self._mode,
            self._gain_setting,
            angle,
            self._transmit_duration,
            self._sample_period,
            self._transmit_frequency,
            self._number_of_samples,
            0,
            0
        )
        return self.wait_message([definitions.PING360_DEVICE_DATA, definitions.COMMON_NACK], 4.0)

    def set_transmit_duration(self, transmit_duration):
        self.control_transducer(
            self._mode,
            self._gain_setting,
            self._angle,
            transmit_duration,
            self._sample_period,
            self._transmit_frequency,
            self._number_of_samples,
            0,
            0
        )
        return self.wait_message([definitions.PING360_DEVICE_DATA, definitions.COMMON_NACK], 4.0)

    def set_sample_period(self, sample_period):
        self.control_transducer(
            self._mode,
            self._gain_setting,
            self._angle,
            self._transmit_duration,
            sample_period,
            self._transmit_frequency,
            self._number_of_samples,
            0,
            0
        )
        return self.wait_message([definitions.PING360_DEVICE_DATA, definitions.COMMON_NACK], 4.0)

    def set_transmit_frequency(self, transmit_frequency):
        self.control_transducer(
            self._mode,
            self._gain_setting,
            self._angle,
            self._transmit_duration,
            self._sample_period,
            transmit_frequency,
            self._number_of_samples,
            0,
            0
        )
        return self.wait_message([definitions.PING360_DEVICE_DATA, definitions.COMMON_NACK], 4.0)

    def set_number_of_samples(self, number_of_samples):
        self.control_transducer(
            self._mode,
            self._gain_setting,
            self._angle,
            self._transmit_duration,
            self._sample_period,
            self._transmit_frequency,
            number_of_samples,
            0,
            0
        )
        return self.wait_message([definitions.PING360_DEVICE_DATA, definitions.COMMON_NACK], 4.0)


    def readDeviceInformation(self):
        return self.request(definitions.PING360_DEVICE_DATA)

    def transmitAngle(self, angle):
        self.control_transducer(
            0, # reserved
            self._gain_setting,
            angle,
            self._transmit_duration,
            self._sample_period,
            self._transmit_frequency,
            self._number_of_samples,
            1,
            0
        )
        return self.wait_message([definitions.PING360_DEVICE_DATA, definitions.COMMON_NACK], 0.5)

    def transmit(self):
        return self.transmitAngle(self._angle)


    ## New Ping360 class methods for adjusting scanning range
    # Based on Ping-viewer c++ implementation for similar functionality

    def get_speed_of_sound(self):
        return self._speed_of_sound

    def set_speed_of_sound(self, newSpeed):
        if (newSpeed == self.get_speed_of_sound()):
            print("Requested speed of sound is already set")
            return
        else:
            self._speed_of_sound = newSpeed

    def samplePeriod(self):
        # Multiply with samplePeriodTickDuration which is 25 nanoseconds
        return self._sample_period * 25E-9


    # Returns the set scanning range currently set
    def get_range(self):
        return self.samplePeriod() * self._number_of_samples * self.get_speed_of_sound() / 2


    # Sets new sonar scan range
    def set_range(self, newRange):
        # Checks if new argument is different from set range
        if (newRange == self.get_range()):
            return
        else:
            # Calculate the new sample period to achieve requested distance
            self._sample_period = int(newRange/(self._number_of_samples*25E-9*750))


    def set_step(self, newStep):
        self._step = newStep


    def get_step(self):
        return self._step

    def changeOperatingMode(self, newMode):
        # Checks if input is different from last iteration


        if newMode == 0:
            self.set_range(20) # Short range collision avoidance mode
            self.set_step(4)
            self.set_gain_setting(0)
        elif newMode == 1:
            self.set_range(50) # Medium range collision avoidance mode
            self.set_step(2)
            self.set_gain_setting(1)
        elif newMode == 2:
            self.set_range(2) # Aquaculture inspection mode
            self.set_step(10)
            self.set_gain_setting(0)
        elif newMode == 3:
            self.set_range(4) # Aquaculture inspection mode
            self.set_step(8)
            self.set_gain_setting(0)
        else:
            print("Did not recognize mode command")
            print("Corrupt or invalid data given")
            return