import time
from socket import AF_INET, SOCK_STREAM, socket

from pymodbus.client import ModbusTcpClient

from robot.robots.robot import Robot


class Omron(Robot):
    """Interface for the Omron TM5 cobot."""

    PORT = 5890
    RESPONSE_LENGTH = 1024
    DEFAULT_SCRIPT_ID = 0
    modbus_addresses = {
        'start' : 7104,
        'stop' : 7105,
        'get_x' : 7001,
        'get_y' : 7003,
        'get_z' : 7005,
        'get_rx' : 7007,
        'get_ry' : 7009,
        'get_rz' : 7011,
        'get_tcp_speed' : 7871,
        'get_auto_manual_mode' : 7102,
        'project_running' : 7202,
        'emergency_stopped' : 7208
    }

    modbus_fcs = {
        'start' : 4,
        'stop' : 4,
        'get_x' : 4,
        'get_y' : 4,
        'get_z' : 4,
        'get_rx' : 4,
        'get_ry' : 4,
        'get_rz' : 4,
        'get_tcp_speed' : 4,
        'get_auto_manual_mode' : 4,
        'project_running' : 2,
        'emergency_stopped' : 2
    }

    def __init__(self, ip, config=None):
        """
        Initialize the robot connection.

        :param ip: The IP address of the robot.
        :param config: Unused; kept for the Robot interface.
        """
        self.ip = ip
        self.connected = False
        self.modbus = ModbusTcpClient(ip)
        self.current_move_tag = 1
        self.last_target = None
        self.free_drive = False


    def connect(self):
        """
        Connect to the robot.

        :return: True if successful, otherwise False.
        """
        if self.free_drive:
            print("Cannot connect, free drive enabled!")
            return False

        if self.connected:
            print("Already connected to the robot")
            return True

        if not self.modbus.connect():
            print("Could not connect to the robot's Modbus server.")
            return False

        try:
            if self.__read_modbus_int16('get_auto_manual_mode') == 2:
                print('Robot is in manual mode. Set robot to auto mode to connect. Waiting...')
                # Wait at most 30 s: without a timeout this loop hung forever if
                # the robot stayed in manual mode (and loops forever on a None
                # reply from a failed Modbus read).
                waited = 0.0
                mode = self.__read_modbus_int16('get_auto_manual_mode')
                while mode != 1 and waited < 30.0:
                    time.sleep(0.05)
                    waited += 0.05
                    mode = self.__read_modbus_int16('get_auto_manual_mode')
                if mode != 1:
                    print("Robot did not switch to auto mode within 30 s -- aborting connect.")
                    return False

            result = self.modbus.write_coil(self.modbus_addresses['stop'], True)
            # self.last_start = False
            if not result.isError():
                time.sleep(1)
                # if self.last_start == True:
                #     result = self.modbus.write_coil(self.modbus_cmds['stop'], True)
                #     self.last_start = False
                result = self.modbus.write_coil(self.modbus_addresses['start'], True)
                # self.last_start = True
                if not result.isError():
                    self.socket = socket(AF_INET, SOCK_STREAM)
                    self.socket.setblocking(True)
                    self.socket.settimeout(None)
                    self.socket.connect((self.ip, self.PORT))
                    self.connected = True
        except Exception as e:
            print(f"Failed to connect to the robot:\n{e}")

        return self.connected

    def disconnect(self):
        """
        Disconnects from the robot.

        :return: True if successful, otherwise False.
        """
        success = False

        if not self.connected:
            print("Not connected to the robot, therefore cannot disconnect")
            return success

        try:
            self.socket.close()
            time.sleep(0.5)
            self.modbus.write_coil(self.modbus_addresses['stop'], True)
            # self.last_start = False
            self.connected = False
            self.modbus.close()
            success = True
        except:
            print("Failed to disconnect from the robot")

        return success

    def is_connected(self):
        if self.get_error_state() is not None:
            self.connected = False
        return self.connected

    def get_pose(self):
        """
        Get the current pose of the robot.

        :return: ``(success, pose)``. The pose is a list of 6 values:
          [x, y, z, rx, ry, rz], where x, y, z are in mm and rx, ry, rz are
          Euler angles in degrees (sxyz convention).
        """
        # The socket-based implementation was replaced with Modbus.
        # timestamp_tag = str(time.time()).replace('.','_')
        # command = f'float[] f_{timestamp_tag} = Robot[0].CoordRobot\r\nListenSend(91,f_{timestamp_tag})'
        # response_check = lambda s: s[:6]==b'$TMSTA' and s.split(b',')[2] == b'91'
        # success, raw_response = self.__format_and_send_request(command, 'TMSCT', expect_response = True, response_format_check_func = response_check, verbose=True)
        # print(f'rawr{raw_response}')
        # coordinates = struct.unpack('<6f', raw_response.split(b',')[3])

        coordinates = [self.__read_modbus_float32(addrs) for addrs in ['get_x', 'get_y', 'get_z', 'get_rx', 'get_ry', 'get_rz', ]]
        if any(c is None for c in coordinates):
            # A Modbus read failed -- report a failed update instead of a
            # coordinate list containing None (which would crash the caller).
            return False, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        success = True

        # command = f'ListenSend(91,Robot[0].CoordRobot)'
        # response_check = lambda s: s[:6]==b'$TMSTA' and s.split(b',')[2] == b'91'

        # start = time.time()
        # success, raw_response = self.__format_and_send_request(command, 'TMSCT', expect_response = True, response_format_check_func = response_check, verbose=False)
        # print(f'sendandreceivegetPOSE:{time.time() - start}')
        # print(f'pid{os.getpid()}')
        # while not success:
        #     print('failed message, project stopped')
        #     quit()
        #     self.connect()
        #     print('reconnected')
        #     success, raw_response = self.__format_and_send_request(command, 'TMSCT', expect_response = True, response_format_check_func = response_check, verbose=False)
        # comma_inds = [i for i, b in enumerate(raw_response) if b == ord(',')]
        # coordinates = list(struct.unpack('<6f', raw_response[comma_inds[2]+1:comma_inds[-1]]))

        return success, coordinates



    def is_moving(self):
        """
        Return True if the robot is currently moving.

        :return: True if moving, False if stopped, None if the speed is
          unavailable.
        """
        speed = self.__read_modbus_float32('get_tcp_speed')
        if speed is None:
            return None
        # command = f'{self.current_move_tag}'
        # response_check = lambda s: s[:6]==b'$TMSTA' and s.split(b',')[2] == b'01' and s.split(b',')[3] == str(self.current_move_tag).encode()
        # success, raw_response = self.__format_and_send_request(command, 'TMSTA', expect_response = True, response_format_check_func = response_check, verbose = False)
        # while not success:
        #     print('failed message, reconnecting')
        #     self.connect()
        #     print('reconnected')
        #     success, raw_response = self.__format_and_send_request(command, 'TMSTA', expect_response = True, response_format_check_func = response_check, verbose = False)
        # data = raw_response.decode('utf-8').split(',')[4]

        # The state machine uses this as a plain bool: a tuple is always
        # truthy, so returning one would leave the state machine in MOVING.
        return speed > 0.8


    def get_error_state(self):
        project_running = self.__read_modbus_int16('project_running')
        emergency_stopped = self.__read_modbus_int16('emergency_stopped')
        if project_running is None or emergency_stopped is None:
            # Modbus read failed -- report the error state as not available
            # instead of misreading None as 0 ("Socket Broken").
            return None
        if project_running == 0:
            return "Socket Broken"
        elif emergency_stopped == 1:
            return "Emergency Stopped"
        return None


    def is_error_state(self):
        """
        Return True if the robot is in an error state, False if it is not in an error state, and None if the
        information is not available.

        :return: True if in error state, False if not, and None if the information is not available.
        """

        return self.get_error_state() is not None

    #NOTE: blending not used, set as 0, can be changed later
    def move_linear(self, target, speed_ratio):
        """
        Move the robot in a linear path to the target in robot's base coordinate system with a given speed.

        If the robot is already moving, this method should stop the current movement and start the new one.

        :param target: The target position in a linear path, as a list of 6 values: [x, y, z, rx, ry, rz].
            [x, y, z] define the position in mm, and [rx, ry, rz] define the Euler angles in degrees,
            using the 'sxyz' convention.
        :param speed_ratio: The speed of the movement (as a proportion of the maximum speed; 0-1).
        """
        print(f"move_linear called with speed_ratio = {speed_ratio}")
        # command = (f'Line("CAP", '
        #            f'{target[0]}, {target[1]}, {target[2]}, {target[3]}, {target[4]}, {target[5]}, '
        #            f'{int(speed_ratio*1000)}, 0, 100, true)')
        command = f'Line("CPP", {target[0]}, {target[1]}, {target[2]}, {target[3]}, {target[4]}, {target[5]}, {int(speed_ratio * 100)}, 0, 100, true)'
        success, _ = self.__format_and_send_request(command, 'TMSCT', append_tag=True, verbose = False)
        return success

    def move_circular(self, start_position, waypoint, target, speed_ratio):
        """
        Move the robot in a circular path through a specified waypoint to the target in robot's base
        coordinate system with a given speed.

        If the robot is already moving, this method should stop the current movement and start the new one.

        :param start_position: The starting pose of the robot, as a list of 6 values: [x, y, z, rx, ry, rz].
            The position [x, y, z] is in mm and the Euler angles [rx, ry, rz] are in degrees, using the 'sxyz' convention.
        :param waypoint: The intermediate waypoint, as a list of 6 values: [x, y, z, rx, ry, rz].
            The position [x, y, z] is in mm and the Euler angles [rx, ry, rz] are in degrees, using the 'sxyz' convention.
        :param target: The target pose, as a list of 6 values: [x, y, z, rx, ry, rz].
            The position [x, y, z] is in mm and the Euler angles [rx, ry, rz] are in degrees, using the 'sxyz' convention.
        :param speed_ratio: The speed of the movement (as a proportion of the maximum speed; 0-1).
        """

        command = f'Circle("CPP", {waypoint[0]}, {waypoint[1]}, {waypoint[2]}, {waypoint[3]}, {waypoint[4]}, {waypoint[5]},' + \
                                   f'{target[0]}, {target[1]}, {target[2]}, {target[3]}, {target[4]}, {target[5]},' + \
                                   f'{int(speed_ratio * 100)}, 200, 0, {0}, false)'
        success, _ = self.__format_and_send_request(command, 'TMSCT', append_tag = True, verbose = False)
        return success

    # NOTE: THIS MIGHT NOT BE NEEDED BUT MIGHT ALSO NEED TO JUST PASS FORWARD TO move_linear (see elfin script)
    def dynamic_motion(self, target, speed_ratio):
        """
        Change the robot movement dynamically/immediately in a linear path to the target in robot's base coordinate system
        with a given speed.

        :param target: The target position in a linear path, as a list of 6 values: [x, y, z, rx, ry, rz].
            [x, y, z] define the position in mm, and [rx, ry, rz] define the Euler angles in degrees,
            using the 'sxyz' convention.
        :param speed_ratio: The speed of the movement (as a proprecvortion of the maximum speed; 0-1).
        """
        return self.move_linear(target=target, speed_ratio=speed_ratio)


    def stop_robot(self):
        """
        Stop the robot's movement.

        :return: True if the stop command was successful, False otherwise.
        """
        command = 'StopAndClearBuffer()'
        # NOTE: __format_and_send_request returns a (success, raw_response)
        # tuple -- without unpacking, `if success:` at the call site is always
        # True and failed stops are reported as successful.
        success, _ = self.__format_and_send_request(command, 'TMSCT', verbose=False)
        return success

    def enable_free_drive(self):
        """
        free drive requires the robot to be in manual mode, so this disconnects,
        allowing for the mode to be switched physically via the robot stick/remote
        """
        print('Enabling free drive...')
        self.disconnect()
        self.free_drive = True
        print('Free drive enabled - Robot Controller frozen\n(Switch to Manual mode to move the robot)')


    def disable_free_drive(self):
        """ reconnects to the robot/project after free drive has been completed """
        print('Exiting free drive...')
        self.free_drive = False
        self.connect()
        print('Free drive disabled - Robot Controller resumed')



    # This is never called...
    def close(self):
        """
        Perform necessary cleanup and disconnection procedures for the robot.
        """
        success, _ = self.stop_robot()
        self.disconnect()
        self.modbus.close()

        return success

    # We do not use this but the base abstract class demands we have it implemented.
    def read_force_sensor(self):
        pass

    # We do not use this but the base abstract class demands we have it implemented.
    def initialize(self):
        pass

    def clean_errors(self):
        """Omron errors are cleared on the robot teach pendant."""
        return True

    def __read_modbus_float32(self, command):
        fc = self.modbus_fcs[command]
        address = self.modbus_addresses[command]
        # Read 2 input registers (32-bit float = 2 x 16-bit registers)
        if fc == 1:
            print("Error: Parsing for function code 1 not implemented")
            #result = self.modbus.read_coils(address, count=2)
        elif fc == 2:
            print("Error: Parsing for function code 2 not implemented")
            #result = self.modbus.read_discrete_inputs(address, count=2)
        elif fc == 3:
            print("Error: Parsing for function code 3 not implemented")
           # result = self.modbus.read_holding_registers(address, count=2)
        elif fc == 4:
            result = self.modbus.read_input_registers(address, count=2)
            if result is None or result.isError():
                # Any Modbus timeout / error returns None-ish replies; using
                # result.registers raised AttributeError every loop cycle.
                print(f"W: Modbus read failed for '{command}'.")
                return None
            val = self.modbus.convert_from_registers(result.registers, data_type=self.modbus.DATATYPE.FLOAT32, word_order="big")
        else:
            result = None
            print(f"Function code '{fc}' does not exist!")

        return val

    def __read_modbus_int16(self, command):
        fc = self.modbus_fcs[command]
        address = self.modbus_addresses[command]
        if fc == 1:
            print("Error: Parsing for function code 1 not implemented")
            #result = self.modbus.read_coils(address, count=1)
        elif fc == 2:
            result = self.modbus.read_discrete_inputs(address, count=1)
            if result is None or result.isError():
                print(f"W: Modbus read failed for '{command}'.")
                return None
            return result.bits[0]
        elif fc == 3:
            print("Error: Parsing for function code 3 not implemented")
            #result = self.modbus.read_holding_registers(address, count=1)
        elif fc == 4:
            result = self.modbus.read_input_registers(address, count=1)
            if result is None or result.isError():
                print(f"W: Modbus read failed for '{command}'.")
                return None
        else:
            result = None
            print(f"Function code '{fc}' does not exist!")

        val = self.modbus.convert_from_registers(result.registers, data_type=self.modbus.DATATYPE.INT16, word_order="big")
        return val

    def __calc_checksum(self, data):
        """ calculates the checksum, used as part of the request to the Omron PC used by __format_and_send_request. Explained in the TM robot script protocol documentation """
        # calculated according to xor
        checksum = 0
        for char in data:
            checksum ^= ord(char)

        return checksum

    def __format_and_send_request(self, command, header, append_tag=False, expect_response=False, response_format_check_func=None, verbose=False):
        """ Sends commands to the Omron robot PC. Explained in the TM robot script protocol documentation """
        # NOTE: tags are no longer used as we use Modbus, so the associated code is commented out
        success = True
        raw_response = None

        if header == "TMSCT":
            # if append_tag:
            #     self.current_move_tag += 1
            #     if self.current_move_tag == 16:
            #         self.current_move_tag = 1
            #     tag_command = f'QueueTag({self.current_move_tag},0)'
            #     command = f"{'StopAndClearBuffer()'}\r\n{command}\r\n{tag_command}"

            # script id is needed for a valid request but we have no need for it presently, so it is defaulted to 0.
            length = len(f"{self.DEFAULT_SCRIPT_ID},{command}")
            # checksum is structured as * followed by the hexadecimal of the checksum excluding "0x".
            checksum = self.__calc_checksum(f"{header},{length},{self.DEFAULT_SCRIPT_ID},{command},")
            checksum_formatted = "*{:02X}".format(checksum)
            full_message = f"${header},{length},{self.DEFAULT_SCRIPT_ID},{command},{checksum_formatted}\r\n"
        else:
            # script id is needed for a valid request but we have no need for it presently, so it is defaulted to 0.
            length = len(f"01,{command}")
            # checksum is structured as * followed by the hexadecimal of the checksum excluding "0x".
            checksum = self.__calc_checksum(f"{header},{length},01,{command},")
            checksum_formatted = "*{:02X}".format(checksum)
            full_message = f"${header},{length},01,{command},{checksum_formatted}\r\n"

        if verbose:
            print(f"Sending request: '{command}' formatted as '{full_message}'")

        # This makes socket.recv send a BlockingIOError if no message is present in the buffer, instead of freezing the program to wait for a message.
        self.socket.setblocking(False)

        # Clearing buffer. This is primarily needed due to the robot sending a "script compilation success" message to our socket for every command.
        if verbose:
            print("Clearing socket recieve buffer...")
        try:
            while True:
                m = self.socket.recv(self.RESPONSE_LENGTH)
                if not m:
                    break
                # To ensure we do not overload CPU
                time.sleep(0.001)
        except BlockingIOError:
            pass


        try:
            self.socket.setblocking(True)
            self.socket.settimeout(2.0)

            # Send the request to the robot.
            if verbose:
                print('Sending formatted command...')

            self.socket.sendall(full_message.encode('utf-8'))
            if expect_response:
                if verbose:
                    print('Receiving inputs...')

                while not raw_response:
                    m = self.socket.recv(self.RESPONSE_LENGTH)
                    packets = [segment for segment in m.split(b'\r\n') if segment]
                    for p in packets:
                        if response_format_check_func(p):
                            raw_response = p

                    if m == b'':
                        self.connected = False
                        return False, None

                    time.sleep(0.01)

                if verbose:
                    print(f'Data response received: "{raw_response}"')
        except (BrokenPipeError, ConnectionResetError, TimeoutError) as e:
            print("Robot connection error: {}".format(e))
            self.connected = False
            return False, None

        return success, raw_response
