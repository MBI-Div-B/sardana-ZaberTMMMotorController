from zaber.serial import BinarySerial, BinaryCommand
import time
from sardana import State
from sardana.pool.controller import MotorController
from sardana.pool.controller import Type, Description, DefaultValue


class ZaberTMMMotorController(MotorController):
    ctrl_properties = {'port': {Type: str,
                                Description: 'The port of the rs232 device',
                                DefaultValue: '/dev/ttyZaber'}}
    
    MaxDevice = 2
    
    def __init__(self, inst, props, *args, **kwargs):
        super(ZaberTMMController, self).__init__(
            inst, props, *args, **kwargs)

        # initialize hardware communication
        self.con = BinarySerial(self.port, timeout=5)
        
        print('Zaber TMM Controller Initialization ...'),
        print('SUCCESS on port %s' % self.port)
        # do some initialization
        self._motors = {}

    def AddDevice(self, axis):
        self._motors[axis] = True
        # change setting of devices, because they are non-volatile
        # disable auto-reply 1*2^0 = 1
        # enable backlash correction 1*2^1 = 2
        command_number = 40 # set device mode
        command = BinaryCommand(axis, command_number, 3)
        self.con.write(command)

    def DeleteDevice(self, axis):
        del self._motors[axis]

    StateMap = {
        1: State.On,
        2: State.Moving,
        3: State.Fault,
    }

    def StateOne(self, axis):
        limit_switches = MotorController.NoLimitSwitch     
        command_number = 54 # return status
        command = BinaryCommand(axis, command_number)
        self.con.write(command)
        reply = self.con.read()
        
        while (reply.command_number != command_number) & (reply.device_number != axis):
            self.con.write(command)
            reply = self.con.read()
            time.sleep(0.2)
            
        if reply.data == 0: # idle
            return self.StateMap[1], 'Zaber is idle', limit_switches
        elif (reply.data >= 1) & (reply.data <=23):
            return self.StateMap[2], 'Zaber is moving', limit_switches
        else:
            return self.StateMap[3], 'Zaber is faulty', limit_switches      

    def ReadOne(self, axis):
        command_number = 60 # return current position
        command = BinaryCommand(axis, command_number)        
        
        for i in range(50):
            self.con.write(command)
            reply = self.con.read()
            if (reply.command_number != command_number) & (reply.device_number != axis):
                time.sleep(0.05)
            else:
                break
        
        return int(reply.data)
        
    def StartOne(self, axis, position):
        command_number = 20 # move absolute
        command = BinaryCommand(axis, command_number, int(position))
        self.con.write(command)

    def StopOne(self, axis):
        command_number = 23 # move absolute
        command = BinaryCommand(axis, command_number)
        self.con.write(command)

    def AbortOne(self, axis):
        command_number = 23 # move absolute
        command = BinaryCommand(axis, command_number)
        self.con.write(command)

    def SendToCtrl(self, cmd):
        """
        Send custom native commands. The cmd is a space separated string
        containing the command information. Parsing this string one gets
        the command name and the following are the arguments for the given
        command i.e.command_name, [arg1, arg2...]

        :param cmd: string
        :return: string (MANDATORY to avoid OMNI ORB exception)
        """
        # Get the process to send
        mode = cmd.split(' ')[0].lower()
        args = cmd.strip().split(' ')[1:]

        if mode == 'homing':
            try:
                if len(args) == 1:
                    axis = args[0]
                    axis = int(axis)
                else:
                    raise ValueError('Invalid number of arguments')
            except Exception as e:
                self._log.error(e)
                
            self._log.info('Starting homing for Zaber mirror')
            
            try:
                command_number = 1 # homing
                command = BinaryCommand(axis, command_number)
                self.con.write(command)
            except Exception as e:
                self._log.error(e)
                print(e)
                return 'Error'

            self._log.info('Homing was finished')
            return "[DONE]"
        else:
            self._log.warning('Invalid command')
            return 'ERROR: Invalid command requested.'