#! /usr/bin/env python
from urx.robot import Robot
import threading
import logging
import time


class RG2:

    def __init__(self, RobotParameter):
        import netifaces as ni
        ni.ifaddresses('enp0s31f6')
        self.ip = ni.ifaddresses('enp0s31f6')[ni.AF_INET][0]['addr']
        self.Robot = (RobotParameter)
        self.GripperMonitorSocket = 50002
        self.MonitorThreadRunning = False
        self.currentwidth = 0.0
        self.logger = logging.getLogger("urx")
        self

    '''Gets the current width between the gripper fingers in mm'''

    def getWidth(self):
        '''opens a thread in the background to listen to a message from the gripper'''

        def StartBackgroundListenerService():
            data = float(-1)
            while self.MonitorThreadRunning == True:
                try:
                    import socket
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    server_address = (self.ip, 50002)
                    sock.bind(server_address)
                    sock.listen(1)
                    connection, client_address = sock.accept()
                    data = connection.recv(64)
                    data = float(data)
                    self.currentwidth = data
                    connection.close()
                    self.MonitorThreadRunning = False
                except Exception as e:
                    print
                    e
                    pass
            return

        cmd_str = "def rg2GripDetect():\n"
        cmd_str += "    zscale = (get_analog_in(2)-0.026)/2.9760034\n"
        cmd_str += "    zangle = zscale*1.57079633-0.087266462\n"
        cmd_str += "    zwidth = 5+110*sin(zangle)\n"
        cmd_str += "    global measure_width = (floor(zwidth*10))/10-9.2\n"
        cmd_str += "    textmsg(\"width\",measure_width)\n"
        cmd_str += "    socket_open(\"" + self.ip + "\", " + "50002" + ")\n"
        cmd_str += "        socket_send_string(measure_width)\n"
        cmd_str += "    socket_close()\n"
        cmd_str += "end\n"
        thread = threading.Thread(target=StartBackgroundListenerService, args=())
        thread.daemon = True
        self.MonitorThreadRunning = True
        thread.start()
        self.Robot.send_program(cmd_str)
        # small delay to wait for the thread
        time.sleep(0.2)
        return self.currentwidth

    '''Sets the width of the robot
    width can be anywhere between 0 and 110, required parameter
        force can be between 3 and 40, default is 20
    '''

    def setWidth(self, width, force=20):
        try:
            if width >= 0 and width <= 110 and force >= 3 and force <= 40:
                cmd_str = "def rg2ProgOpen():\n"
                cmd_str += "\ttextmsg(\"inside RG2 function called\")\n"

                cmd_str += "\ttarget_width="+str(width)+"\n"
                cmd_str += "\ttarget_force="+str(force)+"\n"
                cmd_str += "\tpayload=1.0\n"
                cmd_str += "\tset_payload1=False\n"
                cmd_str += "\tdepth_compensation=False\n"
                cmd_str += "\tslave=False\n"

                cmd_str += "\ttimeout = 0\n"
                cmd_str += "\twhile get_digital_in(9) == False:\n"
                cmd_str += "\t\ttextmsg(\"inside while\")\n"
                cmd_str += "\t\tif timeout > 400:\n"
                cmd_str += "\t\t\tbreak\n"
                cmd_str += "\t\tend\n"
                cmd_str += "\t\ttimeout = timeout+1\n"
                cmd_str += "\t\tsync()\n"
                cmd_str += "\tend\n"
                cmd_str += "\ttextmsg(\"outside while\")\n"

                cmd_str += "\tdef bit(input):\n"
                cmd_str += "\t\tmsb=65536\n"
                cmd_str += "\t\tlocal i=0\n"
                cmd_str += "\t\tlocal output=0\n"
                cmd_str += "\t\twhile i<17:\n"
                cmd_str += "\t\t\tset_digital_out(8,True)\n"
                cmd_str += "\t\t\tif input>=msb:\n"
                cmd_str += "\t\t\t\tinput=input-msb\n"
                cmd_str += "\t\t\t\tset_digital_out(9,False)\n"
                cmd_str += "\t\t\telse:\n"
                cmd_str += "\t\t\t\tset_digital_out(9,True)\n"
                cmd_str += "\t\t\tend\n"
                cmd_str += "\t\t\tif get_digital_in(8):\n"
                cmd_str += "\t\t\t\tout=1\n"
                cmd_str += "\t\t\tend\n"
                cmd_str += "\t\t\tsync()\n"
                cmd_str += "\t\t\tset_digital_out(8,False)\n"
                cmd_str += "\t\t\tsync()\n";
                cmd_str += "\t\t\tinput=input*2\n"
                cmd_str += "\t\t\toutput=output*2\n"
                cmd_str += "\t\t\ti=i+1\n"
                cmd_str += "\t\tend\n"
                cmd_str += "\t\treturn output\n"
                cmd_str += "\tend\n"
                cmd_str += "\ttextmsg(\"outside bit definition\")\n"

                cmd_str += "\ttarget_width=target_width+9.2\n"
                cmd_str += "\tif target_force>40:\n"
                cmd_str += "\t\ttarget_force=40\n"
                cmd_str += "\tend\n"

                cmd_str += "\tif target_force<4:\n"
                cmd_str += "\t\ttarget_force=4\n"
                cmd_str += "\tend\n";
                cmd_str += "\tif target_width>110:\n"
                cmd_str += "\t\ttarget_width=110\n"
                cmd_str += "\tend\n";
                cmd_str += "\tif target_width<0:\n"
                cmd_str += "\t\ttarget_width=0\n"
                cmd_str += "\tend\n"
                cmd_str += "\trg_data=floor(target_width)*4\n"
                cmd_str += "\trg_data=rg_data+floor(target_force/2)*4*111\n"
                cmd_str += "\trg_data=rg_data+32768\n"
                cmd_str += "\tif slave:\n"
                cmd_str += "\t\trg_data=rg_data+16384\n"
                cmd_str += "\tend\n"

                cmd_str += "\ttextmsg(\"about to call bit\")\n"
                cmd_str += "\tbit(rg_data)\n"
                cmd_str += "\ttextmsg(\"called bit\")\n"

                cmd_str += "\tif depth_compensation:\n"
                cmd_str += "\t\tfinger_length = 55.0/1000\n"
                cmd_str += "\t\tfinger_heigth_disp = 5.0/1000\n"
                cmd_str += "\t\tcenter_displacement = 7.5/1000\n"

                cmd_str += "\t\tstart_pose = get_forward_kin()\n"
                cmd_str += "\t\tset_analog_inputrange(2, 1)\n"
                cmd_str += "\t\tzscale = (get_analog_in(2)-0.026)/2.976\n"
                cmd_str += "\t\tzangle = zscale*1.57079633-0.087266462\n"
                cmd_str += "\t\tzwidth = 5+110*sin(zangle)\n"

                cmd_str += "\t\tstart_depth = cos(zangle)*finger_length\n"

                cmd_str += "\t\tsync()\n"
                cmd_str += "\t\tsync()\n"
                cmd_str += "\t\ttimeout = 0\n"

                cmd_str += "\t\twhile get_digital_in(9) == True:\n"
                cmd_str += "\t\t\ttimeout=timeout+1\n"
                cmd_str += "\t\t\tsync()\n"
                cmd_str += "\t\t\tif timeout > 20:\n"
                cmd_str += "\t\t\t\tbreak\n"
                cmd_str += "\t\t\tend\n"
                cmd_str += "\t\tend\n"
                cmd_str += "\t\ttimeout = 0\n"
                cmd_str += "\t\twhile get_digital_in(9) == False:\n"
                cmd_str += "\t\t\tzscale = (get_analog_in(2)-0.026)/2.976\n"
                cmd_str += "\t\t\tzangle = zscale*1.57079633-0.087266462\n"
                cmd_str += "\t\t\tzwidth = 5+110*sin(zangle)\n"
                cmd_str += "\t\t\tmeasure_depth = cos(zangle)*finger_length\n"
                cmd_str += "\t\t\tcompensation_depth = (measure_depth - start_depth)\n"
                cmd_str += "\t\t\ttarget_pose = pose_trans(start_pose,p[0,0,-compensation_depth,0,0,0])\n"
                cmd_str += "\t\t\tif timeout > 400:\n"
                cmd_str += "\t\t\t\tbreak\n"
                cmd_str += "\t\t\tend\n"
                cmd_str += "\t\t\ttimeout=timeout+1\n"
                cmd_str += "\t\t\tservoj(get_inverse_kin(target_pose),0,0,0.008,0.033,1700)\n"
                cmd_str += "\t\tend\n"
                cmd_str += "\t\tnspeed = norm(get_actual_tcp_speed())\n"
                cmd_str += "\t\twhile nspeed > 0.001:\n"
                cmd_str += "\t\t\tservoj(get_inverse_kin(target_pose),0,0,0.008,0.033,1700)\n"
                cmd_str += "\t\t\tnspeed = norm(get_actual_tcp_speed())\n"
                cmd_str += "\t\tend\n"
                cmd_str += "\tend\n"
                cmd_str += "\tif depth_compensation==False:\n"
                cmd_str += "\t\ttimeout = 0\n"
                cmd_str += "\t\twhile get_digital_in(9) == True:\n"
                cmd_str += "\t\t\ttimeout = timeout+1\n"
                cmd_str += "\t\t\tsync()\n"
                cmd_str += "\t\t\tif timeout > 20:\n"
                cmd_str += "\t\t\t\tbreak\n"
                cmd_str += "\t\t\tend\n"
                cmd_str += "\t\tend\n"
                cmd_str += "\t\ttimeout = 0\n"
                cmd_str += "\t\twhile get_digital_in(9) == False:\n"
                cmd_str += "\t\t\ttimeout = timeout+1\n"
                cmd_str += "\t\t\tsync()\n"
                cmd_str += "\t\t\tif timeout > 400:\n"
                cmd_str += "\t\t\t\tbreak\n"
                cmd_str += "\t\t\tend\n"
                cmd_str += "\t\tend\n"
                cmd_str += "\tend\n"
                cmd_str += "\tif set_payload1:\n"
                cmd_str += "\t\tif slave:\n"
                cmd_str += "\t\t\tif get_analog_in(3) < 2:\n"
                cmd_str += "\t\t\t\tzslam=0\n"
                cmd_str += "\t\t\telse:\n"
                cmd_str += "\t\t\t\tzslam=payload\n"
                cmd_str += "\t\t\tend\n"
                cmd_str += "\t\telse:\n"
                cmd_str += "\t\t\tif get_digital_in(8) == False:\n"
                cmd_str += "\t\t\t\tzmasm=0\n"
                cmd_str += "\t\t\telse:\n"
                cmd_str += "\t\t\t\tzmasm=payload\n"
                cmd_str += "\t\t\tend\n"
                cmd_str += "\t\tend\n"
                cmd_str += "\t\tzsysm=0.0\n"
                cmd_str += "\t\tzload=zmasm+zslam+zsysm\n"
                cmd_str += "\t\tset_payload(zload)\n"
                cmd_str += "\tend\n"
                # cmd_str += "\ttextmsg(\"csw\")\n"
                cmd_str += "end\n"
                cmd_str+="rg2ProgOpen()\n"
                #print(cmd_str)
                self.Robot.send_program(cmd_str)

            else:
                self.logger.debug("Width is required to be between 0 and 110 and force is required to be 3 and 40")
                raise RobotException(
                    "Please ensure the gripper width is between 0 and 110 and the force is between 3 and 40")
        except:
            raise RobotException("An unexpected error occured")
        return
