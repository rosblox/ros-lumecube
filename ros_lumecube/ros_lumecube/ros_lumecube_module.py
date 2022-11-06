# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
import random

import struct
from bluepy import btle

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class Lumecube:
    def __init__(self, mac):
        self.SERVICE_UUID = "33826a4c-486a-11e4-a545-022807469bf0"
        self.LIGHT_CHARACTERISTIC = "33826a4d-486a-11e4-a545-022807469bf0"
        self.LIGHT_ON = struct.pack('>L', 0xFCA16400)
        self.LIGHT_OFF = struct.pack('>L', 0xFCA10000)

        self.connect(mac)
        self._service = self._cube.getServiceByUUID(self.SERVICE_UUID)
        self._ch = self._service.getCharacteristics(self.LIGHT_CHARACTERISTIC)[0]
        self.blink()

    def connect(self,mac):
        isConnected = False
        while not isConnected:
            self._cube = btle.Peripheral(mac, addrType=btle.ADDR_TYPE_RANDOM)
            time.sleep(2.0+random.random())
            try:
                isConnected = self._cube.getState() == "conn"  
            except btle.BTLEDisconnectError:
                pass

    def blink(self):
        self.on()
        time.sleep(0.2)
        self.off()
        time.sleep(0.2)
        self.on()
        time.sleep(0.2)
        self.off()

    def on(self):
        self._ch.write(self.LIGHT_ON)

    def off(self):
        self._ch.write(self.LIGHT_OFF)


class RosLumecube(Node):

    def __init__(self):
        super().__init__('lumecube')
        self.srv = self.create_service(Trigger, '~/trigger', self.trigger_cb)
        self.lumecube = Lumecube("F6:82:8B:E5:C3:1B")
        self.isOn = False
        self.get_logger().info("Lumecube initiated")

    def trigger_cb(self, request, response):
        try:
            if self.isOn:
                self.lumecube.off()
                self.isOn = False
            else:
                self.lumecube.on()
                self.isOn = True

        except btle.BTLEException as e:
            self.get_logger().warn(str(e))

        response.success=True
        response.message=f"Lume is {'on' if self.isOn else 'off'}"
        return response

        

def main(args=None):
    rclpy.init(args=args)

    ros_lumecube = RosLumecube()

    try:
        rclpy.spin(ros_lumecube)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
