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

import struct
from bluepy import btle

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class LumeCube:
    def __init__(self, mac):
        self.mac = mac
        self.SERVICE_UUID = "33826A4C-486A-11E4-A545-022807469BF0"
        self.LIGHT_CHARACTERISTIC = "33826A4D-486A-11E4-A545-022807469BF0"
        self.LIGHT_ON = struct.pack('>L', 0xFCA16400)
        self.LIGHT_OFF = struct.pack('>L', 0xFCA10000)
        self._cube = btle.Peripheral(self.mac, addrType=btle.ADDR_TYPE_RANDOM)
        self._service = self._cube.getServiceByUUID(self.SERVICE_UUID)
        self._ch = self._service.getCharacteristics(self.LIGHT_CHARACTERISTIC)[0]

    def on(self):
        self._ch.write(self.LIGHT_ON)

    def off(self):
        self._ch.write(self.LIGHT_OFF)



class RosLumecube(Node):

    def __init__(self):
        super().__init__('lumecube')
        self.srv = self.create_service(Trigger, '~/trigger', self.trigger_cb)
        self.lumecube = LumeCube("F6:82:8B:E5:C3:1B")
        self.isOn = False
        self.get_logger().warn("Done")

    def trigger_cb(self, request, response):
        while rclpy.ok():
            try:
                if self.isOn:
                    self.lumecube.off()
                    self.isOn = False
                else:
                    self.lumecube.on()
                    self.isOn = True
                break 

            except btle.BTLEDisconnectError as e:
                self.get_logger().warn(str(e))
                self.lumecube = LumeCube("F6:82:8B:E5:C3:1B")
            except btle.BTLEException as e:
                self.get_logger().warn(str(e))
                time.sleep(1)
                pass

        response.success=True
        response.message=f"Lume is {'on' if self.isOn else 'off'}"
        return response

        

def main(args=None):
    rclpy.init(args=args)

    lumecube = RosLumecube()

    rclpy.spin(lumecube)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lumecube.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()