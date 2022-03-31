"""
BSD 3-Clause License
Copyright (c) 2022, Mohamed Abdelkader Zahana
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

"""
@brief This script tests setting and getting speed from a a specific CAN node.
"""

import logging
import  ZLAC8030L_CAN_controller.canopen_controller
from ZLAC8030L_CAN_controller.canopen_controller import MotorController
import time

logging.basicConfig(level=logging.INFO)
node_id = [1,2,3,4]
def main():
   print("This scripts tests speed setting \n")
   time.sleep(1)
   
   obj = MotorController(channel='can0', bustype='socketcan_ctypes', bitrate=500000, node_ids=None, debug=True, eds_file='./eds/ZLAC8030L-V1.0.eds')

   test_time = 3.0 # seconds
   dt = 0.01 # time step
   N = int(test_time/dt)
   
   target_speed = 50.0 # rpm
   

   t1 = time.time()
   for node in node_id:
      obj.setVelocity(node_id=node, vel=target_speed)
      for i in range(N):
         try:
            vel_dict =  obj.getVelocity(node)
            t = vel_dict['timestamp'] # seconds
            speed = vel_dict['value'] # rpm
            logging.warn("Node {} - Curent velocity = {} rpm \n".format(node, speed))
         except Exception as e:
            logging.error("Could not get speed of node {}. Error {}".format(node, e))

         try:
            enc_dict = obj.getEncoder(node_id=node)
            t = enc_dict['timestamp'] # seconds
            counts = enc_dict['value'] # counts
            logging.warn("Node {} - Curent encoder count = {} \n".format(node, counts))
         except Exception as e:
            logging.error("Could not get ecnoder counts of node {}. Error {}".format(node, e))

         try:
            volt_dict = obj.getVoltage(node)
            t = volt_dict['timestamp'] # seconds
            volts = volt_dict['value']  # volts
            logging.warn("Node {} - Voltage = {} \n".format(node, volts))
         except Exception as e:
            logging.error("Could not get DC voltage of node {}. Error {}".format(node, e))

         try:
            err_dict = obj.getErrorCode(node_id=node)
            t = err_dict['timestamp'] # seconds
            code = err_dict['value'] # code
            logging.warn("Node {} - Curent error code = {} \n".format(node, code))
         except Exception as e:
            logging.error("Could not get error code of node {}. Error {}".format(node, e))

         try:
            curr_dict = obj.getMotorCurrent(node)
            t = curr_dict['timestamp'] # seconds
            code = curr_dict['value'] # amps
            logging.warn("Node {} - Motor current = {} \n".format(node, code))
         except Exception as e:
            logging.error("Could not get motor current of node {}. Error {}".format(node, e))


         time.sleep(dt)
      obj.setVelocity(node_id=node, vel=0)


   obj.disconnectNetwork()
  

if __name__=="__main__":
    main()