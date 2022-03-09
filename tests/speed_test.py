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

node_id = 3
def main():
   print("This scripts tests speed setting \n")
   time.sleep(1)
   
   obj = MotorController(channel='can0', bustype='socketcan_ctypes', bitrate=500000, node_ids=None, debug=True, eds_file='./eds/ZLAC8030L-V1.0.eds')

   test_time = 5.0 # seconds
   dt = 0.01 # time step
   N = int(test_time/dt)
   
   target_speed = 50.0 # rpm
   obj.setVelocity(node_id=node_id, vel=target_speed)

   t1 = time.time()
   for i in range(N):
     vel_dict =  obj.getVelocity(node_id)
     t = vel_dict['timestamp'] # seconds
     speed = vel_dict['value'] # rpm

     enc_dict = obj.getEncoder(node_id=node_id)
     t = enc_dict['timestamp'] # seconds
     counts = enc_dict['value'] # counts

     logging.info("Curent velocity = {} rpm \n".format(speed))
     logging.info("Curent encoder count = {} \n".format(counts))

     time.sleep(dt)

   #   obj.setVelocity(node_id=1, vel=40.)
   #   obj.setVelocity(node_id=2, vel=40.)
   #   obj.setVelocity(node_id=3, vel=40.)
   #   obj.setVelocity(node_id=4, vel=40.)

   # obj.setVelocity(node_id=1, vel=0.0)
   # obj.setVelocity(node_id=2, vel=0.0)
   # obj.setVelocity(node_id=3, vel=0.0)
   # obj.setVelocity(node_id=4, vel=0.0)


   logging.info("Getting {} velocity readings took {} second(s)\n".format(N, time.time()-t1))

   obj.disconnectNetwork()
  

if __name__=="__main__":
    main()