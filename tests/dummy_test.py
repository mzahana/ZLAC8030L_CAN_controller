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
@brief This is a dummy test script.
"""

import logging
import  ZLAC8030L_CAN_controller.canopen_controller
from ZLAC8030L_CAN_controller.canopen_controller import MotorController
import time

node_id = 3
def main():
   print("This is the dummy_test.py script. Nothing to do!!! \n")
   obj = MotorController(channel='can0', bustype='socketcan_ctypes', bitrate=500000, node_ids=None, debug=True, eds_file='./eds/ZLAC8030L-V1.0.eds')

   # Get some velocities
   t1 = time.time()
   N = 3000
   obj.setVelocity(node_id=node_id, vel=40.)
   for i in range(N):
     vel =  obj.getVelocity(node_id)
     enc = obj.getEncoder(node_id)
     logging.info("Curent velocity = {} rpm \n".format(vel))
     logging.info("Curent encoder count = {} \n".format(enc))

   #   obj.setVelocity(node_id=1, vel=40.)
   #   obj.setVelocity(node_id=2, vel=40.)
   #   obj.setVelocity(node_id=3, vel=40.)
   #   obj.setVelocity(node_id=4, vel=40.)

   # obj.setVelocity(node_id=1, vel=0.0)
   # obj.setVelocity(node_id=2, vel=0.0)
   # obj.setVelocity(node_id=3, vel=0.0)
   # obj.setVelocity(node_id=4, vel=0.0)


   # logging.info("Getting 100 velocity readings took {} second(s)\n".format(time.time()-t1))

   obj.disconnectNetwork
  

if __name__=="__main__":
    main()