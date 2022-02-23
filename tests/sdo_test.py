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
@brief This is a test script for requesting object dictionary entries from a node.
"""

import canopen

node_id = 3
eds_file = 'eds/ZLAC8030L-V1.0.eds'
obj_dict_id = 0x1018
obj_id2 = 0x1000

def main():
   network = canopen.Network()
   node = canopen.BaseNode402(node_id, eds_file)

   network.connect(bustype='socketcan', channel='can1', bitrate=500000)
   network.add_node(node)

   # Send NMT start to all nodes
   network.send_message(0x0, [0x1, 0])
   node.nmt.wait_for_heartbeat()
   assert node.nmt.state == 'OPERATIONAL'

   #  vednor_id = node.sdo[obj_dict_id][1].raw

   #  print('Vendor ID [{}] = {} \n'.format(obj_dict_id, vednor_id))
  

if __name__=="__main__":
    main()
