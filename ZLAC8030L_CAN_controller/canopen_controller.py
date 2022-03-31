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

import sys
import os
import logging
import traceback
import time
import canopen
from canopen.profiles.p402 import BaseNode402
import time

from numpy import var

# TODO get & report driver errors

# Set logging level. logging.DEBUG will log/print all messages
logging.basicConfig(level=logging.WARN)

class MotorController:
   """MotorController
   Implements convenience methods to monitor and operate ZLAC8030L drivers
   """
   def __init__(self, channel='can0', bustype='socketcan', bitrate=500000, node_ids=None, debug=False, eds_file=None, sync_dt=0.01):
      """
      @brief Creates and connects to CAN bus

      Parameters
      --
      @param channel CAN channel
      @param bustype CAN bus type. See Python canopen & can docs
      @param bitrate CAN bus bitrate. See Python canopen & can docs
      @param node_ids List of expected node IDs. If None, will continue with the available nodes. If specified, will exit if an expected node does not exist on the bus 
      @debug Print logging/debug messages
      """
      self._debug = debug   # what is the use of the _ self._debug
      self._network = canopen.Network() # CAN bus
      self._channel = channel
      self._bustype = bustype
      self._bitrate = bitrate
      self._node_ids = node_ids

      # RPM scaler to be multiplied tby the feedback (current) velocity [rpm]
      self._rpm_scaler = 0.1

      # Velocity dictionary, for all nodes
      self._vel_dict= {}
      # Encoder dictionary, for all nodes
      self._enc_dict= {}
      # DC Voltage read by each node
      self._voltage_dict= {}
      # Motor currents
      self._current_dict = {}
      # Error Register
      self._error_dict = {}

      if eds_file is None:
         raise Exception("eds_file can't be None, please provide valid eds file path!")

      # Following the eaxmple in https://github.com/christiansandberg/canopen/blob/master/examples/simple_ds402_node.py
      # Try to connect
      t0 = time.time() # to calculate total initialization time!
      try:
         self._network.connect(bustype=self._bustype, channel=self._channel, bitrate=self._bitrate)
         self._network.check()

         # Detecet connected nodes
         # This will attempt to read an SDO from nodes 1 - 127
         t1 = time.time()
         self._network.scanner.search()
         dt = time.time() - t1
         # We may need to wait a short while here to allow all nodes to respond
         logging.warn('Available nodes: %s', self._network.scanner.nodes)
         logging.info('Took : %s seconds to search for available nodes', dt)
                  
         if node_ids is not None: # User specified the expected nodes that should be available
            for node_id in node_ids:
               # Sanity checks
               if not type(node_id) is int:
                  logging.error("Only integers are allowed for node ID") 
                  raise TypeError("Only integers are allowed for node ID")
               if node_id < 0 or node_id > 127:
                  logging.error("Node ID %s is not in the range [0,127]", node_id) 
                  raise Exception("Node ID {} is not in the range [0,127]".format(node_id))

               if not (node_id in self._network.scanner.nodes):
                  logging.error("Node ID %s is not available in %s",node_id, self._network.scanner.nodes)
                  raise Exception("Node ID {} is not available in {}".format(node_id, self._network.scanner.nodes)) 

         
         # Add CANopen nodes to the network
         # This will add the detect nodes to the network and set them up with corresponding Object Dictionaries
         for node_id in self._network.scanner.nodes:
            node = canopen.BaseNode402(node_id, eds_file) # This assumes that we can use the same eds file for all motor drivers
            self._network.add_node(node)
            
            # Reset communication
            node.nmt.state = 'RESET COMMUNICATION'
            node.nmt.wait_for_bootup(10)

            node.nmt.state = 'PRE-OPERATIONAL' # Required before setting PDOs
            time.sleep(0.1)
            assert node.nmt.state == 'PRE-OPERATIONAL'
            self.clearTPDO(node=node_id, pdo_id=[1,2,3,4])
            self.setTPDO(node_id=node_id, pdo_id=1, callback=self.pdoCallback)
            self.setTPDO(node_id=node_id, pdo_id=2, callback=self.pdoCallback, var2beMapped=['Error Code', 'Battery voltage', 'Motor current'])

            # Adding the DCvoltage in a separate TPDO as it starts to complain about max PDO size
            #  when adding more than 2 ODs
            # self.setTPDO(node_id=node_id, pdo_id=2, callback=self.pdoCallback, var2beMapped=['DCvoltage'])

            # This does not start RPDO transmission. Need to be done later adter configuring all nodes
            self.setRPDO(node_id=node_id, pdo_id=1)
            

            logging.info("Setting OPERATIONAL NMT state of node {}".format(node_id))
            node.nmt.state = 'OPERATIONAL'
            time.sleep(0.1)
            assert node.nmt.state == 'OPERATIONAL'

            logging.info("Enabling operation mode for node {}".format(node_id))
            node.state = 'SWITCH ON DISABLED'
            node.state = 'READY TO SWITCH ON'
            node.state = 'SWITCHED ON'
            node.state = 'OPERATION ENABLED'
            time.sleep(0.1)
            assert node.state == 'OPERATION ENABLED'
            
            #logging.info('Device operation mode {}'.format(node.tpdo[1]['Mode of operation display'].phys))

         # Separately start the RPDO transmission.
         # This is to avoid flooding the bus before being done witht the configuration
         for id in self._network.scanner.nodes:
            self.startRPDO(node=id, pdo_id=1, dt=0.1)

         # Transmit SYNC every 100 ms
         logging.info("Starting network Sync\n")
         self._network.sync.start(sync_dt)

         logging.warn("Total initialization time  = {} seconds".format(time.time() - t0))
            
      except Exception as e:
         exc_type, exc_obj, exc_tb = sys.exc_info()
         fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
         logging.error('%s %s %s', exc_type, fname, exc_tb.tb_lineno)
         traceback.print_exc()

         # Disconnect from CAN bus
         self.disconnectNetwork()
         exit(1)
      
      # Reaching here means that CAN connection went well.

   def disconnectNetwork(self):
      """
      Disconnects the entire CAN network
      """
      logging.info('Disconnecting CAN network...')
      if self._network:
         for node_id in self._network:
            node =self._network[node_id]
            node.nmt.state = 'PRE-OPERATIONAL'
            node.nmt.stop_node_guarding()
            self._network.sync.stop()
            self._network.disconnect()
            return

   def resetComms(self, wait_time=5):
      """Reset network status"""
      self._network.nmt.state = 'RESET COMMUNICATION'
      time.sleep(wait_time)

   def setNMTPreOperation(self, node_ids=None):
      """
      If node_ids=None, this method broadcasts network.nmt.state to 'PRE-OPERATIONAL' for all nodes in the network 
      Sets node.nmt.state to 'PRE-OPERATIONAL'. Raises exceptions on failures

      Parameters
      --
      @param node_ids list of Node IDs. List of [int]
      """
      if node_ids == None:
         self._network.nmt.state = 'PRE-OPERATIONAL' 

         # TODO Needs checking
         for node_id in self._network:
            node =self._network[node_id]
            node.nmt.wait_for_heartbeat()
            assert node.nmt.state == 'PRE-OPERATIONAL'
         return

      # Sanity checks
      if not type(node_ids) is list:
         raise Exception("[preOperation] node_ids should be a list of at least one node ID")

      # TODO Needs checking  
      logging.error('changing state to PRE-OPERATIONAL for all nodes')
      for node_id in self._network:
            node =self._network[node_id]
            try:
               node.nmt.state = 'PRE-OPERATIONAL'
            except:
               canopen.nmt.NmtError
               logging.error("Setting NMT [PreOperation] failed for Node ID %s",node_id)
               raise Exception("Setting NMT [PreOperation] failed for Node ID {}".format(node_id)) 
            return

   def clearTPDO(self, node=1, pdo_id=[1]):
      """Clear TPDO configuration

      Params
      --
      @param node Node ID
      @param pdo_id List of TPDO IDs
      """
      node = self._network[node]
      try:
         node.tpdo.read()
         for id in pdo_id:
            node.tpdo[id].clear()
            node.tpdo[id].enabled = False
         node.tpdo.save()
      except Exception as e:
         logging.error("Could not clear TPDO {}. Error {}".format(pdo_id, e))

   def setTPDO(self, node_id=1, pdo_id=1, var2beMapped=['Current speed', 'Actual position'], callback=None):
      """Sets the TPDO mapping"""
      logging.info("Setting up TPDO {} of node {}".format(pdo_id, node_id))
      node = self._network[node_id]
      try:
         # Read PDO configuration from node
         node.tpdo.read()
         
         # Re-map TxPDO_id
         node.tpdo[pdo_id].clear()
         #node.tpdo[pdo_id].add_variable('StatusWord')

         for var in var2beMapped:
            # current speed is in 0.1 rpm
            # actual position is in encoder counts
            node.tpdo[pdo_id].add_variable(var)

         node.tpdo[pdo_id].trans_type = 1
         node.tpdo[pdo_id].event_timer = 0
         node.tpdo[pdo_id].enabled = True
         if callback is not None:
            node.tpdo[pdo_id].add_callback(callback)
         # node.tpdo[pdo_id].add_callback(self.pdoCallback)
         # Save new PDO configuration to node
         node.tpdo.save() # node must be in PRE-OPERATIONAL NMT state
      except Exception as e:
         logging.error("Could not configure TPDO {} for node {}".format(pdo_id, node_id))

   def setRPDO(self, node_id=1, pdo_id=1):
      """Sets the RPDO mapping"""
      logging.info("Setting up RPDO {} of node {}".format(pdo_id, node_id))
      node = self._network[node_id]
      try:
         node.rpdo.read()
         node.rpdo[pdo_id].clear()
         node.rpdo[pdo_id].add_variable('Target speed')
         node.rpdo[pdo_id].enabled = True
         node.rpdo.save()

         node.rpdo[pdo_id]['Target speed'].phys = 0.0
         # Start only after you set all nodes
         # Otherwise, the bus gets busy and can fail!
         #node.rpdo[pdo_id].start(0.1)
      except Exception as e:
         logging.error("Could not configure RPDO {} for node {}".format(pdo_id, node_id))

   def startRPDO(self, node=1, pdo_id=1, dt=0.1):
      """ Start a particular RPDO for a particular node"""
      logging.info("Starting RPDO {} of node {}".format(pdo_id, node))
      node = self._network[node]
      try:
         node.rpdo[pdo_id].start(dt)
      except Exception as e:
         logging.error("Could not start RPDO {} of  node {}. Error: {}".format(pdo_id, node, e))

   def enableOperation(self, node_ids):
      """
      Sets node.state to 'OPERATION ENABLED'. Raises exceptions on failures

      Parameters
      --
      @param node_id list of Node IDs. List of [int]
      """
      # Sanity checks
      if not type(node_ids) is list:
         raise Exception("[enableOperation] node_ids should be a list of at least one node ID")

      for id in node_ids:
         # Sanity check
         if not type(id) is int:
            raise Exception("[enableOperation] Node ID {} is not int".format(id))

         if not id in self._network:
            raise Exception("Node ID {} is not available. Skipping operation enabling.".format(id))

         node = self._network[id]
         timeout = time.time() + 15
         node.state = 'OPERATION ENABLED'
         while node.state != 'OPERATION ENABLED':
            if time.time() > timeout:
               raise Exception('Timeout when trying to change state of node ID {}'.format(id))
         time.sleep(0.001)

   def getOperationStatus(self, node_id):
      """
      Gets the operation status (not network status), of a particular node

      Parameters
      --
      @param node_id Node ID [int]

      Returns
      --
      @return status Operation status. Raises an exception on error
      """
      # Sanity checks # Needs checking
      self.checkNodeID(node_id)
            
      # TODO Needs checking 
      # Read the state of the Statusword
      return self._network[node_id].statusword # 
      

   def getNMTStatus(self, node_id):
      """
      Gets the NMT status of a particular node
      
      Parameters
      --
      @param node_id Node ID [int]

      Returns
      --
      @return status NMT status. Raises an exception on error
      """
      # Sanity checks

      # TODO Needs implementation
      pass

   def checkNodeID(self, node_id):
      if not type(node_id) is int:
         raise Exception("[nodeID_sanity_checks] Node ID {} is not int".format(node_id))
      if node_id < 0 or node_id > 127:
         logging.error("Node ID %s is not in the range [0,127]", node_id) 
         raise Exception("Node ID {} is not in the range [0,127]".format(node_id))
      if not self._node_ids==None:
         if not node_id in self._node_ids:
            raise Exception("Node ID {} is not available!".format(node_id))
        

   def setVelocity(self, node_id=1, vel=10.0):
      """
      Sets velocity value of a particular node

      Parameters
      --
      @param node_id Node ID [int]
      @param vel Velocity PRM
      """
      
      try:
         # Sanity checks
         self.checkNodeID(node_id)
         node = self._network[node_id]
         node.rpdo[1]['Target speed'].phys = vel
      except Exception as e:
         logging.error("Could not set velocity for node {}. Error: ".format(node_id, e))



   # def getVelocity(self, node_id):
   #    """
   #    Gets velocity value of a particular node

   #    Parameters
   #    --
   #    @param node_id Node ID [int]

   #    Returns
   #    --
   #    @return vel Velocity in rpm. Raises an exception on error
   #    """
   #    try:
   #       # Sanity checks
   #       self.checkNodeID(node_id=node_id)
   #       # TODO Needs testing for scaling factor 
   #       #actual_speed = self._network[node_id].sdo['Current Speed']
   #       node = self._network[node_id]
   #       node.tpdo[1].wait_for_reception()
   #       speed = node.tpdo[1]['Current speed'].phys
   #       return speed * self._rpm_scaler
   #    except Exception as e:
   #       logging.error("Could not get velocity for node {}. Error: {}".format(node_id, e))

   def getVelocity(self, node_id):
      """
      Returns the current speed stored in self._vel_dict[node_id]

      Parameters
      --
      @param node_id Node ID [int]

      Returns
      --
      @return vel Velocity dictionary {'timestamp': seconds, 'value': rpm}
      """
      # Sanity checks
      self.checkNodeID(node_id=node_id)
      
      return self._vel_dict[node_id]

   # def getEncoder(self, node_id):
   #    """
   #    Gets encoder value of a particular node

   #    Parameters
   #    --
   #    @param node_id Node ID [int]

   #    Returns
   #    --
   #    @return enc Encoder value. Raises an exception on error
   #    """
   #    try:
   #       # Sanity checks
   #       self.checkNodeID(node_id=node_id)
   #       # TODO Needs testing for scaling factor 
   #       node = self._network[node_id]
   #       node.tpdo[1].wait_for_reception()
   #       enc = node.tpdo[1]['Actual position'].phys
   #       return enc
   #    except Exception as e:
   #       logging.error("Could not get encoder position for node {}. Error: {}".format(node_id, e))

   def getEncoder(self, node_id):
      """
      Gets encoder value of a particular node

      Parameters
      --
      @param node_id Node ID [int]

      Returns
      --
      @return enc Encoder dictionary {'timestamp': seconds, 'value': counts}
      """
      # Sanity checks
      self.checkNodeID(node_id=node_id)
      
      return self._enc_dict[node_id]

   def getVoltage(self, node_id):
      """
      Gets DC voltage value of a particular node

      Parameters
      --
      @param node_id Node ID [int]

      Returns
      --
      @return DC voltage dictionary {'timestamp': seconds, 'value': volt}
      """
      # Sanity checks
      self.checkNodeID(node_id=node_id)
      
      return self._voltage_dict[node_id]

   def getErrorCode(self, node_id):
      """
      Gets error code of a particular node

      Parameters
      --
      @param node_id Node ID [int]

      Returns
      --
      @return Error code dictionary {'timestamp': seconds, 'value': code}
      """
      # Sanity checks
      self.checkNodeID(node_id=node_id)
      
      return self._error_dict[node_id]

   def getMotorCurrent(self, node_id):
      """
      Gets motor current of a particular node

      Parameters
      --
      @param node_id Node ID [int]

      Returns
      --
      @return Motor current dictionary {'timestamp': seconds, 'value': amps}
      """
      # Sanity checks
      self.checkNodeID(node_id=node_id)
      
      return self._current_dict[node_id]

   def EStop(self):
      """Emergency STOP"""
      pass

   def pdoCallback(self, msg):
      try:
         node_id = msg.cob_id & 0x7F
         for var in msg:
            if var.name == 'Current speed':
               self._vel_dict[node_id] = {'timestamp':msg.timestamp, 'value':var.raw * self._rpm_scaler}
               logging.debug('Speed of node {} = {}'.format(node_id, self._vel_dict[node_id]))
            if var.name == 'Actual position':
               self._enc_dict[node_id] = {'timestamp':msg.timestamp, 'value':var.raw}
               logging.debug('Encoder counts of node {} = {}'.format(node_id, self._enc_dict[node_id]))
            if var.name == 'Battery voltage':
               self._voltage_dict[node_id] = {'timestamp':msg.timestamp, 'value':var.raw * 0.01}
               logging.debug('DC voltage read by node {} = {}'.format(node_id, self._voltage_dict[node_id]))
            if var.name == 'Error Code':
               self._error_dict[node_id] = {'timestamp':msg.timestamp, 'value':var.raw}
               logging.debug('Error register of node {} = {}'.format(node_id, self._error_dict[node_id]))
            if var.name == 'Motor current':
               self._current_dict[node_id] = {'timestamp':msg.timestamp, 'value':var.raw}
               logging.debug('Motor current of node {} = {}'.format(node_id, self._current_dict[node_id]))
               
      except Exception as e:
         logging.error("Error in  TPDO1 callback of node = {}. Error: {}".format(node_id, e))
   