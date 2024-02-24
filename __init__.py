"""simple behaviour tree for stretchbox, making it quite universal"""
import bluetooth
from ble_simple_peripheral import BLESimplePeripheral
import math

class RemoteControl(Action): ## somehow I think it could be leaf node
    def __init__(self, bluetooth_communicator, blackboard=None, communicator=None):
        super().__init__("", blackboard, communicator)
        self.bluetooth_communicator = bluetooth_communicator

        self.command = ""
        self.blackboard["min_l"] = 0
        self.blackboard["max_l"] = 0
        self.blackboard["min_r"] = 0
        self.blackboard["max_r"] = 0 
        self.blackboard["min_pos"] = "0 0"
        self.blackboard["min_pos"] = "0 0"
    def on_rx(self,v):
        print("remote:",v)
        if v ==  "stopm":
            self.communicator.send_command_to_driver("J") #send soft stop
            self.blackboard["run_sequence"] = False
            self.blackboard["reset_node_flag"] = True
            self.command = "J"

            self.status = Status.RUNNING # switch to remote control
            return
        elif v == "wylacz":
            self.communicator.send_command_to_driver("H") #send hard stop
            self.command = "H"
            self.blackboard["run_sequence"] = False
            #self.blackboard["sequence"].reset_recursive()
            self.blackboard["reset_node_flag"] = True
            self.status = Status.RUNNING
            return
        elif v == "uruchom":
            self.blackboard["run_sequence"] = True
            self.status = Status.SUCCESS # proceed, no remote control
            return
        elif v == "umin": #set current position as min
            print(self.blackboard)
            self.blackboard["min_l"] = self.blackboard["pos_l"]
            self.blackboard["min_r"] = self.blackboard["pos_r"]
            #format min_pos to one point after .
            self.blackboard["min_pos"]= f"{self.blackboard['min_l']:.1f} {self.blackboard['min_r']:.1f}"
        elif v == "umax":
            print(self.blackboard)
            self.blackboard["max_l"] = self.blackboard["pos_l"]
            self.blackboard["max_r"] = self.blackboard["pos_r"]
            self.blackboard["max_pos"]= f"{self.blackboard['max_l']:.1f} {self.blackboard['max_r']:.1f}"
        elif v == "uzero":
            #self.communicator.send_command_to_driver("Z") 
            self.command = "Z"
            #TODO move relative min and max
        elif v == "wl":
            self.command = "C"
        elif v == "st":
            #self.communicator.send_command_to_driver("X 1")
            self.command = "X 1"
        elif v == "re":
            #self.communicator.send_command_to_driver("X -1")
            self.command = "X -1"
        elif v == "lo":
            #self.communicator.send_command_to_driver("M 1")
            self.command = "M 1"
        elif v == "po":
            #self.communicator.send_command_to_driver("M -1")
            self.command = "M -1"
        elif v == "ll": #do lewa lewy aktuator
            #self.communicator.send_command_to_driver("L 1")
            self.command = "L 1"
        elif v == "lp":  #do lewa prawy aktuator
            #self.communicator.send_command_to_driver("L -1")
            self.command = "R -1"
        elif v == "pl": #do prawa lewy akt
            #self.communicator.send_command_to_driver("P 1")
            self.command = "L -1"
        elif v == "pp": #do prawa prawy
            #self.communicator.send_command_to_driver("P -1")
            self.command = "R 1"
    def execute(self): #"""will be called by the tree, only then common actions run"""
        #print("remote control")
        self.bluetooth_communicator.add_callback(self.on_rx,"remote_control")


        #check if we are still connected (there are connections)
        
        if len(self.bluetooth_communicator.uart._connections) <1:
            print(len(self.bluetooth_communicator.uart._connections))
            self.command = "J"
            self.status = Status.RUNNING
            #print("no connections")
        #    self.status = Status.FAILURE
        #    return self.status
        #send feedback
        self.bluetooth_communicator.send_feedback()
        if self.status == Status.RUNNING and self.command != "": #in remote control
            self.communicator.send_command_to_driver(self.command)
            self.command = "" # send once
        return self.status


            
class BasicCommunicator():
    def __init__(self, uart, blackboard = None):
        self.uart = uart
        self.sent_status = "NOT_STARTED"
        self.done = False
        self.unpacked_data = [0,0,0,0,0,0,0,0,0,0,0,0]
        self.last_command = "NOT_STARTED"
        self.last_successful_comms = time.time()  # Initialize to current time
        self.wait_comms = True #no new comms
        self.callbacks = {}
        if blackboard:
            self.blackboard = blackboard
        else:
            self.blackboard = Blackboard()
        self.n = 0
    def send_command_to_driver(self,command):
        print("sending command",command)
        uart1.write(command+"\n")

    def unpack_to_blackboard(self,packed): #we push the status to blackboard
        #
        keys = ["torque_l", "torque_r", "noTorque_l", "noTorque_r", "pos_l", "pos_r", "target_l",
                "target_r", "vel_l", "vel_r", "last_command", "last_command_param"]
        #torque_l, torque_r, noTorque_l, noTorque_r, pos_l, pos_r, target_l, target_r, vel_l, vel_r, last_command, last_command_param 
        # = unpacked_data
        for keys, values in zip(keys,packed):
            self.blackboard[keys] = values
        self.blackboard["last_command_human"] = status_list[self.blackboard["last_command"]]
        self.blackboard["done"] = self.done
        self.blackboard["SELF_STATUS"] = self.sent_status
    def comms(self):
        """decodes incoming communication and translates statues also tries to understand if action is still conducted"""
        uart1 = self.uart
        current_time = time.time()
        self.done = False
        self.n += 1
        if self.n % 100 == 0:
                self.send_command_to_driver("Y")
        if uart1.any() >= (58):  # 48 bytes total
            data_from_uart1 = uart1.read()
            

            # Check if "status:" was found in the byte string
            


            try:
                status_position = data_from_uart1.find(b'status:')
                end_position = data_from_uart1.find(b"end")
                if status_position != -1 and end_position !=-1:
                    # Extract what's after "status:"
                    status_value = data_from_uart1[status_position + 8+1:end_position]
                    #print("Status value is:", status_value)
                    #print(len(status_value))
                    
                    if len(status_value) != 48:
                        print(len(status_value))
                        return
                else:
                    print("'status:' not found in data_from_uart1")
                    # raise not found exception
                    #raise ValueError
                    return



                unpacked_data = struct.unpack('<10fif', status_value)
                #print(unpacked_data)
                #print(unpacked_data)
                torque_l, torque_r, noTorque_l, noTorque_r, pos_l, pos_r, target_l, target_r, vel_l, vel_r, last_command, last_command_param = unpacked_data
                sent_status = status_list[unpacked_data[-2]]
                self.last_successful_comms = time.time()
            except Exception as e:
                #print("unpacking failed",e)
                #print(len(status_value))
                unpacked_data = [0,0,0,0,0,0,0,0,0,0,0,0]
                self.sent_status = "COMMS_ERROR"
                print("comms error")
                return False

            #print("sent status",sent_status)
            if sent_status in  motions: #check if target acquired
                error_l = pos_l - target_l
                error_r = pos_r - target_r
                #print("error:", error_l, error_r)
                done = abs(error_l) < POS_PRECISION and abs(error_r) < POS_PRECISION

            elif sent_status not in motions: #we don't know what state to expect
                done = True
                #done = abs(vel_l)< 0.1 and abs(vel_r) < 0.1 #for 

            else:
                done = False

            self.unpacked_data = unpacked_data
            #print(self.unpacked_data)
            self.sent_status = sent_status
            self.done = done
            self.wait_comms = False
            
            self.unpack_to_blackboard(unpacked_data)
            #print("unpacking to blackboard", self.blackboard)
            self.update_callbacks()
            #print(self.blackboard)
            #every 10th loop send 'H' to check if we are still connected
            
            return True

        else:
            if current_time - self.last_successful_comms > 5:  # 1 second timeout
                self.sent_status = "COMMUNICATION_TIMEOUT"
                
                return False
    def update_callbacks(self):  
        """will run when new data is available"""
        for id,callback in self.callbacks.items():
            callback(self.get_driver_status())
            
    def remove_callback(self, id):
        self.callbacks.pop(id, None)
    def add_callback(self, callback,id):
        self.callbacks[id] = callback

    def get_driver_status(self):
        # If the current action has a motion status and is done

        if self.sent_status in motions and self.done:
            return "SUCCESS"
        
        # If the current action has a motion status but is not done
        elif self.sent_status in motions and not self.done:
            return "EXECUTING"
        
        # If there's a communication error or timeout
        elif self.sent_status == "COMMS_ERROR" or self.sent_status == "COMMUNICATION_TIMEOUT":
            #print(self.sent_status)
            return "FAILURE"
            #return "EXECUTING"
        
        # Default or unknown status
        elif self.done:
            return "SUCCESS"
        return "EXECUTING"


class Action:
    def __init__(self, command, blackboard=None, communicator=None):
        self.command = command
        self.blackboard = blackboard
        self.status = Status.RUNNING
        self.communicator = communicator
        self.id = id(self) #unique id for each action
    def reset(self):
        self.status = Status.RUNNING
        self.blackboard.set('pending_command', None)
        self.communicator.remove_callback(self.id)
    def send_command(self):
        self.communicator.send_command_to_driver(self.command)
        #self.communicator.done = False
        #self.communicator.wait_comms =  True
        self.blackboard.set('pending_command', self.command)
        self.status = Status.RUNNING
    def check_command_completion(self, driver_status):
        """subscriber function for checking if command is completed"""
        
        
        status = driver_status
        #feedback = self.communicator.unpacked_data


        #if self.communicator.wait_comms:
        #    self.status = Status.RUNNING
        #    return    
        #print("completed status",status)
        #print(self.blackboard)
        # Check if the expected action matches the current action
        #if expected_action and self.sent_status != expected_action:
        #    self.status = Status.FAILURE
        #    return
        

        if status == 'EXECUTING':
            self.status = Status.RUNNING
        elif status == 'SUCCESS':
            #self.blackboard.set('pending_command', None)
            self.status = Status.SUCCESS
            #print(self.command, self.blackboard)
        elif status == 'FAILURE':
            #self.blackboard.set('pending_command', None)
            self.status = Status.FAILURE
        return status
    def execute(self):
        pending_command = self.blackboard.get('pending_command')

        if pending_command is None or pending_command!=self.command:
            #print(pending_command, self.command)
            #print("sending command" , self.command)
            self.send_command()
            self.communicator.add_callback(self.check_command_completion, self.id)
        
        if self.status == Status.SUCCESS or self.status == Status.FAILURE:
            self.blackboard.set('pending_command', None)
            self.communicator.remove_callback(self.id)
        #print("executed action",self.command,self.blackboard.get("pending_command"))
        return self.status

class ActionFromBlackboard(Action):
    def __init__(self,blackboard,communicator,default_command = "", key ="min_pos", command_prefix = "T"):
        super().__init__(default_command, blackboard, communicator)
        self.key = key
        self.command_prefix = command_prefix
        self.command = self.construct_command()
        
        #self.expected_result = expected_result
        #self.status = Status.RUNNING
    def check_command_completion(self, driver_status):
        status = super().check_command_completion(driver_status)

        #additional test based on comparoson with expected result
        #convert command to position vector and compare with expected command in format "C pos_l pos_r"
        try:
            C,exp_l,exp_p = self.command.split(" ")
        except ValueError as e:
                print("error",e)
                self.status = Status.RUNNING
        errl = abs(self.blackboard["pos_l"]-float(exp_l))

        errp = abs(self.blackboard["pos_l"]-float(exp_l))
        if (errl + errp)/2 > POS_PRECISION:
            self.status = Status.RUNNING
            print("too large error")


        #print(status)
        print(self.status)
    def construct_command(self):
        command = f"{self.command_prefix}{self.blackboard.get(self.key)}"

        return command
    def execute(self):
        self.command = self.construct_command()
        status = super().execute()
        #print("executing",self.command, self.status, self.command)
        #self.communicator.send_command_to_driver(self.command)
        #return Status.SUCCESS
        return status
        
    
class Status:
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"
    RUNNING = "RUNNING"


class Blackboard:
    def __init__(self):
        self.data = {}

    def get(self, key, default=None):
        return self.data.get(key, default)

    def set(self, key, value):
        self.data[key] = value

    def clear(self):
        self.data.clear()
    def __repr__(self):
        return self.data
    def __getitem__(self, key):
        return self.data.get(key, None)
    def has_key(self, k):
        return k in self.data
    # Define the method to set an item like a dict
    def __setitem__(self, key, value):
        self.data[key] = value


class Node:
    def __init__(self, blackboard=None):
        self.children = []
        self.parent = None
        self.status = Status.RUNNING
        self.blackboard = blackboard

    def add_child(self, node):
        self.children.append(node)
        node.parent = self
    def reset_recursive(self):
        self.status = Status.RUNNING
        for child in self.children:
            child.reset_recursive()

    def execute(self):
        pass


class CompositeNode(Node):
    def __init__(self, blackboard=None):
        super().__init__(blackboard)


class Selector(CompositeNode):
    
    def execute(self):
        for child in self.children:
            status = child.execute()
            if status != Status.FAILURE:
                return status
        return Status.FAILURE


class Sequence(CompositeNode):
    def execute(self):
        for child in self.children:
            status = child.execute()
            if status != Status.SUCCESS:
                return status
        return Status.SUCCESS

class SequenceWithMemory(Sequence):  # Inherits from Sequence now
    def __init__(self, blackboard=None):
        super().__init__(blackboard)  # Initialize the parent class
        self.memory = 0  # Initialize the memory to the first child

    def reset_recursive(self):
        self.memory = 0
        super().reset_recursive()  # Call the parent class's reset_recursive method

    def execute(self):
        # Start from the last remembered node
        for i in range(self.memory, len(self.children)):
            status = self.children[i].execute()
            # If any child returns FAILURE, the whole node returns FAILURE
            if status == Status.FAILURE:
                self.memory = 0  # Reset memory for the next tick
                return Status.FAILURE
            # If the child returns RUNNING, remember it and return RUNNING
            elif status == Status.RUNNING:
                self.memory = i  # Remember the currently running child
                return Status.RUNNING
        # If all children return SUCCESS, reset memory and return SUCCESS
        self.memory = 0
        return Status.SUCCESS


class LeafNode(Node):
    def __init__(self, action, blackboard=None):
        super().__init__(blackboard)
        self.action = action
    def reset_recursive(self):
        self.action.status = Status.RUNNING
        self.action.reset()

    def execute(self):
        self.status = self.action.execute()
        
        
        return self.status



class DecoratorNode(Node):
    def __init__(self, child, blackboard=None):
        super().__init__(blackboard)
        self.add_child(child)

class RunifBlackboard(DecoratorNode):
    def __init__(self, child, key, value, blackboard=None):
        super().__init__(child, blackboard)
        self.key = key
        self.value = value
    def execute(self):
        if self.blackboard.get(self.key) == self.value:
            return self.children[0].execute()
        else:
            return Status.FAILURE

class Inverter(DecoratorNode):
    def execute(self):
        status = self.children[0].execute()
        if status == Status.SUCCESS:
            return Status.FAILURE
        elif status == Status.FAILURE:
            return Status.SUCCESS
        else:
            return Status.RUNNING
class AlwaysFail(DecoratorNode):
    def execute(self):
        status = self.children[0].execute()
        return Status.FAILURE      
class Repeater(DecoratorNode):
    def __init__(self, child, n, blackboard=None, key=None):
        super().__init__(child, blackboard)
        self.repeat_count = n  # Number of times to repeat the child
        self.current_count = 0  # Current count
        self.key = key
    def reset_recursive(self):
        self.current_count = 0
        super().reset_recursive()
    def execute(self):
        status = self.children[0].execute()
        if self.key:
            self.blackboard[self.key] = self.current_count
        # If the child node is successful, increase the counter
        if status == Status.SUCCESS:
            self.current_count += 1
            #print("repeater count",self.current_count)
            # If the count reaches the required number of repetitions, return SUCCESS
            if self.current_count >= self.repeat_count:
                self.current_count = 0  # Reset counter for future runs
                self.children[0].reset_recursive()  # Reset child node
                return Status.SUCCESS
            else:
                # Keep running since we haven't reached the required number of repetitions
                return Status.RUNNING
        elif status == Status.FAILURE:
            # Reset counter and return FAILURE
            self.current_count = 0
            self.children[0].reset_recursive()
            return Status.FAILURE
        else:
            return Status.RUNNING

import time

class TimeLimit(DecoratorNode):
    def __init__(self, child, time_limit, blackboard=None, key=None):
        super().__init__(child, blackboard)
        self.time_limit = time_limit  # Time limit in seconds
        self.start_time = None  # Initialize start time to None
        self.key = key
    def reset_recursive(self):
        self.start_time = None


        super().reset_recursive()
    def execute(self):
        # Start the timer the first time this method is called
        if self.start_time is None:
            self.start_time = time.time()

        elapsed_time = time.time() - self.start_time

        if self.key:
            self.blackboard[self.key] = elapsed_time

        #print(elapsed_time)
        # Check if the time limit has been reached
        if elapsed_time >= self.time_limit:
            self.start_time = None  # Reset start time for future runs
            self.children[0].reset_recursive() # Reset child node
            return Status.SUCCESS

        # Execute the child node
        status = self.children[0].execute()

        if status == Status.FAILURE:
            self.start_time = None  # Reset start time for future runs
            return Status.FAILURE
        else:
            return Status.RUNNING
        

class ExpectedResult:
    fields = ["torque_l", "torque_r", "noTorque_l", "noTorque_r", "pos_l", "pos_r", "target_l", "target_r", "vel_l", "vel_r", "last_command", "last_command_param"]
        
    def __init__(self, **kwargs):
        self.expectations = kwargs # Dictionary of expected values

    def compare_to_feedback(self,blackboard):
        # Define the field names in the same order as in the tuple
        
        # Convert unpacked_data tuple to dictionary
        #unpacked_dict = dict(zip(fields, unpacked_data))
        unpacked_dict = blackboard #blackboard keeps the state of the robot
        for key, expected_value in self.expectations.items():
            if key in unpacked_dict:
                actual_value = unpacked_dict[key]
                
                # Check if the expected_value is a tuple (meaning it has an accuracy level)
                if isinstance(expected_value, tuple):
                    expected_value, accuracy = expected_value
                    
                    # Check if both actual and expected values are numeric
                    if isinstance(actual_value, (int, float)) and isinstance(expected_value, (int, float)):
                        if abs(actual_value - expected_value) > accuracy:
                            print(f"Mismatch: {key} - expected {expected_value} +/- {accuracy}, got {actual_value}")
                            return False
                    else:
                        if actual_value != expected_value:
                            print(f"Mismatch: {key} - expected {expected_value}, got {actual_value}")
                            return False
                
                # If not a tuple, do a strict comparison
                elif actual_value != expected_value:
                    print(f"Mismatch: {key} - expected {expected_value}, got {actual_value}")
                    return False
            else:
                print(f"Key {key} not found in unpacked_data")
                return False
        print("All fields match the expected values.")
        return True
        
        
class BlueToothCommunicator():
    def __init__(self, blackboard = None) -> None:
        ble = bluetooth.BLE()
        self.uart = BLESimplePeripheral(ble, name=setup_comms.device_name)
        self.uart.on_write(self.on_rx)

        self.message = ""
        self.n = 0
        self.callbacks = {}

        if blackboard:
            self.blackboard = blackboard
        else:
            self.blackboard = Blackboard()
        self.possible_callbacks = ["ll","pl","lp","pp","lo","po","re","st","umin","umax","uzero",
                                   "uruchom","stopm","wylacz","wl","H"]
        
        
    def form_feedback(self):
        # format ['state','min_l','max_l','min_r','max_r','pos_l','pos_r']
        feedback = [self.blackboard["elapsed_time"],
                    self.blackboard["min_l"],
                    self.blackboard["max_l"],
                    self.blackboard["min_r"],
                    self.blackboard["max_r"],
                    self.blackboard["pos_l"],
                    self.blackboard["pos_r"],
                    self.blackboard["repeat"]]
        
        return feedback
    def on_rx(self, v):
        #print(v)
        if self.decode_callbacks(v):
            self.update_callbacks(self.blackboard["last_command"])

    def decode_callbacks(self,v):

        try:
            v = v.decode("utf-8").rstrip("\x00")
        except:
            return False

        if v in self.possible_callbacks:
            self.blackboard["last_command"] = v
            return True
        else:
            return False   
        
    def update_callbacks(self,v):  
        """will run when new data is available"""
        for id,callback in self.callbacks.items():
            callback(v)
            
    def remove_callback(self, id):
        self.callbacks.pop(id, None)
    def add_callback(self, callback,id):
        self.callbacks[id] = callback


    def send_feedback(self):


        if not self.uart.is_connected():
            return
        sensors = self.form_feedback()
        spec = "8h"
        try:
            #print(sensors)
            sensors = [-1000 if math.isnan(s) else round(s) for s in sensors]
            #print("sending",sensors)
            if len(self.uart._connections)>0:
                self.uart.send(struct.pack(spec, *sensors))
            else:
                print("no connections")
        except Exception as e:
            try:
                print("exception when sending")
                print(e)
                conn = self.uart._connections[0]  # hopefully one connection
                self.uart._ble.gap_disconnect(conn)
                self.uart._connections.clear()

            except Exception as ee:
                print(ee)

            print("could not send", e, sensors)
