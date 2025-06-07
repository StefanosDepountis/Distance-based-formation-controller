import time
import math
import numpy as np
import threading
import csv

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.commander import Commander
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

"""This code is based on the cflib and is implementing a distance-based formation controller for crazyflies and LPS system."""

"""Add here your drone's uris and preferably use a capital letter (e.g. 'E':'radio//0/80/2M/EE)"""
uris_dict = {
    'A':'radio://0/80/2M/EA',
    'B':'radio://0/80/2M/EB',
    'C':'radio://0/80/2M/EC',
    'D':'radio://0/80/2M/ED'
}

class Drone:
    """The main class which keeps all the informations for a drone. Many functions are connection, logging, take off, land, move.
    As an addition there is a function to reset the estimators, although in experiments does not seem to make any big difference"""
    def __init__(self, name, uri):
        self.name = name
        self.uri = uri
        self.scf = None
        self.log_config = None
        self.position_x = 0
        self.position_y = 0
        self.position_z = 0
        self.position = [self.position_x, self.position_y,self.position_z]
        self.x_holder = []
        self.y_holder = []
        self.z_holder = []
        self.logging_size = 10 #number of values to hold until we calculate the mean value
        self.i = 0
        self.commander = None
        self.take_off_height = 0.25#used to make a small take off
        self.take_off_mode = False#used to change modes from take off to move
        self.input = [0.0, 0.0, 0.0]
        self.initial_yaw = 0. #in deegres
        self.initial_z_offset = 0.
    
    def open_link(self):
        self.scf = SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache='./cache'))
        self.scf.open_link()

    def close_link(self):
        self.scf.close_link()

    def setup_log(self):
        # Set up the Loco Positioning System for Crazyflie
        #this take a value every 10 ms. We need 10 values = 100ms. For initial position we use 100 values = 1 sec
        self.log_config = LogConfig(name = 'Position' + self.name, period_in_ms=10)
        # self.log_config.add_variable('kalman.stateX', 'float')
        # self.log_config.add_variable('kalman.stateY', 'float')
        # self.log_config.add_variable('kalman.stateZ', 'float')
        self.log_config.add_variable('stateEstimate.x', 'float')
        self.log_config.add_variable('stateEstimate.y', 'float')
        self.log_config.add_variable('stateEstimate.z', 'float')


        # Start the log configuration for Crazyflie
        self.scf.cf.log.add_config(self.log_config)

        def log_data_callback(timestamp, data, logconf):
            if (not self.i % self.logging_size) and self.i>0:
                #Maybe use it without moving average. check both
                self.position_x = np.mean(self.x_holder) 
                self.position_y = np.mean(self.y_holder)
                self.position_z = np.mean(self.z_holder)

                self.x_holder.clear()
                self.y_holder.clear()
                self.z_holder.clear()
                self.update_position()
                self.i = -1
            else:
                # self.x_holder.append(data['kalman.stateX'])
                # self.y_holder.append(data['kalman.stateY'])
                # self.z_holder.append(data['kalman.stateZ'])
                self.x_holder.append(data['stateEstimate.x'])
                self.y_holder.append(data['stateEstimate.y'])
                self.z_holder.append(data['stateEstimate.z'])
            self.i = self.i + 1
        
        self.log_config.data_received_cb.add_callback(log_data_callback)
  
    def start_logging(self):
        if self.log_config:
            self.log_config.start()
            
    def stop_logging(self):
        if self.log_config:
            self.log_config.stop()

    def update_position(self):
        self.position = [self.position_x, self.position_y,self.position_z]

    def setup_commander(self):
        self.commander = Commander(self.scf.cf)

    def take_off(self, height = None):
        """send_hover_setpoint(self, vx, vy, yawrate, zdistance). Take off function is very unstable, must fix it."""
        print("Take off {}".format(self.name))
        self.take_off_mode = True
        # if height == None: height = self.take_off_height
        self.commander.send_hover_setpoint(0.0, 0.0, 0.0, 0.3)
        time.sleep(1)
        start = time.time()
        while time.time() < start + 4:
            z_vel = np.maximum(np.minimum((self.take_off_height) - self.position_z, 0.3), -0.3)
            self.commander.send_velocity_world_setpoint(0., 0., z_vel, 0.)
            time.sleep(0.1)
        self.commander.send_velocity_world_setpoint(0., 0., 0., 0.)
        self.take_off_mode = False
        # self.commander.send_hover_setpoint(0.0, 0.0, 0.0, 0.5) 
    
    def land(self):
        print("Landing {}".format(self.name))
        start = time.time()
        while time.time() < start + 2:
            self.commander.send_velocity_world_setpoint(0., 0., -0.3, 0.)
        self.commander.send_stop_setpoint()
        pass

    def move(self):
        """send_velocity_world_setpoint(self, vx, vy, vz, yawrate). Used to parse the input"""
        # print("Drone {} gets [{}, {}]".format(self.name, self.input[0], self.input[1]))
        # self.commander.send_velocity_world_setpoint(0.0, 0.0, 0.0, 0.0)
        self.commander.send_velocity_world_setpoint(self.input[0], self.input[1], 0.0, 0.0)

    def wait_for_position_estimator(self):
        scf = self.scf.cf
        print('Waiting for estimator to find position...')

        log_config = LogConfig(name='Pos_estimator', period_in_ms=500)
        log_config.add_variable('kalman.varPX', 'float')
        log_config.add_variable('kalman.varPY', 'float')
        log_config.add_variable('kalman.varPZ', 'float')
        
        var_y_history = [1000] * 10
        var_x_history = [1000] * 10
        var_z_history = [1000] * 10

        threshold = 0.001

        with SyncLogger(scf, log_config) as logger:
            for log_entry in logger:
                data = log_entry[1]

                var_x_history.append(data['kalman.varPX'])
                var_x_history.pop(0)
                var_y_history.append(data['kalman.varPY'])
                var_y_history.pop(0)
                var_z_history.append(data['kalman.varPZ'])
                var_z_history.pop(0)

                min_x = min(var_x_history)
                max_x = max(var_x_history)
                min_y = min(var_y_history)
                max_y = max(var_y_history)
                min_z = min(var_z_history)
                max_z = max(var_z_history)

                # print("{} {} {}".
                #       format(max_x - min_x, max_y - min_y, max_z - min_z))

                if (max_x - min_x) < threshold and (
                        max_y - min_y) < threshold and (
                        max_z - min_z) < threshold:
                    break

    def reset_estimator(self):
        self.scf.cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        self.scf.cf.param.set_value('kalman.resetEstimation', '0')
        self.wait_for_position_estimator()

    def set_initial_position(self, x, y, z, yaw_deg):
        """Used to set the initial yaw based on deegres (positive x is 0, position y is 90)"""
        self.scf.cf.param.set_value('kalman.initialX', x)
        self.scf.cf.param.set_value('kalman.initialY', y)
        self.scf.cf.param.set_value('kalman.initialZ', z)

        yaw_radians = math.radians(yaw_deg)
        self.scf.cf.param.set_value('kalman.initialYaw', yaw_radians)

class Formation_control:
    """The main class for distance-based fomartion control. Has a setup function for changing the desired graph (desired distances and connection graph).
    Also it has the input generator based on the controller presented in thesis. Finally termination criterio has not been developed because we terminate it manually."""
    def __init__(self, drones):
        self.n = len(drones)
        self.p = []
        self.names = list(drones.keys())
        self.edge_matrix = np.ones((self.n,self.n))
        self.desired_matrix = np.ones((self.n,self.n))
        self.pNorm_squared_matrix = np.zeros((self.n,self.n))

        self.input_gen = np.zeros((3,self.n))
        self.termination_condition = False

    def input_generator(self, drones):
        """Apply formation controller ui = - Σ2*(βij^2 - dij^2)/βij^2 * (qi - qj)
        βij is the distance between drones βij = ||pi - pj|| 
        dij is the desired distance"""
        # start = time.time()
        n = self.n
        self.input_gen = np.zeros((3,self.n))
        #Get the position of each drone
        for i, drone in enumerate(drones.values()):
            self.p[i] = (np.array(drone.position)).reshape((3,1))
    
        for i in range(n):
            for j in range(n):
                if i!=j:
                    # self.pNorm_squared_matrix[i,j] = np.linalg.norm(self.p[i][0:2] - self.p[j][0:2])**2 # you may use this for a faster convergence. Change also in the self.input_gen
                    self.pNorm_squared_matrix[i,j] = np.linalg.norm(self.p[i][0:2] - self.p[j][0:2])


        kp = 0.1
        for i in range(n):
            for j in range(n):
                if i!=j:
                    # self.input_gen[:,i:i+1] += -kp * self.edge_matrix[i,j] * ((self.pNorm_squared_matrix[i,j] - self.desired_matrix[i,j]**4)/self.pNorm_squared_matrix[i,j]) * (self.p[i] - self.p[j])
                    self.input_gen[:,i:i+1] += -kp * self.edge_matrix[i,j] * ((self.pNorm_squared_matrix[i,j] - self.desired_matrix[i,j]**2)/self.pNorm_squared_matrix[i,j]) * (self.p[i] - self.p[j])

            for k in range(len(self.input_gen[:,i:i+1])):#do not give very small inputs, instead give zero
                if abs(self.input_gen[k,i:i+1]) < 0.1 : self.input_gen[k,i:i+1] = 0 
            self.input_gen[:,i:i+1] = np.maximum(np.minimum(self.input_gen[:,i:i+1],0.8),-0.8)


        for i in range(len(self.names)):
            # print('We give at drone {} an input [{}, {}]'.format(drones[self.names[i]].name, self.input_gen[0,i], self.input_gen[1,i]))
            drones[self.names[i]].input = self.input_gen[:,i]
    
        pass

    def setup_desired_formation(self):
        """The drones are sorted by name. So 1,2,3,4 are pointing the a sorted list of letter A-D(for 4 drones).
        Also the desired distance is inside the np.sqrt(desired). The reason is the controller we are using"""
        n = self.n
        for i in range(n):
            """For different graphs we have to change the edge_matrix by hand"""
            self.edge_matrix[i,i] = 0.
            self.desired_matrix[i,i] = 0.

        # self.edge_matrix = np.array([[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]])
        #TO CHANGE THE DESIRED VALUE CHANGE THE VALUE INSIDE THE SQRT
        if n >= 2:#default for 2 drones distances of 1.5
            self.desired_matrix[0, 1] = self.desired_matrix[1, 0] = np.sqrt(1)  # desired12
            p1 = (np.array(drones[self.names[0]].position)).reshape((3,1))
            p2 = (np.array(drones[self.names[1]].position)).reshape((3,1))
            self.p.append(p1)
            self.p.append(p2)

        if n >= 3:#default for 3 drones distances of 1.5
            self.desired_matrix[0, 2] = self.desired_matrix[2, 0] = np.sqrt(1.5)  # desired13
            self.desired_matrix[1, 2] = self.desired_matrix[2, 1] = np.sqrt(1.5)  # desired23
            self.desired_matrix[0, 1] = self.desired_matrix[1, 0] = np.sqrt(1.5)  # desired12
            p3 = (np.array(drones[self.names[2]].position)).reshape((3,1))
            self.p.append(p3)

        if n >= 4:#default for 4 drones distances of 1.5
            #Below we create a square with edge equal to 1 
            self.desired_matrix[0, 2] = self.desired_matrix[2, 0] = np.sqrt(1)  # desired13
            self.desired_matrix[1, 2] = self.desired_matrix[2, 1] = np.sqrt(np.sqrt(2))  # desired23 #here desired is sqrt(2)
            self.desired_matrix[1, 3] = self.desired_matrix[3, 1] = np.sqrt(1)  # desired24
            self.desired_matrix[0, 3] = self.desired_matrix[3, 0] = np.sqrt(np.sqrt(2))  # desired14
            self.desired_matrix[2, 3] = self.desired_matrix[3, 2] = np.sqrt(1)  # desired34
            self.desired_matrix[0, 1] = self.desired_matrix[1, 0] = np.sqrt(1)  # desired12
            p4 = (np.array(drones[self.names[3]].position)).reshape((3,1))
            self.p.append(p4)
        pass

    def check_termination(self, drones):
        # np.power(self.desired_matrix,2)
        pass

class Move_to_target:
    """This class implements a simple P controller to move the drones to desired targets"""
    def __init__(self, drones):
        self.n = len(drones)
        self.p = []
        self.desired_targets = {}
        self.input_gen = {}
        self.kp = 0.2
        self.termination_criterio = False 
        self.termination_holder = {}
        pass

    def input_generator(self):
        for drone in drones: 
            pos = (np.array(drones[drone].position)).reshape(3,1)
            delta_p = self.desired_targets[drone] - pos
            
            if abs(np.linalg.norm(self.desired_targets[drone][0][0:2] - pos[0][0:2]))<0.1:
                self.termination_holder[drone] = True

            else: self.termination_holder[drone] = False
            self.input_gen[drone] = self.kp * delta_p
            drones[drone].input = ((self.input_gen[drone]).reshape(3,)).tolist()
            self.check_termination()
        pass

    def setup_desired_targets(self):
        for drone in drones:
            print("For drone " + drone[-1])
            print('Give x_desired')
            x = float(input())
            print('Give y_desired')
            y = float(input())
            self.desired_targets.update({drone[-1]: np.array([[x,y,0.]]).T})

            self.input_gen.update({drone[-1]:np.zeros((3,1))})

            self.termination_holder.update({drone:False})
        pass

    def check_termination(self):
        if all(self.termination_holder.values()):
            self.termination_criterio = True
            pass
        pass

class CSVLogger:
    """We use this class to log the desired variable to a csv file for further analysis. You may change to a preferable logging configuration by changing the positions_dict"""
    def __init__(self, filename, names):
        self.filename = filename
        self.names = names  
        self.num_agents = len(names)

    def __enter__(self):
        self.file = open(self.filename, mode='w', newline='')
        self.writer = csv.writer(self.file)

        self.writer.writerow([f'Agent_{name}' for name in self.names])
        return self 

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def log(self, positions_dict):
        """
        positions_dict: a dictionary {agent_name: [x, y, z], ...}
        Any missing agent will leave that cell blank.
        """
        row = ["" for _ in range(self.num_agents)]
        for i, name in enumerate(self.names):
            if name in positions_dict:
                row[i] = str(positions_dict[name])
        self.writer.writerow(row)

    def close(self):
        self.file.close()

def move_to_target(drones):
    """algorithm to move to a specific target each drone independently"""
    for drone in drones.values():#connect each drone
        drone.open_link()
        drone.setup_log()
        drone.setup_commander()
        time.sleep(2.)
        #Take some time to calculate the initial position
        for drone in drones.values():
            drone.logging_size = 100
        drone.start_logging()
        time.sleep(2.)
    
    #Use this to change the initial yaw of the drone
    # drones['A'].initial_yaw = 0.
    # drones['B'].initial_yaw = 0.
    # drones['C'].initial_yaw = 0.
    # drones['D'].initial_yaw = 0.
    # drones['E'].initial_yaw = 90
    
    print("Starting Position:")
    for drone in drones.values():
        print("Drona {} -> pos: [{}, {}, {}]".format(drone.name,drone.position[0],drone.position[1],drone.position[2]))
  
    #MAY NOT BE NECESSARY. Check if really change the initial position
    for drone in drones.values():
        drone.set_initial_position(drone.position[0], drone.position[0], 0., drone.initial_yaw)
        drone.reset_estimator()

    for drone in drones.values():
        print("Drona {} -> pos: [{}, {}, {}]".format(drone.name,drone.position[0],drone.position[1],drone.position[2]))
    
    csv_logger.log({drone.name: drones[drone.name].position for drone in drones.values()})
    #return to 10 logging size because 100 is a lot 
    for drone in drones.values():
        drone.logging_size = 10

    #setup the desired targets
    move2t = Move_to_target(drones)
    move2t.setup_desired_targets()
    
    print("insert desired height(preferably 0.6m)")
    height_ask = float(input())
    for drone in drones.values():
        drone.take_off_height = height_ask

    time.sleep(2.)

    print("READY TO FLY")
    print("PROCEED? press any (zero exit)")
    answer = float(input())
    if answer == 0:
        exit()

    ######FROM HERE WE MUST USE THREADING FOR EACH DRONE
    def _move_to_target_sequence(drone, move_controller):
        try:
            drone.take_off()
            
            time.sleep(2.)
            
            while not move_controller.termination_criterio:
                # Only generate inputs once (in main thread or synchronized)
                drone.move()  # Each drone moves independently
                time.sleep(0.2)  # Small delay to prevent CPU overload
            
            drone.land()
        except Exception as e:
            print(f"Error in drone {drone.name} thread: {str(e)}")

    # Create and start threads
    threads = []
    for drone in drones.values():
        thread = threading.Thread(
            target=_move_to_target_sequence,
            args=(drone, move2t),
            name=f"Drone_{drone.name}_Thread"
        )
        thread.daemon = True  # Ensures threads exit when main thread exits
        threads.append(thread)
        thread.start()

    # Main thread can monitor or control the operation
    try:
        while any(thread.is_alive() for thread in threads):
            # Update termination criteria based on some condition
            move2t.input_generator()  # If needed, but be careful with thread safety
            csv_logger.log({drone.name: drones[drone.name].position for drone in drones.values()})
            time.sleep(0.4)
            
    except KeyboardInterrupt:
        print("\nEmergency stop requested!")
        move2t.termination_criterio = True
    
    # Wait for all threads to finish
    for thread in threads:
        thread.join(timeout=10.0)  # Give each thread 10 seconds to finish

    move2t.termination_criterio = True # When we reach here we must stop the threads 
    time.sleep(2.)
    ######
    for drone in drones.values():
        drone.stop_logging()
        drone.close_link()

    print("Final Position:")
    for drone in drones.values():
        print("Drona {} -> pos: [{}, {}, {}]".format(drone.name,drone.position[0],drone.position[1],drone.position[2]))
    pass

def just_logging(drones):
    """A code for a simple log function. May use for as many drones as you want. You can change the time of logging and the number of values used for the a mean average filter."""
    for drone in drones.values():
        drone.open_link()
        drone.setup_log()
        time.sleep(2.)
        drone.start_logging()

    start = time.time()
    next_print = start  # Initialize the next print time
    while time.time() < start + 5:
        current_time = time.time()
        if current_time >= next_print:  # Check if 1 second has passed
            for drone in drones.values():
                print(f"Drona {drone.name} -> pos: [{drone.position[0]}, {drone.position[1]}, {drone.position[2]}]")
            next_print = current_time + 1  # Schedule next print in 1 second
        # Optional: Small sleep to reduce CPU usage (adjust as needed)
        time.sleep(0.001)

    for drone in drones.values():
        drone.stop_logging()
        drone.close_link()    

    pass

def formation_control(drones):
    """The main algorithm for distance-based formation control. For later work we must update the termination condition, may be a time condition or a error condition.
    Right now the algorithm terminates by pressing ctrl+C manually."""
    for drone in drones.values():
        drone.open_link()
        drone.setup_log()
        drone.setup_commander()
    time.sleep(3.)

    #Take some time to calculate the initial position
    for drone in drones.values():
        drone.logging_size = 100
        drone.start_logging()
    time.sleep(2.)
        
    #Use this to change the initial yaw of the drone
    # drones['A'].initial_yaw = 0.
    # drones['B'].initial_yaw = 0.
    # drones['C'].initial_yaw = 0.
    # drones['D'].initial_yaw = 0.
    
    print("Starting Position:")
    for drone in drones.values():
        print("Drona {} -> pos: [{}, {}, {}]".format(drone.name,drone.position[0],drone.position[1],drone.position[2]))
        drone.initial_z_offset = drone.position[2]

    #MAY NOT BE NECESSARY. Check if really change the initial position
    #As seen in the first experiments for four drones it couldnt reset_estimator successfully so we continue without it.

    # for drone in drones.values():
        # drone.set_initial_position(drone.position[0], drone.position[0], 0., drone.initial_yaw)
        # drone.reset_estimator()
    
    # time.sleep(2.)
    # print("Position after resseting:")
    # for drone in drones.values():
    #     print("Drona {} -> pos: [{}, {}, {}]".format(drone.name,drone.position[0],drone.position[1],drone.position[2]))
    

    csv_logger.log({drone.name: drones[drone.name].position for drone in drones.values()})
    for drone in drones.values():#return to 10 logging size because 100 is a lot 
        drone.logging_size = 10

    formation = Formation_control(drones)
    formation.setup_desired_formation()#to change the setup you must change in the class Formation_control
    
    print("insert desired height")
    height_ask = float(input())

    for drone in drones.values():
        drone.take_off_height = height_ask

    time.sleep(2.)

    print("READY TO FLY")
    print("PROCEED? press any (zero exit)")
    answer = float(input())
    if answer == 0:
        exit()

    ######FROM HERE WE MUST USE THREADING FOR EACH DRONE
    def _formation_sequence(drone, formation_controller):
        try:
            drone.take_off()
            
            
            while not formation_controller.termination_condition:
                # Only generate inputs once (in main thread or synchronized)
                drone.move()  # Each drone moves independently
                time.sleep(0.1)  # Small delay to prevent CPU overload
            
            drone.land()
            drone.stop_logging()
            drone.close_link()
        except Exception as e:
            print(f"Error in drone {drone.name} thread: {str(e)}")
    
    # Create and start threads
    threads = []
    for drone in drones.values():
        thread = threading.Thread(
            target=_formation_sequence,
            args=(drone, formation),
            name=f"Drone_{drone.name}_Thread"
        )
        thread.daemon = True  # Ensures threads exit when main thread exits
        threads.append(thread)
        thread.start()

    #TODO start this after the take_off
    time.sleep(2.)
    try:
        while any(thread.is_alive() for thread in threads):
            # Update termination criteria based on some condition. Right now we terminate manually.
            formation.input_generator(drones)  # If needed, but be careful with thread safety
            csv_logger.log({drone.name: drones[drone.name].position for drone in drones.values()})          
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nEmergency stop requested!")
        formation.termination_condition = True
    
    # Wait for all threads to finish
    for thread in threads:
        thread.join(timeout=20.0)  # Give each thread 20 timeout for threads. After this we manually stop the threads and land the drones.

    formation.termination_condition = True # When we reach here we must stop the threads 
    time.sleep(2.)

    for drone in drones.values():
        drone.stop_logging()
        drone.close_link()

    print("Final Position:")
    for drone in drones.values():
        print("Drona {} -> pos: [{}, {}, {}]".format(drone.name,drone.position[0],drone.position[1],drone.position[2]))
    pass

if __name__ == "__main__":
    cflib.crtp.init_drivers() #for drives initial
    choose = ['A', 'B', 'C', 'D'] #choose the drones you are using. must be in a list contain a capital letter. you may change based o nthe uris_dict
    choose.sort()
    uris = set()
    
    drones = {}
    for name in choose:# Create drone instances
        drones.update({name:Drone(name,uris_dict[name])})

    #call the desired function (algorithm)
    with CSVLogger('output_test_square_4.csv', choose) as csv_logger:
        # just_logging(drones) 
        # move_to_target(drones)
        formation_control(drones)
        print('FINISHED')