'''
Motor Control
Vasilije Rakcevic 
'''
import controlmodule as mm
import time
from threading import Timer
import matplotlib.pyplot as plt
import re 
import numpy as np
import traceback
import math


FILENAME = '/tmp/motor_control.txt'
time_triggered = False
POS_THRESHOLD = 0.02
MAX_STP = 100 # maximum number of samples to plot
DT = 0.0035
MOTOR_NAMES = ["Hip", "Knee", "Ankle"]
MOTOR_IDS = [1, 2, 3]
# p_des: {}\t ,p_actual: {}\r\n

gen_metch = '[0-9A-Za-z_\s()]'
reg_template = re.compile(r"%s*:(?P<first>[0-9-\.]*)%s*:(?P<second>[0-9-\.]*)\s*" % (gen_metch, gen_metch))


def terminate():
    global time_triggered
    time_triggered = True
    print("Time triggered")


def plot_data_from_file(filename):
    # global counter //if live plot is used than should be declared global 
    counter = 0
    x = []
    y = []
    fdata = open(filename)
    lines = fdata.readlines()
    numl = len(lines)

    if (numl > MAX_STP) and not (MAX_STP == 0):
        numl = MAX_STP

    for line in lines[-numl:]:
        # metch the data
        m_data = reg_template.match(line)
        # print(diff)
        # check if line corresponds to data line
        if m_data:
            # add next val to x axis
            x += [counter]
            # increase counter 
            counter += 1
            # get dictionary of matched groups
            d = m_data.groupdict() 
            # update data
            y += [[float(d['first']), float(d['second'])]]
    
    # close file
    fdata.close()

    # format plotting
    plotxy(x, np.array(y))


def plotxy(x,y, id, labels = None, title = None, ylabels = None, subplots = True):
    # # clear the current axis
    # plt.cla()      
    
    colors = ['orange','blue','brown','purple'] 
    
    if not subplots:
        plt.figure()
        for i in range(len(y[0])):   
            if labels == None:
                lbl = "Dat: {}".format(i)
            else:
                lbl = labels[i]

            plt.plot(x, y[:,i], label=lbl, color = colors[i])

        # axis = plt.gca()
        # axis.set_ylim(-10000, 10000)
        plt.legend(loc='lower right')
        plt.xlabel('time [s]')
        plt.ylabel('y') 
        # plt.tight_layout()
    else:
        fig, axs = plt.subplots(len(y[0][0])-1, len(y), sharex='col')

        for j in range (len(y)):
            for i in range(1,len(y[0][0])):   
                if labels == None:
                    lbl = "Dat: {}".format(i)
                else:
                    lbl = labels[i]
                    
                if ylabels == None:
                    ylbl = ["y{}".format(i+1)]
                else:
                    ylbl = ylabels[i-1]   

                axs[i-1, j].plot(x, y[j][:,i], label=lbl, color = colors[i])
                axs[i-1, j].set_ylabel(ylbl)
                
                if i == 1:
                    if labels == None:
                        lbl = "Dat: {}".format(0)
                    else:
                        lbl = labels[0]
                    axs[i-1, j].plot(x, y[j][:,0], label=lbl, color = colors[0]) # plot 0 values together with first valuess
                    axs[i-1, j].title.set_text('{} Actuator'.format(MOTOR_NAMES[id[j]-1]))

                axs[i-1, j].grid(True)
                axs[i-1, j].legend(loc='upper right')
                

            if not title == None:
                fig.suptitle(title)
            axs[len(y[0][0])-2, j].set_xlabel('time [s]')
        
        fig.tight_layout()
        plt.subplots_adjust(wspace=0.2, hspace=0)
    


def run_motors(mmc, id, runtime, pdes, vdes, pv_gains = [0.,0.], zero_pos=False, m_check = False, tracking = False, loop_dt = 1., logfile='/tmp/log_controller.txt'):
    '''
    mmc     -   Instance of the controller
    id      -   Motor ids                                       [1,m_ids]
    runtime -   Time intervals                                  [1, ev_count] or [None]
    pdes    -   Desired position for each time interval         [ev_count, m_ids]
    vdes    -   Desired speed for each time interval            [1, m_ids]
    pv_gains-   Position and velocity gains                     [1, 2]
    zero_pos-   Require to zero out positions of the motors
    m_check -   Check the motor current values. If enabled, pdes, vdes, pv_gains are ignored - Motor should not move
    loop_dt -   When runtime not provided every loop iteration execution will be increased by loop_dt
    logfile -   File for logging 
    '''
    if not (    ((len(runtime) == len(pdes) and tracking == False) or (len(runtime) == 1 and tracking == True)) and 
                len(pdes[0]) == len(id) == len(vdes)and
                len(pv_gains) == 2 and
                isinstance(id, list) and 
                isinstance(runtime, list) and 
                isinstance(vdes, list) and 
                isinstance(pv_gains, list)
            ):

        raise ValueError("\nFunction run_motors() called with wrong values!\n\tPlease check function description.\n\tExiting ...\n")

    # f = open(logfile, "w")
    global time_triggered
    VALUE_CHECK_REPEAT = 4
    loop_count = 0  # Loop counter
    ev_count = 0    # Event counter
    m_ids = len(id)
    np_pdes = np.array(pdes)
    x = []
    y = []
    pdiff = m_ids*[False]
    for i in range(m_ids):
        y +=[list([])] #to store all motor data

    ############ Zeroing procedure 
    if(zero_pos):
        print("Motor position zeroing started")
        for i in range(m_ids):
            mmc.zero_motor(id[i])	                # Set current position to be zero position						
        time.sleep(3.)
        print("Motor position should be zeroed")

    ############ Enabling motors
    for i in range(m_ids):
        mmc.enable_motor(id[i])						# Enable motor with CAN ID i

    ############ Value check
    for i in range(VALUE_CHECK_REPEAT):             # To be sure that the valid data is received
        for i in range(m_ids):
            # Performs check of current values 
            # Send a command:  
            # (CAN ID, position setpoint, velocity setpoint, position gain, velocity gain, feed-forward torque)
            # Units are:  Radians, Rad/s, N-m/rad, N-m/rad/s, N-m
            mmc.send_command(id[i], 0. , 0., 0., 0., 0)	

            # rx_values are ordered (CAN ID, Position, Velocity, Current)
            print("Motor {} with ID {} values:".format(MOTOR_NAMES[id[i]-1], id[i]))
            print(mmc.rx_data[0][0])
            print(mmc.rx_data[1][0])
            print(mmc.rx_data[2][0])

    if not (runtime[0] == 0.):
        t = Timer(runtime[0], terminate)
        t.start() # schedule termination

    ############ Loop
    # while abs(mmc.rx_data[0][0]-np_pdes) > POS_THRESHOLD :
    while True :
        # Check for the time expirations
        if time_triggered :
            time_triggered = 0
            ev_count += 1
            if not (ev_count == len(runtime) or m_check or tracking):
                t = Timer(runtime[ev_count], terminate)
                t.start() # schedule termination    
            else:
                break
        
        if not m_check:
            x += [loop_count*DT]
            loop_count += 1    # increase counter
            pdiff = m_ids*[False]
            p = np_pdes[ev_count,:]
            for i in range(m_ids):
                sucess = mmc.send_command(id[i], p[i] , vdes[i], pv_gains[0], pv_gains[1], 0)
                # mmc.send_command(MOTOR_ID, np_pdes , vdes, 20., 4., 0)
                # f.write("np_pdes: {}\t ,p_actual: {}\r\n".format(p[i], mmc.rx_data[0][0]))
                if sucess:
                    y[i] += [[p[i], mmc.rx_data[0][0], mmc.rx_data[1][0], mmc.rx_data[2][0]]]  # store values
                    # if abs(p[i] - mmc.rx_data[0][0]) < POS_THRESHOLD:
                    #     pdiff[i] = True
                else:
                    y[i]+= [y[i][-1]] if len(y[i])>0 else [[0., 0., 0., 0.]]
            print("loop")   # Indicate that loop is performed
        
        if tracking:   # for tracking the position
            ev_count += 1
            if ev_count == len(pdes):
                break            
                
            

    ############ Disabling motors
    for i in range(m_ids):
        mmc.disable_motor(id[i]) # Disable motor with CAN ID i
    
    # f.close()

    ############ Ploting data
    if not m_check:
        # for i in range(m_ids):
        plotxy(x, np.array(y), id, ["Position des","Position actual","Velocity","Current Q Axis"],'Simulated jumping motion execution', ["rad","rad/s","A"])
        
        plt.show()

def sin_track(mmc, id, runtime, vdes, pv_gains = [0.,0.], zero_pos=False, freq = 1.,  loop_dt = 1., logfile='/tmp/log_controller.txt'):
    '''
    mmc     -   Instance of the controller
    id      -   Motor ids                                       [1,m_ids]
    runtime -   Time interval                                   [1, 1]
    vdes    -   Desired speed for each time interval            [1, m_ids]
    pv_gains-   Position and velocity gains                     [1, 2]
    loop_dt -   When runtime not provided every loop iteration execution will be increased by loop_dt
    logfile -   File for logging 
    '''


    # f = open(logfile, "w")
    global time_triggered
    VALUE_CHECK_REPEAT = 4
    loop_count = 0  # Loop counter
    ev_count = 0    # Event counter
    m_ids = len(id)
    np_pdes = np.array(pdes)
    x = []
    y = []
    pdiff = 0
    for i in range(m_ids):
        y +=[list([])] #to store all motor data


    ############ Zeroing procedure 
    if(zero_pos):
        print("Motor position zeroing started")
        for i in range(m_ids):
            mmc.zero_motor(id[i])	                # Set current position to be zero position						
        time.sleep(3.)
        print("Motor position should be zeroed")

    ############ Enabling motors
    for i in range(m_ids):
        mmc.enable_motor(id[i])						# Enable motor with CAN ID i

    ############ Value check
    for i in range(VALUE_CHECK_REPEAT):             # To be sure that the valid data is received
        for i in range(m_ids):
            # Performs check of current values 
            # Send a command:  
            # (CAN ID, position setpoint, velocity setpoint, position gain, velocity gain, feed-forward torque)
            # Units are:  Radians, Rad/s, N-m/rad, N-m/rad/s, N-m
            mmc.send_command(id[i], 0. , 0., 0., 0., 0)	

            # rx_values are ordered (CAN ID, Position, Velocity, Current)
            print("Motor {} with ID {} values:".format(MOTOR_NAMES[id[i]-1], id[i]))
            print(mmc.rx_data[0][0])
            print(mmc.rx_data[1][0])
            print(mmc.rx_data[2][0])

    if not (runtime[0] == 0.):
        t = Timer(runtime[0], terminate)
        t.start() # schedule termination

    ############ Loop
    # while abs(mmc.rx_data[0][0]-np_pdes) > POS_THRESHOLD :
    while True :
        # Check for the time expirations
        if time_triggered :
            time_triggered = 0
            ev_count += 1
            if not (ev_count == len(runtime)):
                t = Timer(runtime[ev_count], terminate)
                t.start() # schedule termination    
            else:
                break
        
        
        x += [loop_count*DT]
        loop_count += 1    # increase counter
        p = m_ids*[math.sin(freq*2*math.pi*DT*loop_count)]
        
        for i in range(m_ids):
            sucess = mmc.send_command(id[i], p[i] , vdes[i], pv_gains[0], pv_gains[1], 0)
            # mmc.send_command(MOTOR_ID, np_pdes , vdes, 20., 4., 0)
            # f.write("np_pdes: {}\t ,p_actual: {}\r\n".format(p[i], mmc.rx_data[0][0]))
            if sucess:
                y[i] += [[p[i], mmc.rx_data[0][0], mmc.rx_data[1][0], mmc.rx_data[2][0]]]  # store values
                if abs(p[i] - mmc.rx_data[0][0]) < POS_THRESHOLD:
                    pdiff[i] = True
            else:
                y[i]+= [y[i][-1]] if len(y[i])>0 else [[0., 0., 0., 0.]]
               
            
        print("loop")   # Indicate that loop is performed
        
        # if tracking and all(pdiff):   # for tracking the position
        #     ev_count += 1
        #     if ev_count == len(pdes):
        #         break            
                
            

    ############ Disabling motors
    for i in range(m_ids):
        mmc.disable_motor(id[i]) # Disable motor with CAN ID i
    
    # f.close()

    ############ Mean difference motors
    print("Mean difference: {}".format(pdiff/loop_count))

    ############ Ploting data
    
    for i in range(m_ids):
        plotxy(x, np.array(y[i]), ["Position desired","Position actual","Velocity","Current Q Axis"],'{} Actuator'.format(MOTOR_NAMES[id[i]-1]),["rad","rad/s","A"] )
    
    plt.show()


RUNTIMES = [5.,0.18,4,0.5,0.5,0.5,10.]
TIME_INDEXES = 47
pdes_sim = [
				[0.00, -0.05, -0.10, -0.14, -0.18, -0.21, -0.09, -0.08, -0.10, -0.11, -0.13, -0.15, -0.16, -0.18, -0.19, -0.20, -0.22, -0.23, -0.24, -0.25, -0.26, -0.27, -0.28, -0.28, -0.29, -0.29, -0.30, -0.30, -0.30, -0.30, -0.30, -0.29, -0.29, -0.29, -0.28, -0.27, -0.26, -0.25, -0.24, -0.23, -0.29, -0.29, -0.29, -0.28, -0.27, -0.27, -0.27 ],
				[0.00, 0.05, 0.09, 0.14, 0.19, 0.23, 0.14, 0.16, 0.16, 0.17, 0.18, 0.18, 0.19, 0.20, 0.20, 0.21, 0.22, 0.22, 0.23, 0.24, 0.24, 0.25, 0.25, 0.26, 0.27, 0.27, 0.28, 0.29, 0.29, 0.30, 0.31, 0.31, 0.32, 0.33, 0.33, 0.34, 0.35, 0.35, 0.36, 0.37, 0.43, 0.42, 0.41, 0.40, 0.39, 0.39, 0.39 ],
				[0.00, 0.11, 0.21, 0.32, 0.42, 0.50, 0.29, 0.30, 0.31, 0.32, 0.33, 0.35, 0.36, 0.37, 0.38, 0.39, 0.40, 0.41, 0.42, 0.44, 0.45, 0.46, 0.47, 0.48, 0.49, 0.50, 0.51, 0.52, 0.54, 0.55, 0.56, 0.57, 0.58, 0.59, 0.60, 0.61, 0.63, 0.64, 0.65, 0.66, 0.77, 0.76, 0.75, 0.74, 0.73, 0.72, 0.72 ]
				]
vdes_sim = [
				[0.00, -5.43, -4.92, -4.82, -4.26, -2.74, 12.69, 0.94, -1.78, -1.75, -1.71, -1.67, -1.62, -1.57, -1.50, -1.43, -1.36, -1.27, -1.19, -1.09, -1.00, -0.89, -0.79, -0.68, -0.56, -0.45, -0.33, -0.21, -0.09, 0.03, 0.16, 0.28, 0.40, 0.52, 0.64, 0.76, 0.87, 0.99, 1.10, 1.20, -6.42, 0.09, 0.84, 0.65, 0.46, 0.27, 0.05 ],
				[0.00, 4.90, 4.93, 5.31, 5.26, 4.13, -9.31, 1.21, 0.71, 0.71, 0.70, 0.70, 0.70, 0.69, 0.69, 0.69, 0.69, 0.69, 0.69, 0.69, 0.69, 0.69, 0.69, 0.69, 0.69, 0.69, 0.69, 0.69, 0.69, 0.70, 0.70, 0.70, 0.70, 0.69, 0.69, 0.69, 0.69, 0.69, 0.68, 0.68, 6.26, -0.38, -1.39, -1.06, -0.74, -0.43, -0.10 ],
				[0.00, 11.36, 10.82, 11.11, 10.55, 8.71, -22.04, 1.20, 1.18, 1.18, 1.18, 1.18, 1.18, 1.18, 1.18, 1.18, 1.18, 1.18, 1.18, 1.18, 1.18, 1.18, 1.18, 1.17, 1.17, 1.17, 1.17, 1.17, 1.17, 1.17, 1.17, 1.17, 1.17, 1.17, 1.17, 1.17, 1.17, 1.17, 1.17, 1.17, 12.23, -1.11, -1.65, -1.28, -0.90, -0.54, -0.13 ]
				]

if __name__ == '__main__':
    SAFE_EXIT = True
    v_des = [0.1, 0.1, 0.1]
    pdes = [np.repeat(i,3) for i in pdes_sim] # Increase amount of data points 
    pdes = np.transpose(2*np.array(pdes))   # Increase joint motion for the sake of better visualisation
    pdes = np.array([-1.,1.,-1.]) * pdes        # Flip joint rotation directions for Hip and Ankle joints
    # pdes = np.append(pdes, pdes[::-1], axis=0)  # Append so it returns to zero after jump
    mmc = mm.MotorModuleController('/dev/ttyACM1')		# Connect to the controller's serial port

    if (SAFE_EXIT):
        try:
            # run_motors(mmc, MOTOR_IDS[2:3], RUNTIMES[0:1], pdes=[[0]], vdes=[0.], pv_gains=[50.,0.], zero_pos=False, m_check=False)
            run_motors(mmc, MOTOR_IDS[0:3], RUNTIMES[0:1], pdes=[[1., 1.5, -2]], vdes=[0.1, 0.1, 0.1], pv_gains=[70.,0.5], zero_pos=False, m_check=False)
            # run_motors(mmc, MOTOR_IDS[0:3], RUNTIMES[0:1], pdes=pdes, vdes=[0.1, 0.1, 0.1], pv_gains=[200.,3.], loop_dt=0.0, zero_pos=False, m_check=False, tracking = True)
            run_motors(mmc, MOTOR_IDS[0:3], RUNTIMES[0:3], pdes=[[1., 1.5, -2],[0., 0., -0.11],[0.72, 1.36, -1.8]], vdes=[10., 10., 10.], pv_gains=[100.,0.], zero_pos=False, m_check=False, tracking= False)
            # run_motors(mmc, MOTOR_IDS[0:3], RUNTIMES[0:1], pdes=[[0., 0., 0.]], vdes=[10., 10., 10.], pv_gains=[70.,0.5], zero_pos=True, m_check=False)
            # run_motors(mmc, MOTOR_IDS[0:3], RUNTIMES[0:3], pdes=[[1.17, 2.2, -1.3],[0., 0., -0.2],[1.17, 2.2, -1.3]], vdes=[0.1, 0.1, 0.1], pv_gains=[200.,3.], zero_pos=False, m_check=False)
            # sin_track(mmc, MOTOR_IDS[0:1], RUNTIMES[0:1], [0.1,0.1,0.1], [70.,0.], zero_pos=False, freq=2.)
        except:
            for i in range(len(MOTOR_IDS)): # Disable all motors
                mmc.disable_motor(MOTOR_IDS[i]) 
            print(traceback.format_exc())
            print("Motors disabled")
    # else:
    #     run_motors()
