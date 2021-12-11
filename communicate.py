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


FILENAME = '/tmp/motor_control.txt'
time_triggered = False
POS_THRESHOLD = 0.2
MAX_STP = 100 # maximum number of samples to plot
DT = 0.001
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


def plotxy(x,y,labels = None, title = None):
    # # clear the current axis
    # plt.cla()      
    plt.figure()
    colors = ['orange','blue','brown','purple'] 

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
    if not title == None:
        plt.title(title)
    # plt.tight_layout()


def run_motors(mmc, id, runtime, pdes, vdes, pv_gains = [0.,0.], zero_pos=False, m_check = False, logfile='/tmp/log_controller.txt'):
    '''
    mmc     -   Instance of the controller
    id      -   Motor ids                                       [1,m_ids]
    runtime -   Time intervals                                  [1, ev_count]
    pdes    -   Desired position for each time interval         [ev_count, m_ids]
    vdes    -   Desired speed for each time interval            [1, m_ids]
    pv_gains-   Position and velocity gains                     [1, 2]
    zero_pos-   Require to zero out positions of the motors
    m_check -   Check the motor current values. If enabled, pdes, vdes, pv_gains are ignored - Motor should not move
    logfile -   File for logging 
    '''
    if not (    len(runtime) == len(pdes) and 
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
            if not (ev_count == len(runtime) or m_check):
                t = Timer(runtime[ev_count], terminate)
                t.start() # schedule termination    
            else:
                break

        if not m_check:
            x += [loop_count*DT]
            loop_count += 1    # increase counter

            for i in range(m_ids):
                p = np_pdes[ev_count,:]
                mmc.send_command(id[i], p[i] , vdes[i], pv_gains[0], pv_gains[1], 0)
                # mmc.send_command(MOTOR_ID, np_pdes , vdes, 20., 4., 0)
                # f.write("np_pdes: {}\t ,p_actual: {}\r\n".format(p[i], mmc.rx_data[0][0]))
                y[i] += [[p[i], mmc.rx_data[0][0], mmc.rx_data[1][0], mmc.rx_data[2][0]]]  # store values
            
            print("loop")   # Indicate that loop is performed



        # time.sleep(.01) # Introduce bigger loop time interval

    ############ Disabling motors
    for i in range(m_ids):
        mmc.disable_motor(id[i]) # Disable motor with CAN ID i
    
    # f.close()

    ############ Ploting data
    if not m_check:
        for i in range(m_ids):
            plotxy(x, np.array(y[i]), ["Position des","Position actual","Current Phase A","Current Phase B"],'{} Actuator'.format(MOTOR_NAMES[id[i]-1]))
        
        plt.show()



RUNTIMES = [10.,4.,0.5,0.5,0.5,0.5,10.]

if __name__ == '__main__':
    SAFE_EXIT = True
    v_des = [0.1, 0.1, 0.1]

    mmc = mm.MotorModuleController('/dev/ttyACM1')		# Connect to the controller's serial port

    if (SAFE_EXIT):
        try:
            # run_motors(mmc, MOTOR_IDS[0:2], RUNTIMES[0:2], pdes=[[0.72, 1.39],[0.,0.]], vdes=[0., 0.], pv_gains=[50.,0.], zero_pos=False, m_check=False)
            run_motors(mmc, MOTOR_IDS[0:3], RUNTIMES[0:2], pdes=[[0.72, 1.36, -1.1],[0.,0.,0.4]], vdes=[0., 0., 0.], pv_gains=[70.,0.], zero_pos=False, m_check=False)
            # run_motors(mmc, MOTOR_IDS[0:3], RUNTIMES[0:1], pdes=[[1., 1.26, -1.1]], vdes=[0., 0., 0.], pv_gains=[70.,0.], zero_pos=False, m_check=False)
        except:
            for i in range(len(MOTOR_IDS)): # Disable all motors
                mmc.disable_motor(MOTOR_IDS[i]) 
            print(traceback.format_exc())
            print("Motors disabled")
    # else:
    #     run_motors()
