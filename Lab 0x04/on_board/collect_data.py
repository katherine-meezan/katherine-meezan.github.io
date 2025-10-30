def collect_data():
    
    state = 0
    while True:
        # Initialize state
        if state == 0:
            # Size of queue data being sent back to UI
            QUEUE_SIZE = 100
            
            # Initialize rightside queues
            RIGHT_POS_Q = cqueue.IntQueue(QUEUE_SIZE)
            RIGHT_VEL_Q = cqueue.IntQueue(QUEUE_SIZE)
            R_TIME_Q = cqueue.IntQueue(QUEUE_SIZE)
            
            # Initialize leftside queues
            LEFT_POS_Q = cqueue.IntQueue(QUEUE_SIZE)
            LEFT_VEL_Q = cqueue.IntQueue(QUEUE_SIZE)
            L_TIME_Q = cqueue.IntQueue(QUEUE_SIZE)
            
            state = 1
        # Wait state    
        elif state == 1:
            # run is a shared integer, should be initialized in UI
            # run = task_share.Share('b', thread_protect=False, name="run")
            if run.get():
                state = 2
        # Data collection state        
        elif state == 2:
            
            if run.get():
                
                # Putting shares from right motor task into queues
                RIGHT_POS_Q.put(RIGHT_POS)
                RIGHT_VEL_Q.put(RIGHT_VEL)
                R_TIME_Q.put(R_TIME)
                
                # Putting shares from left motor task into queues
                LEFT_POS_Q.put(LEFT_POS)
                LEFT_VEL_Q.put(LEFT_VEL)
                L_TIME_Q.put(L_TIME)
                
            else:
                # Storing all the queues into a data packet for ease of transfer
                # I'm not certain this will work so modify if needed
                data_packet = {
                    "right_pos": list(RIGHT_POS_Q),
                    "right_vel": list(RIGHT_VEL_Q),
                    "right_time": list(R_TIME_Q),
                    "left_pos": list(LEFT_POS_Q),
                    "left_vel": list(LEFT_VEL_Q),
                    "left_time": list(L_TIME_Q),
                }
                
                # This should be initialized in the UI task
                # data_share = task_share.Share('', name="data_share")
                data_share.put(data_packet)
                
                # Clear rightside queues
                RIGHT_POS_Q.clear()
                RIGHT_VEL_Q.clear()
                R_TIME_Q.clear()
                # Clear leftside queues
                LEFT_POS_Q.clear()
                LEFT_VEL_Q.clear()
                L_TIME_Q.clear()
                
                state = 1
                
        yield
