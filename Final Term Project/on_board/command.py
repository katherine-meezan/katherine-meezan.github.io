class Command():
    def __init__(self, mode: str, end_condition: float, lin_speed: float, x_coord=0, y_coord=0):
        self.mode = mode
        """
        modes: 
        "lin" for line follower mode
        "pos" for position follower mode
        "bmp" for bump sensor mode
        "rev" for blind reverse mode 
        """
        self.end_condition = end_condition
        """
        for line follower mode:
        end condition is the linear distance travelled in mm (s)
        for position follower mode:
        end condition is the distance between Romi's current position and the target coordinates
        for blind reverse mode:
        end condition is the linear distance travelled in mm (s)
        """
        self.x_coord = x_coord #X coordinate. Can be ignored for modes other than 1
        self.y_coord = y_coord #Y coordinate. Can be ignored for modes other than 1
        self.lin_speed = lin_speed # desired linear speed of romi in mm/s
    def check_end_condition(self, state): # returns 1 if the end condition has been reached
        if self.mode == "pos":
            if state <= 10: # check that Romi is closer than the threshold
                return 1
            
        else:
            if state >= self.end_condition: # for other states, the command has been fulfilled when the threshold is passed
                return 1
       
        return 0