from time import ticks_us, ticks_diff  # Use to get dt value in update()
from pyb import Timer, Pin


class Encoder:
    '''A quadrature encoder decoding interface encapsulated in a Python class'''

    def __init__(self, tim, chA_pin, chB_pin):  # timer, and two Pin objects
        '''Initializes an Encoder object'''
        self.position = 0  # Total accumulated position of the encoder
        self.prev_count = 0  # Counter value from the most recent update
        self.delta = 0  # Change in count between last two updates
        self.dt = 1.0  # Amount of time between last two updates; measured in seconds
        self.tim = tim # timer object
        self.ticks_old = ticks_us()
        self.enc_count_old = self.tim.counter()
        self.comp = (tim.period() + 1) // 2  # comparator equal to half of AR + 1

    def update(self):
        '''Runs one update step on the encoder's timer counter to keep
           track of the change in count and check for counter reload'''
        ticks_new = ticks_us()
        enc_count_new = self.tim.counter()
        
        
        # Calculates elapsed time in seconds
        self.dt = ticks_diff(ticks_new, self.ticks_old)/1000000  # retrieves delta
        
        # How much encoder has turned, raw difference
        raw_diff = enc_count_new - self.enc_count_old
        
        # check for overflow and perform overflow correction calculation
        if raw_diff > self.comp:
            raw_diff -= (self.comp*2)
            
        elif raw_diff < -self.comp:
            raw_diff += (self.comp*2)
        
        # Store corrected difference
        self.delta = raw_diff
        
        # Update position; in encoder counts
        self.position += raw_diff
        
        # Update state for next update
        self.ticks_old = ticks_new # microseconds
        self.enc_count_old = enc_count_new # Encoder counts
        pass

    def get_position(self):
        '''Returns the most recently updated value of position as determined
           within the update() method'''
        return self.position * -1

    def get_velocity(self):
        '''Returns a measure of velocity using the most recently updated
           value of delta as determined within the update() method'''
        return (self.delta * -1) / (self.dt)

    def zero(self):
        '''Sets the present encoder position to zero and causes future updates
           to measure with respect to the new zero position'''
        self.position = 0
        self.enc_count_old = self.tim.counter()
        self.delta = 0
        # self.ticks_new = ticks_us()
        # self.dt = 0
        # pass


# PA8 = Pin(Pin.cpu.A8, mode=Pin.OUT_PP)  # Timer1_Channel1: Encoder A left
# PA9 = Pin(Pin.cpu.A9, mode=Pin.OUT_PP)  # Timer1_Channel2: Encoder B left
# timLeft = Timer(1, freq=100)
# ch1Left = timLeft.channel(1, Timer.ENC_AB, pin=PA8)
# ch2Left = timLeft.channel(2, Timer.ENC_AB, pin=PA9)
# left_encoder = Encoder(timLeft, ch1Left, ch2Left)
#
# PB4 = Pin(Pin.cpu.B4, mode=Pin.OUT_PP)  # Timer3_Channel1: Encoder A right
# PB5 = Pin(Pin.cpu.B5, mode=Pin.OUT_PP)  # Timer3_Channel2: Encoder B right
# timRight = Timer(3, freq=100)
# ch1Right = timRight.channel(1, Timer.ENC_AB, pin=PB4)
# ch2Right = timRight.channel(2, Timer.ENC_AB, pin=PB5)
# right_encoder = Encoder(timRight, ch1Right, ch2Right)

# tim6 = Timer(6, freq=100)
# # t6ch = tim6.channel(1, Timer.OC_FORCED_INACTIVE)
# def t6cb():
#     left_encoder.update()
#     print("left pos: ", left_encoder.get_position(), "left vel: ", left_encoder.get_velocity())
#     right_encoder.update()
#     print("right pos: ", right_encoder.get_position(), "right pos: ", right_encoder.get_velocity())

# tim6.callback(t6cb)
#
# while True:
#     left_encoder.update()
#     print("left pos: ", left_encoder.get_position(), "left vel: ", left_encoder.get_velocity())
#     right_encoder.update()
#     print("right pos: ", right_encoder.get_position(), "right pos: ", right_encoder.get_velocity())



