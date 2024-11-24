import math

class Callback:
    def __init__(self):
        pass

    def debug(self):
        print('Operation debug has been called by the state machine')

    def debug_real(self, val):
        print('Operation debug has been called by the state machine: ' + str(val))

    def sqrt(self, val):
        return math.sqrt(val)
