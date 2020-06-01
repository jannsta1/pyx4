from transitions import Machine
from time import sleep, time
from random import choice

class Passport_cam_sm(object):

    def __init__(self):

        self.face_ok = False
        self.height_ok = False

states = ['move2face', 'validate_face', 'open_gate']

transitions = [
    {'trigger': 'at_face', 'source': 'move2face', 'dest': 'validate_face', 'conditions': 'height_ok'},
    {'trigger': 'face_valid', 'source': 'validate_face', 'dest': 'open_gate', 'conditions': 'face_valid'},
    {'trigger': 'face_invalid', 'source': 'validate_face', 'dest': 'move2face'},
]

class Passport_cam(object):

    def __init__(self, terminate_time_s=10, time_step_s=1):

        self.model = Passport_cam_sm()
        self.machine = Machine(model=self.model, states=states, transitions=transitions, initial='move2face', send_event=True)
        self.running = True
        self.init_time = time()
        self.terminate_time_s = terminate_time_s
        self.time_step_s = time_step_s
        self.face_pos_z = 5
        self.camera_pos_z = 0
        self.error_z = 888

    def run(self):
        '''
        main program loop
        :return: 
        '''

        while self.running and not self.main_timeout():

            self.machine.face_ok = self.face_ok()
            self.machine.height_ok = self.height_ok()
            print ('At state ' + self.model.state) #, ' camera height (', self.error_z ,') ok is ', self.machine.height_ok, ' face check is ', self.machine.face_ok)
            # todo - poll latest state here? (self.model.state)
            self.camera_dynamics()
            sleep(1)

    def face_ok(self):
        '''
        very robust method for determining if the face is valid...
        :return: 
        '''
        return choice([True, False])

    def height_ok(self, tol=0.5):
        '''
        Checks if the face height is OK to do comparison.
        :return: 
        '''
        if abs(self.error_z) < tol:
            return True
        else:
            return False

    def camera_dynamics(self, max_displacement=1):
        '''
        Moves camera height towards face height at a maximum of "max_displacement" per function call 
        :return: 
        '''
        self.error_z = self.camera_pos_z - self.face_pos_z
        threshold_error = (min(max(self.error_z, -max_displacement), max_displacement))
        self.camera_pos_z = self.camera_pos_z - threshold_error
        print ('Camera height error is: {0}'.format(self.error_z))

    def main_timeout(self):
        if time() > self.init_time + self.terminate_time_s:
            return True
        else:
            return False

pc = Passport_cam()
pc.run()

