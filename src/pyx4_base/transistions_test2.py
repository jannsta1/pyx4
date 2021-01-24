#!/usr/bin/env python3

from transitions import Machine
from time import sleep
from random import choice

class Passport_cam_sm(object):

    def __init__(self):
        self._face_okay = False
        self._height_okay = False

    def face_ok(self, even_data):
        self._face_okay = choice([True, False])
        return self._face_okay

    def height_ok(self, even_data):
        # tol = even_data.kwargs.pop('tol', 0.5)
        self._height_okay = choice([True, False])
        return self._height_okay

states = ['move2face', 'validate_face', 'open_gate']

transitions = [
    {'trigger': 'check', 'source': 'move2face', 'dest': 'validate_face',
     'conditions': ['height_ok', 'face_ok']},
    {'trigger': 'check', 'source': 'validate_face', 'dest': 'open_gate',
     'conditions': 'face_ok'},  # (1)
    {'trigger': 'check', 'source': 'validate_face', 'dest': 'move2face'},  # (2)
    {'trigger': 'check', 'source': 'open_gate', 'dest': 'move2face'},  # (2)
]


class Passport_cam(object):

    def __init__(self):

        self.model = Passport_cam_sm()
        self.machine = Machine(model=self.model, states=states, transitions=transitions,
                               initial='move2face', send_event=True)
        self.running = True

    def run(self):
        while self.running:
            print(('At state ' + self.model.state))
            #while not self.model.check():
            self.model.check()

            sleep(1)

    def camera_dynamics(self):
        print("Processing...")
        self.running = False


pc = Passport_cam()
pc.run()
print('Done')