
class Interface:

    def __init__(self):
        pass

    def connect(self):
        raise NotImplementedError

    def disconnect(self):
        raise NotImplementedError

    def send_forces(self, u):
        raise NotImplementedError

    def get_feedback(self):
        raise NotImplementedError