
class Policy:
    def __init__(self, target_position, target_orientation, **kwargs):
        self.target_position = target_position
        self.target_orientation = target_orientation
        raise NotImplementedError

    def get_action(self, current_state, **kwargs):
        raise NotImplementedError
