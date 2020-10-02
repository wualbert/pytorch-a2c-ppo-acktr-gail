from policy.policy_skeleton import Policy


class TurnPushTurnPolicy(Policy):
    def __init__(self, target_position, target_orientation, **kwargs):
        super(TurnPushTurnPolicy, self).__init__(
            target_position, target_orientation)

    def get_action(self, current_state, **kwargs):
        raise NotImplementedError
