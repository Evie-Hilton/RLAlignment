from src.environments.base_environment import BaseEnv

class StewartEnv1(BaseEnv):
    def __init__(self, max_steps, observation_space, action_space):
        super().__init__(max_steps, observation_space, action_space)

    