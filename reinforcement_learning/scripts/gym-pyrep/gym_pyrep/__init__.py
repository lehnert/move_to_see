from gym.envs.registration import register

register(
    id='pyrep-v0',
    entry_point='gym_pyrep.envs:PyrepEnv',
)