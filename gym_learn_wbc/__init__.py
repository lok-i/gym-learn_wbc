from gym.envs.registration import register

register(
    id='learn_wbc-v0',
    entry_point='gym_learn_wbc.envs:Learn_wbcEnv',
)
