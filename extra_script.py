Import('env')
from os.path import join, realpath

env.Append(
    LIBPATH=[realpath(join('src', env.get('BOARD_MCU')))],
    LIBS=['algobsec']
)
