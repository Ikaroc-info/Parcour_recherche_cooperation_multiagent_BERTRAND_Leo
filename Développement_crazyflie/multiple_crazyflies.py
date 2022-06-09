'''This enable the user to give sequences of instructions to multiple crazyflies simultaneously'''
import time

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

# Change uris and sequences according to your setup
URI1 = 'radio://0/80/2M/E7E7E7E700'
URI2 = 'radio://0/80/2M/E7E7E7E701'
URI3 = 'radio://0/80/2M/E7E7E7E703'



z0 = 0.1
z = 0.5

x0 = 0
x1 = 0
x2 = -0.7

y0 = 0
y1 = -0
y2 = 0
y3 = 1.0

#    x   y   z  time
sequence1 = [
    (x0-0.1, y0, z, 35),
]

sequence2 = [
    (x0-0.4, y0-0.3, z, 1),
    (x0-0.4, y0, z, 1),
    (x0-0.4, y0+0.3, z, 1),
    (x0-0.1, y0+0.3, z, 1),
    (x0+0.2, y0+0.3, z, 1),
    (x0+0.2, y0, z, 1),
    (x0+0.2, y0-0.3, z, 1),
    (x0-0.1, y0-0.3, z, 1),
    (x0-0.4, y0-0.3, z, 1),   
    (x0-0.4, y0-0.3, z, 1),
    (x0-0.4, y0, z, 1),
    (x0-0.4, y0+0.3, z, 1),
    (x0-0.1, y0+0.3, z, 1),
    (x0+0.2, y0+0.3, z, 1),
    (x0+0.2, y0, z, 1),
    (x0+0.2, y0-0.3, z, 1),
    (x0-0.1, y0-0.3, z, 1),
    (x0-0.4, y0-0.3, z, 1), 
    (x0-0.4, y0-0.3, z, 1),
    (x0-0.4, y0, z, 1),
    (x0-0.4, y0+0.3, z, 1),
    (x0-0.1, y0+0.3, z, 1),
    (x0+0.2, y0+0.3, z, 1),
    (x0+0.2, y0, z, 1),
    (x0+0.2, y0-0.3, z, 1),
    (x0-0.1, y0-0.3, z, 1),
    (x0-0.4, y0-0.3, z, 1), 
]

sequence3= [
    (x0-0.4, y0+0.3, z, 1),
    (x0-0.1, y0+0.3, z, 1),
    (x0+0.2, y0+0.3, z, 1),
    (x0+0.2, y0, z, 1),
    (x0+0.2, y0-0.3, z, 1),
    (x0-0.1, y0-0.3, z, 1),
    (x0-0.4, y0-0.3, z, 1),   
    (x0-0.4, y0-0.3, z, 1),
    (x0-0.4, y0, z, 1),
    (x0-0.4, y0+0.3, z, 1),
    (x0-0.1, y0+0.3, z, 1),
    (x0+0.2, y0+0.3, z, 1),
    (x0+0.2, y0, z, 1),
    (x0+0.2, y0-0.3, z, 1),
    (x0-0.1, y0-0.3, z, 1),
    (x0-0.4, y0-0.3, z, 1), 
    (x0-0.4, y0-0.3, z, 1),
    (x0-0.4, y0, z, 1),
    (x0-0.4, y0+0.3, z, 1),
    (x0-0.1, y0+0.3, z, 1),
    (x0+0.2, y0+0.3, z, 1),
    (x0+0.2, y0, z, 1),
    (x0+0.2, y0-0.3, z, 1),
    (x0-0.1, y0-0.3, z, 1),
    (x0-0.4, y0-0.3, z, 1), 
    (x0-0.4, y0-0.3, z, 1),
    (x0-0.4, y0, z, 1),
]


seq_args = {
    URI1: [sequence1],
    URI2: [sequence2],
    URI3: [sequence3]

}

# List of URIs, comment the one you do not want to fly
uris = {
    URI1,
    URI2,
    URI3,

}


def wait_for_param_download(scf):
    while not scf.cf.param.is_updated:
        time.sleep(1.0)
    print('Parameters downloaded for', scf.cf.link_uri)


def take_off(cf, position):
    take_off_time = 1.0
    sleep_time = 0.1
    steps = int(take_off_time / sleep_time)
    vz = position[2] / take_off_time

    print(vz)

    for i in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)


def land(cf, position):
    landing_time = 1.0
    sleep_time = 0.1
    steps = int(landing_time / sleep_time)
    vz = -position[2] / landing_time

    print(vz)

    for _ in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)

    cf.commander.spositioend_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


def run_sequence(scf, sequence):

    try:
        cf = scf.cf

        take_off(cf, sequence[0])
        for position in sequence:
            print('Setting position {}'.format(position))
            end_time = time.time() + position[3]
            while time.time() < end_time:
                print(position)
                cf.commander.send_position_setpoint(position[0],
                                                    position[1],
                                                    position[2], 0)
                time.sleep(0.1)
        land(cf, sequence[-1])
    except Exception as e:
        print(e)


if __name__ == '__main__':
    # logging.basicConfig(level=logging.DEBUG)
    cflib.crtp.init_drivers()

    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        # If the copters are started in their correct positions this is
        # probably not needed. The Kalman filter will have time to converge
        # any way since it takes a while to start them all up and connect. We
        # keep the code here to illustrate how to do it.
        # swarm.reset_estimators()

        # The current values of all parameters are downloaded as a part of the
        # connections sequence. Since we have 10 copters this is clogging up
        # communication and we have to wait for it to finish before we start
        # flying.
        print('Waiting for parameters to be downloaded...')
        swarm.parallel(wait_for_param_download)

        swarm.parallel(run_sequence, args_dict=seq_args)