# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2017-2018 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
Version of the AutonomousSequence.py example connecting to 10 Crazyflies.
The Crazyflies go straight up, hover a while and land but the code is fairly
generic and each Crazyflie has its own sequence of setpoints that it files
to.
The layout of the positions:
    x2      x1      x0
y3  10              4
            ^ Y
            |
y2  9       6       3
            |
            +------> X
y1  8       5       2
y0  7               1
"""
import time

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

global position_tot

# Change uris and sequences according to your setup
URI1 = 'radio://0/80/2M/E7E7E7E7E7'
URI2 = 'radio://0/80/2M/E7E7E7E702'



z0 = 0.1
z = 0.4

x0 = 0
x1 = 0
x2 = -0.7

y0 = 0
y1 = -0
y2 = 0
y3 = 1.0

#    x   y   z  time
sequence1 = [
    (0,0,0, 30),
]

sequence2 = [
    (0, 0, 0, 3),
]



seq_args = {
    URI1: [sequence1],
    URI2: [sequence2],

}

# List of URIs, comment the one you do not want to fly
uris = {
    URI1,
    #URI2,

}

def addition_terme(l1,l2):
    total=[]
    for i in range(len(l1)):
        total+=[l1[i]+l2[i]]
    return total

def recup_position_cf(cf):
    return [swarm.get_estimated_positions()[cf.link_uri][0],swarm.get_estimated_positions()[cf.link_uri][1],swarm.get_estimated_positions()[cf.link_uri][2]]

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
    global position_tot

    try:
        cf = scf.cf
        print(cf.link_uri)

        #take_off(cf, sequence[0])
        for position in sequence:
            print('Setting position {}'.format(position))
            end_time = time.time() + position[3]
            while time.time() < end_time:
                pos_cf=recup_position_cf(cf)
                print(pos_cf)
                position_tot=addition_terme(position_tot,[1,0,0])
                print(position_tot)

                # cf.commander.send_position_setpoint(position[0],
                #                                     position[1],
                #                                     position[2], 0)
                time.sleep(0.1)
        #land(cf, sequence[-1])
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
        position_tot=[0,0,0]
        print('Waiting for parameters to be downloaded...')
        swarm.parallel(wait_for_param_download)
        swarm.parallel(run_sequence, args_dict=seq_args)