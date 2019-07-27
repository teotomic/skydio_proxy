from vehicle.skills.util.core import AttrDict

# 2D named waypoints so we can reuse them
WAYPOINTS_2D = AttrDict(
    home=AttrDict(position=[0.0, 0.0]),
    corner_a=AttrDict(position=[-1.0,  1.0]),
    corner_b=AttrDict(position=[ 1.0,  1.0]),
    corner_c=AttrDict(position=[ 1.0, -1.0]),
    corner_d=AttrDict(position=[-1.0, -1.0]),
)

# Flying altitudes
ALTITUDES = AttrDict(
    takeoff=1.0,
    flight_altitude=1.5,
)

OBSTACLE_MARGIN = dict(
    normal=0.6,
    low=0.0
)

# Fly up to the elevator, through the chimney, look around, and back out through the chimney
WAYPOINTS = [
    AttrDict(name='Takeoff point',
             position='home',
             altitude='takeoff',
             camera=AttrDict(yaw=0, pitch=0),
             obstacle_margin='normal'),

    AttrDict(name='Corner A',
             position='corner_a',
             altitude='flight_altitude',
             camera=AttrDict(yaw=45, pitch=20),
             obstacle_margin='normal'),

    AttrDict(name='Corner B',
             position='corner_b',
             altitude='flight_altitude',
             camera=AttrDict(yaw=135, pitch=20),
             obstacle_margin='normal'),

    AttrDict(name='Corner C',
             position='corner_c',
             altitude='flight_altitude',
             camera=AttrDict(yaw=225, pitch=20),
             obstacle_margin='normal'),

    AttrDict(name='Corner D',
             position='corner_d',
             altitude='flight_altitude',
             camera=AttrDict(yaw=315, pitch=20),
             obstacle_margin='normal'),

    AttrDict(name='Landing point',
             position='home',
             altitude='takeoff',
             camera=AttrDict(yaw=0, pitch=0),
             obstacle_margin='normal'),
]
