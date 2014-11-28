
#ifndef __START_DRONE_COMMUNICATION_H__
#define __START_DRONE_COMMUNICATION_H__

struct global_positioning_data
{
  double	drone_altitude;
  double	drone_longitude;
  double	drone_height;

  int		drone_n_sats;

  int		drone_vel_north;
  int		drone_vel_east;
  int		drone_vel_down;
};

#endif
