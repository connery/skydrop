
#ifndef __RASPBERRY_UART_COMMUNICATION_H__
#define __RASPBERRY_UART_COMMUNICATION_H__

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

struct piksi_data_shared_memory
{
  //struct shared_double	lat; // Latitude [deg]             

  int                   lat_sign;
  int                   lat_whole_part;
  char                  lat_decimal_part[10];

  //struct shared_double	lon; // Longitude [deg]

  int                   lon_sign;
  int                   lon_whole_part;
  char                  lon_decimal_part[10];

  //struct shared_double	height; // Height [m]                                   

  int                   height_sign;
  int                   height_whole_part;
  char                  height_decimal_part[10];

  // Nombre de staellites connectes
  int			n_sats;

  // GPS Time
  int			gps_time_week;
  char			gps_time_s[10];

  // Vitesse
  int			vel_north;
  int			vel_east;
  int			vel_down;
};

#endif
