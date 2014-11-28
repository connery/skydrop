#define _XOPEN_SOURCE

#define _GNU_SOURCE

#include <fcntl.h>
#include <termios.h>

#include <sys/wait.h>
#include <sys/shm.h>
#include <sys/types.h>

#include <sys/select.h>
#include <sys/time.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "start_drone_communication.h"

#include "linked_lists_lib/my_list.h"
#include "sbp_piksi/libswiftnav/include/libswiftnav/sbp_messages.h" // DEFINE STRUCTURE USED FOR SHARED MEMORY

#include "shared_memory_double.h"
#include "sbp_piksi/raspberry_uart_communication.h"

#include "errno.h"

#define		SHARED_MEMORY_KEY 11114

int *	pipe_tab_initialisation(int * file_descriptor_tab);

  // file_descriptor_tab[0] : SORTIE DE PIPE (DISPONIBLE EN LECTURE)
  // file_descriptor_tab[1] : ENTREE DE PIPE (DISPONIBLE EN ECRITURE)

  
  // file_descriptor_tab[2] : SORTIE DE PIPE (DISPONIBLE EN LECTURE)
  // file_descriptor_tab[3] : ENTREE DE PIPE (DISPONIBLE EN ECRITURE)

  
  // file_descriptor_tab[4] : SORTIE DE PIPE (DISPONIBLE EN LECTURE)
  // file_descriptor_tab[5] : ENTREE DE PIPE (DISPONIBLE EN ECRITURE)

  
  // file_descriptor_tab[6] : SORTIE DE PIPE (DISPONIBLE EN LECTURE)
  // file_descriptor_tab[7] : ENTREE DE PIPE (DISPONIBLE EN ECRITURE)


int *	pipe_tab_initialisation(int * file_descriptor_tab)
{
  int pipefd[2];

  if (pipe(&(file_descriptor_tab[0])) == -1) { exit(EXIT_FAILURE); }
  if (pipe(&(file_descriptor_tab[2])) == -1) { exit(EXIT_FAILURE); }
  if (pipe(&(file_descriptor_tab[4])) == -1) { exit(EXIT_FAILURE); }
  if (pipe(&(file_descriptor_tab[6])) == -1) { exit(EXIT_FAILURE); }

  return (file_descriptor_tab);
}

int	close_file_descriptor(int * file_descriptor_tab, int file_descriptor)
{
  int	incrementor;

  if (file_descriptor == 0) // Fermeture de tout les fd encore ouvert dans le processus
    {
      for (incrementor = 0; incrementor < 8; incrementor++)
	{
	  if (file_descriptor_tab[incrementor] != 0) { close(file_descriptor_tab[incrementor]); file_descriptor_tab[incrementor] = 0; }
	}
    }
  else
    {
      for (incrementor = 0; incrementor < 8; incrementor++) // Fermeture du fd en argument
	{
	  if (file_descriptor_tab[incrementor] == file_descriptor && file_descriptor_tab[incrementor] != 0) { close(file_descriptor_tab[incrementor]); file_descriptor_tab[incrementor] = 0; }
	}
    }

  return (EXIT_SUCCESS);
}

int	client_drone_process(int * file_descriptor_tab)
{
  /* close_file_descriptor(file_descriptor_tab, file_descriptor_tab[0]); // Fermeture du fd */
  /* close_file_descriptor(file_descriptor_tab, file_descriptor_tab[1]); // Fermeture du fd */

  /* close_file_descriptor(file_descriptor_tab, file_descriptor_tab[2]); // Fermeture du fd */
  /* close_file_descriptor(file_descriptor_tab, file_descriptor_tab[3]); // Fermeture du fd */

  /* close_file_descriptor(file_descriptor_tab, file_descriptor_tab[5]); // Fermeture du fd */

  /* close_file_descriptor(file_descriptor_tab, file_descriptor_tab[6]); // Fermeture du fd */

  // execl("/bin/sh", "client_drone", "127.0.0.1", "client_drone#01", file_descriptor_tab[4]); // parametre : file_descriptor_tab[4] 

  return (EXIT_SUCCESS);
}

int	read_global_positioning_system_data(void * shared_memory, struct piksi_data_shared_memory * k, struct shared_double * p, struct global_positioning_data ** global_positioning;)
{
  k = ((struct piksi_data_shared_memory *)shared_memory);

  p->sign = k->lat_sign;
  p->whole_part = k->lat_whole_part;

  p->decimal_part[0] = k->lat_decimal_part[0];
  p->decimal_part[1] = k->lat_decimal_part[1];
  p->decimal_part[2] = k->lat_decimal_part[2];
  p->decimal_part[3] = k->lat_decimal_part[3];
  p->decimal_part[4] = k->lat_decimal_part[4];
  p->decimal_part[5] = k->lat_decimal_part[5];
  p->decimal_part[6] = k->lat_decimal_part[6];
  p->decimal_part[7] = k->lat_decimal_part[7];
  p->decimal_part[8] = k->lat_decimal_part[8];
  p->decimal_part[9] = k->lat_decimal_part[9];
      
  *(global_positioning->drone_latitude) = decimal_number_recomposition(0, p);

  p->sign = k->lon_sign;
  p->whole_part = k->lon_whole_part;

  p->decimal_part[0] = k->lon_decimal_part[0];
  p->decimal_part[1] = k->lon_decimal_part[1];
  p->decimal_part[2] = k->lon_decimal_part[2];
  p->decimal_part[3] = k->lon_decimal_part[3];
  p->decimal_part[4] = k->lon_decimal_part[4];
  p->decimal_part[5] = k->lon_decimal_part[5];
  p->decimal_part[6] = k->lon_decimal_part[6];
  p->decimal_part[7] = k->lon_decimal_part[7];
  p->decimal_part[8] = k->lon_decimal_part[8];
  p->decimal_part[9] = k->lon_decimal_part[9];
      
  *(global_positioning->drone_longitude) = decimal_number_recomposition(0, p);

  p->sign = k->height_sign;
  p->whole_part = k->height_whole_part;

  p->decimal_part[0] = k->height_decimal_part[0];
  p->decimal_part[1] = k->height_decimal_part[1];
  p->decimal_part[2] = k->height_decimal_part[2];
  p->decimal_part[3] = k->height_decimal_part[3];
  p->decimal_part[4] = k->height_decimal_part[4];
  p->decimal_part[5] = k->height_decimal_part[5];
  p->decimal_part[6] = k->height_decimal_part[6];
  p->decimal_part[7] = k->height_decimal_part[7];
  p->decimal_part[8] = k->height_decimal_part[8];
  p->decimal_part[9] = k->height_decimal_part[9];
      
  *(global_positioning->drone_height) = decimal_number_recomposition(0, p);

  *(global_positioning->drone_n_sats) = k->n_sats;

  *(global_positioning->drone_vel_north) = k->vel_north;
  *(global_positioning->drone_vel_east) = k->vel_east;
  *(global_positioning->drone_vel_down) = k->vel_down;

  int	gps_time_week = k->gps_time_week;

  char	gps_time_s[10];
  gps_time_s[0] = k->gps_time_s[0];
  gps_time_s[1] = k->gps_time_s[1];
  gps_time_s[2] = k->gps_time_s[2];
  gps_time_s[3] = k->gps_time_s[3];
  gps_time_s[4] = k->gps_time_s[4];
  gps_time_s[5] = k->gps_time_s[5];
  gps_time_s[6] = k->gps_time_s[6];
  gps_time_s[7] = k->gps_time_s[7];
  gps_time_s[8] = k->gps_time_s[8];
  gps_time_s[9] = 0;

  return (0);
}

int	serveur_drone_process(int * file_descriptor_tab)
{
  /* close_file_descriptor(file_descriptor_tab, file_descriptor_tab[0]); // Fermeture du fd */

  /* close_file_descriptor(file_descriptor_tab, file_descriptor_tab[3]); // Fermeture du fd */

  /* close_file_descriptor(file_descriptor_tab, file_descriptor_tab[4]); // Fermeture du fd */

  /* close_file_descriptor(file_descriptor_tab, file_descriptor_tab[7]); // Fermeture du fd */

  // definition des variables specifiques a la gestion de memoire partagee

  int   shared_memory_segment_id;
  void          * shared_memory;
  struct piksi_data_shared_memory * data;


  if ((data = malloc(sizeof(data))) == NULL) { write(2, "malloc_error", 12); }

  double	drone_latitude;
  double	drone_longitude;
  double	drone_height;

  while ((shared_memory_segment_id = shmget(SHARED_MEMORY_KEY, sizeof(data), 0444)) < 0)
    {
      write(2, "SHMGET ERROR : LA CLEF MEMOIRE PARTAGEE NE CORRESPOND A AUCUNE ZONE MEMOIRE OUVERTE", strlen("SHMGET ERROR : LA CLEF MEMOIRE PARTAGEE NE CORRESPOND A AUCUNE ZONE MEMOIRE OUVERTE"));
    }

  if ((shared_memory = shmat(shared_memory_segment_id, NULL, 0)) == (void*)(-1)) { write(2, "shmat_error", 11); exit(EXIT_FAILURE); }

  // shmdt(shared_memory); // DESTRUCTION DE LA MEMOIRE PARTAGEE


  // creation de la boucle de lecture

  t_mylist      * pointer;
  int		i;
  char		buf;

  for (pointer = my_put_in_list(0, ' '), i = 0; i < 20; i++)
    {
      pointer = my_put_in_list(pointer, ' ');
    }
  
  t_mylist        * begin;
  for (begin = pointer; begin->next; begin = begin->next);

  begin->next = pointer;
  pointer->prev = begin;

  fd_set rdfs;
  int max_fd;
	    
  if (file_descriptor_tab[2] < file_descriptor_tab[5])
    max_fd = file_descriptor_tab[5] + 1;
  else
    max_fd = file_descriptor_tab[2] + 1;



  ////////////////////////// LECTURE GPS DATA ///////////////////////////////////////
  
  struct piksi_data_shared_memory * k;
  struct shared_double * p;
  
  if ((k = malloc(sizeof(k))) == NULL) { write(2, "malloc_error", 12); }
  if ((p = malloc(sizeof(p))) == NULL) { write(2, "malloc_error", 12); }
  
  struct global_positioning_data * global_positioning;

  if ((global_positioning = malloc(sizeof(global_positioning))) == NULL) { write(2, "malloc_error", 12); }

  ///////////////////////////////////////////////////////////////////////////////////

  struct timeval time_out;
  time_out.tv_usec = 1000000; // Passage select toutes les 1 seconde si pas d'activite sur les fd



  FD_ZERO(&rdfs);
    
  FD_SET(0, &rdfs); // Add standard input

  char	standard_input_buffer[1024];
  int	nb;

  FD_SET(file_descriptor_tab[2], &rdfs); // Add exit pipe arduino file_descriptor_tab[2]
  FD_SET(file_descriptor_tab[5], &rdfs); // Add exit pipe client file_descriptor_tab[5]

  
  write(file_descriptor_tab[1], "X#", 2);
  //write(file_descriptor_tab[1], "z", 1);

  while (1)
    {
      errno = 0;
      if (select(max_fd, &rdfs, NULL, NULL, /*&time_out*/ NULL) == -1)
	{ 
	  write(2, "select_error", 12);
	  if (errno != 0)
	    {
	      (void)fprintf(stderr, "Select error, %s\n", strerror(errno));
	    }
	
	  exit(1);
	}

      time_out.tv_usec = 1000000; // Passage select toutes les 1 seconde si pas d'activite sur les fd


      ////////////////////////// POSITIONING DATA ///////////////////////////////////////

      read_global_positioning_system_data(shared_memory, k, p, &global_positioning);

      printf("Latitude : %.10f \nLongitude : %.10f\nHauteur : %.10f \nSatellites : %d \nVitesse (North) %6d \nVitesse (East) %6d \nVitesse (Down) %6d \n\n", global_positioning->drone_latitude, global_positioning->drone_longitude, global_positioning->drone_height, global_positioning->drone_n_sats, global_positioning->drone_vel_north, global_positioning->drone_vel_east, global_positioning->drone_vel_down);

      ////////////////////////// POSITIONING DATA ///////////////////////////////////////


      if (FD_ISSET(0, &rdfs)) // Read on standard input
	{
	  if ((nb = read(0, (void *)(standard_input_buffer), 1024)) < 0) { write(2, "read_error", 10); }
	  
	  standard_input_buffer[nb] = 0;
	  
	  printf("Interpretation de l'instruction suivante : %s\n", standard_input_buffer);
	  
	  if (write(file_descriptor_tab[1], (void *)(standard_input_buffer), (size_t)(nb)) < 0) { write(2, "write error", 11); }

	}

      else if (FD_ISSET(file_descriptor_tab[2], &rdfs)) // Read on exit pipe arduino file_descriptor_tab[2]
	{
	  printf("Read on exit pipe arduino\n");

	  while (read(file_descriptor_tab[2], &buf, 1) > 0)
	    {
	      
	      pointer->c = buf;
	      pointer = pointer->next;

	      // (1) lecture des instructions recues depuis la carte arduino (parseur de donnees)
	      // (2) interpretation / enregistrement des donnees
	      // (3) reponse si necessaire : write(file_descriptor_tab[1], '', 1);
	      	      
	      write(0, &buf, 1);
	      
	      // DRONE GEODETIC POSITION
	      
	      // SEND DRONE GEODETIC POSITION TO ARDUINO
	      
	      // write(file_descriptor_tab[1], drone_latitude, sizeof(drone_latitude));
	      // write(file_descriptor_tab[1], drone_longitude, sizeof(drone_longitude));
	      
	      
	      // Exemple : write(file_descriptor_tab[1], "X#", 2);
	    }
	}
      
      else if (FD_ISSET(file_descriptor_tab[5], &rdfs)) // Read on exit pipe client file_descriptor_tab[5]
	{
	  printf("Read on exit pipe client\n");

	  while (read(file_descriptor_tab[5], &buf, 1) > 0)
	    {
	      write(file_descriptor_tab[1], &buf, 1); // transmission des donnees recues depuis le client vers la carte arduino
	      
	      write(STDOUT_FILENO, &buf, 1);
	    }
	}
    }
  return (0);
}

int	python_communication_serial_process(int * file_descriptor_tab)
{
  /* close_file_descriptor(file_descriptor_tab, file_descriptor_tab[1]); // Fermeture du fd */

  /* close_file_descriptor(file_descriptor_tab, file_descriptor_tab[2]); // Fermeture du fd */

  /* close_file_descriptor(file_descriptor_tab, file_descriptor_tab[4]); // Fermeture du fd */
  /* close_file_descriptor(file_descriptor_tab, file_descriptor_tab[5]); // Fermeture du fd */

  /* close_file_descriptor(file_descriptor_tab, file_descriptor_tab[6]); // Fermeture du fd */
  /* close_file_descriptor(file_descriptor_tab, file_descriptor_tab[7]); // Fermeture du fd */

  char	* communication_command_line = malloc(30 * sizeof(char));
  char	* string_nb = malloc(10 * sizeof(char));
  
  strcat(communication_command_line, "python communication.py");
  strcat(communication_command_line, " ");

  sprintf(string_nb, "%d", file_descriptor_tab[0]);

  strcat(communication_command_line, string_nb);
  strcat(communication_command_line, " ");

  sprintf(string_nb, "%d", file_descriptor_tab[3]);
  strcat(communication_command_line, string_nb);
 
  execl("/bin/sh", "sh", "-c", communication_command_line);

  return (EXIT_SUCCESS);
}

int	gps_receiver_process(int * file_descriptor_tab)
{
  /* close_file_descriptor(file_descriptor_tab, 0); // Fermeture de tout les fd ouvert dans le processus */

  //execl("/bin/sh", "gps_communication"); 

  return (EXIT_SUCCESS);
}

int	create_new_process(int * file_descriptor_tab, int * processus_id_tab, int processus_id,int (** process_function_list)(int * file_descriptor_tab))
{
  int incrementor, pid;

  if ((processus_id_tab[processus_id] = fork()) == -1)
    {
      return (EXIT_FAILURE);
    }

  if (processus_id_tab[processus_id] == 0) // Creation d'un nouveau processus, processus fils
    {
      (* process_function_list[processus_id])(file_descriptor_tab);
    }

  else
    {
      if (processus_id < 2) 
	{
	  create_new_process(file_descriptor_tab, processus_id_tab, processus_id + 1, process_function_list);
	}

      else
	{	  
	  (* process_function_list[3])(file_descriptor_tab);
	  
	  while (processus_id_tab[1] != 0 && processus_id_tab[2] != 0 && processus_id_tab[3] != 0) // ATTENTE DE STATUT DE FIN DES PROCESSUS FILS
	    {
	      pid = wait(NULL);
	      for (incrementor = 1; incrementor < 4; incrementor++) { if (processus_id_tab[incrementor] == pid) { processus_id_tab[incrementor] = 0; } }
	    }

	  close_file_descriptor(file_descriptor_tab, 0); // Fermeture de tout les fd ouvert dans le processus pere
      
	  return (EXIT_SUCCESS);
	} 
    }
  
  return (EXIT_SUCCESS);
}

int	main(int argc, char * argv[])
{
  int * file_descriptor_tab = malloc(sizeof(int) * 8); // UNE SERIE DE 2 PIPES VERS ARDUINO, UNE SERIE DE 2 PIPES VERS LE CLIENT
  int * processus_id_tab = malloc(sizeof(int) * 4); // TABLEAU DE PID
  
  processus_id_tab[0] = getpid();

  file_descriptor_tab = pipe_tab_initialisation(file_descriptor_tab); // Creation des pipes de communication inter-processus

  int (* process_function_list[4])(int *) = {gps_receiver_process, client_drone_process, python_communication_serial_process, serveur_drone_process};

  create_new_process(file_descriptor_tab, processus_id_tab, 0, process_function_list); // Creation des processus et appel des fonctions respectivement executee dans chaque processus

  return (EXIT_SUCCESS);
}
