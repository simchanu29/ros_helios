// Uses POSIX functions to send and receive data from a Maestro.
// NOTE: The Maestro's serial mode must be set to "USB Dual Port".
// NOTE: You must change the 'const char * device' line below.

/**
 * Version c du code python ci-dessous
 */
#pragma once

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
 
#ifdef _WIN32
#define O_NOCTTY 0
#else
#include <termios.h>
#endif

/**
 * Gets the position of a Maestro channel.
 * See the "Serial Servo Commands" section of the user's guide.
 * @param fd
 * @param channel
 * @return
 */
int maestroGetPosition(int fd, unsigned char channel);

/**
 * Sets the target of a Maestro channel.
 * See the "Serial Servo Commands" section of the user's guide.
 * The units of 'target' are quarter-microseconds.
 * @param fd
 * @param channel
 * @param target
 * @return
 */
int maestroSetTarget(int fd, unsigned char channel, unsigned short target);

/**
 * Sets the acceleration of a Maestro channel.
 * See the "Serial Servo Commands" section of the user's guide.
 * The speed limit is given in units of (0.25 μs)/(10 ms)
 * Speed of 0 is unlimited
 * @param fd
 * @param channel
 * @param target
 * @return
 */
int maestroSetSpeed(int fd, unsigned char channel, unsigned short target);

/**
 * Sets the speed of a Maestro channel.
 * See the "Serial Servo Commands" section of the user's guide.
 * The acceleration limit is a value from 0 to 255 in units of (0.25 μs)/(10 ms)/(80 ms)
 * Acceleration of 0 is unlimited
 * @param fd
 * @param channel
 * @param target
 * @return
 */
int maestroSetAccel(int fd, unsigned char channel, unsigned short target);

/**
 * Connexion de la maestro
 * @param device
 * @return
 */
int maestroConnect(const char * device);