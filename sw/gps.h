/* trackuino copyright (C) 2010  EA5HAV Javi
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __GPS_H__
#define __GPS_H__

extern char gps_time[7];       // HHMMSS
extern char gps_date[7];       // DDMMYY
extern float gps_lat;
extern float gps_lon;
extern char gps_rtty_lat[10];
extern char gps_rtty_lon[11];
extern float gps_course;
extern float gps_speed;
extern float gps_altitude;
extern char gps_sat;
extern bool active;

void gps_parser_setup();
bool gps_decode(char c);

#endif
