/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../gcode.h"
#include "../../module/motion.h"

#include "../../MarlinCore.h"

#if ALL(FWRETRACT, FWRETRACT_AUTORETRACT)
  #include "../../feature/fwretract.h"
#endif

#include "../../sd/cardreader.h"


#include "../../module/planner.h"


extern xyze_pos_t destination;

#if ENABLED(VARIABLE_G0_FEEDRATE)
  feedRate_t fast_move_feedrate = MMM_TO_MMS(G0_FEEDRATE);
#endif


/**
 * G101 movment like G0, G1: Coordinated movement of X Y Z E axes but with one row of fire sequences for the nozzles
 */
void GcodeSuite::G101(int8_t info[MJM_INFO_BUFFER_SIZE]) {
  if (!MOTION_CONDITIONS) return;

  TERN_(FULL_REPORT_TO_HOST_FEATURE, set_and_report_grblstate(M_RUNNING));

  #ifdef G0_FEEDRATE
    feedRate_t old_feedrate;
    #if ENABLED(VARIABLE_G0_FEEDRATE)
      if (fast_move) {
        old_feedrate = feedrate_mm_s;             // Back up the (old) motion mode feedrate
        feedrate_mm_s = fast_move_feedrate;       // Get G0 feedrate from last usage
      }
    #endif
  #endif
  float Stepsize = 0.1;
  float Abstand = 4;
  get_destination_from_command();                 // Get X Y [Z[I[J[K]]]] [E] F (and set cutter power)
  if (parser.seenval('V'))  Stepsize = parser.value_float();
  if (parser.seenval('W'))  Abstand = parser.value_float();
  #ifdef G0_FEEDRATE
    if (fast_move) {
      #if ENABLED(VARIABLE_G0_FEEDRATE)
        fast_move_feedrate = feedrate_mm_s;       // Save feedrate for the next G0
      #else
        old_feedrate = feedrate_mm_s;             // Back up the (new) motion mode feedrate
        feedrate_mm_s = MMM_TO_MMS(G0_FEEDRATE);  // Get the fixed G0 feedrate
      #endif
    }
  #endif

  #if ALL(FWRETRACT, FWRETRACT_AUTORETRACT)

    if (MIN_AUTORETRACT <= MAX_AUTORETRACT) {
      // When M209 Autoretract is enabled, convert E-only moves to firmware retract/recover moves
      if (fwretract.autoretract_enabled && parser.seen_test('E')
        && !parser.seen(STR_AXES_MAIN)
      ) {
        const float echange = destination.e - current_position.e;
        // Is this a retract or recover move?
        if (WITHIN(ABS(echange), MIN_AUTORETRACT, MAX_AUTORETRACT) && fwretract.retracted[active_extruder] == (echange > 0.0)) {
          current_position.e = destination.e;       // Hide a G1-based retract/recover from calculations
          sync_plan_position_e();                   // AND from the planner
          return fwretract.retract(echange < 0.0);  // Firmware-based retract/recover (double-retract ignored)
        }
      }
    }

  #endif // FWRETRACT

  #define NOZZLECOUNT   12
  int16_t feedrate = 20;
  int16_t data = 0;
  bool dataInside = false;
  for (int i =0;i<= Abstand/Stepsize;i++){
    if (destination.x > current_position.x){ current_position.x = current_position.x + Stepsize;}    // sollte für alle Achsen verallgeeinert werden
    else{current_position.x -= Stepsize;}
    planner.buffer_line(current_position, feedrate);
    delayMicroseconds(30);
}
  for (int j = 0 ; j < int( MJM_INFO_BUFFER_SIZE /2);j++){
    for (int i =0; i< int((NOZZLECOUNT-1)/6)+1;i++){  // Converts the binary data of contained in the chars int an int16_t 
      //SERIAL_ECHOPGM("-", info[i]);
      if (info[j*(int((NOZZLECOUNT-1)/6)+1)+i] != 0){
      dataInside = true;
      data = data << 6;
      info[j*(int((NOZZLECOUNT-1)/6)+1)+i] = info[j*(int((NOZZLECOUNT-1)/6)+1)+i] & (~64);
      data = data | info[j*(int((NOZZLECOUNT-1)/6)+1)+i];
      //SERIAL_ECHOPGM(" ", data);
      }
      else{dataInside = false;}
      //SERIAL_ECHOPGM("_", data);
    }
    if (dataInside){
      if (destination.x > current_position.x){ current_position.x = current_position.x + Stepsize;}
      else{current_position.x -= Stepsize;}
      planner.buffer_line(current_position, feedrate);
      fireHp(data);
    }
  }
  planner.buffer_line(destination,feedrate);
  current_position = destination;
memset(info,0,MJM_INFO_BUFFER_SIZE);
  #ifdef G0_FEEDRATE
    // Restore the motion mode feedrate
    if (fast_move) feedrate_mm_s = old_feedrate;
  #endif
}