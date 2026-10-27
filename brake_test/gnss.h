// GNSS (any NMEA receiver via TinyGPSPlus) for the brake test.
// Reference location is typed into the Serial Monitor:
//   "lat,lon"  e.g. 19.076090,72.877426
//   "here"     use the current GNSS fix
#pragma once

void gnssBegin();
void gnssUpdate();   // call every loop: feeds the parser and reads Serial input
float gnssSteer();   // -1..1, + = turn right (right brake); 0 if no fix/reference/course or arrived
void gnssPrint();
