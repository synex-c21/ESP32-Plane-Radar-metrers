#pragma once

#include <cstddef>

namespace services::adsb {

struct Aircraft {
  float lat;
  float lon;
  float nose_deg;
  float track_deg;
  float gs_knots;
  /** Vertical rate in ft/min from baro_rate/geom_rate; NAN when unknown. */
  float vrate_fpm;
  /** Route display, e.g. "CLJ>LTN"; empty when unknown. */
  char route[10];
  char callsign[9];
  char type[5];
  char alt[12];
};

constexpr size_t kMaxAircraft = 64;

size_t aircraftCount();
const Aircraft* aircraftList();

/** Hook invoked during long HTTP I/O (e.g. wifiLoop). Optional. */
using PollFn = void (*)();
void setPollFn(PollFn fn);

/** Fetch aircraft within fetch_radius_km of center_lat/lon from adsb.fi. */
bool fetchUpdate(double center_lat, double center_lon, float fetch_radius_km);

/** millis() when the current positions were fetched; 0 before the first fix.
 *  Used to dead-reckon aircraft forward between fetches. */
unsigned long lastFetchMs();

}  // namespace services::adsb
