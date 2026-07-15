#include "services/adsb_client.h"

#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

#include <ArduinoJson.h>

#include <cmath>
#include <cstring>

#include "config.h"

namespace services::adsb {

namespace {

constexpr char kApiBase[] = "https://opendata.adsb.fi/api/v3/lat/";
constexpr float kKmPerNm = 1.852f;
constexpr int kConnectAttemptMs = 200;
constexpr unsigned long kRequestTimeoutMs = 10000;

// ---- Route lookup (VRS standing-data static files on GitHub Pages) ----
// https://vrs-standing-data.adsb.lol/routes/<first 2 of callsign>/<callsign>.json
// Plain static JSON, refreshed hourly upstream: no POST body, no API quirks.
constexpr char kRouteBaseUrl[] = "https://vrs-standing-data.adsb.lol/routes/";
/** Callsign->route cache entries kept across refreshes. */
constexpr size_t kRouteCacheSize = 32;
/** Skip route lookups entirely when free heap drops below this (bytes). */
constexpr uint32_t kRouteMinFreeHeap = 60000;
/** Min gap between route lookups. One callsign per window keeps a single TLS
 *  session open at a time; routes are static so there is no rush. */
constexpr unsigned long kRouteFetchIntervalMs = 5000;
unsigned long s_last_route_fetch_ms = 0;

struct RouteCacheEntry {
  char callsign[9];
  char route[10];
  bool used;
};
RouteCacheEntry s_route_cache[kRouteCacheSize] = {};
size_t s_route_cache_next = 0;

Aircraft s_aircraft[kMaxAircraft];
size_t s_aircraft_count = 0;
PollFn s_poll_fn = nullptr;

void pollNetwork() {
  if (s_poll_fn != nullptr) {
    s_poll_fn();
  }
}

int performGetWithPoll(HTTPClient& http) {
  http.setConnectTimeout(kConnectAttemptMs);
  const unsigned long deadline = millis() + kRequestTimeoutMs;
  while (millis() < deadline) {
    pollNetwork();
    const int code = http.GET();
    if (code > 0) {
      return code;
    }
    if (code != HTTPC_ERROR_CONNECTION_REFUSED &&
        code != HTTPC_ERROR_NOT_CONNECTED) {
      return code;
    }
    delay(5);
  }
  return HTTPC_ERROR_READ_TIMEOUT;
}

bool readResponseBodyWithPoll(HTTPClient& http, String& payload) {
  WiFiClient* stream = http.getStreamPtr();
  if (stream == nullptr) {
    return false;
  }

  const int content_length = http.getSize();
  if (content_length > 0) {
    payload.reserve(static_cast<unsigned>(content_length + 1));
  }

  uint8_t buffer[512];
  const unsigned long deadline = millis() + kRequestTimeoutMs;
  while (millis() < deadline) {
    pollNetwork();
    const int available = stream->available();
    if (available > 0) {
      const int to_read =
          available > static_cast<int>(sizeof(buffer)) ? static_cast<int>(sizeof(buffer))
                                                       : available;
      const int read_bytes = stream->readBytes(buffer, to_read);
      if (read_bytes > 0) {
        payload.concat(reinterpret_cast<const char*>(buffer),
                       static_cast<unsigned>(read_bytes));
      }
    }
    if (content_length > 0 &&
        static_cast<int>(payload.length()) >= content_length) {
      break;
    }
    if (!http.connected() && stream->available() <= 0) {
      break;
    }
    delay(1);
  }

  return payload.length() > 0;
}

float kmToNauticalMiles(float km) { return km / kKmPerNm; }

bool readJsonFloat(const JsonObject& obj, const char* key, float* out) {
  if (obj[key].is<float>() || obj[key].is<double>() || obj[key].is<int>()) {
    *out = obj[key].as<float>();
    return true;
  }
  return false;
}

float pickNoseHeading(const JsonObject& plane) {
  float v = 0.0f;
  if (readJsonFloat(plane, "true_heading", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "mag_heading", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "track", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "dir", &v)) {
    return v;
  }
  return 0.0f;
}

float pickTrackHeading(const JsonObject& plane) {
  float v = 0.0f;
  if (readJsonFloat(plane, "track", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "true_heading", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "mag_heading", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "dir", &v)) {
    return v;
  }
  return 0.0f;
}

float pickGroundSpeed(const JsonObject& plane) {
  float v = 0.0f;
  if (readJsonFloat(plane, "gs", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "tas", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "ias", &v)) {
    return v;
  }
  return 0.0f;
}

bool isOnGround(const JsonObject& plane) {
  if (!plane["alt_baro"].is<const char*>()) {
    return false;
  }
  return strcmp(plane["alt_baro"].as<const char*>(), "ground") == 0;
}

void copyJsonStringTrimmed(const JsonObject& obj, const char* key, char* out,
                           size_t out_len) {
  out[0] = '\0';
  if (out_len == 0 || !obj[key].is<const char*>()) {
    return;
  }
  const char* s = obj[key].as<const char*>();
  size_t n = strnlen(s, out_len - 1);
  while (n > 0 && s[n - 1] == ' ') {
    --n;
  }
  memcpy(out, s, n);
  out[n] = '\0';
}

void formatAltitudeTag(const JsonObject& plane, char* out, size_t out_len) {
  out[0] = '\0';
  if (out_len == 0) {
    return;
  }

  if (plane["alt_baro"].is<const char*>()) {
    const char* s = plane["alt_baro"].as<const char*>();
    if (strcmp(s, "ground") == 0) {
      strncpy(out, "GND", out_len - 1);
      out[out_len - 1] = '\0';
      return;
    }
  }

  float alt = 0.0f;
  if (readJsonFloat(plane, "alt_baro", &alt) ||
      readJsonFloat(plane, "alt_geom", &alt)) {
    snprintf(out, out_len, "%d m", static_cast<int>(lroundf(alt * 0.3048f)));
  }
}

void fillTagFields(Aircraft* ac, const JsonObject& plane) {
  copyJsonStringTrimmed(plane, "flight", ac->callsign, sizeof(ac->callsign));
  if (ac->callsign[0] == '\0') {
    copyJsonStringTrimmed(plane, "hex", ac->callsign, sizeof(ac->callsign));
  }

  copyJsonStringTrimmed(plane, "t", ac->type, sizeof(ac->type));
  formatAltitudeTag(plane, ac->alt, sizeof(ac->alt));
  ac->route[0] = '\0';  // filled later by fetchRoutes()

  float rate = 0.0f;
  if (readJsonFloat(plane, "baro_rate", &rate) ||
      readJsonFloat(plane, "geom_rate", &rate)) {
    ac->vrate_fpm = rate;
  } else {
    ac->vrate_fpm = NAN;
  }
}

const char* routeCacheFind(const char* callsign) {
  for (const RouteCacheEntry& e : s_route_cache) {
    if (e.used && strcmp(e.callsign, callsign) == 0) {
      return e.route;
    }
  }
  return nullptr;
}

void routeCachePut(const char* callsign, const char* route) {
  RouteCacheEntry& e = s_route_cache[s_route_cache_next];
  s_route_cache_next = (s_route_cache_next + 1) % kRouteCacheSize;
  e.used = true;
  strlcpy(e.callsign, callsign, sizeof(e.callsign));
  strlcpy(e.route, route, sizeof(e.route));
}

/** "CLJ-LTN" / "EGKK-LROP-OMDB" -> "CLJ>LTN" (first>last); "unknown" -> "". */
void buildRouteDisplay(const char* codes, char* out, size_t out_len) {
  out[0] = '\0';
  if (codes == nullptr || strcmp(codes, "unknown") == 0) {
    return;
  }
  const char* dash = strchr(codes, '-');
  if (dash == nullptr) {
    return;
  }
  const char* last = strrchr(codes, '-') + 1;
  size_t first_len = static_cast<size_t>(dash - codes);
  if (first_len > 4) {
    first_len = 4;
  }
  const size_t last_len = strnlen(last, 4);
  if (first_len + 1 + last_len + 1 > out_len) {
    return;
  }
  memcpy(out, codes, first_len);
  out[first_len] = '>';
  memcpy(out + first_len + 1, last, last_len);
  out[first_len + 1 + last_len] = '\0';
}

/** One POST for all uncached callsigns; fills Aircraft::route from cache/API. */
/** Resolves at most one callsign per window from the static VRS route files. */
void fetchRoutes() {
  // Fill everything the cache already knows; note the first unresolved one.
  int pending = -1;
  for (size_t i = 0; i < s_aircraft_count; ++i) {
    Aircraft& a = s_aircraft[i];
    if (a.callsign[0] == '\0') {
      continue;
    }
    const char* cached = routeCacheFind(a.callsign);
    if (cached != nullptr) {
      strlcpy(a.route, cached, sizeof(a.route));
      continue;
    }
    if (pending < 0 && strnlen(a.callsign, 3) >= 2) {
      pending = static_cast<int>(i);
    }
  }
  if (pending < 0) {
    return;
  }
  if (s_last_route_fetch_ms != 0 &&
      millis() - s_last_route_fetch_ms < kRouteFetchIntervalMs) {
    return;  // the rest resolve over the next windows
  }
  if (ESP.getFreeHeap() < kRouteMinFreeHeap) {
    return;  // radar first; try again when memory allows
  }
  s_last_route_fetch_ms = millis();

  char cs[9];
  strlcpy(cs, s_aircraft[pending].callsign, sizeof(cs));

  char url[96];
  snprintf(url, sizeof(url), "%s%c%c/%s.json", kRouteBaseUrl, cs[0], cs[1], cs);

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  if (!http.begin(client, url)) {
    return;
  }
  http.setTimeout(kRequestTimeoutMs);
  const int code = http.GET();
  if (code == HTTP_CODE_NOT_FOUND) {
    routeCachePut(cs, "");  // no such route: negative-cache, stop asking
    http.end();
    return;
  }
  if (code < 200 || code >= 300) {
    Serial.printf("route: HTTP %d\n", code);
    http.end();
    return;
  }

  String resp = http.getString();
  http.end();
  if (resp.length() == 0) {
    Serial.println("route: empty body");
    return;
  }

  JsonDocument filter;
  filter["_airport_codes_iata"] = true;
  JsonDocument doc;
  const DeserializationError err =
      deserializeJson(doc, resp, DeserializationOption::Filter(filter));
  if (err) {
    Serial.printf("route: JSON %s len=%u head=%.60s\n", err.c_str(),
                  static_cast<unsigned>(resp.length()), resp.c_str());
    return;
  }

  const char* codes = doc["_airport_codes_iata"] | "unknown";
  char route[10];
  buildRouteDisplay(codes, route, sizeof(route));
  routeCachePut(cs, route);
  for (size_t i = 0; i < s_aircraft_count; ++i) {
    if (strcmp(s_aircraft[i].callsign, cs) == 0) {
      strlcpy(s_aircraft[i].route, route, sizeof(s_aircraft[i].route));
    }
  }
  Serial.printf("route: %s -> %s\n", cs, route[0] != '\0' ? route : "(unknown)");
}

}  // namespace

void setPollFn(PollFn fn) { s_poll_fn = fn; }

size_t aircraftCount() { return s_aircraft_count; }

const Aircraft* aircraftList() { return s_aircraft; }

/** Fetches + parses the aircraft list. Keeps payload/doc scoped to this call so
 *  they are freed before any further TLS session is opened. */
bool fetchAircraftList(double center_lat, double center_lon,
                       float fetch_radius_km) {
  const float dist_nm = kmToNauticalMiles(fetch_radius_km);

  String url = kApiBase;
  url += String(center_lat, 6);
  url += "/lon/";
  url += String(center_lon, 6);
  url += "/dist/";
  url += String(dist_nm, 1);

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  if (!http.begin(client, url)) {
    Serial.println("adsb: http.begin failed");
    return false;
  }

  http.setTimeout(kRequestTimeoutMs);
  const int code = performGetWithPoll(http);
  if (code != HTTP_CODE_OK) {
    Serial.printf("adsb: HTTP %d\n", code);
    http.end();
    return false;
  }

  String payload;
  if (!readResponseBodyWithPoll(http, payload)) {
    Serial.println("adsb: empty response");
    http.end();
    return false;
  }
  http.end();

  JsonDocument doc;
  const DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.printf("adsb: JSON parse error: %s\n", err.c_str());
    return false;
  }

  JsonArray ac = doc["ac"].as<JsonArray>();
  if (ac.isNull()) {
    s_aircraft_count = 0;
    return true;
  }

  size_t n = 0;
  for (JsonObject plane : ac) {
    if (n >= kMaxAircraft) {
      break;
    }
    if (!plane["lat"].is<float>() || !plane["lon"].is<float>()) {
      continue;
    }
    if (isOnGround(plane) && !config::kAdsbShowGroundAircraft) {
      continue;
    }

    s_aircraft[n].lat = plane["lat"].as<float>();
    s_aircraft[n].lon = plane["lon"].as<float>();
    s_aircraft[n].nose_deg = pickNoseHeading(plane);
    s_aircraft[n].track_deg = pickTrackHeading(plane);
    s_aircraft[n].gs_knots = pickGroundSpeed(plane);
    fillTagFields(&s_aircraft[n], plane);
    ++n;
  }

  s_aircraft_count = n;
  Serial.printf("adsb: %u aircraft\n", static_cast<unsigned>(n));
  return true;
}

bool fetchUpdate(double center_lat, double center_lon, float fetch_radius_km) {
  // Aircraft first: its payload/doc/TLS are all released when this returns.
  if (!fetchAircraftList(center_lat, center_lon, fetch_radius_km)) {
    return false;
  }
  // Only now, on a clean heap, look up routes.
  fetchRoutes();
  return true;
}

}  // namespace services::adsb
