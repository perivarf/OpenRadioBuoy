-- D1 schema for wave-analysis messages routed out of Notehub.
--
-- One row per wave measurement. (buoy_id, ts_start) is the primary key so a
-- re-delivered Notehub event overwrites its own row instead of duplicating it -
-- routes retry, and the same note can arrive twice.
--
-- The spectrum stays as the raw 0-65535 wire values in a JSON array. Absolute
-- PSD is value/65535 * max_value, done at read time: storing the raw values
-- keeps the row small and loses nothing.

CREATE TABLE IF NOT EXISTS wave (
  buoy_id    INTEGER NOT NULL,
  ts_start   INTEGER NOT NULL,   -- epoch s, start of the analysis window
  ts_end     INTEGER NOT NULL,   -- epoch s, end of the analysis window
  reading_id INTEGER,
  hs         REAL,               -- significant wave height, m
  tc         REAL,               -- crest period, s
  tp         REAL,               -- peak period, s
  tz         REAL,               -- zero-crossing period, s
  max_value  REAL,               -- peak elevation PSD, m^2/Hz
  f_min      REAL,               -- first bin centre, Hz
  f_max      REAL,               -- last bin centre, Hz
  num_bins   INTEGER NOT NULL,
  spectrum   TEXT NOT NULL,      -- JSON array of num_bins uint16
  rssi       INTEGER,
  device     TEXT,               -- Notehub device UID
  received   INTEGER NOT NULL,   -- epoch s, when Notehub received the note
  PRIMARY KEY (buoy_id, ts_start)
);

-- The dashboard's time filter runs on `received`, not ts_start: a bench unit has
-- no GPS and never syncs its RTC, so ts_start is a synthetic epoch there while
-- received is always real.
CREATE INDEX IF NOT EXISTS idx_wave_received ON wave(received);

-- GPS fixes ('G'). lat/lng/vel/direction arrive as int32 scaled by scale_factor
-- and are stored as the real units: degrees, m/s, degrees.
CREATE TABLE IF NOT EXISTS gps (
  buoy_id    INTEGER NOT NULL,
  timestamp  INTEGER NOT NULL,   -- epoch s, from the GPS fix
  reading_id INTEGER,
  lat        REAL,
  lng        REAL,
  vel        REAL,               -- speed over ground, m/s
  direction  REAL,               -- heading of motion, degrees
  rssi       INTEGER,
  device     TEXT,
  received   INTEGER NOT NULL,
  PRIMARY KEY (buoy_id, timestamp)
);

CREATE INDEX IF NOT EXISTS idx_gps_received ON gps(received);

-- Temperature strings ('T'). One row per reading; `temps` is a JSON array of
-- degrees Celsius, num_sensors long, in sensor order.
CREATE TABLE IF NOT EXISTS temperature (
  buoy_id     INTEGER NOT NULL,
  timestamp   INTEGER NOT NULL,  -- epoch s
  reading_id  INTEGER,
  num_sensors INTEGER NOT NULL,
  temps       TEXT NOT NULL,     -- JSON array of REAL, degrees Celsius
  rssi        INTEGER,
  device      TEXT,
  received    INTEGER NOT NULL,
  PRIMARY KEY (buoy_id, timestamp)
);

CREATE INDEX IF NOT EXISTS idx_temperature_received ON temperature(received);
