/*
  Ingest and read API for the messages the base station forwards to Notehub.

  Three message types are decoded, all framed <tag> ... 'E' with big-endian
  integers and fixed-point floats:

    'W'  wave analysis     parse_wave_analysis_message
    'G'  GPS fix           parse_gps_message
    'T'  temperature       parse_temperature_message

  The wire formats are the firmware's own - see
  common_libraries/message_tools/src/message_parser.cpp. Timestamps are 8 bytes:
  time_t is 64-bit in the ARM toolchain this firmware is built with, and the G
  and T serialisers write it at its natural width (the wave message is the
  exception - it casts its two timestamps down to uint32 explicitly).

  buoy.qo carries every type at once, so the Notehub route cannot filter by
  message type. The tag byte is the filter, and an unrecognised message is
  acknowledged and dropped.
*/

const SCALE_FACTOR = 100000;       // scale_factor, firmware config.h
const WAVE_FREQ_SCALE = 10000000;  // wave_freq_scale, readings.h
const WAVE_HEADER_BYTES = 41;      // 'W' through num_bins, inclusive
const GPS_MESSAGE_BYTES = 28;      // 'G' + u16 + 4x i32 + u64 + 'E'
const TEMP_HEADER_BYTES = 4;       // 'T' + u16 + num_sensors
const TEMP_TAIL_BYTES = 9;         // u64 timestamp + 'E'
const TEMP_SENSOR_BYTES = 5;       // sign byte + u32 magnitude

const TAG_WAVE = 0x57;  // 'W'
const TAG_GPS = 0x47;   // 'G'
const TAG_TEMP = 0x54;  // 'T'
const SIGN_NEGATIVE = 0x4e;  // 'N'

function json(data, status = 200) {
  return new Response(JSON.stringify(data), {
    status,
    headers: { "content-type": "application/json; charset=utf-8" },
  });
}

function decodeBase64(s) {
  const bin = atob(s);
  const out = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
  return out;
}

// Cursor over a message, big-endian throughout.
function reader(bytes) {
  const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  let o = 1;  // every parse starts past the tag byte
  return {
    u8: () => dv.getUint8(o++),
    u16: () => { const v = dv.getUint16(o, false); o += 2; return v; },
    u32: () => { const v = dv.getUint32(o, false); o += 4; return v; },
    i32: () => { const v = dv.getInt32(o, false); o += 4; return v; },
    // time_t is 8 bytes on the sender. Epochs are far inside Number's exact
    // integer range, so the BigInt only exists for the width of the read.
    time: () => { const v = dv.getBigUint64(o, false); o += 8; return Number(v); },
    at: () => o,
  };
}

/*
  Wave analysis. num_bins is taken from the wire and clamped to what actually
  arrived, so a truncated packet yields a short spectrum rather than a parse past
  the end of the buffer.
*/
export function parseWaveMessage(bytes) {
  if (bytes.length < WAVE_HEADER_BYTES || bytes[0] !== TAG_WAVE) return null;
  const r = reader(bytes);

  const reading_id = r.u16();
  const ts_start = r.u32();
  const ts_end = r.u32();
  const hs = r.u32() / SCALE_FACTOR;
  const tc = r.u32() / SCALE_FACTOR;
  const tp = r.u32() / SCALE_FACTOR;
  const tz = r.u32() / SCALE_FACTOR;
  const max_value = r.u32() / SCALE_FACTOR;
  const f_min = r.u32() / WAVE_FREQ_SCALE;
  const f_max = r.u32() / WAVE_FREQ_SCALE;
  const declared_bins = r.u16();

  // One trailing byte is the 'E' terminator, so it is not spectrum data.
  const available_bins = Math.max(0, Math.floor((bytes.length - r.at() - 1) / 2));
  const num_bins = Math.min(declared_bins, available_bins);

  const spectrum = new Array(num_bins);
  for (let i = 0; i < num_bins; i++) spectrum[i] = r.u16();

  return { type: "W", reading_id, ts_start, ts_end, hs, tc, tp, tz, max_value, f_min, f_max, num_bins, spectrum };
}

/*
  GPS fix. All four values are int32 scaled by scale_factor.

  Reading them as SIGNED is deliberate even though the firmware's own parser uses
  uint32: the reading struct's fields are int32_t, so a southern or western
  position is meant to come back negative. (The drifter currently casts a
  negative double straight to uint32 when it builds the reading, which saturates
  to 0 on this target - so those coordinates arrive as 0, not as a negative
  number. Reading signed here is still correct, and becomes correct end to end
  the day that cast is fixed.)
*/
export function parseGpsMessage(bytes) {
  if (bytes.length < GPS_MESSAGE_BYTES || bytes[0] !== TAG_GPS) return null;
  const r = reader(bytes);

  const reading_id = r.u16();
  const lat = r.i32() / SCALE_FACTOR;
  const lng = r.i32() / SCALE_FACTOR;
  const vel = r.i32() / SCALE_FACTOR;
  const direction = r.i32() / SCALE_FACTOR;
  const timestamp = r.time();

  return { type: "G", reading_id, lat, lng, vel, direction, timestamp };
}

/*
  Temperature string. Each sensor is a sign byte ('P'/'N') followed by the
  magnitude as uint32 - msg_insert_int's encoding, not two's complement.
  num_sensors is clamped to what arrived, same reasoning as the spectrum.
*/
export function parseTempMessage(bytes) {
  if (bytes.length < TEMP_HEADER_BYTES + TEMP_TAIL_BYTES || bytes[0] !== TAG_TEMP) return null;
  const r = reader(bytes);

  const reading_id = r.u16();
  const declared_sensors = r.u8();

  const room = bytes.length - r.at() - TEMP_TAIL_BYTES;
  const num_sensors = Math.min(declared_sensors, Math.max(0, Math.floor(room / TEMP_SENSOR_BYTES)));

  const temps = new Array(num_sensors);
  for (let i = 0; i < num_sensors; i++) {
    const sign = r.u8() === SIGN_NEGATIVE ? -1 : 1;
    temps[i] = (sign * r.u32()) / SCALE_FACTOR;
  }

  const timestamp = r.time();
  return { type: "T", reading_id, num_sensors, temps, timestamp };
}

export function parseMessage(bytes) {
  if (!bytes.length) return null;
  switch (bytes[0]) {
    case TAG_WAVE: return parseWaveMessage(bytes);
    case TAG_GPS: return parseGpsMessage(bytes);
    case TAG_TEMP: return parseTempMessage(bytes);
    default: return null;
  }
}

const INSERT_WAVE = `
  INSERT OR REPLACE INTO wave
    (buoy_id, ts_start, ts_end, reading_id, hs, tc, tp, tz,
     max_value, f_min, f_max, num_bins, spectrum, rssi, device, received)
  VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)`;

const INSERT_GPS = `
  INSERT OR REPLACE INTO gps
    (buoy_id, timestamp, reading_id, lat, lng, vel, direction, rssi, device, received)
  VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)`;

const INSERT_TEMP = `
  INSERT OR REPLACE INTO temperature
    (buoy_id, timestamp, reading_id, num_sensors, temps, rssi, device, received)
  VALUES (?, ?, ?, ?, ?, ?, ?, ?)`;

function statementFor(env, msg, meta) {
  const { buoyId, rssi, device, received } = meta;
  if (msg.type === "W") {
    return env.DB.prepare(INSERT_WAVE).bind(
      buoyId, msg.ts_start, msg.ts_end, msg.reading_id,
      msg.hs, msg.tc, msg.tp, msg.tz,
      msg.max_value, msg.f_min, msg.f_max, msg.num_bins,
      JSON.stringify(msg.spectrum), rssi, device, received,
    );
  }
  if (msg.type === "G") {
    return env.DB.prepare(INSERT_GPS).bind(
      buoyId, msg.timestamp, msg.reading_id,
      msg.lat, msg.lng, msg.vel, msg.direction, rssi, device, received,
    );
  }
  return env.DB.prepare(INSERT_TEMP).bind(
    buoyId, msg.timestamp, msg.reading_id, msg.num_sensors,
    JSON.stringify(msg.temps), rssi, device, received,
  );
}

async function handleIngest(request, env) {
  if (request.method !== "POST") return json({ error: "POST only" }, 405);
  if (!env.INGEST_SECRET) return json({ error: "INGEST_SECRET is not configured" }, 500);
  if (request.headers.get("x-ingest-secret") !== env.INGEST_SECRET) {
    return json({ error: "forbidden" }, 403);
  }

  let payload;
  try {
    payload = await request.json();
  } catch {
    return json({ error: "body is not JSON" }, 400);
  }

  const events = Array.isArray(payload) ? payload : [payload];
  const statements = [];
  const stored = { W: 0, G: 0, T: 0 };
  let skipped = 0;

  for (const ev of events) {
    const body = ev && ev.body;
    if (!body || typeof body.payload !== "string") { skipped++; continue; }

    let msg = null;
    try {
      msg = parseMessage(decodeBase64(body.payload));
    } catch {
      msg = null;
    }
    if (!msg) { skipped++; continue; }

    statements.push(statementFor(env, msg, {
      buoyId: Number.isFinite(body.id) ? body.id : 0,
      rssi: Number.isFinite(body.rssi) ? Math.round(body.rssi) : null,
      device: typeof ev.device === "string" ? ev.device : null,
      received: Math.round(Number(ev.received) || Date.now() / 1000),
    }));
    stored[msg.type]++;
  }

  if (statements.length) await env.DB.batch(statements);

  // 200 even when nothing was stored: an unrecognised note is a normal thing to
  // see, and a non-2xx would only make Notehub retry it forever.
  return json({ ok: true, stored, skipped });
}

/*
  Shared read path for the three tables. `hours` filters on `received` - the
  Notehub receipt time, which is real even when the buoy's own RTC is not.
*/
async function handleSeries(url, env, { table, timeColumn, jsonColumns = [] }) {
  const buoy = url.searchParams.get("buoy");
  const hours = Number(url.searchParams.get("hours"));
  const limit = Math.min(Math.max(Number(url.searchParams.get("limit")) || 1000, 1), 5000);

  const where = [];
  const binds = [];
  if (buoy !== null && buoy !== "" && buoy !== "all") {
    where.push("buoy_id = ?");
    binds.push(Number(buoy));
  }
  if (Number.isFinite(hours) && hours > 0) {
    where.push("received >= ?");
    binds.push(Math.round(Date.now() / 1000 - hours * 3600));
  }

  const sql = `SELECT * FROM ${table}
               ${where.length ? "WHERE " + where.join(" AND ") : ""}
               ORDER BY ${timeColumn} DESC LIMIT ?`;
  const { results } = await env.DB.prepare(sql).bind(...binds, limit).all();

  // Newest-first in SQL so LIMIT keeps the most recent rows; oldest-first out,
  // which is the order the charts plot in.
  const rows = results.reverse().map((r) => {
    const out = { ...r };
    for (const c of jsonColumns) out[c] = JSON.parse(r[c]);
    return out;
  });

  return json({ rows });
}

async function handleBuoys(env) {
  // A buoy that has only ever sent GPS still belongs in the picker, so the list
  // is the union across the three tables.
  const { results } = await env.DB.prepare(
    `SELECT buoy_id, SUM(n) AS readings, MAX(last_seen) AS last_seen FROM (
       SELECT buoy_id, COUNT(*) AS n, MAX(received) AS last_seen FROM wave GROUP BY buoy_id
       UNION ALL
       SELECT buoy_id, COUNT(*), MAX(received) FROM gps GROUP BY buoy_id
       UNION ALL
       SELECT buoy_id, COUNT(*), MAX(received) FROM temperature GROUP BY buoy_id
     ) GROUP BY buoy_id ORDER BY buoy_id`,
  ).all();
  return json({ buoys: results });
}

export default {
  async fetch(request, env) {
    const url = new URL(request.url);
    try {
      if (url.pathname === "/api/ingest") return await handleIngest(request, env);
      if (url.pathname === "/api/wave") {
        return await handleSeries(url, env, { table: "wave", timeColumn: "ts_start", jsonColumns: ["spectrum"] });
      }
      if (url.pathname === "/api/gps") {
        return await handleSeries(url, env, { table: "gps", timeColumn: "timestamp" });
      }
      if (url.pathname === "/api/temperature") {
        return await handleSeries(url, env, { table: "temperature", timeColumn: "timestamp", jsonColumns: ["temps"] });
      }
      if (url.pathname === "/api/buoys") return await handleBuoys(env);
    } catch (err) {
      return json({ error: String(err && err.message ? err.message : err) }, 500);
    }
    return json({ error: "not found" }, 404);
  },
};
