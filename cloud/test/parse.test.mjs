/*
  Round-trip tests for the wire-format parsers.

  The G and T cases are built here byte for byte the way the firmware's
  serialisers write them (gps_manager.cpp updateTransmitMessage,
  thermo_manager.cpp updateTransmitMessage), so a change on either side shows up
  as a failure rather than as silently wrong numbers on the dashboard. The W case
  is a real payload captured from Notehub.

  Run: node --test test/  (or npm test)
*/
import test from "node:test";
import assert from "node:assert/strict";
import { parseMessage, parseWaveMessage, parseGpsMessage, parseTempMessage } from "../src/worker.js";

const SCALE = 100000;

// A real buoy.qo payload: the DEBUG_WAVE_MSG bench message, 144 bytes.
const REAL_WAVE_B64 =
  "VwAFaVW5tWlVwL0AAhcoAAPcSAAKizgABovIAAAg5AAAvrwAlcG1ADMDpAXICPMNexPGHD4nRjUtRhpZ" +
  "+3B0iNWiHbsC0grlsfSQ/Yr/6/uE8K3gPctqs6qagoFfaXlTuUC1MLEjrRlzEa0L9AffBQwDJgHqASIA" +
  "pwBeADMAGwAOAAcAAwACAAEAAAAAAABF";

const bytesOf = (b64) => new Uint8Array(Buffer.from(b64, "base64"));

class Writer {
  constructor(size) {
    this.buf = new Uint8Array(size);
    this.dv = new DataView(this.buf.buffer);
    this.o = 0;
  }
  u8(v) { this.dv.setUint8(this.o, v); this.o += 1; return this; }
  u16(v) { this.dv.setUint16(this.o, v, false); this.o += 2; return this; }
  i32(v) { this.dv.setInt32(this.o, v, false); this.o += 4; return this; }
  u32(v) { this.dv.setUint32(this.o, v, false); this.o += 4; return this; }
  time(v) { this.dv.setBigUint64(this.o, BigInt(v), false); this.o += 8; return this; }
  tag(c) { return this.u8(c.charCodeAt(0)); }
}

// 'G' + u16 reading_ID + 4x i32 (scaled) + u64 time_t + 'E' = GPS_message_size.
function buildGps({ reading_id, lat, lng, vel, direction, timestamp }) {
  const w = new Writer(28);
  w.tag("G").u16(reading_id)
    .i32(Math.round(lat * SCALE)).i32(Math.round(lng * SCALE))
    .i32(Math.round(vel * SCALE)).i32(Math.round(direction * SCALE))
    .time(timestamp).tag("E");
  return w.buf;
}

// 'T' + u16 reading_ID + u8 count + per sensor {sign byte, u32 magnitude}
// + u64 time_t + 'E'. msg_insert_int writes sign-and-magnitude, not two's complement.
function buildTemp({ reading_id, temps, timestamp }) {
  const w = new Writer(13 + 5 * temps.length);
  w.tag("T").u16(reading_id).u8(temps.length);
  for (const c of temps) {
    w.tag(c < 0 ? "N" : "P").u32(Math.abs(Math.round(c * SCALE)));
  }
  w.time(timestamp).tag("E");
  return w.buf;
}

test("wave: real Notehub payload decodes to the bench test values", () => {
  const w = parseWaveMessage(bytesOf(REAL_WAVE_B64));
  assert.equal(w.type, "W");
  assert.equal(w.reading_id, 5);
  assert.equal(w.ts_start, 1767225781);
  assert.equal(w.ts_end - w.ts_start, 1800);
  assert.equal(w.hs.toFixed(2), "1.37");
  assert.equal(w.tc.toFixed(2), "2.53");
  assert.equal(w.tp.toFixed(2), "6.91");
  assert.equal(w.tz.toFixed(2), "4.29");
  assert.equal(w.max_value.toFixed(4), "0.0842");
  assert.equal(w.num_bins, 51);
  assert.equal(w.spectrum.length, 51);
  assert.equal(w.spectrum.indexOf(Math.max(...w.spectrum)), 18);
});

test("wave: a truncated message yields a short spectrum, not a bad parse", () => {
  const w = parseWaveMessage(bytesOf(REAL_WAVE_B64).slice(0, 100));
  assert.equal(w.num_bins, 29);
  assert.equal(w.spectrum.length, 29);
  assert.equal(w.hs.toFixed(2), "1.37");
});

test("gps: round-trips position, speed and heading", () => {
  const src = { reading_id: 41, lat: 59.9139, lng: 10.7522, vel: 1.234, direction: 275.5, timestamp: 1767225781 };
  const g = parseGpsMessage(buildGps(src));
  assert.equal(g.type, "G");
  assert.equal(g.reading_id, 41);
  assert.equal(g.lat.toFixed(5), "59.91390");
  assert.equal(g.lng.toFixed(5), "10.75220");
  assert.equal(g.vel.toFixed(3), "1.234");
  assert.equal(g.direction.toFixed(1), "275.5");
  assert.equal(g.timestamp, 1767225781);
});

test("gps: southern/western positions survive as negative", () => {
  const g = parseGpsMessage(buildGps({ reading_id: 1, lat: -33.918, lng: -18.4233, vel: 0, direction: 0, timestamp: 1 }));
  assert.equal(g.lat.toFixed(4), "-33.9180");
  assert.equal(g.lng.toFixed(4), "-18.4233");
});

test("gps: a message shorter than GPS_message_size is rejected", () => {
  assert.equal(parseGpsMessage(buildGps({ reading_id: 1, lat: 1, lng: 1, vel: 1, direction: 1, timestamp: 1 }).slice(0, 27)), null);
});

test("temperature: sign-and-magnitude decodes both polarities", () => {
  const temps = [7.25, -1.5, 0, 21.375];
  const t = parseTempMessage(buildTemp({ reading_id: 9, temps, timestamp: 1767225781 }));
  assert.equal(t.type, "T");
  assert.equal(t.num_sensors, 4);
  assert.deepEqual(t.temps.map((v) => Number(v.toFixed(3))), temps);
  assert.equal(t.timestamp, 1767225781);
});

test("temperature: declared sensor count is clamped to what arrived", () => {
  const full = buildTemp({ reading_id: 9, temps: [1, 2, 3, 4, 5, 6, 7, 8], timestamp: 5 });
  // Drop three sensors' worth of bytes from the middle of the sensor block.
  const short = full.slice(0, full.length - 15);
  const t = parseTempMessage(short);
  assert.equal(t.num_sensors, 5);
  assert.equal(t.temps.length, 5);
});

test("dispatch: known tags route, unknown tags are dropped", () => {
  assert.equal(parseMessage(bytesOf(REAL_WAVE_B64)).type, "W");
  assert.equal(parseMessage(buildGps({ reading_id: 1, lat: 1, lng: 1, vel: 1, direction: 1, timestamp: 1 })).type, "G");
  assert.equal(parseMessage(buildTemp({ reading_id: 1, temps: [1], timestamp: 1 })).type, "T");
  assert.equal(parseMessage(new Uint8Array([0x41, 0, 0, 0])), null);  // 'A', analog
  assert.equal(parseMessage(new Uint8Array([])), null);
});
