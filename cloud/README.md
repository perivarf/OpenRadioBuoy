# ORB wave dashboard (Cloudflare Worker + D1)

Receives wave-analysis (`W`) messages routed out of Notehub, stores them in D1,
and serves a dashboard. Free tier throughout: a buoy transmitting every 10
minutes produces ~144 requests/day, against a 100k/day limit.

```
drifter --LoRa--> base station --Notecard--> Notehub --route--> /api/ingest --> D1
                                                                                 |
                                                        dashboard <-- /api/wave -+
```

| File | What it is |
|---|---|
| `src/worker.js` | ingest + read API, and the wire-format parser |
| `schema.sql` | D1 table |
| `public/index.html` | dashboard, no build step and no dependencies |
| `wrangler.toml` | Worker config — `database_id` must be filled in |

## Setup

Requires Node (for `wrangler`) and a free Cloudflare account.

```bash
# 1. database
npx wrangler d1 create orb-wave          # paste database_id into wrangler.toml
npx wrangler d1 execute orb-wave --remote --file=./schema.sql

# 2. the shared secret the Notehub route will send
npx wrangler secret put INGEST_SECRET    # paste a long random string

# 3. deploy
npx wrangler deploy                      # prints https://orb-wave.<subdomain>.workers.dev
```

Then in Notehub: **Routes → Create Route → HTTP/HTTPS**, URL
`https://<your-worker>/api/ingest`, method POST, add the header
`X-Ingest-Secret: <the same string>`, and restrict the route to the `buoy.qo`
notefile. Leave the transform as the default JSON — the Worker expects Notehub's
own event shape.

## Endpoints

| Route | Purpose |
|---|---|
| `POST /api/ingest` | Notehub route target. Requires the `X-Ingest-Secret` header. |
| `GET /api/wave?buoy=&hours=&limit=` | measurements, oldest first, spectrum expanded |
| `GET /api/buoys` | buoy ids with reading counts and last-seen |
| `GET /` | the dashboard |

## Two things worth knowing

**`buoy.qo` carries every message type.** G, T, A and W all share the notefile,
so the route cannot filter by type. `/api/ingest` filters on the `W` tag byte and
answers 200 for the rest — a non-2xx would make Notehub retry them forever.

**The time filter runs on `received`, not `ts_start`.** A bench unit built with
`DEBUG_WAVE_MSG` has no GPS, never syncs its RTC, and stamps its measurements
with a synthetic epoch (2026-01-01 plus uptime). Filtering on the measurement
timestamp would hide that data; the charts still plot against `ts_start`, so an
unsynced RTC is visible rather than silently corrected.
