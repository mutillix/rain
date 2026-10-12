A handheld RF radar for the ESP32-S3. It listens — BLE advertisements,
ESP-NOW frames, probe requests, WiFi beacons — and shows you what's
around you, how far away it is, and how strong it's screaming.

Then it turns it into a game.

## The Hunter

Every new device you catch scores points. First catches only — no
farming the same AirTag in your junk drawer.

- **Threshold:** -60 dBm held for 3 seconds
- **Base score:** `(peakRSSI + 100) × 10`
- **Rarity tiers:** common ×1, uncommon ×2, rare ×3, legendary ×5
  (by OUI — that random Espressif board nobody's seen before pays out)
- **Combos:** catches within 60 seconds chain ×2 through ×5
- **Bonuses:** +50 for default SSIDs, +30 for personal-best RSSI
- **14 badges:** per-band pioneers, count milestones, tracker spotters
  (AirTag/Tile/SmartTag), Night Owl, Marathon, Globetrotter
- **Streaks:** consecutive days with at least one new catch

All offline. All persisted. No cloud, no account, no phone home.

## Views

BOOT tap cycles through 12: Matrix rain → scope → list → hunter →
waterfall → strip chart → density → detail → GPS → tracker → find →
credits. Hold BOOT to switch bands (BLE / ESP-NOW / WiFi).

## Hardware

- Hosyond ESP32-S3 LCD board (320×480, SDIO)
- NEO-6M GPS (TX→45, RX→46)
- SD card for CSV logging (`/ble_log.csv`)
- WS2812 deliberately off

## Logging

Every catch hits the SD card with GPS stamp. `radar_kml.py` converts
the log to KML for Google Earth — your wardriving sessions as a map.

## Building

1. Arduino IDE, ESP32 core 3.x
2. **Pin `ESP32_Display_Panel` at 0.2.2** — v1.x broke the headers
3. Select the correct board (not the S3 if you're building for P4)
4. Flash, tap BOOT, go outside

## License

MIT. Do what you can.# BLE RADAR

A handheld RF radar for the ESP32-S3. It listens — BLE advertisements,
ESP-NOW frames, probe requests, WiFi beacons — and shows you what's
around you, how far away it is, and how strong it's screaming.

Then it turns it into a game.

## The Hunter

Every new device you catch scores points. First catches only — no
farming the same AirTag in your junk drawer.

- **Threshold:** -60 dBm held for 3 seconds
- **Base score:** `(peakRSSI + 100) × 10`
- **Rarity tiers:** common ×1, uncommon ×2, rare ×3, legendary ×5
  (by OUI — that random Espressif board nobody's seen before pays out)
- **Combos:** catches within 60 seconds chain ×2 through ×5
- **Bonuses:** +50 for default SSIDs, +30 for personal-best RSSI
- **14 badges:** per-band pioneers, count milestones, tracker spotters
  (AirTag/Tile/SmartTag), Night Owl, Marathon, Globetrotter
- **Streaks:** consecutive days with at least one new catch

All offline. All persisted. No cloud, no account, no phone home.

## Views

BOOT tap cycles through 12: Matrix rain → scope → list → hunter →
waterfall → strip chart → density → detail → GPS → tracker → find →
credits. Hold BOOT to switch bands (BLE / ESP-NOW / WiFi).

## Hardware

- Hosyond ESP32-S3 LCD board (320×480, SDIO)
- NEO-6M GPS (TX→45, RX→46)
- SD card for CSV logging (`/ble_log.csv`)
- WS2812 deliberately off

## Logging

Every catch hits the SD card with GPS stamp. `radar_kml.py` converts
the log to KML for Google Earth — your wardriving sessions as a map.

## Building

1. Arduino IDE, ESP32 core 3.x
2. **Pin `ESP32_Display_Panel` at 0.2.2** — v1.x broke the headers
3. Select the correct board
4. Flash, tap BOOT, go outside

## License

MIT. Do what you can.
