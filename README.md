Arduino library for the Quintic QN8027, a single-chip stereo FM transmitter with RDS. It talks to the chip over I2C at address `0x2C`.
Register values follow the QN8027 datasheet. The public API is the `QN8027` class: gains, clock, deviation, stereo, and RDS are set with enums and plain values instead of raw bit masks.
Martin Ondrak, 2017–2018. Refactored 2026.
FM transmission is legal only with a license, or at a very low power. You are responsible for the rules where you use it.
## What it sets
Call the setters before `begin()`. `begin()` runs the chip's reset sequence and writes the values you stored. After `begin()`, the same setters update the chip immediately.
| Setting | Method | Default |
| --- | --- | --- |
| Frequency | `setFrequency(MHz)` | `88.9` (`FREQ_DEFAULT`) |
| FM deviation | `setDeviationKhz` | 75 kHz |
| TX power | `setTxPower` | 75, clamped to 20–75 |
| Input PGA | `setInputGain` | 0 dB (`Db0`, `Db3`, `Db6`, `Db9`) |
| Digital gain | `setDigitalGain` | 0 dB, same steps |
| Input resistance | `setInputResistance` | 40 kΩ (`Ohm80k`, `Ohm40k`, `Ohm20k`, `Ohm10k`) |
| Pre-emphasis | `setPreEmphasis` | 50 µs (`Us50` or `Us75`) |
| Stereo pilot | `setPilotDeviation` | `PilotCode` (9) |
| Power-save timer | `setPowerSave` | `Disabled` |
| Crystal | `setCrystal` | 12 MHz |
| Clock | `setClockSource` | crystal, drive `0x0F` |
| Stereo / mute | `setStereo`, `mute` | stereo, unmuted |
Frequency is 76.0–108.0 MHz in 50 kHz steps. `enable(true)` starts the transmitter; `begin()` does that for you.
## RDS
RDS is off until `setRdsEnabled(true)`. The chip holds one RDS group and repeats it, so the sketch must call `rdsRefresh()` on a timer. Each call sends one group, alternating:
- **Group 0A, program service.** The 8-character station name, two characters at a time. `setProgramService` space-pads a shorter name and restarts the four segments.
- **Group 2A, radiotext.** Up to 64 characters, four per segment. Each segment is sent twice before the next one. `setRadiotext` ignores an unchanged string. A new string toggles the A/B flag so receivers drop the old text.
Also set `setRdsPi` (default `0x4000`), `setProgramType` (the RDS program-type list, default `None`), and `setRdsDeviationKhz` (default 3.5 kHz). The music/speech bit is music. Group 0A carries no alternate frequencies. On the first program-service segment the decoder-info bit follows `setStereo`.
The bytes are sent as you pass them. RDS receivers expect the basic RDS character set, which is essentially ASCII. Fold accents to plain letters before calling `setProgramService` or `setRadiotext`.
`rdsTick()` and `getCurrentRds()` walk the radiotext buffer in 8-character slices. A sketch that pages the name itself, as the example below does, does not need them.
```cpp
QN8027 fm;
void setup() {
  fm.setCrystal(QN8027::Crystal::Mhz12);
  fm.setClockSource(QN8027::ClockSource::Crystal);
  fm.setInputGain(QN8027::InputGain::Db0);
  fm.setDigitalGain(QN8027::DigitalGain::Db0);
  fm.setPreEmphasis(QN8027::PreEmphasis::Us50);
  fm.setPowerSave(QN8027::PowerSave::Disabled);
  fm.setPilotDeviation(QN8027::PilotCode);
  fm.setDeviationKhz(75);
  fm.setStereo(true);
  fm.setTxPower(75);
  fm.setRdsPi(0x4000);
  fm.setProgramType(QN8027::ProgramType::Pop);
  fm.setRdsDeviationKhz(3.5f);
  fm.setRdsEnabled(true);
  fm.begin(16, 17, 100000, 88.9f);   // ESP32: SDA, SCL, I2C Hz, MHz
  fm.setProgramService("ESP32_FM");
  fm.setRadiotext("ESP32_FM");
}
void loop() {
  fm.rdsRefresh();
  delay(200);
}
```
On boards other than ESP32 and ESP8266, use `begin(fmFreq)`. That calls `Wire.begin()` with the board's default pins.
## FM DevKit example
`src/main.cpp` in the ESP32 FM DevKit firmware is the working test of this library. It is a Bluetooth A2DP sink: audio goes to an I2S DAC and then into the QN8027. WiFi stays off.
- **OLED.** Frequency, Bluetooth peer (or "BT waiting"), and the 8-character RDS page now on the air.
- **Frequency.** The up/down buttons change the carrier. A press stores it in flash and the next boot restores it.
- **RDS paging.** With no track, the station name is `ESP32_FM`. A track becomes `Title - Artist`. That line is split into 8-character pages, then the station name is shown again, and the cycle repeats. The long radiotext field keeps the whole line. Accented letters from Spanish, Czech, and other Latin metadata are folded to plain ASCII (`é` and `ě` become `e`, `ñ` becomes `n`, and so on) before they are sent.
- **Air settings used on this board.** 12 MHz crystal, 75 kHz deviation, 50 µs pre-emphasis, pilot code 9, stereo, power 75, input gain 6 dB, digital gain 3 dB, 20 kΩ input. Program type is Pop.
`RDS_ENABLED` in `symbols.h` turns the subcarrier on. `RDS_REFRESH_INTERVAL` is how often `rdsRefresh()` runs. `RDS_PAGING_INTERVAL` is how long each 8-character page stays up.
## License
Apache License 2.0. See `LICENSE`.