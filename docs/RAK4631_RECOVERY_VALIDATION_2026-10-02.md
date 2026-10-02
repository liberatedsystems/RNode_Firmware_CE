# RAK4631 / nRF52: persistent InternalFS state causes post-flash USB reset loop; proven recovery validates EEPROM + firmware-integrity fixes

## Summary
On a physical RAK4631 (SX1262 / nRF52), flashing otherwise valid RNode firmware left the board in a rapid USB application reset loop (`239a:8029`) with repeated disconnects and Linux `error -71`. Reflashing normal firmware did not clear the fault because the nRF52 application region and persistent InternalFS/LittleFS region are separate.

A complete recovery required:
1. Explicitly formatting `InternalFS`.
2. Reinstalling a previously proven RNode build containing the NRF52 EEPROM and firmware-integrity fixes.
3. Provisioning EEPROM with deliberately paced ROM writes so each byte persisted reliably.
4. Supplying the exact application length to the NRF52 firmware-integrity path.
5. Setting the SHA-256 of the exact application binary.

After that sequence, USB remained stable, EEPROM checksum/signature validation succeeded, and Reticulum brought the RNode interface online at 915 MHz.

## Relation to existing upstream work
This real-hardware recovery independently reproduces the failure family described in:
- Issue #113: RAK4631 failure around NRF52 emulated EEPROM / InternalFS handling.
- PR #114: Fix NRF52 emulated EEPROM handling.
- PR #115: Fix firmware integrity validation on NRF52 boards.

The useful new evidence is end-to-end: a normal reflash alone was insufficient; persistent InternalFS state had to be reset before the patched firmware and provisioning/integrity path could fully recover the board.

## Hardware / environment
- Board: RAK4631 / WisBlock, SX1262, NRF52.
- Host: Linux Mint 22.3 XFCE.
- RNode firmware reported after recovery: `1.75`.
- Reticulum test parameters: 915 MHz, BW 125 kHz, SF7, CR5, TX 22 dBm.

## Failure symptoms
Before recovery, application mode repeatedly enumerated as `239a:8029`, then disconnected almost immediately. Linux logged repeated USB errors including `device descriptor read/all, error -71`, `can't set config #1, error -71`, and rapid `ttyACM` creation/destruction.

The bootloader remained stable as `239a:0029`, showing that the MCU/bootloader path was alive and that the failure was tied to application/persistent-state behavior rather than a dead board.

Normal firmware flashes completed successfully but did not clear the reset loop.

## Why a normal reflash did not fix it
The RAK4631 NRF52 application image occupies the application flash region, while the persistent InternalFS/LittleFS region is separate. A normal DFU application flash therefore does not necessarily erase the persistent state used by emulated EEPROM and firmware-validation metadata.

This explains why the bad state survived multiple successful firmware flashes and why explicitly formatting InternalFS was the decisive recovery step.

## Proven recovery sequence
### 1. Format InternalFS
A minimal NRF52 recovery sketch calling `InternalFS.begin(); InternalFS.format();` was flashed through the stable bootloader. After this step, application USB mode became stable instead of resetting.

### 2. Reinstall the known-good patched RNode build
Exact recovered application artifact:
- DFU package SHA-256: `78d0fa145b2f6d8227f0fd988cb99686e148bf3d23274d6afeaebd4b32c1e499`
- Application size: `223468` bytes
- Application SHA-256: `0fb4157e7f466952a8d57f84e6b46d22bd8c122c8a8c30ea19aea2857c3d497c`

The build contains the NRF52 EEPROM persistence work and the NRF52 firmware-length/hash validation work corresponding to the fixes investigated in PRs #114 and #115.

### 3. Pace EEPROM provisioning
The EEPROM was initially blank. Provisioning was performed through `CMD_ROM_WRITE = 0x52`, waiting 0.5 seconds after each EEPROM byte and writing the information-lock byte last.

Result:
- EEPROM checksum correct.
- Device signature validated.
- Product and hardware revision identified correctly.

### 4. Supply exact firmware length and hash
The recovery build used the NRF52 firmware-length command to store the exact application length (`223468`, hex `0x000368EC`), followed by setting the SHA-256 hash of the exact application binary.

Result: firmware hash validation succeeded while EEPROM checksum and signature remained valid.

## Validation after recovery
USB stability check: `40/40` consecutive samples remained in normal application mode (`239a:8029`) with the serial device continuously present.

Final device state:
- Firmware version: 1.75
- EEPROM checksum: correct
- Device signature: validated
- Device mode: Normal (host-controlled)
- Modem: SX1262
- Frequency range: 779–928 MHz
- Max TX power: 22 dBm

Reticulum validation:
- `RNodeInterface[RAK4631-Recovery-Test]`
- Status: Up
- Mode: Full
- Rate: 5.47 kbps
- MTU: 508
- Noise floor during test: -105 dBm
- RX traffic observed

No return of the previous USB `-71` reset storm was observed after the complete recovery.

## Suggested upstream action
1. Complete the NRF52 EEPROM persistence fix (#114).
2. Complete the NRF52 firmware-integrity validation fix (#115).
3. Document a recovery path for already-corrupted/stale InternalFS state, since application reflashing alone may not repair affected RAK4631 units.
4. Consider stronger persistence/flush semantics or pacing during provisioning so host-side ROM writes cannot outrun NRF52 emulated EEPROM persistence.

This report is intended as independent Linux/real-hardware validation of the existing NRF52 EEPROM and firmware-integrity work, plus a reproducible recovery sequence for devices already stuck with bad persistent state.
