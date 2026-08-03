# Olimex ESP32-POE2 Vendor Documentation

This directory contains local copies of official Olimex ESP32-POE2
documentation used while adding ESP32-POE2 support.

## Source

| Item | Value |
| --- | --- |
| Vendor | Olimex |
| Product page | https://www.olimex.com/Products/IoT/ESP32/ESP32-POE2/open-source-hardware |
| Upstream repository | https://github.com/OLIMEX/ESP32-POE2 |
| Source revision | `2298e609769a2bdc516b0e3894ee2901db9fcbaa` |
| Download date | 2026-08-03 |

The upstream repository README states:

| Material | License |
| --- | --- |
| Hardware | CERN Open Hardware Licence Version 2 - Strongly Reciprocal |
| Software | GPL-3.0 |
| Documentation | CC BY-SA 4.0 |
| Box design | CC BY 4.0 |

## Files

| Local file | Upstream URL | SHA-256 |
| --- | --- | --- |
| `DOCUMENTS/ESP32-POE2-user-manual.pdf` | https://github.com/OLIMEX/ESP32-POE2/blob/main/DOCUMENTS/ESP32-POE2-user-manual.pdf | `203e5cddef00174d89ef85bfeb292aa8d4b0630f3f257729cede09367d0506d3` |
| `DOCUMENTS/ESP32-PoE2-dimensions.pdf` | https://github.com/OLIMEX/ESP32-POE2/blob/main/DOCUMENTS/ESP32-PoE2-dimensions.pdf | `98a52f2e52a7eca03c94c09caaaf0d80dec717003273239deae702e670249c27` |
| `DOCUMENTS/UEXT-PINOUT.ods` | https://github.com/OLIMEX/ESP32-POE2/blob/main/DOCUMENTS/UEXT-PINOUT.ods | `a469d4c945466e53440ca1fce29a46abb5e68fb8193e2a91d79206cd64d6b107` |
| `HARDWARE/Hardware-revision-changes.txt` | https://github.com/OLIMEX/ESP32-POE2/blob/main/HARDWARE/Hardware-revision-changes.txt | `bc7bd3f478d7cbbea2587c7a559740e5c9f5c07d400e74895f8fc52e7e698d17` |
| `HARDWARE/ESP32-PoE2_Rev_B/ESP32-PoE2_Rev_B.pdf` | https://github.com/OLIMEX/ESP32-POE2/blob/main/HARDWARE/ESP32-PoE2_Rev_B/ESP32-PoE2_Rev_B.pdf | `591febdfc1d2ba6d55dbcf3ee2444884f02acd4f67672a0fc2e0e4a7332e11b8` |

## Update Procedure

1. Select the upstream Olimex commit to vendor from.
2. Download files from `raw.githubusercontent.com` using that commit hash.
3. Recalculate SHA-256 checksums with `sha256sum`.
4. Update this README with the source revision, download date, file list, and
   checksums.
5. Re-check the upstream README for license changes before committing.
