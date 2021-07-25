# albireo-pi

## Wiring

### MCP3002
| MCP3002 | Raspberry Pi     | 5000K  | 3000K  |
| ------- | ---------------- | ------ | ------ |
| CS      | SS               | GPIO23 | GPIO24 |
| CH0     | To Analog device |        |        |
| CH1     | To Analog device |        |        |
| VSS     | 0V               |        |        |
| Din     | MOSI             | GPIO17 | GPIO17 |
| Dout    | MISO             | GPIO27 | GPIO27 |
| CLK     | SCK              | GPIO22 | GPIO22 |
| VDD     | 3.3V             |        |        |

### BME280
| BME280 | Raspberry Pi | 5000K  | 3000K  |
| ------ | ------------ | ------ | ------ |
| VDD    | 3.3V         |        |        |
| GND    | 0V           |        |        |
| SCK    | SCLK         | GPIO11 | GPIO11 |
| SDI    | MOSI         | GPIO10 | GPIO10 |
| CSB    | CE0          | GPIO8  | GPIO7  |
| SDO    | MISO         | GPIO9  | GPIO9  |

### TSL2561
| TSL2561 | Raspberry Pi | 5000K | 3000K |
| ------- | ------------ | ----- | ----- |
| VCC     | 3.3V         |       |       |
| GND     | 0V           |       |       |
| SCL     | SCL          | GPIO3 | GPIO3 |
| SDA     | SDA          | GPIO2 | GPIO2 |
| INT     | None         |       |       |
| ADDRESS |              | 0x39  | 0x29  |
