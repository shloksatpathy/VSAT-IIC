# Setting the Net ID and PAN ID on a LoRa Module

This guide explains how to give the CanSat and the ground station their own network so that they only hear each other. At CanSat India, many teams fly LoRa radios on the same band at the same time. If two teams keep the factory defaults, each ground station can receive and decode the other team's packets.

> **The LoRa module hasn't been chosen yet** (see [Hardware Components](../README.md#hardware-components)). This guide covers the three options we are most likely to use. Once the module is chosen, check the steps against its datasheet and delete the sections that don't apply.

---

## Contents

- [What the IDs Do](#what-the-ids-do)
- [Our Values](#our-values)
- [Ebyte E22 (UART, register-based)](#ebyte-e22-uart-register-based)
- [REYAX RYLR896 / RYLR998 (UART, AT commands)](#reyax-rylr896--rylr998-uart-at-commands)
- [Bare SX127x / SX126x Module (SPI)](#bare-sx127x--sx126x-module-spi)
- [Checking the Link](#checking-the-link)
- [Troubleshooting](#troubleshooting)

---

## What the IDs Do

| Term | What it is | Who shares it |
| --- | --- | --- |
| **Net ID** (network ID) | Says which network a packet belongs to. The module drops packets whose Net ID doesn't match its own. | Every node in our system (CanSat and ground station) |
| **PAN ID** (personal area network ID) | The Zigbee / IEEE 802.15.4 name for the same idea: one number that identifies a whole network. | Every node in our system |
| **Node address** | Identifies one radio inside the network. | Each node has its own |
| **Channel / frequency** | The carrier frequency. Radios on different channels can't hear each other at all. | Every node in our system |

Plain LoRa has no field called "PAN ID". Each module type puts the network identity somewhere different:

| Module | Field used as the Net ID | Field used as the PAN ID | Node address |
| --- | --- | --- | --- |
| Ebyte E22 | `NETID` register (`0x02`) | `NETID` together with the channel | `ADDH`:`ADDL` (`0x00`–`0x01`) |
| REYAX RYLR | `AT+NETWORKID` | `AT+NETWORKID` (plus `AT+CPIN` on the RYLR998) | `AT+ADDRESS` |
| Bare SX127x / SX126x | LoRa sync word | A 16-bit ID we put in our own packet header | A byte in our own packet header |

If the module we pick has a setting that is literally called "PAN ID", treat it the same way as the Net ID: use one value that every node shares.

**Rule:** the CanSat and the ground station must use the **same** Net ID / PAN ID and the **same** channel, and **different** node addresses.

---

## Our Values

Don't use the factory defaults (`0`, or a sync word of `0x12`), because other teams will be on them. We use values based on our team number, 064:

| Setting | Value | Notes |
| --- | --- | --- |
| Net ID | `0x40` (64) | Use the closest allowed value if the module's range is smaller (see each section) |
| PAN ID (when we set it ourselves) | `0x0640` | For the software header on bare SX127x / SX126x modules |
| CanSat address | `0x0641` | |
| Ground station address | `0x0642` | |
| Frequency | 866.125 MHz | Inside India's licence-free 865–867 MHz band |

If we hear another team on these values during testing, change the Net ID and record the new value here.

---

## Ebyte E22 (UART, register-based)

This covers the E22-900T22S and E22-900T22D, and other E22 modules work the same way. The settings are stored in registers that you write over UART while the module is in configuration mode.

### 1. Wiring

| E22 pin | ESP32-S3 | Notes |
| --- | --- | --- |
| `M0` | GPIO (TBD) | Mode select |
| `M1` | GPIO (TBD) | Mode select |
| `TXD` | UART RX (TBD) | |
| `RXD` | UART TX (TBD) | |
| `AUX` | GPIO (TBD) | Goes high when the module is ready |
| `VCC` | 3.3–5 V | Check the module's power-supply requirement |
| `GND` | GND | |

### 2. Enter configuration mode

| Mode | `M1` | `M0` |
| --- | --- | --- |
| Normal (transmit and receive) | 0 | 0 |
| Wake-on-radio | 0 | 1 |
| **Configuration** | **1** | **0** |
| Deep sleep | 1 | 1 |

Set `M1 = HIGH` and `M0 = LOW`, then wait for `AUX` to go high. In configuration mode the UART always runs at **9600 baud, 8N1**, whatever baud rate the module is set to.

### 3. Registers

| Address | Name | Meaning |
| --- | --- | --- |
| `0x00` | `ADDH` | Node address, high byte |
| `0x01` | `ADDL` | Node address, low byte |
| `0x02` | `NETID` | **Net ID** (0–255) |
| `0x03` | `REG0` | UART baud rate, parity, air data rate |
| `0x04` | `REG1` | Packet size, transmit power |
| `0x05` | `REG2` | Channel (frequency = 850.125 MHz + channel, for 900 MHz modules) |
| `0x06` | `REG3` | RSSI, fixed/transparent mode, wake-on-radio |
| `0x07`–`0x08` | `CRYPT_H` / `CRYPT_L` | Encryption key (write only) |

### 4. Commands

| Command | Bytes | Effect |
| --- | --- | --- |
| Write and save | `C0 <start> <length> <data...>` | Writes the registers and keeps them after power-off |
| Write, temporary | `C2 <start> <length> <data...>` | Writes the registers until the next power-off |
| Read | `C1 <start> <length>` | Reads the registers back |

The module replies with `C1 <start> <length> <data...>`.

**Set the CanSat address `0x0641` and Net ID `0x40`:**

```
C0 00 03 06 41 40
```

**Set channel 16 (866.125 MHz):**

```
C0 05 01 10
```

**Read them back:**

```
C1 00 03        → C1 00 03 06 41 40
C1 05 01        → C1 05 01 10
```

For the ground station, write `C0 00 03 06 42 40` instead. The address changes, and the Net ID and channel stay the same.

### 5. Arduino example (ESP32-S3)

```cpp
#include <Arduino.h>

// Pins are placeholders until the PCB pin map is fixed.
constexpr int PIN_M0  = 4;
constexpr int PIN_M1  = 5;
constexpr int PIN_AUX = 6;
constexpr int PIN_RX  = 17;   // ESP32 RX  <- E22 TXD
constexpr int PIN_TX  = 18;   // ESP32 TX  -> E22 RXD

constexpr uint8_t ADDH  = 0x06;
constexpr uint8_t ADDL  = 0x41;   // 0x42 on the ground station
constexpr uint8_t NETID = 0x40;
constexpr uint8_t CHAN  = 0x10;   // 866.125 MHz

HardwareSerial LoRaSerial(1);

static void waitAux() {
  uint32_t start = millis();
  while (digitalRead(PIN_AUX) == LOW && millis() - start < 1000) {}
  delay(10);
}

static bool writeRegs(uint8_t start, const uint8_t *data, uint8_t len) {
  LoRaSerial.write(0xC0);
  LoRaSerial.write(start);
  LoRaSerial.write(len);
  LoRaSerial.write(data, len);

  uint8_t reply[3 + 8];
  size_t n = LoRaSerial.readBytes(reply, 3 + len);
  if (n != 3u + len || reply[0] != 0xC1 || reply[1] != start || reply[2] != len) return false;
  return memcmp(reply + 3, data, len) == 0;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_M0, OUTPUT);
  pinMode(PIN_M1, OUTPUT);
  pinMode(PIN_AUX, INPUT);

  // Configuration mode: M1 = 1, M0 = 0, 9600 baud.
  digitalWrite(PIN_M0, LOW);
  digitalWrite(PIN_M1, HIGH);
  waitAux();
  LoRaSerial.begin(9600, SERIAL_8N1, PIN_RX, PIN_TX);
  LoRaSerial.setTimeout(500);

  const uint8_t ids[] = {ADDH, ADDL, NETID};
  const uint8_t chan[] = {CHAN};
  bool ok = writeRegs(0x00, ids, sizeof ids) && writeRegs(0x05, chan, sizeof chan);
  Serial.println(ok ? "E22 configured" : "E22 config FAILED");

  // Back to normal mode.
  waitAux();
  digitalWrite(PIN_M1, LOW);
  waitAux();
}

void loop() {}
```

### 6. Transparent mode vs. fixed mode

- **Transparent mode** (default, `REG3` bit 6 = 0): every byte written to the UART is sent. The receiver only accepts it if the address, channel and Net ID all match its own.
- **Fixed mode** (`REG3` bit 6 = 1): every packet starts with the destination `ADDH ADDL CHAN`, followed by the payload. Address `0xFFFF` broadcasts to every node with the same Net ID and channel.

You can also use Ebyte's **RF Setting** tool on a PC with a USB–UART adapter to set the same registers. Set `M1 = 1`, `M0 = 0` before you connect.

> The older **E32** series has no `NETID` register, only `ADDH`/`ADDL` and a channel. If we end up with an E32, use the channel plus a software ID in the packet header (see the [bare-module section](#bare-sx127x--sx126x-module-spi)).

---

## REYAX RYLR896 / RYLR998 (UART, AT commands)

These modules are set up with AT commands over UART. The default baud rate is 115200, 8N1, and every command ends with `\r\n`. Settings are saved automatically and kept after power-off.

| Command | Range | Default | Purpose |
| --- | --- | --- | --- |
| `AT+NETWORKID=<id>` | RYLR896: 0–16 · RYLR998: 3–15 or 18 | RYLR896: 0 · RYLR998: 18 | **Net ID** |
| `AT+ADDRESS=<addr>` | 0–65535 | 0 | Node address |
| `AT+BAND=<Hz>` | Module dependent | 915000000 | Frequency |
| `AT+CPIN=<8 hex chars>` | RYLR998 only | none | Network password; only nodes with the same password can talk |

Our Net ID `0x40` is outside both ranges, so use **`12`** on these modules and note it in [Our Values](#our-values).

**CanSat:**

```
AT                      → +OK
AT+NETWORKID=12         → +OK
AT+ADDRESS=1601         → +OK          (0x0641)
AT+BAND=866125000       → +OK
AT+CPIN=CA7064A1        → +OK          (RYLR998 only)
```

**Ground station:** the same commands, but with `AT+ADDRESS=1602` (`0x0642`).

**Read back:**

```
AT+NETWORKID?           → +NETWORKID=12
AT+ADDRESS?             → +ADDRESS=1601
AT+BAND?                → +BAND=866125000
```

**Send to the ground station:**

```
AT+SEND=1602,5,HELLO
```

The ground station prints `+RCV=1601,5,HELLO,<RSSI>,<SNR>`. Sending to address `0` broadcasts to every node with the same Net ID.

---

## Bare SX127x / SX126x Module (SPI)

Bare radio modules (Ai-Thinker Ra-01/Ra-02, HopeRF RFM95, SX1262 boards, and so on) have no Net ID, PAN ID or address registers. We set the network identity in two layers.

**1. Sync word (Net ID, set in hardware).** The radio drops packets whose sync word doesn't match its own. Don't use `0x12` (the default) or `0x34` (reserved for LoRaWAN).

```cpp
// Sandeep Mistry "LoRa" library (SX127x)
LoRa.begin(866.125E6);
LoRa.setSyncWord(0x40);

// RadioLib (SX126x)
radio.begin(866.125, 125.0, 9, 7, 0x40);   // freq, bandwidth, SF, CR, sync word
```

A sync word only gives about 256 options, and nearby values sometimes get through. So we also add:

**2. PAN ID and address in the packet header (set in software).** Every packet starts with this header, and the receiver drops any packet where the fields don't match:

```
| PAN ID (2 B) | Destination (2 B) | Source (2 B) | Payload ... |
|   0x0640     |   0x0642          |   0x0641     |             |
```

```cpp
struct __attribute__((packed)) LinkHeader {
  uint16_t pan;   // 0x0640
  uint16_t dst;   // 0x0642 = ground station, 0xFFFF = broadcast
  uint16_t src;   // 0x0641 = CanSat
};

bool acceptPacket(const LinkHeader &h, uint16_t myAddr) {
  return h.pan == 0x0640 && (h.dst == myAddr || h.dst == 0xFFFF);
}
```

The telemetry packet format in `communication/` should start with this header.

---

## Checking the Link

Do these checks after every reconfiguration and again before flight:

1. **Read back** every setting (`C1 ...` or `AT+...?`) on both radios and compare them with [Our Values](#our-values).
2. **Matching IDs:** send a test packet from the CanSat and check that the ground station receives it.
3. **Mismatched Net ID:** change the Net ID on one radio and check that **nothing** is received. This shows the filter works. Then change it back.
4. **Wrong address** (E22 fixed mode, RYLR, or the software header): send to a different address and check that the ground station ignores it.
5. **Power cycle** both radios and read the settings again, to make sure they were saved. (On the E22, `C2` writes are lost at power-off.)

---

## Troubleshooting

| Symptom | Likely cause |
| --- | --- |
| E22 doesn't reply to `C1`/`C0` | Not in configuration mode (`M1 = 1`, `M0 = 0`), UART not at 9600 baud, or TX/RX swapped |
| E22 settings lost after power-off | Written with `C2` instead of `C0` |
| RYLR replies `+ERR=...` | Value out of range (for example a Net ID above 16), or the line doesn't end with `\r\n` |
| No packets received with matching IDs | Different channel/frequency, spreading factor, bandwidth or air data rate on the two sides |
| Packets from other teams still arriving | Still on a default Net ID or sync word; pick a less common value |
| Works on the bench, fails after a reset | The firmware writes temporary settings at boot; switch to the permanent command |
