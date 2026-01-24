# OBC SpaceFruit Project

This project implements a satellite On-Board Computer (OBC) system that acts as a bridge and coordinator between a Ground Station (PC) and a Virtual Reality (VR) Payload (Raspberry Pi).

## System Architecture

The system consists of three main components communicating via SLIP-encoded KISS frames with CRC32 integrity checks.

```mermaid
graph LR
    subgraph GS ["Ground Station (PC)"]
        direction TB
        UI["CLI & Display"]
        RX["pc_receiver.py"]
        UI --- RX
    end

    subgraph OBC ["OBC (STM32L496ZG)"]
        direction TB
        Main["main.c"]
        subgraph Software ["OBC_Software"]
            Packet["Common/obc_packet"]
            VR["Subsystems/VR"]
        end
        Main -.-> Packet
        Main --> VR
    end

    subgraph PL ["VR Payload (Raspberry Pi)"]
        RPi["rpi_vr.py"]
    end

    %% Connections
    RX <==>|"LPUART1 (9600 bps)<br/>SLIP/KISS Protocol"| Main
    Main <==>|"USB CDC (Virtual COM)<br/>SLIP/KISS Protocol"| RPi
```

## Communications Protocol

The communication stack is standardized across both links (GS-OBC and OBC-Payload):

1.  **Physical Layer**: LPUART (GS) / USB CDC (Payload)
2.  **Framing**: SLIP (RFC 1055)
3.  **Transport**: KISS (FEND/Command Byte)
4.  **Application**: Custom Binary Packets with CRC32

### Packet Structure
`[FEND] [CMD_TYPE] [PAYLOAD_ID] [DATA...] [CRC32] [FEND]`

## Project Structure

*   `GroundStation/`: Python tools for the PC operator.
*   `NUCLEO_L496ZG_OBC_VR/`: C firmware for the STM32 microcontroller.
    *   `OBC_Software/`: Modular application logic.
*   `VR/`: Python simulator/driver for the Payload.
