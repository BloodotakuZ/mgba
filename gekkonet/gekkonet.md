# GekkoNet — High‑Level Overview & Integration Guide

> **Scope:** This README summarizes the public API and the runtime/architecture of **GekkoNet**, a compact peer‑to‑peer rollback networking SDK for games, based on the project’s published Doxygen documentation at `https://heatxd.github.io/GekkoNet/`.

---

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Getting Started](#getting-started)
- [Quick Start (C API)](#quick-start-c-api)
- [Public C API (High Level)](#public-c-api-high-level)
  - [Opaque types & configuration](#opaque-types--configuration)
  - [Data types](#data-types)
  - [Events](#events)
  - [Functions](#functions)
  - [Default UDP adapter](#default-udp-adapter)
  - [Memory & ownership](#memory--ownership)
- [Architecture (C++ internals)](#architecture-c-internals)
  - [Networking & wire formats](#networking--wire-formats)
  - [MessageSystem](#messagesystem)
  - [SyncSystem](#syncsystem)
  - [Players & status](#players--status)
  - [Event buffering](#event-buffering)
  - [State storage](#state-storage)
  - [Compression helper](#compression-helper)
- [Data & Control Flow](#data--control-flow)
- [Example Game Loop](#example-game-loop)
- [Files & Responsibilities](#files--responsibilities)
- [Build & Platform Notes](#build--platform-notes)
- [Glossary](#glossary)
- [License](#license)

---

## Overview

**GekkoNet** is a small, embeddable P2P rollback netcode layer for C/C++ games. It distributes per‑frame inputs among peers, predicts/rolls back when inputs arrive late, and surfaces a compact **C API** while implementing the heavy lifting in C++ (`namespace Gekko`).

**Core ideas**

- **Distributed inputs:** each frame’s inputs for all players are exchanged among peers.
- **Rollback:** your game provides *Save/Load state* callbacks; GekkoNet requests saves/loads and tells you when to **Advance** a frame, optionally in rollback mode.
- **Sync & health:** periodic session checksums and network pings (RTT/jitter) keep peers aligned; mismatches can report desyncs.
- **Minimal host API:** create a session, plug a transport, push local input, poll events, and drive your sim.

---

## Features

- Peer‑to‑peer input exchange with prediction windows
- Rollback with game‑managed state save/load
- Session/network health messages (checksums, RTT, jitter)
- Pluggable transport (`GekkoNetAdapter`) with a **default UDP/ASIO adapter**
- Small C API that’s engine‑agnostic

---

## Getting Started

1. Include the public header(s) (notably `gekkonet.h`) and link the library.
2. Choose a transport:
   - Use the **built‑in UDP adapter** (`gekko_default_adapter(port)`) *or*
   - Provide your own `GekkoNetAdapter` (send/receive/free).
3. Fill `GekkoConfig` (players, input/state sizes, prediction window).
4. Start the session and run the loop: push local inputs, poll network, consume events.

> **Input/state sizes:** Your input struct must be a fixed POD byte‑array of size `config.input_size`. State buffers for save/load should fit within `config.state_size`.

---

## Quick Start (C API)

```c
#include "gekkonet.h"

typedef struct {
    unsigned short buttons;
    signed char    lx, ly;   // example input bytes
} GameInput;

int main(void) {
    GekkoSession* s = NULL;
    GekkoConfig cfg = {0};

    cfg.num_players = 2;
    cfg.input_size  = sizeof(GameInput);
    cfg.state_size  = 1 * 1024 * 1024;  // up to 1 MiB per save
    cfg.input_prediction_window = 2;
    cfg.desync_detection = true;

    // Create and wire a transport (use default UDP adapter on port 50000)
    gekko_create(&s);
    GekkoNetAdapter* net = gekko_default_adapter(50000);
    gekko_net_adapter_set(s, net);

    // Add actors (example: 1 local, 1 remote)
    int local  = gekko_add_actor(s, LocalPlayer,  NULL);
    GekkoNetAddress remoteAddr = {/* fill with your endpoint */};
    int remote = gekko_add_actor(s, RemotePlayer, &remoteAddr);

    gekko_start(s, &cfg);

    while (/* running */) {
        GameInput in = readControllers();
        gekko_add_local_input(s, local, &in);

        // Pump network → deliver inbound packets to the session
        gekko_network_poll(s);

        // Drive the session → handle game events
        int n = 0;
        GekkoGameEvent** evs = gekko_update_session(s, &n);
        for (int i = 0; i < n; ++i) {
            switch (evs[i]->type) {
            case AdvanceEvent: {
                GekkoAdvanceEvent e = evs[i]->data.adv;
                if (e.rolling_back) {
                    // Load the snapshot for e.frame (provided earlier on SaveEvent)
                }
                // Advance your simulation to e.frame using e.inputs (e.input_len bytes)
            } break;
            case SaveEvent: {
                GekkoSaveEvent e = evs[i]->data.save;
                // Serialize current game state to *e.state (<= cfg.state_size)
                // Set *e.state_len and *e.checksum
            } break;
            case LoadEvent: {
                GekkoLoadEvent e = evs[i]->data.load;
                // Restore state from e.state (length e.state_len) for e.frame
            } break;
            default: break;
            }
        }

        // Optional: read session events (connect/disconnect/sync/desync)
        int m = 0;
        GekkoSessionEvent** se = gekko_session_events(s, &m);
        /* ... */

        // Optional: stats
        GekkoNetworkStats st = {0};
        gekko_network_stats(s, remote, &st);
    }

    gekko_destroy(s);
    return 0;
}
```

---

## Public C API (High Level)

### Opaque types & configuration

- **`GekkoSession`** — opaque session handle
- **`GekkoConfig`**  
  `num_players`, `max_spectators`, `input_prediction_window`, `spectator_delay`,  
  `input_size`, `state_size`, `limited_saving`, `post_sync_joining`, `desync_detection`

### Data types

- **`GekkoPlayerType`** — `LocalPlayer`, `RemotePlayer`, `Spectator`
- **`GekkoNetAddress`** — opaque buffer for transport address
- **`GekkoNetResult`** — adapter receive result (`addr`, `data`, `data_len`)
- **`GekkoNetAdapter`** — callbacks:  
  `send_data(addr, data, len)`, `receive_data(&out_count) -> GekkoNetResult**`, `free_data(ptr)`
- **`GekkoNetworkStats`** — `last_ping` (ms), `avg_ping`, `jitter`

### Events

- **Game events (`GekkoGameEventType`)**
  - `AdvanceEvent` → `{ frame, input_len, inputs*, rolling_back }`
  - `SaveEvent`    → `{ frame, checksum*, state_len*, state* }`
  - `LoadEvent`    → `{ frame, state_len, state* }`

- **Session events (`GekkoSessionEventType`)**
  - `PlayerSyncing { handle, current, max }`
  - `PlayerConnected { handle }`
  - `PlayerDisconnected { handle }`
  - `SessionStarted`
  - `SpectatorPaused` / `SpectatorUnpaused`
  - `DesyncDetected { frame, local_checksum, remote_checksum, remote_handle }`

### Functions

- **Lifecycle**  
  `bool gekko_create(GekkoSession** s)`  
  `bool gekko_destroy(GekkoSession* s)`  
  `void gekko_start(GekkoSession* s, GekkoConfig* cfg)`

- **Transport**  
  `void gekko_net_adapter_set(GekkoSession* s, GekkoNetAdapter* a)`  
  `GekkoNetAdapter* gekko_default_adapter(unsigned short port)`

- **Players & input**  
  `int  gekko_add_actor(GekkoSession* s, GekkoPlayerType type, GekkoNetAddress* addr)`  
  `void gekko_set_local_delay(GekkoSession* s, int player, unsigned char frames)`  
  `void gekko_add_local_input(GekkoSession* s, int player, void* input_bytes)`

- **Pump / query**  
  `GekkoGameEvent**    gekko_update_session(GekkoSession* s, int* count)`  
  `GekkoSessionEvent** gekko_session_events(GekkoSession* s, int* count)`  
  `float               gekko_frames_ahead(GekkoSession* s)`  
  `void                gekko_network_stats(GekkoSession* s, int player, GekkoNetworkStats* out)`  
  `void                gekko_network_poll(GekkoSession* s)`

### Default UDP adapter

- Built‑in UDP adapter backed by header‑only **ASIO**.  
- Available via `gekko_default_adapter(port)`; can be disabled at build time with `GEKKONET_NO_ASIO`.

### Memory & ownership

- Event payloads are valid for the tick they’re retrieved; consume immediately.
- If you implement a custom `GekkoNetAdapter`, **you** own memory you allocate
  for receive buffers; expose `free_data` so GekkoNet can release them.

---

## Architecture (C++ internals)

The public C API is backed by C++ systems in `namespace Gekko`.

### Networking & wire formats

Header: `net.h`

- **`PacketType`**: `Inputs`, `SpectatorInputs`, `InputAck`, `SyncRequest`, `SyncResponse`, `SessionHealth`, `NetworkHealth`
- **Messages** (serialized via `zpp::serializer`):
  - `InputMsg { Frame start_frame; u8 input_count; u16 total_size; bytes inputs... }`
  - `InputAckMsg { Frame ack_frame; i8 frame_advantage; }`
  - `SyncMsg { u16 rng_data; }`
  - `SessionHealthMsg { Frame frame; u32 checksum; }`
  - `NetworkHealthMsg { u64 send_time; bool received; }`
- **Wrappers/structs**: `MsgHeader`, `NetPacket`, `NetData`, `NetStats` (timers, RTT history)

### MessageSystem

Header/Src: `backend.h/.cpp`

- Sends batched inputs (players & spectators) and resends as needed
- Parses inbound packets and handles: Sync, Inputs, InputAck, Session/Network health
- Maintains players and per‑peer `NetStats` (RTT/jitter/timeout windows)
- Emits session‑level events (connect/disconnect/sync/desync)

### SyncSystem

Header/Src: `sync.h/.cpp`

- Aggregates local/remote inputs per frame and enforces prediction windows
- Exposes `GetLocalInputs`, `GetSpectatorInputs`, `SetLocalDelay`
- Reports `GetMinIncorrectFrame` / `GetMinReceivedFrame` to drive rollback & late joins

### Players & status

- `PlayerStatus`: `Initiating`, `Connected`, `Disconnected`
- `Player` holds `handle`, `GekkoPlayerType`, `NetAddress`, `NetStats`, and per‑frame checksums

### Event buffering

Header/Src: `event.h/.cpp`

- `GameEventBuffer` (separate rings for `Advance` vs others) with input payload storage
- `SessionEventBuffer` / `SessionEventSystem` keep short histories for the host to drain

### State storage

Header/Src: `storage.h/.cpp`

- `StateStorage` and `StateEntry` manage buffers for save/load snapshots used in rollback

### Compression helper

Header: `compression.h`

- Small interface used by the backend to (optionally) compress payloads

---

## Data & Control Flow

1. **Setup** — create session, configure, set or request a transport, add actors.
2. **Handshake** — peers exchange sync data (magic, RNG seed).
3. **Loop** — each tick:
   - Push **local input** for the current frame
   - **Poll network** to ingest inbound packets
   - **Update session** to receive:
     - `AdvanceEvent` (may be `rolling_back`)
     - `SaveEvent` / `LoadEvent` for state snapshots
   - Optionally read **session events** and **network stats**

4. **Health** — periodic `SessionHealth` (frame+checksum) and `NetworkHealth` (RTT/jitter). Mismatches → `DesyncDetected`.

---

## Example Game Loop

See [Quick Start](#quick-start-c-api) for a complete snippet. The essence is:

```c
gekko_add_local_input(s, local, &input);
gekko_network_poll(s);
int n = 0;
GekkoGameEvent** evs = gekko_update_session(s, &n);
for (int i = 0; i < n; ++i) { /* handle Advance/Save/Load */ }
```

---

## Files & Responsibilities

**Headers**
- `gekkonet.h` — public C API
- `gekko_types.h` — numeric typedefs and frame/handle aliases
- `gekko.h` — C++ Session glue backing the C API
- `backend.h` — `MessageSystem` (send/receive, stats, events)
- `event.h` — game/session event buffers
- `input.h` — input containers & helpers
- `net.h` — wire types, packets, stats
- `storage.h` — save/load snapshot storage
- `sync.h` — input aggregation, prediction windows, frame clock

**Sources**
- `gekkonet.cpp`, `gekko.cpp` — C API & session implementation
- `backend.cpp` — message handling implementation
- `event.cpp`, `input.cpp`, `net.cpp`, `player.cpp`
- `storage.cpp`, `sync.cpp`

---

## Build & Platform Notes

- `GEKKONET_API` handles symbol visibility (DLL export on Windows unless `GEKKONET_STATIC` is set).
- Define `GEKKONET_NO_ASIO` to **disable** the built‑in ASIO UDP adapter.
- The default adapter uses header‑only **ASIO** (`ASIO_STANDALONE`).
- Message serialization uses `zpp::serializer` (ensure it’s available to your build).

---

## Glossary

- **Numeric typedefs**: `u8`, `u16`, `u32`, `u64`, `i8`, `i16`, `i32`, `i64`
- **Aliases**: `Frame` (frame index), `Handle` (player handle/ID)
- **Enums**:
  - `GekkoPlayerType`: `LocalPlayer`, `RemotePlayer`, `Spectator`
  - `GekkoGameEventType`: `AdvanceEvent`, `SaveEvent`, `LoadEvent`
  - `GekkoSessionEventType`: `PlayerSyncing`, `PlayerConnected`, `PlayerDisconnected`, `SessionStarted`, `SpectatorPaused`, `SpectatorUnpaused`, `DesyncDetected`
  - `Gekko::PacketType`: `Inputs`, `SpectatorInputs`, `InputAck`, `SyncRequest`, `SyncResponse`, `SessionHealth`, `NetworkHealth`
  - `Gekko::PlayerStatus`: `Initiating`, `Connected`, `Disconnected`

---

## License

See the project’s `LICENSE` file for exact terms (commonly BSD‑2‑Clause in public docs).

---

> **Notes & Tips**
>
> - Pack exactly `config.input_size` bytes per player per frame when calling `gekko_add_local_input`.
> - Keep state snapshots ≤ `config.state_size`; provide a checksum for desync detection.
> - Spectators can be given extra delay to reduce rollbacks.
> - For non‑UDP transports, implement `GekkoNetAdapter` and map your IO to the three callbacks.
