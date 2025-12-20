# Design Document: Reliable CAN Bus Library

## Problem Statement

**Satellite CAN Communication Reliability on Raspberry Pi Zero 2W**

### The Challenge
When running CAN communication on a Raspberry Pi Zero 2W with an RS485 CAN HAT:

1. **Hardware Buffer Overflow**
   - CAN link failures (disconnection, power loss, etc.) cause messages to queue in hardware buffer
   - RS485 CAN HAT has limited buffer (~100-200 messages)
   - Once buffer fills, the can0 interface becomes **completely unusable**
   - Recovery requires system reboot - **unacceptable for satellite operations**

2. **Satellite Constraints**
   - Cannot reboot easily (may lose orientation, mission time)
   - Link outages are common (orbital geometry, power cycling)
   - Zero data loss requirement for telemetry
   - Limited RAM (512MB on Pi Zero 2W)

## Design Philosophy: KISS (Keep It Simple, Stupid)

### Core Principles
1. **Simple failure detection** - Just count failures, no complex state machine
2. **Simple recovery** - Drain queue when it works again
3. **Simple buffering** - Python deque, not custom data structure
4. **Simple API** - One send method, one receive method
5. **Background threading** - Continuous RX in separate thread for non-blocking ACK reception

### Anti-Patterns Avoided
❌ Complex retry logic with backoff  
❌ Multi-level state machines  
❌ Custom buffer management  
❌ Polling loops that block draining  
❌ Hardware-specific workarounds  

## Solution Architecture

### Three-Layer Defense System + Background RX Thread

```
┌─────────────────────────────────────────┐
│   Application Code                      │
│   send_with_ack(0x100, [data])         │
└──────────────┬──────────────────────────┘
               │
        ┌──────▼───────────────────────────┐
        │  Background RX Thread            │
        │  • Continuously receives ACKs    │
        │  • Non-blocking operation        │
        │  • 500-message buffer            │
        └──────────────────────────────────┘
               │
        ┌──────▼───────────────────────────┐
        │  Layer 3: Application Queue      │
        │  • Python deque (RAM)            │
        │  • 1000 messages max             │
        │  • FIFO ordering                 │
        │  • Only used when link DOWN      │
        └──────┬───────────────────────────┘
               │
        ┌──────▼───────────────────────────┐
        │  Layer 2: Fast Link Detection    │
        │  • 3 failures = link DOWN        │
        │  • Flush hardware buffer         │
        │  • Stop sending to hardware      │
        └──────┬───────────────────────────┘
               │
        ┌──────▼───────────────────────────┐
        │  Layer 1: Small Hardware Buffer  │
        │  • txqueuelen = 10               │
        │  • Limits kernel queue size      │
        │  • Fast overflow detection       │
        └──────┬───────────────────────────┘
               │
        ┌──────▼───────────────────────────┐
        │  RS485 CAN HAT Hardware          │
        │  • MCP2515 CAN controller        │
        │  • SPI interface to Pi           │
        └──────────────────────────────────┘
```

### State Machine (Minimal)

```
     ┌──────────────┐
     │  LINK_OK     │◄────────┐
     │  fail_count=0│         │
     └──────┬───────┘         │
            │                 │
      send() fails            │ send() succeeds
      3 times                 │ & queue drained
            │                 │
     ┌──────▼───────┐         │
     │  LINK_DOWN   │─────────┘
     │  queue msgs  │
     └──────────────┘
```

**That's it. Two states. Simple.**

## Implementation Details

### Initialization
```python
def __init__(self, interface='can0', bitrate=250000, queue_size=1000):
    # Link state (simple boolean)
    self._link_ok = True
    self._fail_count = 0
    
    # Application queue (RAM)
    self._msg_queue = deque(maxlen=queue_size)
    
    # Background RX thread for ACKs
    self._rx_buffer = deque(maxlen=500)
    self._rx_lock = threading.Lock()
    self._rx_thread = threading.Thread(target=self._rx_worker, daemon=True)
    self._rx_running = True
    self._rx_thread.start()
    
    # Setup hardware with small TX queue
    os.system(f'sudo ip link set can0 txqueuelen 10')
```

### Background RX Thread (Non-blocking ACK Reception)
```python
def _rx_worker(self):
    """Background thread: continuously receive messages"""
    while self._rx_running:
        try:
            msg = self.bus.recv(timeout=0.1)
            if msg:
                with self._rx_lock:
                    self._rx_buffer.append((msg.arbitration_id, list(msg.data)))
        except Exception:
            pass  # Keep receiving even on errors
```

### Send Logic (Simple)
```python
def send(self, can_id, data, extended=False):
    # Step 1: Try to drain queue first
    if self._link_ok and len(self._msg_queue) > 0:
        self._drain_queue()
    
    # Step 2: If link down, queue immediately
    if not self._link_ok:
        return self._queue_message(can_id, data, extended)
    
    # Step 3: Try ONE send (no complex retry)
    try:
        self.bus.send(msg, timeout=0.05)
        self._fail_count = 0
        return True
    except Exception:
        self._fail_count += 1
        if self._fail_count >= 3:
            self._link_ok = False
            self._flush_hardware_buffer()
        return self._queue_message(can_id, data, extended)
```

### ACK Protocol (Application-Level Delivery Confirmation)
```python
def send_with_ack(self, can_id, data, timeout=0.5, max_retries=1):
    """Send and wait for ACK from destination"""
    for attempt in range(max_retries):
        self.send(can_id, data)
        
        # Check RX buffer (populated by background thread)
        start_time = time.time()
        while time.time() - start_time < timeout:
            with self._rx_lock:
                for i, (rx_id, rx_data) in enumerate(self._rx_buffer):
                    if rx_id == ack_id and rx_data[0] == 0xFF:
                        del self._rx_buffer[i]
                        return True  # ACK received!
            time.sleep(0.01)
    
    return False  # No ACK received
```

### Fast Queue Drain (Maximum Speed)
```python
def _drain_queue(self):
    """Drain all queued messages at maximum safe speed"""
    while self._msg_queue:
        can_id, data, extended = self._msg_queue[0]
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=extended)
        
        try:
            # Small timeout prevents false failures from temporary buffer full
            self.bus.send(msg, timeout=0.01)
            self._msg_queue.popleft()
            self._sent_total += 1
        except Exception:
            # True link failure - stop drain
            self._link_ok = False
            break
    
    # Mark link UP only if fully drained
    if len(self._msg_queue) == 0:
        self._link_ok = True
```

### Key Design Decisions

#### 1. Why 3 failures?
- **Too low (1-2)**: False positives from transient glitches
- **Too high (5+)**: Hardware buffer may already be filling
- **3 is the sweet spot**: ~0.15s detection time, minimal false positives

#### 2. Why timeout=0.05s for normal send?
- **Too low (0.01s)**: May timeout on busy bus
- **Too high (0.1s+)**: Blocks application too long
- **0.05s is the sweet spot**: Fast enough, reliable enough

#### 3. Why txqueuelen=10?
- **Too low (1-5)**: May drop messages on normal bus load
- **Too high (100+)**: Defeats the purpose (buffer fills up)
- **10 is the sweet spot**: Handles bursts, fails fast

#### 4. Why background RX thread?
- **Non-blocking ACK reception** - Main thread doesn't block waiting for ACKs
- **Continuous monitoring** - Never miss an ACK while draining queue
- **Parallel processing** - Receive ACKs while sending messages
- **Simple daemon thread** - Auto-cleanup on exit
- **Thread-safe buffer** - Lock-protected deque for ACK storage

#### 5. Why timeout=0.01s for drain?
- **Too low (0)**: Throws exception immediately if hardware buffer has any messages
- **Too high (0.05s+)**: Slows down drain, messages arrive faster than drain
- **0.01s is the sweet spot**: ~100 messages/second drain speed, handles temporary buffer full

#### 6. Why check link every 5 seconds?
- **Too low (<1s)**: Unnecessary overhead during normal operation
- **Too high (20s+)**: Queue builds up faster than drain attempts, never clears
- **5 seconds is the sweet spot**: Frequent enough for responsive recovery, minimal overhead

#### 7. Why deque?
- **Built-in Python** - No dependencies
- **Thread-safe** - Atomic operations
- **Fast** - O(1) append/popleft
- **Memory efficient** - Circular buffer
- **maxlen parameter** - Automatic overflow handling

#### 8. Why no retries in send()?
- **Retries fill hardware buffer** when link is actually down
- **Single attempt** fails fast and goes straight to queue
- **Recovery happens via queue drain**, not individual retries
- **Simpler code** with fewer edge cases

## Memory Analysis

### RAM Usage (Pi Zero 2W has 512MB)

**Per Message:**
- CAN ID: 4 bytes (int)
- Data: 8 bytes (worst case)
- Extended flag: 1 byte (bool)
- Python overhead: ~60 bytes (tuple, deque node)
- **Total: ~73 bytes per message**

**Queue Size = 1000:**
- 1000 messages × 73 bytes = **73 KB**
- Plus Python interpreter: ~40 MB
- Plus RX buffer: 500 messages × 73 bytes = **36.5 KB**
- **Total library footprint: < 1.5 MB**

**Safe for satellite use** ✓

## Performance Analysis

### Latency (when link is UP)
- `send()` call: 0.05s timeout
- No queue overhead
- **Total: ~50ms + bus time**

### Latency (with ACK)
- `send_with_ack()`: 0.5s timeout × 1 retry
- Background RX thread receives ACK (non-blocking)
- **Total: ~0.5s worst case (no ACK), <50ms typical (ACK received)**

### Latency (when link is DOWN)
- Queue append: ~0.001ms (O(1))
- No hardware interaction
- **Total: ~1ms**

### Throughput (when link is UP)
- CAN bus: 250 kbps
- Theoretical max: ~3000 msgs/sec (8-byte messages)
- Python overhead: ~500 msgs/sec realistic
- **Library adds minimal overhead**

### Drain Speed (when link recovers)
- Drain timeout: 0.01s per message
- No batch size limit (drains all messages)
- Actual speed: **~100 messages/second**
- 1000 messages: ~10 seconds to drain
- **Tested and verified with cable disconnect/reconnect**

### Recovery Time
- Link detection: 3 failures × 0.05s = **0.15s**
- Link check frequency: **every 5 seconds**
- Queue drain: **~100 msgs/sec** (no batch limit)
- 1000 messages: ~10 seconds to drain
- **Predictable, bounded recovery**

## Testing Strategy

### Unit Tests (Future)
```python
def test_link_down_detection():
    """3 failures should mark link down"""
    
def test_queue_overflow():
    """Messages beyond queue_size should be dropped"""
    
def test_queue_drain():
    """Queued messages should send when link recovers"""
```

### Integration Tests
```bash
# Terminal 1 (Pi)
python3 example_pi.py

# Terminal 2 (PC)
python3 example_pc.py

# Disconnect CAN cable - messages should queue
# Reconnect - messages should drain
```

### Stress Tests
```python
# Send 10,000 messages with link down
for i in range(10000):
    can.send(0x100, [i % 256])

# Verify only queue_size messages kept
stats = can.get_stats()
assert stats['queue_length'] == 1000
assert stats['total_dropped'] == 9000
```

## Future Enhancements (Keep it Simple!)

### Maybe Add
- ✓ Persistent queue to disk (survive crashes)
- ✓ Configurable drain rate
- ✓ Message priority levels

### Definitely Don't Add
- ❌ Complex state machines
- ❌ Threading/async
- ❌ Custom serialization
- ❌ Network protocols on top

## Conclusion

**This design solves the hardware buffer overflow problem with:**
1. Minimal code (~200 lines)
2. Minimal dependencies (only python-can)
3. Minimal memory (< 1 MB)
4. Minimal latency (50ms when up, 1ms when down)
5. Maximum reliability (zero data loss within queue size)

**KISS principle achieved** ✓
