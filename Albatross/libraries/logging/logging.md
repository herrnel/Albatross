## `logging/`
**Purpose:** Persistent recording of runtime data for debugging, replay, and analysis.  
**What lives here:** Run loggers, event loggers, schemas, and serialization helpers.  
**How it relates to the rest of the code:** Logging should read from shared topics without affecting control timing. The saved logs are later consumed by `replay/`, `eval/`, and training pipelines.

---

## 1) What replay logging is actually for

We want replay so you can do things like:

- run one sim flight once
    
- save all sensor inputs and module outputs
    
- later rerun just perception on the same frames
    
- later rerun estimation on the same IMU + detections
    
- later compare old policy vs new policy on the same observations
    

So replay logging should let us answer:

- what did the sensors produce?
    
- when did they produce it?
    
- what did each module see?
    
- what did each module output?
    
- what command got sent?
    

That means we want to log **dataflow events**, not just periodic snapshots.

---

## 2) Do not use one global tick for replay

With multiple threads, a single global tick becomes misleading.

You have different timing domains:

- platform telemetry pump
    
- camera frame pump
    
- perception thread
    
- estimation thread
    
- control thread
    
- command stream thread
    

These do **not** all run in lockstep.

So instead of one global tick, use:

- **timestamps**
    
- **sequence numbers**
    
- **optional local tick counters per module**
    

## What each one is for

### Timestamp

The most important field.

Use it to know:

- when a camera frame was captured
    
- when IMU was received
    
- when a module processed something
    
- how old an observation was at control time
    

### Sequence number


Helps identify ordering within a topic.

Examples:

- camera frame `seq`
    
- IMU sample `seq`
    
- observation `seq`
    

### Local tick counter

Useful for debugging one module’s cadence.

Example:

- control module tick #18452
    
- estimator tick #2203
    

But this is **not** the master timeline.


### What is the difference between a sequence and a lock tick?

Sequence = data identity

- “Which data item is this?”

Local tick = execution identity

- “Which run of this module is this?”

#### Sequence Explanation

A **sequence number** belongs to a **data stream / topic**.

It answers:

- “Which item in this stream is this?”
    
- “What came before and after it in this topic?”
    
- “Did I miss any items from this source?”
    

Examples:

- camera frame seq: 101, 102, 103
    
- IMU seq: 8801, 8802, 8803
    
- observation seq: 55, 56, 57
    

So sequence is about the **identity and ordering of data**.

##### Example

If the vision adapter publishes frames:

- frame 201
    
- frame 202
    
- frame 203
    

then `seq=202` means:  
“this is the 202nd published frame on the camera topic.”

That is useful for:

- matching a detection back to the frame it came from
    
- checking frame drops
    
- tracking lineage through the pipeline
    

For example:

- `GateDetection.seq = 202` might mean “this detection came from frame 202”
    
- or you might store `frame_seq_ref = 202`

#### Local tick

A **local tick** belongs to a **thread or module execution loop**.

It answers:

- “How many times has this module run?”
    
- “Did this module execute at the rate I expected?”
    
- “What happened on this particular pass through the loop?”
    

Examples:

- control tick: 14001
    
- estimation tick: 3021
    
- logger tick: 990
    

So local tick is about the **execution count of a module**, not the identity of the data.

##### Example

Your control module runs at 500 Hz.

It may do:

- tick 14001: read observation 55
    
- tick 14002: read observation 55 again
    
- tick 14003: read observation 56
    

That means the control loop ran three times, but observation only updated once.

That is completely normal.


---

## 3) What should be logged



##### A. Sensor input logs

These are the most important because they let you reconstruct the run.

Log:

- IMU samples
    
- attitude / local position / heartbeat
    
- camera frames
    
- camera info if it changes
    

These should be logged as close to the adapters as possible.

##### B. Module output logs

These let you debug and compare algorithm versions.

Log:

- gate detections
    
- segmentation summaries
    
- gate-relative state
    
- fused state
    
- observations
    
- policy actions
    
- safe actions
    
- sent commands
    

##### C. Event logs

These are for lifecycle and debugging.

Log:

- arm
    
- offboard entered
    
- module started
    
- module stale
    
- safety override reason
    
- dropped frame
    
- replay started/stopped
    

---

## 4) What a replayable log entry should look like

Every log record should have a common header.

Something like:

```python
{
    "topic": "camera_frame",
    "seq": 128,
    "t_capture": 123.456,
    "t_wall": 456789.123,
    "source": "vision_adapter",
    "payload": {...}
}
```

## Recommended core fields

- `topic`: what stream this belongs to
    
- `seq`: per-topic sequence number
    
- `t_capture`: when the sensor/module says this data is from
    
- `t_publish`: when it was published into the topic store
    
- `source`: adapter/module name
    
- `payload`: serialized typed data
    

For module outputs I’d also add:

- `input_seq_refs`: which upstream items were used
    
- `compute_ms`: how long the module took
    
- `valid`: whether the result was usable
    

Example for observation:

```python
{
    "topic": "observation",
    "seq": 501,
    "t_ref": 124.102,
    "source": "ObservationModule",
    "input_seq_refs": {
        "gate_relative": 220,
        "attitude": 8402
    },
    "valid": true,
    "payload": {
        "vec": [...],
        "mask": [...]
    }
}
```

That makes debugging much easier.

### More Log Types

#### 1. Topic publish records

These log actual data flowing through the system.

```python
{
  "record_type": "topic_publish",
  "topic": "observation",
  "seq": 52,
  "t_ref": 10.033,
  "t_publish": 10.034,
  "source_module": "ObservationModule",
  "source_local_tick": 1881,
  "input_refs": {
    "gate_relative_seq": 88,
    "attitude_seq": 3012
  },
  "valid": true,
  "payload": {
    "vec": [0.12, -0.03, 0.88, 0.02],
    "mask": [1, 1, 1, 1]
  }
}
```

Here:

- seq=52 means observation item #52
- source_local_tick=1881 means ObservationModule created it on its 1881st loop iteration