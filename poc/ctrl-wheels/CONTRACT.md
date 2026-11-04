# ctrl-wheels contract (twin wheel pair)

This file is the wire and file contract for the twin-wheel proof of concept. It covers the WHL2 line sent by the firmware, the UDP v2 messages between the bridge and the Webots controller, the ports, the stale and hold rule, the CSV log columns and the golden vectors. Code written for the twin must match this file. When code and this file disagree, fix one of them in a separate, reviewed change; do not let them drift.

## S0 Status

- SIM ONLY. Nothing in this contract is validated on real hardware. Nothing here has been built, run or tested by the author of this file.
- Plan: https://github.com/pluto-atom-4/balancing-robot-controller/discussions/266 (cited below as "discussion 266", with the section or row name).
- Umbrella issue: freecad-workspace FW#482. This contract is freecad-workspace #486, task F02 in the Task table of discussion 266.
- The firmware for the twin will live in balancing-robot-controller (BRC) `programs/c3_ctrl_wheels/` (Wokwi, sim only). The firmware for the twin is not present. A placeholder directory exists there (README only); a verbatim copy of the single-wheel program may follow in an open PR. Neither is a twin implementation.
- The bridge, relay, UDP, log and Webots side will live in freecad-workspace `poc/ctrl-wheels/`. This is planned: at the time of writing that directory holds only a README and no twin code. The single-wheel code that this contract copies from lives in freecad-workspace `poc/ctrl-wheel/` (no `s`), which is not changed by this file.
- The sign convention is UNCONFIRMED (see S2). The mirror of the two wheels in Webots is UNVERIFIED.
- The WHL v1 format (`lib/wheel_frame` in BRC, `ctrl_wheel_msg.py` in `poc/ctrl-wheel`) is unchanged by this file. The single-wheel directory `poc/ctrl-wheel` and the single-wheel programs `programs/c3_ctrl_wheel` and `ttl_servo` are untouched by this file.
- Source citations of the form `ctrl_wheel_*.py:NN`, `fake_wheel.py:NN` or `README.md:NN` refer to freecad-workspace `poc/ctrl-wheel/`, as read for this contract. Citations of the form `CLAUDE.md` refer to the BRC repository root.

## S1 WHL2 line

Format, one line per sample, ASCII:

    WHL2,<seq>,<t_us>,<model %.6f>,<rawL>,<rawR>,<state>*<HH>\n

The line has 7 comma-separated fields counting the `WHL2` prefix: the prefix, seq, t_us, model, rawL, rawR and state. The `*` introduces the checksum. There is exactly one `state` field for the pair (discussion 266, Questions for the human, row 3b).

Field rules:

- seq: uint16. The formatter takes seq modulo 65536 (`ctrl_wheel_msg.py:143`). The parser rejects a seq of 65536 or more (`ctrl_wheel_msg.py:193-194`). Decimal digits only (`ctrl_wheel_msg.py:55`, `103-106`).
- t_us: uint32, decimal digits only, range 0 to 4294967295 (`ctrl_wheel_msg.py:46`, `137-138`, `196-198`).
- model: printed with `%.6f`. The value must be finite. The formatter refuses a non-finite model (`ctrl_wheel_msg.py:145-147`). A model that prints as `-0.000000` is printed as `0.000000` (`ctrl_wheel_msg.py:148-150`). The model is a plant velocity in rad/s, forward-positive by this contract's definition (see S2; whether that matches Webots is UNVERIFIED).
- rawL and rawR: signed decimal integers, int32 range -2147483648 to 2147483647 (`ctrl_wheel_msg.py:47-48`, `89-91`, `139`, `202`). They are the raw goal velocities that the firmware wrote to each chip (see S2). They are not a readback from the servo.
- state: one of 0, 1 or 2 (`ctrl_wheel_msg.py:49`, `140-141`, `205-206`). 0 is run. 1 is hold after a fall. 2 is bus down. Both wheels latch together, so the state applies to the pair. If both a fall-hold and a bus-down condition are present, 2 wins (discussion 266, Task table row B11; Questions row 4).
- HH: exactly 2 characters, uppercase hex `0-9A-F`, the XOR of the ASCII bytes of the body. The body is everything from the leading `W` of `WHL2` up to, not including, the `*`. A lowercase digit or a checksum that is not exactly 2 characters is rejected (`ctrl_wheel_msg.py:74-81`, `169-184`). A non-ASCII character in the body is refused (`ctrl_wheel_msg.py:78-79`).
- Line terminator: the formatter writes `\n` (`ctrl_wheel_msg.py:153`). The parser accepts a trailing `\n` or `\r\n` (`ctrl_wheel_msg.py:167`, the `rstrip` in the code; the same is stated in the docstring at `ctrl_wheel_msg.py:158`).
- No whitespace and no underscore in any token. The float token check and the integer token check of the single-wheel parser are the reference (`ctrl_wheel_msg.py:115-127` for floats, `ctrl_wheel_msg.py:109-112` for signed integers). The parser must reject a non-finite model token: `nan`, `inf` and `-inf` are rejected (`ctrl_wheel_msg.py:125-126`).
- A leading `+` is accepted only for the raw token (signed integer pattern `_INT_RE`, `ctrl_wheel_msg.py:56`, used at `:201`) and for the model token (through `float()` at `ctrl_wheel_msg.py:122`). The seq, t_us and state tokens use the unsigned pattern `_UINT_RE` (`ctrl_wheel_msg.py:55`, `103-106`), which rejects a `+`. Whether the WHL2 parser keeps this leniency for raw and model is not decided here; see S8.

Non-finite model, firmware side: the firmware formatter refuses the line. It prints no line for that tick. The cross-repo sync points of discussion 266 say the C++ formatter returns 0 in that case and the Python formatter raises (discussion 266, Cross-repo contract sync points). This is the planned behaviour for WHL2 and is not yet built. For the single-wheel WHL v1 formatter, the `ctrl_wheel_msg.py` docstring says the BRC firmware still printed `0.000000` for a non-finite model until BRC PR #262 (issue #261) merges (`ctrl_wheel_msg.py:10-11`, `19`). The merge status of that PR was not checked for this file.

Rejects that the WHL2 parser must make:

- A WHL line (prefix `WHL`, 6 fields) is rejected by the prefix check and by the field count of 7 (`ctrl_wheel_msg.py:186-190` is the reference for the field count and prefix check).
- A WHL2 line is rejected by the WHL v1 parser, which requires 6 fields (`ctrl_wheel_msg.py:50`, `187-188`). This is the reverse direction and must be tested in both repos (discussion 266, Cross-repo contract sync points).
- A line with 6 or 8 fields, a bad HH, state 3 and a nan model are rejected. The full list of reject cases is in S7.

## S2 Sign convention

There are two independent places where a sign is applied. They are not proven to agree.

- Chip side (planned firmware, not built). The firmware must apply the chip sign when it writes to the two servos: `rawL = +raw` and `rawR = -raw`. This is the mirror for the chip and servo side. The `rawR` on the wire is already negated. The plan puts the sign in one constants place (discussion 266, Task table rows B08 and B09; the rule is in the bold "Sign convention" paragraph of the Task table).
- Webots side (bridge). The bridge applies its own per-wheel sign to `model` for the command it sends to the Webots controller. The defaults are +1 for left and +1 for right (discussion 266, Task table sign convention paragraph). The relay takes a sign per instance and defaults to 1 (`ctrl_wheel_relay.py:57`). The relay applies the sign before the symmetric clamp (`ctrl_wheel_relay.py:19-20`, `90-91`).
- Why the Webots defaults are +1/+1 (decided design intent, not built or checked): the Webots world is to be built so that a positive command drives both hinges forward, using mirrored offsets and axes in the world (discussion 266, Task table row F17). The +1/+1 defaults follow from that intended world design. The firmware sign and the bridge sign are independent. Nothing in this contract, and nothing run so far, proves that the chip-side mirror and the Webots-side mirror agree. This is UNVERIFIED.
- `model` is one plant velocity in rad/s. "Forward-positive" is the DEFINITION used by this contract (discussion 266, Questions row 3, and the sign convention paragraph of the Task table). Whether that definition matches the Webots world is UNVERIFIED (see S2 and S8). Both wheels share `model` in this version. The model is printed with `%.6f`.
- `rawL` and `rawR` are the raw goal velocities exactly as written to each chip. The wire carries these values, not the model.
- The wire accepts any int32 for rawL and rawR. The raw cap of 125 is a firmware step cap and is not a wire rule (`ctrl_wheel_msg.py:20-22` and `51-52`). In the single-wheel code, the Python step generator `step_lines` applies it (`ctrl_wheel_msg.py:211-214`, `217-232`). CLAUDE.md (BRC repository root) says, for `ttl_servo`, that the wheel velocity is "capped on the host at `segway::kWheelCapRaw` = 125". That is the only cap statement read for this file. The twin firmware's cap, and where it sits before the write, is planned and was not re-read. If the planned firmware cap holds, rawL and rawR on the wire stay within ±125 in practice, but the wire parser does not enforce that cap.
- The conversion used by the Python step generator is rad/s to Dynamixel raw: `raw = rad/s * 60 / (2 * pi) / 0.229`, clamped to ±125, then rounded (`ctrl_wheel_msg.py:53`, `211-214`). The firmware conversion is a separate C++ implementation and is not restated here.
- SIGNS ARE UNCONFIRMED. The sign of the model against Webots, the sign of the raw against the servo and the mirror of the two wheels are all UNVERIFIED (discussion 266, Risks and unverified items; Assessment, row Q2). The `poc/ctrl-wheel` sign for the single wheel is also unverified (`ctrl_wheel_msg.py:6`, `ctrl_wheel_udp.py:6`).

## S3 UDP v2

Transport: one datagram per message, compact JSON, UTF-8.

Provenance of the rules. The following are copied from the single-wheel `ctrl_wheel_udp.py` header and checks: no duplicate keys (`ctrl_wheel_udp.py:106-112`, applied to every JSON object through the object hook at `ctrl_wheel_udp.py:124`, so nested objects are covered), booleans rejected for numbers (`ctrl_wheel_udp.py:71`, `83`, `91`; the clamp reaches the same check through `_finite_real` at `:98`), `NaN`, `Infinity` and `-Infinity` rejected (`ctrl_wheel_udp.py:102-103`, `123`), integer fields must be JSON integers (`ctrl_wheel_udp.py:83-84`), the top-level key set check (`ctrl_wheel_udp.py:132-135`), compact encoding with `allow_nan=False` (`ctrl_wheel_udp.py:146`, `172`), and the key-name duplication rule for the Webots controller (`ctrl_wheel_udp.py:9-11`). The header lists the same rules at `ctrl_wheel_udp.py:16-23`.

NEW for v2, not copied: the exact-key-set rule for nested objects. The single-wheel source checks only the top-level key set (`ctrl_wheel_udp.py:132-135`); it has no nested objects. The exact key sets inside `wheels`, `left` and `right` are therefore a rule of this contract, not a copied rule.

Version rule for v2 (this contract, not copied): the `v` value must be the JSON integer 2. The single-wheel messages have no `v` key, so v1 and v2 are told apart by key set (see below).

Command, host to sim (`cmd`):

    {"v":2,"cmd_seq":<int >= 0>,"wheels":{"left":<float>,"right":<float>},"stale":<0|1>}

- Exact top-level key set: `v`, `cmd_seq`, `wheels`, `stale`.
- Exact key set inside `wheels`: `left`, `right` (this contract; see the provenance note above). Both are finite real numbers, not booleans.
- `cmd_seq`: JSON integer, 0 or more (`ctrl_wheel_udp.py:85-86`).
- `stale`: JSON integer 0 or 1, not a boolean (`ctrl_wheel_udp.py:42`, `90-93`).
- Each wheel value is clamped to ±3.0 rad/s on encode and again on decode (`ctrl_wheel_udp.py:41`, `96-99`, `139-146`, `149-156`). The clamp is silent: an out-of-range value is clamped, not refused.

Echo, sim to host (`echo`), nested. This layout is decided in this contract. Discussion 266 names the echo only as "left and right angle and velocity" (discussion 266, Blueprint, one-tick sequence diagram).

    {"v":2,"cmd_seq_echo":<int >= 0 or null>,"sim_t_s":<float>,"left":{"angle_rad":<float>,"vel_rad_s":<float or null>},"right":{"angle_rad":<float>,"vel_rad_s":<float or null>}}

- Exact top-level key set: `v`, `cmd_seq_echo`, `sim_t_s`, `left`, `right`.
- Exact key set inside `left` and inside `right`: `angle_rad`, `vel_rad_s` (this contract; see the provenance note above).
- `cmd_seq_echo` may be null, otherwise a JSON integer 0 or more (`ctrl_wheel_udp.py:17-18`, `23`, `181`).
- `vel_rad_s` may be null, otherwise a finite float (`ctrl_wheel_udp.py:23`, `184`).
- `sim_t_s` and `angle_rad` are finite floats (`ctrl_wheel_udp.py:168-169`, `182-183`).

Version separation: v1 and v2 reject each other by key set. A v1 command has keys `cmd_seq`, `wheel_rad_s`, `stale` and no `v`. A v2 command has `v` and `wheels`. The v1 decoder rejects the v2 key set and the v2 decoder rejects the v1 key set (`ctrl_wheel_udp.py:132-135` for the top-level key check).

## S4 Ports

- UDP command, host to sim: port 5018. The Webots controller binds this port. This is decided, and it follows the single-wheel design: the controller binds the command port 5008 (`README.md:46`, `README.md:101`).
- UDP echo, sim to host: port 5017. The host bridge receives on this port. This is decided, and it follows the single-wheel design, where echoes go from the controller to the host echo port 5007 (`README.md:46`, `README.md:101`; the port roles are stated in `ctrl_wheel_udp.py:13`). The single-wheel bridge's own bind call was not re-read for this file.
- Host address for the single-wheel helpers is `127.0.0.1` (`ctrl_wheel_udp.py:37`, `188`). The twin uses the same default unless a later task changes it. This is an assumption carried over from the single wheel.
- Reserved: the literal set {5005, 5006, 5007, 5008, 5017, 5018} must not be bound by a test or a fake wheel. This set is the rule of this contract. The single-wheel fake wheel refuses only command ports 5005 to 5008 and echo port 5006 (`fake_wheel.py:70-73`); it does not refuse 5017 or 5018. Copies of the code must not derive it from a range expression such as `range(5005, PORT_CMD + 1)`. The single-wheel `fake_wheel.py` builds its reserved set that way (`fake_wheel.py:52`) and adds `_RESERVED_ECHO_PORT = 5006` (`fake_wheel.py:53`). Changing `PORT_CMD` would silently widen such a range, so the literal set must be written out in the twin copies.
- Port 5006: the single-wheel source says it is never used by `ctrl_wheel_udp.py` (`ctrl_wheel_udp.py:13-14`, `README.md:41`). The rule that no module of the twin uses 5006 is a decision of this contract, not a copied source statement. The single-wheel README also says that 5006 belongs to `servo_wheel_dof` (`README.md:101`), which is a different project.
- The single-wheel fake wheel builds its command-port set as `range(5005, PORT_CMD + 1)` (`fake_wheel.py:52`, `70-71`). The twin adds 5017 and 5018 to the literal reserved set (discussion 266, Task table row F12).
- RFC2217 for the Wokwi serial port: port 4002. Ports 4000 (`ttl_servo`) and 4001 (`c3_ctrl_wheel`) are taken by other programs (discussion 266, Task table row B05; Questions row 5; CLAUDE.md).
- RFC2217 is plain TCP with no authentication. The bind address is not checked. The VS Code extension keeps its listener on port 4000 open after the simulation stops; this is observed for 4000 (CLAUDE.md). For 4002 the same may happen; it is not observed. Running three RFC2217 listeners at once (4000, 4001 and 4002) has never been tried.
- Before any run, check that nothing already listens on the ports the twin uses: `ss -ltn` for TCP port 4002, and `ss -lun` for UDP ports 5017 and 5018. `ss -lunp` is the UDP form used by the single-wheel README (`README.md:87`). `ss -ltn` lists TCP only, so it does not check the UDP ports.
- The bridge reads the Wokwi port with `--port rfc2217://localhost:4002` (discussion 266, Task table rows F13 and F16).

## S5 Stale and hold

The stale and hold rule is the single-wheel relay rule, applied to the pair. It is pure logic, with no sockets and no sleeping.

Single-wheel source (`ctrl_wheel_relay.py`):

- `STALE_S = 0.200` (`ctrl_wheel_relay.py:36`).
- `CLAMP_RAD_S = 3.0` (`ctrl_wheel_relay.py:38`).
- `poll()` before the first sample returns None, so nothing is sent (`ctrl_wheel_relay.py:81-82`).
- A sample with state other than 0 gives `RelayOut(0.0, 1)` (`ctrl_wheel_relay.py:84-85`, rule text at `:12`).
- An age of 200 ms or more since the latest sample gives `RelayOut(0.0, 1)` (`ctrl_wheel_relay.py:86-87`, rule text at `:13`).
- A non-finite model on a fresh state-0 sample gives `RelayOut(0.0, 1)` (`ctrl_wheel_relay.py:88-89`, rule text at `:14-18`). This is defence in depth; the formatter and parser already refuse non-finite values.
- Otherwise the output is `clamp(sign * model_rad_s)` with stale 0 (`ctrl_wheel_relay.py:90-92`, rule text at `:19-20`).
- The age uses the host clock, `time.monotonic` by default, not the device `t_us` (`ctrl_wheel_relay.py:8`, `54`, `77`, `86`).

Twin rule (this contract, following discussion 266, Task table row F14 for the bridge and row B11 for the firmware latch):

- One WHL2 state drives both wheels. If the state is not 0, or the host age is 200 ms or more, both wheels get `0.0` and stale is 1. Both wheels stop together (discussion 266, Task table row F13, "state 1 or 2 stops both").
- Otherwise the left output is `clamp(signLeft * model)` and the right output is `clamp(signRight * model)`, with both signs defaulting to +1 (discussion 266, Task table sign convention paragraph). The clamp is ±3.0 rad/s. Stale is 0.
- The twin keeps one stale flag per command row (see S6).

## S6 CSV

Source: `ctrl_wheel_log.py`. The single-wheel column tuple is at `ctrl_wheel_log.py:30-46`. The column docs are at `ctrl_wheel_log.py:50-66` and a test pins their keys to the column tuple (`ctrl_wheel_log.py:48`). The twin changes the columns as follows:

- `whl_raw` becomes `whl_raw_l` and `whl_raw_r`.
- `cmd_rad_s` becomes `cmd_left_rad_s` and `cmd_right_rad_s`.
- `angle_rad` becomes `angle_rad_l` and `angle_rad_r`.
- `vel_rad_s` becomes `vel_rad_s_l` and `vel_rad_s_r`.
- `whl_model_rad_s`, `whl_state`, `stale`, `cmd_seq`, `echo_cmd_seq`, `sim_t_s`, `host_rtt_ms`, `missed_whl`, `host_t_s`, `whl_seq` and `whl_t_us` stay single columns, as in the single-wheel log.

Meaning of the `whl_raw_l` and `whl_raw_r` columns in the twin: they are the raw goal velocities that the firmware wrote to each chip, taken from the WHL2 line. They are NOT a servo readback. The single-wheel `COLUMN_DOC` describes `whl_raw` as "as reported by the servo" (`ctrl_wheel_log.py:55`). That wording does not apply to the twin and must not be carried over.

Final column tuple, in this order (19 columns):

    host_t_s, whl_seq, whl_t_us, whl_model_rad_s, whl_raw_l, whl_raw_r, whl_state, cmd_seq, cmd_left_rad_s, cmd_right_rad_s, stale, echo_cmd_seq, sim_t_s, angle_rad_l, angle_rad_r, vel_rad_s_l, vel_rad_s_r, host_rtt_ms, missed_whl

Column meanings copied from `COLUMN_DOC` where they still apply: `host_t_s` is host wall-clock seconds, 3 decimals (`ctrl_wheel_log.py:51`). `whl_raw_l` and `whl_raw_r` are the raw values written to each chip, as described above (not the servo readback). `stale` is 0 or 1 as supplied by the caller (`ctrl_wheel_log.py:59`). `echo_cmd_seq` is the value of the echo key `cmd_seq_echo`, empty when there is no echo (`ctrl_wheel_log.py:23-24`, `60`). `sim_t_s`, `angle_rad_*` and `vel_rad_s_*` are empty when there is no echo (`ctrl_wheel_log.py:61-63`). `host_rtt_ms` is wall milliseconds from sending `cmd_seq` to receiving the echo with the same `cmd_seq_echo`. It includes up to one simulation step and one bridge poll, so it is not link latency. It is empty when no echo has arrived (`ctrl_wheel_log.py:17-21`, `64`). `missed_whl` is the count of missed wheel telemetry frames as supplied by the caller (`ctrl_wheel_log.py:65`).

Formatting rules. The rules are stated in the module docstring (`ctrl_wheel_log.py:11-15`) and implemented in the cell code (`ctrl_wheel_log.py:84-111`):

- None or a missing key gives an empty cell (`ctrl_wheel_log.py:86-87`).
- bool is refused with ValueError (`ctrl_wheel_log.py:88-89`).
- Integer columns are written as decimal integers. They are `whl_seq`, `whl_t_us`, `whl_raw_l`, `whl_raw_r`, `whl_state`, `cmd_seq`, `stale`, `echo_cmd_seq` and `missed_whl`. Any non-int value is refused (`ctrl_wheel_log.py:68-77`, `90-93`).
- Float columns use `%.6f`, except `host_t_s` and `host_rtt_ms`, which use `%.3f` (`ctrl_wheel_log.py:79`, `102-103`).
- NaN, inf and out-of-float-range values are refused (`ctrl_wheel_log.py:96-101`).
- A row with an unknown column name is refused (`ctrl_wheel_log.py:108-110`).
- A row is fully validated before anything is written, so a refused row leaves no partial line (`ctrl_wheel_log.py:8-9`, `139-144`).
- The header is written on construction. Lines end with `\n`. A path target is opened with mode `x` unless overwrite is requested (`ctrl_wheel_log.py:122-131`).

## S7 Golden vectors

GOLD: to be filled by the main session. The values below are placeholders. The main session computes the checksums with Python in scratch and fills every value cell. This file does not compute any checksum or any line.

| case | seq | t_us | model | rawL | rawR | state | exact line |
|---|---|---|---|---|---|---|---|
| positive (all numeric values positive, a format case, not a sign claim) | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| rawL positive, rawR negative | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| both raws negative | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| -0 model (prints 0.000000) | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| state 0 | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| state 1 | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| state 2 | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| checksum with a leading-zero HH | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| seq 65535 | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| t_us 4294967295 | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| int32 extremes (-2147483648 and 2147483647; which side takes which is TBD) | TBD | TBD | TBD | TBD | TBD | TBD | TBD |

Each exact line ends with `\n`. Each line is checked by the C++ test (BRC) and by the Python test (freecad-workspace). The C++ output must equal each string byte for byte. The Python parser must give back the fields and the Python formatter must reproduce the string (discussion 266, Cross-repo contract sync points).

Reject cases. Each of these must be refused by the WHL2 parser. The main session fills in a concrete line for each case, or marks it TBD, and does not compute the checksum by hand.

- 6-field line: a WHL2 line with one field removed (prefix, seq, t_us, model, rawL, rawR, state minus one). Refused by the field count.
- 8-field line: a WHL2 line with one extra field. Refused by the field count.
- WHL line: a valid WHL v1 line with prefix `WHL` and 6 fields. Refused by the prefix and field count.
- Bad HH: a line whose HH does not match the body. Refused by the checksum check.
- state 3: a line with state 3 and a correct checksum. Refused by the state range.
- nan model: a line whose model token is `nan`. Refused by the finite check.

The S1 rules also imply further rejects (seq 65536 or more, t_us above 4294967295, raw outside int32, lowercase HH, non-ASCII body). They are not required here.

## S8 Not verified

Nothing in this file has been verified, tested or validated on hardware. In particular:

- All signs are UNCONFIRMED: the chip-side `rawL = +raw` and `rawR = -raw`, the bridge defaults of +1/+1 on `model`, and the sign of the model against Webots (discussion 266, Risks and unverified items; Assessment, row Q2).
- The chip-side mirror and the Webots-side mirror are two independent sign paths (S2). Nothing proves that they agree. The mirror of the two wheels in Webots is UNVERIFIED.
- Two servo chips on one DATA bus: electrical sharing, the chip ID attribute (ID 1 and ID 2) and the diagram attribute form are UNVERIFIED (discussion 266, Risks and unverified items; Not verified list of the web research section). The Wokwi chip attribute API is not confirmed.
- Port 4002 behaviour: the bind address is not checked, the VS Code listener may stay open after the run (observed for 4000 only, CLAUDE.md), and three RFC2217 listeners at once have never been tried (discussion 266, Risks and unverified items).
- UDP port 5017 and 5018 binding: the single-wheel bridge's bind call was not re-read for this file.
- Webots limits `maxVelocity` and `maxTorque` may stall the twin. These values are not read here (discussion 266, Risks and unverified items).
- Timing: the 20 ms tick with two writes and two blocking reads is an estimate of about 9.4 ms for the writes only. It is not measured. The reads add more. OWN1 is the owner step that would measure it (discussion 266, Risks and unverified items; Task table row OWN1).
- The firmware raw cap value in the twin and its position before the write were not re-read for this file (S2).
- The WHL2 parser's handling of a leading `+` in tokens is inherited from the single-wheel parser and is not decided here (`ctrl_wheel_msg.py:56`, `122`).
- The status of the non-finite refusal in BRC firmware (BRC PR #262, issue #261) was not checked for this file (`ctrl_wheel_msg.py:10-11`, `19`).
- The copied clamp (3.0 rad/s) and reserved-port checks in the twin copies are duplicated by design and may drift (discussion 266, Task table row F01; Assessment, row Q6).
- The v2 `v` rule, the nested `left`/`right` echo layout and the `wheels` object, and the nested exact-key-set rule, are decided for this contract. The single-wheel sources do not define them, so they are not yet built or tested.
- The 5006 never-used rule for the twin is a decision of this contract; the single-wheel source states it only for its own module (S4).
- Behaviour of the host-clock stale rule under bridge load is not measured.
- The golden vectors in S7 are not computed. No test, build, simulation, Webots run or hardware run has been made for this file.
