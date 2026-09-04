# Laser operation guide

Covers the Enable/Ready sequence, what each state actually means, and what to
do when arming does not take.

---

## Read this first

`READY_ON_NC = true` — pin 8 is on the relay's **NC** terminal:

| pin 8 | coil | contact | laser |
|---|---|---|---|
| LOW | released | **closed** | **ARMED** |
| HIGH | energised | open | standby |

Standby requires the coil to stay **energised**. Anything that de-energises it
— a Teensy reset, USB replug, firmware upload, loose jumper, or loss of the
relay supply — closes the contact and **arms the laser with nobody commanding
it**.

- **De-energise the relay supply before flashing.**
- **Power-up order:** Teensy running → relay supply on → Dornier on.
- **Shutdown:** Dornier off → relay supply off → Teensy off.
- Never leave the rig energised unattended.

The real fix is hardware: move pin 8 from NC to NO, set `READY_ON_NC = false` in
`shootingyesready_inoF-1.ino`, regenerate. Then de-energised means standby and
power loss is fail-safe.

---

## The controls

| Button | Action |
|---|---|
| Left secondary (LB) | **Enable + Ready** — arms the laser |
| Right secondary (RB) | **Fire** — hold to fire, release to idle |
| Left primary (LT) | **Standby** — opens the ready contact, disarms |
| Right primary (RT) | **Pause** — fire channels to idle, stays armed |
| X | **E-stop** — stops motion, relocks the laser |
| H (hold 0.6 s) | Home all axes |

Normal sequence: **LB** to arm → hold **RB** to fire → release → **LT** to
disarm when done.

---

## Why arming used to be inconsistent

Two things in the firmware, both now handled:

**The ready contact arms on an EDGE, not a level.** The Dornier looks for the
contact going open → closed. Sending `r` while the contact is *already* closed
does nothing at all. Arming from an unknown state was therefore a coin flip —
sometimes the contact happened to be open and it worked, sometimes it did not.

**`r` is ignored unless `e` was accepted first.** A blind `e` then `r` never
checked whether either landed.

`arm()` now:

1. Sends `x` — relock. Guarantees the contact is **open**, so the next `r`
   produces a genuine closing edge.
2. Sends `e`, and checks `enabled` came back set.
3. Sends `r`, and checks `ready` came back set.
4. Retries the whole sequence once if ready did not latch.
5. Raises with guidance if it still did not.

Pressing LB twice in a row is now safe and deterministic — it relocks and
re-arms rather than sending a no-op `r`.

---

## What the state line means

```
laser arm -> enabled=1 ready=1 firing=0 pins=0/0/0
```

`enabled` / `ready` / `firing` are the **firmware's own command state**. Nothing
in this system reads back from the Dornier — the firmware says so itself. They
mean "the commands were issued and acknowledged", not "the machine is armed".

`pins` are the raw levels on 8/9/10. On NC wiring, **pin 8 = 0 means armed**
and **pin 8 = 1 means standby** — inverted from what you would expect.

**Always confirm against the Dornier's own panel.**

---

## Troubleshooting

**Software says ready=1, the Dornier does not show READY.**
The pin 8 sense is likely wrong. Flip it at runtime without reflashing:

```bash
cd "/Users/aryan/Documents/Code/KatzLab/python" && ./.venv/bin/python -m katzlab monitor
```

Type `w` to flip, then `x`, `e`, `r`, and watch the machine. `w` is runtime-only
and resets on reboot — if the flipped sense is the correct one, set
`READY_ON_NC = false` in the source sketch and regenerate.

**Arming raises "could not arm the laser after 2 attempts".**
The firmware did not acknowledge `e` or `r`. Check the relay supply is on and
that the Teensy is running. Listen for the relay clicking.

**Fire is accepted but the laser does not fire.**
This is the open hardware item documented in `shootingyesready_inoF-1.ino`. The
fire relays land on a Steute footswitch, not on the Dornier directly, so the
pedal's own contacts are still in the circuit. The decisive test needs no code:
arm the machine and press the **physical pedal**. Fires on foot but not on
relays → the difference is in the contacts, not the software. Then meter the
Steute terminals pressed vs released to get the contact map.

Runtime experiments, all via `katzlab monitor`:

| Command | Effect |
|---|---|
| `1` / `2` | toggle fire channel 1 / 2 |
| `3` / `4` | invert channel 1 / 2 polarity |
| `o` | swap which channel leads |
| `g` | step the inter-channel gap |
| `t` / `y` | bench-test one relay in isolation (ready forced open) |
| `?` | full status |

**"could not relock laser during shutdown".**
The firmware does not read serial while a motion move is in flight, so a stop
issued mid-move waits for that move. The timeout is now 35 s with one retry. If
it still fails you get a CRITICAL line — treat the machine as armed, use the
Dornier's own standby, and cut the relay supply.

---

## Limits you cannot fix in software

- **Nothing reads back from the Dornier.** Every state here is what was
  commanded, not what the machine did.
- **A laser stop queues behind an in-flight motion command** on a single board —
  both share one serial port. Bounded by the chunk size (~0.8 mm, ~130 ms), not
  zero.
- **Cutting the relay supply is the real emergency stop.** Everything else is
  software asking politely.
