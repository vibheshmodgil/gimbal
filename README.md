# ESP32-S3 CAM Pan/Tilt Gimbal

Two motors in total: **pan** in the base, **tilt** on the yoke.
- **Base** (free-standing, battery powered): 3S Li-ion pack + BMS under the MG996R pan servo, 5 V buck, WAGO 5 V/GND bus,
  0.96" OLED screen at the front, rocker switch + charge jack at the back. Nothing hangs outside.
- **Yoke** on the base: MG90S tilt servo on the right arm, 608ZZ bearing on the left arm.
- **Head** (one printed piece): Freenove ESP32-S3 WROOM CAM + 12 mm laser.

Size: Ø100 base (69 mm tall), 89 mm wide, 168 mm tall.

## Folder
```
Gimbal/
├── gimbal.py                 CAD source: rebuilds STL/, STEP/, PRINT_BambuLab/ and runs the checks
├── README.md                 this file (print → parts → assembly → wiring)
├── PRINT_BambuLab/           ★ PRINT THESE
│   ├── 1_fit_test_round4_A1.3mf          fit-test coupons
│   ├── 2_…4_FINAL_*_A1.3mf               final parts, sliced for A1 (ANVYXO on the head front + screen facet)
│   ├── Gimbal_0.20_*_A1.json             process profiles (Strong / FitTest)
│   ├── stl_oriented/                     same parts as single STLs, print-oriented
│   ├── other_printers_geometry_only/     plain plates for any other printer
│   └── last_slice_result.json            slicer log from the last CLI slice
├── STEP/                     single parts + gimbal_assembly.step + gimbal_exploded.step (bought parts = BUY_*)
├── STL/                      single parts in assembly orientation (editing, not printing); fit_test/ coupons
├── vendor_models/            SG90/MG90S servo + horn STEP (read by gimbal.py, don't move)
├── Firmware/
│   ├── ANVYXO_Gimbal_Phase3/ ★ CURRENT firmware (camera + pan/tilt + laser + OLED, OTA) — see WIRING.md
│   └── archive/              Phase1_OLED_only, Phase2_CAM_OLED_WebSocket (earlier test sketches)
└── docs/
    ├── guides/               assembly guides, fit-test guides, measure/check sheets (PNG)
    └── previews/             renders: front, back, exploded
```

`python gimbal.py` runs three checks:
1. No collisions at pan 0/±90° and tilt 0/±45°, including bought parts (battery, buck, WAGOs, switch, jack) and header + Dupont plug space.
2. Every part goes in along its real assembly path: MG996R up through the deck, OLED / buck / WAGOs / capacitor / battery up from below, yoke down onto the horn, board up into the head, laser into its tube, head down into the yoke, MG90S into the arm.
3. A screwdriver or hex key reaches every screw at the step where it's fitted.

---

## 1. Print (set up for your Bambu Lab A1, 0.4 nozzle, Bambu PLA Basic, Textured PEI plate)
Open these in Bambu Studio in order. They are already sliced with the settings below. Just check, then **Print plate**.

| File | Parts | Time |
|---|---|---|
| `1_fit_test_round4_A1.3mf` | fit-test coupons (round 4: new screen slot + back panel) | ~4.3 h |
| `2_FINAL_head_yoke_A1.3mf` | head, yoke | ~9.6 h |
| `3_FINAL_base_A1.3mf` | base, servo cap | ~13.9 h |
| `4_FINAL_bottom_plate_A1.3mf` | bottom plate, head floor, axle pin | ~2.8 h |

**Process profiles** (in `PRINT_BambuLab/`, both based on *0.20mm Standard @BBL A1*):
- `Gimbal 0.20mm Strong`: 4 walls, 5 top/bottom layers, 25% grid, supports off, thick bridges, auto brim.
- `Gimbal 0.20mm FitTest`: 2 walls, 10% infill (coupons only check shape).

**Before you print:**
- Using a different plate (for example Cool Plate)? Change it in the top-left plate dropdown, then re-slice.
- Using PETG? Pick your PETG filament profile, then re-slice.

**Re-slicing from the command line:** `python gimbal.py` rebuilds the parts. The projects are made with the Bambu Studio command line (`--load-settings` machine + profile, `--slice 0 --export-3mf`).

`other_printers_geometry_only/` has plain plates for any other printer.

**Fit test** (plate 1). Check each coupon against your real part:
| Coupon | Check |
|---|---|
| fit_board_top_antenna_camera | Board's antenna tab slides into the roof slot; lens is inside the window |
| fit_head_floor_usb | USB-C cable plugs in through the floor |
| fit_horn_pocket | MG90S double-arm horn lies flat |
| fit_tilt_servo_arm | MG90S goes through the window; ears sit flat |
| fit_608_pocket | 608ZZ presses in |
| fit_pan_horn / fit_pan_servo_deck | MG996R horn in the pocket; MG996R case flush in the deck |
| fit_laser_bore | Laser slides in snug |
| fit_screen | OLED pushes onto the 4 pegs (20.5 × 23 spacing), glass flat on the wall, display fully inside the window |
| fit_back_panel | rocker switch snaps in, charge jack thread + nut fit |

If anything is off, change the matching `MEASURE` value at the top of `gimbal.py` and re-run.

## 2. Bought parts
| Item | Qty |
|---|---|
| Freenove ESP32-S3 WROOM CAM (FNK0085) | 1 |
| 0.96" SSD1306 I2C OLED (4-pin) | 1 |
| 3S Li-ion pack (3× 18650 flat) with 3S BMS, XT30 or JST pigtail | 1 |
| 5 V buck, MP1584 / Mini560 size (3 A+; Mini560 Pro 5 A is better for MG996R stall) | 1 |
| WAGO 221-413 lever connector | 2 |
| KCD1 rocker switch (13 × 19 cut-out) | 1 |
| DC-022B 5.5 × 2.1 panel jack + 12.6 V 3S Li-ion charger | 1 |
| 1–2 mm foam pad (between battery and MG996R) | 1 |
| 12 mm laser module (5 V) + NPN transistor (S8050 / 2N2222) + 1 kΩ resistor | 1 |
| MG996R + round horn | 1 |
| MG90S + double-arm horn | 1 |
| 608ZZ bearing | 1 |
| M3x16 screw (locks the printed axle pin, from the top of the left arm) | 1 |
| M3x10 screw (MG996R → base lugs) | 4 |
| M3x8 countersunk (base bottom plate) | 4 |
| M3x10 countersunk (head floor) | 4 |
| Small self-tapping screws supplied with the servos (horn → head, MG90S ears) | 4 |
| M3x6 screw or grub (laser lock) | 1 |
| 1000 µF 16 V capacitor (Ø10 × 20) | 1 |
| Female Dupont jumpers, 6-core silicone cable (about 30 cm), heat-shrink | — |
| Rubber feet Ø10 | 4 |

## 3. Sub-assemblies and order

**Before you start:** set both servos to centre (90°) with a servo tester or the ESP32, and leave them there.

### Sub-assembly A: Base (pan + power), bottom plate off, base upside-down
**Before you start:** set the buck to **5.1 V** on the bench (trimmer pot) before it touches anything.
1. **Back panel:** snap the rocker switch in from outside. Push the charge jack through from inside, nut on the outside.
2. Push the **MG996R** up through the deck from below until its case top is flush. Shaft to the centre. Fix with 4× M3x10 from below into the lugs.
3. **OLED:** plug 4 Dupont leads on (GND, VCC, SCL, SDA). Slide it up between the rails (glass towards the window, pins on top), then push it forward onto the 4 pegs. A dot of hot glue on each peg tip.
4. **Buck:** solder IN/OUT leads first, then tape it (double-sided foam) flat on the pad on the left wall between the two lips, IN at the bottom.
5. **WAGOs:** one = 5 V bus, one = GND bus (label them). Clamp the wires (below), then push both up into the pocket at the front right, wire entries down.
6. **Capacitor:** push it up into the clip at the front left, leads down, into the WAGOs (+ to 5 V, − to GND).
7. **Harness:** feed the 6-core harness up through the deck slot at the back (behind the servo).
8. **Battery last:** foam pad on top, slide the pack in between the bottom-plate ribs, plug its connector, fit the **bottom plate** (4× M3x8) after testing. Feet on.

**Inside the base, wire by wire:**
| From | To |
|---|---|
| Battery P+ (after BMS) | rocker switch pin 1, charge jack + |
| Battery P− | buck IN−, charge jack − |
| Rocker switch pin 2 | buck IN+ |
| Buck OUT+ | 5 V WAGO |
| Buck OUT− | GND WAGO |
| 5 V WAGO | MG996R red, harness 5 V, capacitor + |
| GND WAGO | MG996R brown, harness GND, capacitor − |
| MG996R orange | harness PAN signal |
| OLED VCC / GND / SDA / SCL | harness 3V3 / GND (spare WAGO slot) / SDA / SCL |

The charge jack sits before the switch: the pack charges (through the BMS) with the switch on or off. Use a **12.6 V 3S Li-ion CC/CV charger**, never a plain 12 V adapter.

### Sub-assembly B: Head
1. Press the **608ZZ** into the left pocket. Use a vise or tap it gently on a flat surface.
2. Screw the **MG90S double-arm horn** into the right pocket (2 self-tappers into the slots), arm vertical.
3. Slide the **laser** into the top tube from the front. Feed its wires back and down through the hole into the head. Lock it with the M3 grub from the top.
4. Wire the laser switch (see wiring below) and heat-shrink it.
5. Plug the Dupont leads onto the **ESP32** headers (see the pin table).
6. Slide the board **up from the bottom**, camera facing the front window, until the antenna tab drops into the roof slot.
7. Lead all wires out through the slot in the floor plate. Screw the **head floor** on with 4× M3x10. Its stops clamp the board's bottom edge.

### Sub-assembly C: Yoke (built on the base)
1. Fit the **round horn** on the MG996R spline with the servo still centred. Fit the horn's centre screw.
2. Set the **yoke** on the horn with the open front facing the screen side. Screw it to the horn through the 4 slots.

### Final assembly
1. Lower the **head (B)** between the yoke arms, horn on the right.
2. Push the printed **axle pin** (flat side up) through the left hub into the 608 bearing until its flange touches the hub. Lock it with an M3x16 screw from the top of the left arm.
3. Push the **MG90S** in from outside through the right-arm window onto the horn spline, cable at the bottom. Line up the shaft and fix the ears through the slots.
4. Slide on the **servo cap**.
5. **Wire routing:** harness comes out of the deck slot at the back → zip-tie it (with the MG90S lead, which runs down the outside of the right arm) through the tunnel in the block on the back of the yoke → up into the head through the floor slot. Leave ~6 cm loose between the deck slot and the yoke block (pan ±90°) and ~8 cm between the block and the head (tilt ±45°). The 6-core is thicker than the 3 mm yoke-to-deck gap, so it can't get pinched under the yoke.
6. Power up with the switch and test. Then fit the **bottom plate** (4× M3x8) and stick on the 4 feet.

## 4. Wiring
Pins are chosen to avoid the camera pins (4–13, 15–18), the strapping pins (0, 3, 45, 46), USB (19/20), PSRAM (35–37) and the RGB LED (48). The OLED pins match your firmware's `config.h` (SDA 21, SCL 47).

| From | To ESP32 pin |
|---|---|
| OLED SDA | GPIO 21 |
| OLED SCL | GPIO 47 |
| OLED VCC / GND | 3V3 / GND |
| MG996R signal (pan, in base) | GPIO 14 |
| MG90S signal (tilt) | GPIO 2 |
| Laser switch (NPN base through 1 kΩ) | GPIO 1 |
| 5 V from base harness | 5V pin + MG90S red |
| GND from base harness | GND + MG90S brown + laser switch emitter |

**Base ↔ head harness (6 wires):** 5V, GND, 3V3 (OLED), SDA, SCL, PAN signal.
- Use 22–24 AWG for 5V/GND and 26–28 AWG for the rest.

**Laser switch:**
- Laser + → 5 V.
- Laser − → NPN collector.
- Emitter → GND.
- Base → GPIO 1 through 1 kΩ.

**Power:** 3S pack → BMS → rocker switch → 5 V buck → WAGO bus (wire table in sub-assembly A).
- Never power the servos from the ESP32.
- When programming over the head's USB-C, switch the base off first, or update over Wi-Fi (OTA).
- Firmware: limit tilt to ±45° and pan to ±90°.

**Laser safety:** never look into the beam or point it at people or animals.
