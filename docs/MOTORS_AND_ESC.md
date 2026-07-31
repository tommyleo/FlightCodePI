# Motors and ESC wiring

FlightCodePI controls four ESCs with unidirectional DSHOT. DSHOT150, DSHOT300
and DSHOT600 are selectable from the Configurator; DSHOT300 is the default.
Bidirectional DSHOT telemetry is not currently used.

## Signal connections and motor order

View the quad from above with its nose pointing forward:

```text
              FRONT

        M4               M2
   front left       front right
      GP6               GP2


        M3               M1
    rear left        rear right
      GP3               GP1
```

| Motor | Position | Pico GPIO | Physical pin |
|---:|---|---:|---:|
| M1 | Rear right | GP1 | 2 |
| M2 | Front right | GP2 | 4 |
| M3 | Rear left | GP3 | 5 |
| M4 | Front left | GP6 | 9 |

Connect each GPIO only to the corresponding ESC signal input. Connect every
ESC signal ground to Pico GND, even when all ESCs are part of a 4-in-1 unit.
For a 4-in-1 ESC, follow its connector pinout rather than wire color alone.

## Power

The motor battery connects to the ESC or power-distribution board, never to a
Pico GPIO or 3V3(OUT). The Pico may be powered from USB during setup or from a
suitable regulated supply through VSYS in the finished aircraft.

If several ESCs provide BEC outputs, do not connect multiple regulated outputs
together unless the ESC manufacturer explicitly permits it. All grounds must
remain common.

## Motor direction

Motor position and motor rotation are separate settings. First verify that M1
through M4 match the physical positions above. Then configure normal or
reversed motor direction in the Configurator and arrange propeller direction to
match that choice.

If a brushless motor rotates the wrong way, disconnect power and swap any two
of its three phase wires, or reverse that ESC through supported ESC software.
Never change motor phase wiring while the battery is connected.

## Safe motor test

1. Remove all four propellers.
2. Secure the frame and keep CH6 in the disarmed position.
3. Connect the Configurator and open the Motors page.
4. Confirm M1, M2, M3 and M4 individually at the lowest useful test value.
5. Stop immediately if the selected motor does not match the diagram.

The firmware disables motor test when the arm switch is high and applies a
one-second command timeout. These interlocks do not replace removing the
propellers.
