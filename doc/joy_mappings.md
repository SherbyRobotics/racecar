# Joystick Controller Mappings

These are the default mappings for `joy_node`.

## Xbox Series X Controller

| Axis | ID  | Name                        |
| ---- | --- | ----------------------------|
| 0    | LJH | Left Joystick (Horizontal)  |
| 1    | LJV | Left Joystick (Vertical)    |
| 2    | LT  | Left Trigger                |
| 3    | RJH | Right Joystick (Horizontal) |
| 4    | RJV | Right Joystick (Vertical)   |
| 5    | RT  | Right Trigger               |
| 6    | DPH | D-Pad (Horizontal)          |
| 7    | DPV | D-Pad (Vertical)            |

| Button | ID    | Name                     |
| ------ | ----- | ------------------------ |
| 0      | A     | A                        |
| 1      | B     | B                        |
| 2      | X     | X                        |
| 3      | Y     | Y                        |
| 4      | LB    | Left Bumper              |
| 5      | RB    | Right Bumper             |
| 6      | BACK  | Back/Select              |
| 7      | START | Start                    |
| 8      | HOME  | Home                     |
| 9      | LJP   | Left Joystick (Pressed)  |
| 10     | RJP   | Right Joystick (Pressed) |
| 11     | M1    | Share\*                  |

> \* Not all controllers have this button.

## Logitech F710

### DirectInput

| Axis | ID  | Name                        |
| ---- | --- | ----------------------------|
| 0    | LJH | Left Joystick (Horizontal)  |
| 1    | LJV | Left Joystick (Vertical)    |
| 2    | RJH | Right Joystick (Horizontal) |
| 3    | RJV | Right Joystick (Vertical)   |
| 4    | DPH | D-Pad (Horizontal)          |
| 5    | DPV | D-Pad (Vertical)            |

| Button | ID    | Name                     |
| ------ | ----- | ------------------------ |
| 0      | X     | X                        |
| 1      | A     | A                        |
| 2      | B     | B                        |
| 3      | Y     | Y                        |
| 4      | LB    | Left Bumper              |
| 5      | RB    | Right Bumper             |
| 6      | LT    | Left Trigger             |
| 7      | RT    | Right Trigger            |
| 8      | BACK  | Back/Select              |
| 9      | START | Start                    |
| 10     | LJP   | Left Joystick (Pressed)  |
| 11     | RJP   | Right Joystick (Pressed) |

### XInput

| Axis | ID  | Name                        |
| ---- | --- | ----------------------------|
| 0    | LJH | Left Joystick (Horizontal)  |
| 1    | LJV | Left Joystick (Vertical)    |
| 2    | LT  | Left Trigger                |
| 3    | RJH | Right Joystick (Horizontal) |
| 4    | RJV | Right Joystick (Vertical)   |
| 5    | RT  | Right Trigger               |
| 6    | DPH | D-Pad (Horizontal)          |
| 7    | DPV | D-Pad (Vertical)            |

| Button | ID    | Name                     |
| ------ | ----- | ------------------------ |
| 0      | A     | A                        |
| 1      | B     | B                        |
| 2      | X     | X                        |
| 3      | Y     | Y                        |
| 4      | LB    | Left Bumper              |
| 5      | RB    | Right Bumper             |
| 6      | BACK  | Back/Select              |
| 7      | START | Start                    |
| 8      | HOME  | Home                     |
| 9      | LJP   | Left Joystick (Pressed)  |
| 10     | RJP   | Right Joystick (Pressed) |

### `Mode` button

The `Mode` toggle button swaps the left joystick and the D-Pad's indices. In other words:

- LJH <---> DPH;
- LJV <---> DPV.

Additionally, the left joystick now behaves as a D-Pad.

This feature works identically in both input modes.
