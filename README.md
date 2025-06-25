Project Overview
Block 1: System Boot + Access Control (Keypad + LCD)
System powers on.

LCD displays "Welcome".

Waits for user to press '*' on keypad.

Prompts: "Input Passkey:".

User enters the 4-digit passkey (e.g., 2580).

If passkey matches:

LCD: "Access Granted"

System starts.

If wrong:

LCD: "Wrong Passkey!" → Try again

2.Peripheral Initializations
GPIOs enabled for:

Traffic lights (RYG LEDs for Lane A and B).

Ultrasonic sensors (TRIG and ECHO for each lane).

Keypad (Rows as Output, Cols as Input with Pull-Up).

TM1637 7-segment displays (Lane A & B timers).

Timer 2 (TIM2) setup:

Prescaler = 16-1 → 1 µs tick (for measuring echo time).

DWT Cycle Counter setup (used for microsecond delays).

LCD initialized.

3.Initial Traffic Light State Setup:
Both Lane A and B traffic lights start as:

RED = ON

YELLOW = OFF

GREEN = OFF

A countdown of 10 seconds shown on both TM1637 displays before starting.

Then:

Lane A: Starts with GREEN (stateA = 0, timerA = 5)

Lane B: Starts with RED (stateB = 2, timerB = 7)

5. Ultrasonic Object Detection
6. ➤ Each 1 second:
Trigger ultrasonic sensors (TRIG pin pulse).

Wait for ECHO pin to go HIGH, then LOW.

Measure duration of echo pulse using TIM2->CNT.

➤ If object is detected:
If detected for ≥ 7 seconds continuously (DETECTION_TIME_THRESHOLD):

For Lane A:

If Lane B is not green:

Force Lane A to GREEN, Lane B to RED.

For Lane B:

If Lane A is not green:

Force Lane B to GREEN, Lane A to RED.

Only one time extension per detection using ultrasonic_granted_A/B flags.

Resets after GREEN ends.

6.Traffic Light Management
state == 0 → GREEN ON
    → after timerA/B expires → state = 1

state == 1 → YELLOW ON
    → after timer expires → state = 2

state == 2 → RED ON
    → after timer expires → state = 0
    Timers (timerA/B) decrement every second.

Corresponding LED pins controlled.

Time shown on 7-segment TM1637 display for both lanes.


7. Keypad Input – scan_keypad()
Rows are driven LOW one by one.

Columns are checked for LOW to detect key press.

Implements basic debounce delay (50ms).


8.TM1637 Display Handling:
Displays 4-digit countdown of timerA and timerB each second.

Uses:

TM1637_start/stop/write_byte

Segment mappings to digits via digit_to_segment[].


9.Supporting Functions:
delay_us() – Microsecond delay using DWT.

delay_ms() – Millisecond delay using delay_us().

Trigger_Pulse() – 10µs HIGH on TRIG pin.

Object_Detection() / Detect_Object() – Echo time measurement and threshold check.
