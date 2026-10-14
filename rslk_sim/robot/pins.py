"""TI-RSLK MAX wiring.  Edit here if your robot is wired differently.

Entries marked VERIFIED were checked against the TI driver code in
RSLK-MAX-SelfTest.out, the TI-RSLK MAX course drivers (inc/) and the Pololu
"TI-RSLK MAX Chassis Board v1.0" pinout (ti_rslk_max_chassis_board_pinout.pdf).
"""

# Motors (DRV8838 drivers).  PWM pins are TA0.4 (left) / TA0.3 (right).     VERIFIED
LEFT_MOTOR = {"pwm": (2, 7), "dir": (5, 4), "sleep": (3, 7)}
RIGHT_MOTOR = {"pwm": (2, 6), "dir": (5, 5), "sleep": (3, 6)}

# Bump switches, active low with internal pull-ups.  Bump0 = right-most.   VERIFIED (pins)
# (port, bit, centre angle in degrees relative to forward, + = left)
BUMP_SWITCHES = [
    (4, 0, -75.0),   # Bump0
    (4, 2, -45.0),   # Bump1
    (4, 3, -15.0),   # Bump2
    (4, 5, 15.0),    # Bump3
    (4, 6, 45.0),    # Bump4
    (4, 7, 75.0),    # Bump5
]
BUMP_HALF_WIDTH_DEG = 15.0

# QTRX-MD-08A reflectance array on P7.0-P7.7 (bit 0 = right-most sensor).   VERIFIED
REFLECT_PORT = 7
REFLECT_LED_EVEN = (5, 3)   # IR LEDs for sensors 2,4,6,8 (bits 1,3,5,7)      VERIFIED
REFLECT_LED_ODD = (9, 2)    # IR LEDs for sensors 1,3,5,7 (bits 0,2,4,6)      VERIFIED
# lateral sensor positions in metres (+ = left), indexed by bit (from Reflectance_Position weights)
REFLECT_Y = [-0.0334, -0.0238, -0.0142, -0.0048, 0.0048, 0.0142, 0.0238, 0.0334]

# Wheel encoders: channel A on Timer_A3 capture inputs, channel B on P5.     VERIFIED
# (chassis pinout: ELA P10.5, ELB P5.2, ERA P10.4, ERB P5.0)
LEFT_ENCODER = {"a": (10, 5), "b": (5, 2)}
RIGHT_ENCODER = {"a": (10, 4), "b": (5, 0)}
# True: channel B is high at a rising edge of A while the wheel drives forward
ENCODER_B_HIGH_WHEN_FORWARD = {"left": True, "right": True}

# Sharp GP2Y0A21YK0F IR distance sensors -> ADC inputs.                      VERIFIED
# (pin, mount x forward, mount y left, pointing angle deg)
# The course ADC14.c and TI's self-test read the centre sensor on A12/P4.1, while the
# chassis board's DISTC header is P6.1/A14, so the centre voltage appears on both pins
# (IR_EXTRA_PINS).
IR_SENSORS = {
    "left": ((9, 1), 0.040, 0.045, 45.0),     # A16
    "center": ((4, 1), 0.060, 0.000, 0.0),    # A12
    "right": ((9, 0), 0.040, -0.045, -45.0),  # A17
}
IR_EXTRA_PINS = {"center": [(6, 1)]}          # A14

# LaunchPad                                                                  VERIFIED
SW1 = (1, 1)
SW2 = (1, 4)
LED_RED = (1, 0)
LED_RGB = {"red": (2, 0), "green": (2, 1), "blue": (2, 2)}

# RSLK MAX chassis LEDs (per RSLK MAX docs; no course driver uses them)
CHASSIS_LEDS = {"front_left": (8, 0), "front_right": (8, 5), "back_left": (8, 6), "back_right": (8, 7)}

# Nokia 5110 LCD on eUSCI_A3 (SPI: SCLK P9.5, MOSI P9.7, CS P9.4); D/C and RST GPIO   VERIFIED
LCD = {"spi": "EUSCI_A3", "dc": (9, 6), "rst": (9, 3)}
