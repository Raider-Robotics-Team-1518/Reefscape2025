# Crescendo2024
Code for the 2024 Crescendo game

# Dependencies
Relying on Falon motors with TalonFX controllers
Swerve Drive is based on YAGSL 2024
    CANBus IDs imported from 2023
    Conversion constants from 2023
    Module positions input from 2023

Phoenix6 Library
NavX Library
PathPlanner Library upgraded to 2024

# Field Actions to Plan 1/17/2024
    1. Pick up note with intake system
        a. Use color sensor to stop intake when loaded
        b. Only allow eject or index when note is loaded
    2. Lower arm to pick up note
        a. Use pass-thru encoder on hex shaft - move to low limit
        b. Only allow intake while at low position
        c. Need to determin min & max positions
        d. Need to set encoder to absolute mode
    3. Read april tags for field position
    4. Use april tag field position to determine aim point
        a. Speaker
            1. Move arm to angle
            2. Align robot left/right to target
            3. Spin up thrower to set speed
            4. Index note into thrower
        b. Amp
            1. Move arm to angle
            2. Align robot left/right to target
            3. Spin up thrower to set speed
            4. Index note into thrower
    4. Manual Aim
        a. Move arm up/down to aim?
    5. Engage climb at chain
        a. Raise hook
        b. Move robot to line up
        c. Lower hook to lift robot
