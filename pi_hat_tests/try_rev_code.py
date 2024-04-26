def update_steering_angle(angle, reversing=False):
    if reversing:
        angle = 180 - angle  # Adjust steering logic for reversing if required by your setup
    servo_steering.angle = angle
