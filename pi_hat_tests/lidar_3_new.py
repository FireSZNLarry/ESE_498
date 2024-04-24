from adafruit_motor import servo

def update_steering_angle(angle):
    # Ensure the steering angle is within the typical servo range
    # Modify the range if your servo specifications differ
    bounded_angle = max(0, min(180, angle))
    servo_steering.angle = bounded_angle

# Later in your main loop, adjust how the steering angle is decided to ensure it doesn't go out of bounds.
# Error calculation should ensure the result is within the physical limits of your hardware.

def main():
    try:
        scan_data = [0]*360
        while True:
            for scan in lidar.iter_scans():
                for (_, angle, distance) in scan:
                    angle = int(angle)
                    if 0 <= angle < 360:
                        scan_data[angle] = distance

                right_distance = scan_data[90] or desired_distance_from_wall
                left_distance = scan_data[270] or desired_distance_from_wall
                right_error = desired_distance_from_wall - right_distance
                left_error = desired_distance_from_wall - left_distance

                average_error = (right_error + left_error) / 2
                if abs(average_error) > distance_tolerance:
                    speed = (max_speed * average_error / desired_distance_from_wall)
                    speed = max(-max_speed, min(max_speed, speed))
                else:
                    speed = 0
                update_motor_speed(speed)

                error_difference = left_error - right_error
                # Calculate steering angle so it stays within [0, 180] after processing
                basic_steering = 90 + (error_difference / distance_tolerance) * turn_sensitivity
                steering_angle = max(0, min(180, basic_steering))
                update_steering_angle(steering_angle)

                lcd.fill((0, 0, 0))
                ...
    except KeyboardInterrupt:
        print('Stopping.')
    finally:
        lidar.stop()
        lidar.disconnect()
        pygame.quit()
