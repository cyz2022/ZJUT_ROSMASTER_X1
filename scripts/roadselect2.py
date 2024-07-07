from Rosmaster_Lib import Rosmaster
import time
import math
import numpy as np

class KalmanFilter:
    def __init__(self, process_variance, measurement_variance):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = 0.0
        self.error_estimate = 1.0

    def update(self, measurement):
        # Prediction Step
        estimate_priori = self.estimate
        error_estimate_priori = self.error_estimate + self.process_variance

        # Update Step
        kalman_gain = error_estimate_priori / (error_estimate_priori + self.measurement_variance)
        self.estimate = estimate_priori + kalman_gain * (measurement - estimate_priori)
        self.error_estimate = (1 - kalman_gain) * error_estimate_priori

        return self.estimate

bot = Rosmaster()

if __name__ == '__main__':
    bot.clear_auto_report_data()
    bot.reset_flash_value()
    time.sleep(0.5)
    bot.create_receive_threading()
    enable = True
    bot.set_auto_report_state(enable, forever=False)

    process_variance = 0.01  # Process noise covariance for Kalman filter
    measurement_variance = 1.0  # Measurement noise covariance for Kalman filter

    kalman_m1 = KalmanFilter(process_variance, measurement_variance)
    kalman_m2 = KalmanFilter(process_variance, measurement_variance)
    kalman_m3 = KalmanFilter(process_variance, measurement_variance)
    kalman_m4 = KalmanFilter(process_variance, measurement_variance)

    flag = 0
    d = 0.104
    l = math.pi * d
    start = time.time()
    a = 70
    b = 70
    c = 70
    d = 70
    old_time = 0
    turns = 0

    while True:
        bot.set_motor(a, b, c, d)

        if turns < 1 and flag == 0: 
            flag = 1

        if flag == 1:
            new_time = time.time()
            delta_t = new_time - old_time
            old_time = new_time
            new_a, new_b, new_c, new_d = bot.get_motor_encoder()
            turns = new_a * 1.0 / 1320
            new_a = new_a * 1.0 / 1320
            new_b = new_b * 1.0 / 1320
            new_c = new_c * 1.0 / 1320
            new_d = new_d * 1.0 / 1320   

            # Apply Kalman filter to motor speeds
            filtered_speed_m1 = kalman_m1.update((new_a - a) / delta_t)
            filtered_speed_m2 = kalman_m2.update((new_b - b) / delta_t)
            filtered_speed_m3 = kalman_m3.update((new_c - c) / delta_t)
            filtered_speed_m4 = kalman_m4.update((new_d - d) / delta_t)

            print("Filtered Speeds:", filtered_speed_m1, filtered_speed_m2, filtered_speed_m3, filtered_speed_m4)

            a = new_a
            b = new_b
            c = new_c
            d = new_d
            distance = turns * l

        if time.time() - start > 5:
            bot.set_car_motion(0, 0, 0)
            break
