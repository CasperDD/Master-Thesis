from kinematics import kinematics
import math


class learningWalk:
    kinematic = kinematics()
    minSpeed = 200
    maxSpeed = 255
    speed = 255

    def goStraight(self, tics_to_go):
        tics_l = 0
        tics_r = 0
        pwm_factor = 1  # This value need to be adjusted and tested
        current_encoder = self.kinematic.control.get_encode_values()

        while tics_l <= tics_to_go and tics_r <= tics_to_go:
            temp = self.kinematic.control.get_encode_values()

            if temp[0] != current_encoder[0] or temp[1] != current_encoder[1]:
                tics_l += 1

            if temp[2] != current_encoder[2] or temp[3] != current_encoder[3]:
                tics_r += 1

            current_encoder = temp

            self.kinematic.control.setMotorSpeedDirection(
                self.speed, self.speed, 1, 1, False)

            if tics_r <= tics_to_go and tics_r < tics_l:
                diff_r = (tics_l - tics_r) * pwm_factor

                self.kinematic.control.setRightMotor(self.speed + diff_r, 1)
            else:
                self.kinematic.control.setRightMotor(0, 1)
                self.kinematic.control.setLeftMotor(0, 1)
                tics_l = tics_to_go + 1

            if tics_l <= tics_to_go and tics_l < tics_r:
                diff_l = (tics_r - tics_l) * pwm_factor

                self.kinematic.control.setLeftMotor(self.speed + diff_l, 1)
            else:
                self.kinematic.control.setRightMotor(0, 1)
                self.kinematic.control.setLeftMotor(0, 1)
                tics_r = tics_to_go + 1

    
    def turn(self, theta):
        tics_l = 0
        tics_r = 0
        tics_turn = self.kinematic.setTics180(self.speed)

        tics_to_rotate = (tics_turn / math.pi) * (abs(theta))
        print("Tics to rotate: ", tics_to_rotate)

        current_encoder = self.kinematic.control.get_encode_values()

        while tics_l <= tics_to_rotate and tics_r <= tics_to_rotate:
            temp = self.kinematic.control.get_encode_values()

            if temp[0] != current_encoder[0] or temp[1] != current_encoder[1]:
                tics_l += 1

            if temp[2] != current_encoder[2] or temp[3] != current_encoder[3]:
                tics_r += 1

            current_encoder = temp

            if theta < 0:
                if tics_r <= tics_to_rotate:
                    self.kinematic.control.setRightMotor(self.speed, 1)
                else:
                    self.kinematic.control.setRightMotor(0, 1)

                if tics_l <= tics_to_rotate:
                    self.kinematic.control.setLeftMotor(self.speed, 0)
                else:
                    self.kinematic.control.setLeftMotor(0, 0)
            else:
                if tics_r <= tics_to_rotate:
                    self.kinematic.control.setRightMotor(self.speed, 0)
                else:
                    self.kinematic.control.setRightMotor(0, 0)

                if tics_l <= tics_to_rotate:
                    self.kinematic.control.setLeftMotor(self.speed, 1)
                else:
                    self.kinematic.control.setLeftMotor(0, 1)
