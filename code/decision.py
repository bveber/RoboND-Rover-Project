import numpy as np


def get_steer_angle(Rover):
    """
    Get the next desired steering angle by finding the weighted average of angles. Where the angles are weighted by the
    pixel distances. Favoring closer pixels.
    :param Rover: Rover class containing state information
    :return: steering angle in degrees
    """
    if Rover.nav_dists is not None and Rover.nav_dists.sum() > 0:
        return np.clip(np.average(Rover.nav_angles * 180/np.pi, weights=1/np.sqrt(Rover.nav_dists)), -15, 15)
    else:
        return 0


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
    """
    Decide the next action to make
    :param Rover: Rover class containing state information
    :return: Updated Rover object
    """
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check if a rock is in view
            if Rover.vision_image[:, :, 1].any():
                print('Rock found!')
                Rover.mode = 'collect'
                Rover.time_collecting_start = Rover.total_time
            # Check if the Rover is stuck on an obstacle
            # Stuck is defined as the Rover having a velocity < .01m/s after more than 2 seconds of being in the
            # 'forward' state
            elif Rover.total_time - Rover.time_forward_start > 4.0 and np.abs(Rover.vel) < 0.01:
                Rover.mode = 'stuck'
                Rover.time_stuck_start = Rover.total_time
            # Check if the current view is out or navigable terrain
            elif len(Rover.nav_angles) >= Rover.stop_forward:
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else:  # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering angle
                Rover.steer = get_steer_angle(Rover)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15  # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = get_steer_angle(Rover)
                    Rover.mode = 'forward'
                    Rover.time_forward_start = Rover.total_time
                else:
                    pass
        # If we're in stuck mode perform actions to get unstuck
        elif Rover.mode == 'stuck':
            #  Reverse the steering angle when starting to back up
            if np.abs(Rover.vel) < .05:
                Rover.steer = -get_steer_angle(Rover)
            #  Back up the Rover
            Rover.throttle = -Rover.throttle_set * 1.5
            #  Back up for 2 seconds then try again
            if Rover.total_time - Rover.time_stuck_start > 4:
                Rover.mode = 'stop'
                Rover.steer = 0
        #  If we're in collect mode perform actions to navigate towards rock and collect it
        elif Rover.mode == 'collect':
            print('Collecting!')
            # Slow down
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            # Start moving slowly towards the rock
            Rover.throttle = Rover.throttle_set / 2
            Rover.steer = np.clip(np.mean(Rover.rock_angle * 180/np.pi), -15, 15)
            Rover.brake = 0
            #  Check if the Rover is stuck on anything
            if np.abs(Rover.vel) < 0.01 and (Rover.total_time - Rover.time_collecting_start) > 4:
                Rover.mode = 'stuck'
                Rover.time_stuck_start = Rover.total_time
            # If we pick up or lose the rock change to 'stop' mode
            if not Rover.vision_image[:, :, 1].any() and (Rover.total_time - Rover.time_collecting_start) > 4:
                Rover.mode = 'stop'
    # Just to make the rover do something in the empty initial state
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and not Rover.picking_up:
        Rover.throttle = 0
        Rover.brake = Rover.brake_set
        Rover.send_pickup = True
        Rover.brake = 0
    
    return Rover
