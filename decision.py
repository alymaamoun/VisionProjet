#!/usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function

def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # this offset to make the rover to move closely to a wall to prevent circular movement stuck and increase mapping and to enter       extended roads

    offset = 0
    factor = 0
    if Rover.total_time < 0.5:
        Rover.StartAngle = Rover.nav_angles
        Rover.StartDist = Rover.nav_dists
        Rover.Startpos  =Rover.pos
    if Rover.samples_collected > 4 and Rover.mapped > 93 :
        Rover.done = True
    if Rover.total_time > 10 \
        and Rover.mapped < 85 or Rover.mapped > 92:

        # this function is standart deviation function that takes values of all posssible angles to give the accumlated angle ,              multipling it by number less than 1 make the angle left to the standart deviation

        factor = 0.8
    else:

        # cover the central areas to increase the mapped area

        factor = 0
    offset = factor * np.std(Rover.nav_angles)


    # Example:
    # Check if we have vision data to make decisions with

    if Rover.nav_angles is not None:

        # Check for Rover.mode status

        if Rover.mode == 'forward':
            Rover.throttle = Rover.throttle_set
            Rover.brake = 0
            # first check for existance of sample rock
            if Rover.samples_angles is not None \
            	and any(Rover.samples_dists) \
                and np.min(Rover.samples_dists) < 30:
                # Initiate rock mode
                # initiate rock time , this time is used in stuck analysis
                Rover.rock_time = Rover.total_time
                # save the previous mode
                Rover.prevmode = 'forward'
                # go to rock mode
                Rover.mode = 'rock'
                Rover.brake = Rover.brake_set
                SteeringTime=Rover.total_time
                Rover.throttle=0
                Rover.steer = np.clip(np.mean(Rover.samples_angles * 180 / np.pi), -15, 15)                     
            elif Rover.StartAngle is not None \
                and any(Rover.StartDist)\
                and np.min(Rover.StartDist) < 5\
                and Rover.done==True:                
                #initiate go back to start position
                Rover.brake=Rover.brake_set
                Rover.steer = np.clip(np.mean(Rover.StartAngle
                        * 180 / np.pi), -25, 25)
                Rover.mode = 'end' 
            elif len(Rover.nav_angles) >= Rover.stop_forward:
            # Check the extent of navigable terrain
                if Rover.vel > 0.3:
                    Rover.stuck_time = Rover.total_time
                # check if rover is stucked
                if Rover.vel < 0.2 and Rover.total_time \
                    - Rover.stuck_time > 4:
                    # initiate stuck mode
                    # save current time in stuck mode
                    Rover.stuck_time = Rover.total_time
                    # save current mode in prevoius mode
                    Rover.prevmode = 'forward'
                    # go to stuck mode
                    Rover.mode = 'stuck'
                elif Rover.vel < Rover.max_vel:
                # If mode is forward, navigable terrain looks good
                # and velocity is below max, then throttle
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else:
                      # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean((Rover.nav_angles
                        + offset) * 180 / np.pi), -15, 15)
            elif len(Rover.nav_angles) < Rover.stop_forward:
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
                    # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                    # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.prevmode = 'forward'
                Rover.mode = 'stop'
                
        elif Rover.mode=='end':
        	Rover.brake=0
        	print('end',np.min(Rover.StartDist))
        	if np.min(Rover.StartDist)<3:
        		Rover.brake=Rover.brake_set
        		Rover.steer = 0
        	elif Rover.vel<0.8:
        		Rover.throttle=0.1
        		Rover.steer = np.clip(np.mean(Rover.StartAngle
                        * 180 / np.pi), -25, 25)
        		
        elif Rover.mode == 'stuck':

        # If we're in stuck mode
            # check if 1 second ellapsed

            if Rover.total_time - Rover.stuck_time > 1:

                # initiate returning to previous state
                # set throttle

                Rover.throttle = Rover.throttle_set

                # release brakes

                Rover.brake = 0

                # move heading to the navigational angles with the offset added

                Rover.steer = np.clip(np.mean((Rover.nav_angles
                        + offset) * 180 / np.pi), -15, 15)

                # return to the previous mode

                Rover.mode = Rover.prevmode
            else:
                
                Rover.steer = -15
                
                    
                
                   
                Rover.throttle = 0

        elif Rover.mode == 'rock':

        # if we're in rock mode

            Rover.rockprev = 'forward'

            # check first that the rock is still insight

            if not np.isnan(np.mean(Rover.samples_angles * 180
                            / np.pi)):

                # move heading to the rock
                
                Rover.steer = np.clip(np.mean(Rover.samples_angles
                        * 180 / np.pi), -25, 25)
            else:

                # turn back to the prevoius mode
		
                Rover.mode = 'forward'
	
            # assign a deadline for the task to be completed, if that time is completed and the task isn't then there's a fault and return to the previous state

            if Rover.total_time - Rover.rock_time > 15:
                Rover.mode = 'forward'
            else:

                # stop to pick

                if Rover.near_sample:
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                elif Rover.vel < 0.1 and Rover.total_time \
                    - Rover.stuck_time > 5:

                # check stuck mode
                    # declare stuck

                    Rover.stuck_time = Rover.total_time
                    Rover.throttle = 0
                    Rover.mode = 'stuck'
                    Rover.prevmode = 'rock'
                elif Rover.vel < 0.4:

                # divide by 2 to approach slowly

                    Rover.throttle = Rover.throttle_set
                    Rover.brake = 0
                    Rover.stuck_time = Rover.total_time
                else:
                    Rover.stuck_time = Rover.total_time
                    Rover.throttle = 0
        elif Rover.mode == 'stop':

        # If we're already in "stop" mode then make different decisions
            # If we're in stop mode but still moving keep braking

            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            elif Rover.vel <= 0.2:

            # If we're not moving (vel < 0.2) then do something else
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

                    Rover.steer = np.clip(np.mean(Rover.nav_angles
                            * 180 / np.pi), -15, 15)
                    Rover.mode = 'forward'
    else:

    # Just to make the rover do something
    # even if no modifications have been made to the code

        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    # If in a state where want to pickup a rock send pickup command

    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True

    return Rover

