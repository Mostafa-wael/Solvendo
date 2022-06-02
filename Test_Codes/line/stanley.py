 ######################################################
    ######################################################
    # IMPLEMENTATION OF LATERAL CONTROLLER HERE
    ######################################################
    ######################################################
    # stanley parameters
    ke = 0.3
    kv = 10

    # calculate heading error
    diff_x = waypoints[-1][0] - waypoints[0][0]
    diff_y = waypoints[-1][1] - waypoints[0][1]
    yaw_path = np.arctan2 (diff_y, diff_x) # the current optimum yaw of the car

    psi = yaw_path - yaw # heading error 
    # psi = psi - 2*np.pi if psi > np.pi else psi + 2*np.pi # preserve sign



    current_xy = np.array([x, y]) # add the current x and y in an array
    waypoints_array = np.array(waypoints)
    crosstrack_error = np.min(np.sum ((current_xy - waypoints_array[:, :2])**2, axis=1)) # calc the least square error
    yaw_cross_track = np.arctan2(y - waypoints[0][1], x - waypoints[0][0]) # crosstrack_error should have the same sign as this

    yaw_modify = yaw_path -yaw_cross_track
    # yaw_modify = yaw_modify - 2*np.pi if yaw_modify > np.pi else yaw_modify + 2*np.pi # preserve sign


    # crosstrack_error = abs(crosstrack_error) if yaw_modify > 0 else -abs(crosstrack_error) # preserve sign

    psi_crosstrack = np.arctan(ke * crosstrack_error / (kv + v))


    sigma = psi + psi_crosstrack
    # Change the steer output with the lateral controller. 
    steer_output    = sigma - 2*np.pi if sigma > np.pi else  sigma + 2*np.pi 

    print(crosstrack_error,psi,psi_crosstrack)
