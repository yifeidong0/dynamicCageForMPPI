import math

def limit_angle_to_pi(angle, theta_min=-math.pi, theta_max=math.pi):
    # Make input angle fall in [-pi, pi]
    if angle > theta_max:
        theta = (angle - theta_max) % (2*math.pi) + theta_min
    elif angle < theta_min:
        theta = theta_max - (theta_min - angle) % (2*math.pi)
    else:
        theta = angle
    return theta