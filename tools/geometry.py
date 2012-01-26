#!/usr/bin/env python

import numpy as np
from math import pi, sqrt, sin, cos, acos, atan2

def rotation_to_sample(roll, pitch):
    '''generate a rotation matrix to transform into the frame of the sample
    given the pitch and roll angle'''
    r = np.array([[           cos(roll),           -sin(roll),           0],
                  [cos(pitch)*sin(roll), cos(pitch)*cos(roll), -sin(pitch)],
                  [sin(pitch)*sin(roll), sin(pitch)*cos(roll), cos(pitch)]])
    return r

def cartesian_to_spherical(v):
    '''convert a vector in cartesian coordinates (x, y, z) to spherical
    coordinates (r, theta, phi)'''
    x, y, z = v
    r = sqrt(x*x + y*y + z*z)
    theta = acos(z/r)
    phi = atan2(y, x)
    return np.array([r, theta, phi])

def spherical_to_cartesian(v):
    '''convert a vector in spherical coordinates (r, theta, phi) to cartesian
    coordinates (x, y, z)'''
    r, theta, phi = v
    x = r * sin(theta) * cos(phi)
    y = r * sin(theta) * sin(phi)
    z = r * cos(theta)
    return np.array([x, y, z])

def calculate_brdf_angles(arm_length_source, arm_length_sensor, source_angle,
                          sensor_angle, sample_pitch, sample_roll):
    '''find incoming and outgoing ray angles (theta_r, phi_r, theta_i, phi_i)
    given the experiment geometry'''
    source_pos = np.array([sin(source_angle),  0.0, cos(source_angle)]) * arm_length_source
    sensor_pos = np.array([-sin(sensor_angle), 0.0, cos(sensor_angle)]) * arm_length_sensor

    r = rotation_to_sample(sample_roll, sample_pitch)

    vi = np.dot(r, source_pos)
    vr = np.dot(r, sensor_pos)

    ai = cartesian_to_spherical(vi)
    ar = cartesian_to_spherical(vr)

    return (ar[1], ar[2], ai[1], ai[2])

if __name__ == '__main__':
    arm_length_source = arm_length_sensor = 1.0
    source_angle = 0.0
    sensor_angle = 0.0
    sample_pitch = 0.0
    sample_roll  = 0.0

    angles = calculate_brdf_angles(arm_length_source, arm_length_sensor,
                                   source_angle, sensor_angle,
                                   sample_pitch, sample_roll)

    print '(tr, pr, ti, pi) =', angles


