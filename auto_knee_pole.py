import bpy
import mathutils


def get_direction(seq):
    return mathutils.Vector(seq).normalized()

def proj_on_plane(v, n):
    return v - v.dot(n) * n

def get_angle_signed_with_axis(from_vec, to_vec, axis):
    dot_result = from_vec.cross(to_vec).dot(axis)
    dir = 0
    if dot_result < 0:
        dir = -1
    elif dot_result > 0:
        dir = 1

    return from_vec.angle(to_vec) * dir

def flip_by_plane(v, n):
    return v - 2 * v.dot(n) * n

def get_result_directions(ik_stem_up, foot_forward, foot_up): # results of foot_forward and foot_up

    projected_foot_forward = proj_on_plane(foot_forward, ik_stem_up)
    projected_foot_up = proj_on_plane(foot_up, ik_stem_up)

    is_foot_pointing_back = ik_stem_up.dot(foot_up) < 0
    if is_foot_pointing_back:
        result_by_foot_forward = -projected_foot_forward.normalized()
    else:
        result_by_foot_forward = projected_foot_forward.normalized()

    is_foot_pointing_up = ik_stem_up.dot(foot_forward) > 0
    if is_foot_pointing_up:
        if projected_foot_forward.length_squared == 0:
            result_by_foot_up = -projected_foot_up.normalized()
        else:
            result_by_foot_up = flip_by_plane(projected_foot_up.normalized(), projected_foot_forward.normalized())
    else:
        result_by_foot_up = projected_foot_up.normalized()

    return (result_by_foot_forward, result_by_foot_up)

def get_result_rotation_direction_of_foot_forward(ik_stem_forward_input, ik_stem_up_input, foot_forward_input, foot_up_input):
    ik_stem_forward = get_direction(ik_stem_forward_input)
    ik_stem_up = get_direction(ik_stem_up_input)
    foot_forward = get_direction(foot_forward_input)
    foot_up = get_direction(foot_up_input)
    dir = get_result_directions(ik_stem_up, foot_forward, foot_up)[0]
    return get_angle_signed_with_axis(ik_stem_forward, dir, ik_stem_up)

def get_result_rotation_direction_of_foot_up(ik_stem_forward_input, ik_stem_up_input, foot_forward_input, foot_up_input):
    ik_stem_forward = get_direction(ik_stem_forward_input)
    ik_stem_up = get_direction(ik_stem_up_input)
    foot_forward = get_direction(foot_forward_input)
    foot_up = get_direction(foot_up_input)
    dir = get_result_directions(ik_stem_up, foot_forward, foot_up)[1]
    return get_angle_signed_with_axis(ik_stem_forward, dir, ik_stem_up)


######################################################
def get_ik_pole_direction(ik_stem_up, foot_forward, foot_up):

    projected_foot_forward = proj_on_plane(foot_forward, ik_stem_up)
    projected_foot_up = proj_on_plane(foot_up, ik_stem_up)

    is_foot_pointing_back = ik_stem_up.dot(foot_up) < 0
    result_by_foot_forward = projected_foot_forward.normalized() * (-1 if is_foot_pointing_back else 1)

    is_foot_pointing_up = ik_stem_up.dot(foot_forward) > 0
    result_by_foot_up = projected_foot_up.normalized() * (-1 if is_foot_pointing_up else 1)
                                                   
    len_factor = projected_foot_forward.length_squared
    threshold = 0.05
    lower_threshold = 0.01

    if len_factor < threshold:
        t = (threshold - max(0, len_factor - lower_threshold)) / threshold
        return result_by_foot_forward.slerp(result_by_foot_up, t)
    else:
        return result_by_foot_forward
    
def get_rotation_to_ik_pole_direction_along_ik_stem_up(ik_stem_forward_input, ik_stem_up_input, foot_forward_input, foot_up_input):

    ik_stem_forward = get_direction(ik_stem_forward_input)
    ik_stem_up = get_direction(ik_stem_up_input)
    foot_forward = get_direction(foot_forward_input)
    foot_up = get_direction(foot_up_input)

    ik_pole_dir = get_ik_pole_direction(ik_stem_up, foot_forward, foot_up)

    return get_angle_signed_with_axis(ik_stem_forward, ik_pole_dir, ik_stem_up)


bpy.app.driver_namespace['get_rot_to_ik_pole'] = get_rotation_to_ik_pole_direction_along_ik_stem_up
bpy.app.driver_namespace['get_result_rot_of_forward'] = get_result_rotation_direction_of_foot_forward
bpy.app.driver_namespace['get_result_rot_of_up'] = get_result_rotation_direction_of_foot_up