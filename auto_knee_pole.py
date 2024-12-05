# Auto Knee Pole

"""

1. Add bones for knee pole pointing.

2. Add custom properties to the armature object to enable the Auto-Knee-Pole Assistance Panel:

- AKP_L_Active: Boolean
- AKP_L_Auto_Pole_Bone_Name: String
- AKP_L_Pole_Bone_Name: String

- AKP_R_Active: Boolean
- AKP_R_Auto_Pole_Bone_Name: String
- AKP_R_Pole_Bone_Name: String

"""

import bpy
import math
import bl_math
import mathutils


left_active_prop_name = 'AKP_L_Active'
left_auto_pole_bone_name = 'AKP_L_Auto_Pole_Bone_Name'
left_actual_pole_bone_name = 'AKP_L_Pole_Bone_Name'

right_active_prop_name = 'AKP_R_Active'
right_auto_pole_bone_name = 'AKP_R_Auto_Pole_Bone_Name'
right_actual_pole_bone_name = 'AKP_R_Pole_Bone_Name'


x_axis = mathutils.Vector((1, 0, 0))
y_axis = mathutils.Vector((0, 1, 0))
z_axis = mathutils.Vector((0, 0, 1))


def inverseLerp(a, b, v):
    return bl_math.clamp((v - a) / (b - a))

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


def get_result_direction(ik_stem_up, foot_forward, foot_up, foot_right, debug_properties_showing_bone):

    projected_foot_forward = proj_on_plane(foot_forward, ik_stem_up)
    projected_foot_up = proj_on_plane(foot_up, ik_stem_up)

    is_foot_pointing_back = ik_stem_up.dot(foot_up) < 0
    is_foot_pointing_down = ik_stem_up.dot(foot_forward) < 0

    corrected_foot_forward_result = projected_foot_forward.normalized() * (-1 if is_foot_pointing_back else  1)
    corrected_foot_up_result      = projected_foot_up.normalized()      * ( 1 if is_foot_pointing_down else -1)

    right_axis_to_ik_stem_angle = ik_stem_up.angle(foot_right)
    if right_axis_to_ik_stem_angle > math.pi * 0.5:
        right_axis_to_ik_stem_angle = math.pi - right_axis_to_ik_stem_angle

    thresholds_half_gap = right_axis_to_ik_stem_angle * 0.5
    mid_of_thresholds = math.pi * 0.5 + thresholds_half_gap
    threshold_angle_forward_side = mid_of_thresholds - thresholds_half_gap
    threshold_angle_up_side = mid_of_thresholds + thresholds_half_gap
    ik_stem_to_foot_forward_angle = ik_stem_up.angle(foot_forward)


    if is_foot_pointing_down:
        if ik_stem_to_foot_forward_angle < threshold_angle_forward_side:
            return corrected_foot_forward_result
        if ik_stem_to_foot_forward_angle > threshold_angle_up_side:
            return corrected_foot_up_result
        
        mix_progress = inverseLerp(threshold_angle_forward_side, threshold_angle_up_side, ik_stem_to_foot_forward_angle)
        foot_forward_result_to_up_result_ratio = bl_math.smoothstep(0, 1, mix_progress)

        return corrected_foot_forward_result.slerp(corrected_foot_up_result, foot_forward_result_to_up_result_ratio, corrected_foot_up_result)

    if corrected_foot_forward_result.length_squared > 0:
        return corrected_foot_forward_result
    else:
        return corrected_foot_up_result

# Driver function
def get_to_knee_rotation(ik_stem_up_bone, ik_stem_forward_bone, foot_bone):
    
    global x_axis, y_axis, z_axis
    
    ik_stem_up_matrix = mathutils.Matrix(ik_stem_up_bone.matrix)
    ik_stem_forward_matrix = mathutils.Matrix(ik_stem_forward_bone.matrix)
    foot_matrix = mathutils.Matrix(foot_bone.matrix)

    ik_stem_up = ik_stem_up_matrix.to_quaternion() @ y_axis
    ik_stem_forward = ik_stem_forward_matrix.to_quaternion() @ y_axis

    foot_rotation = foot_matrix.to_quaternion()
    foot_forward = foot_rotation @ y_axis
    foot_up = foot_rotation @ z_axis
    foot_right = foot_rotation @ x_axis

    dir = get_result_direction(ik_stem_up, foot_forward, foot_up, foot_right, foot_bone)
    return get_angle_signed_with_axis(ik_stem_forward, dir, ik_stem_up)

bpy.app.driver_namespace['get_to_knee_rotation'] = get_to_knee_rotation


# Custom UI to switch on/off auto knee pole & snap manual position to auto position

## Global variables
actual_pole_bone = None
auto_pole_bone = None

class POSE_OT_pole_snap(bpy.types.Operator):
    bl_idname = "pose.knee_pole_snap_to_auto"
    bl_label = "Snap Pole"


    @classmethod
    def poll(cls, context):
        if context.mode != 'POSE':
            return False
        if context.active_object is None:
            return False
        if context.active_object.type != 'ARMATURE':
            return False
        return True

    def execute(self, context):
        global actual_pole_bone, auto_pole_bone
        
        if (actual_pole_bone is None) or (auto_pole_bone is None):
            return {'CANCELLED'}
        
        actual_pole_bone.matrix.translation = auto_pole_bone.matrix.translation
        context.view_layer.update()
        return {'FINISHED'}


class ControlUI(bpy.types.Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_label = 'Auto Knee Pole'
    bl_idname = 'VIEW3D_PT_AutoKneePole_Control_UI'
    bl_category = 'Item'

    @classmethod
    def poll(self, context):
        if context.mode != 'POSE':
            return False
        try:
            object = context.active_object
            if context.active_pose_bone.name.endswith('.L') and object.data.get(left_active_prop_name):
                return True
            if context.active_pose_bone.name.endswith('.R') and object.data.get(right_active_prop_name):
                return True
        except (AttributeError, KeyError, TypeError):
            return False
    
    def draw(self, context):

        object = context.active_object
        pose_bones = object.pose.bones
        
        is_left_side = context.active_pose_bone.name.endswith('.L')
        side = 'L' if is_left_side else 'R'
        
        global left_actual_pole_bone_name, left_auto_pole_bone_name, right_actual_pole_bone_name, right_auto_pole_bone_name
        actual_pole_bone_name = object.data.get(left_actual_pole_bone_name) if is_left_side else object.data.get(right_actual_pole_bone_name)
        auto_pole_bone_name = object.data.get(left_auto_pole_bone_name) if is_left_side else object.data.get(right_auto_pole_bone_name)

        global actual_pole_bone, auto_pole_bone
        actual_pole_bone = pose_bones[actual_pole_bone_name]
        auto_pole_bone = pose_bones[auto_pole_bone_name]

        # Layout
        layout = self.layout

        layout.prop(pose_bones[actual_pole_bone_name].constraints['Copy Location'], 'influence', text=f'Mix.{side}', slider=True)
        layout.operator('pose.knee_pole_snap_to_auto', text=f'Snap to Auto Pole.{side}')


def register():
    bpy.utils.register_class(ControlUI)
    bpy.utils.register_class(POSE_OT_pole_snap)

def unregister():
    bpy.utils.unregister_class(ControlUI)
    bpy.utils.unregister_class(POSE_OT_pole_snap)
    

if __name__ == '__main__':
    register()