# Auto IK Knee Pole

# """

# 1. Add bones for knee pole pointing.

# 2. Add drivers on Z rotation of the bones AKP_point_to_knee.L and AKP_point_to_knee.R.

# 3. Add custom properties to the armature object to enable the Auto-Knee-Pole Assistance Panel:

# - AKP_Enable_L_UI: Boolean
# - AKP_Pole_Snap_Target_Name: String
# - AKP_L_Pole_Bone_Name: String

# - AKP_Enable_R_UI: Boolean
# - AKP_R_Auto_Pole_Bone_Name: String
# - AKP_R_Pole_Bone_Name: String

# - AKP_Whitelist_of_Keywords: String

# ---

# Bones Names:
# - AKP-ik_stem_up.L/R
# - AKP-ik_stem_forward.L/R
# - AKP-point_to_knee.L/R
# - AKP-auto_knee_pole.L/R

# """

import bpy
import math
import bl_math
import mathutils


#region - Constants
"""
"PN_" prefix means "Property Name"
"GB_" prefix means "Generated Bone"
"""

# Initiator Custom Properties Names
Init_PN_postfixes_of_sides = 'Postfixes of Sides'
Init_PN_leg_ik_root_bone_name = 'Leg_IK_Root_Bone_Name'
Init_PN_of_leg_ik_target_bone_name = 'Leg_IK_Target_Bone_Name'
Init_PN_of_leg_ik_pole_bone_name = 'Leg_IK_Pole_Bone_Name'

# Custom Properties Names
PN_ui_enable_name = 'AKP_Enable_UI'
PN_keywords_whitelist_name = 'AKP_Whitelist_of_Keywords'

PN_postfixes_of_sides = 'AKP_Postfixes_of_Sides'
PN_pole_snap_target_name = 'AKP_Pole_Snap_Target_Name'
PN_actual_pole_bone_name = 'AKP_Pole_Bone_Name'

Bone_PN_AKP_mix = 'Auto_Knee_Pole_mix'

# Generated Bone Names
GB_stem_mid_base_name = 'AKP_ik_stem_mid'
GB_stem_forward_base_name = 'AKP_ik_stem_forward'
GB_point_to_pole_base_name = 'AKP_point_to_pole'
GB_auto_pole_base_name = 'AKP_auto_pole'
GB_auto_pole_delta_base_name = 'AKP_auto_pole_delta'

# Vector
X_AXIS = mathutils.Vector((1, 0, 0))
Y_AXIS = mathutils.Vector((0, 1, 0))
Z_AXIS = mathutils.Vector((0, 0, 1))
#endregion


#region - Utility Functions
def get_axis(axis_name):
    if axis_name == 'X' or axis_name == 'x':
        return X_AXIS
    if axis_name == 'Y' or axis_name == 'y':
        return Y_AXIS
    if axis_name == 'Z' or axis_name == 'z':
        return Z_AXIS
    if axis_name == '-X' or axis_name == '-x':
        return -X_AXIS
    if axis_name == '-Y' or axis_name == '-y':
        return -Y_AXIS
    if axis_name == '-Z' or axis_name == '-z':
        return -Z_AXIS
    return mathutils.Vector((0, 0, 0))

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
#endregion


#region - Initialization

class POSE_OT_init_initiator_properties(bpy.types.Operator):
    bl_idname = "pose.init_initiator_properties"
    bl_label = "Init Auto Knee Pole Initiator Properties"

    @classmethod
    def poll(cls, context):
        if context.active_object is None:
            return False
        return True

    def execute(self, context):
        obj = bpy.context.active_object

        obj[Init_PN_postfixes_of_sides] = '.L,.R'
        obj[Init_PN_leg_ik_root_bone_name] = ''
        obj[Init_PN_of_leg_ik_target_bone_name] = ''
        obj[Init_PN_of_leg_ik_pole_bone_name] = ''
        return {'FINISHED'}
    

class Init_Initiator_UI(bpy.types.Panel):
    bl_idname = 'VIEW3D_PT_AutoKneePole_Init_Initiator_UI'
    bl_label = 'Auto Knee Pole Initiating'
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = 'object'

    @classmethod
    def poll(self, context):
        if context.active_object is None:
            return False
        return True

    def draw(self, context):
        # Layout
        layout = self.layout
        layout.operator('pose.init_initiator_properties', text=f'Init Auto Knee Pole Initiator Properties')


class POSE_OT_init_for_object(bpy.types.Operator):
    bl_idname = "pose.init_for_object"
    bl_label = "Init Auto Knee Pole for object"
    
    @classmethod
    def poll(self, context):
        obj = context.active_object
        if obj is None:
            return False
        
        target_armatures = self.get_target_armatures(self, context)
        if len(target_armatures) == 0:
            return False
        return True

    def execute(self, context):
        
        initiator = bpy.context.active_object

        context.selected_objects
        target_armatures = self.get_target_armatures(self, context)
        for target_armature in target_armatures:

            postfixes = self.get_postfix_of_sides(self, context, initiator)

            for postfix in postfixes:
                self.init_bones(self, context, initiator, target_armature, postfix)

            self.init_custom_properties(self, context, initiator, target_armature)

        context.view_layer.update()
        return {'FINISHED'}

    def init_bones(self, context, initiator, target, post_fix):
        currentMode = bpy.context.object.mode

        # -- Add bones --
        bpy.ops.object.mode_set(mode='EDIT', toggle=False)

        edit_bones = target.data.edit_bones

        stem_mid_name = GB_stem_mid_base_name + post_fix
        stem_forward_name = GB_stem_forward_base_name + post_fix
        point_to_pole_name = GB_point_to_pole_base_name + post_fix
        auto_pole_name = GB_auto_pole_base_name + post_fix
        auto_pole_delta_name = GB_auto_pole_delta_base_name + post_fix
        stem_bone_length_multiplier = 0.167  # 1/6
        auto_pole_bone_length_multiplier = 0.5
        
        bone_root = edit_bones[initiator[Init_PN_leg_ik_root_bone_name]]
        bone_foot = edit_bones[initiator[Init_PN_of_leg_ik_target_bone_name]]
        bone_pole = edit_bones[initiator[Init_PN_of_leg_ik_pole_bone_name]]
        root_to_tip = bone_foot.head - bone_root.head
        stem_bone_length = root_to_tip.length * stem_bone_length_multiplier
        forward = proj_on_plane(bone_foot.vector, root_to_tip).normalized()

        def add_bone(bone_name, parent):
            bone = edit_bones.new(bone_name)
            bone.use_connect = False
            if parent is not None:
                bone.parent = parent
            return bone

        bone_stem_mid = add_bone(stem_mid_name, bone_root)
        bone_stem_mid.head = bone_root.head
        bone_stem_mid.tail = bone_stem_mid.head - root_to_tip.normalized() * stem_bone_length

        bone_stem_forward = add_bone(stem_forward_name, bone_stem_mid)
        bone_stem_forward.head = bone_stem_mid.head
        bone_stem_forward.tail = bone_stem_mid.head + forward * stem_bone_length

        bone_point_to_pole = add_bone(point_to_pole_name, bone_stem_mid)
        bone_point_to_pole.head = bone_stem_mid.head
        bone_point_to_pole.tail = bone_point_to_pole.head + proj_on_plane(bone_pole.head - bone_point_to_pole.head, root_to_tip).normalized() * stem_bone_length
        bone_point_to_pole.roll = 0

        bone_auto_pole = add_bone(auto_pole_name, bone_point_to_pole)
        bone_auto_pole.head = bone_point_to_pole.tail + proj_on_plane(bone_pole.head - bone_point_to_pole.tail, root_to_tip)
        bone_auto_pole.tail = bone_auto_pole.head + bone_point_to_pole.vector.normalized() * bone_pole.vector.length * auto_pole_bone_length_multiplier

        bone_auto_pole_delta = add_bone(auto_pole_delta_name, bone_auto_pole)

        # -- Add pose constraints --
        bpy.ops.object.mode_set(mode = 'POSE', toggle=False)

        def add_constraint(pose_bone, constraint_type, target_bone_name, head_tail, influence):
            constraint = pose_bone.constraints.new(constraint_type)
            constraint.target = target
            constraint.subtarget = target_bone_name
            constraint.head_tail = head_tail
            constraint.influence = influence

            if (constraint_type == 'COPY_LOCATION'):
                constraint.use_x, constraint.use_y, constraint.use_z = True, True, True
                constraint.target_space, constraint.owner_space = 'POSE', 'POSE'

            return constraint

        pose_bone_stem_mid = target.pose.bones[stem_mid_name]
        
        add_constraint(pose_bone_stem_mid, 'COPY_LOCATION', bone_foot.name, 0, 0.5)
        add_constraint(pose_bone_stem_mid, 'DAMPED_TRACK', bone_foot.name, 0, 1).track_axis = 'TRACK_NEGATIVE_Y'

        pose_bone_pole = target.pose.bones[initiator[Init_PN_of_leg_ik_pole_bone_name]]
        
        add_constraint(pose_bone_pole, 'COPY_LOCATION', bone_auto_pole_delta.name, 0, 1)

        # -- Add driver --
        # for Z rotation of the bone point_to_pole_name
        pose_bone_point_to_pole = target.pose.bones[point_to_pole_name]
        driver = pose_bone_point_to_pole.driver_add('rotation_euler', 2).driver  # Z rotation
        driver.type = 'SCRIPTED'

        def set_up_pose_bone_var(var_name, pose_bone_name):
            var = driver.variables.new()
            var.name = var_name
            var.type = 'SINGLE_PROP'
            var.targets[0].id = target
            var.targets[0].data_path = f'pose.bones["{pose_bone_name}"]'
            return var

        var_ik_up = set_up_pose_bone_var('ik_up', stem_mid_name)
        var_ik_forward = set_up_pose_bone_var('ik_fwd', stem_forward_name)
        var_foot = set_up_pose_bone_var('foot', bone_foot.name)

        driver.expression = f'get_to_knee_rotation({var_ik_up.name}, {var_ik_forward.name}, {var_foot.name})'

        bpy.ops.object.mode_set(mode = currentMode, toggle=False)

    def init_custom_properties(self, context, initiator, target):
        postfixes = self.get_postfix_of_sides(self, context, initiator)

        target.data[PN_postfixes_of_sides] = initiator[Init_PN_postfixes_of_sides]
        target.data[PN_keywords_whitelist_name] = ''

        for postfix in postfixes:
            target.data[PN_ui_enable_name + postfix] = True
            target.data[PN_actual_pole_bone_name + postfix] = self.auto_pole_base_name + postfix
            target.data[PN_pole_snap_target_name + postfix] = self.point_to_pole_base_name + postfix
            target.pose.bones[self.auto_pole_delta_base_name + postfix][Bone_PN_AKP_mix] = 1
            
            driver = target.pose.bones[initiator[Init_PN_of_leg_ik_pole_bone_name]].constraints['Copy Location'].driver_add('influence').driver
            driver.type = 'SCRIPTED'
            var_mix = driver.variables.new()
            var_mix.name = 'mix'
            var_mix.type = 'SINGLE_PROP'
            var_mix.targets[0].id = target
            var_mix.targets[0].data_path = f'pose.bones["{self.auto_pole_delta_base_name + postfix}"]["{Bone_PN_AKP_mix}"]'
            driver.expression = f'{var_mix.name}'


    def get_target_armatures(self, context):
        obj = context.active_object
        return [x for x in context.selected_objects if x != obj and x.type == 'ARMATURE']

    def get_postfix_of_sides(self, context, initiator):
        return initiator[Init_PN_postfixes_of_sides].split(',')


class Init_UI(bpy.types.Panel):
    bl_idname = 'VIEW3D_PT_AutoKneePole_Init_UI'
    bl_label = 'Auto Knee Pole'
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Item'

    @classmethod
    def poll(self, context):
        obj = context.active_object
        if obj is None:
            return False
        if Init_PN_postfixes_of_sides not in obj:
            return False
        if Init_PN_leg_ik_root_bone_name not in obj:
            return False
        if Init_PN_of_leg_ik_target_bone_name not in obj:
            return False
        if Init_PN_of_leg_ik_pole_bone_name not in obj:
            return False
        return True
    
    def draw(self, context):
        obj = context.active_object

        def check_if_can_init():
            target_objs = [x for x in context.selected_objects if x != obj]
            for target_obj in target_objs:
                if target_obj.type != 'ARMATURE':
                    return 'not an armature'
                pose_bones = target_obj.pose.bones
                if pose_bones.get(obj[Init_PN_leg_ik_root_bone_name]) is None:
                    return f'missing bone "{obj[Init_PN_leg_ik_root_bone_name]}"'
                if pose_bones.get(obj[Init_PN_of_leg_ik_target_bone_name]) is None:
                    return f'missing bone "{obj[Init_PN_of_leg_ik_target_bone_name]}"'
                if pose_bones.get(obj[Init_PN_of_leg_ik_pole_bone_name]) is None:
                    return f'missing bone "{obj[Init_PN_of_leg_ik_pole_bone_name]}"'
            return True
        
        # Layout
        layout = self.layout

        can_init_check_result = check_if_can_init()
        if (check_if_can_init == True):
            layout.operator('pose.init_for_object', text=f'Init Auto Knee Pole')
        else:
            layout.label(text=f'Cannot Init: One or more targets are {can_init_check_result}.')

#endregion


#region - Driver Calculation Functions
def get_result_direction(ik_stem_up, foot_forward, foot_up, foot_right):

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
def get_to_knee_rotation(ik_stem_up_bone, ik_stem_forward_bone, foot_bone, foot_bone_up_axis_name = 'Z', foot_bone_forward_axis_name = 'Y'):
    
    ik_stem_up_matrix = mathutils.Matrix(ik_stem_up_bone.matrix)
    ik_stem_forward_matrix = mathutils.Matrix(ik_stem_forward_bone.matrix)
    foot_matrix = mathutils.Matrix(foot_bone.matrix)

    ik_stem_up = ik_stem_up_matrix.to_quaternion() @ Y_AXIS
    ik_stem_forward = ik_stem_forward_matrix.to_quaternion() @ Y_AXIS

    foot_bone_local_up = get_axis(foot_bone_up_axis_name)
    foot_bone_local_forward = get_axis(foot_bone_forward_axis_name)
    foot_bone_local_right = foot_bone_local_forward.cross(foot_bone_local_up)

    foot_rotation = foot_matrix.to_quaternion()
    foot_up = foot_rotation @ foot_bone_local_up
    foot_forward = foot_rotation @ foot_bone_local_forward
    foot_right = foot_rotation @ foot_bone_local_right

    dir = get_result_direction(ik_stem_up, foot_forward, foot_up, foot_right)
    return get_angle_signed_with_axis(ik_stem_forward, dir, ik_stem_up)

bpy.app.driver_namespace['get_to_knee_rotation'] = get_to_knee_rotation
#endregion


#region - Custom UI for Auto Knee Pole controlling
"""
- Switch on/off auto knee pole
- Snap manual position to auto position
"""
class POSE_OT_pole_snap(bpy.types.Operator):
    bl_idname = "pose.knee_pole_snap_to_auto"
    bl_label = "Snap Pole"

    pole_bone = None
    pole_snap_target_bone = None

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
        if (self.pole_bone is None) or (self.pole_snap_target_bone is None):
            return {'CANCELLED'}
        
        self.pole_bone.matrix.translation = self.pole_snap_target_bone.matrix.translation
        context.view_layer.update()
        return {'FINISHED'}


class Control_UI(bpy.types.Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_label = 'Auto Knee Pole'
    bl_idname = 'VIEW3D_PT_AutoKneePole_Control_UI'
    bl_category = 'Item'

    @classmethod
    def poll(self, context):
        if context.mode != 'POSE':
            return False
        
        object_data = context.active_object.data
        active_pose_bone = context.active_pose_bone

        if PN_keywords_whitelist_name in object_data:
            raw_whitelist = object_data.get(PN_keywords_whitelist_name)
            if raw_whitelist is None or raw_whitelist == '':
                return False
            whitelist = raw_whitelist.split(',')
            if active_pose_bone is None:
                return False
            if not any(substring in active_pose_bone.name for substring in whitelist):
                return False

        try:
            postfix_of_side = self.get_postfix_of_sides(self, context, object_data)

            if not context.active_pose_bone.name.endswith(postfix_of_side):
                return False
            if not PN_ui_enable_name + postfix_of_side in object_data:
                return False
            if not object_data.get(PN_ui_enable_name + postfix_of_side):
                return False
            if not PN_actual_pole_bone_name + postfix_of_side in object_data:
                return False
            if not PN_pole_snap_target_name + postfix_of_side in object_data:
                return False
            return True
        except (AttributeError, KeyError, TypeError):
            return False
    
    def draw(self, context):

        obj = context.active_object
        pose_bones = obj.pose.bones
        
        postfix_of_side = self.get_postfix_of_sides(self, context, obj.data)

        pole_bone = pose_bones[obj.data.get(PN_actual_pole_bone_name) + postfix_of_side]
        pole_snap_target_bone = pose_bones[obj.data.get(PN_pole_snap_target_name) + postfix_of_side]

        # Set the necessary properties for the operator
        POSE_OT_pole_snap.pole_bone = pole_bone
        POSE_OT_pole_snap.pole_snap_target_bone = pole_snap_target_bone

        # Layout
        layout = self.layout

        layout.prop(pole_snap_target_bone, f'["{Bone_PN_AKP_mix}"]', text=f'Mix{postfix_of_side}', slider=True)
        layout.operator('pose.knee_pole_snap_to_auto', text=f'Snap to Auto Pole{postfix_of_side}')


    def get_postfix_of_sides(self, context, custom_property_carrier):
        
        postfixes_of_sides = custom_property_carrier.get(PN_postfixes_of_sides).split(',')
        matched = [context.active_pose_bone.endswith(postfix) for postfix in postfixes_of_sides]
        
        if len(matched) == 0:
            return ''
        return matched[0]
    
    
#endregion


#region - Blender Registration
def register():
    bpy.utils.register_class(POSE_OT_init_initiator_properties)
    bpy.utils.register_class(POSE_OT_init_for_object)
    bpy.utils.register_class(Init_Initiator_UI)
    bpy.utils.register_class(Init_UI)
    bpy.utils.register_class(Control_UI)
    bpy.utils.register_class(POSE_OT_pole_snap)

def unregister():
    bpy.utils.unregister_class(POSE_OT_init_initiator_properties)
    bpy.utils.unregister_class(POSE_OT_init_for_object)
    bpy.utils.unregister_class(Init_Initiator_UI)
    bpy.utils.unregister_class(Init_UI)
    bpy.utils.unregister_class(Control_UI)
    bpy.utils.unregister_class(POSE_OT_pole_snap)


if __name__ == '__main__':
    register()
# endregion