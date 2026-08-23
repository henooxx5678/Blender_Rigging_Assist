# Auto IK Knee Pole (AKP) //TODO: rename to AIKKP?

"""

Steps to implement Auto IK Knee Pole:

1. Select any object as Initiator.
2. Press "Initialize Auto Knee Pole Initiator Properties" button to initialize the custom properties.
3. Select the armatures that need Auto IK Knee Pole, then selecting the Initiator object last to make it the active object.
4. Press "Init Auto IK Knee Pole for Selected Armature(s)" button to initialize the Auto IK Knee Pole for the selected armatures.

---
What the initiating process does (WIP):

1. Add bones for knee pole pointing.

2. Add drivers on Z rotation of the bones AKP_point_to_knee.L and AKP_point_to_knee.R.

3. Add custom properties to the armature object to enable the Auto-Knee-Pole Assistance Panel:

- AKP_Enable_L_UI: Boolean
- AKP_Pole_Snap_Target_Name: String
- AKP_L_Pole_Bone_Name: String

- AKP_Enable_R_UI: Boolean
- AKP_R_Auto_Pole_Bone_Name: String
- AKP_R_Pole_Bone_Name: String

- AKP_Whitelist_of_Keywords: String

---

//TODO: deprecated, to be removed, replaced by initiator
AKP Bones Names:
- AKP-ik_stem_up.L/R            # represents the direction from the IK tip (foot) to the IK root (hip), located at the center IK stem
- AKP-ik_stem_forward.L/R       # represents the forward direction of the IK stem 
- AKP-point_to_knee.L/R         # the calculated direction point to knee pole (which has drivers on its Z rotation)
- AKP-auto_knee_pole.L/R        # result pole position to be followed. can add delta position on it for manual adjustment  //TODO change name (add "delta" in the name)

"""

import traceback

import bpy
import math
import bl_math
import mathutils


#region - Constants
"""
"PN_" prefix means "Property Name"
"Pbn_" prefix means "Property base name" # "base name" means the name should be added the postfix of side (e.g. ".L" or ".R") to get the actual name of the property or bone.
"GB_" prefix means "Generated Bone"

"Initiator" means the object that has the initiator properties, which is used to initialize the Auto IK Knee Pole function to target armature(s).

"""

# Initiator Custom Properties Names
Init_PN_postfixes_of_sides = 'Postfixes of Sides'
Init_PN_leg_ik_root_bone_basename = 'Leg_IK_Root_Bone_Basename'
Init_PN_of_leg_ik_target_bone_basename = 'Leg_IK_Target_Bone_Basename'
Init_PN_of_leg_ik_pole_bone_basename = 'Leg_IK_Pole_Bone_Basename'

# Custom Properties Names
PN_postfixes_of_sides = 'AKP_Postfixes_of_Sides'            # e.g. ".L,.R" for 2 legs
PN_keywords_whitelist = 'AKP_Whitelist_of_Keywords'    # this is used to filter the pose bones that can show the Auto Knee Pole UI. e.g. "tip,ik,pole"

Pbn_ui_enable = 'AKP_Enable_UI'

Pbn_pole_snap_target_name = 'AKP_Pole_Snap_Target_Name'
Pbn_actual_pole_bone_name = 'AKP_Pole_Bone_Name'

Bone_PN_AKP_mix = 'Auto_Knee_Pole_mix'

# Generated Bone Names
GB_stem_mid_basename = 'AKP_ik_stem_mid'               # As the root of all generated bones. Represents the direction from the IK tip (foot) to the IK root (hip), located at the center of the IK stem.
GB_stem_forward_basename = 'AKP_ik_stem_forward'       # Represents the forward direction of the IK stem. Just for view check, not used in calculation.
GB_point_to_pole_basename = 'AKP_point_to_pole'        # Calculated direction point to knee pole (which has drivers on its Z rotation)
GB_auto_pole_basename = 'AKP_auto_pole'                # Result pole position to be followed. can add delta position on it for manual adjustment  //TODO change name (add "delta" in the name)
# GB_auto_pole_delta_base_name = 'AKP_auto_pole_delta'  # //TODO may not needed

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
## TO Make "Initialize Auto Knee Pole Initiator Properties" as command instead of a button
# class Init_Initiator_UI(bpy.types.Panel):
#     bl_idname = 'VIEW3D_PT_AutoKneePole_Init_Initiator_UI'
#     bl_label = 'Auto Knee Pole Initiating'
#     bl_space_type = 'VIEW_3D'
#     bl_region_type = 'UI'
#     bl_category = 'Item'

#     @classmethod
#     def poll(self, context):
#         obj = context.active_object
#         if obj is None:
#             return False
#         if self.has_initialized_properties(obj):
#             return False
#         return True

#     def draw(self, context):
#         # Layout
#         layout = self.layout
#         layout.operator('pose.init_initiator_properties', text=f'Initialize Auto Knee Pole Initiator Properties')

#     @staticmethod
#     def has_initialized_properties(obj):
#         return (Init_PN_postfixes_of_sides in obj and
#                 Init_PN_leg_ik_root_bone_basename in obj and
#                 Init_PN_of_leg_ik_target_bone_basename in obj and
#                 Init_PN_of_leg_ik_pole_bone_basename in obj)

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
        if Init_PN_leg_ik_root_bone_basename not in obj:
            return False
        if Init_PN_of_leg_ik_target_bone_basename not in obj:
            return False
        if Init_PN_of_leg_ik_pole_bone_basename not in obj:
            return False
        return True
    
    def draw(self, context):
        obj = context.active_object

        def check_if_can_init():
            target_objs = [x for x in context.selected_objects if x != obj]
            for target_obj in target_objs:
                if target_obj.type != 'ARMATURE':
                    return 'not an armature'
                
                postfixes_of_sides = obj.get(Init_PN_postfixes_of_sides).split(',')
                required_basenames = [
                    obj[Init_PN_leg_ik_root_bone_basename],
                    obj[Init_PN_of_leg_ik_target_bone_basename],
                    obj[Init_PN_of_leg_ik_pole_bone_basename]
                ]

                pose_bones = target_obj.pose.bones
                for basename in required_basenames:
                    for postfix in postfixes_of_sides:
                        required_bone_name = basename + postfix
                        if pose_bones.get(required_bone_name) is None:
                            return f'missing bone "{required_bone_name}"'
            return True
        
        # Layout
        layout = self.layout

        can_init_check_result = check_if_can_init()
        if can_init_check_result is True:
            # Generate the Auto IK Knee Pole bones and drivers for the selected armature(s) excluding the initiator object itself.
            layout.operator('pose.init_for_object', text=f'Init Auto IK Knee Pole for Selected Armature(s)')
        else:
            layout.label(text=f'Cannot Init: One or more targets are {can_init_check_result}.')


class POSE_OT_init_initiator_properties(bpy.types.Operator):
    bl_idname = "pose.init_initiator_properties"
    bl_label = "Init Auto IK Knee Pole Initiator Properties"
    bl_description = "Initialize custom properties for Auto Knee Pole Initiator"
    bl_options = {'REGISTER', 'UNDO'}


    @classmethod
    def poll(cls, context):
        return context.active_object is not None

    def execute(self, context):
        obj = context.active_object

        if self.has_initialized_properties(obj):
            self.report({'WARNING'}, 'Object already has the properties.')
            return {'CANCELLED'}

        obj[Init_PN_postfixes_of_sides] = '.L,.R'
        obj[Init_PN_leg_ik_root_bone_basename] = ''
        obj[Init_PN_of_leg_ik_target_bone_basename] = ''
        obj[Init_PN_of_leg_ik_pole_bone_basename] = ''
        return {'FINISHED'}
    
    @staticmethod
    def has_initialized_properties(obj):
        return (Init_PN_postfixes_of_sides in obj and
                Init_PN_leg_ik_root_bone_basename in obj and
                Init_PN_of_leg_ik_target_bone_basename in obj and
                Init_PN_of_leg_ik_pole_bone_basename in obj)

class POSE_OT_init_for_object(bpy.types.Operator):
    bl_idname = "pose.init_for_object"
    bl_label = "Init Auto Knee Pole for object"
    
    @classmethod
    def poll(self, context):
        obj = context.active_object
        if obj is None:
            return False
        
        target_armatures = self.get_target_armatures(context)
        if len(target_armatures) == 0:
            return False
        return True

    def execute(self, context):
        
        initiator = bpy.context.active_object

        context.selected_objects
        target_armatures = self.get_target_armatures(context)
        for target_armature in target_armatures:

            postfixes = self.get_postfix_of_sides(initiator)

            for postfix in postfixes:
                self.init_bones(context, initiator, target_armature, postfix)

            self.init_custom_properties(context, initiator, target_armature)

        context.view_layer.objects.active = initiator  # Set the initiator back to active object after initialization.
        context.view_layer.update()
        return {'FINISHED'}

    def init_bones(self, context, initiator, target, post_fix):
        context.view_layer.objects.active = target  # Set the target armature as the active object to ensure that the bone creation and constraint addition are applied to the correct armature.

        currentMode = bpy.context.object.mode

        # -- Add bones --
        bpy.ops.object.mode_set(mode = 'EDIT')

        edit_bones = target.data.edit_bones

        stem_mid_name = GB_stem_mid_basename + post_fix
        stem_forward_name = GB_stem_forward_basename + post_fix
        point_to_pole_name = GB_point_to_pole_basename + post_fix
        auto_pole_name = GB_auto_pole_basename + post_fix
        stem_bone_length_multiplier = 0.167  # 1/6
        auto_pole_bone_length_multiplier = 0.5
        
        # Initialize bone names
        root_bone_name = initiator[Init_PN_leg_ik_root_bone_basename] + post_fix
        tip_bone_name = initiator[Init_PN_of_leg_ik_target_bone_basename] + post_fix
        pole_bone_name = initiator[Init_PN_of_leg_ik_pole_bone_basename] + post_fix

        bone_root = edit_bones[root_bone_name]
        bone_foot = edit_bones[tip_bone_name]
        bone_pole = edit_bones[pole_bone_name]

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


        # -- Add pose constraints --
        bpy.ops.object.mode_set(mode = 'POSE')

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
        
        # Make pose_bone_stem_mid located at the middle of the IK stem (between root and tip)
        add_constraint(pose_bone_stem_mid, 'COPY_LOCATION', bone_foot.name, 0, 0.5)
        # Make pose_bone_stem_mid point up alone the IK stem
        add_constraint(pose_bone_stem_mid, 'DAMPED_TRACK', bone_foot.name, 0, 1).track_axis = 'TRACK_NEGATIVE_Y'

        pose_bone_pole = target.pose.bones[pole_bone_name]
        
        add_constraint(pose_bone_pole, 'COPY_LOCATION', bone_auto_pole.name, 0, 1)

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

        bpy.ops.object.mode_set(mode = currentMode)  # Restore the original mode after bone creation and constraint addition.

    def init_custom_properties(self, context, initiator, target):
        target.data[PN_postfixes_of_sides] = initiator[Init_PN_postfixes_of_sides]
        target.data[PN_keywords_whitelist] = ''

        postfixes = self.get_postfix_of_sides(initiator)

        for postfix in postfixes:
            # Bone names
            pole_bone_name = initiator[Init_PN_of_leg_ik_pole_bone_basename] + postfix
            auto_pole_bone_name = GB_auto_pole_basename + postfix
            pole_snap_target_bone_name = auto_pole_bone_name  # Set the snap target as the auto pole bone, which is the result of the Auto IK Knee Pole calculation.

            # Custom property names
            UI_enable_property_name = Pbn_ui_enable + postfix
            actual_pole_bone_name_property_name = Pbn_actual_pole_bone_name + postfix
            pole_snap_target_name_property_name = Pbn_pole_snap_target_name + postfix

            target.data[UI_enable_property_name] = True
            target.data[actual_pole_bone_name_property_name] = pole_bone_name
            target.data[pole_snap_target_name_property_name] = pole_snap_target_bone_name
            
            # Add custom property "Mix" with [0, 1] range in float
            pose_bone_pole_snap_target = target.pose.bones[pole_snap_target_bone_name]
            pose_bone_pole_snap_target[Bone_PN_AKP_mix] = 1.0
            pose_bone_pole_snap_target.id_properties_ui(Bone_PN_AKP_mix).update(
                min=0.0,
                max=1.0,
                soft_min=0.0,
                soft_max=1.0,
                description="Mix between the AIKKP position and the original pole position. 0 means fully original pole position, 1 means fully AIKKP position."
            )
                        
            # Setup driver for the influence of the Copy Location constraint on the pole bone
            driver = target.pose.bones[pole_bone_name].constraints['Copy Location'].driver_add('influence').driver
            driver.type = 'SCRIPTED'
            var_mix = driver.variables.new()
            var_mix.name = 'mix'
            var_mix.type = 'SINGLE_PROP'
            var_mix.targets[0].id = target
            var_mix.targets[0].data_path = f'pose.bones["{pole_snap_target_bone_name}"]["{Bone_PN_AKP_mix}"]'
            driver.expression = f'{var_mix.name}'

    @staticmethod
    def get_target_armatures(context):
        obj = context.active_object
        return [x for x in context.selected_objects if x != obj and x.type == 'ARMATURE']

    @staticmethod
    def get_postfix_of_sides(initiator):
        return initiator[Init_PN_postfixes_of_sides].split(',')

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
        
        obj_data = context.active_object.data
        active_pose_bone = context.active_pose_bone

        if PN_keywords_whitelist in obj_data:
            raw_whitelist = obj_data.get(PN_keywords_whitelist)
            if raw_whitelist is None or raw_whitelist == '':
                return False
            whitelist = raw_whitelist.split(',')
            if active_pose_bone is None:
                return False
            if not any(substring in active_pose_bone.name for substring in whitelist):
                return False

        try:
            postfix_of_side = self.get_postfix_of_sides(context, obj_data)

            if not context.active_pose_bone.name.endswith(postfix_of_side):
                return False
            if not Pbn_ui_enable + postfix_of_side in obj_data:
                return False
            if not obj_data.get(Pbn_ui_enable + postfix_of_side):
                return False
            if not Pbn_actual_pole_bone_name + postfix_of_side in obj_data:
                return False
            if not Pbn_pole_snap_target_name + postfix_of_side in obj_data:
                return False
            return True
        except (AttributeError, KeyError, TypeError):
            print("Error in Control_UI.poll: ", traceback.format_exc())
            return False
    
    def draw(self, context):

        obj = context.active_object
        obj_data = obj.data
        pose_bones = obj.pose.bones
        
        postfix_of_side = self.get_postfix_of_sides(context, obj_data)

        pole_bone = pose_bones[obj_data.get(Pbn_actual_pole_bone_name + postfix_of_side)]
        pole_snap_target_bone = pose_bones[obj_data.get(Pbn_pole_snap_target_name + postfix_of_side)]

        # Set the necessary properties for the operator
        POSE_OT_pole_snap.pole_bone = pole_bone
        POSE_OT_pole_snap.pole_snap_target_bone = pole_snap_target_bone

        # Layout
        layout = self.layout

        layout.prop(pole_snap_target_bone, f'["{Bone_PN_AKP_mix}"]', text=f'Mix{postfix_of_side}', slider=True)
        layout.operator('pose.knee_pole_snap_to_auto', text=f'Snap to Auto Pole{postfix_of_side}')

    @staticmethod
    def get_postfix_of_sides(context, custom_property_carrier):
        
        postfixes_of_sides = custom_property_carrier.get(PN_postfixes_of_sides).split(',')
        matched_postfix = next((postfix for postfix in postfixes_of_sides if context.active_pose_bone.name.endswith(postfix)), '')

        return matched_postfix
    
    
#endregion


#region - Blender Registration
def register():
    bpy.utils.register_class(POSE_OT_init_initiator_properties)
    bpy.utils.register_class(POSE_OT_init_for_object)
    # bpy.utils.register_class(Init_Initiator_UI)
    bpy.utils.register_class(Init_UI)
    bpy.utils.register_class(Control_UI)
    bpy.utils.register_class(POSE_OT_pole_snap)

def unregister():
    bpy.utils.unregister_class(POSE_OT_init_initiator_properties)
    bpy.utils.unregister_class(POSE_OT_init_for_object)
    # bpy.utils.unregister_class(Init_Initiator_UI)
    bpy.utils.unregister_class(Init_UI)
    bpy.utils.unregister_class(Control_UI)
    bpy.utils.unregister_class(POSE_OT_pole_snap)


if __name__ == '__main__':
    try:
        unregister()
    except:
        pass
    register()
# endregion