# Auto IK Knee Pole (AIKKP)

"""
Auto IK Knee Pole (AIKKP)

Steps to implement Auto IK Knee Pole:
1. Select an object to act as the Initiator (can be any Object).
2. Press F3 then search & run the operator "Init Auto IK Knee Pole Initiator Properties" to initialize the required custom properties on the Initiator.
3. Populate the properties on the Initiator.
4. Select the target Armature(s), then select the Initiator object LAST (so it is the Active Object).
5. Press "Init Auto IK Knee Pole for Selected Armature(s)" button in the Item panel to do the initialization.

---
* "Initiator" means the object that has the initiator properties, which is used to initialize the Auto IK Knee Pole function to target armature(s).

Initiator Properties:
- "Postfixes of Sides": Comma-separated postfixes, one per leg, e.g. ".L,.R". Every basename property below is combined with each postfix to look up the actual bone names per side.
- "Leg_IK_Root_Bone_Basename": Basename of the IK root bone (e.g. the hip/thigh bone), without the side postfix.
- "Leg_IK_Target_Bone_Basename": Basename of the IK tip/target bone (e.g. the foot bone), without the side postfix.
- "Leg_IK_Pole_Bone_Basename": Basename of the existing IK pole bone, without the side postfix.
- "Whitelist_of_Keyword": Comma-separated keywords (e.g. "tip,ik,pole"). Only pose bones whose name contains at least one of these keywords will show the Auto IK Knee Pole control UI.
- "Tip_Bone_Points_Backward": Checkbox. Enable it if the tip (foot) bone's local Y axis points backward (toward the heel) instead of forward (toward the toe); the forward direction used for the calculation will be flipped accordingly.

---
What the initialization process does:

1. Creates AIKKP helper bones in the target armature for each side postfix:
   - AIKKP_ik_stem_mid<postfix>     : Root helper located at IK stem center.
   - AIKKP_ik_stem_forward<postfix> : Marks forward direction of the IK stem.
   - AIKKP_point_to_pole<postfix>  : Calculated direction pointing to pole (driven Z rotation).
   - AIKKP_auto_pole<postfix>      : Target position bone for the pole constraint, holds 'Auto_IK_Knee_Pole_mix'.

2. Adds a driver to AIKKP_point_to_pole's Z rotation (Euler) via the custom driver function "get_to_knee_rotation".

3. Applies 'COPY_LOCATION' constraint on the original Pole Bone targeting `AIKKP_auto_pole`, driven by the custom property `Auto_IK_Knee_Pole_mix` on `AIKKP_auto_pole`.

4. Sets custom properties on the Armature Data:
   - AIKKP_Postfixes_of_Sides
   - AIKKP_Whitelist_of_Keywords
   - AIKKP_Enable_UI<postfix>
   - AIKKP_Pole_Bone_Name<postfix>
   - AIKKP_Pole_Snap_Target_Name<postfix>
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
"""

# Initiator Custom Properties Names
Init_PN_postfixes_of_sides = 'Postfixes of Sides'
Init_PN_leg_ik_root_bone_basename = 'Leg_IK_Root_Bone_Basename'
Init_PN_leg_ik_target_bone_basename = 'Leg_IK_Target_Bone_Basename'
Init_PN_leg_ik_pole_bone_basename = 'Leg_IK_Pole_Bone_Basename'
Init_PN_tip_bone_points_backward = 'Tip_Bone_Points_Backward'  # If True, the tip (foot) bone's local Y axis points backward, so it must be flipped to get the actual forward direction.
Init_PN_keywords_whitelist = 'Whitelist_of_Keyword'

list_of_Init_PN = [
    Init_PN_postfixes_of_sides,
    Init_PN_leg_ik_root_bone_basename,
    Init_PN_leg_ik_target_bone_basename,
    Init_PN_leg_ik_pole_bone_basename,
    Init_PN_tip_bone_points_backward,
    Init_PN_keywords_whitelist
]

# Preset Initiator property values matching Rigify's default IK leg bone names.
# Used by "Init AIKKP for Selected Rigify Armature(s)" to skip having to set up a separate Initiator object.
Rigify_Initiator_Preset = {
    Init_PN_postfixes_of_sides: '.L,.R',
    Init_PN_leg_ik_root_bone_basename: 'MCH-thigh_ik_swing',
    Init_PN_leg_ik_target_bone_basename: 'foot_ik',
    Init_PN_leg_ik_pole_bone_basename: 'thigh_ik_target',
    Init_PN_keywords_whitelist: 'ik',
    Init_PN_tip_bone_points_backward: True,
}

# Custom Properties Names
PN_postfixes_of_sides = 'AIKKP_Postfixes_of_Sides'       # e.g. ".L,.R" for 2 legs
PN_keywords_whitelist = 'AIKKP_Whitelist_of_Keywords'    # This is used to filter the pose bones that can show the Auto IK Knee Pole UI. e.g. "tip,ik,pole"

Pbn_ui_enable = 'AIKKP_Enable_UI'

Pbn_pole_snap_target_name = 'AIKKP_Pole_Snap_Target_Name'
Pbn_actual_pole_bone_name = 'AIKKP_Pole_Bone_Name'

Bone_PN_AIKKP_mix = 'Auto_IK_Knee_Pole_mix'

# Generated Bone Names
GB_stem_mid_basename = 'AIKKP_ik_stem_mid'               # As the root of all generated bones. Represents the direction from the IK tip (foot) to the IK root (hip), located at the center of the IK stem.
GB_stem_forward_basename = 'AIKKP_ik_stem_forward'       # Represents the forward direction of the IK stem. Used for calculating the result.
GB_point_to_pole_basename = 'AIKKP_point_to_pole'        # Calculated direction point to knee pole (which has drivers on its Z rotation)
GB_auto_pole_basename = 'AIKKP_auto_pole'                # Result pole position to be followed. can add delta position on it for manual adjustment

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

def get_postfix_of_side(context, custom_property_carrier):
    postfixes_of_sides = custom_property_carrier.get(PN_postfixes_of_sides).split(',')
    matched_postfix = next((postfix for postfix in postfixes_of_sides if context.active_pose_bone.name.endswith(postfix)), '')
    return matched_postfix
#endregion


#region - Initialization

def is_initiator(obj):
    return obj is not None and all(prop in obj for prop in list_of_Init_PN)

def get_target_armatures(context, exclude_obj=None):
    return [x for x in context.selected_objects if x != exclude_obj and x.type == 'ARMATURE']

def get_side_postfixes(initiator):
    return initiator[Init_PN_postfixes_of_sides].split(',')

class Init_UI(bpy.types.Panel):
    bl_idname = 'VIEW3D_PT_AutoKneePole_Init_UI'
    bl_label = 'Auto IK Knee Pole'
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Item'

    @classmethod
    def poll(self, context):
        return is_initiator(context.active_object)

    def draw(self, context):
        obj = context.active_object
        layout = self.layout

        if is_initiator(obj):
            def check_if_can_init():
                target_objs = [x for x in context.selected_objects if x != obj]
                for target_obj in target_objs:
                    if target_obj.type != 'ARMATURE':
                        return 'not an armature'

                    postfixes_of_sides = obj.get(Init_PN_postfixes_of_sides).split(',')
                    required_basenames = [
                        obj[Init_PN_leg_ik_root_bone_basename],
                        obj[Init_PN_leg_ik_target_bone_basename],
                        obj[Init_PN_leg_ik_pole_bone_basename]
                    ]

                    pose_bones = target_obj.pose.bones
                    for basename in required_basenames:
                        for postfix in postfixes_of_sides:
                            required_bone_name = basename + postfix
                            if pose_bones.get(required_bone_name) is None:
                                return f'missing bone "{required_bone_name}"'
                return True

            can_init_check_result = check_if_can_init()
            if can_init_check_result is True:
                # Generate the Auto IK Knee Pole bones and drivers for the selected armature(s) excluding the initiator object itself.
                layout.operator('pose.init_for_object', text=f'Init Auto IK Knee Pole for Selected Armature(s)')
            else:
                layout.label(text=f'Cannot Init: One or more targets are {can_init_check_result}.')


class POSE_OT_init_initiator_properties(bpy.types.Operator):
    bl_idname = "pose.init_initiator_properties"
    bl_label = "Init Auto IK Knee Pole Initiator Properties"
    bl_description = "Initialize custom properties for Auto IK Knee Pole Initiator"
    bl_options = {'REGISTER', 'UNDO'}


    @classmethod
    def poll(cls, context):
        return context.active_object is not None

    def execute(self, context):
        obj = context.active_object

        if self.has_initialized_properties(obj):
            self.report({'WARNING'}, 'Object already has the properties.')
            return {'CANCELLED'}

        defaults = {
            Init_PN_postfixes_of_sides: '.L,.R',
            Init_PN_leg_ik_root_bone_basename: '',
            Init_PN_leg_ik_target_bone_basename: '',
            Init_PN_leg_ik_pole_bone_basename: '',
            Init_PN_keywords_whitelist: 'ik',
            Init_PN_tip_bone_points_backward: False,
        }
        for prop in list_of_Init_PN:
            if prop not in obj:
                obj[prop] = defaults.get(prop, '')
        return {'FINISHED'}

    @staticmethod
    def has_initialized_properties(obj):
        return all(prop in obj for prop in list_of_Init_PN)

class POSE_OT_init_for_object(bpy.types.Operator):
    bl_idname = "pose.init_for_object"
    bl_label = "Init Auto IK Knee Pole for object"

    @classmethod
    def poll(self, context):
        obj = context.active_object
        if obj is None:
            return False

        target_armatures = get_target_armatures(context, obj)
        if len(target_armatures) == 0:
            return False
        return True

    def execute(self, context):

        initiator = bpy.context.active_object

        target_armatures = get_target_armatures(context, initiator)
        for target_armature in target_armatures:

            postfixes = get_side_postfixes(initiator)

            for postfix in postfixes:
                init_aikkp_bones(context, initiator, target_armature, postfix)

            init_aikkp_custom_properties(context, initiator, target_armature)

        context.view_layer.objects.active = initiator  # Set the initiator back to active object after initialization.
        context.view_layer.update()
        return {'FINISHED'}


def init_aikkp_bones(context, initiator, target, post_fix):
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
    tip_bone_name = initiator[Init_PN_leg_ik_target_bone_basename] + post_fix
    pole_bone_name = initiator[Init_PN_leg_ik_pole_bone_basename] + post_fix

    bone_root = edit_bones[root_bone_name]
    bone_foot = edit_bones[tip_bone_name]
    bone_pole = edit_bones[pole_bone_name]

    # If the tip (foot) bone's local Y axis points backward, flip it to get the actual forward direction.
    tip_bone_points_backward = initiator.get(Init_PN_tip_bone_points_backward, False)
    foot_forward_vector = -bone_foot.vector if tip_bone_points_backward else bone_foot.vector

    root_to_tip = bone_foot.head - bone_root.head
    stem_bone_length = root_to_tip.length * stem_bone_length_multiplier
    forward = proj_on_plane(foot_forward_vector, root_to_tip).normalized()

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
    bone_stem_forward.tail = bone_stem_mid.head + forward * stem_bone_length * 0.5

    bone_point_to_pole = add_bone(point_to_pole_name, bone_stem_mid)
    bone_point_to_pole.head = bone_stem_mid.head
    bone_point_to_pole.tail = bone_point_to_pole.head + proj_on_plane(bone_pole.head - bone_point_to_pole.head, root_to_tip).normalized() * stem_bone_length
    # Explicitly align the Z axis to the stem's up direction (same direction as bone_stem_mid's Y axis) instead of relying on Blender's roll=0 heuristic,
    # which is discontinuous and can flip the Z axis depending on the bone's own direction, causing the driver's signed-angle rotation to come out mirrored.
    bone_point_to_pole.align_roll(-root_to_tip)
    bone_point_to_pole.color.palette = 'THEME04'

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
    add_constraint(pose_bone_stem_mid, 'COPY_LOCATION', tip_bone_name, 0, 0.5)
    # Make pose_bone_stem_mid point up along the IK stem
    add_constraint(pose_bone_stem_mid, 'DAMPED_TRACK', tip_bone_name, 0, 1).track_axis = 'TRACK_NEGATIVE_Y'

    pose_bone_pole = target.pose.bones[pole_bone_name]

    add_constraint(pose_bone_pole, 'COPY_LOCATION', auto_pole_name, 0, 1)

    # -- Add driver --
    # for Z rotation of the bone point_to_pole_name
    pose_bone_point_to_pole = target.pose.bones[point_to_pole_name]
    pose_bone_point_to_pole.rotation_mode = 'XYZ' # The default is Quaternion, so we need to change it to Euler, or will fail
    driver = pose_bone_point_to_pole.driver_add('rotation_euler', 2).driver  # Z rotation
    driver.type = 'SCRIPTED'
    # Set color to the bone to represent it has a driver
    pose_bone_point_to_pole.color.palette = 'THEME04'

    def set_up_pose_bone_var(var_name, pose_bone_name):
        var = driver.variables.new()
        var.name = var_name
        var.type = 'SINGLE_PROP'
        var.targets[0].id = target
        var.targets[0].data_path = f'pose.bones["{pose_bone_name}"]'
        return var

    var_ik_up = set_up_pose_bone_var('ik_up', stem_mid_name)
    var_ik_forward = set_up_pose_bone_var('ik_fwd', stem_forward_name)
    var_foot = set_up_pose_bone_var('foot', tip_bone_name)

    # Flip the foot's forward axis in the driver too, so it stays consistent with the forward direction used above to place the generated bones.
    foot_bone_forward_axis_name = '-Y' if tip_bone_points_backward else 'Y'
    driver.expression = f"get_to_knee_rotation({var_ik_up.name}, {var_ik_forward.name}, {var_foot.name}, foot_bone_forward_axis_name='{foot_bone_forward_axis_name}')"

    bpy.ops.object.mode_set(mode = currentMode)  # Restore the original mode after bone creation and constraint addition.

def init_aikkp_custom_properties(context, initiator, target):
    target.data[PN_postfixes_of_sides] = initiator[Init_PN_postfixes_of_sides]
    target.data[PN_keywords_whitelist] = initiator[Init_PN_keywords_whitelist]

    postfixes = get_side_postfixes(initiator)

    for postfix in postfixes:
        # Bone names
        pole_bone_name = initiator[Init_PN_leg_ik_pole_bone_basename] + postfix
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
        pose_bone_pole_snap_target[Bone_PN_AIKKP_mix] = 1.0
        pose_bone_pole_snap_target.id_properties_ui(Bone_PN_AIKKP_mix).update(
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
        var_mix.targets[0].data_path = f'pose.bones["{pole_snap_target_bone_name}"]["{Bone_PN_AIKKP_mix}"]'
        driver.expression = f'{var_mix.name}'


class POSE_OT_init_for_rigify(bpy.types.Operator):
    bl_idname = "pose.init_aikkp_for_rigify"
    bl_label = "Init Auto IK Knee Pole for Selected Rigify Armature(s)"
    bl_description = "Initialize Auto IK Knee Pole on the selected Rigify armature(s) using Rigify's default IK leg bone names, without needing a separate Initiator object"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return len(get_target_armatures(context)) > 0

    def execute(self, context):
        initiator = Rigify_Initiator_Preset
        target_armatures = get_target_armatures(context)

        for target_armature in target_armatures:
            postfixes = get_side_postfixes(initiator)

            for postfix in postfixes:
                init_aikkp_bones(context, initiator, target_armature, postfix)

            init_aikkp_custom_properties(context, initiator, target_armature)

        context.view_layer.update()
        return {'FINISHED'}
#endregion


#region - Driver Calculation Functions
def get_result_direction(ik_stem_up, foot_forward, foot_up, foot_right):
    """
    Decide whether the knee pole should follow the foot's forward or up
    direction, blending smoothly near the switch-over angle instead of
    popping.

    Edge cases: foot pointing straight down/up the leg (forward vector
    degenerates) falls back to foot_up; angles near the threshold are
    slerp'd instead of hard-switched, with blend width scaled by how
    aligned foot_right is with ik_stem_up; sign flips when the foot points
    backward or crosses into the "pointing down" half, to avoid 180° jumps.

    (doc generated by Claude, not fully examined)
    """

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


#region - Custom UI for Auto IK Knee Pole controlling
"""
- Switch on/off Auto IK Knee Pole (0 ~ 1 blending)
- Snap manual position to auto position
"""
class POSE_OT_pole_snap(bpy.types.Operator):
    bl_idname = "pose.knee_pole_snap_to_auto"
    bl_label = "Snap Pole"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return (context.mode == 'POSE' and
                context.active_object and
                context.active_object.type == 'ARMATURE' and
                context.active_pose_bone is not None)

    def execute(self, context):
        obj = context.active_object
        obj_data = obj.data
        pose_bones = obj.pose.bones

        postfix_of_side = get_postfix_of_side(context, obj_data)

        # 1. Get postfix of side
        if not postfix_of_side:
            self.report({'WARNING'}, "Active bone does not match any AIKKP side postfix.")
            return {'CANCELLED'}

        # 2. Get bone names
        pole_name = obj_data.get(Pbn_actual_pole_bone_name + postfix_of_side)
        snap_target_name = obj_data.get(Pbn_pole_snap_target_name + postfix_of_side)

        if not pole_name or not snap_target_name:
            return {'CANCELLED'}

        pole_bone = pose_bones.get(pole_name)
        pole_snap_target_bone = pose_bones.get(snap_target_name)

        if not pole_bone or not pole_snap_target_bone:
            return {'CANCELLED'}

        # 3. Run Snap
        pole_bone.matrix.translation = pole_snap_target_bone.matrix.translation
        context.view_layer.update()
        return {'FINISHED'}


class Control_UI(bpy.types.Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_label = 'Auto IK Knee Pole'
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
            postfix_of_side = get_postfix_of_side(context, obj_data)

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

        postfix_of_side = get_postfix_of_side(context, obj_data)
        pole_snap_target_bone = pose_bones[obj_data.get(Pbn_pole_snap_target_name + postfix_of_side)]

        # Layout
        layout = self.layout
        layout.prop(pole_snap_target_bone, f'["{Bone_PN_AIKKP_mix}"]', text=f'Mix{postfix_of_side}', slider=True)
        layout.operator('pose.knee_pole_snap_to_auto', text=f'Snap to Auto Pole{postfix_of_side}')
#endregion


#region - Blender Registration
def register():
    bpy.utils.register_class(POSE_OT_init_initiator_properties)
    bpy.utils.register_class(POSE_OT_init_for_object)
    bpy.utils.register_class(POSE_OT_init_for_rigify)
    bpy.utils.register_class(Init_UI)
    bpy.utils.register_class(Control_UI)
    bpy.utils.register_class(POSE_OT_pole_snap)

def unregister():
    bpy.utils.unregister_class(POSE_OT_init_initiator_properties)
    bpy.utils.unregister_class(POSE_OT_init_for_object)
    bpy.utils.unregister_class(POSE_OT_init_for_rigify)
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
