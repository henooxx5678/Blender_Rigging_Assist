# Auto IK Shoulder

"""

1. Add bones.

2. Add custom properties to the armature object to enable the assistance UI:

- AIKS_Enable_L_UI: Boolean
- AIKS_L_Effectiveness_Bone_Name: String
- AIKS_L_Mix_Control_Bone_Name: String

- AIKS_Enable_R_UI: Boolean
- AIKS_R_Effectiveness_Bone_Name: String
- AIKS_R_Mix_Control_Bone_Name: String

- AIKS_Whitelist_of_Keywords: String

"""


import bpy
import math
import bl_math
import mathutils

left_ui_enable_name = 'AIKS_Enable_L_UI'
left_effectiveness_bone_name = 'AIKS_L_Effectiveness_Bone_Name'
left_mix_control_bone_name = 'AIKS_L_Mix_Control_Bone_Name'

right_ui_enable_name = 'AIKS_Enable_R_UI'
right_effectiveness_bone_name = 'AIKS_R_Effectiveness_Bone_Name'
right_mix_control_bone_name = 'AIKS_R_Mix_Control_Bone_Name'

keywords_whitelist_name = 'AIKS_Whitelist_of_Keywords'

class ControlUI(bpy.types.Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_label = 'Auto IK Shoulder'
    bl_idname = 'VIEW3D_PT_AutoIKShoulder_Control_UI'
    bl_category = 'Item'

    @classmethod
    def poll(self, context):
        if context.mode != 'POSE':
            return False
        
        object_data = context.active_object.data
        active_pose_bone = context.active_pose_bone

        global keywords_whitelist_name
        if keywords_whitelist_name in object_data:
            raw_whitelist = object_data.get(keywords_whitelist_name)
            if raw_whitelist is None or raw_whitelist == '':
                return False
            whitelist = raw_whitelist.split(',')
            if active_pose_bone is None:
                return False
            if not any(substring in active_pose_bone.name for substring in whitelist):
                return False
        
        try:
            global left_ui_enable_name, right_ui_enable_name
            global left_effectiveness_bone_name, right_effectiveness_bone_name
            if context.active_pose_bone.name.endswith('.L') and left_ui_enable_name in object_data and object_data.get(left_ui_enable_name):
                return True
            if context.active_pose_bone.name.endswith('.R') and right_ui_enable_name in object_data and object_data.get(right_ui_enable_name):
                return True
        except (AttributeError, KeyError, TypeError):
            return False
    
    def draw(self, context):
        layout = self.layout

        object = context.active_object
        pose_bones = object.pose.bones
        
        is_left_side = context.active_pose_bone.name.endswith('.L')
        side = 'L' if is_left_side else 'R'
        
        global left_effectiveness_bone_name, right_effectiveness_bone_name
        effectiveness_bone_name = None
        if is_left_side:
            if left_effectiveness_bone_name in object.data:
                effectiveness_bone_name = object.data.get(left_effectiveness_bone_name)
        else:
            if right_effectiveness_bone_name in object.data:
                effectiveness_bone_name = object.data.get(right_effectiveness_bone_name)

        if effectiveness_bone_name is not None:
            layout.prop(pose_bones[effectiveness_bone_name].constraints['Copy Location'], 'influence', text=f'Effectiveness.{side}', slider=True)

        global left_mix_control_bone_name, right_mix_control_bone_name
        mix_control_bone_name = None
        if is_left_side:
            if left_mix_control_bone_name in object.data:
                mix_control_bone_name = object.data.get(left_mix_control_bone_name)
        else:
            if right_mix_control_bone_name in object.data:
                mix_control_bone_name = object.data.get(right_mix_control_bone_name)

        if mix_control_bone_name is not None:
            layout.prop(pose_bones[mix_control_bone_name].constraints['Damped Track'], 'influence', text=f'Mix.{side}', slider=True)


def register():
    bpy.utils.register_class(ControlUI)

def unregister():
    bpy.utils.unregister_class(ControlUI)  

if __name__ == '__main__':
    register()