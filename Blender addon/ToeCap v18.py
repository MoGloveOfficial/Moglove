"""
MoGlove addon for Blender

FPS set to 30FPS
interval = 0.03333s or 33.33ms
Developed by MoGlove project

Dev Note
[w1,x1,y1,z1,w2,x2,y2,z2,w3,x3,y3,z3,w4,x4,y4,z4]


fin_count
thumb = 0
index = 1
middle = 2
ring = 3
pinky = 4

"""

import time
import mathutils
import math
import bpy
import serial
from bpy.props import PointerProperty

bone_list = ["右足親指1.R","右足中指1.R","右足小指1.R"]
recordStatus = 0

sampleRate = 1/24
FPS = 1/sampleRate



class MyProperties(bpy.types.PropertyGroup):
    portR : bpy.props.IntProperty(name= "COM Port R")
    #portL : bpy.props.IntProperty(name= "COM Port L")
 
class MoGlovePanel(bpy.types.Panel):
    bl_label = "MoGlove"
    bl_idname = "ADDONNAME_PT_main_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
 
    def draw(self, context):
        layout = self.layout
        scene = context.scene
        mytool = scene.my_tool
        
        layout.prop(mytool, "portR")
        #layout.prop(mytool, "portL")
        
        row = self.layout.row(align=True)
        layout.operator("calibrate.myop_operator")
        layout.operator("addonname.myop_operator")
        layout.operator("record.myop_operator")
        
class CalibrateOP(bpy.types.Operator):
    bl_label = "Calibrate"
    bl_idname = "calibrate.myop_operator"
    
    def execute(self, context):
        scene = context.scene
        mytool = scene.my_tool
        arduino = serial.Serial("COM"+str(mytool.portR),115200,timeout = 5)
        while True:
            raw = arduino.readline().decode('utf-8').rstrip().split(',')
            if len(raw) == 16:
                data = list(map(float, raw))
                break
            
        global ankle_cal
        ankle_cal = mathutils.Quaternion((data[0:4]))
        
        global thumb_cal
        thumb_cal = mathutils.Quaternion((data[4:8]))
        
        global middle_cal
        middle_cal = mathutils.Quaternion((data[8:12]))
        
        global pinky_cal
        pinky_cal = mathutils.Quaternion((data[12:]))

        
        print([ankle_cal, thumb_cal, middle_cal, pinky_cal])
        print("Calibrated!")
        return {'FINISHED'}
 
class RecordOP(bpy.types.Operator):
    bl_label = "Record"
    bl_idname = "record.myop_operator"
    def execute(self, context):
        global recordStatus
        recordStatus = not recordStatus #Toggle record Status
        return {'FINISHED'}
 
 
class ConnectOP(bpy.types.Operator):
    bl_label = "Connect"
    bl_idname = "addonname.myop_operator"
    
    def execute(self, context):
        bpy.ops.wm.modal_timer_operator()
        scene = context.scene
        mytool = scene.my_tool
        ModalTimerOperator._connection = serial.Serial("COM"+str(mytool.portR),115200,timeout = 5)
        return {'FINISHED'}
    

class ModalTimerOperator(bpy.types.Operator):
    bl_idname = "wm.modal_timer_operator"
    bl_label = "Modal Timer Operator"
    
    _timer = None
    _connection = None
    def modal(self,context,event):
        #if event.type is 'RIGHTMOUSE' or event.type is 'ESC':
        if event.type in {'RIGHTMOUSE', 'ESC'}:
            self.cancel(context)
            print("Cancelled!")
            exit()
            return {'CANCELLED'}
        
        if event.type == "TIMER":
            scene = context.scene
            mytool = scene.my_tool
            obj = context.active_object
            armature = obj.pose
            arduino = ModalTimerOperator._connection
            #arduino = serial.Serial("COM"+str(mytool.portR),115200,timeout = 5)
            
            while True:
                raw = arduino.readline().decode('utf-8').rstrip().split(',')
                if len(raw) == 16:
                    data = list(map(float, raw))
                    break
                else:
                    pass

            #Raw data
            ankle_quat = mathutils.Quaternion((data[0:4]))
            thumb_quat = mathutils.Quaternion((data[4:8]))
            middle_quat = mathutils.Quaternion((data[8:12]))
            pinky_quat = mathutils.Quaternion((data[12:]))
            
            ankle_rot = ankle_cal @ ankle_quat.inverted()    #rotation of ankle
            thumb_rot = thumb_cal @ thumb_quat.inverted()
            middle_rot = middle_cal @ middle_quat.inverted()
            pinky_rot = pinky_cal @ pinky_quat.inverted()
            
            thumb_relative = thumb_rot @ ankle_rot.inverted()     #relative to ankle quaternion
            middle_relative = middle_rot @ ankle_rot.inverted()
            pinky_relative = pinky_rot @ ankle_rot.inverted()
            
            for bone_name in bone_list:
                #print(bone_name)
                bone = armature.bones.get(bone_name)
                #print(bone)
                
                #Offset the toe orientation from ankle orientation
                if bone_name == "右足親指1.R":
                    rot_quat = mathutils.Quaternion(thumb_relative)
                elif bone_name == "右足中指1.R":
                    rot_quat = mathutils.Quaternion(middle_relative)
                elif bone_name == "右足小指1.R":
                    rot_quat = mathutils.Quaternion(pinky_relative)
                else:
                    print("Bone not found")
                    return {"CANCELLED"}                
                print(bone_name+str(rot_quat))
                bone.rotation_quaternion = rot_quat
                
                global recordStatus
                print(recordStatus)
                if recordStatus == 1:
                    bone.keyframe_insert(data_path = "rotation_quaternion", frame = bpy.context.scene.frame_current)
                    print("Keyframed")

        return {'PASS_THROUGH'}
    
    def execute(self, context):
        wm = context.window_manager
        self._timer = wm.event_timer_add(sampleRate, window=context.window)
        wm.modal_handler_add(self)
        return {'RUNNING_MODAL'}
    
    def cancel(self, context):
        wm = context.window_manager
        wm.event_timer_remove(self._timer)


def quaternion_from_two_quaternions(q1, q2):
    
    # Convert quaternions to mathutils.Quaternion objects
    #q1 => toes, q2 => ankle
    
    q1_math = mathutils.Quaternion(q1)
    q2_math = mathutils.Quaternion(q2)
    
    # Compute the rotation from q1 to q2
    q_diff_math = q2_math.inverted() * q1_math
    # Convert the result to a mathutils.Quaternion object
    q3_math = mathutils.Quaternion((q_diff_math.w, q_diff_math.x, q_diff_math.y, q_diff_math.z))
    q3_math.normalize()
    return q3_math

"""
 
def p_filter(self, object):
    return object.type == 'ARMATURE'
"""


def menu_func(self,context):
    self.layout.operator(ModalTimerOperator.bl_idname, text=ModalTimerOperator.bl_label)
 
 
classes = [MyProperties, MoGlovePanel,CalibrateOP, ConnectOP, RecordOP, ModalTimerOperator]
 
def register():
    for cls in classes:
        bpy.utils.register_class(cls)
        
        bpy.types.Scene.my_tool = bpy.props.PointerProperty(type= MyProperties)
    bpy.types.VIEW3D_MT_view.append(menu_func)
    
"""
    bpy.types.Scene.Armature = bpy.props.PointerProperty(
        type=bpy.types.Object,
        poll=p_filter,
        )
"""
 
def unregister():
    for cls in classes:
        bpy.utils.unregister_class(cls)
        del bpy.types.Scene.my_tool
        
    
    #del bpy.types.Scene.Armature
    
    bpy.types.VIEW3D_MT_view.remove(menu_func)

 
if __name__ == "__main__":
    register()