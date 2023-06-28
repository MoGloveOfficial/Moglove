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

#bone_list = ["親指０.R","人指１.R","中指１.R","薬指１.R","小指１.R"]
bone_list = ["親指０.L","親指１.L","親指２.L", 
            "人指１.L","人指２.L","人指３.L",
            "中指１.L","中指２.L","中指３.L",
            "薬指１.L","薬指２.L","薬指３.L",
            "小指１.L","小指２.L","小指３.L"]

recordStatus = 0

sampleRate = 1/60
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
        layout.operator("addonname.myop_operator")
        layout.operator("record.myop_operator")
        
 
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
            
            while True:
                raw = arduino.readline().decode('utf-8').rstrip().split(',')
                if len(raw) == 60:  #data
                    data = list(map(float, raw))
                    print(data)
                    break
                else:
                    pass

            for fin, bone_name in enumerate(bone_list):
                try:
                    print("Bone name")
                    print(bone_name)
                    bone = armature.bones.get(bone_name)
                    print(bone)
                    rot_quat = mathutils.Quaternion(data[4*fin:4*fin+4])
                    print(rot_quat)
                    bone.rotation_quaternion = rot_quat
                    
                except:
                    print("Bone not found")
                    return {"CANCELLED"}
                
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


"""
 
def p_filter(self, object):
    return object.type == 'ARMATURE'
"""


def menu_func(self,context):
    self.layout.operator(ModalTimerOperator.bl_idname, text=ModalTimerOperator.bl_label)
 
 
classes = [MyProperties, MoGlovePanel, ConnectOP, RecordOP, ModalTimerOperator]
 
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