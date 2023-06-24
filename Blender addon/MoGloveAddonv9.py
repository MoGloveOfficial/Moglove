"""
MoGlove addon for Blender

FPS set to 30FPS
interval = 0.03333s or 33.33ms
Developed by MoGlove project



Dev Note

[w,x,y,z,thumb, index, middle, ring, pinky]

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


bone_list = ["親指０.R","人指１.R","中指１.R","薬指１.R","小指１.R"]
#"親指０.L"
#"人指１.L"
#"中指１.L"
#"薬指１.L"
#"小指１.L"


start_rot = (1,0,0,0) #Quat (w,x,y,z)




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
        #row.prop(context.scene, "Armature")
        layout.operator("addonname.myop_operator")
        #layout.operator("record_operator")
 
class RecordOP(bpy.types.Operator):
    bl_label = "Record"
    bl_idname = "record_operator"
    
    def execuet(self, context):
        pass
 
 
class ConnectOP(bpy.types.Operator):
    bl_label = "Connect"
    bl_idname = "addonname.myop_operator"
    
    
    def execute(self, context):
        bpy.ops.wm.modal_timer_operator()
        return {'FINISHED'}
    
    
class ModalTimerOperator(bpy.types.Operator):
    bl_idname = "wm.modal_timer_operator"
    bl_label = "Modal Timer Operator"
    
    _timer = None
    
    def modal(self,context,event):
        if event.type in {'RIGHTMOUSE', 'ESC'}:
            self.cancel(context)
            print("Cancelled!")
            return {'CANCELLED'}
        
        if event.type == "TIMER":
            #print the data
            scene = context.scene
            mytool = scene.my_tool
            
            obj = context.active_object
            armature = obj.pose
            print(obj)
            print(armature)
            
            arduino = serial.Serial("COM"+str(mytool.portR),115200,timeout = 1)
            fin_count  = 4
            
            while True:
                data = arduino.readline().decode('utf-8').rstrip().split(',')
                if len(data) == 9:
                    #data = data[4:9] #Optional
                    print(data)
                    break
                else:
                    pass


            for bone_name in bone_list:
                try:
                    print(bone_name)
                    bone = armature.bones.get(bone_name)
                    print(bone)
                    rot_quat = mathutils.Quaternion((1, float(data[fin_count]),0,0))
                    rot_quat.normalize()
                    print(rot_quat)
                    bone.rotation_quaternion = rot_quat
                    
                    fin_count+=1
                        
                except:
                    print("Bone not found")
                    return {"CANCELLED"}

        return {'PASS_THROUGH'}
    
    def execute(self, context):
        wm = context.window_manager
        self._timer = wm.event_timer_add(0.1, window=context.window)
        wm.modal_handler_add(self)
        return {'RUNNING_MODAL'}
    
    def cancel(self, context):
        wm = context.window_manager
        wm.event_timer_remove(self._timer)


def bend2Quat(bend):
    #bend value between 0 - 1 of flex sensors
    #w,x,y,z
    return normQuat([1,0,bend,0])


def norm_quat(*quat):
    """
    returns normalised quaternion
    (w,x,y,z)
    """
    rw = quat[0][0]
    rx = quat[0][1]
    ry = quat[0][2]
    rz = quat[0][3]
    mag = abs(math.sqrt(rx**2+ry**2+rz**2+rw**2))
    return (rw/mag, rx/mag, ry/mag, rz/mag)


"""
 
def p_filter(self, object):
    return object.type == 'ARMATURE'
"""


def menu_func(self,context):
    self.layout.operator(ModalTimerOperator.bl_idname, text=ModalTimerOperator.bl_label)
 
 
classes = [MyProperties, MoGlovePanel, ConnectOP, ModalTimerOperator]
 
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