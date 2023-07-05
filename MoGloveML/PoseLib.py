import bpy


num_poses = 21

# Specify the path to the CSV file
header_path = "C:\\Users\\PC-kun\\Desktop\\MoGlove ML\\pattern.h"

# Get the armature object by its name
armature_name = "armature"  # Replace with the name of your armature object
armature = bpy.data.objects[armature_name]



# Get the finger bones
finger_bones = ["親指０.L", "親指１.L", "親指２.L",
                "人指１.L", "人指２.L", "人指３.L",
                "中指１.L", "中指２.L", "中指３.L",
                "薬指１.L", "薬指２.L", "薬指３.L",
                "小指１.L", "小指２.L", "小指３.L"]  # Replace with the names of your finger bones

fin00=[]
fin01=[]
fin02=[]
fin11=[]
fin12=[]
fin13=[]
fin21=[]
fin22=[]
fin23=[]
fin31=[]
fin32=[]
fin33=[]
fin41=[]
fin42=[]
fin43=[]

# Open the CSV file for writing
with open(header_path, mode='w') as f:
    for frame in range(num_poses):
        bpy.context.scene.frame_set(frame)
        
        for num, bone_name in enumerate(finger_bones):
            bone = armature.pose.bones.get(bone_name)
            if bone:
                quat = bone.rotation_quaternion
                if num == 0:
                    fin00.append(quat)
                elif num == 1:
                    fin01.append(quat)
                elif num == 2:
                    fin02.append(quat)
                elif num == 3:
                    fin11.append(quat)
                elif num == 4:
                    fin12.append(quat)
                elif num == 5:
                    fin13.append(quat)
                elif num == 6:
                    fin21.append(quat)
                elif num == 7:
                    fin22.append(quat)
                elif num == 8:
                    fin23.append(quat)
                elif num == 9:
                    fin31.append(quat)
                elif num == 10:
                    fin32.append(quat)
                elif num == 11:
                    fin33.append(quat)
                elif num == 12:
                    fin41.append(quat)
                elif num == 13:
                    fin42.append(quat)
                elif num == 14:
                    fin43.append(quat)
                    
    f.write("float qfins00[" + str(len(fin00))+ "][4] = {\n") 
    for quat in fin00:
        f.write("{" + ', '.join(map(str, quat))+ "},")
    f.write("};")
    
    
    
    
    

                
                
                
                
    
    writer = csv.writer(csvfile, delimiter=',')
    # Write the header row
    header = ["Frame", "Bone Name", "Quaternion W", "Quaternion X", "Quaternion Y", "Quaternion Z"]
    writer.writerow(header)
    # Iterate over each frame
    for frame in range(19):  # 0 to 20 (inclusive)
        # Set the current frame
        bpy.context.scene.frame_set(frame)

        # Write the bone name and quaternion values to the CSV file
        for bone_name in finger_bones:
            bone = armature.pose.bones.get(bone_name)
            if bone:
                # Get the quaternion of the bone
                quaternion = bone.rotation_quaternion

                # Write the frame number, bone name, and quaternion values to the CSV file
                row = [frame, bone_name, quaternion.w, quaternion.x, quaternion.y, quaternion.z]
                writer.writerow(row)
            else:
                print(f"Bone '{bone_name}' not found in the armature.")

print(f"CSV file written successfully to: {csv_path}")
