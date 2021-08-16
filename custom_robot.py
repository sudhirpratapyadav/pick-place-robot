import os, inspect
import pdb
import pybullet as p
import pybullet_data
import time


#Getting Absolute Path from Relative Path of URDF file
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

# robotUrdfPath = os.path.join(currentdir, "./urdf/gripper.urdf")
robotUrdfPath = os.path.join(currentdir, "./urdf/robotiq_2f_85_description.urdf")
# robotUrdfPath = os.path.join(currentdir, "./urdf/sisbot.urdf")
# robotUrdfPath = os.path.join(currentdir, "./urdf/ur5_gripper.urdf")
# robotUrdfPath = os.path.join(currentdir, "./urdf/ur5.urdf")

# connect to engine servers
physicsClient = p.connect(p.GUI) # GUI/DIRECT
# add search path for loadURDF
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# define world
p.setGravity(0,0,-10)
planeID = p.loadURDF("plane.urdf")

# define robot
robotStartPos = [0,0,0]
robotStartOrn = p.getQuaternionFromEuler([0,0,0])
print("----------------------------------------")
print("Loading robot from {}".format(robotUrdfPath))
print("----------------------------------------")
robotID = p.loadURDF(robotUrdfPath, robotStartPos, robotStartOrn)

# get joint information
numJoints = p.getNumJoints(robotID) 
jointTypeList = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
print("------------------------------------------")
print("Number of joints: {}".format(numJoints))
for i in range(numJoints):
    jointInfo = p.getJointInfo(robotID, i)
    jointID = jointInfo[0]
    jointName = jointInfo[1].decode("utf-8")
    jointType = jointTypeList[jointInfo[2]]
    jointLowerLimit = jointInfo[8]
    jointUpperLimit = jointInfo[9]
    print("\tID: {}".format(jointID))
    print("\tname: {}".format(jointName))
    print("\ttype: {}".format(jointType))
    print("\tlower limit: {}".format(jointLowerLimit))
    print("\tupper limit: {}".format(jointUpperLimit))
print("------------------------------------------")

# get links
linkIDs = list(map(lambda linkInfo: linkInfo[1], p.getVisualShapeData(robotID)))
linkNum = len(linkIDs)

# start simulation
try:
    flag = True
    textPose = list(p.getBasePositionAndOrientation(robotID)[0])
    textPose[2] += 1

    p.addUserDebugText("Press \'w\' and magic!!", textPose, [255,0,0], 1)

    prevLinkID = 0
    linkIDIn = p.addUserDebugParameter("linkID", 0, linkNum-1e-3-1, 0)

    # for i in range(5):
    #     print("-----------------iii-----------------")
    #     print(i)
    #     print("-----------------step----------------")

    #     p.stepSimulation()
    #     print("-----------------sleep------------------")
    #     time.sleep(1)
    #     print("------------------------------------")

    # time.sleep(1)

    while(flag):
        p.stepSimulation()
        linkID = p.readUserDebugParameter(linkIDIn)
        if linkID!=prevLinkID:
            p.setDebugObjectColor(robotID, int(prevLinkID), [255,255,255])
            p.setDebugObjectColor(robotID, int(linkID), [255,0,0])
            print(int(linkID))
        prevLinkID = linkID

    p.disconnect()

except:
    p.disconnect()
