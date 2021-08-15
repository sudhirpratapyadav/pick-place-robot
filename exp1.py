import pybullet as simulator
import time
import pybullet_data

physicsClient = simulator.connect(simulator.GUI)   # simulator.GUI for graphical version
#physicsClient = simulator.connect(simulator.DIRECT) # simulator.DIRECT for non-graphical version

con_info = simulator.getConnectionInfo(physicsClient)

print(con_info)

if(simulator.isConnected()):
    simulator.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    simulator.setGravity(0,0,-10)

    print("Path\n\n")
    print(pybullet_data.getDataPath())
    print()

    planeId = simulator.loadURDF("plane.urdf")
    startPos = [0,0,1]
    startOrientation = simulator.getQuaternionFromEuler([0,0,0])
    boxId = simulator.loadURDF("sphere2.urdf",startPos, startOrientation)

    #set the center of mass frame (loadURDF sets base link frame) startPos/Orn
    simulator.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
    #print("Loading done")


    for i in range (240):
        simulator.stepSimulation()
        time.sleep(1./240.)
        #cubePos, cubeOrn = simulator.getBasePositionAndOrientation(boxId)
        #print(cubePos,cubeOrn)

    #print('\n')
    cubePos, cubeOrn = simulator.getBasePositionAndOrientation(boxId)
    print(cubePos,cubeOrn)

    simulator.disconnect()

else:
    print("Simulation Not Connected")