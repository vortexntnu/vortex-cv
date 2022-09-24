
## Path Following Perception Solution

Node for comperhensive solution to the path following task in RoboSub 2022 on Perception side. The Node only operates when fsm is in path_search/converge/execute state, and governs the transition between these.
    
3 main problems need to be solved by the node, and they are associated to the possible states for path:

### path_search:        
During path search the node uses ZED camera feed to detect the path. After path is detected, the coordinates are published to Autonomous and we request switch to converge.

### path_converge:      
During path converge the goal is to center the path in the FOV of the underwater downwards facing camera (in the future refered to as the UDFC). The path is classified in UDFC frame and it's areal centroid is mapped to odom and given as xy reference for the DP controller. The assumption being that this scheme will progressively get the path more and more into FOV, and finally in the centre of UDFC frame. After the path is sufficiently centered we move on to execute.

#### Mapping of points from image plane to odom:
The depth reference for the dp controller is given as the optimal depth to observe the path in UDFC frame. Assumed known depth of pool. Assumed known focal length. Assumed known depth coordinate of AUV. Then mapping from camera to odom is performed as follows:

### path_execute:       
During execute the drone keeps station over the path while Perception solves the estimation problem of the path to the next task. Here we can use the fact that we are given the oportunity for multiple measurements and samples to cast the task as a batch problem and extract as much information as possible before we declare a solution. The search for the next task starts when a waypoint is given to Autonomous after we have estimated path direction.