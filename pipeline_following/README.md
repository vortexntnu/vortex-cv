## Further improvements
- The solution regarding the estimation of waypoint is not robust since the line parameters alpha and beta can end up as infinity. This is kind of handled with the if statement of maximum 90 degrees. 
- The RANSAC computation will fail when alpha and beta is undefined (when the pipeline is exactly 90 in relation to beluga).
- The main function path_following_udfc is killed if beluga is not in the possible states of the pipeline task. This functionality is not tested.
- With the adaptive thresholding there is no lower limit such that when there is no pipeline in the image it will make points all over. This will cause the RANSAC to fail (The number of points within threshold is a prosentage of the total number of points, therefore it will not manage to find a solution when all the points are scattered around the image) and set isDetected = False as wanted, but a more robust solution should be implemented.
- The correctness of the final waipont published to the landmark server is not properly tested. Regarding waypoint estimation based on alpha and beta, and transformation from base link to odom. The DP-controller also needs the correct orientation in order to work adequately.
- The DP-controll is limited to only get one goal, reach that goal and then get feed with a new point. Therefore, the current solution relies on reliable points every time, because we dont know when the state machine will use the point that lies in the landmark server.
- Integration with main launch file.

## note
- Odom and waypoint has been visualized in rviz. Looks promising.
