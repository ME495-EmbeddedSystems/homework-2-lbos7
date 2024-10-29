# ME495 Embedded Systems Homework 2
Author: Logan Boswell

This package uses turtlesim to drive a robot in rviz to catch a brick, if it is able to.

## Quickstart
1. Use `ros2 launch turtle_brick turtle_arena.launch.xml` to start the arena and turtle simulation
2. Use `ros2 service call /place turtle_brick_interfaces/srv/Place "{brick_location: {x: x-value, y: y-value, z: z-value}}"` and `ros2 service call /drop std_srvs/srv/Empty` to place and drop a brick
3. Here is a video of the turtle when the brick is within catching range
   <video src="https://github-production-user-asset-6210df.s3.amazonaws.com/181179449/381034686-bf12cb5a-fbd4-4de1-b234-e39c383f54ae.mp4?X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Credential=AKIAVCODYLSA53PQK4ZA%2F20241029%2Fus-east-1%2Fs3%2Faws4_request&X-Amz-Date=20241029T080729Z&X-Amz-Expires=300&X-Amz-Signature=2ba656d7bfefc38574fd3f971ebd3c69345da9feee4717f0446775fbd0e42d0f&X-Amz-SignedHeaders=host" width="500" />

4. Here is a video of the turtle when the brick cannot be caught

   <video src="https://github-production-user-asset-6210df.s3.amazonaws.com/181179449/381034975-859a523e-5577-4440-8df5-b089464ec17d.mp4?X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Credential=AKIAVCODYLSA53PQK4ZA%2F20241029%2Fus-east-1%2Fs3%2Faws4_request&X-Amz-Date=20241029T080822Z&X-Amz-Expires=300&X-Amz-Signature=d94ea45dc6cdab5d96c858976e6c157bebef68b993be06f92276f0fe72ea64d9&X-Amz-SignedHeaders=host" width="500" />