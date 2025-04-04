## Manip Control Node
The Subscriber node contains 2 subscriptions. 

### Grasp Subscription:
- Topic : `\object_pose`
- Callback : `grasp_callback`
#### Grasp Callback:
- Function to analyze the input poses.
- Performs Inverse kinematics on given pose and obtains appropriate joint angles
- Uses Interbotix `set_joint_positions` to guide the Manipulator to object.
    - Releases Gripper
    - Goes to Home Position
    - Goes to a safe position (X,Y same as object but at a higher Z)
    - Goes to Object location via Inverse Kinematics
    - Grasps using gripper
    - Returns to sleep position
    - Keeps hold of the Object
- Does not perform unless dropeed the previously grabbed object.

### Drop Subscription:
- Topic : `\bin_pose`
- Callback : `drop_callback`
#### Drop Callback:
- Function to analyze the input poses.
- Performs Inverse kinematics on given pose and obtains appropriate joint angles
- Uses Interbotix `set_joint_positions` to guide the Manipulator to object.
    - Goes to safe position (X,Y same as object but at a higher Z)
    - Releases Gripper
    - Comes back to sleep position
- Does not perform drop unless performed a grasp before.
