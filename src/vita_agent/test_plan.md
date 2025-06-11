# VITA Agent Test Plan

## 1. Objective

The objective of this test plan is to verify the functionality, reliability, and performance of the `vita_agent` ROS package. This plan outlines the testing strategy and specific test cases designed to ensure that all components of the `vita_agent` work correctly, both individually and as an integrated system.

## 2. Scope

### In Scope

*   **State Machine Logic:** Testing the state transitions in `zero_shot_vlm_planner.py` (IDLE, WAITING_FOR_VLM, EXECUTING_ACTION).
*   **Text Command Handling:** Verifying that the system correctly processes commands from the `/test/command` topic.
*   **Sensor Data Processing:**
    *   **Lidar:** Correct parsing of `PointCloud2` messages and generation of the local occupancy grid.
    *   **Odometry:** Accurate processing of odometry data for ego-motion tracking.
    *   **Video:** Successful decoding of H.264 compressed video streams.
    *   **UWB:** Correct interpretation of UWB messages for target localization.
*   **VLM Interaction:**
    *   Ensuring the `VlmClient` correctly constructs and sends requests to the VLM server.
    *   Verifying the handling of successful responses and error conditions.
*   **Path Planning:**
    *   Testing the `PathPlanner`'s ability to generate collision-free trajectories.
    *   Verifying the handling of scenarios where no valid path can be found.
*   **Motion Control:**
    *   Ensuring the `MotionController` accurately translates planned trajectories into velocity commands.
    *   Testing the robot's safety features, such as stopping or backing up when obstacles are detected or when sensor data times out.
*   **Integration:** Testing the seamless interaction between all the above components.

### Out of Scope

*   **VLM Server:** The VLM server itself will be treated as a black box. We will mock its responses to test the `VlmClient`.
*   **Low-Level Hardware Drivers:** Testing of the drivers for the camera, lidar, UWB, and robot base is not included.
*   **ROS 2 Framework:** We will assume that the underlying ROS 2 communication infrastructure is reliable.
*   **Network Stability:** Testing related to network latency or instability between the agent and the VLM server is not covered.

## 3. Test Strategy

A combination of unit, integration, and end-to-end (E2E) testing will be employed to ensure comprehensive coverage.

*   **Unit Tests:** Each module (`SensorProcessor`, `VlmClient`, `PathPlanner`, `MotionController`) will be tested in isolation to verify its specific functionality. This will involve using mock data and mock dependencies.
*   **Integration Tests:** We will test the interaction between different components, focusing on the data flow. For example, testing the flow from sensor processing to path planning.
*   **End-to-End Tests:** These tests will simulate a complete user scenario, from issuing a text command to the robot completing the action. This will be performed in a simulated environment or with the actual hardware.

## 4. Test Scenarios

### Component: Text Command Publisher (`text_publisher.py`)

*   **Test Case 1.1:** Verify that a message typed into the console is published to the `/test/command` topic.
*   **Test Case 1.2:** Verify that typing "exit" or "quit" gracefully shuts down the node.

### Component: Sensor Processing (`processors/sensor_processor.py`)

*   **Test Case 2.1 (Lidar):** Provide a sample `PointCloud2` message and verify that a valid occupancy grid is generated.
*   **Test Case 2.2 (Lidar):** Provide a `PointCloud2` message with points outside the defined `pc_range` and verify they are correctly filtered out.
*   **Test Case 2.3 (Odometry):** Provide a sequence of `Odometry` messages and verify that the `Tr_ego2init` transform is updated correctly.
*   **Test Case 2.4 (Video):** Provide a `CompressedVideo` message with H.264 data and verify that it is successfully decoded into an OpenCV image.
*   **Test Case 2.5 (UWB):** Provide a `UWB` message and verify the `uwb_target` is calculated correctly.

### Component: VLM Client (`clients/vlm_client.py`)

*   **Test Case 3.1:** Mock a successful VLM server response and verify that the `get_action` method returns the correct coordinates.
*   **Test Case 3.2:** Mock a VLM server timeout and verify that the client handles the exception gracefully and returns `None`.
*   **Test Case 3.3:** Mock a VLM server error (e.g., 500 status code) and verify the client handles it correctly.
*   **Test Case 3.4:** Provide an invalid image and verify that the client can handle the `convert_image_to_base64` failure.

### Component: Path Planning (`planning/path_planner.py`)

*   **Test Case 4.1:** Given a goal position and an empty occupancy map, verify that a straight-line trajectory is generated.
*   **Test Case 4.2:** Given a goal position and an occupancy map with obstacles, verify that a collision-free path is generated.
*   **Test Case 4.3:** Given a goal that is completely blocked by obstacles, verify that the `stop_flag` is set to `True`.
*   **Test Case 4.4:** Verify that the planned `v_traj` and `w_traj` values are within the `max_linear_speed` and `max_angular_speed` limits.

### Component: Motion Control (`motion/motion_controller.py`)

*   **Test Case 5.1:** Verify that calling `move_to_target` with a valid trajectory results in `Twist` messages being published on the `/vel_cmd` topic.
*   **Test Case 5.2:** Verify that calling `stop()` sets the linear and angular velocities to zero.
*   **Test Case 5.3:** Verify that calling `go_back()` sets a negative linear velocity.
*   **Test Case 5.4:** Simulate a UWB timeout and verify that the `stop()` method is called.
*   **Test Case 5.5:** Simulate a PointCloud timeout and verify that the `stop()` method is called.
*   **Test Case 5.6:** Verify that as the robot approaches the target (`distance < stop_distance`), its velocity decreases.

### Integration: State Machine (`zero_shot_vlm_planner.py`)

*   **Test Case 6.1 (IDLE -> WAITING_FOR_VLM):**
    1. Start in `IDLE` state.
    2. Publish a text command to `/test/command`.
    3. Verify the state transitions to `WAITING_FOR_VLM`.
*   **Test Case 6.2 (WAITING_FOR_VLM -> EXECUTING_ACTION):**
    1. Be in `WAITING_FOR_VLM` state with a valid text command.
    2. Publish an image message.
    3. Mock a successful VLM response.
    4. Verify the state transitions to `EXECUTING_ACTION` and `action_target_position` is set.
*   **Test Case 6.3 (WAITING_FOR_VLM -> IDLE):**
    1. Be in `WAITING_FOR_VLM` state.
    2. Mock a failed VLM response.
    3. Verify the state transitions back to `IDLE`.
*   **Test Case 6.4 (EXECUTING_ACTION -> IDLE):**
    1. Be in `EXECUTING_ACTION` state.
    2. Publish sensor data that makes the robot reach its target (`distance_to_target < stop_distance` for multiple cycles).
    3. Verify the state transitions to `IDLE`.

### End-to-End Scenario

*   **Test Case 7.1: "Come Here" Command**
    1.  **Start:** The robot is in the `IDLE` state.
    2.  **User Input:** The user runs `text_publisher.py` and types "Come Here".
    3.  **State Transition:** The `TaskLogicNode` receives the command and transitions to `WAITING_FOR_VLM`.
    4.  **VLM Call:** The node receives the next video frame and UWB data, calls the VLM server with the image and command.
    5.  **VLM Response:** The VLM (mocked) returns target coordinates.
    6.  **State Transition:** The node receives the coordinates, sets the `action_target_position`, and transitions to `EXECUTING_ACTION`.
    7.  **Path Planning & Motion:** In a loop, the node:
        *   Processes Lidar data to create an occupancy map.
        *   Calls the `PathPlanner` to get a trajectory.
        *   Calls the `MotionController` to move the robot.
    8.  **Obstacle Avoidance:** While moving, an obstacle is placed in the robot's path. The robot should detect it and the `PathPlanner` should issue a `stop_flag` or a new path. The `MotionController` should react accordingly (e.g., `go_back()`).
    9.  **Arrival:** The robot navigates to within `stop_distance` of the target.
    10. **Task Completion:** After `arrive_vlm_count` reaches the threshold, the `MotionController` stops the robot, and the state machine returns to `IDLE`.
    11. **Final Outcome:** The robot successfully navigates to the target location while avoiding obstacles and then stops.

## 5. Success Criteria

A test case is considered passed if:
*   All functional requirements are met.
*   The actual output matches the expected output.
*   The system remains stable and does not crash or enter an unrecoverable state.
*   All state transitions occur as expected.
*   No errors or exceptions are logged, unless they are part of an expected error-handling test.