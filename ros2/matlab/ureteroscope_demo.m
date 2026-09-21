% Minimal MATLAB ROS Toolbox demo against the katzlab bridge.
%
% Prereqs: ROS Toolbox installed, ROS_DOMAIN_ID matching every other
% machine on the graph (see ros2/README.md section 4), and
% katzlab_bridge already running (bridge_node, real or dry_run).
%
% Not run against a real MATLAB/ROS Toolbox install -- written against the
% documented ROS Toolbox ros2* API. If a call here doesn't match your
% installed Toolbox version, `doc ros2subscriber` / `doc ros2publisher` are
% the source of truth, not this file.

clear node sub pub;

node = ros2node("/katzlab_matlab_demo");

% Sanity check: the bridge should already be visible on the graph. If this
% comes back empty, it's a ROS_DOMAIN_ID mismatch far more often than
% anything actually wrong -- see ros2/README.md section 4.
disp("nodes on the graph:");
disp(ros2("node", "list"));

% -- 1. Watch joint state -------------------------------------------------

sub = ros2subscriber(node, "/ureteroscope/joint_states", "sensor_msgs/JointState", ...
    @jointStateCallback);

function jointStateCallback(msg)
    % name/position line up index-for-index: linear (m), rotation (rad),
    % flexion (rad) -- see ros2/README.md for the unit conversion.
    fprintf("%s\n", strjoin(compose("%s=%.4f", string(msg.name), msg.position), "  "));
end

% -- 2. Command a small linear nudge --------------------------------------
%
% cmd_velocity repurposes geometry_msgs/Twist: linear.x = linear m/s,
% angular.z = rotation rad/s, angular.y = flexion rad/s. See
% ros2/README.md for why (avoiding a whole custom-interface package for
% three floats this early).

pub = ros2publisher(node, "/ureteroscope/cmd_velocity", "geometry_msgs/Twist");
msg = ros2message(pub);
msg.linear.x = 0.001;     % 1 mm/s, well inside max_rate_mm_s = 2.5

disp("publishing a 1 mm/s linear nudge for 2 seconds...");
rate = ros2rate(node, 10);
stopTime = tic;
while toc(stopTime) < 2.0
    send(pub, msg);
    waitfor(rate);
end

% Stop -- publish zero rather than just letting the script exit. The
% bridge's own staleness fail-safe (CMD_VELOCITY_STALE_S in
% ros_input_source.py) also zeroes it out on its own after ~0.5s of
% silence, but there is no reason to rely on that here.
msg.linear.x = 0.0;
send(pub, msg);
disp("done.");

% -- 3. Home / e-stop, if ever needed from MATLAB ------------------------
%
% homeClient = ros2svcclient(node, "/ureteroscope/home", "std_srvs/Trigger");
% call(homeClient, ros2message(homeClient), "Timeout", 5);
%
% estopClient = ros2svcclient(node, "/ureteroscope/estop", "std_srvs/Trigger");
% call(estopClient, ros2message(estopClient), "Timeout", 5);
