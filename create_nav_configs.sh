#!/bin/bash

# Create config directory if it doesn't exist
mkdir -p /home/Dojo/Dojo/src/robot_navigation/config

# Create map_server_params.yaml
cat > /home/Dojo/Dojo/src/robot_navigation/config/map_server_params.yaml << 'EOL'
/**:
  ros__parameters:
    use_sim_time: False
    yaml_filename: map.yaml
    topic_name: map
    frame_id: map
EOL

# Create nav2_params.yaml
cat > /home/Dojo/Dojo/src/robot_navigation/config/nav2_params.yaml << 'EOL'
/**:
  ros__parameters:
    use_sim_time: False
    autostart: True
    bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    global_frame: map
    robot_base_frame: base_link
    robot_radius: 0.22
    transform_tolerance: 0.2
    planner_server:
      ros__parameters:
        expected_planner_frequency: 20.0
    controller_server:
      ros__parameters:
        controller_frequency: 20.0
EOL

# Create other required config files with minimal content
for file in localization_params.yaml bt_navigator_params.yaml planner_params.yaml controller_params.yaml costmap_common_params.yaml global_costmap_params.yaml local_costmap_params.yaml; do
    cat > "/home/Dojo/Dojo/src/robot_navigation/config/$file" << 'EOL'
/**:
  ros__parameters:
    use_sim_time: False
EOL
done

echo "Created navigation configuration files"
