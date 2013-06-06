Vizkit::UiLoader.register_3d_plugin('RobotVisualization', 'robot_model', 'RobotVisualization')
Vizkit::UiLoader.register_3d_plugin_for('RobotVisualization', "/workspace/robot_model/JointState", :updateData )
