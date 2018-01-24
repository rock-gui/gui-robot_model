Vizkit::UiLoader.register_3d_plugin('RobotVisualization', 'robot_model', 'RobotVisualization')
Vizkit::UiLoader.register_3d_plugin_for('RobotVisualization', "/base/samples/Joints", :updateData )
Vizkit::UiLoader.register_3d_plugin_for('RobotVisualization', "/base/samples/RigidBodyState", :updateRBS )
