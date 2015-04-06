require 'vizkit'

view3d = Vizkit.vizkit3d_widget

#model_file = File.join(Dir.pwd, 'test_data', 'spacebot_arm', 'spacebot_arm.urdf')
model_file = File.join(Dir.pwd, 'test_data', 'simple_arm', 'model.sdf')

view3d = Vizkit.vizkit3d_widget

vis_gui = Vizkit.default_loader.RobotVisualization
vis_gui.modelFile = model_file.dup
vis_gui.jointsSize = 0.02

override_vel_limits=0
only_positive=true
no_effort=true
no_velocity=true

ctrl_gui = Vizkit.default_loader.ControlUi
ctrl_gui.configureUi(override_vel_limits, only_positive, no_effort, no_velocity)
#ctrl_gui.initFromURDF(model_file.dup)
ctrl_gui.initFromSDF(model_file.dup)
ctrl_gui.connect(SIGNAL('sendSignal()')) do 
    sample = ctrl_gui.getJoints()
    vis_gui.updateData(sample)
end

main = Qt::Widget.new
layout = Qt::VBoxLayout.new(main)
layout.add_widget(view3d)
layout.add_widget(ctrl_gui)
main.show


Vizkit.exec