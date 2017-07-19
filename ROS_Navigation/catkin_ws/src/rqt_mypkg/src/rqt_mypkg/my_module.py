import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from mavros_msgs.msg import RCIn

class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        self._publisher = None

        self._widget = QWidget()

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self._widget.topic_line_edit.textChanged.connect(self._on_topic_changed)
        self._widget.stop_push_button.pressed.connect(self._on_stop_pressed)

        self._widget.throttle_slider.valueChanged.connect(self._on_throttle_slider_changed)
        self._widget.pitch_slider.valueChanged.connect(self._on_pitch_slider_changed)
        self._widget.roll_slider.valueChanged.connect(self._on_roll_slider_changed)
        self._widget.yaw_slider.valueChanged.connect(self._on_yaw_slider_changed)


    def _on_topic_changed(self, topic):
        topic = str(topic)
        self._unregister_publisher()
        try:
            self._publisher = rospy.Publisher(topic, RCIn, queue_size=10)
        except TypeError:
            self._publisher = rospy.Publisher(topic, RCIn)

    def _on_stop_pressed(self):
        self._widget.throttle_slider.setValue(0)
        self._widget.pitch_slider.setValue(0)
        self._widget.roll_slider.setValue(0)
        self._widget.yaw_slider.setValue(0)

    def _on_throttle_slider_changed(self):
        self._widget.current_throttle_label.setText('%d' % (self._widget.throttle_slider.value()))
        self._on_parameter_changed()

    def _on_pitch_slider_changed(self):
        self._widget.current_pitch_label.setText('%d' % (self._widget.pitch_slider.value()))
        self._on_parameter_changed()

    def _on_roll_slider_changed(self):
        self._widget.current_roll_label.setText('%d' % (self._widget.roll_slider.value()))
        self._on_parameter_changed()

    def _on_yaw_slider_changed(self):
        self._widget.current_yaw_label.setText('%d' % (self._widget.yaw_slider.value()))
        self._on_parameter_changed()

    def _unregister_publisher(self):
        if self._publisher is not None:
            self._publisher.unregister()
            self._publisher = None

    def _on_parameter_changed(self):
        self._send_rc(self._widget.throttle_slider.value(), self._widget.pitch_slider.value(), self._widget.roll_slider.value(), self._widget.yaw_slider.value())

    def _send_rc(self, throttle, pitch, roll, yaw):
        if self._publisher is None:
            return
        rcin = RCIn()
        rcin.channels[0] = roll;
        rcin.channels[1] = pitch;
        rcin.channels[2] = throttle;
        rcin.channels[3] = yaw;
        for i in range(4, 9):
            rcin.channels[0] = 65535

        self._publisher.publish(rcin)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
