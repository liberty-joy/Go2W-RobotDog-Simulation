#!/usr/bin/python3
"""Launch rqt_plot with a usable window and retry DDS topic discovery."""
import time

from python_qt_binding.QtCore import QTimer
from rqt_plot.plot_widget import PlotWidget
from rqt_plot.rosplot import ROSData, RosPlotException


def receive_with_wall_time(self, msg):
    # Twist has no header; Odometry uses simulation timestamps. Align both by
    # reception time for this live comparison, without changing ROS messages.
    with self.lock:
        try:
            value = self._get_data(msg)
            self.buff_y.append(value)
            self.buff_x.append(time.time() - self.start_time)
        except AttributeError as exc:
            self.error = RosPlotException(str(exc))


ROSData._ros_cb = receive_with_wall_time

TOPICS = ("/go2w/cmd_vel/linear/x", "/go2w/odom/twist/twist/linear/x")
original_init = PlotWidget.__init__


def init_with_discovery(self, *args, **kwargs):
    original_init(self, *args, **kwargs)
    self._go2w_discovery_timer = QTimer(self)

    def discover():
        for topic in TOPICS:
            if topic not in self._rosdata:
                self.add_topic(topic)
        if all(topic in self._rosdata for topic in TOPICS):
            self._go2w_discovery_timer.stop()
            print("Go2W: both velocity curves subscribed", flush=True)

    self._go2w_discovery_timer.timeout.connect(discover)
    self._go2w_discovery_timer.start(1000)
    QTimer.singleShot(500, lambda: self.window().resize(1100, 720))


PlotWidget.__init__ = init_with_discovery

if __name__ == "__main__":
    from rqt_plot.main import main

    main()
