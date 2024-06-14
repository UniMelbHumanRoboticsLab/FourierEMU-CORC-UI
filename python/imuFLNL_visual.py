from visual_lib_imu import *
from threading import Thread
import pyqtgraph as pg  # type: ignore

from pglive.kwargs import Axis
from pglive.sources.data_connector import DataConnector
from pglive.sources.live_axis import LiveAxis
from pglive.sources.live_axis_range import LiveAxisRange
from pglive.sources.live_plot import LiveLinePlot
from pglive.sources.live_plot_widget import LivePlotWidget

"""
Live plot range is used to increase plotting performance in this example.
"""

layout = pg.LayoutWidget()
layout.layout.setSpacing(0)
args = []
args2 = []
args3 = []

"""
Gravity Widget
"""
grav_x_widget = LivePlotWidget(title=f"Grav x",
                        x_range_controller=LiveAxisRange(roll_on_tick=1000))
grav_x = LiveLinePlot(pen="yellow")
grav_x_widget.addItem(grav_x)
args3.append(DataConnector(grav_x, max_points=1000))
layout.addWidget(grav_x_widget, row=0, col=2)

grav_y_widget = LivePlotWidget(title=f"Grav y",
                        x_range_controller=LiveAxisRange(roll_on_tick=1000))
grav_y = LiveLinePlot(pen="yellow")
grav_y_widget.addItem(grav_y)
args3.append(DataConnector(grav_y, max_points=1000))
layout.addWidget(grav_y_widget, row=1, col=2)

grav_z_left_axis = LiveAxis("left", axisPen="red", textPen="red")
grav_z_bottom_axis = LiveAxis("bottom", axisPen="green", textPen="green", **{Axis.TICK_FORMAT: Axis.TIME})
grav_z_widget = LivePlotWidget(title="Grav z",
                                       axisItems={'bottom': grav_z_bottom_axis, 'left': grav_z_left_axis},
                                       x_range_controller=LiveAxisRange(roll_on_tick=1000))
grav_z = LiveLinePlot(pen="yellow")
grav_z_widget.addItem(grav_z)
args3.append(DataConnector(grav_z, max_points=1000))
layout.addWidget(grav_z_widget, row=2, col=2)

"""
Linear Acceleration Widget
"""
lacc_x_widget = LivePlotWidget(title=f"Linear Acceleration x",
                        x_range_controller=LiveAxisRange(roll_on_tick=1000))
lacc_x = LiveLinePlot(pen="yellow")
lacc_x_widget.addItem(lacc_x)
args2.append(DataConnector(lacc_x, max_points=1000))
layout.addWidget(lacc_x_widget, row=0, col=1)

lacc_y_widget = LivePlotWidget(title=f"Linear Acceleration y",
                        x_range_controller=LiveAxisRange(roll_on_tick=1000))
lacc_y = LiveLinePlot(pen="yellow")
lacc_y_widget.addItem(lacc_y)
args2.append(DataConnector(lacc_y, max_points=1000))
layout.addWidget(lacc_y_widget, row=1, col=1)

lacc_z_left_axis = LiveAxis("left", axisPen="red", textPen="red")
lacc_z_bottom_axis = LiveAxis("bottom", axisPen="green", textPen="green", **{Axis.TICK_FORMAT: Axis.TIME})
lacc_z_widget = LivePlotWidget(title="Linear Acceleration z",
                                       axisItems={'bottom': lacc_z_bottom_axis, 'left': lacc_z_left_axis},
                                       x_range_controller=LiveAxisRange(roll_on_tick=1000))
lacc_z = LiveLinePlot(pen="yellow")
lacc_z_widget.addItem(lacc_z)
args2.append(DataConnector(lacc_z, max_points=1000))
layout.addWidget(lacc_z_widget, row=2, col=1)

"""
Orientation Widget
"""
ort_x_widget = LivePlotWidget(title=f"Orientation x",
                        x_range_controller=LiveAxisRange(roll_on_tick=1000))
ort_x = LiveLinePlot(pen="yellow")
ort_x_widget.addItem(ort_x)
args.append(DataConnector(ort_x, max_points=1000))
layout.addWidget(ort_x_widget, row=0, col=0)

ort_y_widget = LivePlotWidget(title=f"Orientation y",
                        x_range_controller=LiveAxisRange(roll_on_tick=1000))
ort_y = LiveLinePlot(pen="yellow")
ort_y_widget.addItem(ort_y)
args.append(DataConnector(ort_y, max_points=1000))
layout.addWidget(ort_y_widget, row=1, col=0)

ort_z_left_axis = LiveAxis("left", axisPen="red", textPen="red")
ort_z_bottom_axis = LiveAxis("bottom", axisPen="green", textPen="green", **{Axis.TICK_FORMAT: Axis.TIME})
ort_z_widget = LivePlotWidget(title="Orientation z",
                                       axisItems={'bottom': ort_z_bottom_axis, 'left': ort_z_left_axis},
                                       x_range_controller=LiveAxisRange(roll_on_tick=1000))
ort_z = LiveLinePlot(pen="yellow")
ort_z_widget.addItem(ort_z)
args.append(DataConnector(ort_z, max_points=1000))
layout.addWidget(ort_z_widget, row=2, col=0)

# MAIN RUNNING CODE: 
try:
    serialInst.open()
except Exception as error:
    print(error)
    print("COM busy or accelerometer not connected")
else:
    print("can I connect to FLNL?")
    while True: #client.Connected:
        layout.show()
        # Thread(target=sample, args=args, kwargs={"visual_val":"Orient:"}).start()
        Thread(target=sample, args=args2, kwargs={"visual_val":"Linear:"}).start()
        # Thread(target=sample, args=args3, kwargs={"visual_val":"Gravit:"}).start()
        app.exec()
        print("App Finished\n")
        stop()
        break