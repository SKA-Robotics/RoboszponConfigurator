import sys
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QMessageBox,
    QVBoxLayout,
    QWidget,
    QListWidget,
    QListWidgetItem,
    QComboBox,
    QDialog,
    QLabel,
    QDoubleSpinBox,
    QPushButton,
    QHBoxLayout,
    QGridLayout,
)
from PyQt5.QtCore import QStringListModel, QModelIndex, QTimer, Qt
from PyQt5.QtGui import QPixmap
from PyQt5 import uic

import roboszpon_lib
import can, time, os
import pyqtgraph as pg

CONNECTION_TIMEOUT = 1.0
MAX_PLOT_SAMPLES = 10000


class ParameterConfigurationDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Parameter Configuration")
        self.layout = QGridLayout(self)
        self.parameters = {}

    def add_parameter(self, parameter_name, value, update_callback, deactivate_callback):
        if parameter_name in self.parameters:
            return

        row = self.layout.rowCount()

        label = QLabel(parameter_name)
        spinbox = QDoubleSpinBox()
        spinbox.setValue(value)
        spinbox.setDecimals(4)
        spinbox.setRange(-1000000, 1000000)  # Adjust range as needed

        update_button = QPushButton("Update")
        update_button.clicked.connect(lambda: update_callback(parameter_name, spinbox.value()))

        deactivate_button = QPushButton("Deactivate")
        deactivate_button.clicked.connect(lambda: deactivate_callback(parameter_name))

        self.layout.addWidget(label, row, 0)
        self.layout.addWidget(spinbox, row, 1)
        self.layout.addWidget(update_button, row, 2)
        self.layout.addWidget(deactivate_button, row, 3)

        self.parameters[parameter_name] = (label, spinbox, update_button, deactivate_button)

    def remove_parameter(self, parameter_name):
        if parameter_name not in self.parameters:
            return

        widgets = self.parameters.pop(parameter_name)
        for widget in widgets:
            widget.deleteLater()

        self.update_layout()

    def update_layout(self):
        for i in reversed(range(self.layout.count())):
            widget = self.layout.itemAt(i).widget()
            self.layout.removeWidget(widget)

        row = 0
        for param_name, widgets in self.parameters.items():
            for col, widget in enumerate(widgets):
                self.layout.addWidget(widget, row, col)
            row += 1

    def is_empty(self):
        return not bool(self.parameters)


class Signal:
    def __init__(self):
        self.values = []
        self.timestamps = []

    def update(self, value, timestamp):
        if len(self.values) == MAX_PLOT_SAMPLES:
            self.values.pop(0)
        self.values.append(value)
        if len(self.timestamps) == MAX_PLOT_SAMPLES:
            self.timestamps.pop(0)
        self.timestamps.append(timestamp)


class Roboszpon:
    def __init__(self):
        self.duty = Signal()
        self.current = Signal()
        self.velocity = Signal()
        self.position = Signal()
        self.temperature = Signal()

    node_id: int
    mode: str
    flags: int
    timestamp: float


class MyLittleRoboszponSuite(QMainWindow):
    def __init__(self):
        super().__init__()
        path = os.path.dirname(os.path.realpath(__file__))
        uic.loadUi(os.path.join(path, "app.ui"), self)
        self.logoLabel.setPixmap(QPixmap(os.path.join(path, "assets/text1.png")))
        self.startTimestamp = time.time()

        self.roboszpon = None
        self.armed = False

        # Zmiana QStringListModel na QComboBox
        self.comboBoxDevices = self.findChild(QComboBox, "comboBoxDevices")
        self.comboBoxDevices.currentIndexChanged.connect(self.comboBoxDeviceChanged)

        self.deviceIds = []
        self.devices = {}

        self.init_parameters_tabs()

        self.armButton.clicked.connect(self.armButtonClicked)
        self.dutyButton.clicked.connect(self.dutyButtonClicked)
        self.velocityButton.clicked.connect(self.velocityButtonClicked)
        self.positionButton.clicked.connect(self.positionButtonClicked)
        self.stopAllButton.clicked.connect(self.stopAllButtonClicked)
        self.actionCommit_configuration.triggered.connect(self.commitConfiguration)
        self.actionRestore_configuration.triggered.connect(self.restoreConfiguration)
        self.actionFactory_settings.triggered.connect(self.factoryConfiguration)
        self.actionSoftware_reset.triggered.connect(self.softwareReset)

        self.init_plot()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(100)

        try:
            self.canbus = can.interface.Bus("can0", interface="socketcan")
            self.can_notifier = can.Notifier(self.canbus, [self])
        except Exception as e:
            print(f"Couldn't start can: {e}")

        # Okno dialogowe dla konfiguracji parametrów
        self.parameterDialog = None

    def __del__(self):
        self.canbus.shutdown()

    def __call__(self, message):
        self.on_message_received(message)

    def tick(self):
        if self.roboszpon is not None:
            self.updatePlot()
            if (
                self.devices[self.roboszpon].timestamp + CONNECTION_TIMEOUT
                < time.time()
            ):
                self.connectionLabel.setText("Connection: LOST")
                self.stateLabel.setText("")
                self.flagsLabel.setText("")

    def on_message_received(self, message: can.message.Message):
        if message.is_extended_id or message.dlc != 8:
            print("Received invalid CAN message")
        msg = roboszpon_lib.decode_message(
            message.arbitration_id, int.from_bytes(message.data, "big")
        )
        if msg["message_id"] == roboszpon_lib.MSG_STATUS_REPORT:
            node_id = msg["node_id"]
            if node_id not in self.deviceIds:
                self.addConnectedDevice(node_id)
            roboszpon = self.devices[msg["node_id"]]
            roboszpon.mode = msg["mode"]
            roboszpon.flags = msg["flags"]
            roboszpon.timestamp = message.timestamp
            roboszpon.temperature.update(
                msg["temperature"], message.timestamp - self.startTimestamp
            )
            if msg["node_id"] == self.roboszpon:
                self.connectionLabel.setText("Connection: OK")
                self.stateLabel.setText(
                    f"Operation state: {roboszpon_lib.ROBOSZPON_MODES[self.devices[self.roboszpon].mode]}"
                )
                self.flagsLabel.setText(
                    f"Flags: {(bin(self.devices[self.roboszpon].flags + (1 << 17)))[4:]}"
                )
        if msg["message_id"] == roboszpon_lib.MSG_AXIS_REPORT:
            node_id = msg["node_id"]
            if node_id in self.deviceIds:
                self.devices[node_id].position.update(
                    msg["position"], message.timestamp - self.startTimestamp
                )
                self.devices[node_id].velocity.update(
                    msg["velocity"], message.timestamp - self.startTimestamp
                )
        if msg["message_id"] == roboszpon_lib.MSG_MOTOR_REPORT:
            node_id = msg["node_id"]
            if node_id in self.deviceIds:
                self.devices[node_id].current.update(
                    msg["current"], message.timestamp - self.startTimestamp
                )
                self.devices[node_id].duty.update(
                    msg["duty"], message.timestamp - self.startTimestamp
                )

    def addConnectedDevice(self, node_id):
        device_name = f"Roboszpon 0x{node_id:02x}"
        if device_name not in [self.comboBoxDevices.itemText(i) for i in range(self.comboBoxDevices.count())]:
            self.comboBoxDevices.addItem(device_name)
            self.deviceIds.append(node_id)
            self.devices[node_id] = Roboszpon()
            self.devices[node_id].node_id = node_id

    def removeConnectedDevice(self, node_id):
        if node_id not in self.deviceIds:
            raise KeyError("can't remove node. there is no such node")
        index = self.deviceIds.index(node_id)
        self.comboBoxDevices.removeItem(index)
        self.deviceIds.remove(node_id)
        print(self.deviceIds)

    def selectDevice(self, node_id):
        self.roboszpon = node_id
        self.armed = self.devices[node_id].mode != roboszpon_lib.ROBOSZPON_MODE_STOPPED
        if self.armed:
            self.arm()
        else:
            self.disarm()
        self.connectionLabel.setText("Connection: OK")
        self.stateLabel.setText(
            f"Operation state: {roboszpon_lib.ROBOSZPON_MODES[self.devices[self.roboszpon].mode]}"
        )
        self.flagsLabel.setText(
            f"Flags: {(bin(self.devices[self.roboszpon].flags + (1 << 17)))[3:]}"
        )
        print(f"Selected roboszpon: {node_id}")

    def arm(self):
        self.armButton.setText("Disarm")
        self.parameterConfigurationGroup.setEnabled(False)
        self.setpointGroup.setEnabled(True)

    def disarm(self):
        self.armButton.setText("Arm")
        self.parameterConfigurationGroup.setEnabled(True)
        self.setpointGroup.setEnabled(False)

    def comboBoxDeviceChanged(self, index):
        if index >= 0 and index < len(self.deviceIds):
            node_id = self.deviceIds[index]
            self.selectDevice(node_id)

    def armButtonClicked(self):
        self.armed = not self.armed
        if self.armed:
            roboszpon_lib.disarm(self.canbus, self.roboszpon)
            self.disarm()
        else:
            roboszpon_lib.arm(self.canbus, self.roboszpon)
            self.arm()

    def stopAllButtonClicked(self):
        roboszpon_lib.emergency_stop(self.canbus)
        self.armed = False
        self.disarm()

    def dutyButtonClicked(self):
        roboszpon_lib.send_duty_command(
            self.canbus, self.roboszpon, self.dutySpinBox.value()
        )

    def velocityButtonClicked(self):
        roboszpon_lib.send_velocity_command(
            self.canbus, self.roboszpon, self.velocitySpinBox.value()
        )

    def positionButtonClicked(self):
        roboszpon_lib.send_position_command(
            self.canbus, self.roboszpon, self.positionSpinBox.value()
        )

    def parameterItemDoubleClicked(self, item):
        parameter_name = item.text()
        self.activate_parameter_in_dialog(parameter_name)

    def activate_parameter_in_dialog(self, parameter_name):
        if self.parameterDialog is None:
            self.parameterDialog = ParameterConfigurationDialog(self)
        
        def update_callback(param_name, value):
            self.updateParameterValueInDevice(param_name, value)

        def deactivate_callback(param_name):
            self.parameterDialog.remove_parameter(param_name)
            if self.parameterDialog.is_empty():
                self.parameterDialog.close()
                self.parameterDialog = None

        self.updateParameterValue(parameter_name)
        value = self.get_current_parameter_value(parameter_name)  # Pobierz aktualną wartość

        self.parameterDialog.add_parameter(
            parameter_name, value, update_callback, deactivate_callback
        )
        
        self.parameterDialog.show()

    def updateParameterValueInDevice(self, parameter_name, value):
        if self.roboszpon is None:
            return
        if self.devices[self.roboszpon].mode != roboszpon_lib.ROBOSZPON_MODE_STOPPED:
            print("Can't configure running roboszpon")
            return
        parameter_id = roboszpon_lib.ROBOSZPON_PARAMETERS[parameter_name]
        roboszpon_lib.send_parameter_write(
            self.canbus, self.roboszpon, parameter_id, value
        )
        print(f"Updated {parameter_name} to {value}")

    def get_current_parameter_value(self, parameter_name):
        #TODO: Implement this
        if self.roboszpon is None:
            return 0.0
        return 0.0

    def commitConfiguration(self):
        roboszpon_lib.send_action_request(
            self.canbus, self.roboszpon, roboszpon_lib.ACTION_COMMIT_CONFIG
        )

    def restoreConfiguration(self):
        roboszpon_lib.send_action_request(
            self.canbus, self.roboszpon, roboszpon_lib.ACTION_RESTORE_CONFIG
        )

    def factoryConfiguration(self):
        roboszpon_lib.send_action_request(
            self.canbus, self.roboszpon, roboszpon_lib.ACTION_SET_FACTORY_CONFIG
        )

    def softwareReset(self):
        roboszpon_lib.send_action_request(
            self.canbus, self.roboszpon, roboszpon_lib.ACTION_SOFTWARE_RESET
        )

    def init_plot(self):
        self.plot = pg.PlotWidget()
        self.plot.addLegend(offset=(-1, 1))
        layout = QVBoxLayout(self.plotView)
        layout.addWidget(self.plot)
        self.temperature_curve = self.plot.plot(
            [], [], pen="#7E2F8E", name="Temperature"
        )
        self.duty_curve = self.plot.plot([], [], pen="#77AC30", name="Duty")
        self.current_curve = self.plot.plot([], [], pen="#EDB120", name="Current")
        self.velocity_curve = self.plot.plot([], [], pen="#D95319", name="Velocity")
        self.position_curve = self.plot.plot([], [], pen="#0072BD", name="Position")

    def updatePlot(self):
        signal = self.devices[self.roboszpon].position
        self.position_curve.setData(signal.timestamps, signal.values)
        signal = self.devices[self.roboszpon].velocity
        self.velocity_curve.setData(signal.timestamps, signal.values)
        signal = self.devices[self.roboszpon].current
        self.current_curve.setData(signal.timestamps, signal.values)
        signal = self.devices[self.roboszpon].duty
        self.duty_curve.setData(signal.timestamps, signal.values)
        signal = self.devices[self.roboszpon].temperature
        self.temperature_curve.setData(signal.timestamps, signal.values)
    def updateParameterValue(self, parameter_name):
        def callback(value):
            self.parameterSpinBox.setValue(value)
            self.oldParameterValue = value

        if self.roboszpon is None:
            return
        if self.devices[self.roboszpon].mode != roboszpon_lib.ROBOSZPON_MODE_STOPPED:
            print("Can't configure running roboszpon")
            return
        roboszpon_lib.read_parameter_callback(
            self.canbus,
            self.can_notifier,
            self.devices[self.roboszpon].node_id,
            roboszpon_lib.ROBOSZPON_PARAMETERS[parameter_name],
            callback,
        )
    def init_parameters_tabs(self):
        # Funkcja dodająca parametry do zakładek
        def add_parameters_to_tab(group_name, parameters):
            tab = QWidget()
            layout = QVBoxLayout(tab)

            list_widget = QListWidget()
            for param_name in parameters:
                item_widget = QWidget()
                item_layout = QHBoxLayout(item_widget)
                item_layout.setContentsMargins(0, 0, 0, 0)
                label = QLabel(param_name)
                activate_button = QPushButton("Aktywuj")
                activate_button.clicked.connect(lambda _, pn=param_name: self.activate_parameter_in_dialog(pn))
                item_layout.addWidget(label)
                item_layout.addWidget(activate_button)
                list_widget_item = QListWidgetItem(list_widget)
                list_widget_item.setSizeHint(item_widget.sizeHint())
                list_widget.setItemWidget(list_widget_item, item_widget)

            list_widget.itemDoubleClicked.connect(self.parameterItemDoubleClicked)

            layout.addWidget(list_widget)
            self.tabWidgetParameters.addTab(tab, group_name)

        add_parameters_to_tab("Temperature", roboszpon_lib.TEMPERATURE_PARAMETERS)
        add_parameters_to_tab("PPID", roboszpon_lib.PPID_PARAMETERS)
        add_parameters_to_tab("VPID", roboszpon_lib.VPID_PARAMETERS)
        add_parameters_to_tab("CPID", roboszpon_lib.CPID_PARAMETERS)
        add_parameters_to_tab("Encoder", roboszpon_lib.ENCODER_PARAMETERS)
        add_parameters_to_tab("AXIS", roboszpon_lib.AXIS_PARAMETERS)
        add_parameters_to_tab("Current", roboszpon_lib.CURRENT_PARAMETERS)
        add_parameters_to_tab("IIR", roboszpon_lib.IIR_PARAMETERS)
        add_parameters_to_tab("Duty", roboszpon_lib.DUTY_PARAMETERS)
        add_parameters_to_tab("Position", roboszpon_lib.POSITION_PARAMETERS)
        add_parameters_to_tab("Velocity", roboszpon_lib.VELOCITY_PARAMETERS)
        add_parameters_to_tab("Report", roboszpon_lib.REPORTING_PARAMETERS)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    ex = MyLittleRoboszponSuite()
    ex.show()
    sys.exit(app.exec_())
