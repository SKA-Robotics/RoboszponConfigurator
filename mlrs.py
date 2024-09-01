import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox, QVBoxLayout, QFileDialog, QDialog, QDialogButtonBox, QLabel
from PyQt5.QtCore import QStringListModel, QModelIndex, QTimer
from PyQt5.QtGui import QPixmap
from PyQt5 import uic

import roboszpon_lib
import can, time, os, yaml
import pyqtgraph as pg

import asyncio
from unittest.mock import AsyncMock, Mock, patch

CONNECTION_TIMEOUT = 1.0
MAX_PLOT_SAMPLES = 10000


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
        self.current_file_path = None

        self.deviceList = QStringListModel()
        self.deviceIds = []
        self.devices = {}
        self.deviceListView.setModel(self.deviceList)
        self.deviceListView.clicked.connect(self.deviceListClicked)

        self.parameterComboBox.addItems(roboszpon_lib.ROBOSZPON_PARAMETERS.keys())
        self.parameterComboBox.currentTextChanged.connect(self.parameterComboBoxChanged)
        self.oldParameterValue = self.parameterSpinBox.value()

        self.armButton.clicked.connect(self.armButtonClicked)
        self.dutyButton.clicked.connect(self.dutyButtonClicked)
        self.velocityButton.clicked.connect(self.velocityButtonClicked)
        self.positionButton.clicked.connect(self.positionButtonClicked)
        self.stopAllButton.clicked.connect(self.stopAllButtonClicked)
        self.parameterUpdateButton.clicked.connect(self.updateParameterButtonClicked)
        self.actionCommit_configuration.triggered.connect(self.commitConfiguration)
        self.actionRestore_configuration.triggered.connect(self.restoreConfiguration)
        self.actionFactory_settings.triggered.connect(self.factoryConfiguration)
        self.actionSoftware_reset.triggered.connect(self.softwareReset)

        self.actionCommit_configuratio.triggered.connect(self.saveButtonClicked)
        self.actionRestore_configuratio.triggered.connect(self.saveAsButtonClicked)
        self.actionFactory_setting.triggered.connect(self.openButtonClicked)

        self.init_plot()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(100)

        # try:
        #     self.canbus = can.interface.Bus("can0", interface="socketcan")
        #     self.can_notifier = can.Notifier(self.canbus, [self])
        # except Exception as e:
        #     print(f"Couldn't start can: {e}")

        self.canbus = AsyncMock()
        self.can_notifier = Mock()

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
        self.deviceList.insertRow(self.deviceList.rowCount())
        index = self.deviceList.index(self.deviceList.rowCount() - 1)
        self.deviceList.setData(index, f"Roboszpon 0x{node_id:02x}", 0)
        self.deviceIds.append(node_id)
        self.devices[node_id] = Roboszpon()
        self.devices[node_id].node_id = node_id

    def removeConnectedDevice(self, node_id):
        if node_id not in self.deviceIds:
            raise KeyError("can't remove node. there is no such node")
        index = self.deviceIds.index(node_id)
        self.deviceList.removeRow(index)
        self.deviceIds.remove(node_id)
        print(self.deviceIds)

    def selectDevice(self, node_id):
        self.roboszpon = node_id
        self.armed = self.devices[node_id].mode != roboszpon_lib.ROBOSZPON_MODE_STOPPED
        if self.armed:
            self.arm()
        else:
            self.disarm()
            self.updateParameterValue(self.parameterComboBox.currentText())
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

    def deviceListClicked(self, index: QModelIndex):
        self.selectDevice(self.deviceIds[index.row()])

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

    def saveButtonClicked(self):
        if self.current_file_path:
            self.saveParameters(self.current_file_path)
        else:
            self.saveAsButtonClicked()
    
    def saveAsButtonClicked(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save As...", "","All Files (*);;YAML Files (*.yml, *.yaml)")
        if path:
            self.current_file_path = path
            self.saveParameters(path)
    
    def openButtonClicked(self):
        path, _ = QFileDialog.getOpenFileName(self, "Open File...", "","All Files (*);;YAML Files (*.yml, *.yaml)")
        if path:
            self.loadParameters(path)

    def saveParametersToFile(self,parameters,path):
        with open(path,"w") as file:
            yaml.dump(parameters, file)

    def loadParametersFromFile(self, path):
        with open(path,"r") as file:
            loaded_param =  yaml.safe_load(file)
        return loaded_param
    
    def saveParameters(self, path):
        parameters = {}
        for param_name in roboszpon_lib.ROBOSZPON_PARAMETERS.keys():
            response = roboszpon_lib.send_parameter_read(self.canbus, self.roboszpon, param_name)
            parameters[param_name] = response["value"]
        self.saveParametersToFile(parameters, path)
        self.showMessage("","Configuration successfully saved!")

    def loadParameters(self, path):
        parameters = self.loadParametersFromFile(path)
        self.current_file_path = path
        for param_name, value in parameters.items():
            if param_name in roboszpon_lib.ROBOSZPON_PARAMETERS.keys():
                roboszpon_lib.send_parameter_write(self.canbus, self.roboszpon, param_name, value)
            else:
                self.showWarning("Unknown parameter", f"Unknown parameter in opened file:\n{param_name}\nTry again.")
                return

    def showWarning(self, title, message):
        QMessageBox.warning(self, title, message, QMessageBox.Ok)

    def showMessage(self, title, message):
        QMessageBox.information(self,title,message, QMessageBox.Ok)

    def confirmReset(self):
        dlg = QDialog(self)
        dlg.setWindowTitle("Confirm Reset")
        dlg.setFixedSize(400, 200)  # Increased the size for better visibility

        layout = QVBoxLayout(dlg)
        
        # Add a descriptive label to make the message clearer
        label = QLabel("Are you sure you want to reset the system? All unsaved data will be lost.")
        label.setWordWrap(True)
        layout.addWidget(label)
        
        # Create the button box with OK and Cancel buttons
        buttonBox = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        layout.addWidget(buttonBox)

        # Connect the dialog buttons to accept/reject actions
        buttonBox.accepted.connect(dlg.accept)
        buttonBox.rejected.connect(dlg.reject)

        # Return the result of the dialog execution
        return dlg.exec()

    def parameterComboBoxChanged(self, text):
        self.updateParameterValue(text)

    def updateParameterValue(self, parameter_name):
        def callback(value):
            self.parameterSpinBox.setValue(value)
            self.oldParameterValue = value

        if self.roboszpon is None:
            return
        if self.devices[self.roboszpon].mode != roboszpon_lib.ROBOSZPON_MODE_STOPPED:
            print("Can't configure runnig roboszpon")
            return
        roboszpon_lib.read_parameter_callback(
            self.canbus,
            self.can_notifier,
            self.devices[self.roboszpon].node_id,
            roboszpon_lib.ROBOSZPON_PARAMETERS[parameter_name],
            callback,
        )

    def updateParameterButtonClicked(self):
        parameter_name = self.parameterComboBox.currentText()
        parameter_id = roboszpon_lib.ROBOSZPON_PARAMETERS[parameter_name]
        value = self.parameterSpinBox.value()
        if self.roboszpon is None:
            self.parameterSpinBox.setValue(self.oldParameterValue)
            return
        if self.devices[self.roboszpon].mode != roboszpon_lib.ROBOSZPON_MODE_STOPPED:
            # Maybe just disable the button...
            print("Can't configure running roboszpon")
            self.parameterSpinBox.setValue(self.oldParameterValue)
            return
        print(f"Sending: {parameter_name}={value}")
        roboszpon_lib.send_parameter_write(
            self.canbus, self.roboszpon, parameter_id, value
        )
        self.updateParameterValue(parameter_name)

    def commitConfiguration(self):
        roboszpon_lib.send_action_request(
            self.canbus, self.roboszpon, roboszpon_lib.ACTION_COMMIT_CONFIG
        )

    def restoreConfiguration(self):
        roboszpon_lib.send_action_request(
            self.canbus, self.roboszpon, roboszpon_lib.ACTION_RESTORE_CONFIG
        )
        self.updateParameterValue(self.parameterComboBox.currentText())

    def factoryConfiguration(self):
        roboszpon_lib.send_action_request(
            self.canbus, self.roboszpon, roboszpon_lib.ACTION_SET_FACTORY_CONFIG
        )
        self.updateParameterValue(self.parameterComboBox.currentText())

    def softwareReset(self):
        if self.confirmReset():
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


if __name__ == "__main__":
    app = QApplication(sys.argv)
    ex = MyLittleRoboszponSuite()
    ex.show()
    sys.exit(app.exec_())
