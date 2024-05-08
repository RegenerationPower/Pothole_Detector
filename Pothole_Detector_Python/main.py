from PyQt5 import QtWidgets, uic
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtCore import QIODevice
from csv import writer
from PyQt5.QtCore import QTimer, QTime


"""Налаштування програми з графічним інтерфейсом"""
app = QtWidgets.QApplication([])
ui = uic.loadUi("design.ui")
ui.setWindowTitle("SerialGUI")

timer = QTimer()


def update_time():
    current_time = QTime.currentTime()
    ui.lcdNumber.display(current_time.toString('hh:mm:ss'))


# Connect the QTimer's timeout signal to updateTime function
timer.timeout.connect(update_time)

# Set the interval for the QTimer (in milliseconds)
timer.start(1000)  # Update every second

"""Встановлення налаштувань для роботи з послідовним портом"""
serial = QSerialPort()
serial.setBaudRate(115200)
portList = []
ports = QSerialPortInfo().availablePorts()
for port in ports:
    portList.append(port.portName())
ui.comL.addItems(portList)

"""Фіксуємо значення фізичних величин та встановлюємо значення за-замовчуванням"""
ui.units_list.addItems(["mg", "g", "m/s2"])
uint = "mg"
sensitivity = 0.06

"""глобальні змінні для збереження даних"""
listValue = [[], [], []]  # список значень з акселерометра
physicalListValue = [[], [], []]  # список значень з акселерометра, приведених до фізичних величин
received_message = ""
data = 0

'''
def on_read():
    """Постійно опитує порт, працює безперервно"""

    """отримуємо дані та переводимо до зручного вигляду"""
    global received_message, data
    rx = serial.readAll()
    print("RX:", rx)
    rxs = str(rx.data(), 'utf-8')
    print("RXS:", rxs)
    received_message += rxs
    if '\n' in received_message:
        complete_message, remainder = received_message.split('\n', 1)
        data = [int(i) for i in complete_message.split(";") if i.strip()]
        print("Data:", data)
        received_message = remainder.strip()
        """фіксуємо поточні отримані значення"""
        global listValue
        for i in range(len(data)):
            listValue[i].append(data[i])
        print(data)

    """фіксуємо обрані фізичні величини"""
    global uint
    global sensitivity
    if ui.units_list.currentText() != uint:
        """якщо було змінено значення фізичних одиниць, то перерарахувати та встановити нові значення."""
        clear_graph()
        uint = ui.units_list.currentText()
        if uint == "g":
            ui.label_uintsX.setText("g")
            ui.label_uintsY.setText("g")
            ui.label_uintsZ.setText("g")
            # sensitivity = 0.00006
            sensitivity = 1
            ui.sensitive_Value.setText(str(sensitivity))
        if uint == "mg":
            ui.label_uintsX.setText("mg")
            ui.label_uintsY.setText("mg")
            ui.label_uintsZ.setText("mg")
            sensitivity = 0.06
            ui.sensitive_Value.setText(str(sensitivity))
        if uint == "m/s2":
            ui.label_uintsX.setText("m/s2")
            ui.label_uintsY.setText("m/s2")
            ui.label_uintsZ.setText("m/s2")
            sensitivity = 0.000588399
            ui.sensitive_Value.setText(str(sensitivity))

    """Перетворюємо отримані дані до фізичних величин"""
    global physicalListValue
    physic_data = []
    if data != 0:
        for i in range(len(data)):
            physic_data.append(data[i] * sensitivity)
        for i in range(3):
            physicalListValue[i].append(physic_data[i])

        """Виводимо дані у додатку"""
        ui.Xlabel.setText("X: {: .4f}".format(physic_data[0]))
        ui.Ylabel.setText("Y: {: .4f}".format(physic_data[1]))
        ui.Zlabel.setText("Z: {: .4f}".format(physic_data[2]))

        """Встановлюємо дані для гістограми"""
        ui.XBar.setValue(round(data[0] / 150))
        ui.YBar.setValue(round(data[1] / 150))
        ui.ZBar.setValue(round(data[2] / 150))

        i = 0
        """Генеруємо дані осі Х  для побудови графіка"""
        x_axis = [j for j in range(len(physicalListValue[i]))]

        """Очищуємо старий графік"""
        ui.graph.clear()

        """Будуємо нові графіки згідно встановлених чекбоксів"""
        if ui.XcheckBox.isChecked():
            ui.graph.plot(x_axis, physicalListValue[0], pen=(0, 3))
        if ui.YcheckBox.isChecked():
            ui.graph.plot(x_axis, physicalListValue[1], pen=(1, 3))
        if ui.ZcheckBox.isChecked():
            ui.graph.plot(x_axis, physicalListValue[2], pen=(2, 3))
'''

def on_read():
    """Постійно опитує порт, працює безперервно"""

    """отримуємо дані та переводимо до зручного вигляду"""
    rx = serial.readLine()
    rxs = str(rx, 'utf-8').strip()
    data = [int(i) for i in rxs.split(";")]

    """фіксуємо поточні отриманні значення"""
    global listValue
    for i in range(len(data)):
        listValue[i].append(data[i])
    print(data)


    """фіксуємо обрані фізичні величини"""
    global uint
    global sensitivity
    if ui.units_list.currentText() != uint:
        """якщо було змінено значення фізичних одиниць, то перерарахувати та встановити нові значення."""
        clearGraph()
        uint = ui.units_list.currentText()
        if uint == "g":
            ui.label_uintsX.setText("g")
            ui.label_uintsY.setText("g")
            ui.label_uintsZ.setText("g")
            sensitivity = 0.00006
            ui.sensitive_Value.setText(str(sensitivity))
        if uint == "mg":
            ui.label_uintsX.setText("mg")
            ui.label_uintsY.setText("mg")
            ui.label_uintsZ.setText("mg")
            sensitivity = 0.06
            ui.sensitive_Value.setText(str(sensitivity))
        if uint == "m/s2":
            ui.label_uintsX.setText("m/s2")
            ui.label_uintsY.setText("m/s2")
            ui.label_uintsZ.setText("m/s2")
            sensitivity = 0.000588399
            ui.sensitive_Value.setText(str(sensitivity))

    """Перетворюємо отримані данні до фізачних величин"""
    global physicalListValue
    phisicData = []
    for i in range(len(data)):
        phisicData.append(data[i] * sensitivity)
    for i in range(3):
        physicalListValue[i].append(phisicData[i])

    """Виводимо данні у додатку"""
    ui.Xlabel.setText("X: {: .4f}".format(phisicData[0]))
    ui.Ylabel.setText("Y: {: .4f}".format(phisicData[1]))
    ui.Zlabel.setText("Z: {: .4f}".format(phisicData[2]))

    """Встановлюємо данні для гістограми"""
    ui.XBar.setValue(round(data[0] / 150))
    ui.YBar.setValue(round(data[1] / 150))
    ui.ZBar.setValue(round(data[2] / 150))

    """Генеруємо данні осі Х  для побудови графіка"""
    Axis_X = [j for j in range(len(physicalListValue[i]))]

    """Очищаємо старий графік"""
    ui.graph.clear()

    """Будуємо нові графіки згідно встановленних чекбоксів"""
    if ui.XcheckBox.isChecked():
        ui.graph.plot(Axis_X, physicalListValue[0], pen=(0, 3))
    if ui.YcheckBox.isChecked():
        ui.graph.plot(Axis_X, physicalListValue[1], pen=(1, 3))
    if ui.ZcheckBox.isChecked():
        ui.graph.plot(Axis_X, physicalListValue[2], pen=(2, 3))

def on_open():
    """Відкриття обраного порту"""
    serial.setPortName(ui.comL.currentText())
    serial.open(QIODevice.ReadWrite)


def on_close():
    """Закриття порту"""
    serial.close()


def clear_graph():
    """Очищення графіку"""
    ui.graph.clear()
    global physicalListValue
    physicalListValue = [[], [], []]


def save_data():
    """Збереження даних у csv файл"""
    global listValue
    my_data = [["X", "Y", "Z"]]
    for i in range(len(listValue[0])):
        my_data.append([listValue[0][i], listValue[1][i], listValue[2][i]])
    my_file = open('data.csv', 'w')
    with my_file:
        writ = writer(my_file)
        writ.writerows(my_data)
    print("Writing complete")


def showSettingsWindow():
    settingsWindow = QDialog()
    settingsWindow.setWindowTitle("Settings")

    # Add widgets to the settings window
    layout = QVBoxLayout()
    label = QLabel("This is the settings window")
    layout.addWidget(label)
    settingsWindow.setLayout(layout)

    settingsWindow.exec_()

"""постійне зчитування даних з порту"""
serial.readyRead.connect(on_read)

"""опрацювання натиску на кнопки графічного інтерфейсу"""
ui.openB.clicked.connect(on_open)
ui.clearB.clicked.connect(clear_graph)
ui.closeB.clicked.connect(on_close)
ui.outputButton.clicked.connect(save_data)
ui.settingsButton.clicked.connect(showSettingsWindow)

"""запуск програми"""
ui.show()
app.exec()
