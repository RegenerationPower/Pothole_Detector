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

"""глобальні змінні для збереження даних"""
listValue = [[], [], []]  # список значень з акселерометра
physicalListValue = [[], [], []]  # список значень з акселерометра, приведених до фізичних величин
received_message = ""
data = 0


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

    """Перетворюємо отримані данні до фізачних величин"""
    global physicalListValue
    phisic_data = []
    for i in range(len(data)):
        phisic_data.append(data[i])
    for i in range(3):
        physicalListValue[i].append(phisic_data[i])

    """Виводимо данні у додатку"""
    ui.Xlabel.setText("X: {: .4f}".format(phisic_data[0]))
    ui.Ylabel.setText("Y: {: .4f}".format(phisic_data[1]))
    ui.Zlabel.setText("Z: {: .4f}".format(phisic_data[2]))

    """Встановлюємо данні для гістограми"""
    ui.XBar.setValue(round(data[0] / 150))
    ui.YBar.setValue(round(data[1] / 150))
    ui.ZBar.setValue(round(data[2] / 150))

    """Генеруємо данні осі Х  для побудови графіка"""
    axis_x = [j for j in range(len(physicalListValue[i]))]

    """Очищаємо старий графік"""
    ui.graph.clear()

    """Будуємо нові графіки згідно встановленних чекбоксів"""
    if ui.XcheckBox.isChecked():
        ui.graph.plot(axis_x, physicalListValue[0], pen=(0, 3))
    if ui.YcheckBox.isChecked():
        ui.graph.plot(axis_x, physicalListValue[1], pen=(1, 3))
    if ui.ZcheckBox.isChecked():
        ui.graph.plot(axis_x, physicalListValue[2], pen=(2, 3))


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


def show_settings_window():
    settings_window = QDialog()
    settings_window.setWindowTitle("Settings")

    # Add widgets to the settings window
    layout = QVBoxLayout()
    label = QLabel("This is the settings window")
    layout.addWidget(label)
    settings_window.setLayout(layout)

    settings_window.exec_()


"""постійне зчитування даних з порту"""
serial.readyRead.connect(on_read)

"""опрацювання натиску на кнопки графічного інтерфейсу"""
ui.openB.clicked.connect(on_open)
ui.clearB.clicked.connect(clear_graph)
ui.closeB.clicked.connect(on_close)
ui.outputButton.clicked.connect(save_data)
ui.settingsButton.clicked.connect(show_settings_window)

"""запуск програми"""
ui.show()
app.exec()
