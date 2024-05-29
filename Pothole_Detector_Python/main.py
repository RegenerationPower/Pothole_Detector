from PyQt5 import QtWidgets, uic
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtCore import QIODevice, QTimer, QTime, Qt
from PyQt5.QtWidgets import QDialog, QVBoxLayout, QLabel
import pyqtgraph as pg
from csv import writer
import tensorflow as tf
import numpy as np

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        uic.loadUi("design.ui", self)
        self.setWindowTitle("SerialGUI")

        # Налаштування таймера для оновлення часу
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_time)
        self.timer.start(1000)  # Оновлення кожну секунду

        self.serial = QSerialPort()
        self.serial.setBaudRate(115200)
        port_list = [port.portName() for port in QSerialPortInfo().availablePorts()]
        self.comL.addItems(port_list)

        self.listValue = [[], [], [], [], [], []]  # список значень з MPU-6050, плюс для label, latitude, longitude
        self.physicalListValue = [[], [], []]  # список значень з MPU-6050, приведених до фізичних величин
        self.label_status = "smooth"  # Значення за замовчуванням для label
        self.manual_mode = False  # Режим ручного керування

        self.serial.readyRead.connect(self.on_read)

        self.openB.clicked.connect(self.on_open)
        self.clearB.clicked.connect(self.clear_graph)
        self.closeB.clicked.connect(self.on_close)
        self.outputButton.clicked.connect(self.save_data)
        self.settingsButton.clicked.connect(self.toggle_dark_theme)
        self.modeButton.clicked.connect(self.toggle_mode)
        self.modeButton.setStyleSheet("background-color: blue; color: white;")

        self.set_graph_theme("dark")
        self.toggle_dark_theme()

        # Завантаження навченої моделі
        self.model = tf.keras.models.load_model('pothole_detection_model.keras')

    # Оновлення часу на інтерфейсі
    def update_time(self):
        current_time = QTime.currentTime()
        self.lcdNumber.display(current_time.toString('hh:mm:ss'))

    # Опитування COM Port
    def on_read(self):
        rx = self.serial.readLine()
        rxs = str(rx, 'utf-8').strip()
        print(rxs)

        # Розділення даних на MPU-6050 та GPS
        if rxs.startswith('Latitude'):
            self.handle_gps_data(rxs)
        else:
            self.handle_mpu_data(rxs)

    # Обробка даних з MPU-6050
    def handle_mpu_data(self, rxs):
        try:
            data = [int(i) for i in rxs.split(";")]
        except ValueError:
            print("Invalid sensor data format")
            return

        # Отримуємо дані з MPU-6050
        for i in range(len(data)):
            self.listValue[i].append(data[i])

        # Передбачення дороги за допомогою навченої моделі або ручного режиму
        if not self.manual_mode:
            prediction = self.model.predict(np.array([data]))
            label = 'pothole' if prediction < 0.5 else 'smooth'
        else:
            label = self.label_status

        self.listValue[3].append(label)

        # Оновлення інтерфейсу, позначки "яма" "не яма"
        self.statusLabel.setText(f"Status: {label}")
        if label == 'pothole':
            self.statusLabel.setStyleSheet("background-color: red; color: white;")
        else:
            self.statusLabel.setStyleSheet("background-color: green; color: white;")

        # Перетворення отриманих даних до фізичних величин
        phisic_data = []
        for i in range(len(data)):
            phisic_data.append(data[i])
        for i in range(3):
            self.physicalListValue[i].append(phisic_data[i])

        # Виведення даних на інтерфейс
        self.Xlabel.setText(f"X: {phisic_data[0]}")
        self.Ylabel.setText(f"Y: {phisic_data[1]}")
        self.Zlabel.setText(f"Z: {phisic_data[2]}")

        # Генерація даних осі Х для побудови графіка
        axis_x = [j for j in range(len(self.physicalListValue[0]))]

        self.graph.clear()

        # Вимкнення/увімкнення певних осей координат
        if self.XcheckBox.isChecked():
            self.graph.plot(axis_x, self.physicalListValue[0], pen=(0, 3))
        if self.YcheckBox.isChecked():
            self.graph.plot(axis_x, self.physicalListValue[1], pen=(1, 3))
        if self.ZcheckBox.isChecked():
            self.graph.plot(axis_x, self.physicalListValue[2], pen=(2, 3))

    # Отримуємо дані з GPS
    def handle_gps_data(self, rxs):
        parts = rxs.split(', ')
        latitude = parts[0].split(': ')[1]
        longitude = parts[1].split(': ')[1]

        self.listValue[4].append(latitude)
        self.listValue[5].append(longitude)

        self.latitudeLabel.setText(f"Latitude: {latitude}")
        self.longitudeLabel.setText(f"Longitude: {longitude}")

    # Відкривання COM Port
    def on_open(self):
        self.serial.setPortName(self.comL.currentText())
        self.serial.open(QIODevice.ReadWrite)

    # Закривання COM Port
    def on_close(self):
        self.serial.close()

    # Очищення графіку і даних
    def clear_graph(self):
        self.graph.clear()
        self.physicalListValue = [[], [], []]
        self.listValue = [[], [], [], [], [], []]

    # Збереження даних з MPU-6050 у файл .csv
    def save_data(self):
        my_data = [["X", "Y", "Z", "Label"]]
        for i in range(len(self.listValue[0])):
            my_data.append([self.listValue[0][i], self.listValue[1][i], self.listValue[2][i], self.listValue[3][i]])
        with open('data.csv', 'w', newline='') as my_file:
            writ = writer(my_file)
            writ.writerows(my_data)
        print("Saving complete")

    # Переключення між світлою та темною темами
    def toggle_dark_theme(self):
        dark_theme = """
        QMainWindow {
            background-color: #2b2b2b;
            color: white;
        }
        QLabel, QCheckBox, QPushButton {
            color: white;
        }
        QPushButton {   
            background-color: #444444;
            border: 1px solid #666666;
            padding: 3px;
        }
        QComboBox {
            background-color: #444444;
            border: 1px solid #666666;
            color: white;
        }
        QComboBox QAbstractItemView {
            background-color: #2b2b2b;
            color: white;
        }
        """
        light_theme = ""

        if self.styleSheet() == "":
            self.setStyleSheet(dark_theme)
            self.set_graph_theme("dark")
        else:
            self.setStyleSheet(light_theme)
            self.set_graph_theme("light")

    # Встановлення теми для графіка
    def set_graph_theme(self, theme):
        if theme == "dark":
            self.graph.setBackground('k')
            self.graph.getAxis('left').setPen('w')
            self.graph.getAxis('bottom').setPen('w')
        else:
            self.graph.setBackground('w')
            self.graph.getAxis('left').setPen('k')
            self.graph.getAxis('bottom').setPen('k')

    # Ручний режим визначення ями при утримуванні клавіші T(повинна бути включена англійська розкладка клавіатури)
    def keyPressEvent(self, event):
        if event.key() == Qt.Key_T:
            self.label_status = "pothole"
            self.statusLabel.setText("Status: pothole")
            self.statusLabel.setStyleSheet("background-color: red; color: white;")

    #
    def keyReleaseEvent(self, event):
        if event.key() == Qt.Key_T:
            self.label_status = "smooth"
            self.statusLabel.setText("Status: smooth")
            self.statusLabel.setStyleSheet("background-color: green; color: white;")

    # Перемикання між автоматичним та ручним режимами
    def toggle_mode(self):
        self.manual_mode = not self.manual_mode
        if self.manual_mode:
            self.modeButton.setText("Switch to Automatic Mode")
            self.modeButton.setStyleSheet("background-color: yellow; color: black;")
        else:
            self.modeButton.setText("Switch to Manual Mode")
            self.modeButton.setStyleSheet("background-color: blue; color: white;")


app = QtWidgets.QApplication([])
window = MainWindow()
window.show()
app.exec()
