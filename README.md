<p align="center">
  <img src="header.png" alt="header.png" width="80%">
</p>

<h1 align="center">🤖 ROS2 + ESP32 Sensor PIR Detection System</h1>
<p align="center">
  <i>Project MK Robotika Medis</i>
</p>

---
## 👥 Anggota Kelompok
| Nama                       | NIM        |
|----------------------------|-----------|
| Ivan Fulvian Pitoyo                  | 122430064 |
| Aidah Zahran Nurbati Rohmah          | 122430134 |
| Ove Dewanda Ratih                    | 122430083 |
| Meichel Naek Silalahi                | 122430059 |

---

# 📌 Pendahuluan

Sistem ini merupakan implementasi **ROS2** dengan integrasi **ESP32** sebagai perangkat embedded untuk membaca data sensor, yang disusun untuk memenuhi tugas mata kuliah **Robotika Medis**.

Dalam project ini:
- **ESP32 berperan sebagai *Node Publisher*** yang bertugas mengirimkan data apakah ada gerakan yang dibaca oleh sensor PIR **HC-SR04** melalui protokol **micro-ROS**.
- **Relay yang menjalankan ROS2 berfungsi sebagai *Node Subscriber*** yang menerima, memproses, dan menampilkan data jarak tersebut secara *real-time*.

Arsitektur ini memungkinkan komunikasi dua arah antara perangkat embedded dan sistem ROS2 melalui jaringan, sehingga data dari sensor dapat langsung dimanfaatkan pada sisi host untuk monitoring maupun pengolahan lebih lanjut.

---

# 🛠️ LANGKAH-LANGKAH PEMBUATAN SISTEM  

## 1 — Persiapan Komponen
| No | Komponen | Jumlah |
|----|----------|--------|
| 1 | ESP32 Dev Board | 1 |
| 2 | Sensor PIR (Passive Infrared) | 1 |
| 3 | Breadboard | 1 |
| 4 | Kabel jumper | 10 |
| 5 | Kabel USB type C| 1 |
| 6 | PC Windows + Python + PIXI | 1 |
| 7 | Sensor PIR | 1 |
| 8 | Expansion Board ESP 32 | 1 |

---
## 2 — Perakitan HC-SR04 dengan ESP32

### Koneksi Pin
| Sensor | ESP32 | Keterangan |
|--------|-------|------------|
| VCC | 5V | Sumber daya |
| GND | GND | Ground |
| Trig | D5 | Pemicu |
| Out | D26 |

---
## 3 — Diagram Blok Sistem
<p align="center">
  <img src="header.png" alt="Diagram Alir.jpeg" width="80%">
</p>

<h1 align="center">🤖 ROS2 + ESP32 Sensor PIR Detection System</h1>
<p align="center">
  <i>Project MK Robotika Medis</i>
</p>

---
### Rangkaian Resistor Divider (Wajib untuk menggunakan sumber lain ESP32)
Tujuan: 

```markdown
 ESP32 → Sensor PIR
          |
         Relay
          |
         GND
```

### Program ESP32 (Arduino IDE)
```
#include <WiFi.h>

// --- GANTI SESUAI HOTSPOT HP KAMU ---
const char* ssid = "Zero30";
const char* password = "smstr7ip4";

#define PIR_PIN     5
#define RELAY_PIN   26

WiFiServer server(8888); // Port pintu masuk data

void setup() {
  Serial.begin(115200);
  pinMode(PIR_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  // Konek WiFi
  Serial.print("Menghubungkan WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi Terhubung!");
  Serial.print("IP Address ESP32: ");
  Serial.println(WiFi.localIP()); // <--- WAJIB DICATAT ANGKA INI!
  
  server.begin();
}

void loop() {
  WiFiClient client = server.available(); // Cek apakah Ubuntu mau konek

  if (client) {
    while (client.connected()) {
      int pir = digitalRead(PIR_PIN);

      // Kirim angka "1" (Gerak) atau "0" (Diam) ke Ubuntu
      if (pir == HIGH) {
        client.println("1"); 
        digitalWrite(RELAY_PIN, HIGH);
      } else {
        client.println("0");
        digitalWrite(RELAY_PIN, LOW);
      }
      delay(100); 
    }
    client.stop();
  }
}
```
OUTPUT: angka jarak dalam cm via port COM.
---

## 3 — Persiapan Workspace PIXI

### **Akses Folder Workspace**
powershell
```bash
cd C:\pixi_ws
```

### **Aktifkan Shell PIXI**
powershell
```bash
pixi shell
```
Promt akan berubah menjadi:
```powershell
(pixi_ros2_jazzy) PS C:\pixi_ws>
```

### **Masuk ke Workspace ROS2**
powershell
```bash
cd C:\pixi_ws\ros2_ws
```

---
## 4 — Membuat Package ROS2
powershell
```bash
cd src
ros2 pkg create smart_system --build-type ament_python
```
Struktur direktori otomatis terbentuk:
```
smart_system/
  ├── package.xml
  ├── setup.py
  └── smart_system/
       └── __init__.py
```
---

## 5 — Pembuatan _Publisher_ dan _Subscriber_  
### **Publisher: `publisher.py`**
```
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import serial
import time

class PIRPublisher(Node):
    def __init__(self):
        super().__init__('pir_publisher')
        self.publisher_ = self.create_publisher(Bool, 'motion_detected', 10)

        # GANTI PORT sesuai ESP32 kamu
        self.serial_port = serial.Serial('COM9', 115200, timeout=1)

        self.timer = self.create_timer(0.5, self.publish_data)

    def publish_data(self):
        if self.serial_port.in_waiting > 0:
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                motion_detected = line.lower() in ['1', 'true', 'yes', 'detected']
                msg = Bool()
                msg.data = motion_detected
                self.publisher_.publish(msg)
                status = 'Motion Detected' if motion_detected else 'No Motion'
                self.get_logger().info(f'PIR Sensor: {status}')
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = PIRPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```
---
### **Subscriber: `relay_display.py`**
Menampilkan nilai jarak di terminal.
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class MotionSubscriber(Node):
    def __init__(self):
        super().__init__('motion_subscriber')

        self.subscription = self.create_subscription(
            Bool,
            'motion_detected',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        status = 'MOTION DETECTED!' if msg.data else 'No motion detected'
        self.get_logger().info(status)

def main(args=None):
    rclpy.init(args=args)
    node = MotionSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
---

## 5 — Instalasi Dependency
**Install pyserial**
powershell
```bash
pip install pyserial
pip show pyserial
```

**Tambahkan ke `setup.py`**
```phyton
install_requires=['setuptools', 'pyserial'],
```
---

## 6 — Build Workspace ROS2
powershell
```bash
cd C:\pixi_ws\ros2_ws
colcon build
. install/setup.ps1
```
---
## 7 — Menjalankan Node
### **Menjalankan Publisher**
powershell
```bash
ros2 run smart_system publisher
```

### **Menjalankan Subscriber (Terminal baru)**
powershell
```bash
pixi shell
cd C:\pixi_ws\ros2_ws
. install/setup.ps1
ros2 run smart_system subscriber_display
```
Subscriber akan menampilkan data perubahan dengan angka 1 untuk menyalakan lampu relay secara _real time_ dan angka 0 lampu relay tetap off
---

## 8 — Menghentikan Node
- Tekan CTRL + C
- Menutup terminal → otomatis mematikan node
- Jika node macet → hentikan python.exe lewat Task Manager

---

# 🚧 KENDALA & SOLUSI
## **1. Port ESP32 (COM) Berubah-ubah**
**Kendala:**
Setiap kali kabel USB ESP32 dicabut dan dipasang kembali, nomor port COM sering berubah. Akibatnya, ROS2 Publisher tidak bisa membaca data karena kode Python masih menggunakan port lama. Setelah port diubah di kode, workspace juga harus di-build ulang menggunakan:

```
colcon build
. install/setup.ps1
```

**Solusi:**
- Cek ulang nomor port ESP32 di Device Manager setiap kali reconnect.
- Perbarui port di kode Publisher.  
- Build ulang workspace setelah melakukan perubahan.
- (Opsional) Buat script auto-detect COM supaya tidak perlu ganti port manual.

---
## **2. Serial Monitor Arduino Mengunci Port**
**Kendala:**
Saat Serial Monitor Arduino IDE terbuka, port COM milik ESP32 “dikunci” oleh Arduino IDE. ROS2 jadi tidak bisa membaca data dari port tersebut karena hanya satu aplikasi boleh memakai port pada satu waktu.

**Solusi:**
- Tutup Serial Monitor setelah selesai mengecek data dari ESP32.
- Setelah itu jalankan node ROS2 Publisher, agar port tidak konflik dan bisa dibaca oleh ROS2.

---
### Pertanyaan & Komentar
- Silakan buka `issue` di repositori utama untuk bertanya atau memberi masukan.

<p align="center">
  <b>✨ Terima kasih! ✨</b><br>
</p>

---


