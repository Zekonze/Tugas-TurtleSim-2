# Turtle Mission · ROS 1

Package catkin untuk **Waypoint Patrol**, **Safe Explorer**, dan **Hunter**. Satu peluncuran menjalankan turtlesim beserta tiga node buatan sendiri. Dalam mode default, `turtle1` menyelesaikan ketiga level secara berurutan dan berhenti setelah target Hunter terakhir. Warna lintasan berubah pada awal setiap fase sebagai bonus visual.

https://youtu.be/TaF15XkXeHI
Video Bukti

## Build dan jalankan

Gunakan terminal pada sesi desktop dengan **ROS 1 Noetic** dan Python 3. Pada Ubuntu 20.04 dengan repositori ROS Noetic yang sudah dikonfigurasi, pasang dependensi berikut jika belum tersedia.

```bash
sudo apt install ros-noetic-turtlesim ros-noetic-rqt-graph
source /opt/ros/noetic/setup.bash
```

Simpan ZIP di folder Downloads, lalu ekstrak ke direktori `src` workspace catkin.

```bash
mkdir -p ~/catkin_ws/src
unzip ~/Downloads/turtle_mission_ros1.zip -d ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch turtle_mission mission.launch
```

`roslaunch` membuka arena dan memulai misi. Pada terminal baru, jalankan `source /opt/ros/noetic/setup.bash` dan `source ~/catkin_ws/devel/setup.bash` sebelum memakai perintah ROS. Hentikan peluncuran sebelumnya dengan Ctrl+C sebelum membuka instance turtlesim yang baru.

| Keperluan | Perintah |
|---|---|
| Semua level, urutan 1 → 2 → 3 | `roslaunch turtle_mission mission.launch` |
| Level 1 saja | `roslaunch turtle_mission mission.launch mode:=patrol` |
| Level 2 saja | `roslaunch turtle_mission mission.launch mode:=explore` |
| Level 3 saja | `roslaunch turtle_mission mission.launch mode:=hunter` |
| Tunggu permintaan start | `roslaunch turtle_mission mission.launch autostart:=false` |

Pada mode tunggu, panggil service dari terminal lain.

```bash
rosservice call /start_mission "mode: 'all'"
```

Permintaan dengan `mode: ''` memakai mode dari peluncuran. Service juga dapat menjalankan misi baru setelah misi sebelumnya selesai. Permintaan ketika misi sedang berjalan akan ditolak.

## Level yang diimplementasikan

| Level | Perilaku |
|---|---|
| **1 · Waypoint Patrol** | Mengunjungi empat waypoint sesuai urutan YAML. Pada jarak di bawah **0,08 unit**, controller mengirim kecepatan nol dan menunggu konfirmasi berhenti dari pose. Ada jeda di setiap waypoint. Di akhir Patrol, mode `patrol` berhenti dan mode `all` berhenti sejenak sebelum masuk ke Explorer. |
| **2 · Safe Explorer** | Menjelajahi empat titik tepi arena. Sensor virtual menghitung jarak ke setiap dinding dari pose. Controller mengurangi komponen kecepatan yang mengarah ke dinding pada jarak di bawah **1,75 unit**, lalu memblokir komponen itu jika jarak mencapai **0,65 unit**. |
| **3 · Hunter** | Memilih koordinat acak di dalam batas aman, memanggil `/spawn`, dan menerima posisi target diam dari `/<nama_target>/pose`. Controller mengejar target dengan ambang **0,42 unit**. Manager menghitung ulang jarak dari pose kedua kura-kura dan memanggil `/kill` ketika jarak **< 0,5 unit**. Skor bertambah setelah service sukses. Siklus berulang sampai tiga target tertangkap. |
| **Bonus · Warna pena** | Amber untuk Patrol, hijau untuk Explorer, dan ungu untuk Hunter. Perubahan warna dilakukan dengan `/turtle1/set_pen`. |

Waypoint, gain, toleransi, margin dinding, dan jumlah target berada di [config/mission.yaml](config/mission.yaml). Koordinat `(x, y)` berawal dari pojok kiri bawah. `hunter_seed: -1` membuat koordinat spawn acak setiap peluncuran. Gunakan bilangan bulat tetap untuk mengulangi urutan acak dari kondisi awal yang sama. Untuk mencoba YAML di luar package, tambahkan `config:=/path/absolut/mission.yaml` ke perintah launch.

## Node

| Node | Fungsi |
|---|---|
| `/turtlesim` | Node bawaan. Memublikasikan pose, menerima `Twist`, dan menyediakan service simulasi. |
| `/mission_manager` | Mengatur urutan level, memilih waypoint, memproses hasil controller, dan mengelola target serta skor. Menyediakan `/start_mission`. |
| `/motion_controller` | Menerima target, pose, dan data sensor virtual. Menghitung kecepatan 30 kali per detik. Satu-satunya publisher `/turtle1/cmd_vel` dalam peluncuran ini. |
| `/wall_sensor` | Mengubah pose menjadi empat jarak terhadap dinding dan status `near_wall`. |

## Topic dan message

| Topic | Tipe | Publisher → subscriber dan isi |
|---|---|---|
| `/turtle1/pose` | `turtlesim/Pose` | Turtlesim → controller dan sensor. Posisi, orientasi, dan kecepatan aktual. |
| `/turtle1/cmd_vel` | `geometry_msgs/Twist` | Controller → turtlesim. Menggunakan `linear.x` dan `angular.z`. |
| `/mission/goal` | `turtle_mission/MotionGoal` | Manager → controller. ID tujuan, fase, indeks, koordinat, toleransi, dan status aktif. |
| `/mission/status` | `turtle_mission/MissionStatus` | Controller → manager. ID tujuan, fase, indeks, pose, jarak, dan konfirmasi tiba. |
| `/mission/wall_sensor` | `turtle_mission/WallSensor` | Sensor → controller. Jarak kiri, kanan, bawah, atas, jarak minimum, dan status dekat dinding. |
| `/mission/score` | `turtle_mission/HuntScore` | Manager → pemantau. Skor, total, nama target aktif, jarak tangkap terakhir, dan status selesai. |
| `/<nama_target>/pose` | `turtlesim/Pose` | Turtlesim → manager selama target Hunter aktif. |

## Service

| Service | Tipe | Fungsi |
|---|---|---|
| `/start_mission` | `turtle_mission/StartMission` | Request `string mode`. Response `bool accepted` dan `string message`. |
| `/spawn` | `turtlesim/Spawn` | Membuat target Hunter di koordinat terpilih. |
| `/kill` | `turtlesim/Kill` | Menghapus target Hunter setelah syarat tangkap terpenuhi. |
| `/turtle1/set_pen` | `turtlesim/SetPen` | Mengatur warna pena pada pergantian fase. |

## Cara kerja P-controller

Controller menghitung `dx = x_target - x` dan `dy = y_target - y` dari pose terbaru. Error jaraknya `d = hypot(dx, dy)`. Error arah adalah selisih antara `atan2(dy, dx)` dan `theta`, dinormalisasi ke rentang ±π.

Kecepatan sudut adalah `k_angular × error_arah`, dibatasi oleh `max_angular`. Setelah error arah lebih kecil dari `heading_gate`, kecepatan maju adalah `min(max_linear, k_linear × d) × cos(error_arah)`, dengan pembatas tambahan dari sensor dinding. Controller mengirim `Twist` nol di dalam toleransi, saat arahan tidak aktif, atau ketika feedback gerak kedaluwarsa. Manager mencocokkan ID status dengan ID tujuan aktif sebelum maju. Jeda antartitik dan antarlevel memberi waktu untuk melihat turtle berhenti setelah pose mengonfirmasi kedatangan.

## Pemantauan ketika demo

```bash
rosrun rqt_graph rqt_graph
rostopic echo /mission/status
rostopic echo /mission/wall_sensor
rostopic echo /mission/score
rostopic info /turtle1/cmd_vel
```

Di rqt_graph, pilih **Nodes/Topics (all)** dan tekan **Refresh**. Jika `/mission/score` tersembunyi, hilangkan centang **Leaf topics** atau buka `rostopic echo /mission/score` di terminal terpisah. Topic pose target hanya muncul saat target masih ada. Saat demo mode `all`, perhatikan empat pemberhentian Patrol, sensor dinding saat Explorer, lalu tiga siklus spawn, kejar, dan tangkap dengan skor akhir **3/3**.

## Hasil uji yang direkam

Package berhasil dibangun dengan catkin dan dijalankan pada ROS 1 dengan turtlesim dari cabang Noetic. Pada sesi video, misi dimulai otomatis dengan mode `all`.

| Metrik | Hasil |
|---|---|
| Patrol | 4/4 waypoint tercapai, error maksimum **0,079 unit** |
| Explorer | 4/4 waypoint tercapai, jarak minimum ke dinding **1,149 unit** |
| Hunter | **3/3 target** tertangkap dengan jarak **0,360**, **0,385**, dan **0,382 unit** |
| Akhir misi | Turtle berhenti, kecepatan maju **0,00** |

Screenshot rqt_graph diambil dari sesi Hunter yang terpisah. Node `/score_monitor` pada screenshot hanya digunakan untuk mengamati topic skor ketika mengambil bukti.
