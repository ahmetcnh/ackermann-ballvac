
# BallVac — Ackermann Vehicle Simulation & Ball Collection

Bu depo, Ackermann direksiyonlu bir robotun Gazebo / ros_gz ile simülasyonunda otonom top toplama davranışlarını ve ilgili ROS2 paketlerini içerir. Docker ile ilgili bilgiler bu README'de yer almamaktadır.

**Öne çıkan paketler:**
- `ballvac_description` — robot ve dünyalar için URDF/SDFormat modelleri ve varlıklar.
- `ballvac_bringup` — simülasyon, SLAM ve Nav2 için launch dosyaları.
- `ballvac_ball_collector` — top algılama, FSM davranışı ve toplayıcı düğümleri (C++).
- `ballvac_control` — kontrol yardımcı paketleri ve düğümler.
- `ballvac_msgs` — proje için tanımlı özel mesajlar.

## Gereksinimler
- ROS 2 (ör. `ros-<distro>`)
- `ros_gz` / Gazebo (simülasyon köprüsü)
- Standart ROS 2 paketleri: `rclcpp`, `nav2`, `slam_toolbox`, `tf2`, vb.

Not: Sistemin tam bağımlılık listesi ve paket bağımlılıkları her paketin `package.xml` içinde belirtilmiştir.

## Derleme
Çalışma alanı kökünden:

```bash
source /opt/ros/<distro>/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Örnek Başlatma (Launch)
Aşağıdaki örnekler `launch` dizinindeki temel senaryoları çalıştırır. (Docker komutları veya görüntüleri burada yer almıyor.)

- Tam toplama senaryosu (araştırma + toplama):

```bash
ros2 launch ballvac_ball_collector ball_collection_full.launch.py
```

- Sahnede top spawn ve simülasyon setup:

```bash
ros2 launch ballvac_ball_collector ball_arena.launch.py
ros2 launch ballvac_bringup ball_arena_spawn.launch.py
```

- SLAM + Nav2 başlatma (haritalama ve navigasyon):

```bash
ros2 launch ballvac_bringup slam.launch.py
ros2 launch ballvac_bringup navigation_bringup.launch.py
```

Launch dosyalarının tam yolları örneğin: [ballvac_ball_collector/launch/ball_arena.launch.py](ballvac_ball_collector/launch/ball_arena.launch.py) ve [ballvac_ball_collector/launch/ball_collection_full.launch.py](ballvac_ball_collector/launch/ball_collection_full.launch.py).

## Temel Konseptler ve Konular
- Top algılama: `/ball_detections` (özel mesaj `ballvac_msgs::msg::BallDetectionArray`)
- Hareket komutları: `/cmd_vel` veya robotun kontrolüne bağlı alternatif topicler
- Lidar/mesafe: `/scan`
- Odom / TF: `/odom`, TF ikili dönüşümleri

## Paketler ve önemli dosyalar
- `ballvac_ball_collector/src/` — toplayıcı düğümleri: `ball_collector_node.cpp`, `ball_perception_node.cpp`, vb.
- `ballvac_bringup/launch/` — SLAM, spawn ve Nav2 başlatmaları.
- `ballvac_msgs/msg/` — `BallDetection.msg`, `BallDetectionArray.msg`, `Map.msg`.

## Geliştirme ve Katkı
- Yeni özellikler veya hata düzeltmeleri için branch açıp pull request gönderin.
- Kod stilleri ve build kuralları paketlerin `CMakeLists.txt` ve `package.xml` dosyalarında belirtilmiştir.

## Lisans
Paketlerin her biri kendi `package.xml` içinde lisans bilgisi içerir; projede kullanılan bazı paketler MIT veya Apache-2.0 gibi açık kaynak lisansları ile belirtilmiştir.

---
