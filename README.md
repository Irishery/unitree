# Unitree G1 ROS 2 workspace

Готовая основа для связи G1 с ROS 2 на ноутбуке:

```text
G1 native DDS                 ROS 2 bridge on robot                  Laptop
lowstate  ------------------> /g1/joint_states, /g1/imu/data -----> RViz / rqt
sport request <-------------- /cmd_vel <--------------------------- teleop / GUI
```

Bridge запускается на вычислителе G1. Так Unitree-зависимости и native DDS
остаются на роботе, а ноутбук получает обычные ROS-сообщения через отдельный
Humble/CycloneDDS viewer. Физический bridge нельзя запускать из Jazzy-среды.

Текущая конфигурация рассчитана на **G1 EDU, 29 DoF, ROS 2 Humble, Ubuntu 22.04** на роботе. Физический DDS-контур на ноутбуке также использует ROS 2 Humble и CycloneDDS внутри Docker; смешивать Jazzy и Humble в одном домене нельзя. Jazzy остаётся только в изолированных симуляционных доменах. Перед первым запуском обязательно сверить модель G1 (23/29 DoF), прошивку и имя сетевого интерфейса.

## Что уже есть

- преобразование `unitree_hg/LowState` в `/g1/joint_states` и `/g1/imu/data`;
- `/cmd_vel` в официальный high-level G1 locomotion request, API `7105`;
- явное включение управления через `/g1/enable_control`;
- ограничение скоростей, watchdog команд и watchdog телеметрии;
- launch/config, DDS network setup и smoke test;
- фиксированная версия официальных Unitree ROS 2 messages (`v0.3.0`).
- реальное управление двумя кистями DEX3-1 через официальный DDS-интерфейс,
  с отдельным enable, ограничениями, watchdog состояния/команд и stop-service.

Контракт топиков описан в [docs/TOPICS.md](docs/TOPICS.md), физическая проверка — в [docs/SAFETY.md](docs/SAFETY.md).

## Gazebo на ноутбуке

Для ROS 2 Jazzy + Gazebo Harmonic подготовлен отдельный Docker-стенд с
официальной моделью G1 29 DoF и двумя артикулированными кистями DEX3-1:

```bash
./scripts/sim_build.sh
./scripts/sim_up.sh
```

Во втором терминале можно запустить управление:

```bash
./scripts/sim_teleop.sh
```

Стенд открывает Gazebo вместе с RViz, публикует состояние G1, RGB/depth,
point cloud и IMU виртуальной RealSense D435i, штатный Livox Mid-360 как
горизонтальный `/scan`, измеренную `/odom`, а также принимает `/cmd_vel`.
SLAM Toolbox и Nav2 запускаются по умолчанию: в RViz отображаются карта,
costmap и инструмент `Nav2 Goal`.
В сцене также есть автоматическая демонстрация perception-to-motion: стол,
коробка, depth-сегментация, двуручные grasp targets, численный IK и движение
14 суставов DEX3-1. В RViz добавлены маркеры распознанной коробки, целевых точек ладоней и
траектории; демо можно запускать вручную через `/g1/task/start`. Ограничения описаны в
[docs/SIMULATION.md](docs/SIMULATION.md#демонстрация-стол--коробка--взять-в-руку).
Это кинематическая проверка ROS/GUI-контура, не физически достоверная ходьба.
Подробности, headless-режим и launch для физической D435i — в
[docs/SIMULATION.md](docs/SIMULATION.md).

Навигацию или SLAM можно отключить для изолированных тестов:

```bash
./scripts/sim_up.sh headless navigation:=false
./scripts/sim_up.sh headless slam:=false
```

## MuJoCo: физический DEX3 grasp

MuJoCo сейчас используется как навигационный стенд A → B: SLAM Toolbox, Nav2,
`/scan`, `/odom` и RViz запускаются по умолчанию, а распознавание/захват коробки
отключены до явного `tabletop_pick:=true`. Сцена всё ещё содержит официальный
MJCF G1 29 DoF с DEX3-1, стол и свободную коробку, чтобы можно было вернуться к
контактному grasp без пересборки архитектуры.

```bash
./scripts/mujoco_build.sh
./scripts/mujoco_up.sh
```

Задать точку B в уже запущенном контейнере:

```bash
./scripts/mujoco_nav_goal.sh 1.0 0.0
```

Подробный поток Nav2 feedback включается отдельно:

```bash
./scripts/mujoco_nav_goal.sh --feedback 1.0 0.0
```

Управление отдельными суставами идёт через
`/g1/mujoco/joints/<joint>/command`; команды физических кистей
`/g1/dex3/{left,right}/command` также принимаются напрямую. Индикаторы
`/g1/mujoco/hand_box_contacts` и `/g1/mujoco/physical_grasp` сообщают только
о реальных контактах MuJoCo с коробкой — они не создают constraint или attach.

MuJoCo-стенд также поднимает SLAM Toolbox, Nav2, `/scan`, `/odom` и RViz.
Навигационная база в MuJoCo пока кинематическая (`/cmd_vel → odom/tf`), а не
динамическая ходьба; это слой для проверки ROS2 GUI/SLAM/Nav2-контракта перед
переносом на Gazebo или физический G1.

Для подготовки к железу MuJoCo можно запустить с эмуляцией родного G1
high-level locomotion API:

```bash
./scripts/mujoco_up.sh loco_api:=true
./scripts/mujoco_loco_request.sh start
./scripts/mujoco_loco_request.sh move 0.15 0.0 0.0 1.0
./scripts/mujoco_loco_request.sh stop
```

Это проверяет ROS 2 wire contract `/api/sport/request` →
`/api/sport/response`; закрытый firmware-контроллер Unitree в MuJoCo при этом
не запускается.

Полная проверка Nav2 через этот hardware-like API:

```bash
./scripts/mujoco_loco_nav_up.sh viewer_lite:=true publish_camera:=false
RVIZ_PROFILE=lite ./scripts/mujoco_rviz.sh
```

Цепочка будет такой: `Nav2 /cmd_vel → /api/sport/request → MuJoCo`, как
подготовка к замене sim-адаптера настоящим G1 high-level locomotion service.

## 1. Подготовка робота (Ubuntu 22.04 / Humble)

Установить ROS 2 Humble, затем зависимости:

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions python3-vcstool libyaml-cpp-dev \
  ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-generator-dds-idl
```

Скопировать эту папку на робота, перейти в неё и получить официальные Unitree messages:

```bash
vcs import src < unitree_ros2.repos
./scripts/build.sh
```

`vcs import` закреплён на официальном релизе `unitree_ros2 v0.3.0`; случайное обновление `master` не изменит wire contract.

## 2. Сеть

Робот и ноутбук должны быть в одном L2/VLAN, с одинаковым `ROS_DOMAIN_ID`. Для прямого Ethernet Unitree обычно использует сеть `192.168.123.0/24`; официальный пример задаёт ноутбуку адрес `192.168.123.99/24`.

На роботе найти интерфейс:

```bash
ip -br link
ip -br address
```

После сборки в каждом новом терминале робота:

```bash
cd /path/to/unitree
source scripts/hardware_env.sh eth0 wlan0   # внутренний и внешний интерфейсы робота
```

Скрипт фиксирует Humble, CycloneDDS и физический domain `0`. На ноутбуке
используется отдельный Humble/CycloneDDS Docker-образ из следующего раздела;
ноутбуку не нужны `unitree_hg` и `unitree_api`.

## 3. Запуск bridge на роботе

Для первого запуска на физическом G1 используется отдельный telemetry-only
launch. Он публикует модель, joint states, IMU, измеренную `/odom` и TF, но не
создаёт интерфейс управления движением:

```bash
source scripts/hardware_env.sh eth0 wlan0
ros2 launch g1_bridge hardware_telemetry.launch.py
```

`hardware_bringup.launch.py` и `bridge.launch.py` сохранены как совместимые
алиасы, но теперь также запускают только telemetry. Сам telemetry-бинарник не
линкуется с `unitree_api` и не содержит `/cmd_vel` или sport-request publisher.

Проверить результат и записать полный вывод в один файл:

```bash
./scripts/hardware_telemetry_check.sh
```

Пока реальный LiDAR не публикует облако, SLAM/Nav2 и управление движением на
роботе не запускаются. Текущий подтверждённый статус и точная процедура
описаны в [docs/HARDWARE.md](docs/HARDWARE.md).

Для G1 с DEX3-1 сначала проверить обратную связь обеих физических кистей:

```bash
ros2 topic echo /g1/dex3/left/joint_states --once
ros2 topic echo /g1/dex3/right/joint_states --once
```

Освободить пальцы от предметов и только затем разрешить управление кистями:

```bash
ros2 service call /g1/dex3/enable_control std_srvs/srv/SetBool "{data: true}"
```

Команда содержит ровно семь положений моторов в радианах и должна поступать
непрерывно. Пример малой тестовой цели для левой кисти:

```bash
ros2 topic pub -r 10 /g1/dex3/left/command sensor_msgs/msg/JointState \
  "{position: [0.0, 0.0, 0.1, -0.1, -0.1, -0.1, -0.1]}"
```

Немедленно снять программное управление обеими кистями:

```bash
ros2 service call /g1/dex3/stop std_srvs/srv/Trigger "{}"
```

Реальные DEX3-топики, пределы и watchdog описаны в
[docs/TOPICS.md](docs/TOPICS.md#real-dex3-1-hands). Не запускайте параллельно
официальный `g1_dex3_example` или другой publisher команд кистей.

Если `/lowstate` отсутствует, сначала исправить DDS/interface/domain. Управление при этом включить невозможно.

## 4. RViz на ноутбуке

Собрать Humble/CycloneDDS образ один раз и запустить RViz:

```bash
./scripts/hardware_rviz_build.sh
./scripts/hardware_lidar_rviz.sh
```

Физический motion-интерфейс вынесен в отдельный launch и остаётся
заблокированным, пока явно не переданы оба подтверждения:

```bash
ros2 launch g1_bridge hardware_motion.launch.py \
  motion_interface:=true allow_hardware_motion:=true
```

Даже после такого запуска управление остаётся выключенным до вызова
`/g1/enable_control`. Эту команду нельзя использовать, пока не завершена
пассивная проверка DDS, LiDAR, TF и watchdog на поддерживаемом роботе.

На роботе с опорой/страховкой и свободной зоной:

```bash
ros2 service call /g1/enable_control std_srvs/srv/SetBool "{data: true}"
```

Аварийно прекратить отправку команд:

```bash
ros2 service call /g1/enable_control std_srvs/srv/SetBool "{data: false}"
```

Остановка процесса bridge также отправляет нулевую скорость, но штатной кнопкой остановки следует считать `enable_control=false` и физический пульт G1.

## 5. Что нужно уточнить перед загрузкой на G1

- G1 EDU 23 DoF или 29 DoF;
- Ubuntu/ROS 2 на вычислителе робота;
- версия firmware/`ai_sport`;
- интерфейс, через который видны `lowstate` и ноутбук;
- нужен ли в GUI только state/teleop или также URDF-модель, камеры, LiDAR и карта.

Для 23 DoF нельзя просто использовать текущий список суставов: надо добавить отдельный YAML с фактическим соответствием motor ID конкретной комплектации.

## Ограничения первой версии

- На G1 `/odom` формируется только из измеренной
  `/state_estimator/odom_pelvis`; симуляционная одометрия не используется.
- Для Gazebo включён официальный URDF/mesh-набор G1 29 DoF с DEX3-1; перед реальным запуском всё равно надо сверить точную комплектацию робота.
- Для D435i добавлен отдельный hardware launch; остальные native камеры и LiDAR
  потребуют инвентаризации топиков на конкретной комплектации робота.

Официальная база: [Unitree ROS 2](https://github.com/unitreerobotics/unitree_ros2) (Humble recommended, CycloneDDS, G1 examples) и текущая реализация [G1 LocoClient](https://github.com/unitreerobotics/unitree_ros2/blob/master/example/src/include/g1/g1_loco_client.hpp), где velocity использует API `7105` с `velocity` и `duration`.
