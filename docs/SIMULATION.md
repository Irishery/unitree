# Gazebo simulation

Симулятор использует ROS 2 Jazzy и Gazebo Harmonic внутри Docker. В образ
включены официальный URDF Unitree G1 29 DoF с двумя DEX3-1 и только необходимые
ему meshes. Все 14 суставов пальцев видны в Gazebo, RViz и `/g1/joint_states`.

## Запуск

Один раз собрать образ:

```bash
./scripts/sim_build.sh
```

Запустить Gazebo и RViz:

```bash
./scripts/sim_up.sh
```

Запустить так, чтобы робот стоял с руками вниз и ждал ручной команды захвата:

```bash
docker run --rm --name unitree-g1-gazebo --network host \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  unitree-g1-gazebo:jazzy \
  ros2 launch g1_gazebo sim.launch.py pick_auto_start:=false
```

Запуск без окна для CI/smoke test:

```bash
./scripts/sim_up.sh headless
```

## Проверка ROS 2

В другом терминале:

```bash
docker exec unitree-g1-gazebo bash -lc \
  'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && ros2 topic list'

docker exec unitree-g1-gazebo bash -lc \
  'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && ros2 topic echo /g1/imu/data --once'
```

Доступные интерфейсы:

- `/g1/joint_states` — 29 суставов;
- `/g1/imu/data` — IMU торса;
- `/g1/pose` — положение модели в мире;
- `/cmd_vel` — линейная и угловая скорость модели;
- `/camera/camera/color/image_raw` — RGB виртуальной D435i, 640x480, настроено на 30 Hz симуляционного времени;
- `/camera/camera/depth/image_rect_raw` — depth image;
- `/camera/camera/depth/color/points` — облако точек для RViz;
- `/camera/camera/imu` — IMU виртуальной D435i;
- `/g1/task/box_pose` — оцененный центр ближайшего depth-кластера в optical frame камеры;
- `/g1/task/box_pose_pelvis` — тот же центр, переведённый в систему `pelvis`;
- `/g1/task/bimanual_trajectory` — `JointTrajectory` из четырёх фаз, построенная через IK;
- `/g1/task/markers` — RViz-маркеры коробки, palm targets и линий pre-grasp/grasp/lift;
- `/g1/task/status` — состояние демонстрационного сценария захвата;
- `/g1/task/start` — ручной запуск пайплайна, если `pick_auto_start:=false`;
- `/g1/task/reset` — сброс демо, detach коробки и возврат рук вниз;
- `/tf`, `/tf_static`, `/clock` — стандартные ROS 2 данные.

RViz стартует автоматически: модель G1, TF, RGB и point cloud уже добавлены в
профиль. Виртуальная камера закреплена на штатном `d435_link` официального URDF.
Это приближённая RGBD-модель D435i: она воспроизводит ROS-интерфейс, разрешение,
частоту и поле зрения, но не моделирует стереопару, ИК-проектор, шум и аппаратную
калибровку настоящей камеры.

Gazebo использует camera-body оси (`+X` вперёд), а ROS optical frame — `+Z`
вперёд. Внутренний `/camera/camera/points_gz` остаётся в `d435_link`; relay
геометрически поворачивает точки и публикует внешний
`/camera/camera/depth/color/points` в `d435_color_optical_frame`. Поэтому
облако в RViz согласовано с моделью, а не только переименовано через `frame_id`.

Клавиатурное управление:

```bash
./scripts/sim_teleop.sh
```

## Демонстрация «стол — коробка — взять в руку»

В мире перед G1 стоят стол и оранжевая коробка. При старте `g1_tabletop_pick_demo`
удерживает обе руки в нулевой позе «по швам», получает публичное ROS-облако
`/camera/camera/depth/color/points` и в optical frame выделяет ближайший кластер
только по глубине. По bounding box кластера он оценивает центр объекта,
переводит его из `d435_color_optical_frame` в систему таза G1, строит левую и
правую целевые точки ладоней вокруг коробки и численно решает IK для трёх фаз:
`pre-grasp`, `bimanual grasp`, `lift`. После восьми устойчивых кадров узел
публикует найденный центр в `/g1/task/box_pose` и рассчитанную траекторию в
`/g1/task/bimanual_trajectory`: `arms-down → pre-grasp → bimanual grasp → lift`.
Исполнитель интерполирует её восемь секунд, закрепляет коробку на фазе grasp и
поднимает её обеими руками. RViz-профиль показывает `/g1/task/markers`: оранжевый
куб — оценка коробки, синие/зелёные точки — целевые положения ладоней, линии —
путь от pre-grasp к grasp и lift.

Если `pick_auto_start:=false`, восприятие и markers продолжают обновляться, но
траектория не стартует до команды:

```bash
docker exec unitree-g1-gazebo bash -lc \
  'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && \
   ros2 service call /g1/task/start std_srvs/srv/Trigger {}'
```

Сбросить захват и вернуть руки вниз:

```bash
docker exec unitree-g1-gazebo bash -lc \
  'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && \
   ros2 service call /g1/task/reset std_srvs/srv/Trigger {}'
```
Наблюдать этап можно так:

```bash
docker exec unitree-g1-gazebo bash -lc \
  'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && \
   ros2 topic echo /g1/task/status --qos-reliability reliable \
     --qos-durability transient_local --once'
```

Статусы идут в порядке `WAITING_FOR_DEPTH_BOX`, `IK_PLANNED_FROM_DEPTH`,
`BIMANUAL_GRASP`, `BOX_LIFTED`. Отдельные ROS-команды представлены в YAML bridge как
`/g1/task/joints/<joint>/command`, а attach/detach —
`/g1/task/grasp/{attach,detach}`. В статусе `IK_PLANNED_FROM_DEPTH` выводятся
оцененный центр коробки в системе таза и максимальная ошибка IK.

В симуляции используется официальная модель DEX3-1: обе кисти действительно
сгибают по семь моделируемых суставов во время фазы grasp. После контакта Gazebo
дополнительно создаёт detachable constraint с правой ладонью, потому что текущий
кинематический стенд не моделирует надёжно трение и силовое удержание коробки.
Это ограничение относится только к физике объекта, а не к геометрии кистей.
Детектор — геометрический depth-кластер в известной зоне
стола, а не нейросеть и не универсальная семантическая модель распознавания
объектов. IK считает суставы под найденный центр коробки, но ещё не делает
collision-aware planning как MoveIt 2. Нельзя переносить эти позы, тайминги или
захват на реального робота без отдельного восприятия, full-body/arm planning и
safety-проверки. Для реальных DEX3 предусмотрен отдельный `dex3_bridge_node`.

Остановить симулятор:

```bash
./scripts/sim_stop.sh
```

## Граница применимости

Это кинематический стенд для проверки ROS 2 GUI, топиков и командного контура,
а не симуляция динамической ходьбы. Официальный `unitree_ros` публикует G1 URDF,
инерции и геометрию, но не содержит Gazebo-контроллера баланса/ходьбы. Поэтому
в подготовленной модели гравитация для звеньев отключена, суставы сохраняют
стартовую позу, а `/cmd_vel` перемещает модель как единое целое.

Для реалистичной ходьбы следующим отдельным этапом нужен low-level controller:
`ros2_control` плюс контроллер равновесия/локомоции либо адаптация официального
Unitree Isaac Lab / MuJoCo policy. Такой контроллер нельзя считать эквивалентом
high-level `ai_sport` API реального робота без отдельной валидации.

## Физическая RealSense D435i

На компьютере, к которому камера подключена по USB 3, установить официальный
ROS wrapper (подставить `humble` на роботе или `jazzy` на ноутбуке):

```bash
sudo apt install ros-jazzy-realsense2-camera ros-jazzy-realsense2-description
```

После сборки workspace:

```bash
ros2 launch g1_gazebo d435i.launch.py
```

Launch включает RGB, depth, цветное облако точек и объединённую IMU, сохраняет
тот же namespace `/camera/camera/...` и присоединяет `camera_link` к штатному
`d435_link` G1. Для выбора конкретной камеры передать, например,
`serial_no:=_123456789`.
