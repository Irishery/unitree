# MuJoCo DEX3 contact grasp

MuJoCo-стенд запускается отдельно от Gazebo. По умолчанию это сейчас
навигационный стенд A → B: SLAM/Nav2/RViz включены, а распознавание и захват
коробки не стартуют.

```bash
./scripts/mujoco_build.sh
./scripts/mujoco_up.sh
```

По умолчанию вместе с окном MuJoCo запускается RViz. В нём отображаются модель
G1, `/scan`, карта, costmaps и Nav2 Goal. Виртуальная грудная RGB-D камера
остаётся доступной для последующего tabletop-pick режима. Для камеры публикуются
RealSense-совместимые топики:

```text
/camera/camera/color/image_raw
/camera/camera/depth/image_rect_raw
/camera/camera/color/camera_info
/camera/camera/depth/camera_info
/camera/camera/depth/color/points
```

Кадр облака `d435_color_optical_frame` связан с фиксированным штатным местом
в передней части головы (`torso_link`) через TF. Это модель камеры 640×480 с
углом обзора 69°; это виртуальный RGB-D сенсор, а не подключение физической
D435.

## SLAM and Nav2 in MuJoCo

MuJoCo-стенд также поднимает штатный для G1 навигационный контур:

```text
mid360_scan -> /scan -> SLAM Toolbox -> /map -> Nav2 -> /cmd_vel -> /odom + /tf
```

По умолчанию для навигации загружается отдельная MuJoCo-сцена
`g1_29dof_with_dex3_nav.xml`: стол в ней является препятствием для lidar/SLAM,
но не стоит вплотную к роботу и не используется как физический tabletop-grasp
контакт. Близкая сцена со столом и коробкой для захвата включается только через
`tabletop_pick:=true`.

По умолчанию `./scripts/mujoco_up.sh` запускает SLAM, Nav2 и RViz. В RViz
используйте инструмент `Nav2 Goal`; отображаются `map`, `/scan`, global/local
costmap и `/plan`.

Для навигационной проверки с GUI на ноутбуке лучше сначала отключить RGB-D
renderer, если камера не нужна в этом прогоне:

```bash
./scripts/mujoco_up.sh publish_camera:=false
```

MuJoCo viewer и RViz останутся включены, но D435i image/depth/points не будут
рендериться каждый кадр. Это снижает задержки TF/scan, из-за которых Nav2 может
сорвать цель под нагрузкой.

Если RViz в Docker падает с `failed to create drawable` или `exit code -11`,
оставьте только окно MuJoCo:

```bash
./scripts/mujoco_up.sh publish_camera:=false rviz:=false
```

Nav2/SLAM при этом продолжают работать, а цель можно отправлять тем же
`./scripts/mujoco_nav_goal.sh X Y`.

RViz можно запустить отдельным контейнером поверх уже работающего MuJoCo/Nav2:

```bash
./scripts/mujoco_rviz.sh
```

Для headless-проверки без окна MuJoCo, RViz и RGB-D renderer:

```bash
docker run --rm --name unitree-g1-mujoco --network host --ipc=host \
  -e MUJOCO_GL=egl \
  unitree-g1-mujoco:jazzy \
  ros2 launch g1_mujoco sim.launch.py \
    viewer:=false rviz:=false publish_camera:=false navigation:=true slam:=true tabletop_pick:=false
```

Отправить цель B в уже запущенный контейнер:

```bash
./scripts/mujoco_nav_goal.sh 1.0 0.0
```

Скрипт перед отправкой цели ждёт `controller_server`, `planner_server`, первый
`/odom`, `/map`, `/scan`, `/local_costmap/costmap` и
`/global_costmap/costmap`. Это важно: action server Nav2 может стать доступен
раньше, чем SLAM успеет создать `map -> odom` и прогреть costmap, а ранняя цель
в этот момент часто завершается `ABORTED`.

Аргументы: `X Y [YAW_RAD]` в frame `map`. Например, `1.0 0.0` — проехать из
текущей точки A примерно на метр вперёд. Если нужно видеть каждое сообщение
Nav2 feedback с текущей позой и оставшейся дистанцией:

```bash
./scripts/mujoco_nav_goal.sh --feedback 1.0 0.0
```

Контракт навигации:

```text
/scan      sensor_msgs/msg/LaserScan    360° Mid-360-like scan, frame mid360_scan
/odom      nav_msgs/msg/Odometry        odom -> pelvis
/tf        tf2_msgs/msg/TFMessage       dynamic odom -> pelvis plus robot TF
/map       nav_msgs/msg/OccupancyGrid   online SLAM map
/plan      nav_msgs/msg/Path            Nav2 global plan
/cmd_vel   geometry_msgs/msg/Twist      Nav2 velocity command
```

Важное ограничение: в MuJoCo `/cmd_vel` пока двигает базу кинематически в ROS
TF/odom, а не включает динамическую ходьбу G1. Это сделано, чтобы отладить
SLAM/Nav2/RViz и интерфейс управления на том же топик-контракте, что и Gazebo.
Контактный tabletop grasp при этом остаётся физическим MuJoCo-стендом:
коробка свободная, без `attach`/`weld`.

Визуальная модель в окне MuJoCo тоже следует этой кинематической базе через
`floating_base_joint`. Для Nav2 симулятор берёт скорость с `/cmd_vel_smoothed`
после `velocity_smoother`; обычный `/cmd_vel` остаётся fallback для ручных
команд, когда Nav2 не запущен.

Чтобы явно вернуться к режиму распознавания и захвата коробки:

```bash
./scripts/mujoco_up.sh tabletop_pick:=true
```

Он использует официальный `g1_29dof_with_hand_rev_1_0.xml` Unitree и добавляет
стол, свободную коробку массой 0.25 kg, gravity, collision и friction. В сцене
нет `attach`, `weld` коробки или другого механизма, меняющего её движение после
контакта. Руки удерживают коробку только нормальными силами и трением пальцев.

Геометрия текущего стенда: стол `0.600 × 1.400 м` (длина × ширина), верх на
`0.755 м`, зазор до переднего габарита робота `0.20 м` и коробка
`0.255 × 0.370 × 0.090 м` (длина × ширина × высота).

Для воспроизводимого теста таз G1 имеет фиксированное основание, а ноги и корпус
исключены из contact-physics; это не баланс-контроллер и не модель ходьбы.
Контакты остаются у DEX3, стола, пола и коробки. Управляйте суставами рук через:

При запуске руки находятся в вертикальной исходной позе: плечевой pitch/yaw и
запястья равны нулю, elbow — `1.50 рад`, а shoulder roll симметрично задан
`±0.25 рад`, чтобы вынести руки немного наружу от корпуса и ног. При elbow `0`
механический offset официальной MJCF ориентирует предплечье вперёд. Поза задаётся
до первого шага физики, поэтому стартового движения рук к ней нет.

Пальцы DEX3 стартуют в симметричной полусжатой позе. Это только визуальная
исходная конфигурация: для захвата контроллер подаёт отдельные команды в
`/g1/dex3/{left,right}/command`.

```bash
ros2 topic pub --once /g1/mujoco/joints/left_hand_index_0_joint/command \
  std_msgs/msg/Float64 "{data: -0.4}"
```

Или используйте тот же интерфейс, что и у hardware bridge:

```bash
ros2 topic pub --once /g1/dex3/left/command sensor_msgs/msg/JointState \
  "{position: [0.0, 0.2, 0.8, -0.8, -0.8, -0.8, -0.8]}"
```

Проверяйте состояние контактов:

```bash
ros2 topic echo /g1/mujoco/hand_box_contacts
ros2 topic echo /g1/mujoco/physical_grasp
```

`physical_grasp=true` означает минимум две точки контакта кисть–коробка. Это
наблюдаемый контактный сигнал, а не гарантия, что коробка выдержит подъём;
реальность удержания проверяется её динамикой после движения руки.

## RGB-D perception

`g1_box_detector` сегментирует красную коробку в RGB, использует медиану
зарегистрированной depth и публикует её центр в системе камеры:

```text
/g1/perception/box_pose       geometry_msgs/msg/PoseStamped
/g1/perception/box_detected   std_msgs/msg/Bool
/g1/perception/box_marker     visualization_msgs/msg/Marker
```

Зелёная сфера `Detected Box Centre` в RViz проверяет контур `RGB-D → 3D pose`.
Следующие IK и RL-узлы должны использовать этот pose, а не координаты объекта
напрямую из MuJoCo.

## Reference pick controller

`g1_pick_controller` — безопасный эталонный автомат `idle → pregrasp → close
→ lift`. Он использует RGB-D pose как обязательное условие старта, но не
включается сам. Запуск:

```bash
ros2 topic pub --once /g1/pick/start std_msgs/msg/Bool "{data: true}"
ros2 topic echo /g1/pick/status
```

Подъём разрешён только после `/g1/mujoco/physical_grasp=true`; иначе автомат
переходит в `failed`. Его траектории и контактные логи — baseline для
последующего Gymnasium/RL, не политика для реального робота.

Текущая baseline-поза подведения рассчитана для заданной сцены: пальцы проходят
над столешницей и снаружи боковых граней коробки. Для другой позиции коробки
нужен следующий этап — IK от `/g1/perception/box_pose`; не используйте этот
фиксированный baseline на реальном роботе.

## Reinforcement learning

`G1PickEnv` — headless Gymnasium-окружение с теми же MuJoCo-контактами. Его
наблюдение: pose коробки, относительные позиции пальцев, положения и скорости
суставов; действие: ограниченное приращение 14 arm + 14 DEX3 target-суставов.
Reward поощряет подведение, контакты и высоту коробки, а падение завершает эпизод.

После пересборки образа запустите обучение без GUI:

```bash
mkdir -p models
docker run --rm --network host -v "$PWD/models:/ws/models" unitree-g1-mujoco:jazzy \
  ros2 run g1_mujoco train_rl --timesteps 1000000 --envs 8
```

`--envs 8` запускает восемь независимых headless MuJoCo-процессов и собирает
опыт параллельно. Выбирайте число не выше доступных ядер CPU; например,
`nproc` покажет их количество. Checkpoint'ы сохраняются в
`models/checkpoints/` каждые 100 000 aggregate steps. Для более точной, но
медленной физики увеличьте `--physics-steps` с быстрого значения `5` до `10`.

На первом этапе policy получает 3D pose коробки из состояния симулятора: это
позволяет сначала отладить именно контактный захват. В ROS-исполнителе этот же
элемент наблюдения будет заменён на pose от RGB-D (`/g1/perception/box_pose`) с
преобразованием системы координат. Это не end-to-end обучение по пикселям.
После стабильного захвата policy будет подключена к ROS-исполнителю вместо
фиксированных целей `g1_pick_controller`.

### Curriculum

Не продолжайте checkpoint, созданный до curriculum: он обучался из положения
рук у бёдер и поэтому закрепляет вращение суставов вместо контакта. Обучайте
фазы последовательно, передавая модель предыдущей фазы через `--load`:

1. `grasp` — руки зафиксированы в безопасной pregrasp-позе; policy учит
   смыкание DEX3 и физический подъём. Коробка почти в центре.
2. `approach` — руки начинают на 55% пути к pregrasp, а разброс коробки шире.
3. `full` — руки по швам и полный разброс положения коробки.

Для первой фазы:

```bash
docker run --rm --network host -v "$PWD/models:/ws/models" unitree-g1-mujoco:jazzy \
  ros2 run g1_mujoco train_rl --stage grasp --timesteps 500000 --envs 8 \
  --output /ws/models/g1_pick_grasp
```

После устойчивого захвата переходите к следующей фазе:

```bash
docker run --rm --network host -v "$PWD/models:/ws/models" unitree-g1-mujoco:jazzy \
  ros2 run g1_mujoco train_rl --stage approach --timesteps 500000 --envs 8 \
  --load /ws/models/g1_pick_grasp.zip --output /ws/models/g1_pick_approach
```

## Visual RL evaluation

Не останавливая обучение, уже сохранённый checkpoint можно проиграть в окне
MuJoCo. Например, для checkpoint после 200 000 aggregate steps:

```bash
./scripts/mujoco_eval_rl.sh models/checkpoints/g1_pick_sac_200000_steps.zip --episodes 10 --realtime
```

В терминал выводятся контакты, высота коробки и успешность каждого эпизода.
Закройте окно MuJoCo для досрочной остановки. Оценка не меняет checkpoint и не
запускает обучение.
