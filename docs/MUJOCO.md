# MuJoCo DEX3 contact grasp

MuJoCo-стенд запускается отдельно от Gazebo. По умолчанию это сейчас
навигационный стенд A → B: SLAM/Nav2 включены, RViz запускается отдельно, а
распознавание и захват коробки не стартуют.

```bash
./scripts/mujoco_build.sh
./scripts/mujoco_up.sh
```

Если нужно проверить железо-совместимый high-level locomotion API, включите
эмулятор Unitree `/api/sport/request`:

```bash
./scripts/mujoco_up.sh loco_api:=true
```

Он принимает тот же ROS 2 wire contract, что реальный G1 `LocoClient`, и
отвечает в `/api/sport/response`. Поддерживаемый минимум: FSM/state queries,
`SetFsmId`, `SetBalanceMode`, `SetSwingHeight`, `SetStandHeight`,
`SetSpeedMode`, `SetVelocity` (`7105`) и stop/stand/move aliases. Внутри
симуляции это переводится в `geometry_msgs/Twist`; закрытый firmware-контроллер
Unitree в MuJoCo не запускается.

Ручной smoke-test:

```bash
./scripts/mujoco_loco_request.sh start
./scripts/mujoco_loco_request.sh move 0.15 0.0 0.0 1.0
./scripts/mujoco_loco_request.sh stop
```

Для более строгой проверки без прямого `/cmd_vel`-bypass можно запустить
симулятор так, чтобы он слушал только командный топик API-адаптера:

```bash
./scripts/mujoco_up.sh \
  loco_api:=true \
  loco_api_cmd_vel_topic:=/g1/sim/cmd_vel \
  sim_cmd_vel_topic:=/g1/sim/cmd_vel \
  sim_smoothed_cmd_vel_topic:=/g1/sim/cmd_vel_smoothed
```

Полная hardware-like цепочка с Nav2 включается отдельным скриптом:

```bash
./scripts/mujoco_loco_nav_up.sh viewer_lite:=true publish_camera:=false
RVIZ_PROFILE=lite ./scripts/mujoco_rviz.sh
```

В этом режиме `Navigation2 Goal` в RViz больше не управляет MuJoCo напрямую:

```text
RViz/Nav2 Goal
  -> Nav2 /cmd_vel
  -> g1_cmd_vel_loco_bridge
  -> /api/sport/request
  -> g1_loco_api_sim
  -> /g1/sim/cmd_vel
  -> MuJoCo odom/tf
```

Если нужна динамическая walking-policy вместо кинематического перемещения базы,
добавьте `walk:=true`:

```bash
./scripts/mujoco_loco_nav_up.sh walk:=true viewer_lite:=true publish_camera:=false
```

По умолчанию вместе с окном MuJoCo RViz больше не запускается. Виртуальная
грудная RGB-D камера остаётся доступной для последующего tabletop-pick режима.
Для камеры публикуются RealSense-совместимые топики:

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
mid360_scan -> /mid360/points + /scan -> SLAM Toolbox -> /map -> Nav2 -> /cmd_vel -> /odom + /tf
```

По умолчанию для навигации загружается отдельная MuJoCo-сцена
`g1_29dof_with_dex3_nav.xml`: стол в ней является препятствием для lidar/SLAM,
но не стоит вплотную к роботу и не используется как физический tabletop-grasp
контакт. Близкая сцена со столом и коробкой для захвата включается только через
`tabletop_pick:=true`.

По умолчанию `./scripts/mujoco_up.sh` запускает SLAM и Nav2, но не RViz.
RViz открывайте отдельным скриптом:

```bash
./scripts/mujoco_rviz.sh
```

В RViz используйте инструмент `Navigation2 Goal`; в nav-профиле отображаются
online SLAM `/map`, `/scan`, global/local costmap, `/plan`, footprint и
Navigation 2 panel. Это та самая стандартная 2D-карта, которая рисуется по
lidar/scan и запоминается SLAM Toolbox во время движения. `RVIZ_PROFILE=lite`
показывает тот же 2D Nav2-набор, но с облегчённой овальной моделью робота.
`RVIZ_PROFILE=full` оставлен для 3D-осмотра модели и облаков.

Для навигационной проверки с GUI на ноутбуке лучше сначала отключить RGB-D
renderer, если камера не нужна в этом прогоне:

```bash
./scripts/mujoco_up.sh publish_camera:=false
```

MuJoCo viewer останется включён, но D435i image/depth/points не будут
рендериться каждый кадр. Это снижает задержки TF/scan, из-за которых Nav2 может
сорвать цель под нагрузкой.

Если нужно вернуть старое поведение и поднять RViz из того же launch-файла:

```bash
./scripts/mujoco_up.sh rviz:=true
```

Но для обычной отладки лучше держать MuJoCo и RViz в разных терминалах. Если
RViz в Docker падает с `failed to create drawable` или `exit code -11`, просто
закройте отдельный RViz-контейнер; MuJoCo/Nav2 продолжат работать.

Цель можно отправлять тем же
`./scripts/mujoco_nav_goal.sh X Y`.

RViz можно запустить отдельным контейнером поверх уже работающего MuJoCo/Nav2:

```bash
./scripts/mujoco_rviz.sh
```

По умолчанию скрипт открывает стандартный 2D Nav2-профиль `RVIZ_PROFILE=nav`:
модель G1, SLAM map, `/scan`, global/local costmaps, footprint, `/plan` и
Navigation 2 panel. Полный 3D-профиль с D435i-панелями можно попробовать
отдельно:

```bash
RVIZ_PROFILE=full ./scripts/mujoco_rviz.sh
```

Для слабых машин (software GL / llvmpipe) есть лёгкий профиль с тем же 2D
Nav2-видом карты/костмапов, но с овальной моделью робота вместо тяжёлых meshes
(капсулы/простые примитивы вместо 51 текстурированного меша):

```bash
RVIZ_PROFILE=lite ./scripts/mujoco_rviz.sh
```

Овальная модель публикуется `description_relay` на `/robot_description_lite`.
MuJoCo-вьювер тоже можно разгрузить: в `viewer_lite` тяжёлые robot meshes
прячутся, робот рисуется овалами/капсулами, тени отключаются, а checker-сетка
пола остаётся включённой:

```bash
./scripts/mujoco_up.sh viewer_lite:=true
```

## Режим ходьбы (walk)

По умолчанию база робота в nav-сцене движется кинематически. Режим ходьбы
включает настоящую локомоцию: ноги ведёт предобученная policy Unitree
(12 DoF, `models/walk/g1_12dof_motion.pt` из
[unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym), BSD-3),
остальное тело удерживается жёстким PD:

```bash
./scripts/mujoco_up.sh walk:=true
# RViz как обычно, например:
RVIZ_PROFILE=lite ./scripts/mujoco_rviz.sh
./scripts/mujoco_nav_goal.sh 1.20 0.90 --feedback
```

Что меняется внутри:

- робот стоит на полу через реальные контакты (маски включаются на уровне
  компиляции MJCF — в MuJoCo 3.3.6 runtime-override масок не работает);
- гравитация реальная, демпфирование ног приведено к тренировочным
  значениям legged_gym;
- `/cmd_vel` → скорость policy (vx, vy, wz), inference 50 Гц, PD 500 Гц;
- `/odom` и TF строятся из фактического корня физики;
- когда входной `cmd_vel` нулевой дольше короткой задержки, policy не
  вызывается: ноги удерживаются PD-контроллером в дефолтной стойке, а
  floating-base фиксируется как симуляционный “park brake”, чтобы робот не
  “перетаптывался” и не падал без отдельной stand-policy;
- локальный velocity-серво компенсирует ошибку усиления скорости во время
  движения (политика обучена на 12-DoF теле без рук).

Ограничения: `walk` несовместим с `tabletop_pick`; робот физически не
сталкивается со столом/стенами (как и раньше — их держит costmap/Nav2);
падение детектируется по высоте таза и логируется. Скорость фактическая
ниже командуемой (~0.2 м/с при команде 0.35), Nav2 это компенсирует
замкнутым контуром.

`mujoco_rviz.sh` перед стартом RViz ждёт `/navigate_to_pose`, `/map` и
`/global_costmap/costmap`. Если нужно открыть RViz сразу, без ожидания Nav2:

```bash
RVIZ_WAIT_NAV=false ./scripts/mujoco_rviz.sh
```

Если RViz пишет `navigate_to_pose action server is not available`, проверьте,
что MuJoCo/Nav2 контейнер действительно запущен и видит action server:

```bash
docker exec unitree-g1-mujoco bash -lc \
  'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && ros2 action list | grep navigate_to_pose'
```

Если в одном терминале вы задавали `ROS_DOMAIN_ID` через `scripts/use_network.sh`,
запускайте MuJoCo и RViz из терминалов с тем же `ROS_DOMAIN_ID`; скрипты
пробрасывают этот env внутрь Docker.

Если снова появляется `failed to create drawable`, сначала пересоберите образ
после добавления Mesa-драйверов, затем попробуйте режим с полным доступом к GPU:

```bash
./scripts/mujoco_build.sh
RVIZ_GL=privileged ./scripts/mujoco_rviz.sh
```

Альтернативные режимы:

```bash
RVIZ_GL=llvmpipe ./scripts/mujoco_rviz.sh
RVIZ_GL=software ./scripts/mujoco_rviz.sh
RVIZ_GL=diagnose ./scripts/mujoco_rviz.sh
```

Когда нужно проверить SLAM/Nav2 без RViz, можно сохранить PNG-снимок текущих
`/scan`, `/odom`, `/plan` и costmap:

```bash
./scripts/mujoco_nav_snapshot.sh
```

Файл появится в `debug/mujoco_nav_snapshot.png`.

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
./scripts/mujoco_nav_goal.sh 1.20 0.90 --feedback
```

Скрипт перед отправкой цели ждёт `controller_server`, `planner_server`, первый
`/odom`, `/map`, `/scan`, `/local_costmap/costmap` и
`/global_costmap/costmap`. Это важно: action server Nav2 может стать доступен
раньше, чем SLAM успеет создать `map -> odom` и прогреть costmap, а ранняя цель
в этот момент часто завершается `ABORTED`.

Не используйте `1.0 0.0` для nav-сцены со столом: эта точка находится внутри
inflated-зоны стола. Для проверки обхода отправляйте цель сбоку от стола,
например `1.20 0.90` или `1.20 -0.90`.

Аргументы: `X Y [YAW_RAD]` в frame `map`. Например, `1.20 0.90` — пройти из
текущей точки A к точке B сбоку за столом. Если нужно видеть каждое сообщение
Nav2 feedback с текущей позой и оставшейся дистанцией:

```bash
./scripts/mujoco_nav_goal.sh --feedback 1.20 0.90
```

В MuJoCo-навигации локальный `FollowPath` переключён с дефолтного MPPI на
`RegulatedPurePursuitController`: он детерминированнее для текущей
кинематической модели G1, не семплирует задний ход вокруг препятствия и лучше
подходит для проверки простого прохода A → B.

Контракт навигации:

```text
/mid360/points sensor_msgs/msg/PointCloud2  3D Mid-360-like cloud, frame odom
/scan      sensor_msgs/msg/LaserScan    2D navigation projection of the Mid-360 cloud
/odom      nav_msgs/msg/Odometry        odom -> base_footprint
/tf        tf2_msgs/msg/TFMessage       dynamic odom -> base_footprint -> pelvis plus robot TF
/map       nav_msgs/msg/OccupancyGrid   online SLAM map
/plan      nav_msgs/msg/Path            Nav2 global plan
/cmd_vel   geometry_msgs/msg/Twist      Nav2 velocity command
```

Локальный `voxel_layer` отмечает препятствия по `/mid360/points`, поэтому
`/local_costmap/voxel_points` сохраняет фактическую высоту столов и других
объектов. Плоский `/scan` используется этим слоем только для очистки лучами и
отдельно остаётся входом SLAM Toolbox.

Важное ограничение: без `walk:=true` MuJoCo `/cmd_vel` двигает базу
кинематически в ROS TF/odom. С `walk:=true` ноги ведёт locomotion-policy, но
это всё ещё симуляционный bringup; idle park-brake нужен только MuJoCo. На
реальном G1 стояние и ходьба должны переключаться через штатные Unitree
FSM/API, а не через этот MuJoCo-bypass.
Контактный tabletop grasp остаётся отдельным физическим MuJoCo-стендом:
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
