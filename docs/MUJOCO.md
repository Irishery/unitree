# MuJoCo DEX3 contact grasp

MuJoCo-стенд запускается отдельно от Gazebo:

```bash
./scripts/mujoco_build.sh
./scripts/mujoco_up.sh
```

По умолчанию вместе с окном MuJoCo запускается RViz. В нём отображаются модель
G1 и облако точек от виртуальной грудной RGB-D камеры. Для камеры публикуются
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
