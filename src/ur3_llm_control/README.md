# Bài 03 — LLM Skill Planning với Gripper và Camera

Luồng điều khiển: ngôn ngữ tự nhiên → LLM (JSON symbolic plan) → Plan Validator →
đọc trạng thái camera → xử lý zone bị chiếm → kiểm tra execution plan → Robot Skills →
MoveIt 2 → UR3e + gripper trong Gazebo Fortress.

World có một bàn, camera RGB nhìn từ trên, 3 zone và 5 cube màu. Ban đầu `blue_cube`
chiếm Zone B, `purple_cube` ở Zone C; red/yellow/green nằm ngoài zone. Các tọa độ ban
đầu trong SDF chỉ dùng để tạo world. Code điều khiển **không đọc model pose Gazebo** và
không dùng tọa độ cube khai báo sẵn làm dữ liệu lập kế hoạch.

## Build và chạy

```bash
cd ~/workspaces/ur_gz
source /opt/ros/humble/setup.bash
colcon build --packages-select ur_simulation_gz ur3_grasp_plugin ur3_draw_letter ur3_llm_control --symlink-install
source install/setup.bash
# Đặt NINEROUTER_API_KEY trong môi trường; không commit hoặc ghi key vào video.
ros2 launch ur3_llm_control llm_robot.launch.py \
  command:='Put the red cube in zone B.'
```

Một launch khởi động Gazebo, MoveIt, camera bridge, arm/gripper controller, RViz và
task node. Task node bắt đầu sau khoảng 40 giây; tiếp tục đợi ảnh mới và ổn định.
RViz cấu hình sẵn màn hình `Table Camera`. Tư thế khởi tạo/home Bài 03 đặt tay máy
bên cạnh vùng thao tác để camera nhìn đủ block.

Có thể chỉ định gateway của Bài 02 bằng `endpoint:=http://localhost:20128/v1/chat/completions`
và `model:=<model-trong-gateway>`. `execute:=false` vẫn đọc camera, gọi LLM và kiểm tra
plan mở rộng nhưng không thực thi. Sửa tên sinh viên trong `config/student_config.yaml`;
ID mặc định 23020749 giữ mapping A=red, B=blue, C=yellow. Với 5 cube, yêu cầu
`Arrange all objects according to my student ID.` chỉ sắp xếp 3 cube trong mapping;
2 cube còn lại có thể được di chuyển nếu chắn zone đích.

## Nhập lệnh liên tục khi mô phỏng đang chạy

Chế độ `continuous:=true` giữ task node hoạt động và nhận lệnh mới qua ROS.
Mỗi lệnh dùng camera hiện tại, gọi LLM, kiểm tra plan rồi mới chạy. Thời gian phản
hồi phụ thuộc camera, mạng và model; robot xử lý một nhiệm vụ tại một thời điểm.

Terminal 1: đặt `NINEROUTER_API_KEY`, source ROS/workspace rồi chạy:

```bash
ros2 launch ur3_llm_control llm_robot.launch.py \
  continuous:=true demo_plan:=false \
  endpoint:='http://localhost:20128/v1/chat/completions' \
  model:='TEN_MODEL_TRONG_GATEWAY' \
  evidence_path:=/tmp/llm_task.json
```

Thay tên model bằng ID trong gateway. Gateway local phải đang chạy và API key phải
được gateway chấp nhận. Nếu dùng dịch vụ khác, thay endpoint và model tương ứng.
`execute:=false` có thể dùng để kiểm tra plan mà chưa thực hiện thao tác.

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/workspaces/ur_gz/install/setup.bash
ros2 run ur3_llm_control llm_command
```

Nhập yêu cầu sau dấu `LLM >`, ví dụ:

```text
Put the red cube in zone B.
Put the green cube in zone A.
Move the purple cube to zone C.
exit
```

CLI đợi server sẵn sàng, gửi lệnh có task ID và hiển thị ACCEPTED → PLANNING →
VALIDATED → EXECUTING → SUCCEEDED. Với `execute:=false`, kết quả là PLANNED.
Sau kết quả có thể nhập lệnh tiếp mà không restart Gazebo. Gửi một lệnh từ script:

```bash
ros2 run ur3_llm_control llm_command --command 'Put the red cube in zone B.'
```

`/llm_command` nhận `std_msgs/String`, chứa câu lệnh hoặc JSON
`{"task_id":"my_task_1","command":"Put the red cube in zone B."}`.
`/llm_status` xuất JSON với task ID, trạng thái, bước hiện tại và lỗi nếu có.
Hai terminal phải dùng cùng `ROS_DOMAIN_ID`. Status dùng reliable/transient-local;
command dùng reliable/volatile để lệnh cũ không phát lại khi server khởi động.
Tham khảo [ROS 2 QoS](https://github.com/ros2/ros2_documentation/blob/humble/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst).

Lệnh đến khi robot bận bị REJECTED, không xếp hàng cho một trạng thái bàn đã cũ.
Trong lịch sử 128 task, gửi lại cùng task ID/câu lệnh chỉ trả trạng thái trước đó;
không chạy lại. Evidence được ghi riêng thành `/tmp/llm_task_<task_id>.json`.
RobotSkills và trạng thái vật đang giữ được giữ nguyên giữa các nhiệm vụ. Lỗi
planning cho phép nhập lại; lỗi sau khi bắt đầu execution đưa session sang FAULT,
không nhận nhiệm vụ tiếp cho tới khi kiểm tra và restart simulation.

`exit`, EOF hoặc Ctrl+C trong CLI chỉ đóng cửa sổ nhập; nhiệm vụ đang chạy vẫn tiếp
tục ở tiến trình launch. Để kết thúc phiên mô phỏng, Ctrl+C ở terminal launch.

Đã kiểm chứng chế độ liên tục với nhiều task trên cùng một phiên Gazebo và 23 unit
tests. Xem [bằng chứng](../../artifacts/realtime/VALIDATION.md). Integration này dùng
plan mẫu; gateway local yêu cầu API key nên chưa kiểm chứng LLM thật.

## Camera và trạng thái

`/table_camera/image` là `sensor_msgs/Image` được bridge từ sensor Gazebo. OpenCV
phân đoạn HSV theo 5 màu, lấy tâm mặt trên và chiếu pixel xuống mặt phẳng đỉnh cube
bằng camera pinhole đã hiệu chuẩn. `config/scene.yaml` chỉ chứa camera, kích thước
cube, bàn, zone và giới hạn tìm buffer; không chứa pose ban đầu của block.

`/environment_state` chứa vị trí nhận dạng, occupancy từng zone, nguồn `RGB_CAMERA`,
image timestamp và cờ `complete`. Một snapshot dùng ít nhất 3 ảnh mới, kiểm tra vật
ổn định trong 8 mm và dữ liệu mới trong 2 giây. Khi thiếu/che khuất block, code đợi
và dừng nếu quá hạn. Không suy đoán rằng vật không nhìn thấy nghĩa là zone trống.
Khi ảnh chưa đầy đủ, zone không có occupant nhìn thấy được xuất `null` (unknown),
không xuất danh sách trống. Video cũng hiển thị `unknown` trong trường hợp này.

Thiết kế perception dành cho cube đồng kích thước, màu duy nhất, nằm trên mặt bàn,
camera cố định và ánh sáng world này. Nó chưa xử lý vật xếp chồng, cube nghiêng, camera
di chuyển hoặc vật lạ không thuộc 5 màu. Vật đang được giữ không dùng phép chiếu mặt
bàn; các vật còn lại vẫn phải nhìn thấy trước khi đặt.

Khi đang giữ vật, tay máy có thể che cube khác trong ảnh nhìn từ trên, đặc biệt
sau khi gắp red từ Zone A. Trước place, skill thử đọc ảnh mới trong 2 giây. Nếu
ảnh vẫn cập nhật nhưng thiếu block, robot giữ nguyên grasp và dùng MoveIt chuyển
sang tư thế quan sát, sau đó yêu cầu 3 ảnh mới ổn định có đủ các block còn lại.
Log ghi `CAMERA VIEW RECOVERY` và `CAMERA VIEW RECOVERED`. Nếu camera mất ảnh,
vật chưa ổn định hoặc block vẫn thiếu sau đổi tư thế, hệ thống tiếp tục chờ theo
timeout rồi dừng; không dùng pose cũ hay suy đoán zone trống để đặt vật.
Chuỗi `put red to zone a` → `swap red and blue` đã được kiểm chứng trong Gazebo;
xem [bằng chứng sửa lỗi camera](../../artifacts/camera_recovery/VALIDATION.md).

## Plan và skill

LLM chỉ được tạo `pick(object)`, `place(object, zone)`, `home()`. Validator từ chối
skill/object/zone lạ, pose/joint/trajectory, tham số thừa, pick khi đang giữ vật,
place sai vật, hoặc home trước khi thả. `home` phải là bước cuối.

Resolver duyệt plan trên bản sao trạng thái camera. Nếu zone đích đang có vật khác,
resolver tìm ô trống trong miền bàn có thể tiếp cận, giữ khoảng cách 12 cm tới các
block và tránh footprint zone; sau đó chèn pick/place vật cản. Buffer là kết quả
tính toán từ vị trí **quan sát được**, không phải tham số do LLM tự phát minh. MoveIt
vẫn kiểm tra IK và đường đi tới buffer; không có IK/đường đi hợp lệ thì dừng.

Với yêu cầu đưa red vào Zone B, execution plan có dạng:

```text
detect_objects()
pick(blue_cube)
place(blue_cube, temporary_1)
check_zone(zone_b)
pick(red_cube)
place(red_cube, zone_b)
home()
```

Execution Validator mô phỏng lại holding state và occupancy của plan mở rộng.
Trước mỗi pick/đặt, skill đọc lại camera và cập nhật collision scene qua dịch vụ
ApplyPlanningScene có xác nhận. Đích đặt bị chiếm hoặc thiếu dữ liệu camera thì dừng.
Sau mỗi lần thả, robot rút ngàm và về tư thế quan sát để camera xác nhận vị trí vật
trong 2.5 cm. Cuối nhiệm vụ camera phải xác nhận đúng vật trong từng zone yêu cầu.

## Gắp/thả vật lý

Gripper có 2 khớp prismatic được `gripper_controller` điều khiển qua
FollowJointTrajectory, với interface effort và PID bám vị trí. Hai ngón có contact sensor. Package `ur3_grasp_plugin` tạo
ràng buộc fixed joint của engine vật lý **chỉ khi cả hai ngón đang tiếp xúc với đúng
cube**. Mọi cube khởi tạo ở trạng thái detached. Khi mở ngàm, constraint bị xóa;
vật rơi/đặt xuống bàn theo physics. Plugin không ghi Pose, PoseCmd, vận tốc hoặc gọi
set-pose để di chuyển cube. Đây là mô hình giữ vật có hỗ trợ constraint, không phải
mô phỏng chỉ bằng lực ma sát ngàm.

MoveIt AttachedCollisionObject chỉ biểu diễn vật đang giữ trong collision scene;
nó không di chuyển vật Gazebo. Các tiếp xúc finger–target được allow có phạm vi.
Zone là ký hiệu trực quan, không phải chướng ngại vật giả. Bàn và các cube khác vẫn
được kiểm tra va chạm khi tiếp cận, hạ ngàm, nâng và đặt. Riêng tiếp xúc cube–bàn
ở đầu thao tác nâng được allow trong đoạn nâng thẳng đứng, rồi reset trước vận chuyển.

MoveIt phải trả Cartesian fraction 100% trước khi thực thi. Góc joint được quy về
nhánh liên tục gần trạng thái đo; từng mẫu sau đó được kiểm tra lại giới hạn trong
URDF và GetStateValidity. LLM không sinh trajectory hoặc joint command.

## Kiểm thử và video

Kiểm thử mô phỏng không cần API key dùng **plan mẫu cố định**, để tách kiểm chứng
camera/gripper/MoveIt khỏi dịch vụ LLM. Đây chưa phải demo hiểu ngôn ngữ bằng LLM:

```bash
ros2 launch ur3_llm_control llm_robot.launch.py \
  command:='Put the red cube in zone B.' demo_plan:=true \
  record_path:=/tmp/lesson3_demo.mp4 evidence_path:=/tmp/lesson3_evidence.json
```

`record_path` ghi video ảnh camera kèm occupancy; `evidence_path` ghi plan, buffer,
kết quả từng skill và trạng thái camera đầu/cuối. Để nộp demo LLM, bỏ `demo_plan:=true`,
bật Gazebo/RViz và quay cả yêu cầu, JSON plan/log, gripper, cảnh mô phỏng. File camera
đơn lẻ không hiển thị cửa sổ RViz hay phản hồi API. Thư mục output phải tồn tại.

Chạy unit tests:

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH="src/ur3_llm_control:$PYTHONPATH" /usr/bin/python3 -m pytest src/ur3_llm_control/test -q
```

Log thành công phải có đủ bước xử lý blue, `CAMERA VERIFIED` cho mỗi vật thả, và
`TASK SUCCESS — verified by camera`. Nếu chỉ có `VALIDATED EXECUTION PLAN`, chưa thể
kết luận gắp/thả vật lý thành công. Xem [báo cáo thiết kế](REPORT_BAI03.md).

Đã kiểm chứng ngày 05/10/2026: 19 tests đạt; chạy với Gazebo GUI/RViz hoàn thành
7 skills và 8 đường Cartesian 100%. Bằng chứng nằm trong
[artifacts/lesson3](../../artifacts/lesson3/VALIDATION.md), gồm video camera, JSON và
log. Plan của lần chạy này là `offline_fixture`; API LLM thật chưa được kiểm thử.

Tham khảo API physics/sensor chính thức:
[Gazebo DetachableJoint](https://gazebosim.org/api/sim/10/classgz_1_1sim_1_1systems_1_1DetachableJoint.html),
[Gazebo Fortress Sensors](https://gazebosim.org/docs/fortress/sensors/).
