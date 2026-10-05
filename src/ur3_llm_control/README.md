# `ur3_llm_control` — thiết kế Bài 03

Package điều khiển UR3e bằng kế hoạch skill do LLM tạo, sử dụng camera RGB để
quan sát trạng thái bàn và xử lý zone bị chiếm trước khi gắp/thả.

**Build, kết nối LLM và các lệnh chạy:** xem [README chính](../../README.md).
Tài liệu này mô tả logic thực thi, giao tiếp ROS và giới hạn của mô hình.
Phân tích phục vụ báo cáo nằm trong [REPORT_BAI03.md](REPORT_BAI03.md).

## Thành phần

| File | Vai trò |
| --- | --- |
| [llm_planner.py](ur3_llm_control/llm_planner.py) | Gửi yêu cầu và trạng thái camera tới API chat completions, nhận JSON plan |
| [task_validator.py](ur3_llm_control/task_validator.py) | Kiểm tra schema, skill, đối tượng, zone và thứ tự gắp/thả |
| [perception.py](ur3_llm_control/perception.py) | Nhận RGB, nhận dạng cube và lấy snapshot mới, ổn định |
| [scene_state.py](ur3_llm_control/scene_state.py) | Tính occupancy, tìm buffer, mở rộng và kiểm tra execution plan |
| [robot_skills.py](ur3_llm_control/robot_skills.py) | Thực thi skill qua MoveIt/gripper và xác nhận bằng camera/physics ACK |
| [llm_task_node.py](ur3_llm_control/llm_task_node.py) | Điều phối task, cập nhật status và ghi evidence |
| [command_session.py](ur3_llm_control/command_session.py) | Quản lý task ID, trạng thái bận và FAULT |
| [command_cli.py](ur3_llm_control/command_cli.py) | Client nhập lệnh liên tục hoặc gửi một lệnh từ script |

## Plan của LLM và kiểm tra trước thực thi

LLM chỉ được sinh `pick(object)`, `place(object, zone)` và `home()`:

```json
{
  "plan": [
    {"skill": "pick", "object": "red_cube"},
    {"skill": "place", "object": "red_cube", "zone": "zone_b"},
    {"skill": "home"}
  ]
}
```

Đối tượng hợp lệ: `red_cube`, `yellow_cube`, `blue_cube`, `green_cube`,
`purple_cube`. Zone hợp lệ: `zone_a`, `zone_b`, `zone_c`.
Validator từ chối skill hoặc tham số lạ, pose/joint/trajectory, pick khi đang
giữ vật và place sai vật. Plan phải kết thúc bằng home sau khi đã thả vật.

MSSV được planner đọc từ tham số `student_id`. Hai chữ số cuối modulo 6 chọn
mapping; `23020749` tương ứng A=red, B=blue, C=yellow. Chỉ định đích rõ ràng
của người dùng được ưu tiên. `config/student_config.yaml` không được đọc trong
runtime; chỉnh file này chỉ cập nhật thông tin sinh viên/báo cáo.

## Xử lý zone bị chiếm

Sau khi gọi API, node lấy snapshot camera mới và duyệt plan trên bản sao trạng
thái bàn. Nếu zone đích có vật khác, resolver tìm buffer trong miền bàn cấu hình,
cách các block quan sát được ít nhất 12 cm và tránh các zone. Buffer do code tính,
không phải tọa độ LLM sinh. MoveIt vẫn phải tìm được IK và đường đi hợp lệ.

Với B chứa blue và yêu cầu đưa red vào B, plan mở rộng có dạng:

```text
detect_objects()
pick(blue_cube)
place(blue_cube, temporary_1)
check_zone(zone_b)
pick(red_cube)
place(red_cube, zone_b)
home()
```

Execution Validator kiểm tra lại holding state, occupancy và khoảng trống của
plan mở rộng. Trước pick/place, skill đọc lại camera và cập nhật collision scene
qua ApplyPlanningScene có xác nhận. Đích bị chiếm hoặc thiếu dữ liệu thì dừng.
Sau mỗi lần thả, robot rút ngàm, về tư thế quan sát và yêu cầu camera xác nhận vị
trí trong 2.5 cm. Cuối task, camera kiểm tra vật trong từng zone được yêu cầu.

## Camera và phục hồi khi bị che

Topic `/table_camera/image` nhận `sensor_msgs/Image` từ camera Gazebo. OpenCV
phân đoạn HSV theo năm màu, tìm tâm mặt trên cube và chiếu pixel xuống mặt phẳng
đỉnh cube bằng camera pinhole đã hiệu chuẩn. [scene.yaml](config/scene.yaml)
chứa hiệu chuẩn camera, kích thước cube/bàn, zone và miền buffer.
Tọa độ cube trong SDF chỉ tạo trạng thái ban đầu; bộ điều khiển không đọc model
pose Gazebo để lập kế hoạch.

`/environment_state` chứa vị trí nhận dạng, occupancy, nguồn `RGB_CAMERA`,
timestamp và cờ `complete`. Snapshot yêu cầu ít nhất **3 ảnh mới**, sai lệch vị
trí không quá **8 mm**, dữ liệu mới trong **2 giây**. Khi thiếu block, zone chưa
có occupant nhìn thấy được có giá trị `null` (unknown); không coi là trống.
Vật đang được giữ không dùng phép chiếu xuống mặt bàn; các vật còn lại vẫn phải
được quan sát trước khi đặt.

Trước place, skill thử đọc camera trong 2 giây. Nếu RGB vẫn mới nhưng thiếu
cube, robot giữ grasp và attached collision body, dùng MoveIt về tư thế quan sát
rồi đọc lại ảnh ổn định. Log ghi `CAMERA VIEW RECOVERY` và `CAMERA VIEW RECOVERED`.
Nếu ảnh không cập nhật hoặc vật chưa ổn định, code tiếp tục đợi theo timeout.
Đổi tư thế vẫn thiếu block, quá hạn hoặc motion thất bại đều dừng task.

Perception hiện dành cho cube đồng kích thước, mỗi màu một vật, đặt trên mặt bàn,
camera cố định và ánh sáng của world này. Chưa xử lý vật xếp chồng, cube nghiêng,
camera di chuyển hoặc vật lạ. Xem [bằng chứng lỗi che yellow khi swap và bản sửa](../../artifacts/camera_recovery/VALIDATION.md).

## Gripper, physics và MoveIt

Gripper có hai khớp prismatic, điều khiển bằng `gripper_controller` qua
FollowJointTrajectory với interface effort và PID bám vị trí. Hai ngón có
contact sensor. [ur3_grasp_plugin](../ur3_grasp_plugin) tạo fixed joint trong
engine vật lý **chỉ khi cả hai ngón đang tiếp xúc với đúng cube**.
Cube khởi tạo detached; mở ngàm xóa constraint, vật tiếp tục chịu physics.
Plugin không ghi pose, PoseCmd hoặc vận tốc để di chuyển cube. Đây là grasp
có hỗ trợ constraint sau contact, thay vì mô hình giữ vật chỉ bằng ma sát.

MoveIt AttachedCollisionObject biểu diễn vật đang giữ trong collision scene.
Các tiếp xúc finger–target được cho phép trong phạm vi thao tác; bàn và các
cube khác vẫn được kiểm tra va chạm. Tiếp xúc cube–bàn ở đầu đoạn nâng thẳng
đứng được cho phép tạm thời rồi khôi phục kiểm tra trước vận chuyển.

Cấu hình sử dụng group `ur_manipulator`, end effector `tool0`, frame `world`.
Đường Cartesian phải đạt fraction **100%** trước thực thi. Góc joint được chọn
trên nhánh liên tục gần trạng thái đo, sau đó kiểm tra lại giới hạn URDF và
GetStateValidity từng mẫu. LLM không sinh joint trajectory hay lệnh joint.

## Nhập lệnh liên tục khi mô phỏng đang chạy

Hướng dẫn hai terminal và API key: [Điều khiển bằng LLM](../../README.md#3-điều-khiển-bằng-llm).
Chế độ `continuous:=true` giữ planner/RobotSkills hoạt động giữa các nhiệm vụ;
mỗi task lấy camera state mới. Cùng ROS domain chỉ chạy một server điều khiển.

| Topic | Kiểu | Nội dung và QoS |
| --- | --- | --- |
| `/llm_command` | `std_msgs/String` | Câu lệnh hoặc JSON; reliable/volatile |
| `/llm_status` | `std_msgs/String` | JSON chứa task ID, trạng thái, bước và lỗi; reliable/transient-local |

Payload command có task ID:

```json
{"task_id": "my_task_1", "command": "Put the red cube in zone B."}
```

Lệnh tới khi robot bận bị REJECTED. Trong lịch sử 128 task, gửi lại cùng task ID
và câu lệnh trả trạng thái đã có, không thực thi lại. Hai terminal phải có cùng
`ROS_DOMAIN_ID`. Evidence mỗi task có tên riêng: đặt
`evidence_path:=/tmp/llm_task.json` sẽ tạo `/tmp/llm_task_<task_id>.json`.

Lỗi planning cho phép nhập lại; lỗi sau khi bắt đầu execution khóa phiên ở
FAULT. Kiểm tra mô phỏng và khởi động lại trước nhiệm vụ tiếp theo. Đóng CLI
bằng `exit`, EOF hoặc Ctrl+C không dừng task; Ctrl+C ở terminal launch kết thúc
mô phỏng. CLI hỗ trợ `--command`, `--connect-timeout` (mặc định 90 giây) và
`--timeout` (300 giây).

## Ghi bằng chứng và kiểm chứng

Ví dụ ghi demo mẫu khi chưa có API key, sau khi build/source workspace:

```bash
ros2 launch ur3_llm_control llm_robot.launch.py \
  command:='Put the red cube in zone B.' demo_plan:=true \
  record_path:=/tmp/lesson3_demo.mp4 evidence_path:=/tmp/lesson3_evidence.json
```

`record_path` ghi ảnh camera kèm occupancy; `evidence_path` ghi plan, buffer,
kết quả từng skill và camera state đầu/cuối. Thư mục output phải tồn tại.
Log thành công có physics ACK, `CAMERA VERIFIED` sau thả và
`TASK SUCCESS — verified by camera`; chỉ validate plan hoặc controller báo
thành công chưa chứng minh đã gắp/thả vật lý.

Bộ tests mới nhất có **29/29 tests đạt**. Các hồ sơ ghi số tests tại thời điểm
chạy riêng, nên demo đầu có 19 tests và lần kiểm chứng lệnh liên tục có 23 tests:

- [Demo Bài 03](../../artifacts/lesson3/VALIDATION.md): chạy Gazebo GUI/RViz, dọn blue rồi đặt red vào B, dùng offline fixture.
- [Chế độ liên tục](../../artifacts/realtime/VALIDATION.md): task liên tiếp, từ chối khi bận, chống chạy lại ID và lifecycle, dùng offline fixture.
- [Camera recovery](../../artifacts/camera_recovery/VALIDATION.md): hai lệnh red → A rồi swap red/blue, dùng HTTP fixture theo plan trong log LLM gốc; lần regression không gọi model thật.

Video camera có sẵn không quay desktop. Demo nộp bài cần chạy LLM thật với
endpoint/model/key hợp lệ và ghi thêm câu lệnh, phản hồi LLM, Gazebo và RViz.
