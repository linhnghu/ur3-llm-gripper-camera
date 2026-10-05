# Báo cáo Bài 03: LLM Skill Planning với Gripper và Camera

Sinh viên: **điền họ tên trước khi nộp**. MSSV: **23020749**.

## Mục tiêu và kiến trúc

Bài 03 kế thừa client LLM, JSON validator, UR description và MoveIt skills từ Bài 02.
Bổ sung perception, resolver kiểm tra occupancy, execution validator và physics
plugin kiểm tra contact. Các lớp có trách nhiệm riêng:

| Thành phần | Đầu vào | Đầu ra / trách nhiệm |
| --- | --- | --- |
| LLMPlanner | yêu cầu + trạng thái camera | chọn skill, object, zone và thứ tự |
| PlanValidator | JSON từ LLM | schema allow-list, pick/place hợp lệ, home cuối |
| CameraObserver | ảnh RGB | vị trí cube, occupancy; đợi ảnh mới và ổn định |
| Zone resolver | plan hợp lệ + trạng thái quan sát | chèn di chuyển vật cản, tính buffer |
| Execution Validator | plan mở rộng | kiểm tra holding state, occupancy và destination |
| RobotSkills | bước symbolic | MoveIt/action gripper, kiểm tra camera trước/sau |
| ContactGrasp | contact ngón + yêu cầu attach/detach | tạo/xóa constraint vật lý |

LLM không được quyền chọn pose, joint, trajectory hoặc địa chỉ dịch vụ điều khiển.

## Logic khi Zone B bị chiếm

Camera phân loại blue cube trong footprint Zone B. Resolver chọn một vị trí ngoài
zone từ grid 2.5 cm, loại các điểm gần block dưới 12 cm và gần footprint zone. Nếu
không còn vị trí, dừng trước khi robot chạy. Resolver chèn pick blue → place buffer
trước pick red → place B. Check_zone đọc lại camera để xác nhận B đã trống.

Mỗi pick lấy tọa độ mới từ camera. Mỗi place kiểm tra các block còn lại và đường đi
có collision checking. Khi thả, physics plugin xóa constraint, robot rút lên và về
tư thế quan sát; camera xác nhận cube thực sự tại đích. Trạng thái cuối được kiểm
tra một lần nữa trước khi ghi success.

Cách này dùng LLM để xác định ý định symbolic và logic xác định để xử lý tiền điều
kiện môi trường. Phản hồi LLM có thể chỉ chứa nhiệm vụ chính; resolver là logic
bổ sung được phép trong yêu cầu Bài 03.

## Gripper và camera

Ngàm hai ngón đóng/mở bằng ros2_control, có contact sensor riêng. Physics plugin
khởi tạo detached, chỉ cho phép tạo joint khi đồng thời có tiếp xúc hai phía với
cube được chọn. Không thay pose vật. Ràng buộc fixed giữ vật ổn định khi nâng và
vận chuyển; việc thả được engine physics xử lý. Đây là hỗ trợ mô phỏng grasp và
cần nêu rõ trong demo, không được mô tả là grasp chỉ bằng ma sát.

Camera nhìn vuông góc từ trên với intrinsics/extrinsics đã biết. HSV tách 5 màu;
mặt trên được chiếu xuống mặt phẳng đỉnh cube đã biết. Vị trí ban đầu SDF không
tham gia state của planner. Calibration và kích thước fixture được khai báo cố
định; vị trí block luôn đo từ ảnh. Zone occupancy gồm cả footprint block chạm zone.

## Kiểm tra và giới hạn

Unit tests kiểm tra thứ tự dọn zone, schema/holding state, thiếu vật, không có
buffer, đích bị chiếm, yêu cầu mâu thuẫn, vật đã ở đúng zone, ảnh trống/ambiguity
và vị trí quan sát thay đổi.

Ngày 05/10/2026 đã build và chạy thực tế trên ROS 2 Humble, Gazebo Fortress và UR3e.
19/19 unit tests đạt. Integration chạy với cả Gazebo GUI và RViz: 7/7 skill thành
công; 8/8 đường Cartesian đạt 100%; physics xác nhận attach/detach cho cả blue và
red. Camera ban đầu thấy Zone B chứa blue; sau khi dọn thấy B trống; cuối cùng thấy
B chứa red và cả 5 block trên bàn. Blue được quan sát ở `(0.550021, -0.125285, 0.5)`;
red ở `(0.540321, 0.000808, 0.5)` m. Robot về home trước khi ghi success.

Bằng chứng đã lưu: [video camera 58.5 giây](../../artifacts/lesson3/demo_camera.mp4),
[JSON kết quả](../../artifacts/lesson3/evidence.json),
[log thực thi](../../artifacts/lesson3/execution.log) và
[ghi chú kiểm chứng](../../artifacts/lesson3/VALIDATION.md).
Lần chạy này dùng `demo_plan=true`, nguồn plan được ghi là `offline_fixture`.
Chưa kiểm thử gọi LLM thật vì môi trường chưa có API key; video camera này chưa phải
video nộp đầy đủ có phản hồi LLM và màn hình RViz.

Perception giả định một cube cho mỗi màu, vật nằm phẳng trên bàn, không xếp chồng,
camera/ánh sáng cố định. Khi robot che vật hoặc thiếu ảnh, hệ thống dừng thay vì
suy ra zone trống. Miền tìm buffer được giới hạn nhưng vẫn có thể không tìm được
đường đi; MoveIt quyết định khả năng tiếp cận từng điểm. Hiện tại không tự thử một
buffer khác khi IK thất bại. Cảnh có vật lạ cần perception/occupancy rộng hơn.

## Bổ sung chế độ nhập lệnh liên tục

`continuous:=true` giữ planner hoạt động sau mỗi nhiệm vụ. CLI `llm_command` ở
terminal riêng gửi câu lệnh qua `/llm_command` và theo dõi `/llm_status`. Mỗi task có
ID riêng và lấy lại trạng thái camera; một worker thực thi tuần tự, từ chối lệnh
đến khi bận. Giữ nguyên RobotSkills giữa các task để không mất trạng thái ngàm.
Lỗi planning cho phép gửi lại; lỗi execution khóa phiên để tránh dùng trạng thái
robot chưa được xác nhận. Đây là điều khiển theo nhiệm vụ với độ trễ LLM/camera.

Đã chạy nhiều task trong cùng một phiên Gazebo: task đầu xử lý B bị chiếm; task
tiếp nhận ra red đã ở B. Kiểm chứng thêm busy rejection, chống chạy lại ID và phục
hồi lỗi planning. Tổng bộ unit tests sau bổ sung đạt 23/23.
[Bằng chứng chế độ liên tục](../../artifacts/realtime/VALIDATION.md).

## Bằng chứng và cách nộp

Lỗi camera trong chuỗi red → A rồi swap red/blue xảy ra khi nâng red từ A: tay máy
che một phần yellow, nên không đủ diện tích mặt trên để nhận dạng. Đã bổ sung
phục hồi góc nhìn trước place: chỉ khi ảnh RGB còn mới nhưng thiếu block, robot
giữ grasp, chuyển tới tư thế quan sát qua MoveIt có collision checking, rồi đọc
lại toàn bộ các vật không được giữ. Nếu không thể di chuyển hoặc vẫn thiếu block,
dừng và giữ trạng thái lỗi. Không dùng pose cố định/cached để bỏ qua kiểm tra đích.

Kiểm chứng lại với plan HTTP fixture mô phỏng đúng JSON trong log LLM của người
dùng: nhiệm vụ red → A đạt 5/5 bước; swap đạt 10/10 bước; camera cuối xác nhận
blue ở A, red ở B và đủ 5 cube. Bộ unit tests sau sửa đạt 29/29.
[Bằng chứng](../../artifacts/camera_recovery/VALIDATION.md). Fixture này không phải
lần gọi LLM thật mới; log lỗi gốc của người dùng đã được lưu để đối chiếu.

Dùng `record_path` và `evidence_path` để lưu camera video và JSON gồm trạng thái
đầu/cuối, plan mở rộng và kết quả từng skill. `demo_plan=true` là fixture kiểm thử,
phải ghi rõ nếu sử dụng; demo LLM thật phải chạy `demo_plan=false` với API được cấu
hình. Video nộp cần thấy lệnh người dùng, plan LLM, Zone B ban đầu có blue, thao tác
dọn blue, gắp/thả red vào B, RViz/MoveIt và home cuối cùng.

Repository Bài 03: https://github.com/linhnghu/ur3-llm-gripper-camera . Bao gồm đầy đủ
mã nguồn workspace, video camera, log và JSON kiểm chứng. Trước khi nộp cần điền
họ tên, quay demo LLM thật đầy đủ cửa sổ Gazebo/RViz, rồi bổ sung link video và
commit dùng để nộp.
