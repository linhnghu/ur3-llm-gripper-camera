# Kiểm chứng Bài 03 — 05/10/2026

Môi trường: Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress 6.18, MoveIt 2, UR3e.
Build thành công các package `ur_simulation_gz`, `ur3_grasp_plugin`,
`ur3_draw_letter`, `ur3_llm_control`; 19/19 unit tests đạt.

## Kết quả integration thực tế

Lệnh: `Put the red cube in zone B.`. Chạy với `gazebo_gui:=true`,
`launch_rviz:=true`, `demo_plan:=true`. RViz khởi tạo OpenGL 4.6 thành công;
Gazebo GUI mở cùng simulation. Plan mẫu có nguồn `offline_fixture`, chưa gọi LLM.

| Bước | Bằng chứng |
| --- | --- |
| Quan sát ban đầu | Camera phát hiện đủ 5 cube; B chứa blue, C chứa purple, A trống |
| Gắp blue | Physics ACK `blue_cube attached` sau contact cả hai ngón |
| Đặt blue tạm | Physics ACK detached; camera xác nhận `(0.550021, -0.125285, 0.5)` m |
| Kiểm tra B | Camera xác nhận `occupants=[]` |
| Gắp red | Physics ACK `red_cube attached` sau contact cả hai ngón |
| Đặt red ở B | Physics ACK detached; camera xác nhận `(0.540321, 0.000808, 0.5)` m |
| Home và hậu điều kiện | `TASK SUCCESS — verified by camera`; B chỉ chứa red |

7/7 bước SUCCESS. 8/8 đường Cartesian đạt 100%; collision checking được bật.
Các vật chuyển động qua constraint của engine physics; không gọi set-pose.
Constraint hỗ trợ giữ vật sau contact, không phải mô phỏng grasp chỉ bằng ma sát.

## Files

- [demo_camera.mp4](demo_camera.mp4): 585 ảnh, 800×800, 10 FPS, 58.5 giây;
  video sensor RGB có caption occupancy và vật đang giữ. Không phải video quay desktop.
- [evidence.json](evidence.json): trạng thái camera đầu/cuối, plan gốc, plan mở rộng,
  buffer và kết quả từng skill.
- [execution.log](execution.log): log nguyên bản của lần chạy GUI.
- [initial.png](initial.png), [final.png](final.png): ảnh trích từ đầu/cuối video.

Launch được giới hạn 150 giây để kết thúc kiểm thử; log cuối có SIGINT/shutdown và
exit code -2 của Gazebo khi timeout dừng ứng dụng. Task đã SUCCESS trước thời điểm này.

## Tái hiện

```bash
cd ~/workspaces/ur_gz
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur3_llm_control llm_robot.launch.py \
  command:='Put the red cube in zone B.' demo_plan:=true \
  record_path:=/tmp/lesson3_demo.mp4 evidence_path:=/tmp/lesson3_evidence.json
```

Chạy LLM thật: đặt `NINEROUTER_API_KEY` trong môi trường, chọn endpoint/model phù hợp
và bỏ `demo_plan:=true`. Môi trường kiểm thử hiện chưa có API key nên phần gọi LLM
chưa được kiểm chứng trong lần chạy này. Mã nguồn và artifacts được đưa vào
[repository Bài 03](https://github.com/linhnghu/ur3-llm-gripper-camera).
Khi nộp cần quay thêm lệnh người dùng, phản hồi LLM, Gazebo và RViz.
