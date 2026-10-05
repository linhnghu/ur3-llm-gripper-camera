# Khắc phục camera mất yellow_cube khi swap — 05/10/2026

## Nguyên nhân xác nhận

[Log người dùng](original_user_run.log) ghi:
`put red to zone a` thành công; `swap red and blue` sinh plan red → B, blue → A.
Resolver dọn blue ra buffer, rồi gắp red từ A. Lỗi xảy ra ở trước place red, sau
physics ACK attached và thao tác nâng thành công.

Tái hiện cùng điểm gắp red ở A trong Gazebo, lỗi cũ lặp lại.
[Ảnh lúc lỗi](occluded_yellow.png) cho thấy tay máy che một phần mặt trên yellow.
HSV detector từ chối footprint bị che theo ngưỡng diện tích, không phải lỗi gọi LLM.

## Bản sửa và kết quả

Skill place thử lấy trạng thái mới trong 2 giây. Khi RGB còn mới nhưng thiếu cube,
giữ grasp và attached collision body, dùng MoveIt tới tư thế quan sát rồi đọc lại
3 ảnh mới ổn định. Không thay pose cube. Thiếu ảnh/không ổn định thì vẫn chờ với
timeout thông thường, không kích hoạt di chuyển phục hồi. Nếu đổi tư thế vẫn thiếu
block hoặc MoveIt thất bại, dừng trước place.

29/29 unit tests đạt. Integration mới chạy đúng chuỗi người dùng trên một phiên
Gazebo headless. HTTP server fixture trả đúng hai plan symbolic lấy từ log gốc;
không gọi một dịch vụ LLM thật trong lần regression này. Trường `planner_source`
trong JSON vẫn là `LLM` vì đi qua HTTP client; nguồn kiểm thử thực tế là
`camera-regression-fixture`, được ghi rõ ở [results.json](results.json).

- `put red to zone a`: 5/5 bước SUCCESS.
- `swap red and blue`: 10/10 bước SUCCESS.
- Recovery ghi missing yellow khi đang giữ red; sau đổi tư thế camera thấy đủ vật
  không được giữ. Grasp chỉ detached khi thả red vào B.
- Camera cuối: A chứa blue, B chứa red, C chứa purple; đủ cả 5 block, bao gồm yellow.

Xem [task đầu](first_task.json), [task swap](swap_task.json),
[log client](client.log), [log execution](execution.log),
[ảnh sau swap](after_swap.png).

Sau cập nhật cần dừng launch cũ, build/source lại `ur3_llm_control` và khởi động lại
mô phỏng. Session cũ đã FAULT và có thể còn giữ red; gửi lại lệnh trên session đó
không thay thế việc tải code mới và reset world.
