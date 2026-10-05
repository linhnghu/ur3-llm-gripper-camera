# Chế độ lệnh liên tục — kiểm chứng 05/10/2026

23/23 unit tests đạt, gồm kiểm tra nhiều luồng gửi lệnh cùng lúc, chống chạy lại
task ID, phục hồi lỗi planning, khóa session khi execution lỗi và giới hạn lịch sử.
Package `ur3_llm_control` build thành công; `llm_command` đã được cài vào ROS.

Integration thực tế trên một phiên Gazebo/MoveIt, `continuous:=true demo_plan:=true`:

1. Task đầu thực hiện đủ 7 bước: camera → dọn blue → kiểm tra B → đặt red → home.
2. Một task khác gửi khi robot đang execution bị REJECTED.
3. Gửi lại task ID đã thành công chỉ trả SUCCEEDED, không chạy robot lần nữa.
4. Một câu không hỗ trợ bởi fixture trả FAILED trong planning; server tiếp tục chạy.
5. Task tiếp đọc trạng thái bàn mới, thấy red đã ở B, chỉ detect_objects và home.
   Evidence riêng chứa 2 bước, không lẫn 7 bước của task trước.

Xem [kết quả kiểm tra](validation.json), [log client](client.log),
[task đầu](first_task.json), [task tiếp](second_task.json),
[log Gazebo/MoveIt](execution.log).

Lần integration dùng **offline_fixture**, chưa gọi LLM thật. Gateway ở
`http://localhost:20128/v1/chat/completions` đã kiểm tra và trả `401 Missing API key`;
terminal không có `NINEROUTER_API_KEY`. Cần key và model hợp lệ để kiểm chứng LLM.

Log integration kết thúc bởi timeout 180 giây và có lỗi wait-set khi rclpy tự
shutdown background executor. Sau đó đã sửa thứ tự shutdown để main thread dừng
executor trước context. Kiểm tra riêng sau sửa: CLI kết nối và `exit` không dừng
server, câu rỗng bị REJECTED, SIGINT dừng server với exit code 0 và không Traceback.
Xem [kết quả lifecycle](lifecycle.json) và [log lifecycle](lifecycle.log).

Hướng dẫn sử dụng: [README](../../src/ur3_llm_control/README.md#nhập-lệnh-liên-tục-khi-mô-phỏng-đang-chạy).
