from concurrent.futures import ThreadPoolExecutor
from ur3_llm_control.command_session import CommandSession


def test_concurrent_submit_only_accepts_one_task():
    session = CommandSession()
    with ThreadPoolExecutor(max_workers=8) as pool:
        results = list(pool.map(lambda i: session.submit("Put red in B", f"task_{i}"), range(16)))
    assert sum(task is not None for task, _ in results) == 1
    assert sum(status["state"] == "REJECTED" for _, status in results) == 15
    assert session.availability()["state"] == "BUSY"


def test_duplicate_task_is_never_executed_twice():
    session = CommandSession()
    task, _ = session.submit("Put red in B", "first")
    assert task is not None
    assert session.submit(task.command, task.task_id)[0] is None
    session.update(task.task_id, "PLANNING")
    session.finish(task.task_id, "SUCCEEDED")
    task, status = session.submit("Put red in B", "first")
    assert task is None and status["state"] == "SUCCEEDED"
    assert session.availability()["state"] == "READY"
    assert session.submit("Put blue in C", "first")[1]["state"] == "REJECTED"
    assert session.submit("Put blue in C", "second")[0] is not None


def test_planning_failure_recovers_but_execution_fault_blocks_next_task():
    session = CommandSession()
    session.submit("nonsense", "first")
    session.finish("first", "FAILED", error="Invalid LLM plan")
    assert session.submit("Put red in B", "second")[0] is not None
    session.finish("second", "FAILED", fault="Gripper release timed out")
    assert session.availability()["state"] == "FAULT"
    task, status = session.submit("Put blue in C", "third")
    assert task is None and "Execution fault" in status["error"]


def test_command_validation_and_bounded_history_preserves_active_task():
    session = CommandSession(history_limit=3)
    for command, task_id in [("", "x"), ([], "x"), ("x"*1001, "x"), ("home", "../bad"), ("home", [])]:
        assert session.submit(command, task_id)[1]["state"] == "REJECTED"
    task, _ = session.submit("home", "active")
    for i in range(12):
        session.submit("home", str(i))
    assert len(session.history) == 3
    assert task.task_id in session.history
    assert session.submit("home", "active")[0] is None
    session.finish("active", "PLANNED")
    assert session.availability()["state"] == "READY"
