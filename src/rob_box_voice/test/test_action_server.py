import asyncio
import pytest

from rob_box_voice.action_server import ActionServer, ActionState, PastePlanner


@pytest.mark.asyncio
async def test_action_server_feedback_result_and_health():
    async def run(goal, feedback, cancelled):
        feedback({"progress": 0.5})
        await asyncio.sleep(0)
        return {"text": goal["text"]}

    server = ActionServer(run)
    handle = server.submit({"text": "hello", "goal_id": "g1"})
    assert await handle.wait() == {"text": "hello"}
    assert handle.state is ActionState.SUCCEEDED
    assert handle.feedback == [{"progress": 0.5}]
    assert server.health().ok
    await server.shutdown()


@pytest.mark.asyncio
async def test_cancel_propagates_to_handler():
    started = asyncio.Event()
    async def run(_goal, _feedback, cancelled):
        started.set()
        await cancelled.wait()

    server = ActionServer(run)
    handle = server.submit({"text": "slow", "goal_id": "cancel-me"})
    await started.wait()
    assert server.cancel(handle.goal_id)
    with pytest.raises(asyncio.CancelledError):
        await handle.wait()
    assert handle.state is ActionState.CANCELLED
    await server.shutdown()


@pytest.mark.asyncio
async def test_paste_speculation_commit_and_discard():
    seen = []
    async def execute(goal):
        seen.append(goal["text"])
        return goal["text"].upper()

    paste = PastePlanner(execute)
    paste.speculate("next", {"text": "hello"})
    assert await paste.commit("next") == "HELLO"
    paste.speculate("stale", {"text": "stale"})
    assert paste.discard("stale")
    await asyncio.sleep(0)
    assert seen == ["hello"]
