const callRobotButton = document.getElementById("callRobotButton");
const callResult = document.getElementById("callResult");

callRobotButton.addEventListener("click", () => {
    const location = callRobotButton.dataset.location;

    callRobotButton.disabled = true;
    callRobotButton.innerText = "호출 중...";
    callResult.className = "call-result";
    callResult.innerText = "로봇 호출 요청을 보내는 중입니다.";

    fetch("/call_robot", {
        method: "POST",
        headers: {
            "Content-Type": "application/json"
        },
        body: JSON.stringify({
            location: location
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success === true) {
            callResult.className = "call-result success";
            callResult.innerText = "로봇 호출이 완료되었습니다. 잠시만 기다려주세요.";
            callRobotButton.innerText = "호출 완료";
        } else {
            callResult.className = "call-result error";
            callResult.innerText = "로봇 호출에 실패했습니다. 다시 시도해주세요.";
            callRobotButton.disabled = false;
            callRobotButton.innerText = "로봇 호출하기";
        }
    })
    .catch(error => {
        console.error(error);

        callResult.className = "call-result error";
        callResult.innerText = "서버 통신 중 오류가 발생했습니다.";
        callRobotButton.disabled = false;
        callRobotButton.innerText = "로봇 호출하기";
    });
});