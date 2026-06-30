const callRobotButton = document.getElementById("callRobotButton");
const callResult = document.getElementById("callResult");

callRobotButton.addEventListener("click", () => {
    const location = callRobotButton.dataset.location;

    if (!location) {
        callResult.className = "error";
        callResult.innerHTML =
            "현재 위치 정보가 없습니다.<br>QR 코드를 다시 확인해주세요.";
        return;
    }

    callRobotButton.disabled = true;
    callRobotButton.innerHTML = "호출 중...";

    callResult.className = "";
    callResult.innerHTML =
        "로봇 호출 요청을 보내는 중입니다.<br>잠시만 기다려 주세요.";

    fetch("/api/qrcall/callrobot", {
        method: "POST",
        headers: {
            "Content-Type": "application/json"
        },
        body: JSON.stringify({
            location: location
        })
    })
    .then(response => {
        if (!response.ok) {
            throw new Error(`서버 응답 오류: ${response.status}`);
        }

        return response.json();
    })
    .then(data => {
        if (data.success === true) {
            callResult.className = "success";
            callResult.innerHTML =
                "로봇이 호출 위치로 이동 중입니다.<br>잠시만 기다려 주세요.";

            callRobotButton.innerHTML = "호출 완료";
        } else {
            callResult.className = "error";
            callResult.innerHTML =
                data.message || "로봇 호출에 실패했습니다.<br>다시 시도해주세요.";

            callRobotButton.disabled = false;
            callRobotButton.innerHTML = "로봇 호출하기";
        }
    })
    .catch(error => {
        console.error(error);

        callResult.className = "error";
        callResult.innerHTML =
            "서버 통신 중 오류가 발생했습니다.<br>다시 시도해주세요.";

        callRobotButton.disabled = false;
        callRobotButton.innerHTML = "로봇 호출하기";
    });
});