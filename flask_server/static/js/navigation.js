const socket = io();

const statusValue = document.getElementById("statusValue");
const socketStatus = document.getElementById("socketStatus");
const guideMessage = document.getElementById("guideMessage");
const currentTarget = document.getElementById("currentTarget");

const cancelButton = document.getElementById("cancelButton");
const finishButton = document.getElementById("finishButton");
const homeButton = document.getElementById("homeButton");


socket.on("connect", () => {
    console.log("WebSocket connected");

    if (socketStatus) {
        socketStatus.innerText = "연결됨";
    }
});


socket.on("disconnect", () => {
    console.log("WebSocket disconnected");

    if (socketStatus) {
        socketStatus.innerText = "연결 끊김";
    }
});


socket.on("navigation_status", (data) => {
    console.log("navigation_status:", data);

    if (statusValue && data.status) {
        statusValue.innerText = convertStatusText(data.status);
    }

    if (guideMessage && data.message) {
        guideMessage.innerText = data.message;
    }

    if (currentTarget && data.location_name) {
        currentTarget.innerText = data.location_name;
    }

    if (data.status === "finished") {
        setTimeout(() => {
            location.href = "/finish";
        }, 1500);
    }
});


function convertStatusText(status) {
    if (status === "connected") {
        return "연결 완료";
    }

    if (status === "started") {
        return "안내 시작";
    }

    if (status === "moving") {
        return "이동 중";
    }

    if (status === "arrived") {
        return "목적지 도착";
    }

    if (status === "finished") {
        return "안내 완료";
    }

    if (status === "error") {
        return "오류 발생";
    }

    return status;
}


cancelButton.addEventListener("click", () => {
    const result = confirm("길 안내를 취소하시겠습니까?");

    if (!result) {
        return;
    }

    // 아직 로봇 정지 API는 안 만들었으므로 화면만 이동
    location.href = "/idle";
});


finishButton.addEventListener("click", () => {
    location.href = "/finish";
});


homeButton.addEventListener("click", () => {
    location.href = "/idle";
});