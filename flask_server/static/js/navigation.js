const socketStatus = document.getElementById("socketStatus");
const socketDot = document.getElementById("socketDot");
const routeList = document.getElementById("routeList");

const pauseButton = document.getElementById("pauseButton");
const pauseModal = document.getElementById("pauseModal");

const changeRouteButton = document.getElementById("changeRouteButton");
const continueButton = document.getElementById("continueButton");
const stopServiceButton = document.getElementById("stopServiceButton");


function renderRouteList() {
    const savedRoute = localStorage.getItem("navigationRoute");

    if (!savedRoute) {
        return;
    }

    const route = JSON.parse(savedRoute);

    routeList.innerHTML = route.map((item, index) => {
        let className = "route-item waiting";
        let statusText = "대기 중";

        if (index === 0) {
            className = "route-item active";
            statusText = "이동 중";
        }

        return `
            <div class="${className}">
                <div class="route-index">${index + 1}</div>
                <div class="route-info">
                    <div class="route-name">${item.location_name}</div>
                    <div class="route-status">${statusText}</div>
                </div>
            </div>
        `;
    }).join("");
}


function setSocketDummyState() {
    if (socketStatus) {
        socketStatus.innerText = "디자인 확인용";
    }

    if (socketDot) {
        socketDot.classList.add("connected");
    }
}


function openPauseModal() {
    pauseModal.classList.add("show");

    // 나중에 여기서 ROS2 일시정지 API 호출 예정
    // fetch("/api/navigation/pause", { method: "POST" });
}


function closePauseModal() {
    pauseModal.classList.remove("show");

    // 나중에 여기서 ROS2 재개 API 호출 예정
    // fetch("/api/navigation/resume", { method: "POST" });
}


pauseButton.addEventListener("click", () => {
    openPauseModal();
});


changeRouteButton.addEventListener("click", () => {
    // 나중에 여기서 기존 goal 취소 API 호출 예정
    // fetch("/api/navigation/cancel", { method: "POST" });

    location.href = "/destination";
});


continueButton.addEventListener("click", () => {
    closePauseModal();
});


stopServiceButton.addEventListener("click", () => {
    localStorage.removeItem("navigationRoute");

    // 나중에 여기서 기존 goal 취소 + 로밍 복귀 API 호출 예정
    // fetch("/api/navigation/stop", { method: "POST" });

    location.href = "/idle";
});


pauseModal.addEventListener("click", (event) => {
    if (event.target === pauseModal) {
        closePauseModal();
    }
});


renderRouteList();
setSocketDummyState();