const socketStatus = document.getElementById("socketStatus");
const socketDot = document.getElementById("socketDot");
const routeList = document.getElementById("routeList");

const pauseButton = document.getElementById("pauseButton");
const pauseModal = document.getElementById("pauseModal");

const changeRouteButton = document.getElementById("changeRouteButton");
const continueButton = document.getElementById("continueButton");
const stopServiceButton = document.getElementById("stopServiceButton");

let isPaused = false;


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


function setPauseButtonText(paused) {
    if (!pauseButton) {
        return;
    }

    if (paused) {
        pauseButton.innerText = "▶ 계속";
    } else {
        pauseButton.innerText = "⏸ 일시정지";
    }
}


async function postNavigationApi(url) {
    const response = await fetch(url, {
        method: "POST",
        headers: {
            "Content-Type": "application/json"
        }
    });

    const data = await response.json();

    if (data.status !== "success") {
        throw new Error(data.message || "navigation api failed");
    }

    return data;
}


async function openPauseModal() {
    try {
        await postNavigationApi("/api/navigation/pause");

        isPaused = true;
        setPauseButtonText(true);

        pauseModal.classList.add("show");
    } catch (error) {
        console.error(error);
        alert("일시정지 요청 중 오류가 발생했습니다.");
    }
}


async function closePauseModal() {
    try {
        await postNavigationApi("/api/navigation/resume");

        isPaused = false;
        setPauseButtonText(false);

        pauseModal.classList.remove("show");
    } catch (error) {
        console.error(error);
        alert("안내 재개 요청 중 오류가 발생했습니다.");
    }
}


pauseButton.addEventListener("click", () => {
    if (isPaused) {
        pauseModal.classList.add("show");
        return;
    }

    openPauseModal();
});


changeRouteButton.addEventListener("click", async () => {
    try {
        await postNavigationApi("/api/navigation/stop");

        location.href = "/destination";
    } catch (error) {
        console.error(error);
        alert("경로 변경 요청 중 오류가 발생했습니다.");
    }
});


continueButton.addEventListener("click", () => {
    closePauseModal();
});


stopServiceButton.addEventListener("click", async () => {
    try {
        localStorage.removeItem("navigationRoute");

        await postNavigationApi("/api/navigation/stop");

        location.href = "/idle";
    } catch (error) {
        console.error(error);
        alert("이용 중지 요청 중 오류가 발생했습니다.");
    }
});


pauseModal.addEventListener("click", (event) => {
    if (event.target === pauseModal) {
        closePauseModal();
    }
});


renderRouteList();
setSocketDummyState();
setPauseButtonText(false);