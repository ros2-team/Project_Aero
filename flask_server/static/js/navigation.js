const socketStatus = document.getElementById("socketStatus");
const socketDot = document.getElementById("socketDot");
const routeList = document.getElementById("routeList");

const pauseButton = document.getElementById("pauseButton");
const pauseModal = document.getElementById("pauseModal");

const changeRouteButton = document.getElementById("changeRouteButton");
const continueButton = document.getElementById("continueButton");
const stopServiceButton = document.getElementById("stopServiceButton");

let isPaused = false;
let isFinishRedirecting = false;

function updateNavigationStatus(data){
    if(!socketStatus){
        return;
    }
    if(data.status === "connected"){
        socketStatus.innerText = "서버 연결됨";
        return;
    }
    
    if(data.status === "moving"){
        socketStatus.innerText = "이동 중";
    }
    else if(data.status === "paused"){
        socketStatus.innerText = "일시정지 중";
    }
    else if(data.status === "stopped"){
        socketStatus.innerText = "안내 중지됨";
    }
    else if(data.status === "finished"){
        socketStatus.innerText = "안내 완료";

        if(!isFinishRedirecting){
            isFinishRedirecting = true;
            moveToFinishPage();
        }
    }
    else {
        socketStatus.innerText = data.status;
    }

    if(typeof data.current_index === "number"){
        renderRouteList(
            data.current_index,
            data.status
        );
    }
}

function connectSocket() {
    const socket = io();
    
    socket.on("connect", () => {
        if(socketStatus){
            socketStatus.innerText = "서버 연결됨";
        }
        if(socketDot){
            socketDot.classList.add("connected");
        }
        console.log("Socket connected");
    });
    
    socket.on("disconnect", () => {
        if(socketStatus) {
            socketStatus.innerText = "연결 끊김";
        }
        if(socketDot) {
            socketDot.classList.remove("connected");
        }
        console.log("Socket disconnected");
    });

    socket.on("navigation_status", (data) => {
        console.log("navigation_status", data);
        updateNavigationStatus(data);
    });

}

function renderRouteList(currentIndex = 0, navigationStatus = "moving") {
    const savedRoute = localStorage.getItem("navigationRoute");

    if (!savedRoute) {
        return;
    }

    const route = JSON.parse(savedRoute);

    routeList.innerHTML = route.map((item, index) => {
        let className = "route-item waiting";
        let statusText = "대기 중";

        if (index < currentIndex) {
            className = "route-item completed";
            statusText = "도착 완료";
        } else if (index === currentIndex) {
            if (navigationStatus === "paused") {
                className = "route-item paused";
                statusText = "일시정지 중";
            } else if (navigationStatus === "finished") {
                className = "route-item completed";
                statusText = "도착 완료";
            } else {
                className = "route-item active";
                statusText = "이동 중";
            }
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

function moveToFinishPage() {
    setTimeout(() => {
        location.href = "/finish";
    }, 1200);
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
connectSocket();
setPauseButtonText(false);