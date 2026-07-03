const socketStatus = document.getElementById("socketStatus");
const socketDot = document.getElementById("socketDot");
const routeList = document.getElementById("routeList");

const navBatteryValue = document.getElementById("navBatteryValue");
const navRobotStatus = document.getElementById("navRobotStatus");
const navRobotPosition = document.getElementById("navRobotPosition");

const pauseButton = document.getElementById("pauseButton");
const pauseModal = document.getElementById("pauseModal");

const changeRouteButton = document.getElementById("changeRouteButton");
const continueButton = document.getElementById("continueButton");
const stopServiceButton = document.getElementById("stopServiceButton");

const mapImage = document.getElementById("mapImage");
const mapCanvas = document.getElementById("mapCanvas");

const MAP_INFO = {
    resolution: 0.05,
    originX: -3.44,
    originY: -3.2,
    width: 128,
    height: 126
};

// 임시 현재 위치
let currentRobotPosition = {
    x : -1.5,
    y : -0.95,
    yaw : 0.0
}

let isPaused = false;
let isFinishRedirecting = false;
let navigationPath = [];
let currentNavigationIndex = 0;
// --------------------------------------------------------------------------------------------------------지도
function rosToMapPixel(rosX, rosY) {
    const pixelX = (rosX - MAP_INFO.originX) / MAP_INFO.resolution;
    const pixelY = MAP_INFO.height - ((rosY - MAP_INFO.originY) / MAP_INFO.resolution);
    return {
        x : pixelX,
        y : pixelY
    };
}

function rosToCanvasPoint(rosX, rosY) {
    const mapPixel = rosToMapPixel(rosX, rosY);

    return mapPixelToCanvasPoint(
        mapPixel.x,
        mapPixel.y
    );
}

function mapPixelToCanvasPoint(pixelX, pixelY) {
    const canvasRect = mapCanvas.getBoundingClientRect();

    const canvasWidth = canvasRect.width;
    const canvasHeight = canvasRect.height;

    const mapRatio = MAP_INFO.width / MAP_INFO.height;
    const canvasRatio = canvasWidth / canvasHeight;

    let drawWidth;
    let drawHeight;
    let offsetX;
    let offsetY;

    if (canvasRatio > mapRatio) {
        drawHeight = canvasHeight;
        drawWidth = drawHeight * mapRatio;
        offsetX = (canvasWidth - drawWidth) / 2;
        offsetY = 0;
    } else {
        drawWidth = canvasWidth;
        drawHeight = drawWidth / mapRatio;
        offsetX = 0;
        offsetY = (canvasHeight - drawHeight) / 2;
    }

    return {
        x: offsetX + (pixelX / MAP_INFO.width) * drawWidth,
        y: offsetY + (pixelY / MAP_INFO.height) * drawHeight
    };
}

function drawNavigationMap() {
    if (!mapCanvas || !mapImage) {
        return;
    }

    const savedRoute = localStorage.getItem("navigationRoute");

    if (!savedRoute) {
        return;
    }

    const route = JSON.parse(savedRoute);

    const canvasRect = mapCanvas.getBoundingClientRect();

    mapCanvas.width = canvasRect.width;
    mapCanvas.height = canvasRect.height;

    const ctx = mapCanvas.getContext("2d");

    ctx.clearRect(
        0,
        0,
        mapCanvas.width,
        mapCanvas.height
    );

    const points = [];

    const currentPoint = rosToCanvasPoint(
        currentRobotPosition.x,
        currentRobotPosition.y
    );

    points.push({
        ...currentPoint,
        type: "current",
        name: "현재 위치"
    });

    route.forEach((target) => {
        const point = rosToCanvasPoint(
            target.x,
            target.y
        );

        points.push({
            ...point,
            type: "target",
            name: target.location_name
        });
    });
    
    const plannedPoints = [
        {
            ...currentPoint,
            type: "current",
            name: "현재 위치"
        }
    ];
    
    route.forEach((target, index) => {
        if(index < currentNavigationIndex){
            return;
        }
        const point = rosToCanvasPoint(
            target.x, 
            target.y
        );
        plannedPoints.push({
            ...point,
            type: "target",
            name: target.location_name
        });
    });
    

    drawPlannedRouteLine(ctx, plannedPoints);

    if (navigationPath.length > 1) {
        drawNavigationPath(ctx, navigationPath);
    }

    drawMarkers(ctx, points);   
}


function drawRouteLine(ctx, points) {
    if (points.length < 2) {
        return;
    }

    ctx.beginPath();

    points.forEach((point, index) => {
        if (index === 0) {
            ctx.moveTo(point.x, point.y);
        } else {
            ctx.lineTo(point.x, point.y);
        }
    });

    ctx.lineWidth = 5;
    ctx.strokeStyle = "#0875ff";
    ctx.lineCap = "round";
    ctx.lineJoin = "round";
    ctx.setLineDash([10, 8]);
    ctx.stroke();

    ctx.setLineDash([]);
}

function drawPlannedRouteLine(ctx, points) {
    if (!Array.isArray(points) || points.length < 2) {
        return;
    }

    ctx.beginPath();

    points.forEach((point, index) => {
        if (index === 0) {
            ctx.moveTo(
                point.x,
                point.y
            );
        } else {
            ctx.lineTo(
                point.x,
                point.y
            );
        }
    });

    ctx.lineWidth = 3;
    ctx.strokeStyle = "rgba(80, 90, 110, 0.35)";
    ctx.lineCap = "round";
    ctx.lineJoin = "round";
    ctx.setLineDash([6, 8]);
    ctx.stroke();

    ctx.setLineDash([]);
}

function drawMarkers(ctx, points) {
    points.forEach((point, index) => {
        if (point.type === "current") {
            drawCurrentMarker(ctx, point);
        } else {
            drawTargetMarker(ctx, point, index);
        }
    });
}

function drawCurrentMarker(ctx, point) {
    ctx.beginPath();
    ctx.arc(point.x, point.y, 11, 0, Math.PI * 2);
    ctx.fillStyle = "#0875ff";
    ctx.fill();

    ctx.lineWidth = 4;
    ctx.strokeStyle = "white";
    ctx.stroke();

    ctx.font = "bold 14px Arial";
    ctx.fillStyle = "#061b4e";
    ctx.textAlign = "center";
    ctx.fillText("현재", point.x, point.y - 18);
}

function drawTargetMarker(ctx, point, index) {
    ctx.beginPath();
    ctx.arc(point.x, point.y, 9, 0, Math.PI * 2);
    ctx.fillStyle = "#ff4b4b";
    ctx.fill();

    ctx.lineWidth = 4;
    ctx.strokeStyle = "white";
    ctx.stroke();
}

function drawNavigationPath(ctx, path) {
    if (!Array.isArray(path) || path.length < 2) {
        return;
    }

    const closest = findClosestPathIndex(
        path,
        currentRobotPosition
    );

    /*
        로봇이 path에서 너무 멀리 떨어져 있으면
        잘못된 index로 경로가 확 잘리는 걸 막기 위한 안전장치.
        단위는 ROS 좌표 기준 meter.
    */
    const MAX_PATH_DISTANCE = 0.7;

    let remainingPath;

    if (closest.distance > MAX_PATH_DISTANCE) {
        remainingPath = path;
    } else {
        remainingPath = [
            {
                x: currentRobotPosition.x,
                y: currentRobotPosition.y
            },
            ...path.slice(closest.index + 1)
        ];
    }

    if (remainingPath.length < 2) {
        return;
    }

    ctx.beginPath();

    remainingPath.forEach((pose, index) => {
        const point = rosToCanvasPoint(
            pose.x,
            pose.y
        );

        if (index === 0) {
            ctx.moveTo(
                point.x,
                point.y
            );
        } else {
            ctx.lineTo(
                point.x,
                point.y
            );
        }
    });

    ctx.lineWidth = 6;
    ctx.strokeStyle = "#0875ff";
    ctx.lineCap = "round";
    ctx.lineJoin = "round";
    ctx.setLineDash([]);
    ctx.stroke();
}

function findClosestPathIndex(path, robotPosition) {
    if (!Array.isArray(path) || path.length === 0) {
        return {
            index: 0,
            distance: Infinity
        };
    }

    let closestIndex = 0;
    let closestDistance = Infinity;

    path.forEach((pose, index) => {
        const dx = pose.x - robotPosition.x;
        const dy = pose.y - robotPosition.y;
        const distance = Math.sqrt(
            dx * dx + dy * dy
        );

        if (distance < closestDistance) {
            closestDistance = distance;
            closestIndex = index;
        }
    });

    return {
        index: closestIndex,
        distance: closestDistance
    };
}

// --------------------------------------------------------------------------------------------------------

function updateRobotStatus(data) {
    if (typeof data.x === "number") {
        currentRobotPosition.x = data.x;
    }

    if (typeof data.y === "number") {
        currentRobotPosition.y = data.y;
    }

    if (typeof data.yaw === "number") {
        currentRobotPosition.yaw = data.yaw;
    }

    updateRobotStatusPanel(data);

    drawNavigationMap();
}

function updateRobotStatusPanel(data) {
    if (navBatteryValue) {
        if (typeof data.battery === "number") {
            navBatteryValue.innerText = `${data.battery}%`;
        } else {
            navBatteryValue.innerText = "--%";
        }
    }

    if (navRobotStatus) {
        navRobotStatus.innerText = convertRobotStatusText(
            data.robot_status
        );
    }

    if (navRobotPosition) {
        const x = typeof currentRobotPosition.x === "number"
            ? currentRobotPosition.x.toFixed(2)
            : "--";

        const y = typeof currentRobotPosition.y === "number"
            ? currentRobotPosition.y.toFixed(2)
            : "--";

        navRobotPosition.innerText = `${x}, ${y}`;
    }
}

function convertRobotStatusText(status) {
    if (status === "idle") {
        return "대기 중";
    }

    if (status === "moving") {
        return "이동 중";
    }

    if (status === "paused") {
        return "일시정지";
    }

    if (status === "stopped") {
        return "정지";
    }

    if (status === "charging") {
        return "충전 중";
    }

    if (status === "error") {
        return "오류";
    }

    if (status === "finished") {
        return "안내 완료";
    }

    return "알 수 없음";
}

async function loadInitialRobotStatus() {
    try {
        const response = await fetch("/api/robot/status");

        if (!response.ok) {
            throw new Error(`robot status request failed: ${response.status}`);
        }

        const data = await response.json();

        if (data.status === "success") {
            updateRobotStatus(data.robot_status);
        }
    } catch (error) {
        console.error(error);
    }
}

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
        currentNavigationIndex = data.current_index;
        
        renderRouteList(
            data.current_index,
            data.status
        );
        
        drawNavigationMap();
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

    socket.on("robot_status", (data) => {
        console.log("robot_status", data);
        updateRobotStatus(data);
    })

    socket.on("navigation_path", (data) => {
        console.log("navigation_path", data);
        if(Array.isArray(data.path)){
            navigationPath = data.path;
            drawNavigationMap();
        }
    })

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
loadInitialRobotStatus();
connectSocket();
setPauseButtonText(false);

if (mapImage) {
    if (mapImage.complete) {
        drawNavigationMap();
    } else {
        mapImage.addEventListener("load", () => {
            drawNavigationMap();
        });
    }
}

window.addEventListener("resize", () => {
    drawNavigationMap();
});