const client = mqtt.connect('ws://localhost:9001');

client.on('connect', () => {
    console.log("✔ MQTT 브로커 연결 성공 (Websocket Port: 9001)");
    client.subscribe('robot/navigation/status', (err) => {
        if (!err) {
            console.log("로봇 상태 토픽 구독 시작 ('robot/navigation/status')");
        }
    });
});

client.on('error', (err) => {
    console.error("MQTT 연결 실패:", err);
});


const PLACE_DATA = [
    { id: 1, name: "폭주한 로봇 위치", x: 27.5, y: 4.1 },
    //{ id: 1, name: "화장실", x: 0.0, y: 0.0 },
    { id: 2, name: "2번 출국장", x: 5.2, y: -3.4 },
    { id: 3, name: "3번출국장 ", x: -2.1, y: 8.5 },
    { id: 4, name: "면세수령 장소 ", x: 12.4, y: 4.1 },
    { id: 5, name: "버거킹", x: -7.8, y: -2.3 }
];

let routeQueue = [];

// 화면 초기화
function init() {
    const placeListEl = document.getElementById("placeList");
    placeListEl.innerHTML = PLACE_DATA.map(place => `
        <li class="item">
            <div>
                <strong>${place.name}</strong> <br>
                <small style="color:#888;">X: ${place.x}, Y: ${place.y}</small>
            </div>
            <button class="btn-add" onclick="addRoute(${place.id})">추가</button>
        </li>
    `).join('');
}

function addRoute(id) {
    const targetPlace = PLACE_DATA.find(p => p.id === id);
    if (!targetPlace) return;

    routeQueue.push({
        ...targetPlace,
        instanceId: Date.now() + Math.random(),
        status: 'pending'
    });
    renderRoute();
}

function deleteRoute(instanceId) {
    routeQueue = routeQueue.filter(item => item.instanceId !== instanceId);
    renderRoute();
}

function moveUp(index) {
    if (index === 0) return; 
    const temp = routeQueue[index];
    routeQueue[index] = routeQueue[index - 1];
    routeQueue[index - 1] = temp;
    renderRoute();
}

function moveDown(index) {
    if (index === routeQueue.length - 1) return; 
    const temp = routeQueue[index];
    routeQueue[index] = routeQueue[index + 1];
    routeQueue[index + 1] = temp;
    renderRoute();
}

function renderRoute() {
    const selectedListEl = document.getElementById("selectedList");
    if (routeQueue.length === 0) {
        selectedListEl.innerHTML = `<li style="color:#aaa; text-align:center; margin-top:20px;">선택된 장소가 없습니다.</li>`;
        return;
    }

    selectedListEl.innerHTML = routeQueue.map((place, index) => {
        let statusColor = "#666"; 
        if (place.status === "finish") statusColor = "#4caf50"; 
        const isFinished = place.status === "finish";

        return `
            <li class="item selected-item" style="${isFinished ? 'background:#e8f5e9; border-color:#a5d6a7; opacity: 0.8;' : ''}">
                <div>
                    <strong>${index + 1}. ${place.name}</strong> 
                    <span style="color: ${statusColor}; font-weight: bold; margin-left: 10px;">
                        [${place.status || 'pending'}]
                    </span>
                    <br>
                    <small style="color:#666;">(X: ${place.x}, Y: ${place.y})</small>
                </div>
                <div class="btn-group">
                    <button onclick="moveUp(${index})" ${isFinished ? 'disabled' : ''}>▲</button>
                    <button onclick="moveDown(${index})" ${isFinished ? 'disabled' : ''}>▼</button>
                    <button class="btn-delete" onclick="deleteRoute(${place.instanceId})" ${isFinished ? 'disabled' : ''}>삭제</button>
                </div>
            </li>
        `;
    }).join('');
}

// 완료된 장소는 제외하고 아직 방문하지 않은 경유지만 필터링하여 전송하는 함수
function startNavigation() {
    // 1. 전체 큐에서 이미 완료된('finish') 장소는 걷어내고 'pending' 상태인 것만 추출
    const activeRoute = routeQueue.filter(place => place.status !== 'finish');

    if (activeRoute.length === 0) {
        alert("최소 한 개 이상의 미방문 경유지를 선택해 주세요.");
        return;
    }

    // 2. 필터링된 미방문 장소들로만 전송용 Payload 데이터 빌드
    const finalPayload = activeRoute.map((place, index) => ({
        order: index + 1, // 남은 장소들 기준으로 순서 재정렬
        instanceId: place.instanceId, 
        name: place.name,
        coordinate: { x: place.x, y: place.y }
    }));

    console.log("🚀 로봇으로 전송되는 실제 미방문 경로 데이터:", finalPayload);

    // Flask 엔드포인트로 전송
    fetch('/api/navigation/start', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify(finalPayload)
    })
    .then(response => response.json())
    .then(data => {
        console.log("서버 응답:", data);
        if (data.status === "success") {
            alert("로봇에게 새로운 미방문 경로 전송을 완료했습니다.");
        } else {
            alert("경로 전송 실패: " + data.message);
        }
    })
    .catch(error => {
        console.error("Flask 통신 에러:", error);
    });
}


// 💡 [필수 추가] ROS2 노드(test_sub.py)로부터 완료 메시지를 수신하는 리스너
client.on('message', (topic, message) => {
    if (topic === 'robot/navigation/status') {
        try {
            const robotStatusList = JSON.parse(message.toString());
            
            // 1. 브라우저 콘솔창(F12)에 수신 데이터 출력
            console.log("================ [MQTT 수신 데이터] ================");
            console.log(robotStatusList);
            console.log("===================================================");
            
            // 2. 화면 상태 업데이트 함수 호출
            updateRouteStatus(robotStatusList);
        } catch (err) {
            console.error("데이터 파싱 에러:", err);
        }
    }
});

// [필수 추가] 수신한 상태를 routeQueue에 반영하고 화면을 새로 그리는 함수
function updateRouteStatus(statusList) {
    statusList.forEach(statusItem => {
        const target = routeQueue.find(item => item.instanceId === statusItem.instanceId);
        if (target) {
            target.status = statusItem.status; // 'pending' -> 'finish' 변경
            console.log(`📌 상태 변경 알림: [${target.name}] 장소의 상태가 [${statusItem.status}]로 업데이트되었습니다.`);
        }
    });
    renderRoute(); // 화면 리렌더링 (버튼 disabled 처리됨)
}

window.onload = init;