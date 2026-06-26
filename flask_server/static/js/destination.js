let PLACE_DATA = [];
let routeQueue = [];

// 화면 초기화
function init() {
    const placeListEl = document.getElementById("placeList");
    placeListEl.innerHTML = PLACE_DATA.map(place => `
        <li class="item">
            <div>
                <strong>${place.location_name}</strong> <br>
                <small style="color:#888;">X: ${place.pos_x}, Y: ${place.pos_y}</small>
            </div>
            <button class="btn-add" onclick="addRoute('${place.location_code}')">추가</button>
        </li>
    `).join('');
}

function addRoute(locationCode) {
    const targetPlace = PLACE_DATA.find(p => p.location_code === locationCode);
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
                    <strong>${index + 1}. ${place.location_name}</strong> 
                    <span style="color: ${statusColor}; font-weight: bold; margin-left: 10px;">
                        [${place.status || 'pending'}]
                    </span>
                    <br>
                    <small style="color:#666;">(X: ${place.pos_x}, Y: ${place.pos_y})</small>
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
        location_code: place.location_code
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
            const firstDestination = data.route[0].location_name;
            location.href = '/navigation?destination=${encodeURIComponent(firstDestination)}';    
        } 
        else {
            alert("경로 전송 실패: " + data.message);
        }
    })
    .catch(error => {
        console.error("Flask 통신 에러:", error);
        alert("Flask 통신 에러 발생");
    });
}
window.onload = async() => {
    try{
        const response = await fetch("/api/locations");
        PLACE_DATA = await response.json();
        console.log("목적지 목록", PLACE_DATA);
        init();
    }
    catch(error){
        console.error(error);
        alert("목적지 목록 조회 실패");
    }
};

