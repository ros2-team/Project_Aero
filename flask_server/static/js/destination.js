let PLACE_DATA = [];
let routeQueue = [];

// 화면 초기화
function init() {
    renderPlaceList(PLACE_DATA);
    renderRoute();
    bindSearchEvent();
}

function getPlaceIcon(locationCode, locationName) {
    if (locationCode.includes("gate")) {
        return "✈️";
    }

    if (locationName.includes("화장실")) {
        return "🚻";
    }

    if (locationName.includes("식당")) {
        return "🍴";
    }

    if (locationName.includes("카페")) {
        return "☕";
    }

    if (locationName.includes("안내")) {
        return "ℹ️";
    }

    if (locationName.includes("수하물")) {
        return "🧳";
    }

    if (locationName.includes("충전")) {
        return "🔋";
    }

    return "📍";
}

function renderPlaceList(list) {
    const placeListEl = document.getElementById("placeList");

    placeListEl.innerHTML = list.map(place => `
        <li class="place-item">
            <div class="place-icon">
                ${getPlaceIcon(place.location_code, place.location_name)}
            </div>

            <div class="place-info">
                <div class="place-name">${place.location_name}</div>
                <div class="place-code">${place.location_code}</div>
            </div>

            <button
                class="place-add-btn"
                type="button"
                onclick="addRoute('${place.location_code}')"
            >
                +
            </button>
        </li>
    `).join("");
}

function bindSearchEvent() {
    const searchInput = document.getElementById("destinationSearch");

    if (!searchInput) {
        return;
    }

    searchInput.addEventListener("input", () => {
        const keyword = searchInput.value.trim().toLowerCase();

        const filteredList = PLACE_DATA.filter(place => {
            return (
                place.location_name.toLowerCase().includes(keyword) ||
                place.location_code.toLowerCase().includes(keyword)
            );
        });

        renderPlaceList(filteredList);
    });
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
        selectedListEl.innerHTML = `
            <li class="empty-selected">
                선택된 목적지가 없습니다.<br>
                왼쪽 목록에서 목적지를 추가해주세요.
            </li>
        `;
        return;
    }

    selectedListEl.innerHTML = routeQueue.map((place, index) => {
        return `
            <li class="selected-item">
                <div class="selected-order">
                    ${index + 1}
                </div>

                <div>
                    <div class="selected-name">
                        ${getPlaceIcon(place.location_code, place.location_name)}
                        ${place.location_name}
                    </div>
                    <div class="selected-status">
                        ${index === 0 ? "첫 번째 안내 목적지" : "대기 중"}
                    </div>
                </div>

                <div class="selected-actions">
                    <button
                        class="small-btn"
                        type="button"
                        onclick="moveUp(${index})"
                    >
                        ↑
                    </button>

                    <button
                        class="small-btn"
                        type="button"
                        onclick="moveDown(${index})"
                    >
                        ↓
                    </button>

                    <button
                        class="small-btn delete"
                        type="button"
                        onclick="deleteRoute(${place.instanceId})"
                    >
                        ×
                    </button>
                </div>
            </li>
        `;
    }).join("");
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
            localStorage.setItem(
                "navigationRoute",
                JSON.stringify(data.route)
            );
            const firstDestination = data.route[0].location_name;
            const firstLocationCode = data.route[0].location_code;
            location.href =
                `/navigation?destination=${encodeURIComponent(firstDestination)}&location_code=${encodeURIComponent(firstLocationCode)}`;    
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

