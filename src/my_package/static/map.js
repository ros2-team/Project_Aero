const canvas = document.getElementById("mapCanvas");
const ctx = canvas.getContext("2d");

let robotX = 0;
let robotY = 0;

let pathData = [];

let goalX = null;
let goalY = null;

// =====================
// 좌표 변환
// =====================

function worldToCanvas(x, y) {
    const MARGIN = 50;
    const SCALE_X = (1500 - MARGIN * 2) / (2.9 - (-0.07));
    const SCALE_Y = (900  - MARGIN * 2) / (0.6 - (-1.5));

    return {
        x: MARGIN + (x - (-0.07)) * SCALE_X,
        y: (900 - MARGIN) - (y - (-1.5)) * SCALE_Y
    };
}


// =====================
// Flask 데이터 수신
// =====================

async function updatePose() {

    const response = await fetch('/robot_pose');
    const data = await response.json();

    robotX = data.x;
    robotY = data.y;
}

async function updatePlan() {

    const response = await fetch('/plan');
    pathData = await response.json();
}


// =====================
// 약도 그리기
// =====================

function drawMap() {
    ctx.strokeStyle = "black";
    ctx.lineWidth = 4;

    const p1 = worldToCanvas(-0.07, -1.5);
    const p2 = worldToCanvas(2.9, 0.6);

    ctx.strokeRect(
        p1.x,
        p2.y,
        p2.x - p1.x,
        p1.y - p2.y
    );

    //일자 벽은 실제 위치 모르니 일단 주석처리
    const wallStart = worldToCanvas(-0.07, -0.7);
    const wallEnd   = worldToCanvas(0.92, -0.7);

    ctx.beginPath();
    ctx.moveTo(wallStart.x, wallStart.y);
    ctx.lineTo(wallEnd.x, wallEnd.y);
    ctx.stroke();
}


// =====================
// 계획 경로
// =====================
function drawPlan(){

    if(pathData.length === 0)
        return;

    ctx.strokeStyle = "#2563eb";
    ctx.lineWidth = 4;
    ctx.setLineDash([15,10]);

    ctx.beginPath();

    let start = worldToCanvas(pathData[0].x, pathData[0].y);
    ctx.moveTo(start.x, start.y)

    for(let i = 1; i < pathData.length; i++){
        let p = worldToCanvas(pathData[i].x, pathData[i].y);
        ctx.lineTo(p.x, p.y);
    }

    ctx.stroke();

    ctx.setLineDash([]);
}
// =====================
// 목적지 마커
// =====================

function drawGoal(){

    if(goalX === null) return;

    const p = worldToCanvas(goalX, goalY);

    // 빨간 핀

    ctx.fillStyle = "red";

    ctx.beginPath();

    ctx.arc(
        p.x,
        p.y-40,
        18,
        0,
        Math.PI*2
    );

    ctx.fill();

    ctx.fillStyle = "white";

    ctx.beginPath();

    ctx.arc(
        p.x,
        p.y-40,
        7,
        0,
        Math.PI*2
    );

    ctx.fill();

    ctx.fillStyle = "red";

    ctx.beginPath();

    ctx.moveTo(p.x-10,p.y-25);
    ctx.lineTo(p.x+10,p.y-25);
    ctx.lineTo(p.x,p.y-5);
    ctx.fill();
}


async function updateGoal() {
    const response = await fetch('/get_goal');
    const data = await response.json();
    goalX = data.x;
    goalY = data.y;
}


// =====================
// 로봇
// =====================

function drawRobot(){

    const p = worldToCanvas(
        robotX,
        robotY
    );

    ctx.fillStyle = "#2563eb";

    ctx.beginPath();

    ctx.arc(
        p.x,
        p.y,
        14,
        0,
        Math.PI*2
    );

    ctx.fill();
}

// =====================
// 일시정지 버튼
// =====================

function createPauseButton() {
    const btn = document.createElement("button");
    btn.id = "pauseBtn";
    btn.textContent = "⏸ 일시정지";

    btn.style.cssText = `
        position: fixed;
        bottom: 20px;
        left: 50%;
        transform: translateX(-50%);
        padding: 12px 24px;
        font-size: 16px;
        font-weight: bold;
        background-color: #2563eb;
        color: white;
        border: none;
        border-radius: 8px;
        cursor: pointer;
        z-index: 1000;
        box-shadow: 0 2px 8px rgba(0,0,0,0.2);
    `;

    btn.addEventListener("mouseenter", () => {
        btn.style.backgroundColor = "#1d4ed8";
    });
    btn.addEventListener("mouseleave", () => {
        btn.style.backgroundColor = "#2563eb";
    });

    btn.addEventListener("click", () => {
        goToPreviousPage();
    });

    document.body.appendChild(btn);
}

function goToPreviousPage() {

    //window.location.href = "/previous-page";
    console.log("이전 페이지로 이동");
}



// =====================
// 전체 그리기
// =====================

function draw(){

    createPauseButton();

    ctx.clearRect(
        0,
        0,
        canvas.width,
        canvas.height
    );

    drawMap();

    drawPlan();

    drawGoal();

    drawRobot();
}


// =====================
// 주기 갱신
// =====================

async function update() {

    await updatePose();

    await updatePlan();

    await updateGoal();

    draw();
}

setInterval(update, 200);

update();

