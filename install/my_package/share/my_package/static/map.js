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
    const wallStart = worldToCanvas(-1.5, 1.8);
    const wallEnd   = worldToCanvas(0.0, 1.8);
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

    // 시작점 = 현재 로봇 위치
    let robot = worldToCanvas(
        robotX,
        robotY
    );

    ctx.moveTo(
        robot.x,
        robot.y
    );

    // 나머지 경로
    for(let i=0;i<pathData.length;i++){

        let p = worldToCanvas(
            pathData[i].x,
            pathData[i].y
        );

        ctx.lineTo(
            p.x,
            p.y
        );
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
// 전체 그리기
// =====================

function draw(){

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

