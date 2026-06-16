console.log("자바스크립트 파일이 성공적으로 로드되었습니다.");

// 버튼을 누르면 실행될 함수
function sendCoordinate(x, y) {
    // 웹에서 선택한 좌표를 묶음
    const coordinateData = { 
        x: x, 
        y: y,
        timestamp: new Date().toLocaleTimeString() // 현재 시간
    };

    // 파이썬 노드로 넘겨줄 JSON 문자열 형태로 변환
    const payload = JSON.stringify(coordinateData);

    // 콘솔에서 데이터 형태 확인(,.... 바꼈는지 )
    console.log("==========================================");
    console.log("토픽명: robot/goal_pose");
    console.log("데이터(JSON):", payload);
}