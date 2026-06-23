
function callRobot() {
    const button = document.getElementById("call-btn");
    button.disabled = true;
    const location = button.dataset.location;

    fetch("/call_robot", {
        method: "POST", 
        headers: {"Content-Type": "application/json"}, 
        body: JSON.stringify({
            location: location
        })
    })
    .then(response => {
        if(!response.ok){
            throw new Error("Error: 서버로 부터 응답이 없습니다");
        }
        return response.json();
    })
    .then(data => {
        if(data.success){
            button.innerText = "호출 환료";
            button.disabled = true;
            alert("로봇을 호출했습니다.");
        }
        else{
            alert("로봇 호출에 실패했습니다.");
        }
    })
    .catch(error => {
        console.error(error);
        alert("호출 실패");
    });
}