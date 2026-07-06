const batteryValue = document.getElementById("batteryValue");
const networkValue = document.getElementById("networkValue");
const networkDot = document.getElementById("networkDot");


function updateBattery(battery) {
    if (!batteryValue) {
        return;
    }

    if (typeof battery !== "number") {
        batteryValue.innerText = "--%";
        return;
    }

    batteryValue.innerText = `${battery}%`;
}


function updateNetwork(network) {
    if (!networkValue || !networkDot) {
        return;
    }

    if (network === "connected") {
        networkValue.innerText = "정상";
        networkDot.classList.add("connected");
        networkDot.classList.remove("disconnected");
    } else {
        networkValue.innerText = "연결 끊김";
        networkDot.classList.remove("connected");
        networkDot.classList.add("disconnected");
    }
}


function updateRobotStatus(data) {
    updateBattery(data.battery);
    updateNetwork(data.network);
}


function connectSocket() {
    if (typeof io === "undefined") {
        console.error("Socket.IO client is not loaded");

        if (networkValue) {
            networkValue.innerText = "소켓 오류";
        }

        return;
    }

    const socket = io();

    socket.on("connect", () => {
        console.log("Idle socket connected");
    });

    socket.on("disconnect", () => {
        console.log("Idle socket disconnected");

        updateNetwork("disconnected");
    });

    socket.on("robot_status", (data) => {
        console.log("robot_status", data);

        updateRobotStatus(data);
    });
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


loadInitialRobotStatus();
connectSocket();